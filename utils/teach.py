#!/usr/bin/env python3
"""
Drag-and-Teach: Record arm + hand trajectories by physically moving the robot.

Uses rt/arm_sdk to overlay arm control on top of Unitree's locomotion
controller. The legs and body keep full balance (push-resistant) while
the arms are set to compliant mode for drag teaching.

Usage:
    python teach.py                    # auto-save to trajectories/
    python teach.py -o my_motion.json  # custom output path
    python teach.py --no-hands         # skip hand recording
"""

import argparse
import json
import os
import signal
import sys
import threading
import time

from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
    MotionSwitcherClient,
)
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import (
    unitree_hg_msg_dds__HandCmd_,
    unitree_hg_msg_dds__LowCmd_,
    unitree_hg_msg_dds__MotorCmd_,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (
    HandCmd_,
    HandState_,
    LowCmd_,
    LowState_,
)
from unitree_sdk2py.utils.crc import CRC

RECORD_HZ = 50
RECORD_DT = 1.0 / RECORD_HZ

ARM_JOINTS = list(range(15, 29))
WAIST_JOINTS = [12, 13, 14]
ARM_SDK_JOINTS = WAIST_JOINTS + ARM_JOINTS

ARM_SDK_ENABLE_IDX = 29

TEACH_KP = 0.0
TEACH_KD = 1.0
WAIST_KP = 40.0
WAIST_KD = 2.0

# Passive teach: kp=0, kd=0, no servo status bit (matches dex3_release_hands).
# Optional light damping if fingers feel too loose: TEACH_HAND_KD=0.05
TEACH_HAND_KP = 0.0
TEACH_HAND_KD = 0.0
N_HAND_MOTORS_PER_HAND = 7

TOPIC_LEFT_HAND_CMD = "rt/dex3/left/cmd"
TOPIC_RIGHT_HAND_CMD = "rt/dex3/right/cmd"
TOPIC_LEFT_HAND_STATE = "rt/dex3/left/state"
TOPIC_RIGHT_HAND_STATE = "rt/dex3/right/state"


def _hand_motor_mode(motor_id, status=0x01, timeout=0):
    """Pack RIS motor mode byte: id(4b) | status(3b) | timeout(1b)."""
    return (motor_id & 0x0F) | ((status & 0x07) << 4) | ((timeout & 0x01) << 7)


def _hand_motor_mode_passive(motor_id):
    """Passive / release mode (no status enable bit) — easy to move by hand."""
    return motor_id & 0x0F

URDF_PATH = (
    "/home/humanoid-pc/unitree_rl_gym/resources/robots/"
    "g1_description/g1_29dof_with_hand_rev_1_0.urdf"
)

UNITREE_TO_PIN = {}
for _i in range(15):
    UNITREE_TO_PIN[_i] = _i
for _i in range(7):
    UNITREE_TO_PIN[15 + _i] = 15 + _i
    UNITREE_TO_PIN[22 + _i] = 29 + _i


class GravityCompensator:
    """Compute per-joint gravity torques using Pinocchio."""

    def __init__(self):
        self.model = None
        self.data = None
        self.available = False
        try:
            import pinocchio as pin
            self.pin = pin
            self.model = pin.buildModelFromUrdf(URDF_PATH)
            self.data = self.model.createData()
            self.neutral_q = pin.neutral(self.model)
            self.available = True
            print("Gravity compensation: ENABLED "
                  "(Pinocchio + URDF)")
        except Exception as e:
            print(f"Gravity compensation: DISABLED "
                  f"({e})")

    def compute(self, low_state):
        """Return {unitree_joint_idx: tau_ff} for arm joints."""
        if not self.available:
            return {}
        q = self.neutral_q.copy()
        for u_idx, p_idx in UNITREE_TO_PIN.items():
            if p_idx < self.model.nq:
                q[p_idx] = low_state.motor_state[u_idx].q
        G = self.pin.computeGeneralizedGravity(
            self.model, self.data, q
        )
        tau_ff = {}
        for j in ARM_JOINTS:
            p_idx = UNITREE_TO_PIN[j]
            tau_ff[j] = float(G[p_idx])
        for j in WAIST_JOINTS:
            p_idx = UNITREE_TO_PIN[j]
            tau_ff[j] = float(G[p_idx])
        return tau_ff

ARM_JOINT_NAMES = {
    15: "L_ShoulderPitch", 16: "L_ShoulderRoll",
    17: "L_ShoulderYaw", 18: "L_Elbow",
    19: "L_WristRoll", 20: "L_WristPitch",
    21: "L_WristYaw",
    22: "R_ShoulderPitch", 23: "R_ShoulderRoll",
    24: "R_ShoulderYaw", 25: "R_Elbow",
    26: "R_WristRoll", 27: "R_WristPitch",
    28: "R_WristYaw",
}

HAND_JOINT_NAMES = [
    "L_Thumb0", "L_Thumb1", "L_Thumb2",
    "L_Index0", "L_Index1", "L_Middle0", "L_Middle1",
    "R_Thumb0", "R_Thumb1", "R_Thumb2",
    "R_Index0", "R_Index1", "R_Middle0", "R_Middle1",
]

from teach_poses import (
    CLOSED_LEFT_HAND_Q,
    CLOSED_RIGHT_HAND_Q,
    CLOSE_HAND_KP,
    CLOSE_HAND_KD,
    COMPLIANT_RAMP_SEC,
    FORWARD_ARM_POSE,
    OPEN_HAND_Q,
    PARK_CLEARANCE_SEC,
    PARK_FORWARD_SEC,
    PREPARE_CLEARANCE_SEC,
    PREPARE_CLOSE_HANDS_SEC,
    PREPARE_FORWARD_SEC,
    BETWEEN_STEPS_COMPLIANT_RAMP_SEC,
    BETWEEN_STEPS_FORWARD_SEC,
    BETWEEN_STEPS_OPEN_HANDS_SEC,
    RELEASE_ARM_SEC,
    SETTLE_AT_FORWARD_SEC,
    SPREAD_ARM_POSE,
)

FORWARD_POSE_TOLERANCE = 0.2

KP_HOLD = 60.0
KD_HOLD = 2.0
OPEN_HAND_KP = 1.0
OPEN_HAND_KD = 0.2


def _smooth_ratio(t):
    t = max(0.0, min(1.0, float(t)))
    return t * t * (3.0 - 2.0 * t)


class TeachRecorder:
    def __init__(self, record_hands=True, grav_comp=None, hand_kd=TEACH_HAND_KD):
        self.record_hands = record_hands
        self.grav_comp = grav_comp
        self.hand_kd = hand_kd
        self.low_state = None
        self.left_hand_state = None
        self.right_hand_state = None
        self.state_received = False
        self.hand_received = False
        self.crc = CRC()

        self.waist_lock_pos = {}
        self.recording = False
        self.frames = []
        self._stop = False
        self._session_stop = False
        self._session_thread = None
        self._skip_release_on_stop = False

    def init(self):
        self.arm_sdk_pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_sdk_pub.Init()
        self.lowstate_sub = ChannelSubscriber(
            "rt/lowstate", LowState_
        )
        self.lowstate_sub.Init(self._on_low_state, 10)

        # Separate publishers for left and right hand
        self.left_hand_pub = ChannelPublisher(
            TOPIC_LEFT_HAND_CMD, HandCmd_
        )
        self.left_hand_pub.Init()
        self.right_hand_pub = ChannelPublisher(
            TOPIC_RIGHT_HAND_CMD, HandCmd_
        )
        self.right_hand_pub.Init()
        if self.record_hands:
            self.left_hand_sub = ChannelSubscriber(
                TOPIC_LEFT_HAND_STATE, HandState_
            )
            self.left_hand_sub.Init(
                self._on_left_hand_state, 10
            )
            self.right_hand_sub = ChannelSubscriber(
                TOPIC_RIGHT_HAND_STATE, HandState_
            )
            self.right_hand_sub.Init(
                self._on_right_hand_state, 10
            )

    def _on_low_state(self, msg: LowState_):
        self.low_state = msg
        if not self.state_received:
            self.state_received = True

    def _on_left_hand_state(self, msg: HandState_):
        self.left_hand_state = msg
        if not self.hand_received:
            self.hand_received = True

    def _on_right_hand_state(self, msg: HandState_):
        self.right_hand_state = msg
        if not self.hand_received:
            self.hand_received = True

    def wait_for_state(self, timeout=5.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.state_received:
                if not self.record_hands or self.hand_received:
                    return True
            time.sleep(0.1)
        return self.state_received

    def lock_waist(self):
        """Snapshot waist positions to hold them during teach."""
        for j in WAIST_JOINTS:
            self.waist_lock_pos[j] = float(
                self.low_state.motor_state[j].q
            )

    def _send_teach_cmd(self):
        """Publish arm_sdk: waist locked, arms compliant + gravity comp."""
        cmd = unitree_hg_msg_dds__LowCmd_()
        cmd.motor_cmd[ARM_SDK_ENABLE_IDX].q = 1.0

        grav = {}
        if self.grav_comp and self.low_state:
            grav = self.grav_comp.compute(self.low_state)

        for j in WAIST_JOINTS:
            cmd.motor_cmd[j].mode = 1
            cmd.motor_cmd[j].q = self.waist_lock_pos.get(
                j, self.low_state.motor_state[j].q
            )
            cmd.motor_cmd[j].dq = 0.0
            cmd.motor_cmd[j].tau = grav.get(j, 0.0)
            cmd.motor_cmd[j].kp = WAIST_KP
            cmd.motor_cmd[j].kd = WAIST_KD

        for j in ARM_JOINTS:
            cmd.motor_cmd[j].mode = 1
            cmd.motor_cmd[j].q = (
                self.low_state.motor_state[j].q
            )
            cmd.motor_cmd[j].dq = 0.0
            cmd.motor_cmd[j].tau = grav.get(j, 0.0)
            cmd.motor_cmd[j].kp = TEACH_KP
            cmd.motor_cmd[j].kd = TEACH_KD

        cmd.crc = self.crc.Crc(cmd)
        self.arm_sdk_pub.Write(cmd)

    def _get_arm_positions(self):
        return {
            j: float(self.low_state.motor_state[j].q)
            for j in ARM_JOINTS
        }

    def _send_arm_hold_cmd(
        self,
        positions,
        weight=1.0,
        kp_scale=1.0,
        kd_scale=1.0,
    ):
        """Stiff position hold (used before release / between record and replay)."""
        if not self.low_state:
            return
        cmd = unitree_hg_msg_dds__LowCmd_()
        cmd.motor_cmd[ARM_SDK_ENABLE_IDX].q = weight

        grav = {}
        if self.grav_comp and self.low_state:
            grav = self.grav_comp.compute(self.low_state)

        for j in WAIST_JOINTS:
            cmd.motor_cmd[j].mode = 1
            cmd.motor_cmd[j].q = self.waist_lock_pos.get(
                j, self.low_state.motor_state[j].q
            )
            cmd.motor_cmd[j].dq = 0.0
            cmd.motor_cmd[j].tau = grav.get(j, 0.0)
            cmd.motor_cmd[j].kp = WAIST_KP * kp_scale
            cmd.motor_cmd[j].kd = WAIST_KD * kd_scale

        arm_kp = KP_HOLD * kp_scale
        arm_kd = KD_HOLD * kd_scale
        for j in ARM_JOINTS:
            cmd.motor_cmd[j].mode = 1
            cmd.motor_cmd[j].q = float(positions.get(j, 0.0))
            cmd.motor_cmd[j].dq = 0.0
            cmd.motor_cmd[j].tau = grav.get(j, 0.0)
            cmd.motor_cmd[j].kp = arm_kp
            cmd.motor_cmd[j].kd = arm_kd

        cmd.crc = self.crc.Crc(cmd)
        self.arm_sdk_pub.Write(cmd)

    def _build_hand_cmd(
        self, q_targets, kp=0.0, kd=0.2, passive=False,
    ):
        """Build a HandCmd_ with 7 motors."""
        cmd = unitree_hg_msg_dds__HandCmd_()
        for i in range(N_HAND_MOTORS_PER_HAND):
            if passive:
                cmd.motor_cmd[i].mode = _hand_motor_mode_passive(i)
            else:
                cmd.motor_cmd[i].mode = _hand_motor_mode(i)
            cmd.motor_cmd[i].q = float(q_targets[i])
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].tau = 0.0
            cmd.motor_cmd[i].kp = kp
            cmd.motor_cmd[i].kd = kd
        return cmd

    def _send_hand_passive(self):
        """Release finger hold so they can be posed by hand (still records q)."""
        if not self.record_hands:
            return
        if self.hand_kd <= 0.0:
            zeros = [0.0] * N_HAND_MOTORS_PER_HAND
            cmd = self._build_hand_cmd(
                zeros, kp=0.0, kd=0.0, passive=True,
            )
            self.left_hand_pub.Write(cmd)
            self.right_hand_pub.Write(cmd)
            return
        # Optional light damping: track current q with kp=0
        if self.left_hand_state is not None:
            lq = [
                self.left_hand_state.motor_state[i].q
                for i in range(N_HAND_MOTORS_PER_HAND)
            ]
            self.left_hand_pub.Write(
                self._build_hand_cmd(
                    lq, kp=0.0, kd=self.hand_kd, passive=False,
                )
            )
        if self.right_hand_state is not None:
            rq = [
                self.right_hand_state.motor_state[i].q
                for i in range(N_HAND_MOTORS_PER_HAND)
            ]
            self.right_hand_pub.Write(
                self._build_hand_cmd(
                    rq, kp=0.0, kd=self.hand_kd, passive=False,
                )
            )

    def _prime_hand_passive(self, hold_arm_pose, duration=0.4):
        """Burst passive hand cmds; must keep arm hold to avoid arm drop."""
        t_end = time.time() + duration
        while time.time() < t_end:
            self._send_arm_hold_cmd(hold_arm_pose)
            if self.record_hands:
                self._send_hand_passive()
            time.sleep(RECORD_DT)

    def _release_hands(self):
        """Drive all fingers to open (q=0) via correct dex3 topics."""
        print("Opening fingers...")
        zeros = [0.0] * N_HAND_MOTORS_PER_HAND
        open_kp = 1.5
        open_kd = 0.2

        # Phase A: drive to open
        steps = int(2.0 / RECORD_DT)
        for i in range(steps):
            ratio = i / steps
            cmd = self._build_hand_cmd(
                zeros, kp=open_kp * ratio, kd=open_kd
            )
            self.left_hand_pub.Write(cmd)
            self.right_hand_pub.Write(cmd)
            time.sleep(RECORD_DT)

        # Phase B: hold open
        for _ in range(int(0.5 / RECORD_DT)):
            cmd = self._build_hand_cmd(
                zeros, kp=open_kp, kd=open_kd
            )
            self.left_hand_pub.Write(cmd)
            self.right_hand_pub.Write(cmd)
            time.sleep(RECORD_DT)

        # Phase C: ramp down
        print("Releasing hand control...")
        steps = int(0.5 / RECORD_DT)
        for i in range(steps):
            w = 1.0 - (i / steps)
            cmd = self._build_hand_cmd(
                zeros, kp=open_kp * w, kd=open_kd * w
            )
            self.left_hand_pub.Write(cmd)
            self.right_hand_pub.Write(cmd)
            time.sleep(RECORD_DT)
        print("Hands released.")

    def _release_arm_control(
        self,
        hold_pose: dict | None = None,
        duration: float = RELEASE_ARM_SEC,
    ):
        """Slowly release arm_sdk while holding a safe pose + gravity comp."""
        if not self.low_state:
            return
        if hold_pose is None:
            hold_pose = self._get_arm_positions()
        self.lock_waist()
        steps = max(1, int(duration / RECORD_DT))
        print(f"Releasing arm control over {duration:.1f}s...")
        for i in range(steps):
            s = _smooth_ratio((i + 1) / steps)
            w = 1.0 - s
            kp_scale = max(0.05, 1.0 - s)
            self._send_arm_hold_cmd(
                hold_pose, weight=w, kp_scale=kp_scale, kd_scale=kp_scale,
            )
            if self.record_hands:
                self._send_dual_hand_cmds(
                    CLOSED_LEFT_HAND_Q, CLOSED_RIGHT_HAND_Q,
                    kp=CLOSE_HAND_KP, kd=CLOSE_HAND_KD,
                )
            time.sleep(RECORD_DT)
        if self.record_hands:
            self._release_hands_closed()

    def _release_hands_closed(self):
        """Leave fingers in closed fist while dropping hand servo gains."""
        if not self.record_hands:
            return
        print("Releasing hand control (fingers closed)...")
        steps = max(1, int(0.8 / RECORD_DT))
        for i in range(steps):
            w = 1.0 - _smooth_ratio((i + 1) / steps)
            self._send_dual_hand_cmds(
                CLOSED_LEFT_HAND_Q, CLOSED_RIGHT_HAND_Q,
                kp=CLOSE_HAND_KP * max(w, 0.05),
                kd=CLOSE_HAND_KD,
            )
            time.sleep(RECORD_DT)
        self._send_dual_hand_cmds(
            CLOSED_LEFT_HAND_Q, CLOSED_RIGHT_HAND_Q,
            kp=0.0, kd=0.0, passive=True,
        )

    def _disable_arm_sdk(self):
        """Backward-compatible alias — slow release from current pose."""
        self._release_arm_control(
            hold_pose=self._get_arm_positions() if self.low_state else None,
            duration=RELEASE_ARM_SEC,
        )

    def _capture_frame(self):
        frame = {"t": time.time()}
        arm = {}
        for j in ARM_JOINTS:
            arm[str(j)] = float(
                self.low_state.motor_state[j].q
            )
        frame["arm"] = arm

        if self.record_hands:
            hand = {}
            if self.left_hand_state is not None:
                for i in range(N_HAND_MOTORS_PER_HAND):
                    hand[str(i)] = float(
                        self.left_hand_state.motor_state[i].q
                    )
            if self.right_hand_state is not None:
                for i in range(N_HAND_MOTORS_PER_HAND):
                    hand[str(i + 7)] = float(
                        self.right_hand_state.motor_state[i].q
                    )
            if hand:
                frame["hand"] = hand

        return frame

    def start_recording(self):
        """Begin capturing frames (UI or programmatic control)."""
        if self.recording:
            return False
        self.recording = True
        self.frames = []
        self._rec_start = time.time()
        return True

    def stop_recording(self):
        """Stop capturing; returns (n_frames, duration_s) or None."""
        if not self.recording:
            return None
        self.recording = False
        dur = time.time() - self._rec_start
        return len(self.frames), dur

    def begin_compliant_session(self):
        """Background loop: compliant arms until stop_compliant_session()."""
        if self._session_thread and self._session_thread.is_alive():
            return
        self._session_stop = False
        self.lock_waist()

        def _loop():
            while not self._session_stop:
                self._send_teach_cmd()
                self._send_hand_passive()
                time.sleep(RECORD_DT)
                if self.recording and self.low_state:
                    self.frames.append(self._capture_frame())
                    n = len(self.frames)
                    if n % RECORD_HZ == 0:
                        elapsed = time.time() - self._rec_start
                        print(f"  Recording: {n} frames "
                              f"({elapsed:.1f}s)")
            if not self._skip_release_on_stop:
                self._disable_arm_sdk()

        self._session_thread = threading.Thread(
            target=_loop, daemon=True
        )
        self._session_thread.start()

    def close_channels(self):
        """Release DDS readers/writers (call before standalone replay)."""
        for attr in (
            "arm_sdk_pub", "left_hand_pub", "right_hand_pub",
            "lowstate_sub", "left_hand_sub", "right_hand_sub",
        ):
            ch = getattr(self, attr, None)
            if ch is None:
                continue
            try:
                ch.Close()
            except Exception:
                pass

    def suspend_compliant_session(self):
        """Stop background compliant loop without releasing arm_sdk."""
        self.recording = False
        self._skip_release_on_stop = True
        self._session_stop = True
        if self._session_thread:
            self._session_thread.join(timeout=8.0)
            self._session_thread = None

    def stop_compliant_session(self, release_control=True):
        """End background session; optionally release arm_sdk."""
        self.suspend_compliant_session()
        if release_control and self.low_state:
            self._disable_arm_sdk()

    def _send_dual_hand_cmds(
        self, left_q, right_q, kp, kd, passive=False,
    ):
        self.left_hand_pub.Write(
            self._build_hand_cmd(left_q, kp=kp, kd=kd, passive=passive)
        )
        self.right_hand_pub.Write(
            self._build_hand_cmd(right_q, kp=kp, kd=kd, passive=passive)
        )

    @staticmethod
    def _lerp_hand_q(q0, q1, alpha):
        return [
            q0[i] + (q1[i] - q0[i]) * alpha
            for i in range(N_HAND_MOTORS_PER_HAND)
        ]

    def _current_hand_q(self):
        """Current left/right finger q (7 each) or open if unknown."""
        if self.left_hand_state is not None:
            left = [
                float(self.left_hand_state.motor_state[i].q)
                for i in range(N_HAND_MOTORS_PER_HAND)
            ]
        else:
            left = list(OPEN_HAND_Q)
        if self.right_hand_state is not None:
            right = [
                float(self.right_hand_state.motor_state[i].q)
                for i in range(N_HAND_MOTORS_PER_HAND)
            ]
        else:
            right = list(OPEN_HAND_Q)
        return left, right

    def _close_hands(
        self,
        duration: float = PREPARE_CLOSE_HANDS_SEC,
        hold_arm_pose: dict | None = None,
    ):
        """Close Dex3 fingers (teleop dex3_close_hands targets)."""
        if not self.record_hands:
            return
        if hold_arm_pose is None and self.low_state:
            hold_arm_pose = self._get_arm_positions()
        start_left, start_right = self._current_hand_q()
        steps = max(1, int(duration / RECORD_DT))
        print("  Closing fingers...")
        for i in range(steps):
            ratio = _smooth_ratio((i + 1) / steps)
            if hold_arm_pose is not None:
                self._send_arm_hold_cmd(hold_arm_pose)
            left = self._lerp_hand_q(start_left, CLOSED_LEFT_HAND_Q, ratio)
            right = self._lerp_hand_q(start_right, CLOSED_RIGHT_HAND_Q, ratio)
            self._send_dual_hand_cmds(
                left, right,
                kp=CLOSE_HAND_KP * ratio,
                kd=CLOSE_HAND_KD,
            )
            time.sleep(RECORD_DT)
        if hold_arm_pose is not None:
            self._send_arm_hold_cmd(hold_arm_pose)
        self._send_dual_hand_cmds(
            CLOSED_LEFT_HAND_Q, CLOSED_RIGHT_HAND_Q,
            kp=CLOSE_HAND_KP, kd=CLOSE_HAND_KD,
        )

    def _ramp_arm_to(
        self,
        target: dict,
        duration: float,
        label: str,
        *,
        hold_hands_closed: bool = False,
        open_hands_to_forward: bool = False,
    ):
        """Smooth stiff hold to a target arm pose; optional hand close/open."""
        if not self.low_state:
            return
        self.lock_waist()
        current = self._get_arm_positions()
        steps = max(1, int(duration / RECORD_DT))
        print(f"  {label}...")
        for i in range(steps):
            s = _smooth_ratio(i / steps)
            blended = {
                j: current[j] + (target[j] - current[j]) * s
                for j in ARM_JOINTS
            }
            self._send_arm_hold_cmd(blended)
            if self.record_hands:
                if hold_hands_closed:
                    self._send_dual_hand_cmds(
                        CLOSED_LEFT_HAND_Q, CLOSED_RIGHT_HAND_Q,
                        kp=CLOSE_HAND_KP, kd=CLOSE_HAND_KD,
                    )
                elif open_hands_to_forward:
                    left = self._lerp_hand_q(
                        CLOSED_LEFT_HAND_Q, OPEN_HAND_Q, s,
                    )
                    right = self._lerp_hand_q(
                        CLOSED_RIGHT_HAND_Q, OPEN_HAND_Q, s,
                    )
                    self._send_dual_hand_cmds(
                        left, right,
                        kp=OPEN_HAND_KP * max(s, 0.15),
                        kd=OPEN_HAND_KD,
                    )
            time.sleep(RECORD_DT)
        if self.record_hands and open_hands_to_forward:
            self._send_dual_hand_cmds(
                OPEN_HAND_Q, OPEN_HAND_Q,
                kp=OPEN_HAND_KP, kd=OPEN_HAND_KD,
            )

    def _hold_pose_settle(
        self,
        pose: dict,
        duration: float,
        label: str,
        *,
        hands_closed: bool = False,
    ):
        """Stiff hold at pose (gravity comp) before mode changes."""
        if duration <= 0 or not self.low_state:
            return
        steps = max(1, int(duration / RECORD_DT))
        print(f"  {label}...")
        for _ in range(steps):
            self._send_arm_hold_cmd(pose)
            if self.record_hands:
                if hands_closed:
                    self._send_dual_hand_cmds(
                        CLOSED_LEFT_HAND_Q, CLOSED_RIGHT_HAND_Q,
                        kp=CLOSE_HAND_KP, kd=CLOSE_HAND_KD,
                    )
                else:
                    self._send_dual_hand_cmds(
                        OPEN_HAND_Q, OPEN_HAND_Q,
                        kp=OPEN_HAND_KP, kd=OPEN_HAND_KD,
                    )
            time.sleep(RECORD_DT)

    def _ramp_stiff_to_compliant(self, hold_pose: dict, duration: float):
        """Gradually reduce arm stiffness into drag-teach (avoids sudden drop)."""
        if not self.low_state:
            return
        steps = max(1, int(duration / RECORD_DT))
        print(f"  Engaging compliant mode over {duration:.1f}s...")
        for i in range(steps):
            s = _smooth_ratio((i + 1) / steps)
            kp_scale = max(0.0, 1.0 - s)
            kd_scale = max(0.15, 1.0 - 0.85 * s)
            self._send_arm_hold_cmd(
                hold_pose, kp_scale=kp_scale, kd_scale=kd_scale,
            )
            if self.record_hands:
                self._send_hand_passive()
            time.sleep(RECORD_DT)

    def prepare_recording_pose(
        self,
        resume_compliant: bool = True,
        quick: bool = False,
    ):
        """Teleop-style prepare: spread clearance -> forward q=0 -> open hands.

        If quick=True and already near forward, skip spread park (between steps).
        """
        if quick and self.is_near_forward_pose():
            return self._prepare_recording_quick(resume_compliant)

        if self._session_thread and self._session_thread.is_alive():
            self.suspend_compliant_session()
        if not self.low_state:
            print("ERROR: No robot state for prepare pose.")
            return False

        print("Preparing teleop forward recording pose...")
        self.lock_waist()
        arm_hold = self._get_arm_positions()
        if self.record_hands:
            self._close_hands(hold_arm_pose=arm_hold)
        self._ramp_arm_to(
            SPREAD_ARM_POSE, PREPARE_CLEARANCE_SEC,
            "Outward clearance (fingers closed)",
            hold_hands_closed=True,
        )
        self._ramp_arm_to(
            FORWARD_ARM_POSE, PREPARE_FORWARD_SEC,
            "Forward default (q=0), opening hands",
            open_hands_to_forward=True,
        )
        self._hold_pose_settle(
            FORWARD_ARM_POSE, SETTLE_AT_FORWARD_SEC,
            "Settling at forward pose",
        )
        if self.record_hands:
            self._prime_hand_passive(FORWARD_ARM_POSE)

        if resume_compliant:
            self._ramp_stiff_to_compliant(
                FORWARD_ARM_POSE, COMPLIANT_RAMP_SEC,
            )
            self._send_teach_cmd()
            if self.record_hands:
                self._send_hand_passive()
            print("Starting compliant drag-teach loop...")
            self.begin_compliant_session()
        print("Ready to record from forward pose.")
        return True

    @staticmethod
    def _arm_pose_distance(a: dict, b: dict) -> float:
        return max(abs(a[j] - b[j]) for j in ARM_JOINTS)

    def is_near_forward_pose(self, tolerance: float = FORWARD_POSE_TOLERANCE) -> bool:
        if not self.low_state:
            return False
        return (
            self._arm_pose_distance(
                self._get_arm_positions(), FORWARD_ARM_POSE,
            )
            <= tolerance
        )

    def _open_hands_at_forward_hold(
        self,
        duration: float = BETWEEN_STEPS_OPEN_HANDS_SEC,
    ):
        """Open hands while stiff-holding forward arm pose."""
        if not self.record_hands or not self.low_state:
            return
        steps = max(1, int(duration / RECORD_DT))
        print("  Opening hands at forward...")
        for i in range(steps):
            s = _smooth_ratio((i + 1) / steps)
            self._send_arm_hold_cmd(FORWARD_ARM_POSE)
            left = self._lerp_hand_q(CLOSED_LEFT_HAND_Q, OPEN_HAND_Q, s)
            right = self._lerp_hand_q(CLOSED_RIGHT_HAND_Q, OPEN_HAND_Q, s)
            self._send_dual_hand_cmds(
                left, right,
                kp=OPEN_HAND_KP * max(s, 0.15),
                kd=OPEN_HAND_KD,
            )
            time.sleep(RECORD_DT)
        self._send_arm_hold_cmd(FORWARD_ARM_POSE)
        self._send_dual_hand_cmds(
            OPEN_HAND_Q, OPEN_HAND_Q,
            kp=OPEN_HAND_KP, kd=OPEN_HAND_KD,
        )

    def hold_forward_between_steps(self):
        """After saving a step: close hands, stay at forward, ready for next step."""
        if not self.low_state:
            print("ERROR: No robot state.")
            return False

        print("Holding forward pose for next step...")
        if self._session_thread and self._session_thread.is_alive():
            self.suspend_compliant_session()

        self.lock_waist()
        arm_hold = self._get_arm_positions()
        if self.record_hands:
            self._close_hands(hold_arm_pose=arm_hold)

        if not self.is_near_forward_pose():
            self._ramp_arm_to(
                FORWARD_ARM_POSE, BETWEEN_STEPS_FORWARD_SEC,
                "Return to forward (fingers closed)",
                hold_hands_closed=True,
            )
        else:
            self._send_arm_hold_cmd(FORWARD_ARM_POSE)

        self._hold_pose_settle(
            FORWARD_ARM_POSE, SETTLE_AT_FORWARD_SEC * 0.5,
            "Ready at forward for next step",
            hands_closed=True,
        )
        self._ramp_stiff_to_compliant(
            FORWARD_ARM_POSE, BETWEEN_STEPS_COMPLIANT_RAMP_SEC,
        )
        self._send_teach_cmd()
        if self.record_hands:
            self._send_hand_passive()
        self.begin_compliant_session()
        print("At forward — ready to record next step.")
        return True

    def _prepare_recording_quick(self, resume_compliant: bool) -> bool:
        """Skip spread clearance when already at forward between steps."""
        if self._session_thread and self._session_thread.is_alive():
            self.suspend_compliant_session()
        if not self.low_state:
            print("ERROR: No robot state for prepare pose.")
            return False

        print("Quick prepare at forward (next step)...")
        self.lock_waist()
        if not self.is_near_forward_pose():
            arm_hold = self._get_arm_positions()
            if self.record_hands:
                self._close_hands(hold_arm_pose=arm_hold)
            self._ramp_arm_to(
                FORWARD_ARM_POSE, BETWEEN_STEPS_FORWARD_SEC,
                "To forward (fingers closed)",
                hold_hands_closed=True,
            )
        else:
            self._send_arm_hold_cmd(FORWARD_ARM_POSE)

        self._open_hands_at_forward_hold()
        if self.record_hands:
            self._prime_hand_passive(FORWARD_ARM_POSE)

        if resume_compliant:
            self._ramp_stiff_to_compliant(
                FORWARD_ARM_POSE, BETWEEN_STEPS_COMPLIANT_RAMP_SEC,
            )
            self._send_teach_cmd()
            if self.record_hands:
                self._send_hand_passive()
            self.begin_compliant_session()
        print("Ready to record from forward pose.")
        return True

    def park_after_recording_pose(self):
        """Reverse of prepare_recording_pose: forward -> spread (clearance).

        Prepare:  close -> spread -> forward (open hands)
        Park:     close -> forward (closed) -> spread outward (closed)
        Avoids re-running spread->forward after save (outward then drop).
        """
        if self._session_thread and self._session_thread.is_alive():
            self.suspend_compliant_session()
        if not self.low_state:
            print("ERROR: No robot state for park pose.")
            return False

        print("Parking after recording (reverse of prepare)...")
        self.lock_waist()
        arm_hold = self._get_arm_positions()
        if self.record_hands:
            self._close_hands(hold_arm_pose=arm_hold)

        current = self._get_arm_positions()
        if self._arm_pose_distance(current, FORWARD_ARM_POSE) > 0.2:
            self._ramp_arm_to(
                FORWARD_ARM_POSE, PARK_FORWARD_SEC,
                "To forward (fingers closed)",
                hold_hands_closed=True,
            )

        self._ramp_arm_to(
            SPREAD_ARM_POSE, PARK_CLEARANCE_SEC,
            "Outward clearance — away from body (fingers closed)",
            hold_hands_closed=True,
        )
        self._hold_pose_settle(
            SPREAD_ARM_POSE, SETTLE_AT_FORWARD_SEC,
            "Settling at outward spread",
            hands_closed=True,
        )
        print("Parked at outward spread — safe to release control.")
        return True

    def finish_recording_session(self):
        """After save: park outward (reverse prepare), then slow release."""
        self.park_after_recording_pose()
        self._release_arm_control(
            hold_pose=SPREAD_ARM_POSE,
            duration=RELEASE_ARM_SEC,
        )
        print("Arm control released — ready for replay.")

    def save_recording(self, output_path, name=None):
        """Write captured frames to JSON; clears frame buffer."""
        if not self.frames:
            return None
        self._save(output_path, name=name)
        self.frames = []
        return output_path

    def run_teach(self, output_path, name=None):
        signal.signal(
            signal.SIGINT,
            lambda s, f: setattr(self, '_stop', True)
        )

        print("\n" + "=" * 50)
        print("  DRAG-AND-TEACH MODE")
        print("=" * 50)
        grav_str = ("ON — arms feel weightless"
                    if self.grav_comp and self.grav_comp.available
                    else "OFF — arms may feel heavy")
        print(f"\nGravity compensation: {grav_str}")
        print("Arms are in compliant mode — move them freely!")
        print("")
        print("Controls:")
        print("  [Enter]  Start/Stop recording")
        print("  [Ctrl+C] Quit and release arm control")
        print("")

        if self.record_hands and not self.hand_received:
            print("NOTE: Hand state not received. "
                  "Recording arms only.")
            self.record_hands = False

        self.lock_waist()

        key_pressed = [False]

        def _key_listener():
            while not self._stop:
                try:
                    input()
                    key_pressed[0] = True
                except EOFError:
                    break

        threading.Thread(
            target=_key_listener, daemon=True
        ).start()

        print(">> Press [Enter] to START recording...")

        while not self._stop:
            self._send_teach_cmd()
            self._send_hand_passive()
            time.sleep(RECORD_DT)

            if key_pressed[0]:
                key_pressed[0] = False
                if not self.recording:
                    self.start_recording()
                    print("\n** RECORDING ** "
                          "Move the robot! "
                          "Press [Enter] to stop.")
                else:
                    stats = self.stop_recording()
                    if stats:
                        n, dur = stats
                        print(f"\n** STOPPED ** "
                              f"{n} frames, {dur:.1f}s")
                    break

            if self.recording:
                self.frames.append(self._capture_frame())
                n = len(self.frames)
                if n % RECORD_HZ == 0:
                    elapsed = time.time() - self._rec_start
                    print(f"  Recording: {n} frames "
                          f"({elapsed:.1f}s)")

        self._disable_arm_sdk()

        if not self.frames:
            print("No frames recorded.")
            return None

        self._save(output_path, name=name)
        return output_path

    def _save(self, path, name=None):
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)

        t0 = self.frames[0]["t"]
        for f in self.frames:
            f["t"] = round(f["t"] - t0, 4)

        metadata = {
            "record_hz": RECORD_HZ,
            "n_frames": len(self.frames),
            "duration_s": round(self.frames[-1]["t"], 2),
            "arm_joints": ARM_JOINTS,
            "arm_joint_names": ARM_JOINT_NAMES,
            "record_hands": self.record_hands,
            "control_mode": "arm_sdk",
            "created": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        if name:
            metadata["name"] = name.strip()
        if self.record_hands:
            metadata["hand_joint_names"] = HAND_JOINT_NAMES

        data = {
            "metadata": metadata,
            "frames": self.frames,
        }

        with open(path, "w") as f:
            json.dump(data, f, indent=2)

        print(f"\nSaved: {path}")
        print(f"  Frames: {metadata['n_frames']}")
        print(f"  Duration: {metadata['duration_s']}s")
        print(f"  Hz: {metadata['record_hz']}")


def ensure_ai_mode(interactive=True):
    """Check that the locomotion controller (ai sport) is active.

    The balance controller must be started via the hand controller:
      1. L1+A  — power on
      2. L1+UP — stand up
    Only then will the ai sport client be running.

    Set interactive=False for web UI (no stdin prompt).
    """
    msc = MotionSwitcherClient()
    msc.SetTimeout(5.0)
    msc.Init()

    code, result = msc.CheckMode()
    current = result.get("name", "") if result else ""
    print(f"Current mode: '{current}' (form={result})")

    if current == "ai":
        print("OK — ai sport client is active.")
        return True

    # Try to select ai mode via SDK (works in some cases)
    print("ai mode not detected. Attempting "
          "SelectMode('ai')...")
    code, _ = msc.SelectMode("ai")
    print(f"  SelectMode returned code={code}")
    time.sleep(3)

    code, result = msc.CheckMode()
    current = result.get("name", "") if result else ""
    print(f"  Mode after select: '{current}'")

    if current == "ai":
        # Also try to tell the locomotion FSM to stand
        loco = LocoClient()
        loco.SetTimeout(5.0)
        loco.Init()
        ret = loco.SetFsmId(200)
        print(f"  LocoClient.Start() returned: {ret}")
        time.sleep(3)
        return True

    print("\n" + "!" * 50)
    print("  WARNING: Balance controller NOT active!")
    print("!" * 50)
    print("\nPlease start the robot with the hand "
          "controller:")
    print("  1. L1 + A    — power on")
    print("  2. L1 + UP   — stand up")
    print("  3. Robot should be standing and balanced")
    print("  4. Then re-run this script")
    print("\nIf in debug mode (L2+R2), reboot the robot.")
    print("")

    if not interactive:
        print("Aborting (non-interactive mode).")
        return False
    ans = input("Continue anyway? (y/N): ").strip().lower()
    return ans == "y"


def main():
    parser = argparse.ArgumentParser(
        description="Drag-and-Teach Recorder (arm_sdk)"
    )
    parser.add_argument(
        "-o", "--output", default=None,
        help="Output file path"
    )
    parser.add_argument(
        "--no-hands", action="store_true",
        help="Skip hand/finger recording"
    )
    parser.add_argument(
        "--hand-kd", type=float, default=TEACH_HAND_KD,
        help="Finger velocity damping during teach (default: 0 = passive)"
    )
    parser.add_argument(
        "--no-prepare", action="store_true",
        help="Skip teleop forward prepare pose before recording (CLI)"
    )
    parser.add_argument(
        "-n", "--name", default=None,
        help="Short language description saved in metadata "
             "(also used in the output filename)"
    )
    parser.add_argument(
        "network", nargs="?", default=None,
        help="Network interface (optional)"
    )
    args = parser.parse_args()

    traj_dir = os.path.join(
        os.path.dirname(__file__), "..", "trajectories"
    )
    if args.output is None:
        if args.name:
            from teach_catalog import trajectory_output_path
            args.output = trajectory_output_path(
                traj_dir, args.name
            )
        else:
            ts = time.strftime("%Y%m%d_%H%M%S")
            args.output = os.path.join(
                traj_dir, f"traj_{ts}.json"
            )

    print("=" * 50)
    print("  G1 Drag-and-Teach Recorder")
    print("  (arm_sdk + Unitree balance controller)")
    print("=" * 50)

    if args.network:
        ChannelFactoryInitialize(0, args.network)
    else:
        ChannelFactoryInitialize(0)

    ensure_ai_mode()

    grav = GravityCompensator()
    recorder = TeachRecorder(
        record_hands=not args.no_hands,
        grav_comp=grav,
        hand_kd=float(args.hand_kd),
    )
    recorder.init()

    print("Waiting for robot state...")
    if not recorder.wait_for_state():
        print("ERROR: No robot state received!")
        sys.exit(1)
    print("Robot connected!")
    if not args.no_prepare:
        recorder.prepare_recording_pose(resume_compliant=False)

    result = recorder.run_teach(args.output, name=args.name)
    if result:
        print(f"\nTo replay: "
              f"python utils/replay.py {result}")
        if args.name:
            print(f"  Name: {args.name}")


if __name__ == "__main__":
    main()
