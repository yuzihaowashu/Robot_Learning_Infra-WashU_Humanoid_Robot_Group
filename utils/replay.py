#!/usr/bin/env python3
"""
Replay a trajectory recorded by teach.py.

Uses rt/arm_sdk to overlay arm control on top of Unitree's locomotion
controller. The robot maintains full balance while replaying arm motions.

Usage:
    python replay.py trajectories/traj_xxx.json
    python replay.py traj.json --speed 0.5   # half speed
    python replay.py traj.json --loop 3      # repeat 3 times
"""

import argparse
import json
import os
import signal
import sys
import threading
import time

import numpy as np
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

ARM_JOINTS = list(range(15, 29))
WAIST_JOINTS = [12, 13, 14]
ARM_SDK_JOINTS = WAIST_JOINTS + ARM_JOINTS
ARM_SDK_ENABLE_IDX = 29

KP_ARM = 80.0
KD_ARM = 2.0
KP_HAND = 1.5
KD_HAND = 0.2
N_HAND_MOTORS_PER_HAND = 7

TOPIC_LEFT_HAND_CMD = "rt/dex3/left/cmd"
TOPIC_RIGHT_HAND_CMD = "rt/dex3/right/cmd"
TOPIC_LEFT_HAND_STATE = "rt/dex3/left/state"
TOPIC_RIGHT_HAND_STATE = "rt/dex3/right/state"


def _hand_motor_mode(motor_id, status=0x01, timeout=0):
    return ((motor_id & 0x0F) | ((status & 0x07) << 4)
            | ((timeout & 0x01) << 7))
CONTROL_DT = 0.02
MOVE_TO_START_DURATION = 3.0

# Match teach_poses / dex3_close_hands
OPEN_HAND_Q = [0.0] * N_HAND_MOTORS_PER_HAND
CLOSED_LEFT_HAND_Q = [0.0, 1.0, 1.74, -1.57, -1.74, -1.57, -1.74]
CLOSED_RIGHT_HAND_Q = [0.0, -1.0, -1.74, 1.57, 1.74, 1.57, 1.74]
PREPARE_CLOSE_HANDS_SEC = 0.5
CLOSE_HAND_KP = 0.4
CLOSE_HAND_KD = 0.15

URDF_PATH = (
    "/home/humanoid-pc/unitree_rl_gym/resources/robots/"
    "g1_description/g1_29dof_with_hand_rev_1_0.urdf"
)

# Unitree motor index → Pinocchio q-vector index
# Left arm 15-21 → same; Right arm 22-28 → 29-35 (hand joints 22-28 in between)
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


def smooth_ratio(t):
    t = float(np.clip(t, 0.0, 1.0))
    return t * t * (3.0 - 2.0 * t)


def lerp_frame(f0, f1, alpha):
    result = {}
    for key in f0:
        result[key] = f0[key] + (f1[key] - f0[key]) * alpha
    return result


class TrajectoryPlayer:
    def __init__(self, traj_data, speed=1.0,
                 grav_comp=None):
        self.metadata = traj_data["metadata"]
        self.raw_frames = traj_data["frames"]
        self.speed = speed
        self.has_hands = self.metadata.get("record_hands", False)
        self.grav_comp = grav_comp

        self.arm_traj = []
        self.hand_traj = []
        self._parse_frames()

        self.low_state = None
        self.hand_state = None
        self.state_received = False
        self.crc = CRC()
        self._stop = False
        self._recorder = None
        self._owns_channels = False

    def _low_state(self):
        """Robot state from own subscriber or attached TeachRecorder."""
        if self._recorder is not None:
            return self._recorder.low_state
        return self.low_state

    def _parse_frames(self):
        for f in self.raw_frames:
            arm = {}
            for j in ARM_JOINTS:
                arm[j] = f["arm"].get(str(j), 0.0)
            self.arm_traj.append((f["t"], arm))

            if self.has_hands and "hand" in f:
                hand = {}
                for k, v in f["hand"].items():
                    hand[int(k)] = v
                self.hand_traj.append((f["t"], hand))

        self.duration = (
            self.arm_traj[-1][0] if self.arm_traj else 0.0
        )
        n = len(self.arm_traj)
        play_dur = self.duration / self.speed
        print(f"Trajectory: {n} frames, "
              f"{self.duration:.1f}s, "
              f"playback {self.speed:.1f}x = {play_dur:.1f}s")

    def _sample_arm(self, t):
        if t <= self.arm_traj[0][0]:
            return self.arm_traj[0][1]
        if t >= self.arm_traj[-1][0]:
            return self.arm_traj[-1][1]
        for i in range(len(self.arm_traj) - 1):
            t0, f0 = self.arm_traj[i]
            t1, f1 = self.arm_traj[i + 1]
            if t0 <= t <= t1:
                a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
                return lerp_frame(f0, f1, a)
        return self.arm_traj[-1][1]

    def _sample_hand(self, t):
        if not self.hand_traj:
            return None
        if t <= self.hand_traj[0][0]:
            return self.hand_traj[0][1]
        if t >= self.hand_traj[-1][0]:
            return self.hand_traj[-1][1]
        for i in range(len(self.hand_traj) - 1):
            t0, f0 = self.hand_traj[i]
            t1, f1 = self.hand_traj[i + 1]
            if t0 <= t <= t1:
                a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
                return lerp_frame(f0, f1, a)
        return self.hand_traj[-1][1]

    def init(self, recorder=None):
        """Init DDS channels, or reuse an existing TeachRecorder session."""
        if recorder is not None:
            self._recorder = recorder
            self.arm_sdk_pub = recorder.arm_sdk_pub
            self.left_hand_pub = recorder.left_hand_pub
            self.right_hand_pub = recorder.right_hand_pub
            self._owns_channels = False
            print("Replay: using existing teach session DDS channels")
            return

        self._owns_channels = True
        self.arm_sdk_pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_sdk_pub.Init()
        self.lowstate_sub = ChannelSubscriber(
            "rt/lowstate", LowState_
        )
        self.lowstate_sub.Init(self._on_low_state, 10)

        self.left_hand_pub = ChannelPublisher(
            TOPIC_LEFT_HAND_CMD, HandCmd_
        )
        self.left_hand_pub.Init()
        self.right_hand_pub = ChannelPublisher(
            TOPIC_RIGHT_HAND_CMD, HandCmd_
        )
        self.right_hand_pub.Init()
        if self.has_hands:
            self.left_hand_sub = ChannelSubscriber(
                TOPIC_LEFT_HAND_STATE, HandState_
            )
            self.left_hand_sub.Init(
                self._on_hand_state, 10
            )
            self.right_hand_sub = ChannelSubscriber(
                TOPIC_RIGHT_HAND_STATE, HandState_
            )
            self.right_hand_sub.Init(
                self._on_hand_state, 10
            )

    def _on_low_state(self, msg: LowState_):
        self.low_state = msg
        if not self.state_received:
            self.state_received = True

    def _on_hand_state(self, msg: HandState_):
        self.hand_state = msg

    def wait_for_state(self, timeout=5.0):
        if self._recorder is not None:
            return self._recorder.wait_for_state(timeout)
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.state_received:
                return True
            time.sleep(0.1)
        return False

    def _get_current_arm_pos(self):
        ls = self._low_state()
        if ls is None:
            return {j: 0.0 for j in ARM_JOINTS}
        pos = {}
        for j in ARM_JOINTS:
            pos[j] = float(ls.motor_state[j].q)
        return pos

    def _send_arm_cmd(self, positions, weight=1.0):
        """Publish arm_sdk command with gravity-compensated arm positions."""
        ls = self._low_state()
        if ls is None:
            return

        cmd = unitree_hg_msg_dds__LowCmd_()
        cmd.motor_cmd[ARM_SDK_ENABLE_IDX].q = weight

        grav = {}
        if self.grav_comp and ls:
            grav = self.grav_comp.compute(ls)

        for j in ARM_JOINTS:
            cmd.motor_cmd[j].mode = 1
            cmd.motor_cmd[j].q = float(
                positions.get(j, 0.0)
            )
            cmd.motor_cmd[j].dq = 0.0
            cmd.motor_cmd[j].tau = grav.get(j, 0.0)
            cmd.motor_cmd[j].kp = KP_ARM
            cmd.motor_cmd[j].kd = KD_ARM

        for j in WAIST_JOINTS:
            cmd.motor_cmd[j].mode = 1
            cmd.motor_cmd[j].q = float(
                ls.motor_state[j].q
            )
            cmd.motor_cmd[j].dq = 0.0
            cmd.motor_cmd[j].tau = grav.get(j, 0.0)
            cmd.motor_cmd[j].kp = KP_ARM
            cmd.motor_cmd[j].kd = KD_ARM

        cmd.crc = self.crc.Crc(cmd)
        self.arm_sdk_pub.Write(cmd)

    def _build_hand_cmd(self, q_targets, kp=1.5, kd=0.2):
        cmd = unitree_hg_msg_dds__HandCmd_()
        for i in range(N_HAND_MOTORS_PER_HAND):
            cmd.motor_cmd[i].mode = _hand_motor_mode(i)
            cmd.motor_cmd[i].q = float(q_targets[i])
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].tau = 0.0
            cmd.motor_cmd[i].kp = kp
            cmd.motor_cmd[i].kd = kd
        return cmd

    def _release_hands(self):
        """Drive all fingers to open (q=0) via dex3 topics."""
        print("Opening fingers...")
        zeros = [0.0] * N_HAND_MOTORS_PER_HAND

        steps = int(2.0 / CONTROL_DT)
        for i in range(steps):
            ratio = i / steps
            cmd = self._build_hand_cmd(
                zeros, kp=KP_HAND * ratio, kd=KD_HAND
            )
            self.left_hand_pub.Write(cmd)
            self.right_hand_pub.Write(cmd)
            time.sleep(CONTROL_DT)

        for _ in range(int(0.5 / CONTROL_DT)):
            cmd = self._build_hand_cmd(
                zeros, kp=KP_HAND, kd=KD_HAND
            )
            self.left_hand_pub.Write(cmd)
            self.right_hand_pub.Write(cmd)
            time.sleep(CONTROL_DT)

        print("Releasing hand control...")
        steps = int(0.5 / CONTROL_DT)
        for i in range(steps):
            w = 1.0 - (i / steps)
            cmd = self._build_hand_cmd(
                zeros, kp=KP_HAND * w, kd=KD_HAND * w
            )
            self.left_hand_pub.Write(cmd)
            self.right_hand_pub.Write(cmd)
            time.sleep(CONTROL_DT)
        print("Hands released.")

    @staticmethod
    def _lerp_hand_q(q0, q1, alpha):
        return [q0[i] + (q1[i] - q0[i]) * alpha for i in range(N_HAND_MOTORS_PER_HAND)]

    def _send_dual_hand_cmds(self, left_q, right_q, kp, kd):
        self.left_hand_pub.Write(
            self._build_hand_cmd(left_q, kp=kp, kd=kd)
        )
        self.right_hand_pub.Write(
            self._build_hand_cmd(right_q, kp=kp, kd=kd)
        )

    def _close_hands(self, duration=PREPARE_CLOSE_HANDS_SEC):
        if not self.has_hands:
            return
        print("  Closing fingers...")
        steps = max(1, int(duration / CONTROL_DT))
        for i in range(steps):
            ratio = smooth_ratio((i + 1) / steps)
            left = self._lerp_hand_q(OPEN_HAND_Q, CLOSED_LEFT_HAND_Q, ratio)
            right = self._lerp_hand_q(OPEN_HAND_Q, CLOSED_RIGHT_HAND_Q, ratio)
            self._send_dual_hand_cmds(
                left, right,
                kp=CLOSE_HAND_KP * ratio,
                kd=CLOSE_HAND_KD,
            )
            time.sleep(CONTROL_DT)
        self._send_dual_hand_cmds(
            CLOSED_LEFT_HAND_Q, CLOSED_RIGHT_HAND_Q,
            kp=CLOSE_HAND_KP, kd=CLOSE_HAND_KD,
        )

    def _hand_targets_from_dict(self, hand_pos):
        if hand_pos is None:
            return list(OPEN_HAND_Q), list(OPEN_HAND_Q)
        left = [hand_pos.get(i, 0.0) for i in range(N_HAND_MOTORS_PER_HAND)]
        right = [
            hand_pos.get(i + 7, 0.0) for i in range(N_HAND_MOTORS_PER_HAND)
        ]
        return left, right

    def _send_hand_cmd(self, positions, kp_scale=1.0):
        if positions is None:
            return
        left_q, right_q = self._hand_targets_from_dict(positions)
        self._send_dual_hand_cmds(
            left_q, right_q,
            kp=KP_HAND * kp_scale,
            kd=KD_HAND * kp_scale,
        )

    def request_stop(self):
        """Stop playback (safe from Gradio / worker threads)."""
        self._stop = True

    def play(self):
        self._stop = False
        # Gradio invokes callbacks off the main thread; signal only works there.
        if threading.current_thread() is threading.main_thread():
            try:
                signal.signal(
                    signal.SIGINT,
                    lambda s, f: setattr(self, '_stop', True),
                )
            except ValueError:
                pass

        current_arm = self._get_current_arm_pos()
        target_arm = self.arm_traj[0][1]
        target_hand = self._sample_hand(0.0)
        end_left, end_right = self._hand_targets_from_dict(target_hand)

        if self.has_hands:
            self._close_hands()

        # Phase 1: Enable arm_sdk with ramping weight (fingers stay closed)
        print("\nEngaging arm control...")
        steps = int(2.0 / CONTROL_DT)
        for i in range(steps):
            if self._stop:
                return
            w = smooth_ratio(i / steps)
            self._send_arm_cmd(current_arm, weight=w)
            if self.has_hands:
                self._send_dual_hand_cmds(
                    CLOSED_LEFT_HAND_Q, CLOSED_RIGHT_HAND_Q,
                    kp=CLOSE_HAND_KP, kd=CLOSE_HAND_KD,
                )
            time.sleep(CONTROL_DT)

        # Phase 2: Move to start; open fingers closed -> target while moving
        print("Moving to start (opening hands)...")
        steps = int(MOVE_TO_START_DURATION / CONTROL_DT)
        for i in range(steps):
            if self._stop:
                return
            s = smooth_ratio(i / steps)
            blended = lerp_frame(current_arm, target_arm, s)
            self._send_arm_cmd(blended)
            if self.has_hands:
                left = self._lerp_hand_q(CLOSED_LEFT_HAND_Q, end_left, s)
                right = self._lerp_hand_q(CLOSED_RIGHT_HAND_Q, end_right, s)
                self._send_dual_hand_cmds(
                    left, right,
                    kp=KP_HAND * max(s, 0.15),
                    kd=KD_HAND,
                )
            time.sleep(CONTROL_DT)

        # Phase 3: Play trajectory
        print("Playing trajectory...")
        play_duration = self.duration / self.speed
        play_t = 0.0
        frame_count = 0
        while play_t <= play_duration and not self._stop:
            traj_t = min(play_t * self.speed, self.duration)
            arm_pos = self._sample_arm(traj_t)
            hand_pos = self._sample_hand(traj_t)

            self._send_arm_cmd(arm_pos)
            self._send_hand_cmd(hand_pos)

            time.sleep(CONTROL_DT)
            play_t += CONTROL_DT
            frame_count += 1

            if frame_count % 50 == 0:
                pct = min(100, traj_t / self.duration * 100)
                print(f"  {pct:5.1f}% "
                      f"({traj_t:.1f}/{self.duration:.1f}s)")

        print("  100.0% done!")

        # Phase 4: Smoothly release arm_sdk + hands
        print("Releasing control...")
        last_arm = self._sample_arm(self.duration)
        last_hand = self._sample_hand(self.duration)
        steps = int(2.0 / CONTROL_DT)
        for i in range(steps):
            if self._stop:
                break
            w = 1.0 - smooth_ratio(i / steps)
            self._send_arm_cmd(last_arm, weight=w)
            if last_hand is not None:
                self._send_hand_cmd(
                    last_hand, kp_scale=w
                )
            time.sleep(CONTROL_DT)

        self._release_hands()
        print("Replay complete!")


def ensure_ai_mode(interactive=True):
    """Check that the locomotion controller (ai sport) is active."""
    msc = MotionSwitcherClient()
    msc.SetTimeout(5.0)
    msc.Init()

    code, result = msc.CheckMode()
    current = result.get("name", "") if result else ""
    print(f"Current mode: '{current}' (form={result})")

    if current == "ai":
        print("OK — ai sport client is active.")
        return True

    print("ai mode not detected. Attempting "
          "SelectMode('ai')...")
    code, _ = msc.SelectMode("ai")
    print(f"  SelectMode returned code={code}")
    time.sleep(3)

    code, result = msc.CheckMode()
    current = result.get("name", "") if result else ""
    print(f"  Mode after select: '{current}'")

    if current == "ai":
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
        description="Trajectory Replay (arm_sdk)"
    )
    parser.add_argument(
        "trajectory",
        help="Path to trajectory JSON file"
    )
    parser.add_argument(
        "--speed", type=float, default=1.0,
        help="Playback speed multiplier (default: 1.0)"
    )
    parser.add_argument(
        "--loop", type=int, default=1,
        help="Number of loops (default: 1)"
    )
    parser.add_argument(
        "network", nargs="?", default=None,
        help="Network interface (optional)"
    )
    args = parser.parse_args()

    if not os.path.exists(args.trajectory):
        print(f"ERROR: File not found: {args.trajectory}")
        sys.exit(1)

    with open(args.trajectory) as f:
        traj_data = json.load(f)

    print("=" * 50)
    print("  G1 Trajectory Replay")
    print("  (arm_sdk + Unitree balance controller)")
    print("=" * 50)
    meta = traj_data["metadata"]
    print(f"File: {args.trajectory}")
    print(f"Recorded: {meta.get('created', 'unknown')}")
    print(f"Duration: {meta['duration_s']}s "
          f"({meta['n_frames']} frames)")
    print(f"Speed: {args.speed}x, Loops: {args.loop}")

    if args.network:
        ChannelFactoryInitialize(0, args.network)
    else:
        ChannelFactoryInitialize(0)

    ensure_ai_mode()

    grav = GravityCompensator()
    player = TrajectoryPlayer(
        traj_data, speed=args.speed, grav_comp=grav
    )
    player.init()

    print("Waiting for robot state...")
    if not player.wait_for_state():
        print("ERROR: No robot state received!")
        sys.exit(1)
    print("Robot connected!")

    for i in range(args.loop):
        if args.loop > 1:
            print(f"\n--- Loop {i+1}/{args.loop} ---")
        player.play()
        if player._stop:
            break

    print("\nDone!")


if __name__ == "__main__":
    main()
