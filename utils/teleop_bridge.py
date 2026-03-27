#!/usr/bin/env python3
"""
Teleoperation Bridge: TWIST2 (PICO VR via Redis) → G1 upper-body control (DDS).

Reads retargeted arm joint angles and hand commands from Redis
(published by TWIST2's teleop pipeline) and sends them to the real robot
via rt/arm_sdk and rt/dex3/*/cmd.  Optionally records trajectories in
the same JSON format as teach.py so they can be replayed by
collect_dataset.py without any changes.

Default mode: arms only (14 DoF — left arm 7 + right arm 7).
Use --with-waist to also control waist joints (17 DoF total).
Neck data from TWIST2 is always ignored (fixed built-in camera).

Hand finger remapping: TWIST2 uses `unitree_interface` (C++ binding)
which has a different finger ordering for the LEFT hand compared to our
DDS HandCmd_ protocol.  The bridge automatically remaps before sending.

Architecture:
    ┌─────────────────────────────────────────────┐
    │ TWIST2 (conda: gmr)                        │
    │ PICO 4 Ultra → GMR retarget → Redis         │
    │   action_body_unitree_g1_with_hands (35D)   │
    │   action_hand_left/right_*           (7D)   │
    └──────────────────┬──────────────────────────┘
                       │ Redis
    ┌──────────────────▼──────────────────────────┐
    │ This script (conda: lerobot)                │
    │ Redis → extract upper body → rt/arm_sdk     │
    │                             → rt/dex3/*/cmd │
    │                             → trajectory JSON│
    └─────────────────────────────────────────────┘

Usage:
    # Real robot (Redis must be running, TWIST2 teleop active)
    python teleop_bridge.py --redis-ip localhost

    # Mock mode for testing without robot or TWIST2
    python teleop_bridge.py --mock

    # Record trajectory while teleoperating
    python teleop_bridge.py --record -o trajectories/teleop_001.json
"""

import argparse
import json
import os
import signal
import sys
import threading
import time
from dataclasses import dataclass, field

import numpy as np

# ─── Constants ────────────────────────────────────────────────────────────

WAIST_JOINTS = [12, 13, 14]
LEFT_ARM_JOINTS = list(range(15, 22))
RIGHT_ARM_JOINTS = list(range(22, 29))
ARM_JOINTS = list(range(15, 29))
ARM_SDK_JOINTS = WAIST_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS
ARMS_ONLY_JOINTS = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS   # 14 DoF, no waist
ARM_SDK_ENABLE_IDX = 29

N_HAND_MOTORS = 7

# ─── Hand Finger Remapping ────────────────────────────────────────────────
# TWIST2 uses `unitree_interface` (C++ binding) while our repo uses DDS
# `rt/dex3/*/cmd` via `unitree_sdk2py`.  The two SDKs have DIFFERENT finger
# orderings for the Dex3-1 hand:
#
# TWIST2 (unitree_interface):
#   Left:  Thumb(0,1,2), Middle(0,1), Index(0,1)
#   Right: Thumb(0,1,2), Index(0,1),  Middle(0,1)
#
# Our repo (unitree_sdk2py / DDS HandCmd_):
#   Left:  Thumb(0,1,2), Index(0,1),  Middle(0,1)
#   Right: Thumb(0,1,2), Index(0,1),  Middle(0,1)
#
# So for the LEFT hand, Index and Middle are SWAPPED between the two SDKs.
# The right hand happens to match.
#
# These permutation arrays map: our_dds[i] = twist2_data[REMAP[i]]

HAND_REMAP_LEFT = [0, 1, 2, 5, 6, 3, 4]   # twist2 Mid→our Idx, twist2 Idx→our Mid
HAND_REMAP_RIGHT = [0, 1, 2, 3, 4, 5, 6]  # right hand: same ordering


def remap_hand_left(twist2_hand):
    """Remap TWIST2 left hand ordering to DDS HandCmd_ ordering."""
    return np.array([twist2_hand[i] for i in HAND_REMAP_LEFT])


def remap_hand_right(twist2_hand):
    """Remap TWIST2 right hand ordering to DDS HandCmd_ ordering."""
    return np.array([twist2_hand[i] for i in HAND_REMAP_RIGHT])


CONTROL_HZ = 50
CONTROL_DT = 1.0 / CONTROL_HZ

# PD gains for position tracking during teleop
KP_ARM = 60.0
KD_ARM = 2.0
KP_WAIST = 60.0
KD_WAIST = 2.0
KP_HAND = 1.5
KD_HAND = 0.2

MAX_DELTA_PER_STEP = 0.08  # rad — conservative limit per 50Hz step

# Joint limits from URDF (radians)
JOINT_LIMITS = {
    12: (-2.618, 2.618),   13: (-0.52, 0.52),     14: (-0.52, 0.52),
    15: (-3.0892, 2.6704), 16: (-1.5882, 2.2515), 17: (-2.618, 2.618),
    18: (-1.0472, 2.0944), 19: (-1.9722, 1.9722), 20: (-1.6144, 1.6144),
    21: (-1.6144, 1.6144),
    22: (-3.0892, 2.6704), 23: (-2.2515, 1.5882), 24: (-2.618, 2.618),
    25: (-1.0472, 2.0944), 26: (-1.9722, 1.9722), 27: (-1.6144, 1.6144),
    28: (-1.6144, 1.6144),
}

ARM_JOINT_NAMES = {
    15: "L_ShoulderPitch", 16: "L_ShoulderRoll",
    17: "L_ShoulderYaw",   18: "L_Elbow",
    19: "L_WristRoll",     20: "L_WristPitch",     21: "L_WristYaw",
    22: "R_ShoulderPitch", 23: "R_ShoulderRoll",
    24: "R_ShoulderYaw",   25: "R_Elbow",
    26: "R_WristRoll",     27: "R_WristPitch",     28: "R_WristYaw",
}

HAND_JOINT_NAMES = [
    "L_Thumb0", "L_Thumb1", "L_Thumb2",
    "L_Index0", "L_Index1", "L_Middle0", "L_Middle1",
    "R_Thumb0", "R_Thumb1", "R_Thumb2",
    "R_Index0", "R_Index1", "R_Middle0", "R_Middle1",
]

# TWIST2 Redis keys
REDIS_KEY_BODY = "action_body_unitree_g1_with_hands"
REDIS_KEY_HAND_LEFT = "action_hand_left_unitree_g1_with_hands"
REDIS_KEY_HAND_RIGHT = "action_hand_right_unitree_g1_with_hands"
REDIS_KEY_TIMESTAMP = "t_action"

# TWIST2 mimic_obs layout (35D):
#   [0:2]  base_vel_xy      — ignored (lower body)
#   [2:3]  base_height_z    — ignored
#   [3:5]  roll, pitch      — ignored
#   [5:6]  yaw_angular_vel  — ignored
#   [6:35] 29 dof_pos:
#          [6:12]  left_leg   — ignored
#          [12:18] right_leg  — ignored
#          [18:21] waist      — ignored by default (use --with-waist to enable)
#          [21:28] left_arm   → our LEFT_ARM_JOINTS [15:22]
#          [28:35] right_arm  → our RIGHT_ARM_JOINTS [22:29]
#
# TWIST2 also publishes neck data (2D: yaw, pitch) as a separate Redis key
# "action_neck_unitree_g1_with_hands" — we ignore this entirely because our
# G1 uses a fixed built-in camera with no neck servo.
MIMIC_OBS_DIM = 35
MIMIC_WAIST_SLICE = slice(18, 21)
MIMIC_LEFT_ARM_SLICE = slice(21, 28)
MIMIC_RIGHT_ARM_SLICE = slice(28, 35)


# ─── Joint mapping helpers ────────────────────────────────────────────────

def extract_upper_body_from_mimic_obs(mimic_obs, include_waist=False):
    """Extract arm (and optionally waist) joint angles from TWIST2's 35D mimic observation.

    Args:
        mimic_obs: 35D numpy array from TWIST2 Redis.
        include_waist: If True, include waist joints [12,13,14]. Default False
            (arms-only mode: 14 DOF for tabletop manipulation).

    Returns:
        dict {unitree_joint_idx: angle_rad} — 14 DOF (arms) or 17 DOF (arms + waist).
    """
    assert len(mimic_obs) == MIMIC_OBS_DIM, (
        f"Expected {MIMIC_OBS_DIM}D mimic_obs, got {len(mimic_obs)}"
    )
    positions = {}

    if include_waist:
        waist_vals = mimic_obs[MIMIC_WAIST_SLICE]
        for i, j in enumerate(WAIST_JOINTS):
            positions[j] = float(waist_vals[i])

    left_arm_vals = mimic_obs[MIMIC_LEFT_ARM_SLICE]
    right_arm_vals = mimic_obs[MIMIC_RIGHT_ARM_SLICE]
    for i, j in enumerate(LEFT_ARM_JOINTS):
        positions[j] = float(left_arm_vals[i])
    for i, j in enumerate(RIGHT_ARM_JOINTS):
        positions[j] = float(right_arm_vals[i])
    return positions


def clamp_joints(positions, last_positions=None):
    """Apply URDF joint limits and per-step delta clamping."""
    clamped = {}
    for j, val in positions.items():
        lo, hi = JOINT_LIMITS.get(j, (-3.14, 3.14))
        val = max(lo, min(hi, val))
        if last_positions and j in last_positions:
            delta = val - last_positions[j]
            delta = max(-MAX_DELTA_PER_STEP, min(MAX_DELTA_PER_STEP, delta))
            val = last_positions[j] + delta
        clamped[j] = val
    return clamped


def clamp_hand(hand_positions, lo=-0.5, hi=1.5):
    """Clamp hand motor positions to safe range."""
    return np.clip(hand_positions, lo, hi)


# ─── Redis Reader ─────────────────────────────────────────────────────────

@dataclass
class TeleopFrame:
    """One frame of teleop data from Redis."""
    upper_body: dict = field(default_factory=dict)
    left_hand: np.ndarray = field(default_factory=lambda: np.zeros(N_HAND_MOTORS))
    right_hand: np.ndarray = field(default_factory=lambda: np.zeros(N_HAND_MOTORS))
    timestamp_ms: int = 0
    valid: bool = False


class RedisReader:
    """Read TWIST2 teleop commands from Redis."""

    def __init__(self, host="localhost", port=6379, include_waist=False):
        import redis as _redis
        self.client = _redis.Redis(host=host, port=port, db=0)
        self.client.ping()
        self._include_waist = include_waist
        print(f"Redis connected: {host}:{port}")

    def read(self) -> TeleopFrame:
        frame = TeleopFrame()

        raw_body = self.client.get(REDIS_KEY_BODY)
        if raw_body is None:
            return frame

        mimic_obs = np.array(json.loads(raw_body), dtype=np.float64)
        if len(mimic_obs) != MIMIC_OBS_DIM:
            return frame

        frame.upper_body = extract_upper_body_from_mimic_obs(
            mimic_obs, include_waist=self._include_waist
        )

        raw_left = self.client.get(REDIS_KEY_HAND_LEFT)
        if raw_left is not None:
            twist2_left = np.array(json.loads(raw_left), dtype=np.float64)
            frame.left_hand = remap_hand_left(twist2_left)

        raw_right = self.client.get(REDIS_KEY_HAND_RIGHT)
        if raw_right is not None:
            twist2_right = np.array(json.loads(raw_right), dtype=np.float64)
            frame.right_hand = remap_hand_right(twist2_right)

        raw_ts = self.client.get(REDIS_KEY_TIMESTAMP)
        if raw_ts is not None:
            frame.timestamp_ms = int(raw_ts)

        frame.valid = True
        return frame


class MockRedisReader:
    """Generate synthetic teleop data for testing without TWIST2/Redis."""

    def __init__(self, motion="wave", include_waist=False):
        self._t0 = time.time()
        self._motion = motion
        self._include_waist = include_waist
        joints = ARM_SDK_JOINTS if include_waist else ARMS_ONLY_JOINTS
        print(f"MockRedisReader: generating '{motion}' motion "
              f"({'arms + waist' if include_waist else 'arms only'})")

    def read(self) -> TeleopFrame:
        t = time.time() - self._t0
        frame = TeleopFrame()

        joints = ARM_SDK_JOINTS if self._include_waist else ARMS_ONLY_JOINTS
        positions = {j: 0.0 for j in joints}

        if self._motion == "wave":
            positions[15] = -0.3 + 0.5 * np.sin(2.0 * np.pi * 0.5 * t)
            positions[16] = 0.3
            positions[18] = 0.8 + 0.3 * np.sin(2.0 * np.pi * 0.7 * t)
            positions[22] = -0.3 + 0.5 * np.sin(2.0 * np.pi * 0.5 * t + np.pi)
            positions[23] = -0.3
            positions[25] = 0.8 + 0.3 * np.sin(2.0 * np.pi * 0.7 * t + np.pi)
        elif self._motion == "reach":
            progress = min(t / 5.0, 1.0)
            s = progress * progress * (3 - 2 * progress)
            positions[15] = -1.0 * s
            positions[18] = 1.2 * s
            positions[22] = -1.0 * s
            positions[25] = 1.2 * s
        elif self._motion == "static":
            positions[15] = -0.5
            positions[18] = 0.8
            positions[22] = -0.5
            positions[25] = 0.8

        frame.upper_body = positions
        grip = 0.5 + 0.5 * np.sin(2.0 * np.pi * 0.3 * t)
        frame.left_hand = np.full(N_HAND_MOTORS, grip * 0.8)
        frame.right_hand = np.full(N_HAND_MOTORS, grip * 0.8)
        frame.timestamp_ms = int(time.time() * 1000)
        frame.valid = True
        return frame


# ─── Robot Sender ─────────────────────────────────────────────────────────

class RobotSender:
    """Send commands to G1 via DDS (rt/arm_sdk + rt/dex3)."""

    def __init__(self, network=None, include_waist=False):
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize,
            ChannelPublisher,
            ChannelSubscriber,
        )
        from unitree_sdk2py.idl.default import (
            unitree_hg_msg_dds__LowCmd_,
            unitree_hg_msg_dds__HandCmd_,
        )
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (
            LowCmd_, LowState_, HandCmd_, HandState_,
        )
        from unitree_sdk2py.utils.crc import CRC

        if network:
            ChannelFactoryInitialize(0, network)
        else:
            ChannelFactoryInitialize(0)

        self._include_waist = include_waist
        self.crc = CRC()
        self.low_state = None
        self._lock = threading.Lock()

        self.arm_pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_pub.Init()
        self.lh_pub = ChannelPublisher("rt/dex3/left/cmd", HandCmd_)
        self.lh_pub.Init()
        self.rh_pub = ChannelPublisher("rt/dex3/right/cmd", HandCmd_)
        self.rh_pub.Init()

        self.state_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.state_sub.Init(self._on_state, 10)
        print(f"DDS initialized ({'arms + waist' if include_waist else 'arms only'}), "
              "waiting for robot state...")

    def _on_state(self, msg):
        with self._lock:
            self.low_state = msg

    def wait_for_state(self, timeout=5.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self._lock:
                if self.low_state is not None:
                    return True
            time.sleep(0.05)
        return False

    def get_current_positions(self):
        active_joints = ARM_SDK_JOINTS if self._include_waist else ARMS_ONLY_JOINTS
        with self._lock:
            if self.low_state is None:
                return {j: 0.0 for j in active_joints}
            return {
                j: float(self.low_state.motor_state[j].q)
                for j in active_joints
            }

    def send_arm(self, positions):
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_

        cmd = unitree_hg_msg_dds__LowCmd_()
        cmd.motor_cmd[ARM_SDK_ENABLE_IDX].q = 1.0

        # rt/arm_sdk requires all 17 joints; lock waist at 0 if arms-only
        for j in ARM_SDK_JOINTS:
            cmd.motor_cmd[j].mode = 1
            cmd.motor_cmd[j].dq = 0.0
            cmd.motor_cmd[j].tau = 0.0
            if j in WAIST_JOINTS and not self._include_waist:
                cmd.motor_cmd[j].q = 0.0
                cmd.motor_cmd[j].kp = KP_WAIST
                cmd.motor_cmd[j].kd = KD_WAIST
            else:
                cmd.motor_cmd[j].q = float(positions.get(j, 0.0))
                kp = KP_WAIST if j in WAIST_JOINTS else KP_ARM
                kd = KD_WAIST if j in WAIST_JOINTS else KD_ARM
                cmd.motor_cmd[j].kp = kp
                cmd.motor_cmd[j].kd = kd

        cmd.crc = self.crc.Crc(cmd)
        self.arm_pub.Write(cmd)

    def send_hands(self, left, right):
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_

        def _hand_motor_mode(motor_id, status=0x01, timeout=0):
            return (motor_id & 0x0F) | ((status & 0x07) << 4) | ((timeout & 0x01) << 7)

        for pub, positions in [(self.lh_pub, left), (self.rh_pub, right)]:
            hcmd = unitree_hg_msg_dds__HandCmd_()
            for i in range(N_HAND_MOTORS):
                hcmd.motor_cmd[i].mode = _hand_motor_mode(i)
                hcmd.motor_cmd[i].q = float(positions[i])
                hcmd.motor_cmd[i].kp = KP_HAND
                hcmd.motor_cmd[i].kd = KD_HAND
            pub.Write(hcmd)

    def release(self):
        """Smoothly release arm_sdk control."""
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_

        print("Releasing arm control...")
        steps = int(2.0 / CONTROL_DT)
        for i in range(steps):
            w = 1.0 - (i / steps)
            cmd = unitree_hg_msg_dds__LowCmd_()
            cmd.motor_cmd[ARM_SDK_ENABLE_IDX].q = w
            with self._lock:
                if self.low_state:
                    for j in ARM_SDK_JOINTS:
                        cmd.motor_cmd[j].mode = 1
                        cmd.motor_cmd[j].q = self.low_state.motor_state[j].q
                        cmd.motor_cmd[j].kp = 60.0 * w
                        cmd.motor_cmd[j].kd = 1.5 * w
            cmd.crc = self.crc.Crc(cmd)
            self.arm_pub.Write(cmd)
            time.sleep(CONTROL_DT)
        print("Released.")


class MockRobotSender:
    """Print commands instead of sending to real robot."""

    def __init__(self, include_waist=False):
        self._include_waist = include_waist
        active_joints = ARM_SDK_JOINTS if include_waist else ARMS_ONLY_JOINTS
        self._positions = {j: 0.0 for j in active_joints}
        self._step = 0
        print(f"MockRobotSender: commands will be logged "
              f"({'arms + waist' if include_waist else 'arms only'})")

    def wait_for_state(self, timeout=5.0):
        return True

    def get_current_positions(self):
        return dict(self._positions)

    def send_arm(self, positions):
        self._positions.update(positions)
        self._step += 1
        if self._step % (CONTROL_HZ * 2) == 0:
            larm = [positions.get(j, 0.0) for j in LEFT_ARM_JOINTS[:4]]
            rarm = [positions.get(j, 0.0) for j in RIGHT_ARM_JOINTS[:4]]
            parts = [f"L_arm={[f'{v:.2f}' for v in larm]}  "
                     f"R_arm={[f'{v:.2f}' for v in rarm]}"]
            if self._include_waist:
                waist = [positions.get(j, 0.0) for j in WAIST_JOINTS]
                parts.insert(0, f"waist={[f'{v:.2f}' for v in waist]}  ")
            print(f"  [step {self._step:5d}] {''.join(parts)}")

    def send_hands(self, left, right):
        pass

    def release(self):
        print("MockRobotSender: release (no-op)")


# ─── Trajectory Recorder ─────────────────────────────────────────────────

class TrajectoryRecorder:
    """Record teleop frames to teach.py-compatible JSON."""

    def __init__(self, record_hands=True):
        self.record_hands = record_hands
        self.frames = []
        self._recording = False
        self._start_time = None

    @property
    def recording(self):
        return self._recording

    def start(self):
        self.frames = []
        self._recording = True
        self._start_time = time.time()
        print("** RECORDING STARTED **")

    def stop(self):
        self._recording = False
        n = len(self.frames)
        dur = self.frames[-1]["t"] if self.frames else 0
        print(f"** RECORDING STOPPED ** {n} frames, {dur:.1f}s")

    def add_frame(self, upper_body, left_hand=None, right_hand=None):
        if not self._recording:
            return
        t = time.time() - self._start_time
        frame = {"t": round(t, 4)}

        arm = {}
        for j in ARM_JOINTS:
            arm[str(j)] = round(upper_body.get(j, 0.0), 6)
        frame["arm"] = arm

        if self.record_hands and left_hand is not None and right_hand is not None:
            hand = {}
            for i in range(N_HAND_MOTORS):
                hand[str(i)] = round(float(left_hand[i]), 6)
                hand[str(i + N_HAND_MOTORS)] = round(float(right_hand[i]), 6)
            frame["hand"] = hand

        self.frames.append(frame)

    def save(self, path):
        if not self.frames:
            print("No frames to save.")
            return None

        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)

        metadata = {
            "record_hz": CONTROL_HZ,
            "n_frames": len(self.frames),
            "duration_s": round(self.frames[-1]["t"], 2),
            "arm_joints": ARM_JOINTS,
            "arm_joint_names": ARM_JOINT_NAMES,
            "record_hands": self.record_hands,
            "control_mode": "teleop_bridge",
            "source": "twist2_pico_vr",
            "created": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        if self.record_hands:
            metadata["hand_joint_names"] = HAND_JOINT_NAMES

        data = {"metadata": metadata, "frames": self.frames}
        with open(path, "w") as f:
            json.dump(data, f, indent=2)

        print(f"Saved: {path}")
        print(f"  Frames: {metadata['n_frames']}")
        print(f"  Duration: {metadata['duration_s']}s")
        print(f"  Hz: {metadata['record_hz']}")
        return path


# ─── Main Loop ────────────────────────────────────────────────────────────

class TeleopBridge:
    """Main orchestrator: read from Redis, send to robot, optionally record."""

    def __init__(self, reader, sender, recorder=None, include_waist=False):
        self.reader = reader
        self.sender = sender
        self.recorder = recorder
        self._include_waist = include_waist
        self._stop = False
        self._last_positions = None

        self.stats = {
            "frames_read": 0,
            "frames_sent": 0,
            "frames_invalid": 0,
            "start_time": None,
        }

    def run(self):
        signal.signal(signal.SIGINT, lambda s, f: setattr(self, '_stop', True))

        mode_str = "Arms + Waist (17 DoF)" if self._include_waist else "Arms Only (14 DoF)"
        print("\n" + "=" * 55)
        print("  TELEOP BRIDGE — PICO VR → G1 Upper Body")
        print("=" * 55)
        print(f"  Mode: {mode_str}")
        print(f"  Control rate: {CONTROL_HZ} Hz")
        print(f"  Max delta/step: {MAX_DELTA_PER_STEP} rad")
        print(f"  Recording: {'ON' if self.recorder else 'OFF'}")
        print("")
        print("Controls:")
        if self.recorder:
            print("  [Enter]  Start/Stop recording")
        print("  [Ctrl+C] Quit and release control")
        print("")

        if not self.sender.wait_for_state():
            print("WARNING: No robot state received, using zeros as baseline.")
        self._last_positions = self.sender.get_current_positions()

        if self.recorder:
            key_pressed = [False]

            def _key_listener():
                while not self._stop:
                    try:
                        input()
                        key_pressed[0] = True
                    except EOFError:
                        break

            threading.Thread(target=_key_listener, daemon=True).start()

        self.stats["start_time"] = time.time()
        print("Bridge running. Waiting for TWIST2 teleop data...\n")

        while not self._stop:
            t_start = time.time()

            frame = self.reader.read()
            self.stats["frames_read"] += 1

            if not frame.valid:
                self.stats["frames_invalid"] += 1
                time.sleep(CONTROL_DT)
                continue

            positions = clamp_joints(frame.upper_body, self._last_positions)
            left_hand = clamp_hand(frame.left_hand)
            right_hand = clamp_hand(frame.right_hand)

            self.sender.send_arm(positions)
            self.sender.send_hands(left_hand, right_hand)
            self._last_positions = positions
            self.stats["frames_sent"] += 1

            if self.recorder:
                if key_pressed[0]:
                    key_pressed[0] = False
                    if not self.recorder.recording:
                        self.recorder.start()
                    else:
                        self.recorder.stop()

                if self.recorder.recording:
                    self.recorder.add_frame(positions, left_hand, right_hand)

            if self.stats["frames_sent"] % (CONTROL_HZ * 5) == 0:
                elapsed = time.time() - self.stats["start_time"]
                fps = self.stats["frames_sent"] / elapsed if elapsed > 0 else 0
                print(
                    f"  [{elapsed:.0f}s] sent={self.stats['frames_sent']} "
                    f"invalid={self.stats['frames_invalid']} "
                    f"fps={fps:.1f}"
                )

            dt = time.time() - t_start
            if dt < CONTROL_DT:
                time.sleep(CONTROL_DT - dt)

        self._shutdown()

    def _shutdown(self):
        print("\nShutting down...")
        self.sender.release()
        self._print_stats()

    def _print_stats(self):
        elapsed = time.time() - self.stats["start_time"] if self.stats["start_time"] else 0
        print(f"\n{'=' * 55}")
        print("  Session Summary")
        print(f"{'=' * 55}")
        print(f"  Duration: {elapsed:.1f}s")
        print(f"  Frames sent: {self.stats['frames_sent']}")
        print(f"  Frames invalid: {self.stats['frames_invalid']}")
        if elapsed > 0:
            print(f"  Average FPS: {self.stats['frames_sent'] / elapsed:.1f}")


def main():
    parser = argparse.ArgumentParser(
        description="Teleop Bridge: TWIST2 PICO VR → G1 upper body"
    )
    parser.add_argument(
        "--redis-ip", default="localhost",
        help="Redis server IP (default: localhost)",
    )
    parser.add_argument(
        "--redis-port", type=int, default=6379,
        help="Redis server port (default: 6379)",
    )
    parser.add_argument(
        "--mock", action="store_true",
        help="Use mock reader+sender for testing without hardware",
    )
    parser.add_argument(
        "--mock-motion", default="wave",
        choices=["wave", "reach", "static"],
        help="Motion pattern for mock mode (default: wave)",
    )
    parser.add_argument(
        "--record", action="store_true",
        help="Enable trajectory recording",
    )
    parser.add_argument(
        "-o", "--output", default=None,
        help="Output trajectory path (default: auto-generated)",
    )
    parser.add_argument(
        "--no-hands", action="store_true",
        help="Skip hand control and recording",
    )
    parser.add_argument(
        "--with-waist", action="store_true",
        help="Also control waist joints [12,13,14] (default: arms only, 14 DoF)",
    )
    parser.add_argument(
        "--network", default=None,
        help="DDS network interface (for real robot)",
    )
    args = parser.parse_args()

    mode_str = "Arms + Waist (17 DoF)" if args.with_waist else "Arms Only (14 DoF)"
    print("=" * 55)
    print("  G1 Teleoperation Bridge")
    print(f"  TWIST2 (PICO VR) → {mode_str}")
    print("=" * 55)

    if args.mock:
        reader = MockRedisReader(motion=args.mock_motion, include_waist=args.with_waist)
        sender = MockRobotSender(include_waist=args.with_waist)
    else:
        reader = RedisReader(host=args.redis_ip, port=args.redis_port,
                             include_waist=args.with_waist)
        sender = RobotSender(network=args.network, include_waist=args.with_waist)

    recorder = None
    if args.record:
        if args.output is None:
            ts = time.strftime("%Y%m%d_%H%M%S")
            traj_dir = os.path.join(os.path.dirname(__file__), "..", "trajectories")
            args.output = os.path.join(traj_dir, f"teleop_{ts}.json")
        recorder = TrajectoryRecorder(record_hands=not args.no_hands)

    bridge = TeleopBridge(reader, sender, recorder, include_waist=args.with_waist)
    bridge.run()

    if recorder and recorder.frames:
        recorder.save(args.output)
        print(f"\nTo replay: python utils/replay.py {args.output}")
        print(f"To collect dataset: bash run_collect.sh --trajectories {args.output}")


if __name__ == "__main__":
    main()
