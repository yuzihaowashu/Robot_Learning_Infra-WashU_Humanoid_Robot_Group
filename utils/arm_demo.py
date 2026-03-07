"""
G1 Arm Demo — Multi-joint choreographed motion with joint limit protection.

Uses direct rt/lowcmd control with joint limits from URDF to prevent
dangerous positions. Designed for testing on a hanging test stand.

Usage:
    conda activate lerobot
    python arm_demo.py
"""

import sys
import time

import numpy as np
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

PI = np.pi

# ---------------------------------------------------------------------------
# Joint indices
# ---------------------------------------------------------------------------

class J:
    L_SHOULDER_P = 15
    L_SHOULDER_R = 16
    L_SHOULDER_Y = 17
    L_ELBOW = 18
    L_WRIST_R = 19
    L_WRIST_P = 20
    L_WRIST_Y = 21

    R_SHOULDER_P = 22
    R_SHOULDER_R = 23
    R_SHOULDER_Y = 24
    R_ELBOW = 25
    R_WRIST_R = 26
    R_WRIST_P = 27
    R_WRIST_Y = 28


ALL_ARM_JOINTS = list(range(15, 29))

# ---------------------------------------------------------------------------
# Joint limits from URDF (with 10% safety margin applied)
# Source: g1_29dof_with_hand_rev_1_0.urdf
# ---------------------------------------------------------------------------

JOINT_LIMITS = {
    # Left arm                  (lower,    upper)   # URDF raw limits
    J.L_SHOULDER_P: (-2.78,  2.40),                 # (-3.0892,  2.6704)
    J.L_SHOULDER_R: (-1.43,  2.03),                 # (-1.5882,  2.2515)
    J.L_SHOULDER_Y: (-2.36,  2.36),                 # (-2.618,   2.618)
    J.L_ELBOW:      (-0.94,  1.88),                 # (-1.0472,  2.0944)
    J.L_WRIST_R:    (-1.77,  1.77),                 # (-1.97222, 1.97222)
    J.L_WRIST_P:    (-1.45,  1.45),                 # (-1.61443, 1.61443)
    J.L_WRIST_Y:    (-1.45,  1.45),                 # (-1.61443, 1.61443)
    # Right arm (note: shoulder_roll limits are mirrored)
    J.R_SHOULDER_P: (-2.78,  2.40),                 # (-3.0892,  2.6704)
    J.R_SHOULDER_R: (-2.03,  1.43),                 # (-2.2515,  1.5882)
    J.R_SHOULDER_Y: (-2.36,  2.36),                 # (-2.618,   2.618)
    J.R_ELBOW:      (-0.94,  1.88),                 # (-1.0472,  2.0944)
    J.R_WRIST_R:    (-1.77,  1.77),                 # (-1.97222, 1.97222)
    J.R_WRIST_P:    (-1.45,  1.45),                 # (-1.61443, 1.61443)
    J.R_WRIST_Y:    (-1.45,  1.45),                 # (-1.61443, 1.61443)
}


def clamp_joint(joint_idx: int, value: float) -> float:
    """Clamp a joint command to its safe limits."""
    if joint_idx in JOINT_LIMITS:
        lo, hi = JOINT_LIMITS[joint_idx]
        return float(np.clip(value, lo, hi))
    return value


def smooth_ratio(t: float) -> float:
    """Hermite smooth step: zero velocity at start and end."""
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


# ---------------------------------------------------------------------------
# Keyframe sequence (all values validated against joint limits)
# ---------------------------------------------------------------------------

KEYFRAMES = [
    ("neutral", 3.0, {
        J.L_SHOULDER_P: 0.0, J.L_SHOULDER_R: 0.0, J.L_SHOULDER_Y: 0.0,
        J.L_ELBOW: 0.0,
        J.L_WRIST_R: 0.0, J.L_WRIST_P: 0.0, J.L_WRIST_Y: 0.0,
        J.R_SHOULDER_P: 0.0, J.R_SHOULDER_R: 0.0, J.R_SHOULDER_Y: 0.0,
        J.R_ELBOW: 0.0,
        J.R_WRIST_R: 0.0, J.R_WRIST_P: 0.0, J.R_WRIST_Y: 0.0,
    }),

    ("arms forward", 2.5, {
        J.L_SHOULDER_P: -0.8,
        J.L_SHOULDER_R: 0.4,     # outward to avoid leg collision
        J.L_ELBOW: 0.4,
        J.R_SHOULDER_P: -0.8,
        J.R_SHOULDER_R: -0.4,    # outward to avoid leg collision
        J.R_ELBOW: 0.4,
    }),

    ("arms spread", 3.0, {
        J.L_SHOULDER_P: -0.3,
        J.L_SHOULDER_R: 1.2,
        J.L_ELBOW: 0.8,
        J.R_SHOULDER_P: -0.3,
        J.R_SHOULDER_R: -1.2,
        J.R_ELBOW: 0.8,
    }),

    ("right arm up", 2.0, {
        J.L_SHOULDER_P: 0.0,
        J.L_SHOULDER_R: 0.2,
        J.L_ELBOW: 0.3,
        J.R_SHOULDER_P: -1.2,
        J.R_SHOULDER_R: -0.3,
        J.R_ELBOW: 1.2,
    }),

    ("wave right hand", 3.0, None),  # handled dynamically

    ("both arms up", 2.5, {
        J.L_SHOULDER_P: -1.5,
        J.L_SHOULDER_R: 0.3,
        J.L_ELBOW: 1.0,
        J.R_SHOULDER_P: -1.5,
        J.R_SHOULDER_R: -0.3,
        J.R_ELBOW: 1.0,
    }),

    ("flexing pose", 2.5, {
        J.L_SHOULDER_P: -0.4,
        J.L_SHOULDER_R: 1.0,
        J.L_ELBOW: 1.8,         # tight bicep curl
        J.L_WRIST_R: 0.5,
        J.R_SHOULDER_P: -0.4,
        J.R_SHOULDER_R: -1.0,
        J.R_ELBOW: 1.8,
        J.R_WRIST_R: -0.5,
    }),

    ("return neutral", 3.0, {
        J.L_SHOULDER_P: 0.0, J.L_SHOULDER_R: 0.0, J.L_SHOULDER_Y: 0.0,
        J.L_ELBOW: 0.0,
        J.L_WRIST_R: 0.0, J.L_WRIST_P: 0.0, J.L_WRIST_Y: 0.0,
        J.R_SHOULDER_P: 0.0, J.R_SHOULDER_R: 0.0, J.R_SHOULDER_Y: 0.0,
        J.R_ELBOW: 0.0,
        J.R_WRIST_R: 0.0, J.R_WRIST_P: 0.0, J.R_WRIST_Y: 0.0,
    }),
]

WAVE_CYCLES = 3
KP = 30.0
KD = 1.5
CONTROL_DT = 0.02
TAKEOVER_DURATION = 3.0
RELEASE_DURATION = 3.0


# ---------------------------------------------------------------------------
# Validate all keyframe targets at import time
# ---------------------------------------------------------------------------

def _validate_keyframes():
    for name, dur, targets in KEYFRAMES:
        if targets is None:
            continue
        for joint, val in targets.items():
            clamped = clamp_joint(joint, val)
            if abs(clamped - val) > 1e-4:
                print(f"  WARNING: keyframe '{name}' joint {joint} "
                      f"value {val:.3f} clamped to {clamped:.3f}")

_validate_keyframes()


# ---------------------------------------------------------------------------
# Demo controller
# ---------------------------------------------------------------------------

class ArmDemo:
    def __init__(self):
        self.time_ = 0.0
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.state_received = False
        self.crc = CRC()
        self.done = False
        self.mode_machine = 0

        self.last_cmd = {}
        self.phase_start_positions = {}
        self.current_phase_idx = -1

        self._build_timeline()

    def _build_timeline(self):
        self.phases = []
        self.phases.append(("takeover", TAKEOVER_DURATION, None))
        for name, dur, targets in KEYFRAMES:
            self.phases.append((name, dur, targets))
        self.phases.append(("release", RELEASE_DURATION, None))

        self.phase_starts = []
        t = 0.0
        for _, dur, _ in self.phases:
            self.phase_starts.append(t)
            t += dur
        self.total_duration = t

    def init(self):
        self.lowcmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher.Init()
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self._on_low_state, 10)

    def start(self):
        print("Waiting for robot state...")
        while not self.state_received:
            time.sleep(0.1)

        for joint in ALL_ARM_JOINTS:
            q = self.low_state.motor_state[joint].q
            self.last_cmd[joint] = q
            self.phase_start_positions[joint] = q

        print("\nCurrent arm positions:")
        for joint in ALL_ARM_JOINTS:
            q = self.last_cmd[joint]
            lo, hi = JOINT_LIMITS.get(joint, (-999, 999))
            print(f"  [{joint:2d}] q={q:+.4f} ({np.degrees(q):+6.1f} deg)  "
                  f"limits=[{lo:+.2f}, {hi:+.2f}]")

        print(f"\nMotion sequence ({self.total_duration:.1f}s total):")
        for i, (name, dur, _) in enumerate(self.phases):
            print(f"  {i+1:2d}. {name} ({dur:.1f}s)")

        input("\nPress Enter to start demo (Ctrl+C to abort)...")
        self.control_thread = RecurrentThread(
            interval=CONTROL_DT, target=self._control_loop, name="arm_demo"
        )
        self.control_thread.Start()

    def _on_low_state(self, msg: LowState_):
        self.low_state = msg
        self.mode_machine = msg.mode_machine
        if not self.state_received:
            self.state_received = True

    def _get_phase(self):
        for i in range(len(self.phases) - 1, -1, -1):
            if self.time_ >= self.phase_starts[i]:
                name, dur, targets = self.phases[i]
                local_t = self.time_ - self.phase_starts[i]
                ratio = np.clip(local_t / dur, 0.0, 1.0) if dur > 0 else 1.0
                return i, name, ratio, targets
        return 0, self.phases[0][0], 0.0, None

    def _on_phase_change(self, new_phase_idx):
        for joint in ALL_ARM_JOINTS:
            self.phase_start_positions[joint] = self.last_cmd[joint]

    def _compute_targets(self, phase_name, ratio, keyframe_targets):
        targets = {}
        s = smooth_ratio(ratio)

        if phase_name == "wave right hand":
            wave_phase = ratio * WAVE_CYCLES * 2 * PI
            wave_targets = {
                J.R_SHOULDER_P: -1.2 + 0.3 * np.sin(wave_phase),
                J.R_ELBOW: 1.2 + 0.4 * np.sin(wave_phase + PI / 2),
                J.R_WRIST_Y: 0.5 * np.sin(wave_phase * 1.5),
            }
            for joint in ALL_ARM_JOINTS:
                if joint in wave_targets:
                    targets[joint] = wave_targets[joint]
                else:
                    targets[joint] = self.phase_start_positions[joint]
        elif keyframe_targets:
            for joint in ALL_ARM_JOINTS:
                start = self.phase_start_positions[joint]
                end = keyframe_targets.get(joint, start)
                targets[joint] = start + (end - start) * s
        else:
            for joint in ALL_ARM_JOINTS:
                targets[joint] = self.phase_start_positions[joint]

        # Enforce joint limits on every target
        for joint in ALL_ARM_JOINTS:
            targets[joint] = clamp_joint(joint, targets[joint])

        return targets

    def _control_loop(self):
        self.time_ += CONTROL_DT

        if self.time_ >= self.total_duration:
            self.done = True
            return

        phase_idx, phase_name, ratio, keyframe_targets = self._get_phase()

        if phase_idx != self.current_phase_idx:
            self._on_phase_change(phase_idx)
            self.current_phase_idx = phase_idx

        self.low_cmd.mode_pr = 0
        self.low_cmd.mode_machine = self.mode_machine

        if phase_name == "takeover":
            s = smooth_ratio(ratio)
            for joint in ALL_ARM_JOINTS:
                pos = self.phase_start_positions[joint]
                self.low_cmd.motor_cmd[joint].mode = 1
                self.low_cmd.motor_cmd[joint].q = clamp_joint(joint, pos)
                self.low_cmd.motor_cmd[joint].dq = 0.0
                self.low_cmd.motor_cmd[joint].tau = 0.0
                self.low_cmd.motor_cmd[joint].kp = KP * s
                self.low_cmd.motor_cmd[joint].kd = KD * s
                self.last_cmd[joint] = pos

        elif phase_name == "release":
            s = smooth_ratio(ratio)
            for joint in ALL_ARM_JOINTS:
                pos = self.phase_start_positions[joint]
                self.low_cmd.motor_cmd[joint].mode = 1
                self.low_cmd.motor_cmd[joint].q = clamp_joint(joint, pos)
                self.low_cmd.motor_cmd[joint].dq = 0.0
                self.low_cmd.motor_cmd[joint].tau = 0.0
                self.low_cmd.motor_cmd[joint].kp = KP * (1.0 - s)
                self.low_cmd.motor_cmd[joint].kd = KD * (1.0 - s)
                self.last_cmd[joint] = pos

        else:
            targets = self._compute_targets(phase_name, ratio, keyframe_targets)
            for joint in ALL_ARM_JOINTS:
                self.low_cmd.motor_cmd[joint].mode = 1
                self.low_cmd.motor_cmd[joint].q = targets[joint]
                self.low_cmd.motor_cmd[joint].dq = 0.0
                self.low_cmd.motor_cmd[joint].tau = 0.0
                self.low_cmd.motor_cmd[joint].kp = KP
                self.low_cmd.motor_cmd[joint].kd = KD
                self.last_cmd[joint] = targets[joint]

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher.Write(self.low_cmd)

        print(f"\r  [{phase_idx+1}/{len(self.phases)}] {phase_name:<22s} | "
              f"t={self.time_:5.1f}/{self.total_duration:.0f}s | "
              f"ratio={ratio:.2f}", end="", flush=True)


def release_motion_controller():
    msc = MotionSwitcherClient()
    msc.SetTimeout(5.0)
    msc.Init()
    code, result = msc.CheckMode()
    print(f"Current mode: {result}")
    if result and result.get("name"):
        print(f"Releasing mode '{result['name']}'...")
        msc.ReleaseMode()
        time.sleep(2)
        code, result = msc.CheckMode()
        print(f"After release: {result}")


if __name__ == "__main__":
    print("=" * 60)
    print("  G1 Arm Demo — Choreographed Motion (with joint limits)")
    print("  Robot should be HANGING on a test stand")
    print("=" * 60)
    print("\nWARNING: Ensure no obstacles around the robot's arms!")

    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    release_motion_controller()

    demo = ArmDemo()
    demo.init()
    demo.start()

    while not demo.done:
        time.sleep(0.5)

    print("\n\nDemo complete! Motors released.")
