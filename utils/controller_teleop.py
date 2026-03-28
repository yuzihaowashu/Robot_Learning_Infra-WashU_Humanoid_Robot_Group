#!/usr/bin/env python3
"""
Controller-based Teleoperation: PICO VR controllers → Redis (same format as TWIST2).

Replaces the TWIST2 body-tracking pipeline with a simpler approach that only
requires VR controller 6DoF poses (no full-body tracking needed).

Data flow:
    PICO headset + controllers (via XRoboToolkit SDK)
      → IK solver (mink + MuJoCo) maps controller pose → arm joint angles
      → Redis (same keys as TWIST2)
      → teleop_bridge.py reads and sends to robot

Prerequisites:
    - conda activate gmr
    - XRoboToolkit PC Service running
    - XRobot app on PICO with controller tracking active
    - Redis server running

Usage:
    conda activate gmr
    python utils/controller_teleop.py --redis-ip localhost

    # Mock mode (no VR hardware, test with synthetic motion):
    python utils/controller_teleop.py --mock
"""

import argparse
import json
import os
import signal
import sys
import time

import mink
import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation as R

# ─── Paths ────────────────────────────────────────────────────────────────

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
GMR_ASSETS = os.path.join(os.path.dirname(REPO_ROOT), "GMR", "assets")
G1_XML = os.path.join(GMR_ASSETS, "unitree_g1", "g1_mocap_29dof.xml")

# ─── Redis keys (must match teleop_bridge.py / TWIST2) ───────────────────

REDIS_KEY_BODY = "action_body_unitree_g1_with_hands"
REDIS_KEY_HAND_LEFT = "action_hand_left_unitree_g1_with_hands"
REDIS_KEY_HAND_RIGHT = "action_hand_right_unitree_g1_with_hands"
REDIS_KEY_TIMESTAMP = "t_action"

# ─── MuJoCo qpos layout (29 DOF after 7D free joint) ─────────────────────
# qpos[0:7]   = free joint (x, y, z, qw, qx, qy, qz)
# qpos[7:13]  = left leg (6)
# qpos[13:19] = right leg (6)
# qpos[19:22] = waist (3)
# qpos[22:29] = left arm (7)
# qpos[29:36] = right arm (7)

QPOS_LEFT_ARM = slice(22, 29)
QPOS_RIGHT_ARM = slice(29, 36)
QPOS_WAIST = slice(19, 22)

# mimic_obs 35D layout
OBS_LEFT_LEG = slice(6, 12)
OBS_RIGHT_LEG = slice(12, 18)
OBS_WAIST = slice(18, 21)
OBS_LEFT_ARM = slice(21, 28)
OBS_RIGHT_ARM = slice(28, 35)

# Default pose: arms moderately forward (ready for manipulation).
# ShoulderPitch=-0.75 (upper arm ~43° fwd), Elbow=0.75 (~43° bend)
DEFAULT_ARM_LEFT = [-0.75, 0.15, 0.0, 0.75, 0.0, 0.0, 0.0]
DEFAULT_ARM_RIGHT = [-0.75, -0.15, 0.0, 0.75, 0.0, 0.0, 0.0]

DEFAULT_MIMIC_OBS = np.concatenate([
    np.array([0, 0]),          # xy velocity
    np.array([0.8]),           # z position
    np.array([0, 0]),          # roll, pitch
    np.array([0]),             # yaw angular velocity
    np.array([
        -0.2, 0.0, 0.0, 0.4, -0.2, 0.0,    # left leg (6)
        -0.2, 0.0, 0.0, 0.4, -0.2, 0.0,    # right leg (6)
        0.0, 0.0, 0.0,                       # waist (3)
    ]),
    np.array(DEFAULT_ARM_LEFT),               # left arm (7)
    np.array(DEFAULT_ARM_RIGHT),              # right arm (7)
])

# Hand open/close poses (TWIST2 ordering: Thumb(0,1,2), Middle(0,1), Index(0,1) for left)
HAND_LEFT_OPEN = np.array([0, 0, 0, 0, 0, 0, 0])
HAND_LEFT_CLOSE = np.array([0, 1.0, 1.74, -1.57, -1.74, -1.57, -1.74])
HAND_RIGHT_OPEN = np.array([0, 0, 0, 0, 0, 0, 0])
HAND_RIGHT_CLOSE = np.array([0, -1.0, -1.74, 1.57, 1.74, 1.57, 1.74])

TARGET_FPS = 30
POSITION_SCALE = 1.0  # scale factor for VR→robot position mapping

# Safety: max shoulder→wrist distance. G1 physical max ~0.41m; we use 85%
# to keep the IK comfortably within joint limits and avoid extreme poses
# that shift the center of mass and make the balance controller move the feet.
MAX_ARM_REACH = 0.40



# ─── Coordinate transform ────────────────────────────────────────────────

def unity_to_robot(pos):
    """Transform position from Unity/PICO to MuJoCo robot frame.

    Unity:  X=right, Y=up,  Z=forward  (left-handed)
    Robot:  X=forward, Y=left, Z=up    (right-handed, MuJoCo G1)

    Mapping:
        robot_X = unity_Z   (forward)
        robot_Y = -unity_X  (left = -right)
        robot_Z = unity_Y   (up)
    """
    x, y, z = np.asarray(pos)
    return np.array([z, -x, y])


def unity_quat_to_robot(quat_wxyz):
    """Transform quaternion from Unity frame to robot frame.

    Applies the same axis remapping as unity_to_robot to the rotation.
    Input: [w, x, y, z] in Unity frame
    Output: scipy Rotation in robot frame
    """
    w, qx, qy, qz = quat_wxyz
    # Remap quaternion imaginary components to match position axis remapping:
    # robot_x ← unity_z, robot_y ← -unity_x, robot_z ← unity_y
    return R.from_quat([qz, -qx, qy, w])


# ─── Arm IK Solver ───────────────────────────────────────────────────────

class ArmIKSolver:
    """Solve arm IK using mink + MuJoCo, keeping legs/waist fixed."""

    # Freeze non-arm joints; let all 7 arm joints per side (shoulder + elbow + wrist) move.
    FROZEN_QPOS = list(range(0, 22))  # free joint (7) + legs (12) + waist (3)

    def __init__(self, xml_path=G1_XML):
        self.model = mj.MjModel.from_xml_path(xml_path)
        self.data = mj.MjData(self.model)
        self.configuration = mink.Configuration(self.model)

        self._set_default_pose()
        self._frozen_qpos = self.configuration.data.qpos[self.FROZEN_QPOS].copy()

        # Position-only IK: orientation_cost=0 gives maximum freedom to find
        # safe arm configurations and avoids conflicts between position and
        # orientation goals at the workspace boundary.
        self.left_wrist_task = mink.FrameTask(
            frame_name="left_wrist_yaw_link",
            frame_type="body",
            position_cost=50.0,
            orientation_cost=0.0,
            lm_damping=1e-3,
        )
        self.right_wrist_task = mink.FrameTask(
            frame_name="right_wrist_yaw_link",
            frame_type="body",
            position_cost=50.0,
            orientation_cost=0.0,
            lm_damping=1e-3,
        )

        self.tasks = [self.left_wrist_task, self.right_wrist_task]
        self.limits = [mink.ConfigurationLimit(self.model)]

        self.default_left_wrist_pos = self._get_body_pos("left_wrist_yaw_link").copy()
        self.default_left_wrist_rot = self._get_body_rot("left_wrist_yaw_link")
        self.default_right_wrist_pos = self._get_body_pos("right_wrist_yaw_link").copy()
        self.default_right_wrist_rot = self._get_body_rot("right_wrist_yaw_link")

        # Shoulder positions (fixed, since torso is frozen) for workspace clamping.
        self.left_shoulder_pos = self._get_body_pos("left_shoulder_pitch_link").copy()
        self.right_shoulder_pos = self._get_body_pos("right_shoulder_pitch_link").copy()

        print(f"IK Solver initialized. Model: {xml_path}")
        print(f"  Default L wrist: {self.default_left_wrist_pos}")
        print(f"  Default R wrist: {self.default_right_wrist_pos}")
        print(f"  L shoulder: {self.left_shoulder_pos}")
        print(f"  R shoulder: {self.right_shoulder_pos}")
        print(f"  Max arm reach: {MAX_ARM_REACH}")

    def _set_default_pose(self):
        """Set robot to default pose with forearms pointing forward."""
        qpos = np.zeros(self.model.nq)
        qpos[0:3] = [0, 0, 0.793]
        qpos[3:7] = [1, 0, 0, 0]
        qpos[7:13] = [-0.2, 0.0, 0.0, 0.4, -0.2, 0.0]
        qpos[13:19] = [-0.2, 0.0, 0.0, 0.4, -0.2, 0.0]
        qpos[19:22] = [0.0, 0.0, 0.0]
        qpos[22:29] = DEFAULT_ARM_LEFT
        qpos[29:36] = DEFAULT_ARM_RIGHT

        self.configuration.update(qpos)
        mj.mj_forward(self.model, self.configuration.data)

    def _get_body_pos(self, body_name):
        body_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, body_name)
        return self.configuration.data.xpos[body_id].copy()

    def _get_body_rot(self, body_name):
        body_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, body_name)
        rotmat = self.configuration.data.xmat[body_id].reshape(3, 3)
        return R.from_matrix(rotmat)

    def clamp_to_workspace(self, target_pos, shoulder_pos):
        """Clamp target so shoulder→target distance ≤ MAX_ARM_REACH."""
        vec = target_pos - shoulder_pos
        dist = np.linalg.norm(vec)
        if dist > MAX_ARM_REACH:
            target_pos = shoulder_pos + vec * (MAX_ARM_REACH / dist)
        return target_pos

    def solve(self, left_target_pos, left_target_rot,
              right_target_pos, right_target_rot, max_iter=20):
        """Solve IK for both wrists simultaneously.

        Args:
            left_target_pos: (3,) target position for left wrist
            left_target_rot: scipy Rotation for left wrist
            right_target_pos: (3,) target position for right wrist
            right_target_rot: scipy Rotation for right wrist

        Returns:
            left_arm_joints: (7,) joint angles for left arm
            right_arm_joints: (7,) joint angles for right arm
        """
        left_target_pos = self.clamp_to_workspace(
            left_target_pos, self.left_shoulder_pos)
        right_target_pos = self.clamp_to_workspace(
            right_target_pos, self.right_shoulder_pos)

        self.left_wrist_task.set_target(
            mink.SE3.from_rotation_and_translation(
                mink.SO3(left_target_rot.as_quat(scalar_first=True)),
                left_target_pos
            )
        )
        self.right_wrist_task.set_target(
            mink.SE3.from_rotation_and_translation(
                mink.SO3(right_target_rot.as_quat(scalar_first=True)),
                right_target_pos
            )
        )

        dt = 1.0 / TARGET_FPS
        for _ in range(max_iter):
            vel = mink.solve_ik(
                self.configuration, self.tasks, dt,
                solver="daqp", damping=1e-3, limits=self.limits,
            )
            self.configuration.integrate_inplace(vel, dt)
            self.configuration.data.qpos[self.FROZEN_QPOS] = self._frozen_qpos
            mj.mj_forward(self.model, self.configuration.data)

        left_arm = self.configuration.data.qpos[QPOS_LEFT_ARM].copy()
        right_arm = self.configuration.data.qpos[QPOS_RIGHT_ARM].copy()
        return left_arm, right_arm

    def reset_to_default(self):
        """Reset the configuration to standing pose."""
        self._set_default_pose()


# ─── VR Input Source ─────────────────────────────────────────────────────

class VRInputSource:
    """Read controller data from PICO via XRoboToolkit SDK."""

    def __init__(self):
        import xrobotoolkit_sdk as xrt
        self.xrt = xrt
        xrt.init()
        print("XRoboToolkit SDK initialized.")

    def get_controller_poses(self):
        """Return left/right controller positions and orientations.

        Returns:
            left_pos: (3,) position [x, y, z]
            left_rot: (7,) [x, y, z, qx, qy, qz, qw]
            right_pos: (3,)
            right_rot: (7,)
        """
        left_raw = self.xrt.get_left_controller_pose()
        right_raw = self.xrt.get_right_controller_pose()
        return left_raw, right_raw

    def get_headset_pose(self):
        return self.xrt.get_headset_pose()

    def get_buttons(self):
        """Return controller button states."""
        return {
            "a_button": self.xrt.get_A_button(),
            "b_button": self.xrt.get_B_button(),
            "x_button": self.xrt.get_X_button(),
            "y_button": self.xrt.get_Y_button(),
            "left_trigger": self.xrt.get_left_trigger(),
            "right_trigger": self.xrt.get_right_trigger(),
            "left_grip": self.xrt.get_left_grip(),
            "right_grip": self.xrt.get_right_grip(),
        }


class MockVRInputSource:
    """Synthetic VR input for testing without hardware."""

    def __init__(self):
        self._t0 = time.time()
        self._a_pressed = False
        print("MockVRInputSource: generating synthetic controller motion.")

    def get_controller_poses(self):
        t = time.time() - self._t0
        # Simulate gentle hand movements
        lx = 0.1 * np.sin(0.5 * t)
        ly = 0.8 + 0.1 * np.sin(0.3 * t)
        lz = -0.3 + 0.05 * np.sin(0.4 * t)
        rx = -0.1 * np.sin(0.5 * t)
        ry = 0.8 + 0.1 * np.sin(0.3 * t + np.pi)
        rz = -0.3 + 0.05 * np.sin(0.4 * t + np.pi)
        left = [lx, ly, lz, 0, 0, 0, 1]
        right = [rx, ry, rz, 0, 0, 0, 1]
        return left, right

    def get_headset_pose(self):
        return [0, 1.6, 0, 0, 0, 0, 1]

    def get_buttons(self):
        t = time.time() - self._t0
        # Auto-press A after 2 seconds to start teleop
        a = t > 2.0 and not self._a_pressed
        if a:
            self._a_pressed = True
        return {
            "a_button": a,
            "b_button": False,
            "x_button": False,
            "y_button": False,
            "left_trigger": 0,
            "right_trigger": 0,
            "left_grip": 0,
            "right_grip": 0,
        }


# ─── Hand Controller ─────────────────────────────────────────────────────

class HandController:
    """Map trigger/grip inputs to hand motor positions.

    Same logic as TWIST2: trigger closes, grip opens.
    Publishes 7D per hand in TWIST2's ordering.
    """

    def __init__(self):
        self.left_pos = 0.0   # 0 = open, 1 = closed
        self.right_pos = 0.0
        self._step = 0.05

    def update(self, buttons):
        if buttons["left_trigger"] > 0.3:
            self.left_pos = min(1.0, self.left_pos + self._step)
        elif buttons["left_grip"] > 0.3:
            self.left_pos = max(0.0, self.left_pos - self._step)

        if buttons["right_trigger"] > 0.3:
            self.right_pos = min(1.0, self.right_pos + self._step)
        elif buttons["right_grip"] > 0.3:
            self.right_pos = max(0.0, self.right_pos - self._step)

    def get_hand_poses(self):
        """Return (left_7d, right_7d) in TWIST2 ordering."""
        left = HAND_LEFT_OPEN + (HAND_LEFT_CLOSE - HAND_LEFT_OPEN) * self.left_pos
        right = HAND_RIGHT_OPEN + (HAND_RIGHT_CLOSE - HAND_RIGHT_OPEN) * self.right_pos
        return left, right


# ─── State Machine ───────────────────────────────────────────────────────

class TeleopStateMachine:
    """Simple state machine: idle → teleop (A button toggles)."""

    def __init__(self):
        self.state = "idle"
        self._a_prev = False

    def update(self, buttons):
        a_current = bool(buttons["a_button"])
        if a_current and not self._a_prev:
            if self.state == "idle":
                self.state = "teleop"
                print(">> STATE: TELEOP (A pressed)")
            else:
                self.state = "idle"
                print(">> STATE: IDLE (A pressed)")
        self._a_prev = a_current


# ─── Main Controller Teleop ──────────────────────────────────────────────

class ControllerTeleop:
    """Main class: reads VR controllers, solves IK, publishes to Redis."""

    def __init__(self, vr_source, redis_host="localhost", redis_port=6379,
                 position_scale=POSITION_SCALE):
        self.vr = vr_source
        self.ik = ArmIKSolver()
        self.hands = HandController()
        self.sm = TeleopStateMachine()
        self.scale = position_scale

        # Redis
        import redis as _redis
        self.redis = _redis.Redis(host=redis_host, port=redis_port, db=0)
        self.redis.ping()
        self.pipe = self.redis.pipeline()
        print(f"Redis connected: {redis_host}:{redis_port}")

        # Calibration reference (set when entering teleop)
        self._ref_left_pos = None
        self._ref_right_pos = None
        self._ref_left_rot = None
        self._ref_right_rot = None

        self._stop = False

    def _parse_pose(self, raw):
        """Parse raw pose [x,y,z,qx,qy,qz,qw] into position + rotation."""
        pos_unity = np.array(raw[0:3])
        quat_xyzw = raw[3:7]
        quat_wxyz = [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]]
        pos_robot = unity_to_robot(pos_unity)
        rot_robot = unity_quat_to_robot(quat_wxyz)
        return pos_robot, rot_robot

    def _calibrate(self, left_raw, right_raw):
        """Record reference controller poses at teleop start."""
        self._ref_left_pos, self._ref_left_rot = self._parse_pose(left_raw)
        self._ref_right_pos, self._ref_right_rot = self._parse_pose(right_raw)
        self.ik.reset_to_default()
        print(f"  Calibrated. L ref pos: {self._ref_left_pos}")
        print(f"              R ref pos: {self._ref_right_pos}")

    def _compute_targets(self, left_raw, right_raw):
        """Compute robot wrist targets from current controller poses."""
        left_pos, left_rot = self._parse_pose(left_raw)
        right_pos, right_rot = self._parse_pose(right_raw)

        # Position delta from calibration reference
        delta_l = (left_pos - self._ref_left_pos) * self.scale
        delta_r = (right_pos - self._ref_right_pos) * self.scale

        target_l_pos = self.ik.default_left_wrist_pos + delta_l
        target_r_pos = self.ik.default_right_wrist_pos + delta_r

        # Keep default wrist orientations for now (position-only tracking).
        # Rotation tracking can be added later once position is verified.
        target_l_rot = self.ik.default_left_wrist_rot
        target_r_rot = self.ik.default_right_wrist_rot

        return target_l_pos, target_l_rot, target_r_pos, target_r_rot

    def _build_mimic_obs(self, left_arm, right_arm):
        """Build 35D mimic_obs array compatible with teleop_bridge.py."""
        obs = DEFAULT_MIMIC_OBS.copy()
        obs[OBS_LEFT_ARM] = left_arm
        obs[OBS_RIGHT_ARM] = right_arm
        return obs

    def _publish(self, mimic_obs, hand_left, hand_right):
        """Publish to Redis in TWIST2 format."""
        self.pipe.set(REDIS_KEY_BODY, json.dumps(mimic_obs.tolist()))
        self.pipe.set(REDIS_KEY_HAND_LEFT, json.dumps(hand_left.tolist()))
        self.pipe.set(REDIS_KEY_HAND_RIGHT, json.dumps(hand_right.tolist()))
        self.pipe.set(REDIS_KEY_TIMESTAMP, int(time.time() * 1000))
        self.pipe.execute()

    def run(self):
        signal.signal(signal.SIGINT, lambda s, f: setattr(self, '_stop', True))

        print("\n" + "=" * 60)
        print("  Controller Teleop — PICO VR Controllers → Redis")
        print("=" * 60)
        print(f"  Position scale: {self.scale}")
        print(f"  Target FPS: {TARGET_FPS}")
        print("")
        print("Controls:")
        print("  [A button]       Start/stop teleop")
        print("  [Index trigger]  Close hand")
        print("  [Grip button]    Open hand")
        print("  [Ctrl+C]         Quit")
        print("")

        was_idle = True
        frame_count = 0
        t_start = time.time()
        prev_left_raw = None
        stale_count = 0
        STALE_THRESHOLD = 30  # ~1 second at 30fps
        data_is_stale = False

        while not self._stop:
            t0 = time.time()

            buttons = self.vr.get_buttons()
            left_raw, right_raw = self.vr.get_controller_poses()
            self.sm.update(buttons)
            self.hands.update(buttons)

            # Detect stale data (PICO disconnected → SDK returns frozen values)
            if prev_left_raw is not None and np.allclose(left_raw, prev_left_raw, atol=1e-8):
                stale_count += 1
            else:
                if data_is_stale:
                    print(">> VR data LIVE again — will re-calibrate on next teleop")
                    was_idle = True  # force re-calibration
                stale_count = 0
                data_is_stale = False
            prev_left_raw = list(left_raw)

            if stale_count >= STALE_THRESHOLD and not data_is_stale:
                data_is_stale = True
                print(">> WARNING: VR data frozen (PICO disconnected?). Pausing arm commands.")

            hand_left, hand_right = self.hands.get_hand_poses()

            left_arm = right_arm = None
            if self.sm.state == "teleop" and not data_is_stale:
                if was_idle:
                    self._calibrate(left_raw, right_raw)
                    was_idle = False

                tl_pos, tl_rot, tr_pos, tr_rot = self._compute_targets(
                    left_raw, right_raw
                )
                left_arm, right_arm = self.ik.solve(
                    tl_pos, tl_rot, tr_pos, tr_rot
                )
                mimic_obs = self._build_mimic_obs(left_arm, right_arm)
            else:
                if not was_idle:
                    self.ik.reset_to_default()
                    self._ref_left_pos = None
                    self._ref_right_pos = None
                    print(">> Exited teleop — IK & references reset.")
                was_idle = True
                mimic_obs = DEFAULT_MIMIC_OBS.copy()

            self._publish(mimic_obs, hand_left, hand_right)

            frame_count += 1
            if frame_count % (TARGET_FPS * 5) == 0:
                elapsed = time.time() - t_start
                fps = frame_count / elapsed if elapsed > 0 else 0
                state_str = self.sm.state.upper()
                if data_is_stale:
                    state_str += " (STALE)"
                hand_str = f"L={self.hands.left_pos:.1f} R={self.hands.right_pos:.1f}"
                extra = ""
                if self.sm.state == "teleop" and self._ref_left_pos is not None:
                    lp, _ = self._parse_pose(left_raw)
                    dl = (lp - self._ref_left_pos) * self.scale
                    extra = f" | delta_L=[{dl[0]:.3f},{dl[1]:.3f},{dl[2]:.3f}]"
                if left_arm is not None and right_arm is not None:
                    default_l = DEFAULT_MIMIC_OBS[OBS_LEFT_ARM]
                    default_r = DEFAULT_MIMIC_OBS[OBS_RIGHT_ARM]
                    diff_l = np.max(np.abs(left_arm - default_l))
                    diff_r = np.max(np.abs(right_arm - default_r))
                    extra += f" | IK_L={diff_l:.4f} R={diff_r:.4f}"
                print(f"  [{elapsed:.0f}s] {state_str} | fps={fps:.1f} | hands: {hand_str}{extra}")

            dt = time.time() - t0
            sleep_time = (1.0 / TARGET_FPS) - dt
            if sleep_time > 0:
                time.sleep(sleep_time)

        print("\nShutting down... publishing default pose.")
        self._publish(DEFAULT_MIMIC_OBS.copy(), HAND_LEFT_OPEN, HAND_RIGHT_OPEN)
        print("Done.")


# ─── Entry point ─────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Controller-based VR teleoperation → Redis"
    )
    parser.add_argument("--redis-ip", default="localhost")
    parser.add_argument("--redis-port", type=int, default=6379)
    parser.add_argument("--mock", action="store_true",
                        help="Use synthetic VR input for testing")
    parser.add_argument("--scale", type=float, default=POSITION_SCALE,
                        help=f"Position scaling factor (default: {POSITION_SCALE})")
    args = parser.parse_args()

    if args.mock:
        vr = MockVRInputSource()
    else:
        vr = VRInputSource()

    teleop = ControllerTeleop(
        vr_source=vr,
        redis_host=args.redis_ip,
        redis_port=args.redis_port,
        position_scale=args.scale,
    )
    teleop.run()


if __name__ == "__main__":
    main()
