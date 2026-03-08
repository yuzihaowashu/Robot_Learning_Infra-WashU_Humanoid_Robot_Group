#!/usr/bin/env python3
"""
Unitree G1 VLA Client — bridges GR00T PolicyServer ↔ real robot.

Architecture:
    [Camera ZMQ] ─┐
    [rt/lowstate]  ├─→  Package obs  ─→  PolicyServer (GPU)  ─→  Decode actions
    [rt/dex3/*/state]┘                                              │
                                                                    ▼
                                                        [rt/arm_sdk]  arms+waist
                                                        [rt/dex3/*/cmd] hands

Usage:
    # Terminal 1 — start GR00T inference server
    bash run_vla.sh server

    # Terminal 2 — start robot client
    bash run_vla.sh client --task "pick up the apple and place on plate"
"""

import argparse
import base64
import json
import sys
import threading
import time

import cv2
import numpy as np

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
    LowCmd_,
    LowState_,
    HandCmd_,
    HandState_,
)
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
    MotionSwitcherClient,
)
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

try:
    import zmq
    HAS_ZMQ = True
except ImportError:
    HAS_ZMQ = False

try:
    from gr00t.policy.server_client import PolicyClient
    HAS_GROOT = True
except ImportError:
    HAS_GROOT = False

try:
    import pinocchio as pin
    HAS_PINOCCHIO = True
except ImportError:
    HAS_PINOCCHIO = False

# ─── Robot joint mapping ──────────────────────────────────────────────────
LEFT_LEG_JOINTS = list(range(0, 6))
RIGHT_LEG_JOINTS = list(range(6, 12))
WAIST_JOINTS = list(range(12, 15))
LEFT_ARM_JOINTS = list(range(15, 22))
RIGHT_ARM_JOINTS = list(range(22, 29))

# arm_sdk uses joints 12-28 with enable flag at index 29
ARM_SDK_JOINTS = list(range(12, 29))
ARM_SDK_ENABLE_IDX = 29

N_HAND_MOTORS = 7

TOPIC_ARM_SDK = "rt/arm_sdk"
TOPIC_LOW_STATE = "rt/lowstate"
TOPIC_LEFT_HAND_CMD = "rt/dex3/left/cmd"
TOPIC_RIGHT_HAND_CMD = "rt/dex3/right/cmd"
TOPIC_LEFT_HAND_STATE = "rt/dex3/left/state"
TOPIC_RIGHT_HAND_STATE = "rt/dex3/right/state"

ROBOT_IP = "192.168.123.164"
CAMERA_ZMQ_PORT = 5555

CONTROL_DT = 1.0 / 30.0  # 30 Hz action execution
KP_ARM = 80.0
KD_ARM = 2.0
KP_HAND = 1.5
KD_HAND = 0.2

MAX_DELTA_PER_STEP = 0.15  # rad — max joint change per 30Hz step (~8.6 deg)

URDF_PATH = (
    "/home/humanoid-pc/unitree_rl_gym/resources/robots/"
    "g1_description/g1_29dof_with_hand_rev_1_0.urdf"
)

# Joint limits from URDF (radians) — hard safety clamp
JOINT_LIMITS = {
    # waist: indices 12-14
    12: (-2.618, 2.618),   # waist_yaw
    13: (-0.52,  0.52),    # waist_roll
    14: (-0.52,  0.52),    # waist_pitch
    # left arm: indices 15-21
    15: (-3.0892, 2.6704), # left_shoulder_pitch
    16: (-1.5882, 2.2515), # left_shoulder_roll
    17: (-2.618,  2.618),  # left_shoulder_yaw
    18: (-1.0472, 2.0944), # left_elbow
    19: (-1.9722, 1.9722), # left_wrist_roll
    20: (-1.6144, 1.6144), # left_wrist_pitch
    21: (-1.6144, 1.6144), # left_wrist_yaw
    # right arm: indices 22-28
    22: (-3.0892, 2.6704), # right_shoulder_pitch
    23: (-2.2515, 1.5882), # right_shoulder_roll
    24: (-2.618,  2.618),  # right_shoulder_yaw
    25: (-1.0472, 2.0944), # right_elbow
    26: (-1.9722, 1.9722), # right_wrist_roll
    27: (-1.6144, 1.6144), # right_wrist_pitch
    28: (-1.6144, 1.6144), # right_wrist_yaw
}

UNITREE_TO_PIN = {
    0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5,
    6: 6, 7: 7, 8: 8, 9: 9, 10: 10, 11: 11,
    12: 12, 13: 13, 14: 14,
    15: 15, 16: 16, 17: 17, 18: 18, 19: 19, 20: 20, 21: 21,
    22: 22, 23: 23, 24: 24, 25: 25, 26: 26, 27: 27, 28: 28,
}


def _hand_motor_mode(motor_id, status=0x01, timeout=0):
    return motor_id | (status << 4) | (timeout << 7)


# ─── Gravity Compensation ─────────────────────────────────────────────────
class GravityCompensator:
    """Compute per-joint gravity torques using Pinocchio."""

    def __init__(self):
        self.available = False
        if not HAS_PINOCCHIO:
            print("WARNING: pinocchio not installed, gravity compensation disabled")
            return
        try:
            self.model = pin.buildModelFromUrdf(URDF_PATH)
            self.data = self.model.createData()
            self.available = True
            print(f"Gravity compensator loaded ({self.model.nq} DOF)")
        except Exception as e:
            print(f"WARNING: Could not load URDF: {e}")

    def compute(self, low_state):
        """Return {unitree_joint_idx: tau_ff} for arm+waist joints."""
        if not self.available:
            return {}
        q = np.zeros(self.model.nq)
        for unitree_j, pin_j in UNITREE_TO_PIN.items():
            if pin_j < self.model.nq:
                q[pin_j] = low_state.motor_state[unitree_j].q
        G = pin.computeGeneralizedGravity(self.model, self.data, q)
        tau_ff = {}
        for j in LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS:
            p_idx = UNITREE_TO_PIN[j]
            tau_ff[j] = float(G[p_idx])
        for j in WAIST_JOINTS:
            p_idx = UNITREE_TO_PIN[j]
            tau_ff[j] = float(G[p_idx])
        return tau_ff


# ─── Mode management ──────────────────────────────────────────────────────
def ensure_ai_mode():
    """Ensure the locomotion balance controller (ai sport) is active.

    The balance controller keeps legs/body balanced while we overlay
    arm commands via rt/arm_sdk. Must be started via hand controller:
      1. L1+A  — power on
      2. L1+UP — stand up
    """
    msc = MotionSwitcherClient()
    msc.SetTimeout(5.0)
    msc.Init()

    code, result = msc.CheckMode()
    current = result.get("name", "") if result else ""
    print(f"Current mode: '{current}' (raw={result})")

    if current == "ai":
        print("OK — ai sport (balance controller) is active.")
        return True

    print("ai mode not detected. Attempting SelectMode('ai')...")
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
        print(f"  LocoClient.SetFsmId(200) returned: {ret}")
        time.sleep(3)
        return True

    print("\n" + "!" * 50)
    print("  WARNING: Balance controller NOT active!")
    print("!" * 50)
    print("\nPlease start the robot with the hand controller:")
    print("  1. L1 + A    — power on")
    print("  2. L1 + UP   — stand up")
    print("  3. Robot should be standing and balanced")
    print("  4. Then re-run this script")
    print()
    ans = input("Continue anyway? (y/N): ").strip().lower()
    return ans == "y"


# ─── Camera receiver ─────────────────────────────────────────────────────
class CameraReceiver:
    def __init__(self, robot_ip=ROBOT_IP, port=CAMERA_ZMQ_PORT):
        self.endpoint = f"tcp://{robot_ip}:{port}"
        self.latest_frame = None
        self._lock = threading.Lock()
        self._stop = False

    def start(self):
        if not HAS_ZMQ:
            print("WARNING: pyzmq not installed, camera disabled")
            return
        t = threading.Thread(target=self._loop, daemon=True)
        t.start()

    def get_frame(self):
        with self._lock:
            return self.latest_frame

    def _loop(self):
        ctx = zmq.Context()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.SUBSCRIBE, b"")
        sock.setsockopt(zmq.RCVHWM, 2)
        sock.setsockopt(zmq.RCVTIMEO, 3000)
        sock.setsockopt(zmq.LINGER, 0)
        sock.connect(self.endpoint)

        while not self._stop:
            try:
                raw = sock.recv_string()
                data = json.loads(raw)
                b64 = data["images"].get("head_camera", "")
                if not b64:
                    continue
                buf = base64.b64decode(b64)
                arr = np.frombuffer(buf, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is not None:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    with self._lock:
                        self.latest_frame = frame
            except zmq.Again:
                pass
            except Exception:
                time.sleep(0.1)

        sock.close()
        ctx.term()


# ─── G1 Robot Interface ──────────────────────────────────────────────────
class G1Robot:
    """Handles all DDS communication with the G1 robot."""

    def __init__(self):
        self.crc = CRC()
        self.low_state = None
        self.left_hand_state = None
        self.right_hand_state = None
        self._state_lock = threading.Lock()
        self.state_received = False

    def init(self):
        self.arm_pub = ChannelPublisher(TOPIC_ARM_SDK, LowCmd_)
        self.arm_pub.Init()
        self.left_hand_pub = ChannelPublisher(TOPIC_LEFT_HAND_CMD, HandCmd_)
        self.left_hand_pub.Init()
        self.right_hand_pub = ChannelPublisher(TOPIC_RIGHT_HAND_CMD, HandCmd_)
        self.right_hand_pub.Init()

        self.state_sub = ChannelSubscriber(TOPIC_LOW_STATE, LowState_)
        self.state_sub.Init(self._on_state, 10)
        self.left_hand_sub = ChannelSubscriber(TOPIC_LEFT_HAND_STATE, HandState_)
        self.left_hand_sub.Init(self._on_left_hand, 10)
        self.right_hand_sub = ChannelSubscriber(TOPIC_RIGHT_HAND_STATE, HandState_)
        self.right_hand_sub.Init(self._on_right_hand, 10)

    def _on_state(self, msg):
        with self._state_lock:
            self.low_state = msg
            self.state_received = True

    def _on_left_hand(self, msg):
        with self._state_lock:
            self.left_hand_state = msg

    def _on_right_hand(self, msg):
        with self._state_lock:
            self.right_hand_state = msg

    def wait_for_state(self, timeout=5.0):
        t0 = time.time()
        while not self.state_received and time.time() - t0 < timeout:
            time.sleep(0.1)
        return self.state_received

    def get_state(self):
        """Return current joint states as GR00T-compatible dict."""
        with self._state_lock:
            if self.low_state is None:
                return None

            ms = self.low_state.motor_state

            left_leg = np.array([ms[j].q for j in LEFT_LEG_JOINTS], dtype=np.float32)
            right_leg = np.array([ms[j].q for j in RIGHT_LEG_JOINTS], dtype=np.float32)
            waist = np.array([ms[j].q for j in WAIST_JOINTS], dtype=np.float32)
            left_arm = np.array([ms[j].q for j in LEFT_ARM_JOINTS], dtype=np.float32)
            right_arm = np.array([ms[j].q for j in RIGHT_ARM_JOINTS], dtype=np.float32)

            left_hand = np.zeros(N_HAND_MOTORS, dtype=np.float32)
            if self.left_hand_state is not None:
                for i in range(min(N_HAND_MOTORS, len(self.left_hand_state.motor_state))):
                    left_hand[i] = self.left_hand_state.motor_state[i].q

            right_hand = np.zeros(N_HAND_MOTORS, dtype=np.float32)
            if self.right_hand_state is not None:
                for i in range(min(N_HAND_MOTORS, len(self.right_hand_state.motor_state))):
                    right_hand[i] = self.right_hand_state.motor_state[i].q

            mode_machine = self.low_state.mode_machine

        return {
            "left_leg": left_leg,
            "right_leg": right_leg,
            "waist": waist,
            "left_arm": left_arm,
            "right_arm": right_arm,
            "left_hand": left_hand,
            "right_hand": right_hand,
            "_mode_machine": mode_machine,
        }

    def send_arm_cmd(self, waist_pos, left_arm_pos, right_arm_pos, mode_machine,
                     kp=KP_ARM, kd=KD_ARM, grav_comp=None):
        """Send arm+waist command via rt/arm_sdk with gravity compensation."""
        cmd = unitree_hg_msg_dds__LowCmd_()
        cmd.mode_pr = 0
        cmd.mode_machine = mode_machine

        tau_ff = {}
        if grav_comp and self.low_state:
            tau_ff = grav_comp.compute(self.low_state)

        cmd.motor_cmd[ARM_SDK_ENABLE_IDX].q = 1.0

        all_pos = np.concatenate([waist_pos, left_arm_pos, right_arm_pos])
        for i, j in enumerate(ARM_SDK_JOINTS):
            cmd.motor_cmd[j].mode = 1
            cmd.motor_cmd[j].q = float(all_pos[i])
            cmd.motor_cmd[j].dq = 0.0
            cmd.motor_cmd[j].tau = tau_ff.get(j, 0.0)
            cmd.motor_cmd[j].kp = kp
            cmd.motor_cmd[j].kd = kd

        cmd.crc = self.crc.Crc(cmd)
        self.arm_pub.Write(cmd)

    def send_hand_cmd(self, left_hand_pos, right_hand_pos):
        """Send hand commands via rt/dex3/*/cmd."""
        for pub, positions in [(self.left_hand_pub, left_hand_pos),
                               (self.right_hand_pub, right_hand_pos)]:
            hcmd = unitree_hg_msg_dds__HandCmd_()
            for i in range(N_HAND_MOTORS):
                hcmd.motor_cmd[i].mode = _hand_motor_mode(i)
                hcmd.motor_cmd[i].q = float(positions[i])
                hcmd.motor_cmd[i].dq = 0.0
                hcmd.motor_cmd[i].tau = 0.0
                hcmd.motor_cmd[i].kp = KP_HAND
                hcmd.motor_cmd[i].kd = KD_HAND
            pub.Write(hcmd)


# ─── G1 ↔ GR00T Adapter ─────────────────────────────────────────────────
class G1Adapter:
    """Converts between robot observations and GR00T VLA format."""

    def __init__(self, policy_client, action_horizon=8):
        self.policy = policy_client
        self.action_horizon = action_horizon

    def obs_to_policy_input(self, state_dict, frame, task_description):
        """Package robot obs into GR00T format: (B=1, T=1, ...)."""
        obs = {}

        if frame is not None:
            h, w = frame.shape[:2]
            obs["video"] = {
                "ego_view": frame[np.newaxis, np.newaxis, ...],  # (1,1,H,W,3)
            }
        else:
            obs["video"] = {
                "ego_view": np.zeros((1, 1, 480, 640, 3), dtype=np.uint8),
            }

        obs["state"] = {}
        for key in ["left_leg", "right_leg", "waist",
                     "left_arm", "right_arm", "left_hand", "right_hand"]:
            val = state_dict[key]
            obs["state"][key] = val[np.newaxis, np.newaxis, :]  # (1,1,D)

        obs["language"] = {
            "annotation.human.task_description": [[task_description]],
        }

        return obs

    def decode_actions(self, action_chunk, state_dict):
        """Decode GR00T action chunk into per-step robot commands.

        GR00T's decode_action pipeline already converts relative actions to
        absolute positions internally (reference_state + delta). ALL outputs
        are absolute joint positions — do NOT accumulate deltas.

        Returns list of (waist, left_arm, right_arm, left_hand, right_hand) tuples.
        """
        steps = []
        T = min(
            action_chunk["left_arm"].shape[1],
            self.action_horizon,
        )

        prev_left_arm = state_dict["left_arm"].copy()
        prev_right_arm = state_dict["right_arm"].copy()
        prev_waist = state_dict["waist"].copy()

        for t in range(T):
            left_arm = action_chunk["left_arm"][0, t].copy()    # already absolute
            right_arm = action_chunk["right_arm"][0, t].copy()  # already absolute
            waist = action_chunk["waist"][0, t].copy()          # absolute
            left_hand = action_chunk["left_hand"][0, t].copy()  # absolute
            right_hand = action_chunk["right_hand"][0, t].copy()  # absolute

            left_arm = self._clamp_joints(left_arm, LEFT_ARM_JOINTS, prev_left_arm)
            right_arm = self._clamp_joints(right_arm, RIGHT_ARM_JOINTS, prev_right_arm)
            waist = self._clamp_joints(waist, WAIST_JOINTS, prev_waist)

            steps.append((waist, left_arm, right_arm, left_hand, right_hand))

            prev_left_arm = left_arm.copy()
            prev_right_arm = right_arm.copy()
            prev_waist = waist.copy()

        return steps

    @staticmethod
    def _clamp_joints(target, joint_indices, prev_values):
        """Apply URDF joint limits and max-delta-per-step safety clamp."""
        clamped = target.copy()
        for i, j_idx in enumerate(joint_indices):
            lo, hi = JOINT_LIMITS.get(j_idx, (-3.2, 3.2))
            clamped[i] = np.clip(clamped[i], lo, hi)

            delta = clamped[i] - prev_values[i]
            if abs(delta) > MAX_DELTA_PER_STEP:
                clamped[i] = prev_values[i] + np.sign(delta) * MAX_DELTA_PER_STEP
        return clamped

    def get_action(self, state_dict, frame, task_description):
        """Full pipeline: obs → policy → decoded actions."""
        obs = self.obs_to_policy_input(state_dict, frame, task_description)
        action_chunk, info = self.policy.get_action(obs)
        return self.decode_actions(action_chunk, state_dict)


def _fmt_rad(arr):
    return "[" + ", ".join(f"{v:+.4f}" for v in arr) + "]"


def _fmt_deg(arr):
    return "[" + ", ".join(f"{np.degrees(v):+6.1f}°" for v in arr) + "]"


def _print_action_summary(step, state, actions):
    """Print detailed action breakdown for human review."""
    w, la, ra, lh, rh = actions[0]
    d_la = la - state["left_arm"]
    d_ra = ra - state["right_arm"]
    d_w = w - state["waist"]

    max_d_la = np.max(np.abs(d_la))
    max_d_ra = np.max(np.abs(d_ra))

    print(f"  ┌─ Left  Arm ──────────────────────────────")
    print(f"  │ Now:    {_fmt_deg(state['left_arm'])}")
    print(f"  │ Target: {_fmt_deg(la)}")
    print(f"  │ Delta:  {_fmt_deg(d_la)}  (max {np.degrees(max_d_la):.1f}°)")
    print(f"  ├─ Right Arm ──────────────────────────────")
    print(f"  │ Now:    {_fmt_deg(state['right_arm'])}")
    print(f"  │ Target: {_fmt_deg(ra)}")
    print(f"  │ Delta:  {_fmt_deg(d_ra)}  (max {np.degrees(max_d_ra):.1f}°)")
    print(f"  ├─ Waist ──────────────────────────────────")
    print(f"  │ Now:    {_fmt_deg(state['waist'])}")
    print(f"  │ Target: {_fmt_deg(w)}")
    print(f"  │ Delta:  {_fmt_deg(d_w)}")
    print(f"  ├─ Hands ──────────────────────────────────")
    print(f"  │ L_hand: {_fmt_rad(lh)}")
    print(f"  │ R_hand: {_fmt_rad(rh)}")
    print(f"  └──────────────────────────────────────────")

    danger = max_d_la > 0.12 or max_d_ra > 0.12
    if danger:
        print(f"  ⚠  LARGE DELTA detected! Max: "
              f"L={np.degrees(max_d_la):.1f}° R={np.degrees(max_d_ra):.1f}°")
    return danger


def _hold_position(robot, grav_comp, duration=0.5):
    """Keep sending current position to maintain arm control."""
    t0 = time.time()
    while time.time() - t0 < duration:
        state = robot.get_state()
        if state:
            robot.send_arm_cmd(
                state["waist"], state["left_arm"], state["right_arm"],
                state["_mode_machine"], grav_comp=grav_comp,
            )
        time.sleep(CONTROL_DT)


# ─── Main control loop ───────────────────────────────────────────────────
def run_vla(args):
    print("=" * 60)
    print("  G1 VLA Client — GR00T N1.6 Inference")
    print("=" * 60)
    print(f"  Policy server: {args.policy_host}:{args.policy_port}")
    print(f"  Task: {args.task}")
    print(f"  Action horizon: {args.action_horizon}")
    print(f"  Mode: {'CONTINUOUS (auto)' if args.continuous else 'STEP-BY-STEP (press Enter)'}")
    print(f"  Dry run: {args.dry_run}")
    print()

    # ── Initialize DDS ──
    ChannelFactoryInitialize(0)

    # ── Ensure balance controller is active ──
    if not args.dry_run:
        if not ensure_ai_mode():
            print("Aborting — balance controller not active.")
            sys.exit(1)

    # ── Gravity compensation ──
    grav_comp = GravityCompensator()
    grav_str = ("ON — prevents arm sag" if grav_comp.available
                else "OFF — arms may drift down")
    print(f"Gravity compensation: {grav_str}")

    robot = G1Robot()
    robot.init()
    print("Waiting for robot state...")
    if not robot.wait_for_state():
        print("ERROR: No robot state received!")
        sys.exit(1)
    print("Robot connected!")

    # ── Camera ──
    camera = CameraReceiver()
    camera.start()
    print(f"Camera receiver started ({camera.endpoint})")
    time.sleep(1.0)

    # ── Policy client ──
    if not HAS_GROOT:
        print("ERROR: gr00t package not available!")
        sys.exit(1)

    print(f"Connecting to policy server at {args.policy_host}:{args.policy_port}...")
    policy = PolicyClient(
        host=args.policy_host,
        port=args.policy_port,
        timeout_ms=30000,
        strict=False,
    )

    if not policy.ping():
        print("ERROR: Cannot connect to policy server!")
        print("Start the server first: bash run_vla.sh server")
        sys.exit(1)
    print("Policy server connected!")

    modality_config = policy.get_modality_config()
    print(f"Modality config keys: {list(modality_config.keys())}")

    adapter = G1Adapter(policy, action_horizon=args.action_horizon)

    # ── Store home pose ──
    home_state = robot.get_state()
    home_waist = home_state["waist"].copy()
    home_left_arm = home_state["left_arm"].copy()
    home_right_arm = home_state["right_arm"].copy()
    home_left_hand = np.zeros(N_HAND_MOTORS, dtype=np.float32)
    home_right_hand = np.zeros(N_HAND_MOTORS, dtype=np.float32)
    print(f"Home pose saved (L_arm: {_fmt_deg(home_left_arm)})")
    print(f"                (R_arm: {_fmt_deg(home_right_arm)})")

    # ── Ramp up: smoothly engage arm control ──
    print("\nRamping up arm control (2s)...")
    ramp_duration = 2.0
    t0 = time.time()
    while time.time() - t0 < ramp_duration:
        state = robot.get_state()
        s = min((time.time() - t0) / ramp_duration, 1.0)
        s = s * s * (3 - 2 * s)  # smoothstep
        robot.send_arm_cmd(
            state["waist"], state["left_arm"], state["right_arm"],
            state["_mode_machine"], kp=KP_ARM * s, kd=KD_ARM * s,
            grav_comp=grav_comp,
        )
        time.sleep(CONTROL_DT)
    print("Arm control engaged!")

    # ── Main loop ──
    if args.continuous:
        print(f"\nRunning VLA in CONTINUOUS mode (Ctrl+C to stop)...")
    else:
        print(f"\n{'=' * 60}")
        print(f"  STEP-BY-STEP MODE (safe)")
        print(f"  Press ENTER  = approve & execute this step")
        print(f"  Type  's'    = skip this step (hold position)")
        print(f"  Type  'c'    = switch to continuous mode")
        print(f"  Type  'q'    = quit gracefully")
        print(f"  Ctrl+C       = emergency stop")
        print(f"{'=' * 60}")

    print(f'Task: "{args.task}"\n')
    step_count = 0
    continuous_mode = args.continuous

    try:
        while True:
            # Hold position while waiting (keeps arm control active)
            _hold_position(robot, grav_comp, duration=0.3)

            state = robot.get_state()
            frame = camera.get_frame()

            if frame is None:
                print("  [warn] No camera frame, using blank")

            t_start = time.time()
            actions = adapter.get_action(state, frame, args.task)
            t_infer = time.time() - t_start

            print(f"\n{'─' * 60}")
            print(f"[Step {step_count}] Inference: {t_infer:.3f}s, "
                  f"{len(actions)} action sub-steps")

            danger = _print_action_summary(step_count, state, actions)

            # ── Step-by-step approval ──
            if not continuous_mode and not args.dry_run:
                prompt = "  >> Execute? [Enter/s/c/q]: "
                if danger:
                    prompt = "  >> ⚠ LARGE MOVE! Execute? [Enter/s/c/q]: "
                try:
                    user_input = input(prompt).strip().lower()
                except EOFError:
                    break

                if user_input == "q":
                    print("  User requested quit.")
                    break
                elif user_input == "s":
                    print("  Skipped — holding position.")
                    _hold_position(robot, grav_comp, duration=0.5)
                    step_count += 1
                    continue
                elif user_input == "c":
                    print("  Switching to CONTINUOUS mode!")
                    continuous_mode = True
                elif user_input == "":
                    pass  # approved
                else:
                    print(f"  Unknown input '{user_input}', skipping.")
                    _hold_position(robot, grav_comp, duration=0.5)
                    step_count += 1
                    continue

            # ── Execute action steps ──
            for i, (waist, left_arm, right_arm, left_hand, right_hand) in enumerate(actions):
                t_act = time.time()

                if not args.dry_run:
                    current_state = robot.get_state()
                    robot.send_arm_cmd(
                        waist, left_arm, right_arm,
                        current_state["_mode_machine"],
                        grav_comp=grav_comp,
                    )
                    robot.send_hand_cmd(left_hand, right_hand)

                dt = time.time() - t_act
                if dt < CONTROL_DT:
                    time.sleep(CONTROL_DT - dt)

            if not args.dry_run:
                print(f"  ✓ Executed {len(actions)} sub-steps")

            step_count += 1

    except KeyboardInterrupt:
        print("\n\nEmergency stop!")

    # ── Return to home pose smoothly ──
    state = robot.get_state()
    start_waist = state["waist"].copy()
    start_left = state["left_arm"].copy()
    start_right = state["right_arm"].copy()

    dist_left = np.max(np.abs(start_left - home_left_arm))
    dist_right = np.max(np.abs(start_right - home_right_arm))
    max_dist = max(dist_left, dist_right, 0.01)
    return_duration = np.clip(max_dist / 0.3, 2.0, 6.0)

    print(f"Returning to home pose ({return_duration:.1f}s, "
          f"max delta: {np.degrees(max_dist):.1f}°)...")
    t0 = time.time()
    while time.time() - t0 < return_duration:
        alpha = min((time.time() - t0) / return_duration, 1.0)
        alpha = alpha * alpha * (3 - 2 * alpha)  # smoothstep

        w = start_waist + alpha * (home_waist - start_waist)
        la = start_left + alpha * (home_left_arm - start_left)
        ra = start_right + alpha * (home_right_arm - start_right)

        current = robot.get_state()
        robot.send_arm_cmd(w, la, ra, current["_mode_machine"],
                           grav_comp=grav_comp)
        time.sleep(CONTROL_DT)
    print("Home pose reached!")

    # ── Ramp down: smoothly release arm control ──
    print("Releasing arm control (2s)...")
    t0 = time.time()
    while time.time() - t0 < ramp_duration:
        s = min((time.time() - t0) / ramp_duration, 1.0)
        s = s * s * (3 - 2 * s)
        current = robot.get_state()
        robot.send_arm_cmd(
            home_waist, home_left_arm, home_right_arm,
            current["_mode_machine"],
            kp=KP_ARM * (1 - s), kd=KD_ARM * (1 - s),
            grav_comp=grav_comp,
        )
        time.sleep(CONTROL_DT)

    # Release hands to open position
    print("Releasing hands...")
    for _ in range(int(1.0 / CONTROL_DT)):
        robot.send_hand_cmd(home_left_hand, home_right_hand)
        time.sleep(CONTROL_DT)

    print("Done.")


def main():
    parser = argparse.ArgumentParser(description="G1 VLA Client")
    parser.add_argument("--policy-host", default="localhost",
                        help="GR00T policy server host")
    parser.add_argument("--policy-port", type=int, default=5556,
                        help="GR00T policy server port (default 5556, "
                             "camera uses 5555)")
    parser.add_argument("--task", default="pick up the apple and place on plate",
                        help="Language task instruction")
    parser.add_argument("--action-horizon", type=int, default=8,
                        help="Number of action steps to execute per inference")
    parser.add_argument("--continuous", action="store_true",
                        help="Skip step-by-step approval, run continuously")
    parser.add_argument("--dry-run", action="store_true",
                        help="Run inference but don't send commands to robot")
    args = parser.parse_args()
    run_vla(args)


if __name__ == "__main__":
    main()
