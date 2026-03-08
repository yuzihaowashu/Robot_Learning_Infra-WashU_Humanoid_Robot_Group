#!/usr/bin/env python3
"""
Collect a LeRobot-format dataset by replaying taught trajectories.

Replays trajectories from run_teach.sh while simultaneously recording:
  - Joint states (observation.state): 29 body DOF
  - Joint actions (action): 14 arm + 3 waist = 17 DOF
  - Hand states (observation.hand): 14 hand DOF (7 per hand)
  - Hand actions (action.hand): 14 hand DOF
  - Camera images (observation.images.ego_view): from robot head camera

The resulting dataset is stored in LeRobot v2 format (Parquet + MP4) and
can be visualized locally, then optionally pushed to HuggingFace Hub.

Usage:
    python collect_dataset.py \\
        --trajectories trajectories/traj_*.json \\
        --repo-id yuzihaowashu/g1_pick_apple \\
        --task "pick up the apple and place on plate"

    python collect_dataset.py \\
        --trajectories trajectories/traj_001.json trajectories/traj_002.json \\
        --repo-id yuzihaowashu/g1_demo \\
        --task "wave hello" \\
        --push
"""

import argparse
import base64
import glob
import json
import os
import signal
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import (
    unitree_hg_msg_dds__HandCmd_,
    unitree_hg_msg_dds__LowCmd_,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (
    HandCmd_,
    HandState_,
    LowCmd_,
    LowState_,
)
from unitree_sdk2py.utils.crc import CRC

try:
    import pinocchio as pin
    HAS_PINOCCHIO = True
except ImportError:
    HAS_PINOCCHIO = False

try:
    import zmq
    HAS_ZMQ = True
except ImportError:
    HAS_ZMQ = False

# ─── Constants ────────────────────────────────────────────────────────────
LEFT_LEG_JOINTS = list(range(0, 6))
RIGHT_LEG_JOINTS = list(range(6, 12))
WAIST_JOINTS = [12, 13, 14]
LEFT_ARM_JOINTS = list(range(15, 22))
RIGHT_ARM_JOINTS = list(range(22, 29))
ARM_SDK_JOINTS = WAIST_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS
ARM_SDK_ENABLE_IDX = 29

N_HAND_MOTORS = 7
N_BODY_JOINTS = 29

TOPIC_ARM_SDK = "rt/arm_sdk"
TOPIC_LOW_STATE = "rt/lowstate"
TOPIC_LEFT_HAND_CMD = "rt/dex3/left/cmd"
TOPIC_RIGHT_HAND_CMD = "rt/dex3/right/cmd"
TOPIC_LEFT_HAND_STATE = "rt/dex3/left/state"
TOPIC_RIGHT_HAND_STATE = "rt/dex3/right/state"

ROBOT_IP = "192.168.123.164"
CAMERA_ZMQ_PORT = 5555

KP_ARM = 80.0
KD_ARM = 2.0
KP_HAND = 1.5
KD_HAND = 0.2
CONTROL_DT = 1.0 / 30.0

URDF_PATH = (
    "/home/humanoid-pc/unitree_rl_gym/resources/robots/"
    "g1_description/g1_29dof_with_hand_rev_1_0.urdf"
)

UNITREE_TO_PIN = {}
for _i in range(29):
    UNITREE_TO_PIN[_i] = _i

JOINT_NAMES_29 = [
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw",
    "left_knee", "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw",
    "right_knee", "right_ankle_pitch", "right_ankle_roll",
    "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
    "left_elbow", "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
    "right_elbow", "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
]

ARM_WAIST_INDICES = WAIST_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS
HAND_NAMES_14 = [
    "left_thumb_0", "left_thumb_1", "left_thumb_2",
    "left_index_0", "left_index_1", "left_middle_0", "left_middle_1",
    "right_thumb_0", "right_thumb_1", "right_thumb_2",
    "right_index_0", "right_index_1", "right_middle_0", "right_middle_1",
]


def _hand_motor_mode(motor_id, status=0x01, timeout=0):
    return (motor_id & 0x0F) | ((status & 0x07) << 4) | ((timeout & 0x01) << 7)


# ─── Gravity Compensator ─────────────────────────────────────────────────
class GravityCompensator:
    def __init__(self):
        self.available = False
        if not HAS_PINOCCHIO:
            return
        try:
            self.model = pin.buildModelFromUrdf(URDF_PATH)
            self.data = self.model.createData()
            self.available = True
        except Exception:
            pass

    def compute(self, low_state):
        if not self.available:
            return {}
        q = np.zeros(self.model.nq)
        for u_j, p_j in UNITREE_TO_PIN.items():
            if p_j < self.model.nq:
                q[p_j] = low_state.motor_state[u_j].q
        G = pin.computeGeneralizedGravity(self.model, self.data, q)
        tau_ff = {}
        for j in LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS + WAIST_JOINTS:
            p_idx = UNITREE_TO_PIN[j]
            if p_idx < len(G):
                tau_ff[j] = float(G[p_idx])
        return tau_ff


# ─── Camera Receiver ─────────────────────────────────────────────────────
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


# ─── Robot Interface ─────────────────────────────────────────────────────
class RobotInterface:
    def __init__(self):
        self.crc = CRC()
        self.low_state = None
        self.left_hand_state = None
        self.right_hand_state = None
        self._lock = threading.Lock()
        self.state_ready = False

    def init(self):
        self.arm_pub = ChannelPublisher(TOPIC_ARM_SDK, LowCmd_)
        self.arm_pub.Init()
        self.lh_pub = ChannelPublisher(TOPIC_LEFT_HAND_CMD, HandCmd_)
        self.lh_pub.Init()
        self.rh_pub = ChannelPublisher(TOPIC_RIGHT_HAND_CMD, HandCmd_)
        self.rh_pub.Init()

        self.state_sub = ChannelSubscriber(TOPIC_LOW_STATE, LowState_)
        self.state_sub.Init(self._on_low_state, 10)
        self.lh_sub = ChannelSubscriber(TOPIC_LEFT_HAND_STATE, HandState_)
        self.lh_sub.Init(self._on_lh_state, 10)
        self.rh_sub = ChannelSubscriber(TOPIC_RIGHT_HAND_STATE, HandState_)
        self.rh_sub.Init(self._on_rh_state, 10)

    def _on_low_state(self, msg):
        with self._lock:
            self.low_state = msg
            self.state_ready = True

    def _on_lh_state(self, msg):
        with self._lock:
            self.left_hand_state = msg

    def _on_rh_state(self, msg):
        with self._lock:
            self.right_hand_state = msg

    def wait_for_state(self, timeout=5.0):
        t0 = time.time()
        while not self.state_ready and time.time() - t0 < timeout:
            time.sleep(0.05)
        return self.state_ready

    def get_body_state(self):
        """Return 29-dim body joint positions."""
        with self._lock:
            if self.low_state is None:
                return np.zeros(N_BODY_JOINTS, dtype=np.float32)
            return np.array(
                [self.low_state.motor_state[j].q for j in range(N_BODY_JOINTS)],
                dtype=np.float32,
            )

    def get_hand_state(self):
        """Return 14-dim hand joint positions (7 left + 7 right)."""
        with self._lock:
            left = np.zeros(N_HAND_MOTORS, dtype=np.float32)
            right = np.zeros(N_HAND_MOTORS, dtype=np.float32)
            if self.left_hand_state is not None:
                for i in range(min(N_HAND_MOTORS, len(self.left_hand_state.motor_state))):
                    left[i] = self.left_hand_state.motor_state[i].q
            if self.right_hand_state is not None:
                for i in range(min(N_HAND_MOTORS, len(self.right_hand_state.motor_state))):
                    right[i] = self.right_hand_state.motor_state[i].q
            return np.concatenate([left, right]).astype(np.float32)

    def get_mode_machine(self):
        with self._lock:
            if self.low_state is None:
                return 0
            return self.low_state.mode_machine

    def send_arm_cmd(self, arm_positions, mode_machine, tau_ff=None):
        """Send arm+waist command. arm_positions: dict {joint_idx: rad}."""
        cmd = unitree_hg_msg_dds__LowCmd_()
        cmd.mode_pr = 0
        cmd.mode_machine = mode_machine
        cmd.motor_cmd[ARM_SDK_ENABLE_IDX].q = 1.0

        for j in ARM_SDK_JOINTS:
            cmd.motor_cmd[j].mode = 1
            cmd.motor_cmd[j].q = float(arm_positions.get(j, 0.0))
            cmd.motor_cmd[j].dq = 0.0
            cmd.motor_cmd[j].tau = float(tau_ff.get(j, 0.0)) if tau_ff else 0.0
            cmd.motor_cmd[j].kp = KP_ARM
            cmd.motor_cmd[j].kd = KD_ARM

        cmd.crc = self.crc.Crc(cmd)
        self.arm_pub.Write(cmd)

    def send_hand_cmd(self, left_pos, right_pos):
        for pub, positions in [(self.lh_pub, left_pos), (self.rh_pub, right_pos)]:
            hcmd = unitree_hg_msg_dds__HandCmd_()
            for i in range(N_HAND_MOTORS):
                hcmd.motor_cmd[i].mode = _hand_motor_mode(i)
                hcmd.motor_cmd[i].q = float(positions[i])
                hcmd.motor_cmd[i].kp = KP_HAND
                hcmd.motor_cmd[i].kd = KD_HAND
            pub.Write(hcmd)


# ─── Trajectory Loading ──────────────────────────────────────────────────
def load_trajectory(path):
    """Load teach trajectory JSON, return (metadata, frames_list)."""
    with open(path) as f:
        data = json.load(f)
    meta = data["metadata"]
    frames = data["frames"]

    parsed = []
    for fr in frames:
        arm_pos = {}
        for k, v in fr["arm"].items():
            arm_pos[int(k)] = float(v)

        hand_pos = np.zeros(N_HAND_MOTORS * 2, dtype=np.float32)
        if "hand" in fr and fr["hand"]:
            for k, v in fr["hand"].items():
                idx = int(k)
                if idx < N_HAND_MOTORS * 2:
                    hand_pos[idx] = float(v)

        parsed.append({
            "t": fr["t"],
            "arm": arm_pos,
            "left_hand": hand_pos[:N_HAND_MOTORS],
            "right_hand": hand_pos[N_HAND_MOTORS:],
        })
    return meta, parsed


# ─── Dataset Features ────────────────────────────────────────────────────
def make_features(image_shape=(480, 640, 3)):
    """Define LeRobot dataset features for G1."""
    return {
        "observation.state": {
            "dtype": "float32",
            "shape": (N_BODY_JOINTS,),
            "names": {"motors": JOINT_NAMES_29},
        },
        "observation.hand": {
            "dtype": "float32",
            "shape": (N_HAND_MOTORS * 2,),
            "names": {"motors": HAND_NAMES_14},
        },
        "observation.images.ego_view": {
            "dtype": "video",
            "shape": image_shape,
            "names": ["height", "width", "channels"],
        },
        "action": {
            "dtype": "float32",
            "shape": (len(ARM_WAIST_INDICES),),
            "names": {"motors": [JOINT_NAMES_29[j] for j in ARM_WAIST_INDICES]},
        },
        "action.hand": {
            "dtype": "float32",
            "shape": (N_HAND_MOTORS * 2,),
            "names": {"motors": HAND_NAMES_14},
        },
    }


# ─── Main Collection ─────────────────────────────────────────────────────
def collect(args):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    traj_paths = []
    for pattern in args.trajectories:
        traj_paths.extend(sorted(glob.glob(pattern)))
    if not traj_paths:
        print("ERROR: No trajectory files found!")
        sys.exit(1)

    print(f"Found {len(traj_paths)} trajectory file(s):")
    for p in traj_paths:
        print(f"  {p}")

    # ── Initialize DDS ──
    ChannelFactoryInitialize(0)

    robot = RobotInterface()
    robot.init()
    print("Waiting for robot state...")
    if not robot.wait_for_state():
        print("ERROR: No robot state received!")
        sys.exit(1)
    print("Robot connected!")

    grav_comp = GravityCompensator()
    print(f"Gravity compensation: {'ON' if grav_comp.available else 'OFF'}")

    camera = CameraReceiver()
    camera.start()
    print(f"Camera: {camera.endpoint}")

    print("Waiting for camera frames (up to 10s)...")
    test_frame = None
    for _retry in range(50):
        time.sleep(0.2)
        test_frame = camera.get_frame()
        if test_frame is not None:
            break

    if test_frame is not None:
        img_shape = test_frame.shape
        print(f"Camera OK — resolution: {img_shape}")
    else:
        img_shape = (480, 640, 3)
        print("=" * 60)
        print("  WARNING: Camera NOT connected! Images will be BLACK!")
        print(f"  Expected stream at {camera.endpoint}")
        print("  Start camera first: bash run_dashboard.sh  or  bash run_vla.sh client")
        print("=" * 60)
        ans = input("  Continue anyway? [y/N]: ").strip().lower()
        if ans != "y":
            print("Aborting.")
            sys.exit(1)

    # ── Create dataset ──
    features = make_features(img_shape)
    if args.output_dir:
        root = Path(args.output_dir) / args.repo_id
    else:
        root = None

    if root and root.exists():
        print(f"\nWARNING: Dataset directory already exists: {root}")
        ans = input("  Delete and recreate? [y/N]: ").strip().lower()
        if ans == "y":
            import shutil
            shutil.rmtree(root)
            print("  Deleted.")
        else:
            print("  Aborting. Move or delete the directory first.")
            sys.exit(1)

    print(f"\nCreating dataset: {args.repo_id}")
    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=30,
        features=features,
        root=root,
        robot_type="unitree_g1",
        use_videos=True,
    )
    print(f"Dataset root: {dataset.root}")

    # ── Replay each trajectory ──
    home_state = robot.get_body_state()
    stop_flag = False

    def _sigint(sig, frame):
        nonlocal stop_flag
        stop_flag = True
        print("\nStopping after current episode...")

    signal.signal(signal.SIGINT, _sigint)

    for ep_idx, traj_path in enumerate(traj_paths):
        if stop_flag:
            break

        meta, frames = load_trajectory(traj_path)
        print(f"\n{'=' * 60}")
        print(f"Episode {ep_idx + 1}/{len(traj_paths)}: {os.path.basename(traj_path)}")
        print(f"  Frames: {len(frames)}, Duration: {meta.get('duration_s', '?')}s")
        print(f"  Task: \"{args.task}\"")

        input(f"  Press ENTER to start replay+record (or Ctrl+C to stop)...")

        # Ramp up to start position
        start_arm = frames[0]["arm"]
        print("  Moving to start position (3s)...")
        t0 = time.time()
        while time.time() - t0 < 3.0:
            s = min((time.time() - t0) / 3.0, 1.0)
            s = s * s * (3 - 2 * s)
            current_body = robot.get_body_state()
            blended = {}
            for j in ARM_SDK_JOINTS:
                target = start_arm.get(j, current_body[j])
                blended[j] = current_body[j] + s * (target - current_body[j])

            tau_ff = grav_comp.compute(robot.low_state) if grav_comp.available else {}
            robot.send_arm_cmd(blended, robot.get_mode_machine(), tau_ff)
            time.sleep(CONTROL_DT)

        # Replay + record
        print(f"  Recording {len(frames)} frames...")
        for fi, fr in enumerate(frames):
            if stop_flag:
                break

            t_start = time.time()

            tau_ff = grav_comp.compute(robot.low_state) if grav_comp.available else {}
            robot.send_arm_cmd(fr["arm"], robot.get_mode_machine(), tau_ff)
            robot.send_hand_cmd(fr["left_hand"], fr["right_hand"])

            # Capture observation
            body_state = robot.get_body_state()
            hand_state = robot.get_hand_state()
            cam_frame = camera.get_frame()

            if cam_frame is None:
                cam_frame = np.zeros(img_shape, dtype=np.uint8)

            # Build action: commanded arm+waist positions
            action = np.array(
                [fr["arm"].get(j, body_state[j]) for j in ARM_WAIST_INDICES],
                dtype=np.float32,
            )
            hand_action = np.concatenate([fr["left_hand"], fr["right_hand"]]).astype(np.float32)

            frame_data = {
                "observation.state": body_state,
                "observation.hand": hand_state,
                "observation.images.ego_view": cam_frame,
                "action": action,
                "action.hand": hand_action,
                "task": args.task,
            }
            dataset.add_frame(frame_data)

            dt = time.time() - t_start
            if dt < CONTROL_DT:
                time.sleep(CONTROL_DT - dt)

            if (fi + 1) % 100 == 0:
                print(f"    Frame {fi + 1}/{len(frames)}")

        if not stop_flag:
            dataset.save_episode()
            print(f"  Episode {ep_idx + 1} saved! "
                  f"({len(frames)} frames)")

        # Return to home
        print("  Returning to home pose...")
        current_body = robot.get_body_state()
        t0 = time.time()
        while time.time() - t0 < 2.0:
            s = min((time.time() - t0) / 2.0, 1.0)
            s = s * s * (3 - 2 * s)
            blended = {}
            for j in ARM_SDK_JOINTS:
                blended[j] = current_body[j] + s * (home_state[j] - current_body[j])
            tau_ff = grav_comp.compute(robot.low_state) if grav_comp.available else {}
            robot.send_arm_cmd(blended, robot.get_mode_machine(), tau_ff)
            time.sleep(CONTROL_DT)

    # ── Finalize ──
    dataset.finalize()
    print(f"\n{'=' * 60}")
    print(f"Dataset complete!")
    print(f"  Location: {dataset.root}")
    print(f"  Episodes: {dataset.num_episodes}")
    print(f"  Total frames: {dataset.num_frames}")

    # ── Visualize ──
    print(f"\nTo visualize locally:")
    print(f"  python -m lerobot.scripts.lerobot_dataset_viz "
          f"--repo-id {args.repo_id} --root {dataset.root}")

    # ── Push ──
    if args.push:
        print(f"\nPushing to HuggingFace Hub...")
        dataset.push_to_hub()
        print(f"Done! https://huggingface.co/datasets/{args.repo_id}")
    else:
        print(f"\nTo push to HuggingFace Hub later:")
        print(f"  huggingface-cli login")
        print(f"  python -c \"from lerobot.datasets.lerobot_dataset import LeRobotDataset; "
              f"ds = LeRobotDataset('{args.repo_id}', root='{dataset.root}'); "
              f"ds.push_to_hub()\"")


def main():
    parser = argparse.ArgumentParser(
        description="Collect LeRobot dataset by replaying taught trajectories"
    )
    parser.add_argument(
        "--trajectories", nargs="+", required=True,
        help="Trajectory JSON files or glob patterns "
             "(e.g. trajectories/traj_*.json)",
    )
    parser.add_argument(
        "--repo-id", required=True,
        help="HuggingFace dataset ID (e.g. yuzihaowashu/g1_pick_apple)",
    )
    parser.add_argument(
        "--task", required=True,
        help="Task description for all episodes",
    )
    parser.add_argument(
        "--output-dir", default=None,
        help="Local output directory (default: ~/.cache/huggingface/lerobot/)",
    )
    parser.add_argument(
        "--push", action="store_true",
        help="Push dataset to HuggingFace Hub after collection",
    )
    args = parser.parse_args()
    collect(args)


if __name__ == "__main__":
    main()
