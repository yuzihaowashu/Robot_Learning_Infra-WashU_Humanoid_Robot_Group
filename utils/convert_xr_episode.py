#!/usr/bin/env python3
"""
Convert xr_teleoperate EpisodeWriter recordings to collect_dataset.py trajectory JSON.

xr_teleoperate records episodes as:
    episode_NNNN/
        data.json           ← states/actions per frame + image paths
        colors/*.jpg        ← camera frames (not converted, only referenced)

collect_dataset.py expects:
    {"metadata": {...}, "frames": [{"t": 0.0, "arm": {"15": ..}, "hand": {"0": ..}}, ...]}

This script bridges the two formats so that xr_teleoperate recordings can be
replayed and collected into LeRobot v2 datasets using the existing pipeline.

Usage:
    # Convert a single episode
    python utils/convert_xr_episode.py \\
        --input xr_teleoperate/teleop/utils/data/pick_cube/episode_0001 \\
        --output trajectories/xr_ep_0001.json

    # Convert all episodes in a task directory
    python utils/convert_xr_episode.py \\
        --input xr_teleoperate/teleop/utils/data/pick_cube \\
        --output trajectories/ \\
        --batch

    # Use states instead of actions (record actual robot positions)
    python utils/convert_xr_episode.py \\
        --input xr_teleoperate/teleop/utils/data/pick_cube/episode_0001 \\
        --output trajectories/xr_ep_0001.json \\
        --use-states
"""

import argparse
import json
import os
import sys
import time
from pathlib import Path

# Unitree G1 29DoF arm joint indices (matches G1_29_JointArmIndex in
# xr_teleoperate/teleop/robot_control/robot_arm.py and ARM_JOINTS in teach.py)
LEFT_ARM_JOINT_START = 15   # kLeftShoulderPitch
RIGHT_ARM_JOINT_START = 22  # kRightShoulderPitch
ARM_JOINTS_PER_SIDE = 7
ARM_JOINTS = list(range(LEFT_ARM_JOINT_START, LEFT_ARM_JOINT_START + ARM_JOINTS_PER_SIDE)) + \
             list(range(RIGHT_ARM_JOINT_START, RIGHT_ARM_JOINT_START + ARM_JOINTS_PER_SIDE))

N_HAND_MOTORS = 7

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


def load_xr_episode(episode_dir):
    """Load an xr_teleoperate episode directory, return parsed data.json."""
    data_json = os.path.join(episode_dir, "data.json")
    if not os.path.isfile(data_json):
        raise FileNotFoundError(f"data.json not found in {episode_dir}")

    with open(data_json, "r", encoding="utf-8") as f:
        data = json.load(f)

    info = data.get("info", {})
    text = data.get("text", {})
    frames = data.get("data", [])
    return info, text, frames


def xr_frame_to_trajectory_frame(xr_frame, frequency, source_key="actions"):
    """Convert one xr_teleoperate frame to collect_dataset.py frame format.

    Args:
        xr_frame: dict with 'idx', 'states', 'actions', etc.
        frequency: recording frequency (Hz) to compute timestamp.
        source_key: 'actions' (IK command targets) or 'states' (actual robot positions).

    Returns:
        dict with 't', 'arm', 'hand' keys.
    """
    idx = xr_frame["idx"]
    t = round(idx / frequency, 4)

    source = xr_frame.get(source_key) or {}
    if not source:
        source = xr_frame.get("actions") or xr_frame.get("states") or {}

    arm = {}
    left_arm_qpos = source.get("left_arm", {}).get("qpos", [])
    right_arm_qpos = source.get("right_arm", {}).get("qpos", [])

    for i, val in enumerate(left_arm_qpos):
        joint_idx = LEFT_ARM_JOINT_START + i
        arm[str(joint_idx)] = round(float(val), 6)

    for i, val in enumerate(right_arm_qpos):
        joint_idx = RIGHT_ARM_JOINT_START + i
        arm[str(joint_idx)] = round(float(val), 6)

    hand = {}
    left_ee_qpos = source.get("left_ee", {}).get("qpos", [])
    right_ee_qpos = source.get("right_ee", {}).get("qpos", [])

    for i, val in enumerate(left_ee_qpos):
        if i < N_HAND_MOTORS:
            hand[str(i)] = round(float(val), 6)

    for i, val in enumerate(right_ee_qpos):
        if i < N_HAND_MOTORS:
            hand[str(i + N_HAND_MOTORS)] = round(float(val), 6)

    frame = {"t": t, "arm": arm}
    if hand:
        frame["hand"] = hand
    return frame


def convert_episode(episode_dir, output_path, source_key="actions"):
    """Convert a single xr_teleoperate episode to trajectory JSON.

    Returns:
        Path to the written JSON file, or None on failure.
    """
    info, text, xr_frames = load_xr_episode(episode_dir)

    if not xr_frames:
        print(f"  WARNING: No frames in {episode_dir}, skipping.")
        return None

    frequency = info.get("image", {}).get("fps", 30.0)

    frames = []
    for xr_frame in xr_frames:
        frame = xr_frame_to_trajectory_frame(xr_frame, frequency, source_key)
        frames.append(frame)

    has_hands = any("hand" in f and f["hand"] for f in frames)

    metadata = {
        "record_hz": int(frequency),
        "n_frames": len(frames),
        "duration_s": round(frames[-1]["t"], 2) if frames else 0,
        "arm_joints": ARM_JOINTS,
        "arm_joint_names": ARM_JOINT_NAMES,
        "record_hands": has_hands,
        "control_mode": "xr_teleoperate",
        "source": "xr_teleoperate",
        "source_key": source_key,
        "task_goal": text.get("goal", ""),
        "created": time.strftime("%Y-%m-%d %H:%M:%S"),
        "original_episode": os.path.basename(episode_dir),
    }
    if has_hands:
        metadata["hand_joint_names"] = HAND_JOINT_NAMES

    trajectory = {"metadata": metadata, "frames": frames}

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    with open(output_path, "w") as f:
        json.dump(trajectory, f, indent=2)

    print(f"  Converted: {output_path}")
    print(f"    Frames: {metadata['n_frames']}, Duration: {metadata['duration_s']}s, "
          f"Hz: {metadata['record_hz']}, Hands: {has_hands}")
    return output_path


def find_episodes(task_dir):
    """Find all episode_NNNN directories under a task directory."""
    episodes = []
    for entry in sorted(os.listdir(task_dir)):
        full = os.path.join(task_dir, entry)
        if os.path.isdir(full) and entry.startswith("episode_"):
            data_json = os.path.join(full, "data.json")
            if os.path.isfile(data_json):
                episodes.append(full)
    return episodes


def main():
    parser = argparse.ArgumentParser(
        description="Convert xr_teleoperate episodes to collect_dataset.py trajectory JSON"
    )
    parser.add_argument(
        "--input", required=True,
        help="Path to episode directory (episode_NNNN/) or task directory containing episodes",
    )
    parser.add_argument(
        "--output", required=True,
        help="Output JSON path (single mode) or output directory (batch mode)",
    )
    parser.add_argument(
        "--batch", action="store_true",
        help="Convert all episodes in the input directory",
    )
    parser.add_argument(
        "--use-states", action="store_true",
        help="Use 'states' (actual robot positions) instead of 'actions' (IK targets)",
    )
    args = parser.parse_args()

    source_key = "states" if args.use_states else "actions"

    if args.batch:
        episodes = find_episodes(args.input)
        if not episodes:
            print(f"No episodes found in {args.input}")
            sys.exit(1)

        os.makedirs(args.output, exist_ok=True)
        print(f"Converting {len(episodes)} episodes from {args.input}")
        converted = 0
        for ep_dir in episodes:
            ep_name = os.path.basename(ep_dir)
            ep_num = ep_name.replace("episode_", "")
            out_path = os.path.join(args.output, f"xr_teleop_{ep_num}.json")
            result = convert_episode(ep_dir, out_path, source_key)
            if result:
                converted += 1

        print(f"\nDone: {converted}/{len(episodes)} episodes converted.")
        print(f"Output directory: {args.output}")
    else:
        data_json = os.path.join(args.input, "data.json")
        if not os.path.isfile(data_json):
            print(f"Error: {data_json} not found.")
            print("For a single episode, --input should point to episode_NNNN/ directory.")
            print("For batch conversion, add --batch flag.")
            sys.exit(1)

        convert_episode(args.input, args.output, source_key)

    print("\nTo replay and collect into LeRobot dataset:")
    print("  bash run_collect.sh --trajectories trajectories/xr_teleop_*.json \\")
    print('      --task "your task description"')


if __name__ == "__main__":
    main()
