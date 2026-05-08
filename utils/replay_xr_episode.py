#!/usr/bin/env python3
"""Create a visual replay video or EEF dry-run check from an XR episode."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

ROOT_DIR = Path(__file__).resolve().parents[1]
OPEN_HAND_Q = np.zeros(7, dtype=np.float32)
CLOSED_LEFT_HAND_Q = np.array(
    [0.0, 1.0, 1.74, -1.57, -1.74, -1.57, -1.74],
    dtype=np.float32,
)
CLOSED_RIGHT_HAND_Q = np.array(
    [0.0, -1.0, -1.74, 1.57, 1.74, 1.57, 1.74],
    dtype=np.float32,
)


def _resolve_data_json(path: Path) -> Path:
    if path.is_file():
        return path
    data_json = path / "data.json"
    if data_json.is_file():
        return data_json
    matches = sorted(path.glob("**/episode_*/data.json"))
    if len(matches) == 1:
        return matches[0]
    if not matches:
        raise FileNotFoundError(f"No episode data.json found under {path}")
    raise ValueError(
        f"Multiple episodes found under {path}; pass one episode directory."
    )


def _first_color_key(frame: dict[str, Any], requested: str | None) -> str:
    colors = frame.get("colors") or {}
    if requested:
        if requested not in colors:
            raise KeyError(f"Requested color key {requested!r} not in frame")
        return requested
    if not colors:
        raise KeyError("Frame has no color streams")
    return sorted(colors.keys())[0]


def _joint_summary(frame: dict[str, Any], section: str, side: str) -> str:
    qpos = (
        frame.get(section, {})
        .get(f"{side}_arm", {})
        .get("qpos", [])
    )
    if not qpos:
        return f"{side[0].upper()}:-"
    arr = np.asarray(qpos, dtype=float)
    return f"{side[0].upper()}:{arr[0]:+.2f},{arr[1]:+.2f},{arr[3]:+.2f}"


def _draw_overlay(
    image: np.ndarray,
    frame_idx: int,
    total_frames: int,
    color_key: str,
    frame: dict[str, Any],
    metadata: dict[str, Any],
) -> np.ndarray:
    overlay = image.copy()
    cv2.rectangle(overlay, (0, 0), (image.shape[1], 92), (0, 0, 0), -1)
    image = cv2.addWeighted(overlay, 0.45, image, 0.55, 0)

    lines = [
        f"frame {frame_idx + 1}/{total_frames}  stream={color_key}",
        (
            f"arm_mode={metadata.get('arm_mode', '-')}  "
            f"inactive={metadata.get('inactive_arm_pose', '-')}"
        ),
        (
            "action "
            f"{_joint_summary(frame, 'actions', 'left')}  "
            f"{_joint_summary(frame, 'actions', 'right')}"
        ),
    ]
    for i, text in enumerate(lines):
        cv2.putText(
            image,
            text,
            (12, 24 + i * 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
    return image


def create_replay(
    episode_path: Path,
    output_path: Path | None,
    color_key: str | None,
    fps: float | None,
) -> Path:
    data_path = _resolve_data_json(episode_path.expanduser().resolve())
    with data_path.open("r", encoding="utf-8") as f:
        episode = json.load(f)

    frames = episode.get("data", [])
    if not frames:
        raise ValueError(f"No frames in {data_path}")

    info = episode.get("info", {})
    metadata = info.get("metadata", {}) if isinstance(info, dict) else {}
    video_fps = float(fps or info.get("image", {}).get("fps", 30.0))
    key = _first_color_key(frames[0], color_key)

    if output_path is None:
        output_path = data_path.parent / f"replay_{key}.mp4"
    output_path = output_path.expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    writer = None
    written = 0
    try:
        for idx, frame in enumerate(frames):
            rel_path = frame.get("colors", {}).get(key)
            if not rel_path:
                continue
            image_path = data_path.parent / rel_path
            image = cv2.imread(str(image_path))
            if image is None:
                raise FileNotFoundError(f"Could not read image {image_path}")
            image = _draw_overlay(
                image, idx, len(frames), key, frame, metadata
            )
            if writer is None:
                h, w = image.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                writer = cv2.VideoWriter(
                    str(output_path),
                    fourcc,
                    video_fps,
                    (w, h),
                )
            writer.write(image)
            written += 1
    finally:
        if writer is not None:
            writer.release()

    if written == 0:
        raise ValueError(f"No frames written for color stream {key!r}")
    print(f"Wrote {written} frames to {output_path}")
    return output_path


def _quat_norm(pose: list[float]) -> float:
    quat = np.asarray(pose[3:7], dtype=np.float32)
    return float(np.linalg.norm(quat))


def _pose_delta(a: list[float], b: list[float]) -> float:
    pa = np.asarray(a[:3], dtype=np.float32)
    pb = np.asarray(b[:3], dtype=np.float32)
    return float(np.linalg.norm(pb - pa))


def _write_csv(rows: list[dict], out_csv: Path) -> None:
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys()) if rows else []
    with out_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _load_eef_tools() -> tuple[Any, Any, Any, Any]:
    if str(ROOT_DIR / "utils") not in sys.path:
        sys.path.insert(0, str(ROOT_DIR / "utils"))
    from xr_to_lerobot import (
        WristYawLinkFK,
        build_action,
        build_state,
        read_episode,
    )

    return WristYawLinkFK, build_action, build_state, read_episode


def _load_robot_tools() -> tuple[Any, Any, Any, float]:
    if str(ROOT_DIR / "utils") not in sys.path:
        sys.path.insert(0, str(ROOT_DIR / "utils"))
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from vla_client import CONTROL_DT, G1Robot, ensure_ai_mode

    return ChannelFactoryInitialize, G1Robot, ensure_ai_mode, CONTROL_DT


def _qpos(
    frame: dict[str, Any],
    section: str,
    key: str,
    dim: int,
    fallback: np.ndarray | None = None,
) -> np.ndarray:
    block = frame.get(section, {}) or {}
    raw = block.get(key, {}) if isinstance(block, dict) else {}
    if isinstance(raw, dict):
        raw = raw.get("qpos", [])
    arr = np.asarray(raw or [], dtype=np.float32)
    if arr.shape == (dim,):
        return arr
    if fallback is not None:
        return fallback.astype(np.float32).copy()
    raise ValueError(f"Expected {section}.{key}.qpos dim {dim}, got {arr.shape}")


def _smoothstep(alpha: float) -> float:
    alpha = min(max(alpha, 0.0), 1.0)
    return alpha * alpha * (3.0 - 2.0 * alpha)


def _gripper_scalar(hand_qpos: np.ndarray, side: str) -> float:
    closed = CLOSED_LEFT_HAND_Q if side == "left" else CLOSED_RIGHT_HAND_Q
    open_dist = float(np.linalg.norm(hand_qpos - OPEN_HAND_Q))
    close_dist = float(np.linalg.norm(hand_qpos - closed))
    denom = open_dist + close_dist
    if denom <= 1e-6:
        return 0.0
    return float(np.clip(open_dist / denom, 0.0, 1.0))


def _binary_hand_target(
    recorded_hand: np.ndarray,
    side: str,
    previous_target: np.ndarray,
    open_threshold: float,
    close_threshold: float,
) -> np.ndarray:
    scalar = _gripper_scalar(recorded_hand, side)
    if scalar >= close_threshold:
        return (
            CLOSED_LEFT_HAND_Q.copy()
            if side == "left"
            else CLOSED_RIGHT_HAND_Q.copy()
        )
    if scalar <= open_threshold:
        return OPEN_HAND_Q.copy()
    return previous_target.copy()


def _send_replay_cmd(
    robot: Any,
    waist: np.ndarray,
    left_arm: np.ndarray,
    right_arm: np.ndarray,
    left_hand: np.ndarray | None,
    right_hand: np.ndarray | None,
    send_hands: bool,
    mode_machine: int,
) -> None:
    robot.send_arm_cmd(waist, left_arm, right_arm, mode_machine)
    if send_hands and left_hand is not None and right_hand is not None:
        robot.send_hand_cmd(left_hand, right_hand)


def execute_robot_replay(
    episode_path: Path,
    fps: float,
    arm_side: str,
    prepare_seconds: float,
    max_steps: int | None,
    send_hands: bool,
    gripper_mode: str,
    gripper_open_threshold: float,
    gripper_close_threshold: float,
    network_interface: str | None,
) -> None:
    ChannelFactoryInitialize, G1Robot, ensure_ai_mode, control_dt = (
        _load_robot_tools()
    )
    data_path = _resolve_data_json(episode_path.expanduser().resolve())
    with data_path.open("r", encoding="utf-8") as f:
        episode = json.load(f)
    frames = episode.get("data", [])
    if len(frames) < 2:
        raise ValueError(f"Episode needs at least 2 frames: {data_path}")

    print("[REAL REPLAY] Initializing DDS...")
    ChannelFactoryInitialize(0, network_interface)
    if not ensure_ai_mode():
        raise RuntimeError("Robot is not in ai mode; refusing to execute replay.")

    robot = G1Robot()
    robot.init()
    print("[REAL REPLAY] Waiting for robot state...")
    if not robot.wait_for_state(timeout=5.0):
        raise RuntimeError("Timed out waiting for robot lowstate")
    current = robot.get_state()
    if current is None:
        raise RuntimeError("Robot state is not ready")

    first = frames[0]
    start_waist = current["waist"].copy()
    start_left_arm = _qpos(first, "states", "left_arm", 7, current["left_arm"])
    if arm_side == "bimanual":
        start_right_arm = _qpos(
            first, "states", "right_arm", 7, current["right_arm"]
        )
    else:
        start_right_arm = current["right_arm"].copy()
    recorded_start_left_hand = _qpos(
        first, "states", "left_ee", 7, current["left_hand"]
    )
    if gripper_mode == "binary":
        start_left_hand = _binary_hand_target(
            recorded_start_left_hand,
            "left",
            current["left_hand"],
            gripper_open_threshold,
            gripper_close_threshold,
        )
    else:
        start_left_hand = recorded_start_left_hand
    start_right_hand = current["right_hand"].copy()
    if arm_side == "bimanual":
        recorded_start_right_hand = _qpos(
            first, "states", "right_ee", 7, current["right_hand"]
        )
        if gripper_mode == "binary":
            start_right_hand = _binary_hand_target(
                recorded_start_right_hand,
                "right",
                current["right_hand"],
                gripper_open_threshold,
                gripper_close_threshold,
            )
        else:
            start_right_hand = recorded_start_right_hand

    print("[REAL REPLAY] Moving to first recorded state before replay.")
    print(f"  episode: {data_path.parent}")
    print(f"  arm_side: {arm_side}")
    print(f"  gripper_mode: {gripper_mode}")
    print(f"  steps: {len(frames) - 1 if max_steps is None else max_steps}")
    print(f"  fps: {fps}")
    print("  Press Ctrl-C now to abort if robot/workspace is not ready.")
    time.sleep(2.0)

    ramp_start = robot.get_state() or current
    deadline = time.time() + prepare_seconds
    while time.time() < deadline:
        state = robot.get_state() or ramp_start
        alpha = _smoothstep(
            1.0 - max(deadline - time.time(), 0.0) / max(prepare_seconds, 1e-6)
        )
        left_arm = ramp_start["left_arm"] + (
            start_left_arm - ramp_start["left_arm"]
        ) * alpha
        right_arm = ramp_start["right_arm"] + (
            start_right_arm - ramp_start["right_arm"]
        ) * alpha
        left_hand = ramp_start["left_hand"] + (
            start_left_hand - ramp_start["left_hand"]
        ) * alpha
        right_hand = ramp_start["right_hand"] + (
            start_right_hand - ramp_start["right_hand"]
        ) * alpha
        _send_replay_cmd(
            robot,
            start_waist,
            left_arm,
            right_arm,
            left_hand,
            right_hand,
            send_hands,
            state["_mode_machine"],
        )
        time.sleep(control_dt)

    step_dt = 1.0 / fps
    replay_frames = frames[:-1]
    if max_steps is not None:
        replay_frames = replay_frames[:max_steps]

    print("[REAL REPLAY] Start replaying recorded actions.")
    left_hand_target = start_left_hand.copy()
    right_hand_target = start_right_hand.copy()
    for idx, frame in enumerate(replay_frames):
        next_frame = frames[idx + 1]
        loop_t = time.time()
        state = robot.get_state() or current
        left_arm = _qpos(next_frame, "actions", "left_arm", 7, start_left_arm)
        if arm_side == "bimanual":
            right_arm = _qpos(
                next_frame, "actions", "right_arm", 7, start_right_arm
            )
        else:
            right_arm = start_right_arm
        recorded_left_hand = _qpos(
            next_frame, "actions", "left_ee", 7, start_left_hand
        )
        if gripper_mode == "binary":
            left_hand_target = _binary_hand_target(
                recorded_left_hand,
                "left",
                left_hand_target,
                gripper_open_threshold,
                gripper_close_threshold,
            )
            left_hand = left_hand_target
        else:
            left_hand = recorded_left_hand
        right_hand = right_hand_target
        if arm_side == "bimanual":
            recorded_right_hand = _qpos(
                next_frame, "actions", "right_ee", 7, start_right_hand
            )
            if gripper_mode == "binary":
                right_hand_target = _binary_hand_target(
                    recorded_right_hand,
                    "right",
                    right_hand_target,
                    gripper_open_threshold,
                    gripper_close_threshold,
                )
                right_hand = right_hand_target
            else:
                right_hand = recorded_right_hand
        _send_replay_cmd(
            robot,
            start_waist,
            left_arm,
            right_arm,
            left_hand,
            right_hand,
            send_hands,
            state["_mode_machine"],
        )
        if idx % 30 == 0:
            print(f"  replay frame {idx + 1}/{len(replay_frames)}")
        sleep_s = step_dt - (time.time() - loop_t)
        if sleep_s > 0:
            time.sleep(sleep_s)
    print("[REAL REPLAY] Replay complete. Holding final pose briefly.")
    time.sleep(0.5)


def check_eef_replay(
    episode_path: Path,
    fps: float,
    frames_to_print: int,
    out_csv: Path | None,
) -> None:
    WristYawLinkFK, build_action, build_state, read_episode = _load_eef_tools()
    data_path = _resolve_data_json(episode_path.expanduser().resolve())
    payload = read_episode(data_path)
    frames = payload["data"]
    if len(frames) < 2:
        raise ValueError(f"Episode needs at least 2 frames: {data_path}")

    fk = WristYawLinkFK()
    rows = []
    bad_quat_count = 0
    max_left_step = 0.0
    max_right_step = 0.0
    prev_left_action_eef = None
    prev_right_action_eef = None

    for frame_index, frame in enumerate(frames[:-1]):
        next_frame = frames[frame_index + 1]
        state = build_state(frame)
        action = build_action(next_frame)
        obs_eef = fk.wrist_poses(frame, "states")
        action_eef = fk.wrist_poses(next_frame, "actions")
        left_norm = _quat_norm(action_eef["left"])
        right_norm = _quat_norm(action_eef["right"])
        if not (math.isfinite(left_norm) and abs(left_norm - 1.0) < 1e-3):
            bad_quat_count += 1
        if not (math.isfinite(right_norm) and abs(right_norm - 1.0) < 1e-3):
            bad_quat_count += 1
        if prev_left_action_eef is not None:
            max_left_step = max(
                max_left_step,
                _pose_delta(prev_left_action_eef, action_eef["left"]),
            )
            max_right_step = max(
                max_right_step,
                _pose_delta(prev_right_action_eef, action_eef["right"]),
            )
        prev_left_action_eef = action_eef["left"]
        prev_right_action_eef = action_eef["right"]

        rows.append(
            {
                "frame_index": frame_index,
                "timestamp": frame_index / fps,
                "state_dim": len(state),
                "action_dim": len(action),
                "obs_left_x": obs_eef["left"][0],
                "obs_left_y": obs_eef["left"][1],
                "obs_left_z": obs_eef["left"][2],
                "action_left_x": action_eef["left"][0],
                "action_left_y": action_eef["left"][1],
                "action_left_z": action_eef["left"][2],
                "action_left_quat_norm": left_norm,
                "action_right_x": action_eef["right"][0],
                "action_right_y": action_eef["right"][1],
                "action_right_z": action_eef["right"][2],
                "action_right_quat_norm": right_norm,
            }
        )

    print(f"Episode: {data_path.parent}")
    print(f"Frames: {len(frames)} raw, {len(rows)} replay steps")
    print(f"State/action dims: {rows[0]['state_dim']} / {rows[0]['action_dim']}")
    print(
        "Max action EEF step: "
        f"left={max_left_step:.4f}m right={max_right_step:.4f}m"
    )
    print(f"Bad quaternion count: {bad_quat_count}")
    print("Note: dry-run only; no robot command was sent.")

    for row in rows[: max(frames_to_print, 0)]:
        print(
            "  frame={frame_index:04d} t={timestamp:.3f}s "
            "L_action=({action_left_x:.3f}, "
            "{action_left_y:.3f}, {action_left_z:.3f}) "
            "R_action=({action_right_x:.3f}, "
            "{action_right_y:.3f}, {action_right_z:.3f})".format(**row)
        )

    if out_csv is not None:
        _write_csv(rows, out_csv)
        print(f"Wrote CSV: {out_csv}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Create an mp4 replay or EEF dry-run check from an XR episode."
    )
    parser.add_argument("episode", type=Path)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--color-key", default=None)
    parser.add_argument("--fps", type=float, default=None)
    parser.add_argument(
        "--eef-check",
        action="store_true",
        help="Dry-run qpos/EEF replay check instead of writing a video.",
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=10,
        help="Leading frames to print during --eef-check.",
    )
    parser.add_argument(
        "--out-csv",
        type=Path,
        default=None,
        help="Optional full trajectory CSV during --eef-check.",
    )
    parser.add_argument(
        "--execute-robot",
        action="store_true",
        help=(
            "DANGEROUS: command the real robot. Moves to first recorded state "
            "before replaying recorded qpos actions."
        ),
    )
    parser.add_argument(
        "--arm-side",
        choices=("left", "bimanual"),
        default="left",
        help="Default left keeps the right arm at the starting pose.",
    )
    parser.add_argument(
        "--prepare-seconds",
        type=float,
        default=4.0,
        help="Ramp duration to the first recorded state before real replay.",
    )
    parser.add_argument(
        "--max-steps",
        type=int,
        default=None,
        help="Limit real robot replay steps for cautious tests.",
    )
    parser.add_argument(
        "--send-hands",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Send recorded hand qpos during real replay.",
    )
    parser.add_argument(
        "--gripper-mode",
        choices=("recorded", "binary"),
        default="recorded",
        help=(
            "Hand replay mode. binary maps recorded Dex3 qpos to fixed "
            "OPEN/CLOSE targets with hysteresis."
        ),
    )
    parser.add_argument("--gripper-open-threshold", type=float, default=0.4)
    parser.add_argument("--gripper-close-threshold", type=float, default=0.6)
    parser.add_argument("--network-interface", type=str, default=None)
    args = parser.parse_args()

    fps = float(args.fps or 30.0)
    if args.execute_robot:
        execute_robot_replay(
            args.episode,
            fps=fps,
            arm_side=args.arm_side,
            prepare_seconds=args.prepare_seconds,
            max_steps=args.max_steps,
            send_hands=args.send_hands,
            gripper_mode=args.gripper_mode,
            gripper_open_threshold=args.gripper_open_threshold,
            gripper_close_threshold=args.gripper_close_threshold,
            network_interface=args.network_interface,
        )
    elif args.eef_check:
        check_eef_replay(args.episode, fps, args.frames, args.out_csv)
    else:
        create_replay(args.episode, args.output, args.color_key, args.fps)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
