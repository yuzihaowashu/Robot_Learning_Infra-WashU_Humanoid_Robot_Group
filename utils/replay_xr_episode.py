#!/usr/bin/env python3
"""Create a visual replay video from a raw XR teleoperation episode."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np


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


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Create an mp4 replay from a raw XR episode."
    )
    parser.add_argument("episode", type=Path)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--color-key", default=None)
    parser.add_argument("--fps", type=float, default=None)
    args = parser.parse_args()

    create_replay(args.episode, args.output, args.color_key, args.fps)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
