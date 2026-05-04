#!/usr/bin/env python3
"""Validate raw XR teleoperation episodes saved by EpisodeWriter."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


EXPECTED_ARM_DOF = 7


def _as_episode_json_paths(path: Path) -> list[Path]:
    if path.is_file():
        return [path]
    if path.name.startswith("episode_") and (path / "data.json").is_file():
        return [path / "data.json"]
    return sorted(path.glob("**/episode_*/data.json"))


def _qpos_len(frame: dict[str, Any], section: str, side: str) -> int | None:
    value = (
        frame.get(section, {})
        .get(f"{side}_arm", {})
        .get("qpos")
    )
    return len(value) if isinstance(value, list) else None


def _check_episode(data_path: Path) -> tuple[bool, list[str]]:
    messages: list[str] = []
    ok = True

    try:
        with data_path.open("r", encoding="utf-8") as f:
            episode = json.load(f)
    except Exception as exc:
        return False, [f"ERROR invalid JSON: {exc}"]

    if not isinstance(episode, dict):
        return False, ["ERROR expected top-level JSON object"]

    frames = episode.get("data")
    if not isinstance(frames, list):
        return False, ["ERROR missing or invalid `data` list"]

    info = episode.get("info", {})
    text = episode.get("text", {})
    metadata = info.get("metadata", {}) if isinstance(info, dict) else {}
    date = info.get("date", "-") if isinstance(info, dict) else "-"
    arm_mode = "-"
    inactive_pose = "-"
    if isinstance(metadata, dict):
        arm_mode = metadata.get("arm_mode", "-")
        inactive_pose = metadata.get("inactive_arm_pose", "-")
    goal = text.get("goal", "-") if isinstance(text, dict) else "-"
    messages.append(
        f"frames={len(frames)} date={date} arm_mode={arm_mode} "
        f"inactive_pose={inactive_pose} goal={goal}"
    )

    if not frames:
        ok = False
        messages.append(
            "ERROR no frames recorded. Press Left X to start recording "
            "before Right A save."
        )
        return ok, messages

    missing_images = 0
    bad_arm_shapes = 0
    for frame in frames:
        if not isinstance(frame, dict):
            ok = False
            messages.append("ERROR frame is not an object")
            continue

        colors = frame.get("colors") or {}
        if isinstance(colors, dict):
            for rel_path in colors.values():
                if rel_path and not (data_path.parent / rel_path).is_file():
                    missing_images += 1

        for section in ("states", "actions"):
            for side in ("left", "right"):
                qlen = _qpos_len(frame, section, side)
                if qlen != EXPECTED_ARM_DOF:
                    bad_arm_shapes += 1

    if missing_images:
        ok = False
        messages.append(
            f"ERROR missing referenced color images: {missing_images}"
        )
    if bad_arm_shapes:
        ok = False
        messages.append(f"ERROR invalid arm qpos entries: {bad_arm_shapes}")

    first = frames[0]
    colors = first.get("colors") or {}
    messages.append(f"first_frame_color_streams={list(colors.keys())}")
    messages.append("OK" if ok else "FAILED")
    return ok, messages


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Check raw XR episode data.json files for recording "
            "completeness."
        )
    )
    parser.add_argument(
        "path",
        type=Path,
        help=(
            "Path to a data.json, episode_xxxx directory, task directory, "
            "or xr_recordings root."
        ),
    )
    args = parser.parse_args()

    paths = _as_episode_json_paths(args.path.expanduser().resolve())
    if not paths:
        print(f"No episode data.json files found under: {args.path}")
        return 1

    all_ok = True
    for data_path in paths:
        ok, messages = _check_episode(data_path)
        all_ok = all_ok and ok
        print(f"\n{data_path}")
        for message in messages:
            print(f"  {message}")

    return 0 if all_ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
