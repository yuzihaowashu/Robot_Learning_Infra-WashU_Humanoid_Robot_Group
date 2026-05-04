#!/usr/bin/env python3
"""Convert XR teleoperation recordings to a LeRobot-style HF dataset."""

from __future__ import annotations

import argparse
import json
import math
import shutil
from collections import defaultdict
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from datasets import Dataset, Features, Sequence, Value
from huggingface_hub import create_repo, create_tag, upload_large_folder


CODE_VERSION = "xr-teleop-v1"
DEFAULT_TASKS = (
    "place_bottle_in_paper_box",
    "take_bottle_out_of_paper_box",
)


@dataclass
class InfoDict:
    codebase_version: str
    robot_type: str
    total_episodes: int
    total_frames: int
    total_tasks: int
    total_videos: int
    total_chunks: int
    chunks_size: int
    fps: int
    data_path: str
    video_path: str
    features: dict[str, Any]


def read_episode(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        payload = json.load(f)
    if not isinstance(payload, dict) or not isinstance(
        payload.get("data"), list
    ):
        raise ValueError(f"Malformed episode file: {path}")
    return payload


def get_qpos(frame: dict[str, Any], section: str, name: str) -> list[float]:
    block = frame.get(section, {}) or {}
    value = block.get(name, {}) if isinstance(block, dict) else {}
    if isinstance(value, dict):
        qpos = value.get("qpos", [])
    else:
        qpos = value or []
    return [float(x) for x in qpos]


def build_state(frame: dict[str, Any]) -> list[float]:
    return (
        get_qpos(frame, "states", "left_arm")
        + get_qpos(frame, "states", "right_arm")
        + get_qpos(frame, "states", "left_ee")
        + get_qpos(frame, "states", "right_ee")
        + get_qpos(frame, "states", "body")
    )


def build_action(frame: dict[str, Any]) -> list[float]:
    return (
        get_qpos(frame, "actions", "left_arm")
        + get_qpos(frame, "actions", "right_arm")
        + get_qpos(frame, "actions", "left_ee")
        + get_qpos(frame, "actions", "right_ee")
        + get_qpos(frame, "actions", "body")
    )


def image_path(episode_dir: Path, frame: dict[str, Any], camera: str) -> Path:
    colors = frame.get("colors", {}) or {}
    rel = colors.get(camera)
    if rel is None:
        raise ValueError(f"Missing camera '{camera}' in {episode_dir}")
    path = episode_dir / rel
    if not path.exists():
        raise FileNotFoundError(path)
    return path


def make_video(
    image_paths: list[Path], video_path: Path, fps: int
) -> tuple[int, int]:
    if not image_paths:
        raise ValueError("Cannot create video with zero frames")

    first = cv2.imread(str(image_paths[0]), cv2.IMREAD_COLOR)
    if first is None:
        raise ValueError(f"Failed to read image: {image_paths[0]}")
    height, width = first.shape[:2]

    video_path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = video_path.with_suffix(".tmp.mp4")
    writer = cv2.VideoWriter(
        str(tmp_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        float(fps),
        (width, height),
    )
    if not writer.isOpened():
        raise RuntimeError(f"Failed to open video writer: {tmp_path}")

    try:
        for path in image_paths:
            img = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if img is None:
                raise ValueError(f"Failed to read image: {path}")
            if img.shape[:2] != (height, width):
                img = cv2.resize(
                    img, (width, height), interpolation=cv2.INTER_AREA
                )
            writer.write(img)
    finally:
        writer.release()

    tmp_path.replace(video_path)
    return height, width


def episode_sort_key(path: Path) -> tuple[int, int, str]:
    raw_id = -1
    ep_id = -1
    for part in path.parts:
        if part.startswith("raw_episode_"):
            raw_id = int(part.rsplit("_", 1)[1])
        elif part.startswith("episode_"):
            ep_id = int(part.rsplit("_", 1)[1])
    return raw_id, ep_id, str(path)


def discover_episodes(
    data_root: Path, tasks: list[str]
) -> list[tuple[int, str, Path]]:
    out: list[tuple[int, str, Path]] = []
    for task_index, task in enumerate(tasks):
        task_dir = data_root / task
        if not task_dir.exists():
            raise FileNotFoundError(f"Missing task directory: {task_dir}")
        episode_dirs = sorted(
            {p.parent for p in task_dir.glob("**/data.json")},
            key=episode_sort_key,
        )
        for ep_dir in episode_dirs:
            out.append((task_index, task, ep_dir))
    return out


def task_description(payload: dict[str, Any], fallback: str) -> str:
    text = payload.get("text", {}) or {}
    return str(text.get("goal") or text.get("desc") or fallback)


def vector_stats(values: list[list[float]]) -> dict[str, Any]:
    arr = np.asarray(values, dtype=np.float32)
    return {
        "min": arr.min(axis=0).tolist(),
        "max": arr.max(axis=0).tolist(),
        "mean": arr.mean(axis=0).tolist(),
        "std": arr.std(axis=0).tolist(),
        "count": int(arr.shape[0]),
    }


class XrToLeRobotConverter:
    def __init__(self, camera: str, fps: int, chunks_size: int):
        self.camera = camera
        self.fps = fps
        self.chunks_size = chunks_size
        self.features = Features(
            {
                "states": Sequence(Value("float32")),
                "action": Sequence(Value("float32")),
                "timestamp": Value("float32"),
                "frame_index": Value("int64"),
                "episode_index": Value("int64"),
                "index": Value("int64"),
                "task_index": Value("int64"),
                "next.done": Value("bool"),
            }
        )
        self.task_meta: dict[int, dict[str, Any]] = {}
        self.episode_meta: list[dict[str, Any]] = []
        self.episode_stats: list[dict[str, Any]] = []
        self.total_frames = 0
        self.state_dim: int | None = None
        self.action_dim: int | None = None
        self.image_shape: tuple[int, int, int] | None = None

    def convert_one(
        self,
        episode_index: int,
        task_index: int,
        task_name: str,
        episode_dir: Path,
        out_dir: Path,
        dataset_cursor: int,
    ) -> int:
        payload = read_episode(episode_dir / "data.json")
        frames = payload["data"]
        if len(frames) < 2:
            raise ValueError(f"Episode needs at least 2 frames: {episode_dir}")

        if task_index not in self.task_meta:
            self.task_meta[task_index] = {
                "name": task_name,
                "description": task_description(payload, task_name),
            }

        states: list[list[float]] = []
        actions: list[list[float]] = []
        rows: list[dict[str, Any]] = []
        image_paths: list[Path] = []
        global_start = dataset_cursor

        for frame_index, frame in enumerate(frames[:-1]):
            state = build_state(frame)
            action = build_action(frames[frame_index + 1])
            if self.state_dim is None:
                self.state_dim = len(state)
            if self.action_dim is None:
                self.action_dim = len(action)
            if len(state) != self.state_dim:
                raise ValueError(
                    f"State dim changed in {episode_dir}: "
                    f"{len(state)} != {self.state_dim}"
                )
            if len(action) != self.action_dim:
                raise ValueError(
                    f"Action dim changed in {episode_dir}: "
                    f"{len(action)} != {self.action_dim}"
                )

            states.append(state)
            actions.append(action)
            image_paths.append(image_path(episode_dir, frame, self.camera))
            rows.append(
                {
                    "states": state,
                    "action": action,
                    "timestamp": frame_index / float(self.fps),
                    "frame_index": frame_index,
                    "episode_index": episode_index,
                    "index": global_start + frame_index,
                    "task_index": task_index,
                    "next.done": frame_index == len(frames) - 2,
                }
            )

        chunk_id = episode_index // self.chunks_size
        data_path = (
            out_dir
            / "data"
            / f"chunk-{chunk_id:03d}"
            / f"episode_{episode_index:06d}.parquet"
        )
        video_path = (
            out_dir
            / "videos"
            / f"chunk-{chunk_id:03d}"
            / "egocentric"
            / f"episode_{episode_index:06d}.mp4"
        )
        data_path.parent.mkdir(parents=True, exist_ok=True)

        ds = Dataset.from_list(rows, features=self.features)
        ds.to_parquet(str(data_path))

        height, width = make_video(image_paths, video_path, self.fps)
        self.image_shape = (height, width, 3)

        self.episode_meta.append(
            {
                "episode_index": episode_index,
                "tasks": [task_index],
                "length": len(rows),
                "dataset_from_index": dataset_cursor,
                "dataset_to_index": dataset_cursor + len(rows),
                "source": str(episode_dir),
            }
        )
        self.episode_stats.append(
            {
                "episode_index": episode_index,
                "stats": {
                    "states": vector_stats(states),
                    "action": vector_stats(actions),
                },
            }
        )
        self.total_frames += len(rows)
        return len(rows)

    def write_meta(self, out_dir: Path) -> None:
        if (
            self.state_dim is None
            or self.action_dim is None
            or self.image_shape is None
        ):
            raise RuntimeError("No episodes converted")

        meta_dir = out_dir / "meta"
        meta_dir.mkdir(parents=True, exist_ok=True)
        height, width, channels = self.image_shape
        features_meta = {
            "observation.images.egocentric": {
                "dtype": "video",
                "shape": [height, width, channels],
                "names": ["height", "width", "channel"],
                "info": {
                    "video.fps": self.fps,
                    "video.codec": "mp4v",
                    "video.pix_fmt": "yuv420p",
                    "video.is_depth_map": False,
                    "has_audio": False,
                },
            },
            "states": {
                "dtype": "float32",
                "shape": [self.state_dim],
                "names": [f"state_{i}" for i in range(self.state_dim)],
            },
            "action": {
                "dtype": "float32",
                "shape": [self.action_dim],
                "names": [f"action_{i}" for i in range(self.action_dim)],
            },
            "timestamp": {"dtype": "float32", "shape": [1]},
            "frame_index": {"dtype": "int64", "shape": [1]},
            "episode_index": {"dtype": "int64", "shape": [1]},
            "index": {"dtype": "int64", "shape": [1]},
            "task_index": {"dtype": "int64", "shape": [1]},
            "next.done": {"dtype": "bool", "shape": [1]},
        }
        info = InfoDict(
            codebase_version=CODE_VERSION,
            robot_type="g1",
            total_episodes=len(self.episode_meta),
            total_frames=self.total_frames,
            total_tasks=len(self.task_meta),
            total_videos=len(self.episode_meta),
            total_chunks=math.ceil(
                len(self.episode_meta) / self.chunks_size
            ),
            chunks_size=self.chunks_size,
            fps=self.fps,
            data_path=(
                "data/chunk-{episode_chunk:03d}/"
                "episode_{episode_index:06d}.parquet"
            ),
            video_path=(
                "videos/chunk-{episode_chunk:03d}/egocentric/"
                "episode_{episode_index:06d}.mp4"
            ),
            features=features_meta,
        )
        (meta_dir / "info.json").write_text(
            json.dumps(asdict(info), indent=4), encoding="utf-8"
        )

        task_rows = [
            {
                "task_index": task_index,
                "task": meta["name"],
                "category": "default",
                "description": meta["description"],
            }
            for task_index, meta in sorted(self.task_meta.items())
        ]
        for path, rows in (
            (meta_dir / "tasks.jsonl", task_rows),
            (
                meta_dir / "episodes.jsonl",
                sorted(
                    self.episode_meta,
                    key=lambda x: x["episode_index"],
                ),
            ),
            (
                meta_dir / "episodes_stats.jsonl",
                sorted(
                    self.episode_stats,
                    key=lambda x: x["episode_index"],
                ),
            ),
        ):
            with path.open("w", encoding="utf-8") as f:
                for row in rows:
                    json.dump(row, f, ensure_ascii=False)
                    f.write("\n")

        task_summary = "\n".join(
            f"- `{row['task']}`: {row['description']}"
            for row in task_rows
        )
        readme = f"""---
tags:
- lerobot
- robotics
- g1
- xr-teleoperation
task_categories:
- robotics
---

# XR Teleoperation LeRobot Dataset

This dataset was converted from G1 XR teleoperation recordings.

## Summary

- Robot: G1
- FPS: {self.fps}
- Episodes: {len(self.episode_meta)}
- Frames: {self.total_frames}
- Camera: egocentric `{self.camera}`
- State dimension: {self.state_dim}
- Action dimension: {self.action_dim}

## Tasks

{task_summary}

## Format

The dataset follows a LeRobot-style layout with parquet files under
`data/`, MP4 videos under `videos/`, and metadata under `meta/`.
"""
        (out_dir / "README.md").write_text(readme, encoding="utf-8")


def summarize_dataset(out_dir: Path) -> None:
    info = json.loads(
        (out_dir / "meta" / "info.json").read_text(encoding="utf-8")
    )
    counts: dict[str, int] = defaultdict(int)
    tasks = {}
    with (out_dir / "meta" / "tasks.jsonl").open("r", encoding="utf-8") as f:
        for line in f:
            row = json.loads(line)
            tasks[row["task_index"]] = row["task"]
    with (out_dir / "meta" / "episodes.jsonl").open(
        "r", encoding="utf-8"
    ) as f:
        for line in f:
            row = json.loads(line)
            counts[tasks[row["tasks"][0]]] += 1

    print(f"Wrote LeRobot dataset: {out_dir}")
    print(f"  episodes: {info['total_episodes']}")
    print(f"  frames:   {info['total_frames']}")
    print(f"  tasks:    {counts}")
    print(
        f"  state_dim={info['features']['states']['shape'][0]} "
        f"action_dim={info['features']['action']['shape'][0]}"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, default=Path("xr_recordings"))
    parser.add_argument(
        "--out-dir", type=Path, default=Path("xr_recordings_lerobot")
    )
    parser.add_argument("--tasks", type=str, default=",".join(DEFAULT_TASKS))
    parser.add_argument("--camera", type=str, default="color_0")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--chunks-size", type=int, default=1000)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--push", action="store_true")
    parser.add_argument("--repo-id", type=str, default=None)
    parser.add_argument("--private", action="store_true")
    parser.add_argument("--repo-exist-ok", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    tasks = [t.strip() for t in args.tasks.split(",") if t.strip()]
    episodes = discover_episodes(args.data_root, tasks)
    if not episodes:
        raise RuntimeError("No episodes found")

    if args.out_dir.exists():
        if not args.overwrite:
            raise FileExistsError(
                f"{args.out_dir} exists; pass --overwrite to replace it"
            )
        shutil.rmtree(args.out_dir)
    args.out_dir.mkdir(parents=True, exist_ok=True)

    converter = XrToLeRobotConverter(args.camera, args.fps, args.chunks_size)
    cursor = 0
    for episode_index, (
        task_index,
        task_name,
        episode_dir,
    ) in enumerate(episodes):
        cursor += converter.convert_one(
            episode_index=episode_index,
            task_index=task_index,
            task_name=task_name,
            episode_dir=episode_dir,
            out_dir=args.out_dir,
            dataset_cursor=cursor,
        )
        print(f"[{episode_index + 1}/{len(episodes)}] converted {episode_dir}")

    converter.write_meta(args.out_dir)
    summarize_dataset(args.out_dir)

    if args.push:
        if not args.repo_id:
            raise ValueError("--repo-id is required with --push")
        create_repo(
            args.repo_id,
            repo_type="dataset",
            private=args.private,
            exist_ok=args.repo_exist_ok,
        )
        upload_large_folder(
            repo_id=args.repo_id,
            repo_type="dataset",
            folder_path=str(args.out_dir),
        )
        create_tag(args.repo_id, tag=CODE_VERSION, repo_type="dataset")
        print(f"Uploaded to https://huggingface.co/datasets/{args.repo_id}")


if __name__ == "__main__":
    main()
