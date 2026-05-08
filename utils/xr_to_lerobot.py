#!/usr/bin/env python3
"""Convert XR teleoperation recordings to a LeRobot-style HF dataset."""

from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
from collections import defaultdict
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from datasets import Dataset, Features, Sequence, Value
from huggingface_hub import create_repo, create_tag, upload_large_folder
from huggingface_hub.errors import HfHubHTTPError


CODE_VERSION = "xr-teleop-v1"
DEFAULT_TACTILE_DIM = 216
DEFAULT_TASKS = (
    "place_bottle_in_paper_box",
    "take_bottle_out_of_paper_box",
)
ROOT_DIR = Path(__file__).resolve().parents[1]
WBC_DIR = (
    ROOT_DIR
    / "Isaac-GR00T"
    / "external_dependencies"
    / "GR00T-WholeBodyControl"
)
EEF_POSE_DIM = 7
GRIPPER_DIM = 1
OPEN_HAND_Q = np.zeros(7, dtype=np.float32)
CLOSED_LEFT_HAND_Q = np.array(
    [0.0, 1.0, 1.74, -1.57, -1.74, -1.57, -1.74],
    dtype=np.float32,
)
CLOSED_RIGHT_HAND_Q = np.array(
    [0.0, -1.0, -1.74, 1.57, 1.74, 1.57, 1.74],
    dtype=np.float32,
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


def hand_qpos_to_gripper(hand_qpos: list[float], side: str) -> list[float]:
    q = np.asarray(hand_qpos, dtype=np.float32)
    if q.shape != (7,):
        return [0.0]
    closed = CLOSED_LEFT_HAND_Q if side == "left" else CLOSED_RIGHT_HAND_Q
    open_dist = float(np.linalg.norm(q - OPEN_HAND_Q))
    close_dist = float(np.linalg.norm(q - closed))
    denom = open_dist + close_dist
    if denom <= 1e-6:
        return [0.0]
    # Smooth scalar: 0=open, 1=closed, intermediate values keep transitions.
    return [float(np.clip(open_dist / denom, 0.0, 1.0))]


def build_gripper(frame: dict[str, Any], section: str) -> dict[str, list[float]]:
    return {
        "left": hand_qpos_to_gripper(
            get_qpos(frame, section, "left_ee"),
            "left",
        ),
        "right": hand_qpos_to_gripper(
            get_qpos(frame, section, "right_ee"),
            "right",
        ),
    }


def _pose_to_xyz_quat(se3: Any) -> list[float]:
    import pinocchio as pin

    quat = pin.Quaternion(se3.rotation)
    quat.normalize()
    coeffs = quat.coeffs()
    return (
        [float(x) for x in se3.translation]
        + [float(x) for x in coeffs]
    )


class WristYawLinkFK:
    def __init__(self) -> None:
        if not WBC_DIR.exists():
            raise FileNotFoundError(f"Missing WBC dependency: {WBC_DIR}")
        if str(WBC_DIR) not in sys.path:
            sys.path.insert(0, str(WBC_DIR))
        from gr00t_wbc.control.robot_model.instantiation.g1 import (
            instantiate_g1_robot_model,
        )

        self.robot_model = instantiate_g1_robot_model()
        self.left_frame = self.robot_model.supplemental_info.hand_frame_names["left"]
        self.right_frame = self.robot_model.supplemental_info.hand_frame_names["right"]
        self.default_body_q = self.robot_model.get_body_actuated_joints(
            self.robot_model.get_default_body_pose()
        ).astype(np.float32)

    def _body_q_from_frame(
        self,
        frame: dict[str, Any],
        section: str,
    ) -> np.ndarray:
        body_q = self.default_body_q.copy()
        recorded_body = get_qpos(frame, section, "body")
        if len(recorded_body) >= 15:
            body_q[:15] = np.asarray(recorded_body[:15], dtype=np.float32)

        left_arm = get_qpos(frame, section, "left_arm")
        right_arm = get_qpos(frame, section, "right_arm")
        if len(left_arm) == 7:
            body_q[15:22] = np.asarray(left_arm, dtype=np.float32)
        if len(right_arm) == 7:
            body_q[22:29] = np.asarray(right_arm, dtype=np.float32)
        return body_q

    def wrist_poses(
        self,
        frame: dict[str, Any],
        section: str,
    ) -> dict[str, list[float]]:
        body_q = self._body_q_from_frame(frame, section)
        q = self.robot_model.get_configuration_from_actuated_joints(
            body_actuated_joint_values=body_q
        )
        self.robot_model.cache_forward_kinematics(q, auto_clip=False)
        return {
            "left": _pose_to_xyz_quat(
                self.robot_model.frame_placement(self.left_frame)
            ),
            "right": _pose_to_xyz_quat(
                self.robot_model.frame_placement(self.right_frame)
            ),
        }


def flatten_tactile(frame: dict[str, Any], tactile_dim: int) -> list[float]:
    tactiles = frame.get("tactiles") or {}
    values: list[float] = []
    if isinstance(tactiles, dict):
        for name in ("left_ee", "right_ee"):
            raw = tactiles.get(name) or []
            values.extend(float(x) for x in raw)
    elif isinstance(tactiles, list):
        values.extend(float(x) for x in tactiles)

    if tactile_dim <= 0:
        return values
    if len(values) < tactile_dim:
        values.extend([0.0] * (tactile_dim - len(values)))
    return values[:tactile_dim]


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
    def __init__(
        self,
        camera: str,
        fps: int,
        chunks_size: int,
        tactile_dim: int,
        include_eef: bool = False,
        include_gripper: bool = False,
    ):
        self.camera = camera
        self.fps = fps
        self.chunks_size = chunks_size
        self.tactile_dim = tactile_dim
        self.include_eef = include_eef
        self.include_gripper = include_gripper
        feature_dict = {
            "states": Sequence(Value("float32")),
            "action": Sequence(Value("float32")),
            "observation.tactile": Sequence(Value("float32")),
            "timestamp": Value("float32"),
            "frame_index": Value("int64"),
            "episode_index": Value("int64"),
            "index": Value("int64"),
            "task_index": Value("int64"),
            "next.done": Value("bool"),
        }
        if include_eef:
            feature_dict.update(
                {
                    "observation.eef.left": Sequence(Value("float32")),
                    "observation.eef.right": Sequence(Value("float32")),
                    "action.eef.left": Sequence(Value("float32")),
                    "action.eef.right": Sequence(Value("float32")),
                }
            )
        if include_gripper:
            feature_dict.update(
                {
                    "observation.gripper.left": Sequence(Value("float32")),
                    "observation.gripper.right": Sequence(Value("float32")),
                    "action.gripper.left": Sequence(Value("float32")),
                    "action.gripper.right": Sequence(Value("float32")),
                }
            )
        self.features = Features(feature_dict)
        self.eef_fk = WristYawLinkFK() if include_eef else None
        self.task_meta: dict[int, dict[str, Any]] = {}
        self.episode_meta: list[dict[str, Any]] = []
        self.episode_stats: list[dict[str, Any]] = []
        self.total_frames = 0
        self.state_dim: int | None = None
        self.action_dim: int | None = None
        self.observation_tactile_dim: int | None = None
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
        tactile_values: list[list[float]] = []
        obs_eef_left_values: list[list[float]] = []
        obs_eef_right_values: list[list[float]] = []
        action_eef_left_values: list[list[float]] = []
        action_eef_right_values: list[list[float]] = []
        obs_gripper_left_values: list[list[float]] = []
        obs_gripper_right_values: list[list[float]] = []
        action_gripper_left_values: list[list[float]] = []
        action_gripper_right_values: list[list[float]] = []
        rows: list[dict[str, Any]] = []
        image_paths: list[Path] = []
        global_start = dataset_cursor

        for frame_index, frame in enumerate(frames[:-1]):
            next_frame = frames[frame_index + 1]
            state = build_state(frame)
            action = build_action(next_frame)
            tactile = flatten_tactile(frame, self.tactile_dim)
            if self.state_dim is None:
                self.state_dim = len(state)
            if self.action_dim is None:
                self.action_dim = len(action)
            if self.observation_tactile_dim is None:
                self.observation_tactile_dim = len(tactile)
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
            if len(tactile) != self.observation_tactile_dim:
                raise ValueError(
                    f"Tactile dim changed in {episode_dir}: "
                    f"{len(tactile)} != {self.observation_tactile_dim}"
                )

            states.append(state)
            actions.append(action)
            tactile_values.append(tactile)
            image_paths.append(image_path(episode_dir, frame, self.camera))
            row = {
                "states": state,
                "action": action,
                "observation.tactile": tactile,
                "timestamp": frame_index / float(self.fps),
                "frame_index": frame_index,
                "episode_index": episode_index,
                "index": global_start + frame_index,
                "task_index": task_index,
                "next.done": frame_index == len(frames) - 2,
            }
            if self.include_eef:
                assert self.eef_fk is not None
                obs_eef = self.eef_fk.wrist_poses(frame, "states")
                action_eef = self.eef_fk.wrist_poses(next_frame, "actions")
                row.update(
                    {
                        "observation.eef.left": obs_eef["left"],
                        "observation.eef.right": obs_eef["right"],
                        "action.eef.left": action_eef["left"],
                        "action.eef.right": action_eef["right"],
                    }
                )
                obs_eef_left_values.append(obs_eef["left"])
                obs_eef_right_values.append(obs_eef["right"])
                action_eef_left_values.append(action_eef["left"])
                action_eef_right_values.append(action_eef["right"])
            if self.include_gripper:
                obs_gripper = build_gripper(frame, "states")
                action_gripper = build_gripper(next_frame, "actions")
                row.update(
                    {
                        "observation.gripper.left": obs_gripper["left"],
                        "observation.gripper.right": obs_gripper["right"],
                        "action.gripper.left": action_gripper["left"],
                        "action.gripper.right": action_gripper["right"],
                    }
                )
                obs_gripper_left_values.append(obs_gripper["left"])
                obs_gripper_right_values.append(obs_gripper["right"])
                action_gripper_left_values.append(action_gripper["left"])
                action_gripper_right_values.append(action_gripper["right"])
            rows.append(row)

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
        stats = {
            "states": vector_stats(states),
            "action": vector_stats(actions),
            "observation.tactile": vector_stats(tactile_values),
        }
        if self.include_eef:
            stats.update(
                {
                    "observation.eef.left": vector_stats(obs_eef_left_values),
                    "observation.eef.right": vector_stats(obs_eef_right_values),
                    "action.eef.left": vector_stats(action_eef_left_values),
                    "action.eef.right": vector_stats(action_eef_right_values),
                }
            )
        if self.include_gripper:
            stats.update(
                {
                    "observation.gripper.left": vector_stats(
                        obs_gripper_left_values
                    ),
                    "observation.gripper.right": vector_stats(
                        obs_gripper_right_values
                    ),
                    "action.gripper.left": vector_stats(
                        action_gripper_left_values
                    ),
                    "action.gripper.right": vector_stats(
                        action_gripper_right_values
                    ),
                }
            )
        self.episode_stats.append(
            {"episode_index": episode_index, "stats": stats}
        )
        self.total_frames += len(rows)
        return len(rows)

    def write_meta(self, out_dir: Path) -> None:
        if (
            self.state_dim is None
            or self.action_dim is None
            or self.observation_tactile_dim is None
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
            "observation.tactile": {
                "dtype": "float32",
                "shape": [self.observation_tactile_dim],
                "names": [
                    f"tactile_{i}"
                    for i in range(self.observation_tactile_dim)
                ],
            },
            "timestamp": {"dtype": "float32", "shape": [1]},
            "frame_index": {"dtype": "int64", "shape": [1]},
            "episode_index": {"dtype": "int64", "shape": [1]},
            "index": {"dtype": "int64", "shape": [1]},
            "task_index": {"dtype": "int64", "shape": [1]},
            "next.done": {"dtype": "bool", "shape": [1]},
        }
        if self.include_eef:
            eef_names = ["x", "y", "z", "qx", "qy", "qz", "qw"]
            features_meta.update(
                {
                    "observation.eef.left": {
                        "dtype": "float32",
                        "shape": [EEF_POSE_DIM],
                        "names": eef_names,
                    },
                    "observation.eef.right": {
                        "dtype": "float32",
                        "shape": [EEF_POSE_DIM],
                        "names": eef_names,
                    },
                    "action.eef.left": {
                        "dtype": "float32",
                        "shape": [EEF_POSE_DIM],
                        "names": eef_names,
                    },
                    "action.eef.right": {
                        "dtype": "float32",
                        "shape": [EEF_POSE_DIM],
                        "names": eef_names,
                    },
                }
            )
        if self.include_gripper:
            gripper_feature = {
                "dtype": "float32",
                "shape": [GRIPPER_DIM],
                "names": ["open_to_closed"],
            }
            features_meta.update(
                {
                    "observation.gripper.left": gripper_feature,
                    "observation.gripper.right": gripper_feature,
                    "action.gripper.left": gripper_feature,
                    "action.gripper.right": gripper_feature,
                }
            )
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
- Tactile dimension: {self.observation_tactile_dim}
- EEF fields: {'enabled' if self.include_eef else 'disabled'}
- Gripper scalar fields: {'enabled' if self.include_gripper else 'disabled'}

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
    print(
        "  tactile_dim="
        f"{info['features']['observation.tactile']['shape'][0]}"
    )
    eef_keys = [
        key for key in info["features"]
        if key.startswith("observation.eef.") or key.startswith("action.eef.")
    ]
    if eef_keys:
        print(f"  eef_fields={sorted(eef_keys)}")
    gripper_keys = [
        key for key in info["features"]
        if key.startswith("observation.gripper.")
        or key.startswith("action.gripper.")
    ]
    if gripper_keys:
        print(f"  gripper_fields={sorted(gripper_keys)}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--data-root", type=Path, default=Path("xr_recordings")
    )
    parser.add_argument(
        "--out-dir", type=Path, default=Path("xr_recordings_lerobot")
    )
    parser.add_argument("--tasks", type=str, default=",".join(DEFAULT_TASKS))
    parser.add_argument("--camera", type=str, default="color_0")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--tactile-dim", type=int, default=DEFAULT_TACTILE_DIM)
    parser.add_argument("--chunks-size", type=int, default=1000)
    parser.add_argument(
        "--include-eef",
        action="store_true",
        help=(
            "Add wrist yaw link EEF pose fields computed from qpos: "
            "[x,y,z,qx,qy,qz,qw]."
        ),
    )
    parser.add_argument(
        "--include-gripper",
        action="store_true",
        help=(
            "Add smooth binary-gripper scalar fields computed from Dex3 qpos. "
            "0=open, 1=closed."
        ),
    )
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

    converter = XrToLeRobotConverter(
        args.camera,
        args.fps,
        args.chunks_size,
        args.tactile_dim,
        include_eef=args.include_eef,
        include_gripper=args.include_gripper,
    )
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
        try:
            create_tag(args.repo_id, tag=CODE_VERSION, repo_type="dataset")
        except HfHubHTTPError as exc:
            if "Tag reference exists already" not in str(exc):
                raise
        print(f"Uploaded to https://huggingface.co/datasets/{args.repo_id}")


if __name__ == "__main__":
    main()
