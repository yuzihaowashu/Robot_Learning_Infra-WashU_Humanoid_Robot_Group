#!/usr/bin/env python3
"""Split one long raw XR episode into task episodes.

The splitter is intentionally conservative. By default it only prints the
candidate segments. Pass --write after checking the dry-run output.
"""

from __future__ import annotations

import argparse
import copy
import json
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_TASK_LIST = ROOT_DIR / "tasks" / "task_list.json"
DEFAULT_SPLITTER_LIST = ROOT_DIR / "tasks" / "splitter_list.json"
DEFAULT_FORWARD_SEGMENT_TYPE = "place_bottle_into_paper_box"
DEFAULT_BACKWARD_SEGMENT_TYPE = "take_bottle_out_of_paper_box"


@dataclass
class Segment:
    task_label: str
    task_name: str
    segment_type: str
    start: int
    end: int
    grasp_frame: int
    release_frame: int | None
    grasp_distance: float


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


def _qpos(frame: dict[str, Any], section: str, key: str) -> np.ndarray:
    value = frame.get(section, {}).get(key, {}).get("qpos", [])
    return np.asarray(value, dtype=float)


def _active_side(episode: dict[str, Any], frames: list[dict[str, Any]]) -> str:
    metadata = episode.get("info", {}).get("metadata", {})
    arm_mode = metadata.get("arm_mode") if isinstance(metadata, dict) else None
    if arm_mode == "left-only":
        return "left"
    if arm_mode == "right-only":
        return "right"

    # Bimanual fallback: use the hand with the larger close/open signal.
    scores: dict[str, float] = {}
    for side in ("left", "right"):
        vals = [
            _finger_signal(frame, side)
            for frame in frames
            if _qpos(frame, "actions", f"{side}_ee").size
        ]
        scores[side] = max(vals) - min(vals) if vals else 0.0
    return max(scores, key=scores.get)


def _finger_signal(frame: dict[str, Any], side: str) -> float:
    q = _qpos(frame, "actions", f"{side}_ee")
    if q.size == 0:
        return 0.0
    return float(np.mean(np.abs(q)))


def _smooth(values: list[float], window: int) -> list[float]:
    if window <= 1 or len(values) < window:
        return values
    pad = window // 2
    padded = np.pad(np.asarray(values, dtype=float), (pad, pad), mode="edge")
    kernel = np.ones(window, dtype=float) / window
    return np.convolve(padded, kernel, mode="valid").tolist()[:len(values)]


def _closed_states(
    signals: list[float],
    close_threshold: float,
    open_threshold: float,
) -> list[bool]:
    states: list[bool] = []
    closed = False
    for value in signals:
        if closed:
            if value <= open_threshold:
                closed = False
        elif value >= close_threshold:
            closed = True
        states.append(closed)
    return states


def _transitions(states: list[bool]) -> tuple[list[int], list[int]]:
    open_to_close: list[int] = []
    close_to_open: list[int] = []
    for idx in range(1, len(states)):
        if not states[idx - 1] and states[idx]:
            open_to_close.append(idx)
        elif states[idx - 1] and not states[idx]:
            close_to_open.append(idx)
    return open_to_close, close_to_open


def _load_tasks(
    path: Path,
) -> tuple[dict[str, dict[str, Any]], dict[str, Any]]:
    with path.open("r", encoding="utf-8") as f:
        cfg = json.load(f)

    by_type: dict[str, dict[str, Any]] = {}
    for task in cfg.get("tasks", []):
        splitter = task.get("splitter", {})
        if not splitter.get("enabled"):
            continue
        segment_type = splitter.get("segment_type")
        if segment_type:
            by_type[segment_type] = task
    return by_type, cfg.get("splitter_config", {})


def _load_splitter(path: Path, splitter_id: str | None) -> dict[str, Any]:
    if not splitter_id:
        return {}
    with path.open("r", encoding="utf-8") as f:
        cfg = json.load(f)

    for splitter in cfg.get("splitters", []):
        if splitter.get("id") != splitter_id:
            continue
        if not splitter.get("enabled"):
            return {"enabled": False}
        directions = splitter.get("directions", {})
        forward = directions.get("forward", {})
        backward = directions.get("backward", {})
        selected_cfg = dict(splitter.get("config", {}))
        selected_cfg["forward_segment_type"] = forward.get(
            "segment_type",
            DEFAULT_FORWARD_SEGMENT_TYPE,
        )
        selected_cfg["backward_segment_type"] = backward.get(
            "segment_type",
            DEFAULT_BACKWARD_SEGMENT_TYPE,
        )
        selected_cfg["delete_original_on_success"] = bool(
            splitter.get("delete_original_on_success", False)
        )
        selected_cfg["require_all_directions_before_delete"] = bool(
            splitter.get("require_all_directions_before_delete", False)
        )
        selected_cfg["splitter_id"] = splitter_id
        selected_cfg["splitter_label"] = splitter.get("label", splitter_id)
        return selected_cfg

    raise ValueError(f"Splitter id {splitter_id!r} not found in {path}")


def _find_initial_q(
    frames: list[dict[str, Any]],
    side: str,
    grasp_events: list[int],
    initial_frame: int | None,
) -> tuple[np.ndarray, int]:
    if initial_frame is not None:
        idx = int(np.clip(initial_frame, 0, len(frames) - 1))
    elif grasp_events:
        idx = grasp_events[0]
    else:
        idx = 0
    q = _qpos(frames[idx], "states", f"{side}_arm")
    if q.size == 0:
        raise ValueError(f"Missing states.{side}_arm.qpos at frame {idx}")
    return q, idx


def _next_release_after(release_events: list[int], start: int) -> int | None:
    for release in release_events:
        if release > start:
            return release
    return None


def _near_initial(
    frame: dict[str, Any],
    side: str,
    initial_q: np.ndarray,
    threshold: float,
) -> tuple[bool, float]:
    q = _qpos(frame, "states", f"{side}_arm")
    if q.size != initial_q.size:
        return False, float("inf")
    dist = float(np.linalg.norm(q - initial_q))
    return dist <= threshold, dist


def _splitter_phase(frame: dict[str, Any]) -> str | None:
    splitter = frame.get("states", {}).get("splitter", {})
    phase = splitter.get("phase") if isinstance(splitter, dict) else None
    if phase in ("forward", "backward"):
        return phase
    return None


def _segments_from_phase_markers(
    frames: list[dict[str, Any]],
    tasks_by_type: dict[str, dict[str, Any]],
    *,
    forward_segment_type: str,
    backward_segment_type: str,
    min_segment_frames: int,
) -> tuple[list[Segment], dict[str, Any]] | None:
    phases = [_splitter_phase(frame) for frame in frames]
    if not any(phases):
        return None

    phase_to_type = {
        "forward": forward_segment_type,
        "backward": backward_segment_type,
    }
    segments: list[Segment] = []
    phase_ranges: list[tuple[str, int, int]] = []
    start = 0
    current = phases[0] or "forward"
    for idx in range(1, len(frames)):
        phase = phases[idx] or current
        if phase == current:
            continue
        phase_ranges.append((current, start, idx - 1))
        current = phase
        start = idx
    phase_ranges.append((current, start, len(frames) - 1))

    for phase, start_idx, end_idx in phase_ranges:
        segment_type = phase_to_type.get(phase)
        task = tasks_by_type.get(segment_type) if segment_type else None
        if task is None:
            continue
        if end_idx - start_idx + 1 < min_segment_frames:
            continue
        splitter = task.get("splitter", {})
        segments.append(
            Segment(
                task_label=task["label"],
                task_name=task["name"],
                segment_type=splitter.get("segment_type", segment_type),
                start=start_idx,
                end=end_idx,
                grasp_frame=start_idx,
                release_frame=end_idx,
                grasp_distance=0.0,
            )
        )

    debug = {
        "active_side": "marker",
        "initial_frame": 0,
        "grasp_events": [segment.start for segment in segments],
        "release_events": [segment.end for segment in segments],
        "classification_mode": "marker_phase",
        "phase_ranges": phase_ranges,
    }
    return segments, debug


def detect_segments(
    episode: dict[str, Any],
    tasks_by_type: dict[str, dict[str, Any]],
    cfg: dict[str, Any],
    *,
    active_side: str | None = None,
    initial_frame: int | None = None,
) -> tuple[list[Segment], dict[str, Any]]:
    frames = episode.get("data", [])
    if not frames:
        raise ValueError("Episode has no frames")
    side = active_side or _active_side(episode, frames)

    close_threshold = float(cfg.get("finger_close_threshold", 0.35))
    open_threshold = float(
        cfg.get("finger_open_threshold", close_threshold * 0.5)
    )
    smooth_window = int(cfg.get("finger_smooth_window", 3))
    near_threshold = float(cfg.get("near_initial_joint_threshold_rad", 0.35))
    min_segment_frames = int(cfg.get("min_segment_frames", 30))
    pre_frames = int(cfg.get("pre_grasp_frames", 8))
    post_frames = int(cfg.get("post_release_frames", 8))
    forward_segment_type = cfg.get(
        "forward_segment_type",
        DEFAULT_FORWARD_SEGMENT_TYPE,
    )
    backward_segment_type = cfg.get(
        "backward_segment_type",
        DEFAULT_BACKWARD_SEGMENT_TYPE,
    )
    classification_mode = cfg.get("classification_mode", "position")
    duplicate_threshold = float(
        cfg.get("duplicate_grasp_distance_threshold_rad", 0.0)
    )
    min_grasp_gap = int(cfg.get("min_grasp_gap_frames", 0))

    marker_result = _segments_from_phase_markers(
        frames,
        tasks_by_type,
        forward_segment_type=forward_segment_type,
        backward_segment_type=backward_segment_type,
        min_segment_frames=min_segment_frames,
    )
    if marker_result is not None:
        return marker_result

    signals = [_finger_signal(frame, side) for frame in frames]
    signals = _smooth(signals, smooth_window)
    closed = _closed_states(signals, close_threshold, open_threshold)
    grasp_events, release_events = _transitions(closed)
    initial_q, initial_idx = _find_initial_q(
        frames, side, grasp_events, initial_frame
    )

    segments: list[Segment] = []
    accepted_grasps: list[tuple[int, np.ndarray]] = []
    for grasp_idx in grasp_events:
        is_near, distance = _near_initial(
            frames[grasp_idx], side, initial_q, near_threshold
        )
        grasp_q = _qpos(frames[grasp_idx], "states", f"{side}_arm")
        if accepted_grasps:
            prev_idx, prev_q = accepted_grasps[-1]
            close_in_time = (grasp_idx - prev_idx) < min_grasp_gap
            close_in_pose = False
            if prev_q.size == grasp_q.size:
                close_in_pose = (
                    float(np.linalg.norm(grasp_q - prev_q))
                    < duplicate_threshold
                )
            if close_in_time or close_in_pose:
                continue

        if classification_mode == "alternating":
            if len(accepted_grasps) % 2 == 0:
                segment_type = forward_segment_type
            else:
                segment_type = backward_segment_type
        elif is_near:
            segment_type = forward_segment_type
        else:
            segment_type = backward_segment_type

        task = tasks_by_type.get(segment_type)
        if task is None:
            continue

        release_idx = _next_release_after(release_events, grasp_idx)
        if release_idx is None:
            end = len(frames) - 1
        elif segment_type == backward_segment_type:
            end = release_idx
            for idx in range(release_idx, len(frames)):
                near_now, _ = _near_initial(
                    frames[idx], side, initial_q, near_threshold
                )
                if near_now:
                    end = idx
                    break
        else:
            end = release_idx

        start = max(0, grasp_idx - pre_frames)
        if segments and start <= segments[-1].end:
            start = segments[-1].end + 1
        end = min(len(frames) - 1, end + post_frames)
        if start > grasp_idx:
            continue
        if end - start + 1 < min_segment_frames:
            continue

        accepted_grasps.append((grasp_idx, grasp_q))
        splitter = task.get("splitter", {})
        segments.append(
            Segment(
                task_label=task["label"],
                task_name=task["name"],
                segment_type=splitter.get("segment_type", segment_type),
                start=start,
                end=end,
                grasp_frame=grasp_idx,
                release_frame=release_idx,
                grasp_distance=distance,
            )
        )

    debug = {
        "active_side": side,
        "initial_frame": initial_idx,
        "grasp_events": grasp_events,
        "release_events": release_events,
        "close_threshold": close_threshold,
        "open_threshold": open_threshold,
        "near_initial_joint_threshold_rad": near_threshold,
        "classification_mode": classification_mode,
    }
    return segments, debug


def _next_episode_dir(task_dir: Path) -> Path:
    task_dir.mkdir(parents=True, exist_ok=True)
    used = []
    for child in task_dir.iterdir():
        if child.is_dir() and child.name.startswith("episode_"):
            try:
                used.append(int(child.name.split("_", 1)[1]))
            except ValueError:
                pass
    next_idx = max(used, default=0) + 1
    return task_dir / f"episode_{next_idx:04d}"


def _source_group_dir(source_data: Path) -> str:
    source_episode = source_data.parent.name
    if source_episode.startswith("episode_"):
        suffix = source_episode.split("_", 1)[1]
        return f"raw_episode_{suffix}"
    return f"raw_{source_episode}"


def _copy_referenced_file(
    src_episode_dir: Path,
    dst_episode_dir: Path,
    rel: str,
):
    src = src_episode_dir / rel
    dst = dst_episode_dir / rel
    if not src.is_file():
        raise FileNotFoundError(f"Referenced file missing: {src}")
    dst.parent.mkdir(parents=True, exist_ok=True)
    if not dst.exists():
        shutil.copy2(src, dst)


def _write_segment(
    source_data: Path,
    episode: dict[str, Any],
    segment: Segment,
    output_root: Path,
    task: dict[str, Any],
) -> Path:
    src_episode_dir = source_data.parent
    dst_task_dir = output_root / segment.task_name / _source_group_dir(source_data)
    dst_episode_dir = _next_episode_dir(dst_task_dir)
    dst_episode_dir.mkdir(parents=True, exist_ok=False)

    new_episode = copy.deepcopy(episode)
    frames = copy.deepcopy(episode["data"][segment.start:segment.end + 1])
    for new_idx, frame in enumerate(frames):
        frame["idx"] = new_idx
        for section in ("colors", "depths"):
            refs = frame.get(section) or {}
            for rel_path in refs.values():
                if rel_path:
                    _copy_referenced_file(
                        src_episode_dir, dst_episode_dir, rel_path
                    )

    new_episode["data"] = frames
    new_episode["text"] = {
        "goal": task.get("goal", ""),
        "desc": task.get("desc", ""),
        "steps": task.get("steps", ""),
    }
    info = new_episode.setdefault("info", {})
    metadata = info.setdefault("metadata", {})
    metadata.update(
        {
            "split_from": str(source_data),
            "split_segment_type": segment.segment_type,
            "split_source_start": segment.start,
            "split_source_end": segment.end,
            "split_grasp_frame": segment.grasp_frame,
            "split_release_frame": segment.release_frame,
            "split_grasp_distance": segment.grasp_distance,
        }
    )

    with (dst_episode_dir / "data.json").open("w", encoding="utf-8") as f:
        json.dump(new_episode, f, indent=2)
    return dst_episode_dir


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Split a long raw XR episode into shorter task episodes."
    )
    parser.add_argument("episode", type=Path)
    parser.add_argument("--task-list", type=Path, default=DEFAULT_TASK_LIST)
    parser.add_argument(
        "--splitter-list",
        type=Path,
        default=DEFAULT_SPLITTER_LIST,
    )
    parser.add_argument("--splitter-id", default=None)
    parser.add_argument("--output-root", type=Path, default=None)
    parser.add_argument(
        "--active-side",
        choices=["left", "right"],
        default=None,
    )
    parser.add_argument("--initial-frame", type=int, default=None)
    parser.add_argument("--write", action="store_true")
    parser.add_argument("--delete-source", action="store_true")
    args = parser.parse_args()

    data_path = _resolve_data_json(args.episode.expanduser().resolve())
    with data_path.open("r", encoding="utf-8") as f:
        episode = json.load(f)
    tasks_by_type, cfg = _load_tasks(args.task_list.expanduser().resolve())
    splitter_cfg = _load_splitter(
        args.splitter_list.expanduser().resolve(),
        args.splitter_id,
    )
    if splitter_cfg.get("enabled") is False:
        print(f"Splitter {args.splitter_id!r} is disabled.")
        return 1
    cfg.update(splitter_cfg)
    segments, debug = detect_segments(
        episode,
        tasks_by_type,
        cfg,
        active_side=args.active_side,
        initial_frame=args.initial_frame,
    )

    print(f"Source: {data_path}")
    print(f"Frames: {len(episode.get('data', []))}")
    if cfg.get("splitter_label"):
        print(f"Splitter: {cfg['splitter_label']}")
    print(
        "Detection: "
        f"side={debug['active_side']} "
        f"initial_frame={debug['initial_frame']} "
        f"grasps={debug['grasp_events']} "
        f"releases={debug['release_events']}"
    )
    if not segments:
        print("No valid segments detected.")
        return 1

    for idx, segment in enumerate(segments, start=1):
        print(
            f"[{idx}] {segment.task_name}: "
            f"frames {segment.start}-{segment.end} "
            f"grasp={segment.grasp_frame} release={segment.release_frame} "
            f"dist={segment.grasp_distance:.3f} "
            f"type={segment.segment_type}"
        )

    if not args.write:
        print("Dry run only. Re-run with --write to create split episodes.")
        return 0

    output_root = args.output_root
    if output_root is None:
        output_root = data_path.parent.parent.parent
    output_root = output_root.expanduser().resolve()

    delete_source = args.delete_source or bool(
        cfg.get("delete_original_on_success", False)
    )
    if delete_source and cfg.get("require_all_directions_before_delete"):
        detected_types = {segment.segment_type for segment in segments}
        required_types = {
            cfg.get("forward_segment_type", DEFAULT_FORWARD_SEGMENT_TYPE),
            cfg.get("backward_segment_type", DEFAULT_BACKWARD_SEGMENT_TYPE),
        }
        if not required_types.issubset(detected_types):
            raise RuntimeError(
                "Refusing to write/delete because split output did not "
                f"include all directions: {sorted(detected_types)}"
            )

    written: list[Path] = []
    task_by_name = {task["name"]: task for task in tasks_by_type.values()}
    for segment in segments:
        written.append(
            _write_segment(
                data_path,
                episode,
                segment,
                output_root,
                task_by_name[segment.task_name],
            )
        )
    print("Wrote:")
    for path in written:
        print(f"  {path}")

    if delete_source:
        source_dir = data_path.parent.resolve()
        written_dirs = {path.resolve() for path in written}
        if source_dir in written_dirs:
            raise RuntimeError(
                "Refusing to delete source because it is also an output dir"
            )
        shutil.rmtree(source_dir)
        print(f"Deleted source: {source_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
