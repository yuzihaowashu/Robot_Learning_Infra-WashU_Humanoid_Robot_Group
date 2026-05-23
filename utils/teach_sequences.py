"""High-level goals with numbered steps (record + replay)."""

from __future__ import annotations

import json
import os
import time
from typing import Any

from teach_catalog import slugify_name

DEFAULT_MAX_STEPS = 12
DEFAULT_PAUSE_SEC = 0.5


def _now() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S")


def task_output_path(tasks_dir: str, goal_name: str) -> str:
    slug = slugify_name(goal_name)
    return os.path.join(tasks_dir, f"task_{slug}.json")


def trajectory_path_for_step(
    traj_dir: str,
    goal_name: str,
    step_num: int,
) -> str:
    """Path for a new recording assigned to goal step N."""
    slug = slugify_name(goal_name)
    ts = time.strftime("%Y%m%d_%H%M%S")
    fname = f"{slug}_step{step_num}_{ts}.json"
    return os.path.join(traj_dir, fname)


def normalize_task_data(data: dict[str, Any]) -> dict[str, Any]:
    """Ensure steps dict exists; migrate legacy snippets list."""
    if "steps" not in data and data.get("snippets"):
        steps: dict[str, Any] = {}
        for i, sn in enumerate(data["snippets"], 1):
            steps[str(i)] = {
                "path": sn.get("path", ""),
                "label": sn.get("name", f"step {i}"),
                "recorded_at": data.get("updated", ""),
            }
        data["steps"] = steps
    if "steps" not in data:
        data["steps"] = {}
    return data


def create_or_load_goal(
    tasks_dir: str,
    goal_name: str,
    max_steps: int = DEFAULT_MAX_STEPS,
) -> dict[str, Any]:
    """Create task file if missing; return full task dict."""
    os.makedirs(tasks_dir, exist_ok=True)
    name = goal_name.strip()
    path = task_output_path(tasks_dir, name)
    if os.path.isfile(path):
        with open(path) as f:
            data = normalize_task_data(json.load(f))
        data["name"] = name
        data["path"] = path
        return data

    data = {
        "name": name,
        "path": path,
        "max_steps": int(max_steps),
        "pause_sec": DEFAULT_PAUSE_SEC,
        "clearance_between": True,
        "steps": {},
        "created": _now(),
        "updated": _now(),
    }
    save_task_data(data)
    return data


def load_task(path: str) -> dict[str, Any]:
    with open(path) as f:
        data = normalize_task_data(json.load(f))
    data["path"] = path
    return data


def save_task_data(data: dict[str, Any]) -> str:
    path = data.get("path") or task_output_path(
        os.path.dirname(data.get("_tasks_dir", "tasks")),
        data["name"],
    )
    data["updated"] = _now()
    payload = {
        "name": data["name"],
        "max_steps": data.get("max_steps", DEFAULT_MAX_STEPS),
        "pause_sec": float(data.get("pause_sec", DEFAULT_PAUSE_SEC)),
        "clearance_between": bool(data.get("clearance_between", True)),
        "steps": data.get("steps") or {},
        "created": data.get("created", _now()),
        "updated": data["updated"],
    }
    with open(path, "w") as f:
        json.dump(payload, f, indent=2)
    data["path"] = path
    return path


def assign_step_recording(
    task_data: dict[str, Any],
    step_num: int,
    traj_path: str,
    label: str = "",
) -> None:
    steps = task_data.setdefault("steps", {})
    key = str(int(step_num))
    entry = steps.get(key) or {}
    entry["path"] = os.path.abspath(traj_path)
    entry["label"] = (label or "").strip() or f"step {step_num}"
    entry["recorded_at"] = _now()
    steps[key] = entry
    save_task_data(task_data)


def clear_step(task_data: dict[str, Any], step_num: int) -> None:
    steps = task_data.get("steps") or {}
    key = str(int(step_num))
    if key in steps:
        del steps[key]
        save_task_data(task_data)


def step_paths_ordered(
    task_data: dict[str, Any],
    traj_dir: str,
) -> list[str]:
    """Recorded step paths in order 1..max_steps (skip empty)."""
    steps = task_data.get("steps") or {}
    max_n = int(task_data.get("max_steps", DEFAULT_MAX_STEPS))
    paths: list[str] = []
    for n in range(1, max_n + 1):
        entry = steps.get(str(n))
        if not entry:
            continue
        p = entry.get("path") or ""
        if not p:
            continue
        if not os.path.isabs(p):
            p = os.path.join(traj_dir, os.path.basename(p))
        if os.path.isfile(p):
            paths.append(os.path.abspath(p))
    return paths


def count_recorded_steps(task_data: dict[str, Any], traj_dir: str = "") -> int:
    return len(step_paths_ordered(task_data, traj_dir))


def estimate_sequence_duration(
    snippet_paths: list[str],
    pause_sec: float,
) -> float:
    total = 0.0
    for p in snippet_paths:
        try:
            with open(p) as f:
                meta = json.load(f).get("metadata") or {}
            total += float(meta.get("duration_s", 0))
        except (OSError, json.JSONDecodeError, TypeError):
            pass
    if len(snippet_paths) > 1:
        total += pause_sec * (len(snippet_paths) - 1)
    return total


def format_goal_steps_table(
    task_data: dict[str, Any] | None,
    active_step: int | None = None,
) -> str:
    if not task_data:
        return "_Select or create a high-level goal to see steps._"
    name = task_data.get("name", "?")
    max_n = int(task_data.get("max_steps", DEFAULT_MAX_STEPS))
    steps = task_data.get("steps") or {}
    lines = [
        f"### Goal: **{name}**",
        "",
        "| Step | Status | Label | File |",
        "|------|--------|-------|------|",
    ]
    for n in range(1, max_n + 1):
        entry = steps.get(str(n)) or {}
        p = entry.get("path") or ""
        label = entry.get("label") or "—"
        if p and os.path.isfile(p if os.path.isabs(p) else p):
            status = "✅ recorded"
            fname = f"`{os.path.basename(p)}`"
        else:
            status = "⬜ empty"
            fname = "—"
        if active_step == n:
            status += " **← recording**"
        lines.append(f"| {n} | {status} | {label} | {fname} |")
    recorded = len([n for n in range(1, max_n + 1) if steps.get(str(n), {}).get("path")])
    lines.append("")
    lines.append(f"**{recorded}** / {max_n} steps recorded.")
    return "\n".join(lines)


def list_goals(tasks_dir: str, traj_dir: str = "") -> list[dict[str, Any]]:
    if not os.path.isdir(tasks_dir):
        return []
    out = []
    for fname in os.listdir(tasks_dir):
        if not fname.endswith(".json"):
            continue
        path = os.path.join(tasks_dir, fname)
        try:
            data = load_task(path)
            n_rec = count_recorded_steps(data, traj_dir or tasks_dir)
            out.append({
                "path": path,
                "file": fname,
                "name": data.get("name") or fname,
                "n_recorded": n_rec,
                "max_steps": data.get("max_steps", DEFAULT_MAX_STEPS),
                "pause_sec": data.get("pause_sec", DEFAULT_PAUSE_SEC),
                "clearance_between": data.get("clearance_between", True),
            })
        except (OSError, json.JSONDecodeError):
            continue
    out.sort(key=lambda e: os.path.getmtime(e["path"]), reverse=True)
    return out


def goal_choices_for_dropdown(
    tasks_dir: str,
    traj_dir: str = "",
) -> tuple[list[str], dict[str, str]]:
    labels: list[str] = []
    label_to_path: dict[str, str] = {}
    for e in list_goals(tasks_dir, traj_dir):
        label = (
            f"{e['name']}  ({e['n_recorded']}/{e['max_steps']} steps)"
        )
        base = label
        n = 2
        while label in label_to_path:
            label = f"{base} [{n}]"
            n += 1
        labels.append(label)
        label_to_path[label] = e["path"]
    return labels, label_to_path


# Legacy aliases for sequence_player
def resolve_snippet_paths(
    snippets: list[dict[str, Any]],
    traj_dir: str,
) -> list[str]:
    resolved = []
    for sn in snippets:
        p = sn.get("path") or ""
        if not p:
            continue
        if not os.path.isabs(p):
            p = os.path.join(traj_dir, os.path.basename(p))
        if os.path.isfile(p):
            resolved.append(os.path.abspath(p))
    return resolved


def step_paths_from_task_file(
    task_path: str,
    traj_dir: str,
) -> list[str]:
    return step_paths_ordered(load_task(task_path), traj_dir)
