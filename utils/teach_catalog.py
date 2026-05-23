"""Trajectory catalog: human-readable names mapped to JSON files."""

from __future__ import annotations

import json
import os
import re
import time
from typing import Any


def slugify_name(name: str, max_len: int = 48) -> str:
    """Turn a short description into a filesystem-safe slug."""
    slug = name.strip().lower()
    slug = re.sub(r"[^\w\s-]", "", slug, flags=re.UNICODE)
    slug = re.sub(r"[\s_-]+", "_", slug).strip("_")
    if not slug:
        slug = "motion"
    return slug[:max_len]


def trajectory_output_path(traj_dir: str, name: str) -> str:
    """Build a timestamped path under traj_dir for a named recording."""
    ts = time.strftime("%Y%m%d_%H%M%S")
    slug = slugify_name(name)
    return os.path.join(traj_dir, f"traj_{slug}_{ts}.json")


def _read_metadata(path: str) -> dict[str, Any]:
    try:
        with open(path) as f:
            data = json.load(f)
        return data.get("metadata") or {}
    except (OSError, json.JSONDecodeError, KeyError):
        return {}


def list_trajectories(traj_dir: str) -> list[dict[str, Any]]:
    """Return trajectory entries sorted newest first."""
    if not os.path.isdir(traj_dir):
        return []

    entries: list[dict[str, Any]] = []
    for fname in os.listdir(traj_dir):
        if not fname.endswith(".json"):
            continue
        path = os.path.join(traj_dir, fname)
        if not os.path.isfile(path):
            continue
        meta = _read_metadata(path)
        name = meta.get("name") or meta.get("description") or ""
        if not name:
            name = fname.replace(".json", "").replace("traj_", "", 1)
        entries.append({
            "path": path,
            "file": fname,
            "name": name,
            "n_frames": meta.get("n_frames", "?"),
            "duration_s": meta.get("duration_s", "?"),
            "created": meta.get("created", ""),
        })

    entries.sort(key=lambda e: os.path.getmtime(e["path"]), reverse=True)
    return entries


def format_choice_label(entry: dict[str, Any]) -> str:
    """Dropdown label: name — duration, date."""
    dur = entry.get("duration_s", "?")
    if isinstance(dur, (int, float)):
        dur = f"{dur:.1f}s"
    created = entry.get("created") or ""
    if created:
        return f"{entry['name']}  ({dur}, {created})"
    return f"{entry['name']}  ({dur})"


def choices_for_dropdown(traj_dir: str) -> tuple[list[str], dict[str, str]]:
    """Return (labels, label_to_path) for a Gradio Dropdown."""
    entries = list_trajectories(traj_dir)
    labels: list[str] = []
    label_to_path: dict[str, str] = {}
    for entry in entries:
        label = format_choice_label(entry)
        base = label
        n = 2
        while label in label_to_path:
            label = f"{base} [{n}]"
            n += 1
        labels.append(label)
        label_to_path[label] = entry["path"]
    return labels, label_to_path


def format_library_table(traj_dir: str) -> str:
    """Markdown table of saved trajectories."""
    entries = list_trajectories(traj_dir)
    if not entries:
        return "_No trajectories recorded yet._"

    lines = [
        "| Name | Duration | Frames | File |",
        "|------|----------|--------|------|",
    ]
    for e in entries:
        dur = e["duration_s"]
        if isinstance(dur, (int, float)):
            dur = f"{dur:.1f}s"
        lines.append(
            f"| {e['name']} | {dur} | {e['n_frames']} | `{e['file']}` |"
        )
    return "\n".join(lines)
