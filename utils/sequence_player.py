"""Play ordered teach trajectory snippets as one long-horizon task."""

from __future__ import annotations

import json
import os
import time
from typing import Any, Callable

from replay import TrajectoryPlayer, ensure_ai_mode
from teach import GravityCompensator, TeachRecorder
from teach_sequences import estimate_sequence_duration


LogFn = Callable[[str], None]


def play_task_sequence(
    snippet_paths: list[str],
    *,
    speed: float = 1.0,
    pause_sec: float = 0.5,
    clearance_between: bool = True,
    recorder: TeachRecorder | None = None,
    prepare_first: bool = True,
    log: LogFn | None = None,
) -> bool:
    """Replay snippets in order. Returns True on success."""
    _log = log or print
    paths = [p for p in snippet_paths if p and os.path.isfile(p)]
    if not paths:
        _log("ERROR: No valid snippet paths in sequence.")
        return False

    if not ensure_ai_mode(interactive=False):
        _log("ERROR: ai balance mode not active.")
        return False

    est = estimate_sequence_duration(paths, pause_sec)
    _log(
        f"Task sequence: {len(paths)} snippet(s), "
        f"~{est:.1f}s playback (+ transitions)"
    )

    grav = GravityCompensator()

    if recorder and prepare_first:
        _log("Initial forward prepare (once)...")
        if not recorder.wait_for_state(timeout=3.0):
            _log("ERROR: No robot state.")
            return False
        recorder.prepare_recording_pose(resume_compliant=False)

    for idx, path in enumerate(paths):
        with open(path) as f:
            traj_data = json.load(f)
        meta = traj_data.get("metadata", {})
        label = meta.get("name") or os.path.basename(path)
        _log(
            f"--- Snippet {idx + 1}/{len(paths)}: {label} "
            f"({meta.get('duration_s', '?')}s) ---"
        )

        player = TrajectoryPlayer(traj_data, speed=speed, grav_comp=grav)
        if recorder:
            player.init(recorder)
        else:
            player.init()
        if not player.wait_for_state():
            _log(f"ERROR: No state before snippet {idx + 1}.")
            return False

        player.play()
        if player._stop:
            _log("Playback stopped early.")
            return False

        if idx < len(paths) - 1:
            _log(f"Segment done — pause {pause_sec:.1f}s")
            time.sleep(max(0.0, pause_sec))
            if clearance_between and recorder:
                _log("Outward clearance between snippets...")
                if recorder.low_state:
                    recorder.park_after_recording_pose()

    _log("Task sequence finished.")
    return True
