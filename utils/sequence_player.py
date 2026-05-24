"""Play ordered teach trajectory snippets as one long-horizon task."""

from __future__ import annotations

import json
import os
import time
from typing import Callable

from replay import CONTROL_DT, TrajectoryPlayer, ensure_ai_mode
from teach import GravityCompensator, TeachRecorder
from teach_sequences import estimate_sequence_duration


LogFn = Callable[[str], None]


def _hold_between_snippets(
    player: TrajectoryPlayer,
    last_arm: dict,
    last_hand: dict | None,
    duration: float,
) -> None:
    """Keep arm_sdk weight=1.0 at last frame so arms don't drop to downward
    between snippets. The outward 'spread clearance' that used to run here
    is unsafe between recorded steps (arms swing wide near the body), so it
    is intentionally NOT used during execute. Snippets are designed to
    connect end-to-end at forward; the next snippet's fast handoff picks
    up directly from last_arm.
    """
    if duration <= 0:
        return
    steps = max(1, int(duration / CONTROL_DT))
    for _ in range(steps):
        player._send_arm_cmd(
            last_arm, weight=1.0, kp_scale=0.95, kd_scale=0.95,
        )
        if last_hand is not None:
            player._send_hand_cmd(last_hand)
        time.sleep(CONTROL_DT)


def play_task_sequence(
    snippet_paths: list[str],
    *,
    speed: float = 1.0,
    pause_sec: float = 0.5,
    clearance_between: bool = False,
    recorder: TeachRecorder | None = None,
    prepare_first: bool = False,
    return_to_forward_at_end: bool = True,
    log: LogFn | None = None,
) -> bool:
    """Replay snippets in order. Returns True on success.

    clearance_between: DEPRECATED for execute_goal. When True, runs an
        outward spread between snippets — this was a holdover from the
        park-after-recording flow and is unsafe / wastes time during
        normal multi-step execute. Default is now False.
    return_to_forward_at_end: When a recorder is attached, ramp arms back
        to the default forward pose after the last snippet so subsequent
        recording / replay starts from a known safe pose.
    """
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
        _log(
            "WARNING: prepare_first runs full teach prepare — "
            "usually leave this False for replay."
        )
        if not recorder.wait_for_state(timeout=3.0):
            _log("ERROR: No robot state.")
            return False
        recorder.prepare_recording_pose(resume_compliant=False)

    for idx, path in enumerate(paths):
        if recorder:
            if (
                recorder._session_thread
                and recorder._session_thread.is_alive()
            ):
                recorder.suspend_compliant_session()

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

        is_last = (idx == len(paths) - 1)
        # Always hold at last frame across the snippet boundary; the
        # standalone-final release path is only used when no recorder is
        # attached AND we're not returning to forward (e.g. pure CLI replay).
        will_return_forward = bool(
            is_last and recorder is not None and return_to_forward_at_end
        )
        release_at_end = is_last and (recorder is None) and (
            not will_return_forward
        )
        player.play(release_at_end=release_at_end)
        if player._stop:
            _log("Playback stopped early.")
            return False

        if not is_last:
            last_arm = player._sample_arm(player.duration)
            last_hand = player._sample_hand(player.duration)
            _log(
                f"Segment done — holding last frame for "
                f"{pause_sec:.1f}s before next snippet"
            )
            _hold_between_snippets(
                player, last_arm, last_hand, max(0.0, pause_sec),
            )

    if (
        return_to_forward_at_end
        and recorder is not None
        and recorder.low_state
    ):
        _log("Returning to default forward pose...")
        try:
            recorder.hold_forward_between_steps(enable_drag=False)
        except Exception as e:
            _log(f"Return-to-forward warning: {e}")

    _log("Task sequence finished.")
    return True
