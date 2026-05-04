#!/usr/bin/env python3
"""Replay raw XR arm trajectories on G1.

Default mode is dry-run only. Add --execute to publish real robot commands.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT_DIR / "xr_teleoperate"))

ARM_DOF = 14
LEFT = slice(0, 7)
RIGHT = slice(7, 14)
SPREAD_Q = np.zeros(ARM_DOF)
SPREAD_Q[1] = 1.5
SPREAD_Q[8] = -1.5
FORWARD_Q = np.zeros(ARM_DOF)


def _load_episode(path: Path) -> tuple[Path, dict, list[dict]]:
    path = path.expanduser().resolve()
    data_path = path if path.is_file() else path / "data.json"
    if not data_path.is_file():
        raise FileNotFoundError(f"Missing data.json: {data_path}")
    with data_path.open("r", encoding="utf-8") as f:
        episode = json.load(f)
    frames = episode.get("data", [])
    if not frames:
        raise ValueError(f"No frames in {data_path}")
    return data_path, episode, frames


def _qpos(frame: dict, section: str, side: str) -> np.ndarray:
    values = (
        frame.get(section, {})
        .get(f"{side}_arm", {})
        .get("qpos", [])
    )
    arr = np.asarray(values, dtype=np.float64)
    if arr.shape != (7,):
        raise ValueError(
            f"Invalid {section}.{side}_arm.qpos shape: {arr.shape}"
        )
    return arr


def _ee_qpos(frame: dict, section: str, side: str) -> np.ndarray:
    values = (
        frame.get(section, {})
        .get(f"{side}_ee", {})
        .get("qpos", [])
    )
    arr = np.asarray(values, dtype=np.float64)
    if arr.size == 0:
        return np.zeros(7, dtype=np.float64)
    if arr.shape != (7,):
        raise ValueError(f"Invalid {section}.{side}_ee.qpos shape: {arr.shape}")
    return arr


def _build_targets(
    frames: list[dict],
    arm_mode: str,
    source: str,
) -> np.ndarray:
    targets = []
    for frame in frames:
        left_state = _qpos(frame, "states", "left")
        right_state = _qpos(frame, "states", "right")
        left_action = _qpos(frame, "actions", "left")
        right_action = _qpos(frame, "actions", "right")

        if source == "states":
            target = np.concatenate([left_state, right_state])
        elif arm_mode == "left-only":
            target = np.concatenate([left_action, right_state])
        elif arm_mode == "right-only":
            target = np.concatenate([left_state, right_action])
        else:
            target = np.concatenate([left_action, right_action])
        targets.append(target)

    return np.asarray(targets, dtype=np.float64)


def _build_hand_targets(frames: list[dict], source: str) -> np.ndarray:
    targets = []
    for frame in frames:
        left = _ee_qpos(frame, source, "left")
        right = _ee_qpos(frame, source, "right")
        targets.append(np.concatenate([left, right]))
    return np.asarray(targets, dtype=np.float64)


def _summarize(targets: np.ndarray, fps: float) -> None:
    span = targets.max(axis=0) - targets.min(axis=0)
    step = (
        np.abs(np.diff(targets, axis=0))
        if len(targets) > 1
        else targets * 0
    )
    max_step = step.max(axis=0) if len(targets) > 1 else np.zeros(ARM_DOF)
    max_vel = max_step * fps

    print(f"frames: {len(targets)}")
    print(f"duration_s: {len(targets) / fps:.2f}")
    print("joint_span_rad:", np.round(span, 4).tolist())
    print("max_step_rad:", np.round(max_step, 4).tolist())
    print("max_vel_rad_s:", np.round(max_vel, 4).tolist())
    print("first_q:", np.round(targets[0], 4).tolist())
    print("last_q:", np.round(targets[-1], 4).tolist())


def _summarize_hands(hand_targets: np.ndarray) -> None:
    span = hand_targets.max(axis=0) - hand_targets.min(axis=0)
    print("hand_span:", np.round(span, 4).tolist())
    print("hand_first:", np.round(hand_targets[0], 4).tolist())
    print("hand_last:", np.round(hand_targets[-1], 4).tolist())


def _check_limits(
    targets: np.ndarray,
    fps: float,
    max_step_rad: float,
    max_vel_rad_s: float,
) -> None:
    if len(targets) < 2:
        return
    step = np.abs(np.diff(targets, axis=0))
    if float(step.max()) > max_step_rad:
        raise ValueError(
            f"Trajectory jump too large: {step.max():.3f} rad/frame "
            f"> {max_step_rad:.3f}"
        )
    vel = step * fps
    if float(vel.max()) > max_vel_rad_s:
        raise ValueError(
            f"Trajectory velocity too large: {vel.max():.3f} rad/s "
            f"> {max_vel_rad_s:.3f}"
        )


def _sleep_until_next(start_time: float, frame_idx: int, fps: float) -> None:
    target_time = start_time + (frame_idx + 1) / fps
    sleep_time = target_time - time.time()
    if sleep_time > 0:
        time.sleep(sleep_time)


def _make_hand_mode(motor_id: int) -> int:
    return (motor_id & 0x0F) | ((0x01 & 0x07) << 4)


def _make_hand_command(hand_cmd_factory, qpos: np.ndarray, kp: float, kd: float):
    cmd = hand_cmd_factory()
    for i in range(7):
        cmd.motor_cmd[i].mode = _make_hand_mode(i)
        cmd.motor_cmd[i].q = float(qpos[i])
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].tau = 0.0
        cmd.motor_cmd[i].kp = kp
        cmd.motor_cmd[i].kd = kd
    return cmd


def _interpolate_to_start(
    arm_ctrl,
    target_q: np.ndarray,
    seconds: float,
    label: str,
) -> None:
    current_q = arm_ctrl.get_current_dual_arm_q()
    print(f"Moving to {label} in {seconds:.1f}s")
    steps = max(1, int(seconds * 100))
    for i in range(steps):
        alpha = (i + 1) / steps
        alpha = alpha * alpha * (3.0 - 2.0 * alpha)
        q = (1.0 - alpha) * current_q + alpha * target_q
        arm_ctrl.ctrl_dual_arm(q, np.zeros(ARM_DOF))
        time.sleep(0.01)


def _prepare_start_waypoints(
    arm_mode: str,
    first_q: np.ndarray,
    args: argparse.Namespace,
) -> list[tuple[str, np.ndarray, float]]:
    if arm_mode == "left-only":
        clearance_q = first_q.copy()
        clearance_q[LEFT] = SPREAD_Q[LEFT]
        forward_q = first_q.copy()
        forward_q[LEFT] = FORWARD_Q[LEFT]
    elif arm_mode == "right-only":
        clearance_q = first_q.copy()
        clearance_q[RIGHT] = SPREAD_Q[RIGHT]
        forward_q = first_q.copy()
        forward_q[RIGHT] = FORWARD_Q[RIGHT]
    else:
        clearance_q = SPREAD_Q.copy()
        forward_q = FORWARD_Q.copy()

    return [
        ("outward clearance waypoint", clearance_q, args.prepare_clearance_sec),
        ("q=0 forward/default pose", forward_q, args.prepare_forward_sec),
        ("first replay frame", first_q, args.prepare_first_sec),
    ]


def _execute(
    targets: np.ndarray,
    hand_targets: np.ndarray,
    args: argparse.Namespace,
    arm_mode: str,
) -> None:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.core.channel import ChannelPublisher
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_
    from teleop.robot_control.robot_arm import (
        G1_29_ArmController,
        _clear_yield_flag,
    )
    from teleop.robot_control.robot_hand_unitree import (
        dex3_open_hands,
        dex3_release_hands,
        kTopicDex3LeftCommand,
        kTopicDex3RightCommand,
    )

    ChannelFactoryInitialize(0, networkInterface=args.network_interface)
    if not args.skip_open_hands:
        print("Opening Dex3 hands before replay...")
        dex3_open_hands(duration=args.open_hands_sec)

    hand_left_pub = None
    hand_right_pub = None
    if not args.skip_hands:
        hand_left_pub = ChannelPublisher(kTopicDex3LeftCommand, HandCmd_)
        hand_left_pub.Init()
        hand_right_pub = ChannelPublisher(kTopicDex3RightCommand, HandCmd_)
        hand_right_pub.Init()

    arm_ctrl = G1_29_ArmController(
        motion_mode=True,
        simulation_mode=False,
        safe_deploy=False,
        wrist_kp=args.wrist_kp,
        wrist_kd=args.wrist_kd,
    )
    arm_ctrl.speed_gradual_max()

    try:
        if args.prepare_start:
            for label, target_q, seconds in _prepare_start_waypoints(
                arm_mode,
                targets[0],
                args,
            ):
                _interpolate_to_start(arm_ctrl, target_q, seconds, label)
        else:
            _interpolate_to_start(
                arm_ctrl,
                targets[0],
                args.ramp_sec,
                "first replay frame",
            )
        print("Replaying arm trajectory...")
        start_time = time.time()
        for i, target in enumerate(targets):
            arm_ctrl.ctrl_dual_arm(target, np.zeros(ARM_DOF))
            if hand_left_pub is not None and hand_right_pub is not None:
                hand_q = hand_targets[i]
                hand_left_pub.Write(
                    _make_hand_command(
                        unitree_hg_msg_dds__HandCmd_,
                        hand_q[:7],
                        args.hand_kp,
                        args.hand_kd,
                    )
                )
                hand_right_pub.Write(
                    _make_hand_command(
                        unitree_hg_msg_dds__HandCmd_,
                        hand_q[7:],
                        args.hand_kp,
                        args.hand_kd,
                    )
                )
            _sleep_until_next(start_time, i, args.fps)

        print(f"Holding final pose for {args.hold_final_sec}s")
        end_time = time.time() + args.hold_final_sec
        while time.time() < end_time:
            arm_ctrl.ctrl_dual_arm(targets[-1], np.zeros(ARM_DOF))
            time.sleep(0.01)

        if args.relax_after:
            print("Replay finished. Moving outward stretch, then relaxing...")
            arm_ctrl.ctrl_dual_arm_go_home(
                lower_to_zero=True,
                keep_holder_yield=True,
                prepare_hands=False,
                skip_zero_waypoint=True,
                park_q=SPREAD_Q,
                spread_min_duration=args.relax_stretch_sec,
                spread_timeout=args.relax_stretch_sec + 1.5,
                spread_settle=False,
            )
            dex3_release_hands(duration=args.release_hands_sec)
    finally:
        if not args.keep_holder_yield:
            _clear_yield_flag()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Dry-run or execute raw XR arm trajectory replay."
    )
    parser.add_argument("episode", type=Path)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--source",
        choices=["states", "actions"],
        default="states",
        help=(
            "states replays the measured robot trajectory; actions replays "
            "the command targets saved during teleop."
        ),
    )
    parser.add_argument("--network-interface", default=None)
    parser.add_argument("--ramp-sec", type=float, default=3.0)
    parser.add_argument("--prepare-clearance-sec", type=float, default=2.0)
    parser.add_argument("--prepare-forward-sec", type=float, default=4.0)
    parser.add_argument("--prepare-first-sec", type=float, default=2.0)
    parser.add_argument(
        "--no-prepare-start",
        dest="prepare_start",
        action="store_false",
        help=(
            "Skip outward-clearance and q=0 forward/default preparation; "
            "interpolate directly to the first replay frame."
        ),
    )
    parser.set_defaults(prepare_start=True)
    parser.add_argument("--hold-final-sec", type=float, default=1.0)
    parser.add_argument(
        "--no-relax-after",
        dest="relax_after",
        action="store_false",
        help="Do not move outward stretch and relax after replay.",
    )
    parser.set_defaults(relax_after=True)
    parser.add_argument("--relax-stretch-sec", type=float, default=3.0)
    parser.add_argument("--keep-holder-yield", action="store_true")
    parser.add_argument("--skip-hands", action="store_true")
    parser.add_argument("--skip-open-hands", action="store_true")
    parser.add_argument("--open-hands-sec", type=float, default=1.0)
    parser.add_argument("--release-hands-sec", type=float, default=1.0)
    parser.add_argument("--hand-kp", type=float, default=1.0)
    parser.add_argument("--hand-kd", type=float, default=0.3)
    parser.add_argument("--max-step-rad", type=float, default=0.35)
    parser.add_argument("--max-vel-rad-s", type=float, default=12.0)
    parser.add_argument("--wrist-kp", type=float, default=60.0)
    parser.add_argument("--wrist-kd", type=float, default=2.0)
    args = parser.parse_args()

    data_path, episode, frames = _load_episode(args.episode)
    info = episode.get("info", {})
    metadata = info.get("metadata", {}) if isinstance(info, dict) else {}
    arm_mode = metadata.get("arm_mode", "bimanual")
    targets = _build_targets(frames, arm_mode, args.source)
    hand_targets = _build_hand_targets(frames, "actions")

    print(f"episode: {data_path}")
    print(f"arm_mode: {arm_mode}")
    print(f"source: {args.source}")
    print(f"execute: {args.execute}")
    print(f"prepare_start: {args.prepare_start}")
    _summarize(targets, args.fps)
    _summarize_hands(hand_targets)
    _check_limits(targets, args.fps, args.max_step_rad, args.max_vel_rad_s)

    if not args.execute:
        print(
            "Dry-run OK. Add --execute only when the real robot area "
            "is clear."
        )
        return 0

    _execute(targets, hand_targets, args, arm_mode)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
