#!/usr/bin/env python3
"""Step-gated G1 client for the bottle Diffusion Policy server."""

from __future__ import annotations

import argparse
import base64
import json
import signal
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import cv2
import numpy as np

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR / "utils") not in sys.path:
    sys.path.insert(0, str(ROOT_DIR / "utils"))

try:
    from vla_client import (  # noqa: E402
        CONTROL_DT,
        CAMERA_ZMQ_PORT,
        MAX_DELTA_PER_STEP,
        N_HAND_MOTORS,
        ROBOT_IP,
        CameraReceiver,
        G1Adapter,
        G1Robot,
        ensure_ai_mode,
        _fmt_deg,
    )
    VLA_IMPORT_ERROR = None
except ModuleNotFoundError as exc:
    VLA_IMPORT_ERROR = exc
    CONTROL_DT = 0.02
    CAMERA_ZMQ_PORT = 5555
    MAX_DELTA_PER_STEP = 0.10
    N_HAND_MOTORS = 7
    ROBOT_IP = "192.168.123.164"
    CameraReceiver = None
    G1Robot = None

    class G1Adapter:
        @staticmethod
        def _clamp_joints(
            q: np.ndarray, motor_indices: list[int], current: np.ndarray
        ) -> np.ndarray:
            return np.asarray(q, dtype=np.float32)

    def ensure_ai_mode() -> bool:
        return False

    def _fmt_deg(values: np.ndarray) -> str:
        return str(np.round(np.rad2deg(values), 1).tolist())


DEFAULT_SERVER_URL = "http://localhost:8020/act"
DEFAULT_TASK = "place the bottle into the paper box"
DEFAULT_STEP_SECONDS = 0.25
DEFAULT_ACTION_EXEC_STEPS = 4
DEFAULT_ARM_KP = 200.0
DEFAULT_ARM_KD = 5.0
STATE_DIM = 63
ACTION_DIM = 31
POLICY_ACTION_DIM = 14
STATE_INDICES = np.array(
    [
        0, 1, 2, 3, 4, 5, 6,
        15, 16, 17, 18, 19, 20, 21, 22,
        23, 24, 25, 26, 27, 28, 29, 30,
    ],
    dtype=np.int64,
)
ACTION_INDICES = np.array(
    [0, 1, 2, 3, 4, 5, 6, 14, 15, 16, 17, 18, 19, 20],
    dtype=np.int64,
)
WAIST_UPRIGHT_Q = np.zeros(3, dtype=np.float32)
OPEN_HAND_Q = np.zeros(7, dtype=np.float32)
FORWARD_READY_ARM_Q = np.zeros(14, dtype=np.float32)
SPREAD_ARM_Q = np.zeros(14, dtype=np.float32)
SPREAD_ARM_Q[1] = 1.5
SPREAD_ARM_Q[8] = -1.5
RELAXED_ARM_Q = np.zeros(14, dtype=np.float32)
RELAXED_ARM_Q[0] = 0.45
RELAXED_ARM_Q[1] = 0.35
RELAXED_ARM_Q[3] = 0.85
RELAXED_ARM_Q[7] = 0.45
RELAXED_ARM_Q[8] = -0.35
RELAXED_ARM_Q[10] = 0.85


def encode_image_b64(frame: np.ndarray) -> str:
    rgb = np.asarray(frame, dtype=np.uint8)
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    ok, encoded = cv2.imencode(
        ".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 90]
    )
    if not ok:
        raise RuntimeError("Failed to encode camera frame")
    return base64.b64encode(encoded.tobytes()).decode("ascii")


def show_robot_view(
    frame: np.ndarray,
    enabled: bool,
    status: str,
) -> None:
    if not enabled:
        return
    vis = np.asarray(frame, dtype=np.uint8)
    if vis.ndim != 3 or vis.shape[2] != 3:
        return
    bgr = cv2.cvtColor(vis, cv2.COLOR_RGB2BGR)
    cv2.putText(
        bgr,
        status,
        (18, 34),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    cv2.imshow("G1 DP Robot View", bgr)
    cv2.waitKey(1)


def build_state_63(
    robot: G1Robot | None, state: dict[str, np.ndarray]
) -> np.ndarray:
    body = np.zeros(35, dtype=np.float32)
    if robot is not None:
        with robot._state_lock:  # Reuse the existing subscriber lock.
            if robot.low_state is None:
                motor_state = []
            else:
                motor_state = robot.low_state.motor_state
        if len(motor_state) > 0:
            body[: min(35, len(motor_state))] = [
                motor_state[i].q for i in range(min(35, len(motor_state)))
            ]
    return np.concatenate(
        [
            state["left_arm"],
            state["right_arm"],
            state["left_hand"],
            state["right_hand"],
            body,
        ]
    ).astype(np.float32)


def state_index_name(index: int) -> str:
    if 0 <= index <= 6:
        return f"left_arm[{index}]"
    if 7 <= index <= 13:
        return f"right_arm[{index - 7}]"
    if 14 <= index <= 20:
        return f"left_hand[{index - 14}]"
    if 21 <= index <= 27:
        return f"right_hand[{index - 21}]"
    return f"body_lowstate[{index - 28}]"


def action_index_name(index: int) -> str:
    if 0 <= index <= 6:
        return f"left_arm[{index}]"
    if 7 <= index <= 13:
        return f"right_arm[{index - 7}]"
    if 14 <= index <= 20:
        return f"left_hand[{index - 14}]"
    if 21 <= index <= 27:
        return f"right_hand[{index - 21}]"
    return f"body_action[{index - 28}]"


def joint_audit(args: argparse.Namespace, robot: G1Robot | None) -> dict:
    state = get_runtime_state(args, robot)
    if state is None:
        return {"ok": False, "message": "No robot state available"}
    state_63 = build_state_63(robot, state)
    selected_state = state_63[STATE_INDICES]
    return {
        "ok": True,
        "note": (
            "This shows the checkpoint's trained mapping. Execution still only "
            "commands left_arm and left_hand plus local waist safety target."
        ),
        "current": {
            "waist_deg": np.rad2deg(state["waist"]).round(2).tolist(),
            "left_arm_deg": np.rad2deg(state["left_arm"]).round(2).tolist(),
            "right_arm_deg": np.rad2deg(state["right_arm"]).round(2).tolist(),
            "left_hand": np.round(state["left_hand"], 3).tolist(),
            "right_hand": np.round(state["right_hand"], 3).tolist(),
        },
        "model_state_indices": [
            {
                "policy_dim": int(i),
                "state_63_index": int(index),
                "source": state_index_name(int(index)),
                "value": float(selected_state[i]),
            }
            for i, index in enumerate(STATE_INDICES)
        ],
        "model_action_indices": [
            {
                "policy_dim": int(i),
                "action_31_index": int(index),
                "target": action_index_name(int(index)),
            }
            for i, index in enumerate(ACTION_INDICES)
        ],
    }


def post_json(url: str, payload: dict, timeout: float) -> dict:
    data = json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        url,
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"Server HTTP {exc.code}: {detail}") from exc


def reset_policy_history(server_url: str, timeout: float) -> None:
    reset_url = server_url.rsplit("/", 1)[0] + "/reset"
    with urllib.request.urlopen(reset_url, timeout=timeout) as response:
        response.read()


def request_action(
    server_url: str,
    frame: np.ndarray,
    state_63: np.ndarray,
    task: str,
    timeout: float,
) -> tuple[np.ndarray, np.ndarray]:
    payload = {
        "image_b64": encode_image_b64(frame),
        "state": state_63.tolist(),
        "task": task,
    }
    result = post_json(server_url, payload, timeout)
    if "error" in result:
        raise RuntimeError(result["error"])
    action_14 = np.asarray(result["action_14"], dtype=np.float32)
    action_31 = np.asarray(result["action_31"], dtype=np.float32)
    if action_14.ndim != 2 or action_14.shape[1] != POLICY_ACTION_DIM:
        raise ValueError(f"Expected action chunk [T,14], got {action_14.shape}")
    if action_31.ndim != 2 or action_31.shape[1] != ACTION_DIM:
        raise ValueError(f"Expected action chunk [T,31], got {action_31.shape}")
    return action_14, action_31


def expand_action_14_to_31(action_14: np.ndarray) -> np.ndarray:
    action_31 = np.zeros(ACTION_DIM, dtype=np.float32)
    action_31[ACTION_INDICES] = np.asarray(action_14, dtype=np.float32)
    return action_31


def decode_left_only_action(
    action_31: np.ndarray,
    state: dict[str, np.ndarray],
    max_delta: float,
    waist_mode: str,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
    action_31 = np.asarray(action_31, dtype=np.float32)
    if action_31.shape[-1] != ACTION_DIM:
        raise ValueError(f"Expected expanded action dim 31, got {action_31.shape}")
    left_arm_target = action_31[:7].copy()
    left_hand_target = action_31[14:21].copy()

    left_arm_target = G1Adapter._clamp_joints(
        left_arm_target,
        list(range(15, 22)),
        state["left_arm"],
    )
    delta = left_arm_target - state["left_arm"]
    max_seen = float(np.max(np.abs(delta))) if delta.size else 0.0
    if max_seen > max_delta:
        scale = max_delta / max_seen
        left_arm_target = state["left_arm"] + delta * scale

    return (
        waist_target(state, waist_mode),
        left_arm_target,
        state["right_arm"].copy(),
        left_hand_target[:N_HAND_MOTORS],
        max_seen,
    )


def waist_target(
    state: dict[str, np.ndarray],
    waist_mode: str,
) -> np.ndarray:
    if waist_mode == "current":
        return state["waist"].copy()
    return WAIST_UPRIGHT_Q.copy()


def hand_target(
    policy_left_hand: np.ndarray,
    state: dict[str, np.ndarray],
    hand_mode: str,
) -> np.ndarray:
    if hand_mode == "policy":
        return np.asarray(policy_left_hand, dtype=np.float32).copy()
    if hand_mode == "current":
        return state["left_hand"].copy()
    return OPEN_HAND_Q.copy()


def left_only_forward_ready_targets(
    state: dict[str, np.ndarray],
    waist_mode: str,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    target = FORWARD_READY_ARM_Q.copy()
    target[7:14] = RELAXED_ARM_Q[7:14]
    return waist_target(state, waist_mode), target[:7].copy(), target[7:14].copy()


def left_only_clearance_targets(
    state: dict[str, np.ndarray],
    waist_mode: str,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    target = SPREAD_ARM_Q.copy()
    target[7:14] = RELAXED_ARM_Q[7:14]
    return waist_target(state, waist_mode), target[:7].copy(), target[7:14].copy()


def relaxed_arm_targets(
    state: dict[str, np.ndarray],
    waist_mode: str,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    return waist_target(state, waist_mode), RELAXED_ARM_Q[:7].copy(), RELAXED_ARM_Q[7:14].copy()


def ramp_to_arm_pose(
    robot: G1Robot,
    waist: np.ndarray,
    left_arm: np.ndarray,
    right_arm: np.ndarray,
    seconds: float,
    arm_kp: float,
    arm_kd: float,
    stop_event: threading.Event | None = None,
) -> bool:
    start = robot.get_state()
    if start is None:
        raise RuntimeError("Cannot prepare pose without robot state")
    start_left = start["left_arm"].copy()
    start_right = start["right_arm"].copy()
    start_waist = start["waist"].copy()
    deadline = time.time() + seconds
    while time.time() < deadline:
        if stop_event is not None and stop_event.is_set():
            return False
        now = time.time()
        alpha = 1.0 - max(deadline - now, 0.0) / max(seconds, 1e-6)
        current = robot.get_state() or start
        robot.send_arm_cmd(
            start_waist + (waist - start_waist) * alpha,
            start_left + (left_arm - start_left) * alpha,
            start_right + (right_arm - start_right) * alpha,
            current["_mode_machine"],
            kp=arm_kp,
            kd=arm_kd,
        )
        time.sleep(CONTROL_DT)
    current = robot.get_state() or start
    robot.send_arm_cmd(
        waist,
        left_arm,
        right_arm,
        current["_mode_machine"],
        kp=arm_kp,
        kd=arm_kd,
    )
    return True


def open_hands(
    robot: G1Robot,
    state: dict[str, np.ndarray] | None = None,
    duration: float = 0.8,
) -> None:
    current = state or robot.get_state()
    if current is None:
        raise RuntimeError("Cannot open hands without robot state")
    deadline = time.time() + duration
    while time.time() < deadline:
        latest = robot.get_state() or current
        robot.send_hand_cmd(OPEN_HAND_Q, OPEN_HAND_Q)
        current = latest
        time.sleep(CONTROL_DT)


def prepare_left_only_forward_pose(
    robot: G1Robot,
    via_seconds: float,
    final_seconds: float,
    arm_kp: float,
    arm_kd: float,
    waist_mode: str,
    stop_event: threading.Event | None = None,
) -> None:
    state = robot.get_state()
    if state is None:
        raise RuntimeError("Cannot prepare forward pose without robot state")
    print("[DP] Preparing teleop-style left-only forward pose.")
    print("     Phase 1: outward clearance, right arm relaxed.")
    if not ramp_to_arm_pose(
        robot,
        *left_only_clearance_targets(state, waist_mode),
        seconds=via_seconds,
        arm_kp=arm_kp,
        arm_kd=arm_kd,
        stop_event=stop_event,
    ):
        return
    state = robot.get_state() or state
    print("     Phase 1b: opening Dex3 hands after clearance.")
    open_hands(robot, state, duration=0.8)
    state = robot.get_state() or state
    print("     Phase 2: left arm forward-ready, right arm relaxed.")
    ramp_to_arm_pose(
        robot,
        *left_only_forward_ready_targets(state, waist_mode),
        seconds=final_seconds,
        arm_kp=arm_kp,
        arm_kd=arm_kd,
        stop_event=stop_event,
    )
    print("[DP] Forward-ready pose reached.")


def stop_and_relax_arms(
    robot: G1Robot,
    via_seconds: float,
    relax_seconds: float,
    arm_kp: float,
    arm_kd: float,
    waist_mode: str,
) -> None:
    state = robot.get_state()
    if state is None:
        raise RuntimeError("Cannot relax arms without robot state")
    print("[DP] Stop requested: moving arms to relaxed pose.")
    ramp_to_arm_pose(
        robot,
        *left_only_clearance_targets(state, waist_mode),
        seconds=via_seconds,
        arm_kp=arm_kp,
        arm_kd=arm_kd,
    )
    state = robot.get_state() or state
    open_hands(robot, state, duration=0.8)
    state = robot.get_state() or state
    ramp_to_arm_pose(
        robot,
        *relaxed_arm_targets(state, waist_mode),
        seconds=relax_seconds,
        arm_kp=arm_kp,
        arm_kd=arm_kd,
    )
    print("[DP] Arms relaxed.")


def print_step_summary(
    step_idx: int,
    state: dict,
    action_14: np.ndarray,
    action_31: np.ndarray,
) -> None:
    action_31_expanded = expand_action_14_to_31(action_14)
    left_arm = action_31_expanded[:7]
    left_hand = action_31_expanded[14:21]
    ignored_body = action_31[28:31]
    print(f"\n[DP] Step {step_idx}")
    print(f"  left_arm target deg: {_fmt_deg(left_arm)}")
    print(f"  left_hand target:    {np.round(left_hand, 3).tolist()}")
    print(f"  current left deg:    {_fmt_deg(state['left_arm'])}")
    if np.linalg.norm(ignored_body) > 1e-5:
        print(
            "  note: ignoring body action "
            f"{np.round(ignored_body, 4).tolist()}"
        )


def execute_step(
    robot: G1Robot,
    state: dict,
    waist: np.ndarray,
    left_arm: np.ndarray,
    right_arm: np.ndarray,
    left_hand: np.ndarray,
    send_hands: bool,
    seconds: float,
    arm_kp: float,
    arm_kd: float,
    stop_event: threading.Event | None = None,
) -> bool:
    start = robot.get_state() or state
    start_waist = start["waist"].copy()
    start_left = start["left_arm"].copy()
    start_right = start["right_arm"].copy()
    start_left_hand = start["left_hand"].copy()
    deadline = time.time() + seconds
    while time.time() < deadline:
        if stop_event is not None and stop_event.is_set():
            return False
        now = time.time()
        alpha = 1.0 - max(deadline - now, 0.0) / max(seconds, 1e-6)
        current = robot.get_state() or state
        robot.send_arm_cmd(
            start_waist + (waist - start_waist) * alpha,
            start_left + (left_arm - start_left) * alpha,
            start_right + (right_arm - start_right) * alpha,
            current["_mode_machine"],
            kp=arm_kp,
            kd=arm_kd,
        )
        if send_hands:
            robot.send_hand_cmd(
                start_left_hand + (left_hand - start_left_hand) * alpha,
                current["right_hand"],
            )
        time.sleep(CONTROL_DT)
    current = robot.get_state() or state
    robot.send_arm_cmd(
        waist,
        left_arm,
        right_arm,
        current["_mode_machine"],
        kp=arm_kp,
        kd=arm_kd,
    )
    if send_hands:
        robot.send_hand_cmd(left_hand, current["right_hand"])
    return True


def get_runtime_state(
    args: argparse.Namespace,
    robot: G1Robot | None,
) -> dict[str, np.ndarray] | None:
    if args.mock:
        return {
            "waist": np.zeros(3, dtype=np.float32),
            "left_arm": np.zeros(7, dtype=np.float32),
            "right_arm": np.zeros(7, dtype=np.float32),
            "left_hand": np.zeros(7, dtype=np.float32),
            "right_hand": np.zeros(7, dtype=np.float32),
            "_mode_machine": 0,
        }
    assert robot is not None
    return robot.get_state()


def get_runtime_frame(camera: CameraReceiver | None) -> np.ndarray:
    frame = None if camera is None else camera.get_frame()
    if frame is None:
        return np.zeros((480, 640, 3), dtype=np.uint8)
    return frame


def run_single_policy_step(
    args: argparse.Namespace,
    robot: G1Robot | None,
    camera: CameraReceiver | None,
    step_idx: int,
    stop_event: threading.Event | None = None,
) -> dict:
    state = get_runtime_state(args, robot)
    if state is None:
        raise RuntimeError("Robot state is not ready")
    frame = get_runtime_frame(camera)
    state_63 = build_state_63(robot, state)
    action_chunk_14, action_chunk_31 = request_action(
        args.server_url,
        frame,
        state_63,
        args.task,
        args.timeout,
    )
    exec_steps = min(int(args.action_exec_steps), len(action_chunk_14))
    step_results = []
    stopped = False
    for local_idx in range(exec_steps):
        if stop_event is not None and stop_event.is_set():
            stopped = True
            break
        action_14 = action_chunk_14[local_idx]
        action_31 = action_chunk_31[local_idx]
        current_state = get_runtime_state(args, robot) or state
        print_step_summary(step_idx + local_idx, current_state, action_14, action_31)
        waist, left_arm, right_arm, left_hand, max_seen = decode_left_only_action(
            action_31,
            current_state,
            args.max_delta,
            args.waist_mode,
        )
        policy_left_hand = left_hand.copy()
        left_hand = hand_target(policy_left_hand, current_state, args.hand_mode)
        clipped = max_seen > args.max_delta
        if clipped:
            print(
                "  large arm delta clipped: "
                f"{max_seen:.3f} -> {args.max_delta:.3f} rad"
            )
        if args.execute:
            assert robot is not None
            ok = execute_step(
                robot,
                current_state,
                waist,
                left_arm,
                right_arm,
                left_hand,
                args.send_hands,
                args.step_seconds,
                arm_kp=args.arm_kp,
                arm_kd=args.arm_kd,
                stop_event=stop_event,
            )
            if not ok:
                stopped = True
                break
        step_results.append(
            {
                "step": step_idx + local_idx,
                "clipped": clipped,
                "max_delta_rad": float(max_seen),
                "left_arm_deg": np.rad2deg(action_31[:7]).round(2).tolist(),
                "policy_left_hand": np.round(policy_left_hand, 3).tolist(),
                "executed_left_hand": np.round(left_hand, 3).tolist(),
                "hand_mode": args.hand_mode,
            }
        )
    return {
        "step": step_idx,
        "mode": "execute" if args.execute else "dry-run",
        "chunk_len": int(len(action_chunk_14)),
        "exec_steps": len(step_results),
        "stopped": stopped,
        "steps": step_results,
    }


def run_control_ui(
    args: argparse.Namespace,
    robot: G1Robot | None,
    camera: CameraReceiver | None,
) -> None:
    from fastapi import FastAPI
    from fastapi.responses import HTMLResponse, StreamingResponse
    import uvicorn

    app = FastAPI()
    lock = threading.Lock()
    stop_event = threading.Event()
    relax_once_lock = threading.Lock()
    relaxed_on_shutdown = {"done": False}
    server_ref = {"server": None}
    state = {
        "step_idx": 0,
        "busy": False,
        "message": "Ready. Click Reset Forward Pose before first execute.",
        "last_result": None,
    }

    def set_message(message: str) -> None:
        state["message"] = message
        print(f"[DP UI] {message}")

    def emergency_relax(reason: str) -> None:
        if args.mock or not args.execute or robot is None:
            return
        with relax_once_lock:
            if relaxed_on_shutdown["done"]:
                return
            relaxed_on_shutdown["done"] = True
        print(f"[DP UI] Emergency relax requested: {reason}")
        stop_event.set()
        acquired = lock.acquire(timeout=5.0)
        if not acquired:
            print("[DP UI] Could not acquire control lock for emergency relax.")
            return
        try:
            state["busy"] = True
            set_message(f"Emergency relax: {reason}")
            stop_and_relax_arms(
                robot,
                via_seconds=args.prepare_via_seconds,
                relax_seconds=args.prepare_forward_seconds,
                arm_kp=args.arm_kp,
                arm_kd=args.arm_kd,
                waist_mode=args.waist_mode,
            )
            try:
                reset_policy_history(args.server_url, args.timeout)
            except Exception as exc:
                print(f"[DP UI] Policy history reset failed during shutdown: {exc}")
            state["step_idx"] = 0
            set_message("Emergency relax complete.")
        except Exception as exc:
            print(f"[DP UI] Emergency relax failed: {exc}")
        finally:
            state["busy"] = False
            stop_event.clear()
            lock.release()

    def handle_shutdown_signal(signum, frame) -> None:
        signame = signal.Signals(signum).name
        emergency_relax(signame)
        server = server_ref.get("server")
        if server is not None:
            server.should_exit = True

    signal.signal(signal.SIGINT, handle_shutdown_signal)
    signal.signal(signal.SIGTERM, handle_shutdown_signal)

    @app.on_event("shutdown")
    def on_shutdown() -> None:
        emergency_relax("uvicorn shutdown")

    @app.get("/", response_class=HTMLResponse)
    def index() -> str:
        mode = "EXECUTE" if args.execute else "DRY RUN"
        return f"""
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <title>G1 DP Control</title>
  <style>
    body {{ font-family: sans-serif; margin: 18px; background: #111; color: #eee; }}
    .row {{ display: flex; gap: 16px; align-items: flex-start; }}
    img {{ width: 720px; max-width: 70vw; border: 2px solid #444; }}
    button {{ font-size: 18px; padding: 14px 18px; margin: 8px 0; width: 260px; }}
    h3 {{ margin: 0 0 8px 0; }}
    .button-block {{ border: 1px solid #444; border-radius: 8px; padding: 12px; margin-bottom: 14px; }}
    .hint {{ margin: 4px 0 10px 0; color: #bbb; font-size: 14px; }}
    #prepare {{ background: #1b5f9c; color: white; }}
    #step {{ background: #1f7a1f; color: white; }}
    #run20 {{ background: #d97706; color: white; font-weight: bold; }}
    #audit {{ background: #555; color: white; }}
    #reset {{ background: #8a5a00; color: white; }}
    #stop {{ background: #a00000; color: white; }}
    .panel {{ min-width: 320px; }}
    pre {{ white-space: pre-wrap; background: #222; padding: 12px; }}
  </style>
</head>
<body>
  <h2>G1 Diffusion Policy Control ({mode})</h2>
  <div class="row">
    <img src="/video" />
    <div class="panel">
      <div class="button-block">
        <h3>Start</h3>
        <p class="hint">Use Preparation first, then run policy one chunk at a time.</p>
        <button id="prepare" onclick="post('/prepare')">Preparation</button><br/>
        <button id="step" onclick="post('/step')">Run Next Step</button><br/>
        <button id="run20" onclick="confirmPost('/run-20')">
          ATTENTION/CAUTIOUS: Run 20 Model Inferences
        </button>
      </div>
      <div class="button-block">
        <h3>Maintenance</h3>
        <p class="hint">Use these to recover, restart model history, or stop the robot.</p>
        <button id="audit" onclick="post('/joint-audit')">Joint Audit</button><br/>
        <button id="reset" onclick="post('/reset')">Reset + Restart Model</button><br/>
        <button id="stop" onclick="post('/stop')">Stop + Relax Arms</button>
      </div>
      <p>Checkpoint task: <code>g1-bottle-in-out</code></p>
      <p>Text prompt: <code>{args.task}</code> <b>(not used by this DP model)</b></p>
      <p>Every click executes up to <b>{args.action_exec_steps}</b>
      smooth chunk steps. Press <b>Stop + Relax Arms</b> to interrupt.</p>
      <p>Hands: <code>{'enabled' if args.send_hands else 'disabled'}</code>,
      hand mode: <code>{args.hand_mode}</code>,
      arm/waist kp: <code>{args.arm_kp}</code>, kd: <code>{args.arm_kd}</code>,
      waist: <code>{args.waist_mode}</code></p>
      <pre id="status">Loading...</pre>
    </div>
  </div>
  <script>
    async function post(path) {{
      const res = await fetch(path, {{method: 'POST'}});
      document.getElementById('status').textContent =
        JSON.stringify(await res.json(), null, 2);
    }}
    async function confirmPost(path) {{
      const ok = confirm(
        'ATTENTION/CAUTIOUS: this will run 20 model inferences. Continue?'
      );
      if (ok) {{
        await post(path);
      }}
    }}
    async function refresh() {{
      const res = await fetch('/status');
      const payload = await res.json();
      payload.browser_status_refresh = new Date().toLocaleTimeString();
      document.getElementById('status').textContent =
        JSON.stringify(payload, null, 2);
    }}
    setInterval(refresh, 1000);
    refresh();
  </script>
</body>
</html>
"""

    @app.get("/status")
    def status() -> dict:
        return dict(state)

    @app.post("/joint-audit")
    def audit() -> dict:
        result = joint_audit(args, robot)
        state["last_joint_audit"] = result
        return result

    def run_preparation_command(action_name: str) -> dict:
        if not args.execute:
            set_message(f"Dry-run mode: {action_name} did not move the robot.")
            return {"ok": True, "message": state["message"]}
        if not lock.acquire(blocking=False):
            return {"ok": False, "message": "Busy; wait for current command."}
        try:
            state["busy"] = True
            stop_event.clear()
            set_message(
                f"{action_name}: moving to forward pose and restarting model history..."
            )
            assert robot is not None
            prepare_left_only_forward_pose(
                robot,
                via_seconds=args.prepare_via_seconds,
                final_seconds=args.prepare_forward_seconds,
                arm_kp=args.arm_kp,
                arm_kd=args.arm_kd,
                waist_mode=args.waist_mode,
                stop_event=stop_event,
            )
            reset_policy_history(args.server_url, args.timeout)
            state["step_idx"] = 0
            set_message(f"{action_name} complete; model history restarted.")
            return {"ok": True, "message": state["message"]}
        except Exception as exc:
            set_message(f"{action_name} failed: {exc}")
            return {"ok": False, "message": str(exc)}
        finally:
            state["busy"] = False
            lock.release()

    @app.post("/prepare")
    def prepare() -> dict:
        return run_preparation_command("Preparation")

    @app.post("/step")
    def step() -> dict:
        if not lock.acquire(blocking=False):
            return {"ok": False, "message": "Busy; wait for current command."}
        try:
            state["busy"] = True
            stop_event.clear()
            set_message("Running one policy action chunk...")
            result = run_single_policy_step(
                args,
                robot,
                camera,
                int(state["step_idx"]),
                stop_event=stop_event,
            )
            state["step_idx"] = int(state["step_idx"]) + int(result["exec_steps"])
            state["last_result"] = result
            if result["stopped"]:
                set_message("Action chunk stopped.")
            else:
                set_message("Action chunk complete.")
            return {"ok": True, "result": result}
        except Exception as exc:
            set_message(f"Step failed: {exc}")
            return {"ok": False, "message": str(exc)}
        finally:
            state["busy"] = False
            lock.release()

    @app.post("/run-20")
    def run_20() -> dict:
        if not lock.acquire(blocking=False):
            return {"ok": False, "message": "Busy; wait for current command."}
        try:
            state["busy"] = True
            stop_event.clear()
            results = []
            for i in range(20):
                if stop_event.is_set():
                    break
                set_message(f"Running model inference {i + 1}/20...")
                result = run_single_policy_step(
                    args,
                    robot,
                    camera,
                    int(state["step_idx"]),
                    stop_event=stop_event,
                )
                state["step_idx"] = (
                    int(state["step_idx"]) + int(result["exec_steps"])
                )
                results.append(result)
                state["last_result"] = result
                if result["stopped"]:
                    break
            if stop_event.is_set() or any(r["stopped"] for r in results):
                set_message("Run 20 stopped before completion.")
            else:
                set_message("Run 20 complete.")
            return {
                "ok": True,
                "requested_inferences": 20,
                "completed_inferences": len(results),
                "results": results,
            }
        except Exception as exc:
            set_message(f"Run 20 failed: {exc}")
            return {"ok": False, "message": str(exc)}
        finally:
            state["busy"] = False
            lock.release()

    @app.post("/reset")
    def reset() -> dict:
        return run_preparation_command("Reset + Restart Model")

    @app.post("/stop")
    def stop() -> dict:
        stop_event.set()
        if not args.execute:
            set_message("Dry-run mode: stop requested; no robot command sent.")
            return {"ok": True, "message": state["message"]}
        set_message("Stop requested; waiting to send relax command...")
        lock.acquire()
        try:
            state["busy"] = True
            assert robot is not None
            stop_and_relax_arms(
                robot,
                via_seconds=args.prepare_via_seconds,
                relax_seconds=args.prepare_forward_seconds,
                arm_kp=args.arm_kp,
                arm_kd=args.arm_kd,
                waist_mode=args.waist_mode,
            )
            reset_policy_history(args.server_url, args.timeout)
            state["step_idx"] = 0
            set_message("Stopped and arms relaxed.")
            return {"ok": True, "message": state["message"]}
        except Exception as exc:
            set_message(f"Stop/relax failed: {exc}")
            return {"ok": False, "message": str(exc)}
        finally:
            state["busy"] = False
            stop_event.clear()
            lock.release()

    def video_stream():
        while True:
            frame = get_runtime_frame(camera)
            bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            cv2.putText(
                bgr,
                f"DP {'EXECUTE' if args.execute else 'DRY RUN'} | "
                f"step {state['step_idx']}",
                (18, 34),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                bgr,
                time.strftime("stream %H:%M:%S"),
                (18, 68),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            ok, jpg = cv2.imencode(".jpg", bgr)
            if ok:
                yield (
                    b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                    + jpg.tobytes()
                    + b"\r\n"
                )
            time.sleep(1.0 / 15.0)

    @app.get("/video")
    def video():
        return StreamingResponse(
            video_stream(),
            media_type="multipart/x-mixed-replace; boundary=frame",
        )

    print(
        "[DP UI] Open http://"
        f"{args.ui_host if args.ui_host != '0.0.0.0' else 'localhost'}"
        f":{args.ui_port}"
    )
    config = uvicorn.Config(
        app,
        host=args.ui_host,
        port=args.ui_port,
        log_level="warning",
    )
    server = uvicorn.Server(config)
    server_ref["server"] = server
    # Keep our signal handlers so Ctrl-C can relax the robot before exit.
    server.install_signal_handlers = lambda: None
    server.run()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--server-url", type=str, default=DEFAULT_SERVER_URL)
    parser.add_argument("--task", type=str, default=DEFAULT_TASK)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--send-hands",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Send policy/reset commands to Dex3 hands. Use --no-send-hands to disable.",
    )
    parser.add_argument(
        "--hand-mode",
        choices=("open", "current", "policy"),
        default="open",
        help=(
            "Hand target during policy steps. Default keeps Dex3 open; "
            "use policy only after verifying finger mapping."
        ),
    )
    parser.add_argument("--continuous", action="store_true")
    parser.add_argument(
        "--step-seconds", type=float, default=DEFAULT_STEP_SECONDS
    )
    parser.add_argument(
        "--action-exec-steps",
        type=int,
        default=DEFAULT_ACTION_EXEC_STEPS,
        help="Number of predicted chunk steps to execute per UI click/approval.",
    )
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--max-delta", type=float, default=MAX_DELTA_PER_STEP)
    parser.add_argument("--arm-kp", type=float, default=DEFAULT_ARM_KP)
    parser.add_argument("--arm-kd", type=float, default=DEFAULT_ARM_KD)
    parser.add_argument(
        "--waist-mode",
        choices=("upright", "current"),
        default="upright",
        help="Use zero waist target by default, or hold the current waist pose.",
    )
    parser.add_argument("--network-interface", type=str, default=None)
    parser.add_argument("--camera-robot-ip", type=str, default=ROBOT_IP)
    parser.add_argument("--camera-port", type=int, default=CAMERA_ZMQ_PORT)
    parser.add_argument("--ui", action="store_true")
    parser.add_argument("--ui-host", type=str, default="127.0.0.1")
    parser.add_argument("--ui-port", type=int, default=8030)
    parser.add_argument(
        "--no-view",
        action="store_true",
        help="Disable the local robot-view visualization window.",
    )
    parser.add_argument(
        "--no-prepare-forward",
        action="store_true",
        help="Do not move to the teleop-style forward-ready pose before execute.",
    )
    parser.add_argument("--prepare-via-seconds", type=float, default=2.0)
    parser.add_argument("--prepare-forward-seconds", type=float, default=4.0)
    parser.add_argument(
        "--mock",
        action="store_true",
        help="Use zero image/state without connecting to robot DDS.",
    )
    parser.add_argument("--once", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.mock and args.execute:
        raise SystemExit("--mock cannot be combined with --execute")

    if not args.mock:
        if VLA_IMPORT_ERROR is not None:
            raise SystemExit(
                "Real robot mode requires vla_client and Unitree SDK imports. "
                f"Import failed: {VLA_IMPORT_ERROR}"
            )
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize

        ChannelFactoryInitialize(0, args.network_interface)
    if args.execute and not ensure_ai_mode():
        raise SystemExit("Robot is not in ai mode; refusing to execute.")

    robot = None
    if not args.mock:
        robot = G1Robot()
        robot.init()
        if not robot.wait_for_state(timeout=5.0):
            raise RuntimeError("Timed out waiting for robot lowstate")

    camera = None
    if not args.mock:
        camera = CameraReceiver(
            robot_ip=args.camera_robot_ip,
            port=args.camera_port,
        )
        camera.start()
    print(f"[DP] Server: {args.server_url}")
    print(f"[DP] Mode: {'EXECUTE' if args.execute else 'DRY RUN'}")
    if args.mock:
        print("[DP] Mock mode: using zero image and zero robot state.")
    elif camera is not None:
        print(f"[DP] Camera endpoint: {camera.endpoint}")
    if args.execute and not args.continuous:
        print("[DP] Enter=execute chunk, s=skip, c=continuous, q=quit.")

    if args.ui:
        run_control_ui(args, robot, camera)
        return

    show_view = (not args.mock) and (not args.no_view)
    if args.execute and not args.no_prepare_forward:
        assert robot is not None
        user_input = input(
            "[DP] Press Enter to move to teleop-style forward-ready pose "
            "(or q to quit): "
        ).strip().lower()
        if user_input == "q":
            return
        prepare_left_only_forward_pose(
            robot,
            via_seconds=args.prepare_via_seconds,
            final_seconds=args.prepare_forward_seconds,
            arm_kp=args.arm_kp,
            arm_kd=args.arm_kd,
            waist_mode=args.waist_mode,
        )

    step_idx = 0
    continuous = args.continuous
    try:
        while True:
            if args.mock:
                state = {
                    "waist": np.zeros(3, dtype=np.float32),
                    "left_arm": np.zeros(7, dtype=np.float32),
                    "right_arm": np.zeros(7, dtype=np.float32),
                    "left_hand": np.zeros(7, dtype=np.float32),
                    "right_hand": np.zeros(7, dtype=np.float32),
                    "_mode_machine": 0,
                }
            else:
                state = robot.get_state()
            if state is None:
                time.sleep(0.1)
                continue
            frame = None if camera is None else camera.get_frame()
            if frame is None:
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
            show_robot_view(
                frame,
                show_view,
                f"DP {'EXECUTE' if args.execute else 'DRY RUN'} | step {step_idx}",
            )
            state_63 = build_state_63(robot, state)
            action_chunk_14, action_chunk_31 = request_action(
                args.server_url,
                frame,
                state_63,
                args.task,
                args.timeout,
            )
            for action_14, action_31 in zip(action_chunk_14, action_chunk_31):
                print_step_summary(step_idx, state, action_14, action_31)
                waist, left_arm, right_arm, left_hand, max_seen = (
                    decode_left_only_action(
                        action_31,
                        state,
                        args.max_delta,
                        args.waist_mode,
                    )
                )
                policy_left_hand = left_hand.copy()
                left_hand = hand_target(policy_left_hand, state, args.hand_mode)
                if max_seen > args.max_delta:
                    print(
                        "  large arm delta clipped: "
                        f"{max_seen:.3f} -> {args.max_delta:.3f} rad"
                    )

                if args.execute and not continuous:
                    user_input = input(
                        "  Execute? [Enter/s/c/q]: "
                    ).strip().lower()
                    if user_input == "q":
                        return
                    if user_input == "s":
                        step_idx += 1
                        continue
                    if user_input == "c":
                        continuous = True
                    elif user_input != "":
                        print(f"  Unknown input '{user_input}', skipping.")
                        step_idx += 1
                        continue

                if args.execute:
                    assert robot is not None
                    execute_step(
                        robot,
                        state,
                        waist,
                        left_arm,
                        right_arm,
                        left_hand,
                        args.send_hands,
                        args.step_seconds,
                        arm_kp=args.arm_kp,
                        arm_kd=args.arm_kd,
                    )
                step_idx += 1
                if not continuous:
                    break
            if args.once:
                return
    except KeyboardInterrupt:
        print("\n[DP] Stopped by user.")
    finally:
        if not args.mock and not args.no_view:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
