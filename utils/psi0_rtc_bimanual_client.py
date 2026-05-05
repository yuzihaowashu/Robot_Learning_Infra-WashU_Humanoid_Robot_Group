#!/usr/bin/env python3
"""
Psi0 RTC client that keeps the upstream WebSocket/action-timing design but
executes only bimanual arm/hand commands on G1.
"""

import argparse
import base64
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, StreamingResponse
from numpy.lib.format import descr_to_dtype, dtype_to_descr
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from websocket import WebSocketApp

ROOT_DIR = Path(__file__).resolve().parents[1]
PSI0_REAL_DIR = ROOT_DIR / "Psi0" / "real"
if str(PSI0_REAL_DIR) not in sys.path:
    sys.path.insert(0, str(PSI0_REAL_DIR))
if str(ROOT_DIR / "utils") not in sys.path:
    sys.path.insert(0, str(ROOT_DIR / "utils"))

from vla_client import (  # noqa: E402
    ARM_SDK_ENABLE_IDX,
    ARM_SDK_JOINTS,
    CONTROL_DT,
    KD_ARM,
    KP_ARM,
    LEFT_ARM_JOINTS,
    RIGHT_ARM_JOINTS,
    N_HAND_MOTORS,
    WAIST_JOINTS,
    CameraReceiver,
    G1Adapter,
    G1Robot,
    ensure_ai_mode,
    _fmt_deg,
    _fmt_rad,
)


DEFAULT_HOST = "localhost"
DEFAULT_PORT = 8014
DEFAULT_TASK = "put the bottle into the paper box"
FREQ_CTRL = 60.0
OBS_SEND_INTERVAL = 0.01
DEFAULT_UI_APPROVAL_STEPS = 15
DEFAULT_STEP_EXEC_SECONDS = 1.0
TEST_NUDGE_RAD = 0.08
KP_WAIST_XR = 200.0
KD_WAIST_XR = 5.0
OPEN_HAND_Q = np.zeros(N_HAND_MOTORS, dtype=np.float32)
SPREAD_ARM_Q = np.zeros(14, dtype=np.float32)
SPREAD_ARM_Q[1] = 1.5
SPREAD_ARM_Q[8] = -1.5
PREPARE_ARM_Q = np.zeros(14, dtype=np.float32)
RELAXED_ARM_Q = np.zeros(14, dtype=np.float32)
RELAXED_ARM_Q[0] = 0.45
RELAXED_ARM_Q[1] = 0.35
RELAXED_ARM_Q[3] = 0.85
RELAXED_ARM_Q[7] = 0.45
RELAXED_ARM_Q[8] = -0.35
RELAXED_ARM_Q[10] = 0.85
TASK_LIST_PATH = ROOT_DIR / "tasks" / "task_list.json"


def language_prompt_from_task(task):
    parts = [
        str(task.get("goal", "")).strip(),
        str(task.get("desc", "")).strip(),
        str(task.get("steps", "")).strip(),
    ]
    return " ".join(part for part in parts if part)


def load_task_presets(default_prompt):
    tasks = []
    if TASK_LIST_PATH.exists():
        try:
            payload = json.loads(TASK_LIST_PATH.read_text())
            for item in payload.get("tasks", []):
                prompt = language_prompt_from_task(item)
                if not prompt:
                    continue
                tasks.append(
                    {
                        "label": item.get("label") or item.get("name"),
                        "name": item.get("name") or item.get("label"),
                        "prompt": prompt,
                    }
                )
        except Exception as exc:
            print(f"[TASK] Could not load {TASK_LIST_PATH}: {exc}")
    if not tasks:
        tasks.append(
            {
                "label": "Command line task",
                "name": "command_line_task",
                "prompt": default_prompt,
            }
        )
    return tasks


def find_competing_teleop_processes():
    try:
        result = subprocess.run(
            ["pgrep", "-af", "teleop_hand_and_arm.py"],
            check=False,
            capture_output=True,
            text=True,
        )
    except OSError:
        return []
    return [
        line
        for line in result.stdout.splitlines()
        if "teleop_hand_and_arm.py" in line
    ]


def numpy_serialize(obj):
    if isinstance(obj, (np.ndarray, np.generic)):
        data = obj.data if obj.flags["C_CONTIGUOUS"] else obj.tobytes()
        return {
            "__numpy__": base64.b64encode(data).decode(),
            "dtype": dtype_to_descr(obj.dtype),
            "shape": obj.shape,
        }
    raise TypeError(
        f"Object of type {obj.__class__.__name__} is not JSON serializable"
    )


def numpy_deserialize(obj):
    if "__numpy__" in obj:
        arr = np.frombuffer(
            base64.b64decode(obj["__numpy__"]),
            descr_to_dtype(obj["dtype"]),
        )
        return arr.reshape(obj["shape"]) if obj["shape"] else arr[0]
    return obj


def convert_numpy_in_dict(data, func):
    if isinstance(data, dict):
        if "__numpy__" in data:
            return func(data)
        return {
            key: convert_numpy_in_dict(value, func)
            for key, value in data.items()
        }
    if isinstance(data, list):
        return [convert_numpy_in_dict(item, func) for item in data]
    if isinstance(data, (np.ndarray, np.generic)):
        return func(data)
    return data


class LatestActionBuffer:
    def __init__(self):
        self._lock = threading.Lock()
        self._action = None
        self._version = -1

    def update(self, action, version):
        with self._lock:
            self._action = np.asarray(action, dtype=np.float32)
            self._version = int(version)

    def read(self):
        with self._lock:
            if self._action is None:
                return None, self._version
            return self._action.copy(), self._version


class Psi0RTCWebSocketClient:
    def __init__(self, server_url, task, camera, hand_robot, action_buffer,
                 verbose=False):
        self.server_url = server_url
        self.task = task
        self.camera = camera
        self.hand_robot = hand_robot
        self.action_buffer = action_buffer
        self.verbose = verbose
        self._connected = threading.Event()
        self._running = True
        self._send_lock = threading.Lock()
        self._task_lock = threading.Lock()
        self._ws = None

    def set_task(self, task):
        with self._task_lock:
            self.task = task

    def get_task(self):
        with self._task_lock:
            return self.task

    def _on_open(self, ws):
        print("[WS] Connected to Psi0 RTC server.")
        self._connected.set()

    def _on_message(self, ws, message):
        try:
            data = json.loads(message)
            action_data = data.get("action")
            version = data.get("version", -1)
            if action_data is None:
                return
            action = convert_numpy_in_dict(action_data, numpy_deserialize)
            self.action_buffer.update(action, version)
            if self.verbose:
                print(f"[WS] Received action version={version}, shape={action.shape}")
        except Exception as exc:
            print(f"[WS] Message processing error: {exc}")

    def _on_error(self, ws, error):
        print(f"[WS] Error: {error}")

    def _on_close(self, ws, close_status_code, close_msg):
        print(f"[WS] Closed: {close_status_code} - {close_msg}")
        self._running = False

    def _send_observations(self):
        self._connected.wait()
        while self._running:
            try:
                frame = self.camera.get_frame()
                if frame is None:
                    frame = np.zeros((480, 640, 3), dtype=np.uint8)
                state = self.hand_robot.get_state()
                instruction = self.get_task()
                payload = {
                    "image": {
                        "observation.images.egocentric": frame.astype(np.uint8),
                    },
                    "state": {
                        "arm_joints": np.concatenate(
                            [state["left_arm"], state["right_arm"]]
                        ).astype(np.float32),
                        "hand_joints": np.concatenate(
                            [state["left_hand"], state["right_hand"]]
                        ).astype(np.float32),
                    },
                    "gt_action": None,
                    "dataset_name": None,
                    "instruction": instruction,
                    "history": {},
                    "condition": {},
                    "timestamp": None,
                }
                message = json.dumps(
                    convert_numpy_in_dict(payload, numpy_serialize)
                )
                with self._send_lock:
                    if not (self._ws and self._ws.sock and self._ws.sock.connected):
                        break
                    self._ws.send(message)
            except Exception as exc:
                print(f"[WS] Observation send error: {exc}")
                break
            time.sleep(OBS_SEND_INTERVAL)

    def run(self):
        self._ws = WebSocketApp(
            self.server_url,
            on_open=self._on_open,
            on_message=self._on_message,
            on_error=self._on_error,
            on_close=self._on_close,
        )
        send_thread = threading.Thread(target=self._send_observations, daemon=True)
        send_thread.start()
        self._ws.run_forever()
        self._running = False
        send_thread.join(timeout=2.0)

    def stop(self):
        self._running = False
        if self._ws:
            self._ws.close()


def decode_policy_action(
    action,
    current_arm,
    previous_target,
    action_mode,
    arm_side,
):
    if action.ndim == 2:
        action = action[0]
    if action.shape[0] < 28:
        raise ValueError(f"Expected Psi0 action dim >= 28, got {action.shape}")

    # Match the LeRobot training action layout:
    # left_arm, right_arm, left_hand, right_hand, body.
    arm = action[:14].copy()
    hand = action[14:28].copy()
    if action_mode == "delta":
        arm_target = previous_target + arm
    elif action_mode == "absolute":
        arm_target = arm
    else:
        raise ValueError(f"Unknown action mode: {action_mode}")

    left_target = G1Adapter._clamp_joints(
        arm_target[:7],
        LEFT_ARM_JOINTS,
        current_arm[:7],
    )
    if arm_side == "left":
        right_target = previous_target[7:14].copy()
        hand_target = np.concatenate([hand[:N_HAND_MOTORS], OPEN_HAND_Q])
    elif arm_side == "bimanual":
        right_target = G1Adapter._clamp_joints(
            arm_target[7:14],
            RIGHT_ARM_JOINTS,
            current_arm[7:14],
        )
        hand_target = hand
    else:
        raise ValueError(f"Unknown arm side: {arm_side}")
    return np.concatenate([left_target, right_target]), hand_target


def current_dual_arm(robot):
    state = robot.get_state()
    return np.concatenate([state["left_arm"], state["right_arm"]]).astype(
        np.float32
    )


def select_waist_target(state, waist_mode):
    if waist_mode in ("xr-upright", "upright"):
        return np.zeros_like(state["waist"], dtype=np.float32)
    return state["waist"].copy()


def send_bimanual_arm_cmd(robot, state, arm_target, waist_target, waist_mode):
    cmd = unitree_hg_msg_dds__LowCmd_()
    cmd.mode_pr = 0
    cmd.mode_machine = state["_mode_machine"]
    cmd.motor_cmd[ARM_SDK_ENABLE_IDX].q = 1.0

    if waist_mode == "passive":
        waist_pos = state["waist"].copy()
        waist_kp = 0.0
        waist_kd = 0.0
    elif waist_mode == "xr-upright":
        waist_pos = np.zeros_like(state["waist"], dtype=np.float32)
        waist_kp = KP_WAIST_XR
        waist_kd = KD_WAIST_XR
    else:
        waist_pos = waist_target
        waist_kp = KP_ARM
        waist_kd = KD_ARM

    all_pos = np.concatenate([waist_pos, arm_target])
    waist_set = set(WAIST_JOINTS)
    for i, joint_idx in enumerate(ARM_SDK_JOINTS):
        cmd.motor_cmd[joint_idx].mode = 1
        cmd.motor_cmd[joint_idx].q = float(all_pos[i])
        cmd.motor_cmd[joint_idx].dq = 0.0
        cmd.motor_cmd[joint_idx].tau = 0.0
        if joint_idx in waist_set:
            cmd.motor_cmd[joint_idx].kp = waist_kp
            cmd.motor_cmd[joint_idx].kd = waist_kd
        else:
            cmd.motor_cmd[joint_idx].kp = KP_ARM
            cmd.motor_cmd[joint_idx].kd = KD_ARM

    cmd.crc = robot.crc.Crc(cmd)
    robot.arm_pub.Write(cmd)


def run_test_arm_nudge(robot, waist_mode):
    state = robot.get_state()
    waist_target = select_waist_target(state, waist_mode)
    start_left = state["left_arm"].copy()
    start_right = state["right_arm"].copy()
    target_left = start_left.copy()
    target_right = start_right.copy()
    target_left[3] += TEST_NUDGE_RAD
    target_right[3] += TEST_NUDGE_RAD

    print("[TEST] Sending small arm nudge through rt/arm_sdk.")
    print(f"  Left  start:  {_fmt_deg(start_left)}")
    print(f"  Left  target: {_fmt_deg(target_left)}")
    print(f"  Right start:  {_fmt_deg(start_right)}")
    print(f"  Right target: {_fmt_deg(target_right)}")
    try:
        input("  Press Enter to send test nudge, or Ctrl+C to cancel: ")
    except EOFError:
        print("No interactive input available; cancelling test nudge.")
        return

    t0 = time.time()
    while time.time() - t0 < 1.5:
        current = robot.get_state()
        send_bimanual_arm_cmd(
            robot,
            current,
            np.concatenate([target_left, target_right]),
            waist_target,
            waist_mode,
        )
        time.sleep(CONTROL_DT)

    after = robot.get_state()
    print(f"  Left  after:  {_fmt_deg(after['left_arm'])}")
    print(f"  Right after:  {_fmt_deg(after['right_arm'])}")
    print("  If after did not change, arm_sdk is not taking control.")


def get_exit_arm_pose(exit_pose):
    if exit_pose == "default":
        return RELAXED_ARM_Q.copy()
    if exit_pose == "spread":
        return SPREAD_ARM_Q.copy()
    if exit_pose == "hold":
        return None
    raise ValueError(f"Unknown exit pose: {exit_pose}")


def return_arms_on_exit(robot, waist_mode, exit_pose):
    target = get_exit_arm_pose(exit_pose)
    if target is None:
        print("[EXIT] Holding current arm pose.")
        return

    state = robot.get_state()
    start = np.concatenate([state["left_arm"], state["right_arm"]]).astype(
        np.float32
    )
    waist_target = select_waist_target(state, waist_mode)
    max_dist = float(np.max(np.abs(target - start)))
    duration = float(np.clip(max_dist / 0.35, 2.0, 7.0))
    print(f"[EXIT] Returning arms to {exit_pose} pose ({duration:.1f}s)...")
    print(f"  Start:  {_fmt_deg(start)}")
    print(f"  Target: {_fmt_deg(target)}")

    t0 = time.time()
    while time.time() - t0 < duration:
        alpha = min((time.time() - t0) / duration, 1.0)
        alpha = alpha * alpha * (3 - 2 * alpha)
        arm = start + alpha * (target - start)
        current = robot.get_state()
        send_bimanual_arm_cmd(robot, current, arm, waist_target, waist_mode)
        time.sleep(CONTROL_DT)

    # Hold the final pose briefly so the robot settles before the process exits.
    settle_until = time.time() + 0.5
    while time.time() < settle_until:
        current = robot.get_state()
        send_bimanual_arm_cmd(robot, current, target, waist_target, waist_mode)
        time.sleep(CONTROL_DT)
    print("[EXIT] Arm return complete.")


def open_hands(robot, duration=0.8):
    print("[HAND] Opening Dex3 hands.")
    deadline = time.time() + duration
    while time.time() < deadline:
        robot.send_hand_cmd(OPEN_HAND_Q, OPEN_HAND_Q)
        time.sleep(CONTROL_DT)


def ramp_to_preparation_pose(robot, waist_mode, arm_side):
    state = robot.get_state()
    start = np.concatenate([state["left_arm"], state["right_arm"]]).astype(
        np.float32
    )
    target = PREPARE_ARM_Q.copy()
    if arm_side == "left":
        target[7:14] = start[7:14]
    waist_target = select_waist_target(state, waist_mode)
    max_dist = float(np.max(np.abs(target - start)))
    duration = float(np.clip(max_dist / 0.35, 2.0, 6.0))
    print(f"[PREP] Moving to preparation pose ({duration:.1f}s)...")
    print(f"  Start:  {_fmt_deg(start)}")
    print(f"  Target: {_fmt_deg(target)}")

    t0 = time.time()
    while time.time() - t0 < duration:
        alpha = min((time.time() - t0) / duration, 1.0)
        alpha = alpha * alpha * (3 - 2 * alpha)
        arm = start + alpha * (target - start)
        current = robot.get_state()
        send_bimanual_arm_cmd(robot, current, arm, waist_target, waist_mode)
        time.sleep(CONTROL_DT)

    settle_until = time.time() + 0.5
    while time.time() < settle_until:
        current = robot.get_state()
        send_bimanual_arm_cmd(robot, current, target, waist_target, waist_mode)
        time.sleep(CONTROL_DT)
    open_hands(robot)
    print("[PREP] Preparation pose complete.")
    return target


def run_control_loop(args, robot, action_buffer):
    dt = 1.0 / FREQ_CTRL
    previous_version = -1
    state = robot.get_state()
    target_arm = current_dual_arm(robot)
    waist_target = select_waist_target(state, args.waist_mode)
    target_hand = None

    print("[CTRL] RTC control loop started.")
    print(
        f"[CTRL] arm_side={args.arm_side}, action_mode={args.action_mode}, "
        f"execute={args.execute}"
    )
    print(
        f"[CTRL] waist_mode={args.waist_mode}, "
        f"waist_target={_fmt_deg(waist_target)}"
    )
    print("[CTRL] torso/base action[28:31] is ignored.")
    if args.execute and not args.continuous:
        print("[CTRL] Step mode: Enter=execute, s=skip, c=continuous, q=quit.")
        print(f"[CTRL] Each approved step is held for {args.step_seconds:.1f}s.")
    next_tick = time.perf_counter()
    hold_until = 0.0

    while True:
        action, version = action_buffer.read()
        state = robot.get_state()
        current_arm = np.concatenate(
            [state["left_arm"], state["right_arm"]]
        ).astype(np.float32)

        if action is not None and version != previous_version:
            if args.execute and not args.continuous and time.time() < hold_until:
                target_arm, target_hand = decode_policy_action(
                    action,
                    current_arm,
                    target_arm,
                    args.action_mode,
                    args.arm_side,
                )
                previous_version = version
            else:
                proposed_arm, proposed_hand = decode_policy_action(
                    action,
                    current_arm,
                    target_arm,
                    args.action_mode,
                    args.arm_side,
                )
                previous_version = version
                delta = proposed_arm - current_arm
                print("-" * 60)
                print(f"[CTRL] action version={version}")
                print(f"  Arm now:    {_fmt_deg(current_arm)}")
                print(f"  Arm target: {_fmt_deg(proposed_arm)}")
                print(f"  Arm delta:  {_fmt_deg(delta)}")
                if args.send_hands and proposed_hand is not None:
                    print(f"  Hand target: {_fmt_rad(proposed_hand)}")

                should_execute = True
                if args.execute and not args.continuous:
                    try:
                        user_input = input("  Execute? [Enter/s/c/q]: ").strip().lower()
                    except EOFError:
                        print("No interactive input available; exiting safely.")
                        raise KeyboardInterrupt
                    if user_input == "q":
                        raise KeyboardInterrupt
                    if user_input == "s":
                        should_execute = False
                    elif user_input == "c":
                        args.continuous = True
                    elif user_input != "":
                        print(f"  Unknown input '{user_input}', skipping.")
                        should_execute = False

                if should_execute:
                    target_arm = proposed_arm
                    target_hand = proposed_hand
                    if args.execute and not args.continuous:
                        hold_until = time.time() + args.step_seconds
                else:
                    print("  Skipped. Holding previous arm target.")

        if args.execute:
            send_bimanual_arm_cmd(
                robot,
                state,
                target_arm,
                waist_target,
                args.waist_mode,
            )
            if args.send_hands and target_hand is not None:
                robot.send_hand_cmd(
                    target_hand[:N_HAND_MOTORS],
                    target_hand[N_HAND_MOTORS:2 * N_HAND_MOTORS],
                )

        next_tick += dt
        sleep_time = next_tick - time.perf_counter()
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            next_tick = time.perf_counter()


def get_camera_frame(camera):
    frame = camera.get_frame()
    if frame is None:
        return np.zeros((480, 640, 3), dtype=np.uint8)
    return frame


def run_ui_control_loop(
    args,
    robot,
    action_buffer,
    ui_state,
    ui_lock,
    stop_event,
):
    dt = 1.0 / FREQ_CTRL
    previous_version = -1
    state = robot.get_state()
    target_arm = current_dual_arm(robot)
    waist_target = select_waist_target(state, args.waist_mode)
    target_hand = None
    next_tick = time.perf_counter()

    while True:
        with ui_lock:
            if ui_state["shutdown"]:
                return
            paused = ui_state["paused"]
            manual_arm_target = ui_state.get("manual_arm_target")

        if manual_arm_target is not None:
            target_arm = np.asarray(manual_arm_target, dtype=np.float32)
            with ui_lock:
                ui_state["manual_arm_target"] = None

        if stop_event.is_set() or paused:
            time.sleep(0.05)
            next_tick = time.perf_counter()
            continue

        action, version = action_buffer.read()
        state = robot.get_state()
        if state is None:
            time.sleep(0.05)
            continue
        current_arm = np.concatenate(
            [state["left_arm"], state["right_arm"]]
        ).astype(np.float32)

        if action is not None and version != previous_version:
            proposed_arm, proposed_hand = decode_policy_action(
                action,
                current_arm,
                target_arm,
                args.action_mode,
                args.arm_side,
            )
            previous_version = version
            delta = proposed_arm - current_arm
            with ui_lock:
                approve_budget = int(ui_state["approve_budget"])
                continuous = bool(ui_state["continuous"])
                should_execute = continuous or approve_budget > 0
                if approve_budget > 0:
                    ui_state["approve_budget"] = approve_budget - 1
                ui_state["latest_version"] = int(version)
                ui_state["last_preview"] = {
                    "version": int(version),
                    "arm_now_deg": np.rad2deg(current_arm).round(2).tolist(),
                    "arm_target_deg": (
                        np.rad2deg(proposed_arm).round(2).tolist()
                    ),
                    "arm_delta_deg": np.rad2deg(delta).round(2).tolist(),
                    "hand_target": np.round(proposed_hand, 3).tolist(),
                    "action_mode": args.action_mode,
                    "arm_side": args.arm_side,
                }

            if should_execute:
                target_arm = proposed_arm
                target_hand = proposed_hand
                with ui_lock:
                    ui_state["executed_versions"].append(int(version))
                    ui_state["message"] = (
                        f"Approved and executed RTC version {version}."
                    )
            else:
                with ui_lock:
                    ui_state["message"] = (
                        f"Previewed RTC version {version}; waiting for approval."
                    )

        if args.execute:
            send_bimanual_arm_cmd(
                robot,
                state,
                target_arm,
                waist_target,
                args.waist_mode,
            )
            if args.send_hands and target_hand is not None:
                robot.send_hand_cmd(
                    target_hand[:N_HAND_MOTORS],
                    target_hand[N_HAND_MOTORS:2 * N_HAND_MOTORS],
                )

        next_tick += dt
        sleep_time = next_tick - time.perf_counter()
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            next_tick = time.perf_counter()


def run_control_ui(args, robot, camera, action_buffer, ws_client):
    ui_lock = threading.Lock()
    stop_event = threading.Event()
    task_presets = load_task_presets(args.task)
    selected_task = next(
        (
            task
            for task in task_presets
            if task["prompt"] == args.task
            or task["name"] == args.task
            or task["label"] == args.task
        ),
        None,
    )
    if selected_task is None:
        selected_task = next(
            (
                task
                for task in task_presets
                if "place" in task["name"] and "paper_box" in task["name"]
            ),
            task_presets[0],
        )
    ws_client.set_task(selected_task["prompt"])
    ui_state = {
        "mode": "EXECUTE" if args.execute else "DRY RUN",
        "message": "Psi0 RTC UI started; waiting for server action.",
        "paused": False,
        "shutdown": False,
        "continuous": bool(args.continuous),
        "approve_budget": 0,
        "latest_version": -1,
        "executed_versions": [],
        "last_preview": None,
        "server": f"ws://{args.host}:{args.port}/ws",
        "task_name": selected_task["name"],
        "task_label": selected_task["label"],
        "language_prompt": selected_task["prompt"],
        "arm_side": args.arm_side,
        "action_mode": args.action_mode,
        "approval_steps": args.approval_steps,
        "waist_mode": args.waist_mode,
        "send_hands": args.send_hands,
        "manual_arm_target": None,
    }

    ctrl_thread = threading.Thread(
        target=run_ui_control_loop,
        args=(args, robot, action_buffer, ui_state, ui_lock, stop_event),
        daemon=True,
    )
    ctrl_thread.start()

    app = FastAPI()

    def update_state(**kwargs):
        with ui_lock:
            ui_state.update(kwargs)

    @app.get("/", response_class=HTMLResponse)
    def index():
        return """
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <title>G1 Psi0 RTC Control</title>
  <style>
    body { font-family: sans-serif; margin: 18px; background: #111; color: #eee; }
    .row { display: flex; gap: 16px; align-items: flex-start; }
    img { width: 720px; max-width: 70vw; border: 2px solid #444; }
    button { font-size: 18px; padding: 14px 18px; margin: 8px 0; width: 300px; }
    h3 { margin: 0 0 8px 0; }
    .button-block { border: 1px solid #444; border-radius: 8px; padding: 12px; margin-bottom: 14px; }
    .hint { margin: 4px 0 10px 0; color: #bbb; font-size: 14px; }
    #step { background: #1f7a1f; color: white; }
    #prepare { background: #0f766e; color: white; }
    #start-rtc { background: #15803d; color: white; font-weight: bold; }
    #pause-rtc { background: #1b5f9c; color: white; }
    #run20 { background: #d97706; color: white; font-weight: bold; }
    #stop { background: #a00000; color: white; }
    #resume { background: #1b5f9c; color: white; }
    pre { white-space: pre-wrap; background: #222; padding: 12px; }
  </style>
</head>
<body>
  <h2>G1 Psi0 RTC Control</h2>
  <div class="row">
    <img src="/video" />
    <div>
      <div class="button-block">
        <h3>Start</h3>
        <p class="hint">
          Approve latest RTC chunks as they arrive from Psi0. In left-only mode,
          only the left arm/hand follows policy; the right arm stays relaxed.
        </p>
        <label for="task-select">Task preset</label><br/>
        <select id="task-select" onchange="setTask(this.value)"></select>
        <p class="hint">Task name: <code id="task-name">loading</code></p>
        <p class="hint">Language prompt: <code id="task-prompt">loading</code></p>
        <button id="prepare" onclick="post('/prepare')">
          Preparation: Move To Ready Pose
        </button><br/>
        <button id="start-rtc" onclick="confirmPost('/start-rtc')">
          Start RTC Streaming
        </button><br/>
        <button id="pause-rtc" onclick="post('/pause-rtc')">
          Pause RTC Streaming
        </button><br/>
        <button id="step" onclick="post('/step')">Run Next RTC Chunk</button><br/>
        <button id="run20" onclick="confirmPost('/run-20')">
          ATTENTION/CAUTIOUS: Run 20 RTC Actions
        </button>
      </div>
      <div class="button-block">
        <h3>Maintenance</h3>
        <button id="stop" onclick="post('/stop')">Relax Arms</button><br/>
        <button id="resume" onclick="post('/resume')">
          Resume Approval Mode
        </button>
      </div>
      <pre id="status">Loading...</pre>
    </div>
  </div>
  <script>
    async function post(path) {
      const res = await fetch(path, {method: 'POST'});
      document.getElementById('status').textContent =
        JSON.stringify(await res.json(), null, 2);
    }
    async function setTask(name) {
      const res = await fetch('/task', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({name})
      });
      const payload = await res.json();
      renderTask(payload);
      document.getElementById('status').textContent =
        JSON.stringify(payload, null, 2);
    }
    function renderTask(payload) {
      if (payload.task_name) {
        document.getElementById('task-name').textContent = payload.task_name;
      }
      if (payload.language_prompt) {
        document.getElementById('task-prompt').textContent =
          payload.language_prompt;
      }
    }
    async function loadTasks() {
      const res = await fetch('/tasks');
      const payload = await res.json();
      const select = document.getElementById('task-select');
      select.innerHTML = '';
      for (const task of payload.tasks) {
        const option = document.createElement('option');
        option.value = task.name;
        option.textContent = task.label;
        option.selected = task.name === payload.current.task_name;
        select.appendChild(option);
      }
      renderTask(payload.current);
    }
    async function confirmPost(path) {
      const ok = confirm(
        'ATTENTION/CAUTIOUS: this will execute robot actions. Continue?'
      );
      if (ok) { await post(path); }
    }
    async function refresh() {
      const res = await fetch('/status');
      const payload = await res.json();
      payload.browser_status_refresh = new Date().toLocaleTimeString();
      document.getElementById('status').textContent =
        JSON.stringify(payload, null, 2);
    }
    setInterval(refresh, 1000);
    loadTasks();
    refresh();
  </script>
</body>
</html>
"""

    @app.get("/status")
    def status():
        with ui_lock:
            return dict(ui_state)

    @app.get("/tasks")
    def tasks():
        with ui_lock:
            current = dict(ui_state)
        return {"tasks": task_presets, "current": current}

    @app.post("/task")
    def set_task(payload: dict):
        name = str(payload.get("name", ""))
        selected = next(
            (task for task in task_presets if task["name"] == name),
            None,
        )
        if selected is None:
            return {"ok": False, "message": f"Unknown task preset: {name}"}
        ws_client.set_task(selected["prompt"])
        update_state(
            task_name=selected["name"],
            task_label=selected["label"],
            language_prompt=selected["prompt"],
            message=(
                "Updated Psi0 instruction. New observations will use "
                f"{selected['name']}."
            ),
        )
        with ui_lock:
            return dict(ui_state)

    @app.post("/step")
    def step():
        stop_event.clear()
        with ui_lock:
            ui_state["paused"] = False
            ui_state["continuous"] = False
            ui_state["approve_budget"] = (
                int(ui_state["approve_budget"]) + int(args.approval_steps)
            )
            ui_state["message"] = (
                f"Approved next {args.approval_steps} RTC ticks."
            )
            return dict(ui_state)

    @app.post("/start-rtc")
    def start_rtc():
        stop_event.clear()
        with ui_lock:
            ui_state["paused"] = False
            ui_state["continuous"] = True
            ui_state["approve_budget"] = 0
            ui_state["message"] = (
                "RTC streaming started. Each incoming RTC tick will execute "
                "until Pause RTC Streaming or Relax Arms is pressed."
            )
            return dict(ui_state)

    @app.post("/pause-rtc")
    def pause_rtc():
        stop_event.set()
        with ui_lock:
            ui_state["paused"] = True
            ui_state["continuous"] = False
            ui_state["approve_budget"] = 0
            ui_state["message"] = (
                "RTC streaming paused. No actions are approved."
            )
            return dict(ui_state)

    @app.post("/prepare")
    def prepare():
        stop_event.set()
        update_state(
            paused=True,
            approve_budget=0,
            message="Preparation requested; moving to ready pose and opening hands.",
        )
        if args.execute:
            target = ramp_to_preparation_pose(
                robot,
                args.waist_mode,
                args.arm_side,
            )
            stop_event.clear()
            update_state(
                paused=False,
                manual_arm_target=np.round(target, 6).tolist(),
                message=(
                    "Preparation complete; hands opened; waiting for RTC "
                    "action approval."
                ),
            )
        else:
            stop_event.clear()
            update_state(
                paused=False,
                manual_arm_target=np.round(PREPARE_ARM_Q, 6).tolist(),
                message="Dry-run preparation complete.",
            )
        with ui_lock:
            return dict(ui_state)

    @app.post("/run-20")
    def run_20():
        stop_event.clear()
        with ui_lock:
            ui_state["paused"] = False
            ui_state["continuous"] = False
            ui_state["approve_budget"] = int(ui_state["approve_budget"]) + 20
            ui_state["message"] = "Approved 20 upcoming RTC actions."
            return dict(ui_state)

    @app.post("/resume")
    def resume():
        stop_event.clear()
        with ui_lock:
            ui_state["paused"] = False
            ui_state["continuous"] = False
            ui_state["approve_budget"] = 0
            ui_state["message"] = (
                "Resumed approval mode; no action approved. "
                "Press Run Next RTC Chunk to execute one chunk."
            )
            return dict(ui_state)

    @app.post("/stop")
    def stop():
        stop_event.set()
        update_state(
            paused=True,
            continuous=False,
            approve_budget=0,
            message="Relax requested; moving arms to relaxed pose.",
        )
        if args.execute:
            return_arms_on_exit(robot, args.waist_mode, args.exit_pose)
            open_hands(robot)
        with ui_lock:
            ui_state["message"] = "Relaxed arms and opened hands. Control loop is paused."
            return dict(ui_state)

    def video_stream():
        while True:
            frame = get_camera_frame(camera)
            bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            with ui_lock:
                latest = ui_state["latest_version"]
                mode = ui_state["mode"]
            cv2.putText(
                bgr,
                f"Psi0 RTC {mode} | version {latest}",
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
            time.sleep(0.05)

    @app.get("/video")
    def video():
        return StreamingResponse(
            video_stream(),
            media_type="multipart/x-mixed-replace; boundary=frame",
        )

    try:
        uvicorn.run(app, host=args.ui_host, port=args.ui_port, log_level="info")
    finally:
        with ui_lock:
            ui_state["shutdown"] = True
        stop_event.set()
        ctrl_thread.join(timeout=2.0)


def main():
    parser = argparse.ArgumentParser(
        description="Psi0 upstream RTC client with bimanual-only execution"
    )
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--task", default=DEFAULT_TASK)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--send-hands", action="store_true")
    parser.add_argument("--allow-competing-control", action="store_true",
                        help="Do not abort when XR teleop control is running")
    parser.add_argument("--test-arm-nudge", action="store_true",
                        help="Send a tiny diagnostic arm movement and exit")
    parser.add_argument("--waist-mode",
                        choices=["xr-upright", "passive", "current", "upright"],
                        default="xr-upright",
                        help="xr-upright matches XR teleop waist PD")
    parser.add_argument("--continuous", action="store_true",
                        help="Skip Enter approval and execute actions as they arrive")
    parser.add_argument("--ui", action="store_true",
                        help="Open a local web UI with camera and RTC controls")
    parser.add_argument("--ui-host", default="127.0.0.1")
    parser.add_argument("--ui-port", type=int, default=8040)
    parser.add_argument("--step-seconds", type=float,
                        default=DEFAULT_STEP_EXEC_SECONDS,
                        help="How long to hold each approved step before prompting again")
    parser.add_argument("--exit-pose",
                        choices=["default", "spread", "hold"],
                        default="default",
                        help="Arm pose on q/Ctrl+C: default, spread, or hold")
    parser.add_argument("--verbose", action="store_true",
                        help="Print every WebSocket action receive event")
    parser.add_argument("--action-mode",
                        choices=["delta", "absolute"],
                        default="delta",
                        help="Psi0 release config uses delta actions")
    parser.add_argument("--arm-side",
                        choices=["left", "bimanual"],
                        default="left",
                        help="left executes only left arm/hand and relaxes right")
    parser.add_argument("--approval-steps", type=int,
                        default=DEFAULT_UI_APPROVAL_STEPS,
                        help="RTC ticks approved by one UI Run Next click")
    args = parser.parse_args()

    os.environ.setdefault("NO_PROXY", "localhost,127.0.0.1")
    os.environ.setdefault("no_proxy", "localhost,127.0.0.1")

    ChannelFactoryInitialize(0)

    if args.execute and not args.allow_competing_control:
        competing = find_competing_teleop_processes()
        if competing:
            print("Aborting: XR teleop is still controlling the robot:")
            for line in competing:
                print(f"  {line}")
            print("Stop Teleop from the Gradio panel, or stop those processes, "
                  "then run rtc-bimanual again.")
            sys.exit(1)

    if args.execute and not ensure_ai_mode():
        print("Aborting: balance controller is not active.")
        sys.exit(1)

    robot = G1Robot()
    robot.init()
    if not robot.wait_for_state():
        print("ERROR: No robot state received.")
        sys.exit(1)

    if args.test_arm_nudge:
        run_test_arm_nudge(robot, args.waist_mode)
        return

    camera = CameraReceiver()
    camera.start()

    action_buffer = LatestActionBuffer()
    server_url = f"ws://{args.host}:{args.port}/ws"
    ws_client = Psi0RTCWebSocketClient(
        server_url,
        args.task,
        camera,
        robot,
        action_buffer,
        verbose=args.verbose,
    )

    ws_thread = threading.Thread(target=ws_client.run, daemon=True)
    ws_thread.start()

    try:
        if args.ui:
            run_control_ui(args, robot, camera, action_buffer, ws_client)
        else:
            run_control_loop(args, robot, action_buffer)
    except KeyboardInterrupt:
        print("\n[MAIN] Stopping bimanual RTC client.")
    finally:
        ws_client.stop()
        if args.execute:
            return_arms_on_exit(robot, args.waist_mode, args.exit_pose)
        print("[MAIN] Done.")


if __name__ == "__main__":
    main()
