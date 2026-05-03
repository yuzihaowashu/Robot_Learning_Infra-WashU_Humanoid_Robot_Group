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

import numpy as np
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
DEFAULT_STEP_EXEC_SECONDS = 1.0
TEST_NUDGE_RAD = 0.08
KP_WAIST_XR = 200.0
KD_WAIST_XR = 5.0
SPREAD_ARM_Q = np.zeros(14, dtype=np.float32)
SPREAD_ARM_Q[1] = 1.5
SPREAD_ARM_Q[8] = -1.5


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
        self._ws = None

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
                    "instruction": self.task,
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


def decode_bimanual_action(action, current_arm, previous_target, action_mode):
    if action.ndim == 2:
        action = action[0]
    if action.shape[0] < 28:
        raise ValueError(f"Expected Psi0 action dim >= 28, got {action.shape}")

    hand = action[:14].copy()
    arm = action[14:28].copy()
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
    right_target = G1Adapter._clamp_joints(
        arm_target[7:14],
        RIGHT_ARM_JOINTS,
        current_arm[7:14],
    )
    return np.concatenate([left_target, right_target]), hand


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
        return np.zeros(14, dtype=np.float32)
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


def run_control_loop(args, robot, action_buffer):
    dt = 1.0 / FREQ_CTRL
    previous_version = -1
    state = robot.get_state()
    target_arm = current_dual_arm(robot)
    waist_target = select_waist_target(state, args.waist_mode)
    target_hand = None

    print("[CTRL] Bimanual RTC control loop started.")
    print(f"[CTRL] action_mode={args.action_mode}, execute={args.execute}")
    print(
        f"[CTRL] waist_mode={args.waist_mode}, "
        f"waist_target={_fmt_deg(waist_target)}"
    )
    print("[CTRL] torso/base action[28:36] is ignored.")
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
                target_arm, target_hand = decode_bimanual_action(
                    action,
                    current_arm,
                    target_arm,
                    args.action_mode,
                )
                previous_version = version
            else:
                proposed_arm, proposed_hand = decode_bimanual_action(
                    action,
                    current_arm,
                    target_arm,
                    args.action_mode,
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
