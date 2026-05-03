#!/usr/bin/env python3
"""
Safety-first Psi0 client for Unitree G1.

This client talks to Psi0's HTTP /act endpoint and reuses the local G1
safety wrapper from utils/vla_client.py instead of running Psi0's raw
real-world deployment client directly.
"""

import argparse
import base64
import sys
import time

import numpy as np
import requests
from numpy.lib.format import descr_to_dtype, dtype_to_descr

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

from vla_client import (
    CONTROL_DT,
    KP_ARM,
    KD_ARM,
    LEFT_ARM_JOINTS,
    RIGHT_ARM_JOINTS,
    N_HAND_MOTORS,
    CameraReceiver,
    G1Adapter,
    G1Robot,
    GravityCompensator,
    ensure_ai_mode,
    _fmt_deg,
    _fmt_rad,
    _hold_position,
)


DEFAULT_SERVER_URL = "http://localhost:22085/act"
DEFAULT_TASK = "g1/Pick_bottle_and_turn_and_pour_into_cup"
WAIST_MODE_UPRIGHT = "upright"
WAIST_MODE_CURRENT = "current"


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


class Psi0HttpPolicyClient:
    def __init__(self, server_url, timeout=30.0):
        self.server_url = server_url
        self.timeout = timeout
        self.session = requests.Session()

    def health(self):
        health_url = self.server_url.rsplit("/", 1)[0] + "/health"
        try:
            resp = self.session.get(health_url, timeout=3.0)
            return resp.ok
        except requests.RequestException:
            return False

    def get_action(self, state_dict, frame, task):
        if frame is None:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)

        hand_state = np.concatenate(
            [state_dict["left_hand"], state_dict["right_hand"]]
        ).astype(np.float32)
        arm_state = np.concatenate(
            [state_dict["left_arm"], state_dict["right_arm"]]
        ).astype(np.float32)
        torso_state = np.zeros(4, dtype=np.float32)
        psi0_state = np.concatenate(
            [hand_state, arm_state, torso_state]
        )[None, :]
        if psi0_state.shape[1] < 36:
            psi0_state = np.pad(
                psi0_state,
                ((0, 0), (0, 36 - psi0_state.shape[1])),
            )

        payload = {
            "image": {
                "observation.images.egocentric": frame.astype(np.uint8),
            },
            "state": {
                "states": psi0_state,
            },
            "gt_action": None,
            "dataset_name": None,
            "instruction": task,
            "history": {},
            "condition": {},
            "timestamp": None,
        }
        payload = convert_numpy_in_dict(payload, numpy_serialize)

        resp = self.session.post(
            self.server_url,
            json=payload,
            timeout=self.timeout,
        )
        resp.raise_for_status()
        data = convert_numpy_in_dict(resp.json(), numpy_deserialize)
        if isinstance(data, str):
            raise RuntimeError(data)
        action = data.get("action")
        if action is None:
            raise RuntimeError(f"Psi0 response did not include action: {data}")
        return np.asarray(action, dtype=np.float32)


class Psi0G1Adapter:
    """Decode Psi0's 36D output as bimanual-only G1 targets.

    Layout used here:
      action[0:14]  -> left/right hands
      action[14:28] -> left/right arms
      action[28:36] -> ignored torso/base commands
    """

    def __init__(self, policy_client, action_horizon=8, fixed_waist=None):
        self.policy = policy_client
        self.action_horizon = action_horizon
        self.fixed_waist = fixed_waist.copy() if fixed_waist is not None else None

    def get_action(self, state_dict, frame, task):
        action_chunk = self.policy.get_action(state_dict, frame, task)
        if action_chunk.ndim == 1:
            action_chunk = action_chunk[None, :]
        if action_chunk.ndim != 2 or action_chunk.shape[1] < 28:
            raise ValueError(
                "Expected Psi0 action shape (T, >=28), "
                f"got {action_chunk.shape}"
            )
        if action_chunk.shape[1] > 28:
            print(
                "Bimanual-only mode: using action[0:28] for hands/arms; "
                "ignoring action[28:] torso/base."
            )

        steps = []
        count = min(len(action_chunk), self.action_horizon)
        prev_left_arm = state_dict["left_arm"].copy()
        prev_right_arm = state_dict["right_arm"].copy()

        for t in range(count):
            action = action_chunk[t]
            hand = action[:14]
            arm = action[14:28]

            left_hand = hand[:N_HAND_MOTORS].copy()
            right_hand = hand[N_HAND_MOTORS:2 * N_HAND_MOTORS].copy()
            left_arm = arm[:7].copy()
            right_arm = arm[7:14].copy()

            left_arm = G1Adapter._clamp_joints(
                left_arm,
                LEFT_ARM_JOINTS,
                prev_left_arm,
            )
            right_arm = G1Adapter._clamp_joints(
                right_arm,
                RIGHT_ARM_JOINTS,
                prev_right_arm,
            )
            waist = (
                self.fixed_waist.copy()
                if self.fixed_waist is not None
                else state_dict["waist"].copy()
            )

            steps.append((waist, left_arm, right_arm, left_hand, right_hand))
            prev_left_arm = left_arm.copy()
            prev_right_arm = right_arm.copy()

        return steps


def print_action_summary(step, state, actions, send_hands):
    waist, left_arm, right_arm, left_hand, right_hand = actions[0]
    d_left = left_arm - state["left_arm"]
    d_right = right_arm - state["right_arm"]
    d_waist = waist - state["waist"]
    max_d_left = float(np.max(np.abs(d_left)))
    max_d_right = float(np.max(np.abs(d_right)))

    print(f"  Left  arm now:    {_fmt_deg(state['left_arm'])}")
    print(f"  Left  arm target: {_fmt_deg(left_arm)}")
    print(f"  Left  arm delta:  {_fmt_deg(d_left)}")
    print(f"  Right arm now:    {_fmt_deg(state['right_arm'])}")
    print(f"  Right arm target: {_fmt_deg(right_arm)}")
    print(f"  Right arm delta:  {_fmt_deg(d_right)}")
    print(f"  Waist target:     {_fmt_deg(waist)}")
    print(f"  Waist delta:      {_fmt_deg(d_waist)}  (fixed)")
    if send_hands:
        print(f"  Left  hand target: {_fmt_rad(left_hand)}")
        print(f"  Right hand target: {_fmt_rad(right_hand)}")
    else:
        print("  Hands: not sent by default; pass --send-hands to enable.")

    danger = max(max_d_left, max_d_right) > 0.12
    if danger:
        print(
            "  LARGE MOVE detected: "
            f"L={np.degrees(max_d_left):.1f} deg "
            f"R={np.degrees(max_d_right):.1f} deg"
        )
    return danger


def ramp_arm_control(robot, grav_comp, fixed_waist, duration=2.0):
    print(f"Ramping up arm control ({duration:.1f}s)...")
    t0 = time.time()
    while time.time() - t0 < duration:
        state = robot.get_state()
        alpha = min((time.time() - t0) / duration, 1.0)
        alpha = alpha * alpha * (3 - 2 * alpha)
        robot.send_arm_cmd(
            fixed_waist,
            state["left_arm"],
            state["right_arm"],
            state["_mode_machine"],
            kp=KP_ARM * alpha,
            kd=KD_ARM * alpha,
            grav_comp=grav_comp,
        )
        time.sleep(CONTROL_DT)


def hold_fixed_waist(robot, grav_comp, fixed_waist, duration=0.2):
    """Hold arms at current positions while keeping waist at the start pose."""
    t0 = time.time()
    while time.time() - t0 < duration:
        state = robot.get_state()
        robot.send_arm_cmd(
            fixed_waist,
            state["left_arm"],
            state["right_arm"],
            state["_mode_machine"],
            grav_comp=grav_comp,
        )
        time.sleep(CONTROL_DT)


def return_home(robot, grav_comp, home_state, fixed_waist):
    state = robot.get_state()
    start_left = state["left_arm"].copy()
    start_right = state["right_arm"].copy()
    max_dist = max(
        float(np.max(np.abs(start_left - home_state["left_arm"]))),
        float(np.max(np.abs(start_right - home_state["right_arm"]))),
        0.01,
    )
    duration = float(np.clip(max_dist / 0.3, 2.0, 6.0))
    print(f"Returning arms to start pose ({duration:.1f}s)...")
    t0 = time.time()
    while time.time() - t0 < duration:
        alpha = min((time.time() - t0) / duration, 1.0)
        alpha = alpha * alpha * (3 - 2 * alpha)
        left = start_left + alpha * (home_state["left_arm"] - start_left)
        right = start_right + alpha * (home_state["right_arm"] - start_right)
        current = robot.get_state()
        robot.send_arm_cmd(
            fixed_waist,
            left,
            right,
            current["_mode_machine"],
            grav_comp=grav_comp,
        )
        time.sleep(CONTROL_DT)


def select_fixed_waist(state, waist_mode):
    if waist_mode == WAIST_MODE_UPRIGHT:
        return np.zeros_like(state["waist"], dtype=np.float32)
    if waist_mode == WAIST_MODE_CURRENT:
        return state["waist"].copy()
    raise ValueError(f"Unknown waist mode: {waist_mode}")


def run(args):
    print("=" * 60)
    print("  G1 VLA Client - Psi0")
    print("=" * 60)
    print(f"  Server: {args.server_url}")
    print(f"  Task: {args.task}")
    print("  Action mode: bimanual only; torso/base outputs are ignored")
    print(f"  Waist mode: fixed {args.waist_mode}; Unitree ai keeps balance")
    print(f"  Execute robot commands: {args.execute}")
    print(f"  Send hands: {args.send_hands}")
    print()

    ChannelFactoryInitialize(0)

    if args.execute and not ensure_ai_mode():
        print("Aborting: balance controller is not active.")
        sys.exit(1)

    grav_comp = GravityCompensator()
    robot = G1Robot()
    robot.init()
    print("Waiting for robot state...")
    if not robot.wait_for_state():
        print("ERROR: No robot state received.")
        sys.exit(1)

    camera = CameraReceiver()
    camera.start()
    time.sleep(1.0)

    policy = Psi0HttpPolicyClient(args.server_url, timeout=args.timeout)
    if not policy.health():
        print("ERROR: Cannot reach Psi0 server health endpoint.")
        print("Start it first, for example:")
        print("  bash run_psi0.sh server /path/to/run 100000")
        sys.exit(1)

    home_state = robot.get_state()
    fixed_waist = select_fixed_waist(home_state, args.waist_mode)
    print(f"Initial waist:      {_fmt_deg(home_state['waist'])}")
    print(f"Fixed waist target: {_fmt_deg(fixed_waist)}")
    adapter = Psi0G1Adapter(
        policy,
        action_horizon=args.action_horizon,
        fixed_waist=fixed_waist,
    )

    if args.execute:
        ramp_arm_control(robot, grav_comp, fixed_waist)
    else:
        print("Dry-run mode: actions will be printed but not sent.")

    step_count = 0
    continuous = args.continuous
    try:
        while True:
            if args.execute:
                hold_fixed_waist(robot, grav_comp, fixed_waist, duration=0.2)

            state = robot.get_state()
            frame = camera.get_frame()
            start = time.time()
            actions = adapter.get_action(state, frame, args.task)
            elapsed = time.time() - start

            print("-" * 60)
            print(
                f"[Step {step_count}] Psi0 inference: {elapsed:.3f}s, "
                f"{len(actions)} sub-steps"
            )
            print_action_summary(step_count, state, actions, args.send_hands)

            if args.execute and not continuous:
                prompt = "  Execute? [Enter/s/c/q]: "
                try:
                    user_input = input(prompt).strip().lower()
                except EOFError:
                    print("No interactive input available; exiting safely.")
                    break
                if user_input == "q":
                    break
                if user_input == "s":
                    step_count += 1
                    continue
                if user_input == "c":
                    continuous = True
                elif user_input != "":
                    print(f"  Unknown input '{user_input}', skipping.")
                    step_count += 1
                    continue

            for _waist, left_arm, right_arm, left_hand, right_hand in actions:
                start = time.time()
                if args.execute:
                    current = robot.get_state()
                    robot.send_arm_cmd(
                        fixed_waist,
                        left_arm,
                        right_arm,
                        current["_mode_machine"],
                        grav_comp=grav_comp,
                    )
                    if args.send_hands:
                        robot.send_hand_cmd(left_hand, right_hand)
                dt = time.time() - start
                if dt < CONTROL_DT:
                    time.sleep(CONTROL_DT - dt)

            step_count += 1

            if not args.execute and step_count >= args.dry_run_steps:
                print("Dry-run step limit reached.")
                break

    except KeyboardInterrupt:
        print("\nStopping Psi0 client.")
    finally:
        if args.execute:
            return_home(robot, grav_comp, home_state, fixed_waist)
        print("Done.")


def main():
    parser = argparse.ArgumentParser(description="Safety-first Psi0 G1 client")
    parser.add_argument("--server-url", default=DEFAULT_SERVER_URL)
    parser.add_argument("--task", default=DEFAULT_TASK)
    parser.add_argument("--action-horizon", type=int, default=8)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--continuous", action="store_true")
    parser.add_argument("--waist-mode",
                        choices=[WAIST_MODE_UPRIGHT, WAIST_MODE_CURRENT],
                        default=WAIST_MODE_UPRIGHT,
                        help="upright fixes waist at [0,0,0]; current holds "
                             "the startup waist pose")
    parser.add_argument("--execute", action="store_true",
                        help="Actually send arm commands to the robot")
    parser.add_argument("--send-hands", action="store_true",
                        help="Also send Psi0 hand targets. Off by default.")
    parser.add_argument("--dry-run-steps", type=int, default=3,
                        help="Inference steps to print without --execute")
    args = parser.parse_args()
    run(args)


if __name__ == "__main__":
    main()
