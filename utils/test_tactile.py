#!/usr/bin/env python3
"""
Quick test: read Dex3 hand tactile (pressure) sensors via DDS.

Each hand has 7 PressSensorState_ entries, each with 12 pressure values.
That's 84 pressure points per hand, 168 total.

Usage:
    conda activate lerobot
    python utils/test_tactile.py
"""

import sys
import time
import threading

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_

TOPIC_LEFT = "rt/dex3/left/state"
TOPIC_RIGHT = "rt/dex3/right/state"

left_state = None
right_state = None


def on_left(msg: HandState_):
    global left_state
    left_state = msg


def on_right(msg: HandState_):
    global right_state
    right_state = msg


BASELINE = 30000.0


def print_hand(name, state):
    if state is None:
        print(f"  {name}: (no data)")
        return

    print(f"  {name}:")
    print(f"    Motors: {len(state.motor_state)}, "
          f"Press sensors: {len(state.press_sensor_state)}")

    for i, ps in enumerate(state.press_sensor_state):
        pressures = list(ps.pressure)
        connected = any(abs(p) > 0.01 for p in pressures)
        if not connected:
            print(f"    S[{i}]: (disconnected)")
            continue
        above = [p - BASELINE for p in pressures]
        active_count = sum(1 for a in above if a > 1000)
        pmax = max(pressures)
        delta_max = pmax - BASELINE
        bar = "█" * min(40, int(delta_max / 2000))
        print(f"    S[{i}]: {active_count:2d}/12 hit | "
              f"max={pmax:.0f} (Δ{delta_max:+.0f}) {bar}")

    if hasattr(state, 'imu_state') and state.imu_state:
        imu = state.imu_state
        if hasattr(imu, 'rpy'):
            print(f"    IMU rpy: [{imu.rpy[0]:.2f}, {imu.rpy[1]:.2f}, {imu.rpy[2]:.2f}]")

    print(f"    Power: {state.power_v:.2f}V, {state.power_a:.2f}A")


def main():
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    left_sub = ChannelSubscriber(TOPIC_LEFT, HandState_)
    left_sub.Init(on_left, 10)
    right_sub = ChannelSubscriber(TOPIC_RIGHT, HandState_)
    right_sub.Init(on_right, 10)

    print("Subscribing to Dex3 hand state...")
    print(f"  Left:  {TOPIC_LEFT}")
    print(f"  Right: {TOPIC_RIGHT}")
    print("Waiting for data (Ctrl+C to quit)...\n")

    try:
        while True:
            print("\033[2J\033[H")  # clear screen
            print("=" * 60)
            print("  Dex3 Tactile Sensor Monitor")
            print("=" * 60)
            print()
            print_hand("LEFT HAND", left_state)
            print()
            print_hand("RIGHT HAND", right_state)
            print()
            print("(refreshing every 0.5s, Ctrl+C to quit)")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nDone.")


if __name__ == "__main__":
    main()
