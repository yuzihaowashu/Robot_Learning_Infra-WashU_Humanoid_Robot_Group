#!/usr/bin/env python3
"""Standalone test: send commands to the right Dex3-1 hand and monitor state.

Run on humanoid-pc (directly connected to G1 via Ethernet):
    conda activate tv
    python test_right_hand.py

This test does NOT use multiprocessing — everything runs in a single thread
to eliminate any fork-related DDS issues.
"""
import time
import numpy as np
from unitree_sdk2py.core.channel import (
    ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_

MOTOR_NUM = 7
KP = 1.0
KD = 0.3

RIGHT_OPEN  = [0.0] * MOTOR_NUM
RIGHT_CLOSE = [-0.8, -0.8, -1.2, 1.2, 1.4, 1.2, 1.4]

right_state = None

def _make_mode(motor_id, status=0x01, timeout=0):
    return (motor_id & 0x0F) | ((status & 0x07) << 4) | ((timeout & 0x01) << 7)

def state_handler(msg):
    global right_state
    right_state = msg

def send_cmd(pub, positions):
    cmd = unitree_hg_msg_dds__HandCmd_()
    for i in range(MOTOR_NUM):
        cmd.motor_cmd[i].mode = _make_mode(i)
        cmd.motor_cmd[i].q    = float(positions[i])
        cmd.motor_cmd[i].dq   = 0.0
        cmd.motor_cmd[i].tau  = 0.0
        cmd.motor_cmd[i].kp   = KP
        cmd.motor_cmd[i].kd   = KD
    pub.Write(cmd)

def read_state():
    if right_state is None:
        return None
    return [right_state.motor_state[i].q for i in range(MOTOR_NUM)]

def main():
    ChannelFactoryInitialize(0, "enp2s0")

    pub = ChannelPublisher("rt/dex3/right/cmd", HandCmd_)
    pub.Init()

    sub = ChannelSubscriber("rt/dex3/right/state", HandState_)
    sub.Init(state_handler, 10)

    print("Waiting for right hand state...")
    for _ in range(100):
        if right_state is not None:
            break
        time.sleep(0.05)

    if right_state is None:
        print("ERROR: No right hand state received! Check DDS connection.")
        return

    q0 = read_state()
    print(f"Initial R_state: {[f'{v:.3f}' for v in q0]}")

    print("\n--- Phase 1: Sending OPEN (all zeros) for 3 seconds ---")
    t0 = time.time()
    while time.time() - t0 < 3.0:
        send_cmd(pub, RIGHT_OPEN)
        time.sleep(0.01)
    q1 = read_state()
    print(f"After OPEN:  R_state: {[f'{v:.3f}' for v in q1]}")
    delta1 = np.max(np.abs(np.array(q1) - np.array(q0)))
    print(f"  max delta from initial: {delta1:.4f}")

    print("\n--- Phase 2: Sending CLOSE for 3 seconds ---")
    t0 = time.time()
    while time.time() - t0 < 3.0:
        send_cmd(pub, RIGHT_CLOSE)
        time.sleep(0.01)
    q2 = read_state()
    print(f"After CLOSE: R_state: {[f'{v:.3f}' for v in q2]}")
    delta2 = np.max(np.abs(np.array(q2) - np.array(q1)))
    print(f"  max delta from OPEN: {delta2:.4f}")

    print("\n--- Phase 3: Sending OPEN again for 3 seconds ---")
    t0 = time.time()
    while time.time() - t0 < 3.0:
        send_cmd(pub, RIGHT_OPEN)
        time.sleep(0.01)
    q3 = read_state()
    print(f"After OPEN:  R_state: {[f'{v:.3f}' for v in q3]}")
    delta3 = np.max(np.abs(np.array(q3) - np.array(q2)))
    print(f"  max delta from CLOSE: {delta3:.4f}")

    if delta1 < 0.05 and delta2 < 0.05:
        print("\n*** RIGHT HAND DID NOT RESPOND — likely hardware fault or needs re-init ***")
        print("Try: power cycle the robot, or run Unitree official dex3 example.")
    else:
        print("\n*** RIGHT HAND RESPONDED — the issue is in the teleop code ***")

    print("\n--- Also testing LEFT hand for comparison ---")
    left_state_holder = [None]
    def left_handler(msg):
        left_state_holder[0] = msg

    lpub = ChannelPublisher("rt/dex3/left/cmd", HandCmd_)
    lpub.Init()
    lsub = ChannelSubscriber("rt/dex3/left/state", HandState_)
    lsub.Init(left_handler, 10)
    time.sleep(0.5)
    if left_state_holder[0]:
        lq = [left_state_holder[0].motor_state[i].q for i in range(MOTOR_NUM)]
        print(f"Left  state: {[f'{v:.3f}' for v in lq]}")
    print("Done.")

if __name__ == "__main__":
    main()
