#!/usr/bin/env python3
"""Interactive Dex3-1 hand test — both hands, per-finger control.

Usage:
    conda activate tv
    python test_both_hands.py

Commands (type and press Enter):
    open        — open both hands (all q=0)
    close       — close both hands (full CLOSE_Q)
    left open   — open left hand only
    left close  — close left hand only
    right open  — open right hand only
    right close — close right hand only
    lt          — close left thumb only
    lm          — close left middle only
    li          — close left index only
    rt          — close right thumb only
    ri          — close right index only
    rm          — close right middle only
    status      — print current motor state for both hands
    quit / q    — exit
"""
import time
import sys
import threading
import numpy as np
from unitree_sdk2py.core.channel import (
    ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_

MOTOR_NUM = 7
KP = 1.0
KD = 0.3
CMD_HZ = 100

LEFT_CLOSE  = np.array([ 0.0,  1.0,  1.74, -1.57, -1.74, -1.57, -1.74])
RIGHT_CLOSE = np.array([ 0.0, -1.0, -1.74,  1.57,  1.74,  1.57,  1.74])
OPEN_Q = np.zeros(MOTOR_NUM)

# Joint name mapping: [thumb0, thumb1, thumb2, mid0/idx0, mid1/idx1, idx0/mid0, idx1/mid1]
LEFT_NAMES  = ['L_thumb0', 'L_thumb1', 'L_thumb2', 'L_mid0', 'L_mid1', 'L_idx0', 'L_idx1']
RIGHT_NAMES = ['R_thumb0', 'R_thumb1', 'R_thumb2', 'R_idx0', 'R_idx1', 'R_mid0', 'R_mid1']

FINGER_MAP = {
    'lt': ('left',  [1, 2],    'left thumb'),
    'lm': ('left',  [3, 4],    'left middle'),
    'li': ('left',  [5, 6],    'left index'),
    'rt': ('right', [1, 2],    'right thumb'),
    'ri': ('right', [3, 4],    'right index'),
    'rm': ('right', [5, 6],    'right middle'),
}

left_state = None
right_state = None
left_target = np.zeros(MOTOR_NUM)
right_target = np.zeros(MOTOR_NUM)
running = True


def _make_mode(motor_id, status=0x01, timeout=0):
    return (motor_id & 0x0F) | ((status & 0x07) << 4) | ((timeout & 0x01) << 7)


MODE_LUT = [_make_mode(i) for i in range(MOTOR_NUM)]


def state_callback_left(msg):
    global left_state
    left_state = msg


def state_callback_right(msg):
    global right_state
    right_state = msg


def build_cmd(target_q):
    cmd = unitree_hg_msg_dds__HandCmd_()
    for i in range(MOTOR_NUM):
        cmd.motor_cmd[i].mode = MODE_LUT[i]
        cmd.motor_cmd[i].q = float(target_q[i])
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].tau = 0.0
        cmd.motor_cmd[i].kp = KP
        cmd.motor_cmd[i].kd = KD
    return cmd


def publish_loop(left_pub, right_pub):
    while running:
        left_pub.Write(build_cmd(left_target))
        right_pub.Write(build_cmd(right_target))
        time.sleep(1.0 / CMD_HZ)


def print_status():
    print("\n===== Hand Motor Status =====")
    for side, names, state_msg in [
        ('LEFT', LEFT_NAMES, left_state),
        ('RIGHT', RIGHT_NAMES, right_state),
    ]:
        tgt = left_target if side == 'LEFT' else right_target
        print(f"\n  {side} HAND:")
        if state_msg is None:
            print("    (no state received)")
            continue
        print(f"    {'Motor':<12} {'cmd':>8} {'state':>8} {'err':>8} {'temp':>8}")
        print(f"    {'-'*48}")
        for i in range(MOTOR_NUM):
            ms = state_msg.motor_state[i]
            q = ms.q
            err = tgt[i] - q
            temp = max(ms.temperature) if hasattr(ms, 'temperature') else 0
            flag = ' !!!' if abs(err) > 0.5 else ''
            print(f"    {names[i]:<12} {tgt[i]:>8.3f} {q:>8.3f} {err:>8.3f} {temp:>7}C{flag}")
    print()


def main():
    global left_target, right_target, running

    ChannelFactoryInitialize(0, "enp2s0")

    left_pub = ChannelPublisher("rt/dex3/left/cmd", HandCmd_)
    left_pub.Init()
    right_pub = ChannelPublisher("rt/dex3/right/cmd", HandCmd_)
    right_pub.Init()

    left_sub = ChannelSubscriber("rt/dex3/left/state", HandState_)
    left_sub.Init(state_callback_left)
    right_sub = ChannelSubscriber("rt/dex3/right/state", HandState_)
    right_sub.Init(state_callback_right)

    pub_thread = threading.Thread(target=publish_loop, args=(left_pub, right_pub), daemon=True)
    pub_thread.start()

    print("Waiting for hand state...")
    for _ in range(100):
        if left_state is not None and right_state is not None:
            break
        time.sleep(0.05)

    if left_state is None or right_state is None:
        print("WARNING: Could not read hand state (left={}, right={})".format(
            left_state is not None, right_state is not None))

    print("\nDex3-1 Hand Test Ready!")
    print("Commands: open, close, left/right open/close, lt/lm/li/rt/ri/rm, status, quit")
    print_status()

    try:
        while True:
            try:
                cmd = input("\n> ").strip().lower()
            except EOFError:
                break

            if cmd in ('q', 'quit', 'exit'):
                break
            elif cmd == 'open':
                left_target = OPEN_Q.copy()
                right_target = OPEN_Q.copy()
                print("Both hands → OPEN")
            elif cmd == 'close':
                left_target = LEFT_CLOSE.copy()
                right_target = RIGHT_CLOSE.copy()
                print("Both hands → CLOSE")
            elif cmd == 'left open':
                left_target = OPEN_Q.copy()
                print("Left hand → OPEN")
            elif cmd == 'left close':
                left_target = LEFT_CLOSE.copy()
                print("Left hand → CLOSE")
            elif cmd == 'right open':
                right_target = OPEN_Q.copy()
                print("Right hand → OPEN")
            elif cmd == 'right close':
                right_target = RIGHT_CLOSE.copy()
                print("Right hand → CLOSE")
            elif cmd in FINGER_MAP:
                side, indices, name = FINGER_MAP[cmd]
                close_q = LEFT_CLOSE if side == 'left' else RIGHT_CLOSE
                tgt = left_target if side == 'left' else right_target
                new_tgt = tgt.copy()
                for idx in indices:
                    new_tgt[idx] = close_q[idx]
                if side == 'left':
                    left_target = new_tgt
                else:
                    right_target = new_tgt
                print(f"{name} → CLOSE (motors {indices})")
            elif cmd == 'status' or cmd == 's':
                pass
            else:
                print(f"Unknown command: '{cmd}'")
                print("Try: open, close, left/right open/close, lt/lm/li/rt/ri/rm, status, quit")
                continue

            time.sleep(0.3)
            print_status()

    except KeyboardInterrupt:
        pass
    finally:
        print("\nOpening both hands before exit...")
        left_target = OPEN_Q.copy()
        right_target = OPEN_Q.copy()
        time.sleep(1.0)
        running = False
        print("Done.")


if __name__ == "__main__":
    main()
