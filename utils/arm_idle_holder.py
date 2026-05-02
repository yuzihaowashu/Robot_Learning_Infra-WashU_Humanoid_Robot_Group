#!/usr/bin/env python3
"""
arm_idle_holder.py — keep G1 arms spread outward whenever no teleop/RL stack
is running, so the Dex3-1 hands cannot collide with the robot's body or thighs.

Why this exists
---------------
The Unitree G1's default Damping/FixStand FSM rests both arms with shoulder
pitch ~0 and shoulder roll ~0, so the Dex3-1 hand fingers physically touch
the robot's outer thighs. Over time this has destroyed multiple Dex3 finger
motors (left thumb 0/1/2, right index 0/1, see todo_docs/dex3_hand_error.md).
Unitree has acknowledged that the resting pose cannot be changed inside the
factory FSM. Their recommended workaround is exactly this: have the PC
continuously override the arm command stream over `rt/arm_sdk` so the arms
stay outward.

This script is the simplest possible "always-on" daemon that does that:

  * Subscribes to `rt/lowstate`, fetches `mode_machine`.
  * Publishes a constant LowCmd to `rt/arm_sdk` at ~50 Hz with
        - shoulder roll spread outward (q[16] = +1.5, q[23] = -1.5),
        - all other waist/arm joints at q=0,
        - kp/kd matching the values used by `xr_teleoperate`'s
          `G1_29_ArmController` (so the handoff is bumpless),
        - `kNotUsedJoint0.q = 1.0`  ←  arm_sdk weight = 1 (full override).

  * Honors a "yield" flag at /tmp/g1_arm_holder_yield.pid:
        - File present AND the PID it contains is alive  →  stop publishing
          (so teleop/RL processes can drive the arms without fighting us).
        - File present but PID dead (crash)              →  remove flag,
          resume publishing.
        - File absent                                    →  hold spread.

  * On SIGTERM/SIGINT it just stops publishing — it does NOT ramp the arm_sdk
    weight back to 0, because doing that would hand the arms back to the
    motion service whose default pose is exactly the unsafe one.  The right
    way to stop the holder is via systemd (`systemctl stop g1-arm-holder`)
    only after teleop is up and overriding `rt/arm_sdk` itself.

Intended deployment
-------------------
  1.  utils/arm_idle_holder.py   (this script)
  2.  utils/g1-arm-holder.service (systemd unit; copy to /etc/systemd/system)
  3.  Boot the robot, wait ~60 s for zero-torque init, press L2+UP on the
      remote to enter Damping/StandUp; the systemd unit will then take over
      and hold the arms outward.

Usage (manual smoke test)
-------------------------
    # in conda env that has unitree_sdk2py
    python utils/arm_idle_holder.py --iface enp2s0 --hold-time 5.0

    # production (run forever, until SIGTERM)
    python utils/arm_idle_holder.py --iface enp2s0
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import time
from enum import IntEnum
from typing import Optional

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (
    LowCmd_ as hg_LowCmd,
    LowState_ as hg_LowState,
)
from unitree_sdk2py.utils.crc import CRC


# ── Topic / index constants (must match xr_teleoperate/.../robot_arm.py) ──
TOPIC_ARM_SDK = "rt/arm_sdk"
TOPIC_LOW_STATE = "rt/lowstate"

NUM_MOTORS = 35  # G1_29_Num_Motors


class JointIdx(IntEnum):
    L_HipPitch = 0
    L_HipRoll = 1
    L_HipYaw = 2
    L_Knee = 3
    L_AnklePitch = 4
    L_AnkleRoll = 5
    R_HipPitch = 6
    R_HipRoll = 7
    R_HipYaw = 8
    R_Knee = 9
    R_AnklePitch = 10
    R_AnkleRoll = 11
    WaistYaw = 12
    WaistRoll = 13
    WaistPitch = 14
    L_ShoulderPitch = 15
    L_ShoulderRoll = 16
    L_ShoulderYaw = 17
    L_Elbow = 18
    L_WristRoll = 19
    L_WristPitch = 20
    L_WristYaw = 21
    R_ShoulderPitch = 22
    R_ShoulderRoll = 23
    R_ShoulderYaw = 24
    R_Elbow = 25
    R_WristRoll = 26
    R_WristPitch = 27
    R_WristYaw = 28
    NotUsedJoint0 = 29  # arm_sdk weight slot (q=1.0 → full override)


# Arm joint indices (15..28) and waist (12..14) — these are the ones we
# actually drive. The 12 leg joints stay locked at their last sampled q.
ARM_INDICES = list(range(15, 29))
WAIST_INDICES = [12, 13, 14]
WRIST_INDICES = {19, 20, 21, 26, 27, 28}

# Gains: identical to G1_29_ArmController so handoff is bumpless.
KP_LOW = 150.0
KD_LOW = 3.5
KP_WRIST = 60.0
KD_WRIST = 2.0
KP_HIGH = 300.0
KD_HIGH = 3.0
KP_WAIST = 200.0
KD_WAIST = 5.0


# ── Yield flag (pid file) ──
# Teleop / RL wrappers should write their PID into this file before they
# start publishing to rt/arm_sdk, and remove it on clean exit. A stale
# PID (process gone) is auto-cleaned by the holder.
YIELD_FLAG_PATH = "/tmp/g1_arm_holder_yield.pid"


def _is_pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        # Process exists but is owned by someone else.
        return True
    return True


def _should_yield() -> bool:
    if not os.path.exists(YIELD_FLAG_PATH):
        return False
    try:
        with open(YIELD_FLAG_PATH, "r") as f:
            pid_str = f.read().strip()
        pid = int(pid_str) if pid_str else 0
    except (OSError, ValueError):
        # Corrupt flag → treat as stale.
        try:
            os.remove(YIELD_FLAG_PATH)
        except OSError:
            pass
        return False
    if pid <= 0 or not _is_pid_alive(pid):
        try:
            os.remove(YIELD_FLAG_PATH)
        except OSError:
            pass
        return False
    return True


# ── Spread target ──
def _kp_kd_for(idx: int) -> tuple[float, float]:
    if idx in WRIST_INDICES:
        return KP_WRIST, KD_WRIST
    if idx in WAIST_INDICES:
        return KP_WAIST, KD_WAIST
    if idx in ARM_INDICES:
        return KP_LOW, KD_LOW
    # Legs etc. — we do NOT actually want to drive them, but kp/kd must be set.
    return KP_HIGH, KD_HIGH


def _spread_target_for(idx: int, spread_l: float, spread_r: float) -> float:
    """Return the *final* spread-pose target for any arm/waist joint."""
    if idx == JointIdx.L_ShoulderRoll:
        return spread_l
    if idx == JointIdx.R_ShoulderRoll:
        return spread_r
    return 0.0


def _build_spread_lowcmd(state: hg_LowState, mode_machine: int,
                          spread_l: float, spread_r: float,
                          start_arm_q: dict[int, float] | None = None,
                          alpha: float = 1.0) -> hg_LowCmd:
    """Build a fresh LowCmd_ that drives only arms+waist to the spread pose.

    Leg joints are commanded to their currently-measured q so we do not
    fight the locomotion controller (kp_high there is a holding torque, not
    a tracking target).

    For the arm/waist joints, the commanded q is a linear blend
        (1 - alpha) * start_arm_q[i]   +   alpha * spread_target[i]
    so the caller can ramp from the arm's *current* pose to the final
    spread pose smoothly during the first few seconds — otherwise the
    controller's PD (kp=150 on shoulders) would yank the arm at full
    torque if it happened to be far from the target at startup.

    Pass alpha=1.0 (default) to command the final pose unconditionally;
    pass alpha in [0,1] together with `start_arm_q` to ramp.
    """
    msg = unitree_hg_msg_dds__LowCmd_()
    msg.mode_pr = 0
    msg.mode_machine = mode_machine

    a = max(0.0, min(1.0, float(alpha)))
    if start_arm_q is None:
        start_arm_q = {}

    for i in range(NUM_MOTORS):
        kp, kd = _kp_kd_for(i)
        msg.motor_cmd[i].kp = kp
        msg.motor_cmd[i].kd = kd
        msg.motor_cmd[i].dq = 0.0
        msg.motor_cmd[i].tau = 0.0
        msg.motor_cmd[i].mode = 1

        if i in ARM_INDICES or i in WAIST_INDICES:
            final = _spread_target_for(i, spread_l, spread_r)
            start = float(start_arm_q.get(i, final))
            msg.motor_cmd[i].q = (1.0 - a) * start + a * final
        elif i < 12:
            # Leg: sample current q so we do not yank legs.
            try:
                msg.motor_cmd[i].q = float(state.motor_state[i].q)
            except Exception:
                msg.motor_cmd[i].q = 0.0
        else:
            msg.motor_cmd[i].q = 0.0

    # arm_sdk weight = 1.0 (this is the magic that overrides the FSM).
    msg.motor_cmd[JointIdx.NotUsedJoint0].q = 1.0
    return msg


# ── Main loop ──
def _wait_for_lowstate(sub: ChannelSubscriber, timeout_s: float
                        ) -> Optional[hg_LowState]:
    deadline = time.time() + timeout_s
    msg = None
    while time.time() < deadline:
        m = sub.Read()
        if m is not None:
            msg = m
            # need at least mode_machine ≠ 0 ideally, but accept any sample
            if getattr(m, "mode_machine", 0) != 0:
                return msg
        time.sleep(0.02)
    return msg


def hold_spread(iface: str, publish_hz: float, spread_l: float,
                spread_r: float, hold_time: float, ramp_time: float) -> int:
    print(f"[arm_idle_holder] starting on iface={iface!r} "
          f"@ {publish_hz:.0f} Hz  (spread L={spread_l:+.2f}, "
          f"R={spread_r:+.2f}, ramp={ramp_time:.1f}s)")
    if hold_time > 0:
        print(f"[arm_idle_holder]   hold-time = {hold_time:.1f}s "
              "then exit (smoke-test mode)")

    ChannelFactoryInitialize(0, iface)

    pub = ChannelPublisher(TOPIC_ARM_SDK, hg_LowCmd)
    pub.Init()

    sub = ChannelSubscriber(TOPIC_LOW_STATE, hg_LowState)
    sub.Init(handler=None, queueLen=10)

    state = _wait_for_lowstate(sub, timeout_s=8.0)
    if state is None:
        print("[arm_idle_holder] FAIL: no rt/lowstate received in 8 s.")
        print("                 Robot off / wrong iface / cable issue?")
        return 1

    # Capture the arm/waist pose AT THIS MOMENT so we can ramp smoothly
    # from wherever the operator left the arm to the final spread target.
    # Without this, an arm parked at e.g. shoulder_pitch = -3.3 rad would
    # be yanked back at full kp=150 torque the instant we publish.
    start_arm_q: dict[int, float] = {}
    for i in list(ARM_INDICES) + list(WAIST_INDICES):
        try:
            start_arm_q[i] = float(state.motor_state[i].q)
        except Exception:
            start_arm_q[i] = 0.0
    bad_arm_joints = [i for i, q in start_arm_q.items()
                       if i in ARM_INDICES and abs(q) > 0.5]
    if bad_arm_joints:
        names = ", ".join(JointIdx(i).name for i in bad_arm_joints)
        print(f"[arm_idle_holder] start arm pose is FAR from spread:")
        for i in bad_arm_joints:
            print(f"                  {JointIdx(i).name:<18} q={start_arm_q[i]:+.3f}")
        print(f"[arm_idle_holder]   → ramping over {ramp_time:.1f}s "
              "(linear blend) instead of step input.")

    crc = CRC()

    period = 1.0 / publish_hz
    stop = {"v": False}

    def _stop(*_):
        stop["v"] = True
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    started = time.time()
    last_log = 0.0
    last_yield_state: Optional[bool] = None
    n_published = 0
    n_yielded = 0
    ramp_done_logged = False

    while not stop["v"]:
        loop_t0 = time.time()

        if hold_time > 0 and (loop_t0 - started) >= hold_time:
            print("[arm_idle_holder] hold-time reached, exiting cleanly.")
            break

        yielding = _should_yield()
        if yielding != last_yield_state:
            last_yield_state = yielding
            if yielding:
                print("[arm_idle_holder] YIELDING — another process is "
                      f"driving rt/arm_sdk (flag={YIELD_FLAG_PATH})")
            else:
                # Re-arm the ramp: the next time we publish, treat the
                # arm's current q as the new start so we don't snap.
                cur = sub.Read()
                if cur is not None:
                    for i in list(ARM_INDICES) + list(WAIST_INDICES):
                        try:
                            start_arm_q[i] = float(cur.motor_state[i].q)
                        except Exception:
                            pass
                started = time.time()
                ramp_done_logged = False
                print("[arm_idle_holder] resuming spread-pose hold "
                      f"(re-armed {ramp_time:.1f}s ramp from current pose)")

        if not yielding:
            # Refresh leg q from latest state to avoid fighting locomotion.
            cur = sub.Read()
            if cur is not None:
                state = cur

            # Compute ramp alpha ∈ [0, 1].
            elapsed = loop_t0 - started
            alpha = 1.0 if ramp_time <= 0 else min(1.0, elapsed / ramp_time)

            msg = _build_spread_lowcmd(
                state, state.mode_machine, spread_l, spread_r,
                start_arm_q=start_arm_q, alpha=alpha,
            )
            msg.crc = crc.Crc(msg)
            pub.Write(msg)
            n_published += 1

            if alpha >= 1.0 and not ramp_done_logged:
                ramp_done_logged = True
                print(f"[arm_idle_holder] ramp finished — holding spread "
                      f"(L_ShoulderRoll={spread_l:+.2f}, "
                      f"R_ShoulderRoll={spread_r:+.2f}, weight=1.0)")
        else:
            n_yielded += 1

        now = time.time()
        if (now - last_log) > 5.0:
            last_log = now
            mode = "yield" if yielding else "hold"
            elapsed = now - started
            ramp_pct = (
                "100%" if ramp_time <= 0 or elapsed >= ramp_time
                else f"{100*elapsed/ramp_time:.0f}%"
            )
            print(f"[arm_idle_holder] alive ({mode}) "
                  f"published={n_published}  yielded={n_yielded}  "
                  f"ramp={ramp_pct}")

        # Sleep the rest of the period (longer when yielding to be cheap).
        sleep_for = period if not yielding else 0.2
        elapsed = time.time() - loop_t0
        time.sleep(max(0.0, sleep_for - elapsed))

    print("[arm_idle_holder] exiting (last published frame: spread + weight=1)")
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--iface", default=os.environ.get("G1_IFACE", "enp2s0"),
                   help="Network interface bound to the robot "
                        "(default: enp2s0 / $G1_IFACE)")
    p.add_argument("--hz", type=float, default=50.0,
                   help="Publish rate to rt/arm_sdk (default: 50 Hz)")
    p.add_argument("--spread-l", type=float, default=1.5,
                   help="Left  shoulder roll target [rad] (default: +1.5)")
    p.add_argument("--spread-r", type=float, default=-1.5,
                   help="Right shoulder roll target [rad] (default: -1.5)")
    p.add_argument("--hold-time", type=float, default=0.0,
                   help="If >0, exit after this many seconds "
                        "(useful for smoke-testing without systemd)")
    p.add_argument("--ramp-time", type=float, default=8.0,
                   help="Linear ramp from the arm's current pose to the "
                        "final spread pose (seconds, default: 8.0). "
                        "Without this the controller would yank the arm "
                        "at full PD torque if it started far from spread.")
    args = p.parse_args()
    return hold_spread(iface=args.iface, publish_hz=args.hz,
                        spread_l=args.spread_l, spread_r=args.spread_r,
                        hold_time=args.hold_time, ramp_time=args.ramp_time)


if __name__ == "__main__":
    sys.exit(main())
