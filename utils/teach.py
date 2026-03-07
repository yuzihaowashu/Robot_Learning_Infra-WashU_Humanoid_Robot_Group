#!/usr/bin/env python3
"""
Drag-and-Teach: Record arm + hand trajectories by physically moving the robot.

Uses rt/arm_sdk to overlay arm control on top of Unitree's locomotion
controller. The legs and body keep full balance (push-resistant) while
the arms are set to compliant mode for drag teaching.

Usage:
    python teach.py                    # auto-save to trajectories/
    python teach.py -o my_motion.json  # custom output path
    python teach.py --no-hands         # skip hand recording
"""

import argparse
import json
import os
import signal
import sys
import threading
import time

from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
    MotionSwitcherClient,
)
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import (
    unitree_hg_msg_dds__HandCmd_,
    unitree_hg_msg_dds__LowCmd_,
    unitree_hg_msg_dds__MotorCmd_,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (
    HandCmd_,
    HandState_,
    LowCmd_,
    LowState_,
)
from unitree_sdk2py.utils.crc import CRC

RECORD_HZ = 50
RECORD_DT = 1.0 / RECORD_HZ

ARM_JOINTS = list(range(15, 29))
WAIST_JOINTS = [12, 13, 14]
ARM_SDK_JOINTS = WAIST_JOINTS + ARM_JOINTS

ARM_SDK_ENABLE_IDX = 29

TEACH_KP = 0.0
TEACH_KD = 1.0
WAIST_KP = 40.0
WAIST_KD = 2.0

DAMPING_KD_HAND = 0.2
N_HAND_MOTORS_PER_HAND = 7

TOPIC_LEFT_HAND_CMD = "rt/dex3/left/cmd"
TOPIC_RIGHT_HAND_CMD = "rt/dex3/right/cmd"
TOPIC_LEFT_HAND_STATE = "rt/dex3/left/state"
TOPIC_RIGHT_HAND_STATE = "rt/dex3/right/state"


def _hand_motor_mode(motor_id, status=0x01, timeout=0):
    """Pack RIS motor mode byte: id(4b) | status(3b) | timeout(1b)."""
    return (motor_id & 0x0F) | ((status & 0x07) << 4) | ((timeout & 0x01) << 7)

URDF_PATH = (
    "/home/humanoid-pc/unitree_rl_gym/resources/robots/"
    "g1_description/g1_29dof_with_hand_rev_1_0.urdf"
)

UNITREE_TO_PIN = {}
for _i in range(15):
    UNITREE_TO_PIN[_i] = _i
for _i in range(7):
    UNITREE_TO_PIN[15 + _i] = 15 + _i
    UNITREE_TO_PIN[22 + _i] = 29 + _i


class GravityCompensator:
    """Compute per-joint gravity torques using Pinocchio."""

    def __init__(self):
        self.model = None
        self.data = None
        self.available = False
        try:
            import pinocchio as pin
            self.pin = pin
            self.model = pin.buildModelFromUrdf(URDF_PATH)
            self.data = self.model.createData()
            self.neutral_q = pin.neutral(self.model)
            self.available = True
            print("Gravity compensation: ENABLED "
                  "(Pinocchio + URDF)")
        except Exception as e:
            print(f"Gravity compensation: DISABLED "
                  f"({e})")

    def compute(self, low_state):
        """Return {unitree_joint_idx: tau_ff} for arm joints."""
        if not self.available:
            return {}
        q = self.neutral_q.copy()
        for u_idx, p_idx in UNITREE_TO_PIN.items():
            if p_idx < self.model.nq:
                q[p_idx] = low_state.motor_state[u_idx].q
        G = self.pin.computeGeneralizedGravity(
            self.model, self.data, q
        )
        tau_ff = {}
        for j in ARM_JOINTS:
            p_idx = UNITREE_TO_PIN[j]
            tau_ff[j] = float(G[p_idx])
        for j in WAIST_JOINTS:
            p_idx = UNITREE_TO_PIN[j]
            tau_ff[j] = float(G[p_idx])
        return tau_ff

ARM_JOINT_NAMES = {
    15: "L_ShoulderPitch", 16: "L_ShoulderRoll",
    17: "L_ShoulderYaw", 18: "L_Elbow",
    19: "L_WristRoll", 20: "L_WristPitch",
    21: "L_WristYaw",
    22: "R_ShoulderPitch", 23: "R_ShoulderRoll",
    24: "R_ShoulderYaw", 25: "R_Elbow",
    26: "R_WristRoll", 27: "R_WristPitch",
    28: "R_WristYaw",
}

HAND_JOINT_NAMES = [
    "L_Thumb0", "L_Thumb1", "L_Thumb2",
    "L_Index0", "L_Index1", "L_Middle0", "L_Middle1",
    "R_Thumb0", "R_Thumb1", "R_Thumb2",
    "R_Index0", "R_Index1", "R_Middle0", "R_Middle1",
]


class TeachRecorder:
    def __init__(self, record_hands=True, grav_comp=None):
        self.record_hands = record_hands
        self.grav_comp = grav_comp
        self.low_state = None
        self.left_hand_state = None
        self.right_hand_state = None
        self.state_received = False
        self.hand_received = False
        self.crc = CRC()

        self.waist_lock_pos = {}
        self.recording = False
        self.frames = []
        self._stop = False

    def init(self):
        self.arm_sdk_pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_sdk_pub.Init()
        self.lowstate_sub = ChannelSubscriber(
            "rt/lowstate", LowState_
        )
        self.lowstate_sub.Init(self._on_low_state, 10)

        # Separate publishers for left and right hand
        self.left_hand_pub = ChannelPublisher(
            TOPIC_LEFT_HAND_CMD, HandCmd_
        )
        self.left_hand_pub.Init()
        self.right_hand_pub = ChannelPublisher(
            TOPIC_RIGHT_HAND_CMD, HandCmd_
        )
        self.right_hand_pub.Init()
        if self.record_hands:
            self.left_hand_sub = ChannelSubscriber(
                TOPIC_LEFT_HAND_STATE, HandState_
            )
            self.left_hand_sub.Init(
                self._on_left_hand_state, 10
            )
            self.right_hand_sub = ChannelSubscriber(
                TOPIC_RIGHT_HAND_STATE, HandState_
            )
            self.right_hand_sub.Init(
                self._on_right_hand_state, 10
            )

    def _on_low_state(self, msg: LowState_):
        self.low_state = msg
        if not self.state_received:
            self.state_received = True

    def _on_left_hand_state(self, msg: HandState_):
        self.left_hand_state = msg
        if not self.hand_received:
            self.hand_received = True

    def _on_right_hand_state(self, msg: HandState_):
        self.right_hand_state = msg
        if not self.hand_received:
            self.hand_received = True

    def wait_for_state(self, timeout=5.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.state_received:
                if not self.record_hands or self.hand_received:
                    return True
            time.sleep(0.1)
        return self.state_received

    def lock_waist(self):
        """Snapshot waist positions to hold them during teach."""
        for j in WAIST_JOINTS:
            self.waist_lock_pos[j] = float(
                self.low_state.motor_state[j].q
            )

    def _send_teach_cmd(self):
        """Publish arm_sdk: waist locked, arms compliant + gravity comp."""
        cmd = unitree_hg_msg_dds__LowCmd_()
        cmd.motor_cmd[ARM_SDK_ENABLE_IDX].q = 1.0

        grav = {}
        if self.grav_comp and self.low_state:
            grav = self.grav_comp.compute(self.low_state)

        for j in WAIST_JOINTS:
            cmd.motor_cmd[j].mode = 1
            cmd.motor_cmd[j].q = self.waist_lock_pos.get(
                j, self.low_state.motor_state[j].q
            )
            cmd.motor_cmd[j].dq = 0.0
            cmd.motor_cmd[j].tau = grav.get(j, 0.0)
            cmd.motor_cmd[j].kp = WAIST_KP
            cmd.motor_cmd[j].kd = WAIST_KD

        for j in ARM_JOINTS:
            cmd.motor_cmd[j].mode = 1
            cmd.motor_cmd[j].q = (
                self.low_state.motor_state[j].q
            )
            cmd.motor_cmd[j].dq = 0.0
            cmd.motor_cmd[j].tau = grav.get(j, 0.0)
            cmd.motor_cmd[j].kp = TEACH_KP
            cmd.motor_cmd[j].kd = TEACH_KD

        cmd.crc = self.crc.Crc(cmd)
        self.arm_sdk_pub.Write(cmd)

    def _build_hand_cmd(self, q_targets, kp=0.0, kd=0.2):
        """Build a HandCmd_ with 7 motors."""
        cmd = unitree_hg_msg_dds__HandCmd_()
        for i in range(N_HAND_MOTORS_PER_HAND):
            cmd.motor_cmd[i].mode = _hand_motor_mode(i)
            cmd.motor_cmd[i].q = float(q_targets[i])
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].tau = 0.0
            cmd.motor_cmd[i].kp = kp
            cmd.motor_cmd[i].kd = kd
        return cmd

    def _send_hand_damping(self):
        if not self.record_hands:
            return
        if self.left_hand_state is not None:
            lq = [self.left_hand_state.motor_state[i].q
                  for i in range(N_HAND_MOTORS_PER_HAND)]
            self.left_hand_pub.Write(
                self._build_hand_cmd(lq, kp=0.0, kd=DAMPING_KD_HAND)
            )
        if self.right_hand_state is not None:
            rq = [self.right_hand_state.motor_state[i].q
                  for i in range(N_HAND_MOTORS_PER_HAND)]
            self.right_hand_pub.Write(
                self._build_hand_cmd(rq, kp=0.0, kd=DAMPING_KD_HAND)
            )

    def _release_hands(self):
        """Drive all fingers to open (q=0) via correct dex3 topics."""
        print("Opening fingers...")
        zeros = [0.0] * N_HAND_MOTORS_PER_HAND
        open_kp = 1.5
        open_kd = 0.2

        # Phase A: drive to open
        steps = int(2.0 / RECORD_DT)
        for i in range(steps):
            ratio = i / steps
            cmd = self._build_hand_cmd(
                zeros, kp=open_kp * ratio, kd=open_kd
            )
            self.left_hand_pub.Write(cmd)
            self.right_hand_pub.Write(cmd)
            time.sleep(RECORD_DT)

        # Phase B: hold open
        for _ in range(int(0.5 / RECORD_DT)):
            cmd = self._build_hand_cmd(
                zeros, kp=open_kp, kd=open_kd
            )
            self.left_hand_pub.Write(cmd)
            self.right_hand_pub.Write(cmd)
            time.sleep(RECORD_DT)

        # Phase C: ramp down
        print("Releasing hand control...")
        steps = int(0.5 / RECORD_DT)
        for i in range(steps):
            w = 1.0 - (i / steps)
            cmd = self._build_hand_cmd(
                zeros, kp=open_kp * w, kd=open_kd * w
            )
            self.left_hand_pub.Write(cmd)
            self.right_hand_pub.Write(cmd)
            time.sleep(RECORD_DT)
        print("Hands released.")

    def _disable_arm_sdk(self):
        """Smoothly release arm_sdk and hand control."""
        print("Releasing arm control...")
        steps = int(2.0 / RECORD_DT)
        for i in range(steps):
            w = 1.0 - (i / steps)
            cmd = unitree_hg_msg_dds__LowCmd_()
            cmd.motor_cmd[ARM_SDK_ENABLE_IDX].q = w
            for j in ARM_SDK_JOINTS:
                cmd.motor_cmd[j].mode = 1
                cmd.motor_cmd[j].q = (
                    self.low_state.motor_state[j].q
                )
                cmd.motor_cmd[j].dq = 0.0
                cmd.motor_cmd[j].tau = 0.0
                cmd.motor_cmd[j].kp = 60.0 * w
                cmd.motor_cmd[j].kd = 1.5 * w
            cmd.crc = self.crc.Crc(cmd)
            self.arm_sdk_pub.Write(cmd)
            time.sleep(RECORD_DT)
        self._release_hands()

    def _capture_frame(self):
        frame = {"t": time.time()}
        arm = {}
        for j in ARM_JOINTS:
            arm[str(j)] = float(
                self.low_state.motor_state[j].q
            )
        frame["arm"] = arm

        if self.record_hands:
            hand = {}
            if self.left_hand_state is not None:
                for i in range(N_HAND_MOTORS_PER_HAND):
                    hand[str(i)] = float(
                        self.left_hand_state.motor_state[i].q
                    )
            if self.right_hand_state is not None:
                for i in range(N_HAND_MOTORS_PER_HAND):
                    hand[str(i + 7)] = float(
                        self.right_hand_state.motor_state[i].q
                    )
            if hand:
                frame["hand"] = hand

        return frame

    def run_teach(self, output_path):
        signal.signal(
            signal.SIGINT,
            lambda s, f: setattr(self, '_stop', True)
        )

        print("\n" + "=" * 50)
        print("  DRAG-AND-TEACH MODE")
        print("=" * 50)
        grav_str = ("ON — arms feel weightless"
                    if self.grav_comp and self.grav_comp.available
                    else "OFF — arms may feel heavy")
        print(f"\nGravity compensation: {grav_str}")
        print("Arms are in compliant mode — move them freely!")
        print("")
        print("Controls:")
        print("  [Enter]  Start/Stop recording")
        print("  [Ctrl+C] Quit and release arm control")
        print("")

        if self.record_hands and not self.hand_received:
            print("NOTE: Hand state not received. "
                  "Recording arms only.")
            self.record_hands = False

        self.lock_waist()

        key_pressed = [False]

        def _key_listener():
            while not self._stop:
                try:
                    input()
                    key_pressed[0] = True
                except EOFError:
                    break

        threading.Thread(
            target=_key_listener, daemon=True
        ).start()

        print(">> Press [Enter] to START recording...")

        while not self._stop:
            self._send_teach_cmd()
            self._send_hand_damping()
            time.sleep(RECORD_DT)

            if key_pressed[0]:
                key_pressed[0] = False
                if not self.recording:
                    self.recording = True
                    self.frames = []
                    self._rec_start = time.time()
                    print("\n** RECORDING ** "
                          "Move the robot! "
                          "Press [Enter] to stop.")
                else:
                    self.recording = False
                    dur = time.time() - self._rec_start
                    print(f"\n** STOPPED ** "
                          f"{len(self.frames)} frames, "
                          f"{dur:.1f}s")
                    break

            if self.recording:
                self.frames.append(self._capture_frame())
                n = len(self.frames)
                if n % RECORD_HZ == 0:
                    elapsed = time.time() - self._rec_start
                    print(f"  Recording: {n} frames "
                          f"({elapsed:.1f}s)")

        self._disable_arm_sdk()

        if not self.frames:
            print("No frames recorded.")
            return None

        self._save(output_path)
        return output_path

    def _save(self, path):
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)

        t0 = self.frames[0]["t"]
        for f in self.frames:
            f["t"] = round(f["t"] - t0, 4)

        metadata = {
            "record_hz": RECORD_HZ,
            "n_frames": len(self.frames),
            "duration_s": round(self.frames[-1]["t"], 2),
            "arm_joints": ARM_JOINTS,
            "arm_joint_names": ARM_JOINT_NAMES,
            "record_hands": self.record_hands,
            "control_mode": "arm_sdk",
            "created": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        if self.record_hands:
            metadata["hand_joint_names"] = HAND_JOINT_NAMES

        data = {
            "metadata": metadata,
            "frames": self.frames,
        }

        with open(path, "w") as f:
            json.dump(data, f, indent=2)

        print(f"\nSaved: {path}")
        print(f"  Frames: {metadata['n_frames']}")
        print(f"  Duration: {metadata['duration_s']}s")
        print(f"  Hz: {metadata['record_hz']}")


def ensure_ai_mode():
    """Check that the locomotion controller (ai sport) is active.

    The balance controller must be started via the hand controller:
      1. L1+A  — power on
      2. L1+UP — stand up
    Only then will the ai sport client be running.
    """
    msc = MotionSwitcherClient()
    msc.SetTimeout(5.0)
    msc.Init()

    code, result = msc.CheckMode()
    current = result.get("name", "") if result else ""
    print(f"Current mode: '{current}' (form={result})")

    if current == "ai":
        print("OK — ai sport client is active.")
        return True

    # Try to select ai mode via SDK (works in some cases)
    print("ai mode not detected. Attempting "
          "SelectMode('ai')...")
    code, _ = msc.SelectMode("ai")
    print(f"  SelectMode returned code={code}")
    time.sleep(3)

    code, result = msc.CheckMode()
    current = result.get("name", "") if result else ""
    print(f"  Mode after select: '{current}'")

    if current == "ai":
        # Also try to tell the locomotion FSM to stand
        loco = LocoClient()
        loco.SetTimeout(5.0)
        loco.Init()
        ret = loco.SetFsmId(200)
        print(f"  LocoClient.Start() returned: {ret}")
        time.sleep(3)
        return True

    print("\n" + "!" * 50)
    print("  WARNING: Balance controller NOT active!")
    print("!" * 50)
    print("\nPlease start the robot with the hand "
          "controller:")
    print("  1. L1 + A    — power on")
    print("  2. L1 + UP   — stand up")
    print("  3. Robot should be standing and balanced")
    print("  4. Then re-run this script")
    print("\nIf in debug mode (L2+R2), reboot the robot.")
    print("")

    ans = input("Continue anyway? (y/N): ").strip().lower()
    return ans == "y"


def main():
    parser = argparse.ArgumentParser(
        description="Drag-and-Teach Recorder (arm_sdk)"
    )
    parser.add_argument(
        "-o", "--output", default=None,
        help="Output file path"
    )
    parser.add_argument(
        "--no-hands", action="store_true",
        help="Skip hand/finger recording"
    )
    parser.add_argument(
        "network", nargs="?", default=None,
        help="Network interface (optional)"
    )
    args = parser.parse_args()

    if args.output is None:
        ts = time.strftime("%Y%m%d_%H%M%S")
        traj_dir = os.path.join(
            os.path.dirname(__file__), "..", "trajectories"
        )
        args.output = os.path.join(
            traj_dir, f"traj_{ts}.json"
        )

    print("=" * 50)
    print("  G1 Drag-and-Teach Recorder")
    print("  (arm_sdk + Unitree balance controller)")
    print("=" * 50)

    if args.network:
        ChannelFactoryInitialize(0, args.network)
    else:
        ChannelFactoryInitialize(0)

    ensure_ai_mode()

    grav = GravityCompensator()
    recorder = TeachRecorder(
        record_hands=not args.no_hands,
        grav_comp=grav,
    )
    recorder.init()

    print("Waiting for robot state...")
    if not recorder.wait_for_state():
        print("ERROR: No robot state received!")
        sys.exit(1)
    print("Robot connected!")

    result = recorder.run_teach(args.output)
    if result:
        print(f"\nTo replay: "
              f"python utils/replay.py {result}")


if __name__ == "__main__":
    main()
