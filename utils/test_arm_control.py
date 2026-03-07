"""
Direct arm control test for Unitree G1 (hanging/stand mode).

Uses rt/lowcmd to directly control arm motors, bypassing the locomotion
controller. Suitable when the robot is suspended on a test stand.

Steps:
  1. Release motion controller mode
  2. Send direct PD commands to arm joints via rt/lowcmd

Usage:
    conda activate lerobot
    python test_arm_control.py
"""

import sys
import time

import numpy as np
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread


class G1JointIndex:
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21

    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28

    WaistYaw = 12
    WaistRoll = 13
    WaistPitch = 14


ARM_JOINTS = [
    G1JointIndex.LeftShoulderPitch, G1JointIndex.LeftShoulderRoll,
    G1JointIndex.LeftShoulderYaw, G1JointIndex.LeftElbow,
    G1JointIndex.LeftWristRoll, G1JointIndex.LeftWristPitch,
    G1JointIndex.LeftWristYaw,
    G1JointIndex.RightShoulderPitch, G1JointIndex.RightShoulderRoll,
    G1JointIndex.RightShoulderYaw, G1JointIndex.RightElbow,
    G1JointIndex.RightWristRoll, G1JointIndex.RightWristPitch,
    G1JointIndex.RightWristYaw,
]

NUM_MOTORS = 35
KP = 30.0
KD = 1.5
CONTROL_DT = 0.02  # 50 Hz
MOVE_DURATION = 3.0
DELTA_ANGLE = 0.5  # ~28.6 degrees


class ArmControlTest:
    def __init__(self):
        self.time_ = 0.0
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.initial_positions = {}
        self.state_received = False
        self.crc = CRC()
        self.done = False
        self.mode_machine = 0

    def init(self):
        self.lowcmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher.Init()

        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self._on_low_state, 10)

    def start(self):
        print("Waiting for robot state...")
        while not self.state_received:
            time.sleep(0.1)

        for joint in ARM_JOINTS:
            self.initial_positions[joint] = self.low_state.motor_state[joint].q

        print(f"\nmode_machine: {self.mode_machine}")
        print("\nCurrent arm joint positions:")
        for joint in ARM_JOINTS:
            q = self.initial_positions[joint]
            print(f"  [{joint:2d}] q={q:+.4f} rad ({np.degrees(q):+.1f} deg)")

        target = self.initial_positions[G1JointIndex.RightElbow] + DELTA_ANGLE
        print(f"\nTest: Right Elbow [{G1JointIndex.RightElbow}]")
        print(f"  Current: {self.initial_positions[G1JointIndex.RightElbow]:+.4f} rad")
        print(f"  Target:  {target:+.4f} rad")
        print(f"  Delta:   {DELTA_ANGLE:+.4f} rad ({np.degrees(DELTA_ANGLE):+.1f} deg)")

        print(f"\nPlan:")
        print(f"  Phase 1 ({MOVE_DURATION}s): Smoothly take over arm joints")
        print(f"  Phase 2 ({MOVE_DURATION}s): Move right elbow by {np.degrees(DELTA_ANGLE):.1f} deg")
        print(f"  Phase 3 ({MOVE_DURATION}s): Move right elbow back")
        print(f"  Phase 4 ({MOVE_DURATION}s): Smoothly release control")

        input("\nPress Enter to execute (Ctrl+C to abort)...")

        self.control_thread = RecurrentThread(
            interval=CONTROL_DT, target=self._control_loop, name="arm_control"
        )
        self.control_thread.Start()

    def _on_low_state(self, msg: LowState_):
        self.low_state = msg
        self.mode_machine = msg.mode_machine
        if not self.state_received:
            self.state_received = True

    def _control_loop(self):
        self.time_ += CONTROL_DT

        self.low_cmd.mode_pr = 0
        self.low_cmd.mode_machine = self.mode_machine

        if self.time_ < MOVE_DURATION:
            # Phase 1: Smooth takeover — ramp up kp/kd from 0
            ratio = np.clip(self.time_ / MOVE_DURATION, 0.0, 1.0)
            for joint in ARM_JOINTS:
                self.low_cmd.motor_cmd[joint].mode = 1
                self.low_cmd.motor_cmd[joint].q = self.initial_positions[joint]
                self.low_cmd.motor_cmd[joint].dq = 0.0
                self.low_cmd.motor_cmd[joint].tau = 0.0
                self.low_cmd.motor_cmd[joint].kp = KP * ratio
                self.low_cmd.motor_cmd[joint].kd = KD * ratio

        elif self.time_ < MOVE_DURATION * 2:
            # Phase 2: Move right elbow
            ratio = np.clip((self.time_ - MOVE_DURATION) / MOVE_DURATION, 0.0, 1.0)
            for joint in ARM_JOINTS:
                target = self.initial_positions[joint]
                if joint == G1JointIndex.RightElbow:
                    target += DELTA_ANGLE * ratio
                self.low_cmd.motor_cmd[joint].mode = 1
                self.low_cmd.motor_cmd[joint].q = target
                self.low_cmd.motor_cmd[joint].dq = 0.0
                self.low_cmd.motor_cmd[joint].tau = 0.0
                self.low_cmd.motor_cmd[joint].kp = KP
                self.low_cmd.motor_cmd[joint].kd = KD

        elif self.time_ < MOVE_DURATION * 3:
            # Phase 3: Move right elbow back
            ratio = np.clip((self.time_ - MOVE_DURATION * 2) / MOVE_DURATION, 0.0, 1.0)
            for joint in ARM_JOINTS:
                target = self.initial_positions[joint]
                if joint == G1JointIndex.RightElbow:
                    target += DELTA_ANGLE * (1.0 - ratio)
                self.low_cmd.motor_cmd[joint].mode = 1
                self.low_cmd.motor_cmd[joint].q = target
                self.low_cmd.motor_cmd[joint].dq = 0.0
                self.low_cmd.motor_cmd[joint].tau = 0.0
                self.low_cmd.motor_cmd[joint].kp = KP
                self.low_cmd.motor_cmd[joint].kd = KD

        elif self.time_ < MOVE_DURATION * 4:
            # Phase 4: Smooth release — ramp down kp/kd to 0
            ratio = np.clip((self.time_ - MOVE_DURATION * 3) / MOVE_DURATION, 0.0, 1.0)
            for joint in ARM_JOINTS:
                self.low_cmd.motor_cmd[joint].mode = 1
                self.low_cmd.motor_cmd[joint].q = self.initial_positions[joint]
                self.low_cmd.motor_cmd[joint].dq = 0.0
                self.low_cmd.motor_cmd[joint].tau = 0.0
                self.low_cmd.motor_cmd[joint].kp = KP * (1.0 - ratio)
                self.low_cmd.motor_cmd[joint].kd = KD * (1.0 - ratio)
        else:
            self.done = True
            return

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher.Write(self.low_cmd)

        phase = min(int(self.time_ / MOVE_DURATION) + 1, 4)
        current_q = self.low_state.motor_state[G1JointIndex.RightElbow].q
        print(f"\r  Phase {phase}/4 | t={self.time_:.1f}s | "
              f"Right Elbow q={current_q:+.4f} rad ({np.degrees(current_q):+.1f} deg)", end="", flush=True)


def release_motion_controller():
    """Release any active motion controller so we can send rt/lowcmd directly."""
    msc = MotionSwitcherClient()
    msc.SetTimeout(5.0)
    msc.Init()

    code, result = msc.CheckMode()
    print(f"Current mode: {result}")

    if result and result.get("name"):
        print(f"Releasing mode '{result['name']}'...")
        msc.ReleaseMode()
        time.sleep(2)
        code, result = msc.CheckMode()
        print(f"After release: {result}")


if __name__ == "__main__":
    print("=" * 60)
    print("  G1 Direct Arm Control Test (rt/lowcmd)")
    print("  Robot should be HANGING on a test stand")
    print("  Right elbow will move ~28.6 degrees and return")
    print("=" * 60)
    print("\nWARNING: Ensure no obstacles around the robot's arms!")

    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    release_motion_controller()

    test = ArmControlTest()
    test.init()
    test.start()

    while not test.done:
        time.sleep(0.5)

    print("\n\nDone! Motors released.")
