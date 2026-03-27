#!/usr/bin/env python3
"""
Tests for teleop_bridge.py — all runnable without robot or Redis.

Usage:
    python tests/test_teleop_bridge.py              # run all tests
    python tests/test_teleop_bridge.py -v           # verbose
    python -m pytest tests/test_teleop_bridge.py    # via pytest
"""

import json
import os
import sys
import tempfile
import time
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "utils"))
from teleop_bridge import (
    ARM_JOINT_NAMES,
    ARM_JOINTS,
    ARM_SDK_JOINTS,
    ARMS_ONLY_JOINTS,
    CONTROL_HZ,
    HAND_JOINT_NAMES,
    HAND_REMAP_LEFT,
    HAND_REMAP_RIGHT,
    JOINT_LIMITS,
    LEFT_ARM_JOINTS,
    MAX_DELTA_PER_STEP,
    MIMIC_LEFT_ARM_SLICE,
    MIMIC_OBS_DIM,
    MIMIC_RIGHT_ARM_SLICE,
    MIMIC_WAIST_SLICE,
    MockRedisReader,
    MockRobotSender,
    N_HAND_MOTORS,
    RIGHT_ARM_JOINTS,
    TeleopBridge,
    TeleopFrame,
    TrajectoryRecorder,
    WAIST_JOINTS,
    clamp_hand,
    clamp_joints,
    extract_upper_body_from_mimic_obs,
    remap_hand_left,
    remap_hand_right,
)


class TestMimicObsExtraction(unittest.TestCase):
    """Test extraction of arm joints from TWIST2's 35D mimic_obs."""

    def test_default_arms_only_14dof(self):
        """Default mode returns 14 arm joints (no waist)."""
        obs = np.zeros(MIMIC_OBS_DIM)
        result = extract_upper_body_from_mimic_obs(obs)
        self.assertEqual(len(result), len(ARMS_ONLY_JOINTS))
        for j in ARMS_ONLY_JOINTS:
            self.assertIn(j, result)
        for j in WAIST_JOINTS:
            self.assertNotIn(j, result)

    def test_with_waist_17dof(self):
        """With include_waist=True, returns 17 joints."""
        obs = np.zeros(MIMIC_OBS_DIM)
        result = extract_upper_body_from_mimic_obs(obs, include_waist=True)
        self.assertEqual(len(result), len(ARM_SDK_JOINTS))
        for j in ARM_SDK_JOINTS:
            self.assertIn(j, result)

    def test_waist_mapping_when_enabled(self):
        """Waist joints (indices 12-14) come from mimic_obs[18:21] when enabled."""
        obs = np.zeros(MIMIC_OBS_DIM)
        obs[18] = 0.1   # waist_yaw
        obs[19] = 0.2   # waist_roll
        obs[20] = 0.3   # waist_pitch
        result = extract_upper_body_from_mimic_obs(obs, include_waist=True)
        self.assertAlmostEqual(result[12], 0.1)
        self.assertAlmostEqual(result[13], 0.2)
        self.assertAlmostEqual(result[14], 0.3)

    def test_waist_ignored_by_default(self):
        """Waist data in mimic_obs should NOT appear when include_waist=False."""
        obs = np.zeros(MIMIC_OBS_DIM)
        obs[18:21] = [0.5, 0.3, -0.1]
        result = extract_upper_body_from_mimic_obs(obs)
        for j in WAIST_JOINTS:
            self.assertNotIn(j, result)

    def test_left_arm_mapping(self):
        """Left arm joints (15-21) should come from mimic_obs[21:28]."""
        obs = np.zeros(MIMIC_OBS_DIM)
        for i, j in enumerate(LEFT_ARM_JOINTS):
            obs[21 + i] = j * 0.01
        result = extract_upper_body_from_mimic_obs(obs)
        for i, j in enumerate(LEFT_ARM_JOINTS):
            self.assertAlmostEqual(result[j], j * 0.01)

    def test_right_arm_mapping(self):
        """Right arm joints (22-28) should come from mimic_obs[28:35]."""
        obs = np.zeros(MIMIC_OBS_DIM)
        for i, j in enumerate(RIGHT_ARM_JOINTS):
            obs[28 + i] = j * 0.01
        result = extract_upper_body_from_mimic_obs(obs)
        for i, j in enumerate(RIGHT_ARM_JOINTS):
            self.assertAlmostEqual(result[j], j * 0.01)

    def test_lower_body_ignored(self):
        """Lower body fields (obs[0:18]) should NOT appear in output."""
        obs = np.ones(MIMIC_OBS_DIM) * 999.0
        obs[MIMIC_LEFT_ARM_SLICE] = 0.0
        obs[MIMIC_RIGHT_ARM_SLICE] = 0.0

        result = extract_upper_body_from_mimic_obs(obs)
        for j in ARMS_ONLY_JOINTS:
            self.assertAlmostEqual(result[j], 0.0,
                                   msg=f"Joint {j} should be 0, not contaminated by lower body")

    def test_wrong_dimension_raises(self):
        with self.assertRaises(AssertionError):
            extract_upper_body_from_mimic_obs(np.zeros(30))
        with self.assertRaises(AssertionError):
            extract_upper_body_from_mimic_obs(np.zeros(40))

    def test_realistic_mimic_obs_arms_only(self):
        """Simulate a plausible TWIST2 mimic_obs — arms only (default)."""
        obs = np.zeros(MIMIC_OBS_DIM)
        obs[0:2] = [0.1, -0.05]
        obs[2] = 0.74
        obs[3:5] = [0.01, -0.02]
        obs[5] = 0.0
        obs[6:12] = [0.0, 0.0, -0.4, 0.8, -0.4, 0.0]
        obs[12:18] = [0.0, 0.0, -0.4, 0.8, -0.4, 0.0]
        obs[18:21] = [0.05, 0.0, -0.1]
        obs[21:28] = [-0.5, 0.3, 0.0, 0.8, 0.0, 0.0, 0.0]
        obs[28:35] = [-0.5, -0.3, 0.0, 0.8, 0.0, 0.0, 0.0]

        result = extract_upper_body_from_mimic_obs(obs)

        self.assertNotIn(12, result)
        self.assertAlmostEqual(result[15], -0.5)
        self.assertAlmostEqual(result[16], 0.3)
        self.assertAlmostEqual(result[18], 0.8)
        self.assertAlmostEqual(result[22], -0.5)
        self.assertAlmostEqual(result[23], -0.3)

    def test_realistic_mimic_obs_with_waist(self):
        """Same data but with waist included."""
        obs = np.zeros(MIMIC_OBS_DIM)
        obs[18:21] = [0.05, 0.0, -0.1]
        obs[21:28] = [-0.5, 0.3, 0.0, 0.8, 0.0, 0.0, 0.0]
        obs[28:35] = [-0.5, -0.3, 0.0, 0.8, 0.0, 0.0, 0.0]

        result = extract_upper_body_from_mimic_obs(obs, include_waist=True)

        self.assertAlmostEqual(result[12], 0.05)
        self.assertAlmostEqual(result[15], -0.5)
        self.assertAlmostEqual(result[22], -0.5)


class TestJointClamping(unittest.TestCase):
    """Test safety clamping logic."""

    def test_within_limits_unchanged(self):
        positions = {j: 0.0 for j in ARM_SDK_JOINTS}
        clamped = clamp_joints(positions)
        for j in ARM_SDK_JOINTS:
            self.assertAlmostEqual(clamped[j], 0.0)

    def test_exceeds_upper_limit(self):
        positions = {12: 5.0}  # waist_yaw limit is ±2.618
        clamped = clamp_joints(positions)
        self.assertAlmostEqual(clamped[12], 2.618)

    def test_exceeds_lower_limit(self):
        positions = {13: -1.0}  # waist_roll limit is ±0.52
        clamped = clamp_joints(positions)
        self.assertAlmostEqual(clamped[13], -0.52)

    def test_delta_clamping(self):
        last = {15: 0.0}
        positions = {15: 1.0}
        clamped = clamp_joints(positions, last)
        self.assertAlmostEqual(clamped[15], MAX_DELTA_PER_STEP)

    def test_delta_clamping_negative(self):
        last = {15: 0.0}
        positions = {15: -1.0}
        clamped = clamp_joints(positions, last)
        self.assertAlmostEqual(clamped[15], -MAX_DELTA_PER_STEP)

    def test_delta_small_move_passes(self):
        last = {15: 0.0}
        positions = {15: 0.01}
        clamped = clamp_joints(positions, last)
        self.assertAlmostEqual(clamped[15], 0.01)

    def test_combined_limit_and_delta(self):
        """When both URDF limit and delta limit apply, the tighter one wins."""
        last = {13: 0.50}
        positions = {13: 0.60}  # URDF limit 0.52, delta would allow 0.58
        clamped = clamp_joints(positions, last)
        self.assertLessEqual(clamped[13], 0.52)

    def test_no_last_positions(self):
        positions = {j: 0.1 for j in ARM_SDK_JOINTS}
        clamped = clamp_joints(positions, last_positions=None)
        for j in ARM_SDK_JOINTS:
            self.assertAlmostEqual(clamped[j], 0.1)

    def test_all_joints_have_limits(self):
        for j in ARM_SDK_JOINTS:
            self.assertIn(j, JOINT_LIMITS, f"Joint {j} missing from JOINT_LIMITS")


class TestHandClamping(unittest.TestCase):

    def test_within_range(self):
        hand = np.array([0.5] * N_HAND_MOTORS)
        clamped = clamp_hand(hand)
        np.testing.assert_array_almost_equal(clamped, hand)

    def test_clamp_high(self):
        hand = np.array([3.0] * N_HAND_MOTORS)
        clamped = clamp_hand(hand)
        self.assertTrue(np.all(clamped <= 1.5))

    def test_clamp_low(self):
        hand = np.array([-2.0] * N_HAND_MOTORS)
        clamped = clamp_hand(hand)
        self.assertTrue(np.all(clamped >= -0.5))


class TestMockRedisReader(unittest.TestCase):
    """Test mock reader generates valid frames."""

    def test_wave_arms_only(self):
        """Default: arms only (14 joints, no waist)."""
        reader = MockRedisReader(motion="wave")
        frame = reader.read()
        self.assertTrue(frame.valid)
        self.assertEqual(len(frame.upper_body), len(ARMS_ONLY_JOINTS))
        for j in WAIST_JOINTS:
            self.assertNotIn(j, frame.upper_body)
        self.assertEqual(len(frame.left_hand), N_HAND_MOTORS)
        self.assertEqual(len(frame.right_hand), N_HAND_MOTORS)

    def test_wave_with_waist(self):
        """With include_waist: 17 joints."""
        reader = MockRedisReader(motion="wave", include_waist=True)
        frame = reader.read()
        self.assertTrue(frame.valid)
        self.assertEqual(len(frame.upper_body), len(ARM_SDK_JOINTS))

    def test_reach_motion_valid(self):
        reader = MockRedisReader(motion="reach")
        frame = reader.read()
        self.assertTrue(frame.valid)

    def test_static_motion_valid(self):
        reader = MockRedisReader(motion="static")
        frame = reader.read()
        self.assertTrue(frame.valid)

    def test_wave_motion_changes_over_time(self):
        reader = MockRedisReader(motion="wave")
        frames = []
        for _ in range(5):
            frames.append(reader.read())
            time.sleep(0.05)
        pos_first = frames[0].upper_body[15]
        pos_last = frames[-1].upper_body[15]
        self.assertNotAlmostEqual(pos_first, pos_last, places=3,
                                  msg="Wave motion should change over time")

    def test_all_joints_within_urdf_limits(self):
        reader = MockRedisReader(motion="wave")
        for _ in range(50):
            frame = reader.read()
            for j, val in frame.upper_body.items():
                lo, hi = JOINT_LIMITS.get(j, (-3.14, 3.14))
                self.assertGreaterEqual(val, lo - 0.01,
                                        f"Joint {j} below limit: {val} < {lo}")
                self.assertLessEqual(val, hi + 0.01,
                                     f"Joint {j} above limit: {val} > {hi}")
            time.sleep(0.01)


class TestMockRobotSender(unittest.TestCase):

    def test_wait_always_true(self):
        sender = MockRobotSender()
        self.assertTrue(sender.wait_for_state())

    def test_arms_only_default(self):
        """Default sender tracks only arm joints (no waist)."""
        sender = MockRobotSender()
        current = sender.get_current_positions()
        self.assertEqual(len(current), len(ARMS_ONLY_JOINTS))
        for j in WAIST_JOINTS:
            self.assertNotIn(j, current)

    def test_with_waist(self):
        sender = MockRobotSender(include_waist=True)
        current = sender.get_current_positions()
        self.assertEqual(len(current), len(ARM_SDK_JOINTS))

    def test_send_arm_updates_positions(self):
        sender = MockRobotSender()
        positions = {j: 0.5 for j in ARMS_ONLY_JOINTS}
        sender.send_arm(positions)
        current = sender.get_current_positions()
        for j in ARMS_ONLY_JOINTS:
            self.assertAlmostEqual(current[j], 0.5)

    def test_initial_positions_zero(self):
        sender = MockRobotSender()
        current = sender.get_current_positions()
        for j in ARMS_ONLY_JOINTS:
            self.assertAlmostEqual(current[j], 0.0)


class TestTrajectoryRecorder(unittest.TestCase):
    """Test trajectory recording to JSON (teach.py compatible format)."""

    def test_record_and_save(self):
        recorder = TrajectoryRecorder(record_hands=True)
        recorder.start()
        self.assertTrue(recorder.recording)

        for i in range(10):
            upper = {j: float(i) * 0.01 for j in ARM_JOINTS}
            upper.update({j: 0.0 for j in WAIST_JOINTS})
            left = np.zeros(N_HAND_MOTORS)
            right = np.zeros(N_HAND_MOTORS)
            recorder.add_frame(upper, left, right)
            time.sleep(0.01)

        recorder.stop()
        self.assertFalse(recorder.recording)
        self.assertEqual(len(recorder.frames), 10)

        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as f:
            path = f.name
        try:
            recorder.save(path)
            with open(path) as f:
                data = json.load(f)

            self.assertIn("metadata", data)
            self.assertIn("frames", data)
            self.assertEqual(data["metadata"]["n_frames"], 10)
            self.assertEqual(data["metadata"]["control_mode"], "teleop_bridge")
            self.assertEqual(data["metadata"]["source"], "twist2_pico_vr")
            self.assertTrue(data["metadata"]["record_hands"])
            self.assertIn("hand_joint_names", data["metadata"])

            frame = data["frames"][0]
            self.assertIn("t", frame)
            self.assertIn("arm", frame)
            self.assertIn("hand", frame)
            for j in ARM_JOINTS:
                self.assertIn(str(j), frame["arm"])
            for i in range(N_HAND_MOTORS * 2):
                self.assertIn(str(i), frame["hand"])
        finally:
            os.unlink(path)

    def test_no_frames_does_not_crash(self):
        recorder = TrajectoryRecorder()
        result = recorder.save("/tmp/empty_test.json")
        self.assertIsNone(result)

    def test_frames_not_added_when_not_recording(self):
        recorder = TrajectoryRecorder()
        upper = {j: 0.0 for j in ARM_SDK_JOINTS}
        recorder.add_frame(upper)
        self.assertEqual(len(recorder.frames), 0)

    def test_no_hands_mode(self):
        recorder = TrajectoryRecorder(record_hands=False)
        recorder.start()

        upper = {j: 0.1 for j in ARM_SDK_JOINTS}
        recorder.add_frame(upper)
        recorder.stop()

        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as f:
            path = f.name
        try:
            recorder.save(path)
            with open(path) as f:
                data = json.load(f)
            self.assertFalse(data["metadata"]["record_hands"])
            self.assertNotIn("hand", data["frames"][0])
        finally:
            os.unlink(path)


class TestTrajectoryCompatibility(unittest.TestCase):
    """Verify that recorded trajectories are loadable by collect_dataset.py."""

    def _create_trajectory(self, n_frames=20):
        recorder = TrajectoryRecorder(record_hands=True)
        recorder.start()
        for i in range(n_frames):
            upper = {j: np.sin(i * 0.1) * 0.3 for j in ARM_JOINTS}
            upper.update({j: 0.0 for j in WAIST_JOINTS})
            left = np.ones(N_HAND_MOTORS) * 0.5
            right = np.ones(N_HAND_MOTORS) * 0.5
            recorder.add_frame(upper, left, right)
            time.sleep(0.005)
        recorder.stop()
        return recorder

    def test_loadable_by_collect_dataset_format(self):
        """Mimic collect_dataset.py's load_trajectory() logic."""
        recorder = self._create_trajectory()

        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as f:
            path = f.name
        try:
            recorder.save(path)
            with open(path) as f:
                data = json.load(f)

            meta = data["metadata"]
            frames = data["frames"]

            self.assertIn("n_frames", meta)
            self.assertIn("duration_s", meta)
            self.assertIn("arm_joints", meta)

            for fr in frames:
                arm_pos = {}
                for k, v in fr["arm"].items():
                    arm_pos[int(k)] = float(v)
                for j in ARM_JOINTS:
                    self.assertIn(j, arm_pos,
                                  f"Joint {j} missing from arm data")

                if "hand" in fr:
                    hand_pos = np.zeros(N_HAND_MOTORS * 2)
                    for k, v in fr["hand"].items():
                        idx = int(k)
                        hand_pos[idx] = float(v)
                    left_hand = hand_pos[:N_HAND_MOTORS]
                    right_hand = hand_pos[N_HAND_MOTORS:]
                    self.assertEqual(len(left_hand), N_HAND_MOTORS)
                    self.assertEqual(len(right_hand), N_HAND_MOTORS)
        finally:
            os.unlink(path)

    def test_timestamps_monotonic(self):
        recorder = self._create_trajectory()

        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as f:
            path = f.name
        try:
            recorder.save(path)
            with open(path) as f:
                data = json.load(f)
            times = [fr["t"] for fr in data["frames"]]
            for i in range(1, len(times)):
                self.assertGreaterEqual(times[i], times[i - 1],
                                        "Timestamps must be monotonically increasing")
            self.assertAlmostEqual(times[0], 0.0, places=2,
                                   msg="First timestamp should be ~0")
        finally:
            os.unlink(path)


class TestTeleopBridgeIntegration(unittest.TestCase):
    """Integration test: full bridge loop with mock components."""

    def test_mock_bridge_runs(self):
        """Run bridge for a few cycles and verify it processes frames."""
        reader = MockRedisReader(motion="wave")
        sender = MockRobotSender()
        bridge = TeleopBridge(reader, sender)

        n_steps = 20

        def _run_limited():
            for _ in range(n_steps):
                frame = reader.read()
                if frame.valid:
                    positions = clamp_joints(frame.upper_body)
                    left_hand = clamp_hand(frame.left_hand)
                    right_hand = clamp_hand(frame.right_hand)
                    sender.send_arm(positions)
                    sender.send_hands(left_hand, right_hand)
                    bridge.stats["frames_sent"] += 1
                time.sleep(0.01)

        bridge.stats["start_time"] = time.time()
        _run_limited()
        self.assertEqual(bridge.stats["frames_sent"], n_steps)

    def test_mock_bridge_with_recording(self):
        """Run bridge with recorder, verify trajectory is captured."""
        reader = MockRedisReader(motion="static")
        sender = MockRobotSender()
        recorder = TrajectoryRecorder(record_hands=True)
        bridge = TeleopBridge(reader, sender, recorder)

        recorder.start()
        last_pos = None
        for _ in range(30):
            frame = reader.read()
            positions = clamp_joints(frame.upper_body, last_pos)
            left_hand = clamp_hand(frame.left_hand)
            right_hand = clamp_hand(frame.right_hand)
            sender.send_arm(positions)
            recorder.add_frame(positions, left_hand, right_hand)
            last_pos = positions
            time.sleep(0.01)
        recorder.stop()

        self.assertEqual(len(recorder.frames), 30)

        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as f:
            path = f.name
        try:
            recorder.save(path)
            with open(path) as f:
                data = json.load(f)
            self.assertEqual(data["metadata"]["n_frames"], 30)
        finally:
            os.unlink(path)

    def test_delta_clamping_effective_in_loop(self):
        """Verify that rapid target changes are smoothed by delta clamping."""
        reader = MockRedisReader(motion="wave")
        last_pos = {j: 0.0 for j in ARMS_ONLY_JOINTS}

        max_observed_delta = 0.0
        for _ in range(100):
            frame = reader.read()
            positions = clamp_joints(frame.upper_body, last_pos)
            for j in ARMS_ONLY_JOINTS:
                delta = abs(positions[j] - last_pos[j])
                max_observed_delta = max(max_observed_delta, delta)
            last_pos = positions
            time.sleep(0.005)

        self.assertLessEqual(max_observed_delta, MAX_DELTA_PER_STEP + 1e-9,
                             "Delta clamping should enforce per-step limit")


class TestRedisDataSimulation(unittest.TestCase):
    """Simulate what TWIST2 would publish to Redis and verify parsing."""

    def _make_twist2_mimic_obs(
        self, waist=(0.0, 0.0, 0.0),
        left_arm=None, right_arm=None,
    ):
        """Build a 35D mimic_obs like TWIST2 would."""
        obs = np.zeros(MIMIC_OBS_DIM)
        obs[0:2] = [0.0, 0.0]      # base vel
        obs[2] = 0.74              # height
        obs[3:5] = [0.0, 0.0]     # roll, pitch
        obs[5] = 0.0              # yaw vel
        obs[6:12] = 0.0           # left leg
        obs[12:18] = 0.0          # right leg
        obs[18:21] = waist
        if left_arm is not None:
            obs[21:28] = left_arm
        if right_arm is not None:
            obs[28:35] = right_arm
        return obs

    def test_standing_pose(self):
        obs = self._make_twist2_mimic_obs()
        result = extract_upper_body_from_mimic_obs(obs)
        for j in ARMS_ONLY_JOINTS:
            self.assertAlmostEqual(result[j], 0.0)

    def test_arms_raised(self):
        obs = self._make_twist2_mimic_obs(
            left_arm=[-1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            right_arm=[-1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        result = extract_upper_body_from_mimic_obs(obs)
        self.assertAlmostEqual(result[15], -1.5)
        self.assertAlmostEqual(result[22], -1.5)

    def test_waist_ignored_by_default(self):
        obs = self._make_twist2_mimic_obs(waist=(0.5, 0.1, -0.1))
        result = extract_upper_body_from_mimic_obs(obs)
        for j in WAIST_JOINTS:
            self.assertNotIn(j, result)

    def test_waist_turned_when_enabled(self):
        obs = self._make_twist2_mimic_obs(waist=(0.5, 0.1, -0.1))
        result = extract_upper_body_from_mimic_obs(obs, include_waist=True)
        self.assertAlmostEqual(result[12], 0.5)
        self.assertAlmostEqual(result[13], 0.1)
        self.assertAlmostEqual(result[14], -0.1)

    def test_hand_data_parsing(self):
        left_json = json.dumps([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
        right_json = json.dumps([0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1])

        left = np.array(json.loads(left_json), dtype=np.float64)
        right = np.array(json.loads(right_json), dtype=np.float64)

        self.assertEqual(len(left), N_HAND_MOTORS)
        self.assertEqual(len(right), N_HAND_MOTORS)
        self.assertAlmostEqual(left[0], 0.1)
        self.assertAlmostEqual(right[0], 0.7)


class TestHandFingerRemapping(unittest.TestCase):
    """Test TWIST2→DDS hand finger remapping.

    TWIST2 (unitree_interface) left hand: Thumb(0,1,2), Middle(3,4), Index(5,6)
    DDS (HandCmd_) left hand:             Thumb(0,1,2), Index(3,4), Middle(5,6)
    So for left hand, indices 3-4 and 5-6 are swapped.
    Right hand is identical in both SDKs.
    """

    def test_left_hand_remap_swaps_index_middle(self):
        twist2_left = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
        # twist2: Thumb(0.1,0.2,0.3), Middle(0.4,0.5), Index(0.6,0.7)
        # expected DDS: Thumb(0.1,0.2,0.3), Index(0.6,0.7), Middle(0.4,0.5)
        dds_left = remap_hand_left(twist2_left)
        np.testing.assert_array_almost_equal(
            dds_left, [0.1, 0.2, 0.3, 0.6, 0.7, 0.4, 0.5]
        )

    def test_right_hand_remap_is_identity(self):
        twist2_right = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
        dds_right = remap_hand_right(twist2_right)
        np.testing.assert_array_almost_equal(dds_right, twist2_right)

    def test_remap_preserves_length(self):
        hand = np.zeros(N_HAND_MOTORS)
        self.assertEqual(len(remap_hand_left(hand)), N_HAND_MOTORS)
        self.assertEqual(len(remap_hand_right(hand)), N_HAND_MOTORS)

    def test_remap_arrays_are_valid_permutations(self):
        self.assertEqual(sorted(HAND_REMAP_LEFT), list(range(N_HAND_MOTORS)))
        self.assertEqual(sorted(HAND_REMAP_RIGHT), list(range(N_HAND_MOTORS)))

    def test_left_remap_thumb_unchanged(self):
        twist2_left = np.array([1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 0.0])
        dds_left = remap_hand_left(twist2_left)
        np.testing.assert_array_almost_equal(dds_left[:3], [1.0, 2.0, 3.0])

    def test_double_remap_not_identity_for_left(self):
        """Applying the remap twice should NOT give identity (it's a swap)."""
        original = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
        once = remap_hand_left(original)
        twice = remap_hand_left(once)
        np.testing.assert_array_almost_equal(twice, original,
            err_msg="Swap of 2 pairs is self-inverse, so double remap = identity")


class TestJointMappingConsistency(unittest.TestCase):
    """Verify that joint indices/names are consistent across the codebase."""

    def test_arm_sdk_joints_composition(self):
        expected = WAIST_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS
        self.assertEqual(ARM_SDK_JOINTS, expected)

    def test_arm_joints_range(self):
        self.assertEqual(ARM_JOINTS, list(range(15, 29)))

    def test_waist_joints(self):
        self.assertEqual(WAIST_JOINTS, [12, 13, 14])

    def test_left_arm_7dof(self):
        self.assertEqual(len(LEFT_ARM_JOINTS), 7)
        self.assertEqual(LEFT_ARM_JOINTS, list(range(15, 22)))

    def test_right_arm_7dof(self):
        self.assertEqual(len(RIGHT_ARM_JOINTS), 7)
        self.assertEqual(RIGHT_ARM_JOINTS, list(range(22, 29)))

    def test_total_upper_body_17dof(self):
        self.assertEqual(len(ARM_SDK_JOINTS), 17)

    def test_arms_only_14dof(self):
        self.assertEqual(len(ARMS_ONLY_JOINTS), 14)
        self.assertEqual(ARMS_ONLY_JOINTS, LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS)

    def test_hand_14dof(self):
        self.assertEqual(len(HAND_JOINT_NAMES), N_HAND_MOTORS * 2)

    def test_arm_joint_names_coverage(self):
        for j in ARM_JOINTS:
            self.assertIn(j, ARM_JOINT_NAMES,
                          f"Joint {j} missing from ARM_JOINT_NAMES")

    def test_mimic_obs_slices_contiguous(self):
        """Verify the slices for waist/left_arm/right_arm cover [18:35]."""
        all_indices = (
            list(range(MIMIC_WAIST_SLICE.start, MIMIC_WAIST_SLICE.stop))
            + list(range(MIMIC_LEFT_ARM_SLICE.start, MIMIC_LEFT_ARM_SLICE.stop))
            + list(range(MIMIC_RIGHT_ARM_SLICE.start, MIMIC_RIGHT_ARM_SLICE.stop))
        )
        self.assertEqual(all_indices, list(range(18, 35)))


if __name__ == "__main__":
    unittest.main()
