#!/usr/bin/env python3
"""FK / IK for G1 left wrist (`left_wrist_yaw_link`) matching EEF dataset conventions."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ROOT_DIR = Path(__file__).resolve().parents[1]
WBC_DIR = (
    ROOT_DIR
    / "Isaac-GR00T"
    / "external_dependencies"
    / "GR00T-WholeBodyControl"
)

LEFT_ARM_JOINT_NAMES = (
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
)
LEFT_ARM_CONTROL_LIMITS = np.array(
    [
        [-3.0892, 2.6704],
        [-1.5882, 2.2515],
        [-2.6180, 2.6180],
        [-1.0472, 2.0944],
        [-1.9722, 1.9722],
        [-1.6144, 1.6144],
        [-1.6144, 1.6144],
    ],
    dtype=np.float64,
)

OPEN_HAND_Q = np.zeros(7, dtype=np.float64)
CLOSED_LEFT_HAND_Q = np.array(
    [0.0, 1.0, 1.74, -1.57, -1.74, -1.57, -1.74],
    dtype=np.float64,
)


def _require_pinocchio() -> Any:
    try:
        import pinocchio as pin

        return pin
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            "EEF deployment requires pinocchio in the same Python env as "
            "dp_g1_client (e.g. `pip install pin` in robodiff, or run client "
            "with a conda env that has pinocchio)."
        ) from exc


def pose7_to_se3(pose: np.ndarray) -> Any:
    """7D [x,y,z,qx,qy,qz,qw] -> pin.SE3 (same quaternion order as training)."""
    _require_pinocchio()
    import pinocchio as pin

    p = np.asarray(pose, dtype=np.float64).reshape(7)
    t = p[:3]
    quat_xyzw = p[3:7]
    rmat = Rotation.from_quat(quat_xyzw).as_matrix()
    return pin.SE3(rmat, t)


def se3_to_pose7(se3: Any) -> np.ndarray:
    import pinocchio as pin

    assert isinstance(se3, pin.SE3)
    t = se3.translation
    quat_xyzw = Rotation.from_matrix(se3.rotation).as_quat()
    return np.array(
        [t[0], t[1], t[2], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2], quat_xyzw[3]],
        dtype=np.float32,
    )


def gripper_scalar_from_left_hand(left_hand: np.ndarray) -> float:
    q = np.asarray(left_hand, dtype=np.float64).reshape(-1)
    if q.size != 7:
        return 0.0
    open_dist = float(np.linalg.norm(q - OPEN_HAND_Q))
    close_dist = float(np.linalg.norm(q - CLOSED_LEFT_HAND_Q))
    denom = open_dist + close_dist
    if denom <= 1e-6:
        return 0.0
    return float(np.clip(open_dist / denom, 0.0, 1.0))


class G1LeftWristKinematics:
    """Uses GR00T WBC G1 model; FK matches `xr_to_lerobot.WristYawLinkFK`."""

    def __init__(self) -> None:
        if not WBC_DIR.exists():
            raise FileNotFoundError(f"Missing WBC dependency: {WBC_DIR}")
        if str(WBC_DIR) not in sys.path:
            sys.path.insert(0, str(WBC_DIR))
        from gr00t_wbc.control.robot_model.instantiation.g1 import (
            instantiate_g1_robot_model,
        )

        _require_pinocchio()

        self.robot_model = instantiate_g1_robot_model()
        self.left_frame = self.robot_model.supplemental_info.hand_frame_names[
            "left"
        ]
        self.default_body_q = self.robot_model.get_body_actuated_joints(
            self.robot_model.get_default_body_pose()
        ).astype(np.float64)

    def body_q(
        self,
        body15: np.ndarray,
        left_arm7: np.ndarray,
        right_arm7: np.ndarray,
    ) -> np.ndarray:
        b = self.default_body_q.copy()
        body15 = np.asarray(body15, dtype=np.float64).reshape(-1)
        if body15.size >= 15:
            b[:15] = body15[:15]
        b[15:22] = np.asarray(left_arm7, dtype=np.float64).reshape(7)
        b[22:29] = np.asarray(right_arm7, dtype=np.float64).reshape(7)
        return b

    def full_configuration(
        self,
        body_q29: np.ndarray,
        left_hand7: np.ndarray,
        right_hand7: np.ndarray,
    ) -> np.ndarray:
        return self.robot_model.get_configuration_from_actuated_joints(
            np.asarray(body_q29, dtype=np.float64),
            left_hand_actuated_joint_values=np.asarray(
                left_hand7, dtype=np.float64
            ),
            right_hand_actuated_joint_values=np.asarray(
                right_hand7, dtype=np.float64
            ),
        )

    def left_wrist_pose7(
        self,
        body_q29: np.ndarray,
        left_hand7: np.ndarray,
        right_hand7: np.ndarray,
    ) -> np.ndarray:
        q = self.full_configuration(body_q29, left_hand7, right_hand7)
        self.robot_model.cache_forward_kinematics(q, auto_clip=False)
        se3 = self.robot_model.frame_placement(self.left_frame)
        return se3_to_pose7(se3)

    def left_arm_joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        # Use the real arm-sdk safety envelope from `vla_client`. The WBC
        # supplemental model has a narrower left shoulder-roll lower bound
        # (+0.19 rad), which does not match recorded/real G1 arm commands.
        return LEFT_ARM_CONTROL_LIMITS[:, 0], LEFT_ARM_CONTROL_LIMITS[:, 1]

    def solve_left_arm_ik(
        self,
        body15: np.ndarray,
        right_arm7: np.ndarray,
        left_hand7: np.ndarray,
        right_hand7: np.ndarray,
        target_pose7: np.ndarray,
        seed_left_arm7: np.ndarray,
        max_nfev: int = 80,
        seed_regularization: float = 0.01,
    ) -> tuple[np.ndarray, float]:
        """Return (left_arm_q7, residual_norm)."""
        import pinocchio as pin

        target = pose7_to_se3(target_pose7)
        lo, hi = self.left_arm_joint_limits()
        seed = np.clip(
            np.asarray(seed_left_arm7, dtype=np.float64).reshape(7), lo, hi
        )
        right7 = np.asarray(right_arm7, dtype=np.float64).reshape(7)
        lh = np.asarray(left_hand7, dtype=np.float64).reshape(7)
        rh = np.asarray(right_hand7, dtype=np.float64).reshape(7)
        body15 = np.asarray(body15, dtype=np.float64).reshape(-1)

        def residuals(x: np.ndarray) -> np.ndarray:
            bq = self.body_q(body15, x, right7)
            q_full = self.full_configuration(bq, lh, rh)
            self.robot_model.cache_forward_kinematics(q_full, auto_clip=False)
            cur = self.robot_model.frame_placement(self.left_frame)
            d_mi = cur.inverse() * target
            return pin.log6(d_mi).vector

        def objective(x: np.ndarray) -> np.ndarray:
            if seed_regularization <= 0:
                return residuals(x)
            return np.concatenate(
                [residuals(x), seed_regularization * (x - seed)]
            )

        r0 = residuals(seed)
        if float(np.linalg.norm(r0)) < 1e-3:
            return seed.astype(np.float32), float(np.linalg.norm(r0))

        res = least_squares(
            objective,
            seed,
            bounds=(lo, hi),
            max_nfev=max_nfev,
            ftol=1e-8,
            xtol=1e-8,
            gtol=1e-8,
            method="trf",
        )
        x = np.clip(res.x, lo, hi)
        rn = float(np.linalg.norm(residuals(x)))
        return x.astype(np.float32), rn


_kin_singleton: G1LeftWristKinematics | None = None


def get_left_wrist_kinematics() -> G1LeftWristKinematics:
    global _kin_singleton
    if _kin_singleton is None:
        _kin_singleton = G1LeftWristKinematics()
    return _kin_singleton
