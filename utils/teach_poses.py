"""Arm/hand poses aligned with teleop_vla_infer bimanual default (forward q=0).

References:
  - utils/dp_g1_client.py: FORWARD_READY_ARM_Q, SPREAD_ARM_Q, OPEN_HAND_Q
  - utils/replay_xr_robot_arm.py: bimanual prepare waypoints
  - docs/teleop_real_data_collection.md: default inactive / park pose
"""

from __future__ import annotations

# Unitree G1 arm_sdk joint indices
ARM_JOINTS = list(range(15, 29))

# 14D policy layout index -> unitree joint (left 0-6, right 7-13)
_POLICY_TO_UNITREE = {
    0: 15, 1: 16, 2: 17, 3: 18, 4: 19, 5: 20, 6: 21,
    7: 22, 8: 23, 9: 24, 10: 25, 11: 26, 12: 27, 13: 28,
}

# teleop bimanual default: both arms q=0 (hands approach forward)
FORWARD_ARM_Q_14 = [0.0] * 14

# outward clearance before entering forward (Dex3 / body clearance)
SPREAD_ARM_Q_14 = [0.0] * 14
SPREAD_ARM_Q_14[1] = 1.5
SPREAD_ARM_Q_14[8] = -1.5

# Dex3 open / forward hand posture
OPEN_HAND_Q = [0.0] * 7

# Dex3 closed fist (robot_hand_unitree.py DEX3_*_CLOSE_Q)
CLOSED_LEFT_HAND_Q = [0.0, 1.0, 1.74, -1.57, -1.74, -1.57, -1.74]
CLOSED_RIGHT_HAND_Q = [0.0, -1.0, -1.74, 1.57, 1.74, 1.57, 1.74]

PREPARE_CLOSE_HANDS_SEC = 0.5
# Slower arm moves — reduces inward swing into the torso.
PREPARE_CLEARANCE_SEC = 3.5
PREPARE_FORWARD_SEC = 4.5
# Hold forward stiff before engaging drag-teach (avoids sudden drop).
SETTLE_AT_FORWARD_SEC = 0.8
COMPLIANT_RAMP_SEC = 2.5
# Park / release after Stop & save (slower than prepare — safe outward retreat).
PARK_FORWARD_SEC = 4.0
PARK_CLEARANCE_SEC = 5.0
RELEASE_ARM_SEC = 6.0
# Between steps: stay at forward (no spread park / full release).
BETWEEN_STEPS_FORWARD_SEC = 2.0
BETWEEN_STEPS_COMPLIANT_RAMP_SEC = 1.2
BETWEEN_STEPS_OPEN_HANDS_SEC = 0.8
CLOSE_HAND_KP = 0.4
CLOSE_HAND_KD = 0.15


def _dict_from_policy_q(q14: list[float]) -> dict[int, float]:
    out = {j: 0.0 for j in ARM_JOINTS}
    for i, val in enumerate(q14):
        out[_POLICY_TO_UNITREE[i]] = float(val)
    return out


FORWARD_ARM_POSE = _dict_from_policy_q(FORWARD_ARM_Q_14)
SPREAD_ARM_POSE = _dict_from_policy_q(SPREAD_ARM_Q_14)

# Legacy teach safe-home (elbows bent); kept for dashboard compatibility only.
SAFE_HOME_POSE = {j: 0.0 for j in ARM_JOINTS}
SAFE_HOME_POSE.update({
    15: 0.0, 16: 0.5, 18: 1.0,
    22: 0.0, 23: -0.5, 25: 1.0,
})
