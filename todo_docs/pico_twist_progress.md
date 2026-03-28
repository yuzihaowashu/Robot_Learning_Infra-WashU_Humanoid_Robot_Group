# PICO 4 Ultra + TWIST2 Teleoperation Progress

## Overview

Goal: Use a PICO 4 Ultra Enterprise VR headset to teleoperate a Unitree G1 humanoid robot's upper body (arms + hands) for data collection.

Architecture:
```
PICO 4 Ultra (VR headset + controllers)
    ↓  XRoboToolkit SDK
PC (controller_teleop.py or TWIST2)  ← conda: gmr
    ↓  Redis
teleop_bridge.py                      ← conda: lerobot
    ↓  DDS (rt/arm_sdk + rt/dex3/*/cmd)
Unitree G1 Robot
```

---

## 1. Environment Setup

### 1.1 Redis Server

```bash
sudo apt update && sudo apt install -y redis-server
sudo systemctl enable redis-server && sudo systemctl start redis-server
```

Install Python client in the `lerobot` environment:
```bash
conda activate lerobot
pip install "redis[hiredis]"
```

### 1.2 TWIST2 Git Submodule

The TWIST2 submodule uses an SSH URL (`git@github.com:amazon-far/TWIST2.git`) that requires authorized SSH keys. Workaround using HTTPS:

```bash
# Edit .git/config, change the TWIST2 submodule URL to:
#   url = https://github.com/amazon-far/TWIST2.git
# Then:
git submodule update --init TWIST2
```

### 1.3 Conda Environment: `gmr` (for TWIST2 / controller_teleop)

```bash
conda create -n gmr python=3.10 -y
conda activate gmr

# Core dependencies
pip install mujoco mink scipy numpy redis[hiredis] rich opencv-python pybind11

# GMR (General Motion Retargeting) — installed as editable from local clone
cd /home/humanoid-pc/yu.zihao/GMR
pip install -e .

# IK solver backend
pip install daqp

# Fix potential libstdc++ issues
conda install -c conda-forge libstdcxx-ng -y
```

### 1.4 Conda Environment: `lerobot` (for teleop_bridge.py)

Pre-existing environment. Required packages:
- `unitree_sdk2py` (Unitree DDS SDK)
- `redis[hiredis]`
- `numpy`
- `pinocchio` (for gravity compensation)

### 1.5 XRoboToolkit PC Service (on the PC)

Download the `.deb` package from the XRoboToolkit releases:
- File: `XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb`
- Saved at: `/home/humanoid-pc/yu.zihao/XRoboToolkit_PC_Service.deb`

```bash
sudo dpkg -i /home/humanoid-pc/yu.zihao/XRoboToolkit_PC_Service.deb
```

Installed to `/opt/apps/roboticsservice/`. Start the service:
```bash
cd /opt/apps/roboticsservice
bash runService.sh
```

Config file: `/opt/apps/roboticsservice/setting.ini`

### 1.6 XRobot Unity Client APK (on the PICO headset)

The XRobot app must be sideloaded onto the PICO 4 Ultra via ADB:

```bash
# Install ADB
sudo apt install -y android-tools-adb

# Connect PICO via USB, enable developer mode in headset settings
adb devices   # should list the PICO device

# Sideload the APK (download from XRoboToolkit-Unity-Client GitHub releases)
# v1.1.1: https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases
adb install XRoboToolkit-PICO-1.1.1.apk
```

After installing, the XRobot app appears in the PICO library under "Unknown Sources".

---

## 2. Known Issue: Whole-Body Tracking Requires Motion Trackers

### Problem

TWIST2's GMR pipeline requires **whole-body tracking data** (24 body joints) from the PICO headset. In the XRobot app, selecting "whole-body" tracking shows:

> **"Tracker exception, please connect to calibrate tracker!"**

### Root Cause

Found in the XRoboToolkit Unity Client source code (`Assets/Scripts/UI/UIOperate.cs`):

```csharp
// The app requires at least 2 PICO Motion Trackers
res = PXR_MotionTracking.CheckMotionTrackerModeAndNumber(
    MotionTrackerMode.BodyTracking,
    MotionTrackerNum.TWO);

PXR_MotionTracking.GetBodyTrackingSupported(ref support);

if (!support || res != 0)
{
    BodyInfo.text = "Tracker exception, please connect to calibrate tracker!";
}
```

The app **hard-codes a requirement for 2 PICO Motion Trackers** (external ankle-worn devices). Without them, body tracking refuses to start.

### PICO Tracking Modes

PICO's OpenXR API defines two body tracking modes:
- `XR_BTM_WITH_SWIFT_PICO` (default) — requires external Motion Trackers
- `XR_BTM_WITHOUT_SWIFT_PICO` — camera-based, no external trackers needed

The XRobot app only uses the default mode (WITH_SWIFT), hence the failure.

### What Works Without Motion Trackers

- **Controller 6DoF tracking** (position + rotation of each hand controller)
- **Hand tracking** (finger joint positions via PICO's built-in cameras)
- **Headset 6DoF tracking**
- **Controller buttons/triggers/grip**

### What Does NOT Work

- **Body tracking** (24 body joints) — needs external Motion Trackers
- Therefore, TWIST2's full GMR retargeting pipeline is blocked

### Options Considered

| Option | Description | Status |
|--------|-------------|--------|
| **A. Buy Motion Trackers** | 2× PICO Motion Tracker (~$200-300) | Not pursued |
| **B. Modify XRobot APK** | Bypass tracker check, use WITHOUT_SWIFT mode. Needs Unity Editor + PICO SDK to rebuild | Not pursued yet |
| **C. Controller-pose IK** | Use controller 6DoF → MuJoCo IK → arm joints. Skip body tracking entirely | **Implemented** |

---

## 3. Solution: Controller-Based Teleoperation (Option C)

### Architecture

Instead of TWIST2's body tracking → GMR pipeline, we use a simpler approach:

```
PICO VR Controllers (6DoF pose)
    ↓  XRoboToolkit SDK (xrobotoolkit_sdk)
controller_teleop.py (conda: gmr)
    ↓  MuJoCo + mink IK solver (position-only, orientation_cost=0)
    ↓  Arm joint angles (7 per arm)
    ↓  Redis (same keys as TWIST2)
teleop_bridge.py (conda: lerobot)
    ↓  DDS (rt/arm_sdk for arms/waist, rt/dex3/*/cmd for hands)
G1 Robot
```

### How It Works

1. **Controller 6DoF** → `xrt.get_left_controller_pose()` / `get_right_controller_pose()`
2. **Coordinate transform** — Unity (left-handed) → robot frame (right-handed) via `unity_to_robot()` / `unity_quat_to_robot()`
3. **Calibration** — When user presses A, record initial controller positions as reference; IK solver resets to default pose
4. **Delta mapping** — Compute position delta from reference, apply to robot's default wrist position
5. **Workspace clamping** — Clamp target wrist position within `MAX_ARM_REACH` (0.40m) sphere from shoulder to prevent overextension
6. **IK solve** — `mink.solve_ik` with `daqp` solver (20 iterations, damping=1e-3) for 7 arm joints per arm, using the G1 MuJoCo model (`g1_mocap_29dof.xml`). Non-arm joints (free joint, legs, waist) are frozen during IK.
7. **Hand control** — Index trigger closes, grip opens (same as TWIST2)
8. **Redis publish** — Same 35D `mimic_obs` format + 7D hand poses per side

### Default Arm Pose

Arms start in a moderately forward "ready" position:
- `ShoulderPitch = -0.75` (upper arm ~43° forward)
- `ShoulderRoll = ±0.15` (slight outward spread)
- `Elbow = 0.75` (~43° bend)
- Wrist joints at 0

### State Machine

```
IDLE → press A → _calibrate (record VR reference, IK reset) → TELEOP
TELEOP → press A → IK reset, clear references → IDLE (publish default pose)
TELEOP → PICO disconnects (stale data) → IK reset → IDLE (publish default pose)
IDLE → PICO reconnects → force re-calibration on next A press
```

### Files

- `utils/controller_teleop.py` — VR → IK → Redis (runs in `gmr` env)
- `utils/teleop_bridge.py` — Redis → DDS → Robot (runs in `lerobot` env)
- `run_controller_teleop.sh` — Launch script for controller_teleop

### Usage

Terminal 1 (VR → Redis):
```bash
conda activate gmr
cd Robot_Learning_Infra-WashU_Humanoid_Robot_Group
python utils/controller_teleop.py --redis-ip localhost
```

Terminal 2 (Redis → Robot):
```bash
conda activate lerobot
cd Robot_Learning_Infra-WashU_Humanoid_Robot_Group
python utils/teleop_bridge.py
```

VR Controls:
- **A button** — Toggle teleop on/off (calibrates on start)
- **Index trigger** — Close hand
- **Grip** — Open hand
- **Move controllers** — Robot arms follow (relative to calibration reference)

---

## 4. teleop_bridge.py — Key Features & Fixes

### 4.1 DDS mode_machine Echo

`rt/arm_sdk` requires `mode_machine` from `LowState` to be echoed back in `LowCmd`. Added to `send_arm()` and `release()`:
```python
cmd.mode_pr = 0
cmd.mode_machine = self.mode_machine  # from _state_callback
```

### 4.2 ChannelFactoryInitialize Order

Moved `ChannelFactoryInitialize()` to `main()` before `ensure_ai_mode()`, removed redundant call from `RobotSender.__init__()`.

### 4.3 Per-Motor Hand Clamping

Replaced generic `[-0.5, 1.5]` hand clamping with per-motor limits from Dex3-1 spec:
```python
HAND_LIMITS_LEFT_MIN = np.array([-1.0472, -0.7243, 0.0, -1.5708, -1.7453, -1.5708, -1.7453])
HAND_LIMITS_LEFT_MAX = np.array([1.0472, 1.0472, 1.7453, 0.0, 0.0, 0.0, 0.0])
```

### 4.4 EMA Smoothing (Anti-Jitter)

Exponential moving average to reduce VR tracking noise:
```python
EMA_ALPHA_ARM  = 0.6   # arm joint smoothing (higher = more responsive)
EMA_ALPHA_HAND = 0.4   # hand motor smoothing
```

### 4.5 Per-Step Delta Clamping

Limits maximum joint angle change per control step to prevent sudden jerky movements:
```python
MAX_DELTA_PER_STEP = 0.15  # rad per 50Hz step ≈ 430°/s max
```

### 4.6 Waist Stability (High PD + Gravity Compensation)

When running without `--with-waist`, waist joints are locked to 0 with high PD gains:
```python
KP_WAIST = 200.0
KD_WAIST = 5.0
```

### 4.7 Pinocchio Gravity Compensation

Per-joint feedforward gravity torques computed from URDF using Pinocchio, applied to all arm_sdk joints. Prevents arm sag and waist droop:
```python
class GravityCompensator:
    def compute(self, low_state):
        # Uses pinocchio.computeGeneralizedGravity() with current joint positions
        # Returns {joint_idx: tau_ff} for arm + waist joints
```

### 4.8 Startup Diagnostics

On startup, prints robot's initial joint positions and first 5 frames of target vs. command values for debugging convergence.

### 4.9 Smooth Release

`release()` ramps arm_sdk weight from 1.0 → 0.0 over 2 seconds, tracking the robot's actual joint positions to prevent sudden drops.

---

## 5. controller_teleop.py — Key Design Decisions

### 5.1 Position-Only IK (No Orientation Tracking)

`orientation_cost = 0.0` in the IK frame tasks. This avoids conflicts between position and orientation goals at the workspace boundary, which caused dangerous arm poses and IK instability.

### 5.2 Frozen Non-Arm Joints

During IK, all joints except the 14 arm joints are frozen (free joint, legs, waist). This prevents the IK solver from moving the torso or legs to reach a target, which would conflict with the robot's built-in locomotion controller.

### 5.3 Workspace Clamping

Target wrist positions are clamped within a `MAX_ARM_REACH = 0.40m` sphere from the shoulder. This prevents:
- IK from failing on unreachable targets
- The robot's balance controller from stepping forward to compensate

### 5.4 Stale Data Detection

If VR controller data is frozen for >1 second (PICO disconnected), the system pauses arm commands and publishes the default pose. When data becomes live again, the system forces re-calibration.

### 5.5 Clean Re-Calibration

When exiting teleop (A button or stale data), the IK solver is immediately reset to the default pose and all calibration references are cleared. When re-entering teleop, a fresh calibration is performed. This ensures no state leakage between teleop sessions.

---

## 6. Comparison with Other Approaches

### TWIST2 (GMR Pipeline)
- Uses full-body motion retargeting with SMPL-X human model
- Requires external motion trackers for body tracking
- Uses `mink.solve_ik` with rotation offsets from JSON config
- More accurate for whole-body retargeting but blocked by tracker requirement

### LeRobot (HuggingFace)
- Uses physical exoskeleton arms (not VR) for teleoperation
- IK via `Pinocchio` + `CasADi`/`IPOPT` (more robust nonlinear optimizer)
- Uses `WeightedMovingFilter` for smoothing (single layer)
- More accurate IK but requires different hardware

### Our Approach (controller_teleop.py)
- Uses VR controller 6DoF poses with relative position tracking
- IK via `mink` + `daqp` (fast, good enough for position-only tracking)
- Smoothing: EMA in teleop_bridge (single layer) + per-step delta clamping
- Simpler, works with standard VR controllers, good for data collection

---

## 7. Known Limitations

- **Position-only tracking**: No wrist orientation tracking (rotation following disabled to avoid IK conflicts)
- **No lower body**: Legs are controlled by Unitree's built-in locomotion controller; only arms/hands/waist are teleoperated
- **VR drift**: PICO's inside-out tracking can drift over long sessions; re-calibrate periodically by pressing A twice
- **Coordinate alignment**: Unity→robot coordinate transform is approximate; fine direction mapping may need tuning

---

## 8. TODO / Next Steps

- [x] Test full pipeline on real robot: `controller_teleop.py` → `teleop_bridge.py` → G1
- [x] Tune EMA smoothing and delta clamping for smooth + responsive control
- [x] Add gravity compensation (Pinocchio-based)
- [x] Fix re-calibration state cleanup on teleop toggle
- [ ] Tune position scale factor (`--scale`) for optimal VR-to-robot mapping
- [ ] Add trajectory recording support to controller_teleop pipeline
- [ ] Test with waist control (`--with-waist`)
- [ ] Consider Option B (rebuild XRobot APK with WITHOUT_SWIFT mode) for full-body tracking
- [ ] Explore orientation tracking once position mapping is fully stable
- [ ] Consider upgrading IK to CasADi/IPOPT for better accuracy (like LeRobot)
