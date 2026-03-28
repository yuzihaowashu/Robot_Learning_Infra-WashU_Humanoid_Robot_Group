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

## 8. TODO / Next Steps (Legacy — controller_teleop.py)

- [x] Test full pipeline on real robot: `controller_teleop.py` → `teleop_bridge.py` → G1
- [x] Tune EMA smoothing and delta clamping for smooth + responsive control
- [x] Add gravity compensation (Pinocchio-based)
- [x] Fix re-calibration state cleanup on teleop toggle
- [ ] ~~Tune position scale factor (`--scale`) for optimal VR-to-robot mapping~~ → superseded by xr_teleoperate
- [ ] ~~Add trajectory recording support to controller_teleop pipeline~~ → superseded by xr_teleoperate
- [ ] ~~Test with waist control (`--with-waist`)~~ → superseded by xr_teleoperate
- [ ] ~~Consider Option B (rebuild XRobot APK with WITHOUT_SWIFT mode)~~ → not needed with xr_teleoperate
- [ ] ~~Explore orientation tracking once position mapping is fully stable~~ → xr_teleoperate has it
- [ ] ~~Consider upgrading IK to CasADi/IPOPT~~ → xr_teleoperate uses CasADi/IPOPT

---

## 9. Migration to `xr_teleoperate` (Unitree Official)

### Why migrate

Our `controller_teleop.py` (mink + MuJoCo, position-only IK at 30/50Hz) has fundamental
limitations compared to Unitree's official `xr_teleoperate` framework:

| Aspect | Our controller_teleop | xr_teleoperate |
|--------|----------------------|----------------|
| IK solver | mink + daqp (linear QP) | **CasADi + IPOPT** (nonlinear NLP) |
| Orientation tracking | Disabled (orientation_cost=0) | **Enabled** (rotation_cost=1.0) |
| Smoothing | Single-layer EMA | **4-frame WeightedMovingFilter** + smooth_cost in optimizer |
| Gravity compensation | Separate in bridge | **Integrated** — IK outputs `sol_tauff = pin.rnea(...)` |
| Control rate | 50Hz (bridge) / 30Hz (IK) | **250Hz** arm controller |
| Joint limits | Hardcoded dict | **Auto from URDF** |
| Velocity limiting | Simple per-step delta clamp | **Global velocity scaling** + gradual ramp-up |
| VR input | XRoboToolkit SDK | **TeleVuer** (WebRTC/Vuer) |
| Data recording | Custom JSON | **Built-in EpisodeWriter** |
| DDS channel | rt/arm_sdk only | rt/arm_sdk (motion) or rt/lowcmd (debug) |
| PD gains | KP=60, KD=2 flat | **Tiered**: KP=80/40(wrist)/300(locked), KD=3/1.5/3 |
| Dex3-1 hand | Trigger→linear interpolation | **Full finger retargeting** (dex-retargeting lib) |
| PICO support | XRoboToolkit (needs PC Service) | TeleVuer WebRTC (direct browser) |
| Motion Tracker | N/A (controller-only) | N/A — supports **hand tracking + controller tracking** |

### Repository structure

Added as submodule: `xr_teleoperate/` → `git@github.com:unitreerobotics/xr_teleoperate.git`

```
xr_teleoperate/
├── assets/g1/
│   ├── g1_body29_hand14.urdf      ← G1 29DoF + Dex3 hands (self-contained)
│   ├── g1_body29_hand14.xml
│   ├── g1_body23.urdf
│   └── meshes/                    ← complete mesh set
├── teleop/
│   ├── teleop_hand_and_arm.py     ← main entry: VR → IK → arm control → recording
│   ├── robot_control/
│   │   ├── robot_arm_ik.py        ← CasADi/IPOPT IK (G1_29_ArmIK, etc.)
│   │   ├── robot_arm.py           ← DDS arm controller (250Hz, rt/arm_sdk or rt/lowcmd)
│   │   ├── robot_hand_unitree.py  ← Dex3-1 hand control
│   │   ├── hand_retargeting.py    ← finger retargeting wrapper
│   │   └── dex-retargeting/       ← submodule: finger retargeting algorithm
│   ├── teleimager/                ← submodule: camera image service (WebRTC)
│   ├── televuer/                  ← submodule: XR device communication (Vuer)
│   └── utils/
│       ├── episode_writer.py      ← data recording for imitation learning
│       ├── weighted_moving_filter.py  ← joint smoothing filter
│       ├── motion_switcher.py     ← locomotion mode management
│       └── ipc.py                 ← inter-process communication
└── requirements.txt               ← matplotlib, rerun-sdk, meshcat, sshkeyboard
```

### Integration plan

#### Phase 1: Environment setup
- [ ] Create conda env `tv` (Python 3.10, pinocchio=3.1.0, numpy=1.26.4)
- [ ] Install `unitree_sdk2_python` in `tv` env
- [ ] Install `teleimager` and `televuer` submodules (`pip install -e .`)
- [ ] Generate SSL certificates for TeleVuer WebRTC
- [ ] Install requirements: `pip install -r requirements.txt`
- [ ] Install CasADi and IPOPT: `pip install casadi`

#### Phase 2: PICO connection
- [ ] Verify PICO 4U can connect to TeleVuer via browser (https://PC_IP:8012)
- [ ] Test with `--input-mode=controller` (simplest, matches our current setup)
- [ ] Test with `--input-mode=hand` (native hand tracking, no Motion Tracker needed)

#### Phase 3: Robot control
- [ ] Run `teleop_hand_and_arm.py --arm=G1_29 --ee=dex3 --motion` on real G1
- [ ] Verify IK quality: position + orientation tracking
- [ ] Verify Dex3-1 hand finger mapping
- [ ] Compare control quality with our controller_teleop.py

#### Phase 4: Data recording pipeline
- [ ] Test `--record` mode, inspect EpisodeWriter output format
- [ ] Write adapter: EpisodeWriter format → our collect_dataset.py JSON format
- [ ] Or: modify collect_dataset.py to directly support xr_teleoperate format
- [ ] Verify full pipeline: xr_teleoperate → recorded data → LeRobot v2 → GR00T training

#### Phase 5: Cleanup
- [ ] Update run scripts and documentation
- [ ] Consider deprecating controller_teleop.py + teleop_bridge.py
- [ ] Update docs/teleoperation.md for new pipeline

### Key differences from our current setup

1. **No Redis**: xr_teleoperate is a single-process pipeline (no Redis intermediary)
2. **No teleop_bridge.py**: arm control is integrated directly in the same process
3. **TeleVuer instead of XRoboToolkit**: browser-based WebRTC, no PC Service deb needed
4. **250Hz control vs 50Hz**: much smoother and more responsive
5. **CasADi IK outputs gravity torques**: no separate GravityCompensator class needed

### Launch command (target)

```bash
conda activate tv
cd xr_teleoperate/teleop
python teleop_hand_and_arm.py \
    --arm=G1_29 \
    --ee=dex3 \
    --input-mode=controller \
    --motion \
    --record \
    --task-name "pick_and_place" \
    --task-goal "pick up the object and place on plate"
```

Or use the convenience script from repo root:
```bash
bash run_xr_teleop.sh controller record motion
```

### Adapter script

`utils/convert_xr_episode.py` converts xr_teleoperate's `EpisodeWriter` output
to the trajectory JSON format consumed by `collect_dataset.py`:

```bash
# Single episode
python utils/convert_xr_episode.py \
    --input xr_recordings/pick_and_place/episode_0001 \
    --output trajectories/xr_ep_0001.json

# Batch — all episodes in a task directory
python utils/convert_xr_episode.py \
    --input xr_recordings/pick_and_place \
    --output trajectories/ \
    --batch
```

---

## 9.5 xr_teleoperate Recording Format (EpisodeWriter)

xr_teleoperate uses its own recording format via `EpisodeWriter` — **NOT** LeRobot v2.
Recorded data must be converted before entering the LeRobot training pipeline.

### 9.5.1 Directory structure per episode

```
{task_dir}/{task_name}/
  episode_0001/
    data.json               ← structured JSON (joint data + image file paths)
    colors/                 ← RGB images (one JPG per camera per frame)
      000000_color_0.jpg        head left eye (or mono)
      000000_color_1.jpg        head right eye (binocular only)
      000000_color_2.jpg        left wrist camera (optional)
      000000_color_3.jpg        right wrist camera (optional)
      000001_color_0.jpg
      ...
    depths/                 ← depth images (if depth camera available)
    audios/                 ← audio recordings (PCM 16-bit .npy, if mic available)
  episode_0002/
    ...
```

### 9.5.2 data.json schema

Each frame in `data["data"]` contains:

```json
{
  "idx": 0,
  "colors": {"color_0": "colors/000000_color_0.jpg", ...},
  "depths": {},
  "states": {
    "left_arm":  {"qpos": [7 floats], "qvel": [], "torque": []},
    "right_arm": {"qpos": [7 floats], "qvel": [], "torque": []},
    "left_ee":   {"qpos": [7 floats], "qvel": [], "torque": []},
    "right_ee":  {"qpos": [7 floats], "qvel": [], "torque": []},
    "body":      {"qpos": []}
  },
  "actions": {
    "left_arm":  {"qpos": [7 floats], ...},
    "right_arm": {"qpos": [7 floats], ...},
    "left_ee":   {"qpos": [7 floats], ...},
    "right_ee":  {"qpos": [7 floats], ...},
    "body":      {"qpos": []}
  }
}
```

**`states`** = actual robot joint positions read from DDS (`arm_ctrl.get_current_dual_arm_q()`)
**`actions`** = IK target joint positions from the solver (`arm_ik.solve_ik()`)

For Dex3-1 hands:
- `states.left_ee.qpos` / `right_ee.qpos` = actual hand motor positions (from DDS)
- `actions.left_ee.qpos` / `right_ee.qpos` = retargeted hand targets

### 9.5.3 Which cameras get recorded?

Images are captured from teleimager's ZMQ stream. Which cameras get recorded is
determined by `cam_config_server.yaml` on PC2 — any camera with `enable_zmq: true`
is available for recording. The `color_N` index assignment depends on the head
camera mode:

**Binocular head camera** (`binocular: true`):
| `colors` key | Source | Resolution |
|---|---|---|
| `color_0` | Head left eye (left half of stereo image) | 480×640 |
| `color_1` | Head right eye (right half of stereo image) | 480×640 |
| `color_2` | Left wrist camera (if `left_wrist_camera.enable_zmq: true`) | 480×640 |
| `color_3` | Right wrist camera (if `right_wrist_camera.enable_zmq: true`) | 480×640 |

**Monocular head camera** (`binocular: false`):
| `colors` key | Source | Resolution |
|---|---|---|
| `color_0` | Head camera (full frame) | 480×640 |
| `color_1` | Left wrist camera (if enabled) | 480×640 |
| `color_2` | Right wrist camera (if enabled) | 480×640 |

Camera selection is done entirely in `cam_config_server.yaml` on PC2, not in the
teleop launch command. See Section 10.6.1 for how to configure cameras.

### 9.5.4 Data pipeline: xr_teleoperate → LeRobot v2

There are **two paths** to get xr_teleoperate data into LeRobot v2:

**Path A: Joint replay + live camera capture (current pipeline)**
```
xr_teleoperate                    Our pipeline
  data.json ──→ convert_xr_episode.py ──→ trajectory JSON (joints only)
                                                │
                                                ▼
                                        collect_dataset.py
                                        (replays joints on robot,
                                         captures images live from ZMQ camera)
                                                │
                                                ▼
                                        LeRobot v2 (Parquet + MP4)
```
- Pros: Images captured in sync with actual robot motion; camera can be different
- Cons: Requires robot + camera running again for replay

**Path B: Direct conversion with recorded images (future / unitree_IL_lerobot)**
```
xr_teleoperate
  data.json ──→  direct converter  ──→ LeRobot v2 (Parquet + MP4)
  colors/*.jpg       (not yet implemented in our repo;
                      Unitree's unitree_IL_lerobot repo has this)
```
- Pros: No replay needed, use recorded images directly
- Cons: Images are at recording resolution/quality; requires adapter implementation

Unitree provides [unitree_IL_lerobot](https://github.com/unitreerobotics/unitree_IL_lerobot)
which can directly convert `EpisodeWriter` data (including images) to LeRobot format.
This is referenced in xr_teleoperate's README. If we want to skip the replay step
in the future, we should integrate that converter.

**Current decision: use Path A** (joint replay + live camera) because our
`collect_dataset.py` already works and captures images in the correct format.

---

## 10. humanoid-pc Deployment Guide

Step-by-step instructions for setting up `xr_teleoperate` on the robot-connected
machine (humanoid-pc). This involves **three devices**:

- **humanoid-pc** (Host): runs `teleop_hand_and_arm.py`, connects to PICO via WiFi
- **PC2** (192.168.123.164): the G1's onboard computing unit, runs `teleimager-server`
- **PICO 4 Ultra**: VR headset, connects to both Host and PC2 via browser

### 10.1 System Architecture

```
                            WiFi (same network)
     ┌──────────────────────────────────────────────────────────┐
     │                                                          │
     ▼                                                          ▼
┌─────────────┐                                          ┌─────────────────┐
│ PICO 4 Ultra│                                          │ humanoid-pc     │
│  (Browser)  │                                          │ (Host)          │
│             │                                          │                 │
│ WebRTC #1:  │◄── hand/controller ────────────────────►│ TeleVuer :8012  │
│   :8012     │    tracking data                         │                 │
│             │                                          │ teleop_hand_    │
│ WebRTC #2:  │◄── camera feed ──┐                       │   and_arm.py    │
│   :60001    │                  │                       │       │         │
└─────────────┘                  │                       │       │ ZMQ    │
                                 │                       │       ▼ (imgs) │
                          ┌──────┴────────┐              │  ImageClient    │
                          │  PC2          │  Ethernet     │       │         │
                          │  (G1 onboard) │◄────────────►│       │ DDS    │
                          │  192.168.123  │  .164 ↔ .222 │       ▼ (cmds) │
                          │  .164         │              │  ArmController  │
                          │               │              │  HandController │
                          │ teleimager    │              └─────────────────┘
                          │  -server      │
                          │  :55555 (ZMQ) │
                          │  :60001 (RTC) │
                          │               │
                          │ cameras:      │
                          │  head (bino)  │
                          │  L wrist (opt)│
                          │  R wrist (opt)│
                          └───────────────┘
```

**Key insight vs old pipeline**: The old `controller_teleop.py` system didn't use
cameras at all — PICO showed nothing from the robot. With xr_teleoperate, the PICO
displays the robot's first-person camera view via WebRTC, which requires
**teleimager running on PC2**.

### 10.2 Prerequisites

- Ubuntu 22.04 on humanoid-pc
- Ethernet connection: humanoid-pc (`192.168.123.222`) ↔ G1 robot/PC2 (`192.168.123.164`)
- WiFi connection shared between humanoid-pc and PICO 4 Ultra
- SSH access to PC2: `ssh unitree@192.168.123.164`
- This repo cloned with submodules on humanoid-pc

### 10.3 Sync the repo on humanoid-pc

```bash
cd ~/Robot_Learning_Infra-WashU_Humanoid_Robot_Group
git pull
git submodule update --init --recursive --depth 1
```

### 10.4 Create conda environment `tv` on humanoid-pc

```bash
conda create -n tv python=3.10 pinocchio=3.1.0 numpy=1.26.4 -c conda-forge -y
conda activate tv

# Core dependencies
pip install casadi meshcat sshkeyboard matplotlib rerun-sdk logging_mp

# xr_teleoperate requirements
pip install -r xr_teleoperate/requirements.txt

# unitree_sdk2_python (must be commit >= 404fe44d)
# If already cloned:
cd ~/unitree_sdk2_python && git pull && pip install -e .
# If not cloned:
# git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
# cd unitree_sdk2_python && pip install -e .

# Install teleimager CLIENT (on humanoid-pc we only need the client)
cd ~/Robot_Learning_Infra.../xr_teleoperate/teleop/teleimager
pip install -e . --no-deps

# Install televuer submodule (XR communication)
cd ~/Robot_Learning_Infra.../xr_teleoperate/teleop/televuer
pip install -e .
```

### 10.5 Supported XR devices and SSL certificates

xr_teleoperate uses standard WebXR (via [TeleVuer/Vuer](https://docs.vuer.ai)) and
supports **four XR headsets**. The code path after connection is identical — the only
difference is how each device trusts the self-signed HTTPS certificate.

| XR Device | Input Modes | Browser | Certificate Method |
|---|---|---|---|
| **PICO 4 Ultra Enterprise** | `hand` + `controller` | PICO Browser | Simple self-signed; click "Proceed" |
| **Meta Quest 3** | `hand` + `controller` | Quest Browser | Simple self-signed; click "Proceed" |
| **Meta Quest 3S** | `hand` + `controller` | Quest Browser | Simple self-signed; click "Proceed" |
| **Apple Vision Pro** | `hand` only (no controllers) | Safari | Full CA chain; install rootCA via AirDrop |

> **We use PICO 4 Ultra Enterprise**, so the simple certificate method below applies.
> If you ever switch to Apple Vision Pro, see the "Apple Vision Pro" section further down.

#### 10.5.1 PICO / Quest — simple self-signed certificate

```bash
cd ~/Robot_Learning_Infra.../xr_teleoperate/teleop/televuer

# Generate self-signed certificate (valid 1 year)
openssl req -x509 -nodes -days 365 -newkey rsa:2048 \
    -keyout key.pem -out cert.pem \
    -subj "/CN=xr-teleoperate"

# Install on humanoid-pc
mkdir -p ~/.config/xr_teleoperate/
cp cert.pem key.pem ~/.config/xr_teleoperate/

# Copy to PC2 (teleimager needs the same certs for its WebRTC)
scp cert.pem key.pem unitree@192.168.123.164:~/
ssh unitree@192.168.123.164 'mkdir -p ~/.config/xr_teleoperate/ && mv ~/cert.pem ~/key.pem ~/.config/xr_teleoperate/'

# Open firewall on humanoid-pc
sudo ufw allow 8012/tcp
```

When PICO/Quest browser visits `https://...:8012` or `https://...:60001`, it will
show a certificate warning. Tap **"Advanced" → "Proceed to site (unsafe)"**.
This only needs to be done once per certificate.

#### 10.5.2 Apple Vision Pro — CA chain certificate (reference)

Safari on Vision Pro **refuses** the "click to proceed" flow. Instead you must
create a proper CA chain and install the root certificate on the device.

```bash
cd ~/Robot_Learning_Infra.../xr_teleoperate/teleop/televuer

# 1. Create a root CA
openssl genrsa -out rootCA.key 2048
openssl req -x509 -new -nodes -key rootCA.key -sha256 -days 365 \
    -out rootCA.pem -subj "/CN=xr-teleoperate"

# 2. Create server key + CSR
openssl genrsa -out key.pem 2048
openssl req -new -key key.pem -out server.csr -subj "/CN=localhost"

# 3. Create server_ext.cnf with your IPs
cat > server_ext.cnf << 'EOF'
subjectAltName = @alt_names
[alt_names]
DNS.1 = localhost
IP.1 = 192.168.123.164
IP.2 = 192.168.1.100
EOF
# ↑ IP.2 must be your humanoid-pc WiFi IP (check with hostname -I)

# 4. Sign server certificate with root CA
openssl x509 -req -in server.csr -CA rootCA.pem -CAkey rootCA.key \
    -CAcreateserial -out cert.pem -days 365 -sha256 -extfile server_ext.cnf

# 5. Install cert.pem + key.pem on humanoid-pc and PC2 (same as 10.5.1)
mkdir -p ~/.config/xr_teleoperate/
cp cert.pem key.pem ~/.config/xr_teleoperate/
scp cert.pem key.pem unitree@192.168.123.164:~/
ssh unitree@192.168.123.164 'mkdir -p ~/.config/xr_teleoperate/ && mv ~/cert.pem ~/key.pem ~/.config/xr_teleoperate/'

# 6. AirDrop rootCA.pem to the Apple Vision Pro
#    On Vision Pro: Settings → General → VPN & Device Management → Install the profile
```

#### 10.5.3 Device feature comparison

| Feature | PICO 4 Ultra | Quest 3/3S | Vision Pro |
|---|---|---|---|
| Hand tracking (finger joints) | Yes | Yes | Yes |
| Controller tracking (6DoF) | Yes | Yes | No controllers |
| `--input-mode=controller` | Yes | Yes | No — use `hand` only |
| `--display-mode=immersive` | Yes | Yes | Yes |
| `--display-mode=pass-through` | Yes | Yes | Yes (spatial) |
| Locomotion via joystick (`--motion`) | Yes (controller mode) | Yes (controller mode) | Not possible |
| E-stop (both sticks pressed) | Yes (controller mode) | Yes (controller mode) | Not possible |
| Exit teleop (A button) | Yes (controller mode) | Yes (controller mode) | Not possible |
| Recommended WiFi | WiFi 6+ | WiFi 6+ | WiFi 6+ |
| Price range | ~$600 Enterprise | ~$500 | ~$3,500 |

> **For data collection with `--motion` (locomotion while teleoperating), a
> controller-capable device (PICO or Quest) is required.** Vision Pro can only
> do stationary arm/hand teleoperation.

### 10.6 Set up teleimager on PC2 (G1 onboard computer)

**This step is CRITICAL.** `teleop_hand_and_arm.py` connects to an `ImageClient`
at startup (line 119: `ImageClient(host='192.168.123.164')`). If teleimager is not
running on PC2, the program will fail.

```bash
# SSH into PC2
ssh unitree@192.168.123.164

# Install system dependencies for camera access
sudo apt install -y libusb-1.0-0-dev libturbojpeg-dev

# Create conda env for teleimager
conda create -n teleimager python=3.10 -y
conda activate teleimager

# Option A: Install from the repo submodule (if repo is cloned on PC2)
cd ~/Robot_Learning_Infra.../xr_teleoperate/teleop/teleimager
pip install -e ".[server]"

# Option B: Clone standalone
git clone https://github.com/silencht/teleimager.git ~/teleimager
cd ~/teleimager
pip install -e ".[server]"

# Set up USB camera permissions (one-time)
bash setup_uvc.sh
```

#### 10.6.1 Configure cameras: `cam_config_server.yaml`

The camera config file tells teleimager which cameras to use and how to stream them.
Copy the template and edit for your G1's actual camera hardware:

```bash
# On PC2:
cd ~/teleimager   # or ~/Robot_Learning_Infra.../xr_teleoperate/teleop/teleimager
cp cam_config_server.yaml cam_config_server.yaml.bak
```

Edit `cam_config_server.yaml`:

```yaml
# === Head camera (binocular, e.g. ZED Mini / Unitree stereo) ===
head_camera:
  enable_zmq: true          # ZMQ stream to humanoid-pc (for recording + IK)
  zmq_port: 55555
  enable_webrtc: true       # WebRTC stream to PICO (for VR immersive view)
  webrtc_port: 60001
  webrtc_codec: h264        # h264 is preferred for PICO; use vp8 as fallback
  type: uvc                 # "uvc", "opencv", or "realsense"
  image_shape: [480, 1280]  # [height, width] — 1280 wide for binocular (640×2)
  binocular: true           # true for stereo cameras
  fps: 30
  # Identify YOUR camera — run `v4l2-ctl --list-devices` on PC2 to find these:
  video_id: 0               # /dev/video0 — change to match your camera
  serial_number: null        # or use serial number for stable identification
  physical_path: null        # or use sysfs path (most stable across reboots)

# === Left wrist camera (optional) ===
left_wrist_camera:
  enable_zmq: true
  zmq_port: 55556
  enable_webrtc: false      # wrist cams usually don't need WebRTC
  webrtc_port: 60002
  webrtc_codec: h264
  type: uvc
  image_shape: [480, 640]
  binocular: false
  fps: 30
  video_id: 2               # change to match
  serial_number: null
  physical_path: null

# === Right wrist camera (optional) ===
right_wrist_camera:
  enable_zmq: true
  zmq_port: 55557
  enable_webrtc: false
  webrtc_port: 60003
  webrtc_codec: h264
  type: uvc
  image_shape: [480, 640]
  binocular: false
  fps: 30
  video_id: 4               # change to match
  serial_number: null
  physical_path: null
```

**How to find your camera devices on PC2:**
```bash
# List all video devices
v4l2-ctl --list-devices

# Or check /dev/video*
ls -la /dev/video*

# For RealSense cameras:
rs-enumerate-devices
```

#### 10.6.2 Start teleimager on PC2

```bash
ssh unitree@192.168.123.164
conda activate teleimager
cd ~/teleimager   # or wherever cam_config_server.yaml lives

# Start the image server
python -m teleimager.image_server
# Or equivalently:
teleimager-server
```

You should see log output indicating cameras are discovered and streams started.

#### 10.6.3 Verify teleimager is working

From humanoid-pc, test the ZMQ image stream:
```bash
conda activate tv
cd ~/Robot_Learning_Infra.../xr_teleoperate/teleop/teleimager/src
python -m teleimager.image_client --host 192.168.123.164
```

If images are received, the ZMQ link is working.

To test WebRTC (camera stream for PICO), open a browser on any device and visit:
`https://192.168.123.164:60001` — you should see a "Start" button. Click it
to preview the head camera.

### 10.7 Network configuration

```
┌─────────────────┐        WiFi         ┌──────────────────────┐
│ PICO 4 Ultra    │◄───────────────────►│ WiFi Router          │
│                 │                      └──────────┬───────────┘
│ Browser tabs:   │                                 │ WiFi
│  :8012 (teleVuer│                        ┌────────▼───────────┐
│  :60001 (WebRTC)│                        │ humanoid-pc (Host)  │
└─────────────────┘                        │  WiFi: e.g.         │    Ethernet
                                           │    192.168.1.100    │◄──────────►  G1
                                           │  Eth: 192.168.123   │         PC2: 192.168.123.164
                                           │       .222          │
                                           └─────────────────────┘
```

Requirements:
- PICO and humanoid-pc must be on **the same WiFi network**
- Enterprise/campus WiFi with client isolation will NOT work — use a personal router
- humanoid-pc Ethernet to G1: `192.168.123.222` ↔ `192.168.123.164`
- Note your WiFi IP: `hostname -I` (e.g. `192.168.1.100`)
- Firewall: ports 8012 (TeleVuer) and 60001 (WebRTC) must be open

```bash
sudo ufw allow 8012/tcp    # TeleVuer (humanoid-pc)
# On PC2:
ssh unitree@192.168.123.164 'sudo ufw allow 60001/tcp'  # WebRTC camera
```

### 10.8 PICO certificate trust (two-step, first-time only)

PICO's browser must trust two self-signed HTTPS endpoints. This only needs to be
done once per certificate (repeat if you regenerate certs).

**Step 1 — Trust teleimager WebRTC cert (camera stream from PC2):**
1. On PICO, open the built-in browser (PICO Browser)
2. Navigate to: `https://192.168.123.164:60001`
3. You will see a security warning. Tap **"Advanced"** → **"Proceed to site (unsafe)"**
4. If you see a page with a "Start" button and can preview the camera, it's working

**Step 2 — Trust TeleVuer cert (hand tracking from humanoid-pc):**
1. Navigate to: `https://{HUMANOID_PC_WIFI_IP}:8012/?ws=wss://{HUMANOID_PC_WIFI_IP}:8012`
2. Tap **"Advanced"** → **"Proceed to site (unsafe)"**
3. Click **"Virtual Reality"** and allow all permission prompts
4. You should enter a VR session with the robot's camera feed

> **Why two steps?** The PICO connects to two separate HTTPS servers:
> teleimager on PC2 for the camera feed, and TeleVuer on humanoid-pc for hand
> tracking. Each needs its certificate trusted independently. If you skip Step 1,
> the VR view will work but you won't see any camera feed.

### 10.9 Complete startup checklist

```
□  1. Power on G1 robot, wait for boot
□  2. Stand up the robot (hand controller: L1+A → L1+UP for Regular mode R1+X)
□  3. Verify Ethernet: ping 192.168.123.164 from humanoid-pc
□  4. Ensure humanoid-pc and PICO are on the same WiFi
□  5. Note humanoid-pc WiFi IP: hostname -I  (e.g. 192.168.1.100)

--- PC2 (SSH from humanoid-pc) ---
□  6. ssh unitree@192.168.123.164
□  7. conda activate teleimager
□  8. Start teleimager: python -m teleimager.image_server
       (keep this terminal open)

--- humanoid-pc ---
□  9. conda activate tv
□ 10. cd ~/Robot_Learning_Infra-WashU_Humanoid_Robot_Group
□ 11. Start teleoperation:
       bash run_xr_teleop.sh controller motion record
       (or: cd xr_teleoperate/teleop && python teleop_hand_and_arm.py ...)

--- PICO 4 Ultra ---
□ 12. Open PICO Browser
□ 13. (First time only) Visit https://192.168.123.164:60001
       → Accept cert → verify camera preview works
□ 14. Visit https://{WIFI_IP}:8012/?ws=wss://{WIFI_IP}:8012
       → Accept cert → click "Virtual Reality" → allow all prompts
□ 15. You should see the robot's camera feed in VR

--- Back on humanoid-pc terminal ---
□ 16. Align arms to robot's initial pose (see xr_teleoperate docs for reference)
□ 17. Press [r] to start tracking — robot arms follow your movements
□ 18. Press [s] to start recording, [s] again to stop and save
□ 19. Press [q] to quit (arms return to home position over ~5 seconds)
```

### 10.10 Real robot teleoperation commands

```bash
# Option A: convenience script (from repo root)
bash run_xr_teleop.sh controller motion record

# Option B: direct command with full control
conda activate tv
cd xr_teleoperate/teleop
python teleop_hand_and_arm.py \
    --arm=G1_29 \
    --ee=dex3 \
    --input-mode=controller \
    --motion \
    --record \
    --task-name "pick_and_place" \
    --task-goal "pick up the apple and place on plate"
```

### 10.11 Converting recorded data to LeRobot format

After recording episodes with xr_teleoperate:

```bash
conda activate lerobot

# Convert xr_teleoperate episodes to trajectory JSON
python utils/convert_xr_episode.py \
    --input xr_recordings/pick_and_place \
    --output trajectories/ \
    --batch

# Replay and collect into LeRobot v2 dataset
bash run_collect.sh \
    --trajectories trajectories/xr_teleop_*.json \
    --repo-id yuzihaowashu/g1_xr_teleop \
    --task "pick up the apple and place on plate"
```

### 10.12 Safety features comparison (old pipeline vs xr_teleoperate)

| Safety Feature | Old (`teleop_bridge.py`) | xr_teleoperate | Notes |
|---|---|---|---|
| **Velocity limiting** | Delta clamp per step | `clip_arm_q_target()` at 250Hz | xr_teleoperate's is smoother (higher freq) |
| **Startup ramp** | None | `speed_gradual_max()` 5s ramp | Prevents sudden arm movements |
| **Exit behavior** | Stop immediately | `ctrl_dual_arm_go_home()` 5s return | Safer — arms return to rest |
| **EMA smoothing** | Yes (configurable alpha) | `WeightedMovingFilter` + IK regularization | Both effective |
| **Joint limits** | URDF-based clamp | CasADi IK constraints | IK-native, won't exceed limits |
| **Gravity comp** | Pinocchio (separate) | IK feedforward torque (integrated) | Cleaner |
| **E-stop** | Left stick click → freeze | Both sticks pressed → `Damp()` | Both good |
| **Hand limits** | Per-motor Dex3-1 min/max | Dex3 controller in subprocess | Handled internally |

### 10.13 Troubleshooting

| Symptom | Likely Cause | Solution |
|---------|-------------|----------|
| `ImageClient` connection error at startup | teleimager not running on PC2 | SSH to PC2 and start `teleimager-server` (see 10.6) |
| PICO can't reach `https://...:8012` | Wrong IP or firewall | Check WiFi IP with `hostname -I`; `sudo ufw allow 8012` |
| VR works but no camera feed | Didn't trust teleimager cert | Visit `https://192.168.123.164:60001` on PICO first (see 10.8 Step 1) |
| "Certificate error" in PICO browser | Expected for self-signed | Click "Advanced" → "Proceed" |
| Camera shows but upside down / wrong | Camera config mismatch | Edit `cam_config_server.yaml` on PC2 (see 10.6.1) |
| No VR session starts | Browser compatibility | Use PICO's built-in browser, not a sideloaded one |
| `ChannelFactoryInitialize` error | DDS network issue | Add `--network-interface=eth0` (or your interface name) |
| Robot doesn't move | Not in motion/debug mode | Start robot: L1+A → L1+UP; or add `--motion` flag |
| IK solver slow / warnings | CasADi not installed | `pip install casadi` in the `tv` env |
| `ModuleNotFoundError: logging_mp` | Missing dependency | `pip install logging_mp` |
| `ModuleNotFoundError: teleimager` | Client not installed on Host | `cd xr_teleoperate/teleop/teleimager && pip install -e . --no-deps` |
| Arms jerk on startup | Normal — velocity ramp | xr_teleoperate has 5s gradual speed increase built in |
| `v4l2-ctl` shows no cameras on PC2 | USB not connected / permission | Run `bash setup_uvc.sh` and reconnect USB |

### 10.14 Comparison: old vs new pipeline

```
OLD (deprecated):
  PC Service:  xrobotoolkit-pc-service (background)
  Terminal 1:  conda activate gmr  → python controller_teleop.py
  Terminal 2:  conda activate lerobot → python teleop_bridge.py
  PICO:        XRobot app → enter WiFi IP → Connect → Start streaming
  Needs:       Redis, XRoboToolkit PC Service .deb, XRobot APK (ADB sideload),
               2 conda envs, 2+ processes, NO camera feed in VR

NEW (xr_teleoperate):
  PC2:         conda activate teleimager → teleimager-server
  Terminal 1:  conda activate tv → bash run_xr_teleop.sh controller record motion
  PICO:        Built-in browser → https://{IP}:8012 → Virtual Reality
  Needs:       1 conda env on Host, 1 conda env on PC2 (teleimager),
               SSL certs, browser-based VR connection WITH camera feed
```
