# Teleoperation: PICO 4 Ultra VR → G1 Upper Body

Teleoperate the Unitree G1's upper body (waist, arms, and hands) using a
**PICO 4 Ultra** VR headset with two controllers. Record trajectories for
VLA training in the same format as drag-and-teach, so the entire downstream
pipeline (`collect_dataset.py` → LeRobot → GR00T training) works with zero
changes.

## Table of Contents

- [Why This Design](#why-this-design)
- [System Architecture](#system-architecture)
- [Why `rt/arm_sdk` Instead of `rt/lowcmd`](#why-rtarm_sdk-instead-of-rtlowcmd)
- [What is Redis and Why We Use It](#what-is-redis-and-why-we-use-it)
- [Data Flow in Detail](#data-flow-in-detail)
  - [Hand Finger Remapping](#step-4b-hand-finger-remapping)
- [Safety Measures](#safety-measures)
- [Joint Ordering Verification](#joint-ordering-verification)
- [Prerequisites & Installation](#prerequisites--installation)
- [Usage](#usage)
- [Trajectory Format & Downstream Compatibility](#trajectory-format--downstream-compatibility)
- [Troubleshooting](#troubleshooting)
- [References](#references)

---

## Why This Design

We need to bridge **two incompatible worlds**:

1. **TWIST2** — a verified PICO VR teleoperation system that reads VR
   controller poses, retargets them to G1 joint angles via GMR (General
   Motion Retargeting), and publishes results to Redis.  It runs in a
   `conda gmr` environment with **Python 3.10** (required by newest MuJoCo).
   TWIST2 was designed to drive a full-body RL policy via `rt/lowcmd`.

2. **Our repo** — uses Unitree's built-in balance controller for leg/body
   stability and overlays arm commands via `rt/arm_sdk`.  Runs in a
   `conda lerobot` environment with **Python 3.12**.  Data is recorded as
   JSON trajectories and converted to LeRobot v2 datasets.

Rather than rewriting either system, we use **Redis as a decoupling layer**:
TWIST2 publishes retargeted joint angles, and our `teleop_bridge.py` reads
only the upper-body portion and forwards it to the robot.  This gives us:

- **TWIST2's proven VR pipeline** (PICO SDK, GMR retargeting, MuJoCo
  visualization, state machine, hand control) — without modification
- **Our proven data pipeline** (trajectory JSON → `collect_dataset.py` →
  LeRobot v2 → GR00T training) — without modification
- **Clean separation**: two conda environments, two Python versions, zero
  import conflicts
- **Only upper-body control**: legs and body balance are handled entirely
  by Unitree's locomotion controller, which is safer for tabletop
  manipulation tasks

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│  TWIST2 (conda: gmr, Python 3.10)                                  │
│                                                                     │
│  ┌─────────────────┐     ┌──────────────────────┐                   │
│  │ PICO 4 Ultra    │     │ GMR (General Motion   │                  │
│  │ VR Headset +    │────▶│ Retargeting)          │                  │
│  │ Two Controllers │     │  - SMPL-X body model  │                  │
│  └─────────────────┘     │  - MuJoCo G1 XML      │                  │
│         │                │  - IK retarget to G1   │                  │
│         │                └──────────┬─────────────┘                  │
│   XRoboToolkit SDK                  │                                │
│   (PICO PC Service)                 │ retargeted qpos (36D)          │
│                                     ▼                                │
│                          ┌──────────────────────┐                    │
│                          │ xrobot_teleop_to_     │                   │
│                          │ robot_w_hand.py       │                   │
│                          │  - state machine      │                   │
│                          │  - extract mimic_obs  │                   │
│                          │  - hand pose interp.  │                   │
│                          │  - velocity commands  │                   │
│                          └──────────┬─────────────┘                  │
│                                     │                                │
└─────────────────────────────────────┼────────────────────────────────┘
                                      │ Redis SET
                                      ▼
                         ┌────────────────────────┐
                         │        Redis            │
                         │  (in-memory key-value)  │
                         │                         │
                         │  action_body_*   (35D)  │
                         │  action_hand_l_* (7D)   │
                         │  action_hand_r_* (7D)   │
                         │  t_action        (int)  │
                         └────────────┬────────────┘
                                      │ Redis GET
                                      ▼
┌─────────────────────────────────────────────────────────────────────┐
│  teleop_bridge.py (conda: lerobot, Python 3.12)                     │
│                                                                     │
│  ┌──────────────┐  ┌──────────────┐  ┌─────────────────────────┐   │
│  │ Redis Reader  │─▶│ Joint Safety │─▶│ Robot Sender (DDS)      │   │
│  │               │  │  - URDF limit│  │  - rt/arm_sdk (17 DoF) │   │
│  │ extract upper │  │  - delta clamp│  │  - rt/dex3/*/cmd (14D) │   │
│  │ body [18:35]  │  │  - limit check│ └─────────────────────────┘   │
│  └──────────────┘  └──────────────┘                                 │
│                           │                                          │
│                           ▼                                          │
│                    ┌──────────────┐                                   │
│                    │ Trajectory   │                                   │
│                    │ Recorder     │                                   │
│                    │ (optional)   │                                   │
│                    │ → JSON file  │                                   │
│                    └──────────────┘                                   │
│                           │                                          │
└───────────────────────────┼──────────────────────────────────────────┘
                            │ same JSON format as teach.py
                            ▼
                 ┌──────────────────────┐
                 │ collect_dataset.py   │  (no changes needed)
                 │ replay + record →    │
                 │ LeRobot v2 dataset   │
                 └──────────┬───────────┘
                            │
                            ▼
                 ┌──────────────────────┐
                 │ GR00T / VLA Training │
                 └──────────────────────┘
```

---

## Why `rt/arm_sdk` Instead of `rt/lowcmd`

Unitree G1 offers two DDS control topics:

| | `rt/arm_sdk` (our choice) | `rt/lowcmd` (TWIST2/GR00T) |
|---|---|---|
| What it controls | Upper body only (17 DoF: waist + arms) | All 29 joints directly |
| Balance | Unitree's locomotion controller handles legs | You must provide full-body balance (RL policy) |
| Prerequisite | Locomotion controller active + robot standing | `ReleaseMode()` first |
| Safety | Built-in rejection of unsafe whole-body states | No guardrails |

**TWIST2** was designed for whole-body teleoperation with an RL balance
policy running in `server_low_level_g1_real.py`, so it uses `rt/lowcmd`.
**GR00T WholeBodyControl** similarly uses `rt/lowcmd` because its policies
output all 29 joint targets.

**We use `rt/arm_sdk`** because:
1. We only need upper-body control for tabletop manipulation tasks
2. We get free balance from Unitree's locomotion controller
3. No need to train/deploy an RL balance policy
4. Safer — the robot stays push-resistant during teleoperation

The bridge reads TWIST2's 35D `mimic_obs` from Redis but **only forwards
the upper-body slice** (indices 18-34) and ignores the leg data (indices 6-17).
See [setup_and_arm_control.md](setup_and_arm_control.md#two-control-approaches)
for the full comparison.

---

## What is Redis and Why We Use It

### What is Redis?

[Redis](https://redis.io/) is an open-source, **in-memory key-value store**.
Think of it as a shared dictionary that multiple programs can read from and
write to simultaneously, extremely fast (sub-millisecond latency).

In our setup, Redis runs as a background service on your PC:

```
Program A (TWIST2, Python 3.10)  ──SET "key" "value"──▶  Redis (RAM)
Program B (bridge, Python 3.12)  ──GET "key"──────────▶  Redis (RAM)
                                                          returns "value"
```

### Why Redis instead of alternatives?

| Alternative | Problem |
|-------------|---------|
| Direct function call | TWIST2 needs Python 3.10 (MuJoCo), our repo needs 3.12 — can't share a process |
| ZMQ / TCP socket | Would work, but TWIST2 already publishes to Redis — zero changes needed |
| ROS topics | Heavy dependency; our repo uses DDS directly, not ROS |
| Shared file | Too slow for 50 Hz real-time control |
| Pipe / subprocess | Fragile; hard to debug; ties process lifecycles together |

Redis is the natural choice because **TWIST2 already uses it** as its
communication layer between the teleop process and the low-level controller.
We simply read from the same Redis keys instead of running TWIST2's RL
policy.

### Redis keys used

| Key | Type | Shape | Published by | Read by |
|-----|------|-------|-------------|---------|
| `action_body_unitree_g1_with_hands` | JSON array | (35,) | TWIST2 teleop | `teleop_bridge.py` |
| `action_hand_left_unitree_g1_with_hands` | JSON array | (7,) | TWIST2 teleop | `teleop_bridge.py` |
| `action_hand_right_unitree_g1_with_hands` | JSON array | (7,) | TWIST2 teleop | `teleop_bridge.py` |
| `t_action` | integer | — | TWIST2 teleop | `teleop_bridge.py` (staleness check) |

---

## Data Flow in Detail

### Step 1: PICO → SMPL-X body data

The PICO 4 Ultra headset tracks:
- Head pose (6D: position + orientation)
- Left/right controller poses (6D each)
- Trigger, grip, joystick, button states

TWIST2's `XRobotStreamer` (via the XRoboToolkit PC Service SDK) reads this
data over WiFi from the headset.

### Step 2: SMPL-X → G1 joint angles (GMR)

GMR (General Motion Retargeting) takes the human body pose and retargets it
to the G1 robot morphology:
- Human arm reach → scaled to G1 arm length
- Human joint angles → mapped to G1 joint limits
- Result: `qpos` (36D) = root_pos(3) + root_quat(4) + 29 joint angles

### Step 3: Joint angles → mimic_obs (35D)

TWIST2's `extract_mimic_obs_whole_body()` converts `qpos` to a 35D
observation vector:

```
mimic_obs (35 dimensions):
├─ [0:2]   base velocity xy (local frame)     ← ignored by bridge
├─ [2:3]   base height z                      ← ignored
├─ [3:5]   roll, pitch                        ← ignored
├─ [5:6]   yaw angular velocity               ← ignored
├─ [6:12]  left leg joint angles (6 DoF)      ← ignored
├─ [12:18] right leg joint angles (6 DoF)     ← ignored
├─ [18:21] waist joint angles (3 DoF)         ← USED → Unitree joints [12,13,14]
├─ [21:28] left arm joint angles (7 DoF)      ← USED → Unitree joints [15:22]
└─ [28:35] right arm joint angles (7 DoF)     ← USED → Unitree joints [22:29]
```

Hand data is separate: 7 motors per hand, interpolated between open/close
poses based on trigger/grip button state.

### Step 4: Bridge extracts upper body

`teleop_bridge.py` reads Redis at 50 Hz and extracts only the upper-body
slice: `mimic_obs[18:35]` → 17 joint angles mapping to:

| mimic_obs index | Unitree joint index | Joint name |
|-----------------|---------------------|------------|
| 18 | 12 | waist_yaw |
| 19 | 13 | waist_roll |
| 20 | 14 | waist_pitch |
| 21 | 15 | left_shoulder_pitch |
| 22 | 16 | left_shoulder_roll |
| 23 | 17 | left_shoulder_yaw |
| 24 | 18 | left_elbow |
| 25 | 19 | left_wrist_roll |
| 26 | 20 | left_wrist_pitch |
| 27 | 21 | left_wrist_yaw |
| 28 | 22 | right_shoulder_pitch |
| 29 | 23 | right_shoulder_roll |
| 30 | 24 | right_shoulder_yaw |
| 31 | 25 | right_elbow |
| 32 | 26 | right_wrist_roll |
| 33 | 27 | right_wrist_pitch |
| 34 | 28 | right_wrist_yaw |

### Step 4b: Hand finger remapping

TWIST2 controls Dex3-1 hands via `unitree_interface` (a C++ binding), while
our bridge uses DDS `rt/dex3/*/cmd` via `unitree_sdk2py`.  These two SDKs
use **different finger orderings** for the left hand:

| Motor index | TWIST2 (`unitree_interface`) Left | Our DDS (`HandCmd_`) Left |
|-------------|----------------------------------|--------------------------|
| 0 | Thumb joint 0 | Thumb joint 0 |
| 1 | Thumb joint 1 | Thumb joint 1 |
| 2 | Thumb joint 2 | Thumb joint 2 |
| 3 | **Middle** joint 0 | **Index** joint 0 |
| 4 | **Middle** joint 1 | **Index** joint 1 |
| 5 | **Index** joint 0 | **Middle** joint 0 |
| 6 | **Index** joint 1 | **Middle** joint 1 |

The right hand uses the same ordering in both SDKs (Thumb → Index → Middle).

The bridge applies a permutation before sending:

```python
HAND_REMAP_LEFT  = [0, 1, 2, 5, 6, 3, 4]  # swap mid↔idx
HAND_REMAP_RIGHT = [0, 1, 2, 3, 4, 5, 6]  # identity (no swap)
```

This remapping is based on TWIST2's `dex_hand_wrapper.py` source code.
**Verify on the real robot** that the correct fingers move when you squeeze
the VR trigger — if wrong, adjust the remap arrays in `teleop_bridge.py`.

### Step 5: Safety clamping → DDS

After extraction and hand remapping, each target position goes through
two safety filters (see [Safety Measures](#safety-measures)) before being
sent to:
- `rt/arm_sdk` — 17 upper-body joints with PD position tracking (kp=60, kd=2)
- `rt/dex3/left/cmd` and `rt/dex3/right/cmd` — 7 hand motors each (kp=1.5, kd=0.2)

### Step 6: Optional trajectory recording

If `--record` is enabled, each frame is saved in the same JSON format as
`teach.py`.  After the session, the file can be fed directly into
`collect_dataset.py` for LeRobot dataset creation.

---

## Safety Measures

Operating a humanoid robot via VR teleoperation requires multiple layers
of protection.  Here is what `teleop_bridge.py` implements:

### 1. URDF Joint Limits (hard clamp)

Every target joint angle is clamped to the physical range defined in the
G1 URDF.  For example:

| Joint | Lower (rad) | Upper (rad) |
|-------|-------------|-------------|
| waist_yaw (12) | -2.618 | 2.618 |
| waist_roll (13) | -0.52 | 0.52 |
| left_shoulder_pitch (15) | -3.089 | 2.670 |
| left_elbow (18) | -1.047 | 2.094 |
| ... | ... | ... |

This prevents any command outside the robot's mechanical range, regardless
of what TWIST2 sends.

### 2. Per-Step Delta Limit (velocity cap)

Even if the target is within URDF limits, a sudden large jump is dangerous.
The bridge enforces:

```
MAX_DELTA_PER_STEP = 0.08 rad   (at 50 Hz → max 4 rad/s → ~229 deg/s)
```

This means each joint can move at most 0.08 radians per 20ms control step.
If TWIST2 sends a target 1 radian away, it takes ~12.5 steps (0.25s) to
reach it — smooth and predictable.

### 3. Hand Motor Clamping

Hand motor positions are clamped to `[-0.5, 1.5]` radians to stay within
the Dex3 operating range.

### 4. Graceful Shutdown

When you press `Ctrl+C`, the bridge does NOT instantly cut arm_sdk control.
Instead, it:
1. Smoothly ramps down PD gains over 2 seconds
2. Gradually transfers control back to Unitree's balance controller
3. Releases hand motors

This prevents sudden jerks when stopping teleoperation.

### 5. Balance Preservation

The bridge ONLY controls the upper body (joints 12-28 via `rt/arm_sdk`).
Unitree's built-in locomotion controller maintains leg/body balance
throughout.  The robot remains push-resistant during teleoperation.

### 6. What We Do NOT Have (future work)

- **Self-collision avoidance**: TWIST2's RL policy has self-collision
  penalties baked into training.  Since we bypass the RL policy, we don't
  have this.  For upper-body tasks, this is rarely an issue, but crossing
  arms aggressively could cause self-collision.  A future improvement
  would be to add Pinocchio `computeCollisions()` checks before sending.

- **Workspace limits**: No Cartesian workspace boundary is enforced.
  The robot could reach poses that are kinematically valid but awkward.

---

## Joint Ordering Verification

**Critical step before first real-robot use.**

TWIST2 uses a MuJoCo XML (`assets/g1/g1_mocap_29dof.xml`) and our repo uses
Unitree's URDF (`g1_29dof_with_hand_rev_1_0.urdf`).  The 29 joint ordering
should match (both follow Unitree's convention), but you MUST verify this
before running on the real robot:

### Verification procedure

1. Start TWIST2's MuJoCo visualizer and move one arm joint at a time
2. Read the corresponding `mimic_obs` index from Redis
3. Compare with the Unitree DDS `motor_state[j].q` value

```bash
# Quick Redis check (from any terminal)
redis-cli GET action_body_unitree_g1_with_hands
```

```python
# Parse and inspect
import json, redis
r = redis.Redis()
obs = json.loads(r.get("action_body_unitree_g1_with_hands"))
waist = obs[18:21]       # should match motor_state[12:15].q
left_arm = obs[21:28]    # should match motor_state[15:22].q
right_arm = obs[28:35]   # should match motor_state[22:29].q
```

If any joint is swapped or negated, update the mapping in
`extract_upper_body_from_mimic_obs()` in `teleop_bridge.py`.

---

## Prerequisites & Installation

### 1. Redis (one-time)

```bash
sudo apt update && sudo apt install -y redis-server
sudo systemctl enable redis-server
sudo systemctl start redis-server
```

Edit `/etc/redis/redis.conf` if you need cross-machine access:
```
bind 0.0.0.0
protected-mode no
```
Then `sudo systemctl restart redis-server`.

### 2. TWIST2 (already a submodule)

TWIST2 is included as a git submodule at `TWIST2/` in this repo.  Set up
its conda environment:

```bash
# Create the gmr environment for TWIST2
conda create -n gmr python=3.10 -y
conda activate gmr

# Install GMR (General Motion Retargeting)
git clone https://github.com/YanjieZe/GMR.git
cd GMR && pip install -e . && cd ..
conda install -c conda-forge libstdcxx-ng -y
```

### 3. PICO SDK (one-time)

**On your PC (Ubuntu 22.04):**
```bash
# Install the PC Service deb package
sudo dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb

# Build Python binding
conda activate gmr
git clone https://github.com/YanjieZe/XRoboToolkit-PC-Service-Pybind.git
cd XRoboToolkit-PC-Service-Pybind
mkdir -p tmp && cd tmp
git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service.git
cd XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK
bash build.sh
cd ../../../..
mkdir -p lib include
cp tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/PXREARobotSDK.h include/
cp -r tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/nlohmann include/nlohmann/
cp tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/build/libPXREARobotSDK.so lib/
conda install -c conda-forge pybind11
python setup.py install
```

**On PICO 4 Ultra headset**: Install the PICO SDK APK from
[XR-Robotics](https://github.com/XR-Robotics).

### 4. This repo (one new dependency)

```bash
conda activate lerobot
pip install redis[hiredis]
```

---

## Usage

### Quick Test (no hardware needed)

```bash
# Mock mode — synthetic wave motion, no Redis/robot/PICO needed
bash run_teleop.sh mock

# Mock + record a trajectory file
bash run_teleop.sh mock record
```

This validates the entire pipeline: mock data → safety clamping → mock
robot sender → trajectory JSON.

### Live Teleoperation

Open **three terminals**:

**Terminal 1** — Start the PICO PC Service application (from Ubuntu app
launcher: `xrobotoolkit-pc-service`).  Then put on the PICO headset and
ensure it's connected to the same WiFi network as your PC.

**Terminal 2** — Start TWIST2 teleop (publishes to Redis):
```bash
conda activate gmr
cd TWIST2
bash teleop.sh
```
You'll see a MuJoCo window showing the retargeted robot.  Use the right
controller `key_one` to cycle: idle → teleop → pause.

**Terminal 3** — Start the bridge (reads Redis, controls robot):
```bash
# Teleop only (no recording)
bash run_teleop.sh

# Teleop + record trajectory
bash run_teleop.sh record

# Custom output path
python utils/teleop_bridge.py --record -o trajectories/my_task.json

# Custom Redis IP (if Redis is on another machine)
python utils/teleop_bridge.py --redis-ip 192.168.1.100
```

### Controls

| Action | Where | Control |
|--------|-------|---------|
| Start/pause VR tracking | TWIST2 terminal | Right controller `key_one` |
| Exit TWIST2 | TWIST2 terminal | Left controller `key_one` |
| Emergency stop | TWIST2 terminal | Left controller `axis_click` |
| Close/open right hand | VR controller | Right trigger (close) / grip (open) |
| Close/open left hand | VR controller | Left trigger (close) / grip (open) |
| Start/stop recording | Bridge terminal | Press `Enter` |
| Quit bridge | Bridge terminal | `Ctrl+C` |

### After Recording: Create LeRobot Dataset

The recorded trajectory JSON is **identical in format to drag-and-teach**.
Use the existing pipeline:

```bash
# Replay trajectories while recording camera + joint data → LeRobot v2
bash run_collect.sh \
    --trajectories trajectories/teleop_*.json \
    --repo-id yuzihaowashu/g1_vr_teleop \
    --task "pick up the apple and place on plate"
```

---

## Trajectory Format & Downstream Compatibility

The recorded JSON is deliberately identical to `teach.py` output:

```json
{
  "metadata": {
    "record_hz": 50,
    "n_frames": 500,
    "duration_s": 10.0,
    "arm_joints": [15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28],
    "arm_joint_names": {"15": "L_ShoulderPitch", "16": "L_ShoulderRoll", ...},
    "record_hands": true,
    "hand_joint_names": ["L_Thumb0", "L_Thumb1", ..., "R_Middle1"],
    "control_mode": "teleop_bridge",
    "source": "twist2_pico_vr",
    "created": "2026-03-27 14:30:00"
  },
  "frames": [
    {
      "t": 0.0,
      "arm": {"15": -0.5012, "16": 0.3001, "17": 0.0, ...},
      "hand": {"0": 0.1, "1": 0.2, ..., "13": 0.1}
    },
    ...
  ]
}
```

Key differences from drag-and-teach trajectories:
- `control_mode` is `"teleop_bridge"` (vs `"arm_sdk"`)
- `source` is `"twist2_pico_vr"` (new field)
- Everything else is structurally identical

`collect_dataset.py` loads this JSON with its `load_trajectory()` function
and creates the same LeRobot v2 dataset (Parquet + MP4):
```
datasets/<repo-id>/
├── meta/
│   ├── info.json
│   ├── episodes.jsonl
│   ├── tasks.jsonl
│   └── stats.json
├── data/chunk-000/episode_000000.parquet
└── videos/chunk-000/observation.images.ego_view/episode_000000.mp4
```

---

## Troubleshooting

| Symptom | Likely Cause | Solution |
|---------|-------------|----------|
| `redis.ConnectionError` | Redis not running | `sudo systemctl start redis-server` |
| Bridge says "waiting for data" | TWIST2 not publishing | Check TWIST2 terminal; ensure PICO connected |
| Robot doesn't move | Balance mode not active | Power on sequence: L2+Y/B → L2+UP → R1+X |
| Robot doesn't move | arm_sdk not enabled | Check `rt/lowstate` is being received |
| Jerky/vibrating motion | PD gains too high or teleop noise | Lower KP in bridge; add `--smooth` in TWIST2 |
| Wrong joint moves | Joint ordering mismatch | See [Joint Ordering Verification](#joint-ordering-verification) |
| Joint moves in wrong direction | Sign convention differs | Add negation in `extract_upper_body_from_mimic_obs()` |
| Hands don't move | Dex3 not responding | Check `rt/dex3/*/state` subscription |
| Wrong finger moves on left hand | Finger remap mismatch | Adjust `HAND_REMAP_LEFT` in `teleop_bridge.py` (see [Hand Finger Remapping](#step-4b-hand-finger-remapping)) |
| TWIST2 can't find PICO | WiFi/service issue | Ensure PICO and PC on same network; restart PC Service app |

---

## References

- **TWIST2**: [github.com/amazon-far/TWIST2](https://github.com/amazon-far/TWIST2) — Ze et al., "TWIST2: Scalable, Portable, and Holistic Humanoid Data Collection System", arXiv 2025
- **GMR**: [github.com/YanjieZe/GMR](https://github.com/YanjieZe/GMR) — Araujo et al., "Retargeting Matters: General Motion Retargeting for Humanoid Motion Tracking", arXiv 2025
- **XRoboToolkit**: [github.com/XR-Robotics](https://github.com/XR-Robotics) — PICO VR SDK for robotics
- **LeRobot**: [github.com/huggingface/lerobot](https://github.com/huggingface/lerobot) — HuggingFace robot learning framework
- **GR00T**: NVIDIA Isaac GR00T N1.6 — Vision-Language-Action model
