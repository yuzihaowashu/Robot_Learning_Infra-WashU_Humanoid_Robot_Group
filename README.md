# WashU Humanoid Robot Group — Unitree G1 Infrastructure

Control the Unitree G1 humanoid robot's dual arms, Dex3-1 dexterous hands,
and run Vision-Language-Action (VLA) inference via NVIDIA GR00T N1.6.

## Features

| Feature | Entry Point | Docs |
|---------|-------------|------|
| **VLA Inference** — GR00T N1.6 zero-shot control | `bash run_vla.sh` | [docs/vla_inference.md](docs/vla_inference.md) |
| **Data Collection** — Record LeRobot datasets from demos | `bash run_collect.sh` | [docs/data_collection.md](docs/data_collection.md) |
| **Drag-and-Teach** — Record & replay arm trajectories | `bash run_teach.sh` | [docs/teach_and_replay.md](docs/teach_and_replay.md) |
| **VR Teleoperation** — PICO 4 Ultra VR → upper body control | `bash run_teleop.sh` | [docs/teleoperation.md](docs/teleoperation.md) |
| **Dashboard** — Real-time GUI with camera, joints, tactile | `bash run_dashboard.sh` | [docs/dashboard.md](docs/dashboard.md) |
| **Tactile Sensors** — Dex3 pressure sensor visualization | Integrated in dashboard | [docs/tactile_sensors.md](docs/tactile_sensors.md) |
| **Setup & Arm Control** — Low-level SDK, joint maps, limits | — | [docs/setup_and_arm_control.md](docs/setup_and_arm_control.md) |

## Quick Start

### Prerequisites

- Conda environment `lerobot` (Python 3.12) with Unitree SDK2, Pinocchio
- GR00T `uv` venv (Python 3.10) at `Isaac-GR00T/.venv` (for VLA only)
- Robot powered on and connected (IP: `192.168.123.164`)

### 1. VLA Inference (GR00T N1.6)

```bash
# Terminal 1: Start GPU inference server (loads model, ~30s)
bash run_vla.sh server

# Terminal 2: Start robot client (step-by-step approval by default)
bash run_vla.sh client --task "pick up the apple and place on plate"
```

Step-by-step mode shows each action's target angles in degrees and waits
for your approval before execution. See [VLA docs](docs/vla_inference.md).

### 2. Drag-and-Teach + Data Collection

```bash
# Step 1: Record trajectories by physically moving the arms
bash run_teach.sh record

# Step 2: Collect LeRobot dataset by replaying trajectories
bash run_collect.sh --task "pick up the apple" trajectories/traj_*.json

# Step 3: Visualize the dataset locally
python -m lerobot.scripts.lerobot_dataset_viz \
    --repo-id yuzihaowashu/g1_pick_apple --root ./datasets/...
```

See [Teach & Replay docs](docs/teach_and_replay.md) and
[Data Collection docs](docs/data_collection.md).

### 3. VR Teleoperation (PICO 4 Ultra)

Record demonstrations by teleoperating the robot's upper body with a VR
headset.  Uses [TWIST2](https://github.com/amazon-far/TWIST2) for PICO VR
reading and motion retargeting, connected to our control pipeline via Redis.

```bash
# Quick test without any hardware
bash run_teleop.sh mock record

# Live teleoperation (requires PICO + Redis + robot)
# Terminal 1: Start PICO PC Service app
# Terminal 2: Start TWIST2 teleop
conda activate gmr && cd TWIST2 && bash teleop.sh
# Terminal 3: Start bridge + record
bash run_teleop.sh record
```

Recorded trajectories are in the same JSON format as drag-and-teach, so
the downstream `collect_dataset.py` → LeRobot → GR00T pipeline works with
zero changes.  See [Teleoperation docs](docs/teleoperation.md) for full
setup and design details.

### 4. Dashboard

```bash
# Launch with auto camera setup
bash run_dashboard.sh
```

See [Dashboard docs](docs/dashboard.md).

## Robot Controller Sequence

Before any arm control, the robot must be standing with the balance controller active:

```
Remote Controller:
  (1) L2 + Y  and  L2 + B    — Power on
  (2) L2 + UP                 — Joints to home position
  (3) R1 + X                  — Activate balance controller (robot stands)

After usage:
  (1) L2 + UP                 — Home position (no controller)
  (2) L2 + B                  — Power off (MUST BE HANGED!)
```

## Project Structure

```
.
├── run_vla.sh                    # VLA inference (server + client)
├── run_collect.sh                # Data collection (replay → LeRobot)
├── run_teach.sh                  # Drag-and-teach (record + replay)
├── run_teleop.sh                 # VR teleoperation bridge
├── run_dashboard.sh              # GUI dashboard + camera
├── trajectories/                 # Saved trajectory JSON files
├── datasets/                     # LeRobot v2 datasets
├── Isaac-GR00T/                  # [submodule] NVIDIA GR00T N1.6
├── lerobot/                      # [submodule] HuggingFace LeRobot
├── TWIST2/                       # [submodule] PICO VR teleop + GMR
├── docs/
│   ├── vla_inference.md          # VLA pipeline details
│   ├── data_collection.md        # Dataset collection from demos
│   ├── teach_and_replay.md       # Drag-and-teach details
│   ├── teleoperation.md          # VR teleoperation design & setup
│   ├── dashboard.md              # Dashboard GUI details
│   ├── tactile_sensors.md        # Dex3 tactile sensor details
│   └── setup_and_arm_control.md  # Low-level SDK and joint reference
├── utils/
│   ├── vla_client.py             # GR00T ↔ G1 bridge (DDS + ZMQ)
│   ├── collect_dataset.py        # LeRobot dataset collection
│   ├── teleop_bridge.py          # TWIST2 Redis → G1 DDS bridge
│   ├── dashboard.py              # Tkinter GUI (joints, camera, tactile)
│   ├── teach.py                  # Drag-and-teach recorder
│   ├── replay.py                 # Trajectory replay with gravity comp.
│   ├── robot_camera_server.py    # Camera server (deployed to robot)
│   ├── test_tactile.py           # Raw tactile sensor debugging
│   ├── arm_demo.py               # Choreographed arm motion sequences
│   ├── visualize_workspace.py    # Arm workspace envelope (URDF)
│   ├── test_arm_control.py       # Basic arm control test
│   ├── start_camera.sh           # Manual camera start
│   └── stop_camera.sh            # Manual camera stop
└── tests/
    └── test_teleop_bridge.py     # Teleop bridge unit tests (49 tests)
```

## Architecture

```
                        ┌─────────────────────────────────┐
                        │    Host PC (RTX 4080, 16GB)     │
                        │                                 │
  ┌─────────────┐       │  ┌───────────┐   ┌───────────┐ │
  │ GR00T N1.6  │◄─ZMQ──┤  │VLA Client │   │ Dashboard │ │
  │ PolicyServer│──────►─┤  │(vla_client│   │(dashboard │ │
  │  (GPU)      │       │  │   .py)    │   │   .py)    │ │
  └─────────────┘       │  └─────┬─────┘   └─────┬─────┘ │
                        │        │                │       │
                        │   DDS  │           DDS  │       │
  ┌─────────────┐       │        │                │       │
  │ PICO 4 Ultra│       │  ┌─────┴─────────────┐  │       │
  │ VR Headset  │       │  │  teleop_bridge.py  ◄─Redis   │
  │ + TWIST2    │──Redis──►│  (VR → upper body) │  │       │
  │ (gmr env)   │       │  └─────┬──────────────┘  │       │
  └─────────────┘       │        │                 │       │
                        │   DDS  │                 │       │
                        └────────┼─────────────────┼───────┘
                                 │                 │
                        ┌────────▼─────────────────▼───────┐
                        │         Unitree G1 Robot         │
                        │  rt/arm_sdk ← arm commands       │
                        │  rt/lowstate → joint feedback    │
                        │  rt/dex3/*/cmd ← hand commands   │
                        │  rt/dex3/*/state → tactile data  │
                        │  ZMQ:5555 → camera stream        │
                        └──────────────────────────────────┘
```

## Data Collection Workflows

Two paths to create training datasets, both producing the same LeRobot v2
format:

```
Path A: Drag-and-Teach                 Path B: VR Teleoperation
┌─────────────────────┐                ┌─────────────────────┐
│ Physically move arms │                │ Move VR controllers  │
│ teach.py → JSON      │                │ TWIST2 → Redis →     │
│                      │                │ teleop_bridge.py →   │
│                      │                │ JSON                 │
└──────────┬──────────┘                └──────────┬──────────┘
           │                                      │
           │    same JSON format                   │
           ▼                                      ▼
    ┌──────────────────────────────────────────────────┐
    │ collect_dataset.py                               │
    │ replay trajectory + record camera → LeRobot v2   │
    └──────────────────────┬───────────────────────────┘
                           │
                           ▼
                 ┌──────────────────┐
                 │ LeRobot v2 Dataset│
                 │ (Parquet + MP4)  │
                 └────────┬─────────┘
                          │
                          ▼
                 ┌──────────────────┐
                 │ GR00T Training /  │
                 │ VLA Fine-tuning   │
                 └──────────────────┘
```

## Environment

| Component | Version / Path |
|-----------|---------------|
| Conda env (main) | `lerobot` (Python 3.12) |
| Conda env (TWIST2) | `gmr` (Python 3.10) |
| GR00T venv | `Isaac-GR00T/.venv` (Python 3.10, uv) |
| Unitree SDK2 | `/home/humanoid-pc/unitree_sdk2_python` |
| URDF | `/home/humanoid-pc/unitree_rl_gym/resources/robots/g1_description/` |
| GPU | NVIDIA RTX 4080 (16GB), CUDA 12.6, flash-attn 2.7.4 |
| Robot SSH | `unitree@192.168.123.164` |
| Redis | localhost:6379 (for VR teleoperation) |

## Cloning This Repo

```bash
git clone --recurse-submodules https://github.com/yuzihaowashu/WashU_Humanoid_Robot_Group_Infrastructure.git
cd WashU_Humanoid_Robot_Group_Infrastructure

# If already cloned without submodules:
git submodule update --init
```

This pulls three submodules:
- `Isaac-GR00T/` — NVIDIA GR00T N1.6
- `lerobot/` — HuggingFace LeRobot
- `TWIST2/` — PICO VR teleoperation + GMR retargeting

## Running Tests

```bash
# Teleop bridge tests (no hardware needed)
python -m pytest tests/test_teleop_bridge.py -v
```
