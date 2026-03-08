# WashU Humanoid Robot Group — Unitree G1 Infrastructure

Control the Unitree G1 humanoid robot's dual arms, Dex3-1 dexterous hands,
and run Vision-Language-Action (VLA) inference via NVIDIA GR00T N1.6.

## Features

| Feature | Entry Point | Docs |
|---------|-------------|------|
| **VLA Inference** — GR00T N1.6 zero-shot control | `bash run_vla.sh` | [docs/vla_inference.md](docs/vla_inference.md) |
| **Drag-and-Teach** — Record & replay arm trajectories | `bash run_teach.sh` | [docs/teach_and_replay.md](docs/teach_and_replay.md) |
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

### 2. Drag-and-Teach

```bash
# Record: move arms freely, gravity-compensated
bash run_teach.sh record

# Replay a saved trajectory
bash run_teach.sh replay trajectories/traj_20260306_174754.json
```

See [Teach & Replay docs](docs/teach_and_replay.md).

### 3. Dashboard

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
chongjie.zhang/
├── run_vla.sh                    # VLA inference (server + client)
├── run_teach.sh                  # Drag-and-teach (record + replay)
├── run_dashboard.sh              # GUI dashboard + camera
├── trajectories/                 # Saved trajectory JSON files
├── Isaac-GR00T/                  # [submodule] NVIDIA GR00T N1.6
├── lerobot/                      # [submodule] HuggingFace LeRobot
├── docs/
│   ├── vla_inference.md          # VLA pipeline details
│   ├── teach_and_replay.md       # Drag-and-teach details
│   ├── dashboard.md              # Dashboard GUI details
│   ├── tactile_sensors.md        # Dex3 tactile sensor details
│   └── setup_and_arm_control.md  # Low-level SDK and joint reference
└── utils/
    ├── vla_client.py             # GR00T ↔ G1 bridge (DDS + ZMQ)
    ├── dashboard.py              # Tkinter GUI (joints, camera, tactile)
    ├── teach.py                  # Drag-and-teach recorder
    ├── replay.py                 # Trajectory replay with gravity comp.
    ├── robot_camera_server.py    # Camera server (deployed to robot)
    ├── test_tactile.py           # Raw tactile sensor debugging
    ├── arm_demo.py               # Choreographed arm motion sequences
    ├── visualize_workspace.py    # Arm workspace envelope (URDF)
    ├── test_arm_control.py       # Basic arm control test
    ├── start_camera.sh           # Manual camera start
    └── stop_camera.sh            # Manual camera stop
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
                        └────────┼────────────────┼───────┘
                                 │                │
                        ┌────────▼────────────────▼───────┐
                        │         Unitree G1 Robot        │
                        │  rt/arm_sdk ← arm commands      │
                        │  rt/lowstate → joint feedback    │
                        │  rt/dex3/*/cmd ← hand commands  │
                        │  rt/dex3/*/state → tactile data │
                        │  ZMQ:5555 → camera stream       │
                        └─────────────────────────────────┘
```

## Environment

| Component | Version / Path |
|-----------|---------------|
| Conda env | `lerobot` (Python 3.12) |
| GR00T venv | `Isaac-GR00T/.venv` (Python 3.10, uv) |
| Unitree SDK2 | `/home/humanoid-pc/unitree_sdk2_python` |
| URDF | `/home/humanoid-pc/unitree_rl_gym/resources/robots/g1_description/` |
| GPU | NVIDIA RTX 4080 (16GB), CUDA 12.6, flash-attn 2.7.4 |
| Robot SSH | `unitree@192.168.123.164` |

## Cloning This Repo

```bash
git clone --recurse-submodules https://github.com/yuzihaowashu/WashU_Humanoid_Robot_Group_Infrastructure.git
cd WashU_Humanoid_Robot_Group_Infrastructure

# If already cloned without submodules:
git submodule update --init
```
