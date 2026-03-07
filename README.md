# Unitree G1 Arm & Hand Control Workspace

Control the Unitree G1 humanoid robot's dual arms and Dex3-1 dexterous hands
via the Unitree SDK2 Python interface.

## Quick Start

### Prerequisites

- Conda environment `lerobot` (Python 3.12)
- Unitree SDK2 Python installed (`pip install -e /home/humanoid-pc/unitree_sdk2_python`)
- Robot powered on and connected (default IP: `192.168.123.164`)

### 1. Drag-and-Teach (Record & Replay)

Physically move the robot's arms and fingers, record the trajectory, then replay it.

```bash
# Record a new trajectory
#   - Robot must be standing (use hand controller: L1+A → L1+UP)
#   - Arms become compliant — move them freely
#   - Gravity compensation keeps arms weightless
#   - Press [Enter] to start/stop recording, Ctrl+C to quit
bash run_teach.sh record

# Replay a saved trajectory
bash run_teach.sh replay trajectories/traj_20260306_174754.json

# Replay at half speed, loop 3 times
bash run_teach.sh replay trajectories/traj_xxx.json --speed 0.5 --loop 3

# List all saved trajectories
bash run_teach.sh list
```

**How it works:**
- Uses `rt/arm_sdk` to overlay arm control on Unitree's balance controller
  (legs and body stay balanced, push-resistant)
- Arms: `kp=0, kd=1` during teach (compliant), `kp=80, kd=2` during replay
- Gravity compensation via Pinocchio dynamics (`tau_ff = G(q)` from URDF)
- Hands: controlled via `rt/dex3/left/cmd` and `rt/dex3/right/cmd` (7 DOF each)
- Fingers open automatically on exit

### 2. Dashboard (GUI)

Real-time monitoring dashboard with camera feed, joint visualization, and
predefined arm actions.

```bash
# Full launch: deploys camera server to robot + opens dashboard
bash run_dashboard.sh

# Skip camera setup (dashboard only)
bash run_dashboard.sh --no-camera

# Specify camera device on robot
bash run_dashboard.sh --device 4
```

**Features:**
- Live camera stream from robot (ZMQ-based, deployed automatically)
- Real-time joint angle display for all 29 body motors
- Predefined action buttons (Safe Home, Wave, Arms Up, etc.)
- SSH password prompt for automatic camera server deployment

## Project Structure

```
chongjie.zhang/
├── run_teach.sh                  # Entry point: drag-and-teach
├── run_dashboard.sh              # Entry point: GUI dashboard + camera
├── trajectories/                 # Recorded trajectory JSON files
├── docs/
│   ├── setup_and_arm_control.md  # Detailed technical documentation
│   └── g1_arm_workspace.gif      # Arm workspace visualization
├── lerobot/                      # LeRobot repo (v0.4.5, for future VLA)
└── utils/
    ├── teach.py                  # Drag-and-teach recorder (arm_sdk + dex3)
    ├── replay.py                 # Trajectory replay with gravity compensation
    ├── dashboard.py              # Tkinter GUI dashboard
    ├── robot_camera_server.py    # Camera server (deployed to robot via SSH)
    ├── arm_demo.py               # Choreographed arm motion sequences
    ├── visualize_workspace.py    # Arm workspace envelope (Pinocchio + URDF)
    ├── test_arm_control.py       # Basic arm control test
    ├── start_camera.sh           # Manual camera server start
    ├── stop_camera.sh            # Manual camera server stop
    ├── explore_robot.sh          # Robot filesystem exploration via SSH
    └── run_arm_test.sh           # Arm demo launcher
```

## Architecture Overview

### Control Modes

| Mode | Topic | Use Case |
|------|-------|----------|
| **arm_sdk** | `rt/arm_sdk` | Arm overlay on balance controller (robot standing) |
| **lowcmd** | `rt/lowcmd` | Direct motor control (robot hanging / full body) |
| **dex3 hands** | `rt/dex3/{left,right}/cmd` | Dexterous hand control (7 motors each) |

### Communication Stack

```
Host PC (this machine)
  ├── DDS (CycloneDDS) ──────── rt/arm_sdk ──────→ G1 locomotion overlay
  ├── DDS ───────────────────── rt/lowstate ←───── G1 motor feedback
  ├── DDS ───────────────────── rt/dex3/*/cmd ──→ Dex3-1 hand motors
  ├── RPC (via DDS) ─────────── MotionSwitcher ──→ Mode management
  └── ZMQ (tcp:5555) ←───────── Camera server ──── Robot onboard camera
```

### Key Technical Details

- **Gravity compensation**: Pinocchio computes `G(q)` from the URDF at each
  control step. This torque is sent as `tau_ff` (feedforward) to counteract
  gravity, making arms weightless during teaching and droop-free during replay.
- **Dex3 hand protocol**: Each hand has 7 motors. The `mode` field uses RIS
  encoding: `motor_id | (status << 4) | (timeout << 7)`. Default gains:
  `kp=1.5, kd=0.2`.
- **CRC**: Every `LowCmd_` message requires a valid CRC checksum or the robot
  ignores it.
- **Balance**: When using `rt/arm_sdk`, the robot must be in `ai` mode
  (activated via hand controller L1+A, L1+UP). The locomotion controller
  manages legs and body while arm_sdk overlays arm commands.

## Environment

| Component | Version / Path |
|-----------|---------------|
| Conda env | `lerobot` (Python 3.12) |
| Unitree SDK2 | `/home/humanoid-pc/unitree_sdk2_python` |
| URDF | `/home/humanoid-pc/unitree_rl_gym/resources/robots/g1_description/` |
| LeRobot | `./lerobot/` (v0.4.5) |
| Pinocchio | Installed in conda env (dynamics + kinematics) |
| Robot SSH | `unitree@192.168.123.164` |

## Next Steps

- [ ] Integrate VLA model for autonomous arm control
- [ ] Add hand trajectory recording (Dex3 state subscription)
- [ ] Deploy trained policy end-to-end (camera → VLA → arm_sdk)
