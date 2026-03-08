# Setup & Arm Control — Low-Level SDK Reference

> See also: [VLA Inference](vla_inference.md) | [Teach & Replay](teach_and_replay.md) |
> [Dashboard](dashboard.md) | [Tactile Sensors](tactile_sensors.md)

## Machine Overview

- **OS**: Ubuntu 22.04 (Linux 6.8.0)
- **GPU**: NVIDIA GeForce RTX 4080 (16GB)
- **Robot**: Unitree G1 humanoid (29 DOF, `unitree_hg` message type)
- **Robot IPs**: `192.168.123.164`, `192.168.0.231` (user: `unitree`)
- **ROS2**: Humble (installed, 273 packages)
- **DDS**: CycloneDDS (shared between Unitree SDK and ROS2)

## Environment Setup

### Conda Environment

```bash
# Created with Python 3.12 (required by LeRobot >= 0.4.4)
conda create -n lerobot python=3.12 -y
conda activate lerobot
```

### LeRobot Installation

```bash
cd /home/humanoid-pc/chongjie.zhang/lerobot
pip install -e ".[unitree_g1,pi,smolvla]"
```

This installs:
- **LeRobot v0.4.5** (main branch, latest)
- **Unitree G1 deps**: Pinocchio (IK), CasADi, ONNX Runtime, pyzmq
- **VLA model deps**: transformers >= 5.3.0, diffusers, accelerate, scipy

### Unitree SDK2 Python

```bash
# CycloneDDS is installed system-wide at /usr/local
CYCLONEDDS_HOME=/usr/local pip install cyclonedds==0.10.2
pip install -e /home/humanoid-pc/unitree_sdk2_python --no-deps
```

### Verify Installation

```bash
conda activate lerobot
lerobot-info  # Should show LeRobot 0.4.5, PyTorch with CUDA, RTX 4080
```

## Communication Architecture

### DDS Topics (Unitree SDK)

| Topic | Message Type | Purpose |
|-------|-------------|---------|
| `rt/lowcmd` | `unitree_hg.LowCmd_` | **Direct motor commands** (35 motors) |
| `rt/lowstate` | `unitree_hg.LowState_` | Motor state feedback (q, dq, tau, IMU) |
| `rt/arm_sdk` | `unitree_hg.LowCmd_` | Arm overlay commands (requires locomotion active) |

### Two Control Approaches

#### Approach A: `rt/arm_sdk` (High-Level Arm Overlay)
- Publishes arm commands on top of the active locomotion controller
- **Requires**: Locomotion controller running (`SelectMode("ai")` + robot standing)
- **Suitable for**: Robot standing on the ground
- **Does NOT work when**: Robot is hanging or locomotion controller is inactive

#### Approach B: `rt/lowcmd` (Direct Motor Control) ✅ Currently Used
- Sends PD commands directly to motor controllers
- **Requires**: `ReleaseMode()` first (no other controller should be writing to rt/lowcmd)
- **Suitable for**: Robot hanging on test stand, or full-body control
- **Caution**: You are responsible for all joints; no balance controller running

### Control Flow

```
1. ChannelFactoryInitialize(0)          # Init DDS
2. MotionSwitcherClient.ReleaseMode()   # Release locomotion controller
3. ChannelPublisher("rt/lowcmd")        # Create publisher
4. ChannelSubscriber("rt/lowstate")     # Subscribe to state
5. Wait for first LowState message
6. Loop at 50Hz:
   a. Set motor_cmd[joint].mode = 1     # Position mode
   b. Set motor_cmd[joint].q = target   # Target position (rad)
   c. Set motor_cmd[joint].kp / kd      # PD gains
   d. low_cmd.crc = crc.Crc(low_cmd)    # Compute CRC (mandatory)
   e. publisher.Write(low_cmd)          # Send command
```

## G1 Joint Index Map

### Arm Joints (14 DOF total)

| Index | Left Arm | Index | Right Arm |
|-------|----------|-------|-----------|
| 15 | LeftShoulderPitch | 22 | RightShoulderPitch |
| 16 | LeftShoulderRoll | 23 | RightShoulderRoll |
| 17 | LeftShoulderYaw | 24 | RightShoulderYaw |
| 18 | LeftElbow | 25 | RightElbow |
| 19 | LeftWristRoll | 26 | RightWristRoll |
| 20 | LeftWristPitch | 27 | RightWristPitch |
| 21 | LeftWristYaw | 28 | RightWristYaw |

### Waist Joints

| Index | Joint |
|-------|-------|
| 12 | WaistYaw |
| 13 | WaistRoll |
| 14 | WaistPitch |

### Motor Command Fields

```python
motor_cmd[joint].mode  # 1 = position mode
motor_cmd[joint].q     # Target position (rad)
motor_cmd[joint].dq    # Target velocity (rad/s)
motor_cmd[joint].tau   # Feedforward torque (Nm)
motor_cmd[joint].kp    # Position gain (Nm/rad)
motor_cmd[joint].kd    # Velocity gain (Nm·s/rad)
```

### Recommended PD Gains (Hanging Test Stand)

| Parameter | Value | Notes |
|-----------|-------|-------|
| kp | 30.0 | Lower than on-ground (60.0) since no load |
| kd | 1.5 | Damping |

### Joint Limits (from URDF, with 10% safety margin)

Source: `g1_29dof_with_hand_rev_1_0.urdf`

| Index | Joint | Lower (rad) | Upper (rad) | Approx Degrees |
|-------|-------|-------------|-------------|----------------|
| 15 | L ShoulderPitch | -2.78 | +2.40 | -159° ~ +138° |
| 16 | L ShoulderRoll | -1.43 | +2.03 | -82° ~ +116° |
| 17 | L ShoulderYaw | -2.36 | +2.36 | -135° ~ +135° |
| 18 | L Elbow | -0.94 | +1.88 | -54° ~ +108° |
| 19 | L WristRoll | -1.77 | +1.77 | -101° ~ +101° |
| 20 | L WristPitch | -1.45 | +1.45 | -83° ~ +83° |
| 21 | L WristYaw | -1.45 | +1.45 | -83° ~ +83° |
| 22 | R ShoulderPitch | -2.78 | +2.40 | -159° ~ +138° |
| 23 | R ShoulderRoll | -2.03 | +1.43 | -116° ~ +82° |
| 24 | R ShoulderYaw | -2.36 | +2.36 | -135° ~ +135° |
| 25 | R Elbow | -0.94 | +1.88 | -54° ~ +108° |
| 26 | R WristRoll | -1.77 | +1.77 | -101° ~ +101° |
| 27 | R WristPitch | -1.45 | +1.45 | -83° ~ +83° |
| 28 | R WristYaw | -1.45 | +1.45 | -83° ~ +83° |

**Note**: No self-collision detection exists in the SDK or URDF config. MuJoCo models have collision disabled (`contype=0`). Always validate motions visually or in simulation before running on hardware.

### Safety: Self-Collision Avoidance

The SDK and URDF do **not** provide self-collision checking. To avoid collisions:
- Keep shoulder_roll values away from 0 when arms are forward (prevents arms crossing into body)
- Never command both arms inward simultaneously (positive L_SHOULDER_R + negative R_SHOULDER_R while arms are forward)
- Use joint limit clamping (see `JOINT_LIMITS` in `arm_demo.py`)

## MotionSwitcher API

```python
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

msc = MotionSwitcherClient()
msc.SetTimeout(5.0)
msc.Init()

msc.CheckMode()         # Returns (code, {"form": "0", "name": "ai"})
msc.SelectMode("ai")    # Modes: "ai", "normal", "advanced"
msc.ReleaseMode()       # Release current controller
```

**Important**: After `ReleaseMode()`, wait ~2 seconds before re-selecting a mode.

## Camera Streaming

The G1 robot has 6 camera devices (`/dev/video0-5`). Camera is integrated into the dashboard launcher:

```bash
bash run_dashboard.sh              # auto-detect camera + launch dashboard
bash run_dashboard.sh --device 4   # specify camera device
bash run_dashboard.sh --no-camera  # skip camera, dashboard only
```

The launcher:
1. Asks for robot SSH password
2. Deploys `utils/robot_camera_server.py` to the robot
3. Starts it in background (ZMQ PUB on `tcp://0.0.0.0:5555`)
4. Launches the dashboard (auto-connects as ZMQ SUB)
5. On exit, offers to stop the camera server

To manage the camera server independently:
```bash
bash utils/start_camera.sh         # deploy + start
bash utils/stop_camera.sh          # stop
```

### Robot Environment
- **OS**: Ubuntu 22.04 on Jetson (aarch64), Linux 5.10 Tegra
- **Python**: 3.8.10 (system), miniconda with `unifolm-vla` env (Python 3.10)
- **Key packages on robot**: opencv-python, cyclonedds, torch, unitree-sdk2py
- **LeRobot on robot**: v0.3.4 in `/home/unitree/miniconda3/envs/unifolm-vla/`
- **SSH**: `unitree@192.168.123.164`

## GR00T VLA Environment (Python 3.10)

The GR00T inference server requires a separate environment due to Python
version constraints (3.10 required by flash-attn):

```bash
cd Isaac-GR00T
uv sync --python 3.10
```

Key packages: PyTorch 2.7+ (CUDA 12.6), flash-attn 2.7.4, transformers,
unitree-sdk2py (installed in the venv for the client).

## Dex3 Hand Protocol

Each hand has 7 motors. Commands sent via `rt/dex3/{left,right}/cmd`
using `HandCmd_` messages.

### RIS Mode Encoding
```python
mode = motor_id | (status << 4) | (timeout << 7)
# status: 0x01 = enabled
# timeout: 0 = no timeout
```

### Hand Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `rt/dex3/left/cmd` | `HandCmd_` | PC → Robot |
| `rt/dex3/right/cmd` | `HandCmd_` | PC → Robot |
| `rt/dex3/left/state` | `HandState_` | Robot → PC |
| `rt/dex3/right/state` | `HandState_` | Robot → PC |

See [Tactile Sensors](tactile_sensors.md) for pressure sensor details.

## Key External Paths

| Path | Description |
|------|-------------|
| `/home/humanoid-pc/unitree_sdk2_python/` | Unitree SDK2 Python |
| `/home/humanoid-pc/unitree_mujoco/unitree_robots/g1/` | MuJoCo XML models |
| `/home/humanoid-pc/unitree_rl_gym/resources/robots/g1_description/` | URDF models |

## Lessons Learned

1. **`rt/arm_sdk` does not work when robot is hanging** — it needs the locomotion controller to be actively running and the robot to be standing on the ground.

2. **`rt/lowcmd` works in any state** — but requires `ReleaseMode()` first and you control motors directly.

3. **CRC is mandatory** — every `LowCmd_` must have a valid CRC before publishing, or the robot ignores the command.

4. **Smooth transitions** — always ramp `kp`/`kd` up and down gradually to avoid sudden jerks.

5. **`mode_machine` must be echoed back** — read it from `LowState` and set it in `LowCmd` to stay in sync with the robot's internal state machine.
