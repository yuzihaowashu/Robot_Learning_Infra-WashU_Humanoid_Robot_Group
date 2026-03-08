# Dashboard — Real-Time Robot Monitoring GUI

A Tkinter-based dashboard providing live camera feed, joint state visualization,
tactile sensor readout, and predefined action buttons.

## Usage

```bash
# Full launch: auto-deploys camera server to robot + opens dashboard
bash run_dashboard.sh

# Skip camera setup (dashboard only)
bash run_dashboard.sh --no-camera

# Specify camera device on robot (default: auto-detect)
bash run_dashboard.sh --device 4
```

On first launch, the script:
1. Prompts for the robot's SSH password
2. Deploys `utils/robot_camera_server.py` to the robot
3. Starts the camera server in background (ZMQ PUB on `tcp://0.0.0.0:5555`)
4. Opens the dashboard GUI

## Dashboard Layout

```
┌──────────────────────────────────────────────────────────┐
│                                                          │
│  ┌─────────────────┐   ┌──────────────────────────────┐  │
│  │  Camera Feed    │   │    Joint States (29 DOF)     │  │
│  │  (640×480)      │   │    Real-time bar chart       │  │
│  │                 │   │    Color-coded by group       │  │
│  └─────────────────┘   └──────────────────────────────┘  │
│                                                          │
│  ┌─────────────────┐   ┌──────────────────────────────┐  │
│  │ Tactile Sensors │   │    Action Buttons            │  │
│  │ L/R hand bars   │   │    Safe Home | Wave | ...    │  │
│  │ Pressure + dots │   │                              │  │
│  └─────────────────┘   └──────────────────────────────┘  │
│                                                          │
└──────────────────────────────────────────────────────────┘
```

## Features

### Camera Feed
- Streams from the robot's head camera via ZMQ (base64-encoded JPEG)
- Auto-reconnects if stream drops
- Shows "No Camera" placeholder if unavailable

### Joint State Visualization
- Displays all 29 body motor positions in real-time
- Groups: Left Leg (6), Right Leg (6), Waist (3), Left Arm (7), Right Arm (7)
- Each joint shown as a horizontal bar with current angle in degrees

### Tactile Sensor Visualization
- Reads from `rt/dex3/left/state` and `rt/dex3/right/state`
- Up to 9 sensor modules per hand, each with 12 pressure pads
- Aggregated bar chart per module (filters out disconnected pads)
- Color-coded: cyan → green → yellow → red (by pressure intensity)
- White dots indicate number of active touch points
- See [Tactile Sensors](tactile_sensors.md) for details

### Action Buttons
Predefined arm movements that can be triggered with one click:
- **Safe Home** — Return arms to neutral position
- **Wave** — Wave gesture
- **Arms Forward** — Extend arms forward (offset outward to avoid leg collision)
- **Arms Up** — Raise arms overhead

## Camera Server

The camera server (`utils/robot_camera_server.py`) runs on the robot and streams
images over ZMQ:

- **Protocol**: ZMQ PUB/SUB, port 5555
- **Format**: JSON with base64-encoded JPEG images
- **Payload**: `{"images": {"head_camera": "<base64>"}}`

Manual management:
```bash
bash utils/start_camera.sh    # Deploy + start
bash utils/stop_camera.sh     # Stop
```

## DDS Topics Used

| Topic | Direction | Purpose |
|-------|-----------|---------|
| `rt/lowstate` | Robot → PC | Joint positions, velocities, torques |
| `rt/arm_sdk` | PC → Robot | Arm position commands |
| `rt/dex3/left/state` | Robot → PC | Left hand motor + tactile state |
| `rt/dex3/right/state` | Robot → PC | Right hand motor + tactile state |

## Implementation

- **File**: `utils/dashboard.py`
- **Framework**: Tkinter + OpenCV + PIL
- **Update rate**: ~30 Hz for joints, ~10 Hz for camera, ~5 Hz for tactile
- **Window size**: 1280×800
