# xr_teleoperate — VR Teleoperation for G1

Unitree's official XR teleoperation framework, replacing our previous
`controller_teleop.py` + `teleop_bridge.py` pipeline. Uses CasADi/IPOPT IK
at 250Hz arm control, TeleVuer WebRTC for PICO connection, and built-in
`EpisodeWriter` for data recording.

Repository: <https://github.com/unitreerobotics/xr_teleoperate>
(cloned as git submodule at `xr_teleoperate/`)

---

## Table of Contents

- [Supported XR Devices](#supported-xr-devices)
- [System Architecture](#system-architecture)
- [Recording Format](#recording-format)
  - [Directory Structure](#directory-structure)
  - [data.json Schema](#datajson-schema)
  - [Camera Recording](#camera-recording)
  - [Data Pipeline to LeRobot](#data-pipeline-to-lerobot)
- [Launch Parameters](#launch-parameters)
  - [Convenience Script](#convenience-script)
  - [All Parameters](#all-parameters)
  - [Frequency Explained](#frequency-explained)
  - [Output Directory](#output-directory)
- [Deployment Checklist](#deployment-checklist)
- [SSL Certificates by Device](#ssl-certificates-by-device)
- [Safety Features](#safety-features)

---

## Supported XR Devices

xr_teleoperate uses standard WebXR (via [TeleVuer/Vuer](https://docs.vuer.ai)).
After connection, all devices share the same code path — the only difference is
how the self-signed HTTPS certificate is trusted.

| XR Device | Input Modes | Browser | Certificate Method |
|---|---|---|---|
| **PICO 4 Ultra Enterprise** | `hand` + `controller` | PICO Browser | Simple self-signed; click "Proceed" |
| **Meta Quest 3** | `hand` + `controller` | Quest Browser | Simple self-signed; click "Proceed" |
| **Meta Quest 3S** | `hand` + `controller` | Quest Browser | Simple self-signed; click "Proceed" |
| **Apple Vision Pro** | `hand` only (no controllers) | Safari | Full CA chain; install rootCA via AirDrop |

### Feature Comparison

| Feature | PICO 4 Ultra | Quest 3/3S | Vision Pro |
|---|---|---|---|
| Hand tracking (finger joints) | Yes | Yes | Yes |
| Controller tracking (6DoF) | Yes | Yes | No controllers |
| `--input-mode=controller` | Yes | Yes | No — `hand` only |
| Locomotion via joystick (`--motion`) | Yes | Yes | Not possible |
| E-stop (both sticks pressed) | Yes | Yes | Not possible |
| Exit teleop (A button) | Yes | Yes | Not possible |
| `--display-mode=immersive` | Yes | Yes | Yes |
| `--display-mode=pass-through` | Yes | Yes | Yes (spatial) |
| Recommended WiFi | WiFi 6+ | WiFi 6+ | WiFi 6+ |

> **We use PICO 4 Ultra Enterprise.** For locomotion (`--motion`) during data
> collection, a controller-capable device (PICO or Quest) is required. Vision Pro
> can only do stationary arm/hand teleoperation.

---

## System Architecture

Three devices involved:

```
                            WiFi (same network)
     ┌──────────────────────────────────────────────────────────┐
     │                                                          │
     ▼                                                          ▼
┌─────────────┐                                          ┌─────────────────┐
│ PICO 4 Ultra│                                          │ humanoid-pc     │
│  (Browser)  │                                          │ (Host)          │
│             │                                          │                 │
│ WebRTC #1:  │◄── hand/controller tracking ───────────►│ TeleVuer :8012  │
│   :8012     │                                          │                 │
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
                          └───────────────┘
```

Key differences from old pipeline:
- **No XRoboToolkit** — no `.deb` package, no APK sideloading
- **No Redis** — single-process, no inter-process bridge
- **Camera feed in VR** — PICO shows first-person robot camera view
- **1 conda env** on Host (`tv`), 1 on PC2 (`teleimager`)

---

## Recording Format

xr_teleoperate uses its own `EpisodeWriter` format — **NOT** LeRobot v2 directly.

### Directory Structure

```
{task-dir}/{task-name}/
  episode_0001/
    data.json               ← structured JSON (joints + image paths)
    colors/                 ← RGB images (JPG per camera per frame)
      000000_color_0.jpg        head left eye (or mono)
      000000_color_1.jpg        head right eye (binocular)
      000000_color_2.jpg        left wrist camera (optional)
      000000_color_3.jpg        right wrist camera (optional)
      000001_color_0.jpg
      ...
    depths/                 ← depth images (if depth camera available)
    audios/                 ← audio (.npy, if mic available)
  episode_0002/
    ...
```

### data.json Schema

Top-level:
```json
{
  "info": {
    "version": "1.0.0",
    "date": "2026-03-28",
    "image": {"width": 640, "height": 480, "fps": 30},
    "joint_names": {"left_arm": [], "right_arm": [], "left_ee": [], "right_ee": [], "body": []}
  },
  "text": {
    "goal": "pick up the apple",
    "desc": "tabletop manipulation",
    "steps": "step1: reach; step2: grasp; step3: place"
  },
  "data": [ ... frames ... ]
}
```

Each frame in `data`:
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
  "actions": { "left_arm": ..., "right_arm": ..., "left_ee": ..., "right_ee": ..., "body": ... }
}
```

| Field | Meaning | Source in Code |
|---|---|---|
| `states.left_arm.qpos` | Actual robot joint angles (7 DOF) | `arm_ctrl.get_current_dual_arm_q()` via DDS |
| `actions.left_arm.qpos` | IK target joint angles (7 DOF) | `arm_ik.solve_ik()` output |
| `states.left_ee.qpos` | Actual Dex3 hand motor positions (7) | `dual_hand_state_array` via DDS |
| `actions.left_ee.qpos` | Retargeted hand motor targets (7) | `dual_hand_action_array` |
| `states.body.qpos` | Full-body motor positions | Only in controller+motion mode |
| `actions.body.qpos` | Locomotion velocity commands | `[vx, vy, vyaw]` in controller+motion mode |

### Camera Recording

Which cameras get recorded depends on `cam_config_server.yaml` on PC2 — any camera
with `enable_zmq: true` is captured. The `color_N` key assignment:

**Binocular head camera** (`binocular: true`):
| Key | Source |
|---|---|
| `color_0` | Head left eye (left half of stereo frame) |
| `color_1` | Head right eye (right half of stereo frame) |
| `color_2` | Left wrist camera (if enabled) |
| `color_3` | Right wrist camera (if enabled) |

**Monocular head camera** (`binocular: false`):
| Key | Source |
|---|---|
| `color_0` | Head camera (full frame) |
| `color_1` | Left wrist camera (if enabled) |
| `color_2` | Right wrist camera (if enabled) |

Camera selection is done in `cam_config_server.yaml` on PC2, not in the teleop
launch command. See
[pico_twist_progress.md Section 10.6.1](../todo_docs/pico_twist_progress.md)
for configuration details.

### Data Pipeline to LeRobot

Two paths exist:

**Path A: Joint replay + live camera (current)**
```
EpisodeWriter data.json
    → convert_xr_episode.py → trajectory JSON (joints only)
        → collect_dataset.py (replay on robot, capture live camera)
            → LeRobot v2 (Parquet + MP4)
```

**Path B: Direct conversion with recorded images (future)**
```
EpisodeWriter data.json + colors/*.jpg
    → unitree_IL_lerobot converter
        → LeRobot v2 (Parquet + MP4)
```

Path A is what we use now. Path B is available via Unitree's
[unitree_IL_lerobot](https://github.com/unitreerobotics/unitree_IL_lerobot) repo
and would skip the replay step entirely.

---

## Launch Parameters

### Convenience Script

```bash
# Basic usage
bash run_xr_teleop.sh controller record motion

# With task metadata
bash run_xr_teleop.sh controller record motion \
    --task-name "pick_apple" \
    --task-goal "pick up the apple and place on plate" \
    --task-desc "tabletop manipulation" \
    --task-steps "step1: reach; step2: grasp; step3: place"

# Custom frame rate
bash run_xr_teleop.sh controller record motion --frequency 15

# Custom output directory
bash run_xr_teleop.sh controller record motion --task-dir /data/recordings

# Headless mode (no display on Host)
bash run_xr_teleop.sh controller record motion --headless

# Specify DDS network interface (multi-NIC systems)
bash run_xr_teleop.sh controller record motion --network-interface eth0
```

The script hardcodes `--arm=G1_29 --ee=dex3` and sets `--task-dir` to
`xr_recordings/` in the repo root. All other flags pass through directly.

### All Parameters

| Parameter | Default | Description | Change for real robot? |
|---|---|---|---|
| `--frequency` | `30.0` | Main loop Hz (IK + recording + display) | Maybe — 30 is fine for most cases |
| `--input-mode` | `hand` | `hand` or `controller` | **Yes** — choose per preference |
| `--display-mode` | `immersive` | `immersive`, `ego`, or `pass-through` | Rarely |
| `--arm` | `G1_29` | Robot model | No (hardcoded in script) |
| `--ee` | `None` | End effector (`dex3` for us) | No (hardcoded in script) |
| `--img-server-ip` | `192.168.123.164` | teleimager IP (PC2) | No (default is PC2) |
| `--network-interface` | `None` (auto) | DDS network interface name | Maybe — specify `eth0` if multi-NIC |
| `--motion` | `False` | Enable locomotion (walk while teleoperating) | **Yes** — usually enable |
| `--headless` | `False` | No display (Rerun viz disabled) | If Host has no monitor |
| `--sim` | `False` | Isaac Sim mode | No (real robot) |
| `--record` | `False` | Enable data recording | **Yes** — for data collection |
| `--task-dir` | `./utils/data/` | Recording root directory | **Yes** — script uses `xr_recordings/` |
| `--task-name` | `pick cube` | Task subdirectory name | **Yes** — change per task |
| `--task-goal` | `pick up cube.` | Task goal (in data.json) | **Yes** — describe your task |
| `--task-desc` | `task description` | Task description | Optional |
| `--task-steps` | `step1: do this...` | Task steps | Optional |
| `--ipc` | `False` | IPC control mode (for agent integration) | No (use keyboard) |
| `--affinity` | `False` | CPU core pinning | No (unless performance tuning) |

### Frequency Explained

`--frequency` controls three things simultaneously:

| What | Rate | Notes |
|---|---|---|
| Main loop (IK solve + image grab) | `1/frequency` sleep per iteration | Upper-layer control rate |
| Recording frame rate | `frequency` | Written to `info.image.fps` in data.json |
| VR display refresh | `frequency` | Image push to PICO headset |

**The arm low-level control is always 250Hz** in `ArmController`'s dedicated thread,
independent of `--frequency`. The `--frequency` only controls the "upper layer".

Recommendations:
- **30 Hz** (default) — good for most VLA training, matches typical camera FPS
- **15 Hz** — if PC is underpowered or network bandwidth is limited
- **>30 Hz** — not recommended; cameras are usually 30 FPS, higher is wasteful

### Output Directory

`--task-dir` and `--task-name` combine to form the recording path:

```
{task-dir}/{task-name}/episode_NNNN/
```

| Launch method | Example path |
|---|---|
| `run_xr_teleop.sh record --task-name pick_apple` | `xr_recordings/pick_apple/episode_0001/` |
| Direct: `--task-dir ./data --task-name pick_apple` | `xr_teleoperate/teleop/data/pick_apple/episode_0001/` |
| Direct: default | `xr_teleoperate/teleop/utils/data/pick cube/episode_0001/` |

Episode numbers auto-increment. If `episode_0003` exists, next will be `episode_0004`.

> **Disk space warning**: Each frame saves 1-4 JPG images. At 30 Hz with binocular
> head + 2 wrist cameras, that's ~120 images/second. A 1-minute episode at 640×480
> uses roughly **200-400 MB**. Monitor disk space with `df -h` during recording.

---

## Deployment Checklist

Complete multi-device startup sequence:

```
□  1. Power on G1, stand up (hand controller: L1+A → L1+UP, Regular mode R1+X)
□  2. Verify Ethernet: ping 192.168.123.164
□  3. Ensure humanoid-pc + PICO on same WiFi; note WiFi IP (hostname -I)
□  4. [PC2] ssh unitree@192.168.123.164
□  5. [PC2] conda activate teleimager && teleimager-server
□  6. [Host] conda activate tv
□  7. [Host] bash run_xr_teleop.sh controller record motion \
              --task-name "pick_apple" --task-goal "pick up the apple"
□  8. [PICO] Open browser → https://192.168.123.164:60001 → accept cert (first time)
□  9. [PICO] Open browser → https://{WIFI_IP}:8012/?ws=wss://{WIFI_IP}:8012
             → accept cert → click "Virtual Reality"
□ 10. [Host] Press [r] to start tracking
□ 11. [Host] Press [s] to start recording, [s] again to stop and save
□ 12. [Host] Press [q] to quit (arms return home over ~5 seconds)
```

---

## SSL Certificates by Device

### PICO / Quest (simple)

```bash
openssl req -x509 -nodes -days 365 -newkey rsa:2048 \
    -keyout key.pem -out cert.pem -subj "/CN=xr-teleoperate"
mkdir -p ~/.config/xr_teleoperate/
cp cert.pem key.pem ~/.config/xr_teleoperate/
scp cert.pem key.pem unitree@192.168.123.164:~/.config/xr_teleoperate/
```

Browser shows certificate warning → tap "Advanced" → "Proceed".

### Apple Vision Pro (CA chain)

Safari won't accept "click to proceed". Must create a full CA chain:

```bash
# Root CA
openssl genrsa -out rootCA.key 2048
openssl req -x509 -new -nodes -key rootCA.key -sha256 -days 365 \
    -out rootCA.pem -subj "/CN=xr-teleoperate"

# Server key + CSR
openssl genrsa -out key.pem 2048
openssl req -new -key key.pem -out server.csr -subj "/CN=localhost"

# server_ext.cnf (set IP.2 to your Host WiFi IP)
cat > server_ext.cnf << 'EOF'
subjectAltName = @alt_names
[alt_names]
DNS.1 = localhost
IP.1 = 192.168.123.164
IP.2 = 192.168.1.100
EOF

# Sign
openssl x509 -req -in server.csr -CA rootCA.pem -CAkey rootCA.key \
    -CAcreateserial -out cert.pem -days 365 -sha256 -extfile server_ext.cnf

# AirDrop rootCA.pem to Vision Pro → Settings → VPN & Device Mgmt → Install
```

---

## Safety Features

| Feature | How It Works |
|---|---|
| **Velocity limiting** | `clip_arm_q_target()` caps joint velocity at 250Hz control rate |
| **Startup ramp** | `speed_gradual_max()` — 5-second acceleration from 20 to 30 rad/s limit |
| **Exit go-home** | `ctrl_dual_arm_go_home()` — arms smoothly return to rest over ~5 seconds |
| **IK constraints** | CasADi solver respects joint limits; won't command beyond range |
| **Gravity compensation** | IK computes feedforward torques (integrated, no separate module) |
| **Smoothing** | `WeightedMovingFilter` on joint targets + IK regularization term |
| **E-stop** | Both controller thumbsticks pressed → `Damp()` (soft emergency stop) |
| **Controller exit** | Right A button → exit teleoperation |
| **Tiered PD gains** | Shoulder/elbow: kp=300, Wrist: kp=40 — prevents wrist oscillation |
