# VLA Inference — GR00T N1.6 on Unitree G1

Run Vision-Language-Action inference using NVIDIA's GR00T N1.6 model to
control the G1 robot's arms and hands from camera images and language instructions.

## Architecture

```
┌──────────────┐         ┌──────────────┐         ┌──────────────┐
│  Robot Camera │──ZMQ──→│  VLA Client   │──ZMQ──→│  GR00T Server │
│  (port 5555) │         │ (vla_client)  │←─ZMQ──│  (GPU, 5556)  │
└──────────────┘         │              │         │              │
                         │  DDS ↕       │         │  Model:      │
┌──────────────┐         │ rt/lowstate   │         │  N1.6-G1     │
│  G1 Robot    │◄──DDS──│ rt/arm_sdk    │         │  Eagle+DiT   │
│  (arms+hands)│──DDS──→│ rt/dex3/*/    │         └──────────────┘
└──────────────┘         └──────────────┘
```

**Two-process design**: The GPU inference server and robot client run in
separate terminals. They communicate via ZMQ (port 5556).

## Usage

### Quick Start

```bash
# Terminal 1: Start GPU inference server (~30s to load model)
bash run_vla.sh server

# Terminal 2: Start robot client (step-by-step by default)
bash run_vla.sh client --task "pick up the apple and place on plate"
```

### All Commands

```bash
bash run_vla.sh server                          # GPU inference server
bash run_vla.sh client --task "pick up apple"   # Step-by-step (safe)
bash run_vla.sh client --continuous             # Auto-execute (careful!)
bash run_vla.sh client --dry-run                # Inference only, no robot
bash run_vla.sh test                            # Verify GPU + imports
```

### Client Options

| Flag | Default | Description |
|------|---------|-------------|
| `--task TEXT` | `"pick up the apple and place on plate"` | Language instruction |
| `--continuous` | off | Skip step-by-step approval |
| `--dry-run` | off | Run inference but don't send commands |
| `--action-horizon N` | 8 | Steps per inference chunk |
| `--policy-host` | localhost | Server address |
| `--policy-port` | 5556 | Server port |

## Step-by-Step Mode (Default)

Each inference step shows the proposed action before execution:

```
──────────────────────────────────────────────────────────
[Step 0] Inference: 0.234s, 8 action sub-steps
  ┌─ Left  Arm ──────────────────────────────
  │ Now:    [ +17.2°,  -5.3°, +11.8°, +45.2°,  -3.1°,  +8.4°,  +0.2°]
  │ Target: [ +18.0°,  -4.9°, +12.3°, +46.1°,  -2.8°,  +8.9°,  +0.3°]
  │ Delta:  [  +0.8°,  +0.4°,  +0.5°,  +0.9°,  +0.3°,  +0.5°,  +0.1°]  (max 0.9°)
  ├─ Right Arm ...
  ├─ Waist ...
  ├─ Hands ...
  └──────────────────────────────────────────
  >> Execute? [Enter/s/c/q]:
```

Controls:
- **Enter** — Approve and execute this step
- **s** — Skip (hold current position)
- **c** — Switch to continuous mode
- **q** — Quit gracefully (returns to home pose)
- **Ctrl+C** — Emergency stop (still returns to home)

## Safety Features

### 1. Home Pose Restore
The initial arm position is saved at startup. On exit (quit or Ctrl+C),
the arms smoothly interpolate back to the home pose before releasing control.
Duration adapts to distance (2–6 seconds).

### 2. URDF Joint Limit Clamping
Every action is hard-clamped to the URDF-defined joint limits before being
sent to the robot. No joint can exceed its physical range.

### 3. Max Delta Per Step
Each joint is limited to `MAX_DELTA_PER_STEP = 0.15 rad` (~8.6°) per control
step at 30 Hz. Even if the model outputs a large jump, the robot moves smoothly.

### 4. Gravity Compensation
Pinocchio computes `G(q)` from the URDF and applies it as feedforward torque,
preventing arms from drooping under their own weight.

### 5. Smooth Ramp-Up/Down
Arm control gains (kp/kd) ramp up over 2 seconds at startup and ramp down
over 2 seconds at shutdown, avoiding sudden jerks.

## GR00T Model Details

### Input Modalities

| Modality | Key | Shape | Source |
|----------|-----|-------|--------|
| Video | `ego_view` | (1,1,H,W,3) uint8 | Robot head camera |
| State | `left_leg`, `right_leg`, `waist`, `left_arm`, `right_arm`, `left_hand`, `right_hand` | (1,1,D) float32 | DDS joint states |
| Language | `annotation.human.task_description` | string | `--task` argument |

### Output Actions

| Key | Dim | Representation | Description |
|-----|-----|----------------|-------------|
| `left_arm` | 7 | **Absolute** (converted from relative internally) | Left arm joint positions (rad) |
| `right_arm` | 7 | **Absolute** (converted from relative internally) | Right arm joint positions (rad) |
| `left_hand` | 7 | Absolute | Left hand motor positions |
| `right_hand` | 7 | Absolute | Right hand motor positions |
| `waist` | 3 | Absolute | Waist joint positions (rad) |
| `base_height_command` | 1 | Absolute | (not used on real robot) |
| `navigate_command` | 3 | Absolute | (not used on real robot) |

**Critical**: GR00T's `decode_action` pipeline internally converts relative
arm actions to absolute positions (`reference_state + delta`). The client
uses these values directly — they must NOT be accumulated as deltas.

### Action Horizon
The model outputs 30 action steps per inference. The client executes
`--action-horizon` (default 8) of them at 30 Hz before re-querying.

## Camera Auto-Start

The client automatically manages the robot's camera server:
1. Checks if a ZMQ stream exists at `tcp://192.168.123.164:5555`
2. If not, prompts for SSH password
3. Deploys `robot_camera_server.py` to the robot
4. Starts it in background
5. Verifies the stream is active

## Environment Setup

The GR00T inference server requires a separate Python 3.10 environment:

```bash
cd Isaac-GR00T
uv sync --python 3.10     # Creates .venv with all dependencies
```

Key dependencies:
- PyTorch 2.7+ with CUDA 12.6
- flash-attn 2.7.4 (pre-built wheel)
- GR00T model weights (auto-downloaded from HuggingFace)

## Implementation Files

- `utils/vla_client.py` — G1 ↔ GR00T bridge (DDS, ZMQ, safety)
- `run_vla.sh` — Orchestration script (server, client, camera, test)
- `Isaac-GR00T/` — NVIDIA GR00T repository (submodule)

## Psi0 RTC Bimanual Backend

Psi0 is available as a second VLA backend for zero-shot experiments. Use the
WebSocket RTC path for real-robot tests because it preserves Psi0's original
action timing while restricting execution to the G1 arms and Dex3 hands.

```bash
# Terminal 1: Psi0 WebSocket RTC server
bash run_psi0.sh server-rtc /home/humanoid-pc/psi0_runtime/runs/psi0_baseline_release 0

# Terminal 2: bimanual-only client, step-by-step by default
bash run_psi0.sh rtc-bimanual --task "put the bottle into the paper box" --execute --send-hands
```

Controls match the GR00T step-by-step client:
- **Enter** — approve the next chunk of RTC actions
- **s** — skip this proposed action
- **c** — switch to continuous execution
- **q** — quit

Important implementation details:
- Psi0 outputs 36D whole-body actions. The local client uses `action[0:14]`
  for hands, `action[14:28]` for arms, and ignores `action[28:36]`
  torso/base commands.
- The tested release checkpoint uses delta actions, so
  `utils/psi0_rtc_bimanual_client.py` accumulates arm deltas before sending
  targets to `rt/arm_sdk`.
- Default waist handling is `--waist-mode xr-upright`, matching the working
  XR teleop behavior: waist target is `[0, 0, 0]` with waist PD
  `kp=200, kd=5`.
- `--step-seconds` controls how long each approved Enter press consumes and
  executes subsequent RTC actions. Increase it, for example to `2.0`, if each
  approved step is too short.
- The client aborts if `teleop_hand_and_arm.py` is still running, because XR
  teleop would overwrite Psi0 arm commands. Stop teleop first, or pass
  `--allow-competing-control` only for debugging.
- `--test-arm-nudge` sends a tiny diagnostic arm command without Psi0. If this
  does not move, the issue is the robot control mode or `arm_sdk`, not model
  inference.
- On `q` or `Ctrl+C`, the client stops receiving RTC actions and smoothly
  returns the arms to `--exit-pose default`. Use `--exit-pose spread` for the
  outward Dex3-safe pose, or `--exit-pose hold` only when debugging.

### Stand/Default Arm Reset Helper

If a VLA or teleop process exits unexpectedly and leaves the arms away from the
stand pose, use the standalone reset helper. It does not require a Psi0 or
GR00T server.

```bash
# Return both arms to the stand/default q=0 pose
bash run_reset_arms.sh

# Move both arms to the outward Dex3-safe spread pose
bash run_reset_arms.sh --pose spread
```

The script refuses to run while teleop or VLA clients are still active, because
two processes publishing arm commands will fight each other. Stop the active
client first, or pass `--force` only for debugging.

## Pretrained Checkpoints

| Checkpoint | Task | Notes |
|-----------|------|-------|
| `nvidia/GR00T-N1.6-G1-PnPAppleToPlate` | Pick apple → place on plate | Default, simulation-trained |

Set custom checkpoint: `MODEL_PATH=/path/to/ckpt bash run_vla.sh server`
