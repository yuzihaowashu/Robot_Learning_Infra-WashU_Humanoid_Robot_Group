# DP and Psi0 Deployment Commands

This document records the recommended two-terminal deployment commands for
the G1 bottle task.

Use this as the operator checklist. Terminal 1 runs the model server.
Terminal 2 runs the robot-facing client and UI.

## Robot Camera Stream

After the robot reboots, the camera server on the robot may not be running.
Start/check it from this workstation before launching the DP or Psi0 UI.
See also [Before Terminal 1: Robot camera stream](#before-terminal-1-robot-camera-stream)
under Psi0 RTC.

```bash
cd /home/humanoid-pc/yu.zihao/Robot_Learning_Infra-WashU_Humanoid_Robot_Group
bash run_dp.sh camera
```

Expected stream endpoint:

```text
tcp://192.168.123.164:5555
```

If the UI shows blank/stale video, rerun this command in an interactive
terminal and then refresh the browser UI.

## Diffusion Policy

Diffusion Policy uses an HTTP server on port `8020` and a G1 client UI on
port `8030`.

### Terminal 1: DP Policy Server

```bash
cd /home/humanoid-pc/yu.zihao/Robot_Learning_Infra-WashU_Humanoid_Robot_Group
bash run_dp.sh server --host 0.0.0.0 --port 8020 --device cuda:0
```

Wait until the server finishes loading the checkpoint and is ready to serve
`/act`.

### Terminal 2: DP Robot UI Client

```bash
cd /home/humanoid-pc/yu.zihao/Robot_Learning_Infra-WashU_Humanoid_Robot_Group
bash run_dp.sh client \
  --server-url http://localhost:8020/act \
  --ui \
  --ui-port 8030 \
  --execute \
  --task "take bottle out from paper box" \
  --waist-mode upright \
  --hand-mode open \
  --action-exec-steps 1
```

Open:

```text
http://127.0.0.1:8030
```

### DP Flags To Care About

| Flag | Recommended | Why it matters |
|---|---:|---|
| `--execute` | on for real robot | Without it, the client is dry-run only. |
| `--ui` | on | Enables camera view, Preparation, Run Next Step, Run 20, Reset + Restart Model, Stop. |
| `--server-url` | `http://localhost:8020/act` | Must match Terminal 1 server port. |
| `--waist-mode` | `upright` | Commands waist to `[0, 0, 0]` instead of holding a tilted pose. |
| `--hand-mode` | `open` | Keeps fingers open instead of trusting policy finger output. |
| `--action-exec-steps` | `1` first | Executes only the first chunk step per UI click; increase only after checking behavior. |
| `--ui-port` | `8030` | Browser UI port. |
| `--no-send-hands` | optional safety | Disables Dex3 commands completely if finger motion is risky. |
| `--mock --once` | debug only | Tests server I/O without connecting to robot DDS. |

### DP Notes

- The DP checkpoint is not text-conditioned. The task string is displayed for
  operator context, but the model was trained for the fixed bottle task.
- The UI `Reset + Restart Model` button resets robot pose and clears the DP
  server observation history so the next inference starts fresh.
- Use `Joint Audit` in the UI when checking model state/action joint mapping.

## Psi0 RTC

Psi0 RTC uses an upstream WebSocket server on port `8014` and a bimanual
robot UI client on port `8040`.

The local 10k checkpoint path is:

```text
/home/humanoid-pc/psi0_runtime/runs/g1-bottle-left-arm/checkpoints/ckpt_10000/model.safetensors
```

Only `model.safetensors` was downloaded. Optimizer, scheduler, and random
state files were intentionally not downloaded.

### Before Terminal 1: Robot camera stream

After a robot reboot, the head camera is usually off. Start it **before** the
Psi0 server and client. The Psi0 UI reads from ZMQ port `5555` on the robot
(not the XR `teleimager-server` stream on `55555`).

```bash
cd /home/humanoid-pc/yu.zihao/Robot_Learning_Infra-WashU_Humanoid_Robot_Group
bash run_dp.sh camera
```

Expected stream endpoint:

```text
tcp://192.168.123.164:5555
```

If the UI at `http://127.0.0.1:8040` shows blank video, rerun this command
in an interactive terminal, then refresh the browser.

### Terminal 1: Psi0 10k RTC Server

Use memory guards. This prevents Psi0 model loading from consuming all system
RAM/swap and crashing the desktop session.

```bash
cd /home/humanoid-pc/yu.zihao/Robot_Learning_Infra-WashU_Humanoid_Robot_Group
PSI0_SERVER_MEMORY_GB=22 PSI0_MIN_AVAILABLE_RAM_GB=20 \
  bash run_psi0.sh server-rtc \
    /home/humanoid-pc/psi0_runtime/runs/g1-bottle-left-arm \
    10000 \
    8014
```

Wait until the server prints the WebSocket endpoint:

```text
WebSocket endpoint: ws://0.0.0.0:8014/ws
```

### Terminal 2: Psi0 Bimanual Robot UI Client

```bash
cd /home/humanoid-pc/yu.zihao/Robot_Learning_Infra-WashU_Humanoid_Robot_Group
bash run_psi0.sh rtc-bimanual \
  --host localhost \
  --port 8014 \
  --ui \
  --ui-port 8040 \
  --execute \
  --send-hands \
  --arm-side left \
  --approval-steps 15 \
  --task "put the bottle into the paper box" \
  --waist-mode xr-upright \
  --action-mode delta \
  --exit-pose default
```

Open:

```text
http://127.0.0.1:8040
```

The Psi0 UI has a task preset dropdown loaded from `tasks/task_list.json`.
Changing the dropdown updates the language prompt sent to the RTC server.

Before running policy actions, use the Start block's
`Preparation: Move To Ready Pose` button. It pauses pending RTC execution,
clears the approval budget, moves the arms to the ready pose, and opens both
Dex3 hands even when normal policy hand execution is disabled.

For normal RTC deployment, use `Start RTC Streaming` after Preparation. The
client will execute each incoming RTC tick until `Pause RTC Streaming` or
`Relax Arms` is pressed. Keep `Run Next RTC Chunk` only as a cautious debug
mode.

### Psi0 Flags To Care About

| Flag | Recommended | Why it matters |
|---|---:|---|
| `PSI0_SERVER_MEMORY_GB` | `22` first | Caps server RAM usage. Increase only if loading is killed by the cap. |
| `PSI0_MIN_AVAILABLE_RAM_GB` | `20` first | Refuses to start if there is not enough free RAM. |
| `server-rtc RUN_DIR CKPT PORT` | `g1-bottle-left-arm 10000 8014` | Selects the 10k checkpoint and WebSocket port. |
| `--execute` | on for real robot | Without it, the bimanual client is dry-run only. |
| `--ui` | on | Enables camera view, Preparation, Start/Pause RTC Streaming, Run Next RTC Chunk, Relax Arms, Resume Approval Mode. |
| `--send-hands` | on when testing grasping | Enables policy hand actions during approved RTC steps. Preparation and Relax Arms still force hands open. |
| `--arm-side` | `left` | Executes only the left arm/hand from the policy; keeps the right arm at its current held target and the right hand open. Use `bimanual` only when testing both arms. |
| `--approval-steps` | `15` | One UI Run Next click approves 15 RTC ticks, about 0.5s at the server's 30Hz RTC loop. |
| `--waist-mode` | `xr-upright` | Matches the XR-style upright waist PD behavior. |
| `--action-mode` | `delta` | The current Psi0 release config uses delta arm actions. |
| `--exit-pose` | `default` | Relax behavior on `q`/Ctrl+C/Relax Arms. `default` is a bent relaxed pose; use `spread` for clearance if needed. |
| `--allow-competing-control` | only when needed | Bypasses the XR teleop process safety check. |

### Psi0 10k Checkpoint Shape

The 10k checkpoint action head expects:

```text
state / obs dim:      63
action dim:           31
action chunk size:    30
```

The local run metadata must match those shapes:

```text
/home/humanoid-pc/psi0_runtime/runs/g1-bottle-left-arm/run_config.json
/home/humanoid-pc/psi0_runtime/runs/g1-bottle-left-arm/argv.txt
```

If the run config incorrectly uses the baseline `36D / 16-step` settings, the
server fails with action head size mismatch errors such as:

```text
obs_proj._obs_proc.1.weight: checkpoint [1536, 63] vs model [1536, 36]
action_proj_in.dec_pos: checkpoint [30, 1536] vs model [16, 1536]
action_proj_out.linear.weight: checkpoint [31, 1536] vs model [36, 1536]
```

### Safe Dry-Run Checks

DP:

```bash
bash run_dp.sh server --device cuda:0 --load-only
bash run_dp.sh client --mock --once
```

Psi0:

```bash
PSI0_MIN_AVAILABLE_RAM_GB=999 \
  bash run_psi0.sh server-rtc \
    /home/humanoid-pc/psi0_runtime/runs/g1-bottle-left-arm \
    10000 \
    8014
```

The Psi0 command above should fail before model loading. It validates the RAM
preflight guard.
