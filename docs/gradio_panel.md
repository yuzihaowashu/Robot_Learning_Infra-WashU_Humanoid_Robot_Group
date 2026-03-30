# Gradio Control Panel

Web-based control panel for `xr_teleoperate`, providing task configuration,
live teleop control, real-time status monitoring, and episode history from a
browser on the PC.

File: `gradio_panel.py` (project root)

---

## Quick Start

```bash
conda activate tv
python gradio_panel.py
```

The panel starts on **http://0.0.0.0:7860**. Open it in a browser on the same
PC or any device on the local network.

---

## Architecture

```
┌──────────────┐         ZMQ IPC          ┌──────────────────────┐
│ Gradio Panel │ ◄──── CMD / Heartbeat ──► │ teleop_hand_and_arm  │
│  (browser)   │                           │   (subprocess)       │
└──────────────┘                           └──────────────────────┘
       │                                            │
       │  subprocess.Popen                          │  Robot, VR, Camera
       └────────────────────────────────────────────┘
```

The panel launches `teleop_hand_and_arm.py` as a subprocess with `--ipc`
enabled. All commands (start tracking, toggle recording, stop, toggle
locomotion) are sent via ZMQ REQ/REP (`ipc://@xr_teleoperate_data.ipc`).
Status is received via ZMQ PUB/SUB heartbeat (`ipc://@xr_teleoperate_hb.ipc`).

---

## UI Layout

### Status Bar (auto-refreshes every 0.8 s)

| Indicator   | Meaning                                       |
|-------------|-----------------------------------------------|
| Process     | RUNNING (green) / STOPPED (red)               |
| IPC         | CONNECTED (green) / OFFLINE (gray)            |
| Tracking    | TRACKING (green) / WAITING (orange)           |
| Recording   | ● REC (red) / READY (green) / IDLE (gray)     |
| Locomotion  | WALK ON (green) / WALK OFF (gray)             |

### Left Column — Task Configuration

| Field            | Description                                    | Default          |
|------------------|------------------------------------------------|------------------|
| Task Name        | Directory name under `xr_recordings/`          | `pick_apple`     |
| Task Goal        | Saved to `data.json → text.goal`               | —                |
| Task Description | Saved to `data.json → text.description`        | —                |
| Task Steps       | Saved to `data.json → text.steps`              | —                |
| Input Mode       | `controller` or `hand`                         | `controller`     |
| Enable Locomotion (--motion) | Passes `--motion` to the teleop script | checked      |

Buttons:
- **Launch Teleop** — starts `teleop_hand_and_arm.py` with the above settings.
- **Stop Teleop** — sends `CMD_STOP` via IPC, then kills the process group.

### Right Column — Live Control

| Button                  | IPC Command         | Keyboard Equivalent |
|-------------------------|---------------------|---------------------|
| Start Tracking (r)      | `CMD_START`         | `r`                 |
| Toggle Recording (s)    | `CMD_RECORD_TOGGLE` | `s`                 |
| Toggle Locomotion (m)   | `CMD_LOCO_TOGGLE`   | `m`                 |
| EMERGENCY STOP (q)      | `CMD_STOP`          | `q`                 |

### Episode History

Displays a table of all recorded episodes for the current task, reading
`xr_recordings/{task_name}/episode_*/data.json`. Columns: Episode ID, Date,
Frame count, Goal. Auto-refreshes when Task Name changes; also has a manual
**Refresh Episodes** button.

### Network (accordion)

- **Check PC2 Connectivity** — pings `192.168.123.164` (robot onboard PC2).

---

## Locomotion Toggle

Locomotion (joystick → `Move()`) is **disabled by default** for safety.
The robot will stay in place until locomotion is explicitly enabled.

Ways to enable:
1. Press **Toggle Locomotion (m)** in the Gradio panel.
2. Press `m` on the keyboard (if using `sshkeyboard` mode).
3. Send `CMD_LOCO_TOGGLE` via IPC.

The VR HUD shows "WALK ON" (green) or "WALK OFF" (gray) in the top-right
corner. The Gradio status bar also reflects the current state.

---

## PC Monitor (VR Mirror)

When the teleop process runs **without** `--headless`, an OpenCV window
titled **"VR Mirror"** appears on the PC, displaying the same camera feed
and HUD overlay that the VR operator sees. This allows observers to watch
the teleoperation session on the PC monitor.

---

## IPC Protocol Summary

### Commands (REQ/REP)

| Command             | Mapped Key | Action                          |
|---------------------|------------|---------------------------------|
| `CMD_START`         | `r`        | Begin tracking                  |
| `CMD_STOP`          | `q`        | Stop and exit                   |
| `CMD_RECORD_TOGGLE` | `s`        | Toggle recording on/off         |
| `CMD_LOCO_TOGGLE`   | `m`        | Toggle locomotion on/off        |

### Heartbeat (PUB/SUB, 10 Hz)

```json
{
  "START": true,
  "STOP": false,
  "READY": true,
  "RECORD_RUNNING": false,
  "LOCO_ENABLED": false
}
```

---

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| "IPC offline" in panel | Teleop process not running or crashed | Check terminal output; re-launch |
| Status stays WAITING | `CMD_START` not sent, or VR not connected | Press "Start Tracking" after VR connects |
| Locomotion not working | `LOCO_ENABLED` is OFF (default) | Press "Toggle Locomotion (m)" |
| No VR Mirror window | `--headless` flag is set | Remove `--headless` from launch args |
| PC2 unreachable | Network cable disconnected or wrong subnet | Check cable; verify IP `192.168.123.164` |
