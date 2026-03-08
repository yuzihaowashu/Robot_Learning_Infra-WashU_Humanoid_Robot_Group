# Drag-and-Teach — Record & Replay Arm Trajectories

Physically move the robot's arms and Dex3 hand fingers while gravity
compensation keeps the arms weightless, then replay the recorded trajectory.

## Usage

```bash
# Record a new trajectory
bash run_teach.sh record

# Replay a saved trajectory
bash run_teach.sh replay trajectories/traj_20260306_174754.json

# Replay at half speed, loop 3 times
bash run_teach.sh replay trajectories/traj_xxx.json --speed 0.5 --loop 3

# List all saved trajectories
bash run_teach.sh list
```

## How Recording Works

1. **Balance controller must be active** — The robot must be standing with
   the `ai` locomotion mode running (use hand controller: L2+B → L2+UP → R1+X).
   The legs and body stay balanced and push-resistant throughout.

2. **Arms go compliant** — During recording, arm PD gains are set to
   `kp=0, kd=1` (near-zero stiffness, light damping). You can physically
   move the arms freely.

3. **Gravity compensation** — Pinocchio computes the generalized gravity
   vector `G(q)` from the URDF at each control step. This torque is sent as
   `tau_ff` (feedforward) so the arms feel weightless — they stay wherever
   you place them instead of drooping.

4. **Hand recording** — Dex3 finger positions are recorded simultaneously.
   Move the fingers manually; they record at the same rate as the arms.

5. **Press Enter to start/stop** — Recording begins when you press Enter
   and stops on the next Enter. Ctrl+C quits without saving.

6. **Trajectory saved** — Saved as JSON in `trajectories/` with timestamp.

## How Replay Works

1. **Smooth ramp-up** — Arm control engages gradually over 2 seconds
   (kp/kd ramp from 0 to full) to avoid sudden jerks.

2. **Position tracking** — Arms track the recorded trajectory at
   `kp=80, kd=2` with gravity compensation (`tau_ff`).

3. **Hands replay** — Finger positions are replayed via `rt/dex3/*/cmd`
   with RIS-encoded mode fields.

4. **Smooth ramp-down** — On completion (or Ctrl+C), arms smoothly
   return to home position then kp/kd ramps down to zero.

5. **Fingers open** — Hands actively drive to open position (`q=0`)
   before releasing control (Dex3 has no passive return springs).

## Trajectory File Format

```json
{
  "metadata": {
    "timestamp": "2026-03-06T17:47:54",
    "duration_s": 12.5,
    "num_frames": 375,
    "control_dt": 0.033
  },
  "frames": [
    {
      "t": 0.0,
      "waist": [0.0, 0.0, 0.0],
      "left_arm": [0.1, -0.2, 0.3, 0.8, 0.0, 0.0, 0.0],
      "right_arm": [-0.1, 0.2, -0.3, 0.8, 0.0, 0.0, 0.0],
      "left_hand": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      "right_hand": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    },
    ...
  ]
}
```

## Key Parameters

| Parameter | Record | Replay | Description |
|-----------|--------|--------|-------------|
| `kp` (arm) | 0.0 | 80.0 | Position stiffness |
| `kd` (arm) | 1.0 | 2.0 | Velocity damping |
| `tau_ff` | G(q) | G(q) | Gravity compensation torque |
| `kp` (hand) | — | 1.5 | Hand position stiffness |
| `kd` (hand) | — | 0.2 | Hand velocity damping |
| Control rate | 30 Hz | 30 Hz | Command publish frequency |

## Control Architecture

```
                    Record Mode                     Replay Mode
                    ───────────                     ───────────
Arms (rt/arm_sdk):  kp=0, kd=1, tau=G(q)          kp=80, kd=2, q=target, tau=G(q)
                    ↓ compliant, gravity-free       ↓ stiff tracking + gravity comp
                    User moves arms freely          Arms follow recorded trajectory

Hands (rt/dex3):    Read motor_state.q              Write motor_cmd.q = recorded
                    ↓ record finger positions       ↓ replay finger positions

Legs/Body:          Balance controller (ai mode)    Balance controller (ai mode)
                    ↓ fully autonomous               ↓ fully autonomous
```

## Safety Notes

- Arms only — legs and body are always controlled by Unitree's balance controller
- Gravity compensation prevents arm droop during both recording and replay
- Smooth ramp-up/down on every start and stop
- Fingers are actively opened on exit (no passive springs in Dex3)

## Implementation Files

- `utils/teach.py` — Recording logic
- `utils/replay.py` — Replay logic with gravity compensation
- `run_teach.sh` — Entry point script
