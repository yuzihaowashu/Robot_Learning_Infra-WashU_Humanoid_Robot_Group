# WashU G1 Teleoperation and VLA Inference

This branch is a trimmed runtime workspace for two workflows on the Unitree G1:

- PICO 4 Ultra XR teleoperation for demonstration collection.
- NVIDIA GR00T N1.6 VLA inference on the robot.
- Psi0 VLA inference experiments as an optional backend.

It intentionally removes older training, dashboard, drag-and-teach, and alternate teleoperation stacks so the repository is easier to deploy on the robot workstation.

## Branch Layout

The GitHub default branch is `teleop_vla_infer`.

The previous development branch is preserved as `initial_dev`. The old `main` branch is also kept on GitHub for safety, but this branch is the recommended entry point.

## Clone

Clone with submodules:

```bash
git clone --recurse-submodules https://github.com/yuzihaowashu/Robot_Learning_Infra-WashU_Humanoid_Robot_Group.git
cd Robot_Learning_Infra-WashU_Humanoid_Robot_Group
```

If you already cloned without submodules:

```bash
git submodule update --init --recursive
```

Submodules are pinned to WashU forks:

- `Isaac-GR00T` -> `https://github.com/yuzihaowashu/Isaac-GR00T.git`
- `Psi0` -> `https://github.com/yuzihaowashu/Psi0.git`
- `xr_teleoperate` -> `https://github.com/yuzihaowashu/xr_teleoperate.git`
- `xr_teleoperate/teleop/televuer` -> `https://github.com/yuzihaowashu/televuer.git`

## Repository Contents

```text
.
├── gradio_panel.py                 # Teleop control panel
├── run_xr_session.sh               # Recommended XR teleop launcher
├── run_xr_teleop.sh                # Direct XR teleop launcher
├── run_vla.sh                      # GR00T VLA inference launcher
├── run_psi0.sh                     # Psi0 inference helper
├── run_reset_arms.sh               # Stand/default arm reset helper
├── docs/
│   ├── teleoperation_commands.txt  # Bilingual quick commands
│   ├── teleoperation.md            # Teleoperation details
│   ├── gradio_panel.md             # Gradio panel notes
│   ├── xr_teleoperate.md           # XR teleop details
│   └── vla_inference.md            # VLA inference details
├── utils/
│   ├── arm_idle_holder.py          # Safe arm-spread holder daemon
│   ├── g1-arm-holder.service       # systemd unit for arm holder
│   ├── robot_camera_server.py      # Camera helper deployed to robot PC
│   └── vla_client.py               # GR00T policy client + G1 control bridge
├── Isaac-GR00T/                    # Submodule: GR00T inference server
├── Psi0/                           # Submodule: optional Psi0 backend
└── xr_teleoperate/                 # Submodule: PICO/WebXR teleop stack
```

## Robot Stand-Up Commands

Before teleoperation or VLA control, make the G1 stand up with the Unitree remote controller:

```text
1. L2 + Y, then L2 + B
   Press and hold L2 until the remote vibrates, press the button once, then release L2 quickly.

2. L2 + UP
   Joints move to the home / stand-ready position slowly.

3. R1 + X
   Activate the balance controller; the robot stands up.
```

If `g1-arm-holder.service` is active, `L2 + UP` may ramp the arms to the outward safety pose instead of the factory/default arm pose. This is intentional for Dex3 hand collision avoidance.

## XR Teleoperation

Recommended launcher:

```bash
bash run_xr_session.sh
```

The script checks the PC WiFi IP, certificate paths, PC2 camera reachability, and existing XR services. It prints the URL to open in the PICO browser:

```text
https://<PC_WiFi_IP>:8012/?ws=wss://<PC_WiFi_IP>:8012
```

Typical workflow:

1. Start PC2 camera stream if needed:

   ```bash
   ssh unitree@192.168.123.164
   conda activate teleimager && teleimager-server
   ```

2. Run `bash run_xr_session.sh` on the host PC.
3. Click `(1) Launch Teleop` in Gradio.
4. Open the PICO URL and click `Enter VR`.
5. Use PICO controllers:
   - `Left X`: start/resume tracking and begin a new episode.
   - `Right A`: stop and save the current episode.
   - `Right B`: manual recording toggle backup.
   - Both joysticks pressed: emergency damping.
6. Finish with `(2) Stop Teleop + Relax Arms`.
7. Use `(3) Relax Arms to Default Pose` only as recovery if automatic relax did not finish.

See `docs/teleoperation_commands.txt` for the bilingual quick reference.

## VLA Inference

GR00T inference uses two terminals:

```bash
# Terminal 1: GPU policy server
bash run_vla.sh server

# Terminal 2: robot client, step-by-step by default
bash run_vla.sh client --task "shake the bottle"
```

`run_vla.sh` includes the robot stand-up commands at the top of the file. The script expects the GR00T environment at:

```text
Isaac-GR00T/.venv/bin/python
```

See `docs/vla_inference.md` for details.

## Psi0 Experiments

Psi0 is kept as an optional VLA backend for comparison with GR00T. The recommended real-robot path is the `rtc-bimanual` client: it uses Psi0's upstream WebSocket RTC timing, executes only the bimanual arm/hand portion of the 36D action, ignores torso/base policy outputs, and keeps step-by-step approval by default.

```bash
# Print Psi0 environment setup commands
bash run_psi0.sh setup

# Terminal 1: start the tested local Psi0 WebSocket RTC server.
# Model files live outside git under /home/humanoid-pc/psi0_runtime.
bash run_psi0.sh server-rtc /home/humanoid-pc/psi0_runtime/runs/psi0_baseline_release 0

# Terminal 2: run bimanual-only Psi0 RTC inference.
bash run_psi0.sh rtc-bimanual --task "put the bottle into the paper box" --execute --send-hands

# Optional: hold each approved Enter step longer.
bash run_psi0.sh rtc-bimanual --task "put the bottle into the paper box" --execute --send-hands --step-seconds 2.0
```

The bimanual RTC client in `utils/psi0_rtc_bimanual_client.py` treats the Psi0 release actions as deltas, accumulates approved RTC steps, clamps G1 arm targets, and uses XR teleop's waist behavior (`--waist-mode xr-upright` by default) to keep the robot balanced while ignoring Psi0 torso/base outputs. It refuses to run with XR teleop still active unless `--allow-competing-control` is explicitly passed. The older HTTP safety client remains available as `bash run_psi0.sh client`, and the upstream raw RTC client is gated behind `bash run_psi0.sh rtc --raw-robot-control` for debugging only.

On `q` or `Ctrl+C`, `rtc-bimanual` smoothly parks the arms at `--exit-pose default` by default. Use `--exit-pose spread` if Dex3 thigh clearance is preferred, or `--exit-pose hold` only for debugging.

If a client exits unexpectedly and leaves the arms away from the stand pose, use the standalone recovery helper:

```bash
bash run_reset_arms.sh

# Or move to the outward Dex3-safe spread pose.
bash run_reset_arms.sh --pose spread
```

## Safety Notes

- Gradio always launches teleop with Unitree balance / `arm_sdk` mode enabled.
- Walking commands are off by default and only enabled through the advanced manual control.
- Teleop stop first parks arms at a safe outward pose, then relaxes toward the default/down pose.
- The `arm_idle_holder.py` daemon can hold arms in the outward safety pose between sessions.
- Use `EMERGENCY STOP` in Gradio if the robot behaves unexpectedly.

## Useful Commands

Check submodules:

```bash
git submodule status --recursive
```

Update submodules after pulling:

```bash
git submodule update --init --recursive
```

Log out of GitHub on a shared robot PC:

```bash
gh auth logout
gh auth status
```
