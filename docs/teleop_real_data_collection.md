# Teleop Real Data Collection Notes

This note records the latest teleoperation changes and the recommended workflow for moving from robot-side testing to real data collection and replay/checking.

## What We Changed So Far

### Documentation

- Converted `docs/teleoperation_commands.txt` into `docs/teleoperation_commands.md`.
- Added English and Chinese descriptions for the teleoperation workflow.
- Added `docs/psi0_environment_setup.md` to clarify when to use `uv` and when to use `conda`.

### Gradio Panel

- Removed the manual `Inactive Arm Pose` control from the UI.
- In `left-only` or `right-only`, Gradio now automatically uses `inactive_arm_pose=relaxed`.
- In `bimanual`, Gradio keeps `inactive_arm_pose=default`.
- The panel now documents that single-arm modes keep the inactive arm relaxed automatically.

### Single-Arm Teleoperation

- Added a relaxed inactive-arm joint target in `xr_teleoperate/teleop/teleop_hand_and_arm.py`.
- Added normalized arm-mode handling for `bimanual`, `left-only`, and `right-only`.
- In single-arm mode, the inactive arm is no longer driven by the idle VR controller target.
- The inactive arm wrist target is replaced with a forward-kinematics target based on the relaxed inactive-arm joints, so IK can solve the active arm while holding the inactive arm.
- For controller input:
  - `left-only`: right hand/arm actions are zeroed in the recording.
  - `right-only`: left hand/arm actions are zeroed in the recording.

### Deploy And Between-Episode Parking

- `G1_29_ArmController` now accepts `safe_deploy_q`.
- `safe_arm_deploy()` now uses that configured deploy target instead of always using the spread pose.
- In single-arm modes, the active arm deploys to the safe spread pose while the inactive arm goes directly to relaxed.
- `ctrl_dual_arm_go_home()` now accepts `park_q`.
- After saving an episode, the robot parks with the same mixed target. In single-arm modes, the active arm moves through an outward clearance waypoint and then slowly enters the original q=0 forward/default start pose while the inactive arm remains relaxed. This avoids sweeping directly from a relaxed/down pose into q=0 and keeps the next recording from starting with a large transition.

### Wrist Resistance Sound

- We tested several ideas for the wrist/hand motor resistance sound:
  - rotation low-pass filtering,
  - lower wrist PD gains,
  - lower IK rotation weight for single-arm mode.
- Because the sound also appears in `bimanual` mode, we decided to skip this for now.
- The active-arm control path was returned to the original bimanual-style defaults:
  - `--vr-rot-filter-alpha=1.0`
  - `--ik-rotation-weight=1.0`
  - wrist PD defaults remain `kp=60.0`, `kd=2.0`
- The remaining sound is likely part of the existing IK/PD control behavior, not specifically caused by the single-arm relaxed-arm changes.

## Real Data Collection Workflow

1. Launch the Gradio panel or the XR teleop script with `--record`.
2. Choose the task name, task goal, description, and steps carefully before recording.
3. Select `left-only`, `right-only`, or `bimanual`.
4. Press VR Left X to start/resume tracking and start a new episode.
5. Perform the task.
6. For auto-split collection, press VR Left Y at the transition to toggle `forward` / `backward`.
7. Press VR Right A to stop and save the current episode.
8. Wait until the UI/log says the episode is saved.
9. Press VR Left X again for the next episode.

Saved raw episodes are under:

```bash
xr_recordings/<task_name>/episode_xxxx/data.json
```

## Quick Recording Check

After collecting data, first check that each episode has frames and valid referenced images:

```bash
python utils/check_xr_episode.py xr_recordings/<task_name>
```

Or check one episode:

```bash
python utils/check_xr_episode.py xr_recordings/<task_name>/episode_0000
```

The check should report:

- `frames` greater than zero.
- `OK`.
- Expected `arm_mode` and `inactive_pose` metadata.
- Non-empty color streams when cameras are enabled.

If `frames=0`, the episode was created/saved but no recording frames were written. Usually this means recording was not actually running long enough between Left X and Right A, or tracking/camera data was paused.

## Replay / Visualization Path

The raw XR `data.json` format is the first thing to verify. It is not the same as the LeRobot dataset format consumed by `Psi0/scripts/viz/viz_episode_real.py`.

Recommended order:

1. Validate raw episodes with `utils/check_xr_episode.py`.
2. Confirm images under `colors/` match the `data.json` entries.
3. Convert raw episodes to LeRobot only after the raw data looks correct.
4. Use `Psi0/scripts/viz/viz_episode_real.py` on the converted LeRobot dataset.

Example visualization command after conversion:

```bash
cd Psi0
python scripts/viz/viz_episode_real.py --data-dir <converted_lerobot_dataset> --episode-idx 0
```

Then open the printed Viser URL in a browser and check:

- arm state playback follows the real demonstration,
- action playback is close to the commanded motion,
- left-only/right-only inactive-arm action is zero as expected,
- camera stream and robot motion are time-aligned.

## Current Caution

The existing test episodes in `xr_recordings` may include empty or interrupted recordings from debugging. Before collecting a real dataset, use a new task name or move old test episodes aside so the dataset is clean.
