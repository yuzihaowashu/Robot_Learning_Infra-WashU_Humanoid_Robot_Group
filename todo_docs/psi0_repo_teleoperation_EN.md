# Psi0 repository teleoperation analysis — comparison with xr_teleoperate (Dex3-1 three-finger hands)

> Date: 2026-03-31  
> Repository: https://github.com/physical-superintelligence-lab/Psi0  
> Cloned as submodule at `Psi0/`

---

## I. Overview of Psi0’s teleoperation stack

Psi0 **does not use xr_teleoperate**; it uses a separate stack based on **Apple Vision Pro + Vuer + OpenTeleVision**.

| Aspect | xr_teleoperate (ours) | Psi0 (`real/teleop/`) |
|--------|------------------------|------------------------|
| VR headset | PICO 4U (TeleVuer/WebXR) | Apple Vision Pro (Vuer/OpenTeleVision) |
| Middleware | TeleVuer + TeleVuerWrapper | Vuer + OpenTeleVision |
| Architecture | Single process `teleop_hand_and_arm.py` | Worker/Master multi-process split |
| Where retargeting runs | Inside hand control subprocess (`robot_hand_unitree.py`) | VR preprocessing (`vr.py`); 7-D result passed to hand via shared memory |
| Credits / upstream | Unitree official xr_teleoperate | avp_teleoperate, OpenTeleVision, vuer |

**Shared ground**: Same Unitree G1 + Dex3-1 hardware, same DDS topics (`rt/dex3/{left,right}/cmd`), same `HandCmd_` message type, and nearly identical `hand_retargeting.py` and `JointIndex` enums.

---

## II. Main difference: retargeting algorithm type

### xr_teleoperate — DexPilot mode

```yaml
# xr_teleoperate/assets/unitree_hand/unitree_dex3.yml
left:
  type: DexPilot
```

- **Input**: 25-point hand skeleton → six pairwise vector differences (via `target_link_human_indices_dexpilot: [[9,14,14,0,0,0],[4,4,9,4,9,14]]`)
- **Algorithm**: NLopt + DexPilot optimizer (Huber loss + `project_dist` / `escape_dist` projection terms)
- **Code location**: Inside `robot_hand_unitree.py` `control_process`

```python
# xr_teleoperate/teleop/robot_control/robot_hand_unitree.py:187-191
ref_left_value  = left_hand_data[self.hand_retargeting.left_indices[1,:]] - left_hand_data[self.hand_retargeting.left_indices[0,:]]
ref_right_value = right_hand_data[self.hand_retargeting.right_indices[1,:]] - right_hand_data[self.hand_retargeting.right_indices[0,:]]
left_q_target   = self.hand_retargeting.left_retargeting.retarget(ref_left_value)[self.hand_retargeting.right_dex_retargeting_to_hardware]
right_q_target  = self.hand_retargeting.right_retargeting.retarget(ref_right_value)[self.hand_retargeting.right_dex_retargeting_to_hardware]
```

### Psi0 — vector mode

```yaml
# Psi0/real/assets/unitree_hand/unitree_dex3.yml
left:
  type: vector
```

- **Input**: 25-point hand skeleton → only three fingertip positions `[4, 9, 14]` (thumb / index / middle tips)
- **Algorithm**: vector optimizer (simpler and more direct)
- **Extra step**: **Per-finger scale factors** thumb×1.15, index×1.05, middle×0.95
- **Code location**: `vr.py`, `VuerPreprocessor.process()`

```python
# Psi0/real/teleop/vr.py:134-163
unitree_tip_indices = [4, 9, 14]
ref_left_value = unitree_left_hand[unitree_tip_indices].copy()
ref_left_value[0] *= 1.15   # thumb
ref_left_value[1] *= 1.05   # index
ref_left_value[2] *= 0.95   # middle
left_q_target = hand_retargeting.left_retargeting.retarget(ref_left_value)[
    hand_retargeting.right_dex_retargeting_to_hardware
]
```

---

## III. Data-flow comparison

### xr_teleoperate

```
Pico VR 25-point hands → TeleVuerWrapper transform → shared memory (75 floats, 25×3)
→ Dex3 control subprocess reads → DexPilot retargeting → 7-D q_target → DDS HandCmd_
```

### Psi0

```
AVP 25-point hands → vr.py transform → extract 3 fingertips → vector retargeting → 7-D q_target
→ shared memory (14 floats, left 7 + right 7) → Dex3 control subprocess reads and publishes DDS HandCmd_
```

Psi0’s hand subprocess is simpler: it only reads shared memory and sends commands; it does **not** run retargeting:

```python
# Psi0/real/teleop/robot_control/robot_hand_unitree.py:228-229
left_q_target = hand_shm_array[0:7]
right_q_target = hand_shm_array[7:14]
```

---

## IV. Configuration comparison

| Parameter | xr_teleoperate | Psi0 |
|-----------|----------------|------|
| Retarget type | DexPilot | vector |
| Per-finger scaling | None | thumb×1.15, index×1.05, middle×0.95 |
| Low-pass alpha | 0.2 | 0.2 |
| PD gains kp / kd | 1.5 / 0.2 | 1.5 / 0.2 |
| Control rate | 100 Hz | 100 Hz |
| DDS topic | rt/dex3/{left,right}/cmd | rt/dex3/{left,right}/cmd |
| URDF | unitree_dex3_{left,right}.urdf | Same |

---

## V. Shared suspicious code: left hand uses `right` indexing

Both repos use the same pattern: the **left** retarget output is reindexed with `right_dex_retargeting_to_hardware`:

```python
# Same in both repos:
left_q_target = ...retarget(ref_left_value)[hand_retargeting.right_dex_retargeting_to_hardware]  # ← should this be left?
```

**Practical impact**: For both hands the URDF joint order is thumb → middle → index (aligned with the DDS API order), so `left_dex_retargeting_to_hardware` and `right_dex_retargeting_to_hardware` are both the identity permutation `[0,1,2,3,4,5,6]`. **This bug does not change runtime results**, but the naming should be fixed for clarity.

Psi0’s comments in `hand_retargeting.py` document this:

```python
# Psi0/real/teleop/robot_control/hand_retargeting.py:51-59
# left  retargeting joint_names: [thumb_0, thumb_1, thumb_2, middle_0, middle_1, index_0, index_1]
# right retargeting joint_names: [thumb_0, thumb_1, thumb_2, middle_0, middle_1, index_0, index_1]
# Same order for both → _to_hardware maps are identity
```

---

## VI. Joint definitions (identical across repos)

```python
# Left hand DDS motor order: thumb (3) → middle (2) → index (2)
class Dex3_1_Left_JointIndex(IntEnum):
    kLeftHandThumb0 = 0;  kLeftHandThumb1 = 1;  kLeftHandThumb2 = 2
    kLeftHandMiddle0 = 3; kLeftHandMiddle1 = 4
    kLeftHandIndex0 = 5;  kLeftHandIndex1 = 6

# Right hand DDS motor order: thumb (3) → index (2) → middle (2)
class Dex3_1_Right_JointIndex(IntEnum):
    kRightHandThumb0 = 0; kRightHandThumb1 = 1; kRightHandThumb2 = 2
    kRightHandIndex0 = 3; kRightHandIndex1 = 4
    kRightHandMiddle0 = 5; kRightHandMiddle1 = 6
```

Note: index vs middle **order in the enum differs** between left (middle before index) and right (index before middle), matching Unitree’s “sort by message structure” documentation.

---

## VII. Psi0 data collection format

- **Teleop logging**: Worker writes `robot_data.jsonl` (states: arm 14 + hand 14 + IMU + odom); Master writes `ik_data.jsonl` (includes `left_angles` / `right_angles`, 7 each).
- **Merged `data.json`**: Each frame has states + actions (hand 7+7 + arm 7+7 + torso rpy / height / velocity = **36-D**).
- **Training actions**: **28-D** joint angles (left hand 7 + right hand 7 + left arm 7 + right arm 7).

---

## VIII. Recommendations for our finger issues

1. **Try vector mode**: Change `unitree_dex3.yml` from `type: DexPilot` to `type: vector`, and update `robot_hand_unitree.py` so `ref_value` uses fingertip positions instead of vector differences.
2. **Add per-finger scaling**: Mirror Psi0’s thumb×1.15, index×1.05, middle×0.95.
3. **Validate Pico hand joint indices**: Check that Pico’s 25-point layout matches OpenXR-style fingertip indices `[4, 9, 14]`.

---

## IX. Key file paths

### Psi0 (`Psi0/real/`)

| File | Role |
|------|------|
| `teleop/vr.py` | VR preprocessing + hand retargeting (vector mode) |
| `teleop/TeleVision.py` | OpenTeleVision WebXR server |
| `teleop/worker.py` | Teleop Worker (teleop shared memory + logging) |
| `teleop/master_whole_body.py` | Master (whole-body IK + hand shared memory → Dex3) |
| `teleop/robot_control/hand_retargeting.py` | `HandRetargeting` wrapper + joint index maps |
| `teleop/robot_control/robot_hand_unitree.py` | `Dex3_1_Controller` (DDS command publishing) |
| `assets/unitree_hand/unitree_dex3.yml` | Retargeting config (vector mode) |
| `assets/unitree_hand/unitree_dex3_{left,right}.urdf` | Hand kinematics + joint limits |

### xr_teleoperate

| File | Role |
|------|------|
| `teleop/teleop_hand_and_arm.py` | Main teleop loop |
| `teleop/robot_control/robot_hand_unitree.py` | Dex3 control (includes DexPilot retargeting) |
| `teleop/robot_control/hand_retargeting.py` | `HandRetargeting` wrapper |
| `teleop/televuer/src/televuer/tv_wrapper.py` | TeleVuer coordinate transforms |
| `assets/unitree_hand/unitree_dex3.yml` | Retargeting config (DexPilot mode) |
| `teleop/robot_control/dex-retargeting/` | dex-retargeting submodule |

---

## X. xr_teleoperate submodule branch structure

The `xr_teleoperate/` submodule is our **fork** ([yuzihaowashu/xr_teleoperate](https://github.com/yuzihaowashu/xr_teleoperate)), with the original Unitree upstream kept as a second remote for tracking.

### Remote configuration

| Remote | URL | Purpose |
|--------|-----|---------|
| `origin` | `git@github.com:yuzihaowashu/xr_teleoperate.git` | **Our fork** (default push/pull target) |
| `unitree` | `git@github.com:unitreerobotics/xr_teleoperate.git` | Unitree upstream (for syncing new features) |

### Branch layout

```
unitree/main (Unitree upstream latest)
    │
    ├── local "main" branch (tracks unitree/main, fork point 9fadc51)
    │
origin/washu-pico-controller (our modifications)  ← ACTIVE
    │
    ├── 0a5ed64  feat: PICO controller mode — VR buttons, Dex3 trigger, safety fixes, TTS
    │              robot_hand_unitree.py (+19), teleop_hand_and_arm.py (+88)
    │
    ├── 10f07c2  feat: VR HUD overlay, connection monitoring, rich terminal UI
    │              teleop_hand_and_arm.py (+88)
    │
    └── 32fcc94  fix: hand control DDS fork issue, safe arm deploy/release, gravity comp  ← HEAD
                   robot_arm.py (+152), robot_hand_unitree.py (+142),
                   teleop_hand_and_arm.py (+181), ipc.py (+3)
```

### Our modifications (3 commits on `washu-pico-controller`)

| Commit | Summary | Key changes |
|--------|---------|-------------|
| `0a5ed64` | PICO controller mode | VR button mapping (X=start, A=stop, B=record); Dex3 trigger control (open/close interpolation via `DEX3_LEFT_CLOSE_Q` / `DEX3_RIGHT_CLOSE_Q`); joystick deadzone; safety `Move(0,0,0)` on exit; TTS announcements |
| `10f07c2` | VR HUD + monitoring | Status HUD overlay on camera frame; VR disconnect detection (2s stale threshold); auto-pause tracking on disconnect; safe ramp-up on reconnect |
| `32fcc94` | DDS fork fix + safe arm | `Process` → `Thread` for Dex3 (fixes right hand DDS); left/right `CLOSE_Q` with mirrored joint signs; `safe_deploy` 2-phase arm startup; 3-phase `go_home` (spread→zero→ramp-down); gravity comp restricted to waist only; `SIGHUP` ignored; motor gains kp=1.0 kd=0.3 |

### Usage

```bash
# Current state: on washu-pico-controller (our modifications)
cd xr_teleoperate
git branch
# * washu-pico-controller

# Switch to Unitree upstream for comparison
git checkout main
git pull unitree main

# Switch back to our modified version
git checkout washu-pico-controller

# Sync upstream changes into our branch (when Unitree releases updates)
git checkout washu-pico-controller
git rebase unitree/main    # or merge
git push origin washu-pico-controller
```
