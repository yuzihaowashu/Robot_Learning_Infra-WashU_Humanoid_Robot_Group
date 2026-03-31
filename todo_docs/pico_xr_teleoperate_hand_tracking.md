# PICO 4U Hand Tracking + Vector Retargeting for Dex3-1

## Background

Currently our `xr_teleoperate` pipeline supports two input modes:
- `--input-mode controller`: Trigger-based open/close (linear interpolation between OPEN_Q and CLOSE_Q)
- `--input-mode hand`: DexPilot retargeting (6-pair difference vectors, optimization-based)

Psi0 uses a simpler and proven **vector retargeting** approach with per-finger scaling,
running on Apple Vision Pro. Since PICO 4U also supports WebXR hand tracking and
`xr_teleoperate`'s TeleVuer already provides the same 25-landmark hand data, we can
integrate Psi0's vector method **without changing the VR layer or DDS layer**.

## Architecture Decision

**Chosen approach**: Add vector retargeting as an independent code path inside
`xr_teleoperate`, toggled by a new `--retarget-type` CLI parameter.
The existing DexPilot and controller-trigger paths remain completely untouched.

**Why not modify Psi0 to use PICO?** Psi0's VR layer (OpenTeleVision), process
architecture (Worker/Master/SharedMemory), and data format are all fundamentally
different. Rewriting them would be a massive effort and would throw away all the
bug fixes we've accumulated in `xr_teleoperate` (DDS fork fix, safe arm deploy,
Gradio panel, SIGHUP handling, etc.).

## How It Works

### Data Flow (hand tracking mode, vector retargeting)

```
PICO 4U (WebXR hand tracking)
    │
    ▼
TeleVuer → TeleData.left_hand_pos (25,3) / right_hand_pos (25,3)
    │        (already in Unitree hand URDF frame, wrist-relative)
    ▼
teleop_hand_and_arm.py → writes to shared Array (75 floats per hand)
    │
    ▼
Dex3_1_Controller.control_process (thread)
    │
    ├─ [controller mode]  trigger → CLOSE_Q interpolation      (UNCHANGED)
    ├─ [hand + dexpilot]  6-pair diff → DexPilot optimizer      (UNCHANGED)
    └─ [hand + vector]    3 fingertips → per-finger scale → VectorOptimizer  (NEW)
          │
          ▼
       7 joint angles → DDS HandCmd_ → rt/dex3/{left,right}/cmd
```

### Key Difference: DexPilot vs Vector

| Aspect | DexPilot (current) | Vector (Psi0 style) |
|--------|-------------------|---------------------|
| Input to optimizer | 6 difference vectors between hand landmark pairs | 3 fingertip positions (indices 4, 9, 14) |
| YAML `type` | `DexPilot` | `vector` |
| YAML human indices | `[[9,14,14,0,0,0],[4,4,9,4,9,14]]` (6 pairs) | `[[0,0,0],[4,9,14]]` (3 pairs) |
| Per-finger scaling | No | Yes: thumb×1.15, index×1.05, middle×0.95 |
| Complexity | Higher (more pairs, heavier optimization) | Lower (3 targets, faster) |
| Proven on Dex3 | xr_teleoperate upstream | Psi0 (published, real robot demos) |

### OpenXR Hand Landmark Indices

The 25 landmarks follow WebXR Hand Input specification:
- Index 0: Wrist
- Index 4: Thumb tip
- Index 9: Index finger tip
- Index 14: Middle finger tip

### Per-Finger Scaling (from Psi0)

Psi0 applies per-finger scaling BEFORE passing to the vector optimizer.
This compensates for the Dex3-1's three fingers having different mechanical
ranges despite identical specs:

```python
ref_value[0] *= 1.15   # thumb  — needs larger input to close fully
ref_value[1] *= 1.05   # index  — slightly larger
ref_value[2] *= 0.95   # middle — slightly smaller
```

These values were calibrated on Apple Vision Pro. On PICO 4U, the hand tracking
accuracy/scale may differ, so these may need re-tuning.

## Implementation Details

### Files Modified

1. **`assets/unitree_hand/unitree_dex3_vector.yml`** (NEW FILE)
   - Copy of Psi0's vector config, adapted for xr_teleoperate's `RetargetingConfig`
   - Key difference from xr's existing YAML: uses `target_link_human_indices_vector`
     key name (xr's `RetargetingConfig.from_dict` expects this for vector type)

2. **`teleop/robot_control/hand_retargeting.py`**
   - Add `UNITREE_DEX3_VECTOR` to `HandType` enum
   - Points to the new YAML file

3. **`teleop/robot_control/robot_hand_unitree.py`**
   - `Dex3_1_Controller.__init__`: Accept new `retarget_type` parameter ("dexpilot" or "vector")
   - `control_process`: Add independent vector code path (branch on `retarget_type`)
   - Vector path extracts fingertip positions [4,9,14], applies scaling, calls `retarget()`
   - Existing DexPilot path and controller path are NOT modified

4. **`teleop/teleop_hand_and_arm.py`**
   - Add `--retarget-type` CLI argument (choices: "dexpilot", "vector", default: "dexpilot")
   - Pass to `Dex3_1_Controller` constructor

### Usage

```bash
# Existing controller mode (unchanged)
python teleop_hand_and_arm.py --ee dex3 --input-mode controller ...

# Existing hand tracking + DexPilot (unchanged, default)
python teleop_hand_and_arm.py --ee dex3 --input-mode hand ...

# NEW: hand tracking + Psi0-style vector retargeting
python teleop_hand_and_arm.py --ee dex3 --input-mode hand --retarget-type vector ...
```

### Known Issue: Left Hand Index Mapping

Both xr_teleoperate and Psi0 use `right_dex_retargeting_to_hardware` for
BOTH hands in the retarget output reordering. For the left hand, this swaps
index and middle finger mappings compared to using the correct
`left_dex_retargeting_to_hardware`. Since Psi0 has this same behavior and
their trained models depend on it, we replicate it exactly for compatibility.
This should be investigated separately if training our own models.

### Coordinate Consistency

Both systems use the same transformation chain:
1. WebXR landmarks → `T_ROBOT_OPENXR` (or `T_robot_openxr`)
2. World frame → wrist-relative via `inv(wrist_mat)`
3. Wrist frame → Unitree hand URDF frame via `T_TO_UNITREE_HAND`

xr_teleoperate's `TeleVuerWrapper.get_tele_data()` already outputs hand positions
in this final Unitree hand frame as `TeleData.left_hand_pos` (25,3). This is the
same frame that Psi0's `vr.py` computes as `unitree_left_hand`. So the vector
retargeting can consume `left_hand_data` directly without additional transforms.

### Future: Per-Finger Scaling Calibration for PICO

The scaling factors [1.15, 1.05, 0.95] were tuned for Apple Vision Pro.
PICO 4U's hand tracking may have different scale/accuracy characteristics.
To re-calibrate:
1. Run with `--retarget-type vector` and print the raw fingertip positions
2. Compare physical hand closure with robot hand closure
3. Adjust scaling factors (increase if robot doesn't close enough, decrease if too much)
