# XR Teleoperate — G1 + PICO 4 Ultra Enterprise

## Status: **WIP — waiting for new Ethernet cable**

Hardware: Unitree G1 (29-DOF arms, Dex3 hands) + PICO 4 Ultra Enterprise VR + humanoid-pc (Ubuntu)

---

## 2026-03-28/29 progress summary

### Phase 1: Environment setup ✅

- conda env `tv` (humanoid-pc): all dependencies verified
  - pinocchio, numpy, unitree_sdk2py, CasADi/IPOPT, sshkeyboard, etc.
- `televuer` install fixes:
  - `vuer[all]==0.0.60` + `params-proto==2.13.2` (pinned compatible versions)
  - Reinstall televuer from local path (`pip install -e teleop/televuer`)
- `teleimager` (PC2 side) fixes:
  - `logging_mp.get_logger` → `getLogger` API mismatch; scp latest files to PC2
  - `pip install psutil` (missing on PC2)
- SSL certificates confirmed in place (`~/.config/xr_teleoperate/`)
- Submodule integrity verified

### Phase 2: Connectivity ✅

- humanoid-pc ↔ PC2 (192.168.123.164) Ethernet reachable
- PC2 `teleimager-server` starts normally
- humanoid-pc `ImageClient` ZMQ image stream OK
- PICO browser connects to TeleVuer (:8012) successfully
- `cam_config_server.yaml` changes:
  - `head_camera`: `type: opencv`, `video_id: 4` (color stream), `binocular: false`, `image_shape: [480, 640]`
  - `enable_webrtc: false` → images relayed via humanoid-pc TeleVuer
  - Disabled left/right wrist cameras (no wrist cameras)

### Phase 3: Teleop debugging ✅ (basic validation)

#### 3.1 Robot standing + tracking
- **Important**: G1 must be standing (L2 + Y and L2 + B, L2 + UP, R1 + X) before starting xr_teleoperate
  - Otherwise non-arm joints stay locked in seated pose, causing arms to reach forward and ~90° waist twist

#### 3.2 VR black screen fix
- **Cause**: `render_to_xr()` received `ImageFrame` objects instead of NumPy arrays
- **Fix**: Use `render_to_xr(head_img.bgr)` + None checks (two places)

#### 3.3 VR grayscale fix
- **Cause**: PC2 `video_id: 2` is the IR stream (grayscale), not RGB
- **Fix**: Test `/dev/video*` on PC2; confirmed `/dev/video4` is color; update yaml

#### 3.4 VR controller button mapping (new)
| Button | Action | File |
|--------|--------|------|
| Left X (aButton) | Start tracking (START) | `teleop_hand_and_arm.py` |
| Right A (aButton) | Exit teleop (STOP) | `teleop_hand_and_arm.py` |
| Right B (bButton) | Toggle recording (RECORD_TOGGLE) | `teleop_hand_and_arm.py` |
| Left stick | Walk forward/back, strafe | `teleop_hand_and_arm.py` |
| Right stick | Turn left/right | `teleop_hand_and_arm.py` |
| Both sticks pressed | Emergency damp brake (Damp) | `teleop_hand_and_arm.py` |
| Left/right triggers | Dex3 finger open/close | `robot_hand_unitree.py` |

#### 3.5 Stick dead zone
- Added `_deadzone = 0.15` to filter small stick drift
- Prevents slow creep when sticks are at rest

#### 3.6 Dex3 finger control (controller mode)
- Original code only mapped fingers in hand-tracking mode
- New: trigger value (0~1) → linear blend between `DEX3_OPEN_Q` / `DEX3_CLOSE_Q`
- `DEX3_CLOSE_Q = [0.8, 0.8, 1.2, -1.2, -1.4, -1.2, -1.4]`
- Share trigger values to finger subprocess via `multiprocessing.Value`

#### 3.7 TTS voice prompts (new)
- `pyttsx3` background thread + queue for non-blocking playback
- Events: Start teleoperation / Start recording / Stop recording / Stop teleoperation / Teleoperation ended

#### 3.8 Safety fix (critical bug)
- **Issue**: After pressing A to exit, robot kept moving and could collide
- **Root cause**: `Move(0,0,0)` was overwritten in the same frame by subsequent stick `Move(vx,vy,vyaw)`
- **Fix**:
  1. On A → `Move(0,0,0)` + `continue` to skip rest of frame
  2. Send `Move(0,0,0)` again at start of `finally` as backup
  3. Keyboard [q] also triggers stop voice prompt

### Recording ✅
- `EpisodeWriter` recording OK; toggle with [s] or right B

---

## Open items

- [ ] **Ethernet cable broken** — restore link when replacement arrives
- [ ] Fallback: WiFi (green0161) needs PC2 WiFi setup first
- [ ] Dex3 close pose may need tuning per real grasps (`DEX3_CLOSE_Q`)
- [ ] Validate full data pipeline (record → replay → train)
- [ ] Long-session teleop stability test

---

## Files changed (vs upstream unitreerobotics/xr_teleoperate)

| File | Changes |
|------|---------|
| `teleop/teleop_hand_and_arm.py` | +88 lines: TTS, VR mapping, dead zone, safe exit, Dex3 triggers, image fix |
| `teleop/robot_control/robot_hand_unitree.py` | +19 lines: DEX3_OPEN/CLOSE_Q, trigger-based fingers |
| `teleop/teleimager` (submodule) | Pointer update |

## Launch commands

```bash
# PC2 (SSH)
conda activate teleimager && teleimager-server

# humanoid-pc
conda activate tv
cd xr_teleoperate
bash run_xr_teleop.sh controller motion record \
    --task-name "pick_apple" --task-goal "pick up the apple"

# PICO
# 1. Browser: https://192.168.123.164:60001 → accept certificate
# 2. Browser: https://{WIFI_IP}:8012/?ws=wss://{WIFI_IP}:8012
# 3. Accept certificate → tap "Virtual Reality"
# 4. Left X to start tracking
```
