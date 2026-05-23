# Teleoperation Quick Commands

This document is split into two standalone sections: **English** and **中文**. Do not mix the two languages line by line—this makes it easier to copy commands during robot operation.

本文档分为两个独立章节：**English** 与 **中文**。请勿在中英文之间逐行混排，以便在机器人操作时复制命令。

---

## English

### Prerequisites

- G1 robot is powered on and standing/balancing.
- `humanoid-pc` and PICO 4U are on the same WiFi network.
- Do not use `192.168.123.x` in the PICO browser. That is the robot Ethernet subnet.
- Recommended: run `run_xr_session.sh`; it prints the correct PICO URL.

### Recommended Start

```bash
cd /home/humanoid-pc/yu.zihao/Robot_Learning_Infra-WashU_Humanoid_Robot_Group
bash run_xr_session.sh
```

What `run_xr_session.sh` does:

- Detects the PC WiFi IP.
- Prints the full PICO URL:
  - `https://<PC_WiFi_IP>:8012/?ws=wss://<PC_WiFi_IP>:8012`
- Checks or generates `~/.config/xr_teleoperate/cert.pem` and `key.pem`.
- Checks PC2 reachability.
- Asks whether to kill old Gradio/teleop processes if they already exist.
- Starts the Gradio panel at `http://<PC_WiFi_IP>:7860`.

### Step 1: Start PC2 Video Stream

Use another terminal if PC2 teleimager is not already running.

```bash
ssh unitree@192.168.123.164
conda activate teleimager && teleimager-server
```

### Step 2: Start Host Gradio Panel Manually

Only use this if you are not using `run_xr_session.sh`.

```bash
cd /home/humanoid-pc/yu.zihao/Robot_Learning_Infra-WashU_Humanoid_Robot_Group
conda activate tv
python gradio_panel.py
```

Open browser: `http://localhost:7860`

### Step 3: Gradio Panel Setup

1. Check Preflight:
   - PICO VR URL uses WiFi IP, not `192.168.123.x`.
   - XR HTTPS certificate is OK.
   - PC2 is reachable.
2. Fill Task Name and Task Goal.
3. Input Mode is fixed to `controller`; triggers control fingers.
4. Choose Arm Mode:
   - `bimanual`: both arms follow XR.
   - `left-only`: left arm follows XR; right arm is held automatically in a relaxed pose.
   - `right-only`: right arm follows XR; left arm is held automatically in a relaxed pose.
5. Gradio does not show inactive-arm pose options: single-arm modes always use **`relaxed`** for the non-XR arm from Launch deploy onward. Use CLI `--inactive-arm-pose=...` only if you need `spread`, `current`, or `default`.
6. Balance mode is always ON in Gradio.
7. Walking commands stay OFF unless `Toggle Locomotion` is manually enabled.
8. Keep `Show PC VR Mirror` checked if observers need to watch.
9. Click `(1) Launch Teleop`.

### Step 4: PICO 4U

After Launch Teleop, open the full URL shown in Gradio on the PICO browser:

`https://<PC_WiFi_IP>:8012/?ws=wss://<PC_WiFi_IP>:8012`

Click `Enter VR` to enter immersive mode.

## Special Attention: Purple Button on the right joystick. 
# Remember to press it twice to re-localize the VR to make sure the direction aligns. 


If the page opens but cannot enter VR, accept/trust the HTTPS certificate on PICO, or regenerate the certificate with the current WiFi IP.

### Step 5: Control and Recording

Recommended single-person workflow:

- PC: click `(1) Launch Teleop`.
- PICO: wear headset, open the 8012 URL, and click `Enter VR`.
- VR Left X: start/resume tracking, recalibrate controller pose, and start a new episode.
- VR Left Y: mark the splitter transition and toggle `forward` / `backward`.
- VR Right A: stop and save the current episode.
- Wait for `Episode xxxx saved. Press X for next episode.`
- Press VR Left X again for the next episode.
- Final exit: click `(2) Stop Teleop + Relax Arms` on PC, or use EMERGENCY STOP if needed.

### Controller Mode VR Buttons

- Left X: start/resume tracking and begin the next episode.
- Left Y: mark splitter transition; toggles `forward` / `backward`.
- Right A: stop current episode and save.
- Right B: discard current episode; only Right A saves.
- Both joysticks pressed: emergency damping.

### PC Gradio Backup Buttons

- Start Tracking: start tracking only.
- Toggle Recording: manually start/stop recording.
- Toggle Locomotion: toggle walking commands; balance remains ON.
- EMERGENCY STOP: stop and exit teleop.

### Save Path

Episodes are saved to:

`<Save Path>/<Task Name>/episode_xxxx`

Default Save Path: `xr_recordings`

### Single-Arm Data Collection

`left-only` and `right-only` keep the same 14D arm data shape. The inactive side is still held by the low-level arm controller so it does not drop under gravity.

Recording behavior:

- Active arm action: real teleop target/action.
- Inactive arm action: zeros.
- Inactive end-effector action: zeros.
- Real joint states are still recorded for both arms.

Episode metadata:

- `info.metadata.arm_mode`
- `info.metadata.inactive_arm_pose`
- `info.metadata.inactive_arm_action = zero_delta`

### Finish / Recovery

Finish with `(2) Stop Teleop + Relax Arms` on the panel. Stop first parks arms in the safe outward pose, then relaxes toward default/down pose. Use `(3) Relax Arms to Default Pose` only as recovery if automatic relax did not complete.

### Direct CLI Start: Controller Mode

```bash
cd /home/humanoid-pc/yu.zihao/Robot_Learning_Infra-WashU_Humanoid_Robot_Group/xr_teleoperate/teleop
conda activate tv
python teleop_hand_and_arm.py \
    --arm=G1_29 --ee=dex3 \
    --input-mode=controller \
    --arm-mode=bimanual \
    --record \
    --task-dir=../../xr_recordings \
    --task-name="pick_apple" \
    --task-goal="pick up the apple"
```

### Direct CLI Start: Left-Only Controller Mode

```bash
python teleop_hand_and_arm.py \
    --arm=G1_29 --ee=dex3 \
    --input-mode=controller \
    --arm-mode=left-only \
    --inactive-arm-pose=default \
    --record \
    --task-dir=../../xr_recordings \
    --task-name="single_left_arm" \
    --task-goal="perform the task with the left arm"
```

### Direct CLI Start: Hand Tracking + DexPilot

```bash
python teleop_hand_and_arm.py \
    --arm=G1_29 --ee=dex3 \
    --input-mode=hand \
    --retarget-type=dexpilot \
    --record \
    --task-dir=../../xr_recordings \
    --task-name="pick_apple" \
    --task-goal="pick up the apple"
```

### Direct CLI Start: Hand Tracking + Vector Retargeting

```bash
python teleop_hand_and_arm.py \
    --arm=G1_29 --ee=dex3 \
    --input-mode=hand \
    --retarget-type=vector \
    --record \
    --task-dir=../../xr_recordings \
    --task-name="pick_apple" \
    --task-goal="pick up the apple"
```

### Important CLI Options

| Option | Description |
| --- | --- |
| `--input-mode=controller` | Controller triggers linearly open/close Dex3 fingers. This is the only input mode exposed in Gradio. |
| `--input-mode=hand` | Kept for CLI compatibility; not exposed in Gradio to avoid accidental use. |
| `--arm-mode=bimanual\|left-only\|right-only` | Select bimanual or single-arm collection. |
| `--inactive-arm-pose=default\|relaxed\|spread\|current` | Hold pose for the inactive side in single-arm modes. |
| `--retarget-type=dexpilot` | Original xr_teleoperate 6-vector hand retargeting. |
| `--retarget-type=vector` | Simpler Psi0-style fingertip retargeting. |
| `--motion` / `--no-motion` | Default is `--motion`: keep Unitree balance/motion controller active and publish arm commands through `rt/arm_sdk`. |
| `--record` | Enable recording. Left X starts an episode; Right A saves it. |
| `--ipc` | Enable ZMQ IPC. Gradio adds this automatically. |
| `--force-zmq-video` | Force host PC ZMQ frames for PICO display instead of PICO direct WebRTC to PC2. |
| `--mirror-vr` / `--no-mirror-vr` | Show or hide the PC VR Mirror observer window. |
| `--vr-pose-jump-threshold` | One-frame controller wrist target jump threshold. Default: 0.15 m. |
| `--vr-pose-filter-alpha` | Controller wrist target translation low-pass filter alpha. Default: 0.35. |
| `--vr-rot-filter-alpha` | Controller wrist target rotation low-pass filter alpha. Default: `1.0` (no extra filtering). |
| `--ik-rotation-weight` | IK wrist orientation tracking weight. Default: `1.0`. |
| `--wrist-kp` / `--wrist-kd` | G1_29 wrist joint PD gains. Defaults: `kp=60.0`, `kd=2.0`. Tune only if IK rotation weight is not enough. |
| `--teleop-start-ramp-sec` | Smooth transition time from current robot arm pose to the first IK target. Default: 2.5 s. |
| `--park-arms-on-stop=spread\|default` | Final stop pose. `spread` is safer for Dex3 clearance. |

### Troubleshooting

**Black video:** Restart `teleimager-server` on PC2.

**PICO page does not open:** Make sure Launch Teleop was clicked. Port 8012 is started by TeleVuer. URL must include `?ws=wss://<PC_WiFi_IP>:8012`. Make sure PICO uses the WiFi IP, not `192.168.123.x`. If needed: `sudo ufw allow 8012/tcp`.

**PICO page opens but cannot enter VR:** The HTTPS certificate is not trusted by PICO, or it was generated for an old WiFi IP. Run `bash run_xr_session.sh` again, then accept/trust the certificate on PICO.

**Launch Teleop exits quickly:** Check Gradio Output or `teleop_latest.log`. Common causes: PC2 teleimager not running, bad camera config, wrong DDS network interface.

**Arm jumps backward/sideways:** Usually caused by a one-frame PICO/OpenXR controller pose jump or relocalization. The code pauses tracking for wrist target jumps above 0.15 m. Move your hands back to a safe pose and press Left X to recalibrate/resume.

**Arm drops when pressing Left X:** The first IK target may be far from the robot's current safe spread pose. `--teleop-start-ramp-sec` smooths this transition. Default: 2.5 s. Before pressing Left X, keep your hands above table height.

**Right hand does not move:** Check `[Hand DBG] R_state` in logs for abnormal values such as -238. If present, power-cycle the robot to clear encoder faults.

**DDS has no communication:** Check network interface: `ip link show`. Typical interface: `enp2s0`.

### Live Log

```bash
tail -f /home/humanoid-pc/yu.zihao/Robot_Learning_Infra-WashU_Humanoid_Robot_Group/teleop_latest.log
```

---

## 中文

### 前提条件

- G1 机器人已经开机，并处于站立/平衡状态。
- `humanoid-pc` 和 PICO 4U 连接到同一个 WiFi。
- 不要在 PICO 浏览器里使用 `192.168.123.x`，这是机器人有线网段。
- 推荐直接运行 `run_xr_session.sh`，它会自动打印正确的 PICO URL。

### 推荐启动方式

```bash
cd /home/humanoid-pc/yu.zihao/Robot_Learning_Infra-WashU_Humanoid_Robot_Group
bash run_xr_session.sh
```

`run_xr_session.sh` 会做这些事:

- 检测 PC WiFi IP。
- 打印完整 PICO URL: `https://<PC_WiFi_IP>:8012/?ws=wss://<PC_WiFi_IP>:8012`
- 检查或生成 `~/.config/xr_teleoperate/cert.pem` 和 `key.pem`。
- 检查 PC2 是否可达。
- 如果已有旧的 Gradio/teleop 进程，会询问是否停止旧进程。
- 启动 Gradio 面板: `http://<PC_WiFi_IP>:7860`。

### 第 1 步: 启动 PC2 视频流

如果 PC2 的 teleimager 还没有启动，在另一个 terminal 里运行:

```bash
ssh unitree@192.168.123.164
conda activate teleimager && teleimager-server
```

### 第 2 步: 手动启动 Host Gradio 面板

如果没有使用 `run_xr_session.sh`，才需要手动运行这一段。

```bash
cd /home/humanoid-pc/yu.zihao/Robot_Learning_Infra-WashU_Humanoid_Robot_Group
conda activate tv
python gradio_panel.py
```

打开浏览器: `http://localhost:7860`

### 第 3 步: Gradio 面板设置

1. 检查 Preflight:
   - PICO VR URL 使用 WiFi IP，不是 `192.168.123.x`。
   - XR HTTPS certificate 显示 OK。
   - PC2 reachable。
2. 填写 Task Name 和 Task Goal。
3. Input Mode 固定为 `controller`，扳机控制手指开合。
4. 选择 Arm Mode:
   - `bimanual`: 双臂都跟随 XR。
   - `left-only`: 左臂跟随 XR；右臂自动保持放松姿态（relaxed）。
   - `right-only`: 右臂跟随 XR；左臂自动保持放松姿态（relaxed）。
5. Gradio 不再展示 inactive arm 姿态选项：单臂模式从 Launch deploy 阶段起，就固定对非 XR 手臂使用 **`relaxed`**。若需要 `spread` / `current` / `default`，请用命令行 `--inactive-arm-pose=...`（见下文 CLI）。
6. Gradio 中 balance mode 默认一直开启。
7. 行走命令默认关闭，除非手动点击 `Toggle Locomotion`。
8. 如果旁观者需要看画面，保持 `Show PC VR Mirror` 勾选。
9. 点击 `(1) Launch Teleop`。

### 第 4 步: PICO 4U

Launch Teleop 后，在 PICO 浏览器打开 Gradio 显示的完整 URL:

`https://<PC_WiFi_IP>:8012/?ws=wss://<PC_WiFi_IP>:8012`

点击 `Enter VR` 进入沉浸模式。

如果页面能打开但不能进入 VR，先在 PICO 上接受/信任 HTTPS 证书，或者用当前 WiFi IP 重新生成证书。

### 第 5 步: 控制和录制

推荐单人操作流程:

- PC: 点击 `(1) Launch Teleop`。
- PICO: 戴上头显，打开 8012 URL，点击 `Enter VR`。
- VR Left X: 开始/恢复 tracking，重新校准 controller pose，并开始新 episode。
- VR Left Y: 标记 splitter transition，并在 `forward` / `backward` 之间切换。
- VR Right A: 停止并保存当前 episode。
- 等待 `Episode xxxx saved. Press X for next episode.`。
- 再按 VR Left X 采集下一条 episode。
- 最后退出: 在 PC 点击 `(2) Stop Teleop + Relax Arms`，必要时使用 EMERGENCY STOP。

### Controller 模式 VR 按键

- Left X: 开始/恢复 tracking，并开始下一条 episode。
- Left Y: 标记 splitter transition；在 `forward` / `backward` 之间切换。
- Right A: 停止当前 episode 并保存。
- Right B: 丢弃当前 episode；只有 Right A 会保存。
- 同时按下两个摇杆: emergency damping。

### PC Gradio 备用按钮

- Start Tracking: 只启动 tracking。
- Toggle Recording: 手动开始/停止录制。
- Toggle Locomotion: 开关行走命令；腿部平衡仍然保持开启。
- EMERGENCY STOP: 停止并退出 teleop。

### 保存路径

episode 会保存到:

`<Save Path>/<Task Name>/episode_xxxx`

默认 Save Path: `xr_recordings`

### 单臂数据采集

`left-only` 和 `right-only` 仍然保持 14D arm 数据结构不变。inactive side 仍然由底层 arm controller 保持姿态，因此不会因为重力自然下垂。

录制行为:

- active arm action: 真实 teleop target/action。
- inactive arm action: 全 0。
- inactive end-effector action: 全 0。
- 左右双臂的真实 joint state 仍然都会记录。

episode metadata:

- `info.metadata.arm_mode`
- `info.metadata.inactive_arm_pose`
- `info.metadata.inactive_arm_action = zero_delta`

### 结束和恢复

正常结束时，点击面板上的 `(2) Stop Teleop + Relax Arms`。Stop 会先把手臂停到安全外展位，再自动放松到 default/down pose。`(3) Relax Arms to Default Pose` 只作为自动 relax 没完成时的恢复按钮。

### 直接命令行启动: Controller 模式

```bash
cd /home/humanoid-pc/yu.zihao/Robot_Learning_Infra-WashU_Humanoid_Robot_Group/xr_teleoperate/teleop
conda activate tv
python teleop_hand_and_arm.py \
    --arm=G1_29 --ee=dex3 \
    --input-mode=controller \
    --arm-mode=bimanual \
    --record \
    --task-dir=../../xr_recordings \
    --task-name="pick_apple" \
    --task-goal="pick up the apple"
```

### 直接命令行启动: 只控制左臂

```bash
python teleop_hand_and_arm.py \
    --arm=G1_29 --ee=dex3 \
    --input-mode=controller \
    --arm-mode=left-only \
    --inactive-arm-pose=default \
    --record \
    --task-dir=../../xr_recordings \
    --task-name="single_left_arm" \
    --task-goal="perform the task with the left arm"
```

### 直接命令行启动: Hand Tracking + DexPilot

```bash
python teleop_hand_and_arm.py \
    --arm=G1_29 --ee=dex3 \
    --input-mode=hand \
    --retarget-type=dexpilot \
    --record \
    --task-dir=../../xr_recordings \
    --task-name="pick_apple" \
    --task-goal="pick up the apple"
```

### 直接命令行启动: Hand Tracking + Vector Retargeting

```bash
python teleop_hand_and_arm.py \
    --arm=G1_29 --ee=dex3 \
    --input-mode=hand \
    --retarget-type=vector \
    --record \
    --task-dir=../../xr_recordings \
    --task-name="pick_apple" \
    --task-goal="pick up the apple"
```

### 关键 CLI 参数

| 选项 | 说明 |
| --- | --- |
| `--input-mode=controller` | controller 扳机会线性控制 Dex3 手指开合。Gradio 只开放这个模式。 |
| `--input-mode=hand` | 保留给命令行兼容使用；Gradio 不开放，避免误选。 |
| `--arm-mode=bimanual\|left-only\|right-only` | 选择双臂或单臂采集。 |
| `--inactive-arm-pose=default\|relaxed\|spread\|current` | 单臂模式中 inactive side 的保持姿态。 |
| `--retarget-type=dexpilot` | xr_teleoperate 原版 6-vector 手部 retargeting。 |
| `--retarget-type=vector` | 更简单的 Psi0 风格 fingertip retargeting。 |
| `--motion` / `--no-motion` | 默认是 `--motion`: 保持 Unitree balance/motion controller 开启，并通过 `rt/arm_sdk` 发送手臂命令。 |
| `--record` | 启用录制。Left X 开始 episode；Right A 保存 episode。 |
| `--ipc` | 启用 ZMQ IPC。Gradio 会自动添加。 |
| `--force-zmq-video` | 强制使用 host PC 的 ZMQ 画面给 PICO 显示，避免 PICO 直连 PC2 WebRTC。 |
| `--mirror-vr` / `--no-mirror-vr` | 开启或关闭 PC VR Mirror 旁观窗口。 |
| `--vr-pose-jump-threshold` | controller wrist target 单帧跳变阈值。默认: 0.15 m。 |
| `--vr-pose-filter-alpha` | controller wrist target 平移低通滤波 alpha。默认: 0.35。 |
| `--vr-rot-filter-alpha` | controller wrist target 旋转低通滤波 alpha。默认: `1.0`（不额外滤波）。 |
| `--ik-rotation-weight` | IK wrist 姿态跟踪权重。默认: `1.0`。 |
| `--wrist-kp` / `--wrist-kd` | G1_29 wrist 关节 PD 增益。默认: `kp=60.0`, `kd=2.0`。只有 IK 姿态权重仍不够时再调。 |
| `--teleop-start-ramp-sec` | 从机器人当前手臂姿态平滑过渡到第一帧 IK target 的时间。默认: 2.5 s。 |
| `--park-arms-on-stop=spread\|default` | 最终停止姿态。`spread` 对 Dex3 手指留空间更安全。 |

### 常见排查

**视频流全黑:** 重启 PC2 上的 `teleimager-server`。

**PICO 页面打不开:** 确认已经点击 Launch Teleop。8012 端口由 TeleVuer 启动。URL 必须包含 `?ws=wss://<PC_WiFi_IP>:8012`。确认 PICO 使用的是 WiFi IP，不是 `192.168.123.x`。必要时运行: `sudo ufw allow 8012/tcp`。

**PICO 页面能打开但不能进入 VR:** HTTPS 证书没有被 PICO 信任，或者证书是旧 WiFi IP 生成的。重新运行 `bash run_xr_session.sh`，然后在 PICO 上接受/信任证书。

**Launch Teleop 后很快退出:** 查看 Gradio Output 或 `teleop_latest.log`。常见原因: PC2 teleimager 未启动、相机配置异常、DDS 网络接口不对。

**手臂突然向后或侧向跳:** 通常是 PICO/OpenXR controller pose 单帧跳变或重定位。当前代码会在 wrist target 跳变超过 0.15 m 时暂停 tracking。把手放回安全位置，再按 Left X 重新校准/恢复。

**按 Left X 开始时手臂突然下落:** 第一帧 IK target 可能离机器人当前安全外展位很远。`--teleop-start-ramp-sec` 会平滑这个过渡。默认: 2.5 s。按 Left X 前，建议把双手抬到桌面上方安全高度。

**右手不动:** 查看日志里的 `[Hand DBG] R_state` 是否有异常值，例如 -238。如果有，断电重启机器人以清除编码器故障。

**DDS 无通信:** 检查网络接口: `ip link show`。常见接口名: `enp2s0`。

### 实时日志

```bash
tail -f /home/humanoid-pc/yu.zihao/Robot_Learning_Infra-WashU_Humanoid_Robot_Group/teleop_latest.log
```
