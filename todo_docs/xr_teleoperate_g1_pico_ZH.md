# XR Teleoperate — G1 + PICO 4 Ultra Enterprise

## Status: **WIP — 等新网线**

硬件：Unitree G1 (29-DOF arms, Dex3 hands) + PICO 4 Ultra Enterprise VR + humanoid-pc (Ubuntu)

---

## 2026-03-28/29 进展总结

### Phase 1: 环境搭建 ✅

- conda env `tv`（humanoid-pc）所有依赖验证通过
  - pinocchio, numpy, unitree_sdk2py, CasADi/IPOPT, sshkeyboard 等
- `televuer` 安装修复：
  - `vuer[all]==0.0.60` + `params-proto==2.13.2` (pin 兼容版本)
  - 从本地路径重装 televuer (`pip install -e teleop/televuer`)
- `teleimager` (PC2 端) 修复：
  - `logging_mp.get_logger` → `getLogger` API 不匹配，scp 最新文件到 PC2
  - `pip install psutil` (PC2 缺失)
- SSL 证书确认就位 (`~/.config/xr_teleoperate/`)
- 子模块完整性验证通过

### Phase 2: 连接验证 ✅

- humanoid-pc ↔ PC2 (192.168.123.164) 以太网连通
- PC2 `teleimager-server` 启动正常
- humanoid-pc `ImageClient` ZMQ 图像流正常
- PICO 浏览器连接 TeleVuer (:8012) 成功
- `cam_config_server.yaml` 修改：
  - `head_camera`: `type: opencv`, `video_id: 4` (color stream), `binocular: false`, `image_shape: [480, 640]`
  - `enable_webrtc: false` → 图像走 humanoid-pc TeleVuer 中转
  - 禁用 left/right wrist camera（无腕部摄像头）

### Phase 3: 遥操作调试 ✅ (基础功能验证)

#### 3.1 机器人站立 + 追踪
- **重要发现**: 必须先让 G1 站立（L2 + Y and L2 + B, L2 + UP, R1 + X），再启动 xr_teleoperate
  - 否则非手臂关节会锁定在坐姿位置，导致手臂前伸、腰部 90° 偏转

#### 3.2 VR 黑屏修复
- **原因**: `render_to_xr()` 接收到的是 `ImageFrame` 对象而非 NumPy array
- **修复**: 改为 `render_to_xr(head_img.bgr)` + None 检查（两处）

#### 3.3 VR 灰度图修复
- **原因**: PC2 的 `video_id: 2` 是红外流（灰度），不是 RGB
- **修复**: 在 PC2 上逐一测试 `/dev/video*`，确认 `/dev/video4` 为彩色流，更新 yaml

#### 3.4 VR Controller 按键映射（新增）
| 按键 | 功能 | 文件 |
|------|------|------|
| 左手 X (aButton) | 开始追踪 (START) | `teleop_hand_and_arm.py` |
| 右手 A (aButton) | 退出遥操作 (STOP) | `teleop_hand_and_arm.py` |
| 右手 B (bButton) | 切换录制 (RECORD_TOGGLE) | `teleop_hand_and_arm.py` |
| 左摇杆 | 前后/左右行走 | `teleop_hand_and_arm.py` |
| 右摇杆 | 左右转向 | `teleop_hand_and_arm.py` |
| 左+右摇杆同按 | 紧急阻尼制动 (Damp) | `teleop_hand_and_arm.py` |
| 左/右扳机 | Dex3 手指开合 | `robot_hand_unitree.py` |

#### 3.5 摇杆死区
- 添加 `_deadzone = 0.15` 过滤摇杆微小漂移
- 防止机器人在摇杆静止时缓慢移动

#### 3.6 Dex3 手指控制（controller 模式）
- 原代码仅支持 hand tracking 模式的手指映射
- 新增：扳机值 (0~1) → `DEX3_OPEN_Q` / `DEX3_CLOSE_Q` 线性插值
- `DEX3_CLOSE_Q = [0.8, 0.8, 1.2, -1.2, -1.4, -1.2, -1.4]`
- 通过 `multiprocessing.Value` 共享扳机值到手指控制子进程

#### 3.7 TTS 语音提示（新增）
- 使用 `pyttsx3` 后台线程 + 队列，非阻塞播放
- 事件：Start teleoperation / Start recording / Stop recording / Stop teleoperation / Teleoperation ended

#### 3.8 安全性修复（关键 Bug Fix）
- **问题**: 按 A 键退出后机器人继续前进、撞倒
- **根因**: `Move(0,0,0)` 被同帧内后续的摇杆 `Move(vx,vy,vyaw)` 覆盖
- **修复**:
  1. 按 A 键 → `Move(0,0,0)` + `continue` 跳过当前帧其余指令
  2. `finally` 块开头再发一次 `Move(0,0,0)` 双保险
  3. 键盘 [q] 也触发停止语音

### 录制功能 ✅
- `EpisodeWriter` 录制正常，按 [s] 或右 B 键切换

---

## 待解决

- [ ] **网线断了** — 等新网线到后恢复以太网连接
- [ ] 备选方案: WiFi (green0161) 需要先给 PC2 配置 WiFi
- [ ] Dex3 手指闭合角度可能需要根据实际抓取调参 (`DEX3_CLOSE_Q`)
- [ ] 验证完整数据采集流程（录制 → 回放 → 训练）
- [ ] 长时间遥操作稳定性测试

---

## 修改的文件（相对 upstream unitreerobotics/xr_teleoperate）

| 文件 | 改动 |
|------|------|
| `teleop/teleop_hand_and_arm.py` | +88 行: TTS, VR 按键映射, 死区, 安全退出, Dex3 trigger, 图像修复 |
| `teleop/robot_control/robot_hand_unitree.py` | +19 行: DEX3_OPEN/CLOSE_Q, trigger-based 手指控制 |
| `teleop/teleimager` (submodule) | 指针更新 |

## 启动命令

```bash
# PC2 (SSH)
conda activate teleimager && teleimager-server

# humanoid-pc
conda activate tv
cd xr_teleoperate
bash run_xr_teleop.sh controller motion record \
    --task-name "pick_apple" --task-goal "pick up the apple"

# PICO
# 1. 浏览器打开 https://192.168.123.164:60001 → 接受证书
# 2. 浏览器打开 https://{WIFI_IP}:8012/?ws=wss://{WIFI_IP}:8012
# 3. 接受证书 → 点击 "Virtual Reality"
# 4. 左手 X 键开始追踪
```
