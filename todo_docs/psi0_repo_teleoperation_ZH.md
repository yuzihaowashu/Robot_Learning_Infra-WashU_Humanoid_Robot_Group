# Psi0 仓库遥操作分析 — 与 xr_teleoperate 对比（聚焦 Dex3-1 三指手）

> 日期: 2026-03-31
> 仓库: https://github.com/physical-superintelligence-lab/Psi0
> 已作为 submodule 克隆到 `Psi0/`

---

## 一、Psi0 遥操作方案概述

Psi0 **不使用 xr_teleoperate**，而是基于 **Apple Vision Pro + Vuer + OpenTeleVision** 的独立遥操作栈。

| 维度 | xr_teleoperate（我们的） | Psi0 (`real/teleop/`) |
|------|--------------------------|------------------------|
| VR 设备 | PICO 4U (TeleVuer/WebXR) | Apple Vision Pro (Vuer/OpenTeleVision) |
| 中间件 | TeleVuer + TeleVuerWrapper | Vuer + OpenTeleVision |
| 架构 | 单进程 `teleop_hand_and_arm.py` | Worker/Master 多进程分离 |
| retargeting 位置 | 手部控制子进程内 (`robot_hand_unitree.py`) | VR 预处理阶段 (`vr.py`)，7维结果通过共享内存传到手部 |
| 致谢/上游 | unitree 官方 xr_teleoperate | avp_teleoperate、OpenTeleVision、vuer |

**共同点**：同样的 Unitree G1 + Dex3-1 手硬件、同样的 DDS topic (`rt/dex3/{left,right}/cmd`)、
同样的 `HandCmd_` 消息、几乎相同的 `hand_retargeting.py` 和 `JointIndex` 枚举。

---

## 二、最关键差异：retargeting 算法类型

### xr_teleoperate — DexPilot 模式

```yaml
# xr_teleoperate/assets/unitree_hand/unitree_dex3.yml
left:
  type: DexPilot
```

- 输入：25 点手部骨架 → 6 对向量差分（通过 `target_link_human_indices_dexpilot: [[9,14,14,0,0,0],[4,4,9,4,9,14]]`）
- 算法：NLopt + DexPilot 优化器（Huber 损失 + project_dist/escape_dist 投影距离）
- 代码位置：`robot_hand_unitree.py` control_process 内

```python
# xr_teleoperate/teleop/robot_control/robot_hand_unitree.py:187-191
ref_left_value  = left_hand_data[self.hand_retargeting.left_indices[1,:]] - left_hand_data[self.hand_retargeting.left_indices[0,:]]
ref_right_value = right_hand_data[self.hand_retargeting.right_indices[1,:]] - right_hand_data[self.hand_retargeting.right_indices[0,:]]
left_q_target   = self.hand_retargeting.left_retargeting.retarget(ref_left_value)[self.hand_retargeting.right_dex_retargeting_to_hardware]
right_q_target  = self.hand_retargeting.right_retargeting.retarget(ref_right_value)[self.hand_retargeting.right_dex_retargeting_to_hardware]
```

### Psi0 — vector 模式

```yaml
# Psi0/real/assets/unitree_hand/unitree_dex3.yml
left:
  type: vector
```

- 输入：25 点手部骨架 → 仅取 3 个指尖位置 `[4, 9, 14]`（thumb/index/middle tip）
- 算法：vector 优化器（更简单直接）
- 额外处理：**逐指缩放因子** thumb×1.15, index×1.05, middle×0.95
- 代码位置：`vr.py` VuerPreprocessor.process() 内

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

## 三、数据流对比

### xr_teleoperate

```
Pico VR 25点手部 → TeleVuerWrapper 坐标变换 → 共享内存(75 float, 25×3)
→ Dex3 控制子进程读取 → DexPilot retargeting → 7维 q_target → DDS HandCmd_
```

### Psi0

```
AVP 25点手部 → vr.py 坐标变换 → 提取3指尖 → vector retargeting → 7维 q_target
→ 共享内存(14 float, 左7+右7) → Dex3 控制子进程直接读取并发 DDS HandCmd_
```

Psi0 的手部控制子进程更简单——只负责读共享内存并下发，不做 retargeting：

```python
# Psi0/real/teleop/robot_control/robot_hand_unitree.py:228-229
left_q_target = hand_shm_array[0:7]
right_q_target = hand_shm_array[7:14]
```

---

## 四、配置参数对比

| 参数 | xr_teleoperate | Psi0 |
|------|----------------|------|
| retarget type | DexPilot | vector |
| 逐指缩放 | 无 | thumb×1.15, index×1.05, middle×0.95 |
| 低通滤波 alpha | 0.2 | 0.2 |
| PD 增益 kp/kd | 1.5 / 0.2 | 1.5 / 0.2 |
| 控制频率 | 100 Hz | 100 Hz |
| DDS topic | rt/dex3/{left,right}/cmd | rt/dex3/{left,right}/cmd |
| URDF | unitree_dex3_{left,right}.urdf | 相同 |

---

## 五、共同的可疑代码：左手 retarget 用了 right 索引

两个仓库都存在同一问题——左手 retarget 结果用 `right_dex_retargeting_to_hardware` 做索引重排：

```python
# 两个仓库的写法相同：
left_q_target = ...retarget(ref_left_value)[hand_retargeting.right_dex_retargeting_to_hardware]  # ← 应该用 left？
```

**实际影响**：经分析，左右手 URDF 关节顺序均为 thumb→middle→index（与 DDS API 顺序一致），
因此 `left_dex_retargeting_to_hardware` 和 `right_dex_retargeting_to_hardware` 都是恒等映射 `[0,1,2,3,4,5,6]`。
**这个 bug 在实际运行中不会产生错误结果**，但语义上应修正。

Psi0 `hand_retargeting.py` 的归档注释验证了这一点：

```python
# Psi0/real/teleop/robot_control/hand_retargeting.py:51-59
# left  retargeting joint_names: [thumb_0, thumb_1, thumb_2, middle_0, middle_1, index_0, index_1]
# right retargeting joint_names: [thumb_0, thumb_1, thumb_2, middle_0, middle_1, index_0, index_1]
# 两者顺序相同 → _to_hardware 映射均为恒等
```

---

## 六、关节定义（两仓库一致）

```python
# 左手 DDS 电机顺序：thumb(3) → middle(2) → index(2)
class Dex3_1_Left_JointIndex(IntEnum):
    kLeftHandThumb0 = 0;  kLeftHandThumb1 = 1;  kLeftHandThumb2 = 2
    kLeftHandMiddle0 = 3; kLeftHandMiddle1 = 4
    kLeftHandIndex0 = 5;  kLeftHandIndex1 = 6

# 右手 DDS 电机顺序：thumb(3) → index(2) → middle(2)
class Dex3_1_Right_JointIndex(IntEnum):
    kRightHandThumb0 = 0; kRightHandThumb1 = 1; kRightHandThumb2 = 2
    kRightHandIndex0 = 3; kRightHandIndex1 = 4
    kRightHandMiddle0 = 5; kRightHandMiddle1 = 6
```

注意：左右手的 index/middle 在枚举中顺序不同（左手 middle 在前，右手 index 在前），
与 Unitree 官方文档 "Sort by message structure" 一致。

---

## 七、Psi0 数据采集格式

- 遥操作录制：Worker 写 `robot_data.jsonl`（states: arm 14 + hand 14 + IMU + odom），Master 写 `ik_data.jsonl`（含 left_angles/right_angles 各 7）
- 合并后 `data.json`：每帧含 states + actions（hand 7+7 + arm 7+7 + torso rpy/height/velocity = 36 维）
- 训练用 action：28 维关节角（左手7 + 右手7 + 左臂7 + 右臂7）

---

## 八、对我们手指问题的建议

1. **尝试 vector 模式**：将 `unitree_dex3.yml` 的 `type: DexPilot` 改为 `type: vector`，
   同时修改 `robot_hand_unitree.py` 中 ref_value 计算方式（从向量差分改为指尖位置提取）
2. **添加逐指缩放**：参考 Psi0 的 thumb×1.15, index×1.05, middle×0.95
3. **确认 Pico 手部骨架索引**：验证 Pico 的 25 点手部关节与 OpenXR 标准 [4,9,14] 指尖索引是否一致

---

## 九、关键文件路径

### Psi0 (`Psi0/real/`)

| 文件 | 作用 |
|------|------|
| `teleop/vr.py` | VR 预处理 + 手指 retargeting（vector 模式） |
| `teleop/TeleVision.py` | OpenTeleVision WebXR 服务端 |
| `teleop/worker.py` | 遥操作 Worker（写 teleop 共享内存 + 录数据） |
| `teleop/master_whole_body.py` | Master（whole-body IK + 手部共享内存 → Dex3） |
| `teleop/robot_control/hand_retargeting.py` | HandRetargeting 封装 + 关节索引映射 |
| `teleop/robot_control/robot_hand_unitree.py` | Dex3_1_Controller（DDS 下发） |
| `assets/unitree_hand/unitree_dex3.yml` | retargeting 配置（vector 模式） |
| `assets/unitree_hand/unitree_dex3_{left,right}.urdf` | 手部运动学 + 关节限位 |

### xr_teleoperate

| 文件 | 作用 |
|------|------|
| `teleop/teleop_hand_and_arm.py` | 遥操作主流程 |
| `teleop/robot_control/robot_hand_unitree.py` | Dex3 控制（含 DexPilot retargeting） |
| `teleop/robot_control/hand_retargeting.py` | HandRetargeting 封装 |
| `teleop/televuer/src/televuer/tv_wrapper.py` | TeleVuer 坐标变换 |
| `assets/unitree_hand/unitree_dex3.yml` | retargeting 配置（DexPilot 模式） |
| `teleop/robot_control/dex-retargeting/` | dex-retargeting 子模块 |

---

## 十、xr_teleoperate submodule 分支结构

`xr_teleoperate/` 子模块指向 **我们的 fork**（[yuzihaowashu/xr_teleoperate](https://github.com/yuzihaowashu/xr_teleoperate)），Unitree 上游作为第二个 remote 保留用于同步。

### Remote 配置

| Remote | URL | 用途 |
|--------|-----|------|
| `origin` | `git@github.com:yuzihaowashu/xr_teleoperate.git` | **我们的 fork**（默认 push/pull 目标） |
| `unitree` | `git@github.com:unitreerobotics/xr_teleoperate.git` | Unitree 上游（用于同步新功能） |

### 分支布局

```
unitree/main（Unitree 上游最新）
    │
    ├── 本地 "main" 分支（跟踪 unitree/main，fork 起点 9fadc51）
    │
origin/washu-pico-controller（我们的修改）  ← 当前激活
    │
    ├── 0a5ed64  feat: PICO controller 模式 — VR 按钮、Dex3 扳机、安全修复、TTS
    │              robot_hand_unitree.py (+19), teleop_hand_and_arm.py (+88)
    │
    ├── 10f07c2  feat: VR HUD 叠加、连接监控、终端 UI
    │              teleop_hand_and_arm.py (+88)
    │
    └── 32fcc94  fix: 手部 DDS fork 问题、安全手臂启动/释放、重力补偿  ← HEAD
                   robot_arm.py (+152), robot_hand_unitree.py (+142),
                   teleop_hand_and_arm.py (+181), ipc.py (+3)
```

### 我们的修改（washu-pico-controller 上的 3 个 commit）

| Commit | 摘要 | 关键改动 |
|--------|------|----------|
| `0a5ed64` | PICO controller 模式 | VR 按钮映射（X=开始, A=停止, B=录制）；Dex3 扳机控制（开/闭插值，`DEX3_LEFT_CLOSE_Q`/`DEX3_RIGHT_CLOSE_Q`）；摇杆死区；安全 `Move(0,0,0)` 退出；TTS 语音 |
| `10f07c2` | VR HUD + 监控 | 相机画面叠加状态 HUD；VR 断连检测（2s 超时）；断连自动暂停；重连后安全速度爬升 |
| `32fcc94` | DDS fork 修复 + 安全手臂 | `Process`→`Thread`（修复右手 DDS）；左右手 `CLOSE_Q` 方向取反；`safe_deploy` 两阶段安全启动；三阶段 `go_home`（外展→零位→缓降）；重力补偿仅施加于腰部；忽略 `SIGHUP`；增益 kp=1.0 kd=0.3 |

### 使用方法

```bash
# 当前状态：在 washu-pico-controller 分支（包含我们的修改）
cd xr_teleoperate
git branch
# * washu-pico-controller

# 切到 Unitree 上游对比
git checkout main
git pull unitree main

# 切回我们的修改版
git checkout washu-pico-controller

# 合并 Unitree 新功能到我们的分支
git checkout washu-pico-controller
git rebase unitree/main    # 或 merge
git push origin washu-pico-controller
```
