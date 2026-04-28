# Unitree Support Request — G1 Left Arm Persistent Lockout
# Unitree 售后请求 — G1 左臂持续锁定

> Robot: Unitree G1 EDU (29-DOF) with Dex3-1 Hands
> Location: Washington University in St. Louis, Humanoid Robot Lab
> Date observed: 2026-04-27 → 2026-04-28
> Status: **All 7 left arm motors stuck in `mode=0` across full power cycle.**
>         Right arm + waist + legs all healthy (`mode=1`).
>         Active damping is felt on the left arm (it's not pure zero-torque),
>         but firmware refuses PD commands from FSM **and** from `rt/arm_sdk`.

---

## Fields to fill in before sending / 发邮件前需要填写的字段

| Field                                  | Value                       |
| -------------------------------------- | --------------------------- |
| Robot SN (机器人 SN)                   | `[FILL IN — App → About]`  |
| Firmware version (固件版本)            | `[FILL IN — App → Settings → Version Info]` |
| Left arm assembly SN (新左臂 SN)       | `[FILL IN — label on arm; or write "no SN visible, replaced 2026-04-27"]` |
| Calibration step where it was interrupted | `[FILL IN — a/b/c, see below]` |
| Customer contact (your name + email)   | `[FILL IN]`                 |

Calibration interruption step options (pick one):
- **(a)** Interrupted at "rest pose" stage — never reached the rotate-to-target image step
- **(b)** Interrupted while rotating left arm to the target pose shown in the App image
- **(c)** Finished the rotation but never confirmed/saved before exiting the App

---

## English

**Subject:** [G1-EDU SN: ____] All 7 left arm motors persistent `mode=0` across full power cycle — firmware lockout, requesting recovery procedure

Dear Unitree Support Team,

After replacing our G1's left arm assembly on 2026-04-27 and attempting an Auxiliary Calibration that was interrupted before completion, **all 7 left arm motors entered `mode=0` and stay in this state across multiple battery-removed power cycles.** We have already done extensive on-site diagnosis and need your help to recover.

### 1. Symptom

- All 7 left arm joints (15..21: `L_ShoulderPitch`, `L_ShoulderRoll`, `L_ShoulderYaw`, `L_Elbow`, `L_WristRoll`, `L_WristPitch`, `L_WristYaw`) report `mode=0` continuously.
- Right arm (22..28), waist (12..14), legs (0..11) all report `mode=1` (healthy).
- The left arm exhibits **active damping** when moved by hand (clearly more resistance than pure zero-torque mode) — so the motor drivers are powered and partially active, they are just refusing PD control.
- Encoders are working: `q` values update normally over `rt/lowstate`.
- Temperatures are normal across all 7 left arm motors (33–40°C, well below any thermal protection threshold). **No thermal cause.**
- The fault persists across the following test matrix:

| Action                          | Left arm motors | Right arm |
| ------------------------------- | --------------- | --------- |
| Boot, no remote-control input   | 7× `mode=0`     | `mode=1`  |
| Press `L2+Y` (zero torque)      | 7× `mode=0`     | `mode=1`  |
| Press `L2+B` (damping)          | 7× `mode=0`     | `mode=1`  |
| Battery-removed full power cycle (60 s wait) | unchanged | unchanged |

### 2. How it happened

1. New left arm assembly was installed on 2026-04-27.
2. Initial App-based "Auxiliary Calibration" was started but **was exited before completion** at step `[FILL IN: a/b/c]`.
3. After this, pressing `L2+UP` to enter StandUp mode caused **low-frequency oscillation (~0.5–2 rad amplitude) only on the left arm** while the right arm remained stable.
4. After 1–2 such oscillation events, the 7 left arm motors fell into `mode=0` and have stayed there since, regardless of FSM transitions or power cycles.

### 3. Diagnosis we have already performed

We wrote a custom diagnostic script (`utils/test_left_arm_trajectory.py`) that publishes `rt/arm_sdk` with `weight=1.0` and a smooth ramp to a known safe target pose, while sampling current `q` for the right arm (so the right arm should hold its current position). Result:

- **Right arm DID respond** to the `arm_sdk` publishes (small residual drift visible because we feed back its sampled `q`).
- **Left arm DID NOT move at all** — all 7 joints showed `actual_q ≈ start_q` despite a smooth 8-second ramp toward a target ~3 rad away on `L_ShoulderPitch`.

This is a clean A/B test on the **same** DDS topic, **same** publisher, **same** weight: the firmware is selectively blocking PD control on the left arm motors. **Calibration alone cannot fix this** because calibration writes via the same firmware path that is currently blocked.

### 4. What we need from you

Please advise on **any one** of the following:

1. **How to clear the firmware-level fault flag / lockout** on the 7 left arm motors of this specific G1 unit. If there is a service-mode handshake or a hidden App procedure, please send instructions.
2. **Re-flash the factory calibration offsets** for this left arm assembly (SN above). We are willing to ship the arm back if remote service is not possible.
3. **Confirm whether interrupting Auxiliary Calibration mid-procedure is known to leave motors in this persistent locked state** — if so, we will document and avoid it going forward.

We can provide:
- `check_motors.py` snapshots (full 29-motor state, post power-cycle, in 3 different remote-control states).
- `test_left_arm_trajectory.py` output showing `STUCK` verdict on left arm vs. responsive right arm.
- Video of the StandUp oscillation event prior to lockout.

We have meanwhile **physically secured the left arm** to avoid any further damage and are not running any control scripts on it.

Best regards,
[FILL IN: name]
Washington University in St. Louis — Humanoid Robot Lab

---

## 中文

**主题：** [G1-EDU SN: ____] 全断电重启后左臂 7 个电机持续 `mode=0` —— 固件级锁定，请求恢复方案

Unitree 售后团队您好，

我们在 2026-04-27 更换了 G1 的整条左臂总成，之后启动 App 的辅助标定流程但**在完成之前中途退出**。从那以后，**左臂 7 个电机全部进入 `mode=0` 状态，并且在多次"拔电池 → 等 60 秒 → 重新装上 → 开机"完整 power cycle 之后依然保持这个状态。** 我们已经在现场做了大量诊断，需要您的协助恢复。

### 一、当前症状

- 左臂 7 个关节（15..21：`L_ShoulderPitch`, `L_ShoulderRoll`, `L_ShoulderYaw`, `L_Elbow`, `L_WristRoll`, `L_WristPitch`, `L_WristYaw`）持续报 `mode=0`。
- 右臂（22..28）、腰部（12..14）、双腿（0..11）全部 `mode=1`，健康。
- 用手扳左臂能感觉到**明显的主动 damping**（阻尼明显大于纯零力矩状态）—— 说明电机驱动是通电且部分激活的，**只是拒绝接收 PD 控制命令**。
- 编码器正常工作：`rt/lowstate` 上 `q` 数值正常更新。
- 7 个左臂电机温度全部正常（33–40°C，远低于任何热保护阈值）。**不是热保护**。
- 在以下完整测试矩阵下故障都持续存在：

| 操作                                     | 左臂电机           | 右臂      |
| ---------------------------------------- | ------------------ | --------- |
| 开机后不按任何遥控器                     | 7 个 `mode=0`     | `mode=1`  |
| 按 `L2+Y`（零力矩）                      | 7 个 `mode=0`     | `mode=1`  |
| 按 `L2+B`（阻尼）                        | 7 个 `mode=0`     | `mode=1`  |
| **拔电池完整断电（等 60 秒）后开机**     | 无变化             | 无变化    |

### 二、问题发生过程

1. 2026-04-27 装上新的左臂总成。
2. 启动 App 的"辅助标定"流程，但**在第 `[填入: a/b/c]` 步未完成的情况下退出**。
3. 此后按 `L2+UP` 进入 StandUp 时，**只有左臂出现低频振荡（约 0.5–2 rad 幅度）**，右臂稳定。
4. 经过 1–2 次这样的振荡事件后，左臂 7 个电机进入 `mode=0`，并且无论 FSM 切换还是 power cycle 都无法清除。

### 三、我们已经做过的诊断

我们写了一个诊断脚本（`utils/test_left_arm_trajectory.py`），通过 `rt/arm_sdk` 发布 `weight=1.0` 并对左臂做一个平滑 ramp 到已知安全目标位姿，同时把右臂的目标设为它当前采样的 `q`（理论上右臂应原地不动）。结果：

- **右臂有响应**（因为我们每帧把它当前采样 `q` 喂回去，能看到少量自然漂移）。
- **左臂完全不动** —— 7 个关节的 `actual_q ≈ start_q`，即使我们用 8 秒钟平滑 ramp 让 `L_ShoulderPitch` 移动约 3 rad。

这是同一条 DDS topic、同一个 publisher、同一个 weight 下的干净 A/B 对比：**firmware 在选择性地屏蔽左臂的 PD 控制路径**。这意味着**仅靠标定无法解决这个问题**，因为标定写入的是同一条已经被屏蔽的固件通路。

### 四、需要您协助的内容

以下任一项均可：

1. **如何清除这台 G1 左臂 7 个电机的固件级 fault flag / lockout 状态。** 如果存在服务模式握手或 App 内的隐藏恢复流程，请告知。
2. **针对这条左臂总成（SN 见上）重新下发出厂 calibration offset。** 如果远程无法操作，我们可以把左臂寄回。
3. **请确认"标定流程中途退出"是否会导致电机进入此持续锁定状态。** 如果是，我们会记录文档并在后续避免这种操作。

我们可以提供：
- `check_motors.py` 快照（完整 29 电机状态，power cycle 之后，3 种遥控器状态）。
- `test_left_arm_trajectory.py` 输出（左臂 `STUCK`、右臂响应的对比）。
- StandUp 振荡发生时的视频。

目前我们已**物理固定左臂**避免进一步损伤，未在其上运行任何控制脚本。

此致
[填入: 姓名]
Washington University in St. Louis — Humanoid Robot Lab

---

## Attachments to send / 发送时附上的附件

1. `todo_docs/unitree_left_arm_lockout_email.md` — this email
2. **Snapshot 1**: `check_motors.py` output — boot, no remote-control input. All 7 left arm motors `mode=0`, all other motors `mode=1`. Sample data:

   ```
   15 L_ShoulderPitch       0   -0.036   0.009    0.06   40   39 FAULT(mode=0)
   16 L_ShoulderRoll        0   -0.074   0.006   -0.19   36   35 FAULT(mode=0)
   17 L_ShoulderYaw         0   -0.314   0.005    0.00   39   37 FAULT(mode=0)
   18 L_Elbow               0    1.327  -0.011    0.00   39   38 FAULT(mode=0)
   19 L_WristRoll           0    0.000   0.003   -0.12   37   36 FAULT(mode=0)
   20 L_WristPitch          0    1.444   0.001   -0.02   35   29 FAULT(mode=0)
   21 L_WristYaw            0    0.000   0.000    0.02   33   29 FAULT(mode=0)
   22 R_ShoulderPitch       1   -0.033  -0.008    0.06   44   43 OK
   23 R_ShoulderRoll        1    0.013  -0.009   -0.06   43   42 OK
   24 R_ShoulderYaw         1   -0.185   0.012    0.00   43   41 OK
   25 R_Elbow               1    1.256  -0.003    0.06   41   40 OK
   ```

3. **Snapshot 2**: same `check_motors.py` after pressing `L2+Y`. Identical pattern.
4. **Snapshot 3**: same after pressing `L2+B`. Identical pattern.
5. **Trajectory test**: `utils/test_left_arm_trajectory.py` output — verdict `STUCK` on all 7 left arm joints (max tracking error > 0.5 rad during a 3-second hold at the target pose), while right arm responded.
6. (Optional) Video of left arm StandUp oscillation prior to lockout.

---

## Internal notes (do NOT send to Unitree) / 内部备忘（不要发给 Unitree）

- Diagnostic script: `utils/test_left_arm_trajectory.py`
- Health check script: `utils/check_motors.py`
- Related on-site reports: `todo_docs/dex3_hand_error.md`, `todo_docs/motor20_wrist_pitch_fault_report.md`, `todo_docs/unitree_support_message.md`
- Recovery contact: `support@unitree.com` and the original sales rep (sales rep is usually faster).
- Until Unitree responds: keep left arm physically supported, **do not press `L2+UP`** (this is what triggered the oscillation that caused lockout in the first place).
