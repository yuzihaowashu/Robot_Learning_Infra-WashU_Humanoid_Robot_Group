# Motor 20 (L_WristPitch) Hardware Fault Report / 电机20 (左手腕俯仰) 硬件故障报告

> Date / 日期: 2026-03-28
> Robot / 机器人: Unitree G1 (29-DOF) with Dex3-1 hands
> Location / 地点: WashU Humanoid Robot Lab
> Reporter / 报告人: Yu Zihao

---

## Summary / 概述

Motor 20 (L_WristPitch, left wrist pitch) has a suspected **internal winding short circuit**, causing uncontrolled self-heating, electromagnetic locking, and uncontrolled torque pulses. The motor needs to be replaced.

电机 20（L_WristPitch，左手腕俯仰）疑似**绕组内部短路**，导致不可控自发热、电磁锁死和不可控扭矩脉冲。该电机需要更换。

---

## Evidence / 证据

### 1. Asymmetry test: Left vs Right wrist pitch / 左右对比测试

| | L_WristPitch (motor 20) | R_WristPitch (motor 27) |
|---|---|---|
| Position at boot / 开机位置 | Correct (q≈0) / 正确 | Correct (q≈0) / 正确 |
| Manual rotation at boot / 开机时手动旋转 | **Cannot rotate — locked** / **无法旋转，锁死** | Can rotate freely / 可正常旋转 |
| Temperature trend / 温度趋势 | Rises to 112°C within minutes / 几分钟内升至112°C | Stays normal ~46°C / 保持正常 |
| Final state / 最终状态 | Flips to limit and locks / 翻转到极限位锁死 | Normal / 正常 |

Both motors are symmetric, same model, same load. The only difference is motor 20 has an internal fault.

两个电机对称、同型号、同负载。唯一的区别是电机20内部有故障。

### 2. Thermal behavior / 热行为

- At boot: temp = [45, 45] (normal) / 开机时温度正常
- After a few minutes with **no control commands running**: temp = [45, **112**] / 几分钟后（**未运行任何控制程序**）：温度升至112°C
- Motor enters thermal shutdown: mode=0 / 电机进入热保护停机
- The two temperature sensors show large asymmetry (45 vs 112), indicating **localized heating** at the short circuit point / 两个温度传感器读数差异巨大，说明短路点**局部发热**

### 3. Electromagnetic locking / 电磁锁死

- After thermal shutdown (mode=0), the motor should be unpowered and freely rotatable / 热保护停机后电机应断电、可自由旋转
- **Actual behavior**: The motor is **extremely stiff** and cannot be rotated by hand / **实际表现**：电机**极其僵硬**，手动无法旋转
- This persists even after pressing L2+B (turning off all controllers) / 即使按 L2+B 关闭所有控制器后依然如此
- This is characteristic of **shorted windings**: rotating the rotor generates back-EMF through the short circuit path, producing a braking torque / 这是**绕组短路**的典型表现：旋转转子产生反电动势通过短路路径，产生制动扭矩

### 4. Uncontrolled torque pulse / 不可控扭矩脉冲

- At some point after boot (when temperature reaches threshold), the motor **suddenly flips** to an extreme angle (~106°) / 开机后某个时刻（温度达到阈值时），电机**突然翻转**到极限角度（约106°）
- This happens instantly, not gradually / 这是瞬间发生的，不是逐渐的
- After flipping, the motor locks at the extreme position / 翻转后电机锁死在极限位置

---

## Timeline / 时间线

1. **Before 2026-03-25**: Motor functioning normally / 电机工作正常
2. **2026-03-25 ~ 2026-03-27**: During teleoperation, IK solver repeatedly drove L_WristPitch to extreme angles under high load / 遥操作期间，IK求解器反复将该电机驱动到极限角度，高负载运行
3. **2026-04-01 (first observation)**: Motor overheated to **129°C** during teleoperation, entered thermal shutdown (mode=0) / 遥操作中电机过热至129°C，热保护停机
4. **2026-04-01 (subsequent restarts)**: Motor self-heats to 112°C within minutes even with no commands, locks and flips / 此后每次重启，电机在无命令状态下几分钟内自行过热至112°C，锁死并翻转

---

## Root Cause Analysis / 根因分析

```
IK drives motor to extreme angles repeatedly
IK反复驱动电机到极限角度
        ↓
Sustained high-load operation → Motor overheats to 129°C
持续高负载运行 → 电机过热至129°C
        ↓
Winding insulation damaged by heat
绕组绝缘层被高温损坏
        ↓
Inter-turn short circuit forms (irreversible)
匝间短路形成（不可逆）
        ↓
Short circuit current flows continuously → Self-heating
短路电流持续流过 → 自发热
        ↓
Temperature rises to >100°C → Thermal shutdown (mode=0)
温度升至>100°C → 热保护停机
        ↓
Uncontrolled torque pulse → Motor flips to extreme angle
不可控扭矩脉冲 → 电机翻转到极限角度
        ↓
Electromagnetic braking locks motor in place
电磁制动将电机锁死在该位置
```

---

## DDS Diagnostic Data / DDS 诊断数据

```
Motor ID:    20
Joint Name:  kLeftWristPitch / left_wrist_pitch_joint
DDS Topic:   rt/lowstate (motor_state[20])

At boot (normal):
  q = +0.000 rad    temp = [45, 45]    mode = 1

After self-heating (no control commands):
  q = +0.000 rad    temp = [45, 112]   mode = 0

After flip:
  q = +1.854 rad    temp = [49, 46]    mode = 1  (sometimes)
  q = +0.000 rad    temp = [45, 112]   mode = 0  (sometimes)
```

Note: q=0.000 when mode=0 may be a default/invalid reading, not the actual physical position.

注意：mode=0 时 q=0.000 可能是默认/无效读数，不代表实际物理位置。

---

## Software Mitigations Applied / 已应用的软件保护措施

These cannot fix the hardware fault but protect other motors from the same issue:

以下措施无法修复硬件故障，但可防止其他电机出现同样问题：

1. **Joint disabled in IK solver**: `self.opti.subject_to(self.var_q[5] == 0.0)` — IK no longer drives this joint / IK不再驱动该关节
2. **Joint disabled in motor commands**: `_G1_29_DISABLED_ARM_JOINTS = {5}` — command always q=0 / 命令始终为q=0
3. **Tightened joint limits**: All wrist joints limited to 60% of URDF range / 所有手腕关节限制在URDF范围的60%
4. **IK solver bounds tightened**: Same 60% limits applied as CasADi constraints / 同样的60%限制应用在IK优化约束中
5. **Soft boundary damping**: Velocity reduction within 0.15 rad of limits / 接近限位0.15rad范围内减速
6. **Temperature monitoring**: Warning at 70°C, auto-disable at 85°C / 70°C警告，85°C自动禁用
7. **Increased PD gains**: kp_low 80→150, kp_wrist 40→60 for better tracking / 提高PD增益改善跟踪

---

## Recommended Actions / 建议措施

### Immediate / 立即

1. **Minimize operating time** — short circuit current causes continuous heating, potential fire risk / 减少运行时间，短路电流持续发热，有潜在火灾风险
2. **Do not attempt to manually force-rotate** the left wrist / 不要试图强行手动旋转左手腕

### Repair / 维修

3. **Contact Unitree support** for motor 20 replacement / 联系Unitree售后更换电机20
4. Provide this report as reference / 提供本报告作为参考

### After Repair / 维修后

5. Keep all software safety measures active (joint limits, temperature monitoring, disabled joint list) / 保留所有软件安全措施
6. Remove motor 20 from `_G1_29_DISABLED_ARM_JOINTS` and `_DISABLED_IK_JOINTS` after replacement / 更换后从禁用列表中移除电机20
