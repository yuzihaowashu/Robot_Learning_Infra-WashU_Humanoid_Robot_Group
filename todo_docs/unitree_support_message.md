# Unitree Support Request / Unitree 售后维修请求

> Robot: Unitree G1 (29-DOF) with Dex3-1 Hands
> Location: Washington University in St. Louis, Humanoid Robot Lab
> Date: 2026-03-28

---

## English

Dear Unitree Support Team,

We are experiencing two hardware issues on our G1 robot:

### Issue 1: Left Wrist Pitch Motor (Motor 20) — Suspected Winding Short Circuit

The left wrist pitch motor (motor_state[20], left_wrist_pitch_joint) has developed an internal fault, likely a winding short circuit caused by a previous overheating event (129°C).

**Symptoms:**
- At boot, the motor is at the correct position but **cannot be manually rotated**, while the symmetric right wrist pitch motor (motor 27) can be rotated freely
- Without any control software running, the motor's second temperature sensor rises from ~45°C to **112°C within minutes**
- The motor then enters thermal protection (mode=0), produces an **uncontrolled torque pulse** that flips the wrist to an extreme angle (~106°), and **electromagnetically locks** in that position
- The motor cannot be rotated by hand even after turning off all controllers (L2+B)
- Power cycling temporarily restores the motor to its correct position, but the fault recurs within minutes

**Request:** Please replace motor 20 (left_wrist_pitch_joint).

### Issue 2: Dex3-1 Hand Motors — 5 Motors Disconnected

5 out of 14 hand motors are completely non-functional. They report temperature=0°C and position=0.000 regardless of commands, indicating a communication/wiring fault rather than a mechanical issue.

**Affected motors:**
- **Left hand thumb**: all 3 motors (motor 0, 1, 2) — DEAD
- **Right hand index finger**: both motors (motor 3, 4) — DEAD
- Remaining 9 motors function normally (temperature 35-47°C, position tracking within 0.1 rad)

**Request:** Please inspect wiring/connectors for the affected fingers, or replace the hand driver boards if necessary.

---

## 中文

Unitree 售后团队您好，

我们的 G1 机器人出现了两个硬件问题：

### 问题一：左手腕俯仰电机（电机20）— 疑似绕组短路

左手腕俯仰电机（motor_state[20]，left_wrist_pitch_joint）出现内部故障，疑似因之前过热（129°C）导致绕组短路。

**症状：**
- 开机后电机位置正确，但**手动无法旋转**；而对称位置的右手腕俯仰电机（电机27）可以正常旋转
- 不运行任何控制软件的情况下，该电机第二温度传感器在几分钟内从约45°C升至**112°C**
- 随后电机进入热保护（mode=0），产生**不可控扭矩脉冲**，手腕翻转到极限角度（约106°），并**电磁锁死**在该位置
- 即使按 L2+B 关闭所有控制器后，电机仍无法手动旋转
- 断电重启后电机暂时恢复正常位置，但几分钟内故障复现

**请求：** 请更换电机20（left_wrist_pitch_joint）。

### 问题二：Dex3-1 灵巧手电机 — 5个电机断联

14个手部电机中有5个完全无法工作。它们报告温度=0°C、位置=0.000，不响应任何指令，判断为通信/接线故障而非机械故障。

**故障电机：**
- **左手拇指**：全部3个电机（电机0、1、2）— 无响应
- **右手食指**：2个电机（电机3、4）— 无响应
- 其余9个电机工作正常（温度35-47°C，位置跟踪误差<0.1rad）

**请求：** 请检查故障手指的接线/连接器，如需要请更换手部驱动板。

---

## Detailed Reports / 详细报告

- Motor 20 full analysis: `todo_docs/motor20_wrist_pitch_fault_report.md`
- Dex3-1 hand diagnosis: `todo_docs/dex3_hand_error.md`
