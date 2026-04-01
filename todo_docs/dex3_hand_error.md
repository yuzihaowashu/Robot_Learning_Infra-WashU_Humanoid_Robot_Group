# Dex3-1 Dexterous Hand — Hardware Fault Report

> Date: 2026-04-01
> Robot: Unitree G1 (29-DOF) with Dex3-1 hands
> Location: WashU Humanoid Robot Lab
> Tested by: Yu Zihao
> Test script: `test_both_hands.py` (standalone DDS, single-thread, no teleop framework)

---

## Summary

5 out of 14 hand motors are **completely disconnected** — they report `temperature = 0°C` and `state.q = 0.000` regardless of commands sent. These motors do not respond to any DDS `HandCmd_` messages.

- **Left hand thumb**: all 3 motors dead (motor 0, 1, 2)
- **Right hand index finger**: both motors dead (motor 3, 4)
- **Remaining 9 motors**: fully functional

---

## Affected Motors

### Left Hand (`rt/dex3/left/cmd` / `rt/dex3/left/state`)

| Motor ID | Joint Name | Symptom | Temperature | Status |
|----------|------------|---------|-------------|--------|
| 0 | kLeftHandThumb0 (lateral) | state=0.000 always, no response | 0°C | **DEAD** |
| 1 | kLeftHandThumb1 (flexion) | state=0.000 always, no response | 0°C | **DEAD** |
| 2 | kLeftHandThumb2 (flexion) | state=0.000 always, no response | 0°C | **DEAD** |
| 3 | kLeftHandMiddle0 | Tracks to -1.479 (cmd=-1.57, err=0.09) | 38°C | OK |
| 4 | kLeftHandMiddle1 | Tracks to -1.704 (cmd=-1.74, err=0.04) | 37°C | OK |
| 5 | kLeftHandIndex0 | Tracks to -1.492 (cmd=-1.57, err=0.08) | 37°C | OK |
| 6 | kLeftHandIndex1 | Tracks to -1.689 (cmd=-1.74, err=0.05) | 35°C | OK |

### Right Hand (`rt/dex3/right/cmd` / `rt/dex3/right/state`)

| Motor ID | Joint Name | Symptom | Temperature | Status |
|----------|------------|---------|-------------|--------|
| 0 | kRightHandThumb0 (lateral) | state=-0.027, stable | 41°C | OK |
| 1 | kRightHandThumb1 (flexion) | Tracks to -0.945 (cmd=-1.0, err=0.06) | 46°C | OK (slow return) |
| 2 | kRightHandThumb2 (flexion) | Tracks to -1.650 (cmd=-1.74, err=0.09) | 43°C | OK (slow return) |
| 3 | kRightHandIndex0 | state=0.000 always, no response | 0°C | **DEAD** |
| 4 | kRightHandIndex1 | state=0.000 always, no response | 0°C | **DEAD** |
| 5 | kRightHandMiddle0 | Tracks to 1.484 (cmd=1.57, err=0.09) | 40°C | OK |
| 6 | kRightHandMiddle1 | Tracks to 1.691 (cmd=1.74, err=0.05) | 37°C | OK |

---

## Diagnostic Details

### Test Methodology

1. Standalone Python script (`test_both_hands.py`) running in a single thread
2. DDS communication via `ChannelFactoryInitialize(0, "enp2s0")`
3. Commands sent at 100 Hz with kp=1.0, kd=0.3
4. Each finger tested individually: open (q=0) → close (CLOSE_Q) → open → status
5. Robot was freshly restarted before testing (power cycle)

### Key Observations

**Dead motors (temp=0°C, state=0.000):**
- These 5 motors do not report ANY telemetry — temperature reads 0°C (impossible for a powered motor), and position reads exactly 0.000 even when the physical finger is not at zero
- Commands are sent but produce no physical movement
- The fault persists across robot power cycles
- Conclusion: **communication link broken** (possible wiring, connector, or driver board fault)

**Working motors:**
- All 9 functional motors track close commands within 0.1 rad steady-state error
- Open commands return to near-zero (< 0.06 rad) for most motors
- Right thumb (motors 1,2) returns to open slowly (settles at ~0.27 rad offset), likely due to kp=1.0 being insufficient to fully overcome mechanical friction in the thumb assembly
- Temperature range for working motors: 35–47°C (normal operating range)

### Critical Issue: Left Wrist Pitch (Motor 20) — Permanent Hardware Fault

**Timeline:**
1. During teleop testing, IK solver drove wrist to extreme angle → motor overheated to **129°C** → entered thermal shutdown (mode=0)
2. Software safety added: temperature monitoring + auto-disable at 85°C + URDF joint clipping
3. After robot restart (2026-03-28): motor appeared normal (mode=1, temp=[49,46]) but was **physically unstable** — light touch caused it to snap to **+1.854 rad (+106.2°)** and lock there
4. Neither `rt/arm_sdk` nor `rt/lowcmd` commands can move it — motor reads position but **cannot produce torque**
5. The fault cascades: once motor 20 locks up, other arm joints begin drifting under gravity (no holding torque), suggesting the robot's internal arm controller also fails

**Diagnosis:** The 129°C overheating event likely caused permanent damage to the motor driver or winding. The motor encoder works (reads position), but the drive stage is dead (zero torque output). This is a **hardware replacement** issue.

**Symptoms:**
- `mode=1` (firmware thinks it's OK), `temp=[49,46]` (normal)
- `q=+1.854 rad` stuck, `dq≈0` — completely unresponsive to PD commands (kp=40)
- Physical perturbation causes immediate snap to mechanical limit
- Other arm joints gradually drift once motor 20 is stuck (loss of overall arm control)

---

## DDS Topic Reference

| Topic | Type | Direction |
|-------|------|-----------|
| `rt/dex3/left/cmd` | `HandCmd_` | PC → Left Hand |
| `rt/dex3/right/cmd` | `HandCmd_` | PC → Right Hand |
| `rt/dex3/left/state` | `HandState_` | Left Hand → PC |
| `rt/dex3/right/state` | `HandState_` | Right Hand → PC |

### Motor Layout (DDS message order)

```
Left Hand:  [thumb0, thumb1, thumb2, middle0, middle1, index0, index1]
Right Hand: [thumb0, thumb1, thumb2, index0,  index1,  middle0, middle1]
```

Note: left and right hands have different middle/index ordering in the DDS message.

---

## Recommended Actions

### Priority 1: Left Wrist Pitch Motor (Motor 20) — Arm Motor
1. **Do NOT physically touch/push the left wrist** — motor 20 cannot hold position and will snap to limit
2. **Motor replacement required** — the 129°C thermal event caused permanent drive-stage damage
3. **Contact Unitree support** with this report, specifically mentioning the thermal shutdown history and current zero-torque symptom

### Priority 2: Dex3-1 Hand Motors (5 dead)
4. **Inspect wiring** for left thumb (3 motors) and right index finger (2 motors) — check connectors at the hand PCB and the motor cable harness
5. **Check driver board** — the temp=0°C pattern suggests the motor driver is not communicating, not just that the motor is mechanically stuck
6. **Test with Unitree's official hand test tool** if available, to rule out our software

---

## Current Workaround

The teleoperation system continues to function with reduced capability:
- Left hand: middle finger + index finger (2 fingers, no thumb)
- Right hand: thumb + middle finger (2 fingers, no index)
- Software sends commands to all motors; dead motors simply don't respond
