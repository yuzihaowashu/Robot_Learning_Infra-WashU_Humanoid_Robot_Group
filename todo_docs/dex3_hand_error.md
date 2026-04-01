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

### Additional Issue: Left Wrist Pitch (Motor 20) Overheating

During earlier teleoperation testing (same session), the left wrist pitch arm motor (not a hand motor) overheated to 129°C and entered thermal shutdown (mode=0). This was caused by the IK solver driving the wrist to extreme angles during teleoperation. A software safety mechanism (temperature monitoring + auto-disable at 85°C) has been added to prevent recurrence.

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

1. **Inspect wiring** for left thumb (3 motors) and right index finger (2 motors) — check connectors at the hand PCB and the motor cable harness
2. **Check driver board** — the temp=0°C pattern suggests the motor driver is not communicating, not just that the motor is mechanically stuck
3. **Test with Unitree's official hand test tool** if available, to rule out our software
4. **Contact Unitree support** with this report if wiring inspection does not resolve the issue

---

## Current Workaround

The teleoperation system continues to function with reduced capability:
- Left hand: middle finger + index finger (2 fingers, no thumb)
- Right hand: thumb + middle finger (2 fingers, no index)
- Software sends commands to all motors; dead motors simply don't respond
