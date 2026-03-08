# Tactile Sensors — Dex3-1 Hand Pressure Sensing

The Unitree Dex3-1 dexterous hands have built-in pressure sensors that detect
contact force across the finger surfaces.

## Hardware

- **Sensor type**: Capacitive/Piezoresistive pressure sensors
- **Modules per hand**: Up to 9 sensor modules
- **Pads per module**: 12 sensing points
- **Total per hand**: Up to 108 pressure measurement points
- **Force range**: ~10g to ~2500g
- **Output**: Raw ADC counts (not calibrated to physical units)

## Raw Values

| Value Range | Meaning |
|-------------|---------|
| ~30,000 | Baseline (no contact) |
| 30,000–50,000 | Light touch |
| 50,000–80,000 | Moderate pressure |
| 80,000–120,000+ | Strong press |
| 0 or exactly 0 | Disconnected/unused pad |

The raw values are ADC (Analog-to-Digital Converter) counts from the sensor's
analog frontend. They are not calibrated to Newtons or grams — the mapping
depends on the specific sensor characteristics and is approximately linear
in the active range.

## DDS Interface

### Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `rt/dex3/left/state` | `HandState_` | Robot → PC |
| `rt/dex3/right/state` | `HandState_` | Robot → PC |

### Message Structure

```python
HandState_:
    motor_state: list[MotorState_]        # 7 finger motors
    press_sensor_state: list[PressSensorState_]  # 7-9 sensor modules
    imu_state: IMUState_                  # Hand IMU

PressSensorState_:
    pressure: float32[12]     # 12 pressure readings per module
    temperature: float32[12]  # 12 temperature readings per module
    lost: uint32              # Communication loss counter
    reserve: uint32           # Reserved
```

## Reading Tactile Data

### Quick Test

```bash
conda activate lerobot
python utils/test_tactile.py
```

This subscribes to both hands and prints:
- Number of connected sensor modules
- Active pad count per module (above baseline threshold)
- Peak pressure values with ASCII bar visualization

### In Python

```python
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_

ChannelFactoryInitialize(0)

def on_hand_state(msg: HandState_):
    for i, ps in enumerate(msg.press_sensor_state):
        pressures = list(ps.pressure)
        # Filter: values near 0 = disconnected pad
        active = [p for p in pressures if abs(p) > 0.01]
        if active:
            above_baseline = [p - 30000.0 for p in active]
            print(f"  Module {i}: {len(active)} pads, "
                  f"max pressure: {max(active):.0f}")

sub = ChannelSubscriber("rt/dex3/left/state", HandState_)
sub.Init(on_hand_state, 10)
```

## Dashboard Visualization

The dashboard (`run_dashboard.sh`) includes tactile visualization:

- **Bar chart** per active sensor module
- **Color coding**: cyan → green → yellow → red (pressure intensity)
- **White dots** within bars show count of active touch points
- **Percentage** shows normalized pressure (0% = baseline, 100% = max)
- Disconnected modules are automatically hidden

### Normalization

```
normalized = (raw_pressure - BASELINE) / (MAX - BASELINE)
BASELINE = 30,000
MAX = 120,000
```

## Physical Layout

Not all 12 pads per module are physically connected. Typical active pad
counts per module vary from 4 to 12 depending on the finger surface area.
The `test_tactile.py` script helps identify which modules and pads are
physically present on your specific hand unit.

## Sensor Module Mapping

The exact mapping of module index → finger location is not officially
documented by Unitree. Empirical testing (pressing individual fingers
while monitoring) reveals the approximate mapping:

| Module Index | Approximate Location |
|-------------|---------------------|
| 0–1 | Index finger (proximal, distal) |
| 2–3 | Middle finger (proximal, distal) |
| 4–5 | Thumb (proximal, distal) |
| 6–8 | Palm / finger tips |

*Note: This may vary between hand units. Use `test_tactile.py` to verify.*

## Implementation Files

- `utils/test_tactile.py` — Raw tactile sensor debugging
- `utils/dashboard.py` — Tactile visualization (integrated in dashboard)
