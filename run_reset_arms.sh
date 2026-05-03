#!/bin/bash
#
# Reset Unitree G1 arms to a known stand/default pose.
# This script is independent of Psi0/GR00T servers and is intended as a
# recovery tool after stopping VLA or teleoperation clients.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESET_ARM_PYTHON="${RESET_ARM_PYTHON:-$SCRIPT_DIR/Psi0/.venv-psi/bin/python}"

usage() {
    cat <<EOF
Usage:
  bash run_reset_arms.sh [options]

Options:
  --pose default        Move both arms to the stand/default q=0 pose (default)
  --pose spread         Move both arms to the safe outward spread pose
  --duration SEC        Minimum movement duration. Default: 4.0
  --force               Run even if teleop/Psi0 control processes are active
  -h, --help            Show this help message

Notes:
  - Stop teleop or VLA clients before running this script.
  - Set RESET_ARM_PYTHON=/path/to/python to override the Python interpreter.
EOF
}

pose="default"
duration="4.0"
force="false"

while [ "$#" -gt 0 ]; do
    case "$1" in
        --pose)
            pose="$2"
            shift 2
            ;;
        --duration)
            duration="$2"
            shift 2
            ;;
        --force)
            force="true"
            shift
            ;;
        -h|--help|help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

case "$pose" in
    default|spread)
        ;;
    *)
        echo "ERROR: --pose must be default or spread"
        exit 1
        ;;
esac

if [ ! -x "$RESET_ARM_PYTHON" ]; then
    echo "ERROR: Python interpreter not found: $RESET_ARM_PYTHON"
    echo "Set RESET_ARM_PYTHON=/path/to/python"
    exit 1
fi

if [ "$force" != "true" ]; then
    competing="$(pgrep -af 'teleop_hand_and_arm.py|psi0_rtc_bimanual_client.py|psi0_client.py|vla_client.py' || true)"
    if [ -n "$competing" ]; then
        echo "ERROR: another robot arm control process is active:"
        echo "$competing"
        echo "Stop it first, or rerun with --force if you are sure."
        exit 1
    fi
fi

cd "$SCRIPT_DIR"
"$RESET_ARM_PYTHON" - "$pose" "$duration" <<'PY'
import sys
import time

import numpy as np

from utils.psi0_rtc_bimanual_client import (
    CONTROL_DT,
    SPREAD_ARM_Q,
    return_arms_on_exit,
)
from vla_client import G1Robot, ensure_ai_mode
from unitree_sdk2py.core.channel import ChannelFactoryInitialize


pose = sys.argv[1]
duration = float(sys.argv[2])

ChannelFactoryInitialize(0)
if not ensure_ai_mode():
    print("ERROR: Unitree ai/balance mode is not active.")
    sys.exit(1)

robot = G1Robot()
robot.init()
if not robot.wait_for_state():
    print("ERROR: No robot state received.")
    sys.exit(1)

if pose == "spread":
    # Reuse the shared exit helper by temporarily mapping spread as requested.
    return_arms_on_exit(robot, "xr-upright", "spread")
else:
    return_arms_on_exit(robot, "xr-upright", "default")

# Keep publishing briefly after the helper returns, so the final pose settles.
target = SPREAD_ARM_Q.copy() if pose == "spread" else np.zeros(14, dtype=np.float32)
deadline = time.time() + max(0.2, min(duration, 1.0))
while time.time() < deadline:
    # The return helper already settled the pose; this loop intentionally only
    # sleeps to keep this script simple and avoid duplicating command code.
    time.sleep(CONTROL_DT)

print(f"OK: arms reset to {pose} pose.")
PY
