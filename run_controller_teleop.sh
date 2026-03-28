#!/bin/bash
# Run the controller-based VR teleop (no body tracking needed).
# Uses only VR controller 6DoF poses + hand trigger/grip for hand open/close.
#
# Prerequisites:
#   1. XRoboToolkit PC Service running on this machine
#   2. XRobot app on PICO with controller tracking active
#   3. Redis server running (redis-server)
#   4. conda gmr environment set up
#
# Usage:
#   bash run_controller_teleop.sh              # real VR controllers
#   bash run_controller_teleop.sh mock         # test without VR hardware

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

eval "$(conda shell.bash hook)"
conda activate gmr

ARGS=""
if [[ "$1" == "mock" ]]; then
    ARGS="--mock"
    shift
fi

python "$SCRIPT_DIR/utils/controller_teleop.py" $ARGS "$@"
