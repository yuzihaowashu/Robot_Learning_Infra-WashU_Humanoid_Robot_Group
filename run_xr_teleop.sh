#!/bin/bash
# =============================================================
#  G1 Teleoperation — xr_teleoperate (Unitree Official)
#
#  Uses Unitree's xr_teleoperate framework with CasADi/IPOPT IK,
#  250Hz arm control, and TeleVuer WebRTC for PICO 4U connection.
#
#  Prerequisites:
#    1. conda env 'tv' with pinocchio, casadi, unitree_sdk2_python
#    2. SSL certs configured (see docs)
#    3. PICO 4U on same WiFi as this PC
#
#  Usage:
#    bash run_xr_teleop.sh                         # hand tracking (default)
#    bash run_xr_teleop.sh controller              # controller tracking
#    bash run_xr_teleop.sh record                  # hand tracking + record
#    bash run_xr_teleop.sh controller record       # controller + record
#    bash run_xr_teleop.sh sim                     # simulation mode
#    bash run_xr_teleop.sh sim record              # simulation + record
#    bash run_xr_teleop.sh controller record motion  # + locomotion control
# =============================================================

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XR_DIR="${ROOT_DIR}/xr_teleoperate/teleop"

eval "$(conda shell.bash hook 2>/dev/null)"
conda activate tv

INPUT_MODE="hand"
RECORD=0
SIM=0
MOTION=0
EXTRA_ARGS=""

for arg in "$@"; do
    case "$arg" in
        controller)  INPUT_MODE="controller" ;;
        record)      RECORD=1 ;;
        sim)         SIM=1 ;;
        motion)      MOTION=1 ;;
        *)           EXTRA_ARGS="${EXTRA_ARGS} ${arg}" ;;
    esac
done

CMD_ARGS="--arm=G1_29 --ee=dex3 --input-mode=${INPUT_MODE}"

if [ "$RECORD" -eq 1 ]; then
    CMD_ARGS="${CMD_ARGS} --record"
    CMD_ARGS="${CMD_ARGS} --task-dir=${ROOT_DIR}/xr_recordings"
fi

if [ "$SIM" -eq 1 ]; then
    CMD_ARGS="${CMD_ARGS} --sim"
fi

if [ "$MOTION" -eq 1 ]; then
    CMD_ARGS="${CMD_ARGS} --motion"
fi

echo "=========================================="
echo "  G1 Teleoperation — xr_teleoperate"
echo "=========================================="
echo "  Input mode:  ${INPUT_MODE}"
echo "  Recording:   $([ "$RECORD" -eq 1 ] && echo 'ON' || echo 'OFF')"
echo "  Simulation:  $([ "$SIM" -eq 1 ] && echo 'ON' || echo 'OFF')"
echo "  Motion ctrl: $([ "$MOTION" -eq 1 ] && echo 'ON' || echo 'OFF')"
echo ""
echo "Controls:"
echo "  [r]  Start syncing robot with your movements"
echo "  [s]  Start/stop recording (toggle)"
echo "  [q]  Quit"
echo ""

cd "${XR_DIR}"
python teleop_hand_and_arm.py ${CMD_ARGS} ${EXTRA_ARGS}
