#!/bin/bash
# =============================================================
#  G1 Teleoperation Bridge — TWIST2 (PICO VR) → Upper Body
#
#  Prerequisites:
#    1. Redis server running (sudo systemctl start redis-server)
#    2. TWIST2 teleop pipeline active (in gmr conda env):
#         cd /path/to/TWIST2 && bash teleop.sh
#
#  Usage:
#    bash run_teleop.sh                   # live teleop (no recording)
#    bash run_teleop.sh record            # teleop + record trajectory
#    bash run_teleop.sh mock              # test without hardware
#    bash run_teleop.sh mock record       # test mock + record
# =============================================================

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UTILS_DIR="${ROOT_DIR}/utils"
TRAJ_DIR="${ROOT_DIR}/trajectories"

eval "$(conda shell.bash hook 2>/dev/null)"
conda activate lerobot

mkdir -p "${TRAJ_DIR}"

EXTRA_ARGS=""
MOCK=0
RECORD=0

for arg in "$@"; do
    case "$arg" in
        mock)    MOCK=1 ;;
        record)  RECORD=1 ;;
        *)       EXTRA_ARGS="${EXTRA_ARGS} ${arg}" ;;
    esac
done

if [ "$MOCK" -eq 1 ]; then
    EXTRA_ARGS="${EXTRA_ARGS} --mock"
fi

if [ "$RECORD" -eq 1 ]; then
    EXTRA_ARGS="${EXTRA_ARGS} --record"
fi

echo "=========================================="
echo "  G1 Teleoperation Bridge"
echo "=========================================="
if [ "$MOCK" -eq 1 ]; then
    echo "  Mode: MOCK (no hardware needed)"
else
    echo "  Mode: LIVE (Redis + Robot)"
fi
if [ "$RECORD" -eq 1 ]; then
    echo "  Recording: ON"
else
    echo "  Recording: OFF"
fi
echo ""

python "${UTILS_DIR}/teleop_bridge.py" ${EXTRA_ARGS}
