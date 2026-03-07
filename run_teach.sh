#!/bin/bash
# =============================================================
#  G1 Drag-and-Teach — Record & Replay arm + hand trajectories
#
#  Usage:
#    bash run_teach.sh                  # record new trajectory
#    bash run_teach.sh record           # record new trajectory
#    bash run_teach.sh replay <file>    # replay a trajectory
#    bash run_teach.sh replay <file> --speed 0.5 --loop 3
#    bash run_teach.sh list             # list saved trajectories
# =============================================================

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UTILS_DIR="${ROOT_DIR}/utils"
TRAJ_DIR="${ROOT_DIR}/trajectories"

eval "$(conda shell.bash hook 2>/dev/null)"
conda activate lerobot

mkdir -p "${TRAJ_DIR}"

MODE="${1:-record}"

case "$MODE" in
    record)
        shift
        echo "=========================================="
        echo "  G1 Drag-and-Teach — RECORD"
        echo "=========================================="
        python "${UTILS_DIR}/teach.py" "$@"
        ;;
    replay)
        shift
        TRAJ_FILE="$1"
        shift
        if [ -z "$TRAJ_FILE" ]; then
            echo "Usage: bash run_teach.sh replay <trajectory.json> [--speed X] [--loop N]"
            echo ""
            echo "Available trajectories:"
            ls -lt "${TRAJ_DIR}"/*.json 2>/dev/null || echo "  (none found)"
            exit 1
        fi
        # Resolve relative paths from trajectories dir
        if [ ! -f "$TRAJ_FILE" ] && [ -f "${TRAJ_DIR}/${TRAJ_FILE}" ]; then
            TRAJ_FILE="${TRAJ_DIR}/${TRAJ_FILE}"
        fi
        echo "=========================================="
        echo "  G1 Trajectory — REPLAY"
        echo "=========================================="
        python "${UTILS_DIR}/replay.py" "$TRAJ_FILE" "$@"
        ;;
    list)
        echo "Saved trajectories:"
        echo ""
        ls -lt "${TRAJ_DIR}"/*.json 2>/dev/null | while read line; do
            file=$(echo "$line" | awk '{print $NF}')
            if [ -f "$file" ]; then
                frames=$(python3 -c "import json; d=json.load(open('$file')); m=d['metadata']; print(f\"{m['n_frames']} frames, {m['duration_s']}s\")" 2>/dev/null)
                basename=$(basename "$file")
                echo "  ${basename}  —  ${frames}"
            fi
        done
        [ "$(ls -A "${TRAJ_DIR}"/*.json 2>/dev/null)" ] || echo "  (no trajectories recorded yet)"
        ;;
    *)
        echo "Usage:"
        echo "  bash run_teach.sh                  # record"
        echo "  bash run_teach.sh record           # record"
        echo "  bash run_teach.sh replay <file>    # replay"
        echo "  bash run_teach.sh list             # list saved"
        ;;
esac
