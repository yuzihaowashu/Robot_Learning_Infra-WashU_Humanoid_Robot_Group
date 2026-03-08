#!/bin/bash
#
# Collect LeRobot-format dataset by replaying taught trajectories.
#
# This replays trajectories from run_teach.sh on the real robot while
# simultaneously recording joint states, hand states, and camera images
# into a LeRobot dataset (Parquet + MP4).
#
# Prerequisites:
#   - Robot standing with balance controller active (R1+X)
#   - Camera server running (auto-started by run_dashboard.sh or run_vla.sh)
#   - Trajectories recorded via: bash run_teach.sh record
#
# Usage:
#   bash run_collect.sh --task "pick up apple" trajectories/traj_*.json
#   bash run_collect.sh --task "wave hello" --push trajectories/traj_001.json
#   bash run_collect.sh --help
#

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Use lerobot conda env
PYTHON="python"
if command -v conda &>/dev/null; then
    eval "$(conda shell.bash hook 2>/dev/null)"
    conda activate lerobot 2>/dev/null || true
fi

# Default repo ID
DEFAULT_REPO="yuzihaowashu/g1_demonstrations"

# Robot / camera config
ROBOT_IP="192.168.123.164"
ROBOT_USER="unitree"
CAMERA_PORT=5555
REMOTE_DIR="/home/unitree/camera_server"

# ── Camera auto-start ──
start_camera() {
    echo ""
    echo "--- Camera Setup ---"

    if $PYTHON -c "
import zmq, sys
ctx = zmq.Context()
s = ctx.socket(zmq.SUB)
s.setsockopt(zmq.SUBSCRIBE, b'')
s.setsockopt(zmq.RCVTIMEO, 2000)
s.connect('tcp://${ROBOT_IP}:${CAMERA_PORT}')
try:
    s.recv()
    print('Camera stream detected')
    sys.exit(0)
except:
    sys.exit(1)
" 2>/dev/null; then
        echo "Camera already running at tcp://${ROBOT_IP}:${CAMERA_PORT}"
        return 0
    fi

    echo "No camera stream detected. Starting camera on robot..."
    read -sp "Enter robot SSH password: " ROBOT_PASS
    echo ""

    export SSHPASS="$ROBOT_PASS"
    SSH_OPTS="-o StrictHostKeyChecking=no -o LogLevel=ERROR"
    # Clear LD_LIBRARY_PATH to avoid conda OpenSSL vs system sshpass mismatch
    SSH_CMD="env LD_LIBRARY_PATH= sshpass -e ssh ${SSH_OPTS} ${ROBOT_USER}@${ROBOT_IP}"
    SCP_CMD="env LD_LIBRARY_PATH= sshpass -e scp ${SSH_OPTS}"

    $SSH_CMD "pkill -f robot_camera_server.py 2>/dev/null" || true
    $SSH_CMD "mkdir -p ${REMOTE_DIR}"
    $SCP_CMD "${SCRIPT_DIR}/utils/robot_camera_server.py" \
        "${ROBOT_USER}@${ROBOT_IP}:${REMOTE_DIR}/"

    $SSH_CMD "python3 -c 'import zmq' 2>/dev/null" || {
        echo "Installing pyzmq on robot..."
        $SSH_CMD "pip3 install pyzmq"
    }

    env LD_LIBRARY_PATH= sshpass -e ssh -T ${SSH_OPTS} ${ROBOT_USER}@${ROBOT_IP} \
        "setsid python3 ${REMOTE_DIR}/robot_camera_server.py \
           --device -1 --port ${CAMERA_PORT} \
           >${REMOTE_DIR}/camera_server.log 2>&1 </dev/null &"

    sleep 3

    RUNNING=$($SSH_CMD "pgrep -f robot_camera_server.py" 2>/dev/null || true)
    if [ -n "$RUNNING" ]; then
        echo "Camera started! PID: ${RUNNING}"
        echo "Stream: tcp://${ROBOT_IP}:${CAMERA_PORT}"
    else
        echo "WARNING: Camera failed to start (will record black frames)"
        $SSH_CMD "tail -5 ${REMOTE_DIR}/camera_server.log 2>/dev/null" || true
    fi

    unset SSHPASS
}

# Parse arguments
TASK=""
PUSH=""
REPO_ID="$DEFAULT_REPO"
OUTPUT_DIR="$SCRIPT_DIR/datasets"
TRAJ_FILES=()

show_help() {
    echo "Usage: bash run_collect.sh [options] <trajectory_files...>"
    echo ""
    echo "Options:"
    echo "  --task TEXT        Task description (required)"
    echo "  --repo-id ID      HuggingFace repo ID (default: $DEFAULT_REPO)"
    echo "  --output-dir DIR   Local output dir (default: ./datasets/)"
    echo "  --push             Push to HuggingFace Hub after collection"
    echo "  --help             Show this help"
    echo ""
    echo "Examples:"
    echo "  # Collect from all trajectories"
    echo "  bash run_collect.sh --task 'pick up apple' trajectories/traj_*.json"
    echo ""
    echo "  # Collect specific trajectories and push to Hub"
    echo "  bash run_collect.sh --task 'wave' --push trajectories/traj_001.json"
    echo ""
    echo "  # Visualize collected dataset"
    echo "  python -m lerobot.scripts.lerobot_dataset_viz \\"
    echo "      --repo-id $DEFAULT_REPO --root ./datasets/$DEFAULT_REPO"
    echo ""
    echo "Workflow:"
    echo "  1. Record trajectories:  bash run_teach.sh record"
    echo "  2. Collect dataset:      bash run_collect.sh --task '...' trajectories/*.json"
    echo "  3. Visualize locally:    (printed after collection)"
    echo "  4. Push to Hub:          add --push flag, or push later"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --task)
            TASK="$2"; shift 2 ;;
        --repo-id)
            REPO_ID="$2"; shift 2 ;;
        --output-dir)
            OUTPUT_DIR="$2"; shift 2 ;;
        --push)
            PUSH="--push"; shift ;;
        --help|-h)
            show_help; exit 0 ;;
        *)
            TRAJ_FILES+=("$1"); shift ;;
    esac
done

if [ -z "$TASK" ]; then
    echo "ERROR: --task is required"
    echo ""
    show_help
    exit 1
fi

if [ ${#TRAJ_FILES[@]} -eq 0 ]; then
    echo "ERROR: No trajectory files specified"
    echo ""
    show_help
    exit 1
fi

echo "============================================"
echo "  G1 Dataset Collection"
echo "============================================"
echo "  Task:    $TASK"
echo "  Repo:    $REPO_ID"
echo "  Output:  $OUTPUT_DIR"
echo "  Files:   ${#TRAJ_FILES[@]} trajectory file(s)"
echo "  Push:    ${PUSH:-no}"
echo ""

cd "$SCRIPT_DIR"
start_camera
echo ""
exec $PYTHON utils/collect_dataset.py \
    --trajectories "${TRAJ_FILES[@]}" \
    --repo-id "$REPO_ID" \
    --task "$TASK" \
    --output-dir "$OUTPUT_DIR" \
    $PUSH
