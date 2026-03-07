#!/bin/bash
# =============================================================
#  G1 Robot Dashboard — One-click launcher
#  Deploys camera server to robot + launches the dashboard GUI.
#
#  Usage:
#    bash run_dashboard.sh              # auto-detect camera
#    bash run_dashboard.sh --no-camera  # skip camera, dashboard only
#    bash run_dashboard.sh --device 4   # use /dev/video4
# =============================================================

set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UTILS_DIR="${ROOT_DIR}/utils"

ROBOT_IP="192.168.123.164"
ROBOT_USER="unitree"
REMOTE_DIR="/home/unitree/camera_server"
CAMERA_PORT=5555

SKIP_CAMERA=false
DEVICE_ID="-1"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-camera) SKIP_CAMERA=true; shift ;;
        --device)    DEVICE_ID="$2"; shift 2 ;;
        *)           shift ;;
    esac
done

# Activate conda env
eval "$(conda shell.bash hook 2>/dev/null)"
conda activate lerobot

echo "=========================================="
echo "  G1 Robot Dashboard"
echo "=========================================="

# --- Camera setup ---
if [ "$SKIP_CAMERA" = false ]; then
    echo ""
    echo "[Camera] Connecting to ${ROBOT_USER}@${ROBOT_IP}"
    read -sp "Enter robot SSH password: " ROBOT_PASS
    echo ""

    export SSHPASS="$ROBOT_PASS"
    SSH_OPTS="-o StrictHostKeyChecking=no -o LogLevel=ERROR"
    SSH="sshpass -e ssh ${SSH_OPTS} ${ROBOT_USER}@${ROBOT_IP}"
    SCP="sshpass -e scp ${SSH_OPTS}"

    # Check if camera server is already running
    EXISTING=$($SSH "pgrep -f robot_camera_server.py" 2>/dev/null || true)
    if [ -n "$EXISTING" ]; then
        echo "[Camera] Server already running (PID: ${EXISTING}), restarting..."
        $SSH "pkill -f robot_camera_server.py 2>/dev/null" || true
        sleep 1
    fi

    # Deploy and start
    echo "[Camera] Deploying camera server..."
    $SSH "mkdir -p ${REMOTE_DIR}"
    $SCP "${UTILS_DIR}/robot_camera_server.py" \
         "${ROBOT_USER}@${ROBOT_IP}:${REMOTE_DIR}/"

    $SSH "python3 -c 'import zmq' 2>/dev/null" || {
        echo "[Camera] Installing pyzmq on robot..."
        $SSH "pip3 install pyzmq"
    }

    echo "[Camera] Starting (device=${DEVICE_ID})..."
    sshpass -e ssh -T ${SSH_OPTS} ${ROBOT_USER}@${ROBOT_IP} \
        "setsid python3 ${REMOTE_DIR}/robot_camera_server.py \
           --device ${DEVICE_ID} --port ${CAMERA_PORT} \
           >${REMOTE_DIR}/camera_server.log 2>&1 </dev/null &"
    sleep 3

    RUNNING=$($SSH "pgrep -f robot_camera_server.py" 2>/dev/null || true)
    if [ -n "$RUNNING" ]; then
        echo "[Camera] Started! PID: ${RUNNING}  →  tcp://${ROBOT_IP}:${CAMERA_PORT}"
        echo ""
        $SSH "head -5 ${REMOTE_DIR}/camera_server.log 2>/dev/null" || true
    else
        echo "[Camera] WARNING: Failed to start. Check robot logs."
        echo ""
        $SSH "cat ${REMOTE_DIR}/camera_server.log 2>/dev/null" || true
        echo ""
        echo "Continuing without camera..."
    fi

    unset SSHPASS
    echo ""
else
    echo ""
    echo "[Camera] Skipped (--no-camera)"
    echo ""
fi

# --- Launch dashboard ---
echo "[Dashboard] Launching..."
python "${UTILS_DIR}/dashboard.py" "$@"

# --- Cleanup: offer to stop camera on exit ---
if [ "$SKIP_CAMERA" = false ] && [ -n "$RUNNING" ]; then
    echo ""
    read -p "Stop camera server on robot? [y/N] " STOP_CAM
    if [[ "$STOP_CAM" =~ ^[Yy]$ ]]; then
        read -sp "Robot SSH password: " ROBOT_PASS
        echo ""
        export SSHPASS="$ROBOT_PASS"
        SSH="sshpass -e ssh -o StrictHostKeyChecking=no ${ROBOT_USER}@${ROBOT_IP}"
        $SSH "pkill -f robot_camera_server.py 2>/dev/null" && \
            echo "Camera server stopped." || echo "Already stopped."
        unset SSHPASS
    else
        echo "Camera server left running on robot."
    fi
fi
