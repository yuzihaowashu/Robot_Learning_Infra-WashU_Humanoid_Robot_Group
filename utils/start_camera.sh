#!/bin/bash
# Deploy and start camera server on the G1 robot.
# Copies the lightweight camera server script to the robot and runs it.
#
# Usage: bash start_camera.sh [device_id]
#   bash start_camera.sh        # auto-detect camera
#   bash start_camera.sh 4      # use /dev/video4

ROBOT_IP="192.168.123.164"
ROBOT_USER="unitree"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEVICE_ID="${1:--1}"
REMOTE_DIR="/home/unitree/camera_server"

echo "=== G1 Camera Server Launcher ==="
echo "Robot: ${ROBOT_USER}@${ROBOT_IP}"
echo ""
read -sp "Enter robot SSH password: " ROBOT_PASS
echo ""

export SSHPASS="$ROBOT_PASS"
SSH_OPTS="-o StrictHostKeyChecking=no -o LogLevel=ERROR"
SSH_CMD="sshpass -e ssh ${SSH_OPTS} ${ROBOT_USER}@${ROBOT_IP}"
SCP_CMD="sshpass -e scp ${SSH_OPTS}"

# Kill any existing camera server
echo "Stopping any existing camera server..."
$SSH_CMD "pkill -f robot_camera_server.py 2>/dev/null" || true

# Create remote directory and copy script
echo "Deploying camera server to robot..."
$SSH_CMD "mkdir -p ${REMOTE_DIR}"
$SCP_CMD "${SCRIPT_DIR}/robot_camera_server.py" "${ROBOT_USER}@${ROBOT_IP}:${REMOTE_DIR}/"

# Check if zmq is available, install if needed
echo "Checking dependencies on robot..."
$SSH_CMD "python3 -c 'import zmq' 2>/dev/null" || {
    echo "Installing pyzmq on robot..."
    $SSH_CMD "pip3 install pyzmq"
}

# Start the camera server in background via nohup
# -T disables PTY so SSH returns immediately
echo "Starting camera server (device=${DEVICE_ID})..."
sshpass -e ssh -T ${SSH_OPTS} ${ROBOT_USER}@${ROBOT_IP} \
    "setsid python3 ${REMOTE_DIR}/robot_camera_server.py \
       --device ${DEVICE_ID} --port 5555 \
       >${REMOTE_DIR}/camera_server.log 2>&1 </dev/null &"

sleep 3

# Verify it's running
RUNNING=$($SSH_CMD "pgrep -f robot_camera_server.py" 2>/dev/null || true)
if [ -n "$RUNNING" ]; then
    echo ""
    echo "Camera server started! PID: ${RUNNING}"
    echo "Stream available at: tcp://${ROBOT_IP}:5555"
    echo ""
    echo "To view logs on robot: cat ${REMOTE_DIR}/camera_server.log"
    echo "To stop: bash utils/stop_camera.sh"
    
    sleep 1
    echo ""
    echo "--- Server log ---"
    $SSH_CMD "head -10 ${REMOTE_DIR}/camera_server.log 2>/dev/null" || true
else
    echo ""
    echo "ERROR: Camera server failed to start!"
    echo "--- Server log ---"
    $SSH_CMD "cat ${REMOTE_DIR}/camera_server.log 2>/dev/null" || true
fi

unset SSHPASS
