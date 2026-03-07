#!/bin/bash
# Stop the camera server running on the G1 robot.

ROBOT_IP="192.168.123.164"
ROBOT_USER="unitree"

echo "=== Stop G1 Camera Server ==="
read -sp "Enter robot SSH password: " ROBOT_PASS
echo ""

export SSHPASS="$ROBOT_PASS"
SSH_CMD="sshpass -e ssh -o StrictHostKeyChecking=no ${ROBOT_USER}@${ROBOT_IP}"

$SSH_CMD "pkill -f robot_camera_server.py 2>/dev/null" && \
    echo "Camera server stopped." || \
    echo "No camera server was running."

unset SSHPASS
