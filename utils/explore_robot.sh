#!/bin/bash
# Explore the G1 robot's filesystem to understand what's available
# Usage: bash explore_robot.sh

ROBOT_IP="192.168.123.164"
ROBOT_USER="unitree"

echo "=== G1 Robot Explorer ==="
echo "Connecting to ${ROBOT_USER}@${ROBOT_IP}"
echo ""
read -sp "Enter robot SSH password: " ROBOT_PASS
echo ""

CMD="sshpass -p '${ROBOT_PASS}' ssh -o StrictHostKeyChecking=no ${ROBOT_USER}@${ROBOT_IP}"

echo "--- System info ---"
eval $CMD "uname -a; echo '==='; cat /etc/os-release | head -5"
echo ""

echo "--- Python version ---"
eval $CMD "python3 --version 2>&1; which python3"
echo ""

echo "--- Camera devices ---"
eval $CMD "ls -la /dev/video* 2>/dev/null; echo '==='; v4l2-ctl --list-devices 2>/dev/null || echo 'v4l2-ctl not found'"
echo ""

echo "--- Home directory ---"
eval $CMD "ls -la /home/unitree/ 2>/dev/null | head -30"
echo ""

echo "--- Pip packages (unitree/lerobot related) ---"
eval $CMD "pip3 list 2>/dev/null | grep -iE 'unitree|lerobot|opencv|zmq|cyclone|numpy|torch'"
echo ""

echo "--- Running processes (robot related) ---"
eval $CMD "ps aux | grep -iE 'unitree|robot|camera|video|dds|zmq' | grep -v grep | head -20"
echo ""

echo "--- Network ports listening ---"
eval $CMD "ss -tlnp 2>/dev/null | head -20"
echo ""

echo "--- Check for lerobot / image_server ---"
eval $CMD "find /home/unitree -name 'image_server*' -o -name 'lerobot*' 2>/dev/null | head -20"
echo ""

echo "--- Check unitree SDK ---"
eval $CMD "ls /home/unitree/unitree* 2>/dev/null; ls /home/unitree/sdk* 2>/dev/null; ls /opt/unitree* 2>/dev/null"
echo ""

echo "Done!"
