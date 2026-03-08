#!/bin/bash
#
# ##################################################################################################
# ATTENTION: Zero-shot without any training might lead to arms collide with body! Control C to quit! 
# ##################################################################################################
#
# G1 VLA Inference — GR00T N1.6
# First, on Remote Robot Controller, run:
# (1) L2 + Y and L2 + B 
# (2) L2 + UP --> Joints move to home position slowly
# (3) R1 + X  --> balance controller will be activated and the robot will stand up!
# 
# After usage, turn off the balance controller by pressing: 
# (1) L2 + UP --> Still balance position but no controller
# (2) L2 + B --> Turn off all controllers (NEED TO BE HANGED!)
# 
# Usage:
#   bash run_vla.sh server                          # Terminal 1: GPU inference server
#   bash run_vla.sh client --task "pick up apple"   # Terminal 2: step-by-step (default, safe)
#   bash run_vla.sh client --continuous              # Terminal 2: auto-execute (dangerous!)
#   bash run_vla.sh client --dry-run                 # Terminal 2: inference only, no robot cmd
# 
# Other commands: 
# (1) L2 + Left --> Sit down

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GROOT_DIR="$SCRIPT_DIR/Isaac-GR00T"
GROOT_PYTHON="$GROOT_DIR/.venv/bin/python"

SERVER_PORT=5556
MODEL_PATH="${MODEL_PATH:-nvidia/GR00T-N1.6-G1-PnPAppleToPlate}"

ROBOT_IP="192.168.123.164"
ROBOT_USER="unitree"
CAMERA_PORT=5555
REMOTE_DIR="/home/unitree/camera_server"

if [ ! -f "$GROOT_PYTHON" ]; then
    echo "ERROR: GR00T venv not found at $GROOT_DIR/.venv"
    echo "Run: cd $GROOT_DIR && uv sync --python 3.10"
    exit 1
fi

# ── Camera helper ──
start_camera() {
    echo ""
    echo "--- Camera Setup ---"

    # Check if camera is already streaming
    if "$GROOT_PYTHON" -c "
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
    SSH_CMD="sshpass -e ssh ${SSH_OPTS} ${ROBOT_USER}@${ROBOT_IP}"
    SCP_CMD="sshpass -e scp ${SSH_OPTS}"

    $SSH_CMD "pkill -f robot_camera_server.py 2>/dev/null" || true

    $SSH_CMD "mkdir -p ${REMOTE_DIR}"
    $SCP_CMD "${SCRIPT_DIR}/utils/robot_camera_server.py" \
        "${ROBOT_USER}@${ROBOT_IP}:${REMOTE_DIR}/"

    $SSH_CMD "python3 -c 'import zmq' 2>/dev/null" || {
        echo "Installing pyzmq on robot..."
        $SSH_CMD "pip3 install pyzmq"
    }

    sshpass -e ssh -T ${SSH_OPTS} ${ROBOT_USER}@${ROBOT_IP} \
        "setsid python3 ${REMOTE_DIR}/robot_camera_server.py \
           --device -1 --port ${CAMERA_PORT} \
           >${REMOTE_DIR}/camera_server.log 2>&1 </dev/null &"

    sleep 3

    RUNNING=$($SSH_CMD "pgrep -f robot_camera_server.py" 2>/dev/null || true)
    if [ -n "$RUNNING" ]; then
        echo "Camera started! PID: ${RUNNING}"
        echo "Stream: tcp://${ROBOT_IP}:${CAMERA_PORT}"
    else
        echo "WARNING: Camera failed to start (VLA will use blank frames)"
        $SSH_CMD "tail -5 ${REMOTE_DIR}/camera_server.log 2>/dev/null" || true
    fi

    unset SSHPASS
}

MODE="${1:-help}"
shift 2>/dev/null || true

case "$MODE" in
    server)
        echo "============================================"
        echo "  GR00T N1.6 Policy Server (GPU)"
        echo "============================================"
        echo "  Model: $MODEL_PATH"
        echo "  Port:  $SERVER_PORT"
        echo ""
        cd "$GROOT_DIR"
        exec "$GROOT_PYTHON" gr00t/eval/run_gr00t_server.py \
            --model-path "$MODEL_PATH" \
            --embodiment-tag UNITREE_G1 \
            --host 0.0.0.0 \
            --port "$SERVER_PORT" \
            "$@"
        ;;

    client)
        echo "============================================"
        echo "  G1 VLA Client"
        echo "============================================"
        start_camera
        echo ""
        cd "$SCRIPT_DIR"
        exec "$GROOT_PYTHON" utils/vla_client.py \
            --policy-port "$SERVER_PORT" \
            "$@"
        ;;

    test)
        echo "============================================"
        echo "  Quick test: verify GR00T imports + GPU"
        echo "============================================"
        cd "$GROOT_DIR"
        "$GROOT_PYTHON" -c "
import torch
print(f'PyTorch {torch.__version__}, CUDA: {torch.cuda.is_available()}')
print(f'GPU: {torch.cuda.get_device_name(0)}')
print(f'VRAM: {torch.cuda.get_device_properties(0).total_memory / 1e9:.1f} GB')

import flash_attn
print(f'flash-attn: {flash_attn.__version__}')

from gr00t.policy.gr00t_policy import Gr00tPolicy
print('GR00T Policy: OK')

from gr00t.data.embodiment_tags import EmbodimentTag
print(f'UNITREE_G1 tag: {EmbodimentTag.UNITREE_G1}')

print()
print('All checks passed!')
"
        ;;

    *)
        echo "Usage: bash run_vla.sh <mode> [options]"
        echo ""
        echo "Modes:"
        echo "  server    Start GR00T N1.6 GPU inference server"
        echo "  client    Start G1 robot client (step-by-step approval by default)"
        echo "  test      Quick sanity check (imports + GPU)"
        echo ""
        echo "Client options:"
        echo "  --task TEXT       Language instruction (default: pick up apple)"
        echo "  --continuous      Auto-execute without approval (DANGEROUS!)"
        echo "  --dry-run         Inference only, no commands sent to robot"
        echo "  --action-horizon N  Steps per inference chunk (default: 8)"
        echo ""
        echo "Examples:"
        echo "  bash run_vla.sh server"
        echo "  bash run_vla.sh client --task 'pick up the apple'    # safe step-by-step"
        echo "  bash run_vla.sh client --continuous                  # auto (careful!)"
        echo "  bash run_vla.sh client --dry-run                     # test inference only"
        echo ""
        echo "Step-by-step controls:"
        echo "  Enter = approve & execute    s = skip    c = switch to continuous    q = quit"
        ;;
esac
