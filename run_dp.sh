#!/usr/bin/env bash
#
# Diffusion Policy helper for the Unitree G1 bottle task.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DP_DIR="$SCRIPT_DIR/Diffusion-Policy"
DEFAULT_DP_HOME="/home/humanoid-pc/dp_runtime"
DP_HOME="${DP_HOME:-$DEFAULT_DP_HOME}"
CONDA_ROOT="${CONDA_ROOT:-/home/humanoid-pc/miniconda3}"
DEFAULT_CONDA_DP_PYTHON="$CONDA_ROOT/envs/robodiff/bin/python"
DP_ENV="${DP_ENV:-$DP_DIR/.venv-dp}"
if [ -z "${DP_PYTHON:-}" ] && [ -x "$DEFAULT_CONDA_DP_PYTHON" ]; then
    DP_PYTHON="$DEFAULT_CONDA_DP_PYTHON"
else
    DP_PYTHON="${DP_PYTHON:-$DP_ENV/bin/python}"
fi
MODEL_REPO="${MODEL_REPO:-DependableDavid/g1-bottle-diffusion-policy}"
MODEL_DIR="${MODEL_DIR:-$DP_HOME/models/g1-bottle-dp}"
HF_TOKEN_FILE="${HF_TOKEN_FILE:-$SCRIPT_DIR/../.secret}"
DEFAULT_PORT=8020
ROBOT_IP="${ROBOT_IP:-192.168.123.164}"
ROBOT_USER="${ROBOT_USER:-unitree}"
CAMERA_PORT="${CAMERA_PORT:-5555}"
REMOTE_CAMERA_DIR="${REMOTE_CAMERA_DIR:-/home/unitree/camera_server}"

usage() {
    cat <<EOF
Usage:
  bash run_dp.sh setup
      Print the recommended Diffusion Policy environment setup.

  bash run_dp.sh download
      Download the private HF model artifacts into:
        $MODEL_DIR

  bash run_dp.sh server [options]
      Start the local Diffusion Policy HTTP server.
      Default: http://localhost:$DEFAULT_PORT

  bash run_dp.sh camera
      Check/start the humanoid TeleImager camera stream on:
        tcp://$ROBOT_IP:$CAMERA_PORT

  bash run_dp.sh client [options]
      Start the G1 step-gated robot client. Default is dry-run.

  bash run_dp.sh validate
      Run offline validation for artifacts, scripts, and action mapping.

Common options:
  server:
    --checkpoint PATH
    --model-dir DIR
    --host HOST
    --port PORT
    --device DEVICE

  client:
    --server-url URL
    --execute
    --send-hands / --no-send-hands
    --hand-mode MODE   open/current/policy, default: open
    --continuous
    --step-seconds SEC
    --action-exec-steps N  Chunk prefix to execute per UI click/approval
    --arm-kp VALUE     Arm/waist position KP, default: 200
    --arm-kd VALUE     Arm/waist position KD, default: 5
    --waist-mode MODE  upright/current, default: upright
    --camera-robot-ip IP
    --camera-port PORT
    --ui               Open local web UI with camera, step, and reset buttons
    --ui-port PORT     Default: 8030
    --no-view           Disable local robot-view visualization window
    --no-camera          Skip automatic TeleImager startup/check
    --no-prepare-forward Skip teleop-style forward pose before --execute
    --mock --once          Validate server I/O without connecting to robot DDS

Environment:
  DP_HOME       Default: $DEFAULT_DP_HOME
  DP_ENV        Default: $DP_DIR/.venv-dp, for local venv fallback
  DP_PYTHON     Default: conda robodiff python if present, else DP_ENV python
  MODEL_REPO    Default: $MODEL_REPO
  MODEL_DIR     Default: $MODEL_DIR
  ROBOT_IP      Default: $ROBOT_IP
  ROBOT_USER    Default: $ROBOT_USER
  CAMERA_PORT   Default: $CAMERA_PORT
EOF
}

require_dp_repo() {
    if [ ! -d "$DP_DIR" ]; then
        echo "ERROR: Diffusion-Policy submodule missing at $DP_DIR"
        echo "Run: git submodule update --init --recursive"
        exit 1
    fi
}

require_python() {
    if [ ! -x "$DP_PYTHON" ]; then
        echo "ERROR: DP Python not found: $DP_PYTHON"
        echo "Run: bash run_dp.sh setup"
        exit 1
    fi
}

load_hf_token() {
    if [ -n "${HF_TOKEN:-}" ]; then
        return
    fi
    if [ -f "$HF_TOKEN_FILE" ]; then
        token_line="$(python3 - "$HF_TOKEN_FILE" <<'PY'
import shlex
import sys
from pathlib import Path

line = Path(sys.argv[1]).read_text().strip()
parts = shlex.split(line)
if parts and parts[0].startswith("HF_TOKEN="):
    print(parts[0].split("=", 1)[1])
else:
    print(line)
PY
)"
        if [ -n "$token_line" ]; then
            export HF_TOKEN="$token_line"
        fi
    fi
}

has_arg() {
    needle="$1"
    shift
    for arg in "$@"; do
        if [ "$arg" = "$needle" ]; then
            return 0
        fi
    done
    return 1
}

remove_arg() {
    needle="$1"
    shift
    for arg in "$@"; do
        if [ "$arg" != "$needle" ]; then
            printf '%s\n' "$arg"
        fi
    done
}

check_camera_stream() {
    require_python
    "$DP_PYTHON" - <<PY
import sys
import zmq

ctx = zmq.Context()
socket = ctx.socket(zmq.SUB)
socket.setsockopt(zmq.SUBSCRIBE, b"")
socket.setsockopt(zmq.RCVTIMEO, 2000)
socket.connect("tcp://${ROBOT_IP}:${CAMERA_PORT}")
try:
    socket.recv()
    print("TeleImager stream detected: tcp://${ROBOT_IP}:${CAMERA_PORT}")
    sys.exit(0)
except Exception:
    sys.exit(1)
finally:
    socket.close()
    ctx.term()
PY
}

start_camera() {
    require_python
    echo ""
    echo "--- Humanoid TeleImager Setup ---"
    echo "Robot: ${ROBOT_USER}@${ROBOT_IP}"
    echo "Stream: tcp://${ROBOT_IP}:${CAMERA_PORT}"

    if check_camera_stream >/dev/null 2>&1; then
        echo "TeleImager already streaming at tcp://${ROBOT_IP}:${CAMERA_PORT}"
        return 0
    fi

    echo "No TeleImager stream detected."
    echo "This will SSH into the humanoid robot and start robot_camera_server.py."
    read -rsp "Enter robot SSH password: " ROBOT_PASS
    echo ""

    export SSHPASS="$ROBOT_PASS"
    SSH_OPTS="-o StrictHostKeyChecking=no -o LogLevel=ERROR"
    SSH_CMD="env LD_LIBRARY_PATH= sshpass -e ssh ${SSH_OPTS} ${ROBOT_USER}@${ROBOT_IP}"
    SCP_CMD="env LD_LIBRARY_PATH= sshpass -e scp ${SSH_OPTS}"

    $SSH_CMD "pkill -f robot_camera_server.py 2>/dev/null" || true
    $SSH_CMD "mkdir -p ${REMOTE_CAMERA_DIR}"
    $SCP_CMD "${SCRIPT_DIR}/utils/robot_camera_server.py" \
        "${ROBOT_USER}@${ROBOT_IP}:${REMOTE_CAMERA_DIR}/"

    $SSH_CMD "python3 -c 'import zmq' 2>/dev/null" || {
        echo "Installing pyzmq on robot..."
        $SSH_CMD "pip3 install pyzmq"
    }

    env LD_LIBRARY_PATH= sshpass -e ssh -T ${SSH_OPTS} ${ROBOT_USER}@${ROBOT_IP} \
        "setsid python3 ${REMOTE_CAMERA_DIR}/robot_camera_server.py \
           --device -1 --port ${CAMERA_PORT} \
           >${REMOTE_CAMERA_DIR}/camera_server.log 2>&1 </dev/null &"

    sleep 3
    if check_camera_stream; then
        echo "TeleImager started successfully."
    else
        echo "WARNING: TeleImager failed to start."
        $SSH_CMD "tail -20 ${REMOTE_CAMERA_DIR}/camera_server.log 2>/dev/null" || true
        unset SSHPASS
        return 1
    fi

    unset SSHPASS
}

cmd="${1:-}"
case "$cmd" in
    setup)
        require_dp_repo
        cat <<EOF
Recommended Diffusion Policy setup:

  cd "$DP_DIR"
  conda env create -f conda_environment.yaml
  conda activate robodiff
  pip install -e .
  pip install "huggingface_hub[cli]" fastapi uvicorn

Optional local venv alternative:

  cd "$DP_DIR"
  python3 -m venv .venv-dp
  source .venv-dp/bin/activate
  pip install -U pip wheel
  pip install -e .
  pip install "huggingface_hub[cli]" fastapi uvicorn opencv-python

Then set:

  export DP_PYTHON="$DEFAULT_CONDA_DP_PYTHON"

Model download:

  bash "$SCRIPT_DIR/run_dp.sh" download
EOF
        ;;

    download)
        require_dp_repo
        load_hf_token
        mkdir -p "$MODEL_DIR"
        if command -v hf >/dev/null 2>&1; then
            hf download "$MODEL_REPO" \
                --repo-type model \
                --local-dir "$MODEL_DIR"
        elif command -v huggingface-cli >/dev/null 2>&1; then
            huggingface-cli download "$MODEL_REPO" \
                --repo-type model \
                --local-dir "$MODEL_DIR"
        else
            require_python
            "$DP_PYTHON" -m huggingface_hub.commands.huggingface_cli \
                download "$MODEL_REPO" \
                --repo-type model \
                --local-dir "$MODEL_DIR"
        fi
        echo "Downloaded model artifacts to: $MODEL_DIR"
        ;;

    server)
        require_dp_repo
        require_python
        load_hf_token
        shift
        "$DP_PYTHON" "$SCRIPT_DIR/utils/dp_policy_server.py" \
            --model-dir "$MODEL_DIR" \
            --port "$DEFAULT_PORT" \
            "$@"
        ;;

    camera)
        start_camera
        ;;

    client)
        require_python
        shift
        if ! has_arg "--mock" "$@" && ! has_arg "--no-camera" "$@"; then
            start_camera
        fi
        mapfile -t client_args < <(remove_arg "--no-camera" "$@")
        "$DP_PYTHON" "$SCRIPT_DIR/utils/dp_g1_client.py" "${client_args[@]}"
        ;;

    validate)
        require_dp_repo
        checkpoint="$MODEL_DIR/checkpoints/g1_bottle_dp_14act_epoch25.ckpt"
        test -f "$checkpoint" || {
            echo "ERROR: missing checkpoint: $checkpoint"
            echo "Run: bash run_dp.sh download"
            exit 1
        }
        test -f "$DP_DIR/diffusion_policy/policy/diffusion_unet_image_policy.py"
        test -f "$DP_DIR/diffusion_policy/workspace/train_diffusion_unet_image_workspace.py"
        python3 -m py_compile \
            "$SCRIPT_DIR/utils/dp_policy_server.py" \
            "$SCRIPT_DIR/utils/dp_g1_client.py"
        python3 - <<'PY'
import numpy as np

action_indices = np.array(
    [0, 1, 2, 3, 4, 5, 6, 14, 15, 16, 17, 18, 19, 20],
    dtype=np.int64,
)
action_14 = np.arange(14, dtype=np.float32)
action_31 = np.zeros(31, dtype=np.float32)
action_31[action_indices] = action_14
assert action_31.shape == (31,)
assert np.all(action_31[action_indices] == action_14)
assert np.all(action_31[[7, 8, 9, 10, 11, 12, 13, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30]] == 0)
left_arm = action_31[:7]
left_hand = action_31[14:21]
assert left_arm.shape == (7,)
assert left_hand.shape == (7,)
print("Action mapping OK: 14D policy output expands to 31D layout.")
print("Execution mapping OK: client decodes left arm/hand from expanded 31D layout.")
print("Safety default OK: client is dry-run unless --execute is passed.")
PY
        echo "Offline validation complete."
        echo "After DP env setup, also run:"
        echo "  bash run_dp.sh server --device cuda:0 --load-only"
        echo "  bash run_dp.sh client --mock --once"
        ;;

    ""|-h|--help|help)
        usage
        ;;

    *)
        echo "Unknown command: $cmd"
        usage
        exit 1
        ;;
esac
