#!/bin/bash
#
# Psi0 inference helper for the Unitree G1 workspace.
#
# This script is intentionally conservative. The local client defaults to
# dry-run and reuses the same G1 safety wrapper style as run_vla.sh.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PSI0_DIR="$SCRIPT_DIR/Psi0"
PSI0_ENV="$PSI0_DIR/.venv-psi"
PSI0_PYTHON="${PSI0_PYTHON:-$PSI0_ENV/bin/python}"
DEFAULT_CLIENT_PYTHON="$SCRIPT_DIR/Isaac-GR00T/.venv/bin/python"
PSI0_CLIENT_PYTHON="${PSI0_CLIENT_PYTHON:-$DEFAULT_CLIENT_PYTHON}"

DEFAULT_TASK="g1/Pick_bottle_and_turn_and_pour_into_cup"
DEFAULT_PORT=8014
DEFAULT_SERVER_PORT=22085
DEFAULT_PSI_HOME="/home/humanoid-pc/psi0_runtime"
DEFAULT_ACTION_EXEC_HORIZON=9

usage() {
    cat <<EOF
Usage:
  bash run_psi0.sh setup
      Print the recommended Psi0 environment setup commands.

  bash run_psi0.sh server RUN_DIR CKPT_STEP [PORT]
      Start the generic Psi0 HTTP policy server.
      Example:
        bash run_psi0.sh server /home/humanoid-pc/psi0_runtime/runs/psi0_baseline_release 0

  bash run_psi0.sh server-rtc RUN_DIR CKPT_STEP [PORT]
      Start Psi0's upstream WebSocket RTC server. Default port: $DEFAULT_PORT
      Example:
        bash run_psi0.sh server-rtc /home/humanoid-pc/psi0_runtime/runs/psi0_baseline_release 0

  bash run_psi0.sh client [options]
      Start the safety-first local G1 client. Default is dry-run.
      Important options:
        --server-url URL       Default: http://localhost:$DEFAULT_SERVER_PORT/act
        --task TEXT            Default: $DEFAULT_TASK
        --execute              Actually send arm commands to the robot
        --send-hands           Also send hand targets; off by default
        --continuous           Skip step-by-step approval after --execute
        --waist-mode MODE      upright (default) fixes waist at [0,0,0];
                               current holds the startup waist pose

  bash run_psi0.sh rtc-bimanual [options]
      Use Psi0's WebSocket RTC timing, but execute only bimanual arms/hands.
      Default is dry-run; pass --execute to move the robot.
      Important options:
        --host HOST            Default: localhost
        --port PORT            Default: $DEFAULT_PORT
        --task TEXT            Default: put the bottle into the paper box
        --execute              Actually send arm commands to the robot
        --send-hands           Also send hand targets; off by default
        --allow-competing-control
                               Do not abort if XR teleop is still running
        --test-arm-nudge       Send a tiny arm command through arm_sdk and exit
        --waist-mode MODE      xr-upright (default), passive, current, or upright
        --continuous           Skip step-by-step approval after --execute
        --step-seconds SEC     Hold each approved step before prompting again
        --exit-pose POSE       default (default), spread, or hold on Ctrl+C/q
        --action-mode MODE     delta (default for this release) or absolute

  bash run_psi0.sh rtc --raw-robot-control [--task TASK] [--port PORT]
      Run Psi0's upstream raw RTC client. Use only for debugging because it
      bypasses this workspace's safety-first client.

Notes:
  - Psi0 is a submodule pinned to https://github.com/yuzihaowashu/Psi0.git
  - Set PSI_HOME to override the runtime/cache directory.
  - Set PSI0_PYTHON to override the Python interpreter.
  - Set PSI0_CLIENT_PYTHON to override the local client interpreter.
  - Set PSI0_ACTION_EXEC_HORIZON to override server action horizon.
EOF
}

require_psi0() {
    if [ ! -d "$PSI0_DIR" ]; then
        echo "ERROR: Psi0 submodule not found at $PSI0_DIR"
        echo "Run: git submodule update --init --recursive"
        exit 1
    fi
}

require_python() {
    if [ ! -x "$PSI0_PYTHON" ]; then
        echo "ERROR: Psi0 Python not found or not executable: $PSI0_PYTHON"
        echo "Run setup first, or set PSI0_PYTHON=/path/to/python"
        exit 1
    fi
}

cmd="${1:-}"
case "$cmd" in
    setup)
        require_psi0
        cat <<EOF
Recommended Psi0 setup:

  cd "$PSI0_DIR"
  uv venv .venv-psi --python 3.10
  source .venv-psi/bin/activate
  GIT_LFS_SKIP_SMUDGE=1 uv sync --all-groups --index-strategy unsafe-best-match --active
  uv pip install flash_attn==2.7.4.post1 --no-build-isolation

Validate:

  source "$PSI0_ENV/bin/activate"
  python -c "import psi; print(psi.__version__)"

For real-world deployment details, read:
  $PSI0_DIR/real/README.md
EOF
        ;;

    server)
        require_psi0
        if [ "$#" -lt 3 ]; then
            usage
            exit 1
        fi
        run_dir="$2"
        ckpt_step="$3"
        port="${4:-$DEFAULT_SERVER_PORT}"
        export PSI_HOME="${PSI_HOME:-$DEFAULT_PSI_HOME}"
        action_exec_horizon="${PSI0_ACTION_EXEC_HORIZON:-$DEFAULT_ACTION_EXEC_HORIZON}"
        printf 'PSI_HOME=%s\n' "$PSI_HOME" > "$PSI0_DIR/.env"
        echo "Starting Psi0 server: run_dir=$run_dir ckpt_step=$ckpt_step port=$port"
        echo "Psi0 runtime: $PSI_HOME"
        echo "Action exec horizon: $action_exec_horizon"
        cd "$PSI0_DIR"
        if [ ! -d "$PSI0_ENV" ]; then
            echo "ERROR: $PSI0_ENV does not exist. Run: bash run_psi0.sh setup"
            exit 1
        fi
        source "$PSI0_ENV/bin/activate"
        uv run --active --group psi --group serve serve_psi0 \
            --host 0.0.0.0 \
            --port "$port" \
            --policy psi0 \
            --run-dir "$run_dir" \
            --ckpt-step "$ckpt_step" \
            --action-exec-horizon "$action_exec_horizon" \
            --rtc
        ;;

    server-rtc)
        require_psi0
        if [ "$#" -lt 3 ]; then
            usage
            exit 1
        fi
        run_dir="$2"
        ckpt_step="$3"
        port="${4:-$DEFAULT_PORT}"
        export PSI_HOME="${PSI_HOME:-$DEFAULT_PSI_HOME}"
        printf 'PSI_HOME=%s\n' "$PSI_HOME" > "$PSI0_DIR/.env"
        echo "Starting Psi0 WebSocket RTC server: run_dir=$run_dir ckpt_step=$ckpt_step port=$port"
        echo "Psi0 runtime: $PSI_HOME"
        cd "$PSI0_DIR"
        if [ ! -d "$PSI0_ENV" ]; then
            echo "ERROR: $PSI0_ENV does not exist. Run: bash run_psi0.sh setup"
            exit 1
        fi
        source "$PSI0_ENV/bin/activate"
        uv run --active --group psi --group serve \
            python src/psi/deploy/psi_serve_rtc-trainingtimertc.py \
            --host 0.0.0.0 \
            --port "$port" \
            --policy psi0 \
            --run-dir "$run_dir" \
            --ckpt-step "$ckpt_step" \
            --rtc
        ;;

    rtc-bimanual)
        require_psi0
        shift
        require_python
        cd "$SCRIPT_DIR"
        "$PSI0_PYTHON" utils/psi0_rtc_bimanual_client.py "$@"
        ;;

    rtc)
        require_psi0
        task="$DEFAULT_TASK"
        port="$DEFAULT_PORT"
        raw_robot_control="false"
        shift
        while [ "$#" -gt 0 ]; do
            case "$1" in
                --raw-robot-control)
                    raw_robot_control="true"
                    shift
                    ;;
                --task)
                    task="$2"
                    shift 2
                    ;;
                --port)
                    port="$2"
                    shift 2
                    ;;
                *)
                    echo "Unknown rtc option: $1"
                    usage
                    exit 1
                    ;;
            esac
        done
        if [ "$raw_robot_control" != "true" ]; then
            echo "Refusing to run Psi0's upstream raw RTC client by default."
            echo "Use: bash run_psi0.sh client"
            echo "If you really need the upstream raw client, pass --raw-robot-control."
            exit 1
        fi
        require_python
        echo "Starting Psi0 RTC inference: task=$task port=$port"
        cd "$PSI0_DIR/real/teleop"
        "$PSI0_PYTHON" ../deploy/psi-inference_rtc.py \
            --port "$port" \
            --task "$task"
        ;;

    client)
        require_psi0
        shift
        if [ ! -x "$PSI0_CLIENT_PYTHON" ]; then
            if command -v python3 >/dev/null 2>&1; then
                PSI0_CLIENT_PYTHON="$(command -v python3)"
            else
                echo "ERROR: Psi0 client Python not found: $PSI0_CLIENT_PYTHON"
                echo "Set PSI0_CLIENT_PYTHON=/path/to/python"
                exit 1
            fi
        fi
        cd "$SCRIPT_DIR"
        "$PSI0_CLIENT_PYTHON" utils/psi0_client.py "$@"
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
