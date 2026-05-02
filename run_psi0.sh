#!/bin/bash
#
# Psi0 inference helper for the Unitree G1 workspace.
#
# This script is intentionally conservative. It helps start Psi0 serving /
# RTC inference processes, but it does not directly send actions to the robot
# unless the underlying Psi0 real-world script does so.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PSI0_DIR="$SCRIPT_DIR/Psi0"
PSI0_ENV="$PSI0_DIR/.venv-psi"
PSI0_PYTHON="${PSI0_PYTHON:-$PSI0_ENV/bin/python}"

DEFAULT_TASK="g1/Pick_bottle_and_turn_and_pour_into_cup"
DEFAULT_PORT=8014
DEFAULT_SERVER_PORT=22085

usage() {
    cat <<EOF
Usage:
  bash run_psi0.sh setup
      Print the recommended Psi0 environment setup commands.

  bash run_psi0.sh server RUN_DIR CKPT_STEP [PORT]
      Start the generic Psi0 policy server using Psi0/scripts/deploy/serve_psi0_simple.sh.
      Example:
        bash run_psi0.sh server /path/to/run 100000

  bash run_psi0.sh rtc [--task TASK] [--port PORT]
      Start Psi0 real-world RTC inference.
      Default task: $DEFAULT_TASK
      Default port: $DEFAULT_PORT

  bash run_psi0.sh client --dry-run
      Safety placeholder. This does not control the robot yet.

Notes:
  - Psi0 is a submodule pinned to https://github.com/yuzihaowashu/Psi0.git
  - Set PSI0_PYTHON to override the Python interpreter.
  - Real robot control should be enabled only after verifying Psi0 action format.
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
        export PSI0_SERVE_PORT="$port"
        echo "Starting Psi0 server: run_dir=$run_dir ckpt_step=$ckpt_step port=$port"
        cd "$PSI0_DIR"
        if [ ! -d "$PSI0_ENV" ]; then
            echo "ERROR: $PSI0_ENV does not exist. Run: bash run_psi0.sh setup"
            exit 1
        fi
        # The upstream script currently uses port 22085 internally.
        bash scripts/deploy/serve_psi0_simple.sh "$run_dir" "$ckpt_step"
        ;;

    rtc)
        require_psi0
        require_python
        task="$DEFAULT_TASK"
        port="$DEFAULT_PORT"
        shift
        while [ "$#" -gt 0 ]; do
            case "$1" in
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
        echo "Starting Psi0 RTC inference: task=$task port=$port"
        cd "$PSI0_DIR/real/teleop"
        "$PSI0_PYTHON" ../deploy/psi-inference_rtc.py \
            --port "$port" \
            --task "$task"
        ;;

    client)
        if [ "${2:-}" != "--dry-run" ]; then
            echo "Psi0 robot client is not wired into this workspace yet."
            echo "Use --dry-run until we verify Psi0 action format and G1 command mapping."
            exit 1
        fi
        require_psi0
        echo "Psi0 dry-run placeholder OK."
        echo "Next implementation step: adapt Psi0 actions to the same G1 safety wrapper used by utils/vla_client.py."
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
