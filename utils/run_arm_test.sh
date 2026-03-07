#!/bin/bash
# Quick launcher for G1 arm control test
# Usage: bash utils/run_arm_test.sh [script_name]
#   bash utils/run_arm_test.sh                    # runs test_arm_control.py
#   bash utils/run_arm_test.sh arm_demo.py        # runs arm_demo.py

UTILS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT="${1:-test_arm_control.py}"

eval "$(conda shell.bash hook 2>/dev/null)"
conda activate lerobot

echo "Running: ${SCRIPT}"
echo "Working dir: ${UTILS_DIR}"
echo "---"

python "${UTILS_DIR}/${SCRIPT}"
