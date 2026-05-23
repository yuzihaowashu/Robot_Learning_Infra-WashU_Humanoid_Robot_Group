#!/bin/bash
# Launch Gradio UI for drag-and-teach (named record + replay).
#
#   bash run_teach_ui.sh
#   TEACH_PANEL_PORT=7862 bash run_teach_ui.sh

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${ROOT_DIR}"

eval "$(conda shell.bash hook 2>/dev/null)"
conda activate lerobot
if ! python -c "import gradio" 2>/dev/null; then
    echo "gradio not found in lerobot — using tv env (has gradio + robot deps)"
    conda activate tv
fi

mkdir -p "${ROOT_DIR}/trajectories" "${ROOT_DIR}/tasks"

echo "=========================================="
echo "  G1 Drag-and-Teach — Web UI"
echo "=========================================="
echo "  Open: http://localhost:${TEACH_PANEL_PORT:-7861}"
echo ""

python "${ROOT_DIR}/teach_panel.py"
