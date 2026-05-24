# Drag-and-Teach Web UI

Put the G1 in **ai** balance mode (hand controller: L1+A → L1+UP), then from the repo root:

```bash
bash run_teach_ui.sh
```

Open **http://localhost:7861** (or set `TEACH_PANEL_PORT`). In the UI: **Connect** → **Prepare (forward pose)** → **Record Step N** → **Stop & save step** → **Execute goal**. Do not run `gradio_panel.py` at the same time (shared `rt/arm_sdk`).
