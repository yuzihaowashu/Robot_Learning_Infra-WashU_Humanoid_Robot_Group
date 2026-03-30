#!/usr/bin/env python3
"""
Gradio Control Panel for XR Teleoperate.

Provides task configuration, live teleop control, real-time status monitoring,
and episode history in a web GUI.

Usage:
    conda activate tv
    python gradio_panel.py
"""

import os
import sys
import json
import glob
import signal
import subprocess
import threading
import time

import gradio as gr

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
XR_DIR = os.path.join(ROOT_DIR, "xr_teleoperate", "teleop")
RECORDINGS_DIR = os.path.join(ROOT_DIR, "xr_recordings")

sys.path.insert(0, os.path.join(ROOT_DIR, "xr_teleoperate"))

# ---------------------------------------------------------------------------
# Global state
# ---------------------------------------------------------------------------
_teleop_proc: subprocess.Popen | None = None
_ipc_client = None
_ipc_lock = threading.Lock()


def _get_ipc_client():
    global _ipc_client
    with _ipc_lock:
        if _ipc_client is None:
            try:
                from teleop.utils.ipc import IPC_Client
                _ipc_client = IPC_Client(hb_fps=10.0)
            except Exception:
                return None
        return _ipc_client


def _destroy_ipc_client():
    global _ipc_client
    with _ipc_lock:
        if _ipc_client is not None:
            try:
                _ipc_client.stop()
            except Exception:
                pass
            _ipc_client = None


# ---------------------------------------------------------------------------
# Process management
# ---------------------------------------------------------------------------
def launch_teleop(task_name, task_goal, task_desc, task_steps, input_mode, motion):
    global _teleop_proc
    if _teleop_proc is not None and _teleop_proc.poll() is None:
        return "Teleop process is already running."

    _destroy_ipc_client()

    cmd = [
        sys.executable, os.path.join(XR_DIR, "teleop_hand_and_arm.py"),
        "--ipc",
        "--arm=G1_29", "--ee=dex3",
        f"--input-mode={input_mode}",
        "--record",
        f"--task-dir={RECORDINGS_DIR}",
        f"--task-name={task_name}",
        f"--task-goal={task_goal}",
        f"--task-desc={task_desc}",
        f"--task-steps={task_steps}",
    ]
    if motion:
        cmd.append("--motion")

    _teleop_proc = subprocess.Popen(
        cmd,
        cwd=XR_DIR,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,
    )

    for _ in range(40):
        time.sleep(0.25)
        client = _get_ipc_client()
        if client and client.is_online():
            return f"Teleop launched (PID {_teleop_proc.pid}). IPC connected."
    return f"Teleop launched (PID {_teleop_proc.pid}), but IPC heartbeat not yet detected."


def stop_teleop():
    global _teleop_proc
    client = _get_ipc_client()
    if client and client.is_online():
        try:
            client.send_data("CMD_STOP")
        except Exception:
            pass
    time.sleep(0.5)
    if _teleop_proc is not None and _teleop_proc.poll() is None:
        try:
            os.killpg(os.getpgid(_teleop_proc.pid), signal.SIGTERM)
        except Exception:
            pass
        _teleop_proc.wait(timeout=5)
    _teleop_proc = None
    _destroy_ipc_client()
    return "Teleop stopped."


def send_ipc_cmd(cmd: str):
    client = _get_ipc_client()
    if client is None or not client.is_online():
        return f"Cannot send {cmd}: IPC offline."
    reply = client.send_data(cmd)
    return f"{cmd} -> {reply.get('status', 'unknown')}"


def ipc_start():
    return send_ipc_cmd("CMD_START")


def ipc_record_toggle():
    return send_ipc_cmd("CMD_RECORD_TOGGLE")


def ipc_stop():
    return send_ipc_cmd("CMD_STOP")


def ipc_loco_toggle():
    return send_ipc_cmd("CMD_LOCO_TOGGLE")


# ---------------------------------------------------------------------------
# Status polling
# ---------------------------------------------------------------------------
def poll_status():
    proc_alive = _teleop_proc is not None and _teleop_proc.poll() is None
    client = _get_ipc_client() if proc_alive else None
    online = client.is_online() if client else False
    state = client.latest_state() if (client and online) else {}

    proc_html = _badge("RUNNING", "green") if proc_alive else _badge("STOPPED", "red")
    ipc_html = _badge("CONNECTED", "green") if online else _badge("OFFLINE", "gray")

    if state.get("RECORD_RUNNING"):
        rec_html = _badge("● REC", "red")
    elif state.get("READY"):
        rec_html = _badge("READY", "green")
    else:
        rec_html = _badge("IDLE", "gray")

    tracking_html = _badge("TRACKING", "green") if state.get("START") else _badge("WAITING", "orange")
    loco_html = _badge("WALK ON", "green") if state.get("LOCO_ENABLED") else _badge("WALK OFF", "gray")

    html = f"""
    <div style="display:flex; gap:18px; align-items:center; flex-wrap:wrap; padding:6px 0;">
        <span><b>Process:</b> {proc_html}</span>
        <span><b>IPC:</b> {ipc_html}</span>
        <span><b>Tracking:</b> {tracking_html}</span>
        <span><b>Recording:</b> {rec_html}</span>
        <span><b>Locomotion:</b> {loco_html}</span>
    </div>"""
    return html


def _badge(text, color):
    bg_map = {
        "green": "#16a34a", "red": "#dc2626", "orange": "#ea580c",
        "gray": "#6b7280", "blue": "#2563eb",
    }
    bg = bg_map.get(color, "#6b7280")
    return (
        f'<span style="background:{bg};color:white;padding:2px 10px;'
        f'border-radius:12px;font-size:13px;font-weight:600;">{text}</span>'
    )


# ---------------------------------------------------------------------------
# PC2 teleimager reachability
# ---------------------------------------------------------------------------
def check_pc2(ip="192.168.123.164"):
    try:
        result = subprocess.run(
            ["ping", "-c", "1", "-W", "1", ip],
            capture_output=True, text=True, timeout=3,
        )
        if result.returncode == 0:
            return _badge(f"PC2 ({ip}) REACHABLE", "green")
        return _badge(f"PC2 ({ip}) UNREACHABLE", "red")
    except Exception:
        return _badge(f"PC2 ({ip}) CHECK FAILED", "gray")


# ---------------------------------------------------------------------------
# Episode history
# ---------------------------------------------------------------------------
def load_episodes(task_name):
    task_dir = os.path.join(RECORDINGS_DIR, task_name)
    if not os.path.isdir(task_dir):
        return []
    rows = []
    for ep_dir in sorted(glob.glob(os.path.join(task_dir, "episode_*"))):
        ep_id = os.path.basename(ep_dir)
        data_file = os.path.join(ep_dir, "data.json")
        if not os.path.isfile(data_file):
            rows.append([ep_id, "—", "—", "—"])
            continue
        try:
            with open(data_file, "r") as f:
                d = json.load(f)
            date = d.get("info", {}).get("date", "—")
            frames = len(d.get("data", []))
            goal = d.get("text", {}).get("goal", "—")
            rows.append([ep_id, date, str(frames), goal])
        except Exception:
            rows.append([ep_id, "err", "err", "err"])
    return rows


def refresh_episodes(task_name):
    rows = load_episodes(task_name)
    if not rows:
        return [["(no episodes)", "", "", ""]]
    return rows


# ---------------------------------------------------------------------------
# Gradio UI
# ---------------------------------------------------------------------------
def build_ui():
    with gr.Blocks(
        title="XR Teleoperate Control Panel",
        theme=gr.themes.Soft(),
        css="""
        .emergency-btn {background: #dc2626 !important; color: white !important; font-weight: bold !important; font-size: 16px !important;}
        .launch-btn {background: #16a34a !important; color: white !important; font-weight: bold !important;}
        """,
    ) as demo:
        gr.Markdown("# XR Teleoperate Control Panel")

        status_html = gr.HTML(value=poll_status(), label="Status")
        pc2_html = gr.HTML(value=check_pc2(), label="PC2")

        timer = gr.Timer(value=0.8)
        timer.tick(fn=poll_status, outputs=status_html)

        with gr.Row():
            # ---- Left column: Task configuration ----
            with gr.Column(scale=1):
                gr.Markdown("### Task Configuration")
                task_name = gr.Textbox(label="Task Name", value="pick_apple", placeholder="e.g. pick_apple")
                task_goal = gr.Textbox(label="Task Goal", value="Pick up the apple from the table.", placeholder="Short description of the goal")
                task_desc = gr.Textbox(label="Task Description", value="", placeholder="Detailed description (optional)", lines=2)
                task_steps = gr.Textbox(label="Task Steps", value="", placeholder="step1: ...; step2: ...;", lines=2)
                input_mode = gr.Radio(["controller", "hand"], label="Input Mode", value="controller")
                motion_flag = gr.Checkbox(label="Enable Locomotion (--motion)", value=True)

                with gr.Row():
                    launch_btn = gr.Button("Launch Teleop", variant="primary", elem_classes=["launch-btn"])
                    stop_btn = gr.Button("Stop Teleop", variant="stop")

                launch_output = gr.Textbox(label="Output", interactive=False, lines=2)

                launch_btn.click(
                    fn=launch_teleop,
                    inputs=[task_name, task_goal, task_desc, task_steps, input_mode, motion_flag],
                    outputs=launch_output,
                )
                stop_btn.click(fn=stop_teleop, outputs=launch_output)

            # ---- Right column: Live control + history ----
            with gr.Column(scale=1):
                gr.Markdown("### Live Control")
                with gr.Row():
                    start_btn = gr.Button("Start Tracking (r)", variant="primary")
                    rec_btn = gr.Button("Toggle Recording (s)", variant="secondary")
                with gr.Row():
                    loco_btn = gr.Button("Toggle Locomotion (m)", variant="secondary")
                ctrl_output = gr.Textbox(label="Command Result", interactive=False, lines=1)
                start_btn.click(fn=ipc_start, outputs=ctrl_output)
                rec_btn.click(fn=ipc_record_toggle, outputs=ctrl_output)
                loco_btn.click(fn=ipc_loco_toggle, outputs=ctrl_output)

                estop_btn = gr.Button("EMERGENCY STOP (q)", elem_classes=["emergency-btn"], size="lg")
                estop_btn.click(fn=ipc_stop, outputs=ctrl_output)

                gr.Markdown("### Episode History")
                episode_table = gr.Dataframe(
                    headers=["Episode", "Date", "Frames", "Goal"],
                    value=refresh_episodes("pick_apple"),
                    interactive=False,
                    wrap=True,
                )
                refresh_btn = gr.Button("Refresh Episodes")
                refresh_btn.click(fn=refresh_episodes, inputs=task_name, outputs=episode_table)
                task_name.change(fn=refresh_episodes, inputs=task_name, outputs=episode_table)

        with gr.Accordion("Network", open=False):
            pc2_refresh_btn = gr.Button("Check PC2 Connectivity")
            pc2_refresh_btn.click(fn=check_pc2, outputs=pc2_html)

    return demo


if __name__ == "__main__":
    demo = build_ui()
    demo.launch(server_name="0.0.0.0", server_port=7860, share=False)
