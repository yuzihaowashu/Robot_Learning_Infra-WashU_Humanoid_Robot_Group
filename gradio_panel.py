#!/usr/bin/env python3
"""
Gradio Control Panel for XR Teleoperate.

Provides task configuration, live teleop control, real-time status monitoring,
and episode history in a web GUI.

Usage:
    conda activate tv
    python gradio_panel.py
"""

import atexit
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
TELEOP_LOG = os.path.join(ROOT_DIR, "teleop_latest.log")

sys.path.insert(0, os.path.join(ROOT_DIR, "xr_teleoperate"))


def _get_local_ips():
    """Return list of non-loopback IPv4 addresses."""
    import socket
    ips = []
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith("127."):
                ips.append(ip)
    except Exception:
        pass
    if not ips:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ips.append(s.getsockname()[0])
            s.close()
        except Exception:
            ips.append("<PC-IP>")
    return list(dict.fromkeys(ips))


LOCAL_IPS = _get_local_ips()

# ---------------------------------------------------------------------------
# Global state
# ---------------------------------------------------------------------------
_teleop_proc: subprocess.Popen | None = None
_teleop_log_fh = None
_ipc_client = None
_ipc_lock = threading.Lock()
_current_task_name = "pick_apple"


def _get_ipc_client():
    """Only create IPC client when teleop process is actually running."""
    global _ipc_client
    with _ipc_lock:
        proc_alive = _teleop_proc is not None and _teleop_proc.poll() is None
        if not proc_alive:
            return None
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
def _kill_stale_teleop():
    """Find and kill any leftover teleop_hand_and_arm.py processes."""
    try:
        out = subprocess.check_output(
            ["pgrep", "-f", "teleop_hand_and_arm.py"],
            text=True,
        ).strip()
    except subprocess.CalledProcessError:
        return
    for pid_str in out.splitlines():
        pid = int(pid_str.strip())
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
    time.sleep(1)
    for pid_str in out.splitlines():
        pid = int(pid_str.strip())
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass


_kill_stale_teleop()


def launch_teleop(task_name, task_goal, task_desc, task_steps, input_mode, motion):
    global _teleop_proc, _teleop_log_fh, _current_task_name
    _current_task_name = task_name

    if _teleop_proc is not None and _teleop_proc.poll() is None:
        return "Teleop process is already running."

    _kill_stale_teleop()
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

    _teleop_log_fh = open(TELEOP_LOG, "w")
    _teleop_proc = subprocess.Popen(
        cmd,
        cwd=XR_DIR,
        stdout=_teleop_log_fh,
        stderr=subprocess.STDOUT,
    )

    for _ in range(40):
        time.sleep(0.25)
        client = _get_ipc_client()
        if client and client.is_online():
            return f"Teleop launched (PID {_teleop_proc.pid}). IPC connected."
    return f"Teleop launched (PID {_teleop_proc.pid}), but IPC heartbeat not yet detected."


def stop_teleop():
    global _teleop_proc, _teleop_log_fh
    client = _get_ipc_client()
    if client and client.is_online():
        try:
            client.send_data("CMD_STOP")
        except Exception:
            pass
    # give the process time to run go_home (~6s) before killing
    for _ in range(20):
        time.sleep(0.5)
        if _teleop_proc is None or _teleop_proc.poll() is not None:
            break
    if _teleop_proc is not None and _teleop_proc.poll() is None:
        try:
            _teleop_proc.send_signal(signal.SIGTERM)
        except Exception:
            pass
        try:
            _teleop_proc.wait(timeout=8)
        except Exception:
            try:
                _teleop_proc.kill()
            except Exception:
                pass
    _teleop_proc = None
    if _teleop_log_fh is not None:
        try:
            _teleop_log_fh.close()
        except Exception:
            pass
        _teleop_log_fh = None
    _destroy_ipc_client()
    return "Teleop stopped."


def reset_arms():
    """Spread arms outward, then slowly release. Skip if already relaxed."""
    try:
        result = subprocess.run(
            ["python", "-c", "\n".join([
                "import sys, time, numpy as np",
                f"sys.path.insert(0, '{os.path.join(ROOT_DIR, 'xr_teleoperate')}')",
                f"sys.path.insert(0, '{XR_DIR}')",
                "from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber",
                "from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as hg_LowCmd, LowState_ as hg_LowState",
                "from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_",
                "from unitree_sdk2py.utils.crc import CRC",
                "ChannelFactoryInitialize(0)",
                "",
                "# Quick check: read arm state to see if arms are already relaxed",
                "sub = ChannelSubscriber('rt/lowstate', hg_LowState)",
                "sub.Init()",
                "time.sleep(0.3)",
                "state = sub.Read()",
                "if state is not None:",
                "    arm_ids = [15,16,17,18,19,20,21, 22,23,24,25,26,27,28]",
                "    arm_q = [abs(state.motor_state[i].q) for i in arm_ids]",
                "    if max(arm_q) < 0.15:",
                "        print('Arms already near rest. OK')",
                "        sys.exit(0)",
                "",
                "# Arms are NOT relaxed — do the safe release sequence",
                "from teleop.robot_control.robot_arm import G1_29_ArmController",
                "arm = G1_29_ArmController(motion_mode=True, safe_deploy=False)",
                "",
                "# Use go_home which now does: spread → q=0 → slow ramp down",
                "arm.ctrl_dual_arm_go_home()",
                "print('OK')",
            ])],
            capture_output=True, text=True, timeout=20,
            cwd=XR_DIR,
        )
        if "OK" in result.stdout:
            return "[OK] Arms relaxed safely."
        return f"[ERROR] {result.stderr[:200] if result.stderr else 'Unknown error'}"
    except subprocess.TimeoutExpired:
        return "[ERROR] Arm release timed out (20s)."
    except Exception as e:
        return f"[ERROR] {e}"


_CMD_LABELS = {
    "CMD_START": "Start Tracking",
    "CMD_STOP": "Emergency Stop",
    "CMD_RECORD_TOGGLE": "Toggle Recording",
    "CMD_LOCO_TOGGLE": "Toggle Locomotion",
}


def send_ipc_cmd(cmd: str):
    label = _CMD_LABELS.get(cmd, cmd)
    client = _get_ipc_client()
    if client is None or not client.is_online():
        return f"[FAILED] {label}: IPC offline — is teleop launched (Step 2)?"
    reply = client.send_data(cmd)
    status = reply.get("status", "unknown")
    if status == "ok":
        return f"[OK] {label} — command sent successfully"
    return f"[ERROR] {label}: {reply.get('msg', status)}"


def ipc_start():
    return send_ipc_cmd("CMD_START")


def ipc_record_toggle():
    return send_ipc_cmd("CMD_RECORD_TOGGLE")


def ipc_stop():
    return send_ipc_cmd("CMD_STOP")


def ipc_loco_toggle():
    return send_ipc_cmd("CMD_LOCO_TOGGLE")


# ---------------------------------------------------------------------------
# Status polling (returns status HTML + episode rows + event log)
# ---------------------------------------------------------------------------
_last_seen_events = 0


def poll_status_and_episodes():
    global _last_seen_events
    proc_alive = _teleop_proc is not None and _teleop_proc.poll() is None
    if not proc_alive and _ipc_client is not None:
        _destroy_ipc_client()
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

    status_html = f"""
    <div style="display:flex; gap:18px; align-items:center; flex-wrap:wrap; padding:6px 0;">
        <span><b>Process:</b> {proc_html}</span>
        <span><b>IPC:</b> {ipc_html}</span>
        <span><b>Tracking:</b> {tracking_html}</span>
        <span><b>Recording:</b> {rec_html}</span>
        <span><b>Locomotion:</b> {loco_html}</span>
    </div>"""

    events = state.get("EVENTS", [])
    if events:
        lines = "\n".join(events[-10:])
    elif not proc_alive:
        lines = "(teleop not running)"
    else:
        lines = "(waiting for events...)"

    episodes = refresh_episodes(_current_task_name)
    return status_html, episodes, lines


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


def _update_task_name(name):
    global _current_task_name
    _current_task_name = name
    return refresh_episodes(name)


# ---------------------------------------------------------------------------
# Gradio UI — step-based layout
# ---------------------------------------------------------------------------
_CSS = """
.emergency-btn {
    background: #dc2626 !important;
    color: white !important;
    font-weight: bold !important;
    font-size: 16px !important;
}
.launch-btn {
    background: #16a34a !important;
    color: white !important;
    font-weight: bold !important;
}
.step-title {
    margin: 0 0 4px 0 !important;
    padding: 0 !important;
}
"""


def build_ui():
    with gr.Blocks(
        title="XR Teleoperate Control Panel",
        css=_CSS,
    ) as demo:
        gr.Markdown("# XR Teleoperate Control Panel")

        # ---- live status bar (always visible at top) ----
        status_html = gr.HTML(value="", label="Status")

        # ==================================================================
        # Step 1: Configure Task
        # ==================================================================
        with gr.Group():
            gr.Markdown("## Step 1 — Configure Task", elem_classes=["step-title"])
            with gr.Row():
                task_name = gr.Textbox(
                    label="Task Name", value="pick_apple",
                    placeholder="e.g. pick_apple", scale=1,
                )
                task_goal = gr.Textbox(
                    label="Task Goal",
                    value="Pick up the apple from the table.",
                    placeholder="Short goal description", scale=2,
                )
            with gr.Row():
                task_desc = gr.Textbox(
                    label="Task Description", value="",
                    placeholder="Detailed description (optional)", lines=2, scale=2,
                )
                task_steps = gr.Textbox(
                    label="Task Steps", value="",
                    placeholder="step1: ...; step2: ...;", lines=2, scale=2,
                )
            with gr.Row():
                input_mode = gr.Radio(
                    ["controller", "hand"], label="Input Mode",
                    value="controller", scale=1,
                )
                motion_flag = gr.Checkbox(
                    label="Enable Locomotion (--motion)", value=True, scale=1,
                )

        # ==================================================================
        # Step 2: Launch and Connect
        # ==================================================================
        with gr.Group():
            gr.Markdown("## Step 2 — Launch and Connect", elem_classes=["step-title"])
            gr.Markdown(
                "**Before launching, make sure:**\n"
                "1. Robot G1 is **standing up** (use remote controller)\n"
                "2. PC2 teleimager is running: `ssh unitree@192.168.123.164` → `cd ~/teleimager` → `conda activate teleimager` → `teleimager-server`\n"
                "3. PICO VR is on and connected by wire / to the same WiFi"
            )
            with gr.Row():
                launch_btn = gr.Button(
                    "Launch Teleop", variant="primary",
                    elem_classes=["launch-btn"], scale=2,
                )
                stop_btn = gr.Button("Stop Teleop", variant="stop", scale=1)
                reset_btn = gr.Button("Relax Arms", variant="secondary", scale=1)
                pc2_btn = gr.Button("Check PC2", scale=1)

            launch_output = gr.Textbox(label="Output", interactive=False, lines=2)
            pc2_html = gr.HTML(value="", label="PC2 Status")

            launch_btn.click(
                fn=launch_teleop,
                inputs=[task_name, task_goal, task_desc, task_steps, input_mode, motion_flag],
                outputs=launch_output,
            )
            stop_btn.click(fn=stop_teleop, outputs=launch_output)
            reset_btn.click(fn=reset_arms, outputs=launch_output)
            pc2_btn.click(fn=check_pc2, outputs=pc2_html)

        # ==================================================================
        # Step 3: Control and Record
        # ==================================================================
        with gr.Group():
            gr.Markdown("## Step 3 — Control and Record", elem_classes=["step-title"])
            gr.Markdown(
                "**PICO VR:** Open browser → `https://192.168.0.89:8012` → Enter VR\n\n"
                "**Workflow (repeat for each episode):**\n"
                "1. Click **Start Tracking** → robot arms follow VR controllers\n"
                "2. Click **Toggle Recording** → recording starts (status shows ● REC)\n"
                "3. Perform the task\n"
                "4. Click **Toggle Recording** again → episode saved, arms return home\n"
                "5. Repeat from step 2 for the next episode (no need to re-launch)\n\n"
                "*Locomotion is OFF by default. Click Toggle Locomotion to enable walking.*"
            )
            with gr.Row():
                start_btn = gr.Button("Start Tracking (r)", variant="primary", scale=2)
                rec_btn = gr.Button("Toggle Recording (s)", variant="secondary", scale=2)
                loco_btn = gr.Button("Toggle Locomotion (m)", variant="secondary", scale=2)

            estop_btn = gr.Button(
                "EMERGENCY STOP (q)",
                elem_classes=["emergency-btn"], size="lg",
            )

            ctrl_output = gr.Textbox(label="Command Result", interactive=False, lines=1)

            start_btn.click(fn=ipc_start, outputs=ctrl_output)
            rec_btn.click(fn=ipc_record_toggle, outputs=ctrl_output)
            loco_btn.click(fn=ipc_loco_toggle, outputs=ctrl_output)
            estop_btn.click(fn=ipc_stop, outputs=ctrl_output)

            with gr.Row():
                with gr.Column(scale=2):
                    gr.Markdown("### Episode History (auto-refreshes)")
                    episode_table = gr.Dataframe(
                        headers=["Episode", "Date", "Frames", "Goal"],
                        value=refresh_episodes("pick_apple"),
                        interactive=False,
                        wrap=True,
                    )
                    refresh_btn = gr.Button("Refresh Episodes")
                    refresh_btn.click(fn=refresh_episodes, inputs=task_name, outputs=episode_table)
                    task_name.change(fn=_update_task_name, inputs=task_name, outputs=episode_table)

                with gr.Column(scale=1):
                    gr.Markdown("### Event Log (live)")
                    event_log = gr.Textbox(
                        value="(teleop not running)",
                        interactive=False, lines=10,
                        show_label=False, max_lines=12,
                    )

        # ---- timer: auto-refresh status + episodes + events every 1.5 s ----
        timer = gr.Timer(value=1.5)
        timer.tick(fn=poll_status_and_episodes, outputs=[status_html, episode_table, event_log])

    return demo


def _atexit_cleanup():
    try:
        stop_teleop()
    except Exception:
        pass
    _kill_stale_teleop()


if __name__ == "__main__":
    atexit.register(_atexit_cleanup)
    demo = build_ui()
    demo.launch(server_name="0.0.0.0", server_port=7860, share=False)
