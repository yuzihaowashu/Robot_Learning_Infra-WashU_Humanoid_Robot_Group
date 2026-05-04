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
ROBOT_PC2_IP = "192.168.123.164"
ROBOT_SUBNET_PREFIX = "192.168.123."
PICO_VR_PORT = 8012
GRADIO_PORT = 7860
TELEIMAGER_CONFIG_PORT = 60000
TELEIMAGER_HEAD_ZMQ_PORT = 55555
TASK_LIST_PATH = os.path.join(ROOT_DIR, "tasks", "task_list.json")
SPLITTER_LIST_PATH = os.path.join(ROOT_DIR, "tasks", "splitter_list.json")

_FALLBACK_TASK_PRESETS = {
    "Shake bottle (default)": {
        "name": "shake_bottle",
        "goal": "Shake the bottle with proper arm.",
        "desc": "Shake the bottle steadily using the selected arm mode.",
        "steps": (
            "step1: reach and grasp the bottle; "
            "step2: lift it slightly; "
            "step3: shake it steadily; "
            "step4: place it back safely."
        ),
    },
    "Place bottle into paper box": {
        "name": "place_bottle_in_paper_box",
        "goal": "Place the bottle into the paper box with proper arm.",
        "desc": (
            "Pick up the bottle and place it accurately inside the paper box "
            "using the selected arm mode."
        ),
        "steps": (
            "step1: reach and grasp the bottle; "
            "step2: lift the bottle; "
            "step3: move it above the paper box; "
            "step4: place it into the box and release."
        ),
    },
}


def _load_task_presets():
    try:
        with open(TASK_LIST_PATH, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        presets = {}
        for item in cfg.get("tasks", []):
            label = item["label"]
            presets[label] = {
                "name": item["name"],
                "goal": item["goal"],
                "desc": item.get("desc", ""),
                "steps": item.get("steps", ""),
                "splitter": item.get("splitter", {}),
            }
        default_label = cfg.get("default") or next(iter(presets))
        if default_label not in presets:
            default_label = next(iter(presets))
        if presets:
            return presets, default_label
    except Exception as exc:
        print(f"[tasks] failed to load {TASK_LIST_PATH}: {exc}")
    return _FALLBACK_TASK_PRESETS, "Shake bottle (default)"


TASK_PRESETS, DEFAULT_TASK_PRESET = _load_task_presets()

_FALLBACK_SPLITTER_PRESETS = {
    "Off": {
        "id": "off",
        "enabled": False,
        "delete_original_on_success": False,
    }
}


def _load_splitter_presets():
    try:
        with open(SPLITTER_LIST_PATH, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        presets = {}
        for item in cfg.get("splitters", []):
            label = item["label"]
            presets[label] = item
        default_label = cfg.get("default") or next(iter(presets))
        if default_label not in presets:
            default_label = next(iter(presets))
        if presets:
            return presets, default_label
    except Exception as exc:
        print(f"[splitters] failed to load {SPLITTER_LIST_PATH}: {exc}")
    return _FALLBACK_SPLITTER_PRESETS, "Off"


SPLITTER_PRESETS, DEFAULT_SPLITTER_PRESET = _load_splitter_presets()

sys.path.insert(0, os.path.join(ROOT_DIR, "xr_teleoperate"))


def _get_local_ips():
    """Return non-loopback IPv4 addresses, preferring live interface data."""
    import socket
    ips = []
    try:
        out = subprocess.check_output(
            ["ip", "-4", "-o", "addr", "show", "scope", "global"],
            text=True,
            timeout=2,
        )
        for line in out.splitlines():
            parts = line.split()
            if "inet" in parts:
                ip = parts[parts.index("inet") + 1].split("/", 1)[0]
                if not ip.startswith("127."):
                    ips.append(ip)
    except Exception:
        pass
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


def _preferred_pico_ip():
    """Choose the IP PICO should use; avoid the robot-only 192.168.123.x subnet."""
    ips = _get_local_ips()
    for ip in ips:
        if not ip.startswith(ROBOT_SUBNET_PREFIX):
            return ip
    return ips[0] if ips else "<PC-WiFi-IP>"


def _pico_vr_url(ip=None):
    ip = ip or _preferred_pico_ip()
    return f"https://{ip}:{PICO_VR_PORT}/?ws=wss://{ip}:{PICO_VR_PORT}"


def _gradio_url(ip=None):
    ip = ip or _preferred_pico_ip()
    return f"http://{ip}:{GRADIO_PORT}"


def _resolve_xr_cert_paths():
    env_cert = os.getenv("XR_TELEOP_CERT")
    env_key = os.getenv("XR_TELEOP_KEY")
    if env_cert and env_key:
        return env_cert, env_key
    user_conf = os.path.join(os.path.expanduser("~"), ".config", "xr_teleoperate")
    user_cert = os.path.join(user_conf, "cert.pem")
    user_key = os.path.join(user_conf, "key.pem")
    if os.path.exists(user_cert) and os.path.exists(user_key):
        return user_cert, user_key
    module_dir = os.path.join(XR_DIR, "televuer")
    return os.path.join(module_dir, "cert.pem"), os.path.join(module_dir, "key.pem")


def _cert_matches_ip(cert_path, ip):
    if not os.path.exists(cert_path) or ip.startswith("<"):
        return False
    try:
        out = subprocess.check_output(
            ["openssl", "x509", "-in", cert_path, "-noout", "-text"],
            text=True,
            stderr=subprocess.DEVNULL,
            timeout=3,
        )
        return f"IP Address:{ip}" in out or f"IP:{ip}" in out
    except Exception:
        return False


def _port_open(host, port, timeout=0.25):
    import socket
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def _existing_service_pids():
    """Return known XR service PIDs except this process."""
    current_pid = os.getpid()
    pids = []
    for pattern in ("gradio_panel.py", "teleop_hand_and_arm.py"):
        try:
            out = subprocess.check_output(["pgrep", "-f", pattern], text=True).strip()
        except subprocess.CalledProcessError:
            continue
        for pid_str in out.splitlines():
            try:
                pid = int(pid_str)
            except ValueError:
                continue
            if pid != current_pid and pid not in pids:
                pids.append(pid)
    return pids


def _confirm_kill_existing_services():
    pids = _existing_service_pids()
    port_busy = _port_open("127.0.0.1", GRADIO_PORT)
    if not pids and not port_busy:
        return

    print("\nExisting XR service detected.")
    if pids:
        print("Known XR service processes:")
        for pid in pids:
            try:
                cmd = subprocess.check_output(
                    ["ps", "-p", str(pid), "-o", "pid=,cmd="],
                    text=True,
                ).strip()
                print(f"  {cmd}")
            except Exception:
                print(f"  {pid}")
    if port_busy:
        print(f"Port {GRADIO_PORT} is already in use.")

    if not sys.stdin.isatty():
        raise SystemExit(
            "Cannot prompt in non-interactive mode. Stop the old service first "
            "or start through run_xr_session.sh."
        )

    reply = input("Kill existing XR service before starting a new one? [y/N] ").strip().lower()
    if reply not in ("y", "yes"):
        raise SystemExit("Aborted. Existing service was left running.")

    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
    time.sleep(2)
    for pid in pids:
        try:
            os.kill(pid, 0)
        except OSError:
            continue
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass
    print("Old XR service processes stopped.\n")


def _ping_ok(ip):
    try:
        result = subprocess.run(
            ["ping", "-c", "1", "-W", "1", ip],
            capture_output=True, text=True, timeout=3,
        )
        return result.returncode == 0
    except Exception:
        return False


def _last_log_lines(n=12):
    if not os.path.exists(TELEOP_LOG):
        return "(no teleop log yet)"
    try:
        with open(TELEOP_LOG, "r", errors="replace") as f:
            lines = f.readlines()[-n:]
        return "".join(lines).strip() or "(teleop log is empty)"
    except Exception as exc:
        return f"(failed to read teleop log: {exc})"


def connection_info_markdown():
    ips = _get_local_ips()
    pico_ip = _preferred_pico_ip()
    cert_path, key_path = _resolve_xr_cert_paths()
    cert_ok = os.path.exists(cert_path) and os.path.exists(key_path)
    cert_ip_ok = cert_ok and _cert_matches_ip(cert_path, pico_ip)
    pc2_ok = _ping_ok(ROBOT_PC2_IP)
    teleimager_config_ok = _port_open(ROBOT_PC2_IP, TELEIMAGER_CONFIG_PORT)
    teleimager_head_ok = _port_open(ROBOT_PC2_IP, TELEIMAGER_HEAD_ZMQ_PORT)
    vr_running = _port_open("127.0.0.1", PICO_VR_PORT)

    ip_text = ", ".join(ips) if ips else "no LAN IP detected"
    if cert_ip_ok:
        cert_text = "OK"
    elif cert_ok:
        cert_text = "PRESENT, but may not include current WiFi IP"
    else:
        cert_text = "MISSING"
    pc2_text = "reachable" if pc2_ok else "unreachable"
    if teleimager_config_ok and teleimager_head_ok:
        teleimager_text = "running"
    elif pc2_ok:
        teleimager_text = "not listening; start teleimager-server on PC2"
    else:
        teleimager_text = "unknown; PC2 unreachable"
    vr_text = "running" if vr_running else "not running yet (expected before Launch Teleop)"

    return (
        f"**PICO VR URL:** `{_pico_vr_url(pico_ip)}`\n\n"
        f"**Gradio URL:** `{_gradio_url(pico_ip)}`\n\n"
        f"- Detected PC IPs: `{ip_text}`\n"
        f"- Recommended PICO IP: `{pico_ip}` (avoid `{ROBOT_SUBNET_PREFIX}x` unless PICO is on that subnet)\n"
        f"- XR HTTPS certificate: **{cert_text}** (`{cert_path}`, `{key_path}`)\n"
        f"- PC2 / teleimager host `{ROBOT_PC2_IP}`: **{pc2_text}**\n"
        f"- PC2 teleimager ports `{TELEIMAGER_CONFIG_PORT}`/`{TELEIMAGER_HEAD_ZMQ_PORT}`: **{teleimager_text}**\n"
        f"- TeleVuer port `{PICO_VR_PORT}`: **{vr_text}**\n\n"
        "Open the PICO URL **after** Launch Teleop starts the TeleVuer server. "
        "If PICO opens the page but cannot enter VR, regenerate/trust the HTTPS certificate for the current WiFi IP."
    )

# ---------------------------------------------------------------------------
# Global state
# ---------------------------------------------------------------------------
_teleop_proc: subprocess.Popen | None = None
_teleop_log_fh = None
_ipc_client = None
_ipc_lock = threading.Lock()
_current_task_name = "shake_bottle"
_current_recordings_dir = RECORDINGS_DIR
_auto_split_lock = threading.Lock()
_auto_split_context = None
_auto_split_baseline: set[str] = set()
_auto_split_attempted: set[str] = set()
_auto_split_messages: list[str] = []


def _resolve_recordings_dir(path):
    if not path or not str(path).strip():
        path = RECORDINGS_DIR
    path = os.path.expanduser(str(path).strip())
    if not os.path.isabs(path):
        path = os.path.abspath(os.path.join(ROOT_DIR, path))
    return path


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


def launch_teleop(task_name, task_goal, task_desc, task_steps, save_dir,
                  input_mode, arm_mode, mirror_vr):
    global _teleop_proc, _teleop_log_fh, _current_task_name, _current_recordings_dir
    _current_task_name = task_name
    _current_recordings_dir = _resolve_recordings_dir(save_dir)
    pico_url = _pico_vr_url()

    if _teleop_proc is not None and _teleop_proc.poll() is None:
        return f"Teleop process is already running.\nPICO VR URL: {pico_url}"

    _kill_stale_teleop()
    _destroy_ipc_client()
    os.makedirs(_current_recordings_dir, exist_ok=True)

    # Gradio: single-arm modes always hold the inactive arm in `relaxed` pose (no UI toggle).
    inactive_arm_pose = (
        "relaxed" if arm_mode in ("left-only", "right-only") else "default"
    )

    cmd = [
        sys.executable, os.path.join(XR_DIR, "teleop_hand_and_arm.py"),
        "--ipc",
        "--arm=G1_29", "--ee=dex3",
        f"--input-mode={input_mode}",
        "--record",
        "--force-zmq-video",
        f"--task-dir={_current_recordings_dir}",
        f"--task-name={task_name}",
        f"--task-goal={task_goal}",
        f"--task-desc={task_desc}",
        f"--task-steps={task_steps}",
        f"--arm-mode={arm_mode}",
        f"--inactive-arm-pose={inactive_arm_pose}",
    ]
    # Always keep Unitree balance / arm_sdk mode active in the Gradio workflow.
    # Disabling this is only for bench/debug CLI use and can make the robot unsafe.
    cmd.append("--motion")
    if mirror_vr:
        cmd.append("--mirror-vr")
    else:
        cmd.append("--no-mirror-vr")
    # Stop inside teleop parks arms at the safe outward pose first; the Gradio
    # Stop button then runs Relax Arms to Default Pose after the process exits.
    cmd.append("--park-arms-on-stop=spread")

    _teleop_log_fh = open(TELEOP_LOG, "w")
    _teleop_proc = subprocess.Popen(
        cmd,
        cwd=XR_DIR,
        stdout=_teleop_log_fh,
        stderr=subprocess.STDOUT,
    )

    for _ in range(40):
        time.sleep(0.25)
        if _teleop_proc.poll() is not None:
            return (
                f"Teleop exited early with code {_teleop_proc.returncode}.\n"
                f"Last log lines:\n{_last_log_lines()}"
            )
        client = _get_ipc_client()
        if client and client.is_online():
            _arm_detail = (
                f"Arm mode: {arm_mode}; non-XR arm relaxed\n"
                if arm_mode in ("left-only", "right-only")
                else f"Arm mode: {arm_mode}\n"
            )
            return (
                f"Teleop launched (PID {_teleop_proc.pid}). IPC connected.\n"
                f"Save dir: {_current_recordings_dir}\n"
                f"{_arm_detail}"
                "Balance mode: always ON\n"
                f"PICO VR URL: {pico_url}"
            )
    _arm_detail = (
        f"Arm mode: {arm_mode}; non-XR arm relaxed\n"
        if arm_mode in ("left-only", "right-only")
        else f"Arm mode: {arm_mode}\n"
    )
    return (
        f"Teleop launched (PID {_teleop_proc.pid}), but IPC heartbeat not yet detected.\n"
        f"Save dir: {_current_recordings_dir}\n"
        f"{_arm_detail}"
        "Balance mode: always ON\n"
        f"PICO VR URL: {pico_url}\n"
        "If PICO cannot enter VR, check the log and refresh Preflight."
    )


def stop_teleop():
    global _teleop_proc, _teleop_log_fh
    messages = []
    client = _get_ipc_client()
    proc = _teleop_proc
    if proc is not None and proc.poll() is None and client and client.is_online():
        try:
            client.send_data("CMD_STOP")
            messages.append("Sent stop command through IPC.")
        except Exception:
            messages.append("IPC stop command failed; falling back to process signal.")
    elif proc is not None and proc.poll() is None:
        messages.append("IPC offline; stopping teleop process directly.")
    else:
        messages.append("No running teleop process found; running relax/recovery only.")

    # Give a healthy teleop process a short window to run its own safe stop.
    for _ in range(12):
        if proc is None or proc.poll() is not None:
            break
        time.sleep(0.5)

    if proc is not None and proc.poll() is None:
        try:
            proc.send_signal(signal.SIGTERM)
            messages.append("Teleop did not exit in time; sent SIGTERM.")
        except Exception:
            pass
        try:
            proc.wait(timeout=4)
        except Exception:
            try:
                proc.kill()
                proc.wait(timeout=2)
                messages.append("Teleop required SIGKILL.")
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
    relax_result = reset_arms()
    messages.append(relax_result)
    return "\n".join(messages)


def reset_arms():
    """Return arms to the factory/default stand pose and pause the holder."""
    if _teleop_proc is not None and _teleop_proc.poll() is None:
        return (
            "[BLOCKED] Teleop is still running. Click Stop Teleop first, "
            "wait until it exits, then use Relax Arms to Default Pose."
        )

    try:
        result = subprocess.run(
            [sys.executable, "-c", "\n".join([
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
                "    current_arm_q = np.array([state.motor_state[i].q for i in arm_ids], dtype=float)",
                "else:",
                "    current_arm_q = None",
                "",
                "# Arms are NOT relaxed. If teleop already parked at outward",
                "# stretch, do not stretch again; just release from there.",
                "from teleop.robot_control.robot_arm import G1_29_ArmController",
                "arm = G1_29_ArmController(motion_mode=True, safe_deploy=False)",
                "already_outer = False",
                "if current_arm_q is not None:",
                "    non_roll_idx = [i for i in range(14) if i not in (1, 8)]",
                "    already_outer = (current_arm_q[1] > 1.25 and current_arm_q[8] < -1.25 and np.max(np.abs(current_arm_q[non_roll_idx])) < 0.40)",
                "    print(f'Already outer stretch: {already_outer}')",
                "",
                "# keep_holder_yield=True prevents arm_idle_holder from pulling",
                "# the arms back to the outward spread pose immediately.",
                "arm.ctrl_dual_arm_go_home(lower_to_zero=True, keep_holder_yield=True, skip_spread=already_outer, clearance_path=False, skip_zero_waypoint=True, spread_min_duration=3.0, spread_timeout=4.5, spread_settle=False)",
                "from teleop.robot_control.robot_hand_unitree import dex3_release_hands",
                "dex3_release_hands(duration=1.0)",
                "print('OK')",
            ])],
            capture_output=True, text=True, timeout=35,
            cwd=XR_DIR,
        )
        if "OK" in result.stdout:
            return "[OK] Arms returned to default stand pose. arm_idle_holder is paused."
        return f"[ERROR] {result.stderr[:200] if result.stderr else 'Unknown error'}"
    except subprocess.TimeoutExpired:
        return "[ERROR] Arm release timed out (35s)."
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


def poll_status_and_episodes(auto_splitter_label):
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
    arm_mode = state.get("ARM_MODE", "—")

    status_html = f"""
    <div style="display:flex; gap:18px; align-items:center; flex-wrap:wrap; padding:6px 0;">
        <span><b>Process:</b> {proc_html}</span>
        <span><b>IPC:</b> {ipc_html}</span>
        <span><b>Tracking:</b> {tracking_html}</span>
        <span><b>Recording:</b> {rec_html}</span>
        <span><b>Locomotion:</b> {loco_html}</span>
        <span><b>Arm Mode:</b> <code>{arm_mode}</code></span>
    </div>"""

    events = state.get("EVENTS", [])
    _maybe_auto_split_latest(auto_splitter_label)

    if events:
        lines = "\n".join(events[-10:])
    elif not proc_alive:
        lines = "(teleop not running)"
    else:
        lines = "(waiting for events...)"
    if _auto_split_messages:
        lines = lines + "\n\n" + "\n".join(_auto_split_messages[-5:])

    episodes = refresh_episodes(_current_task_name, _current_recordings_dir)
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
    if _ping_ok(ip):
        return _badge(f"PC2 ({ip}) REACHABLE", "green")
    return _badge(f"PC2 ({ip}) UNREACHABLE", "red")


# ---------------------------------------------------------------------------
# Episode history
# ---------------------------------------------------------------------------
def load_episodes(task_name, save_dir=None):
    base_dir = _resolve_recordings_dir(save_dir)
    task_dir = os.path.join(base_dir, task_name)
    if not os.path.isdir(task_dir):
        return []
    rows = []
    ep_dirs = glob.glob(os.path.join(task_dir, "episode_*"))
    ep_dirs += glob.glob(os.path.join(task_dir, "raw_episode_*", "episode_*"))
    for ep_dir in sorted(ep_dirs):
        ep_id = os.path.relpath(ep_dir, task_dir)
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


def refresh_episodes(task_name, save_dir=None):
    rows = load_episodes(task_name, save_dir)
    if not rows:
        return [["(no episodes)", "", "", ""]]
    return rows


def _episode_data_paths(task_name, save_dir):
    base_dir = _resolve_recordings_dir(save_dir)
    task_dir = os.path.join(base_dir, task_name)
    if not os.path.isdir(task_dir):
        return []
    return sorted(glob.glob(os.path.join(task_dir, "episode_*", "data.json")))


def _is_split_episode(data_path):
    try:
        with open(data_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        metadata = data.get("info", {}).get("metadata", {})
        return bool(metadata.get("split_from"))
    except Exception:
        return False


def _append_auto_split_message(message):
    _auto_split_messages.append(message)
    del _auto_split_messages[:-8]


def _splitter_raw_task_name(preset, fallback_task_name):
    raw_task = preset.get("raw_task", {}) if isinstance(preset, dict) else {}
    return raw_task.get("name") or fallback_task_name


def _configure_auto_splitter(
    splitter_label,
    task_name,
    task_goal,
    task_desc,
    task_steps,
    save_dir,
):
    global _auto_split_context, _auto_split_baseline, _auto_split_attempted
    preset = SPLITTER_PRESETS.get(splitter_label, SPLITTER_PRESETS["Off"])
    raw_task = preset.get("raw_task", {}) if preset.get("enabled") else {}
    monitor_task_name = _splitter_raw_task_name(preset, task_name)
    next_task_name = raw_task.get("name", task_name)
    next_task_goal = raw_task.get("goal", task_goal)
    next_task_desc = raw_task.get("desc", task_desc)
    next_task_steps = raw_task.get("steps", task_steps)
    with _auto_split_lock:
        _auto_split_context = (
            splitter_label,
            monitor_task_name,
            _resolve_recordings_dir(save_dir),
        )
        _auto_split_baseline = set()
        _auto_split_attempted = set()
    if not preset.get("enabled"):
        status = "Episode Splitter: Off"
        episodes = refresh_episodes(task_name, save_dir)
        return (
            status,
            next_task_name,
            next_task_goal,
            next_task_desc,
            next_task_steps,
            episodes,
        )
    status = (
        f"Episode Splitter: {splitter_label} enabled. Future raw episodes "
        f"will be saved under `{monitor_task_name}`, split into task "
        "episodes, and kept as source data."
    )
    episodes = refresh_episodes(monitor_task_name, save_dir)
    return (
        status,
        next_task_name,
        next_task_goal,
        next_task_desc,
        next_task_steps,
        episodes,
    )


def _maybe_auto_split_latest(splitter_label):
    global _auto_split_context, _auto_split_baseline, _auto_split_attempted
    preset = SPLITTER_PRESETS.get(splitter_label, SPLITTER_PRESETS["Off"])
    if not preset.get("enabled"):
        return

    monitor_task_name = _splitter_raw_task_name(preset, _current_task_name)
    context = (splitter_label, monitor_task_name, _current_recordings_dir)
    with _auto_split_lock:
        if _auto_split_context != context:
            _auto_split_context = context
            _auto_split_baseline = set()
            _auto_split_attempted = set()
            _append_auto_split_message(
                f"[autosplit] armed: {splitter_label} on {monitor_task_name}"
            )

        candidates = []
        for data_path in _episode_data_paths(
            monitor_task_name, _current_recordings_dir
        ):
            if data_path in _auto_split_attempted:
                continue
            if _is_split_episode(data_path):
                _auto_split_baseline.add(data_path)
                continue
            if time.time() - os.path.getmtime(data_path) < 2.0:
                continue
            candidates.append(data_path)
        if not candidates:
            return
        data_path = candidates[-1]
        _auto_split_attempted.add(data_path)

    splitter_id = preset.get("id")
    cmd = [
        sys.executable,
        os.path.join(ROOT_DIR, "utils", "split_xr_episode.py"),
        data_path,
        "--splitter-id",
        splitter_id,
        "--write",
    ]
    try:
        proc = subprocess.run(
            cmd,
            cwd=ROOT_DIR,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=120,
        )
    except Exception as exc:
        _append_auto_split_message(f"[autosplit] failed: {exc}")
        return

    if proc.returncode == 0:
        with _auto_split_lock:
            _auto_split_baseline.add(data_path)
        episode_name = os.path.basename(os.path.dirname(data_path))
        _append_auto_split_message(
            f"[autosplit] split {episode_name}; raw kept"
        )
    else:
        _append_auto_split_message(
            "[autosplit] splitter failed:\n" + proc.stdout[-1200:]
        )


def _update_task_context(name, save_dir):
    global _current_task_name, _current_recordings_dir
    _current_task_name = name
    _current_recordings_dir = _resolve_recordings_dir(save_dir)
    return refresh_episodes(name, _current_recordings_dir)


def _apply_task_preset(preset_name, save_dir):
    preset = TASK_PRESETS.get(preset_name, TASK_PRESETS[DEFAULT_TASK_PRESET])
    task_name = preset["name"]
    task_goal = preset["goal"]
    task_desc = preset["desc"]
    task_steps = preset["steps"]
    episodes = _update_task_context(task_name, save_dir)
    return task_name, task_goal, task_desc, task_steps, episodes


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
            task_preset = gr.Dropdown(
                choices=list(TASK_PRESETS.keys()),
                value=DEFAULT_TASK_PRESET,
                label="Task Preset",
            )
            auto_splitter = gr.Dropdown(
                choices=list(SPLITTER_PRESETS.keys()),
                value=DEFAULT_SPLITTER_PRESET,
                label="Episode Splitter",
            )
            auto_split_status = gr.Textbox(
                label="Episode Splitter Status",
                value="Episode Splitter: Off",
                interactive=False,
                lines=2,
            )
            gr.Markdown(
                "*Choose a predefined task, then edit the fields below if "
                "needed. Episode Splitter is optional and should be enabled "
                "before recording one long continuous episode.*"
            )
            with gr.Row():
                default_preset = TASK_PRESETS[DEFAULT_TASK_PRESET]
                task_name = gr.Textbox(
                    label="Task Name", value=default_preset["name"],
                    placeholder="e.g. shake_bottle", scale=1,
                )
                task_goal = gr.Textbox(
                    label="Task Goal",
                    value=default_preset["goal"],
                    placeholder="Short goal description", scale=2,
                )
            with gr.Row():
                task_desc = gr.Textbox(
                    label="Task Description", value=default_preset["desc"],
                    placeholder="Detailed description (optional)", lines=2, scale=2,
                )
                task_steps = gr.Textbox(
                    label="Task Steps", value=default_preset["steps"],
                    placeholder="step1: ...; step2: ...;", lines=2, scale=2,
                )
            with gr.Row():
                save_dir = gr.Textbox(
                    label="Save Path",
                    value=RECORDINGS_DIR,
                    placeholder="Directory for saved episodes, e.g. /home/.../xr_recordings",
                    scale=3,
                )
            with gr.Row():
                input_mode = gr.Radio(
                    ["controller"], label="Input Mode",
                    value="controller", scale=1,
                )
                arm_mode = gr.Radio(
                    ["bimanual", "left-only", "right-only"],
                    label="Arm Mode",
                    value="bimanual",
                    scale=2,
                )
                mirror_flag = gr.Checkbox(
                    label="Show PC VR Mirror", value=True, scale=1,
                )
            gr.Markdown(
                "*`left-only` / `right-only`: the active arm parks in a "
                "**q=0 forward/default** pose between recordings via an "
                "**outward clearance** waypoint, and the non-teleoperated "
                "arm is held in a **relaxed** pose automatically "
                "(no extra setting).*"
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
                "3. PICO VR is on and connected to the same WiFi as this PC\n\n"
                "**Stop behavior:** `(2) Stop Teleop + Relax Arms` first parks arms in the safe outward pose, "
                "then releases them toward the default/down pose."
            )
            preflight_md = gr.Markdown(value=connection_info_markdown())
            with gr.Row():
                launch_btn = gr.Button(
                    "(1) Launch Teleop", variant="primary",
                    elem_classes=["launch-btn"], scale=2,
                )
                stop_btn = gr.Button("(2) Stop Teleop + Relax Arms", variant="stop", scale=1)
                refresh_btn = gr.Button("Refresh Preflight", scale=1)

            launch_output = gr.Textbox(label="Output", interactive=False, lines=6)

            launch_btn.click(
                fn=launch_teleop,
                inputs=[
                    task_name,
                    task_goal,
                    task_desc,
                    task_steps,
                    save_dir,
                    input_mode,
                    arm_mode,
                    mirror_flag,
                ],
                outputs=launch_output,
            )
            stop_btn.click(fn=stop_teleop, outputs=launch_output)
            refresh_btn.click(fn=connection_info_markdown, outputs=preflight_md)

            with gr.Accordion("Recovery / advanced", open=False):
                gr.Markdown(
                    "Use this if automatic relax after Stop did not complete, or after recovery. "
                    "It pauses arm_idle_holder, so confirm Dex3 hands are clear of the thighs. "
                    "Stop Teleop must finish before this button will run."
                )
                reset_btn = gr.Button("(3) Relax Arms to Default Pose", variant="secondary")
                reset_btn.click(fn=reset_arms, outputs=launch_output)

        # ==================================================================
        # Step 3: Manual backup controls
        # ==================================================================
        with gr.Group():
            gr.Markdown("## Step 3 — Manual Backup / Advanced Controls", elem_classes=["step-title"])
            gr.Markdown(
                f"**PICO VR:** Open the URL shown in Preflight / Launch output, normally `{_pico_vr_url()}` → Enter VR\n\n"
                "**Normal workflow (repeat for each episode):**\n"
                "1. Put on PICO and click **Enter VR**\n"
                "2. Press **VR Left X** -> start/resume tracking and begin a new episode\n"
                "3. Perform the task\n"
                "4. Press **VR Right A** -> stop and save the current episode\n"
                "5. Wait for 'saved' notification, then press **VR Left X** for the next episode\n\n"
                "*The PC buttons below are backups for debugging or if the PICO buttons are unavailable. "
                "The PC VR Mirror window lets audiences observe the operator view. "
                "Balance mode is always ON; walking commands are OFF until Toggle Locomotion is enabled.*"
            )
            with gr.Accordion("Manual backup buttons", open=False):
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
                        value=refresh_episodes("shake_bottle", RECORDINGS_DIR),
                        interactive=False,
                        wrap=True,
                    )
                    refresh_btn = gr.Button("Refresh Episodes")
                    refresh_btn.click(
                        fn=refresh_episodes,
                        inputs=[task_name, save_dir],
                        outputs=episode_table,
                    )
                    task_name.change(
                        fn=_update_task_context,
                        inputs=[task_name, save_dir],
                        outputs=episode_table,
                    )
                    task_preset.change(
                        fn=_apply_task_preset,
                        inputs=[task_preset, save_dir],
                        outputs=[
                            task_name,
                            task_goal,
                            task_desc,
                            task_steps,
                            episode_table,
                        ],
                    )
                    save_dir.change(
                        fn=_update_task_context,
                        inputs=[task_name, save_dir],
                        outputs=episode_table,
                    )
                    auto_splitter.change(
                        fn=_configure_auto_splitter,
                        inputs=[
                            auto_splitter,
                            task_name,
                            task_goal,
                            task_desc,
                            task_steps,
                            save_dir,
                        ],
                        outputs=[
                            auto_split_status,
                            task_name,
                            task_goal,
                            task_desc,
                            task_steps,
                            episode_table,
                        ],
                    )

                with gr.Column(scale=1):
                    gr.Markdown("### Event Log (live)")
                    event_log = gr.Textbox(
                        value="(teleop not running)",
                        interactive=False, lines=10,
                        show_label=False, max_lines=12,
                    )

        # ---- timer: auto-refresh status + episodes + events every 1.5 s ----
        timer = gr.Timer(value=1.5)
        timer.tick(
            fn=poll_status_and_episodes,
            inputs=[auto_splitter],
            outputs=[status_html, episode_table, event_log],
        )

    return demo


def _atexit_cleanup():
    # Ctrl+C / process exit should only clean up background processes. Do not
    # run the UI Stop+Relax path here, because it intentionally closes fingers.
    global _teleop_proc, _teleop_log_fh
    if _teleop_proc is not None and _teleop_proc.poll() is None:
        try:
            _teleop_proc.send_signal(signal.SIGTERM)
            _teleop_proc.wait(timeout=2)
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
    _kill_stale_teleop()


if __name__ == "__main__":
    _confirm_kill_existing_services()
    atexit.register(_atexit_cleanup)
    demo = build_ui()
    demo.launch(server_name="0.0.0.0", server_port=7860, share=False)
