"""
G1 Robot Dashboard — Real-time joint visualization + arm action control.

Features:
  - Real-time joint angle bars for all 14 arm joints (with URDF limits)
  - Camera feed from robot via ZMQ (run start_camera.sh first)
  - Action buttons to trigger arm motions
  - IMU orientation display
  - Emergency stop

Usage:
    conda activate lerobot
    bash start_camera.sh          # first terminal: start camera on robot
    python dashboard.py           # second terminal: launch dashboard
"""

import base64
import json
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk

import cv2
import numpy as np
from PIL import Image, ImageTk

try:
    import zmq
    HAS_ZMQ = True
except ImportError:
    HAS_ZMQ = False
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (
    LowCmd_, LowState_, HandState_,
)
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

PI = np.pi

# ---------------------------------------------------------------------------
# Joint definitions
# ---------------------------------------------------------------------------

JOINT_INFO = {
    15: ("L ShoulderPitch", -2.78, 2.40),
    16: ("L ShoulderRoll",  -1.43, 2.03),
    17: ("L ShoulderYaw",   -2.36, 2.36),
    18: ("L Elbow",         -0.94, 1.88),
    19: ("L WristRoll",     -1.77, 1.77),
    20: ("L WristPitch",    -1.45, 1.45),
    21: ("L WristYaw",      -1.45, 1.45),
    22: ("R ShoulderPitch", -2.78, 2.40),
    23: ("R ShoulderRoll",  -2.03, 1.43),
    24: ("R ShoulderYaw",   -2.36, 2.36),
    25: ("R Elbow",         -0.94, 1.88),
    26: ("R WristRoll",     -1.77, 1.77),
    27: ("R WristPitch",    -1.45, 1.45),
    28: ("R WristYaw",      -1.45, 1.45),
}

ALL_ARM_JOINTS = list(JOINT_INFO.keys())

# ---------------------------------------------------------------------------
# Predefined actions (keyframe sequences)
# ---------------------------------------------------------------------------

# Safe home: elbows bent + shoulders outward so arms never pass through legs/body
SAFE_HOME = {j: 0.0 for j in ALL_ARM_JOINTS}
SAFE_HOME.update({
    15: 0.0,  16: 0.5,  18: 1.0,   # L: shoulder outward, elbow bent 57 deg
    22: 0.0,  23: -0.5, 25: 1.0,   # R: shoulder outward, elbow bent 57 deg
})

ACTIONS = {
    "Home": [("home", 3.0, dict(SAFE_HOME))],

    "Arms Forward": [
        ("forward", 3.0, {
            **SAFE_HOME,
            15: -0.8, 16: 0.6, 18: 0.5,
            22: -0.8, 23: -0.6, 25: 0.5,
        }),
    ],

    "Arms Spread": [
        ("spread", 3.0, {
            **SAFE_HOME,
            15: -0.3, 16: 1.3, 18: 0.8,
            22: -0.3, 23: -1.3, 25: 0.8,
        }),
    ],

    "Right Arm Up": [
        ("right up", 3.0, {
            **SAFE_HOME,
            22: -1.2, 23: -0.5, 25: 1.2,
        }),
    ],

    "Wave Right": [
        ("right up", 2.0, {
            **SAFE_HOME,
            22: -1.2, 23: -0.5, 25: 1.2,
        }),
        ("wave", 3.0, None),
    ],

    "Both Arms Up": [
        ("both up", 3.0, {
            **SAFE_HOME,
            15: -1.5, 16: 0.5, 18: 1.0,
            22: -1.5, 23: -0.5, 25: 1.0,
        }),
    ],

    "Flex": [
        ("flex", 3.0, {
            **SAFE_HOME,
            15: -0.4, 16: 1.0, 18: 1.8, 19: 0.5,
            22: -0.4, 23: -1.0, 25: 1.8, 26: -0.5,
        }),
    ],
}

ROBOT_IP = "192.168.123.164"
CAMERA_ZMQ_PORT = 5555

TOPIC_LEFT_HAND_STATE = "rt/dex3/left/state"
TOPIC_RIGHT_HAND_STATE = "rt/dex3/right/state"
N_HAND_SENSORS = 9
N_PRESS_PER_SENSOR = 12

PRESS_BASELINE = 30000.0
PRESS_MAX = 120000.0

HAND_SENSOR_NAMES = [
    "S0", "S1", "S2", "S3", "S4", "S5", "S6", "S7", "S8",
]

KP = 30.0
KD = 1.5
CONTROL_DT = 0.02
RAMP_DURATION = 2.0


# ---------------------------------------------------------------------------
# ZMQ Camera Receiver
# ---------------------------------------------------------------------------

class CameraReceiver:
    """Background thread that receives JPEG frames from robot_camera_server.py."""

    def __init__(self, robot_ip=ROBOT_IP, port=CAMERA_ZMQ_PORT):
        self.endpoint = f"tcp://{robot_ip}:{port}"
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.connected = False
        self._stop = False
        self._thread = None

    def start(self):
        if not HAS_ZMQ:
            return
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop = True

    def get_frame(self):
        with self.frame_lock:
            return self.latest_frame

    def _recv_loop(self):
        ctx = zmq.Context()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.SUBSCRIBE, b"")
        sock.setsockopt(zmq.RCVHWM, 2)
        sock.setsockopt(zmq.RCVTIMEO, 2000)
        sock.setsockopt(zmq.LINGER, 0)
        sock.connect(self.endpoint)

        while not self._stop:
            try:
                raw = sock.recv_string()
                data = json.loads(raw)
                b64 = data["images"].get("head_camera", "")
                if not b64:
                    continue
                buf = base64.b64decode(b64)
                arr = np.frombuffer(buf, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is not None:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    with self.frame_lock:
                        self.latest_frame = frame
                        self.connected = True
            except zmq.Again:
                with self.frame_lock:
                    self.connected = False
            except Exception:
                time.sleep(0.1)

        sock.close()
        ctx.term()


def smooth_ratio(t):
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def clamp_joint(j, v):
    _, lo, hi = JOINT_INFO[j]
    return float(np.clip(v, lo, hi))


# ---------------------------------------------------------------------------
# Robot controller (runs in background thread)
# ---------------------------------------------------------------------------

class RobotController:
    def __init__(self):
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.state_received = False
        self.crc = CRC()
        self.mode_machine = 0

        self.joint_positions = {j: 0.0 for j in ALL_ARM_JOINTS}
        self.joint_velocities = {j: 0.0 for j in ALL_ARM_JOINTS}
        self.joint_torques = {j: 0.0 for j in ALL_ARM_JOINTS}
        self.imu_rpy = [0.0, 0.0, 0.0]

        self.left_hand_state = None
        self.right_hand_state = None
        self.left_pressures = [[PRESS_BASELINE] * N_PRESS_PER_SENSOR for _ in range(N_HAND_SENSORS)]
        self.right_pressures = [[PRESS_BASELINE] * N_PRESS_PER_SENSOR for _ in range(N_HAND_SENSORS)]
        self.hand_lock = threading.Lock()

        self.is_controlling = False
        self.action_queue = []
        self.action_lock = threading.Lock()
        self.current_action_name = ""

        self._time = 0.0
        self._phase_idx = 0
        self._phase_start = 0.0
        self._phase_start_pos = {}
        self._last_cmd = {}
        self._ramp_up = True
        self._ramp_down = False
        self._ramp_time = 0.0

        self.control_thread = None
        self.e_stop = False

    def init(self):
        self.lowcmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_pub.Init()
        self.lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_sub.Init(self._on_state, 10)

        self.left_hand_sub = ChannelSubscriber(TOPIC_LEFT_HAND_STATE, HandState_)
        self.left_hand_sub.Init(self._on_left_hand, 10)
        self.right_hand_sub = ChannelSubscriber(TOPIC_RIGHT_HAND_STATE, HandState_)
        self.right_hand_sub.Init(self._on_right_hand, 10)

    def wait_for_state(self, timeout=5.0):
        t0 = time.time()
        while not self.state_received and time.time() - t0 < timeout:
            time.sleep(0.1)
        return self.state_received

    def _on_state(self, msg: LowState_):
        self.low_state = msg
        self.mode_machine = msg.mode_machine
        for j in ALL_ARM_JOINTS:
            self.joint_positions[j] = msg.motor_state[j].q
            self.joint_velocities[j] = msg.motor_state[j].dq
            self.joint_torques[j] = msg.motor_state[j].tau_est
        self.imu_rpy = list(msg.imu_state.rpy)
        if not self.state_received:
            self.state_received = True

    def _on_left_hand(self, msg: HandState_):
        self.left_hand_state = msg
        with self.hand_lock:
            for i, ps in enumerate(msg.press_sensor_state[:N_HAND_SENSORS]):
                self.left_pressures[i] = list(ps.pressure[:N_PRESS_PER_SENSOR])

    def _on_right_hand(self, msg: HandState_):
        self.right_hand_state = msg
        with self.hand_lock:
            for i, ps in enumerate(msg.press_sensor_state[:N_HAND_SENSORS]):
                self.right_pressures[i] = list(ps.pressure[:N_PRESS_PER_SENSOR])

    def start_action(self, action_name):
        if self.e_stop:
            return
        with self.action_lock:
            keyframes = ACTIONS.get(action_name, [])
            if not keyframes:
                return
            self.action_queue = list(keyframes)
            self.current_action_name = action_name
            self._phase_idx = -1
            self._time = 0.0
            self._ramp_up = True
            self._ramp_down = False
            self._ramp_time = 0.0
            for j in ALL_ARM_JOINTS:
                self._last_cmd[j] = self.joint_positions[j]
                self._phase_start_pos[j] = self.joint_positions[j]

            if not self.is_controlling:
                self.is_controlling = True
                self.control_thread = RecurrentThread(
                    interval=CONTROL_DT, target=self._control_tick, name="ctrl"
                )
                self.control_thread.Start()

    def stop_action(self):
        """Gracefully stop current action and ramp down."""
        with self.action_lock:
            self.action_queue = []
            if self.is_controlling:
                self._ramp_down = True
                self._ramp_time = 0.0

    def emergency_stop(self):
        self.e_stop = True
        self.is_controlling = False
        self.action_queue = []
        self.current_action_name = "E-STOP"

    def _control_tick(self):
        if self.e_stop or not self.is_controlling:
            return

        self.low_cmd.mode_pr = 0
        self.low_cmd.mode_machine = self.mode_machine

        with self.action_lock:
            # Ramp up phase
            if self._ramp_up:
                self._ramp_time += CONTROL_DT
                s = smooth_ratio(self._ramp_time / RAMP_DURATION)
                for j in ALL_ARM_JOINTS:
                    pos = self._phase_start_pos[j]
                    self.low_cmd.motor_cmd[j].mode = 1
                    self.low_cmd.motor_cmd[j].q = clamp_joint(j, pos)
                    self.low_cmd.motor_cmd[j].dq = 0.0
                    self.low_cmd.motor_cmd[j].tau = 0.0
                    self.low_cmd.motor_cmd[j].kp = KP * s
                    self.low_cmd.motor_cmd[j].kd = KD * s
                    self._last_cmd[j] = pos
                if self._ramp_time >= RAMP_DURATION:
                    self._ramp_up = False
                    self._advance_phase()
                self._send()
                return

            # Ramp down phase
            if self._ramp_down:
                self._ramp_time += CONTROL_DT
                s = smooth_ratio(self._ramp_time / RAMP_DURATION)
                for j in ALL_ARM_JOINTS:
                    self.low_cmd.motor_cmd[j].mode = 1
                    self.low_cmd.motor_cmd[j].q = clamp_joint(j, self._last_cmd[j])
                    self.low_cmd.motor_cmd[j].dq = 0.0
                    self.low_cmd.motor_cmd[j].tau = 0.0
                    self.low_cmd.motor_cmd[j].kp = KP * (1.0 - s)
                    self.low_cmd.motor_cmd[j].kd = KD * (1.0 - s)
                if self._ramp_time >= RAMP_DURATION:
                    self.is_controlling = False
                    self._ramp_down = False
                    self.current_action_name = ""
                self._send()
                return

            # No more phases
            if self._phase_idx >= len(self.action_queue):
                self._ramp_down = True
                self._ramp_time = 0.0
                return

            name, dur, targets = self.action_queue[self._phase_idx]
            self._time += CONTROL_DT
            local_t = self._time - self._phase_start
            ratio = np.clip(local_t / dur, 0.0, 1.0) if dur > 0 else 1.0

            # Wave handling
            if name == "wave":
                wave_phase = ratio * 3 * 2 * PI
                wave = {
                    22: -1.2 + 0.3 * np.sin(wave_phase),
                    25: 1.2 + 0.4 * np.sin(wave_phase + PI / 2),
                    28: 0.5 * np.sin(wave_phase * 1.5),
                }
                for j in ALL_ARM_JOINTS:
                    pos = wave.get(j, self._phase_start_pos[j])
                    self.low_cmd.motor_cmd[j].mode = 1
                    self.low_cmd.motor_cmd[j].q = clamp_joint(j, pos)
                    self.low_cmd.motor_cmd[j].dq = 0.0
                    self.low_cmd.motor_cmd[j].tau = 0.0
                    self.low_cmd.motor_cmd[j].kp = KP
                    self.low_cmd.motor_cmd[j].kd = KD
                    self._last_cmd[j] = pos
            elif targets:
                s = smooth_ratio(ratio)
                for j in ALL_ARM_JOINTS:
                    start = self._phase_start_pos[j]
                    end = targets.get(j, start)
                    pos = start + (end - start) * s
                    self.low_cmd.motor_cmd[j].mode = 1
                    self.low_cmd.motor_cmd[j].q = clamp_joint(j, pos)
                    self.low_cmd.motor_cmd[j].dq = 0.0
                    self.low_cmd.motor_cmd[j].tau = 0.0
                    self.low_cmd.motor_cmd[j].kp = KP
                    self.low_cmd.motor_cmd[j].kd = KD
                    self._last_cmd[j] = pos

            if ratio >= 1.0:
                self._advance_phase()

        self._send()

    def _advance_phase(self):
        for j in ALL_ARM_JOINTS:
            self._phase_start_pos[j] = self._last_cmd[j]
        self._phase_idx += 1
        self._phase_start = self._time

    def _send(self):
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_pub.Write(self.low_cmd)


# ---------------------------------------------------------------------------
# Dashboard GUI
# ---------------------------------------------------------------------------

class Dashboard:
    def __init__(self, controller: RobotController, camera: CameraReceiver = None):
        self.ctrl = controller
        self.camera = camera
        self._cam_photo = None  # prevent GC of Tk photo image
        self.root = tk.Tk()
        self.root.title("Unitree G1 — Arm Control Dashboard")
        self.root.configure(bg="#1e1e1e")
        self.root.geometry("1280x800")
        self.root.resizable(True, True)

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background="#1e1e1e")
        style.configure("TLabel", background="#1e1e1e", foreground="#e0e0e0", font=("Monospace", 10))
        style.configure("Title.TLabel", font=("Monospace", 12, "bold"), foreground="#ffffff")
        style.configure("Status.TLabel", font=("Monospace", 10), foreground="#80ff80")
        style.configure("Action.TButton", font=("Monospace", 10), padding=6)
        style.configure("Stop.TButton", font=("Monospace", 11, "bold"), padding=8)

        self._build_ui()
        self.bar_canvas_items = {}
        self._draw_joint_bars_init()

    def _build_ui(self):
        # Main layout: left (camera + IMU) | center (joints) | right (actions)
        main = ttk.Frame(self.root)
        main.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        # ---- Left column: Camera + IMU ----
        left = ttk.Frame(main, width=320)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))

        ttk.Label(left, text="Camera Feed", style="Title.TLabel").pack(pady=(0, 4))
        self.cam_canvas = tk.Canvas(left, width=300, height=225, bg="#2a2a2a",
                                     highlightthickness=1, highlightbackground="#555")
        self.cam_canvas.pack()
        self._cam_placeholder = self.cam_canvas.create_text(
            150, 100, text="Connecting to camera...\n\nRun: bash start_camera.sh",
            fill="#555", font=("Monospace", 9), justify=tk.CENTER
        )

        ttk.Label(left, text="IMU Orientation", style="Title.TLabel").pack(pady=(16, 4))
        self.imu_frame = ttk.Frame(left)
        self.imu_frame.pack(fill=tk.X, padx=4)

        self.imu_labels = {}
        for name in ["Roll", "Pitch", "Yaw"]:
            row = ttk.Frame(self.imu_frame)
            row.pack(fill=tk.X, pady=1)
            ttk.Label(row, text=f"  {name}:", width=8).pack(side=tk.LEFT)
            lbl = ttk.Label(row, text="0.0°", width=10, anchor=tk.E)
            lbl.pack(side=tk.RIGHT)
            self.imu_labels[name] = lbl

        ttk.Label(left, text="Status", style="Title.TLabel").pack(pady=(16, 4))
        self.status_label = ttk.Label(left, text="Idle", style="Status.TLabel",
                                       wraplength=280, justify=tk.LEFT)
        self.status_label.pack(fill=tk.X, padx=4)

        self.mode_label = ttk.Label(left, text="mode_machine: --", style="TLabel")
        self.mode_label.pack(fill=tk.X, padx=4, pady=(4, 0))

        # ---- Tactile sensor panel (below status, in left column) ----
        ttk.Label(left, text="Tactile Sensors", style="Title.TLabel").pack(pady=(12, 4))
        self.tactile_canvas = tk.Canvas(
            left, width=300, height=230, bg="#1e1e1e",
            highlightthickness=1, highlightbackground="#555",
        )
        self.tactile_canvas.pack(fill=tk.X, padx=0)
        self._tactile_placeholder = self.tactile_canvas.create_text(
            150, 110, text="Waiting for hand data...",
            fill="#555", font=("Monospace", 9), justify=tk.CENTER,
        )

        # ---- Center: Joint bars ----
        center = ttk.Frame(main)
        center.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 8))

        ttk.Label(center, text="Arm Joint States (real-time)", style="Title.TLabel").pack(pady=(0, 4))

        self.joints_canvas = tk.Canvas(center, bg="#1e1e1e", highlightthickness=0)
        self.joints_canvas.pack(fill=tk.BOTH, expand=True)

        # ---- Right column: Actions ----
        right = ttk.Frame(main, width=180)
        right.pack(side=tk.RIGHT, fill=tk.Y)

        ttk.Label(right, text="Actions", style="Title.TLabel").pack(pady=(0, 8))

        for action_name in ACTIONS:
            btn = ttk.Button(right, text=action_name, style="Action.TButton",
                             command=lambda n=action_name: self._on_action(n))
            btn.pack(fill=tk.X, pady=3, padx=4)

        ttk.Separator(right, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=12, padx=4)

        stop_btn = tk.Button(right, text="STOP", bg="#cc3333", fg="white",
                             font=("Monospace", 12, "bold"), relief=tk.RAISED,
                             activebackground="#ff4444", padx=10, pady=6,
                             command=self._on_stop)
        stop_btn.pack(fill=tk.X, padx=4, pady=2)

        estop_btn = tk.Button(right, text="E-STOP", bg="#880000", fg="white",
                              font=("Monospace", 12, "bold"), relief=tk.RAISED,
                              activebackground="#aa0000", padx=10, pady=6,
                              command=self._on_estop)
        estop_btn.pack(fill=tk.X, padx=4, pady=2)

    def _draw_joint_bars_init(self):
        """Create initial bar chart items on the canvas."""
        self.bar_items = {}
        self.root.update_idletasks()

    def _on_action(self, name):
        self.ctrl.start_action(name)

    def _on_stop(self):
        self.ctrl.stop_action()

    def _on_estop(self):
        self.ctrl.emergency_stop()

    def _update_joint_bars(self):
        c = self.joints_canvas
        c.delete("all")

        w = c.winfo_width()
        h = c.winfo_height()
        if w < 10 or h < 10:
            return

        n = len(JOINT_INFO)
        bar_h = max(14, min(28, (h - 20) // n - 4))
        label_w = 120
        margin_r = 60
        bar_max_w = w - label_w - margin_r - 10

        y = 10
        for j in ALL_ARM_JOINTS:
            name, lo, hi = JOINT_INFO[j]
            q = self.ctrl.joint_positions.get(j, 0.0)

            # Label
            c.create_text(label_w - 4, y + bar_h // 2, text=name,
                          anchor=tk.E, fill="#c0c0c0", font=("Monospace", 9))

            # Background bar (full range)
            c.create_rectangle(label_w, y, label_w + bar_max_w, y + bar_h,
                               fill="#333", outline="#555")

            # Zero line
            if lo < 0 < hi:
                zero_x = label_w + (-lo) / (hi - lo) * bar_max_w
                c.create_line(zero_x, y, zero_x, y + bar_h, fill="#666", width=1)

            # Current position bar
            frac = (q - lo) / (hi - lo) if hi > lo else 0.5
            frac = np.clip(frac, 0.0, 1.0)
            bar_x = label_w + frac * bar_max_w

            # Color based on how close to limits
            margin = 0.1
            if frac < margin or frac > (1 - margin):
                color = "#ff4444"
            elif frac < 2 * margin or frac > (1 - 2 * margin):
                color = "#ffaa00"
            else:
                color = "#44aaff"

            # Draw indicator
            c.create_line(bar_x, y + 1, bar_x, y + bar_h - 1, fill=color, width=3)
            c.create_oval(bar_x - 4, y + bar_h // 2 - 4,
                          bar_x + 4, y + bar_h // 2 + 4, fill=color, outline="")

            # Value text
            deg = np.degrees(q)
            c.create_text(label_w + bar_max_w + 6, y + bar_h // 2,
                           text=f"{deg:+5.1f}°", anchor=tk.W,
                           fill="#e0e0e0", font=("Monospace", 9))

            y += bar_h + 4

    def _update_camera(self):
        if self.camera is None:
            return
        frame = self.camera.get_frame()
        if frame is not None:
            cw = self.cam_canvas.winfo_width()
            ch = self.cam_canvas.winfo_height()
            if cw < 10 or ch < 10:
                return
            h, w = frame.shape[:2]
            scale = min(cw / w, ch / h)
            nw, nh = int(w * scale), int(h * scale)
            resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
            img = Image.fromarray(resized)
            self._cam_photo = ImageTk.PhotoImage(img)
            self.cam_canvas.delete("all")
            self.cam_canvas.create_image(cw // 2, ch // 2, image=self._cam_photo)
        elif self.camera.connected is False:
            self.cam_canvas.delete("all")
            self._cam_placeholder = self.cam_canvas.create_text(
                150, 100, text="No camera stream\n\nRun: bash start_camera.sh",
                fill="#555", font=("Monospace", 9), justify=tk.CENTER
            )

    @staticmethod
    def _pressure_normalized(vals):
        """Compute mean pressure of active pads (above baseline threshold)."""
        active = [v for v in vals if v > PRESS_BASELINE + 500]
        if not active:
            return 0.0
        mean_above = sum(v - PRESS_BASELINE for v in active) / len(active)
        return mean_above / (PRESS_MAX - PRESS_BASELINE)

    @staticmethod
    def _pressure_bar_color(t):
        """Map normalized 0..1 to color: dark→cyan→green→yellow→red."""
        t = max(0.0, min(t, 1.0))
        if t < 0.01:
            return "#333333"
        if t < 0.33:
            s = t / 0.33
            r, g, b = 0, int(120 + 135 * s), int(200 * (1 - s * 0.5))
        elif t < 0.66:
            s = (t - 0.33) / 0.33
            r, g, b = int(255 * s), 255, 0
        else:
            s = (t - 0.66) / 0.34
            r, g, b = 255, int(255 * (1 - s)), 0
        return f"#{r:02x}{g:02x}{b:02x}"

    def _update_tactile(self):
        """Draw per-sensor aggregated pressure bars for both hands."""
        c = self.tactile_canvas
        has_left = self.ctrl.left_hand_state is not None
        has_right = self.ctrl.right_hand_state is not None

        if not has_left and not has_right:
            return

        c.delete("all")
        cw = c.winfo_width()
        ch = c.winfo_height()
        if cw < 10 or ch < 10:
            return

        with self.ctrl.hand_lock:
            lp = [row[:] for row in self.ctrl.left_pressures]
            rp = [row[:] for row in self.ctrl.right_pressures]

        hands = []
        if has_left:
            hands.append(("LEFT", lp))
        if has_right:
            hands.append(("RIGHT", rp))

        n_hands = len(hands)
        section_h = (ch - 6) // max(n_hands, 1)
        label_w = 28
        value_w = 50
        bar_max_w = cw - label_w - value_w - 16

        for hi, (hand_name, pressures) in enumerate(hands):
            base_y = 3 + hi * section_h

            c.create_text(
                4, base_y, text=hand_name, anchor=tk.NW,
                fill="#b0b0b0", font=("Monospace", 8, "bold"),
            )

            active_sensors = []
            for si in range(min(N_HAND_SENSORS, len(pressures))):
                vals = pressures[si]
                is_connected = any(v > 1.0 for v in vals)
                if not is_connected:
                    continue
                norm = self._pressure_normalized(vals)
                n_active = sum(1 for v in vals if v > PRESS_BASELINE + 500)
                peak = max(vals)
                active_sensors.append((si, norm, n_active, peak))

            if not active_sensors:
                c.create_text(
                    cw // 2, base_y + section_h // 2,
                    text="(no active sensors)", fill="#555",
                    font=("Monospace", 8),
                )
                continue

            n_rows = len(active_sensors)
            bar_h = max(8, min(18, (section_h - 18) // n_rows - 2))
            y = base_y + 14

            for si, norm, n_active, peak in active_sensors:
                label = f"S{si}"
                c.create_text(
                    label_w - 2, y + bar_h // 2,
                    text=label, anchor=tk.E,
                    fill="#888", font=("Monospace", 7),
                )

                c.create_rectangle(
                    label_w, y, label_w + bar_max_w, y + bar_h,
                    fill="#252525", outline="#3a3a3a",
                )

                fill_w = int(norm * bar_max_w)
                if fill_w > 1:
                    color = self._pressure_bar_color(norm)
                    c.create_rectangle(
                        label_w, y,
                        label_w + fill_w, y + bar_h,
                        fill=color, outline="",
                    )

                    for pi in range(n_active):
                        dot_x = label_w + fill_w - 3 - pi * 5
                        if dot_x < label_w + 2:
                            break
                        c.create_oval(
                            dot_x, y + 2, dot_x + 3, y + bar_h - 2,
                            fill="#ffffff", outline="",
                        )

                delta_k = (peak - PRESS_BASELINE) / 1000.0
                pct = int(norm * 100)
                c.create_text(
                    label_w + bar_max_w + 4, y + bar_h // 2,
                    text=f"{pct}% ({n_active})",
                    anchor=tk.W, fill="#aaa", font=("Monospace", 7),
                )

                y += bar_h + 2

    def _update_loop(self):
        if not self.ctrl.state_received:
            self.status_label.configure(text="Waiting for robot...")
        else:
            # IMU
            rpy = self.ctrl.imu_rpy
            for i, name in enumerate(["Roll", "Pitch", "Yaw"]):
                self.imu_labels[name].configure(text=f"{np.degrees(rpy[i]):+6.1f}°")

            # Status
            if self.ctrl.e_stop:
                self.status_label.configure(text="EMERGENCY STOP", foreground="#ff4444")
            elif self.ctrl.is_controlling:
                self.status_label.configure(
                    text=f"Running: {self.ctrl.current_action_name}",
                    foreground="#80ff80"
                )
            else:
                self.status_label.configure(text="Idle — ready", foreground="#aaaaaa")

            self.mode_label.configure(text=f"mode_machine: {self.ctrl.mode_machine}")

            # Joint bars
            self._update_joint_bars()

        # Tactile sensors
        self._update_tactile()

        # Camera
        self._update_camera()

        self.root.after(50, self._update_loop)  # ~20 Hz UI update

    def run(self):
        self.root.after(100, self._update_loop)
        self.root.mainloop()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def release_motion_controller():
    msc = MotionSwitcherClient()
    msc.SetTimeout(5.0)
    msc.Init()
    code, result = msc.CheckMode()
    print(f"Current mode: {result}")
    if result and result.get("name"):
        print(f"Releasing mode '{result['name']}'...")
        msc.ReleaseMode()
        time.sleep(2)
        code, result = msc.CheckMode()
        print(f"After release: {result}")


if __name__ == "__main__":
    print("=" * 50)
    print("  G1 Arm Control Dashboard")
    print("=" * 50)

    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    release_motion_controller()

    ctrl = RobotController()
    ctrl.init()

    print("Waiting for robot state...")
    if not ctrl.wait_for_state():
        print("ERROR: No robot state received!")
        sys.exit(1)
    print("Robot connected!")

    cam = None
    if HAS_ZMQ:
        cam = CameraReceiver()
        cam.start()
        print(f"Camera receiver started (connecting to {cam.endpoint})")
    else:
        print("Warning: pyzmq not installed, camera feed disabled")

    print("Launching dashboard...")
    dashboard = Dashboard(ctrl, camera=cam)
    dashboard.run()
