#!/usr/bin/env python3
"""
Gradio UI for G1 drag-and-teach: high-level goals with numbered steps.

Usage:
    conda activate lerobot
    python teach_panel.py
    # or: bash run_teach_ui.sh
"""

from __future__ import annotations

import os
import sys
import threading
import time
import traceback

import gradio as gr

ROOT = os.path.dirname(os.path.abspath(__file__))
UTILS = os.path.join(ROOT, "utils")
TRAJ_DIR = os.path.join(ROOT, "trajectories")
TASKS_DIR = os.path.join(ROOT, "tasks")
sys.path.insert(0, UTILS)

from teach_sequences import (  # noqa: E402
    DEFAULT_MAX_STEPS,
    assign_step_recording,
    create_or_load_goal,
    estimate_sequence_duration,
    format_goal_steps_table,
    goal_choices_for_dropdown,
    load_task,
    save_task_data,
    step_paths_ordered,
    trajectory_path_for_step,
)
from sequence_player import play_task_sequence  # noqa: E402
from teach import (  # noqa: E402
    GravityCompensator,
    TeachRecorder,
    ensure_ai_mode,
)
from unitree_sdk2py.core.channel import ChannelFactoryInitialize  # noqa: E402

DEFAULT_PORT = 7861
MAX_RECORD_STEP_BTNS = 12

RECORDING_BTN_CSS = """
button.step-recording {
    background-color: #22c55e !important;
    border-color: #16a34a !important;
    color: #ffffff !important;
}
button.step-recording:hover {
    background-color: #16a34a !important;
}
"""


class TeachUISession:
    """Shared robot session for record + replay from the web UI."""

    def __init__(self):
        self.lock = threading.Lock()
        self.recorder: TeachRecorder | None = None
        self.connected = False
        self.forward_ready = False
        self.arms_compliant = False
        self.prepared = False
        self.replay_busy = False
        self.log: list[str] = []
        self._dds_init = False
        self._network: str | None = None
        self.current_goal: dict | None = None
        self.recording_step: int | None = None

    def _append_log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        self.log.append(f"[{ts}] {msg}")
        if len(self.log) > 200:
            self.log = self.log[-200:]

    def log_text(self) -> str:
        return "\n".join(self.log) or "(no messages yet)"

    def _init_dds(self, network: str | None):
        if self._dds_init:
            return
        if network and network.strip():
            ChannelFactoryInitialize(0, network.strip())
            self._network = network.strip()
        else:
            ChannelFactoryInitialize(0)
            self._network = None
        self._dds_init = True

    def connect(self, network: str, record_hands: bool) -> str:
        with self.lock:
            if self.connected:
                return self.log_text()
            try:
                self._init_dds(network)
                self._append_log("Checking balance mode (ai)...")
                if not ensure_ai_mode(interactive=False):
                    self._append_log(
                        "Aborted: start ai balance mode first "
                        "(L1+A, L1+UP on hand controller)."
                    )
                    return self.log_text()

                grav = GravityCompensator()
                self.recorder = TeachRecorder(
                    record_hands=record_hands,
                    grav_comp=grav,
                )
                self.recorder.init()
                self._append_log("Waiting for robot state...")
                if not self.recorder.wait_for_state():
                    self.recorder = None
                    self._append_log("ERROR: No robot state received.")
                    return self.log_text()

                self.connected = True
                self.forward_ready = False
                self.arms_compliant = False
                self.prepared = False
                self._append_log(
                    "Connected. Click **Prepare (forward pose)**, "
                    "then **Record Step N**."
                )
            except Exception as e:
                self.recorder = None
                self.connected = False
                self._append_log(f"Connect failed: {e}")
                self._append_log(traceback.format_exc())
            return self.log_text()

    def _release_robot_control(self) -> None:
        """Slowly release arm_sdk from the current pose (relax)."""
        if not self.recorder or not self.recorder.low_state:
            return
        if self.recorder.recording:
            self.recorder.stop_recording()
        if (
            self.recorder._session_thread
            and self.recorder._session_thread.is_alive()
        ):
            self.recorder.suspend_compliant_session()
        hold = self.recorder._get_arm_positions()
        self.recorder._release_arm_control(hold_pose=hold)

    def _relax_orphaned(self, network: str, record_hands: bool) -> str:
        """Release when UI session was reset but the robot still holds pose."""
        try:
            self._init_dds(network)
            self._append_log(
                "No active UI session — releasing arm_sdk anyway..."
            )
            grav = GravityCompensator()
            rec = TeachRecorder(
                record_hands=record_hands,
                grav_comp=grav,
            )
            rec.init()
            if not rec.wait_for_state(timeout=4.0):
                self._append_log(
                    "ERROR: No robot state. Connect, then try again."
                )
                rec.close_channels()
                return self.log_text()
            rec._release_arm_control(
                hold_pose=rec._get_arm_positions(),
            )
            rec.close_channels()
            self._append_log(
                "Arm control released. Click **Connect** before next teach."
            )
        except Exception as e:
            self._append_log(f"Relax failed: {e}")
            self._append_log(traceback.format_exc())
        return self.log_text()

    def disconnect(self, network: str, record_hands: bool) -> str:
        with self.lock:
            if not self.connected or not self.recorder:
                return self._relax_orphaned(network, record_hands)
            try:
                self._release_robot_control()
                self._append_log(
                    "Disconnected & relaxed — arm control released."
                )
            except Exception as e:
                self._append_log(f"Disconnect error: {e}")
                self._append_log(traceback.format_exc())
            finally:
                if self.recorder:
                    self.recorder.close_channels()
                self.recorder = None
                self.connected = False
                self.forward_ready = False
                self.arms_compliant = False
                self.prepared = False
                self.current_goal = None
                self.recording_step = None
            return self.log_text()

    def step_button_updates(self) -> list:
        """Gradio updates: green highlight on the step currently recording."""
        out = []
        for n in range(1, MAX_RECORD_STEP_BTNS + 1):
            if n == self.recording_step:
                out.append(
                    gr.update(variant="primary", elem_classes=["step-recording"])
                )
            else:
                out.append(gr.update(variant="secondary", elem_classes=[]))
        return out

    def prepare_forward(self) -> str:
        """Move arms to forward pose (stiff). Required before recording."""
        with self.lock:
            if not self.connected or not self.recorder:
                self._append_log("Connect to the robot first.")
                return self.log_text()
            if self.replay_busy:
                self._append_log("Wait for replay to finish.")
                return self.log_text()
            if self.recorder.recording:
                self._append_log("Stop & save the current step first.")
                return self.log_text()
            try:
                self._append_log(
                    "Prepare: close hands first, then slow move to "
                    "forward (stiff, ~20s)..."
                )
                self._append_log(
                    "Stop gradio_panel.py teleop if it is still running."
                )
                if self.recorder._session_thread and (
                    self.recorder._session_thread.is_alive()
                ):
                    self.recorder.suspend_compliant_session()
                ok = self.recorder.prepare_recording_pose(
                    resume_compliant=False,
                    quick=False,
                    ui_prepare=True,
                )
                if not ok:
                    self.forward_ready = False
                    self._append_log("Prepare failed.")
                    return self.log_text()
                self.forward_ready = True
                self.prepared = True
                self.arms_compliant = False
                self._append_log(
                    "Forward pose ready (stiff). "
                    "You may click **Record Step N**."
                )
            except Exception as e:
                self.forward_ready = False
                self._append_log(f"Prepare failed: {e}")
                self._append_log(traceback.format_exc())
            return self.log_text()

    def _steps_table(self) -> str:
        return format_goal_steps_table(
            self.current_goal, self.recording_step,
        )

    def use_goal(self, goal_name: str, max_steps: int) -> str:
        name = (goal_name or "").strip()
        if not name:
            self._append_log("Enter a high-level goal (e.g. prepare a drink).")
            return self.log_text()
        self.current_goal = create_or_load_goal(
            TASKS_DIR, name, max_steps=int(max_steps),
        )
        self.recording_step = None
        self._append_log(f'Goal ready: "{name}" (up to {int(max_steps)} steps)')
        return self.log_text()

    def load_goal_from_path(self, task_path: str) -> str:
        if not task_path or not os.path.isfile(task_path):
            self._append_log("Select a saved goal from the list.")
            return self.log_text()
        self.current_goal = load_task(task_path)
        self.recording_step = None
        self._append_log(f'Loaded goal: "{self.current_goal.get("name")}"')
        return self.log_text()

    def record_step(self, step_num: int, step_name: str) -> str:
        with self.lock:
            if not self.connected or not self.recorder:
                self._append_log("Connect to the robot first.")
                return self.log_text()
            if not self.current_goal:
                self._append_log("Enter and use a goal first.")
                return self.log_text()
            if self.replay_busy:
                self._append_log("Wait for replay to finish.")
                return self.log_text()
            if self.recorder.recording:
                self._append_log("Stop & save the current step first.")
                return self.log_text()
            if not self.forward_ready:
                self._append_log(
                    "Click **Prepare (forward pose)** before recording."
                )
                return self.log_text()
            step = int(step_num)
            max_n = int(self.current_goal.get("max_steps", DEFAULT_MAX_STEPS))
            if step < 1 or step > max_n:
                self._append_log(f"Step must be 1–{max_n}.")
                return self.log_text()
            self.recording_step = step
            goal_title = self.current_goal["name"]
            try:
                self._append_log(
                    f'Recording **Step {step}** for "{goal_title}" — '
                    "entering drag-teach..."
                )
                ok = self.recorder.begin_step_recording()
                if not ok:
                    self.recording_step = None
                    self._append_log("Could not start drag-teach.")
                    return self.log_text()
                self.arms_compliant = True
                if not self.recorder.start_recording():
                    self.recording_step = None
                    self._append_log("Already recording.")
                    return self.log_text()
                label = (step_name or "").strip() or f"step {step}"
                self._append_log(
                    f'**Recording step {step}** ({label}) — '
                    "Stop & save step when done."
                )
            except Exception as e:
                self.recording_step = None
                self._append_log(f"Record step failed: {e}")
                self._append_log(traceback.format_exc())
            return self.log_text()

    def stop_save_step(self, step_label: str) -> str:
        with self.lock:
            if not self.connected or not self.recorder:
                self._append_log("Connect to the robot first.")
                return self.log_text()
            if not self.current_goal or self.recording_step is None:
                self._append_log("Click a Record Step button first.")
                return self.log_text()
            if not self.recorder.recording:
                self._append_log("Not recording — click Record Step N first.")
                return self.log_text()
            step = self.recording_step
            goal_name = self.current_goal["name"]
            stats = self.recorder.stop_recording()
            if not stats or stats[0] == 0:
                self._append_log("No frames captured — nothing saved.")
                return self.log_text()
            n_frames, dur = stats
            os.makedirs(TRAJ_DIR, exist_ok=True)
            out_path = trajectory_path_for_step(TRAJ_DIR, goal_name, step)
            meta_name = f"{goal_name} — step {step}"
            saved = self.recorder.save_recording(out_path, name=meta_name)
            label = (step_label or "").strip() or f"step {step}"
            assign_step_recording(
                self.current_goal, step, saved, label=label,
            )
            self.current_goal = load_task(self.current_goal["path"])
            self.recording_step = None
            self._append_log(
                f'Saved step {step}: {n_frames} frames, {dur:.1f}s → '
                f'{os.path.basename(saved)}'
            )
            self._append_log(
                "Closing fingers and holding forward for next step..."
            )
            self.recorder.hold_forward_between_steps(enable_drag=False)
            self.arms_compliant = False
            self.forward_ready = True
            self.prepared = True
            self._append_log(
                f'Step {step} saved. At forward (stiff) — '
                f"click Record Step {step + 1} or Execute goal."
            )
            return self.log_text()

    def execute_goal(
        self,
        task_path: str | None,
        speed: float,
        pause_sec: float,
        clearance_between: bool,
    ) -> str:
        if task_path and os.path.isfile(task_path):
            self.current_goal = load_task(task_path)
        if not self.current_goal:
            self._append_log("Select or create a goal first.")
            return self.log_text()

        data = self.current_goal
        data["pause_sec"] = float(pause_sec)
        data["clearance_between"] = bool(clearance_between)
        save_task_data(data)

        paths = step_paths_ordered(data, TRAJ_DIR)
        if not paths:
            self._append_log(
                f'No recorded steps for "{data["name"]}" yet. '
                "Record Step 1, 2, … first."
            )
            return self.log_text()

        with self.lock:
            if self.replay_busy:
                self._append_log("Already running.")
                return self.log_text()
            if self.recorder and self.recorder.recording:
                self._append_log("Stop recording before execute.")
                return self.log_text()
            if self.recorder:
                if (
                    self.recorder._session_thread
                    and self.recorder._session_thread.is_alive()
                ):
                    self.recorder.suspend_compliant_session()
                self.arms_compliant = False
            self.replay_busy = True

        est = estimate_sequence_duration(paths, float(pause_sec))
        self._append_log(
            f'Execute "{data["name"]}": {len(paths)} steps, ~{est:.1f}s'
        )
        self._append_log(
            "Replay: slow move to 1st frame only (no spread→forward prepare)."
        )
        try:
            if not self._dds_init:
                self._init_dds(self._network or "")
            play_task_sequence(
                paths,
                speed=float(speed),
                pause_sec=float(pause_sec),
                clearance_between=False,
                recorder=self.recorder,
                prepare_first=False,
                return_to_forward_at_end=True,
                log=self._append_log,
            )
            # After execute, arms are returned to forward (stiff). Mark UI
            # state so user can immediately Record Step N or Execute again
            # without clicking Prepare a second time.
            self.prepared = True
            self.forward_ready = True
            self.arms_compliant = False
            self._append_log(
                "Execute done — at forward (stiff). Ready for next "
                "Record or Execute."
            )
        except Exception as e:
            self._append_log(f"Execute failed: {e}")
            self._append_log(traceback.format_exc())
        finally:
            with self.lock:
                self.replay_busy = False
        return self.log_text()

SESSION = TeachUISession()


def build_ui() -> gr.Blocks:
    goal_label_to_path: dict[str, str] = {}
    goal_labels, goal_label_to_path = goal_choices_for_dropdown(
        TASKS_DIR, TRAJ_DIR,
    )

    def refresh_goal_dropdown():
        labels, mapping = goal_choices_for_dropdown(TASKS_DIR, TRAJ_DIR)
        nonlocal goal_label_to_path
        goal_label_to_path = mapping
        return gr.update(
            choices=labels,
            value=labels[0] if labels else None,
        )

    def _ui_after_session(*, goal_select=None):
        parts = [SESSION.log_text(), SESSION._steps_table()]
        if goal_select is not None:
            parts.append(goal_select)
        parts.extend(SESSION.step_button_updates())
        return tuple(parts)

    def on_connect(network, record_hands):
        SESSION.connect(network, record_hands)
        return _ui_after_session()

    def on_disconnect(network, record_hands):
        SESSION.disconnect(network, record_hands)
        return _ui_after_session(goal_select=refresh_goal_dropdown())

    def on_prepare():
        SESSION.prepare_forward()
        return _ui_after_session()

    def on_use_goal(goal_name, max_steps):
        SESSION.use_goal(goal_name, max_steps)
        return _ui_after_session(goal_select=refresh_goal_dropdown())

    def on_load_goal(goal_label):
        path = goal_label_to_path.get(goal_label or "")
        if not path:
            return (
                SESSION.log_text(),
                SESSION._steps_table(),
                "",
                DEFAULT_MAX_STEPS,
                0.5,
                True,
            )
        data = load_task(path)
        SESSION.current_goal = data
        SESSION.recording_step = None
        SESSION._append_log(f'Loaded goal: "{data.get("name")}"')
        return (
            SESSION.log_text(),
            SESSION._steps_table(),
            data.get("name", ""),
            int(data.get("max_steps", DEFAULT_MAX_STEPS)),
            float(data.get("pause_sec", 0.5)),
            bool(data.get("clearance_between", False)),
            *SESSION.step_button_updates(),
        )

    def on_record_step(step_num, goal_name, max_steps, step_label):
        if not SESSION.current_goal:
            SESSION.use_goal(goal_name, max_steps)
        SESSION.record_step(step_num, step_label)
        return _ui_after_session()

    def on_stop_save(step_label):
        SESSION.stop_save_step(step_label)
        return _ui_after_session(goal_select=refresh_goal_dropdown())

    def on_execute(goal_label, goal_name, max_steps, speed, pause_sec, clearance):
        path = goal_label_to_path.get(goal_label or "")
        if not path and SESSION.current_goal:
            path = SESSION.current_goal.get("path")
        if not path and (goal_name or "").strip():
            SESSION.use_goal(goal_name, max_steps)
            path = SESSION.current_goal.get("path") if SESSION.current_goal else None
        SESSION.execute_goal(path, speed, pause_sec, clearance)
        return _ui_after_session()

    def on_refresh():
        return _ui_after_session(goal_select=refresh_goal_dropdown())

    with gr.Blocks(
        title="G1 Drag-and-Teach",
        theme=gr.themes.Soft(),
        css=RECORDING_BTN_CSS,
    ) as demo:
        gr.Markdown(
            "# G1 Drag-and-Teach — Goals & Steps\n\n"
            "1. **Connect** to the robot → **Prepare (forward pose)**.\n"
            "2. Enter a **high-level goal** → **Use goal**.\n"
            "3. **Record Step N** (green while recording) → drag-teach → "
            "**Stop & save step**.\n"
            "4. **Execute goal** to replay all steps.\n\n"
            "**Prerequisites:** Robot in `ai` balance mode (L1+A → L1+UP).\n\n"
            "**Do not run `gradio_panel.py` at the same time** (shared `rt/arm_sdk`)."
        )

        gr.Markdown("### Robot")
        with gr.Row():
            network = gr.Textbox(
                label="Network interface (optional)",
                placeholder="e.g. enp2s0",
                lines=1,
                scale=2,
            )
            record_hands = gr.Checkbox(
                label="Record Dex3 hands",
                value=True,
                scale=1,
            )
        with gr.Row():
            connect_btn = gr.Button("Connect", variant="primary")
            disconnect_btn = gr.Button("Disconnect & Relax")
        prepare_btn = gr.Button(
            "Prepare (forward pose)",
            variant="primary",
        )

        with gr.Row():
            with gr.Column(scale=1):
                gr.Markdown("### Goal")
                goal_name = gr.Textbox(
                    label="High-level goal",
                    placeholder='e.g. "prepare a drink"',
                    lines=1,
                )
                max_steps = gr.Slider(
                    minimum=3,
                    maximum=12,
                    value=8,
                    step=1,
                    precision=0,
                    label="Max steps for this goal",
                )
                use_goal_btn = gr.Button("Use / create goal", variant="secondary")
                goal_select = gr.Dropdown(
                    label="Saved goals",
                    choices=goal_labels,
                    interactive=True,
                )
                load_goal_btn = gr.Button("Load selected goal")
                steps_md = gr.Markdown(
                    format_goal_steps_table(None),
                )

            with gr.Column(scale=1):
                gr.Markdown("### Record steps")
                step_label = gr.Textbox(
                    label="Step note (optional)",
                    placeholder='e.g. "grasp cup"',
                    lines=1,
                )
                step_btns: list[gr.Button] = []
                with gr.Row():
                    for n in range(1, 4):
                        step_btns.append(
                            gr.Button(f"Record Step {n}", size="sm")
                        )
                with gr.Row():
                    for n in range(4, 7):
                        step_btns.append(
                            gr.Button(f"Record Step {n}", size="sm")
                        )
                with gr.Row():
                    for n in range(7, 10):
                        step_btns.append(
                            gr.Button(f"Record Step {n}", size="sm")
                        )
                with gr.Row():
                    for n in range(10, 13):
                        step_btns.append(
                            gr.Button(f"Record Step {n}", size="sm")
                        )
                stop_save_btn = gr.Button(
                    "Stop & save step", variant="stop",
                )

                gr.Markdown("### Execute")
                with gr.Row():
                    exec_speed = gr.Slider(
                        minimum=0.25,
                        maximum=2.0,
                        value=1.0,
                        step=0.25,
                        label="Speed",
                    )
                    exec_pause = gr.Slider(
                        minimum=0.0,
                        maximum=3.0,
                        value=0.5,
                        step=0.1,
                        label="Pause between steps (s)",
                    )
                exec_clearance = gr.Checkbox(
                    label=(
                        "Outward clearance between steps "
                        "(unsafe — leave off; ignored during Execute)"
                    ),
                    value=False,
                )
                execute_btn = gr.Button(
                    "Execute goal", variant="primary",
                )
                refresh_btn = gr.Button("Refresh")

        log_box = gr.Textbox(
            label="Log",
            lines=16,
            max_lines=28,
            interactive=False,
        )

        ui_outputs = [log_box, steps_md, *step_btns]
        ui_outputs_with_goal = [log_box, steps_md, goal_select, *step_btns]

        connect_btn.click(
            on_connect,
            inputs=[network, record_hands],
            outputs=ui_outputs,
        )
        disconnect_btn.click(
            on_disconnect,
            inputs=[network, record_hands],
            outputs=ui_outputs_with_goal,
        )
        prepare_btn.click(
            on_prepare,
            outputs=ui_outputs,
        )
        use_goal_btn.click(
            on_use_goal,
            inputs=[goal_name, max_steps],
            outputs=ui_outputs_with_goal,
        )
        load_goal_btn.click(
            on_load_goal,
            inputs=[goal_select],
            outputs=[
                log_box, steps_md, goal_name, max_steps,
                exec_pause, exec_clearance,
                *step_btns,
            ],
        )
        stop_save_btn.click(
            on_stop_save,
            inputs=[step_label],
            outputs=ui_outputs_with_goal,
        )
        execute_btn.click(
            on_execute,
            inputs=[
                goal_select, goal_name, max_steps,
                exec_speed, exec_pause, exec_clearance,
            ],
            outputs=ui_outputs,
        )
        refresh_btn.click(
            on_refresh,
            outputs=ui_outputs_with_goal,
        )

        record_inputs = [goal_name, max_steps, step_label]
        for i, btn in enumerate(step_btns, start=1):
            btn.click(
                lambda gn, ms, sl, n=i: on_record_step(n, gn, ms, sl),
                inputs=record_inputs,
                outputs=ui_outputs,
            )

    return demo


def main():
    os.makedirs(TRAJ_DIR, exist_ok=True)
    os.makedirs(TASKS_DIR, exist_ok=True)
    demo = build_ui()
    demo.launch(
        server_name="0.0.0.0",
        server_port=int(os.environ.get("TEACH_PANEL_PORT", DEFAULT_PORT)),
        show_error=True,
    )


if __name__ == "__main__":
    main()
