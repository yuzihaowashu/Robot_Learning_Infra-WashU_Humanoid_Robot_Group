# Drag-and-Teach Web UI

Put the G1 in **ai** balance mode (hand controller: L1+A → L1+UP), then from the repo root:

```bash
bash run_teach_ui.sh
```

Open **http://localhost:7861** (or set `TEACH_PANEL_PORT`). In the UI: **Connect** → **Prepare (forward pose)** → **Record Step N** → **Stop & save step** → **Execute goal**. Do not run `gradio_panel.py` at the same time (shared `rt/arm_sdk`).

## Network (two laptops)

**Laptop that runs the UI and talks to the robot** — plug **Ethernet** into the G1. Set a static IP on that interface (replace `enp3s0` with your adapter name, e.g. from `ip link`):

```bash
sudo ip addr add 192.168.123.222/24 dev enp3s0
sudo ip link set enp3s0 up
ping 192.168.123.164
```

In the UI **Connect** box, enter that interface name if DDS needs it (e.g. `enp3s0`); leave blank if auto-detect works. WiFi on this machine is optional (internet only).

**Another laptop (browser only)** — no Ethernet to the robot. Join the **same WiFi/LAN** as the robot PC, then open `http://<robot-pc-ip>:7861` (find the PC’s WiFi IP with `hostname -I` on the machine running `run_teach_ui.sh`). The UI listens on `0.0.0.0`, so remote browsers work; only the robot PC must be on `192.168.123.x` Ethernet.

Robot onboard IP: `192.168.123.164`. More detail: [docs/teleoperation.md](docs/teleoperation.md#network-topology).
