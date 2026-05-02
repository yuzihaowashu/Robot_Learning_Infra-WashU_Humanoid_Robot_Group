#!/usr/bin/env bash
# Friendly launcher for the G1 XR teleoperation Gradio panel.
#
# This does local preflight checks, prints the exact PICO WebXR URL, ensures
# a local HTTPS certificate exists, then starts gradio_panel.py in conda env tv.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_PC2_IP="${ROBOT_PC2_IP:-192.168.123.164}"
ROBOT_SUBNET_PREFIX="${ROBOT_SUBNET_PREFIX:-192.168.123.}"
PICO_PORT="${PICO_PORT:-8012}"
GRADIO_PORT="${GRADIO_PORT:-7860}"
TELEIMAGER_CONFIG_PORT="${TELEIMAGER_CONFIG_PORT:-60000}"
TELEIMAGER_HEAD_ZMQ_PORT="${TELEIMAGER_HEAD_ZMQ_PORT:-55555}"
CERT_DIR="${XR_TELEOP_CERT_DIR:-$HOME/.config/xr_teleoperate}"
CERT_FILE="${XR_TELEOP_CERT:-$CERT_DIR/cert.pem}"
KEY_FILE="${XR_TELEOP_KEY:-$CERT_DIR/key.pem}"

usage() {
    cat <<EOF
Usage: bash run_xr_session.sh [--no-cert-generate]

Starts the XR teleoperation control panel and prints the PICO browser URL.

Environment overrides:
  ROBOT_PC2_IP        Default: 192.168.123.164
  PICO_PORT           Default: 8012
  GRADIO_PORT         Default: 7860
  XR_TELEOP_CERT      Default: ~/.config/xr_teleoperate/cert.pem
  XR_TELEOP_KEY       Default: ~/.config/xr_teleoperate/key.pem

EOF
}

GENERATE_CERT=1
for arg in "$@"; do
    case "$arg" in
        --no-cert-generate) GENERATE_CERT=0 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $arg" >&2; usage; exit 1 ;;
    esac
done

prompt_kill_existing_services() {
    mapfile -t pids < <(pgrep -f "gradio_panel.py|teleop_hand_and_arm.py" 2>/dev/null || true)
    if [ "${#pids[@]}" -eq 0 ]; then
        return 0
    fi

    echo "Existing XR service processes are running:"
    for pid in "${pids[@]}"; do
        ps -p "$pid" -o pid=,cmd= 2>/dev/null || true
    done
    echo ""
    read -r -p "Kill existing XR service processes before starting a new one? [y/N] " reply
    case "$reply" in
        y|Y|yes|YES)
            kill -TERM "${pids[@]}" 2>/dev/null || true
            sleep 2
            for pid in "${pids[@]}"; do
                if kill -0 "$pid" 2>/dev/null; then
                    kill -KILL "$pid" 2>/dev/null || true
                fi
            done
            echo "Old XR service processes stopped."
            ;;
        *)
            echo "Aborted. Existing service was left running."
            exit 1
            ;;
    esac
}

detect_ips() {
    ip -4 -o addr show scope global 2>/dev/null | while read -r _idx _ifname _fam cidr _rest; do
        printf '%s\n' "${cidr%%/*}"
    done
}

pick_pico_ip() {
    local first_ip=""
    local ip_addr
    while read -r ip_addr; do
        [ -n "$ip_addr" ] || continue
        [ -n "$first_ip" ] || first_ip="$ip_addr"
        if [[ "$ip_addr" != ${ROBOT_SUBNET_PREFIX}* ]]; then
            printf '%s\n' "$ip_addr"
            return 0
        fi
    done < <(detect_ips)
    printf '%s\n' "${first_ip:-<PC-WiFi-IP>}"
}

port_listening() {
    local port="$1"
    python - "$port" <<'PY'
import socket
import sys

port = int(sys.argv[1])
sock = socket.socket()
sock.settimeout(0.2)
try:
    sock.connect(("127.0.0.1", port))
except OSError:
    sys.exit(1)
else:
    sys.exit(0)
finally:
    sock.close()
PY
}

ensure_cert() {
    if [[ "$PICO_IP" == \<* ]]; then
        echo "WARNING: no usable WiFi IP detected; skip certificate generation."
        return 0
    fi
    if [ -f "$CERT_FILE" ] && [ -f "$KEY_FILE" ]; then
        local cert_text
        cert_text="$(openssl x509 -in "$CERT_FILE" -noout -text 2>/dev/null || true)"
        if [[ "$cert_text" == *"IP Address:${PICO_IP}"* || "$cert_text" == *"IP:${PICO_IP}"* ]]; then
            return 0
        fi
        if [ "$GENERATE_CERT" -eq 0 ]; then
            echo "WARNING: XR HTTPS certificate exists but may not include current WiFi IP: $PICO_IP"
            return 0
        fi
        local stamp
        stamp="$(date +%Y%m%d_%H%M%S)"
        mv "$CERT_FILE" "${CERT_FILE}.bak.${stamp}"
        mv "$KEY_FILE" "${KEY_FILE}.bak.${stamp}"
        echo "Existing XR certificate did not match $PICO_IP; backed it up and will regenerate."
    fi
    if [ "$GENERATE_CERT" -eq 0 ]; then
        echo "WARNING: XR HTTPS cert/key not found:"
        echo "  $CERT_FILE"
        echo "  $KEY_FILE"
        return 0
    fi
    if ! command -v openssl >/dev/null 2>&1; then
        echo "WARNING: openssl not found; cannot generate XR HTTPS certificate."
        return 0
    fi

    mkdir -p "$CERT_DIR"
    echo "Generating local XR HTTPS certificate for $PICO_IP ..."
    openssl req -x509 -newkey rsa:2048 -nodes \
        -keyout "$KEY_FILE" \
        -out "$CERT_FILE" \
        -days 365 \
        -subj "/CN=xr-teleoperate" \
        -addext "subjectAltName=DNS:localhost,IP:127.0.0.1,IP:${PICO_IP}" >/dev/null 2>&1
    echo "Created:"
    echo "  $CERT_FILE"
    echo "  $KEY_FILE"
    echo "If PICO still refuses Enter VR, open the URL once and accept/trust this certificate."
}

prompt_kill_existing_services

PICO_IP="$(pick_pico_ip)"
PICO_URL="https://${PICO_IP}:${PICO_PORT}/?ws=wss://${PICO_IP}:${PICO_PORT}"
GRADIO_URL="http://${PICO_IP}:${GRADIO_PORT}"

echo "=========================================="
echo "  G1 XR Teleoperation Session"
echo "=========================================="
echo "Detected PC IPs:"
detect_ips | sed 's/^/  - /'
echo ""
echo "Use this in PICO Browser after Launch Teleop:"
echo "  $PICO_URL"
echo ""
echo "Gradio panel:"
echo "  $GRADIO_URL"
echo ""

ensure_cert

if ping -c 1 -W 1 "$ROBOT_PC2_IP" >/dev/null 2>&1; then
    echo "PC2 reachable: $ROBOT_PC2_IP"
    if port_listening_remote="$(
        python - "$ROBOT_PC2_IP" "$TELEIMAGER_CONFIG_PORT" "$TELEIMAGER_HEAD_ZMQ_PORT" <<'PY'
import socket
import sys

host = sys.argv[1]
ports = [int(p) for p in sys.argv[2:]]
ok = []
for port in ports:
    sock = socket.socket()
    sock.settimeout(0.4)
    try:
        sock.connect((host, port))
    except OSError:
        ok.append(False)
    else:
        ok.append(True)
    finally:
        sock.close()
print(" ".join("open" if x else "closed" for x in ok))
PY
    )"; then
        set -- $port_listening_remote
        if [ "${1:-closed}" = "open" ] && [ "${2:-closed}" = "open" ]; then
            echo "PC2 teleimager is listening: config=$TELEIMAGER_CONFIG_PORT, head_zmq=$TELEIMAGER_HEAD_ZMQ_PORT"
        else
            echo "WARNING: PC2 is reachable but teleimager ports are not open:"
            echo "  config $TELEIMAGER_CONFIG_PORT: ${1:-closed}"
            echo "  head ZMQ $TELEIMAGER_HEAD_ZMQ_PORT: ${2:-closed}"
            echo "Start teleimager on PC2:"
            echo "  ssh unitree@$ROBOT_PC2_IP"
            echo "  conda activate teleimager && teleimager-server"
        fi
    fi
else
    echo "WARNING: PC2 not reachable: $ROBOT_PC2_IP"
    echo "Start/check teleimager manually:"
    echo "  ssh unitree@$ROBOT_PC2_IP"
    echo "  conda activate teleimager && teleimager-server"
fi

if port_listening "$PICO_PORT"; then
    echo "TeleVuer port $PICO_PORT is already listening."
else
    echo "TeleVuer port $PICO_PORT is not listening yet; this is expected until you click Launch Teleop."
fi

echo ""
echo "Starting Gradio panel..."
cd "$ROOT_DIR"
eval "$(conda shell.bash hook 2>/dev/null)"
conda activate tv
export XR_TELEOP_CERT="$CERT_FILE"
export XR_TELEOP_KEY="$KEY_FILE"
exec python gradio_panel.py
