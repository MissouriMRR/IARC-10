#!/bin/bash

# mavproxy_start.sh
# MAVProxy forwarder: FC -> RPi -> Ground Station + local script access

# ─── CONFIGURATION ───────────────────────────────────────────────
SERIAL_PORT="/dev/serial0"       # FC serial port (change if needed: ttyUSB0, ttyACM0)
BAUD_RATE="115200"                 # Match your FC telemetry baud rate
GS_IP="192.168.1.100"            # Ground station IP (change this!)
GS_PORT="14550"                   # Port the GS connects in on (Mission Planner: UDPCl -> <pi-ip>:14550)
LOCAL_PORT="14551"                # Local loopback port for RPi scripts
PROJECT_DIR="/home/mrrdt-1/IARC-10"   # uv project whose .venv has MAVProxy installed
# ─────────────────────────────────────────────────────────────────

# Strip stray whitespace so a copy-paste like GS_IP=" 1.2.3.4" doesn't become an
# unroutable "--out=udp: 1.2.3.4:14550".
GS_IP="${GS_IP//[[:space:]]/}"

echo "[INFO] Starting MAVProxy..."
echo "[INFO] Serial:  $SERIAL_PORT @ $BAUD_RATE"
echo "[INFO] GS:      $GS_IP:$GS_PORT"
echo "[INFO] Local:   127.0.0.1:$LOCAL_PORT"

# Locate mavproxy.py. Under systemd we run as root with a minimal PATH that has neither
# the uv venv nor ~/.local/bin, so the venv has to be checked explicitly -- `command -v`
# alone only works from an interactive login shell.
MAVPROXY_BIN="${MAVPROXY_BIN:-}"
if [ -z "$MAVPROXY_BIN" ]; then
    for candidate in \
        "$PROJECT_DIR/.venv/bin/mavproxy.py" \
        "$(dirname "$0")/.venv/bin/mavproxy.py" \
        "/home/mrrdt-1/.local/bin/mavproxy.py" \
        "$(command -v mavproxy.py 2>/dev/null)"
    do
        if [ -n "$candidate" ] && [ -x "$candidate" ]; then
            MAVPROXY_BIN="$candidate"
            break
        fi
    done
fi

if [ -z "$MAVPROXY_BIN" ]; then
    echo "[ERROR] mavproxy.py not found. Looked in:"
    echo "        $PROJECT_DIR/.venv/bin/"
    echo "        /home/mrrdt-1/.local/bin/"
    echo "        \$PATH ($PATH)"
    echo "        Fix: run 'uv sync' in $PROJECT_DIR, or set MAVPROXY_BIN to the full path."
    exit 1
fi
echo "[INFO] Binary: $MAVPROXY_BIN"

# Check if serial port exists
if [ ! -e "$SERIAL_PORT" ]; then
    echo "[ERROR] Serial port $SERIAL_PORT not found."
    echo "        Available ports:"
    ls /dev/tty{AMA,USB,ACM}* 2>/dev/null || echo "        None found"
    exit 1
fi

# exec so systemd tracks MAVProxy itself as the main PID and stop/restart signal it
# directly rather than killing this wrapper and orphaning it on the serial port.
exec "$MAVPROXY_BIN" \
    --master="$SERIAL_PORT","$BAUD_RATE" \
    --out=udp:"$GS_IP":"$GS_PORT" \
    --out=udp:127.0.0.1:"$LOCAL_PORT" \
    --streamrate=10 \
    --daemon
