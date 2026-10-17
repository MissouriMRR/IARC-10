#!/bin/bash

# drone_flight_start.sh
# Runs MAVProxy and the flight code together so they can't fight over the FC serial port.
#
#   FC --serial0--> MAVProxy --udp 127.0.0.1:14551--> run.py (dronekit, udpin)
#                       \--udp over bat0--> BATMAN VM 169.254.97.201:(14550+id) --> Mission Planner
#
# MAVProxy is the only thing that opens the serial port. With MAVLINK_FORWARD=0 MAVProxy is
# skipped and run.py opens /dev/serial0 itself (the pre-merge behaviour), as a fallback for
# when the mesh/MAVProxy path misbehaves.
#
# Every setting below can be overridden from /etc/default/drone-flight (EnvironmentFile in
# drone-flight.service) or the environment when running by hand.

# ─── CONFIGURATION ───────────────────────────────────────────────
# Pi number from the hostname (mrrdt-#), same convention as batman-mesh-setup.sh.
PI_NUMBER="${PI_NUMBER:-$(hostname | grep -oP 'mrrdt-\K\d+')}"
# MAVProxy --master. Point it at a SITL (tcp:<host>:5762) to bench-test without an FC.
FC_MASTER="${FC_MASTER:-/dev/serial0,115200}"
# BATMAN VM's bat0 address (PI_NUMBER in vm-batman-setup.sh).
GCS_RELAY_IP="${GCS_RELAY_IP:-169.254.97.201}"
GCS_PORT="${GCS_PORT:-$((14550 + PI_NUMBER))}"
LOCAL_PORT="${LOCAL_PORT:-14551}"
STREAMRATE="${STREAMRATE:-10}"
MAVLINK_FORWARD="${MAVLINK_FORWARD:-1}"
PROJECT_DIR="${PROJECT_DIR:-/home/$(hostname)/IARC-10}"
# MAVProxy writes mav.tlog/mav.parm here rather than into the repo checkout.
MAV_STATE_DIR="${MAV_STATE_DIR:-/home/$(hostname)/mav}"
UV_BIN="${UV_BIN:-$(command -v uv 2>/dev/null || echo /usr/local/bin/uv)}"
# ─────────────────────────────────────────────────────────────────

# Strip stray whitespace so a copy-paste like GCS_RELAY_IP=" 1.2.3.4" doesn't become an
# unroutable "--out=udp: 1.2.3.4:14551".
GCS_RELAY_IP="${GCS_RELAY_IP//[[:space:]]/}"

if [ -z "$PI_NUMBER" ]; then
    echo "[ERROR] Could not get the drone id from hostname '$(hostname)' (expected mrrdt-#)."
    echo "        Set PI_NUMBER in /etc/default/drone-flight."
    exit 1
fi

cd "$PROJECT_DIR" || { echo "[ERROR] $PROJECT_DIR not found."; exit 1; }

if [ "$MAVLINK_FORWARD" = "0" ]; then
    echo "[INFO] MAVLINK_FORWARD=0: no MAVProxy, flight code opens the FC directly."
    exec "$UV_BIN" run --no-sync run.py -i "$PI_NUMBER"
fi

# Locate mavproxy.py. Under systemd we run as root with a minimal PATH that has neither
# the uv venv nor ~/.local/bin, so the venv has to be checked explicitly -- `command -v`
# alone only works from an interactive login shell.
MAVPROXY_BIN="${MAVPROXY_BIN:-}"
if [ -z "$MAVPROXY_BIN" ]; then
    for candidate in \
        "$PROJECT_DIR/.venv/bin/mavproxy.py" \
        "/home/$(hostname)/.local/bin/mavproxy.py" \
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
    echo "        /home/$(hostname)/.local/bin/"
    echo "        \$PATH ($PATH)"
    echo "        Fix: run 'uv sync' in $PROJECT_DIR, set MAVPROXY_BIN to the full path,"
    echo "        or set MAVLINK_FORWARD=0 to fly without telemetry forwarding."
    exit 1
fi

# Only a device path needs to exist up front; tcp:/udp: masters are checked by MAVProxy.
SERIAL_DEV="${FC_MASTER%%,*}"
if [[ "$SERIAL_DEV" == /dev/* ]] && [ ! -e "$SERIAL_DEV" ]; then
    echo "[ERROR] Serial port $SERIAL_DEV not found."
    echo "        Available ports:"
    ls /dev/tty{AMA,USB,ACM}* 2>/dev/null || echo "        None found"
    exit 1
fi

mkdir -p "$MAV_STATE_DIR"

echo "[INFO] Drone:    $PI_NUMBER"
echo "[INFO] MAVProxy: $MAVPROXY_BIN"
echo "[INFO] Master:   $FC_MASTER"
echo "[INFO] GCS:      $GCS_RELAY_IP:$GCS_PORT (via BATMAN VM)"
echo "[INFO] Local:    127.0.0.1:$LOCAL_PORT (flight code)"

# MAVProxy runs in a respawn loop rather than as a peer the script exits on: if it crashes
# mid-flight only MAVProxy restarts, and dronekit's reconnect layer rides through the gap
# instead of the whole flight process being torn down.
(
    while true; do
        "$MAVPROXY_BIN" \
            --master="$FC_MASTER" \
            --out=udp:127.0.0.1:"$LOCAL_PORT" \
            --out=udp:"$GCS_RELAY_IP":"$GCS_PORT" \
            --streamrate="$STREAMRATE" \
            --state-basedir="$MAV_STATE_DIR" \
            --non-interactive
        echo "[WARN] MAVProxy exited ($?); restarting in 2 s"
        sleep 2
    done
) &
MAVPROXY_LOOP_PID=$!

cleanup() {
    # Kill MAVProxy before its loop, then the loop, so it doesn't get a chance to respawn.
    pkill -P "$MAVPROXY_LOOP_PID" 2>/dev/null
    kill "$MAVPROXY_LOOP_PID" 2>/dev/null
}
trap cleanup EXIT
trap 'exit 143' TERM INT

# udpin: MAVProxy's --out is the sender, so dronekit binds and MAVProxy dials it.
# Not exec'd, so the EXIT trap above still runs when the flight code exits.
REAL_MAVLINK_ADDRESS="udpin:127.0.0.1:$LOCAL_PORT" \
    "$UV_BIN" run --no-sync run.py -i "$PI_NUMBER"
exit $?
