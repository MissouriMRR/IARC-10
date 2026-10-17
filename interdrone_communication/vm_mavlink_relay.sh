#!/bin/bash

# vm_mavlink_relay.sh <drone-id>
# Runs on the BATMAN VM. Relays one drone's MAVLink stream from the mesh to Mission Planner
# on the Windows host, both directions:
#
#   Pi MAVProxy --bat0--> VM udpin 0.0.0.0:(14550+id) --> Windows <WINDOWS_HOST_IP>:(14550+id)
#
# One instance per drone (mavlink-relay@<id>.service). A single MAVProxy with several
# --master links would treat them as redundant links to one vehicle, not four vehicles.
#
# Overrides go in /etc/default/mavlink-relay (EnvironmentFile in the unit).

DRONE_ID="$1"
if ! [[ "$DRONE_ID" =~ ^[0-9]+$ ]]; then
    echo "[ERROR] Usage: $0 <drone-id>   (e.g. 1 -> UDP 14551)"
    exit 1
fi

# ─── CONFIGURATION ───────────────────────────────────────────────
PORT="$((14550 + DRONE_ID))"
# The VM's default gateway is the Windows host under Hyper-V's Default Switch and VirtualBox
# NAT. For a bridged adapter or VMware NAT (whose gateway is .2, not the host) set
# WINDOWS_HOST_IP explicitly to the Windows address the VM can reach.
WINDOWS_HOST_IP="${WINDOWS_HOST_IP:-$(ip route show default | awk '/default/ {print $3; exit}')}"
MAV_STATE_DIR="${MAV_STATE_DIR:-/var/lib/mavlink-relay/drone$DRONE_ID}"
# ─────────────────────────────────────────────────────────────────

WINDOWS_HOST_IP="${WINDOWS_HOST_IP//[[:space:]]/}"
if [ -z "$WINDOWS_HOST_IP" ]; then
    echo "[ERROR] No default route to guess the Windows host from; set WINDOWS_HOST_IP."
    exit 1
fi

MAVPROXY_BIN="${MAVPROXY_BIN:-$(command -v mavproxy.py 2>/dev/null)}"
if [ -z "$MAVPROXY_BIN" ]; then
    echo "[ERROR] mavproxy.py not found on PATH ($PATH). Install MAVProxy or set MAVPROXY_BIN."
    exit 1
fi

mkdir -p "$MAV_STATE_DIR"

echo "[INFO] Drone $DRONE_ID: mesh udpin 0.0.0.0:$PORT -> Mission Planner $WINDOWS_HOST_IP:$PORT"

# --streamrate=-1 stops this relay requesting stream rates (MAVProxy's default is 4 Hz). The
# Pi's MAVProxy already sets them, and a second requester would override them for the
# flight code too.
exec "$MAVPROXY_BIN" \
    --master=udpin:0.0.0.0:"$PORT" \
    --out=udp:"$WINDOWS_HOST_IP":"$PORT" \
    --streamrate=-1 \
    --state-basedir="$MAV_STATE_DIR" \
    --non-interactive
