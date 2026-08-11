#!/bin/bash

# mavproxy_start.sh
# MAVProxy forwarder: FC -> RPi -> Ground Station + local script access

# ─── CONFIGURATION ───────────────────────────────────────────────
SERIAL_PORT="/dev/serial0"       # FC serial port (change if needed: ttyUSB0, ttyACM0)
BAUD_RATE="115200"                 # Match your FC telemetry baud rate
GS_IP="192.168.1.100"            # Ground station IP (change this!)
GS_PORT="14550"                   # Port the GS connects in on (Mission Planner: UDPCl -> <pi-ip>:14550)
LOCAL_PORT="14551"                # Local loopback port for RPi scripts
# ─────────────────────────────────────────────────────────────────

echo "[INFO] Starting MAVProxy..."
echo "[INFO] Serial:  $SERIAL_PORT @ $BAUD_RATE"
echo "[INFO] GS:      $GS_IP:$GS_PORT"
echo "[INFO] Local:   127.0.0.1:$LOCAL_PORT"

# Check if MAVProxy is installed
if ! command -v mavproxy.py &> /dev/null; then
    echo "[ERROR] mavproxy.py not found. Install with: pip install MAVProxy"
    exit 1
fi

# Check if serial port exists
if [ ! -e "$SERIAL_PORT" ]; then
    echo "[ERROR] Serial port $SERIAL_PORT not found."
    echo "        Available ports:"
    ls /dev/tty{AMA,USB,ACM}* 2>/dev/null || echo "        None found"
    exit 1
fi

mavproxy.py \
    --master="$SERIAL_PORT","$BAUD_RATE" \
    --out=udp:"$GS_IP":"$GS_PORT" \
    --out=udp:127.0.0.1:"$LOCAL_PORT" \
    --streamrate=10 \
    --daemon