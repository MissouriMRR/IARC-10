#!/bin/bash
# B.A.T.M.A.N. Mesh Network Setup Script
# Finds USB WiFi adapter and configures it for ad-hoc mesh networking

# Log file for debugging
LOG_FILE="/var/log/batman-mesh-setup.log"

log_message() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" | tee -a "$LOG_FILE"
}

log_message "Starting B.A.T.M.A.N. mesh setup..."

# Check if uv is installed
if ! command -v uv &> /dev/null; then
    log_message "ERROR: uv command not found! Please install uv first."
    exit 1
fi

# Open/activate uv environment
log_message "Activating uv environment..."
if [ -f ".venv/bin/activate" ]; then
    source .venv/bin/activate
    log_message "uv virtual environment activated"
elif [ -f "venv/bin/activate" ]; then
    source venv/bin/activate
    log_message "uv virtual environment activated"
else
    log_message "WARNING: No virtual environment found. Running uv sync..."
    uv sync
    if [ $? -eq 0 ] && [ -f ".venv/bin/activate" ]; then
        source .venv/bin/activate
        log_message "uv environment created and activated"
    else
        log_message "ERROR: Failed to create uv environment"
        exit 1
    fi
fi

# Wait for system to fully initialize
sleep 10

# Find USB WiFi adapter interface
# This looks for wireless interfaces that are NOT the built-in WiFi (usually wlan0)
UAIN=""

# First, try to find interfaces matching wlx* pattern (USB adapters with MAC-based names)
for iface in $(ls /sys/class/net/ | grep -E '^wlx'); do
    UAIN=$iface
    break
done

# If not found, look for wlan1, wlan2, etc (excluding wlan0)
if [ -z "$UAIN" ]; then
    for iface in $(ls /sys/class/net/ | grep -E '^wlan[0-9]+$'); do
        # Check if it's a USB device by examining the device path
        if [[ -L "/sys/class/net/$iface/device" ]]; then
            device_path=$(readlink -f "/sys/class/net/$iface/device")
            if [[ $device_path == *"usb"* ]]; then
                UAIN=$iface
                break
            fi
        fi
    done
fi

# Alternative method: find any wireless interface except wlan0
if [ -z "$UAIN" ]; then
    UAIN=$(ls /sys/class/net/ | grep -E '^wlan[1-9][0-9]*$' | head -n 1)
fi

if [ -z "$UAIN" ]; then
    log_message "ERROR: No USB WiFi adapter found!"
    exit 1
fi

log_message "Found USB WiFi adapter: $UAIN"

# Load batman-adv kernel module
modprobe batman-adv
if [ $? -ne 0 ]; then
    log_message "ERROR: Failed to load batman-adv module"
    exit 1
fi
log_message "Loaded batman-adv module"

# Configure the interface
log_message "Setting $UAIN to unmanaged mode..."
nmcli device set "$UAIN" managed no

log_message "Bringing $UAIN down..."
ip link set "$UAIN" down

log_message "Setting $UAIN to ad-hoc (IBSS) mode..."
iw dev "$UAIN" set type ibss

log_message "Bringing $UAIN up..."
ip link set "$UAIN" up

# Small delay to ensure interface is ready
sleep 2

log_message "Joining ad-hoc mesh network..."
iw dev "$UAIN" ibss join my-batman-mesh 5200 HT20 fixed-freq 02:ca:fe:ca:ca:40

# Small delay before adding to batman
sleep 2

log_message "Adding $UAIN to batman-adv..."
batctl if add "$UAIN"

log_message "Bringing bat0 interface up..."
ip link set bat0 up

# Extract Pi number from hostname (mrrdt-#)
HOSTNAME=$(hostname)
PI_NUMBER=$(echo "$HOSTNAME" | grep -oP 'mrrdt-\K\d+')

if [ -z "$PI_NUMBER" ]; then
    log_message "WARNING: Could not extract Pi number from hostname '$HOSTNAME'"
    log_message "Expected format: mrrdt-# (e.g., mrrdt-1, mrrdt-42)"
    log_message "Defaulting to IP 169.254.97.99"
    PI_NUMBER=99
fi

# Set IP address based on Pi number
NODE_IP="169.254.97.$PI_NUMBER"
log_message "Setting IP address $NODE_IP on bat0..."
ip addr add "$NODE_IP/24" dev bat0

log_message "B.A.T.M.A.N. mesh setup complete!"
log_message "Interface: $UAIN, IP: $NODE_IP"

sleep 10

# Change to the Interdrone directory
cd /home/mrrdt-$PI_NUMBER/IARC-10/Interdrone-Communication

log_message "Changed to directory"

# Create/Update config.json with correct IPs and selfId
log_message "Updating config.json..."
uv run python -c "
import json, sys
try:
    with open('config.json','r') as f: d=json.load(f)
    for k in d.get('drones',{}): d['drones'][k]['ip'] = f'169.254.97.{k}'
    if 'localInfo' not in d: d['localInfo']={}
    d['localInfo']['selfId'] = '$PI_NUMBER'
    with open('config.json','w') as f: json.dump(d,f,indent=2)
    print('Successfully updated config.json')
except Exception as e: print(f'Error: {e}'); sys.exit(1)
"

if [ $? -eq 0 ]; then
    log_message "Config updated successfully"
else
    log_message "ERROR: Failed to update config.json"
fi

uv run main.py -i $PI_NUMBER

exit 0