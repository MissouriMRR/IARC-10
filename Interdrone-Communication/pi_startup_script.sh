#!/bin/bash

# B.A.T.M.A.N. Mesh Network Setup Script
# Finds USB WiFi adapter and configures it for ad-hoc mesh networking

# Log file for debugging
LOG_FILE="/var/log/batman-mesh-setup.log"

log_message() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" | tee -a "$LOG_FILE"
}

log_message "Starting B.A.T.M.A.N. mesh setup..."

# Wait for system to fully initialize
sleep 10

# Find USB WiFi adapter interface
# This looks for wireless interfaces that are NOT the built-in WiFi (usually wlan0)
UAIN=""

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

# Set IP address - REPLACE X with your node number (1-254)
NODE_IP="169.254.97.X"
log_message "Setting IP address $NODE_IP on bat0..."
ip addr add "$NODE_IP/24" dev bat0

log_message "B.A.T.M.A.N. mesh setup complete!"
log_message "Interface: $UAIN, IP: $NODE_IP"

exit 0
