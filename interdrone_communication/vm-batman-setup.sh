#!/bin/bash
# B.A.T.M.A.N. Mesh Network Setup Script
# Finds USB WiFi adapter and configures it for ad-hoc mesh networking

# ---- Configuration -----------------------------------------------------
LOG_FILE="/var/log/batman-setup.log"
REGDOMAIN="US"
MESH_SSID="my-batman-mesh"
MESH_FREQ="5200"
MESH_WIDTH="HT20"
MESH_BSSID="02:ca:fe:ca:ca:40"
BATMAN_MTU=1532
PI_NUMBER=201 # SET PI NUM TO 201, 202, 203, or 204
# ------------------------------------------------------------------------

log_message() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" | tee -a "$LOG_FILE"
}

fail() {
    log_message "ERROR: $1"
    exit 1
}

# Must be root: iw reg set, modprobe, and /var/log writes all require it
if [ "$EUID" -ne 0 ]; then
    echo "ERROR: This script must be run as root (use sudo)."
    exit 1
fi

log_message "Starting B.A.T.M.A.N. mesh setup..."

# Check if uv is installed
if ! command -v uv &> /dev/null; then
    fail "uv command not found! Please install uv first."
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
        fail "Failed to create uv environment"
    fi
fi

# Wait for system to fully initialize
sleep 10

# ---- Regulatory domain -------------------------------------------------
# World domain (country 00) marks 5170-5250 MHz as NO-IR, which makes
# IBSS join fail with -22. This must be set and verified before joining.
log_message "Setting regulatory domain to $REGDOMAIN..."
iw reg set "$REGDOMAIN"
sleep 1

CURRENT_REG=$(iw reg get | grep -m1 '^country' | awk '{print $2}' | tr -d ':')
if [ "$CURRENT_REG" != "$REGDOMAIN" ]; then
    fail "Regulatory domain is '$CURRENT_REG', expected '$REGDOMAIN'. 5 GHz IBSS will fail. Install wireless-regdb and set 'options cfg80211 ieee80211_regdom=$REGDOMAIN' in /etc/modprobe.d/cfg80211.conf"
fi
log_message "Regulatory domain confirmed: $CURRENT_REG"

# ---- Find USB WiFi adapter ---------------------------------------------
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
    fail "No USB WiFi adapter found!"
fi

log_message "Found USB WiFi adapter: $UAIN"

# Load batman-adv kernel module
if ! modprobe batman-adv; then
    fail "Failed to load batman-adv module"
fi
log_message "Loaded batman-adv module"

# ---- Configure the wireless interface ----------------------------------
log_message "Setting $UAIN to unmanaged mode..."
nmcli device set "$UAIN" managed no

log_message "Bringing $UAIN down..."
ip link set "$UAIN" down

log_message "Setting $UAIN to ad-hoc (IBSS) mode..."
if ! iw dev "$UAIN" set type ibss; then
    fail "Failed to set $UAIN to IBSS mode"
fi

# batman-adv adds its own header, so the underlying interface needs headroom
# above the standard 1500 MTU or frames will fragment.
log_message "Setting MTU $BATMAN_MTU on $UAIN..."
if ! ip link set "$UAIN" mtu "$BATMAN_MTU"; then
    log_message "WARNING: Could not set MTU $BATMAN_MTU on $UAIN (driver may not support it)"
fi

log_message "Bringing $UAIN up..."
ip link set "$UAIN" up

# Small delay to ensure interface is ready
sleep 2

log_message "Joining ad-hoc mesh network..."
if ! iw dev "$UAIN" ibss join "$MESH_SSID" "$MESH_FREQ" "$MESH_WIDTH" fixed-freq "$MESH_BSSID"; then
    fail "IBSS join failed on $MESH_FREQ MHz. Check 'iw reg get' for NO-IR on this band, or try 2412 MHz."
fi
log_message "Joined mesh: $MESH_SSID @ $MESH_FREQ MHz ($MESH_BSSID)"

# Small delay before adding to batman
sleep 2

log_message "Adding $UAIN to batman-adv..."
if ! batctl if add "$UAIN"; then
    fail "Failed to add $UAIN to batman-adv"
fi

log_message "Bringing bat0 interface up..."
if ! ip link set bat0 up; then
    fail "Failed to bring up bat0"
fi

# ---- Address assignment ------------------------------------------------
NODE_IP="169.254.97.$PI_NUMBER"

# Flush first so re-running the script doesn't fail with "Address already assigned"
log_message "Flushing existing addresses on bat0..."
ip addr flush dev bat0

log_message "Setting IP address $NODE_IP on bat0..."
if ! ip addr add "$NODE_IP/16" dev bat0; then
    fail "Failed to assign $NODE_IP to bat0"
fi

# Gratuitous ARP so peers learn our address without waiting for resolution
if command -v arping &> /dev/null; then
    arping -c 3 -I bat0 "$NODE_IP" > /dev/null 2>&1
else
    log_message "WARNING: arping not found (install iputils-arping) - skipping gratuitous ARP"
fi

log_message "B.A.T.M.A.N. mesh setup complete!"
log_message "Interface: $UAIN, IP: $NODE_IP, Regdomain: $CURRENT_REG"

sleep 10

# Change to the Interdrone directory
#cd /home/mrrdt-iarc-desk-1/IARC-Dev/IARC-10/Interdrone-Communication

#log_message "Changed to directory"

#uv run main.py -i $PI_NUMBER

exit 0