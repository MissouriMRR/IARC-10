#!/bin/bash
# B.A.T.M.A.N. mesh node from WSL2 -- replaces the VMware VM (vm-batman-setup.sh).
# The USB WiFi dongle is passed from Windows into WSL with usbipd-win; Windows keeps its own WiFi.
#
# Usage, from a WSL terminal in this folder (re-run after every WSL restart):
#   sudo bash wsl-batman-setup.sh                    # bat0 gets 169.254.97.<NODE_NUMBER>
#   sudo bash wsl-batman-setup.sh 207                # override NODE_NUMBER for this run
#
# One-time Windows setup: Windows 11 + WSL2, and usbipd-win (`winget install usbipd`).
# The first run also downloads the WSL kernel source (~245 MB) to build batman-adv (~1 min);
# later runs reuse the cached module until the WSL kernel version changes.
#
# Give the dongle back to Windows:  usbipd detach --busid <busid>

# ---- Configuration -----------------------------------------------------
NODE_NUMBER=205 # SET TO A NUMBER NO ONE ELSE USES, 205-254 (Pis are 1-4, VMs 201-204)
REGDOMAIN="US"
MESH_SSID="my-batman-mesh"
MESH_FREQ="5200"
MESH_WIDTH="HT20"
MESH_BSSID="02:ca:fe:ca:ca:40"
BATMAN_MTU=1532
DONGLE_VIDPID="0bda:0811"                 # Realtek RTL8811AU (team dongle)
FIRMWARE_FILES="rtw88/rtw8821a_fw.bin"    # not in Ubuntu 24.04's linux-firmware package
FIRMWARE_URL="https://git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git/plain"
# ------------------------------------------------------------------------

# The WSL kernel loads firmware from its own utility-VM filesystem, not the distro's /lib/firmware.
# /mnt/wsl is a shared tmpfs visible at the same path there, so firmware is served from it.
FW_DIR="/mnt/wsl/firmware"
CACHE_DIR="/var/cache/wsl-batman"
USBIPD="/mnt/c/Program Files/usbipd-win/usbipd.exe"
POWERSHELL="/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe"  # sudo drops Windows dirs from PATH
KVER=$(uname -r)
MODULE="$CACHE_DIR/$KVER/batman-adv.ko"

log_message() {
    echo "$(date '+%H:%M:%S') - $1"
}

fail() {
    log_message "ERROR: $1"
    exit 1
}

NODE=${1:-$NODE_NUMBER}
if ! [[ "$NODE" =~ ^[0-9]+$ ]] || [ "$NODE" -lt 205 ] || [ "$NODE" -gt 254 ]; then
    echo "Usage: sudo bash $0 [node-number 205-254]   (default NODE_NUMBER=$NODE_NUMBER)"
    exit 1
fi
NODE_IP="169.254.97.$NODE"

if [ "$EUID" -ne 0 ]; then
    fail "This script must be run as root (use sudo)."
fi
grep -qi microsoft /proc/version || fail "Not running under WSL. On native Linux use vm-batman-setup.sh."
[ -x "$USBIPD" ] || fail "usbipd-win not found at $USBIPD. Install it on Windows: winget install usbipd"

# ---- Packages ----------------------------------------------------------
if ! command -v iw &> /dev/null || ! command -v batctl &> /dev/null || ! command -v arping &> /dev/null \
    || [ ! -f /lib/firmware/regulatory.db ]; then
    log_message "Installing iw, batctl, wireless-regdb, arping..."
    apt-get update -qq && apt-get install -y -qq iw batctl wireless-regdb curl iputils-arping || fail "apt-get install failed"
fi

# ---- Firmware ----------------------------------------------------------
mkdir -p "$FW_DIR"
cp /lib/firmware/regulatory.db /lib/firmware/regulatory.db.p7s "$FW_DIR/"
for fw in $FIRMWARE_FILES; do
    if [ ! -f "$FW_DIR/$fw" ]; then
        mkdir -p "$CACHE_DIR/firmware/$(dirname "$fw")"
        if [ ! -f "$CACHE_DIR/firmware/$fw" ]; then
            log_message "Downloading firmware $fw..."
            curl -fsSL -o "$CACHE_DIR/firmware/$fw" "$FIRMWARE_URL/$fw" || fail "Could not download $fw"
        fi
        mkdir -p "$FW_DIR/$(dirname "$fw")"
        cp "$CACHE_DIR/firmware/$fw" "$FW_DIR/$fw"
    fi
done
echo -n "$FW_DIR" > /sys/module/firmware_class/parameters/path

# ---- batman-adv module -------------------------------------------------
# Microsoft's WSL kernel does not include batman-adv, so build it from the matching source tag.
# Only modules_prepare is needed: the kernel has CONFIG_MODULE_FORCE_LOAD=y, so a module without
# symbol CRCs still loads (it marks the kernel tainted, which is harmless here).
build_module() {
    local tag="linux-msft-wsl-${KVER%%-*}"
    local src="$CACHE_DIR/src"
    log_message "Building batman-adv for $KVER (first run for this kernel)..."
    apt-get install -y -qq build-essential flex bison libssl-dev libelf-dev bc curl || fail "apt-get install failed"
    rm -rf "$src" && mkdir -p "$src"
    log_message "Downloading WSL kernel source $tag (~245 MB)..."
    curl -fsSL "https://github.com/microsoft/WSL2-Linux-Kernel/archive/refs/tags/$tag.tar.gz" \
        | tar xz -C "$src" --strip-components=1 || fail "Could not download kernel source tag $tag"
    (
        cd "$src" || exit 1
        zcat /proc/config.gz > .config
        scripts/config --module BATMAN_ADV --disable LOCALVERSION_AUTO
        make -s olddefconfig
        [ "$(make -s kernelrelease)" = "$KVER" ] || { echo "kernelrelease mismatch"; exit 1; }
        make -s -j"$(nproc)" modules_prepare
        KBUILD_MODPOST_WARN=1 make -s -j"$(nproc)" M=net/batman-adv modules 2> /dev/null
    ) || fail "batman-adv build failed"
    mkdir -p "$(dirname "$MODULE")"
    strip --strip-debug -o "$MODULE" "$src/net/batman-adv/batman-adv.ko" || fail "batman-adv.ko not produced"
    rm -rf "$src"
    log_message "Built $MODULE"
}

modprobe cfg80211 || fail "Failed to load cfg80211"
modprobe bridge || fail "Failed to load bridge"
if ! lsmod | grep -q '^batman_adv'; then
    [ -f "$MODULE" ] || build_module
    insmod "$MODULE" || fail "Failed to load $MODULE"
fi
log_message "Loaded batman-adv"

# ---- Regulatory domain -------------------------------------------------
iw reg set "$REGDOMAIN"
sleep 1
CURRENT_REG=$(iw reg get | grep -m1 '^country' | awk '{print $2}' | tr -d ':')
if [ "$CURRENT_REG" != "$REGDOMAIN" ]; then
    fail "Regulatory domain is '$CURRENT_REG', expected '$REGDOMAIN' (cfg80211 was loaded before the firmware path was set). Run 'wsl --shutdown' in PowerShell, then re-run this script."
fi
log_message "Regulatory domain confirmed: $CURRENT_REG"

# ---- USB WiFi dongle (usbipd) ------------------------------------------
find_iface() {
    ls /sys/class/net/ | grep -E '^wlx' | head -n 1
}

UAIN=$(find_iface)
if [ -z "$UAIN" ]; then
    USB_LINE=$("$USBIPD" list 2>/dev/null | tr -d '\r' | grep -i " $DONGLE_VIDPID ")
    [ -n "$USB_LINE" ] || fail "WiFi dongle $DONGLE_VIDPID not plugged into Windows (check 'usbipd list')."
    BUSID=$(echo "$USB_LINE" | awk '{print $1}')

    if echo "$USB_LINE" | grep -q 'Not shared$'; then
        log_message "Sharing dongle $BUSID with WSL (one-time; accept the Windows admin prompt)..."
        "$POWERSHELL" -NoProfile -Command \
            "Start-Process usbipd -ArgumentList 'bind','--busid','$BUSID' -Verb RunAs -Wait" \
            || fail "usbipd bind was not run. Run 'usbipd bind --busid $BUSID' in an admin PowerShell."
    elif echo "$USB_LINE" | grep -q 'Attached$'; then
        # Attached before the firmware path was set, so the driver probe failed. Re-attach.
        "$USBIPD" detach --busid "$BUSID" > /dev/null 2>&1
        sleep 2
    fi

    log_message "Attaching dongle $BUSID to WSL..."
    "$USBIPD" attach --wsl --busid "$BUSID" > /dev/null 2>&1 || fail "usbipd attach failed for $BUSID"
    for i in $(seq 1 15); do
        UAIN=$(find_iface)
        [ -n "$UAIN" ] && break
        sleep 1
    done
    [ -n "$UAIN" ] || fail "Dongle attached but no wlx* interface appeared. Check: dmesg | tail"
fi
log_message "Found USB WiFi adapter: $UAIN"

# ---- Configure the wireless interface ----------------------------------
# Re-running: drop the old bat0 so the interface can be re-added cleanly
ip link del bat0 2> /dev/null
if command -v nmcli &> /dev/null; then
    nmcli device set "$UAIN" managed no 2> /dev/null
fi

ip link set "$UAIN" down
iw dev "$UAIN" set type ibss || fail "Failed to set $UAIN to IBSS mode"
if ! ip link set "$UAIN" mtu "$BATMAN_MTU"; then
    log_message "WARNING: Could not set MTU $BATMAN_MTU on $UAIN (driver may not support it)"
fi
ip link set "$UAIN" up
sleep 2

if ! iw dev "$UAIN" ibss join "$MESH_SSID" "$MESH_FREQ" "$MESH_WIDTH" fixed-freq "$MESH_BSSID"; then
    fail "IBSS join failed on $MESH_FREQ MHz. Check 'iw reg get' for NO-IR on this band."
fi
log_message "Joined mesh: $MESH_SSID @ $MESH_FREQ MHz ($MESH_BSSID)"
sleep 2

batctl if add "$UAIN" || fail "Failed to add $UAIN to batman-adv"
ip link set bat0 up || fail "Failed to bring up bat0"

# Duplicate address check: give BATMAN a few seconds to learn the mesh, then ask if anyone
# already answers for our address (arping -D exits non-zero when someone replies).
sleep 5
if ! arping -D -q -c 3 -w 4 -I bat0 "$NODE_IP"; then
    ip link del bat0
    fail "$NODE_IP is already in use on the mesh. Pick another NODE_NUMBER (205-254)."
fi
ip addr add "$NODE_IP/16" dev bat0 || fail "Failed to assign $NODE_IP to bat0"

log_message "B.A.T.M.A.N. mesh setup complete! Interface: $UAIN, IP: $NODE_IP"
log_message "Waiting for neighbors..."
sleep 5
batctl n
exit 0
