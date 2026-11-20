#!/bin/bash
# B.A.T.M.A.N. mesh node from WSL2 -- replaces the VMware VM (vm-batman-setup.sh).
# The USB WiFi dongle is passed from Windows into WSL with usbipd-win; Windows keeps its own WiFi.
#
# Usage, from a WSL terminal in this folder (re-run after every WSL restart):
#   sudo bash wsl-batman-setup.sh -n 207             # bat0 gets 169.254.97.207 (pick a number nobody else uses)
#   sudo bash wsl-batman-setup.sh -n 207 --rebuild   # also discard the cached module and rebuild it
#
# One-time Windows setup: Windows 11 + WSL2, and usbipd-win (`winget install usbipd`).
# The first run also downloads the WSL kernel source (~245 MB) to build batman-adv (~1 min);
# later runs reuse the cached module until the WSL kernel version changes.
#
# Give the dongle back to Windows:  usbipd detach --busid <busid>

# ---- Configuration -----------------------------------------------------
NODE_NUMBER=""   # optional default; normally leave empty and pass -n <number> on the command line
NODE_MIN=201     # allowed node numbers (Pis are 1-4)
NODE_MAX=254
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
BUILD_REV=3   # bump when the build recipe changes; modules cached by older versions of this script are then rebuilt
MODULE="$CACHE_DIR/$KVER-r$BUILD_REV/batman-adv.ko"
SRC_DIR="$CACHE_DIR/src"
FORCE_REBUILD=${FORCE_REBUILD:-0}

log_message() {
    echo "$(date '+%H:%M:%S') - $1"
}

fail() {
    log_message "ERROR: $1"
    exit 1
}

# Print what is needed to debug a failed module build or load, so nobody has to dig through the cache
show_build_diagnostics() {
    if [ -s "$CACHE_DIR/config-drift.diff" ]; then
        log_message "olddefconfig changed these options vs. the running kernel's config:"
        head -20 "$CACHE_DIR/config-drift.diff"
    elif [ -f "$CACHE_DIR/config.used" ]; then
        log_message "Config drift: none (built with the running kernel's config unchanged)"
    fi
    if [ -s "$CACHE_DIR/build.log" ]; then
        log_message "Last lines of $CACHE_DIR/build.log:"
        tail -15 "$CACHE_DIR/build.log"
    fi
}

usage() {
    echo "Usage: sudo bash $0 -n <node-number $NODE_MIN-$NODE_MAX> [--rebuild]"
    echo "  -n, --node N   node number for this machine: bat0 gets 169.254.97.N (must be unique on the mesh)"
    echo "  --rebuild      discard the cached batman-adv module and rebuild it"
}

NODE=$NODE_NUMBER
while [ $# -gt 0 ]; do
    case "$1" in
        -n|--node)  [ $# -ge 2 ] || { usage; exit 1; }; NODE="$2"; shift 2 ;;
        --node=*)   NODE="${1#*=}"; shift ;;
        --rebuild)  FORCE_REBUILD=1; shift ;;
        -h|--help)  usage; exit 0 ;;
        [0-9]*)     NODE="$1"; shift ;;    # a bare number still works
        *)          echo "Unknown option: $1"; usage; exit 1 ;;
    esac
done
if ! [[ "$NODE" =~ ^[0-9]+$ ]] || [ "$NODE" -lt "$NODE_MIN" ] || [ "$NODE" -gt "$NODE_MAX" ]; then
    [ -z "$NODE" ] && echo "You need to give this machine a node number." || echo "Invalid node number: $NODE"
    usage
    exit 1
fi
NODE_IP="169.254.97.$NODE"

if [ "$EUID" -ne 0 ]; then
    fail "This script must be run as root (use sudo)."
fi
grep -qi microsoft /proc/version || fail "Not running under WSL. On native Linux use vm-batman-setup.sh."
[ -x "$USBIPD" ] || fail "usbipd-win not found at $USBIPD. Install it on Windows: winget install usbipd"

# ---- Packages ----------------------------------------------------------
if ! command -v iw &> /dev/null || ! command -v batctl &> /dev/null \
    || [ ! -f /lib/firmware/regulatory.db ]; then
    log_message "Installing iw, batctl, wireless-regdb..."
    apt-get update -qq && apt-get install -y -qq iw batctl wireless-regdb curl || fail "apt-get install failed"
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
#
# Debug files written to $CACHE_DIR:
#   config.running      the running kernel's config (/proc/config.gz), untouched
#   config.used         the config the module was actually built with
#   config-drift.diff   options that olddefconfig changed (other than the ones we set on purpose)
#   build.log           stderr of the module build
build_module() {
    local tag="linux-msft-wsl-${KVER%%-*}"
    log_message "Building batman-adv for $KVER..."
    # dwarves provides pahole: without it olddefconfig silently drops CONFIG_DEBUG_INFO_BTF(_MODULES),
    # which shrinks struct module and makes insmod fail with "Invalid module format".
    apt-get install -y -qq build-essential flex bison libssl-dev libelf-dev bc curl dwarves || fail "apt-get install failed"
    # Optional: gcc plugin headers, so CONFIG_GCC_PLUGINS stays as in the running kernel
    apt-get install -y -qq "gcc-$(gcc -dumpversion | cut -d. -f1)-plugin-dev" > /dev/null 2>&1 || true

    # Reuse the extracted source if it is for this exact tag (saves a 245 MB download on retries)
    if [ "$(cat "$SRC_DIR/.wsl-batman-tag" 2> /dev/null)" != "$tag" ] || [ ! -f "$SRC_DIR/Makefile" ]; then
        rm -rf "$SRC_DIR" && mkdir -p "$SRC_DIR"
        log_message "Downloading WSL kernel source $tag (~245 MB)..."
        curl -fsSL "https://github.com/microsoft/WSL2-Linux-Kernel/archive/refs/tags/$tag.tar.gz" \
            | tar xz -C "$SRC_DIR" --strip-components=1 || fail "Could not download kernel source tag $tag"
        echo "$tag" > "$SRC_DIR/.wsl-batman-tag"
    else
        log_message "Reusing cached kernel source $tag"
    fi

    rm -f "$CACHE_DIR/config-drift.diff" "$CACHE_DIR/config.used"
    : > "$CACHE_DIR/build.log"
    (
        cd "$SRC_DIR" || exit 1
        # Start from a clean tree so no stale objects or config leak into the build
        make -s mrproper
        zcat /proc/config.gz > .config || exit 1
        cp .config "$CACHE_DIR/config.running"
        scripts/config --module BATMAN_ADV --disable LOCALVERSION_AUTO
        make -s olddefconfig
        [ "$(make -s kernelrelease)" = "$KVER" ] || { echo "kernelrelease mismatch: $(make -s kernelrelease) != $KVER"; exit 1; }
        cp .config "$CACHE_DIR/config.used"

        # Report any option olddefconfig changed besides the ones we set on purpose
        diff <(grep -E '^(# )?CONFIG_' "$CACHE_DIR/config.running" | sort) \
             <(grep -E '^(# )?CONFIG_' "$CACHE_DIR/config.used" | sort) \
            | grep -E '^[<>]' | grep -vE 'BATMAN_ADV|LOCALVERSION_AUTO' > "$CACHE_DIR/config-drift.diff"
        if [ -s "$CACHE_DIR/config-drift.diff" ]; then
            echo "WARNING: olddefconfig changed the config; see $CACHE_DIR/config-drift.diff"
        fi

        # Fail early if olddefconfig dropped an option that changes the size of struct module
        cfgval() { grep -E "^CONFIG_$2=" "$1" | cut -d= -f2; }
        for opt in DEBUG_INFO_BTF DEBUG_INFO_BTF_MODULES MODULE_SIG KALLSYMS TRACEPOINTS LIVEPATCH \
                   DYNAMIC_DEBUG_CORE CFI CFI_CLANG MODULE_UNLOAD MODVERSIONS MODULE_FORCE_LOAD SMP PREEMPT_RT; do
            if [ "$(cfgval "$CACHE_DIR/config.running" "$opt")" != "$(cfgval "$CACHE_DIR/config.used" "$opt")" ]; then
                echo "ERROR: CONFIG_$opt differs from the running kernel's config, so the module would not load."
                case "$opt" in
                    DEBUG_INFO_BTF*) echo "       BTF needs a recent pahole (package 'dwarves'); found: $(pahole --version 2>&1 | head -1)" ;;
                esac
                exit 1
            fi
        done

        make -s -j"$(nproc)" modules_prepare 2>> "$CACHE_DIR/build.log" || exit 1
        KBUILD_MODPOST_WARN=1 make -s -j"$(nproc)" M=net/batman-adv modules 2>> "$CACHE_DIR/build.log"
    ) || { show_build_diagnostics; fail "batman-adv build failed (see $CACHE_DIR/build.log)"; }
    mkdir -p "$(dirname "$MODULE")"
    strip --strip-debug -o "$MODULE" "$SRC_DIR/net/batman-adv/batman-adv.ko" || fail "batman-adv.ko not produced"
    log_message "Built $MODULE"
}

# Drop cached modules for other kernel versions or older build revisions
for d in "$CACHE_DIR"/*-microsoft-*; do
    [ -d "$d" ] && [ "$d" != "$(dirname "$MODULE")" ] && rm -rf "$d"
done

modprobe cfg80211 || fail "Failed to load cfg80211"
modprobe bridge || fail "Failed to load bridge"
# batman-adv dependencies (harmless if built in or already loaded)
modprobe crc16 2> /dev/null
modprobe libcrc32c 2> /dev/null

if ! lsmod | grep -q '^batman_adv'; then
    if [ "$FORCE_REBUILD" = "1" ]; then
        log_message "FORCE_REBUILD=1: discarding cached module"
        rm -f "$MODULE"
    fi
    [ -f "$MODULE" ] || build_module
    if ! insmod "$MODULE"; then
        log_message "insmod failed. Module vermagic: $(modinfo -F vermagic "$MODULE" 2> /dev/null)"
        log_message "Running kernel:               $KVER"
        log_message "Last kernel messages:"
        dmesg | tail -5
        show_build_diagnostics
        # Don't keep a module that won't load, or every later run would skip the build and fail again
        rm -f "$MODULE"
        fail "Failed to load $MODULE (cached copy deleted). Inspect $CACHE_DIR/config-drift.diff and $CACHE_DIR/build.log, then re-run."
    fi
    # Module loads: the source tree is no longer needed
    rm -rf "$SRC_DIR"
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

ip addr add "$NODE_IP/16" dev bat0 || fail "Failed to assign $NODE_IP to bat0"

log_message "B.A.T.M.A.N. mesh setup complete! Interface: $UAIN, IP: $NODE_IP"
log_message "Waiting for neighbors..."
sleep 5
batctl n
exit 0
