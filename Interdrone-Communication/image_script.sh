set -euo pipefail

# GitHub repo that contains the startup script (run at every boot).
STARTUP_REPO="${STARTUP_REPO:-https://github.com/MissouriMRR/IARC-10.git}"
# Branch to use.
STARTUP_BRANCH="${STARTUP_BRANCH:-IARC-LVP}"
# Path inside the repo to the script that runs at boot (relative to repo root).
STARTUP_SCRIPT_PATH="${STARTUP_SCRIPT_PATH:-Interdrone-Communication/startup_script.sh}"
# Where to clone the startup repo on the Pi.
STARTUP_INSTALL_DIR="${STARTUP_INSTALL_DIR:-/opt/drone-startup}"
# Path inside the repo to the batman mesh setup script.
BATMAN_MESH_SETUP_SCRIPT_PATH="${BATMAN_MESH_SETUP_SCRIPT_PATH:-Interdrone-Communication/batman-mesh-setup.sh}"

# Pi version: stored in /etc/drone-version so other scripts can read and update it
PI_VERSION="${PI_VERSION:-1.0.0}"
PI_VERSION_FILE="${PI_VERSION_FILE:-/etc/drone-version}"

# If there is already a version file, this script already ran, so exit
if [ -f "$PI_VERSION_FILE" ]; then
    echo "Pi version file already exists, exiting"
    exit 0
fi

# Package install
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq

# Add packages here as needed BEFORE we image the Pis.
# If a pi is already imaged, either install it manually
# or add it to the "instruction set"
apt-get install -y -qq \
    iw \
    iproute2 \
    network-manager \
    git \
    python3 \
    python3-pip \
    python3-venv \
    curl \
    ca-certificates \
    net-tools \
    batctl

echo "Packages installed"

# Install uv
if ! command -v uv &>/dev/null; then
    curl -LsSf https://astral.sh/uv/install.sh | sh
    export PATH="${HOME:-/root}/.local/bin:${PATH}"
    # Ensure root and default user have uv on PATH in future logins
    echo 'export PATH="${HOME}/.local/bin:${PATH}"' >> /etc/profile.d/uv.sh
    chmod 644 /etc/profile.d/uv.sh
fi

echo "uv installed"

# Fetch startup script from GitHub
# Clone (or update) the startup repo so the script is available at boot.
mkdir -p "$(dirname "$STARTUP_INSTALL_DIR")"
if [ -d "$STARTUP_INSTALL_DIR/.git" ]; then
    git -C "$STARTUP_INSTALL_DIR" fetch -q && git -C "$STARTUP_INSTALL_DIR" checkout -q "$STARTUP_BRANCH" && git -C "$STARTUP_INSTALL_DIR" pull -q || true
else
    git clone -q -b "$STARTUP_BRANCH" --depth 1 "$STARTUP_REPO" "$STARTUP_INSTALL_DIR"
fi
chmod +x "$STARTUP_INSTALL_DIR/$STARTUP_SCRIPT_PATH" 2>/dev/null || true

echo "Startup script fetched"

# Boot process: systemd service
# Runner executed by systemd: cd to repo, pull latest, run startup script.
# This allows the startup script to assume it's in the repo and do git pull.
RUNNER_SCRIPT="/usr/local/bin/drone-startup-runner.sh"
cat > "$RUNNER_SCRIPT" << RUNNER
#!/bin/bash
set -euo pipefail
export PATH="/root/.local/bin:\${PATH}"
cd "$STARTUP_INSTALL_DIR"
git pull -q || true
exec ./$STARTUP_SCRIPT_PATH
RUNNER
chmod +x "$RUNNER_SCRIPT"

echo "Runner script created"

# Install systemd unit so the startup script runs every boot (after network)
SERVICE_NAME="drone-startup.service"
cat > "/etc/systemd/system/$SERVICE_NAME" << UNIT
[Unit]
Description=Drone startup script (from GitHub)
After=network-online.target NetworkManager.service
Wants=network-online.target

[Service]
Type=oneshot
ExecStart=$RUNNER_SCRIPT
RemainAfterExit=yes
StandardOutput=journal
StandardError=journal
TimeoutStartSec=300

[Install]
WantedBy=multi-user.target
UNIT

systemctl daemon-reload
systemctl enable "$SERVICE_NAME"

echo "Service enabled"

# Persist Pi version for other scripts to read/update
echo "$PI_VERSION" > "$PI_VERSION_FILE"
chmod 644 "$PI_VERSION_FILE"

echo "Pi version stored"

# Done
echo "Image setup complete. Startup script: $STARTUP_INSTALL_DIR/$STARTUP_SCRIPT_PATH (runs at boot via $SERVICE_NAME). Pi version: $PI_VERSION (stored in $PI_VERSION_FILE)."
echo "Running startup script..."
# Run the startup script (use same cwd as systemd runner so paths match)
cd "$STARTUP_INSTALL_DIR" && exec ./$STARTUP_SCRIPT_PATH