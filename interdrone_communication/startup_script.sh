#!/bin/bash

echo "Starting startup script..."

# Detect pi number from hostname
PI_NUMBER=$(hostname | grep -oP 'mrrdt-\K\d+')

# Detect if pi number is valid
if [ -z "$PI_NUMBER" ]; then
    echo "ERROR: Could not detect pi number from hostname"
    exit 1
fi

# Enable/disable steps. Network connection disables both MST-Guest and Multirotor.
# Do not disable only network connection without disabling everything dependent on it.
ENABLE_NETWORK_CONNECTION=true
ENABLE_UPDATE_PACKAGES=true
ENABLE_INSTRUCTION_SET=true
ENABLE_BATMAN=true

# Step 1: Connect to MST-Guest 
if [ "$ENABLE_NETWORK_CONNECTION" = true ]; then
    nmcli dev wifi connect "MST-Guest" password "miner2020"
fi

# Step 2: Update packages
if [ "$ENABLE_UPDATE_PACKAGES" = true ]; then
    apt-get update -qq && apt-get upgrade -y -qq
fi

# Step 3: Fetch only instruction_set.sh from GitHub and execute it
if [ "$ENABLE_INSTRUCTION_SET" = true ]; then
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
  BRANCH="$(git -C "$REPO_ROOT" rev-parse --abbrev-ref HEAD)"
  TMP_FILE=$(mktemp)
  if git -C "$REPO_ROOT" fetch origin "$BRANCH" 2>/dev/null && \
    git -C "$REPO_ROOT" show "origin/$BRANCH:Interdrone-Communication/instruction_set.sh" > "$TMP_FILE" 2>/dev/null && \
    [ -s "$TMP_FILE" ]; then
    mv "$TMP_FILE" "$SCRIPT_DIR/instruction_set.sh"
    chmod +x "$SCRIPT_DIR/instruction_set.sh"
  else
    rm -f "$TMP_FILE"
  fi
  bash "$SCRIPT_DIR/instruction_set.sh"
fi


# Step 4: Verify the Multirotor network exists and switch to it
if [ "$ENABLE_NETWORK_CONNECTION" = true ] && nmcli dev wifi show | grep -q "Multirotor"; then
    nmcli dev wifi connect "Multirotor" password "mrrdt"
else
    echo "WARNING: Multirotor network not found, continuing on MST-Guest"
fi

# Step 5: Do pi specific stuff (update later)

# Step 6a: Ensure batman's expected path exists (it uses /home/mrrdt-N/IARC-10/Interdrone-Communication)
# Create symlink so batman's cd succeeds when repo is at /opt/drone-startup
if [ "$ENABLE_BATMAN" = true ]; then
  mkdir -p "/home/mrrdt-$PI_NUMBER"
  ln -sfn "$REPO_ROOT" "/home/mrrdt-$PI_NUMBER/IARC-10"

  # Disable power save
  sudo iw dev wlan0 set power_save off 

  # Step 6b: Run the batman mesh setup script
  BATMAN_SCRIPT="${BATMAN_MESH_SETUP_SCRIPT_PATH:-Interdrone-Communication/batman-mesh-setup.sh}"
  chmod +x "$REPO_ROOT/$BATMAN_SCRIPT"
  cd "$SCRIPT_DIR" && exec "$REPO_ROOT/$BATMAN_SCRIPT"
fi

