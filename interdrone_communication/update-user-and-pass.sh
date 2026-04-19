#!/bin/bash

# ─────────────────────────────────────────
# Configuration - Edit these values
# ─────────────────────────────────────────
IMAGE="pi-os-v1.img"       # Path to your .img file
OLD_USER="mrrdt-1"                   # Original username in the image
NEW_USER="mrrdt-4"             # New username to replace it with
NEW_PASS="mrrdt"               # New password (plain text, will be hashed)

MOUNT_POINT="/mnt/piimage"
ROOT_OFFSET=545259520           # Partition 2 start sector × 512 (1064960 × 512)
# ─────────────────────────────────────────

echo "[*] Starting Pi image user update..."
echo "    Image:    $IMAGE"
echo "    Old user: $OLD_USER"
echo "    New user: $NEW_USER"
echo ""

# Check image exists
if [ ! -f "$IMAGE" ]; then
    echo "[!] Image file not found: $IMAGE"
    exit 1
fi

# Create mount point if needed
if [ ! -d "$MOUNT_POINT" ]; then
    echo "[*] Creating mount point $MOUNT_POINT..."
    sudo mkdir -p "$MOUNT_POINT"
fi

# Mount root partition
echo "[*] Mounting root partition..."
sudo mount -o loop,offset=$ROOT_OFFSET "$IMAGE" "$MOUNT_POINT"
if [ $? -ne 0 ]; then
    echo "[!] Failed to mount image. Check offset and image path."
    exit 1
fi

# Generate hashed password
echo "[*] Generating hashed password..."
NEW_HASH=$(echo "$NEW_PASS" | openssl passwd -6 -stdin)
if [ -z "$NEW_HASH" ]; then
    echo "[!] Failed to generate password hash."
    sudo umount "$MOUNT_POINT"
    exit 1
fi

# Update /etc/passwd
echo "[*] Updating /etc/passwd..."
sudo sed -i "s/^$OLD_USER:/$NEW_USER:/g" "$MOUNT_POINT/etc/passwd"
sudo sed -i "s|/home/$OLD_USER|/home/$NEW_USER|g" "$MOUNT_POINT/etc/passwd"

# Update /etc/shadow (username + password hash)
echo "[*] Updating /etc/shadow..."
sudo sed -i "s/^$OLD_USER:/$NEW_USER:/g" "$MOUNT_POINT/etc/shadow"
sudo sed -i "s|^$NEW_USER:[^:]*|$NEW_USER:$NEW_HASH|" "$MOUNT_POINT/etc/shadow"

# Update /etc/group
echo "[*] Updating /etc/group..."
sudo sed -i "s/\b$OLD_USER\b/$NEW_USER/g" "$MOUNT_POINT/etc/group"

# Update /etc/gshadow
echo "[*] Updating /etc/gshadow..."
sudo sed -i "s/\b$OLD_USER\b/$NEW_USER/g" "$MOUNT_POINT/etc/gshadow"

# Rename home directory
if [ -d "$MOUNT_POINT/home/$OLD_USER" ]; then
    echo "[*] Renaming home directory..."
    sudo mv "$MOUNT_POINT/home/$OLD_USER" "$MOUNT_POINT/home/$NEW_USER"
else
    echo "[!] Home directory /home/$OLD_USER not found, skipping rename."
fi

# Verify
echo ""
echo "[*] Verifying changes..."
echo "    passwd:  $(grep "^$NEW_USER:" $MOUNT_POINT/etc/passwd)"
echo "    home:    $(ls $MOUNT_POINT/home/)"

# Unmount
echo ""
echo "[*] Unmounting image..."
sudo umount "$MOUNT_POINT"

echo ""
echo "[✓] Done! Image updated successfully."
echo "    You can now flash $IMAGE to your SD card."
