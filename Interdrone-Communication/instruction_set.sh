# This file is automatically executed by the startup script if specified for
# the pi (i.e. it can be disabled). Instructions are executed in order and on
# startup if enabled.

# Current version of the Pi (semantic versioning)
CURRENT_VERSION="1.0.0"

# Get the current Pi version (create with default if missing)
if [ ! -f /etc/drone-version ]; then
    echo "1.0.0" > /etc/drone-version
fi
PI_VERSION=$(cat /etc/drone-version)





# If the Pi version is the same as the current version, exit
if [ "$PI_VERSION" = "$CURRENT_VERSION" ]; then
    exit 0
fi

# Update Pi version (store current as the new installed version)
echo "$CURRENT_VERSION" > /etc/drone-version