# This file is automatically executed by the startup script if specified for
# the pi (i.e. it can be disabled). Instructions are executed in order and on
# startup if enabled.

# Current version of the Pi (semantic versioning)
CURRENT_VERSION="1.0.0"

# Get the current Pi version
PI_VERSION=$(cat /etc/drone-version)





# If the Pi version is the same as the current version, exit
if [ "$PI_VERSION" -eq "$CURRENT_VERSION" ]; then
    exit 0
fi

# Update Pi version
PI_VERSION=$((PI_VERSION + 1))
echo "$PI_VERSION" > /etc/drone-version