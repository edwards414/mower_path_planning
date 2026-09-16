#!/bin/bash
set -e

# Source ROS environment
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f "${WORKSPACE}/install/local_setup.bash" ]; then
  source ${WORKSPACE}/install/local_setup.bash
fi

# Bring the STM32 to the firmware bundled in this image (no-op without
# /dev/stmcom or with MOWER_FIRMWARE_SYNC=0). Must run before ros2_control
# opens the port.
if [ -x /usr/local/bin/firmware-sync ]; then
  /usr/local/bin/firmware-sync
fi

# Execute the command passed to the container
exec "$@"
