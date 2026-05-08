#!/usr/bin/env bash
set -euo pipefail

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

WORKSPACE_SETUP_PATH="${WORKSPACE_SETUP:-/workspace/install/setup.bash}"
if [[ -f "${WORKSPACE_SETUP_PATH}" ]]; then
  source "${WORKSPACE_SETUP_PATH}"
fi

case "${MOWER_QT_AUTOSTART:-true}" in
  0|false|False|FALSE|no|No|NO)
    exit 0
    ;;
esac

if pgrep -f "mower_qt[./]mower_qt|ros2 run mower_qt mower_qt" >/dev/null 2>&1; then
  exit 0
fi

exec ros2 run mower_qt mower_qt
