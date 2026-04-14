#!/usr/bin/env bash
set -euo pipefail

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

WORKSPACE_SETUP_PATH="${WORKSPACE_SETUP:-/workspace/install/setup.bash}"
if [[ -f "${WORKSPACE_SETUP_PATH}" ]]; then
  source "${WORKSPACE_SETUP_PATH}"
fi

case "${LAWER_QT_AUTOSTART:-true}" in
  0|false|False|FALSE|no|No|NO)
    exit 0
    ;;
esac

if pgrep -f "lawer_qt_py[./]lawer_qt|ros2 run lawer_qt_py lawer_qt" >/dev/null 2>&1; then
  exit 0
fi

exec ros2 run lawer_qt_py lawer_qt
