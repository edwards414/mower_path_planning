#!/usr/bin/env bash
set -euo pipefail

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

if pgrep -f "rviz2" >/dev/null 2>&1; then
  exit 0
fi

RVIZ_CONFIG_PATH="${RVIZ_CONFIG:-}"

if [[ -n "${RVIZ_CONFIG_PATH}" && -f "${RVIZ_CONFIG_PATH}" ]]; then
  exec rviz2 -d "${RVIZ_CONFIG_PATH}"
fi

exec rviz2
