#!/usr/bin/env bash
set -euo pipefail

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

WORKSPACE_SETUP_PATH="${WORKSPACE_SETUP:-/workspace/install/setup.bash}"
if [[ -f "${WORKSPACE_SETUP_PATH}" ]]; then
  # Source the shared workspace so RViz can resolve package:// URIs like mower_description.
  source "${WORKSPACE_SETUP_PATH}"
fi

if pgrep -f "rviz2" >/dev/null 2>&1; then
  exit 0
fi

RVIZ_CONFIG_PATH="${RVIZ_CONFIG:-}"

if [[ -n "${RVIZ_CONFIG_PATH}" && -f "${RVIZ_CONFIG_PATH}" ]]; then
  exec rviz2 -d "${RVIZ_CONFIG_PATH}"
fi

exec rviz2
