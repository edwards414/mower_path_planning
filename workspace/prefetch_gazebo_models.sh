#!/usr/bin/env bash
set -euo pipefail

workspace=${1:-$(pwd)}
ros_distro=${ROS_DISTRO:-jazzy}

set +u
source "/opt/ros/${ros_distro}/setup.bash"
if [[ -f "${workspace}/install/setup.bash" ]]; then
  source "${workspace}/install/setup.bash"
fi
set -u

world_file=${WORLD_FILE:-}
if [[ -z "${world_file}" ]]; then
  world_file="$(ros2 pkg prefix mower_bringup)/share/mower_bringup/worlds/mower_world.world"
fi

export GZ_FUEL_CACHE_PATH="${GZ_FUEL_CACHE_PATH:-${HOME}/.gz/fuel}"
mkdir -p "${GZ_FUEL_CACHE_PATH}"

echo "World: ${world_file}"
echo "Fuel cache: ${GZ_FUEL_CACHE_PATH}"

python3 - "${world_file}" <<'PY' | while IFS= read -r uri; do
import sys
import xml.etree.ElementTree as ET

world = sys.argv[1]
root = ET.parse(world).getroot()
seen = set()

for include in root.findall(".//include"):
    uri = (include.findtext("uri") or "").strip()
    if not uri.startswith(("http://", "https://")):
        continue
    if uri in seen:
        continue
    seen.add(uri)
    print(uri)
PY
  echo
  echo "Downloading ${uri}"
  gz fuel download -u "${uri}"
done
