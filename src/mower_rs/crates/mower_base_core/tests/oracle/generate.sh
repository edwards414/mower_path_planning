#!/usr/bin/env bash
# Rebuilds tests/vectors/* from the original C++ sources.
#
#   tests/oracle/generate.sh [workdir]
#
# workdir defaults to a temp dir and is where ros2_controllers / control_toolbox
# get cloned. Needs clang++ (or g++) and git; no ROS installation.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
crate="$(dirname "$(dirname "$here")")"      # .../crates/mower_base_core
repo="$(cd "$crate/../../../.." && pwd)"     # repo root
work="${1:-$(mktemp -d)}"
out="$crate/tests/vectors"
cxx="${CXX:-clang++}"

mkdir -p "$work" "$out"
[ -d "$work/ros2_controllers" ] || git clone --depth 1 -b jazzy \
  https://github.com/ros-controls/ros2_controllers "$work/ros2_controllers"
[ -d "$work/control_toolbox" ] || git clone --depth 1 -b jazzy \
  https://github.com/ros-controls/control_toolbox "$work/control_toolbox"

ddc="$work/ros2_controllers/diff_drive_controller"
ct="$work/control_toolbox/control_toolbox"
hw="$repo/src/mower_hardware"

common=(-std=c++17 -O1 -Wall)
ros=(-DRCPPUTILS_VERSION_MAJOR=2 -DRCPPUTILS_VERSION_MINOR=6
     -I "$here/stub" -I "$ddc/include" -I "$ct/include")

"$cxx" "${common[@]}" -I "$hw/include" \
  "$here/protocol_oracle.cpp" "$hw/src/mower_protocol.cpp" -o "$work/protocol_oracle"
"$cxx" "${common[@]}" "${ros[@]}" \
  "$here/ddc_oracle.cpp" "$ddc/src/odometry.cpp" -o "$work/ddc_oracle"
"$cxx" "${common[@]}" "${ros[@]}" -I "$hw/include" \
  "$here/replay_gen.cpp" "$hw/src/mower_protocol.cpp" "$ddc/src/odometry.cpp" \
  -o "$work/replay_gen"

"$work/protocol_oracle" > "$out/protocol_oracle.json"
"$work/ddc_oracle"      > "$out/diff_drive_oracle.json"
"$work/replay_gen"      "$out"
echo "vectors regenerated in $out"
