#!/usr/bin/env bash
# The tests of the frozen ROS facts (crates/mower_ros_facts) with no ROS: no
# environment sourced, so r2r's build script would stop the build if they
# reached it, and no test binary may link rcl, rmw or typesupport. It is what
# "the vector tests don't link rcl" means in E9-06; CI's rust-ros job and the
# oracle job run it.
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
cd "$repo/src/mower_rs"
if cargo tree --locked -p mower_ros_facts -e normal,dev,build | grep -E '\b(r2r|r2r_[a-z_]+)\b'; then
  echo "mower_ros_facts reaches r2r" >&2
  exit 1
fi
cargo test --locked -p mower_ros_facts
binaries="$(cargo test --locked -p mower_ros_facts --no-run --message-format=json \
  | python3 -c 'import json, sys; [print(r["executable"]) for r in map(json.loads, sys.stdin) if r.get("executable")]')"
[ -n "$binaries" ]
for binary in $binaries; do
  if ldd "$binary" | grep -E 'librcl|librmw|librosidl|libr2r|libament|libdds|libddsc'; then
    echo "$binary links ROS" >&2
    exit 1
  fi
done
echo "the frozen facts' tests need no ROS: $(echo "$binaries" | wc -l) test binaries, none links rcl"
