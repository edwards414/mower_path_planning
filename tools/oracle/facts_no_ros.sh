#!/usr/bin/env bash
# The tests of the frozen ROS facts (crates/mower_ros_facts) with no ROS: no
# environment sourced, so r2r's build script would stop the build if they
# reached it, and no test binary may link rcl, rmw or typesupport. It is what
# "the vector tests don't link rcl" means in E9-06; CI's rust-ros job and the
# oracle job run it. E9-22's plain types (crates/mower_msgs) and what they are
# generated from (crates/mower_schema) are held to the same: the generated code must
# be what gen_types.py writes, and neither crate may reach r2r, even by a dev-dependency.
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
cd "$repo/src/mower_rs"
python3 crates/mower_schema/gen/gen_types.py --check
crates=(mower_ros_facts mower_msgs mower_schema)
for crate in "${crates[@]}"; do
  if cargo tree --locked -p "$crate" -e normal,dev,build --all-features | grep -E '\b(r2r|r2r_[a-z_]+)\b'; then
    echo "$crate reaches r2r" >&2
    exit 1
  fi
done
packages=(-p mower_ros_facts -p mower_msgs -p mower_schema --features mower_schema/interfaces)
cargo test --locked "${packages[@]}"
binaries="$(cargo test --locked "${packages[@]}" --no-run --message-format=json \
  | python3 -c 'import json, sys; [print(r["executable"]) for r in map(json.loads, sys.stdin) if r.get("executable")]')"
[ -n "$binaries" ]
for binary in $binaries; do
  if ldd "$binary" | grep -E 'librcl|librmw|librosidl|libr2r|libament|libdds|libddsc'; then
    echo "$binary links ROS" >&2
    exit 1
  fi
done
echo "the frozen facts' tests need no ROS: $(echo "$binaries" | wc -l) test binaries, none links rcl"
