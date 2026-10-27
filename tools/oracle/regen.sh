#!/usr/bin/env bash
# Regenerates every vector the Rust ports are compared with, from the real
# implementations, inside the oracle image (tools/oracle/Dockerfile):
#   nav       nav2 1.3.13: mower_nav_core's seven component harnesses and the
#             controller_server and navigator node scenarios (tests/data/)
#   localize  robot_localization 3.8.3: mower_localize_core (tests/data/)
#   base      ros2_controllers' diff drive odometry and this repository's
#             mower_protocol.cpp: mower_base_core (tests/vectors/)
# With no argument it runs all three. `git status` then shows what changed.
# All of it is deterministic except the two node scenarios
# (controller_server.json, navigator.json: real nodes on the wall clock,
# different on every run); tools/oracle/check.sh is what the oracle
# workflow runs on top of this.
#
#   docker run --rm -v "$PWD":/repo mower-oracle:jazzy tools/oracle/regen.sh [nav|localize|base]...
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
crates="$repo/src/mower_rs/crates"
# the node scenarios and the localize oracle start DDS participants
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
parts=("$@")
[ ${#parts[@]} -gt 0 ] || parts=(nav localize base)
for part in "${parts[@]}"; do
  echo "== $part"
  case "$part" in
    nav) bash "$crates/mower_nav_core/tests/oracle/run_oracle.sh" ;;
    localize) bash "$crates/mower_localize_core/tests/oracle/run_oracle.sh" ;;
    base) bash "$crates/mower_base_core/tests/oracle/generate.sh" "$(mktemp -d)" ;;
    *) echo "unknown part '$part' (nav, localize or base)" >&2; exit 2 ;;
  esac
done
