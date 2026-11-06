#!/usr/bin/env bash
# The frozen facts that only need r2r, rcl and rmw, not a running stack
# (deploy/test/golden/ros_facts/README.md), in the oracle image or the dev
# container, from the repository root. mower_interface is built out of the tree.
# freeze_stack.py makes the rest, from a robot image.
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# ROS's setup files read unset variables
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
if ! ros2 pkg prefix mower_interface >/dev/null 2>&1; then
  ws="${ORACLE_FACTS_WS:-/tmp/oracle-facts-ws}"
  (cd "$repo" && colcon --log-base "$ws/log" build --packages-select mower_interface --base-paths src/mower_interface \
    --build-base "$ws/build" --install-base "$ws/install" --event-handlers console_cohesion-)
  # shellcheck disable=SC1091
  source "$ws/install/setup.bash"
fi
set -u
cd "$repo/src/mower_rs"
export MOWER_GOLDEN=write CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-/tmp/oracle-target}"
# Each of these writes its file only after rcl or rmw gave the answer.
cargo test --locked -p mower_bridge --test resolver --test param_overrides
cargo test --locked -p mower_cdr --test rmw_oracle the_frozen_vectors
cargo test --locked -p mower_ws_bridge --test golden_cdr
cargo test --locked -p mower_bus_ros --test dds the_golden_corpus
unset MOWER_GOLDEN
python3 "$repo/tools/oracle/facts_manifest.py"
