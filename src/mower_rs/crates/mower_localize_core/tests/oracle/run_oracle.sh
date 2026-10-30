#!/usr/bin/env bash
# Regenerate the test vectors in ../data from the real robot_localization.
#
# Run it in the oracle image (tools/oracle/), from the repository root:
#   docker buildx build -f tools/oracle/Dockerfile -t mower-oracle:jazzy .
#   docker run --rm -e ROS_DOMAIN_ID=84 -v "$PWD":/repo mower-oracle:jazzy \
#       bash src/mower_rs/crates/mower_localize_core/tests/oracle/run_oracle.sh
set -eo pipefail
source /opt/ros/jazzy/setup.bash
here="$(cd "$(dirname "$0")" && pwd)"
build=/tmp/oracle-build
rm -rf "$build"
cmake -S "$here" -B "$build" -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$build" -j4 >/dev/null
mkdir -p "$here/../data"
"$build/oracle" "$here/../data"
ls -l "$here/../data"
