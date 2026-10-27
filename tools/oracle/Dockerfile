# syntax=docker/dockerfile:1
# The one ROS container that stays (E9-05). It holds the real libraries the
# Rust ports are compared with, at the versions the robot image ships:
#   nav2 1.3.13, robot_localization 3.8.3, rmw_cyclonedds 2.2.4, twist_mux,
#   robot_state_publisher and xacro (E4's oracles)
# on the robot's libc6 (2.39-0ubuntu8.9, the same in ros:jazzy-ros-base and
# ros:jazzy-ros-core: nav2's results depend on glibc's libm). Plus the C++
# toolchain the harnesses build with, the message packages r2r binds, the
# rclpy tools of CI's black-box checks, and the Rust toolchain the robot's
# binaries are built with. It is:
#   - where every committed oracle vector is regenerated (tools/oracle/regen.sh,
#     the `oracle` workflow);
#   - the container CI's rust-ros job runs `cargo test` in;
# so neither of them runs apt.
#
#   docker buildx build -f tools/oracle/Dockerfile -t mower-oracle:jazzy .
#
# The build context is the repository root (the apt pins are bind-mounted
# from tools/oracle/; ros-snapshots.asc is the ROS snapshot key,
# 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA).
ARG ROS_BASE_IMAGE=docker.io/library/ros:jazzy-ros-base@sha256:103b6c417973394659af6804d4917cd489b91e5ed22eb7bef263025a3bed2410
FROM ${ROS_BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Pinned the way the runtime image's apt layer is (Dockerfile, runtime stage):
# every version of tools/oracle/oracle-apt.lock at Pin-Priority 1001, the ROS
# packages from the snapshot of the sync the robot's come from, and the
# image's whole dpkg list checked against the lock. To change the lock, copy
# the list a failed check prints.
ARG ROS_APT_SNAPSHOT=http://snapshots.ros.org/jazzy/2026-10-05/ubuntu
RUN --mount=type=bind,source=tools/oracle/oracle-apt.lock,target=/tmp/apt-lock/oracle-apt.lock \
    --mount=type=bind,source=tools/oracle/ros-snapshots.asc,target=/tmp/apt-lock/ros-snapshots.asc \
    pin=/tmp/apt-pin && mkdir -p "$pin/sources.list.d" "$pin/preferences.d" \
    && cp /etc/apt/sources.list.d/ubuntu.sources "$pin/sources.list.d/" \
    && printf '%s\n' 'Types: deb' "URIs: ${ROS_APT_SNAPSHOT}" 'Suites: noble' 'Components: main' \
        'Signed-By: /tmp/apt-lock/ros-snapshots.asc' > "$pin/sources.list.d/ros2-snapshot.sources" \
    && sed '/^#/d' /tmp/apt-lock/oracle-apt.lock \
        | awk '{ sub(/:arm64$/, "", $1); printf "Package: %s\nPin: version %s\nPin-Priority: 1001\n\n", $1, $2 }' \
        > "$pin/preferences.d/oracle-apt-lock.pref" \
    && printf '%s\n' 'Dir::Etc::SourceList "/dev/null";' "Dir::Etc::SourceParts \"$pin/sources.list.d\";" \
        'Dir::Etc::Preferences "/dev/null";' "Dir::Etc::PreferencesParts \"$pin/preferences.d\";" \
        > "$pin/apt.conf" \
    && export APT_CONFIG="$pin/apt.conf" \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        `# nav2 (mower_nav_core's oracles and node scenarios)` \
        ros-jazzy-nav2-navfn-planner ros-jazzy-nav2-regulated-pure-pursuit-controller \
        ros-jazzy-nav2-rotation-shim-controller ros-jazzy-nav2-controller \
        ros-jazzy-nav2-velocity-smoother ros-jazzy-nav2-costmap-2d ros-jazzy-nav2-util \
        ros-jazzy-nav2-planner ros-jazzy-nav2-bt-navigator ros-jazzy-nav2-behaviors \
        `# robot_localization (mower_localize_core's oracle)` \
        ros-jazzy-robot-localization libgeographiclib-dev libeigen3-dev \
        `# E4: twist_mux and robot_state_publisher, the URDF through xacro` \
        ros-jazzy-twist-mux ros-jazzy-robot-state-publisher ros-jazzy-xacro \
        `# what r2r binds (src/mower_rs/.cargo/config.toml) and the robot's RMW` \
        ros-jazzy-nav2-msgs ros-jazzy-geographic-msgs ros-jazzy-rmw-cyclonedds-cpp \
        `# the rclpy black-box tools (nav_progress_check runs the Python nav server)` \
        ros-jazzy-nav2-simple-commander \
        `# the harnesses' toolchain and bindgen's libclang (curl, git and` \
        `# ca-certificates come with the base image, at the robot's versions)` \
        cmake g++ clang libclang-dev nlohmann-json3-dev \
    && unset APT_CONFIG && rm -rf "$pin" /var/lib/apt/lists/* \
    && dpkg-query -W -f '${binary:Package} ${Version}\n' | LC_ALL=C sort > /tmp/installed.txt \
    && if ! sed '/^#/d' /tmp/apt-lock/oracle-apt.lock | diff -u - /tmp/installed.txt; then \
        echo "dpkg's packages differ from tools/oracle/oracle-apt.lock (diff above). The complete new list:"; \
        cat /tmp/installed.txt; exit 1; \
    fi \
    && rm /tmp/installed.txt

# The toolchain the robot's binaries are built with (rustc 1.99.1 is in the
# shipped mower_rsd) and src/mower_rs/rust-toolchain.toml names.
ARG RUST_TOOLCHAIN=1.99.1
ENV RUSTUP_HOME=/usr/local/rustup \
    CARGO_HOME=/usr/local/cargo \
    PATH=/usr/local/cargo/bin:$PATH
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs \
        | sh -s -- -y --profile minimal --default-toolchain "${RUST_TOOLCHAIN}" --no-modify-path \
    && chmod -R a+rwX "${CARGO_HOME}" "${RUSTUP_HOME}" \
    && rustc --version && cargo --version

WORKDIR /repo
