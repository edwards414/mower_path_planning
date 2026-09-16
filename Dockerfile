# syntax=docker/dockerfile:1
ARG ROS_DISTRO=jazzy
ARG WORKSPACE=/mower_ws

##############################################
# Stage 1: Base
##############################################
FROM ros:${ROS_DISTRO}-ros-base AS base

ARG ROS_DISTRO
ARG WORKSPACE

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV WORKSPACE=${WORKSPACE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
WORKDIR ${WORKSPACE}

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        curl \
        git \
        libcurl4-openssl-dev \
        libusb-1.0-0-dev \
        make \
        pkg-config \
        python-is-python3 \
        python3-colcon-common-extensions \
        python3-pip \
        python3-rosdep \
        python3-vcstool \
        sudo \
        wget \
    && if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update

##############################################
# Stage 2: Builder
##############################################
FROM base AS builder

ARG ROS_DISTRO
ARG WORKSPACE

# Rust toolchain + maturin, required to build the PyO3 package mower_coverage_core.
# Builder-only: the compiled wheel is installed into the workspace, so the runtime
# stage never needs cargo. Placed before COPY so source edits don't bust this layer.
ENV RUSTUP_HOME=/usr/local/rustup \
    CARGO_HOME=/usr/local/cargo \
    PATH=/usr/local/cargo/bin:$PATH
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs \
        | sh -s -- -y --default-toolchain stable --no-modify-path \
    && pip3 install --no-cache-dir --break-system-packages maturin

COPY ./src ${WORKSPACE}/src
COPY ./utils/docker-entrypoint.sh /usr/local/bin/docker-entrypoint.sh

RUN chmod +x /usr/local/bin/docker-entrypoint.sh \
    && sed -i 's/\r$//' /usr/local/bin/docker-entrypoint.sh

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep install --ignore-src --from-paths src -i --rosdistro ${ROS_DISTRO} -y

# Do not use --symlink-install here, otherwise the runtime stage will
# inherit broken links after copying only the install tree.
RUN --mount=type=cache,target=${WORKSPACE}/build \
    --mount=type=cache,target=/usr/local/cargo/registry \
    --mount=type=cache,target=/usr/local/cargo/git \
    source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Harvest the runtime apt dependencies.
# NOTE: do NOT pass --ignore-src here. --ignore-src drops keys for any ament
# package already present in an installed underlay -- and by this point rosdep
# install has populated /opt/ros with all build+exec deps, so --ignore-src would
# wrongly discard nearly every ROS runtime dependency (xacro, nav2, ...). Without
# it, rosdep keys lists every exec dep; the workspace's own mower_* packages have
# no rosdep rule, so `rosdep resolve` fails for them and they are naturally
# excluded -- giving exactly the external apt packages the runtime needs.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep keys --from-paths src --dependency-types=exec --rosdistro ${ROS_DISTRO} \
        | sort -u \
        | while read -r key; do \
            rosdep resolve --rosdistro ${ROS_DISTRO} "${key}" 2>/dev/null \
                | awk '/^#apt$/{getline; print}' || true; \
        done \
        | tr ' ' '\n' \
        | sed -e '/^[[:space:]]*$/d' \
        | sort -u \
        | grep -vxE 'ros-jazzy-(rviz2|joint-state-publisher-gui)' \
        > /tmp/runtime-apt-packages.txt

##############################################
# Stage: firmware (STM32F411 application, cross-compiled)
# Runs on the build host's own platform (no QEMU) and produces the .bin the
# runtime image ships in /opt/mower/firmware; firmware-sync flashes it at
# container start. Same commit as the ros2_control driver in src/mower_hardware.
##############################################
FROM --platform=$BUILDPLATFORM ubuntu:24.04 AS firmware

ARG MOWER_VERSION
ARG MOWER_GIT_SHA
ARG MOWER_BUILD_UNIX
ARG MOWER_BUILD_DIRTY

ENV DEBIAN_FRONTEND=noninteractive
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        gcc-arm-none-eabi \
        libnewlib-arm-none-eabi \
        libstdc++-arm-none-eabi-newlib \
        make \
        python3

WORKDIR /fw
COPY ./firmware /fw
# No .git in the build context: the identity comes from the build args
# (.github/workflows/build.yml / make build-release). Empty args fall back to
# the Makefile defaults (0.0.0, unknown sha, build time = now).
RUN make -j"$(nproc)" TOOLCHAIN_BIN= \
        ${MOWER_VERSION:+MOWER_VERSION="$MOWER_VERSION"} \
        ${MOWER_GIT_SHA:+MOWER_GIT_SHA="$MOWER_GIT_SHA"} \
        ${MOWER_BUILD_UNIX:+MOWER_BUILD_UNIX="$MOWER_BUILD_UNIX"} \
        ${MOWER_BUILD_DIRTY:+MOWER_BUILD_DIRTY="$MOWER_BUILD_DIRTY"} \
    && make -C bootloader TOOLCHAIN_BIN= \
    && cat build/mower_robot_firmware.json

##############################################
# Stage 3: Runtime
##############################################
FROM ros:${ROS_DISTRO}-ros-core AS runtime

ARG ROS_DISTRO
ARG WORKSPACE
ARG USER_NAME=mower
ARG USER_ID=1000
ARG GROUP_NAME=mower
ARG GROUP_ID=1000

# Build identity, reported on /robot/info (mower_mission/version.py).
ARG MOWER_VERSION=dev
ARG MOWER_GIT_SHA=
ARG MOWER_BUILD_UNIX=
ARG MOWER_IMAGE=

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV WORKSPACE=${WORKSPACE}
ENV HOME=/home/${USER_NAME}
ENV MOWER_VERSION=${MOWER_VERSION} \
    MOWER_GIT_SHA=${MOWER_GIT_SHA} \
    MOWER_BUILD_UNIX=${MOWER_BUILD_UNIX} \
    MOWER_IMAGE=${MOWER_IMAGE} \
    MOWER_FIRMWARE_MANIFEST=/opt/mower/firmware/mower_robot_firmware.json

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
WORKDIR ${WORKSPACE}

COPY --from=builder ${WORKSPACE}/install ${WORKSPACE}/install
COPY --from=builder /tmp/runtime-apt-packages.txt /tmp/runtime-apt-packages.txt
COPY ./utils/docker-entrypoint.sh /usr/local/bin/docker-entrypoint.sh
COPY ./utils/firmware-sync ./utils/mower-host-request /usr/local/bin/
# STM32 firmware built from this same commit + the tool that flashes it.
COPY --from=firmware /fw/build/mower_robot_firmware.bin \
                     /fw/build/mower_robot_firmware.json \
                     /fw/bootloader/build/bootloader.bin \
                     /opt/mower/firmware/
COPY ./firmware/tools/mower_flash.py /opt/mower/firmware/mower_flash.py

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean \
    && chmod +x /usr/local/bin/docker-entrypoint.sh /usr/local/bin/firmware-sync /usr/local/bin/mower-host-request \
    && sed -i 's/\r$//' /usr/local/bin/docker-entrypoint.sh /usr/local/bin/firmware-sync /usr/local/bin/mower-host-request \
    && apt-get update \
    && { \
        printf '%s\n' libcurl4 libusb-1.0-0 python3-serial; \
        cat /tmp/runtime-apt-packages.txt; \
    } \
        | sort -u \
        | xargs -r apt-get install -y --no-install-recommends \
    && rm -rf /tmp/runtime-apt-packages.txt

RUN if getent group ${GROUP_ID} > /dev/null; then \
        existing_group="$(getent group ${GROUP_ID} | cut -d: -f1)"; \
        if [ "${existing_group}" != "${GROUP_NAME}" ]; then groupmod --new-name ${GROUP_NAME} "${existing_group}"; fi; \
    elif getent group ${GROUP_NAME} > /dev/null; then \
        groupmod --gid ${GROUP_ID} ${GROUP_NAME}; \
    else \
        groupadd --gid ${GROUP_ID} ${GROUP_NAME}; \
    fi \
    && if getent passwd ${USER_ID} > /dev/null; then \
        existing_user="$(getent passwd ${USER_ID} | cut -d: -f1)"; \
        if [ "${existing_user}" != "${USER_NAME}" ]; then usermod --login ${USER_NAME} "${existing_user}"; fi; \
        usermod --home /home/${USER_NAME} --move-home ${USER_NAME} || true; \
        usermod --gid ${GROUP_ID} --shell /bin/bash ${USER_NAME}; \
    elif id -u ${USER_NAME} >/dev/null 2>&1; then \
        usermod --uid ${USER_ID} --gid ${GROUP_ID} --shell /bin/bash ${USER_NAME}; \
    else \
        useradd --uid ${USER_ID} --gid ${GROUP_ID} --create-home --shell /bin/bash ${USER_NAME}; \
    fi \
    && usermod --append --groups video,dialout ${USER_NAME} \
    && mkdir -p /home/${USER_NAME}/.mower/zone_record \
        /home/${USER_NAME}/.mower/sites \
        /home/${USER_NAME}/.mower/bags \
    && chown -R ${USER_ID}:${GROUP_ID} ${WORKSPACE} /home/${USER_NAME}

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USER_NAME}/.bashrc \
    && echo "source ${WORKSPACE}/install/setup.bash" >> /home/${USER_NAME}/.bashrc \
    && chown ${USER_ID}:${GROUP_ID} /home/${USER_NAME}/.bashrc

USER ${USER_NAME}

ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
CMD ["ros2", "launch", "mower_bringup", "robot.launch.py", "use_sim_time:=false"]

##############################################
# Stage: gps_runtime (輕量 ublox GPS driver)
# 機器人上跑的 GPS 驅動容器。直接 FROM ros-base、不繼承 builder,
# 不含整包 workspace,只裝 ublox-gps + cyclonedds RMW,image 保持精簡。
# BuildKit 只在指定此 target 時才會 build 這個 stage。
##############################################
FROM ros:${ROS_DISTRO}-ros-base AS gps_runtime

ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-ublox-gps

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

CMD ["bash"]
