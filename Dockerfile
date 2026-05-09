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

RUN apt-get update \
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
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

##############################################
# Stage 2: Builder
##############################################
FROM base AS builder

ARG ROS_DISTRO
ARG WORKSPACE

COPY ./src ${WORKSPACE}/src
COPY ./utils/docker-entrypoint.sh /usr/local/bin/docker-entrypoint.sh

RUN chmod +x /usr/local/bin/docker-entrypoint.sh \
    && sed -i 's/\r$//' /usr/local/bin/docker-entrypoint.sh

RUN apt-get update \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep install --ignore-src --from-paths src -i --rosdistro ${ROS_DISTRO} -y \
    && rm -rf /var/lib/apt/lists/*

# Do not use --symlink-install here, otherwise the runtime stage will
# inherit broken links after copying only the install tree.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep keys --from-paths src --ignore-src --dependency-types=exec --rosdistro ${ROS_DISTRO} \
        | sort -u \
        | while read -r key; do \
            rosdep resolve --rosdistro ${ROS_DISTRO} "${key}" \
                | awk '/^#apt$/{getline; print}'; \
        done \
        | tr ' ' '\n' \
        | sed -e '/^[[:space:]]*$/d' \
        | sort -u \
        > /tmp/runtime-apt-packages.txt

##############################################
# Stage 3: Runtime
##############################################
FROM ros:${ROS_DISTRO}-ros-base AS runtime

ARG ROS_DISTRO
ARG WORKSPACE
ARG USER_NAME=mower
ARG USER_ID=1000
ARG GROUP_NAME=mower
ARG GROUP_ID=1000

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV WORKSPACE=${WORKSPACE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
WORKDIR ${WORKSPACE}

COPY --from=builder ${WORKSPACE}/install ${WORKSPACE}/install
COPY --from=builder /tmp/runtime-apt-packages.txt /tmp/runtime-apt-packages.txt
COPY ./utils/docker-entrypoint.sh /usr/local/bin/docker-entrypoint.sh

RUN chmod +x /usr/local/bin/docker-entrypoint.sh \
    && sed -i 's/\r$//' /usr/local/bin/docker-entrypoint.sh \
    && apt-get update \
    && { \
        printf '%s\n' libcurl4 libusb-1.0-0; \
        cat /tmp/runtime-apt-packages.txt; \
    } \
        | sort -u \
        | xargs -r apt-get install -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/* /tmp/runtime-apt-packages.txt

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
    && chown -R ${USER_ID}:${GROUP_ID} ${WORKSPACE} /home/${USER_NAME}

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USER_NAME}/.bashrc \
    && echo "source ${WORKSPACE}/install/setup.bash" >> /home/${USER_NAME}/.bashrc \
    && chown ${USER_ID}:${GROUP_ID} /home/${USER_NAME}/.bashrc

USER ${USER_NAME}

ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
CMD ["ros2", "launch", "mower_bringup", "mower.launch.py"]
