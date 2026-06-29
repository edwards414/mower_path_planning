ARG ROS_DISTRO=jazzy
ARG WORKSPACE=/home/fxrbindi/mower_ws
##############################################
# Stage 1: Base
##############################################
FROM ros:${ROS_DISTRO}-ros-base AS base

ARG ROS_DISTRO
ARG WORKSPACE
ENV ROS_DISTRO=${ROS_DISTRO}
ENV WORKSPACE=${WORKSPACE}

# systemd-resolved is only ever pulled in as a Recommends here, and its postinst
# cannot relink /etc/resolv.conf during a buildx build ("Device or resource
# busy"), which aborts the image build with exit 12. Pin it out so it is never
# installed in any stage (this file lives in base and is inherited by builder/dev).
RUN printf 'Package: systemd-resolved\nPin: release *\nPin-Priority: -1\n' \
    > /etc/apt/preferences.d/no-systemd-resolved

# 复制包列表文件
COPY apt-base-packages /tmp/apt-base-packages

# 安裝必要工具

RUN apt-get update && \
    apt-get install -y gettext-base && \
    sed -e 's/#.*//' -e 's/\r$//' -e '/^[[:space:]]*$/d' /tmp/apt-base-packages | \
    envsubst | \
    xargs -r apt-get install -y && \
    rm -rf /var/lib/apt/lists/* /tmp/apt-base-packages

RUN rosdep update 

WORKDIR ${WORKSPACE}


##############################################
# Stage 2: Builder
##############################################
FROM base AS builder

ARG ROS_DISTRO
ARG WORKSPACE
ENV ROS_DISTRO=${ROS_DISTRO}
ENV WORKSPACE=${WORKSPACE}
# 复制 CV 包列表文件
COPY apt-cv-packages /tmp/apt-cv-packages

RUN apt-get update && \
    sed -e 's/#.*//' -e 's/\r$//' -e '/^[[:space:]]*$/d' /tmp/apt-cv-packages | \
    envsubst | \
    xargs -r apt-get install -y \
    && rm -rf /var/lib/apt/lists/* /tmp/apt-cv-packages

##############################################
# Stage 3: dev
##############################################
FROM builder AS dev

ARG ROS_DISTRO
ARG WORKSPACE
ENV ROS_DISTRO=${ROS_DISTRO}
ENV WORKSPACE=${WORKSPACE}

# 复制开发包列表文件
COPY apt-dev-packages /tmp/apt-dev-packages

# 安装 gettext-base 以提供 envsubst 命令，然后安装开发包
RUN apt-get update && \
    apt-get install -y gettext-base && \
    sed -e 's/#.*//' -e 's/\r$//' -e '/^[[:space:]]*$/d' /tmp/apt-dev-packages | \
    envsubst | \
    xargs -r apt-get install -y && \
    rm -rf /var/lib/apt/lists/* /tmp/apt-dev-packages


# Add user with same UID and GID as your host system, replace if one exists with same UID
ARG USER_NAME=fxrbindi
ARG USER_ID
ARG GROUP_NAME
ARG GROUP_ID

# 建立與主機相同 UID/GID 的使用者，避免檔案權限問題
RUN \
    # === 處理群組 ===
    # 檢查該 GID 是否已存在
    if getent group $GROUP_ID > /dev/null; then \
    # 若存在：取得原群組名稱，並重新命名為目標名稱
    OLD_GROUP_NAME=$(getent group $GROUP_ID | cut -d: -f1); \
    groupmod --new-name $USER_NAME $OLD_GROUP_NAME; \
    else \
    # 若不存在：建立新群組
    groupadd --gid $GROUP_ID $USER_NAME; \
    fi \
    # === 處理使用者 ===
    # 檢查該 UID 是否已存在
    && if getent passwd $USER_ID > /dev/null; then \
    # 若存在：取得原使用者名稱，重新命名並移動家目錄
    OLD_USER_NAME=$(getent passwd $USER_ID | cut -d: -f1); \
    usermod -l $USER_NAME $OLD_USER_NAME; \
    usermod -d /home/$USER_NAME -m $USER_NAME; \
    else \
    # 若不存在：建立新使用者（指定 shell、UID、GID、建立家目錄）
    useradd -s /bin/bash --uid $USER_ID --gid $GROUP_ID -m $USER_NAME; \
    fi \
    # === 設定 sudo 權限 ===
    && apt-get update \
    && apt-get install -y sudo \
    # 允許該使用者以 root 權限執行任何指令且不需密碼
    && echo $USER_NAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USER_NAME \
    # 設定 sudoers 檔案權限為 0440（僅 root 可讀）
    && chmod 0440 /etc/sudoers.d/$USER_NAME



# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USER_NAME

# Install Rust toolchain (system-wide) and maturin for PyO3/maturin builds
ENV RUSTUP_HOME=/usr/local/rustup \
    CARGO_HOME=/usr/local/cargo \
    PATH=/usr/local/cargo/bin:$PATH

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
    sh -s -- -y --default-toolchain stable --no-modify-path && \
    chown -R $USER_NAME:$USER_NAME /usr/local/rustup /usr/local/cargo && \
    pip3 install --break-system-packages maturin

RUN rosdep update

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USER_NAME}/.bashrc && \
    echo "export CARGO_HOME=/usr/local/cargo" >> /home/${USER_NAME}/.bashrc && \
    echo "export RUSTUP_HOME=/usr/local/rustup" >> /home/${USER_NAME}/.bashrc && \
    echo "export PATH=/usr/local/cargo/bin:\$PATH" >> /home/${USER_NAME}/.bashrc
