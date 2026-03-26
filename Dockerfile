##############################################
# Stage 1: Base
##############################################
FROM ros:jazzy-ros-base AS base

# 改用 ENV 確保變數跨階段存活（但僅限繼承的 stage）
ENV WORKSPACE=/mower_ws

# 安裝必要工具
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-vcstool \
    python3-colcon-common-extensions \
    build-essential  \
    libusb-1.0-0-dev \
    libcurl4-openssl-dev \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep update 
WORKDIR ${WORKSPACE}

##############################################
# Stage 2: Builder
##############################################
FROM base AS builder

COPY ./src ${WORKSPACE}/src

# 安裝編譯期與執行期相依套件
RUN apt-get update && \
    . /opt/ros/jazzy/setup.sh && \
    rosdep install --from-paths src -i --rosdistro jazzy -y && \
    rm -rf /var/lib/apt/lists/*
# 修正了 =twist-mux 的錯字
RUN apt-get update && apt-get install -y \
    ros-jazzy-xacro \
    ros-jazzy-nav2-bringup \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-twist-mux \
    && rm -rf /var/lib/apt/lists/*

# 建置 colcon
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --event-handlers console_cohesion+

##############################################
# Stage 3: Runtime
##############################################
FROM ros:jazzy-ros-base AS runtime

ENV WORKSPACE=/mower_ws
WORKDIR ${WORKSPACE}

# 1. 複製編譯好的 install 目錄
COPY --from=builder ${WORKSPACE}/install ${WORKSPACE}/install
# 2. 為了讓 rosdep 能安裝執行期依賴，必須複製 src (建置完若想極致縮小體積可刪除)
COPY --from=builder ${WORKSPACE}/src ${WORKSPACE}/src

# 安裝 Runtime 需要的系統套件與 rosdep 依賴
RUN apt-get update \
    && apt-get install -y python3-opencv \
    # 安裝你在 Builder 額外裝的套件 (因為這是全新 Stage)
    ros-jazzy-xacro \
    ros-jazzy-nav2-bringup \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-twist-mux \
    libusb-1.0-0 \
    libcurl4 \
    && rm -rf /var/lib/apt/lists/*

# 更新 rosdep 並安裝 src 裡面定義的「執行期」依賴
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src --dependency-types=exec -y \
    && rm -rf /var/lib/apt/lists/*

ARG USER_NAME=mower
ARG USER_ID=1000
ARG GROUP_NAME=mower
ARG GROUP_ID=1000

# 建立與主機相同 UID/GID 的使用者
RUN \
    if getent group $GROUP_ID > /dev/null; then \
        OLD_GROUP_NAME=$(getent group $GROUP_ID | cut -d: -f1); \
        groupmod --new-name $USER_NAME $OLD_GROUP_NAME; \
    else \
        groupadd --gid $GROUP_ID $USER_NAME; \
    fi \
    && if getent passwd $USER_ID > /dev/null; then \
        OLD_USER_NAME=$(getent passwd $USER_ID | cut -d: -f1); \
        usermod -l $USER_NAME $OLD_USER_NAME; \
        usermod -d /home/$USER_NAME -m $USER_NAME; \
    else \
        useradd -s /bin/bash --uid $USER_ID --gid $GROUP_ID -m $USER_NAME; \
    fi \
    && apt-get update \
    && apt-get install -y sudo \
    && rm -rf /var/lib/apt/lists/* \
    && echo $USER_NAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USER_NAME \
    && chmod 0440 /etc/sudoers.d/$USER_NAME

# 將使用者加入 video (攝影機) 以及 dialout (STM32/USB Serial) 群組
# RUN 預設就是 root，所以不需要加 sudo
RUN usermod --append --groups video,dialout $USER_NAME

# 設定使用者的 bashrc (加入 ROS 核心與 Workspace 的 source)
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/${USER_NAME}/.bashrc && \
    echo "source ${WORKSPACE}/install/setup.bash" >> /home/${USER_NAME}/.bashrc && \
    chown $USER_ID:$GROUP_ID /home/${USER_NAME}/.bashrc

# 非常重要：切換到該使用者，確保 Container 預設以一般帳號啟動
USER $USER_NAME