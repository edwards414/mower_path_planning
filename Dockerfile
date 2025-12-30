##############################################
# Stage 1: Base
##############################################
FROM ros:jazzy-ros-base AS base
ARG WORKSPACE=/mower_ws

# 安裝必要工具
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-vcstool \
    python3-colcon-common-extensions \
    build-essential  

RUN rosdep update 

WORKDIR ${WORKSPACE}


##############################################
# Stage 2: Builder
##############################################
FROM base AS builder
ARG WORKSPACE=/mower_ws

# 複製 src
COPY ./src ${WORKSPACE}/src

# 安裝相依套件
RUN . /opt/ros/jazzy/setup.sh && \
    rosdep install --from-paths src -i --rosdistro jazzy -y

RUN apt-get update && apt-get install -y libusb-1.0-0-dev \
        ros-$ROS_DISTRO-rtcm-msgs \
        ros-$ROS_DISTRO-ublox-dgnss \
        ros-$ROS_DISTRO-ublox-ubx-msgs  \ 
        ros-$ROS_DISTRO-nav2-common 

# # 建置 colcon
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --event-handlers console_cohesion+

RUN rm -rf /var/lib/apt/lists/*
##############################################
# Stage 3: Runtime
##############################################
FROM ros:jazzy-ros-base AS runtime
# 只複製 install（最小部署）

COPY --from=builder /mower_ws/install /mower_ws/install

RUN apt-get update \
    && apt-get install -y python3-opencv \
    && rm -rf /var/lib/apt/lists/*
    
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /mower_ws/install/setup.bash" >> /root/.bashrc
# ENTRYPOINT ["/bin/bash", "-c"]
# 默认启动命令
# CMD ["source /opt/ros/jazzy/setup.bash && source /mower_ws/install/setup.bash && ros2 launch nav2_gps_waypoint_follower small_test.launch.py"]