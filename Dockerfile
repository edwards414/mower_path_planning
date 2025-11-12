FROM ros:jazzy-ros-core AS base

# Install packages and dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/jazzy/setup.sh
#禁用ubuntu password
RUN passwd -d ubuntu && passwd -l ubuntu 
# RUN apt-get update && apt-get install -y sudo && \
#     useradd -m ubuntu && \
#     echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# USER ubuntu
ENV LAUNCH_COMMAND='ros2 topic pub /talker std_msgs/msg/String "{data: Hello world}"'

RUN echo 'alias run="su - ubuntu --whitelist-environment=\"ROS_DOMAIN_ID\" /run.sh"' >> /etc/bash.bashrc && \
    # echo 'source /opt/ros/jazzy/setup.bash' >> /etc/bash.bashrc && \
    echo "source /opt/ros/jazzy/setup.bash; echo UID: $UID; echo ROS_DOMAIN_ID: $ROS_DOMAIN_ID; $LAUNCH_COMMAND" >> /run.sh && chmod +x /run.sh

