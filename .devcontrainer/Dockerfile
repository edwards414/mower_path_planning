FROM ros:jazzy-desktop-full

# -------- install Gazebo Harmonic --------
RUN apt-get update && apt-get install -y \
    ros-jazzy-gz-harmonic \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-gz-ros2-control-demos \
    ros-jazzy-image-tools \
    ros-jazzy-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

# -------- X11 & GPU Support --------
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV LIBGL_ALWAYS_SOFTWARE=0

# -------- create developer user --------
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd -m -s /bin/bash --uid $USER_UID --gid $USER_GID $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/$USERNAME

USER $USERNAME

# -------- auto source --------
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

WORKDIR /home/$USERNAME/ros2_ws
