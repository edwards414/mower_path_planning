FROM ros:jazzy-ros-core AS base

#arguments
ARG USER_UID
ARG USER_GID
ARG USER_NAME

#create user and group
RUN groupadd -g ${USER_GID} -o ${USER_NAME}

#create user and set password
RUN useradd -m -u ${USER_UID} -g ${USER_GID} -o -s /bin/bash ${USER_NAME} && yes ${USER_NAME} | passwd ${USER_NAME}


RUN apt-get update && apt-get install -y  git net-tools iputils-ping 

RUN rosdep update --rosdistro $ROS_DISTRO

FROM base AS development

#create workspace
# WORKDIR /car_ws



CMD ["bash"]
# #install development tools
# RUN apt-get update && apt-get install -y  zsh git sudo ssh gdb rsync

# #change shell to zsh
# RUN chsh -s /bin/zsh ${USER_NAME}