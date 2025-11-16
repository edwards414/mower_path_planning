# FROM ros:jazzy-ros-core AS base
FROM ros:jazzy AS base

ARG WORKSPACE=/mower_ws
SHELL ["/bin/bash", "-c"]

# Install packages and dependencies
RUN mkdir -p ${WORKSPACE}/src
WORKDIR $WORKSPACE

COPY ./src ${WORKSPACE}/src
COPY Makefile ${WORKSPACE}/

RUN  . /opt/ros/jazzy/setup.sh \
    && apt-get update \
    && rosdep update \
    && make deps \
    && make build-release \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*


#禁用ubuntu password
RUN passwd -d ubuntu && passwd -l ubuntu 

COPY utiles/docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

RUN mkdir -p /mower_ws/zone_record && chmod 777 /mower_ws/zone_record

# RUN echo aris build = colcon build

USER ubuntu
WORKDIR $WORKSPACE

ENTRYPOINT ["/docker-entrypoint.sh"]

CMD ["ros2", "launch", "nav2_gps_waypoint_follower", "small_test.launch.py"]
