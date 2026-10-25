# The oracle environment: the nav2 1.3.13 binaries the robot image ships.
FROM ros:jazzy-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-nav2-navfn-planner ros-jazzy-nav2-regulated-pure-pursuit-controller \
      ros-jazzy-nav2-rotation-shim-controller ros-jazzy-nav2-controller \
      ros-jazzy-nav2-velocity-smoother ros-jazzy-nav2-costmap-2d ros-jazzy-nav2-util \
      nlohmann-json3-dev cmake g++ \
    && rm -rf /var/lib/apt/lists/*
