LOCAL_ROS_ENV := source /opt/ros/$${ROS_DISTRO:-jazzy}/setup.bash && source install/setup.bash

.PHONY: run mower_qt

build:  
	colcon build --symlink-install

build-release:
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

run: mower_qt

mower_qt:
	bash -lc '$(LOCAL_ROS_ENV) && ros2 run mower_qt mower_qt'

test-system:
	ros2 launch nav2_gps_waypoint_follower test_system.launch.py

env:
	bash ./workspace/export_env.sh

build_dev:
	docker-compose -f .devcontainer/docker-compose.dev.yaml up -d

deps:
	rosdep install  --ignore-src --from-paths src -i --rosdistro jazzy -y

