build:  
	colcon build --symlink-install

build-release:
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

test-system:
	ros2 launch nav2_gps_waypoint_follower test_system.launch.py

env:
	bash ./workspace/export_env.sh

build_dev:
	docker-compose -f .devcontainer/docker-compose.dev.yaml up -d

deps:
	rosdep install  --ignore-src --from-paths src -i --rosdistro jazzy -y

