build:  
	colcon build --symlink-install

deps:
	rosdep install --from-paths ./ -i -y -r --rosdistro jazzy

build-release:
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

test-system:
	ros2 launch nav2_gps_waypoint_follower test_system.launch.py




