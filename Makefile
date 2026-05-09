COMPOSE_DEV := docker compose -f .devcontainer/docker-compose.dev.yaml
CONTAINER_WS := /home/$${USER_NAME:-fxrbindi}/mower_ws
CONTAINER_ROS_ENV := cd $(CONTAINER_WS) && source /opt/ros/$${ROS_DISTRO:-jazzy}/setup.bash && source install/setup.bash
LOCAL_ROS_ENV := source /opt/ros/$${ROS_DISTRO:-jazzy}/setup.bash && source install/setup.bash

.PHONY: run mower_qt mission mission-docking-temp apriltag-docking mower_teleop teleop-keyboard blade-teleop-joy

build:  
	colcon build --symlink-install

build-release:
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

build-sim:
	MOWER_BUILD_TARGET=sim colcon build --symlink-install --packages-skip mower_controller

clean:
	powershell -NoProfile -ExecutionPolicy Bypass -Command "$$root=(Resolve-Path '.').Path; $$targets=@('build','install','log','logs'); foreach ($$name in $$targets) { $$path=Join-Path $$root $$name; if ((Test-Path -LiteralPath $$path) -and ((Resolve-Path -LiteralPath $$path).Path.StartsWith($$root))) { Remove-Item -LiteralPath $$path -Recurse -Force } }"

rviz: open_rviz

open_rviz:
	bash -lc '$(LOCAL_ROS_ENV) && ros2 launch mower_bringup rviz.launch.py'

run: mower_qt

mower_qt:
	bash -lc '$(LOCAL_ROS_ENV) && ros2 run mower_qt mower_qt'

mission:
	bash -lc '$(LOCAL_ROS_ENV) && ros2 launch mower_mission mission.launch.py'

mission-docking-temp:
	bash -lc '$(LOCAL_ROS_ENV) && ros2 launch mower_mission mission.launch.py launch_temp_dock_pose_publisher:=true'

apriltag-docking:
	bash -lc '$(LOCAL_ROS_ENV) && ros2 launch mower_bringup apriltag_docking.launch.py'

mower_teleop: teleop-keyboard

teleop-keyboard:
	bash -lc '$(LOCAL_ROS_ENV) && ros2 run mower_teleop teleop_keyboard'

blade-teleop-joy:
	bash -lc '$(LOCAL_ROS_ENV) && ros2 run mower_teleop blade_teleop_joy'

test-system:
	ros2 launch nav2_gps_waypoint_follower test_system.launch.py

sim-containers:
	$(COMPOSE_DEV) up -d gazebo lawan_node_dev

sim-gazebo:
	bash -lc '$(LOCAL_ROS_ENV) && ros2 launch mower_bringup sim.launch.py use_sim_time:=true use_rviz:=true enable_localization:=true enable_navigation:=true'

sim-coverage-system:
	bash -lc '$(LOCAL_ROS_ENV) && ros2 launch mower_bringup system_test.launch.py launch_sim:=false use_sim_time:=true use_rviz:=false'

sim-coverage-test: sim-gazebo sim-coverage-system

env:
	bash ./workspace/export_env.sh

build_dev:
	$(COMPOSE_DEV) up -d

deps:
	rosdep install  --ignore-src --from-paths src -i --rosdistro jazzy -y
