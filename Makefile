COMPOSE_DEV := docker compose -f .devcontainer/docker-compose.dev.yaml
CONTAINER_WS := /home/$${USER_NAME:-fxrbindi}/mower_ws
CONTAINER_ROS_ENV := cd $(CONTAINER_WS) && source /opt/ros/$${ROS_DISTRO:-jazzy}/setup.bash && source install/setup.bash
LOCAL_ROS_ENV := source /opt/ros/$${ROS_DISTRO:-jazzy}/setup.bash && source install/setup.bash
SIM_PACKAGES_SKIP := mower_controller mower_coverage_core
ZIGZAG_ANGLE_DEG ?= 0.0

.PHONY: \
	build build-release build-sim build-rust-core clean rviz open_rviz run \
	bringup mower_qt mission mission-docking-temp apriltag-docking mower_teleop \
	teleop-keyboard blade-teleop-joy sim-containers sim-prefetch-gazebo-models \
	sim-gazebo sim-gazebo-empty sim-coverage-system sim-coverage-system-rust \
	sim-coverage-test env build_dev deps

build:  
	colcon build --symlink-install

build-release:
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

build-sim:
	MOWER_BUILD_TARGET=sim colcon build --symlink-install --packages-skip $(SIM_PACKAGES_SKIP)

build-rust-core:
	bash -lc '$(LOCAL_ROS_ENV) && colcon build --symlink-install --packages-select mower_coverage_core'

clean:
ifeq ($(OS),Windows_NT)
	powershell -NoProfile -ExecutionPolicy Bypass -Command "$$root=(Resolve-Path '.').Path; $$targets=@('build','install','log','logs'); foreach ($$name in $$targets) { $$path=Join-Path $$root $$name; if ((Test-Path -LiteralPath $$path) -and ((Resolve-Path -LiteralPath $$path).Path.StartsWith($$root))) { Remove-Item -LiteralPath $$path -Recurse -Force } }"
else
	rm -rf build install log logs
endif

rviz: open_rviz

open_rviz:
	bash -lc '$(LOCAL_ROS_ENV) && ros2 launch mower_bringup rviz.launch.py'

run: mower_qt

mower_qt:
	bash -lc '$(LOCAL_ROS_ENV) && ros2 run mower_qt mower_qt'

bringup:
	bash -lc '$(LOCAL_ROS_ENV) && ros2 launch mower_bringup mower.launch.py'

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

sim-containers:
	$(COMPOSE_DEV) up -d gazebo lawan_node_dev

sim-prefetch-gazebo-models: sim-containers
	$(COMPOSE_DEV) exec gazebo bash workspace/prefetch_gazebo_models.sh

sim-gazebo:
	bash -lc '$(LOCAL_ROS_ENV) && ros2 launch mower_sim sim.launch.py use_sim_time:=true use_rviz:=true'

sim-gazebo-empty:
	bash -lc '$(LOCAL_ROS_ENV) && ros2 launch mower_bringup sim_with_nav.launch.py use_sim_time:=true use_rviz:=true enable_localization:=true enable_navigation:=true world:=empty.world'

sim-coverage-system:
	bash -lc '$(LOCAL_ROS_ENV) && ros2 launch mower_bringup system_test.launch.py launch_sim:=false use_sim_time:=true use_rviz:=false coverage_backend:=python zigzag_angle_deg:=$(ZIGZAG_ANGLE_DEG)'

sim-coverage-system-rust:
	bash -lc '$(LOCAL_ROS_ENV) && ros2 launch mower_bringup system_test.launch.py launch_sim:=false use_sim_time:=true use_rviz:=false coverage_backend:=rust zigzag_angle_deg:=$(ZIGZAG_ANGLE_DEG)'

sim-coverage-test: sim-gazebo sim-coverage-system

env:
	bash ./workspace/export_env.sh

build_dev:
	$(COMPOSE_DEV) up -d

deps:
	rosdep install  --ignore-src --from-paths src -i --rosdistro jazzy -y
