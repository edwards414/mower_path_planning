COMPOSE_DEV := docker compose -f .devcontainer/docker-compose.dev.yaml
CONTAINER_WS := /home/$${USER_NAME:-fxrbindi}/mower_ws
CONTAINER_ROS_ENV := cd $(CONTAINER_WS) && source /opt/ros/$${ROS_DISTRO:-jazzy}/setup.bash && source install/setup.bash
# Raise CycloneDDS participant-index limit (default ~9 exhausts domain 0 once
# the full stack + gazebo run). Respects an externally-set CYCLONEDDS_URI.
LOCAL_ROS_ENV := export CYCLONEDDS_URI=$${CYCLONEDDS_URI:-file://$$(pwd)/.devcontainer/cyclonedds.xml} && source /opt/ros/$${ROS_DISTRO:-jazzy}/setup.bash && source install/setup.bash
SIM_PACKAGES_SKIP := mower_controller
ZIGZAG_ANGLE_DEG ?= 0.0

.PHONY: \
	build build-release build-sim clean rviz open_rviz run \
	bringup component-bringup mower_qt mission mission-coverage record replay mission-docking-temp apriltag-docking mower_teleop \
	teleop-keyboard sim-containers sim-prefetch-gazebo-models \
	sim-gazebo sim-gazebo-empty sim-coverage-system sim-coverage-system-rust \
	sim-coverage-test env build_dev deps

build:  
	colcon build --symlink-install

build-release:
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

build-sim:
	MOWER_BUILD_TARGET=sim colcon build --symlink-install --packages-skip $(SIM_PACKAGES_SKIP)

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
	bash -lc '$(LOCAL_ROS_ENV) && ros2 launch mower_bringup robot.launch.py'

# Hardware components without app-facing mission/coordinator services.
# This target is for bench debugging only; autonomous Nav2 remains fail-locked.
component-bringup:
	bash -lc '$(LOCAL_ROS_ENV) && ros2 launch mower_bringup mower.launch.py'

mission:
	bash -lc '$(LOCAL_ROS_ENV) && ros2 launch mower_mission mission.launch.py'

# mission + auto-drive coverage (load_zone_list -> create_free_space ->
# create_risk_map -> generate_coverage_path) so a path is ready for the app.
mission-coverage:
	bash -lc '$(LOCAL_ROS_ENV) && ros2 launch mower_mission mission.launch.py auto_coverage:=true'

# ── Record / replay real data ───────────────────────────────────────────────
# Record the app-facing topics from a LIVE real run (run this while your real
# sim-coverage stack + sim are up, then Ctrl-C):  make record BAG=bags/my_run
# Replay the recorded REAL data on a dedicated domain (app connects to 9090):
#   make replay BAG=bags/my_run
REPLAY_DOMAIN ?= 7
BAG ?= bags/real_scene
APP_TOPICS := /robot/online /adapter/robot_pose \
	/adapter/map_layers/free_space_inflated /adapter/map_layers/risk_map_inflated \
	/adapter/map_layers/map_grid /adapter/map_layers/chennal_map_inflated \
	/adapter/marker_layers/zones /adapter/marker_layers/risk_zones \
	/adapter/marker_layers/channels /adapter/marker_layers/coverage_path \
	/adapter/marker_layers/invalid_segments /adapter/marker_layers/connectors \
	/adapter/coverage_settings /adapter/zone_summaries

record:
	bash -lc '$(LOCAL_ROS_ENV) && ros2 bag record -o $(BAG) $(APP_TOPICS)'

replay:
	bash -lc 'export ROS_DOMAIN_ID=$(REPLAY_DOMAIN) && export ROS_LOCALHOST_ONLY=1 && $(LOCAL_ROS_ENV) && ros2 launch mower_mission replay.launch.py bag:=$(BAG)'

mission-docking-temp:
	bash -lc '$(LOCAL_ROS_ENV) && ros2 launch mower_mission mission.launch.py launch_temp_dock_pose_publisher:=true'

apriltag-docking:
	bash -lc '$(LOCAL_ROS_ENV) && ros2 launch mower_bringup apriltag_docking.launch.py'

mower_teleop: teleop-keyboard

teleop-keyboard:
	bash -lc '$(LOCAL_ROS_ENV) && ros2 run mower_teleop teleop_keyboard --ros-args -r /cmd_vel:=/keyboard_cmd_vel'

sim-containers:
	$(COMPOSE_DEV) up -d gazebo lawan_node_dev

sim-prefetch-gazebo-models: sim-containers
	$(COMPOSE_DEV) exec gazebo bash workspace/prefetch_gazebo_models.sh

sim-gazebo:
	bash -lc '$(LOCAL_ROS_ENV) && ros2 launch mower_sim sim.launch.py use_sim_time:=true use_rviz:=true'

sim-gazebo-empty:
	bash -lc '$(LOCAL_ROS_ENV) && ros2 launch mower_bringup sim_with_nav.launch.py use_sim_time:=true use_rviz:=true enable_localization:=true enable_navigation:=true world:=empty.world'

# The coverage planner is the mower_rs mower_coverage node (built by
# build-sim with the rest of mower_rs).
sim-coverage-system:
	bash -lc '$(LOCAL_ROS_ENV) && ros2 launch mower_bringup system_test.launch.py launch_sim:=true use_sim_time:=true use_rviz:=false zigzag_angle_deg:=$(ZIGZAG_ANGLE_DEG)'

# Legacy name from when a Python planner existed; same run as sim-coverage-system.
sim-coverage-system-rust: sim-coverage-system

sim-coverage-test: sim-coverage-system

env:
	bash ./workspace/export_env.sh

build_dev:
	$(COMPOSE_DEV) up -d

deps:
	rosdep install  --ignore-src --from-paths src -i --rosdistro jazzy -y
