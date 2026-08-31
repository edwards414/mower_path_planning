# Mower Path Planning System

<div align="center">
  <img src="img/mower.png" alt="Mower System" width="400">
</div>

Autonomous lawn mower navigation, coverage path planning, simulation, and
mission tooling built on ROS 2 Jazzy and Nav2.

## Overview

This repository is a ROS 2 workspace for an autonomous mower. It includes:

- coverage path generation for mowing zones
- zone, risk-zone, and channel recording
- map generation for free space, risk maps, and channel maps
- Nav2 integration for path execution and localization (docking is dormant)
- Gazebo simulation assets
- STM32 UART hardware control through `ros2_control`
- Qt and rosbridge-facing adapters for operator/front-end control

The current package layout has been refactored around `mower_*` packages. Some
legacy ROS node names and service names remain for compatibility; see
[Naming And Compatibility](#naming-and-compatibility).

## Packages

| Package | Purpose |
| --- | --- |
| `mower_bringup` | Main launch files, Nav2, localization, RViz, Gazebo simulation, rosbridge config |
| `mower_mission` | Zone/path recording, map management, coverage planning, navigation action wrapper, Flutter adapter |
| `mower_interface` | Custom ROS messages, services, and actions |
| `mower_coverage_core` | Optional Rust/PyO3 coverage-planning backend |
| `mower_controller` | Real robot `ros2_control` hardware interface for STM32 UART motor/blade control |
| `mower_description` | URDF/Xacro robot description and mesh assets |
| `mower_teleop` | Keyboard and joystick teleoperation helpers |
| `mower_qt` | PyQt operator panel and STM32 UART monitor |
| `wit_ros2_imu` | WIT IMU ROS 2 node |
| `turtlebot3_mapviz` | Mapviz support package |

## Requirements

- ROS 2 Jazzy
- Ubuntu 24.04 recommended for native ROS 2 Jazzy development
- Docker / Docker Compose for containerized development and runtime builds
- Python 3.10+
- Rust toolchain and `maturin` if building `mower_coverage_core`

## Build

Install dependencies:

```bash
make deps
```

Build the full workspace:

```bash
make build
```

Build a release workspace:

```bash
make build-release
```

Build a simulation-focused workspace while skipping real hardware/Rust packages:

```bash
make build-sim
```

Source the workspace after building:

```bash
source install/setup.bash
```

## Common Commands

Start the Qt operator panel:

```bash
make run
```

Start mission nodes:

```bash
make mission
```

Start RViz:

```bash
make rviz
```

Start keyboard teleop:

```bash
make teleop-keyboard
```

## Simulation

Start Gazebo simulation:

```bash
make sim-gazebo
```

Start Gazebo with the empty test world:

```bash
make sim-gazebo-empty
```

Start the complete Gazebo, Nav2, coordinator, mux, and coverage test graph:

```bash
make sim-coverage-system
```

`sim-coverage-test` is a compatibility alias for the same unified graph:

```bash
make sim-coverage-test
```

If Gazebo needs remote models cached inside the dev container:

```bash
make sim-prefetch-gazebo-models
```

## Runtime Launches

Production real-robot entry point (bringup, mission services, heartbeat, and
rosbridge exactly once):

```bash
ros2 launch mower_bringup robot.launch.py use_sim_time:=false
```

Physical joystick teleop is disabled by default so neutral joystick messages
cannot mask Nav2 commands. Enable drive teleop only with a verified deadman
button index:

```bash
ros2 launch mower_bringup robot.launch.py \
  enable_physical_joystick:=true \
  physical_joystick_enable_button:=4
```

This physical-joystick option is drive-only. Blade joystick teleop and the
blade effort controller are removed from production, and the real hardware
plugin forces the blade output to zero. The standard effort controller has no
publisher-death timeout and could otherwise retain a nonzero command while the
50 Hz hardware loop continuously refreshes the MCU watchdog. Do not re-enable
blade output until a steady-clock watchdog exists inside ros2_control or the
MCU, plus a tested independent hardware blade interlock.

Rosbridge listens on `127.0.0.1` by default. The production Compose file
explicitly binds it to the mower's WireGuard address (`10.77.0.2`) so the
authenticated relay at `10.77.0.1` can reach it without exposing every network
interface. Restrict TCP/9090 to that relay in the mower host firewall. For
direct LAN development only, expose it explicitly and enforce host/network
firewall rules:

```bash
ros2 launch mower_bringup robot.launch.py rosbridge_address:=0.0.0.0
```

The runtime Docker image and both robot Compose files use this entry point.
Compose stores active geometry, named sites, and recorder bags in the
`mower_data` named volume under `/home/mower/.mower`; container replacement or
image pulls therefore do not erase field data. Back up that Docker volume as
part of robot maintenance. Recording is opt-in (`record:=false` by default)
until retention/upload and disk-space monitoring are configured; Compose gives
an enabled recorder 60 seconds to flush during shutdown.

Mission-map edits and navigation share a fail-closed operation lease. If a
service reports that acquiring or releasing this guard is unconfirmed, first
stop the mower and verify that no manual or autonomous command remains active,
then restart the complete robot stack. Restarting only the editing node cannot
clear a lease retained by `nav_action_server`; leases intentionally do not
expire automatically while physical robot state is uncertain.

Navigation dispatch is a correlated two-phase operation. An accepted action
does not authorize Nav2 velocity until CoveragePlanner confirms the matching
UUID; ambiguous acceptance/result responses retain the goal handle and retry a
token-scoped cancel until terminal state is proven. If `nav_action_server`
crashes, its twist-mux heartbeat fail-locks autonomous velocity. Never restart
that node alone while Nav2 remains alive: restart the complete robot/Nav2 stack
so a pre-crash goal cannot survive into a fresh coordinator.

Real-robot navigation is fail-closed: `robot.launch.py` requires a fresh
`/adapter/robot_pose`, a fresh precise `sensor_msgs/NavSatFix` on the one
canonical `gps_fix_topic` (default `/fix`), and fresh precise
`/odometry/gps` output from `navsat_transform`. This prevents an unused raw GPS
topic from satisfying the gate while the localization pipeline runs on stale
prediction. The largest horizontal uncertainty axis must be at most 0.015 m;
loss of any input during a mission requests cancellation. Production fixes the
external contract to `/fix`; if a receiver publishes `/gps/fix`, remap that
receiver output to `/fix` at its own launch boundary.
The repository builds a `gps_runtime` image but does not guess a receiver
device, baud rate, antenna offset, or correction source; the deployment must
configure a real GPS/RTK publisher before navigation can start. Do not disable
`require_navigation_health` on a physical mower.

`mower.launch.py` is a real-hardware entry point and rejects
`use_sim_time:=true`; actuator watchdogs must not freeze when `/clock` stops.
Use `system_test.launch.py` for a complete runnable mission simulation.
`sim_with_nav.launch.py` is only the simulator/Nav2 infrastructure layer and
does not create the saved mission maps needed by coverage services.

Production docking is intentionally disabled. The dormant docking manager and
Nav2 docking server must not be launched until they use the same central
autonomy lease, health gate, correlated dispatch/cancel protocol, blade stop,
and terminal-state proof as coverage navigation.

> **Real-mower release gate:** the current Nav2 configuration has no live
> scan/depth obstacle source and `collision_monitor` is disabled. The fused
> static map now includes inflated boundaries and risk zones, but it cannot see
> a person or object that enters afterward. Do not run unattended until a
> verified obstacle sensor, independent emergency-stop chain, and robot-side
> motor/blade fault stop are installed and acceptance-tested.

Additional real-mower release blockers remain outside software-only tests:

- wheel odometry is still derived from commanded wheel speed rather than
  independently measured encoders, so localization cannot prove that the
  mower stopped, slipped, or followed the requested path;
- the ROS graph does not yet enforce SROS2/domain-bridge permissions. The
  rosbridge allow-list protects remote App traffic, but another process in the
  same ROS domain could publish a downstream drivetrain topic;
- the generic GPS gate validates freshness and covariance, but cannot prove a
  receiver-specific RTK-fixed mode, correction age, antenna calibration, or
  truthful covariance; those checks require the selected receiver driver and
  field acceptance data;
- map mutation currently has no epoch acknowledgement from every active Nav2
  costmap before the mutation lease is released. Do not edit/reload a site and
  immediately drive it on a real mower until that acknowledgement exists.

`mower.launch.py` and `mission.launch.py` remain available separately for
component-level debugging; do not start either one again beside
`robot.launch.py`.

Simulation bringup:

```bash
ros2 launch mower_bringup system_test.launch.py \
  launch_sim:=true use_sim_time:=true
```

Mission services only:

```bash
ros2 launch mower_mission mission.launch.py use_sim_time:=false
```

Coverage path execution currently uses the `nav_action_server` executable:

```bash
ros2 run mower_mission nav_action_server
```

Rosbridge websocket:

```bash
ros2 launch mower_bringup rosbridge.launch.py
```

Rosbridge uses directional topic and service allow-lists from
`mower_bringup/config/rosbridge_params.yaml`; new app-facing ROS APIs must be
added there explicitly.

## Coverage Workflow

Typical service flow:

```bash
ros2 service call /load_zone_list std_srvs/srv/Trigger {}
ros2 service call /create_free_space std_srvs/srv/Trigger {}
ros2 service call /create_risk_map std_srvs/srv/Trigger {}
ros2 service call /generate_coverage_path std_srvs/srv/Trigger {}
ros2 service call /zone_exec_path mower_interface/srv/ZoneExecPath "{zone_id: 1}"
```

Coverage parameters are owned by the `boustrophedon_coverage` node name for
backward compatibility:

- `strip_width_m`: mower cutting strip width, default `0.8`
- `waypoint_spacing_m`: generated waypoint spacing, default `0.2`
- `zigzag_angle_deg`: zigzag scan angle in degrees, default `0.0`, range `0.0`-`180.0`
- `unknown_as_obstacle`: treat unknown cells as obstacles, default `true`
- `min_safe_component_area_m2`: minimum retained safe component area, default `0.05`
- `coverage_pattern`: `zigzag` or `spiral`
- `coverage_backend`: `python` or `rust`
- `allow_backend_fallback`: fall back to Python if Rust backend is unavailable

Map inflation parameters are owned by `map_manage`:

- `inflate_radius_m`: inward free-space/risk inflation radius, minimum/default
  `0.75`; production overrides may only increase it
- `chennal_width_m`: legacy channel width parameter, default `1.2`

## Naming And Compatibility

The project has historical names that are still visible in ROS APIs. New
documentation and UI text should use the preferred names below, while legacy
names remain available until callers are migrated.

| Preferred name | Legacy/current compatibility name | Notes |
| --- | --- | --- |
| `mower_mission` package | `boustrophedon_coverage`, `path_record`, `maphub` | Old package concepts were merged into `mower_mission`; `boustrophedon_coverage` remains a node/executable alias. |
| `mower_bringup` package | `nav2_gps_waypoint_follower` | Launch/config assets now live under `mower_bringup`. |
| `mower_interface` package | `boustrophedon_coverage_interfaces`, `path_record_interface`, `nav2_action_interfaces` | Use `mower_interface` in new code. |
| Channel | `chennal` | Some ROS services, topics, and fields still use the old spelling during migration. |
| Cancel | `cencel` | Prefer `/cancel_nav2`; keep `/cencel_nav2` only as a legacy alias. |
| Spiral | `speiral.py` | Use `path_generators/spiral.py`; the misspelled module is legacy compatibility only. |

## Docker

Build the runtime image:

```bash
docker compose build
```

Start the runtime stack:

```bash
docker compose up -d
```

Enter the runtime container:

```bash
docker compose exec lawan_node bash
```

Development and simulation containers are defined under `.devcontainer/`.

The `mediamtx` service streams the robot cameras to the app over WebRTC (WHEP).
Start just the camera server with `docker compose up -d mediamtx`. See
[docs/webrtc_camera_streaming.md](docs/webrtc_camera_streaming.md) for details,
including how to attach a real camera (synthetic test patterns are used until
one is wired).

## Testing

Run the coverage planner unit tests:

```bash
pytest src/mower_mission/test
```

Run ROS/colcon tests from a ROS 2 environment:

```bash
colcon test
colcon test-result --verbose
```

## Repository Layout

```text
mower_path_planning/
├── .devcontainer/                  # Development, Gazebo, RViz, and Zenoh containers
├── docs/                           # Design notes and implementation specs
├── img/                            # README and documentation images
├── src/
│   ├── mower_bringup/              # Launch, Nav2, localization, simulation, docking
│   ├── mower_controller/           # Real robot hardware interface
│   ├── mower_coverage_core/        # Rust coverage backend
│   ├── mower_description/          # Robot model and assets
│   ├── mower_interface/            # ROS interfaces
│   ├── mower_mission/              # Mission logic and coverage planner
│   ├── mower_qt/                   # Operator panel
│   ├── mower_teleop/               # Teleoperation tools
│   ├── turtlebot3_mapviz/          # Mapviz support
│   └── wit_ros2_imu/               # IMU node
├── zone_record/                    # Saved zones, risk zones, and channel paths
├── Dockerfile
├── docker-compose.yaml
└── Makefile
```

## License

Most maintained packages declare Apache-2.0. Some legacy package metadata still
contains placeholder license text and should be cleaned up before release.

## Maintainer

- Maintainer: fxrbindi
- Email: edwards940428@gmail.com
