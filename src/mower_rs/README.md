# mower_rs

Rust replacements for the rclpy nodes whose job is moving data, built on
[r2r](https://github.com/sequenceplanner/r2r) (plain `cargo build` against
the sourced ROS 2 Jazzy environment, no colcon plugins). Background, per
process measurements and the roll-out order: `docs/RUST_REFACTOR_PLAN.md`.

| binary | replaces | switch |
|---|---|---|
| `robot_status` | `heartbeat_node` + `robot_info_node` + `telemetry_node` (`/robot/online`, `/robot/info`, `/robot/telemetry`, `/system/update`, `/system/restart`, `robot_status.json`, update lights) | `mission.launch.py rust_status:=true` |
| `mower_adapter` | `flutter_adapter_node` (`/adapter/map_layers/*`, `/adapter/marker_layers/*`, `/adapter/robot_pose`, `/adapter/coverage_settings`, `/adapter/zone_summaries`, `/adapter/map_datum`) | `mission.launch.py rust_adapter:=true` |
| `velocity_command_guard` | `mower_bringup/velocity_command_guard.py`, both instances (manual guard with command session, final guard mux -> ros2_control) | `twist_mux.launch.py rust_guards:=true` |
| `mower_imu` | `wit_ros2_imu` (WIT serial IMU on `/dev/imu_usb`, `/imu/data`) | `mower.launch.py rust_imu:=true` |
| `mower_ws_bridge` | `rosbridge_auth_proxy` + `rosbridge_websocket` + `rosapi` (pairing gate on `address:9090`, loopback 9091 for the agent, rosbridge v2 subset, `/rosapi/topics`) | `rosbridge.launch.py rust_bridge:=true` |
| `mower_record` | `path_record_node` (zone / risk / channel recording, `/edit_zone`, channel routing, work files, named-site library `/site_op`) | `mission.launch.py rust_record:=true` |
| `mower_nav` | `nav_action_server` (`nav_action` / `nav_action_follow_path` Waypoint actions, dispatch confirmation, `/cancel_nav2`, `/check_nav_status`, `/mission_operation_lock`, sensor health gate, manual/autonomy exclusivity, the 20 Hz `/navigation_coordinator_lock` + `/navigation_safety_stop` fail-safe heartbeat, bounded Nav2 dispatch with the `uncertain` fault latch) | `mission.launch.py rust_nav:=true` |

`robot.launch.py` takes all five switches (compose: `RUST_STATUS` /
`RUST_ADAPTER` / `RUST_GUARDS` / `RUST_IMU` / `RUST_BRIDGE` in
`/opt/mower/.env`) so the safety-critical guards can be enabled last, after
a supervised drive.

## Build

`colcon build` runs `cargo build --release --locked` through `CMakeLists.txt`
and installs the binaries into `lib/mower_rs/`, so `ros2 run mower_rs
robot_status` and `Node(package='mower_rs', ...)` work as for any package.
Requirements on the build host: the Rust toolchain (rustup), `libclang-dev`
(bindgen) and a sourced workspace that already contains `mower_interface`
(colcon orders this through `package.xml`). The Docker builder stage has all
three.

`.cargo/config.toml` restricts r2r's binding generation to the message
packages listed there (`IDL_PACKAGE_FILTER`); add a package to that list
before using a new message type, including its transitive dependencies.

Fast iteration without colcon, inside any container with the workspace
sourced:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
cd src/mower_rs && cargo build --release && cargo test --release
```

## Parameters (robot_status)

One node, so the three former nodes' parameters carry a prefix where they
would collide: `heartbeat_source_topic`, `heartbeat_stale_timeout_s`,
`heartbeat_publish_rate_hz`, `info_publish_rate_hz`,
`telemetry_publish_rate_hz`; the rest keep their Python names (`odom_topic`,
`gps_fix_topic`, `imu_topic`, `state_dir`, `robot_id`, `led_*`, ...). Defaults
mirror the Python nodes; `mission.launch.py` passes the same values it
passed before.

Known intentional difference: `/robot/telemetry`'s `robot_id` is the paired
identity (`identity.json`, as on `/robot/info`) instead of the container
hostname.

## velocity_command_guard

The decision rules are in `crates/velocity_command_guard/src/core.rs`, free
of ROS, one test vector per rule (stale / future / backward stamps, NaN,
lateral components, speed limits, replay after a stop, receipt-time
watchdog, command session). `main.rs` is the r2r shell: relative
`cmd_vel_in` / `cmd_vel_out` / `command_clock` topics, zero published at
start and on SIGINT/SIGTERM, outgoing stamps from the robot's wall clock,
receipt timeouts on a steady clock. Limits are read once and there is no
parameter service, so they cannot be changed at runtime. The Rust guard
does not follow `use_sim_time`, hence production only.

Differential test (`scratch guard_compare.py`): the Python and Rust guards
fed the same command stream produced identical output sequences and
identical rejection / timeout logs, with and without command sessions.

## mower_adapter

`crates/mower_adapter/src/dto.rs` holds the JSON conversions (OccupancyGrid
-> base64 map layer, MarkerArray -> marker layer, ZoneMap[] -> summaries,
rcl_interfaces ParameterValue -> JSON, datum bearing) with unit tests
against the Python DTO layout; `main.rs` owns the latched relays, the pose
gate and the three service clients (get_parameters x2 at 1 Hz, zone map list
at 2 Hz, toLL until the datum locks, 3 s timeout each). Shadow run against
the Python node on identical inputs and fake services: every `/adapter/*`
document identical, map layers byte for byte.

## mower_imu

`crates/mower_imu/src/wit.rs` is the WIT 11-byte frame parser (0x51 accel,
0x52 gyro, 0x53 angle, 0x54 mag, checksum, upstream scale factors, the
Python quaternion formula) with unit tests; `main.rs` keeps the driver's
fail-closed rules: input discarded after a 200 ms host gap or when more
than 88 bytes queue up, an orientation frame published only with fresh
accel and gyro frames, serial errors end the process. Differential test
through socat pty pairs: identical `/imu/data` sequences, drops and
checksum handling to the Python driver.

## mower_ws_bridge

The rosbridge v2 subset the clients use: `subscribe` (type, throttle_rate),
`unsubscribe`, `advertise`/`unadvertise`, `publish`, `call_service` (with
`/rosapi/topics` answered natively) and `status` replies on errors; no
fragments, compression or actions. The allow-lists and the service types
live in `mower_bringup/config/ws_bridge.yaml` (a test keeps them equal to
`rosbridge_params.yaml`; r2r has no service-type graph query, so a new
service needs its type there). One ROS subscription per topic, its QoS
matched to the publishers like rosbridge, one serialisation per message
fanned out to every subscribed client, latched topics replayed to late
subscribers; service clients are reused and answered off the node thread
(no serial `call_services_in_new_thread` queue). The pairing gate is the
same X-Mower-* HMAC hand-shake as `identity.py` (401 otherwise) on the
public listener; the loopback listener has no gate, as before.

Compared against rosbridge_websocket on the same graph: identical `msg` JSON
for String, Bool, BatteryState (NaN -> null), NavSatFix, Header and
PoseStamped, identical service responses including a 1.5 s call, publish
reaching the ROS subscriber, allow-list denials, `/rosapi/topics`, and the
gate (401 without headers / replayed nonce / bad MAC, accepted with a valid
hand-shake and the latched `/robot/online` delivered immediately).

## mower_record

`crates/mower_record`: `geometry.rs` (Douglas-Peucker, point-in-polygon,
edge distance, shoelace area), `site_store.rs` (the WGS84 site files, the
`.active_site` manifest, atomic fsync'd writes, the same local
equirectangular projection as the app), `guard.rs` (the fail-closed
mission-mutation lease against nav_action_server), `recorder.rs` (every
service body, ROS-free behind the `Outputs` trait, all app-facing messages
verbatim) and `main.rs` (node `path_recorder`, 19 services, latched
lists, the 10 Hz pose sampler only while recording, handlers serialised by
one async mutex like the rclpy callback group). Scenario comparison against
the Python node (32 steps: recordings, cancel, edit_zone, channel routing,
site save / edit sync / load under a rotated datum / rename / delete,
load_zone_list, navigation-active rejection): every reply, every work file,
every published list and the 34 lock calls identical.

## mower_nav

`crates/mower_nav`: `geometry.rs` (path admission rules, canonical
dispatch ids, the coverage-point / turn-angle / max-distance splitters),
`state.rs` (the whole coordinator state behind one mutex: admission order,
sensor health decisions with the same covariance eigenvalue gate, manual
hold, mutation lock, `/check_nav_status` JSON, the `/rosout` Nav2 log
ring), `nav2.rs` (generation-correlated terminal evidence for a Nav2 goal)
and `main.rs` (node `nav_action_server`, both action names, the six
services, the health and manual subscriptions, the 20 Hz heartbeat task
with skip-on-miss timing, the 2 Hz uncertain monitor, and the execution
flow: dispatch confirmation, bounded bt_navigator readiness, NavigateToPose
to the coverage start, FollowPath per segment, cancel confirmation with the
`uncertain` latch when Nav2 does not answer). The safety parameters are
plain launch-time values with no parameter service, so nothing can weaken
them at runtime. r2r detail: after an accepted action cancel only
`cancel()` can still deliver a result and rcl only reaches CANCELED from
CANCELING, so `Execution::terminate` picks the transition the way the
rclpy server ends up doing (`_finish_goal_canceled`).

Differential test against the Python node with a mock Nav2 (fake
`bt_navigator/get_state`, `navigate_to_pose` and `follow_path` action
servers that succeed, abort, honour or ignore cancels): 35 steps
(admission rejections, dispatch confirmation timeout and tokens, a full
run with segment splitting, `/cancel_nav2`, action-level cancel, Nav2
failure, mutation lock, manual-command cancel, cancel-timeout `uncertain`
latch and its correlated recovery, `/nav_operation_active` sequence) plus
13 health-gate steps (each sensor source missing in turn, stale-sensor
cancel while running and while pending) identical, apart from the Nav2
error text which the Rust port reports with the real error code where
BasicNavigator lacks `getTaskError()`.

## Verification

Unit tests: `cargo test`. Shadow comparison against the Python nodes on
synthetic inputs (identical `/robot/info`, identical `/robot/telemetry` apart
from `robot_id`, identical `/robot/online`): see the record in
`docs/RUST_REFACTOR_PLAN.md`. Production switch-over: deploy with
`rust_status:=false`, run the binary manually next to the Python nodes with
its outputs remapped to `/shadow/...`, compare, then flip the launch argument.
