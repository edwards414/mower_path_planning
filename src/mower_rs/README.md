# mower_rs

Rust replacements for the rclpy nodes whose job is moving data, built on
[r2r](https://github.com/sequenceplanner/r2r) (plain `cargo build` against
the sourced ROS 2 Jazzy environment, no colcon plugins). Background, per
process measurements and the roll-out order: `docs/RUST_REFACTOR_PLAN.md`.

| binary | replaces | switch |
|---|---|---|
| `robot_status` | `heartbeat_node` + `robot_info_node` + `telemetry_node` (`/robot/online`, `/robot/info`, `/robot/telemetry`, `/system/update`, `/system/restart`, `robot_status.json`, update lights) | `mission.launch.py rust_nodes:=true` |
| `velocity_command_guard` | `mower_bringup/velocity_command_guard.py`, both instances (manual guard with command session, final guard mux -> ros2_control) | `twist_mux.launch.py rust_nodes:=true` |

`robot.launch.py rust_nodes:=true` (compose: `RUST_NODES=true` in `.env`)
forwards the switch to both launch files.

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

## Verification

Unit tests: `cargo test`. Shadow comparison against the Python nodes on
synthetic inputs (identical `/robot/info`, identical `/robot/telemetry` apart
from `robot_id`, identical `/robot/online`): see the record in
`docs/RUST_REFACTOR_PLAN.md`. Production switch-over: deploy with
`rust_nodes:=false`, run the binary manually next to the Python nodes with
its outputs remapped to `/shadow/...`, compare, then flip the launch argument.
