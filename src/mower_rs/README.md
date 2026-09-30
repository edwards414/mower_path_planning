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
| `mower_gps` | the ROS `ublox_gps` node that the old `gps` compose service ran (u-blox ZED-F9P on `/dev/gps_rtk`, `/fix`, plus `/gps/status`) | `mower.launch.py enable_gps:=true` (no Python/C++ alternative left) |
| `mower_ws_bridge` | `rosbridge_auth_proxy` + `rosbridge_websocket` + `rosapi` (pairing gate on `address:9090`, loopback 9091 for the agent, rosbridge v2 subset, `/rosapi/topics`) | `rosbridge.launch.py rust_bridge:=true` |
| `mower_record` | `path_record_node` (zone / risk / channel recording, `/edit_zone`, channel routing, work files, named-site library `/site_op`) | `mission.launch.py rust_record:=true` |
| `mower_nav` | `nav_action_server` (`nav_action` / `nav_action_follow_path` Waypoint actions, dispatch confirmation, `/cancel_nav2`, `/check_nav_status`, `/mission_operation_lock`, sensor health gate, manual/autonomy exclusivity, the 20 Hz `/navigation_coordinator_lock` + `/navigation_safety_stop` fail-safe heartbeat, bounded Nav2 dispatch with the `uncertain` fault latch) | `mission.launch.py rust_nav:=true` |
| `mower_battery` | `battery_state_node` + `battery_estimator` (`/battery_state`, `/aon_battery_state` from the `charger` / `analog` objects of `/mower_base/telemetry`) | `mission.launch.py rust_battery:=true` |
| `mower_pid_autotune` | `pid_autotune_node` + `pid_tuning` (`/pid_autotune` start/abort/apply/discard, latched `/pid_autotune/status` JSON, open-loop FOPDT identification + SIMC PI, closed-loop verification, `/mower_base/pid_command` / `wheel_override` / `led_command`, mutation lock) | `mission.launch.py rust_pid_autotune:=true` |
| `mower_map` | `map_manage_node` (`/create_free_space`, `/create_risk_map`, `/create_chennal_map`, `/import_image_mask`, `/restore_free_space_coverage`, `/get_zone_map_list_srv`, the eight latched map topics, `/map_manage/{get,set,list,describe}_parameters` with the 0.75 m `inflate_radius_m` floor) | `mission.launch.py rust_map:=true` |
| `mower_coverage` | the former rclpy `coverage_node`, removed on 2026-09-30, so this is the only coverage planner (`/generate_coverage_path` with the zigzag / spiral planner, A* connectors, validation and boundary ring; `/zone_exec_path`, `/run_zone_sequence`, `/stop_zone_sequence`, `/resume_coverage` through the `nav_action_follow_path` action with bounded acceptance, dispatch confirmation and correlated cancel tracking; `/boustrophedon_coverage/*_parameters`) | none, always on: `mission.launch.py` starts it, or `robot.launch.py rust_daemon:=true` runs it as the `coverage` module of `mower_rsd` (the `rust_coverage` switch was removed with the Python node) |
| `mower_base` | the whole `ros2_control` chain: `ros2_control_node` (`controller_manager`), `mower_hardware::MowerSystem` and its `mower_hardware_info` node, `diff_drive_controller` (`diff_controller`), `joint_state_broadcaster` and the two `spawner` processes (`/odom`, `/joint_states`, `/mower_base/telemetry`, `/mower_base/firmware_info` and the five `/mower_base/*_command` channels); also `mission.launch.py`'s `odom_throttle` (`/odom_slow`) | `mower.launch.py rust_base:=true` (+ `mission.launch.py rust_base:=true` for the throttle; `robot.launch.py` passes both) |
| `mower_localize` | `robot_localization`'s two `ekf_node`s and `navsat_transform_node` (`/odometry/local`, `/odometry/global`, `/odometry/gps`, `/gps/filtered`, the `odom -> base_footprint` and `map -> odom` broadcasts, `/toLL`, `/fromLL`, `/fromLLArray`, `/datum`); also `mission.launch.py`'s `global_odom_throttle` (`/odometry/global_slow`) | `dual_ekf_navsat.launch.py rust_localize:=true` (+ `mission.launch.py rust_localize:=true` for the throttle; `robot.launch.py` passes both) |
| `mower_rsd` | nothing: it *is* the binaries below, as modules of one process on one r2r Context (one DDS participant). See the section after this table. | `robot.launch.py rust_daemon:=true` |
| `mower_agent` | `mower_agent` (registration with the provision token, the `mrelay1` relay WebSocket with 10 s heartbeats, phone sessions piped to the pairing gate, WHEP signaling relayed to MediaMTX, TURN credentials into the MediaMTX API; no ROS, reads `/robot/info` + `/robot/telemetry` through the loopback bridge) | `rosbridge.launch.py rust_agent:=true` |

Not every crate is a node: `mower_base_core` is a library only — the ROS-free
core of the Rust base driver (STM32 serial protocol, diff-drive odometry and
speed limits, the `BaseCycle` hardware state machine, serial record/replay).
It has no `r2r` dependency, so `cargo test -p mower_base_core` runs anywhere;
see [`crates/mower_base_core/README.md`](crates/mower_base_core/README.md).

`robot.launch.py` takes all the switches (compose: `RUST_STATUS` /
`RUST_ADAPTER` / `RUST_GUARDS` / `RUST_IMU` / `RUST_BRIDGE` / `RUST_BASE` in
`/opt/mower/.env`) so the safety-critical ones — the guards and the base
driver — can be enabled last, after a supervised drive.

## `mower_rsd`: the same modules in one process

Each crate above is a library (`run(ctx, ModuleCtx)`) plus a three-line
binary, so the same code runs either as its own process or as a module of
`mower_rsd`. `rust_daemon:=true` on `robot.launch.py` starts one `mower_rsd`
with the module set derived from the `rust_*` switches (plus `enable_gps`,
plus `coverage`, which has no switch and is always in the set) and holds the
separate binaries down; `rust_daemon:=false` is the roll-back
and changes nothing else. Background: `docs/ROS_FREE_PLAN.md` Phase A5.

```bash
mower_rsd --modules status,guards,adapter,battery \
          [--module-args bridge=--address 0.0.0.0 --port 9090 ...] \
          [--worker-threads 4] [--stop-timeout 10] \
          --ros-args --params-file config/mower_rsd.yaml \
                     -r imu:imu/data_raw:=imu/data
mower_rsd --list-modules      # module id, launch switch ("(always)" for coverage), node name(s)
```

Why one process: `r2r::Context::create()` is a process-wide `OnceLock`, so
thirteen modules share one context, i.e. **one DDS participant**, one set of
discovery threads and one tokio runtime instead of one of each per binary.

What does **not** change: every module still creates its own `r2r::Node` with
its own name, so the graph, the topic and service and action names, and the
per-node parameter services are identical.

* **Parameters** come from an ordinary `--params-file` with a section per
  *node* name (`mower_bringup/config/mower_rsd.yaml`). rcl parses it into the
  shared context and `Node::create` hands each node the section addressed to
  it (or `/**`), which is exactly what happened when each node had its own
  `--ros-args`. Copy the file into `~/.mower` and pass
  `rust_daemon_params_file:=` to change values without a new image; the
  localize nodes are the exception (`localize_params_file:=`, below).
* **Remapping** uses rcl's node-scoped rules, `-r <node>:<from>:=<to>`, which
  is how the IMU driver's and the two velocity guards' launch `remappings=`
  are reproduced. A bare `-r __node:=x` must never be passed to `mower_rsd`:
  it would rename *every* node in the process. That is also why the module
  table carries the production node names — `path_record_node`, `imu` and
  `gps` differ from the binaries' own defaults.
* **The bridge and the agent** keep their command line, passed as
  `--module-args <id>=<args>`.
* **Supervision**: each module is a task wrapped in `catch_unwind`. One that
  panics, fails, or returns before it was asked to stop logs the reason and
  is started again 2 s later on its own (the launch files' `respawn_delay`)
  while the others keep running — a driver whose USB port is gone restarts
  in a loop exactly as its separate binary did, but inside the same DDS
  participant, so it no longer floods discovery. Only a failure during a
  shutdown, or a module that does not stop within the 10 s unwind window,
  makes the process exit non-zero so launch's `respawn` restarts the set.
  There is deliberately no `panic = "abort"`. (The first version tripped the
  whole process on any failure; on a robot without its IMU/GPS attached that
  restarted all 13 nodes every few seconds.)

Verification (`tools/shadow_compare.py`, run inside a container with the
workspace sourced):

```bash
tools/shadow_compare.py --mode split  --bin-dir <target>/release \
    --params-file <params.yaml> --out /tmp/split.json
tools/shadow_compare.py --mode daemon --bin-dir <target>/release \
    --params-file <params.yaml> --out /tmp/daemon.json
tools/shadow_compare.py --diff /tmp/split.json /tmp/daemon.json
# and, for the cost:
tools/shadow_compare.py --mode split|daemon --measure 30 --modules ...
```


## `mower_base`: the base driver (ROS_FREE_PLAN Phase B)

`rust_base:=true` on `mower.launch.py` replaces five processes with one
node: `ros2_control_node`, the `mower_hardware::MowerSystem` plugin inside
it (including its `mower_hardware_info` side-channel node),
`diff_drive_controller` as `diff_controller`, `joint_state_broadcaster`, and
the two `controller_manager/spawner` processes. On the LubanCat the chain
costs 28 % of a core for a 25 Hz loop that moves eight bytes each way.
`robot_state_publisher` is **not** replaced; it stays, and it is why
`/joint_states` still has to come out every cycle.

Every computation is `mower_base_core`, which is checked against the
original C++ with generated vectors (`crates/mower_base_core/README.md`):
the frame format and CRC, `control_toolbox::RateLimiter`,
`diff_drive_controller::Odometry`, one controller update cycle, the
`MowerSystem` read/write state machine and the telemetry JSON. The crate
here is the transport wrapper — parameters, publishers, subscriptions, the
serial thread and the fail-closed path.

### What it reproduces

| direction | topic | type | QoS | rate | comes from |
|---|---|---|---|---|---|
| sub | `/drivetrain_guarded_cmd_vel` | `geometry_msgs/TwistStamped` | `SystemDefaultsQoS` | whatever twist_mux + the final guard produce | `diff_controller`'s `~/cmd_vel`, remapped in `controller_test.launch.py`. Jazzy's controller is TwistStamped-only; the `use_stamped_vel: true` in the yaml is a no-op left over from Iron. `header.stamp` handled as upstream: 0 means now, a message already `cmd_vel_timeout` old is ignored, the timeout runs from the stamp |
| pub | `/odom` | `nav_msgs/Odometry` | `SystemDefaultsQoS` | `publish_rate` 25 Hz | `diff_controller`'s `~/odom`, remapped. `odom` -> `base_footprint`, covariance diagonals from the yaml (unset there, so the controller's own zeros) |
| pub | `/tf` | `tf2_msgs/TFMessage` | `SystemDefaultsQoS` | 25 Hz **only if `enable_odom_tf`** | production sets `enable_odom_tf: false` — the two EKFs own `odom -> base_footprint` — so nothing is published; the switch is implemented for parity |
| pub | `/joint_states` | `sensor_msgs/JointState` | `SystemDefaultsQoS` | every control cycle, 25 Hz | `joint_state_broadcaster` with `use_local_topics: false`. Two joints, `position` and `velocity` real, `effort` two NaNs as the broadcaster writes them, `frame_id` `base_link` (its `frame_id` parameter's default) |
| pub | `/odom_slow` | `nav_msgs/Odometry` | reliable + **volatile**, depth **10**: what `odom_throttle` publishes on the robot, not derived from the `/odom` row above | 5 Hz at most (`odom_slow_rate_hz`); see [the slow copies](#the-slow-copies-instead-of-topic_tools-throttle) | `mission.launch.py`'s `odom_throttle` (`topic_tools throttle messages /odom 5.0 /odom_slow`), which does not start with `rust_base:=true`. `odom_slow_topic: ""` turns it off |
| pub | `/mower_base/telemetry` | `std_msgs/String` (JSON) | best effort, depth 1 | one per new 0x85, gated at `0.8 / telemetry_rate_hz` (20 Hz) -> ~15-17 Hz against a 50 ms frame and a 40 ms loop | `MowerSystem::publish_telemetry_if_due` |
| pub | `/mower_base/firmware_info` | `std_msgs/String` (JSON) | transient local, reliable, depth 1 | once per distinct 0x87 (latched) | `MowerSystem::on_firmware_info` |
| sub | `/mower_base/led_command` | `std_msgs/String` | transient local, reliable, depth 1 | on change, re-asserted every 5 s | `led_topic` |
| sub | `/mower_base/pid_command` | `std_msgs/String` | reliable, depth 4 | one 0x04 per message | `pid_topic` |
| sub | `/mower_base/wheel_override` | `std_msgs/String` | best effort, depth 1 | raw permille while the ttl holds (<= 1000 ms) | `override_topic` |
| sub | `/mower_base/servo_command` | `std_msgs/String` | best effort, depth 1 | one 0x07 per message | `servo_topic` |
| sub | `/mower_base/blade_command` | `std_msgs/String` | best effort, depth 1 | dead-man, refreshed every cycle, one explicit 0 on expiry | `blade_topic` |

`SystemDefaultsQoS` is what the upstream controllers ask for, and this node
asks for the same (`QosProfile::system_default()`), because it is not a
fixed profile: each RMW resolves it. Under `rmw_cyclonedds_cpp`, which the
robot runs, it is reliable + volatile, keep last 1, on both sides; under
Fast DDS the writers come out TRANSIENT_LOCAL and the readers BEST_EFFORT.
An earlier version of this node hard-coded the Fast DDS reading (the dev
images have no RMW set), which on the robot would have made the cmd_vel
reader best effort where the C++ one is reliable. `tools/base_compare.py`
now fails unless every endpoint of every topic this node owns resolves the
same as the C++ chain's; run it under the robot's RMW (`tools/base_ab.sh`).

### What it adds: the arm latch, a monotonic control clock, future stamps

Three deliberate differences from the C++ chain, the first two for the
restart this node gets and the chain never had:

* **The arm latch.** After every (re)activation, and on each of the losses
  below, the cmd_vel reference is held at zero (the limiter brakes it as a
  cmd_vel timeout would) until cmd_vel shows a stop edge that is
  **sustained**: the stream carries no non-zero command for
  `latch_release_s` (0.5 s) — finite 0/0 commands, nothing at all, or
  both — see "the release window" below. `wheel_override` gets the same
  rule on its own stream: not applied, a running override dropped, until
  0/0 or ttl-0 requests or silence have lasted the window (its silence
  also has to outlast the newest request's ttl). The C++ chain deactivated its
  controllers for good on a runtime error; this node is restarted after 2 s
  and would otherwise resume a live nav2 or teleop command by itself. On the
  robot's native UART a pulled lead is no read or write error at all, so
  the losses are read off what the board sends, one trigger per way the
  link can break:

  | what came out | what the host sees | trigger (`arm latch: ...`) |
  |---|---|---|
  | both leads, or the STM32 TX -> LubanCat RX lead | no 0x85 for longer than `feedback_timeout_s` (0.5 s) | **feedback loss** (`wheel feedback lost`) |
  | the LubanCat TX -> STM32 RX lead only (40-pin pin 8 -> PA10), pulled while running or already out when the node starts | 0x85 keeps coming; 0x81 reports COMMAND_TIMEOUT (0x03 with a growing `command_age_ms`, or 0x02 "no command yet" from a board nothing has reached since it booted) | **command path loss** (`the STM32 is not receiving our commands`, plus `STM32 reports COMMAND_TIMEOUT: no 0x01 for N ms ...` once) |
  | the STM32 restarted (reset, brown-out, a flash) | 0.5 s of silence in its bootloader, then (sometimes) 0x81 COMMAND_VALID cleared or the encoder totals back at zero | feedback loss; **STM32 restart** (`STM32 restart: ...`) only when a signature shows, see below |

  The command path loss is two fresh 0x81 in a row with COMMAND_TIMEOUT
  while this node has been writing a 0x01 at least every 150 ms for the
  last 450 ms (the board's 300 ms timeout plus that half again). Then every
  window the board can time out on holds two of our frames, so the report
  can only mean they are not arriving — whether or not the board ever
  showed them arriving since the activation: a lead that was already out
  when the node started (a restart during a pull, a loose connector at
  boot) is caught 0.5 s after the start, before the idle streams could
  re-arm by silence behind it. A single report is CRC-checked and genuine,
  but a dropout that heals before the second one stopped the wheels for at
  most one 50 ms status period — a stutter, not a lurch — and is not worth
  stranding a nav2 goal on; a pulled lead reports every 50 ms. For the
  first 450 ms after the activation, and as long after a stall of this
  loop, the board's timeout is about the time before (the previous driver,
  our own stall) and does not count. Nor do the reports while the feedback
  is lost and for 450 ms after it returns: they describe the outage the
  feedback loss has already latched, and when both leads are re-seated
  together the board's first reports (two, if the contact bounce eats the
  first frames) still predate our first frame landing. If only the STM32
  TX lead came back, the reports go on, and 0.45-0.55 s after the feedback
  the command path loss names the lead in the log; the latch needs no
  second disarm for it, because nothing re-arms on the feedback alone (see
  below). The restart is COMMAND_VALID dropping (the firmware clears it
  only in `MowerMotor_Init`, and reports "no command yet" as flags 0x02,
  which the protocol table listed as 0x00 until this change;
  `firmware/UART_OPEN_LOOP_PROTOCOL.md` is corrected) or both encoder
  totals back near zero by a jump no wheel can make in the time since the
  previous 0x85 (they count from 0 at `WheelController_Init`). The frame
  `seq` (the last 0x01 seq the board accepted, i.e. ours) and the 0x87
  firmware info (sent every second anyway, identical across a restart) are
  not restart signatures. Neither signature shows every restart: the new
  boot's first 0x81 goes out 60 ms after the app starts, by which time one
  of the 25 Hz 0x01 has usually landed, and the jump has to exceed twice
  full speed over the silence plus 0.1 s — after the 0.5 s bootloader about
  1.3 wheel revolutions, more the longer the boot takes — so the totals
  only show a restart after the wheels have turned that far since the
  board's previous boot. What covers every restart today is the 500 ms
  bootloader grace: the feedback loss fires at 0.52 s, before the board is
  back, so the new boot only ever receives zeros. The restart trigger is the
  backstop for a board that comes back faster than `feedback_timeout_s`;
  then up to one status period of the live command reaches the new boot
  before the first 0x85 tells the host, the one gap only the firmware could
  close.

  Re-arming after any of the three losses needs each stream's stop edge
  **and** the link back both ways: nothing re-arms while the feedback is
  still lost, nor until a fresh 0x81 shows the board receiving again
  (COMMAND_TIMEOUT clear, `command_age_ms` within two control periods,
  80 ms; logged as `STM32 receiving commands again`). That holds after a
  feedback loss too (once the board has sent a 0x81 since the activation;
  one that never does gets the feedback rule alone): when both leads are
  out and the STM32 TX one makes contact first, the returning feedback
  alone re-arms nothing, so a push before pin 8 is back — 0.3 s later, or
  2 s — does not ramp up behind it. The 0x01 frames the board receives by
  then are this node's zeros, so a re-seated lead gets 0/0, whatever the
  stick does. When both leads come back together it costs two or three
  cycles: our zeros land within 40 ms and the next 0x81 shows them. The
  activation waits for the same evidence (COMMAND_VALID set,
  COMMAND_TIMEOUT clear) once the board has sent its first 0x81: a lead
  that was already out can only be named after 450 ms of steady writing,
  and a zero cmd_vel inside that window would otherwise re-arm behind it,
  so that a push and a contact before 0.5 s still reached the wheels as a
  step. A working lead shows up on the first status after our first
  frame, one or two cycles. On the cycle the link returns, a stream that
  has been at rest for the release window re-arms, one at rest for less
  re-arms once the window is up (unless it moves first), and a pushed one
  stays held. Feedback that never arrived since activation only logs a
  warning — a board that does not send 0x85 is not bricked.
  The blade is held too, on the three losses (not at activation): a
  running blade gets one explicit 0 at the disarm (the firmware's own
  timeout has stopped it if the lead is out), and `blade_command` requests
  are not applied until its dead-man has been let go — an explicit stop, or
  no refresh within the ttl of the last one, either for the release window
  — after the link is back. The app refreshes a held blade every 0.2 s;
  without this, a blade held through a pulled lead would restart on the
  re-seat.

  **The release window** (`latch_release_s`, 0.5 s, in
  `mower_bringup/config/mower_rsd.yaml`). Found on the robot on 2026-09-29
  (image ecdd4a8, wheels off the ground, the app's teleop stream held
  forward by a script at 10 Hz): pin 8 pulled at 38.5 s, disarmed by the
  command path loss; the board received again at 39.6 s and the wheels
  stayed stopped under the held stick — correct so far. Pulling the jumper
  had also reset the USB hub (IMU and camera re-enumerated), and during
  that the robot side stalled for ~0.3 s: at 43.05 s `velocity_command_guard`
  logged `Drivetrain command receipt timeout; forced velocity to zero` (the
  manual guard the same 35 ms later, then `velocity timestamp is stale`
  rejections), although the sender on the Mac never paused. At 43.07 s
  `mower_base` logged `cmd_vel re-armed by an explicit stop, 4.48 s after
  the command path loss` — the guard's synthesized zero taken for the
  operator's release — and 0.25 s later the still-held stick drove the
  wheels, 73 -> 366 permille. A single stop is therefore no longer a
  release: a stream re-arms only once it has carried no non-zero command
  for the window. Explicit zeros and silence count together: the window
  runs from the first stop after the newest non-zero command, through more
  zeros or nothing at all, and a non-zero inside it restarts it at the next
  stop; silence alone needs the longer of twice the window and the
  stream's own timeout (`cmd_vel_timeout` 0.25 s, so 1.0 s), still counted
  only while the stream can deliver (the discovery rule below). Silence is
  the weaker evidence: the app and nav2 both send a zero when they let go,
  while a stall of the ROS side (the guards, the executor) is silence to
  this driver's loop, which keeps running, so a stall of up to 1.0 s is not
  a release either. The blade's dead-man gets the same window: a 0
  followed by a refresh within 0.5 s is not a let-go, whoever sent the 0,
  and a lapsed ttl is only a let-go once the refresh is 1.0 s old; it never re-arms
  earlier than it did without the window. The first time per disarm that a
  rest the old rule would have taken (a stop, or silence past the stream's
  timeout, on a cycle where the link would let it re-arm) is cut short by a
  non-zero, the log says so at INFO: `arm latch: cmd_vel stop not held for
  0.50 s (non-zero after 0.24 s), still held` (or `silence not held`,
  `wheel_override ...`, `blade_command ...`). What it costs: a release now
  takes effect 0.5 s after the stick is let go, on a latch that is holding
  (normal driving is never latched); at activation the idle streams re-arm
  by silence at `publisher_settle_s` + 1.0 s after their publisher
  appears, or 0.5 s into the zeros an app at rest sends; after an outage
  at rest they re-arm once the board receives again and 1.0 s have passed
  since the disarm. `latch_release_s: 0` restores the old rule.
  A gap in this driver's own loop (more than two control periods, 80 ms at
  25 Hz: the whole robot stalled) starts every held stream's window again
  at the end of the gap, and says so: `arm latch: the control loop did not
  run for 0.74 s; ...`. The review of d354537 found that without it a stall
  longer than the silence window re-armed on the first cycle after it, and
  the stick that was still held drove the wheels. Known limit, left to the
  guards: a zero that `manual_velocity_guard` or `velocity_command_guard`
  forces is still a zero here. If the app's messages keep arriving stale
  (the echoed `/manual_command_clock` older than the guard's max age, as on
  a slow relay link), the manual guard publishes a zero for each at 10 Hz
  while the stick is held, 0.5 s of them re-arm the stream, and the first
  message that then passes drives. The fix is for the guards to mark the
  zeros they force (or not to send them while the latch holds), not a
  longer window here.
  Two rules keep "silence" honest. A restarted node's reader hears nothing
  from a live writer until DDS discovery has matched the two (2.7 s has been
  seen under load), so silence only counts once the topic has had a
  publisher in the graph for `publisher_settle_s` (2 s), or after a message
  has actually arrived; with neither, only an explicit stop re-arms
  (`cmd_vel: publisher in the graph ... after activation` is logged).
  `mower_pid_autotune`'s flash save stalls the STM32 for about a second (no
  frames, no commands taken), which trips the feedback loss (or, for a
  shorter erase, the command path loss); by then the run's steps are over,
  the board shows our frames arriving on the first status after the stall,
  and the idle override stream re-arms by its silence since the disarm once
  that is 0.5 s old, or 0.5 s after the run's closing cancel (0/0, ttl 0) at
  the latest. The next run starts with 0.6 s of 0/0 overrides
  (`hold(0, 0.6)`) before its first step, longer than the window, so it
  drives the wheels as usual even when started at once. `arm_latch: false` (the
  parity tests only) turns off all of it, the 0x81 checks and the blade
  included.
* **The control clock is CLOCK_MONOTONIC**, as ros2_control's steady trigger
  clock is: the loop period, the command age, the feedback age, the blade,
  override and LED deadlines, the telemetry throttle and the telemetry `t`
  (seconds of uptime, as the C++ printed). The system clock only stamps
  message headers and ages a cmd_vel by its own stamp, so a wall-clock step
  cannot pulse the wheels through the limiter or stretch a dead-man.
* **A cmd_vel stamped in the future ages from its arrival.** Accepting and
  ignoring are exactly diff_drive_controller 4.42.1 (zero stamp = now,
  `now - stamp >= cmd_vel_timeout` ignored), but upstream keeps a
  future-stamped command alive until stamp + timeout. A message stamped just
  before the wall clock steps back by S would then keep driving for
  S + 0.25 s if the stream stopped after it; here it times out 0.25 s after
  it arrived.

The transitions the C++ logged are logged with the same text: `no wheel
feedback for %.2f s` / `feedback resumed`, `driver alarm flag set` (2 s
throttle), the wheel override and blade dead-man edges, and `Velocity
command timed out. Braking.` (once per timeout of a non-zero command, not
every second). A blade or override request that is not JSON is a stop / a
cancel, with a warning.

Frames on the wire, unchanged: 0x01 every cycle with the firmware's own
300 ms `command_timeout_ms`, 0x02/0x03/0x04/0x07 as requested, 0x05 to ack a
0x86 SHUTDOWN_REQUESTED (and then `shutdown_command`), 0x06 once at start-up.
`cmd_vel_timeout` (0.25 s) zeroes the *reference* and the limiter ramps the
command down at `max_deceleration` — the ramp is the safety-zero, the
command is never dropped in one step.

The parameter defaults are `mower_controller/controllers/diff_drive_controller.yaml`,
the file the robot actually launches, **not**
`mower_hardware/config/mower_controllers.yaml`, which is a bench file and
disagrees with it on `open_loop`, `enable_odom_tf`, `base_frame_id`, the
wheel geometry and `cmd_vel_timeout`. Only `device` and `shutdown_command`
are written out, in `mower_bringup/config/mower_rsd.yaml`, plus the slow
copy's `odom_slow_topic` / `odom_slow_rate_hz` at their defaults.

### What it deliberately does not reproduce

* **`/controller_manager/*` services** (`list_controllers`,
  `switch_controller`, `load_controller`, ...) and the controller lifecycle.
  Nothing in this repo calls them — `ros2controlcli` and the two spawners
  are the only users, and the spawners go away with the chain. `ros2 control
  list_controllers` on a robot running `rust_base:=true` finds nothing.
* **pal_statistics / `~/introspection_data`**, four topics at the loop rate
  with no subscriber (~323 KB/s on the robot).
* **`/dynamic_joint_states`** (`control_msgs/DynamicJointState`), which
  `joint_state_broadcaster` also publishes and which carried
  `MowerSystem`'s eleven diagnostic state interfaces (`mower_base/crc_errors`,
  `feedback_age_s`, `power_flags`, `firmware_version`, ...). No consumer:
  every one of those values is already in the `/mower_base/telemetry` JSON,
  which is what the app, `mower_battery`, `mower_pid_autotune` and
  `robot_status` read.
* **`~/cmd_vel_out`** (`publish_limited_velocity`, off in the yaml) and the
  chainable-controller reference interfaces.
* **The `robot_description` parameter/topic.** The driver has its own
  parameters; `robot_state_publisher` still publishes `/robot_description`
  for everything else.

### Verification

The serial port cannot be opened twice, so the two implementations are run
one after the other against a fake STM32 on a pty. `tools/base_ab.sh` runs
one side inside the runtime image — the robot's RMW (CycloneDDS with
`/etc/mower/cyclonedds.xml`), which matters, because `SystemDefaultsQoS`
resolves differently under Fast DDS:

```bash
run() { docker run --rm -u 0 --cap-add NET_ADMIN --cap-add SYS_NICE -e ROS_DOMAIN_ID=61 \
          -v "$PWD":/repo -v /tmp/ab:/out --entrypoint bash \
          mower_path_planning:ros-free-test /repo/src/mower_rs/tools/base_ab.sh "$@"; }
run A /out/a                                     # ros2_control chain
run B /out/b /repo/path/to/new/mower_base        # mower_base (default: the image's)
python3 src/mower_rs/tools/base_compare.py --a /tmp/ab/a --b /tmp/ab/b   # exit 1 on a QoS difference or a failed check
run A /out/a_pull "" --scenario pull             # the cable pull, both sides
run B /out/b_pull /repo/path/to/new/mower_base --scenario pull
python3 src/mower_rs/tools/base_compare.py --a /tmp/ab/a_pull --b /tmp/ab/b_pull
# the same shape for --scenario pullpush (pushed while the lead is out),
# --scenario txpull (the LubanCat TX lead alone: the feedback keeps coming),
# --scenario txstart (that lead already out when the driver starts),
# --scenario live (a restart under a stream that is already running) and
# --scenario autotune (mower_pid_autotune, from $AUTOTUNE_BIN or next to the
# B binary, through a run with a flash save and a second run after it); give
# live a loaded variant too, discovery latency is what it is about:
#   docker run --cpus=1 -e LOAD=8 ... base_ab.sh B /out/b_live_load <bin> --scenario live
```

`fake_base.py` answers 0x85 and 0x81 at the firmware's 50 ms period, the
0x85 from a deterministic first-order wheel model driven by the received
0x01 commands, the 0x81 with COMMAND_VALID / COMMAND_TIMEOUT /
`command_age_ms` as `motor.cpp` computes them; 0x82/0x83/0x84/0x86/0x87/
0x88/0x89 once a second, a 0x84 right after each 0x04, and a 1 s stall
(nothing sent, nothing taken) for a 0x04 that saves to flash, like the
sector erase. It logs every frame with a timestamp. `base_harness.py` publishes the same scripted
`/drivetrain_guarded_cmd_vel` (a *continuous* ramp, so the up-to-40 ms phase
difference between the two runs' control loops costs `a*h^2/2` per corner
instead of `v*h`) and the same side-channel bursts, and records `/odom`,
`/joint_states`, `/tf`, `/mower_base/telemetry` and the graph's QoS. It
waits for every endpoint to match before it starts the clock — without that
the run whose driver started later loses its first seconds of `/odom` and
the two trajectories are offset by the test, not by the code.

Result of a 32 s run on an arm64 Jazzy container (2026-09-23, **Fast DDS**,
before the pre-flight fixes; the Rust node is a **debug** build here, the
release one is faster):

| | `ros2_control` | `mower_base` |
|---|---|---|
| `/odom`, `/joint_states` | 25.001 / 25.002 Hz | 25.004 / 25.004 Hz |
| `/tf` (from `robot_state_publisher`) | 25.11 Hz | 25.06 Hz |
| `/mower_base/telemetry` | 15.07 Hz | 15.37 Hz |
| graph endpoints + QoS on all six topics | — | identical |
| telemetry JSON keys | 84 | 84, none missing or extra |
| odometry | — | 1.9e-3 m apart after 3.006 m travelled (0.06 %); mean \|dx\| 9.8e-4 m, max \|dv_x\| 2.5e-3 m/s |
| wheel angle in `/joint_states` | 39.4606 / 31.6384 rad | 39.4733 / 31.6525 rad (1.3e-2 rad after 39 rad) |
| 0x01 command frames | 25.007 Hz, `timeout_ms` 300 | 25.029 Hz, `timeout_ms` 300; median \|d permille\| = 1 on a 40 ms grid |
| safety zero after the ramp | 13.532 s | 13.520 s (12 ms, under one cycle) |
| wheel_override burst | 56 frames, 16.533-18.733 s | 56 frames, 16.520-18.720 s (13 ms, under one cycle) |
| 0x02 / 0x03 / 0x04 / 0x06 / 0x07 payloads | — | byte-identical |
| CPU, utime+stime over 34 s | 19.16 % of a core | **6.49 %** of a core |

The odometry residual is the two runs' control loops sampling the same
command script at different instants, not a difference in the arithmetic:
`mower_base_core`'s oracle vectors already pin the limiter, the odometry and
a 200-cycle controller run to 1e-12 against the original C++. Repeats of the
whole run land between 8.6e-4 and 1.9e-3 m, in either direction.

Re-run 2026-09-28 after the pre-flight fixes, in the runtime image under
CycloneDDS (`base_ab.sh`, release build), on a host that another job kept
at a load of ~40:

* **QoS: all eleven topics, every endpoint, identical** — reliability,
  durability, history, depth, liveliness. `/odom`, `/tf`, `/joint_states`
  and the cmd_vel reader resolve to reliable + volatile, keep last 1.
* 0x02/0x03/0x04/0x06/0x07 payloads identical, telemetry keys 84 = 84,
  `/joint_states` `frame_id` `base_link` on both, telemetry `t` on the same
  (uptime) clock, the override burst and the safety zero within a cycle.
  The latch never engaged in this script: it arms by silence 0.28 s after
  activation, long before the harness starts.
* The odometry was 0.12 m apart after 2.9 m, and that was the load: at
  6.55 s the C++ run logged `Velocity command timed out. Braking.` after a
  0.18 s stall of the harness and re-ramped from zero; the Rust run had no
  such stall. Take the 2026-09-23 numbers above for the arithmetic.
* `--scenario pull` (stick held at 0.30 m/s, lead out 4-6 s): after the
  re-seat the C++ chain sent 71/71 frames at 549 permille from the first
  frame (6.04 s) — the lurch; `mower_base` sent 74/74 at 0/0 with the stick
  still held, and moved again only after the release (`arm latch: cmd_vel
  re-armed by an explicit stop`).

Re-run the same day after a review found two holes in that latch (silence
counted during DDS discovery; re-arming while the lead was still out),
release build of `fix/rf-base-preflight`, same image and RMW:

* Default script: QoS all eleven topics identical; odometry 6.1e-4 m apart
  after 3.0 m; telemetry keys 84 = 84; side-channel payloads identical.
  The latch now arms by silence 2.57 s after activation (publisher in the
  graph at 0.28 s + `publisher_settle_s` + 0.25 s), still before the
  harness starts.
* `--scenario live` (0.30 m/s at 20 Hz running before the driver starts):
  C++ 230 of 246 frames non-zero from 2.82 s, `mower_base` 0 of 257 until
  the release, then followed the push. Loaded (`--cpus=1`, `LOAD=8`), twice:
  0 of 114 and 0 of 120, re-armed only by the release's explicit stop. (A
  first loaded attempt, before `base_ab.sh` waited for the stream to flow,
  had the harness itself stall: two real 0.27-0.29 s gaps in its own
  stream, which *are* the silence stop edge, and the base followed after
  the second. That is the specified rule, and the C++ controller braked
  and resumed on the same gaps.)
* `--scenario pullpush` (idle pull, pushed while out, re-seated pushed):
  C++ 50/50 frames at 549 permille from the first frame after the re-seat;
  `mower_base` 0/49, re-armed by the release. `--scenario pull` again:
  C++ 71/71 non-zero, `mower_base` 0/75.

Re-run 2026-09-28 for the command-path and restart triggers
(`fix/rf-base-cmdtimeout`: release build, same image, CycloneDDS,
`ROS_DOMAIN_ID=72`, a host at a load of ~7), with `fake_base.py`'s 0x81 now
computed as the firmware does:

* `--scenario txpull` (stick held at 0.30 m/s, only the LubanCat TX lead out
  4-6 s). The fake's view is the robot's: 40 feedback frames still sent
  during the cut, 35 of 40 0x81 with COMMAND_TIMEOUT from 0.30 s after it,
  `command_age_ms` up to ~2000. C++: after the re-seat 75 of 75 frames at
  549 permille from the first one (6.02 s), wheels up to 31.8 rpm: the
  lurch. `mower_base`: `STM32 reports COMMAND_TIMEOUT: no 0x01 for 361 ms`
  and the disarm, once; after the re-seat 0 of 75 non-zero and the wheels at
  0 rpm, `STM32 receiving commands again (command_age_ms 22)`, re-armed by
  the release's explicit stop and followed the next push (47 non-zero
  frames).
* `--scenario autotune`, both sides: `precheck -> open_loop -> fitting ->
  verify -> review -> saving -> done`, then a second run to `review ->
  idle`. The save's 1 s stall left one 0x81 with COMMAND_TIMEOUT
  (`command_age_ms` 1001 / 1002) behind it; `mower_base` logged the feedback
  loss at 0.52 s and re-armed both streams by silence on the cycle the
  feedback came back (a report while the feedback is lost does not count).
  The second run's 400 / 700 / 500 permille steps reached the fake (205 and
  206 non-zero frames after the save).
* No change elsewhere, and no command path loss or restart logged in any of
  them: `pull` C++ 75/75 non-zero after the re-seat, `mower_base` 0/75 with
  only the feedback-loss lines; `pullpush` 50/50 vs 0/50; `live` 223 of 240
  vs 0 of 248; the default script with QoS identical on all eleven topics,
  telemetry keys 84 = 84, the side-channel payloads identical, odometry
  2.3e-4 m apart after 3.0 m.

Re-run 2026-09-29 after a review of those triggers (same branch: the
command path loss no longer waits for the board to have shown it receiving
first; nothing re-arms, at activation or after a feedback loss, before a
board that sends 0x81 shows it receiving; the blade is held on the three
losses; `base_compare.py` fails a run on its scenario checks). Release
build, same image, CycloneDDS, `ROS_DOMAIN_ID=73`, a host at a load of
7-11. Every pair below ends in `verdict PASS`; the same txpull pair with
the sides swapped, and an autotune run edited to fail, exit 1.

* `--scenario txstart` (the LubanCat TX lead out from before the driver
  started until 7.0 s; idle, pushed from 5.0 s, held through the re-seat):
  the fake reported 0x02 with `command_age_ms` 65535 in all 140 0x81 while
  it was out. C++: 50 of 50 frames at 549 permille from the first one after
  the re-seat (7.03 s), wheels up to 31.8 rpm. `mower_base`: `STM32 reports
  COMMAND_TIMEOUT: no 0x01 for 65535 ms or more` and the disarm 0.52 s
  after the start, nothing re-armed by the idle silence behind it, 0 of 50
  non-zero after the re-seat and 0 rpm, `STM32 receiving commands again
  (command_age_ms 17)`, re-armed by the release's explicit stop, 48
  non-zero frames after the next push.
* `txpull`: C++ 75 of 75 at 549 permille from 6.02 s; `mower_base` disarmed
  at `no 0x01 for 355 ms`, 0 of 75, receiving again 1.68 s after the loss,
  re-armed by the release.
* `pull` / `pullpush`: C++ 75/75 and 50/50 non-zero; `mower_base` 0/74 and
  0/50, and now `STM32 receiving commands again` once the feedback was back
  (1.60 s / 2.56 s after the loss) before anything re-armed.
* `autotune`: both sides `saving -> done`, then a second run to `idle`, no
  error. `mower_base` logged the feedback loss 0.52 s into the 1 s stall,
  `feedback resumed`, `receiving commands again (command_age_ms 8)` 0.56 s
  after the loss, and re-armed both streams on that cycle; the second run's
  400 / 500 / 700 permille steps reached the fake (206 non-zero frames after
  the save, C++ 205).
* `live`: C++ 231 of 239 non-zero, `mower_base` 0 of 248, then followed the
  push after the release. Default script: QoS identical on all eleven
  topics, odometry 5.3e-4 m apart after 3.0 m, telemetry keys 84 = 84,
  side-channel payloads identical, the activation re-armed by silence at
  2.56 s as before.
* No false command path loss or restart in any run. One feedback loss
  that is not in a script: at the end of the `txstart` B run, after the
  scenario, the fake itself stalled for 0.67 s (the harness was writing its
  results on the loaded host). `mower_base` latched, saw the board
  receiving 0.12 s later and re-armed 0.16 s after the loss.

### Capturing a real serial recording on the robot

For the exact, timing-free comparison, capture what `ros2_control` really
does on the robot and replay it offline — the format and both capture
recipes (a `socat` pty tee, or `strace` on the live process, neither of
which changes the running stack) are in
[`crates/mower_base_core/README.md`](crates/mower_base_core/README.md#capturing-a-real-one-on-the-robot).
Record `/cmd_vel` and `/odom` alongside (`ros2 bag record
/drivetrain_guarded_cmd_vel /odom /joint_states /mower_base/telemetry`): the
capture has the rx bytes but not the commands that produced them, so the
replay is driven from the bag and `/odom` from the same bag is the
reference. `tests/replay.rs` is the harness; point it at the new
`.mowerlog`.

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

The telemetry `host` block (`host.rs`) is what Mower Studio's 主機監控 page
draws: `top`'s CPU line and per-core %, cpufreq and thermal zones, `free`,
`df` of the state dir (and `bags/` when it is a separate drive), per-drive
throughput and eMMC wear, and the ten busiest processes. It is sampled every
2 s outside the state lock, by scanning /proc (the container runs with
`pid: host`); about 2 ms per scan. `mower_mission/host_stats.py` is the
Python twin with the same test vectors.

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

## mower_gps

`crates/mower_ubx` is the UBX layer with no dependencies (host-testable):
resynchronising frame parser, Fletcher checksum, CFG-RATE / CFG-MSG
encoders, NAV-PVT / NAV-HPPOSLLH / NAV-EOE / ACK decoders and the
`NavSatFix` mapping of the ROS `ublox_gps` driver (NO_FIX unless gnssFixOK
with a 2D/3D/GNSS+DR fix, GBAS_FIX for an RTK fixed carrier solution,
covariance = hAcc², NaN position without a fix). `crates/mower_gps` opens
`/dev/gps_rtk`, enables PVT + HPPOSLLH + EOE at `rate_hz` on the USB port
(RAM only, ACK-checked, everything else in the receiver untouched), builds
one fix per epoch on NAV-EOE with the HPPOSLLH millimetre digits, and
publishes a 1 Hz JSON `gps/status`. It ends the process on a serial error,
EOF, or 5 s without NAV-PVT so launch respawns it onto the re-enumerated
device; the ROS driver spins forever on a dead port instead. Verified on
the robot 2026-09-19: 4.00 Hz, ~2 % of a core with the receiver streaming
its full message set, exit 0.5 s after a simulated unplug, restart on the
same tty minor.

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

## mower_battery

`crates/mower_battery`: `estimator.rs` is the SOC model line for line
(OCV table, sag filter, rate-limited recovery, voltage-only charge ramp,
post-charge re-anchor, coulomb counting with rest re-anchor and the
full-tail detector) with the 14 Python test vectors as `cargo test`;
`main.rs` parses the 16.7 Hz telemetry JSON with Python truthiness for
the `valid` / `online` flags, keeps the meter-first / ADC-fallback voltage
choice, the charger-presence threshold, the signed-current mapping, the
one-shot low-battery warning and the 1 Hz `BatteryState` layout (NaN for
unmeasured fields, `percentage` on 0..1, health from the cell voltage). A
frame whose `charger` / `analog` is not an object is ignored (the Python
node used to crash on it; fixed there too). Differential run against the
Python node on one synthetic telemetry stream (silence, meter, sag,
charger, charger without current, unplugged, ADC fallback + AON cell,
garbage frames, stale, low pack), voltage-only and coulomb-counting
variants: every 1 Hz sample identical within the timers' phase offset.

## mower_pid_autotune

`crates/mower_pid_autotune`: `tuning.rs` is a line-by-line port of
`pid_tuning.py` (FOPDT grid fit with the same 40 x 30 coarse grid and two
refinement passes, `check_model` limits, SIMC PI with the tau floor,
`step_metrics`, the closed-loop simulator used by the tests); the pytest
vectors are `cargo test`s and `tests/pid_tuning_golden.json` holds fits the
Python module produced on fixed sample sets, matched to 1e-9. `main.rs`
keeps the node name `pid_autotune`, every parameter and default, the
`/pid_autotune` service semantics (`start` refused while a session runs,
`apply` / `discard` only in `review`), the 5 Hz latched status JSON with the
same key order, the base side-channel JSON, the `0x84` acknowledgement rules
(fresh `last_rx_seq`, matching gains, mode and `LAST_APPLY_OK`, `FLASH_VALID`
for a persisted save, one retry with `flash_diag`), the review timeout, the
restore of the previous gains in every exit path and the mission operation
lock (missing service = bench, carry on).

Intentional differences: the `/mower_base/telemetry` subscription is
permanent (the Python node created it per session because rclpy paid ~6 ms
per frame; staleness is still judged from the arrival time), and the
progress-0.85 status (verify metrics attached) is usually coalesced into the
0.90 `review` status by the depth-1 latched writer because the two publishes
are microseconds apart -- `verify` stays in every later status.

## mower_map

`crates/mower_map`: `raster.rs` re-implements the OpenCV 4.6.0 primitives the
Python node calls (`fillPoly`, `polylines` thickness 1, `line` with a
thickness and round caps, `getStructuringElement(MORPH_ELLIPSE)`, `erode` /
`dilate` with the default and the constant-0 border) from
`modules/imgproc/src/drawing.cpp` of that release -- fixed-point edges,
Bresenham, `FillConvexPoly`, `FillEdgeCollection`, `Circle` -- and
`tests/raster_oracle.json` holds 720 random cases rendered by the same
OpenCV build, matched pixel for pixel. `grid.rs` is `nav_map_fusion.py` +
`map_safety.py` + `image_mask_import.py` + the risk / zone / channel
rasterisation of the node with numpy's truncation, floor/ceil and integral
image semantics; the pytest vectors are `cargo test`s and
`tests/map_oracle.json` checks the node's own code on the differential
run's geometry. `main.rs` keeps the node name `map_manage`, every service,
message text, log line, latched topic, the fail-closed Nav2 snapshots, the
image-mission backup / restore, the mutation guard on every mutation and
the parameter services with rclpy's validation order (declared type first,
then the node's callback, then the guarded refresh).

One subtlety carried over on purpose: rclpy keeps the Python float in the
float32 `resolution` field until the message is serialised, so the node's
cell indices come from 0.05 exactly while subscribers see
0.05000000074505806. `Map` keeps that f64 next to the message so the Rust
node computes the same indices (and the same bytes).

## mower_coverage

`crates/mower_coverage` is the only coverage planner: there is no rclpy
version and no `rust_coverage` / `RUST_COVERAGE` switch any more (the Python
`coverage_node`, its `coverage/` and `path_generators/` modules and the PyO3
backend were removed on 2026-09-30). It links `mower_coverage_core`, a plain
Rust library crate with no PyO3 and no `python` feature, through a cargo path
dependency (`src/mower_coverage_core`, built by this workspace's `cargo
build`, not a colcon package; its own tests run with `cargo test` in that
directory). `contours.rs` is `cv2.findContours(RETR_EXTERNAL,
CHAIN_APPROX_NONE)` + `contourArea` from OpenCV 4.6.0's `contours.cpp`
(Suzuki border following, newest-first output) for the boundary ring,
checked against 160 masks rendered by that build. `lib.rs` keeps the node
name `boustrophedon_coverage`, every service, parameter default and
message (`coverage_backend` and `allow_backend_fallback` are still accepted
as startup-only legacy parameters, but the planning always runs in
`mower_coverage_core`), the marker layout (colours, ids, arrow every fifth
pose), the risk resampling with `unknown_as_obstacle`, the mission guard,
and the whole dispatch protocol of `_send_follow_path`: 3 s acceptance
deadline with the late-acceptance cancel, `/check_nav_status` reason on
rejection,
`/confirm_navigation_dispatch`, background or blocking (600 s) result,
zone sequences with `/get_channel_route`, and the tracker that cancels the
action, watches the acknowledgment and retries `/cancel_navigation_dispatch`
every 2 s (one live attempt per dispatch id, 0.5-30 s response deadline)
until a terminal state is proven.

## mower_localize

`crates/mower_localize` is the ROS shell; every number comes from
[`crates/mower_localize_core`](crates/mower_localize_core/README.md), the
line-by-line port of robot_localization 3.8.3 that is checked against the
installed `librl_lib.so` on generated vectors (1.3e-15 on the filter state,
1.6e-9 m on the navsat output). Phase C of `docs/ROS_FREE_PLAN.md`.

**Three nodes, one module.** `ekf_filter_node_odom`, `ekf_filter_node_map` and
`navsat_transform` keep their names, so `ros2 node list`, every log line and
the per-node parameter services look as they did. They are three `r2r::Node`s
and — under `mower_rsd` — three supervised instances, so one filter failing
restarts only itself, the same blast radius as the three launch `Node`s; the
`mower_localize` binary runs the same three roles in one process and returns
`Err` if any of them stops. `run()` picks the role from the node name.

The external contract, from the launch file, the yaml and the consumers:

| | topic / service | QoS | rate |
|---|---|---|---|
| in | `/odom`, `/imu/data`, `/odometry/gps` (EKFs) | best effort, keep last `<input>_queue_size` (10) | as published |
| in | `/fix`, `/imu/data`, `/odometry/global` (navsat) | best effort, keep last 1 | as published |
| in | `/tf_static` | reliable, transient local | once |
| out | `/odometry/local` (odom EKF), `/odometry/global` (map EKF) | reliable, keep last 10 | 20 Hz |
| out | `/odometry/global_slow` (map EKF; `odometry_slow_topic`, empty on the odom EKF) | reliable, keep last 10 | 5 Hz at most (`odometry_slow_rate_hz`); see [the slow copies](#the-slow-copies-instead-of-topic_tools-throttle) |
| out | `/odometry/gps`, `/gps/filtered` | reliable, keep last 10 | 30 Hz timer, one message per new fix / per new odometry |
| out | `/tf`: `odom -> base_footprint` (odom EKF), `map -> odom` (map EKF) | reliable, keep last 100 | 20 Hz, stamped with the filter's last measurement time |
| out | `/tf_static`: `map -> utm` | reliable, transient local | once, when the datum locks |
| srv | `/toLL`, `/fromLL`, `/fromLLArray`, `/datum` | services default | — |
| srv | `<node>/{get,set,list,describe}_parameters`, `get_parameter_types`, `set_parameters_atomically` | services default | — |

Consumers that must not notice: `mission.launch.py`'s `global_odom_throttle`
(`/odometry/global` -> `/odometry/global_slow`) while it still runs, i.e. with
`rust_localize:=false` there, nav2 (`map -> odom` and
`/odometry/global`), `mower_adapter` (`/toLL` until the datum locks, then
`/adapter/map_datum`), `mower_record` and `mower_nav` (the health gate wants
`/odometry/gps` within 0.30 s of a fix, which is why navsat stays at 30 Hz).

**Not replicated**, none of it reachable or used here:

* `/set_pose` (topic and service), `/enable`, `/toggle`, `/reset` — the EKF's
  runtime controls. Nothing on this robot calls them, and two `ekf_node`s in
  one namespace advertise the same four names anyway.
* `/setUTMZone` — it needs the MGRS zone strings, which the core does not port.
* `/diagnostics` — upstream advertises it unconditionally and publishes only
  when `print_diagnostics` is true; the yaml turns it off (it was ~1 % of a
  core per filter), so the module does not advertise it at all.
* Setting a parameter. `get` / `list` / `describe` answer with what the filter
  is really running; a `set` is refused with a reason rather than accepted and
  ignored, which is what rclcpp does for the many values `ekf_node` reads only
  at start-up.
* Dropping navsat's IMU subscription once the datum is good. Upstream resets
  it; the module only stops feeding the core, so `/imu/data` keeps one extra
  subscriber that reads nothing.

**Configuration** is the C++ nodes' own: each node reads its section of
`mower_nav2/config/dual_ekf_navsat_params.yaml` (both launch paths pass that
file; `localize_params_file:=` on `robot.launch.py` points both stacks at a
copy under `~/.mower` -- `rust_daemon_params_file` cannot, the localize file
is passed after it and wins), resolved by `mower_localize_core::config` the way `loadParams` and the
`NavSatTransform` constructor declare the keys -- upstream defaults for absent
keys, rclcpp's strict typing, a start-up error for a value the port does not
implement, a warning for a key robot_localization does not know. The parameter
services report every declared key with its value in effect. The topic wiring
is the launch file's `remappings=` as node-scoped rules
(`-r ekf_filter_node_map:odometry/filtered:=odometry/global`), since one
process holds three nodes and an unprefixed rule would hit all of them.
The only keys of the module's own are the two EKFs' `odometry_slow_topic` /
`odometry_slow_rate_hz` ([the slow copies](#the-slow-copies-instead-of-topic_tools-throttle)),
which robot_localization has no equivalent of: `ekf_filter_node_map`'s
section in `mower_rsd.yaml` sets them (the defaults, `odometry/global_slow`
at 5.0, and empty on `ekf_filter_node_odom`), they are taken out of the
section before the resolver sees it, and the parameter services report them
with the rest.

**navsat's `delay`** (3 s) is applied as upstream applies it: the constructor
creates everything and then sleeps before the timer exists, with nothing spun,
so the depth-1 inputs hold only the latest sample and the datum is built from
inputs at least 3 s after start. It matters when `/odom` initialises the map
EKF (yaw 0) before the first IMU sample, which with `rust_base` is the normal
order: locking at once rotated the whole map <-> UTM datum by most of the
robot's initial heading. (The first differential run could not see this: the
stack was started long before the stream, so the delay had passed.)
* `tf2_ros::Buffer`. The static sensor offsets come from `/tf_static`; the two
  dynamic transforms the ported code looks up (`base_footprint <- odom` for the
  map EKF, `map <- base_footprint` for navsat) are the ones these filters
  broadcast, so they are shared in process (`src/tfbus.rs`) instead of being
  read back from `/tf` — that topic also carries robot_state_publisher's wheel
  joints at ~77 Hz, about 1 ms of CPU per message on the RK3568. Consequence:
  another publisher of `odom -> base_footprint` or `map -> odom` would be
  invisible. Nothing does that here (`diff_drive_controller` has
  `enable_odom_tf: false`).

**Differential method** (`tools/localize_compare.py`, arm64 Jazzy container,
`ROS_DOMAIN_ID=87`). One rclpy harness plays the whole upstream side —
`/tf_static` with a real GPS-antenna and IMU offset, `/odom` at 25 Hz with
covariance, `/imu/data` at 10 Hz, `/fix` at 4 Hz in UTM zone 51N, with stale
(0.6 s old), out-of-order and duplicate stamps injected — records
`/odometry/local`, `/odometry/global`, `/odometry/gps`, `/gps/filtered` and
both tf broadcasts, then asks `/toLL`, `/fromLL` and `/datum` a fixed set of
questions. Stamps are `t0 + k*dt` from the harness's own start, so two separate
runs share a relative timeline and every sample can be matched to its
counterpart by stamp. The robot stands still for the first two seconds: the
datum is whichever fix is latest when odometry, IMU and GPS have all been seen,
and that race resolves differently in two runs unless the candidates are all
the same fix.

What must be identical is: `/toLL` 7.1e-15, `/fromLL` 4.7e-10 m, `/toLL` after
a `/datum` call 1.1e-14, the `map -> utm` static transform 4.7e-10, every
frame, child frame and status, and the rates.

The filter state cannot be identical, and the honest way to read it is against
two runs of the *same* stack. Worst absolute difference over the matched
samples of a 30 s run:

| | C++ vs C++ | Rust vs Rust | C++ vs Rust |
|---|---|---|---|
| `/odometry/local` position | 2.7e-4 m | 2.4e-4 m | 2.0e-4 m |
| `/odometry/local` orientation | 1.4e-15 | 8.0e-4 | 1.6e-6 |
| `/odometry/local` velocity | 2.2e-5 | 2.4e-4 | 4.3e-4 |
| `/odometry/global` position | 2.28e-2 m | 2.11e-2 m | 2.28e-2 m |
| `/odometry/gps` position | 4.1e-3 m | 3.2e-3 m | 3.3e-3 m |
| `/gps/filtered` lat/lon | 2.3e-7 deg | 2.1e-7 deg | 2.3e-7 deg |
| tf `map -> odom` position | 1.40e-1 m | — | 1.35e-1 m |

Two runs of the C++ stack differ from each other by as much as the Rust module
differs from it, and two runs of the Rust module differ by more. That is
`periodicUpdate` on a wall clock: `prepareTwist` and `prepareAcceleration` take
their lever-arm terms from the *current* filter state and from an angular
acceleration differentiated on the timer, and a tick that finds an empty
measurement queue predicts forward and re-stamps. The differences are
concentrated at the moment the acceleration ramp starts and grow slowly after
it; the arithmetic itself is the core's, at 1e-15 against the real library.

**Cost** (same container, per-process `/proc` utime+stime over 30 s with the
stream playing; indication only, the RK3568 pays far more per process):

| | processes | threads | CPU |
|---|---|---|---|
| `ekf_node` x2 + `navsat_transform_node` | 3 | 48 | 8.2 % of a core |
| `mower_localize` (release) | 1 | 21 | **4.4 %** |

## The slow copies instead of `topic_tools throttle`

`mission.launch.py` runs two C++ `topic_tools throttle` processes whose only
job is a 5 Hz copy of a fast topic for the status nodes: `odom_throttle`
(`/odom` -> `/odom_slow`: `robot_status` / heartbeat, robot_info, telemetry)
and `global_odom_throttle` (`/odometry/global` -> `/odometry/global_slow`:
`flutter_adapter` and `path_record_node`'s `robot_pose_source_topic`). On the
LubanCat the pair cost 6.3 % of a core, about 1.3 ms per *input* message
(`docs/ROS_FREE_PLAN.md` section 1). Once the source topic comes from a
mower_rs module, the module publishes the copy itself
(`mower_rs_common::throttle::SlowCopy`, fed with every message it has just
published on the source) and the throttle is not started:

| copy | published by, when the switch is on | throttle held down | parameters (`mower_rsd.yaml`) |
|---|---|---|---|
| `/odom_slow` | `mower_base` | `odom_throttle`, by `rust_base` | `mower_base`: `odom_slow_topic`, `odom_slow_rate_hz` |
| `/odometry/global_slow` | `mower_localize`'s `ekf_filter_node_map` | `global_odom_throttle`, by `rust_localize` | `ekf_filter_node_map`: `odometry_slow_topic`, `odometry_slow_rate_hz` |

The defaults are the throttles' arguments (the topic, 5.0); an empty topic
turns a copy off, and is the default on `ekf_filter_node_odom` because nothing
throttles `/odometry/local`. A rate that is not a positive number or a topic
name rcl refuses (`/odom_slow/`, a space) is logged and also turns the copy
off: a typo in the yaml costs the 5 Hz status copy, never the module and its
fast topic (`/odom` and the wheels, or `map -> odom`).

`robot.launch.py` hands `rust_base` and `rust_localize` to
`mission.launch.py` as well as to `mower.launch.py`; with both false (the
default) `mission.launch.py` starts exactly the processes it did before.
Launching the two files separately means giving both the same value, or a
copy has two publishers (twice the rate) or none.

What is reproduced, from topic_tools 1.3.4 (the image's
`ros-jazzy-topic-tools`, `src/throttle_node.cpp`, `src/tool_base_node.cpp`):

* **The rule.** A message is forwarded, unchanged, when at least one period
  has passed since the last forwarded one: `now - last >= period`, with
  `period = rclcpp::Rate(5.0).period()` = 200 000 000 ns, and `last` restarts
  at that message. There is no fixed grid and no catching up. The copy is the
  same message the module published, stamp included, not a re-stamped one.
* **The clock.** The time a message is *seen* on the node clock, never its
  header stamp. The throttles run with `use_sim_time` from the launch file,
  which on the robot is false, so their clock is the system clock
  (`CLOCK_REALTIME`), and that is what the copies use. `mower_base` and
  `mower_localize` do not run on simulated time at all (`mower.launch.py` and
  `robot.launch.py` refuse `use_sim_time:=true`). "Seen" is the moment the
  module publishes the source message instead of the moment DDS delivers it
  to a second process.
* **The first message.** `last` starts when the node is constructed, so a
  source message within the first period after start-up is dropped; in
  practice the source starts later than that and its first message goes.
* **A clock stepped backwards** (NTP) restarts the period at the new time and
  drops that message, with the same warning.
* **QoS**, which the throttle derives from the source publisher it
  *discovers*: keep last **10**, the source's reliability and durability,
  automatic liveliness. What it discovers depends on the RMW.
  `diff_drive_controller` creates `/odom` with `rclcpp::SystemDefaultsQoS()`,
  which the robot's rmw_cyclonedds_cpp announces as reliable + volatile,
  keep last 1, and Fast DDS as reliable + transient local.
  So on the robot `odom_throttle` publishes `/odom_slow` reliable +
  **volatile**, keep last 10 (probed in `mower-runtime-main:local`), and
  that is what `mower_base` sets explicitly rather than deriving it from its
  own `/odom`: that asks for `SystemDefaultsQoS` as the controller does, so
  under Fast DDS a derived copy would be transient local and hand a late
  transient-local joiner (a bag recorder) up to 10 stale samples that no
  throttle on the robot ever did. `robot_localization`'s `/odometry/global` is an explicit
  `rclcpp::QoS(10)`, reliable + volatile under every RMW, and so is
  `/odometry/global_slow`. Every subscriber of either (`robot_status`, the
  rclpy status nodes, `mower_adapter` / `flutter_adapter_node`,
  `mower_record` / `path_record_node`, all `QoS(10)` reliable volatile)
  matches both; the app subscribes to neither.

Not reproduced: `lazy` (false in both launch entries, so the throttle also
published with no subscriber), the `bytes` mode, the `qos_overrides.*`
parameters nobody sets, and the throttle's discovery dance (it has no output
publisher until it has seen a source publisher, and drops it when the source
goes away; the module's copy lives exactly as long as its source).

**It is not 5.0 Hz, and never was.** A 25 Hz `/odom` puts every fifth message
exactly one period after the last forwarded one, so arrival jitter decides
whether the fifth or the sixth goes; the same holds for the 20 Hz
`/odometry/global` and its fourth. Both the C++ throttle and the copy come out
at about 4.3-4.5 Hz on these sources, forwarding 200 or 240 ms apart on
`/odom` and 200 or 250 ms apart on `/odometry/global`. The consumers only
judge freshness over seconds (the heartbeat's stale timeout is 2 s).

Verification, in the arm64 Jazzy container with a C++ throttle on the *same*
source next to the module's copy (`tools/slow_copy_check.py`, 30 s,
`mower_rsd --modules base,localize` with `mower_rsd.yaml`, the fake STM32 of
`tools/fake_base.py` driven by `tools/base_harness.py`):

| | source | module's copy | `topic_tools throttle` on the same source |
|---|---|---|---|
| `/odom` (25.003 Hz) | 750 | `/odom_slow` 135, 4.50 Hz, all 135 byte-identical to an `/odom` message | 134, 4.47 Hz, all identical |
| `/odometry/global` (20.001 Hz) | 601 | `/odometry/global_slow` 130, 4.32 Hz, all 130 identical | 130, 4.32 Hz, all identical |
| publisher QoS | — | reliable + volatile on both, keep last 10, automatic (set, not derived) | `/odometry/global_slow_cpp` the same; `/odom_slow_cpp` transient local, derived from `mower_base`'s `/odom`, which was still hard-coded latched then (before the pre-flight fixes; this image's Fast DDS). The robot's throttle over `diff_drive_controller` under Cyclone publishes reliable + volatile, keep last 10 |

The two copies forward the same message for only 56-65 (`/odom`) and 72-106
(`/odometry/global`) of their ~130 over two runs: where the fifth-or-sixth
decision falls depends on each one's own arrival times.
The standalone binaries give the same picture (`mower_base` with
`mower_rsd.yaml` as `base_params_file`: 4.48 vs 4.45 Hz; `mower_localize`
with the slow copy's defaults: 4.32 vs 4.32 Hz).

With slow-copy topics rcl refuses (`odom_slow_topic: /odom_slow/`,
`odometry_slow_topic: odometry/global slow`) both modules log
`RCL_RET_TOPIC_NAME_INVALID`, publish no copy, and keep `/odom` at 25.0 Hz
and `/odometry/global` at 20.0 Hz.

Re-checked 2026-09-28 after merging onto the pre-flight fixes, in the runtime
image under CycloneDDS (`mower_rsd --modules base,localize` with
`mower_rsd.yaml` + `dual_ekf_navsat_params.yaml` and the launch file's
node-scoped remaps, `fake_base.py`, 20 s, a C++ throttle on each source next
to the copy): `/odom` 25.005 Hz, reliable + volatile, keep last 1 (its
`SystemDefaultsQoS` under Cyclone); `/odom_slow` 88 messages, 4.41 Hz, all
identical to an `/odom` message, against 89 / 4.43 Hz from the throttle;
`/odometry/global` 19.99 Hz, `/odometry/global_slow` 86, 4.33 Hz, all
identical, against 87 / 4.33 Hz. All four copies reliable + volatile, keep
last 10: under the robot's RMW the throttle over `mower_base`'s `/odom`
derives exactly what `mower_base` sets. No resolver warning for the two
`odometry_slow_*` keys, and `ros2 param get` answers them on
`ekf_filter_node_map` (`/odometry/global_slow`, 5.0) and
`ekf_filter_node_odom` (empty).

## mower_agent

`crates/mower_agent` has no r2r dependency: tokio + tokio-tungstenite (rustls)
for the relay, gate and loopback-bridge WebSockets, ureq (rustls) on a
blocking thread for the backend and MediaMTX HTTP calls, exactly where the
Python agent used `run_in_executor`. `relay.rs` is `relay_protocol.py`
(frames, chunking, reassembly, the pytest vectors as `cargo test`s).
`main.rs` keeps every log line, control message (`hb`, `opened`, `open_err`,
`close`, `http_res`), the X-Mower-* signing, the `mower-agent/<api>`
User-Agent Cloudflare's browser check needs, the WHEP path rule, the
64 KiB body cap and the header filter, the 1 s..60 s reconnect back-off
with the re-registration on an HTTP refusal, the 30 s register retry, the
TURN refresh schedule and the 0600 atomic `device_key` write.

## Verification

Unit tests: `cargo test`. Shadow comparison against the Python nodes on
synthetic inputs (identical `/robot/info`, identical `/robot/telemetry` apart
from `robot_id`, identical `/robot/online`): see the record in
`docs/RUST_REFACTOR_PLAN.md`. Production switch-over: deploy with
`rust_status:=false`, run the binary manually next to the Python nodes with
its outputs remapped to `/shadow/...`, compare, then flip the launch argument.
