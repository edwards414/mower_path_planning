# Bus topic map for E2

This is the inventory step E2 of [ROS_FREE_PLAN.md](ROS_FREE_PLAN.md) section 8 works from. In E2, each module's IO moves from
`r2r::Node` to a `mower_bus::Scope` ([crates/mower_bus/README.md](../src/mower_rs/crates/mower_bus/README.md)), and
`mower_bus_ros` bridges to DDS whatever is still outside the process. For every topic, service and action that a
`mower_rsd` module touches, this file lists:

- who is on each end;
- the QoS of each end;
- which ends stay on DDS, and until which step.

Section 6 proposes the order.

Snapshot: branch `feat/e1-inventory` at `7157d1f` (main `ccefdf4` plus the E1 base), read on 2026-10-26. The app and Studio
references are to their own repositories, at the commits named in 1.4.

## At a glance

- **15 modules, 18 node instances, 17 r2r nodes** run in `mower_rsd` in production. `mower_agent` has no node.
- **78 topics** have an endpoint in a typed module. The bridge alone adds 5 more. E4's mux module (`rust_mux`, off by
  default) adds `/diagnostics` and a production reader of `/nav_cmd_vel`, its static_tf module
  (`rust_static_tf`, off by default) `/robot_description`: 81. E5's bag module (`rust_recorder`, off by default;
  E5-3a, branch `feat/e5-3a-bag`) adds the recorder's own seven (`/mower_recorder/{status,bags,command,command_result,
  fault}`, `/graph_snapshot`, `/graph_events`): 88.
- **19 topics never leave the process** (5.1). Another **14 have no consumer on the robot** (rviz or the optional data
  collection only).
- **75 services are served by `mower_rsd`**: 45, plus 30 parameter services on 5 nodes. With the bag module, 48: its
  `/mower_recorder/start`, `/stop` and `/snapshot`.
  - 10 of them are called only from inside the process.
  - 27 are reached from the app and the Studio through `mower_ws_bridge`.
- **Actions:** 2 servers. One action pair is fully in-process (coverage calling `nav_action_follow_path`). 2 action
  clients go to nav2.
- **DDS refuses no publisher/subscriber pair today** (5.5). Moving pairs onto the bus therefore connects nothing new. The
  bridge itself can still create refusals, though: section 5.5 lists the traps.
- **Surprises** (5.8):
  - `/mower_base/rear_light` had two publishers and no subscriber since the Rust base replaced `mower_hardware`;
    mower_base reads it again since 2026-10-30 (5.8 #1).
  - The coverage node holds two latched publishers that never publish.
  - The plan's claim that the app calls `/map_manage/get_parameters` is wrong.
  - The E1 README lists only the guards and the IMU as relying on remapping. Four other modules depend on relative names
    too: gps, localize (all three nodes), nav and coverage.
  - The recorder's watched-node fault can never fire, because of a name mismatch.

## 1. Reading this

### 1.1 What runs (production configuration assumed)

**`lawan_node` container.** It runs `ros2 launch mower_bringup robot.launch.py` (compose:34-88).

- **Modules.** The robot's `/opt/mower/.env` is not in the repo. HANDOFF:15 (2026-10-08) and HANDOFF:296-298
  (2026-10-12) record every `RUST_*` switch as true there; `RUST_NAV_CORE` came later. So:
  - all 15 modules run in one `mower_rsd` (robot.launch.py:58-75, 164-256);
  - no rclpy fallback starts.
- **nav backend.** `RUST_NAV_CORE` defaults to false (compose:88) and is assumed off. `nav_action_server` therefore
  drives nav2 (`nav_backend` default `nav2`, mower_nav:1616-1620). Cells marked *nav_core* apply only with
  `rust_nav_core:=true`.
- **Next to `mower_rsd`:**
  - nav2 as one `component_container_isolated`, with 7 servers and the lifecycle manager (navigation.launch.py:61-69,
    277-370; `NAV_COMPOSITION` compose:53);
  - `robot_state_publisher` (mower.launch.py:347-353);
  - `twist_mux` (twist_mux.launch.py:51-58).
- **Off by default:**
  - joystick and keyboard teleop (mower.launch.py:170-186, 333-345, 406-440);
  - AprilTag docking (mower.launch.py:143-147, 319-331);
  - the battery simulator (mower.launch.py:312);
  - `auto_coverage` and `temp_dock_pose_publisher` (mission.launch.py:77-85, 220-242).
- **nav2's `docking_server`** has parameters (nav2_params:240-301), but it is in neither the lifecycle list nor the
  component list, so it never loads.

**`recorder` container** (compose:159-187). It runs mower_recorder's `graph_snapshot`, `recorder_manager` and
`bag_store` (record.launch.py:62-106), all rclpy. These are the only Python ROS nodes on the robot.

- While a recording runs, `recorder_manager` adds a `ros2 bag record` process (record_profile:109-120) and calls
  `ros2 param dump` (recorder_manager:399-404). `autostart:=false` (compose:174), so nothing records until someone
  presses the app's button.
- A data-collection session (`deploy/docker-compose.data-collection.yaml`) records a few more topics
  (data_collection.yaml:12-28). It is named below where it is the only consumer.

**RMW.** CycloneDDS, graph on loopback (Dockerfile:266-269).

### 1.2 Inside and outside

"In `mower_rsd`" means an endpoint of a typed module.

`mower_ws_bridge` and `mower_agent` are modules too, but for E2 they count as outside.

- **The bridge** reaches the graph only through r2r's untyped endpoints and graph queries (mower_ws_bridge/hub:298-421).
  That lasts until E3 rebuilds it on the erased API (bus/README:18-23).
- **The agent** has no ROS endpoint (rsd/main.rs:181-187). It subscribes through the bridge's loopback WebSocket
  (mower_agent:804-813).

So a topic between a module on the bus and the app needs a bus/DDS bridge until E3.

### 1.3 Notation

**QoS.**

- `R` / `BE` = reliable / best effort.
- `V` / `TL` = volatile / transient local.
- The number is the KEEP_LAST depth.
- r2r `QosProfile::default()` is `R V 10`, and `sensor_data()` is `BE V 5` (bus/README:110-117).
- Every module's `latched()` helper is `R TL 1`:
  - robot_status:123-125, mower_map:68-70, mower_coverage:91-93, mower_record:93-95, mower_adapter:67-69,
    mower_nav:1530-1532, mower_nav/core_backend:550-552;
  - mower_base:937 and mower_pid_autotune:834 build the same profile inline.
- **SD** = `QosProfile::system_default()` (rclcpp `SystemDefaultsQoS`). The robot's CycloneDDS resolves it to `R V 1`
  (README:166-175, measured README:1144-1149).
- Services all use `services_default()`.

**"DDS today".** Does DDS refuse the pair now? That happens for a reliable reader on a best-effort writer, or a
transient-local reader on a volatile writer. The bus connects such pairs (bus/README:101-106), so a refused pair would
change behaviour when it moves. There is none (5.5).

**"unconfirmed".** nav2, twist_mux, robot_state_publisher, tf2 and rosbag2 are apt packages in the image. Their QoS is
upstream behaviour that this repo does not contain, and it was not checked here. For step E4, twist_mux 4.5.0,
robot_state_publisher 3.3.4 and nav2's velocity_smoother were measured in the runtime image, offline
(`tools/mux_oracle.py`, `tools/rsp_capture.py`, and the E4 design's `ros2 topic info -v`): those cells say
"measured".

**Rates and sizes.** Given only where the code or a measurement in the docs states them; "unknown" otherwise.

**E2 column.**

- "in-process" means no bridge once both ends are on the bus.
- "bus→DDS until E*n*" / "DDS→bus until E*n*" name the bridge direction and the step that removes the outside end.

### 1.4 Path aliases

Each citation is `alias:line`.

| Alias | File |
|---|---|
| `mower_adapter`, `mower_agent`, `mower_base`, `mower_battery`, `mower_coverage`, `mower_gps`, `mower_imu`, `mower_map`, `mower_nav`, `mower_pid_autotune`, `mower_record`, `robot_status`, `velocity_command_guard` | `src/mower_rs/crates/<name>/src/lib.rs` |
| `velocity_command_guard/core` | `src/mower_rs/crates/velocity_command_guard/src/core.rs` |
| `mower_localize/{ekf,navsat,paramsrv,tfbus}` | `src/mower_rs/crates/mower_localize/src/<file>.rs` |
| `mower_nav/{core_backend,state}` | `src/mower_rs/crates/mower_nav/src/<file>.rs` |
| `mower_ws_bridge/{hub,client}` | `src/mower_rs/crates/mower_ws_bridge/src/<file>.rs` |
| `rsd/main.rs` | `src/mower_rs/crates/mower_rsd/src/main.rs` |
| `bus/README`, `bus/qos.rs` | `src/mower_rs/crates/mower_bus/README.md`, `.../src/qos.rs` |
| `README` | `src/mower_rs/README.md` |
| `robot.launch.py`, `mower.launch.py`, `twist_mux.launch.py` | `src/mower_bringup/launch/` |
| `mission.launch.py` | `src/mower_mission/launch/` |
| `navigation.launch.py` | `src/mower_nav2/launch/` |
| `record.launch.py` | `src/mower_recorder/launch/` |
| `mower_rsd.yaml`, `gps.yaml`, `ws_bridge.yaml`, `twist_mux_topics.yaml` | `src/mower_bringup/config/` |
| `nav2_params` | `src/mower_nav2/config/nav2_no_map_params.yaml` |
| `ekf_params` | `src/mower_nav2/config/dual_ekf_navsat_params.yaml` |
| `record_topics.yaml`, `qos_overrides.yaml`, `data_collection.yaml` | `src/mower_recorder/config/` |
| `recorder_manager`, `bag_store`, `graph_snapshot` | `src/mower_recorder/mower_recorder/<name>_node.py` |
| `record_profile` | `src/mower_recorder/mower_recorder/record_profile.py` |
| `compose`, `Dockerfile` | `deploy/docker-compose.yaml`, `Dockerfile` |
| `PLAN`, `HANDOFF`, `REFACTOR` | `docs/ROS_FREE_PLAN.md`, `docs/ROS_FREE_HANDOFF.md`, `docs/RUST_REFACTOR_PLAN.md` |
| `probe_app.py`, `datum_check.py` | `src/mower_rs/tools/` |
| `app/<file>` | phone app `mower_lawer_app` at origin/main `34cd5d3` (2026-10-25): `lib/providers/<file>.dart`; `manual_control_overlay` is in `lib/widgets/` |
| `studio` | desktop Studio `mower_sudio_app` at `1825fe3` (2026-10-25): `lib/providers/robot_provider.dart`; `studio/rosbridge_client` is `lib/services/rosbridge_client.dart` |

## 2. Topics

### 2.1 Drivetrain and teleoperation

| Topic | Type | Producers | Consumers in mower_rsd | Outside | Rate | DDS today | E2 |
|---|---|---|---|---|---|---|---|
| `/app_joy_cmd` | `geometry_msgs/TwistStamped` | app through ws_bridge, untyped writer `R V 10` (mower_ws_bridge/hub:417; allowed ws_bridge.yaml:30) | `manual_velocity_guard` as `cmd_vel_in`, `R V 1` (velocity_command_guard:84; rule robot.launch.py:83) | app: every 100 ms while a stick is held, plus on drag (app/manual_control_overlay:48, 305) | 10 Hz while driving | no | DDS→bus until E3 |
| `/manual_command_clock` | `std_msgs/Header` | `manual_velocity_guard` as `command_clock`, `R V 1`; created only with `require_command_session` (velocity_command_guard:85-89; robot.launch.py:85; mower_rsd.yaml:81) | — | app via ws_bridge (app/mission_mock_provider:659-662); stale after 800 ms (:167-169) | 20 Hz (`command_clock_period_s` 0.05, velocity_command_guard:79, 90-91) | no | bus→DDS until E3 |
| `/joy_cmd` | `TwistStamped` | `manual_velocity_guard` as `cmd_vel_out`, `R V 1` (velocity_command_guard:83; robot.launch.py:84) | `nav_action_server`, manual source, `R V 10` (mower_nav:1657-1658) | twist_mux input `app_manual`, priority 90, timeout 0.5 s (twist_mux_topics.yaml:13-16); its reader `SD`, `R V 1` (measured) | one per input, plus one forced zero 0.2 s after a non-zero stream stops (velocity_command_guard:73, 165-173; velocity_command_guard/core:232-240) | no | in-process to nav; bus→DDS until E4 |
| `/physical_joy_cmd`, `/keyboard_cmd_vel` | `TwistStamped` | none (teleop off by default: mower.launch.py:170-186, 333-345, 406-440) | `nav_action_server`, `R V 10` (mower_nav:1657) | twist_mux, priorities 100 and 80 (twist_mux_topics.yaml:17-24) | — | n/a | DDS→bus only if teleop is turned on |
| `/nav_cmd_vel` | `TwistStamped` | nav2 `velocity_smoother` (`cmd_vel_smoothed` remapped: navigation.launch.py:351-355, mower.launch.py:315; stamped, nav2_params:238), `R V 1` (measured). *nav_core*: `nav_action_server`, `R V 10` (mower_nav/core_backend:121, 223-224) | — | twist_mux `navigation`, priority 10, timeout 0.5 s (twist_mux_topics.yaml:9-12); its reader `R V 1` (measured) | 20 Hz while navigating (nav2_params:225; mower_nav/core_backend:22-26) | no | nav2: DDS only. *nav_core*: bus→DDS until E4 |
| `/navigation_safety_stop` | `TwistStamped` (zero) | `nav_action_server`, `R V 10` (mower_nav:1630) | — | twist_mux, priority 50, timeout 0.2 s (twist_mux_topics.yaml:5-8) | 20 Hz whenever autonomy is not authorised: only a running, reserved goal is (mower_nav/state:162-171), so all the time at idle (mower_nav:271-282, 1981-1998) | no | bus→DDS until E4 |
| `/navigation_coordinator_lock` | `std_msgs/Bool` | `nav_action_server`, `R V 10` (mower_nav:1631) | — | twist_mux lock, priority 50, timeout 0.3 s (twist_mux_topics.yaml:25-29) | 20 Hz (mower_nav:276, 1981-1998) | no | bus→DDS until E4. A stopped module leaves the lock stale, i.e. locked (mower_nav:2034-2035) |
| `/cmd_vel_guard_input` | `TwistStamped` | twist_mux `cmd_vel_out` (twist_mux.launch.py:57), `R V 1` (`rclcpp::QoS(KeepLast(1))`, measured) | `velocity_command_guard` as `cmd_vel_in`, `R V 1` (velocity_command_guard:84; robot.launch.py:86) | twist_mux | about 20 Hz at idle, from the safety stop (19.7 Hz measured 2026-09, REFACTOR:24) | no | DDS→bus until E4 |
| `/drivetrain_guarded_cmd_vel` | `TwistStamped` | `velocity_command_guard` as `cmd_vel_out`, `R V 1` (velocity_command_guard:83; robot.launch.py:87) | `mower_base`, `SD` (mower_base:896, 965) | — | as its input | no | in-process only; move guard and base together (6, step 6) |

### 2.2 Base driver

| Topic | Type | Producers | Consumers in mower_rsd | Outside | Rate, size | DDS today | E2 |
|---|---|---|---|---|---|---|---|
| `/odom` | `nav_msgs/Odometry` | `mower_base`, `SD` (mower_base:897, 940) | both EKFs as `odom0`, `BE V 10` (mower_localize/ekf:95-99, 347; ekf_params:16, 22, 68, 74). *nav_core*: `nav_action_server`, `R V 10` (mower_nav/core_backend:226) | nav2 `controller_server`: nav2_params:42-105 sets no `odom_topic`, so the upstream default `odom` applies (unconfirmed); recorder (record_topics.yaml:31) | 25 Hz (README:154; 25.005 Hz measured, README:1148) | no | in-process; bus→DDS until E9 and E5 |
| `/odom_slow` | `Odometry` | `mower_base` slow copy, `R V 10` (mower_base:787-789, 912-913, 941-946; mower_rsd.yaml:49-50) | `robot_status`, one subscription serving both `heartbeat_source_topic` and `odom_topic`, `R V 10` (robot_status:160-166; mower_rsd.yaml:69, 72) | — | ≤5 Hz (4.41 Hz, README:1149) | no | in-process only |
| `/joint_states` | `sensor_msgs/JointState` | `mower_base`, `SD` (mower_base:898, 947-948) | — | robot_state_publisher, `BE V 5` (`SensorDataQoS`, measured) | 25 Hz (README:156) | no | bus→DDS until E4 |
| `/mower_base/telemetry` | `std_msgs/String`, JSON | `mower_base`, `BE V 1` (mower_base:900, 950-954) | `robot_status`, `BE V 1` (robot_status:104, 170); `battery_state`, `BE V 1` (mower_battery:236, 290); `pid_autotune`, `BE V 1`, subscribed only during a session (mower_pid_autotune:807, 896, 951, 969-983) | recorder (record_topics.yaml:37) | one per 0x85 frame, about 15-17 Hz (README:157); about 1.2 kB (REFACTOR:27) | no | in-process; bus→DDS while recording (E5) |
| `/mower_base/firmware_info` | `String`, JSON | `mower_base`, `R TL 1` (mower_base:901-902, 955-959) | `robot_status`, `R TL 1` (robot_status:95, 173-177) | — | once per distinct 0x87 (README:158) | no | in-process only |
| `/mower_base/led_command` | `String`, JSON | `robot_status`, `R TL 1` (robot_status:99, 153-157); `pid_autotune`, `R TL 1` (mower_pid_autotune:810, 840) | `mower_base`, `R TL 1` (mower_base:903, 1002) | — | on change | no | in-process only; two latched writers (5.5 e) |
| `/mower_base/pid_command` | `String`, JSON | `pid_autotune`, `R V 4` (mower_pid_autotune:808, 838) | `mower_base`, `R V 4` (mower_base:904, 1032) | — | per autotune step | no | in-process only |
| `/mower_base/wheel_override` | `String`, JSON | `pid_autotune`, `BE V 1` (mower_pid_autotune:809, 839) | `mower_base`, `BE V 1` (mower_base:905, 1058) | — | during a session; unknown | no | in-process only; base asks the graph whether it has a publisher (mower_base:175-185, 1185-1188) |
| `/mower_base/servo_command` | `String`, JSON | Studio through ws_bridge, `R V 10` (mower_ws_bridge/hub:417; ws_bridge.yaml:32; studio:164, 546) | `mower_base`, `BE V 1` (mower_base:906, 1076) | Studio | per change, debounced 100 ms (studio:546) | no | DDS→bus until E3 |
| `/mower_base/blade_command` | `String`, JSON | Studio through ws_bridge (ws_bridge.yaml:33; studio:165) | `mower_base`, `BE V 1` (mower_base:907, 1093) | Studio | 5 Hz dead-man while held (studio:162, 574) | no | DDS→bus until E3 (a safety path) |
| `/mower_base/rear_light` | `String`, JSON | `path_record_node`, `R TL 1`, refreshed every 2 s while recording (mower_record:183-184, 203-207, 277-292); `recorder_manager`, `R TL 1` (recorder_manager:59-63, 83, 145-146) | `mower_base`, `R TL 8`, on its r2r node (mower_base:927, 1114-1119; 5.8 #1) | recorder (writer) | every 2 s while a recording runs | no | the reader stays on r2r until E5: path_record writes on the bus, the recorder on DDS |

### 2.3 Sensors and localization

| Topic | Type | Producers | Consumers in mower_rsd | Outside | Rate | DDS today | E2 |
|---|---|---|---|---|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | `imu` as `imu/data_raw`, `R V 10` (mower_imu:190; rule robot.launch.py:81) | <ul><li>both EKFs as `imu0`/`imu`, `BE V 10` (mower_localize/ekf:363; ekf_params:26, 34, 88, 96; rules robot.launch.py:93, 95)</li><li>`navsat_transform`, `BE V 1`, used only until the datum locks (mower_localize/navsat:76-78, 164-180, 376-379; robot.launch.py:96)</li><li>`robot_status`, `BE V 5` (robot_status:103, 169)</li><li>`nav_action_server`, `BE V 5` (mower_nav:1613-1615, 1714)</li></ul> | recorder (record_topics.yaml:36) | 10 Hz (mower_imu:1-3) | no | in-process; bus→DDS while recording |
| `/fix` | `sensor_msgs/NavSatFix` | `gps`, `R V 10` (mower_gps:390-392; mower_rsd.yaml:105) | <ul><li>`navsat_transform` as `gps/fix`, `BE V 1` (mower_localize/navsat:39, 148; rule robot.launch.py:238-239)</li><li>`robot_status`, `BE V 5` (robot_status:101, 167)</li><li>`nav_action_server`, `BE V 5` (mower_nav:1605-1607, 1737)</li></ul> | <ul><li>app via ws_bridge, 100 ms throttle (app/mission_mock_provider:184-186, 647-651)</li><li>`recorder_manager`, `R V 1` (recorder_manager:124-126)</li><li>recorder bag (record_topics.yaml:35)</li></ul> | 4 Hz (gps.yaml:22) | no | in-process; bus→DDS until E3 and E5 |
| `/gps/status` | `String`, JSON | `gps`; the name defaults to the relative `gps/status`; `R V 10` (mower_gps:391, 393; gps.yaml:35) | — | app via ws_bridge, 1 s throttle (app/mission_mock_provider:188, 653-657); data collection (data_collection.yaml:17) | 1 Hz and on change (mower_gps:52, 335); size unknown (carries the NAV-SAT list, mower_gps:22-26) | no | bus→DDS until E3 |
| `/odometry/local` | `Odometry` | `ekf_filter_node_odom` as `odometry/filtered`, `R V 10` (mower_localize/ekf:33, 102-104, 208; rule robot.launch.py:92) | — | data collection only (data_collection.yaml:18) | ≤20 Hz (ekf_params:5) | no | bus only in production |
| `/odometry/global` | `Odometry` | `ekf_filter_node_map`, same code (rule robot.launch.py:94) | `navsat_transform` as `odometry/filtered`, `BE V 1` (mower_localize/navsat:38, 130; rule robot.launch.py:97) | velocity_smoother: nav2_params:224 sets `odom_topic`, but with `feedback: OPEN_LOOP` (:227) it does not subscribe (unconfirmed; see `src/mower_rs/crates/mower_nav_core/src/smoother.rs:40-41`); data collection | ≤20 Hz (ekf_params:57; 19.99 Hz, README:1150) | no | in-process only |
| `/odometry/global_slow` | `Odometry` | `ekf_filter_node_map` slow copy, `R V 10` (mower_localize/ekf:44-61, 211-214; mower_rsd.yaml:126-127) | `flutter_adapter`, `R V 10` (mower_adapter:171, 224; mower_rsd.yaml:162); `path_record_node`, `R V 10` (mower_record:180, 219; mower_rsd.yaml:157) | — | ≤5 Hz (4.33 Hz, README:1150-1151) | no | in-process only |
| `/odometry/gps` | `Odometry` | `navsat_transform`, `R V 10` (mower_localize/navsat:41, 181) | `ekf_filter_node_map` as `odom1`, `BE V 10` (ekf_params:78, 84); `nav_action_server`, `R V 10` (mower_nav:1609-1611, 1767) | data collection | unknown: the loop runs at 30 Hz (ekf_params:141) and publishes only what the core prepared (mower_localize/navsat:383-393) | no | in-process only |
| `/gps/filtered` | `NavSatFix` | `navsat_transform`, `R V 10` (mower_localize/navsat:42, 182-186; ekf_params:147) | `robot_status`, `BE V 5` (robot_status:102, 168) | — | unknown (same loop) | no | in-process only |
| `/tf` | `tf2_msgs/TFMessage` | <ul><li>`ekf_filter_node_odom` (odom→base_footprint) and `ekf_filter_node_map` (map→odom), `R V 100` (mower_localize/ekf:107-109, 216, 287-323; ekf_params:9, 61)</li><li>`mower_base`, an `SD` writer that never publishes: `enable_odom_tf` is false (mower_base:619-621, 740; README:155)</li><li>robot_state_publisher: the two wheel joints, `R V 100`, 12.5 Hz: one message per `/joint_states` whose stamp is 50 ms (`publish_frequency` 20 Hz) past the last one it published, stamped with that stamp (measured)</li></ul> | none in production: the EKFs share their edges through `mower_localize::tfbus` (mower_localize/tfbus:1-33), and *nav_core* reads tfbus too (robot.launch.py:222-225) | nav2 tf listeners, `R V 100` (measured); recorder (record_topics.yaml:32) | 76.6 Hz measured 2026-09, with 30 Hz EKFs and 50 Hz joint states (REFACTOR:19); now unknown | no | bus→DDS until E9 and E5; nothing needs DDS→bus |
| `/tf_static` | `TFMessage` | robot_state_publisher, the URDF's 7 fixed joints in one message, sorted by joint name, one stamp (base_footprint→base_link, base_link→imu_link, gps_link, ...), `R TL 1` (measured); `navsat_transform`, the `utm` frame, `R TL 1` (mower_localize/navsat:187-191; ekf_params:146) | both EKFs and `navsat_transform`, `R TL 100` (mower_localize/ekf:111-113, 185; mower_localize/navsat:84-86, 114) | nav2 tf listeners; recorder, override `R TL 100` (qos_overrides.yaml:51-55) | latched: once per writer | no | DDS→bus until E4. Decided: navsat's `utm` writer stays on r2r until E4, so nothing goes bus→DDS and nothing can echo (5.5 d) |

### 2.4 Maps and mission geometry

| Topic | Type | Producers | Consumers in mower_rsd | Outside | Rate, size | DDS today | E2 |
|---|---|---|---|---|---|---|---|
| `/map_grid` | `nav_msgs/OccupancyGrid` | `map_manage`, `R TL 1` (mower_map:978, 1155) | `flutter_adapter`, `R TL 1` (mower_adapter:35-36, 183-185). *nav_core*: `R TL 1` (mower_nav/core_backend:119, 227) | nav2 local costmap StaticLayer, `map_subscribe_transient_local` (nav2_params:126-129; upstream `R TL 1`, unconfirmed) | on change. The start-up map is 400 × 400 cells, i.e. 160 kB of data (mower_map:139-142); zone maps vary | no | bus→DDS until E9; the bridge writer must be TL (5.5 a) |
| `/map_grid_global` | `OccupancyGrid` | `map_manage`, `R TL 1` (mower_map:979) | *nav_core*: `R TL 1` (mower_nav/core_backend:120, 228) | nav2 global costmap StaticLayer (nav2_params:143-146) | as above | no | bus→DDS until E9, TL |
| `/free_space` | `OccupancyGrid` | `map_manage`, `R TL 1` (mower_map:972) | — | rviz only | on change | no | bus only |
| `/free_space_inflated` | `OccupancyGrid` | `map_manage`, `R TL 1` (mower_map:973); `boustrophedon_coverage` had an `R TL 1` writer that never published (mower_coverage:442-445, 1746), dropped in step 5 (5.8 #2) | `flutter_adapter`, `R TL 1` (mower_adapter:37) | recorder, override `R TL 1` (qos_overrides.yaml:46-50; record_topics.yaml:39) | on change | no | in-process; bus→DDS while recording |
| `/risk_map` | `OccupancyGrid` | `map_manage`, `R TL 1` (mower_map:974) | `boustrophedon_coverage`, `R TL 1` (mower_coverage:1788) | rviz | on change | no | in-process only |
| `/risk_map_inflated` | `OccupancyGrid` | `map_manage`, `R TL 1` (mower_map:975); coverage's unused writer (mower_coverage:1747), dropped in step 5 | `flutter_adapter`, `R TL 1` (mower_adapter:38); `boustrophedon_coverage`, `R TL 1` (mower_coverage:1798) | recorder, override (qos_overrides.yaml:41-45; record_topics.yaml:38) | on change | no | in-process; bus→DDS while recording |
| `/chennal_map` | `OccupancyGrid` | `map_manage`, `R TL 1` (mower_map:976) | — | rviz | on change | no | bus only |
| `/chennal_map_inflated` | `OccupancyGrid` | `map_manage`, `R TL 1` (mower_map:977) | `flutter_adapter`, `R TL 1` (mower_adapter:39) | — | on change | no | in-process only |
| `/coverage_path` | `nav_msgs/Path` | `boustrophedon_coverage`, `R TL 1` (mower_coverage:1742) | — | rviz | per plan | no | bus only |
| `/coverage_path_markers`, `/coverage_invalid_segments`, `/coverage_connectors` | `visualization_msgs/MarkerArray` | `boustrophedon_coverage`, `R TL 1` (mower_coverage:1743-1745) | `flutter_adapter`, `R TL 1` (mower_adapter:46-48, 199-204) | rviz | per plan; size unknown | no | in-process only |
| `/zone_sequence_status` | `String`, JSON | `boustrophedon_coverage`, `R TL 1` (mower_coverage:1748) | — | app via ws_bridge (app/mission_mock_provider:593, 602-609) | on change | no | bus→DDS until E3 |
| `/zone_list`, `/risk_zone_list`, `/chennal_path_array` | `MarkerArray` | `path_record_node`, `R TL 1` (mower_record:192, 194, 197) | `flutter_adapter`, `R TL 1` (mower_adapter:43-45) | rviz | on change | no | in-process only |
| `/zone_markers`, `/risk_zone_markers`, `/channel_path_array` | `Marker`, `Marker`, `MarkerArray` | `path_record_node`, `R TL 1` (mower_record:191, 193, 198) | — | rviz | on change | no | bus only |
| `/recorded_path`, `/chennal_path`, `/channel_path` | `Path` | `path_record_node`, `R V 10` (mower_record:190, 195-196) | — | rviz | while recording; unknown | no | bus only |
| `/site_list` | `String`, JSON | `path_record_node`, `R TL 1` (mower_record:199) | — | app via ws_bridge (app/mission_mock_provider:624-628) | on change | no | bus→DDS until E3 |

### 2.5 App relays (`/adapter/*`)

| Topic | Type | Producers | Consumers in mower_rsd | Outside | Rate, size | DDS today | E2 |
|---|---|---|---|---|---|---|---|
| `/adapter/map_layers/{map_grid, free_space_inflated, risk_map_inflated, chennal_map_inflated}` | `String`, JSON | `flutter_adapter`, `R TL 1` (mower_adapter:35-40, 183-197) | — | app, 100 ms throttle (app/mission_mock_provider:595-616); recorder, override `R TL 1` (qos_overrides.yaml:14-33; record_topics.yaml:16-19) | per source change; a map grid encodes to about 214 kB (mission.launch.py:416-418) | no | bus→DDS until E3 and E5 |
| `/adapter/marker_layers/{zones, risk_zones, channels, coverage_path, invalid_segments, connectors}` | `String`, JSON | `flutter_adapter`, `R TL 1`; an empty layer at start (mower_adapter:42-49, 199-218) | — | app (app/mission_mock_provider:584-609); recorder (record_topics.yaml:20-25) | per source change | no | bus→DDS until E3 and E5 |
| `/adapter/robot_pose` | `geometry_msgs/PoseStamped` | `flutter_adapter`, `R V 10` (mower_adapter:222-233) | `nav_action_server`, `R V 10`; a health-gate input with a 0.30 s budget (mower_nav:1571, 1692) | app, 100 ms throttle (app/mission_mock_provider:630-634); recorder (record_topics.yaml:12) | follows `/odometry/global_slow`, ≤5 Hz | no | in-process to nav; bus→DDS until E3 and E5 |
| `/adapter/coverage_settings` | `String`, JSON | `flutter_adapter`, `R TL 1` (mower_adapter:236-284) | — | app; recorder | 1 Hz (mower_adapter:245) | no | bus→DDS until E3 and E5 |
| `/adapter/zone_summaries` | `String`, JSON | `flutter_adapter`, `R TL 1` (mower_adapter:288-313) | — | app; recorder | 2 Hz (mower_adapter:310) | no | bus→DDS until E3 and E5 |
| `/adapter/map_datum` | `String`, JSON | `flutter_adapter`, `R TL 1` (mower_adapter:316-360) | `path_record_node`, `R TL 1` (mower_record:229) | app, 1 s throttle (app/mission_mock_provider:618-622); recorder, override (qos_overrides.yaml:9-13) | once, when navsat has a datum; the fallback is off (mower_adapter:175) | no | in-process to record; bus→DDS until E3 and E5 |

### 2.6 Navigation

| Topic | Type | Producers | Consumers in mower_rsd | Outside | Rate | DDS today | E2 |
|---|---|---|---|---|---|---|---|
| `/nav_operation_active` | `Bool` | `nav_action_server`, `R TL 1` (mower_nav:1629) | <ul><li>`robot_status`, `R TL 1` (robot_status:96, 178)</li><li>`map_manage`, `R TL 1` (mower_map:1010)</li><li>`boustrophedon_coverage`, `R TL 1` (mower_coverage:1777)</li><li>`path_record_node`, `R TL 1` (mower_record:240)</li><li>`pid_autotune`, `R V 10` (mower_pid_autotune:812, 869)</li></ul> | — | on state change (mower_nav:257-259, 284-296) | no. pid_autotune's volatile reader gets no history on DDS or on the bus | in-process only |
| `/coverage_progress` | `mower_interface/CoverageProgress` | `nav_action_server`, `R TL 1` (mower_nav:1639) | — | allowed for the app (ws_bridge.yaml:27), but neither the app nor the Studio subscribes | ≤2 Hz from feedback, plus forced updates (mower_nav:142, 298-326) | no | bus only unless a client appears |
| `/split_path`, `/coverage_split_points` | `Path`, `Marker` | `nav_action_server`, `R V 1` (mower_nav:1632-1633) | — | rviz | per dispatch | no | bus only |
| `/plan` | `Path` | nav2 planner_server. *nav_core*: `nav_action_server`, `R V 1` (mower_nav/core_backend:122, 225) | — | recorder snapshot buffer (record_topics.yaml:55) | while navigating | no | nav2: DDS only. *nav_core*: bus→DDS while recording |
| `/rosout` | `rcl_interfaces/Log` | every node (rcl's rosout publisher; upstream `R TL 1000`, unconfirmed) | `nav_action_server`, `R V 100`, nav2 backend only; keeps WARN and above from nav2's nodes (mower_nav:1779-1789; mower_nav/state:191-204) | recorder (record_topics.yaml:45) | unknown | no | DDS→bus for nav2's logs until E9; for the modules' own logs see 5.8 |

### 2.7 Status, battery, autotune

| Topic | Type | Producers | Consumers in mower_rsd | Outside | Rate | DDS today | E2 |
|---|---|---|---|---|---|---|---|
| `/robot/online` | `Bool` | `robot_status`, `R TL 1` (robot_status:147) | — | app, 200 ms throttle, offline after 3 s (app/mission_mock_provider:161, 635-639); Studio (studio:345); recorder, override (qos_overrides.yaml:34-38) | 2 Hz (robot_status:93, 186; mower_rsd.yaml:71) | no | bus→DDS until E3 and E5 |
| `/robot/info` | `String`, JSON | `robot_status`, `R TL 1` (robot_status:148) | — | app, 500 ms throttle (app/robot_info_provider:21-32); agent (mower_agent:811) | 1 Hz and on demand (robot_status:94, 187) | no | bus→DDS until E3 |
| `/robot/telemetry` | `String`, JSON | `robot_status`, `R V 10` (robot_status:149-152) | — | Studio, 1 s throttle (studio:341; studio/rosbridge_client:24); agent, 5 s throttle (mower_agent:49, 812) | 10 Hz (robot_status:100, 188); size unknown | no | bus→DDS until E3 |
| `/battery_state` | `sensor_msgs/BatteryState` | `battery_state`, `R V 10` (mower_battery:237, 288) | `robot_status`, `R V 10` (robot_status:105, 171) | app, 1 s throttle (app/mission_mock_provider:641-645); recorder (record_topics.yaml:27) | 1 Hz (mower_battery:239, 306-308) | no | in-process; bus→DDS until E3 and E5 |
| `/aon_battery_state` | `BatteryState` | `battery_state`, `R V 10` (mower_battery:238, 289) | `robot_status`, `R V 10` (robot_status:106, 172) | recorder (record_topics.yaml:28) | 1 Hz | no | in-process; bus→DDS while recording |
| `/pid_autotune/status` | `String`, JSON | `pid_autotune`, `R TL 1` (mower_pid_autotune:811, 837) | — | Studio (studio:351) | once at start, then 5 Hz during a session (mower_pid_autotune:928-939) | no | bus→DDS until E3 |

### 2.8 Topics the bridge carries with no typed module involved

| Topic | Type | Producers | Consumers | Rate | DDS today | E2 |
|---|---|---|---|---|---|---|
| `/mower_recorder/status`, `/mower_recorder/bags` | `String`, JSON | `recorder_manager` and `bag_store`, `R TL 1` (recorder_manager:141-142; bag_store:98-99) | app via ws_bridge (app/recorder_provider:86-88, 114-123); `bag_store` reads the status (bag_store:106-107) | status every 2 s (recorder_manager:148); bags unknown | no | DDS only in E2; DDS→bus from E3 until E5. With E5-3a's bag module (`rust_recorder`, off) both are written on the bus and go out `R TL 1` |
| `/mower_recorder/command_result` | `String`, JSON | `bag_store`, `R V 10` (bag_store:100-101) | app (app/recorder_provider:90, 124) | per command | no | same |
| `/mower_recorder/command` | `String`, JSON | app via ws_bridge (ws_bridge.yaml:31; app/recorder_provider:89, 163) | `bag_store`, `R V 10` (bag_store:103-104) | per button press | no | DDS only in E2; bus→DDS from E3 until E5. With the bag module it comes in from DDS for `bag_store`, or is in-process with ws_bridge on the bus |
| `/mower_ws_bridge/wake` | `std_msgs/Empty` | `mower_ws_bridge`, `R V 1` (mower_ws_bridge/hub:208-213) | itself: it interrupts its own `spin_once` | per WebSocket command | no | an r2r artefact; it goes away in E3 |

None of the following has an endpoint in `mower_rsd`:

- **nav2 internals:**
  - `/cmd_vel_nav`: controller and behavior server to velocity smoother (navigation.launch.py:301, 325-327, 351-355);
  - the costmap topics, which the recorder keeps in its snapshot buffer (record_topics.yaml:53-54);
  - action status and feedback;
  - `/bond`, unused because of `bond_timeout` 0 (navigation.launch.py:256-265, 364).
- **robot_state_publisher's `/robot_description`:** the URDF text, one message, `R TL 1` (measured); nothing
  subscribes.
- **The recorder's own topics:** `/mower_recorder/fault`, `/graph_snapshot` and `/graph_events` (graph_snapshot:50-53).
- **`/diagnostics` and `/cmd_vel`:** recorded (record_topics.yaml:34, 44), but no producer of either is in this repo.
  `/diagnostics` comes from twist_mux (two a second, `R V 1`) and nav2's `lifecycle_manager_navigation` (one a second;
  measured).

## 3. Services

### 3.1 Served by mower_rsd

| Service | Type | Server | Clients in mower_rsd | Outside clients | E2 |
|---|---|---|---|---|---|
| `/cancel_nav2` | `std_srvs/Trigger` | `nav_action_server` (mower_nav:1791-1792) | — | app; retried every 2 s while a start is ambiguous (app/mission_mock_provider:3545) | DDS→bus until E3 |
| `/cencel_nav2` | `Trigger` | same | — | allowed (ws_bridge.yaml:56); no client found | bus only |
| `/check_nav_status` | `Trigger` | `nav_action_server` (mower_nav:1838) | `boustrophedon_coverage` (mower_coverage:1766) | app every 1 s (app/mission_mock_provider:50, 3605-3620, 3990-3995); probe_app.py:39 | in-process; DDS→bus until E3 |
| `/coverage_progress_status` | `mower_interface/GetCoverageProgress` | `nav_action_server` (mower_nav:1824) | `boustrophedon_coverage` (mower_coverage:1767) | allowed (ws_bridge.yaml:58); not called by the app or the Studio | in-process |
| `/confirm_navigation_dispatch` | `mower_interface/ConfirmNavigationDispatch` | `nav_action_server` (mower_nav:1856) | `boustrophedon_coverage` (mower_coverage:1764) | — | in-process |
| `/cancel_navigation_dispatch` | `mower_interface/CancelNavigationDispatch` | `nav_action_server` (mower_nav:1881) | `boustrophedon_coverage` (mower_coverage:1765) | — | in-process |
| `/mission_operation_lock` | `mower_interface/MissionOperationLock` | `nav_action_server` (mower_nav:1933) | `map_manage` (mower_map:981); `boustrophedon_coverage` (mower_coverage:1737); `path_record_node` (mower_record:208); `pid_autotune`, through its `lock_service` parameter (mower_pid_autotune:813, 841) | — | in-process |
| `/generate_coverage_path` | `Trigger` | `boustrophedon_coverage` (mower_coverage:1810) | — | app (app/mission_mock_provider:2589, 2753, 2968) | DDS→bus until E3 |
| `/zone_exec_path` | `mower_interface/ZoneExecPath` | `boustrophedon_coverage` (mower_coverage:1843) | — | app (app/mission_mock_provider:3348) | DDS→bus until E3 |
| `/resume_coverage` | `mower_interface/ResumeCoverage` | `boustrophedon_coverage` (mower_coverage:1875) | — | allowed (ws_bridge.yaml:59); no client found | bus only |
| `/run_zone_sequence` | `mower_interface/ZoneSequence` | `boustrophedon_coverage` (mower_coverage:1893) | — | app (app/mission_mock_provider:3380) | DDS→bus until E3 |
| `/stop_zone_sequence` | `Trigger` | `boustrophedon_coverage` (mower_coverage:1934) | — | app (app/mission_mock_provider:3535) | DDS→bus until E3 |
| `/boustrophedon_coverage/get_parameters` | `rcl_interfaces/GetParameters` | `boustrophedon_coverage` (mower_coverage:1957) | `flutter_adapter`, every second (mower_adapter:238-241, 249-260) | recorder's `ros2 param dump` at the start of a run (record.launch.py:85-86; recorder_manager:399-404) | in-process; DDS→bus until E5 |
| `/boustrophedon_coverage/set_parameters` | `rcl_interfaces/SetParameters` | `boustrophedon_coverage` (mower_coverage:2000) | — | app (app/mission_mock_provider:1946; ws_bridge.yaml:64) | DDS→bus until E3 |
| `/boustrophedon_coverage/{get_parameter_types, list_parameters, describe_parameters, set_parameters_atomically}` | `rcl_interfaces/*` | `boustrophedon_coverage` (mower_coverage:1972, 1981, 1991, 2013) | — | `ros2 param dump` (which of them it calls is unconfirmed) | DDS→bus until E5 |
| `/create_risk_map`, `/create_free_space`, `/create_chennal_map`, `/restore_free_space_coverage` | `Trigger` | `map_manage` (mower_map:1022-1028) | — | app (app/mission_mock_provider:1965, 2552-2588) | DDS→bus until E3 |
| `/import_image_mask` | `mower_interface/ImportImageMask` | `map_manage` (mower_map:1038) | — | app (app/mission_mock_provider:2357) | DDS→bus until E3 |
| `/get_zone_map_list_srv` | `mower_interface/ZoneMapList` | `map_manage` (mower_map:1062) | `flutter_adapter`, every 0.5 s (mower_adapter:289-311); `boustrophedon_coverage` (mower_coverage:1763) | — | in-process |
| `/map_manage/get_parameters` | `GetParameters` | `map_manage` (mower_map:1075) | `flutter_adapter`, every second (mower_adapter:242, 262-271) | recorder's `ros2 param dump`. Not callable from the app: it is absent from ws_bridge.yaml:34-66 | in-process; DDS→bus until E5 |
| `/map_manage/{get_parameter_types, list_parameters, describe_parameters, set_parameters, set_parameters_atomically}` | `rcl_interfaces/*` | `map_manage` (mower_map:1093-1146) | — | `ros2 param dump` | DDS→bus until E5 |
| `/record_zone_start`, `/record_zone_end`, `/risk_zone_start`, `/risk_zone_end`, `/channel_record_start`, `/channel_record_end`, `/record_cancel`, `/save_zone_list`, `/load_zone_list` | `Trigger` | `path_record_node` (mower_record:296-378) | — | app (app/mission_mock_provider:1717, 1744, 1796-1811, 2753) | DDS→bus until E3 |
| `/chennal_record_start`, `/chennal_record_end` | `Trigger` | aliases on `path_record_node` (mower_record:324, 333) | — | no client found | bus only |
| `/get_record_zone_info` | `Trigger` | `path_record_node` (mower_record:380) | — | none in production; only the desktop dev tool mower_qt (`src/mower_qt/mower_qt/mower_qt.py:309`) | bus only |
| `/edit_zone` | `mower_interface/EditZone` | `path_record_node` (mower_record:387) | — | app (app/mission_mock_provider:2712, 3095, 3235) | DDS→bus until E3 |
| `/get_record_zone_list`, `/get_risk_zone_list` | `mower_interface/GetZoneList` | `path_record_node` (mower_record:427, 441) | `map_manage` (mower_map:1003-1004) | — | in-process |
| `/get_channel_path_list` | `mower_interface/ChannelPathList` | `path_record_node` (mower_record:455) | — | no client found | bus only |
| `/get_chennal_path_list` | `mower_interface/ChennalPathList` | `path_record_node` (mower_record:465) | `map_manage` (mower_map:1005) | — | in-process |
| `/get_channel_route` | `mower_interface/ChannelRoute` | `path_record_node` (mower_record:475) | `boustrophedon_coverage` (mower_coverage:1768) | — | in-process |
| `/site_op` | `mower_interface/SiteOp` | `path_record_node` (mower_record:489) | — | app (app/mission_mock_provider:2842) | DDS→bus until E3 |
| `/toLL` | `robot_localization/ToLL` | `navsat_transform` (mower_localize/navsat:196) | `flutter_adapter`, every 2 s until a datum exists (mower_adapter:317-357) | datum_check.py:57 | in-process; DDS→bus for the tool |
| `/fromLL` | `robot_localization/FromLL` | `navsat_transform` (mower_localize/navsat:220) | — | datum_check.py:58. nav2's waypoint_follower creates a client for GPS waypoints upstream (unconfirmed); nothing sends such a goal | DDS→bus for the tool, and until E9 |
| `/fromLLArray` | `robot_localization/FromLLArray` | `navsat_transform` (mower_localize/navsat:233) | — | no client found | bus only |
| `/datum` | `robot_localization/SetDatum` | `navsat_transform` (mower_localize/navsat:267) | — | datum_check.py `--fix` (:59) | DDS→bus for the tool |
| `/{ekf_filter_node_odom, ekf_filter_node_map, navsat_transform}/{get_parameters, get_parameter_types, list_parameters, describe_parameters, set_parameters, set_parameters_atomically}` | `rcl_interfaces/*` | each localize node (mower_localize/paramsrv:146-231) | — | datum_check.py calls `/navsat_transform/get_parameters` (:60-61) | DDS→bus for the tools |
| `/system/update`, `/system/restart`, `/system/check_update` | `Trigger` | `robot_status` (robot_status:181-183) | — | app, all three (app/robot_info_provider:33-35); Studio, update and check_update (studio:221, 225) | DDS→bus until E3 |
| `/pid_autotune` | `mower_interface/PidAutotune` | `pid_autotune` (mower_pid_autotune:881) | — | Studio (studio:600) | DDS→bus until E3 |

The other modules serve no parameter services: none calls `make_parameter_handler`. `ros2 param` against those nodes
therefore finds nothing.

### 3.2 Called by mower_rsd, served outside

| Service | Type | Server | Client | E2 |
|---|---|---|---|---|
| `/bt_navigator/get_state` | `lifecycle_msgs/GetState` | nav2 bt_navigator | `nav_action_server`, by the relative name `bt_navigator/get_state` (mower_nav:1644); polled every 0.2 s while it waits for nav2 (mower_nav:973, 1584) | nav2 backend only. Stays an r2r client next to nav2's two actions until E9: nav polls it while it waits for nav2, so it belongs with them (marked `stays_on_r2r` in mower_bridge's manifest) |

### 3.3 Only through ws_bridge

| Service | Server | Clients | E2 |
|---|---|---|---|
| `/mower_recorder/start`, `/mower_recorder/stop` (`Trigger`) | `recorder_manager` (recorder_manager:130-133) | app (app/recorder_provider:91-92, 199) | DDS only in E2; E3 proxies them until E5. The bag module (`rust_recorder`, off; E5-3a: start refused) serves them on the bus, `service_to_ros` |
| `/mower_recorder/snapshot` (`Trigger`) | `recorder_manager` (recorder_manager:134-136) | none found | E5; served by the bag module as start and stop |
| `/rosapi/topics` | emulated by ws_bridge: the DDS graph's topics that match `topics_sub` (mower_ws_bridge/client:149-150, 179-185) | app every 5 s (app/robot_fleet_provider:47, 158); probe_app.py:94 | a topic that leaves DDS also leaves this answer, and probe_app.py fails on it (probe_app.py:134-136) |
| `/rosapi/get_time` | emulated: the wall clock (mower_ws_bridge/client:151-155) | Studio (studio:486); probe_app.py:39 | — |

## 4. Actions

| Action | Type | Server | Clients in mower_rsd | Outside | E2 |
|---|---|---|---|---|---|
| `/nav_action_follow_path` | `mower_interface/action/Waypoint` | `nav_action_server`, by a relative name (mower_nav:1970-1979) | `boustrophedon_coverage`, by the relative name `nav_action_follow_path` (mower_coverage:1769) | test tools only (`nav_progress_check.py`, `coverage_cancel_check.py`, `nav_core_check.py`) | Done (step 5): the bus's actions (bus/README, "Actions"), nav and coverage together. nav's server also goes out with `action_to_ros`, in mower_rsd as standalone, so the tools and the graph keep it; standalone coverage reaches the tool's server through `action_from_ros`, whose bus server exists only while DDS has one (bus_ros/README, "Actions") |
| `/nav_action` | `Waypoint` | same loop (mower_nav:1970) | — | none in production; only the dev utility `src/mower_mission/mower_mission/utils/nav_action_client.py:51` | the same (step 5): on the bus, out through `action_to_ros` |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | nav2 bt_navigator | `nav_action_server`, by a relative name (mower_nav:1642) | the recorder records its `_action/feedback` and `_action/status` (record_topics.yaml:40-41) | stays an r2r client until E9 (*nav_core* replaces it) |
| `/follow_path` | `nav2_msgs/action/FollowPath` | nav2 controller_server | `nav_action_server`, by a relative name (mower_nav:1643) | — | stays an r2r client until E9 |

## 5. Highlights

### 5.1 Topics and services that never leave mower_rsd

Both ends are typed modules, and nothing outside subscribes in production. These are pure in-process wins once both ends
are on the bus. Until then they need a temporary bridge if their two ends move in different steps.

**Topics (19):**

- `/drivetrain_guarded_cmd_vel`
- `/odom_slow`
- `/mower_base/firmware_info`, `/mower_base/led_command`, `/mower_base/pid_command`, `/mower_base/wheel_override`
- `/odometry/global`, `/odometry/global_slow`, `/odometry/gps`, `/gps/filtered`
- `/risk_map`, `/chennal_map_inflated`
- `/coverage_path_markers`, `/coverage_invalid_segments`, `/coverage_connectors`
- `/zone_list`, `/risk_zone_list`, `/chennal_path_array`
- `/nav_operation_active`

`/odometry/global` and `/odometry/gps` are also recorded by the optional data-collection profile.

**Services (10):**

- `/mission_operation_lock`, `/confirm_navigation_dispatch`, `/cancel_navigation_dispatch`, `/coverage_progress_status`
- `/get_zone_map_list_srv` (polled at 2 Hz)
- `/get_record_zone_list`, `/get_risk_zone_list`, `/get_chennal_path_list`, `/get_channel_route`
- `/toLL` (also called by the `datum_check.py` tool)

Three more are in-process except for one outside caller each:

- `/check_nav_status`: called by coverage in process; the outside caller is the app.
- `/boustrophedon_coverage/get_parameters` and `/map_manage/get_parameters`: the adapter polls both at 1 Hz in process;
  the outside caller is the recorder's param dump.

**Actions:** `/nav_action_follow_path` (coverage to nav).

**No consumer on the robot (13).** These can be bus only, or exposed through the E8 `--ros-tap` for rviz (PLAN:361):

- `/free_space`, `/chennal_map`, `/coverage_path`
- `/zone_markers`, `/risk_zone_markers`, `/channel_path_array`
- `/recorded_path`, `/chennal_path`, `/channel_path`
- `/split_path`, `/coverage_split_points`
- `/odometry/local` (data collection only)
- `/coverage_progress` (allowed, but no client)

`/physical_joy_cmd` and `/keyboard_cmd_vel` are subscribed but have no publisher.

### 5.2 In-process today, but only through ws_bridge

These pairs never leave the process today. Even so, they need a bridge in E2, because their bridge end stays on r2r
until E3:

| Topic | Bridge end |
|---|---|
| `/app_joy_cmd` | ws_bridge publishes it; the manual guard reads it |
| `/manual_command_clock` | the manual guard publishes it; ws_bridge reads it |
| `/mower_base/servo_command`, `/mower_base/blade_command` | ws_bridge publishes them; mower_base reads them |

So do E3 before the moves in steps 6 and 7. Then neither the joystick path nor the blade dead-man crosses a bridge.

### 5.3 Ends that stay on DDS, and until when

**Until E3 (ws_bridge).**

- *bus→DDS:*
  - `/manual_command_clock`, `/zone_sequence_status`, `/site_list`
  - the 14 `/adapter/*` topics
  - `/robot/online`, `/robot/info`, `/robot/telemetry`
  - `/battery_state`, `/fix`, `/gps/status`, `/pid_autotune/status`
  - `/coverage_progress`, if a client appears
- *DDS→bus:* `/app_joy_cmd`, `/mower_base/servo_command`, `/mower_base/blade_command`.
- *Services, DDS client to bus server:* the 27 services in ws_bridge.yaml:34-66 that the app or the Studio calls. That is
  the 32 entries, minus `/cencel_nav2`, `/coverage_progress_status` and `/resume_coverage` (no client), minus the two
  `/mower_recorder/*` ones.
- `/rosapi/topics` lists only what is visible on DDS (3.3).

**Until E4 (twist_mux, robot_state_publisher).**

- *bus→DDS:* `/joy_cmd`, `/navigation_safety_stop`, `/navigation_coordinator_lock`, `/joint_states`; with *nav_core*,
  also `/nav_cmd_vel`.
- *DDS→bus:* `/cmd_vel_guard_input`, and rsp's frames on `/tf_static`.
- rsp's wheel joints on `/tf` have no consumer in the process. E4 keeps all of rsp's output anyway (the 7 fixed
  joints, the wheels' `/tf` from `/joint_states`, `/robot_description`): the owner's decision D2 of the E4 design.
- **With E4-3's mux** (`rust_mux` with `rust_daemon`): twist_mux is a module of the process. nav2's `/nav_cmd_vel`
  comes in (DDS→bus, `R V 1`), its output `/cmd_vel_guard_input` and `/diagnostics` go out (bus→DDS, `R V 1`, for the
  final guard until step 6 and the recorder until E5). `/joy_cmd` and the teleop topics come in as before; the lock and
  the safety stop are in-process and still go out for the tools. Nav's own nav_core-only `/nav_cmd_vel` stays
  in-process then (mower_bridge README, "The policy").
- **With E4-4's static_tf** (`rust_static_tf` with `rust_daemon`): robot_state_publisher is a module of the process.
  `/tf_static` goes out (bus→DDS, `R TL 1`) instead of coming in, so the EKFs and navsat get the fixed joints
  in-process; navsat's `utm` stays on r2r and is not brought onto the bus (option A of the E4 design: it moves with E5
  and E9). `/joint_states` comes in from mower_base on r2r (`BE V 5`, rsp's reader) until step 6, the wheels go out on
  `/tf` with the EKFs' transforms, `/robot_description` goes out (`R TL 1`). localize's three nodes start once the
  module's `/tf_static` is on the bus (mower_rsd, `waits_for_static_tf`): from DDS, rsp's frames could reach the map
  EKF after navsat's 3 s delay on a loaded host and rotate the datum by up to the robot's heading.

**Until E5 (recorder), while a recording runs.**

- *bus→DDS:* everything in record_topics.yaml:5-55 that a module produces:
  - the 14 `/adapter/*` topics
  - `/robot/online`, `/battery_state`, `/aon_battery_state`
  - `/odom`, `/tf`, `/tf_static`, `/fix`, `/imu/data`
  - `/mower_base/telemetry`, `/risk_map_inflated`, `/free_space_inflated`, `/rosout`
  - with *nav_core*: `/plan`
  - with the data-collection profile: `/gps/status`, `/odometry/local`, `/odometry/global`, `/odometry/gps`
- *DDS→bus, services:*
  - the param dump of `/boustrophedon_coverage` and `/map_manage`;
  - after E3, also the six `/mower_recorder/*` endpoints (PLAN:358).
- *Graph presence:* `graph_snapshot` logs `NODE DISAPPEARED` and emits a `node_disappeared` event on `/graph_events`
  for every node that leaves the DDS graph (graph_snapshot:63, 90-100). Its node list is part of every bag.
  - Its watched-node fault is meant for `map_manage` and `boustrophedon_coverage` (record.launch.py:42), but it can never
    fire (5.8 #9).
  - So a module that drops its r2r node in E2 shows up in the bags as a disappearance event, not as a fault.

**Until E9 (nav2).**

- *bus→DDS:* `/map_grid`, `/map_grid_global` (TL), `/tf`, `/odom`.
- *DDS→bus:* `/rosout` (nav2's warnings for mower_nav).
- *r2r clients:* `/navigate_to_pose`, `/follow_path`, `/bt_navigator/get_state`.
- `/fromLL`, if nav2's client is real (unconfirmed).
- With *nav_core*, all of this ends earlier: nav2 does not start (robot.launch.py:110-148).

**Tools, any time.**

- `datum_check.py`: `/toLL`, `/fromLL`, `/datum`, `/navsat_transform/get_parameters`.
- The `*_compare.py` and `*_check.py` harnesses talk DDS.
- `probe_app.py` goes through ws_bridge.

### 5.4 Names the resolver has to produce

The bus refuses relative names (bus/README:41-44). E2's resolver must turn every name below into the absolute name that
rcl produces today. For the EKFs that means three steps, in this order:

1. Take the parameter value (`imu0: imu`).
2. Apply the node-scoped rule (`imu:=imu/data`).
3. Resolve in the root namespace, which gives `/imu/data`.

The node namespace is always empty (`ModuleCtx.namespace`, `src/mower_rs/crates/mower_rs_common/src/module.rs:100, 112`).

bus/README:43-44 names only the guards and the IMU as relying on remapping. gps, localize (all three nodes), nav and
coverage rely on relative names too.

| Node | Name in code | Rule or parameter (source) | Absolute |
|---|---|---|---|
| `imu` | `imu/data_raw` (mower_imu:190) | `imu:imu/data_raw:=imu/data` (robot.launch.py:81) | `/imu/data` |
| `manual_velocity_guard` | `cmd_vel_in`, `cmd_vel_out`, `command_clock` (velocity_command_guard:83-86) | robot.launch.py:83-85 | `/app_joy_cmd`, `/joy_cmd`, `/manual_command_clock` |
| `velocity_command_guard` | `cmd_vel_in`, `cmd_vel_out` | robot.launch.py:86-87; no clock: `require_command_session: false` (mower_rsd.yaml:91) | `/cmd_vel_guard_input`, `/drivetrain_guarded_cmd_vel` |
| `ekf_filter_node_odom` | output `odometry/filtered` (mower_localize/ekf:33); inputs from `odom0: odom` and `imu0: imu` (ekf_params:16, 26) | `odometry/filtered:=odometry/local`, `imu:=imu/data` (robot.launch.py:92-93) | `/odometry/local`, `/odom`, `/imu/data` |
| `ekf_filter_node_map` | output `odometry/filtered`; `odom0: odom`, `odom1: odometry/gps`, `imu0: imu` (ekf_params:68, 78, 88); slow copy from the parameter `odometry_slow_topic`, default relative `odometry/global_slow` (mower_localize/ekf:56-61) | `odometry/filtered:=odometry/global`, `imu:=imu/data` (robot.launch.py:94-95); `odometry_slow_topic: /odometry/global_slow` (mower_rsd.yaml:126) | `/odometry/global`, `/odom`, `/odometry/gps`, `/imu/data`, `/odometry/global_slow` |
| `navsat_transform` | `odometry/filtered`, `gps/fix`, `imu`, `odometry/gps`, `gps/filtered` (mower_localize/navsat:38-42) | `imu:=imu/data`, `odometry/filtered:=odometry/global` (robot.launch.py:96-97); `gps/fix:=/fix` from `gps_fix_topic` (robot.launch.py:238-239); none for the two outputs | `/odometry/global`, `/fix`, `/imu/data`, `/odometry/gps`, `/gps/filtered` |
| `gps` | `status_topic` default `gps/status` (mower_gps:391) | gps.yaml:35 keeps it relative | `/gps/status` |
| `nav_action_server` | action clients `navigate_to_pose`, `follow_path`; service `bt_navigator/get_state` (mower_nav:1642-1644); action servers `nav_action`, `nav_action_follow_path` (mower_nav:1970) | none | `/navigate_to_pose`, `/follow_path`, `/bt_navigator/get_state`, `/nav_action`, `/nav_action_follow_path` |
| `boustrophedon_coverage` | action client `nav_action_follow_path` (mower_coverage:1769) | none | `/nav_action_follow_path` |
| `mower_ws_bridge` | the wake topic, built from the node's full name (mower_ws_bridge/hub:208) | — | `/mower_ws_bridge/wake` |

Every rule above is passed to `mower_rsd` as a node-scoped `-r <node>:<from>:=<to>` (robot.launch.py:77-99, 230-243).

- mower_bridge's resolver oracle (`crates/mower_bridge/tests/resolver.rs`) has rcl itself resolve this table, and every
  other name of the modules, with these rules: all 205 come out as the manifest and `Remaps::resolve` say (the
  five action ends of step 5 among them).
- Trap for the standalone `mower_localize`: `run_all` gives its three roles a `ModuleCtx` without the process's rules
  (`src/mower_rs/crates/mower_localize/src/lib.rs:86-88`), so their `resolve_topic` would not apply them. The localize
  conversion hands them `m.remaps`.

### 5.5 QoS: DDS-incompatible pairs, and the traps the bridge can create

**No pair is refused today.**

- **Best-effort writers.** Only two exist: `/mower_base/telemetry` (mower_base:953) and `/mower_base/wheel_override`
  (mower_pid_autotune:839). Every reader of both is best effort (2.2). rosbag2 adapts to whatever writers it finds;
  that is upstream behaviour, unconfirmed.
- **Transient-local readers.** Every one of them has only transient-local writers:
  - `/mower_base/led_command`, `/mower_base/firmware_info`, `/nav_operation_active`, `/adapter/map_datum`;
  - all the OccupancyGrid and MarkerArray readers;
  - `/tf_static`;
  - the recorder's overrides (qos_overrides.yaml);
  - `/mower_recorder/status`.
- **Readers outside the process** (twist_mux, robot_state_publisher, nav2's StaticLayer and tf2) see only reliable
  writers from the process. A reliable writer matches any reader. Where those readers are transient local (StaticLayer,
  tf2 static), the writers are transient local too. So the unconfirmed upstream reader QoS cannot cause a refusal.
  twist_mux's readers (`R V 1`) and robot_state_publisher's (`BE V 5`) are measured now.
- **ws_bridge** picks its reader QoS from the publishers it discovers at the first subscribe: reliable only if all are
  reliable, transient local only if all are (mower_ws_bridge/hub:298-315). It then keeps that QoS for the life of the
  bridge. Only a best-effort writer appearing later on an allow-listed topic could be refused, and none exists.
- **The app's subscribe `qos` hint** (e.g. app/mission_mock_provider:607) is ignored. The bridge reads only `topic`,
  `type` and `throttle_rate` (mower_ws_bridge/client:97-110).

**What could still change behaviour in E2:**

a. **Bridge writers must copy the source's durability and reliability.**
   - A volatile DDS writer for `/map_grid` or `/map_grid_global` would be refused by nav2's transient-local StaticLayer,
     and nav2 would never get a map.
   - A volatile writer for a latched app topic would make ws_bridge's `matching_qos` pick a volatile reader. Late app
     clients would then lose the replay of `/robot/online`, the map layers and the rest.
   - The same applies to the recorder's transient-local overrides.
   - The bridge's DDS *readers* (DDS→bus) mirror the module readers they stand in for instead, so that every pair DDS
     has today stays: best effort where a module reader was (the EKFs' `/odom`, robot_status's `/fix`), which also
     matches the test harnesses' best-effort writers; transient local only where the module readers were and every
     writer is (mower_bridge README, "The policy").

b. **Bridge writer lifetime.** Today a module's latched history disappears when it restarts: the node and its writer go
   away (rsd/main.rs:40-49). The bus behaves the same way (bus/README:33-40). A bridge writer that outlives the bus
   writer would keep serving a stale latched value, e.g. `/robot/online` = `true` after `robot_status` died. Tie each
   bridge writer to its source writer.

c. **SystemDefaults.** `mower_base`'s `/odom`, `/joint_states`, `/tf` and the cmd_vel subscription use `SD`
   (mower_base:936-965), but the bus has no such profile (bus/qos.rs:40-72; bus/README:110-117). Write CycloneDDS's
   reading explicitly: `R V 1`.

d. **Echo on topics bridged both ways.** `/tf_static` is the only one today: rsp's frames go in, navsat's `utm` goes
   out. The bridge's DDS reader must not take back its own samples. rcl's `ignore_local_publications` is not the fix:
   all of `mower_rsd` is one participant (rsd/main.rs:346-354), so it would also hide modules that are still on r2r.
   Filtering by writer is needed. Whether r2r exposes the publisher GID is unconfirmed.
   - **Decided for E2:** navsat's `utm` writer stays on r2r until E4 (mower_bus_ros README, option 1), so `/tf_static`
     is DDS→bus only, and mower_bridge's policy refuses any name that would need both directions.
   - The only combination of module moves that would need one is mower_base on the bus with exactly one of the two
     `/mower_base/led_command` writers (`robot_status`, `pid_autotune`). Step 6 moves pid_autotune with the base,
     after status, so the order never meets it.

e. **Several latched writers on one topic:**
   - `/mower_base/led_command`: `robot_status` and `pid_autotune`;
   - `/free_space_inflated` and `/risk_map_inflated`: `map_manage`, plus coverage's silent writer;
   - `/tf_static`;
   - `/mower_base/rear_light`.

   On the bus, a late transient-local subscriber gets every live writer's retained samples in publish order
   (bus/README:83-90). With depth 1 that means the newest wins. DDS leaves the order across writers unspecified, so the
   bus is deterministic where DDS was not.

f. **Graph-presence checks.**
   - mower_base's arm latch checks whether `/drivetrain_guarded_cmd_vel` and `/mower_base/wheel_override` have a
     publisher (mower_base:160-185). A bridge always counts as one, so move the guard and pid_autotune with the base.
   - `/rosapi/topics` (3.3) and graph_snapshot's node list (5.3) read the DDS graph.
   - `autotune_telemetry_check.py` checks pid_autotune's DDS subscriber count while idle
     (`src/mower_rs/tools/README.md:17`). The bridge's `from_ros` DDS reader exists only while the bus has a reader
     (mower_bus_ros README, "Graph presence"), so the check holds through the bridge, up to its 200 ms dwell.

### 5.6 High-rate and large messages

On the RK3568, every DDS event costs about 1 ms of CPU (PLAN:39-41). An in-process hop costs microseconds (PLAN:39). So
these are where moving to the bus pays most, and where a bridge costs most.

| Topic | Rate | Size | Note |
|---|---|---|---|
| `/tf` | 76.6 Hz in 2026-09 (REFACTOR:19); now unknown | small | no in-process consumer; bus→DDS until E9 |
| `/odom`, `/joint_states` | 25 Hz | small | `/joint_states` exists only for rsp |
| `/navigation_coordinator_lock`, `/navigation_safety_stop`, `/cmd_vel_guard_input`, `/drivetrain_guarded_cmd_vel` | 20 Hz each at idle | small | the idle drivetrain chain runs through twist_mux: bus→DDS→bus until E4 |
| `/manual_command_clock` | 20 Hz | small | to the app |
| `/odometry/global`, `/odometry/local` | ≤20 Hz | small | in-process / no consumer |
| `/nav_cmd_vel` | 20 Hz while navigating | small | nav2 or *nav_core* to twist_mux |
| `/mower_base/telemetry` | 15-17 Hz | about 1.2 kB JSON, so about 20 kB/s | three in-process readers |
| `/imu/data`, `/robot/telemetry`, `/app_joy_cmd` | 10 Hz | telemetry JSON size unknown | |
| `/fix` | 4 Hz | small | |
| `/map_grid`, `/map_grid_global`, `/free_space*`, `/risk_map*`, `/chennal_map*` | on change, latched | 160 kB of data for the 400 × 400 start-up map (mower_map:139-142); zone maps vary | the bus hands out one `Arc` per message with no copy per subscriber (bus/README:74) |
| `/adapter/map_layers/*` | on change, latched | about 214 kB of JSON for a map grid (mission.launch.py:416-418) | to the app and the recorder: bus→DDS until E3 and E5 |
| `/coverage_path`, `/coverage_path_markers` and the other MarkerArrays | per plan or edit, latched | unknown | |
| `/gps/status` | 1 Hz | unknown; includes the satellite list | |

### 5.7 Parameters that rename topics or services

E2 has to keep reading these, or replace them with the resolver's output. "Production value" is what mower_rsd.yaml or
the launch files set. "default" means nothing overrides the default.

| Node | Parameter | Default (code) | Production value |
|---|---|---|---|
| `mower_base` | `cmd_vel_topic`, `odom_topic`, `joint_states_topic`, `tf_topic` | `/drivetrain_guarded_cmd_vel`, `/odom`, `/joint_states`, `/tf` (mower_base:896-899) | default |
| `mower_base` | `telemetry_topic`, `firmware_info_topic`, `led_topic`, `pid_topic`, `override_topic`, `servo_topic`, `blade_topic` | `/mower_base/...` (mower_base:900-907) | default |
| `mower_base` | `rear_light_topic`, `rear_light_timeout_s` | `/mower_base/rear_light`, 6.0 s (mower_base:927-928) | default |
| `mower_base` | `odom_slow_topic` | `/odom_slow` (mower_base:912) | `/odom_slow` (mower_rsd.yaml:49) |
| `robot_status` | `heartbeat_source_topic`, `odom_topic` | `/odom`, `/odom` (robot_status:91, 97) | `/odom_slow`, `/odom_slow` (mower_rsd.yaml:69, 72) |
| `robot_status` | `gps_fix_topic` | `/fix` (robot_status:101) | `/fix` (mower_rsd.yaml:73) |
| `robot_status` | `firmware_info_topic`, `nav_active_topic`, `led_topic`, `gps_filtered_topic`, `imu_topic`, `base_telemetry_topic`, `battery_topic`, `aon_battery_topic` | robot_status:95-106 | default |
| `gps` | `fix_topic` | `/fix` (mower_gps:390) | `/fix` (mower_rsd.yaml:105) |
| `gps` | `status_topic` | `gps/status`, relative (mower_gps:391) | `gps/status` (gps.yaml:35) |
| `ekf_filter_node_odom`, `ekf_filter_node_map` | `odom0`, `odom1`, `imu0` | from ekf_params | `odom`, `odometry/gps`, `imu`, all relative (ekf_params:16, 26, 68, 78, 88) |
| `ekf_filter_node_map` | `odometry_slow_topic` | `odometry/global_slow`; empty on the other nodes (mower_localize/ekf:56-61) | `/odometry/global_slow` (mower_rsd.yaml:126) |
| `nav_action_server` | `gps_fix_topic`, `gps_odometry_topic`, `imu_topic` | `/fix`, `/odometry/gps`, `/imu/data` (mower_nav:1604-1615) | `gps_fix_topic: /fix` (mower_rsd.yaml:146) |
| `nav_action_server` | `nav_core.{odom_topic, map_topic, global_map_topic, cmd_vel_topic, plan_topic}` | `/odom`, `/map_grid`, `/map_grid_global`, `/nav_cmd_vel`, `/plan` (mower_nav/core_backend:114-128; mower_nav:1534-1549) | default; only used with *nav_core* |
| `path_record_node` | `robot_pose_source_topic` | `/odometry/global` (mower_record:180) | `/odometry/global_slow` (mower_rsd.yaml:157) |
| `path_record_node` | `rear_light_topic` | `/mower_base/rear_light` (mower_record:183) | default |
| `flutter_adapter` | `robot_pose_source_topic` | `/odometry/global` (mower_adapter:171) | `/odometry/global_slow` (mower_rsd.yaml:162) |
| `battery_state` | `base_telemetry_topic`, `battery_topic`, `aon_battery_topic` | mower_battery:236-238 | default |
| `pid_autotune` | `telemetry_topic`, `pid_topic`, `override_topic`, `led_topic`, `status_topic`, `nav_active_topic` | mower_pid_autotune:807-812 | default (mower_rsd.yaml:176-180) |
| `pid_autotune` | `lock_service` (a service name) | `/mission_operation_lock` (mower_pid_autotune:813) | default |
| recorder (outside) | `gps_topic`, `fault_topic`, `rear_light_topic` | `/fix`, `/mower_recorder/fault`, `/mower_base/rear_light` (recorder_manager:79-83) | `gps_topic` and `fault_topic` from record.launch.py:87-88; `rear_light_topic` default |

Note that `mission.launch.py` passes the old separate binaries some of these values itself (mission.launch.py:329-333,
425, 519-525). With `rust_daemon` only mower_rsd.yaml applies. Both sources agree today.

### 5.8 Other findings

1. **`/mower_base/rear_light` had no reader** (fixed 2026-10-30, `fix/base-rear-light`).
   - The Rust base had no rear-light subscriber and no overlay byte. The C++ `MowerSystem` subscribed with `QoS(8)` `R TL`
     (src/mower_hardware/src/mower_system.cpp:115-118, 309-321) and set the 0x03 frame's `overlay` byte (payload byte 6,
     `REAR_RECORDING` 0x01: a red breath on the back strip over whatever `/mower_base/led_command` asked for) while any
     `source`'s last `{"effect":"recording"}` was younger than `rear_light_timeout_s` (6 s); a source's `"off"` cleared
     only itself (`RearLightSources`, mower_protocol.cpp:339-353). With no base light asked for yet it sent SHOW with
     the overlay (`send_led_if_needed`, mower_system.cpp:323-352).
   - Since `RUST_BASE`, two writers published into nothing: `path_record_node` (zone recording, `source` `path_record`)
     and the recorder (`source` `bag`, "the rear light breathes red", compose:153).
   - mower_base has the reader back, with the C++ semantics (mower_base_core `RearLightSources` and `led_frame`,
     checked against mower_protocol.cpp by the protocol oracle). It stays on the module's r2r node when the base moves
     (E2 step 6), until E5: path_record writes on the bus and the recorder on DDS, and a bus reader would need the name
     both ways (5.5 d).
2. **Coverage holds silent writers.** It creates publishers on `/free_space_inflated` and `/risk_map_inflated` that never
   publish (`#[allow(dead_code)]`, mower_coverage:442-445). On the bus they are harmless, but they fix the type and show
   up in introspection.
   - They also looked like a two-way name in step 3: map on the bus writes both, the adapter on the bus reads both,
     and coverage, still on r2r, writes them on DDS. A writer that never publishes has nothing to forward, so
     mower_bridge's manifest marks them `silent`, and its policy bridges both names bus→DDS only.
   - **Dropped in step 5.** Nothing depends on them: no reader of either name asks for its writers
     (`publisher_count`, `publisher_changes`), neither name is in ws_bridge's `topics_sub` (so `/rosapi/topics` does
     not see them), and the recorder records data, not writers. The one thing that counts bus writers is the bridge's
     `to_ros` writer lifetime (5.5 b), which a silent writer would have kept alive after map_manage went.
3. **`mower_base` holds a `/tf` writer that never publishes** (mower_base:619-621). It shows up in the graph as a
   publisher, and on the bus it would show up in introspection.
4. **The plan is wrong about `/map_manage/get_parameters`.** PLAN:356 says E3 must emulate it for the app. The app never
   calls it (app 34cd5d3), and ws_bridge.yaml does not allow it. Its only callers are the adapter, in process, and the
   recorder's param dump. The app's only parameter call is `/boustrophedon_coverage/set_parameters`.
5. **Unused policy entries.**
   - With no producer: `/gps/fix` (ws_bridge.yaml:18).
   - Allowed but with no client in the app or the Studio: `/coverage_progress`, `/resume_coverage`,
     `/coverage_progress_status` and `/cencel_nav2`.
   - Used only by the Studio, not the app: `/robot/telemetry`, `/pid_autotune/status`, `/pid_autotune`,
     `/mower_base/servo_command` and `/mower_base/blade_command`.
6. **`/rosout` loses the logs of moved modules.** A module that drops its r2r node will likely stop appearing in
   `/rosout`. rcl publishes a log to `/rosout` only under a live node's logger name; that is expected, not checked for
   r2r. The recorder records `/rosout` as "all node + service logs" (record_topics.yaml:45). Keep each moved module's r2r node
   until E5, or forward its log lines.
7. **The idle drivetrain chain runs at 20 Hz end to end.** The coordinator lock and the safety stop
   (mower_nav:1981-1998) go through twist_mux and then the final guard to the base. In E2 that is two bridge crossings
   per message at 20 Hz, until E4.
8. **The default app checkout is old.** `~/work/mower_lawer_app` is on `feat/custom-coverage-map-pan` at `b67983b`
   (2026-09-18). It lacks `/gps/status`, `/zone_sequence_status` and the zone-sequence services. This file follows the
   app's origin/main `34cd5d3` (2026-10-25).
9. **The recorder's watched-node fault can never fire.**
   - `watch_nodes` holds `map_manage` and `boustrophedon_coverage`, with no leading slash (record.launch.py:42, 69;
     graph_snapshot:48).
   - The names it is compared against are full names such as `/map_manage` (graph_snapshot:33-36, 63, 95).
   - So `WATCHED NODE DOWN` and the snapshot flush on `/mower_recorder/fault` never happen. This is a bug outside E2, but
     it means no fault fires when E2 removes those nodes.

## 6. Proposed E2 order

### Step 0: prerequisites (no module moves)

**Done** (2026-10-27). The resolver is `ModuleCtx::resolve_topic` / `resolve_service`
(`src/mower_rs/crates/mower_rs_common/src/names.rs`). The configuration is
[`crates/mower_bridge`](../src/mower_rs/crates/mower_bridge/README.md):

- a typed manifest of every module endpoint (sections 2 and 3), checked against the launch file's rules, the parameter
  files and rcl's own resolution of 5.4;
- one flag per module (`src/on_bus.rs`), all false at step 0 and flipped as each module moved;
- a policy that derives the bridge's declarations from the modules of a process and their flags;
- a check that every endpoint a module makes on the bus is in the manifest;
- `mower_rsd`'s bridge module (`mower_bus_bridge`, started only when a module is on the bus) and a runner for the
  standalone binaries.

`mower_bus_ros`'s `service_from_ros` now serves on the bus only while the graph shows the DDS server, so a bus caller
sees an r2r server's absence as it did (mower_bus_ros README, "Graph presence").

What it was asked to provide:

- **The resolver** in `ModuleCtx`, covering 5.4. Test it with the table as vectors.
- **`mower_bus_ros`.** Generate its configuration from this file's tables, with:
  - QoS copied from the source endpoint (5.5 a);
  - the lifetime tied to the source writer (5.5 b);
  - writer-filtered two-way topics (5.5 d): not needed, since no name crosses both ways (5.5 d, decided);
  - service proxies in both directions;
  - a per-topic "until step" tag, so a bridge is deleted when its outside end goes away. mower_bridge tags each
    outside node instead (`Outside::until`): a bridge goes when its last outside end is taken out of the manifest.
- **Keep the r2r nodes for now.** Keep each moved module's r2r node alive for its parameter services, graph presence and
  `/rosout` until E3 or E5 (5.3, 5.8 #6).

### Steps 1 to 7: modules, lowest risk first

1. **battery.**
   - It has the smallest IO: one best-effort subscription, two publishers, no service, nothing latched.
   - Its estimator is already a pure core (`estimator.rs`).
   - It is the cheapest test of both bridge directions with nothing safety-relevant: telemetry DDS→bus,
     `/battery_state` bus→DDS. Check with shadow_compare (PLAN:102) and probe_app.
2. **status (`robot_status`).**
   - It commands nothing on the drivetrain, so it is low risk. Its only outputs besides the app topics are the LED
     request and the host update/restart request file behind `/system/*` (robot_status:331).
   - It is the module the app checks hardest. probe_app.py fails on `/robot/online` not true, stale base or odometry in
     the telemetry, and rates below 80 % (probe_app.py:40-83, 127-133).
   - It proves the bridge's latched replay. Most of its 9 inputs are bridged DDS→bus until their producers move.
3. **map, record, adapter (mission data).**
   - They have no motion authority, and they carry most of the large latched traffic: grids, MarkerArrays, JSON layers.
     They also carry the 1 Hz and 2 Hz service polling, which becomes in-process.
   - shadow_compare already checks their JSON layers byte for byte (PLAN:102).
   - It is the first step that needs:
     - the transient-local bridge to nav2's StaticLayer (5.5 a);
     - the parameter-service proxies (app, recorder), or keeping `map_manage`'s r2r node for them (5.3). Decided:
       the parameter services stay on r2r in E2, and so do the adapter's 1 Hz `get_parameters` calls to them.
   - Coverage stays behind because of the action.
4. **imu, gps, localize.**
   - localize is already pure core plus thin IO (PLAN:355) and already shares transforms in process through tfbus.
     Moving the two drivers with it keeps the EKF and navsat inputs `/imu/data` and `/fix` off the bridge.
   - Its inputs then cross the bridge only as `/odom` (base still on r2r) and rsp's `/tf_static`.
   - It steers nav2 through `/tf` (`transform_tolerance` 0.1 s, nav2_params:86, 204), so it comes after the
     display-only steps.
   - Check with localize_compare.py and datum_check.py.
5. **coverage and nav.**
   - This is the only in-process action pair. It uses the bus's actions (bus/README, "Actions"; the cheat sheet maps
     every r2r call nav and coverage make), plus the five dispatch and lock services. nav's IO is the largest
     (`mower_nav/src/lib.rs`, 2094 lines).
   - Action names resolve with the topic rules (rcl_action), so `resolve_topic` names them.
   - nav keeps r2r for nav2's two actions and `/bt_navigator/get_state` until E9, or until *nav_core* is the default.
     Moving nav2's actions onto `action_from_ros` is not mechanical (bus_ros/README, "nav2's actions"). nav2's
     `/rosout` warnings come in through the bridge (`from_ros`, 2.6).
   - Check with nav_progress_check.py, coverage_cancel_check.py and nav_core_check.py. Standalone, nav serves its two
     actions to the tools through `action_to_ros`, and coverage reaches the tool's server through `action_from_ros`.
   - **Done** (2026-10-27, `feat/e2-nav`). Both modules' topics, services, clients and the action pair are on the bus,
     on the module's clock; the r2r nodes stay for the parameters, coverage's six parameter services, the log, the
     graph and nav2. mower_bridge plans actions (`action_to_ros` for a bus server, `action_from_ros` for a bus client
     whose server is elsewhere), and `action_from_ros` serves on the bus only while DDS has a server, as
     `service_from_ros` does. What the conversion keeps of r2r's actions, and where the bus differs:
     - after an accepted action cancel nav ends the goal CANCELED whatever the outcome, as r2r let it (only `cancel()`
       could deliver a result there); and it sends the end only once it has released the goal, since the bus hands the
       result over at once and coverage sends its next leg at once;
     - `GoalRejected` is coverage's one definite refusal; `NoResponse`, `ServiceUnavailable` and `Closed` leave the
       goal possibly running, and enter the tracker, as r2r's other errors did;
     - when nav's module ends mid-goal, coverage learns it at once (the result fails with `ServiceUnavailable`, the
       status is `Unknown`), where r2r told it nothing until the leg's timeout (at least 10 min) or, for
       `/zone_exec_path`, never. The sequence fails at once, and the goal goes to the tracker, which retries the
       correlated cancel every 2 s; a restarted nav cannot confirm a dispatch it never had, so the tracker, and the
       refusal of a new sequence, stay until coverage restarts, as after the timeout before.
6. **base, pid_autotune and the final `velocity_command_guard`, together.**
   - Base and guard are pure core plus thin IO (`mower_base_core`, `core.rs`; PLAN:355).
   - Moving the three together makes the drivetrain edge and the wheel-override edge in-process, so the arm latch's
     publisher check (5.5 f) never sees a bridge.
   - They drive the wheels and the blade, so they go last among the modules. Do this after E3, so the Studio's blade
     dead-man does not cross a bridge.
   - Check with base_compare.py and base_ab.sh, slow_copy_check.py and autotune_telemetry_check.py (updated), then a
     supervised drive.
   - **Converted, flags off** (2026-10-30, `feat/e2-step6`). The three modules have their bus IO next to their r2r IO
     and pick one at start by their own flag (mower_bridge's `on_bus.rs`: `base`, `pid_autotune`, and the final
     guard's instance flag `velocity_command_guard`; `instance_on_bus`). The flags stay false: with them off the plan
     is E2's line for line and nothing of the bus IO runs. The owner switches them on with E3.2, after a supervised
     test. What the bus IO keeps of r2r, and where it differs:
     - every name through `m.resolve_topic` / `m.resolve_service`, every QoS as before, SystemDefaults written as
       CycloneDDS reads it (R V 1); the base's cycle, clocks, telemetry and logs, pid_autotune's session, its timing
       and its status JSON, and the guard's rules (`core.rs`) are the same code;
     - the guard reads its clocks from `m.clock` (in production the system clock and CLOCK_MONOTONIC), and stops its
       command and watchdog tasks before its last zero, so nothing it still had in hand can follow that zero;
     - the arm latch counts the bus's writers of `/drivetrain_guarded_cmd_vel` and `/mower_base/wheel_override`
       (`publisher_changes`) instead of the DDS graph's. In mower_rsd those are the final guard and pid_autotune
       themselves, so a guard restart shows as before (its writer goes and comes back); where such a writer is on
       DDS (the base's binary alone, base_ab.sh, the rclpy guards) the bridge's `from_ros_mirrored` bus writer exists
       only while DDS shows one (5.5 f);
     - a DDS writer of a name that is now in-process (`/drivetrain_guarded_cmd_vel`, `/mower_base/wheel_override`,
       `/odom_slow`, ...) no longer reaches the module that reads it, as for every pair of steps 1-5;
     - pid_autotune's telemetry reader goes at the first frame after its session, where r2r dropped it one frame
       later, and its lock call learns at once what r2r learnt at the next spin;
     - with mower_base on the bus, `rust_status:=false` or `rust_pid_autotune:=false` (their rclpy nodes) refuse the
       plan (two `/mower_base/led_command` writers, one on each side, 5.5 d): mower_rsd does not start. Those two
       switches must stay true once the flags are on.
7. **`manual_velocity_guard`, last, or as part of E4.**
   - Every edge but one leaves the process until E3 and E4: ws_bridge for `/app_joy_cmd` and the clock, twist_mux for
     `/joy_cmd`. The one in-process reader is nav, which also reads `/joy_cmd`. Moving it earlier only adds bridge hops
     to the joystick path.
   - That path runs on a 0.8 s age budget that 4G already strains (mower_rsd.yaml:82-88).
   - **Converted, flag off** (2026-10-30, `feat/e2-step7`, on step 6). The manual guard's bus IO adds the session
     (a fresh `manual-session-v1:<32 hex>` per start) and its command clock on `/manual_command_clock` (R V 1, a
     frame at once and one per `command_clock_period_s` on the module's clock, as rcl's wall timer), behind its
     instance flag `manual_velocity_guard` (false). With steps 6 and 7 on and ws_bridge on the bus, the joystick path
     `/app_joy_cmd` -> manual guard -> `/joy_cmd` (nav in-process, twist_mux on DDS until E4) and the clock to the app
     cross no bridge; with ws_bridge on r2r, `/app_joy_cmd` comes in from DDS (R V 1, the guard's reader).

**Not in E2:**

- `bridge`: E3.
- `agent`: no ROS endpoint (rsd/main.rs:181-187).
