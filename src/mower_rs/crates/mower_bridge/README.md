# mower_bridge

What the bus/DDS bridge of step E2 forwards, and why ([`docs/ROS_FREE_PLAN.md`](../../../../docs/ROS_FREE_PLAN.md)
Phase E; [`docs/BUS_TOPICS.md`](../../../../docs/BUS_TOPICS.md) section 6, step 0).

- **Why:** E2 moves each `mower_rsd` module's topics, services and timers from r2r to the bus
  ([`mower_bus`](../mower_bus/README.md)), one module at a time, while nav2, twist_mux, robot_state_publisher, the Python
  recorder, the app's ws_bridge and the CI tools keep talking DDS. Whatever crosses the boundary has to be forwarded by
  a [`mower_bus_ros`](../mower_bus_ros/README.md) bridge, with the QoS the module used, in one direction per name.
  Nothing outside must notice.
- **What it is:** a library with four parts.
  - **The manifest** (`src/manifest.rs`): every endpoint of every module instance.
  - **The flags** (`src/on_bus.rs`): which modules are on the bus.
  - **The policy** (`plan`): a pure function from the instances of a process and their flags to the bridge's
    declarations.
  - **The check** (`coverage`): every endpoint a module made on the bus must be in the manifest.
- **Where it runs:** as `mower_rsd`'s bridge module, and next to a moved module in its own binary (`module_main`).
- **Today:** battery, status, map, record, adapter, imu, gps, localize, coverage and nav are on the bus (E2 steps
  1-5); base, pid_autotune and the guards stay on r2r until they move after E3. ws_bridge is in the manifest (step E3),
  on r2r.

## The manifest

`MODULES` holds the 14 modules of `mower_rsd` that have endpoints (19 instances): the 13 typed modules, and ws_bridge
(`bridge`, step E3), whose endpoints are its allow-list. `OUTSIDE` holds the nodes they depend on.

- **The module code is the source of truth.** BUS_TOPICS.md sections 2 and 3 cite the lines.
- **One entry per endpoint:** what it does (`publishes`, `subscribes`, `serves`, `calls`, and for an action
  `serves_action`, `calls_action`), its name, its r2r type and, for a topic, the QoS profile the code hands r2r, written as BUS_TOPICS.md writes it (`R V 10`, `BE V 1`, `R TL 1`,
  `SD`). The types are values (`MsgType::of::<T>()`), so the manifest is a static table that also knows how to declare
  each name on a bridge.
- **Two names per endpoint:**
  - `code`: what the code passes in production, a literal or the production value of the parameter named
    (`param`);
  - `name`: the absolute name rcl makes of it with robot.launch.py's rules (BUS_TOPICS.md 5.4, 5.7).
  - In a process, an instance that runs there has its names resolved from `code` with that process's `-r` rules, as
    `ModuleCtx::resolve_topic` resolves the module's own. So the bridge and the module agree in every process, the
    standalone binaries' included. An instance that runs elsewhere has its production `name`.
- **Marks:**
  - `stays_on_r2r`: the endpoint stays on the module's r2r node when the module moves.
    - navsat's `/tf_static` writer (the `utm` frame), until E4: E2 decision 3, option 1 of mower_bus_ros's
      "`/tf_static` question". `/tf_static` then only goes DDS → bus.
    - The adapter's 1 Hz calls to `/map_manage/get_parameters` and `/boustrophedon_coverage/get_parameters`: parameter
      services stay on r2r (E2 decision 2), and so do their callers.
    - Coverage's `/boustrophedon_coverage/set_parameters` server, the one parameter service the app calls (through
      ws_bridge): listed so that the plan names its server (E3 decision 9).
    - nav's clients of nav2, until E9: `/bt_navigator/get_state` and the actions `/navigate_to_pose` and
      `/follow_path`, next to nav2 outside the process.
  - `silent`: a writer that is made and never publishes: mower_base's `/tf` (`enable_odom_tf` is false). Coverage's
    `/free_space_inflated` and `/risk_map_inflated` were (BUS_TOPICS.md 5.8 #2) until step 5 dropped them.
  - `only`: not in production. nav's `nav_core` backend (robot.launch.py `rust_nav_core:=true`, CI's
    `nav_core_check.py`), with `/tf` and `/tf_static` for `nav_core.pose_source: tf`.
  - `replaced_by`: an outside node's endpoint that a module of `mower_rsd` takes the place of (E4). While that module
    runs in the process, robot.launch.py does not start the node, and the policy leaves the endpoint out.
- **Outside** (BUS_TOPICS.md 5.3, "DDS→bus"), each with the step that removes it: twist_mux's `/cmd_vel_guard_input`
  (E4), robot_state_publisher's `/tf_static` and `/tf` (E4), nav2's `/rosout`, `/bt_navigator/get_state` and two
  actions (E9), the two teleop nodes (off by default), and the Python recorder (E5): `recorder_manager`'s status, its
  `/mower_recorder/start`, `/stop` and `/snapshot` and its `/mower_base/rear_light`, and `bag_store`'s bag list and
  command answers. Their QoS is upstream behaviour, unconfirmed (BUS_TOPICS.md 1.3), except the recorder's, which its
  code sets, and twist_mux's and robot_state_publisher's, which were measured for E4.
- **ws_bridge** (`bridge`): every name of its allow-list (`mower_bringup/config/ws_bridge.yaml`), `topics_sub`'s globs
  expanded to the names a module or a node outside writes (28, read `R TL 10`), `topics_pub` (4, written `R V 10`) and
  the services (32, called). `the_bridge_module_is_ws_bridge_yaml_s_allow_list` keeps the two in step, the yaml's
  service types included.
- **Actions** (step 5): nav serves `/nav_action` and `/nav_action_follow_path`, coverage sends its goals to the second.
  An action is a namespace of its own, as on DDS (`<name>/_action/...`) and on the bus, and its name resolves with the
  topic rules, as rcl_action resolves it.
- **Not listed:**
  - parameter services, which stay on r2r in E2 (decision 2), but for coverage's `set_parameters` (above);
  - endpoints only a non-production setting makes, but for nav's `nav_core` ones: robot_status's own heartbeat
    subscription (`heartbeat_source_topic` ≠ `odom_topic`), the final guard's command clock
    (`require_command_session`). The check reports them if they appear.
- **Checked against** (`src/tests.rs`, `tests/resolver.rs`):
  - mower_rsd's module table and node names;
  - ws_bridge's allow-list;
  - robot.launch.py's `_DAEMON_REMAPS` (`PRODUCTION_REMAPS`, in argv order);
  - the parameter files robot.launch.py passes, in its order (mower_rsd.yaml, gps.yaml,
    dual_ekf_navsat_params.yaml): every name a parameter sets has the file's value;
  - rcl itself: the resolver oracle;
  - one type per name, one server per service or action.

## The flags

`src/on_bus.rs` has one flag per module. Moving a module is flipping its flag, in the commit that merges its
conversion. A flag says "this module's code uses the bus"; the module itself does not read it.

- **Set at run time:** `RUNTIME` lists the modules whose flag mower_rsd sets when it starts, from how it runs them:
  ws_bridge, whose backend is a module argument (`--backend bus`, robot.launch.py's `ws_bridge_bus`). It is the app's
  only door, the in-app update's included, so it moves back without a new image. Its flag in `ON_BUS` stays false,
  the default backend's.
- **Where mower_rsd uses it:** the bus bridge's plan (`rsd_plan_with`), which the bridge gets in its arguments
  (`rsd_args`: the ids of `--modules`, then `--on-bus bridge`), and the manifest check (`on_bus_with`). Any other id
  there is refused.

## The policy

`plan(running, remaps)`: what the bridge of a process declares. `running` is the instances that run in that process,
each with its flag; every other instance is elsewhere, on DDS.

- **Topics:**
  - **to_ros:** a topic an instance on the bus publishes, whether or not anything outside reads it. The DDS graph,
    `/rosapi/topics` (probe_app.py fails when a topic leaves it) and the recorder then see the topics they see
    today.
  - **from_ros:** a topic an instance on the bus subscribes to, when one of its writers is not on the bus here (an r2r
    module, a module that runs elsewhere, a node outside), or when no writer is known at all (a tool may write it).
  - **Both:** an error at start-up (BUS_TOPICS.md 5.5 d), with two exceptions:
    - when every bus writer of the name is silent, the name only comes in (coverage, standalone:
      `/risk_map_inflated`);
    - when the second direction is due to endpoints not in production (`only`), those are left out (nav's `/tf`
      reader with `pose_source: tf`, once localize is on the bus). The plan's notes say so.
- **Services:**
  - **service_to_ros:** a service an instance on the bus serves.
  - **service_from_ros:** a service an instance on the bus calls, whose server is not on the bus here, or is unknown.
    The bridge's bus server for it exists only while the graph shows the DDS server, so a caller finds what it found on
    r2r: `ServiceUnavailable` at the call and a `wait_for_service` that waits while that server is down (mower_bus_ros
    README, "Graph presence"). mower_rs_common's guard then says "unavailable", as before, instead of meeting a lock
    service that never answers.
  - **Both** (a server on the bus and another elsewhere): an error.
- **Actions,** by the services' rule:
  - **action_to_ros:** an action an instance on the bus serves: nav's two, which the CI tools' rclpy clients and
    `nav_action_client.py` reach on DDS, also in mower_rsd with coverage on the bus;
  - **action_from_ros:** an action an instance on the bus sends goals to, whose server is not on the bus here: coverage
    standalone (coverage_cancel_check.py's rclpy server), or with nav still on r2r. Its bus server too exists only while
    the graph shows the DDS server, so coverage finds "Navigation action server unavailable" as before.
  - **Both:** an error.
- **QoS of to_ros:** the bus writers' profile (5.5 a, c). Writers that differ get one profile that matches every reader
  either of them did: the deepest depth, reliable if any is reliable. Writers that differ in durability are an error.
  So far that is only `/tf`: the EKFs' `R V 100` and mower_base's silent `SD` give `R V 100`.
- **QoS of from_ros:** the DDS reader stands in for the bus readers, so the pairs DDS has today, (a DDS writer, the
  module's own r2r reader), stay what they are.
  - **Best effort** if any bus reader is. It matches every writer one of them did, the test harnesses' best-effort
    writers included (`localize_compare.py`), and a reliable writer still matches it: the EKFs read mower_base's
    `/odom` as `BE V 10`, robot_status reads `/fix` as `BE V 5`.
  - **Transient local** where a bus reader is and every DDS writer is. The writers' latched values then reach the bus,
    and the bridge's bus writer keeps them for late joiners (`/mower_base/firmware_info` and `/nav_operation_active`
    for robot_status). Otherwise volatile, as the readers were: `/rosout` (rcl writes it `R TL 1000`) comes in
    volatile for nav's `R V 100`, so a restarted bridge does not replay ten seconds of logs.
  - **Depth:** the deepest bus reader's. `/tf_static`, written by robot_state_publisher and navsat (two DDS writers
    behind one reader and one bus writer), comes in `R TL 100`, tf2's static depth, so the bus keeps both writers'
    frames for a late joiner.
  - Writers that differ in durability while a reader wants history are an error.
- **Names:** the policy resolves each name as the module's node does. rcl then applies the process's `-r` rules to
  the bridge's node too, so a rule that would rename a resolved name again (a chain `/a:=/b` `/b:=/c`, a swap, a rule
  for the module's node name on a standalone bridge) would put the bridge's DDS endpoint on another name than the
  module's. `Plan::check_names` refuses such a name: `rsd_plan`, so mower_rsd does not start, and the bridge before it
  declares anything. robot.launch.py's rules rename none.
- **Deterministic:** topics first, then services, then actions, each sorted by name; the order of `--modules` does not
  matter.
- **What it derives for the steps** (`src/tests.rs` has every line):

| Bus | to_ros | from_ros | service_to_ros | service_from_ros |
|---|---|---|---|---|
| nothing | 0 | 0 | 0 | 0 |
| battery, status | 6 | 7 | 3 | 0 |
| + map, record, adapter | 39 | 11 | 28 | 2 |
| + imu, gps, localize | 48 | 9 | 32 | 1 |

- **ws_bridge on the bus** (E3.1, `ws_bridge_bus:=true`) adds the 13 names the app uses that only DDS has, and changes
  nothing else: `from_ros` of `/manual_command_clock` (`R V 10`), `/pid_autotune/status`, `/mower_recorder/status` and
  `/mower_recorder/bags` (`R TL 10`) and `/mower_recorder/command_result` (`R V 10`); `to_ros` of `/app_joy_cmd`, the
  servo and blade commands and `/mower_recorder/command` (`R V 10`); `service_from_ros` of `/pid_autotune`,
  `/boustrophedon_coverage/set_parameters` and the recorder's start and stop. With ws_bridge on r2r, or not in
  mower_rsd, the plan is E2's line for line.
- **E2 steps 6 and 7 after it** take the guards' and pid_autotune's names into the process. What stays for ws_bridge
  is the recorder's six (until E5), `set_parameters` (until the parameter services move), and the `to_ros` of the
  three command topics: ws_bridge writes them on the bus, and every topic written on the bus gets its DDS face.

- **Refused:** of all 16384 combinations of flags with the production set, only those that put mower_base on the bus
  with exactly one of robot_status and pid_autotune: `/mower_base/led_command` would then need both directions. Base and
  pid_autotune move together, after status (decision 4), so the order never meets it.

## Running

- **In mower_rsd:** the module `bus_bridge`, node `mower_bus_bridge`. mower_rsd starts it, first, when a module of
  `--modules` is on the bus, with the ids of `--modules` as its arguments, and supervises it like any module. It is
  not for `--modules`. A set the policy refuses does not start.
- **Standalone:** a moved module's `main.rs` calls `mower_bridge::module_main(id, node_name, worker_threads, run)` in
  place of `mower_rs_common::module_main`. The module runs on a bus of its own, next to a bridge on which every
  endpoint of the module faces DDS: the policy with every other module elsewhere. Without it, a moved module run as
  its own binary (robot.launch.py's `rust_daemon:=false` roll-back, the CI tools) talks to nothing.
  - The instance is the one rcl names after `node_name` (`-r __node:=`); a binary named after its module
    (`mower_localize`: `localize`) runs all of its instances; a module's only instance runs under any name.
  - **The bridge's node** has the module's node name, in the namespace `/mower_bus_bridge`
    (`/mower_bus_bridge/battery_state`). Launch's `name=` passes the binary a bare `-r __node:=<name>`, which renames
    every node of the process, so a node name of the bridge's own would come out as the module's, and two nodes would
    share a full name (mower_bus_ros README, "The node"). In a namespace of its own it never does, without a vendor/r2r
    patch for node options. Its log name is `mower_bus_bridge.<node>`, and tools that look at an endpoint's node name
    (`nav_core_check.py`) see the module's. The bridge warns if a rule renames it anyway (`__ns:=`).
  - A bridge that fails stops the module, and the process exits non-zero, for launch's `respawn`.
  - Every moved module's `main.rs` uses it. `tests/dds.rs` runs a battery module with `run_standalone`, and
    `tests/standalone.rs` runs `module_main` itself, under launch's `-r __node:=battery_state`.

## The check

The policy plans from the manifest, so an endpoint the manifest lacks is a name nothing bridges, and nothing else would
say so.

- **What:** every endpoint on the bus (mower_bus introspection: name, type, each endpoint's QoS), by scope: topics,
  services and actions. An endpoint of a running instance must be in that instance's manifest entry with the same type
  (an action's three) and QoS (`bus_qos` of the profile).
- **What it reports,** one line each:
  - no such endpoint in the manifest (a new or renamed name; a parameter set to a value other than production's);
  - another type or QoS there;
  - an endpoint the manifest keeps on r2r;
  - a module whose flag is still false: its conversion was merged without the flip, and nothing bridges it;
  - a scope that is no instance of the manifest.
  - For example: `robot_status publishes /robot/telemetry (std_msgs/msg/String, R V 1) on the bus, but the manifest
    has it as R V 10`.
- **When:** mower_rsd checks 2 s after its start and every 10 s from then on, with or without a bridge, and reports
  each gap once on stderr (`[mower_rsd] bus bridge: ...`). The standalone runner does the same.
- **In a module's tests:** `mower_bridge::coverage::assert_in_manifest(&m)` checks what the module made on its scope
  `m.bus` (other scopes are the test's), resolving the manifest's names with `m`'s rules, and `gaps_of(&m)` returns the
  lines.

## Moving a module

1. Convert its IO to `m.bus`, naming every endpoint through `m.resolve_topic` / `m.resolve_service`, with the QoS it
   had (`types::R_V_10.bus()` and the rest, or `mower_bus::Qos`). Keep the r2r node for parameters, parameter
   services, the log and graph presence (decision 1), and for what it reaches outside without a bridge (nav2's clients
   until E9).
2. Call `assert_in_manifest` in its tests. If an endpoint is new or changed, change the manifest with it.
3. Switch its `main.rs` to `mower_bridge::module_main`.
4. Flip its flag in `src/on_bus.rs`.
5. Update what looks at its DDS endpoints by node. In mower_rsd they now belong to `mower_bus_bridge`; a binary's
   belong to `/mower_bus_bridge/<node>`, which a tool that compares node names only (`nav_core_check.py`) takes for
   the module's. The bridge's DDS endpoints come and go with the module's bus ones (mower_bus_ros README, "Graph
   presence"), so counts on DDS stay what they were, up to its 200 ms dwell: `autotune_telemetry_check.py` still finds
   no subscriber while pid_autotune is idle (BUS_TOPICS.md 5.5 f). A bus endpoint of the bridge that exists all along
   (a `to_ros` reader, a volatile `from_ros` writer) changes a count on the bus.

## Tests

- `cargo test -p mower_bridge --lib`: the manifest against mower_rsd, the launch file, the parameter files and
  ws_bridge's allow-list; the policy for every step, all flag combinations, a subset (shadow_compare.py's
  `status,battery,adapter,guards`) and every module standalone; with ws_bridge on r2r, the production plan line for
  line as E2 merged it; the check. No DDS.
- `tests/dds.rs` (ROS domain 78, rmw_cyclonedds_cpp): synthetic converted `robot_status` and `flutter_adapter` modules
  and their planned bridge. A latched bus topic reaches a DDS late joiner from a transient-local writer, a volatile
  one reaches a DDS reader, a DDS topic reaches a bus reader, a DDS client gets a bus server's answer, and a bus client
  is refused while there is no DDS server, then gets its answer once there is one. Then the standalone runner with a
  battery module, and with ws_bridge's bus backend (E3.1) against a recorder and a manual guard on DDS: the recorder's
  latched status reaches a WebSocket client that came later, and a second one from ws_bridge's cache; the joystick
  reaches the guard; a call reaches the recorder's server, and one with no server is unavailable after 5 s; ws_bridge's
  own r2r node has no endpoint.
- `tests/standalone.rs` (domain 78, its own `main`): a battery module's binary as the roll-back launches it, with
  `-r __node:=battery_state` in argv. Its node is `/battery_state`, the bridge's `/mower_bus_bridge/battery_state`, and
  telemetry from DDS comes back out as `/battery_state`.
- `tests/resolver.rs` (domain 79, its own `main`): the resolver oracle. In one context with robot.launch.py's rules,
  rcl names every endpoint of every instance as the manifest and `Remaps::resolve` do (270 names, the five action
  ends among them: an action client's goal requests show as `rq<name>/_action/send_goalRequest`).
