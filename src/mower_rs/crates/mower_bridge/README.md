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
- **Today:** every flag is false. `mower_rsd` starts no bridge, and the process is what it was before E2.

## The manifest

`MODULES` holds the 13 typed modules of `mower_rsd` (18 instances); `OUTSIDE` holds the nodes they depend on.

- **The module code is the source of truth.** BUS_TOPICS.md sections 2 and 3 cite the lines.
- **One entry per endpoint:** what it does (`publishes`, `subscribes`, `serves`, `calls`), its name, its r2r type and,
  for a topic, the QoS profile the code hands r2r, written as BUS_TOPICS.md writes it (`R V 10`, `BE V 1`, `R TL 1`,
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
  - `silent`: a writer that is made and never publishes: coverage's `/free_space_inflated` and `/risk_map_inflated`
    (BUS_TOPICS.md 5.8 #2), mower_base's `/tf` (`enable_odom_tf` is false).
  - `only`: not in production. nav's `nav_core` backend (robot.launch.py `rust_nav_core:=true`, CI's
    `nav_core_check.py`), with `/tf` and `/tf_static` for `nav_core.pose_source: tf`.
- **Outside** (BUS_TOPICS.md 5.3, "DDS→bus"), each with the step that removes it: ws_bridge's writers (E3), twist_mux's
  `/cmd_vel_guard_input` (E4), robot_state_publisher's `/tf_static` and `/tf` (E4), nav2's `/rosout` and
  `/bt_navigator/get_state` (E9), the two teleop nodes (off by default) and the recorder's `/mower_base/rear_light`
  (E5). Their QoS is upstream behaviour, unconfirmed (BUS_TOPICS.md 1.3).
- **Not listed:**
  - parameter services and actions, which stay on r2r in E2 (decision 2);
  - endpoints only a non-production setting makes, but for nav's `nav_core` ones: robot_status's own heartbeat
    subscription (`heartbeat_source_topic` ≠ `odom_topic`), the final guard's command clock
    (`require_command_session`). The check reports them if they appear.
- **Checked against** (`src/tests.rs`, `tests/resolver.rs`):
  - mower_rsd's module table and node names;
  - robot.launch.py's `_DAEMON_REMAPS` (`PRODUCTION_REMAPS`, in argv order);
  - the parameter files robot.launch.py passes, in its order (mower_rsd.yaml, gps.yaml,
    dual_ekf_navsat_params.yaml): every name a parameter sets has the file's value;
  - rcl itself: the resolver oracle;
  - one type per name, one server per service.

## The flags

`src/on_bus.rs` has one flag per module. Moving a module is flipping its flag, in the commit that merges its
conversion. A flag says "this module's code uses the bus"; the module itself does not read it.

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
- **Deterministic:** topics first, then services, each sorted by name; the order of `--modules` does not matter.
- **What it derives for the steps** (`src/tests.rs` has every line):

| Bus | to_ros | from_ros | service_to_ros | service_from_ros |
|---|---|---|---|---|
| nothing | 0 | 0 | 0 | 0 |
| battery, status | 6 | 7 | 3 | 0 |
| + map, record, adapter | 39 | 11 | 28 | 2 |
| + imu, gps, localize | 48 | 9 | 32 | 1 |

- **Refused:** of all 8192 combinations of flags with the production set, only those that put mower_base on the bus
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
  - No `main.rs` uses it yet: `tests/dds.rs` runs a battery module with `run_standalone`, and `tests/standalone.rs` runs
    `module_main` itself, under launch's `-r __node:=battery_state`.

## The check

The policy plans from the manifest, so an endpoint the manifest lacks is a name nothing bridges, and nothing else would
say so.

- **What:** every endpoint on the bus (mower_bus introspection: name, type, each endpoint's QoS), by scope. An
  endpoint of a running instance must be in that instance's manifest entry with the same type and QoS (`bus_qos` of the
  profile).
- **What it reports,** one line each:
  - no such endpoint in the manifest (a new or renamed name; a parameter set to a value other than production's);
  - another type or QoS there;
  - an endpoint the manifest keeps on r2r;
  - a module whose flag is still false: its conversion was merged without the flip, and nothing bridges it;
  - a scope that is no instance of the manifest (ws_bridge in E3, until the manifest learns it).
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
   services, actions, `/rosout` and graph presence (decision 1).
2. Call `assert_in_manifest` in its tests. If an endpoint is new or changed, change the manifest with it.
3. Switch its `main.rs` to `mower_bridge::module_main`.
4. Flip its flag in `src/on_bus.rs`.
5. Update what looks at its DDS endpoints by node. In mower_rsd they now belong to `mower_bus_bridge`; a binary's
   belong to `/mower_bus_bridge/<node>`, which a tool that compares node names only (`nav_core_check.py`) takes for
   the module's. A bridge endpoint that exists all along changes a count (`autotune_telemetry_check.py`'s subscriber
   count while idle, BUS_TOPICS.md 5.5 f).

## Tests

- `cargo test -p mower_bridge --lib`: the manifest against mower_rsd, the launch file and the parameter files; the
  policy for every step, all flag combinations, a subset (shadow_compare.py's `status,battery,adapter,guards`) and
  every module standalone; the check. No DDS.
- `tests/dds.rs` (ROS domain 78, rmw_cyclonedds_cpp): synthetic converted `robot_status` and `flutter_adapter` modules
  and their planned bridge. A latched bus topic reaches a DDS late joiner from a transient-local writer, a volatile
  one reaches a DDS reader, a DDS topic reaches a bus reader, a DDS client gets a bus server's answer, and a bus client
  is refused while there is no DDS server, then gets its answer once there is one. Then the standalone runner with a
  battery module.
- `tests/standalone.rs` (domain 78, its own `main`): a battery module's binary as the roll-back launches it, with
  `-r __node:=battery_state` in argv. Its node is `/battery_state`, the bridge's `/mower_bus_bridge/battery_state`, and
  telemetry from DDS comes back out as `/battery_state`.
- `tests/resolver.rs` (domain 79, its own `main`): the resolver oracle. In one context with robot.launch.py's rules,
  rcl names every endpoint of every instance as the manifest and `Remaps::resolve` do (203 names).
