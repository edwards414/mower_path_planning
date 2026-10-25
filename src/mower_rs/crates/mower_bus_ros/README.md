# mower_bus_ros

The bridge between the in-process bus ([`mower_bus`](../mower_bus/README.md)) and DDS (r2r), for step E2 of
[`docs/ROS_FREE_PLAN.md`](../../../../docs/ROS_FREE_PLAN.md) (Phase E).

- **Why:** E2 moves each `mower_rsd` module's IO from r2r to the bus, one module at a time. Until the last ROS consumer is
  gone, every topic and service that straddles the bus/DDS boundary has to be forwarded. The consumers are nav2 (until
  E9), twist_mux and robot_state_publisher (E4), the Python recorder (E5) and the app's ws_bridge (E3).
  [`docs/BUS_TOPICS.md`](../../../../docs/BUS_TOPICS.md) section 5.3 lists which names, in which direction, until when.
- **What it is:** a library. A module gives a `RosBridge` its own r2r node and its bus `Scope`, declares the names, and
  runs it until shutdown. Nothing uses it before E2, so robot behaviour does not change.
- **Rules:** they come from BUS_TOPICS.md 5.5, "the traps a bridge can create".

```rust
// a node of its own ("The node" below)
let node = r2r::Node::create(ctx, &format!("{}_bus_bridge", m.node_name), &m.namespace)?;
let mut bridge = RosBridge::new(node, m.bus.clone());
bridge.to_ros::<Odometry>("/odom", QosProfile::system_default())?;                      // bus writers -> DDS
bridge.from_ros::<TwistStamped>("/app_joy_cmd", QosProfile::default().keep_last(10))?;  // DDS writers -> bus
bridge.service_to_ros::<Trigger::Service>("/save_map")?;                                // DDS clients -> bus server
bridge.service_from_ros::<GetState::Service>("/bt_navigator/get_state")?;               // bus clients -> DDS server
let stats = bridge.stats();                                                             // counters, readable while it runs
bridge.run(m.shutdown.wait()).await?;
```

- **Types:** the same r2r message type on both sides (`BridgeMessage`: r2r's types are bus messages as they are), so
  nothing is converted and no conversion can fail.
- **Names:** absolute, as on the bus.
- **The node** is the bridge's alone, and needs a name of its own. A module that keeps an r2r node calls the bridge's
  `<node>_bus_bridge`; only a module whose one node is the bridge's may give it the module's name.
  - Two nodes with one name in one process must not happen: rcl applies remap rules and parameters addressed to a node
    name (`-r <node>:<from>:=<to>`) to both of them.
  - The graph (`/rosout`, graph_snapshot's node list) would also show one node twice.
- **Declarations** create both endpoints at once, so a relative name, a type that differs from the bus's, a service that
  is served already, a QoS without a bus equivalent or a refused direction fails at the call. The one exception is
  `to_ros`'s DDS writer, which exists only while a bus writer does (below).

## QoS (5.5 a, c)

- **The caller passes the source's r2r `QosProfile`:**
  - for `to_ros`, what the bus writer used as an r2r publisher;
  - for `from_ros`, what the DDS writer uses.
- **Both ends follow it:**
  - the DDS endpoint uses the profile itself;
  - the bus endpoint uses `bus_qos(&profile)`, the one conversion, tested in `src/qos.rs`.
- **The conversion is rmw_cyclonedds's reading, the robot's RMW:**
  - `system_default()` is `Qos::system_default()`, R V 1;
  - unknown policies resolve like the system default;
  - a depth of 0 keeps 1.
  - Under rmw_fastrtps a system-default writer would be transient local instead (`mower_rs_common/src/throttle.rs`).
- **Transient local stays transient local on both sides.** A volatile writer for `/map_grid` would starve nav2's
  StaticLayer, and one for a latched app topic would make ws_bridge pick a volatile reader.
- **Refused, because the bus would differ silently:** KEEP_ALL, a depth above `i32::MAX`, and the best-available
  policies. No module uses them.
- **Left to the DDS endpoint:** deadline, lifespan and liveliness, which the bus does not have.
- **The `to_ros` bus reader** takes the source's reliability and durability, but queues at least 64 messages:
  - it stands in for every DDS reader;
  - with the source's depth of 1, two writers publishing at the same moment (`/mower_base/led_command` has two), or a
    module publishing twice in a row, would reach DDS as one message;
  - what DDS readers see is the DDS writer's history, which keeps the source's depth.

## Topics

- **Order across names:** each name has its own forwarder task. Messages published back to back on two names
  (`/odom`, then `/tf`) can therefore reach DDS in either order, where a module that published through r2r itself
  usually kept it. The order within one name is kept.

### `to_ros`: writer lifetime follows the bus writers (5.5 b)

The forwarder watches the bus topic's live writers (`publisher_changes`).

- **The stream coalesces and is not ordered with the message queue.** Every item is a "resync": the forwarder looks at
  the bus as it is now, not at what the item says.
- **The DDS writer exists while at least one bus writer is live.**
  - When the count drops to 0 it is destroyed, so a DDS late joiner cannot get the latched value of a module that has
    died. Today that value vanishes with the module's node.
  - It is created again when a writer comes, or at the latest for the next bus message.
  - A message whose writer is gone by the time it comes up is not published (`stale` in the stats).
- **Transient local:**
  - At a resync the bus reader is replaced by a fresh one, which starts with exactly the retained history of the writers
    live now. The old one may still hold messages of writers that are gone.
  - The messages still queued are given to the DDS writer. If it would then hold a value that history no longer has, or
    lack one it has, it is destroyed and created again with the history. Which values match is decided by the
    message's `Arc` identity, not by its value.
  - A held value that a queued one supersedes is no reason. An unrelated writer change while a message is queued (a
    ws_bridge session's JSON writer coming) keeps the DDS writer, which nav2's StaticLayer would otherwise see leave
    and come back.
  - A module restart that the stream reports as one item with the count unchanged still drops the old instance's
    value: the DDS writer is replaced, unless the new instance's first value is queued already and supersedes it.
  - Each fresh reader lives in a child scope of the bridge's, closed when the reader is replaced. A child takes no close
    hook in the bridge's scope, so resyncs leave nothing behind there, and it closes with the bridge's scope.
- **Volatile:** a resync forwards what is queued first (it was published before the change), then creates or destroys
  the DDS writer.
- **Several latched writers on one name** (5.5 e: `/mower_base/led_command`, `/tf_static`, ...):
  - one DDS writer carries them all, so a DDS late joiner gets the newest `depth` messages across the bus writers, as a
    bus late joiner does;
  - DDS used to give it each writer's own history;
  - when one writer goes, the DDS history falls back to what the others retain.

### `to_ros`: volatile topics skip work nobody wants

- **A volatile message is published only while `get_inter_process_subscription_count() > 0`** (`skipped` in the stats).
  - The count includes readers of other nodes in the same process. Production is one participant (the test
    `a_writer_counts_the_readers_of_other_nodes_in_the_process...`).
  - r2r's doc comment says otherwise; rcl counts every matched DDS reader.
- **Transient-local messages are never skipped:** the DDS writer's history must hold them for late joiners.

### `from_ros`

- **The DDS reader** takes the source's profile.
- **The bus writer** takes its `bus_qos`, so a transient-local DDS value is retained for bus late joiners. That includes
  the DDS writer's replay to the bridge's own reader when the bridge starts after the DDS writer published.

## One direction per name (5.5 d)

- **Refused with `BridgeError::AlreadyBridged`:**
  - `to_ros` and `from_ros` on the same name. The bridge's DDS reader would take back its own samples: r2r drops
    `rmw_message_info`, so publisher GIDs cannot filter them.
  - Declaring a name twice.
  - The same for services. Topics and services are separate namespaces, as on DDS and on the bus.
- **The registry is process-wide, not per bridge.** The echo travels through DDS, and `mower_rsd` is one participant.
  - A second bridge reading what the first writes would echo too.
  - Two bridges writing one name would put each bus message on DDS twice.
  - Names are given back when `run` has dropped the node, or when an unrun bridge is dropped.

### The `/tf_static` question (needs a decision in E2/E4)

`/tf_static` is the one name that crosses both ways today:

- robot_state_publisher's `imu_link` and `gps_link` frames come in (DDS to bus);
- navsat's `utm` frame goes out (bus to DDS).

A bridge cannot do both on one name. Options:

1. **Keep navsat's `utm` publisher on r2r until E4.** `/tf_static` is then DDS only, and bus readers get it through
   `from_ros`. It is the smallest change for E2.
2. **Do E4's static frames first.** One bus module publishes rsp's two frames from configuration (PLAN E4) together with
   `utm`, and `/tf_static` becomes `to_ros` only. This is the end state.
3. **Filter by writer.** Patch vendor/r2r so that subscriptions also return `rmw_message_info_t`, which r2r drops ("we
   dont care for now", `subscribers.rs`). The bridge would then drop samples whose publisher GID is one of its own DDS
   writers. The GIDs are known from `get_publishers_info_by_topic`. This is general, but it is a second patch to keep
   in vendor/r2r.
4. **Filter by content,** e.g. by `child_frame_id`. Specific to tf, and it breaks for a frame both sides publish.

`ignore_local_publications` does not work: it hides the whole participant, so it would hide modules still on r2r.

Whichever is chosen, the multi-writer rule above applies. tf2's static broadcaster sends all of a writer's frames as one
latched message of depth 1. With two writers behind one bridge writer (option 2), or two DDS writers behind one bridge
reader and bus writer (option 1), a depth-1 late joiner gets only the newer writer's frames. Use a depth of at least the
number of writers on the bridge's side, or let one writer own all static frames.

## Graph presence (5.5 f)

- **A `from_ros` bus writer exists from the declaration until `run` returns,** whether or not DDS has a writer. So it
  counts in `Subscriber::publisher_count()` all along. This is why mower_base, the final guard and pid_autotune move
  together in E2: mower_base's arm latch asks whether `/drivetrain_guarded_cmd_vel` and `/mower_base/wheel_override`
  have a publisher.
- **Likewise for services:**
  - a `service_to_ros` DDS server is advertised all along, so a DDS client sees it as available. While no bus server
    exists, its calls wait up to 4 s for one, then go unanswered ("Services");
  - a `service_from_ros` bus server makes `wait_for_service` succeed at once.
- **Readers too:** a `to_ros` bus reader counts in `Publisher::subscription_count()` all along. No module skips work on
  that count today.

### Proposal: mirror DDS writer presence

The plan assumed that r2r cannot count a topic's writers. It can:

- `Node::get_publishers_info_by_topic` returns them, with node name, GID and QoS;
- ws_bridge's `matching_qos` already uses it;
- the test above shows that it sees writers of other nodes in the process, and loses them with their node.

What r2r lacks is a graph-change event, since rcl's graph guard condition is not exposed. So mirroring means polling, on
the spin thread, which owns the node:

- **For each `from_ros` name that opts in,** query the graph every 100-250 ms. Hold the bus writer while the count is
  above 0, and drop it when it reaches 0. The bus writer must also be created for a sample that comes first: the graph
  learns of a writer after the match.
- **Cost:** a lookup in rmw's graph cache per name per period, no DDS traffic.
- **Benefits:** the arm latch would see the truth across the bridge. A dead DDS writer's latched value would also leave
  the bus, the `from_ros` side of 5.5 b. Today the bridge's bus writer retains it until `run` ends.
- **Limits:** presence lags by up to one period, and services cannot follow, because r2r has no `destroy_service`.
- It needs no `unsafe`.

## Services

`ServiceTimeouts` bounds the calls. Set it with `RosBridge::with_service_timeouts` before declaring the services.

- **`service_to_ros`:** each DDS request becomes a bus call, and the answer goes back.
  - **No bus server (`ServiceUnavailable`):** the call waits up to `bus_server` (4 s) for one, then is tried once more.
    mower_rsd starts a failed module again after 2 s, so a call made during that restart is answered by the restarted
    module.
  - A server that went away holding the call gives the same `ServiceUnavailable`, so that call is tried again too. If
    its handler had already run, it runs twice.
  - **`NoResponse` or `Closed`, or no server by the end of the wait:** the DDS request is dropped unanswered, since ROS
    has no error channel. The ROS client times out, as with a crashed server.
  - Each error kind is logged once, until a call succeeds again.
  - **Proposal: withdraw the DDS service while no bus server exists,** so that a DDS client's `is_available` fails as it
    does for a dead server. ws_bridge would then tell the app "unavailable" within 5 s instead of after its 30 s.
    - r2r has no `destroy_service`. It destroys a service whose request stream was dropped only when the next request
      comes, and that request is lost.
    - So it needs a vendor/r2r patch that adds `Node::destroy_service` next to `destroy_publisher`.
    - With the wait above, only a module that stays down longer than 4 s would still need it.
- **`service_from_ros`:** each bus request becomes a DDS call.
  - r2r tells whether the DDS server is matched only through `Node::is_available`, a future that the node's next spin
    completes.
  - While no server is matched within 500 ms, the bus request is dropped, so the bus caller gets `NoResponse`.
  - One probe is kept, and reused while pending. New ones would pile up in r2r's client for as long as the server is
    missing.
  - A confirmed server counts as available for 1 s.
  - A pending call re-checks every second that a server is still matched, and is dropped (`NoResponse`) once none is.
    Otherwise r2r's response future would never complete.
  - **Deadline:** every call is given up at `dds_call` (30 s, ws_bridge's `service_timeout`), and the bus caller gets
    `NoResponse`. That is what ends a call whose server is replaced while holding it (a crash and a respawn, or a
    second instance that comes up first): a server is matched at every check then, just not the one that has the call.
- **Concurrency:** calls are forwarded concurrently. The servers still decide how they serve them.

## Lifecycle

- **`run(shutdown)` returns when `shutdown` completes or the bridge's scope closes** (`Scope::closed()`).
- **Before returning, in this order:**
  1. it aborts and awaits the forwarder tasks, and their bus endpoints go with them;
  2. it stops the spin thread, which drops the node after its last `spin_once`, so no thread is left in `rcl_wait`
     (`mower_rs_common/src/module.rs`);
  3. it gives the names back.
- **The spin thread:**
  - the node lives on its own thread, as in every module (`spawn_blocking`, `spin_once(20 ms)`);
  - creating and destroying the `to_ros` DDS writers runs there, between two spins, as commands from the forwarders;
  - publishing, requests, responses and probes run in the forwarders. r2r allows that from any thread.
- **Forwarders:** `run` aborts them all before it returns, which is what ends some of them.
  - A `to_ros` and a `service_from_ros` forwarder end on their own when the scope closes.
  - A `from_ros` forwarder would notice only at its next DDS sample.
  - A `service_to_ros` forwarder never would, since its DDS requests keep coming.
- **A panic** in the spin thread (r2r's `spin_once` unwraps its own locks) or in a forwarder (a bug: there are no user
  conversions) makes `run` stop everything and return `BridgeError::Panicked` at once, so that the module fails and is
  restarted. Without the spin thread nothing would cross any more. Await `run` to the end: dropped half-way, it still
  stops the spin thread, but does not wait for it.

## Stats

`bridge.stats()` is a handle that is cheap to clone, readable while the bridge runs. For each name it holds:

- `forwarded`;
- `skipped` (volatile, no DDS reader);
- `stale` (no live bus writer);
- `resyncs`;
- `unanswered` (service calls left without an answer);
- `dropped` (`to_ros` bus messages lost to the bridge's full reader queue: a forwarder stalled for more than 64);
- `dds_writer` (whether the `to_ros` DDS writer exists).

It is for diagnostics (E8's `mowerctl`) and for the tests, which cannot see a skipped publish from DDS.

## JSON parity (E3 prep)

The test `r2r_untyped_json_equals_the_bus_s_erased_json` sends each message bus -> bridge -> DDS. It compares:

- r2r's `subscribe_untyped` JSON, which is what ws_bridge sends the app today;
- the bus's `ErasedMsg::to_json`, which is `serde_json::to_value` of the typed message, and what E3 will send.

The types:

- `std_msgs/Header`;
- `sensor_msgs/NavSatFix` (negative `i8` status, `u16` bit set, NaN altitude);
- a 4 × 3 `nav_msgs/OccupancyGrid` (`f32` resolution, `-1` cells);
- `mower_interface/CoverageProgress` (`f32` fields, `u8`, non-ASCII and escaped strings).

Findings:

- **No field differs,** as values or as text. r2r's untyped path is `serde_json::to_value(&T::from_native(..))`, the same
  derives. The DDS graph names each type the way `mower_bus::ros_type_name` does.
- **The app's bytes are those of a `serde_json::Value`.** Serializing the typed message straight to text is not the
  same:
  - it keeps the declaration order of the fields, where a `Value` sorts them;
  - it prints an `f32` as an `f32` (`0.1`), where a `Value` holds the `f64` it widens to (`0.10000000149011612`).
  - So E3 keeps byte identity only if it builds the text from the `Value`, as ws_bridge does today (the test
    `the_app_s_bytes_are_those_of_a_json_value...`).
- **NaN becomes `null` on both paths,** so the app cannot tell an unknown altitude from a missing one.

## Tests

`tests/dds.rs` runs everything against real DDS in one process. The bridge's node and a peer node stand in for nav2, the
app and the rest, on one participant, as in production.

- **Isolation:** `ROS_DOMAIN_ID=77` and `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` are set before the process's context
  exists, and every test uses names of its own.
- **CI:** CI's `ros:jazzy-ros-base` image ships only Fast DDS, so the `mower-rs-tests` job installs
  `ros-jazzy-rmw-cyclonedds-cpp`.
- **Waiting:** discovery is polled for, never slept for. An absence is asserted only after the bridge's stats say what it
  did, or through an order that a stale value would break: a depth-2 late joiner would get a stale value before the
  fresh one.
