# mower_bus_ros

The bridge between the in-process bus ([`mower_bus`](../mower_bus/README.md)) and DDS (r2r), for step E2 of
[`docs/ROS_FREE_PLAN.md`](../../../../docs/ROS_FREE_PLAN.md) (Phase E).

- **Why:** E2 moves each `mower_rsd` module's IO from r2r to the bus, one module at a time. Until the last ROS consumer is
  gone, every topic, service and action that straddles the bus/DDS boundary has to be forwarded. The consumers are nav2 (until
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
bridge.action_to_ros::<Waypoint::Action>("/nav_action_follow_path")?;                   // DDS action clients -> bus server
bridge.action_from_ros::<Waypoint::Action>("/nav_action_follow_path")?;                 // bus action clients -> DDS server
let stats = bridge.stats();                                                             // counters, readable while it runs
bridge.run(m.shutdown.wait()).await?;
```

- **Types:** the same r2r message type on both sides (`BridgeMessage`: r2r's types are bus messages as they are), so
  nothing is converted between types. r2r still copies each value into its C message, which can refuse one ("Values
  r2r cannot convert"). An r2r action type (`BridgeAction`) is the bus action `(Goal, Result, Feedback)` of its three
  message types.
- **Names:** absolute, as on the bus.
- **The node** is the bridge's alone, and needs a name of its own. A module that keeps an r2r node calls the bridge's
  `<node>_bus_bridge`; only a module whose one node is the bridge's may give it the module's name.
  - Two nodes with one name in one process must not happen: rcl applies remap rules and parameters addressed to a node
    name (`-r <node>:<from>:=<to>`) to both of them.
  - The graph (`/rosout`, graph_snapshot's node list) would also show one node twice.
- **Declarations** check both sides at once, so a relative name, a type that differs from the bus's, a service that is
  served already, a QoS without a bus equivalent or a refused direction fails at the call. The endpoints that mirror the
  other side's presence come once the bridge runs ("Graph presence"): `to_ros`'s DDS writer, `from_ros`'s DDS reader,
  a transient-local `from_ros`'s bus writer, `service_to_ros`'s DDS server and `service_from_ros`'s bus server.

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

- **The DDS reader** takes the source's profile, and exists only while the bus has a reader of the name ("`from_ros`:
  presence").
- **The bus writer** takes its `bus_qos`, so a transient-local DDS value is retained for bus late joiners. That includes
  the DDS writer's replay to the bridge's DDS reader when that comes after the DDS writer published.
- **Transient local:** the bus writer, and the value it retains, exist only while DDS has a writer and the DDS reader
  hears it ("`from_ros`: presence").

## One direction per name (5.5 d)

- **Refused with `BridgeError::AlreadyBridged`:**
  - `to_ros` and `from_ros` on the same name. The bridge's DDS reader would take back its own samples: r2r drops
    `rmw_message_info`, so publisher GIDs cannot filter them.
  - Declaring a name twice.
  - The same for services and for actions. Topics, services and actions are separate namespaces, as on DDS (an action
    is `<name>/_action/...`) and on the bus.
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

An endpoint the bridge holds for the other side exists while the other side has its counterpart, as a module's own r2r
endpoints lived and died with its node. Nobody outside sees a server or a latched value that nothing stands behind.

- **`to_ros`:** the DDS writer exists while a bus writer does ("Topics").
- **`from_ros`:** the DDS reader exists while the bus has a reader of the name ("`from_ros`: presence", below), as a
  module's own r2r reader existed while it subscribed. The plan includes readers a module makes only in some
  configurations (nav's `/rosout` reader, its nav_core inputs); while the module does not read the name, its DDS
  writers have no reader of the bridge to send to.
- **`from_ros`, transient local:** the bus writer exists while the graph shows a DDS writer and the DDS reader exists.
  A dead writer's latched value (coverage's markers, nav's `/nav_operation_active`) leaves the bus with it, as it left
  the module's own r2r reader, and so does what the bridge heard before the last bus reader went.
- **`from_ros`, volatile:** the bus writer exists from the declaration until `run` returns, whether or not DDS has a
  writer. It keeps no history, but it counts in `Subscriber::publisher_count()` all along. This is why mower_base, the
  final guard and pid_autotune move together in E2: mower_base's arm latch asks whether `/drivetrain_guarded_cmd_vel`
  and `/mower_base/wheel_override` have a publisher.
- **`from_ros_mirrored`, volatile:** for a bus reader that does count the writers of its name. The bus writer follows
  the DDS writers as a transient-local one does ("`from_ros`: presence"): it exists while the DDS reader does and the
  graph shows a DDS writer (or a sample proves one), looked at every 200 ms, and goes with the last of them, so a
  bus reader's `publisher_count` is what the graph shows. mower_bridge declares it for mower_base's two command topics
  where their writer is not on the bus (the base's binary alone, the rclpy guards); with the base, the final guard
  and pid_autotune in one mower_rsd those names do not cross at all. On a transient-local name it is `from_ros`.
- **Services:**
  - a `service_to_ros` DDS server exists only while the bus has a server for the name ("Services"). While the module
    is down or in a crash loop, a DDS client's `is_available` waits, as for the module's own r2r server: ws_bridge's
    5 s check tells the app "service unavailable", where an advertised server would have taken the call and let it run
    into the 30 s `service_timeout`;
  - a `service_from_ros` bus server exists only while the graph shows a DDS server, so a bus caller finds what an r2r
    caller found. The forwarder looks every 200 ms, on the spin thread (`get_publishers_info_by_topic` of the
    server's `rr<name>Reply` writer: the bridge never serves the name on DDS itself, one direction per name).
    - No DDS server: no bus server. A call is refused at once with `ServiceUnavailable`, `wait_for_service` waits and
      `is_available` is false, as r2r's `is_available` never completes. mower_rs_common's guard then says "Mission
      operation guard is unavailable" and the caller retries, where a bus server that answered nothing would have
      looked like a lock that may have been granted.
    - A DDS server comes: within 200 ms of the bridge's client matching it the bus server does, and `wait_for_service`
      returns. The graph can show the server before the match, and a call then would come back `NoResponse`, which
      the guard takes for a lock that may have been granted: so the bus server waits for a probe (up to 500 ms per
      look) to find the server matched, as an r2r caller's `is_available` did.
    - It goes: within 200 ms the bus server goes too, and the calls it still holds end with `ServiceUnavailable`.
    - The name, the type and a bus server already there are still checked at the declaration.
- **And for actions,** as for services:
  - an `action_to_ros` DDS action server exists only while the bus has a server for the name ("`action_to_ros`").
    While nav is down a DDS client's `wait_for_server` waits and nothing of the server is on the graph, as for nav's
    own r2r server; a goal sent then is lost, as it was;
  - an `action_from_ros` bus action server exists only while the graph shows a DDS server that the bridge's client is
    matched with, looking at the server's `rr<name>/_action/send_goalReply` writer every 200 ms:
    - no DDS server, no bus server: `send_goal_request` fails at once with `ServiceUnavailable`, `wait_for_server`
      waits and `is_available` is false, as r2r's `is_available` never completes. mower_coverage then says "Navigation
      action server unavailable" after its 2 s wait, as it did against a missing server, instead of sending a goal that
      would end in `NoResponse` and its correlated-cancel tracking;
    - a DDS server comes: once a probe finds the bridge's client matched with all of it (`rcl_action_server_is_available`:
      its three services and two topics, up to 500 ms per look), the bus server does, and `wait_for_server` returns.
      The graph shows the goal reply writer before that, and a goal sent then would come back `NoResponse`;
    - it goes: within 200 ms the bus server goes too. The goal requests it queued fail with `ServiceUnavailable`; the
      goals it handed on end with their DDS server, which is watched (a lost goal, `NoResponse`).
- **Readers too:** a `to_ros` bus reader counts in `Publisher::subscription_count()` all along. No module skips work on
  that count today.

### `from_ros`: presence

**The DDS reader follows the bus readers.** The forwarder watches them (mower_bus's `Publisher::subscription_changes`,
the mirror of `publisher_changes`, from its bus writer; transient local, from one it drops at the declaration, as the
bus writer comes later) and makes the DDS reader for the first, on the spin thread, and destroys it after the last
(vendor/r2r's `Node::destroy_subscription`).

- At most one change per 200 ms, as for `service_to_ros`'s DDS server: a bus reader that comes and goes faster cannot
  keep the spin thread busy. The DDS reader can therefore stay up to 200 ms after the last bus reader went, which only
  means its writers send it a little longer.
- mower_rsd starts the bridge first, and its modules subscribe as they start: the DDS reader comes once the first one
  has, and a transient-local DDS writer replays its history to it, which reaches that module, as its own r2r reader's
  match did.
- Destroying it drops what r2r had queued: no bus reader is left to get it.

**Transient local, the bus writer follows the DDS writers** (the `from_ros` side of 5.5 b; in production
`/nav_operation_active`, `/mower_base/firmware_info`, coverage's three marker layers, `/tf_static`). r2r counts a topic's
writers (`Node::get_publishers_info_by_topic`, with node name, GID and QoS; the test
`a_writer_counts_the_readers_of_other_nodes...` shows that it sees writers of other nodes in the process and loses them
with their node), but has no graph-change event: rcl's graph guard condition is not exposed. So while the DDS reader
exists, the forwarder looks every 200 ms, on the spin thread:

- **A writer shown:** the bus writer exists. **None:** it is dropped, and the history it retained goes with it. A
  restarted adapter then gets coverage's empty layer from the bus until coverage publishes again, as it did on r2r,
  and not the dead instance's path.
- **The DDS reader goes, the bus writer too.** Otherwise a value the bridge heard before the last bus reader went would
  reach the next one first, before what the DDS writer published meanwhile, which the bridge did not hear; a new r2r
  reader got only the latter. The next DDS reader gets the DDS writers' histories again, by their replay.
- **The bridge's own node never counts.** One direction per name keeps it from writing a name it reads (the only
  name a plan could need both ways, `/mower_base/led_command`, is refused: mower_bridge README, "The policy"), but a
  filter by writer (the `/tf_static` question, option 3) would not.
- **A sample is proof of a writer:** the graph learns of a writer after the match, and its history's replay can come
  first. A sample makes the bus writer if there is none, and keeps it through one look that shows no writer.
- **Samples r2r has queued when the count reaches 0** were published before the writer went: they are handed to the
  bus first, so live bus readers get them as the module's own reader did, and the bus writer goes at the next look.
- **Cost:** a lookup in rmw's graph cache per name per period, no DDS traffic.
- **Limits:**
  - presence lags by up to two looks (a writer that published just before it went);
  - only the count reaching 0 clears the history. A writer that goes while others stay (`/tf_static`'s navsat
    restarting, rsp's frames staying), or one replaced between two looks, leaves its samples in the bus history until
    newer ones push them out. DDS gave a late joiner each live writer's history instead. Doing the same would take a
    fresh DDS reader and bus writer at each such change, for the writers' replay, with a gap in between (not done).
    For `/tf_static` the frames are the same after a restart.
  - Volatile names keep their bus writer, unless mirrored (`from_ros_mirrored`): it has no history to go stale.

## Services

`ServiceTimeouts` bounds the calls. Set it with `RosBridge::with_service_timeouts` before declaring the services.

- **`service_to_ros`:** each DDS request becomes a bus call, and the answer goes back.
  - **The DDS server follows the bus server** ("Graph presence"). The forwarder waits on the bus for a server
    (`Client::wait_for_service`) and for none (`wait_for_no_service`), with no polling, and makes or destroys the DDS
    server on the spin thread (vendor/r2r's `Node::destroy_service`).
    - It leaves the graph within one spin of the bus server going, as the module's own server left with its node.
    - **At most one change per 200 ms:** a bus server that comes and goes faster (a module that keeps dropping its
      `Service`) cannot keep the spin thread making and destroying DDS servers. A failed module is started again after
      2 s, so a crash loop flaps the DDS server once per restart, as its r2r node did, and every destroyed server is
      finalized at once: none is left in the node, its wait set or the graph.
    - The DDS server can therefore lag the bus by up to 200 ms; a call to it in that window is lost with it, as one to
      a module's own r2r server that it destroys and makes again.
  - **No bus server when a call in hand is made** (it came as the server went, before the DDS server did): the bus
    refuses it at once with `ServiceUnavailable`, so nothing has run for it yet. It waits up to `bus_server` (4 s) for
    a server, then is made once more, but only while its DDS server still exists. Once that goes, nothing could answer
    the call: it is dropped with the requests r2r has queued, and never made to the server that comes next, which
    would run a dispatch or a lock for a client that gets no answer.
  - **A call a server has taken is never made again,** however it ends. This covers a server whose scope closes while
    holding the call (`ServiceUnavailable` from the response) and one that drops it (`NoResponse`).
    - Its handler may have run, and dispatch, start-coverage and the lock services are not idempotent.
    - The DDS request is lost, as a direct r2r server's crash loses it; a direct r2r server never runs a request twice.
  - **Dropped unanswered:** a call a server lost, `Closed`, no server by the end of the wait or before the DDS server
    went, and an answer r2r cannot convert ("Values r2r cannot convert"). ROS has no error channel, so the ROS client
    times out, as with a crashed server.
  - Each error kind is logged once, until a call succeeds again.
- **`service_from_ros`:** each bus request becomes a DDS call. The bus server exists only while the graph shows a DDS
  server that the bridge's client is matched with ("Graph presence").
  - r2r tells whether the DDS server is matched only through `Node::is_available`, a future that the node's next spin
    completes. The graph can show a server the bridge's client has not matched yet.
  - While no server is matched within 500 ms, the bus request is dropped, so the bus caller gets `NoResponse`. Since
    the bus server comes only once a probe has found the server matched, a call meets this only when the server has
    just gone, before the next look takes the bus server away, or was replaced.
  - One probe is kept, and reused while pending. New ones would pile up in r2r's client for as long as the server is
    missing.
  - A confirmed server counts as available for 1 s, until the graph shows none: a server that comes back is probed
    afresh.
  - A pending call re-checks every second that a server is still matched, and is dropped (`NoResponse`) once none is.
    Otherwise r2r's response future would never complete.
  - **Deadline:** every call is given up at `dds_call` (30 s, ws_bridge's `service_timeout`), and the bus caller gets
    `NoResponse`. That is what ends a call whose server is replaced while holding it (a crash and a respawn, or a
    second instance that comes up first): a server is matched at every check then, just not the one that has the call.
- **Concurrency:** calls are forwarded concurrently. The servers still decide how they serve them.

## Actions

`action_to_ros` serves a bus action on DDS, `action_from_ros` a DDS action on the bus. Both carry everything of a goal
across: the answer to its request, its feedback, its cancel requests and its result. Each goal is forwarded in a future of
its own, and each of its cancel requests too, so goals run concurrently and a server that takes its time over one holds
up none of the others. The bus side is described in the mower_bus README, "Actions".

**Who needs them** (docs/BUS_TOPICS.md 4 and step 5):

- In `mower_rsd`, coverage and nav both move to the bus in step 5, so `/nav_action_follow_path` needs no bridge there.
- `action_to_ros::<Waypoint::Action>` on `/nav_action_follow_path` (and `/nav_action`): `nav_progress_check.py` and
  `nav_core_check.py` drive a standalone `mower_nav` with rclpy clients.
- `action_from_ros::<Waypoint::Action>` on `/nav_action_follow_path`: `coverage_cancel_check.py` runs a standalone
  `mower_coverage` against an rclpy server.
- nav2's `/navigate_to_pose` and `/follow_path` stay r2r clients of mower_nav until E9 ("nav2's actions", below).

### `action_to_ros`

- **The DDS action server follows the bus server** ("Graph presence"), as `service_to_ros`'s DDS server does. The
  forwarder waits on the bus for a server (`ActionClient::wait_for_server`) and for none (`wait_for_no_server`), with no
  polling, and makes or destroys the DDS server on the spin thread (vendor/r2r's `Node::destroy_action_server`).
  - Its three services and two topics leave the graph within one spin of the bus server going, as nav's own server
    left with its node, and at most one change per 200 ms, as for services.
  - **The goals the bridge carries keep it.** A bus server that drops its `ActionServer` while its scope stays open
    leaves its goals running (mower_bus README, "Actions"), as an r2r server stayed while its node lived: the DDS server
    goes only once those goals have ended and their results are out. A server whose scope closes (a module that ends:
    mower_rsd closes its scope, and starts it again 2 s later) loses its goals at once, so its DDS server goes at once.
  - **Withdrawn,** it takes along what it held, as a server that dies: goal requests in hand and those r2r had queued
    are lost (counted in `unanswered`), and so are the cancel requests of goals still within their last refusals
    (below). Its goals' DDS clients get no result and no answer to a cancel; once the module is back, its new DDS
    server does not know them (`GoalCancelUnknownGoalID`), as nav's restarted r2r server did not.
- **Goal requests** follow the rule of `service_to_ros`:
  - No bus server when a request in hand is sent (it came as the server went, before the DDS server did): the bus
    refuses it at once and nothing has run for it, so it waits up to `bus_server` (4 s) for a server, then is sent once
    more, but only while its DDS server still exists. A request a bus server has taken is never sent again.
  - Accepted or rejected on the bus: accepted or rejected on DDS (the CI tools check both).
  - Any other end (the bus server ended or dropped the request, no server came in time, the bridge's scope closed): the
    DDS request is left unanswered, as a crashed server leaves it, and counted in `unanswered`.
- **Feedback** reaches DDS in order, and what the bus server published before the end reaches it before the result.
- **Cancel requests** from DDS go to the bus server, and its answer comes back:
  - accepted: accepted;
  - every refusal (`CancelRejected`, `GoalTerminated`, a lost goal): rejected. DDS has no other word for it.
  - rcl splits "cancel all goals" and "cancel goals accepted before a stamp" into one request per goal, so those work
    as well.
- **Every cancel request the bridge takes gets an answer.** r2r sends none at all for a request that the bridge drops
  unanswered, and its client waits forever, so what the bridge gives up it rejects:
  - once the goal has ended, the answers in flight can only be refusals (an acceptance after the end is
    `GoalTerminated` on the bus). The bridge waits up to 1 s for them, then rejects those still pending;
  - a request that comes in that second, or in the second after the result, is rejected at once. Later ones r2r answers
    itself (`GoalCancelAlreadyTerminated`), except for a goal that got no result (below);
  - when `run` ends, the requests in flight are rejected, and `run` spins the node once more after the forwarders end,
    since r2r sends cancel answers only at a spin.
- **The result** reaches DDS with the bus server's status and message, with one exception: once the bridge has accepted a
  cancel request of the goal, the goal ends CANCELED on DDS, whatever the bus server ended it with. r2r forgets a goal
  when it applies an accepted cancel, and from then on delivers a result only through `cancel`. mower_nav's own r2r
  server falls back to `cancel` there too (`terminate`), so the CI tools see what they saw before.
- **A goal the bus lost** (its server's scope closed, or it dropped the goal before ending it) gets no result on DDS,
  as a goal of a crashed server gets none. It is counted in `unanswered`.
  - Lost with its server's scope, it goes with the DDS server (above).
  - Dropped by a server that lives on, it stays: r2r keeps listing it as executing in its status. A cancel of it within
    the second after the loss is rejected; a later one r2r reports accepted without asking anyone, the same r2r bug as
    for any goal it ended through `cancel` without an accepted cancel request. No module drops a goal so.

### `action_from_ros`

- **The bus server** exists only while the graph shows a DDS server that the bridge's client is matched with ("Graph
  presence"). Each lives in a child scope of the bridge's, as `service_from_ros`'s does, so that a server that comes and
  goes leaves no close hook in the bridge's scope (mower_bus README, "Hooks of dropped endpoints"). The goals a bus
  server handed on run on after it goes, and end with their DDS server, so its scope closes only once the server and
  every goal of it are gone; closing it with the server would lose them at once.
- **The look and the probe** run next to the goals, not in front of them: a probe of up to 500 ms holds no goal's
  feedback or cancel up.
- **Goal requests** follow the rules of `service_from_ros`:
  - No DDS server matched within the probe window (500 ms): the bus request is dropped, and the bus client gets
    `NoResponse`. Since the bus server comes only once a probe has found the server matched, a goal meets this only when
    the server has just gone, before the next look takes the bus server away, or was replaced.
  - Accepted or rejected on DDS: accepted or rejected on the bus.
  - The DDS server goes away while the request is pending (checked every second), or its answer does not come by
    `dds_call` (30 s): `NoResponse`. A goal the server accepts after that, within one more `dds_call`, is canceled on DDS,
    since no bus client can see it. That is what mower_coverage does with a goal accepted after its own 3 s, and
    `coverage_cancel_check.py`'s scenario B (accepted after 4 s) relies on: within `dds_call` the bridge waits, so the
    late acceptance reaches coverage, which cancels the goal itself.
  - A goal r2r cannot convert is dropped ("Values r2r cannot convert"): `NoResponse`.
- **Feedback** from DDS goes to the bus goal. DDS orders feedback and the result on separate entities, so feedback that
  comes after the result is lost.
- **Cancel requests** from the bus go to DDS:
  - accepted: accepted;
  - `GoalCancelRejected`, `GoalCancelAlreadyTerminated` (rcl's answer also for a goal that is canceling) and
    `GoalCancelUnknownGoalID`: rejected;
  - no answer by `dds_call`, or the server gone: dropped, so the bus client gets `NoResponse`. r2r's cancel future
    never completes then.
  - An answer that comes after the result still reaches the bus client, by `dds_call`, as it reached a direct r2r
    client. DDS orders the cancel answer and the result on separate entities, so the bridge waits up to 1 s for the
    answers in flight before it ends the bus goal: an acceptance the server sent before its result still reads `Ok`
    then. One that comes later reads `GoalTerminated`, since the bus goal has ended.
- **The result:** SUCCEEDED, ABORTED and CANCELED end the bus goal the same way, with the server's message.
- **A goal is lost on the bus** (dropped, so its client gets `NoResponse`, never a terminal status) when:
  - the DDS server goes away during the goal: r2r's result future never completes then, so it is watched like a pending
    service call;
  - the result comes with status UNKNOWN: the server that answered did not know the goal.
  - Its cancel requests get `NoResponse` too, those in flight at DDS and those still queued alike: nobody refused them.
- **Limit: a DDS server replaced while it holds a goal** (a crash and a respawn within a second, or a second instance)
  stays matched, and r2r's result future never completes. The bus goal then runs until its client gives up on it
  (coverage's leg timeout), as with a direct r2r client. Doing better needs the result asked for again, or the goal
  looked up in the new server's status array; r2r exposes neither.
- The bridge never cancels a DDS goal because the bus side went away (the bridge's scope closed): a direct r2r client
  whose node goes does not either, and rcl_action servers do not watch their clients. That includes a goal the DDS
  server accepts after the scope closed, while the bus client already got `ServiceUnavailable`: it runs on.

### nav2's actions

mower_nav keeps its r2r node, its clients of `/navigate_to_pose` and `/follow_path` and its "uncertain" machinery as
they are until E9 (or until *nav_core* is the default). `action_from_ros` would carry those goals, but not mechanically:

- mower_nav reads any error from a goal response as a definite rejection (`await_goal_response`: `Ok(Err(_))` →
  `resolve_definitive_nav2_rejection`), and only a response that does not come in time as uncertain.
- Through the bridge, `NoResponse` also means "sent, outcome unknown" (the server went away or missed the deadline).
  Ported as is, nav would treat a goal nav2 may be running as rejected.
- So a port must read only `GoalRejected` as definite, and every other error as uncertain, as mower_coverage already
  does for its own client. That includes the late answer nav still awaits after its own response timeout
  (`nav2_goal_response_timeout_s`, well below `dds_call`): a `NoResponse` there, from the bridge's deadline or a server
  gone, must leave the dispatch uncertain, where nav today resolves any late error as a rejection.

## Values r2r cannot convert

The bus and DDS carry the same Rust type, but r2r copies every value into its C message to publish, send or answer it,
and asserts what the C type fixes (r2r_msg_gen's `copy_to_native`): the length of a fixed-size array (a covariance of 35
where `PoseWithCovariance` has 36), the bound of a bounded sequence, and a string without NUL. A value that breaks one
panics inside r2r.

- **Before E2** that was the publishing module's own panic, and mower_rsd restarted that module alone.
- **Through the bridge** it would be a forwarder's, `run` would return `Panicked`, and every bridged name would stop. So
  each conversion is guarded (`src/convert.rs`): the message or call is dropped, counted in `invalid` and logged once
  until the forwarder succeeds again, and the forwarder carries on.
  - `to_ros`: the message does not reach DDS. Transient local, it is also left out of the DDS writer's history: the
    writer is given the bus's retained history again without it, as when a writer goes. A DDS late joiner gets what the
    other writers retain, and not the value the dropped one superseded on the bus.
  - `service_to_ros`: the bus server's answer does not reach DDS, and the DDS call is unanswered.
  - `service_from_ros`: the bus request does not reach DDS, and the bus caller gets `NoResponse`.
  - `action_to_ros`: a feedback message does not reach DDS, and the goal goes on; a result does not, and the DDS goal
    gets none, as a lost goal (`unanswered` too).
  - `action_from_ros`: the bus goal request does not reach DDS, and the bus client gets `NoResponse`.
- **Converted first:** r2r converts a service response, a client request, an action result and an action goal request
  while it holds that service's, client's or action server's lock, so a panic there would poison the lock, and the spin
  thread would panic at its next `spin_once`. Those are converted once on their own before r2r gets them. A publish,
  an action's feedback too, converts with no lock held, and is guarded as it is.
- **The panic hook** is not called for a guarded conversion: a module that publishes a bad value at 50 Hz would write 50
  lines a second to stderr. The first guard wraps the process's hook, which still gets every other panic.
- **Limit:** transient local with a depth above 1 (no `to_ros` name has one in production), the DDS writer can hold one
  value more than the bus retains: the dropped message takes a place in the bus history, but not in the DDS writer's.
- **Actions:** Waypoint has no fixed-size array or bounded sequence, so only a string with a NUL can fail there; the
  guards are a defence.

## Lifecycle

- **`run(shutdown)` returns when `shutdown` completes or the bridge's scope closes** (`Scope::closed()`).
- **Before returning, in this order:**
  1. it aborts and awaits the forwarder tasks, and their bus endpoints go with them;
  2. it spins the node once more: r2r sends the answers to DDS cancel requests only at a spin, and the forwarders
     rejected the ones they held as they ended ("Actions");
  3. it stops the spin thread, which drops the node after its last `spin_once`, so no thread is left in `rcl_wait`
     (`mower_rs_common/src/module.rs`);
  4. it gives the names back.
- **The spin thread:**
  - the node lives on its own thread, as in every module (`spawn_blocking`, `spin_once(20 ms)`);
  - creating and destroying the `to_ros` DDS writers, the `from_ros` DDS readers, the `service_to_ros` DDS servers and
    the `action_to_ros` DDS action servers, and the looks at the graph, run there, between two spins, as commands from
    the forwarders;
  - publishing, requests, responses and probes run in the forwarders. r2r allows that from any thread.
- **Forwarders:** `run` aborts them all before it returns, and every one also ends on its own when the scope closes.
- **A panic** in the spin thread (r2r's `spin_once` unwraps its own locks) or in a forwarder (a bug: a value r2r cannot
  convert is caught, "Values r2r cannot convert") makes `run` stop everything and return `BridgeError::Panicked` at
  once, so that the module fails and is restarted. Without the spin thread nothing would cross any more. Await `run` to
  the end: dropped half-way, it still stops the spin thread, but does not wait for it.

## Stats

`bridge.stats()` is a handle that is cheap to clone, readable while the bridge runs. For each name it holds:

- `forwarded` (for services, calls answered; for actions, goal requests answered, accepted or rejected);
- `skipped` (volatile, no DDS reader);
- `stale` (no live bus writer);
- `resyncs` (`to_ros`: writer changes; `service_to_ros`, `action_to_ros`: DDS server changes; `service_from_ros`,
  `action_from_ros`: looks at the graph);
- `unanswered` (service calls left without an answer; for actions, goal requests left without an answer and accepted
  goals whose result never reached the other side);
- `dropped` (`to_ros` bus messages lost to the bridge's full reader queue: a forwarder stalled for more than 64);
- `invalid` (messages and calls dropped because r2r cannot convert a value; such a call is `unanswered` too);
- `bus_writer` (whether the `from_ros` bus writer exists; transient local, it follows the DDS writers, and its looks at
  the graph count in `resyncs`);
- `dds_reader` (whether the `from_ros` DDS reader exists; it follows the bus readers);
- `dds_server` (whether the `service_to_ros` or `action_to_ros` DDS server exists; it follows the bus server, and its
  changes count in `resyncs`);
- `dds_writer` (whether the `to_ros` DDS writer exists);
- `bus_server` (whether the `service_from_ros` or `action_from_ros` bus server exists).

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

**The golden corpus** (step E3.0): `the_golden_corpus_is_what_r2r_s_untyped_path_and_the_bus_give` extends the check to
every type the app and the Studio use, in both directions, from the cases of
[`mower_ws_bridge/tests/golden/fixtures.rs`](../mower_ws_bridge/tests/golden/fixtures.rs):

- **Messages the app receives** (`std_msgs` `String`, `Bool`, `Header`; `PoseStamped`; `BatteryState`, `NavSatFix`;
  `CoverageProgress`): bus -> bridge -> r2r's untyped reader, against the bus's erased reader. One of them is a real
  map layer, the frame ws_bridge sent on main for map_manage's start-up grid (213 599 bytes).
- **The services' answers** (`Trigger`, `ImportImageMask`, `EditZone`, `SiteOp`, `ZoneExecPath`, `ZoneSequence`,
  `GetCoverageProgress`, `ResumeCoverage`, `PidAutotune`, `SetParameters`): r2r's untyped client through
  `service_to_ros`, against `call_json`.
- **The app's requests:** its exact frames (a `ParameterValue` with only its type and one value, `points: []`, a
  `ResumeCoverage` without `zone_id`, integers for doubles), decoded by r2r and by the bus codec. The server must get
  the same value from both.
- **What the app and the Studio publish** (`TwistStamped`, `String`): r2r's untyped writer through `from_ros`, against
  the bus's JSON writer.
- **The values** are those that make byte identity hard: `f32` values with no exact form, `f64` extremes and `-0.0`,
  NaN and ±Inf, empty and full arrays, non-ASCII text, control characters, quotes, `i8` -1 and the integer limits.
- **The texts** are checked against `mower_ws_bridge/tests/golden/*.txt`, which `MOWER_GOLDEN=write` rewrites (review
  the diff). mower_ws_bridge's own test builds the same frames through the bus, with no DDS, and must get the same
  bytes.
- **Findings:** none differs. A request decodes alike on both paths, missing fields included (`#[serde(default)]`
  takes r2r's IDL defaults: a `ResumeCoverage` with no `zone_id` asks for zone -1).

## Tests

`tests/dds.rs` runs everything against real DDS in one process. The bridge's node and a peer node stand in for nav2, the
app and the rest, on one participant, as in production.

- **Isolation:** `ROS_DOMAIN_ID=77` and `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` are set before the process's context
  exists, and every test uses names of its own. The action tests (`tests/dds_actions.rs`) are a test binary of their own,
  on domain 89.
- **CI:** CI's `ros:jazzy-ros-base` image ships only Fast DDS, so the `mower-rs-tests` job installs
  `ros-jazzy-rmw-cyclonedds-cpp`.
- **Waiting:** discovery is polled for, never slept for. An absence is asserted only after the bridge's stats or the
  graph say what it did, or through an order that a stale value would break: a depth-2 late joiner would get a stale
  value before the fresh one. That a DDS client sees no server is a probe that has not completed after a second. The
  stats say a DDS endpoint exists once it does, and that it is gone before it is: the graph can show a new one first,
  and an old one a little longer, so a test waits for the stats of a creation and for the graph of a destruction.
- **vendor/r2r's patch** is tested there too, since CI runs only the workspace's tests and the robot's RMW matters:
  `r2r_destroys_a_service...`, `r2r_destroys_an_action_server...` and `r2r_destroys_a_subscription...` destroy and make
  again, on a node that spins, until nothing is left on the graph (`Node::get_subscriptions_info_by_topic` sees the
  readers).
- **Unit tests** (`cargo test -p mower_bus_ros --lib`) check `keep_writer`, the QoS conversion, the guarded
  conversions, and a transient-local `from_ros` on a node the test spins itself (`node::serve`): what r2r has queued
  when the writer goes, its own node's writers, and the DDS reader and bus writer going with the last bus reader.
