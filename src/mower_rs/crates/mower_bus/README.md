# mower_bus

Step E1 of [`docs/ROS_FREE_PLAN.md`](../../../../docs/ROS_FREE_PLAN.md) (Phase E): the in-process
message bus that replaces DDS *between the modules of one process*.

- **What it provides:**
  - typed publish/subscribe, keyed by the topic names in use today;
  - request/response services;
  - actions: goals with cancel requests, feedback and a result (added in E2 for mower_nav and
    mower_coverage);
  - type-erased (JSON) access for the bridge to the app, the recorder and tooling;
  - a `Clock` that can be driven by hand for tests and replay.
- **What it depends on:** no r2r and no ROS, so it builds and tests with a plain `cargo test` on any
  host.
- **Bridging to ROS:** nodes outside the process (nav2, twist_mux, robot_state_publisher, the
  Python recorder, rviz) keep talking DDS through `mower_bus_ros` until step E9 removes them.

Later steps build on it:

| Step | Use |
|---|---|
| E2 | Each module's IO layer moves from `r2r::Node` to a `Scope` (mostly mechanical, see the cheat sheet below). |
| E3 | ws_bridge serves the app from the erased API. |
| E5 | The recorder becomes an erased tap. |
| E8 | `mowerctl topic list/echo/hz` reads the introspection, and replay drives a `ManualClock`. |

## Model

- **`Bus`:** one per process; `mower_rsd` creates it. Cheap to clone. It holds the topic registry
  and the service registry.
- **`Scope`:** one per module instance. It plays the part of an `r2r::Node`, and its name is the
  node name (used only for introspection).
  - Everything a module creates goes through its scope: publishers, subscribers, services, clients,
    erased endpoints, wall timers and child scopes.
  - **Closing a scope ends all of it:**
    - subscriber streams return `None`;
    - `publish` returns `BusError::Closed`;
    - the scope's latched history is dropped;
    - its services disappear, and calls in flight to them fail with `ServiceUnavailable`;
    - its action servers disappear, and the goals they accepted are lost for their clients
      (`ServiceUnavailable`); its action clients' goals run on, abandoned ("Actions");
    - its wall timers' `tick()` returns `Err(BusError::Closed)`, also while it waits;
    - its child scopes close too.
  - **When a close is done:**
    - `close()` returns once all of the above has happened. A `close()` that finds another one under
      way on another thread waits for it, so returning always means every endpoint has ended.
    - The exception is a close re-entered on the thread running a close: drop glue or a waker run
      by the close (see "Panics in a close") that closes the same scope, or its parent. It returns
      at once, before the scope has finished closing, and the close under way finishes the job.
      Closing scopes from such code across threads is not supported: thread A closing a child whose
      drop glue closes its parent, while thread B closes that parent and waits for the child,
      deadlocks.
    - `is_closed()` is true from the moment a close starts. From then on nothing new attaches:
      creating an endpoint or a timer fails with `Closed`, and a child comes out closed.
    - `scope.closed().await` resolves once the close has finished. It needs no runtime, and a
      `select!` loop can poll a fresh one in every turn.
  - **Panics in a close:** a close frees messages and wakes tasks, which runs user code (drop glue,
    wakers). Each endpoint's close does the bus's part first (closed, unregistered), and every
    wake and every drop is a call of its own, so one that panics skips nothing. The close ends
    everything else and wakes its waiters, then re-raises the first panic. The supervisor logs it
    and goes on.
  - The supervisor closes an instance's scope when the instance ends, however it ends, before
    restarting it. A task the old instance spawned and nobody aborted therefore cannot keep
    publishing next to the new instance, and its `while timer.tick().await.is_ok()` loops end. r2r
    gives the same guarantee today by dropping the node: its publishers fail, and its timers'
    `tick()` returns `Err`.
    - A dedicated std thread has no task to wake: it checks `scope.is_closed()` in its loop (it
      takes no lock), or ends when its subscriber's `recv_timeout` returns `Closed`.
  - **Child scopes:** `scope.child(name)` is a scope of its own, with its own node name, that closes
    when its parent closes; a child of a closed or closing scope is created closed. It can also be
    closed on its own. `mower_localize::run_all` gives each of its three roles one, and ws_bridge
    (E3) gives one to each session.
    - The parent holds each child until the child's close has finished. A child whose handles were
      all dropped, while its endpoints live on in tasks it spawned, is therefore still closed with
      its parent. A child holds nothing of its parent.
  - **Hooks of dropped endpoints:** an endpoint dropped before its scope closes leaves a small
    close hook (about 200 B) until the close. Modules mostly create their endpoints at start. The
    exception today is pid_autotune, which subscribes to the base telemetry once per tuning
    session: a hook per session, which is negligible. A component that opens endpoints per session,
    like ws_bridge in E3, uses a child scope per session and closes it on disconnect. A child takes
    no hook in its parent. A closed child is let go when the next one is made; one dropped without
    being closed stays, with its hooks, until the parent closes.
- **Names:** absolute ROS names (`/odometry/global`, `/robot/info`, `/map_manage/get_parameters`).
  - A relative name is refused.
  - Resolving launch remappings, name parameters and namespaces into absolute names is the
    caller's job. E2 adds a resolver to `ModuleCtx`.
  - The guards, the IMU, gps, all three localize nodes, nav and coverage use relative names today.
    [`docs/BUS_TOPICS.md`](../../../../docs/BUS_TOPICS.md) section 5.4 has every name that must come
    out.
- **Messages:** any `T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static`, via the
  blanket trait `Message`.
  - r2r's generated message types qualify: they derive `Clone, Serialize, Deserialize`. E2 can
    therefore carry today's types unchanged, and ROS-free mirrors of them can replace them later
    by changing imports.
  - That switch is all or nothing per name. Every endpoint of a name, the ROS bridge's included,
    must move to the mirror type in the same step. Otherwise the first endpoint fixes one type and
    the others fail with `TypeMismatch` (next item), and the run fails.
- **Type checking:** the first typed endpoint on a name fixes its type (`TypeId`, plus
  `type_name` for messages). A later endpoint with another type gets `BusError::TypeMismatch`.
  - The type stays fixed for the bus's lifetime, even after every endpoint is gone.
  - An erased endpoint can exist before the type is known and attaches when it becomes known.

## Topics

```rust
let odom = scope.publisher::<Odometry>("/odom", Qos::default().keep_last(10))?;
odom.publish(&msg)?;                       // clones once into an Arc
odom.publish_owned(msg)?;                  // or moves; publish_arc(Arc<T>) shares

let mut sub = scope.subscribe::<Odometry>("/odom", Qos::sensor_data())?;
while let Some(msg) = sub.next().await {   // futures::Stream<Item = Arc<T>>
    use_it(&msg.pose.pose.position);
}
// from a dedicated std thread:
loop {
    match sub.recv_timeout(Duration::from_millis(50)) {
        Ok(msg) => use_it(&msg.pose.pose.position),
        Err(RecvError::Timeout | RecvError::Empty) => {}   // Empty: only try_recv returns it
        Err(RecvError::Closed) => break,                     // the scope closed
    }
}
```

**Delivery**
- A subscriber receives every message published after it was created.
- Messages arrive in publish order. Publishes from different threads are ordered by the
  channel's lock.
- The message is shared as `Arc<T>`; nothing is copied per subscriber.

**Queues (KEEP_LAST)**
- Each subscriber has its own queue of `qos.depth` messages (a depth of 0 counts as 1).
- When the queue is full, the oldest queued message is dropped and counted in
  `Subscriber::dropped()`.
- Nothing else is ever dropped. `reliability` is kept for introspection and for the ROS bridge's
  QoS, but has no effect inside the process.

**Late joiners (TRANSIENT_LOCAL)**
- History belongs to each *writer*, as in DDS. A writer is a publisher together with its clones.
- A writer with `TransientLocal` durability keeps its last `depth` messages.
- A new subscriber whose own durability is `TransientLocal` first receives the retained history
  of every live writer, oldest first and in publish order, up to its own depth. After that it
  receives new messages.
- A `Volatile` subscriber never receives history.
- A writer's history goes away when its last clone drops or its scope closes.

**`publish`**
- It never waits for a reader. It takes the channel's lock briefly, to queue the message for every
  reader. After unlocking, it wakes the readers and drops the messages it pushed out of full
  queues. Both happen on the publisher's thread: waking runs the readers' wakers, and the last
  reference to a big message (a 160 kB grid) takes a while to free. Each wake and each drop is
  a call of its own: one that panics keeps no other reader asleep, and the panic goes on to the
  publisher afterwards.
- It returns `Err(BusError::Closed)` only when the writer's scope is closed. The `Result` exists
  so that r2r-style call sites (`publisher.publish(&msg)?`) keep compiling.

**`Publisher::subscription_count()`** counts the live subscribers of the channel: typed and
erased ones, including the ROS bridge's. Like `get_subscription_count()` in ROS, it lets a
producer skip expensive work nobody wants.

**`Subscriber::publisher_count()`** counts the channel's live writers, typed and JSON. Each
writer counts once, clones included, and only while its scope is open. It is rclcpp's
`get_publisher_count()`; mower_base's arm latch asks exactly this question.

**`publisher_changes()`** is a stream (`PublisherChanges`) that yields that count each time
writers come or go.
- Changes in between coalesce, so a module restart can yield an unchanged count.
- It is not ordered with the message queue, so treat every item as "resync".
- It ends when the reader's scope closes. It can outlive the reader; dropping the reader still
  frees what the reader had queued.
- The ROS bridge uses it to drop a latched value on the DDS side once the last bus writer is gone.

**Differences from DDS, all deliberate**
- **No QoS incompatibility.**
  - DDS refuses a reliable reader on a best-effort writer, and a transient-local reader on a
    volatile writer. Here they connect: the transient-local reader just gets no history.
  - The E2 inventory lists any pair DDS refuses today, because moving such a pair to the bus would
    change behaviour.
- **No deadline, liveliness or lifespan.**
- **No inter-process delivery.**

QoS mapping (constructors mirror r2r's builder names, so E2 diffs stay one-to-one):

| r2r `QosProfile` | `Qos` | depth | reliability | durability |
|---|---|---|---|---|
| `default()` | `Qos::default()` | 10 | reliable | volatile |
| `sensor_data()` | `Qos::sensor_data()` | 5 | best effort | volatile |
| `default().keep_last(1).reliable().transient_local()` | `Qos::latched()` or the same chain | 1 | reliable | transient local |
| `system_default()` | `Qos::system_default()` | 1 | reliable | volatile |
| `services_default()` | not needed (services have no QoS here) | | | |

`Qos::system_default()` is what rmw_cyclonedds resolves the system default to on this robot.
`mower_base` uses it for `/odom`, `/joint_states`, `/tf` and its cmd_vel subscription.

## Services

```rust
let mut srv = scope.service::<Trigger_Request, Trigger_Response>("/save_map")?;
while let Some(req) = srv.next().await {           // Stream<Item = Request<Req, Resp>>
    let resp = handle(&req.message);
    req.respond(resp)?;
}

let client = scope.client::<Trigger_Request, Trigger_Response>("/save_map")?;
client.wait_for_service().await?;                  // Err(Closed) if the scope closes first
let resp = client.request(&Trigger_Request {})?.await?;   // the same two-level Result as r2r
```

- **One server per name.** A second one gets `ServiceTaken`, until the first one drops or its
  scope closes.
- **Request queue:** requests queue without a limit and are handed out one at a time. A server
  that wants concurrency spawns a task per request.
- **Errors:**
  - `ServiceUnavailable`: there is no server, or the server's scope closes before it answers;
  - `NoResponse`: the server drops a `Request` without answering. r2r would leave that caller
    waiting forever.
- **Ending a server:**
  - Dropping the `Service` frees the name while its scope stays open. A `Request` answered after
    that returns `Ok` and its answer is dropped.
  - After the server's scope closes, `respond` returns `Closed`.
- **Client clones** are one client: one registration and one introspection entry, like publisher
  clones.
- **Timeouts:** none at the bus level. Callers combine `request` with `clock.sleep`, as they
  combine r2r calls with `tokio::time::timeout` today.
- **Actions** have a section of their own, next.

## Actions

Added in E2 for the one action pair inside the process: mower_coverage's client of mower_nav's
`/nav_action_follow_path` (mower_nav also serves `/nav_action`). The API is r2r's as those two use
it, so their port is a swap of names (cheat sheet below).

```rust
// what r2r calls Waypoint::Action: an action is its three message types
type WaypointAction = (Waypoint::Goal, Waypoint::Result, Waypoint::Feedback);

// the server (mower_nav)
let mut requests = scope.action_server::<WaypointAction>("/nav_action_follow_path")?;
while let Some(req) = requests.next().await {           // Stream<Item = ActionServerGoalRequest<A>>
    if !admissible(&req.goal) {
        let _ = req.reject();                           // the client gets GoalRejected
        continue;
    }
    let (mut goal, mut cancels) = req.accept()?;       // ActionServerGoal<A>, CancelRequests
    tokio::spawn(async move {
        while let Some(request) = cancels.next().await { request.accept() }   // or reject()
    });
    goal.publish_feedback(feedback)?;
    goal.succeed(Waypoint::Result { success: true })?;   // or abort(..) / cancel(..)
}

// the client (mower_coverage)
let client = scope.action_client::<WaypointAction>("/nav_action_follow_path")?;
client.wait_for_server().await?;
let (goal, result, mut feedback) = client.send_goal_request(g)?.await?;   // Err(GoalRejected)
goal.cancel()?.await?;                                  // Err(CancelRejected), ...
let (status, result) = result.await?;                   // a terminal GoalStatus
```

- **Built natively,** not from services and topics the way rcl_action builds actions on DDS:
  - the two ends of a goal share one state: its status, its result, the feedback queued for the
    client and the cancel requests queued for the server;
  - so there is no goal, cancel or result service and no feedback or status topic to see in
    `bus.services()` / `bus.topics()`. `bus.actions()` lists actions ("Introspection").
- **Like a service:** one server per name (`ServiceTaken`), the first endpoint fixes the name's three
  types (`TypeMismatch`), names are absolute, no runtime is needed. Actions are a namespace of their
  own: a topic or a service may have the same name.
- **Action names resolve like topic names.** rcl_action resolves them with the topic rules
  (`rostopic://` rules apply, `rosservice://` rules do not; checked against rcl on Jazzy), so a port
  uses `resolve_topic`.

**Goal requests**
- They queue without a limit and are handed out one at a time, like service requests. A server that
  runs goals concurrently spawns a task per request, as mower_nav does.
- With no live server at send time, `send_goal_request` fails at once with `ServiceUnavailable`.
  (rcl_action sends it anyway, and the client waits forever.)
- The server's answer:
  - `accept()`: the goal is `Executing` at once, as r2r has it (`Accepted` never shows), and the
    client gets the goal's handle, its result future and its feedback stream;
  - `reject()`: the client gets `GoalRejected`;
  - dropping the request unanswered: the client gets `NoResponse` (r2r's client would wait forever).
- **Giving a request up:** dropping the response future does not withdraw the request. The server
  still gets it and may accept it; the goal then runs with nobody to read its feedback or result, as
  with r2r. mower_coverage keeps the future after its 3 s acceptance timeout, to adopt and cancel
  such a goal. Nothing can cancel a goal before it is accepted: there is no handle yet.
- **Goal ids** (`GoalId`) are numbered by the bus per name, from 1 up, not UUIDs that clients pick.
  Both ends see the same id.

**Status and the end of a goal**
- `get_status()` reads the goal's state as it is now: `Executing`, `Canceling` (a cancel request
  accepted), then `Succeeded`, `Canceled` or `Aborted`, or `Unknown` once the goal is lost (below).
  r2r returns the last status message it saw.
- The server ends a goal with `succeed`, `abort` or `cancel`, from `Executing` or `Canceling`. The
  first call counts. A later one, and feedback after the end, fail with `GoalTerminated`.
- **Results** go to the goal's own result future, once: `(status, result)` with a terminal status.
  There is no result service that another client could ask, and no expiry. The result is kept until
  that future takes it or every handle of the goal is gone.

**Cancel requests**
- `goal.cancel()` queues a request for the server's `CancelRequests` stream (r2r's cancel stream).
  The server answers:
  - `accept()`: the goal is `Canceling` and the client's future resolves `Ok`. The server still
    ends the goal itself, usually with `cancel`;
  - `reject()`: `CancelRejected`, and the goal runs on;
  - dropping the request unanswered: `NoResponse`.
- Answered at once, without asking the server:
  - a goal that has ended: `GoalTerminated`;
  - a goal that is canceling already: also `GoalTerminated`, which is what rcl_action answers
    (checked against rcl on Jazzy);
  - a goal whose server dropped its `CancelRequests`: `CancelRejected`. r2r lets rcl_action report
    such a request accepted without asking the server.
- A request still queued when the goal ends, or accepted after the end, gets `GoalTerminated`.
- Both streams end with the goal: `CancelRequests` once it has ended, the feedback stream once it
  has ended and the client has read what came before. r2r's streams never end.

**Feedback**
- Each goal queues up to 10 messages for its client (KEEP_LAST 10, the depth of rcl_action's
  feedback topic). When the queue is full, the oldest message is dropped and counted in
  `FeedbackStream::dropped()`.
- Feedback arrives in publish order, and all of it before the stream ends.
- Feedback that no client reads (the feedback stream was dropped, or the client gave the request
  up) is dropped at once.

**Servers that go away (what mower_nav's restart looks like to mower_coverage)**
- **The server's scope closes:**
  - its queued and handed-out goal requests fail with `ServiceUnavailable`;
  - its goals that have not ended are **lost**: the result future fails with `ServiceUnavailable`,
    the status becomes `Unknown`, cancel requests fail with `ServiceUnavailable`, and the feedback
    stream ends once read;
  - the server's handles get `Closed` from then on.
  - A lost goal never gets a terminal status. It may still be running in the world (nav2 driving a
    path), so mower_coverage takes its "outcome is uncertain" path: an action cancel, which fails at
    once, then the correlated cancel through `/cancel_navigation_dispatch`, which the restarted nav
    answers. With r2r the client sees nothing at all when the server's node goes: no result, no end of
    feedback, no answer to a cancel, until its own timeouts fire.
- **The server drops the last clone of a goal's handle before ending it:** the goal is lost the same
  way, with `NoResponse`. rclcpp would end it CANCELED and rclpy ABORTED, r2r leaves it executing;
  none of those tells the client the truth.
- **The server drops its `ActionServer` while its scope stays open:** the name is free for a new
  server and the queued requests fail with `ServiceUnavailable`. The requests it handed out and the
  goals it accepted go on, and end with their own handles or the scope.

**Clients that go away**
- **The client's scope closes:** its goals are abandoned, as a ROS client process that exits leaves
  them. They run on; the server is not told.
  - The client's response futures, result futures and cancel futures resolve to `Closed`, whatever
    the server did, so a stale task of an ended module does not act on them. Its feedback streams
    end at once.
  - Goal requests sent before the close still reach the server, as service calls do.
- **The client drops a goal's handles:** the goal runs on, as in ROS.

**Timeouts:** none at the bus level, as for services. Clients combine the futures with
`clock.timeout`, as they combine r2r's with `tokio::time::timeout` today.

**Differences from rcl_action and r2r, besides the ones above**
- `succeed` and `abort` work after an accepted cancel request, as rcl_action allows. r2r refuses them
  there (it forgets the goal), which is why mower_nav's `terminate` falls back to `cancel`; that
  fallback keeps working.
- `is_cancelling()` is `true` after an accepted cancel request; r2r fails with an error there.
- `cancel` ends a goal `Canceled` also when no cancel request was accepted. r2r allows it (and
  leaves rcl's status `EXECUTING`); rcl_action reaches CANCELED only from CANCELING.
- Cancel requests are per goal, through its handle. rcl_action's "cancel all goals" and "cancel
  goals accepted before a stamp" do not exist on the bus. Through the bridge they do, since rcl
  splits them into one request per goal.

## Erased access (JSON)

For components that learn topic names and types at run time: ws_bridge (the app's
subscribe / publish / call_service), the recorder and `mowerctl`.

- **Codec:** every typed endpoint registers a codec for its type when it creates the channel. The
  codec is a set of `fn` pointers monomorphized over `T`: `to_json`, `from_json` and the type
  name.
- **`scope.subscribe_erased(name, qos)`** yields `ErasedMsg`s:
  - `to_json()` serializes on the subscriber's side, so a publisher never pays for JSON nobody
    reads;
  - `downcast::<T>()` returns the `Arc<T>` without a copy.
- **`scope.publisher_json(name, qos)`** decodes with the channel's codec and publishes. It is an
  error until some typed endpoint has fixed the type.
- **`scope.call_json(name, json)`** decodes the request with the service's codec, calls, and
  encodes the response.
- **The JSON layout is serde's layout of the message type.** For r2r types that is what ws_bridge
  sends the app today: its untyped r2r path converts through the same derives. E3 verifies this
  byte for byte against the app contract (`tools/probe_app.py`).
- **Names are never forgotten.** `subscribe_erased` and `publisher_json` create the channel of any
  valid name they are given, and channels are never removed. E3 must therefore keep ws_bridge's
  allow-list in front of them, or an app could grow the registry without bound.
- **Actions have no erased access.** The app uses no action, so E3 does not need one. A goal's
  state already holds its messages erased, with the name's codecs, so a JSON client would be a set of
  handle types around the same state, like `call_json` for services.

## Clock

```rust
let now: Time = clock.now();          // wall / ROS time, ns since the UNIX epoch (header stamps)
let t0: Mono = clock.mono();          // monotonic, replaces Instant::now()
clock.sleep(Duration::from_millis(50)).await;
let mut tick = clock.interval(Duration::from_millis(50));   // first tick at once, like tokio
loop { tick.tick().await; ... }
clock.sleep_blocking(d);              // for dedicated std threads
```

- **`Clock::system()`:**
  - wall time is `SystemTime`;
  - `mono` is `Instant`, measured from one process-wide origin, so values from different
    `Clock::system()` handles compare;
  - async sleeps use `tokio::time` and need a tokio runtime with time enabled, which every module
    already has.
- **`Clock::manual(start)`** returns `(Clock, ManualClock)`:
  - the clock moves only when the `ManualClock` handle is told to: `advance(d)`,
    `advance_to_next()` (jump to the earliest pending deadline), `set_wall(t)` (a wall step, as
    from NTP, while `mono` keeps running);
  - when it moves, the tasks whose deadline has passed (sleeps, intervals, timeouts) are woken in
    deadline order, then the threads blocked in a sleep are notified. A waker that panics keeps no
    other sleeper asleep; its panic goes on to whoever moved the clock;
  - it needs no runtime and works for blocking sleepers too.
  - Tests use it instead of tokio's process-wide paused time. Replay (E8) feeds it recorded
    stamps.
- **`Interval`:**
  - **`interval(period)`:** the first tick completes immediately and `MissedTick::Burst` is the
    default (tokio's semantics).
  - **`interval_at(start, period)` with `MissedTick::Skip`:** reproduces an r2r / rcl wall timer.
    The first tick comes after one period, and after an overrun it fires once and stays on the
    original phase. `scope.wall_timer(&clock, period)` is that schedule tied to a scope (see
    "Model"): it is what a port of `node.create_wall_timer` uses.
- **Where `Skip` differs from rcl:** both cases are inherited from tokio's interval.
  - An overrun that ends exactly on a phase point moves on to the next point, where rcl fires at
    once. This only shows on a manual clock, where times are exact.
  - A tick up to 5 ms late still counts as on time, so periods under 5 ms get an extra tick. Every
    period in mower_rs is at least 20 ms.
- **`timeout(d, fut)` / `timeout_at(deadline, fut)`** replace `tokio::time::timeout`, which has
  36 call sites today.
  - The future is polled first, so it wins when both are ready.
  - The result is `Err(Elapsed)` once the deadline has passed.
- **Pending deadlines:** a `Sleep`, an `Interval`'s next tick or a `Timeout` counts as pending from
  the moment it is created until it completes or is dropped, for `next_deadline` and
  `advance_to_next`.
- **A deadline of `Mono::from_ns(u64::MAX)` means never.** That is where `mono + Duration::MAX`
  saturates, so `sleep(Duration::MAX)` and `timeout(Duration::MAX, ..)` wait for good.
  - The system clock never gets there.
  - A manual clock does not count it as pending: `advance_to_next` never jumps to it. It comes only
    when the clock is moved there explicitly (`advance(Duration::MAX)`).
- **Under tokio's paused time** (`start_paused`), the system clock's sleeps follow tokio, but
  `mono()` and `now()` stay real. Tests drive a `Clock::manual` instead.
- **`Time` converts to and from `(sec: i32, nanosec: u32)`**, the `builtin_interfaces/Time`
  layout. Negative times use floor and remainder, so `nanosec` is always in `0..1e9`.
  - Outside the `i32` seconds range the conversion saturates instead of wrapping.
  - Arithmetic saturates at the `i64` limits.

## Introspection

- **`bus.topics()`** returns, for each topic:
  - its name, Rust type and ROS type (`ros_type_name` maps `r2r::nav_msgs::msg::Odometry` to
    `nav_msgs/msg/Odometry`);
  - the names of the scopes that publish and subscribe to it;
  - its retained history;
  - how many messages have been published, and when the last one was. That time is a std
    `Instant`, not a reading of the bus clock: a manual clock does not move it.
- **`bus.services()`** returns the same kind of record for services.
- **`bus.actions()`** returns, for each action: its three Rust types and its ROS type
  (`mower_interface/action/Waypoint`), its server's and its clients' scope names, the goal requests
  that reached a server, and the goals running now.
- These feed `/rosapi/topics` (E3) and `mowerctl` (E8).

## r2r to mower_bus cheat sheet (for E2)

| r2r | mower_bus |
|---|---|
| `r2r::Node::create(ctx, name, ns)` | `m.bus` (the module's `Scope`) |
| `node.create_publisher::<T>(topic, qos)` | `m.bus.publisher::<T>(abs_topic, qos)` |
| `publisher.publish(&msg)` | unchanged |
| `node.subscribe::<T>(topic, qos)` (Stream of `T`) | `m.bus.subscribe::<T>(abs_topic, qos)` (Stream of `Arc<T>`; field access is unchanged, moving a field out needs `Arc::unwrap_or_clone`) |
| `node.create_service::<S>(name, qos)` and `req.respond(resp)` | `m.bus.service::<S::Request, S::Response>(name)` and the same `respond` |
| `node.create_client::<S>(name, qos)` and `client.request(&req)?.await?` | `m.bus.client::<..>(name)` and unchanged |
| `node.is_available(&client)?.await` | `client.wait_for_service().await?` |
| `node.create_action_server::<Waypoint::Action>(name)?` (Stream of `ActionServerGoalRequest`) | `m.bus.action_server::<WaypointAction>(abs_name)?` with `type WaypointAction = (Waypoint::Goal, Waypoint::Result, Waypoint::Feedback)`; `abs_name` from `resolve_topic` |
| `r2r::ActionServerGoalRequest<Waypoint::Action>`, `r2r::ActionServerGoal<..>`, `ActionServerCancelRequest` | `mower_bus::` the same names, over `WaypointAction`; `req.goal`, `req.accept()?`, `req.reject()`, the cancel stream's `accept()` / `reject()`, `goal.succeed/abort/cancel(r)`, `publish_feedback(f)`, `is_cancelling()` unchanged |
| `node.create_action_client::<Waypoint::Action>(name)?` | `m.bus.action_client::<WaypointAction>(abs_name)?` |
| `r2r::Node::is_available(&action_client)?.await` | `client.wait_for_server().await?` |
| `client.send_goal_request(goal)?.await?` → `(ActionClientGoal, result future, feedback stream)` | unchanged |
| `Err(r2r::Error::RCL_RET_ACTION_GOAL_REJECTED)` | `Err(BusError::GoalRejected { .. })` |
| `goal.cancel()?.await` (`GoalCancelRejected`, `GoalCancelAlreadyTerminated`, ...), `goal.get_status()` | unchanged; the errors are `CancelRejected`, `GoalTerminated`, ... |
| `r2r::GoalStatus`, the handles' `uuid` | `mower_bus::GoalStatus` (the same seven variants), `id: GoalId` |
| `node.create_wall_timer(p)?` and `while timer.tick().await.is_ok()` | `m.bus.wall_timer(&m.clock, p)?`, the loop unchanged (`tick()` returns the deadline, where r2r's returns the time since the last tick; none of today's loops uses it) |
| `tokio::time::sleep(d)` / `Instant::now()` / `SystemTime::now()` | `m.clock.sleep(d)` / `m.clock.mono()` / `m.clock.now()` |
| `tokio::time::timeout(d, fut)` | `m.clock.timeout(d, fut)` (`Err(Elapsed)` as in tokio, but `Elapsed` is `mower_bus::Elapsed`: code that names tokio's `Elapsed` type changes) |
| `node.spin_once(..)` thread | gone: the bus needs no spinning |
| parameters (`mower_rs_common::params`) | unchanged in E1; E2 moves them to one YAML read by mowerd |
