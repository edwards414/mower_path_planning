# mower_bus

Step E1 of [`docs/ROS_FREE_PLAN.md`](../../../../docs/ROS_FREE_PLAN.md) (Phase E): the in-process
message bus that replaces DDS *between the modules of one process*.

- **What it provides:**
  - typed publish/subscribe, keyed by the topic names in use today;
  - request/response services;
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
    erased endpoints.
  - **Closing a scope ends all of it:**
    - subscriber streams return `None`;
    - `publish` returns `BusError::Closed`;
    - the scope's latched history is dropped;
    - its services disappear, and calls in flight to them fail with `ServiceUnavailable`.
  - The supervisor closes an instance's scope when the instance ends, however it ends, before
    restarting it. A task the old instance spawned and nobody aborted therefore cannot keep
    publishing next to the new instance. r2r gives the same guarantee today by dropping the node.
- **Names:** absolute ROS names (`/odometry/global`, `/robot/info`, `/map_manage/get_parameters`).
  - A relative name is refused.
  - Resolving launch remappings and namespaces into absolute names is the caller's job (E2 adds a
    resolver to `ModuleCtx`; the guards and the IMU rely on remapping today).
- **Messages:** any `T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static`, via the
  blanket trait `Message`.
  - r2r's generated message types qualify: they derive `Clone, Serialize, Deserialize`. E2 can
    therefore carry today's types unchanged, and ROS-free mirrors of them can replace them later
    by changing imports.
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
match sub.recv_timeout(Duration::from_millis(50)) { Ok(msg) => .., Err(RecvError::Timeout) => .., Err(RecvError::Closed) => break }
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
- It never blocks.
- It returns `Err(BusError::Closed)` only when the writer's scope is closed. The `Result` exists
  so that r2r-style call sites (`publisher.publish(&msg)?`) keep compiling.

**`Publisher::subscription_count()`** counts the live subscribers of the channel: typed and
erased ones, including the ROS bridge's. Like `get_subscription_count()` in ROS, it lets a
producer skip expensive work nobody wants.

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
| `services_default()` | not needed (services have no QoS here) | | | |

## Services

```rust
let mut srv = scope.service::<Trigger_Request, Trigger_Response>("/save_map")?;
while let Some(req) = srv.next().await {           // Stream<Item = Request<Req, Resp>>
    let resp = handle(&req.message);
    req.respond(resp)?;
}

let client = scope.client::<Trigger_Request, Trigger_Response>("/save_map")?;
client.wait_for_service().await;
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
- **Timeouts:** none at the bus level. Callers combine `request` with `clock.sleep`, as they
  combine r2r calls with `tokio::time::timeout` today.
- **Actions are not in E1.** mower_nav's `nav_action*` and coverage's client of it are the only
  action pairs inside the process. They are built from a goal service, a feedback topic and a
  result service when E2 gets there; nav2's actions disappear in E9.

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
  - when it moves, sleepers and intervals whose deadline has passed wake in deadline order;
  - it needs no runtime and works for blocking sleepers too.
  - Tests use it instead of tokio's process-wide paused time. Replay (E8) feeds it recorded
    stamps.
- **`Interval`:**
  - **`interval(period)`:** the first tick completes immediately and `MissedTick::Burst` is the
    default (tokio's semantics).
  - **`interval_at(start, period)` with `MissedTick::Skip`:** reproduces an r2r / rcl wall timer.
    The first tick comes after one period, and after an overrun it fires once and stays on the
    original phase.
- **`Time` converts to and from `(sec: i32, nanosec: u32)`**, the `builtin_interfaces/Time`
  layout. Negative times use floor and remainder, so `nanosec` is always in `0..1e9`.

## Introspection

- **`bus.topics()`** returns, for each topic:
  - its name, Rust type and ROS type (`ros_type_name` maps `r2r::nav_msgs::msg::Odometry` to
    `nav_msgs/msg/Odometry`);
  - the names of the scopes that publish and subscribe to it;
  - its retained history;
  - how many messages have been published, and when the last one was.
- **`bus.services()`** returns the same kind of record for services.
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
| `node.is_available(&client)?.await` | `client.wait_for_service().await` |
| `node.create_wall_timer(p)` and `timer.tick().await` | `m.clock.interval_at(m.clock.mono() + p, p)` with `MissedTick::Skip`, then `tick().await` |
| `tokio::time::sleep(d)` / `Instant::now()` / `SystemTime::now()` | `m.clock.sleep(d)` / `m.clock.mono()` / `m.clock.now()` |
| `node.spin_once(..)` thread | gone: the bus needs no spinning |
| parameters (`mower_rs_common::params`) | unchanged in E1; E2 moves them to one YAML read by mowerd |
