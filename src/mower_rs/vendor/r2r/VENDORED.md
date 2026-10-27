# Vendored r2r

`r2r` 0.9.7 from crates.io (checksum
`1fe8623ef13a4f3c25f4ef407c6032eee9e1c95d0912d8ed758f616cbb95a6cd`), the
latest release and the version `Cargo.lock` already used. Upstream:
<https://github.com/sequenceplanner/r2r>, licensed MIT AND Apache-2.0 (see its
`Cargo.toml`). The workspace uses it through `[patch.crates-io]` in
`../../Cargo.toml`; its own dependencies (`r2r_rcl`, `r2r_msg_gen`, ...) still
come from crates.io.

Files are as published, minus `Cargo.lock` and `bindings/` (pre-generated
bindings for the `doc-only` feature, which mower_rs does not use).

## The patches

Every change is marked `mower_rs patch`.

### 1. `spin_once` keeps its wait set (`src/nodes.rs`)

* `Node::spin_once` upstream calls `rcl_wait_set_init` and `rcl_wait_set_fini`
  on every call. On rmw_cyclonedds that creates a DDS waitset, attaches every
  subscription, timer, client and service, waits, then detaches and deletes it
  all, on each wakeup of each node. A perf profile of `mower_rsd` on XR-02
  (2026-10-25, idle) had 43 % of its CPU under `spin_once`, about a third of
  that in creating and destroying wait sets.
* The node now keeps its wait set (`Node::wait_set`) and rebuilds it only when
  the number of subscriptions, timers, clients or services in it changes.
  `rcl_wait_set_clear` and the adds still run before every `rcl_wait`, so the
  contents are always the node's current entities. rmw_cyclonedds only
  re-attaches entities when they differ from the previous wait on the same
  wait set, so an unchanged node no longer re-attaches anything.
* `Drop for Node` finalizes the kept wait set before destroying the entities.

### 2. Destroying a subscription, a service or an action server (2026-10-27)

mower_bus_ros's bridge holds an endpoint on DDS only while the bus has its
counterpart, as a module's own r2r endpoints lived and died with its node:
`from_ros`'s DDS reader while the bus has a reader, `service_to_ros`'s DDS
server while the bus has a server; `action_to_ros` is to do the same.
Upstream destroys a subscription or a service only when its stream has been
dropped *and* the next message or request comes in (a request then is lost,
and the entity stays on the graph until then), and an action server only with
its node.

* `Node::subscribe` returns a `Subscription<T>`, `create_service` a
  `Service<T>` and `create_action_server` an `ActionServer<T>`
  (`src/subscribers.rs`, `src/services.rs`, `src/action_servers.rs`, exported
  from `src/lib.rs`): the same streams as before, `Send + Sync + Unpin`, which
  also know their entity. The other `subscribe_*` variants are unchanged.
* `Node::destroy_subscription`, `destroy_service` and `destroy_action_server`
  take them back, on the thread that spins the node, between two
  `spin_once`s. They take the entity out of the node and finalize it; a
  service or an action server under its lock, so that a
  `ServiceRequest::respond` or an `ActionServerGoal` call racing with it waits
  and then finds it finalized (an error, not a crash). Queued messages and
  requests are dropped.
* An action server's fini frees its rcl goal handles, so
  `WrappedActionServer::destroy` also forgets them (and its pending cancel
  requests and result requests): its `ActionServerGoal`s, which outlive it,
  then fail as for an unknown goal.
  `ActionServerGoalRequest::accept` and `reject` return
  `RCL_RET_ACTION_SERVER_INVALID` once the server is gone, where upstream
  unwraps (`// todo fixme`) and panics.
* **The kept wait set holds only live entities:** a destroy finalizes it
  (`drop_wait_set`), and so does a `spin_once` that destroyed an entity
  itself (a subscription, timer or service whose stream was dropped). The
  next `spin_once` builds a new one with the node's sizes. Before, a wait set
  that had held a destroyed entity was kept, and reused if a create made the
  sizes match again; `rcl_wait_set_clear` emptied its arrays, but the reuse
  relied on the RMW having dropped its own cache of the entity
  (rmw_cyclonedds does, in `clean_waitset_caches`), or a new entity at the
  old one's address would never be attached.
* `Node::get_subscriptions_info_by_topic`, the readers' counterpart of
  `get_publishers_info_by_topic`, so that the tests can see a service's
  request reader leave the graph too.

Tested in the workspace, on rmw_cyclonedds (CI does not run this crate's own
tests): `crates/mower_bus_ros/tests/dds.rs`, `r2r_destroys_a_subscription...`,
`r2r_destroys_a_service...` and `r2r_destroys_an_action_server...`.

To update r2r: copy the new release here, reapply the `mower_rs patch` hunks
(or drop them if upstream fixed it), update the version and checksum above.
