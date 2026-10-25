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

## The patch

`src/nodes.rs` only, every change marked `mower_rs patch`:

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

To update r2r: copy the new release here, reapply the `mower_rs patch` hunks
(or drop them if upstream fixed it), update the version and checksum above.
