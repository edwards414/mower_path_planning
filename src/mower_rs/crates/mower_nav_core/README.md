# mower_nav_core

The ROS-free core of Phase D of [`docs/ROS_FREE_PLAN.md`](../../../../docs/ROS_FREE_PLAN.md)
(section 8, D1–D4): a Rust port of the parts of **nav2 (Jazzy, 1.3.13)** this
mower actually runs, as configured by
[`mower_nav2/config/nav2_no_map_params.yaml`](../../../mower_nav2/config/nav2_no_map_params.yaml).

Pure core, in the style of `mower_localize_core`: no r2r, no ROS, no I/O, no
clock. Every entry point takes explicit poses, grids and timestamps, so the
same code runs live, in shadow mode against nav2, in replay and under test. It
builds and tests with a plain `cargo test` on any host.

Upstream source: <https://github.com/ros-navigation/navigation2>, tag `1.3.13`
(`f4108e5`), the version of the `ros-jazzy-nav2-*` packages in the runtime
image.

| Rust | Upstream | Plan |
|---|---|---|
| `src/geometry.rs` | `angles`, `tf2::getYaw`, `Quaternion::setRPY`, `nav2_util/geometry_utils.hpp` | — |
| `src/costmap.rs` | `nav2_costmap_2d`: `Costmap2D`, `LayeredCostmap::updateMap`, `StaticLayer` | D1 |
| `src/footprint.rs` | `footprint.cpp`, `costmap_math.cpp`, `FootprintCollisionChecker`, `nav2_util::LineIterator` | D1 |
| `src/navfn.rs`, `src/planner.rs` | `nav2_navfn_planner`: `NavFn`, `NavfnPlanner` | D2 |
| `src/rpp.rs`, `src/path_handler.rs` | `nav2_regulated_pure_pursuit_controller` | D3 |
| `src/shim.rs` | `nav2_rotation_shim_controller` | D3 |
| `src/checkers.rs` | `nav2_controller` plugins: `SimpleGoalChecker`, `SimpleProgressChecker` | D3 |
| `src/smoother.rs` | `nav2_velocity_smoother` | D4 |

## Costmaps (D1)

Both costmaps have a `StaticLayer` and nothing else.

* **global** (`/map_grid_global`, frame `map`, `track_unknown_space: true`, not
  rolling): `static_costmap` — the map's own size, resolution and origin, each
  cell through `StaticLayer::interpretValue` (`-1` -> 255 NO_INFORMATION,
  `>= 100` -> 254 LETHAL, other -> 0 with `trinary_costmap`).
* **local** (`/map_grid`, frame `odom`, `track_unknown_space` left at its
  default false, 3 x 3 m rolling window at 0.05 m = 60 x 60 cells):
  `RollingWindow::update` is one `LayeredCostmap::updateMap` — `updateOrigin`
  re-centres the window by whole cells, the whole window is reset to FREE and
  every cell centre is looked up in the map through `map <- odom`. Off-map and
  unknown cells are FREE here, as upstream.
* the footprint is the 16-point `robot_radius: 0.5` circle padded by
  `footprint_padding` (a float 0.01) with `padFootprint`'s per-axis sign rule;
  `FootprintCollisionChecker` scores its outline only, and a vertex off the
  grid is lethal.

## Planner (D2)

`NavfnPlanner` with `tolerance: 0.0`, `use_astar: false`, `allow_unknown: true`
on the global costmap.

* `navfn.rs` is `NavFn` line by line: `setCostmap` (0..252 -> `50 + 0.8 c`,
  253/254 -> obstacle, unknown -> 253 when allowed), the Dijkstra over the three
  10000-cell priority buffers (a push that does not fit is dropped, as
  upstream), the `max(nx * ny / 20, nx + ny)` cycle limit, A*, `calcPath` and
  `gradCell`. The FMA contractions of the shipped aarch64
  `libnav2_navfn_planner.so` (read from its disassembly) and glibc's `hypotf`
  are reproduced, so potentials and paths are bit-identical to nav2's.
* `planner.rs` is `NavfnPlanner`: `createPlan` with every early return,
  `makePlan` with the tolerance search, `getPlanFromPotential`,
  `smoothApproachToGoal`, `use_final_approach_orientation`, its rounding
  `worldToMap`, and `clearRobotCell`, which writes into the costmap it is given.
  Errors are `PlannerError` with the `ComputePathToPose` codes 200–208.
  `compute_path_to_pose` adds planner_server's "empty path -> NO_VALID_PATH".
* Not ported: lifecycle, the action server, the costmap wait, TF and plan
  publishing (they belong to the caller).
* Upstream behaviour kept on purpose, so a shadow run compares like with like:
  a goal in unknown space is accepted (GoalOccupied only for LETHAL), the bounds
  check uses the truncated cell but the plan the rounded one (plans sit half a
  cell towards the origin), `clearRobotCell` leaves a FREE hole in the global
  costmap until the next `/map_grid_global`, a goal that rounds onto the border
  row or column is unreachable, and a connected but far goal can run out of
  cycles. The first pose is the robot's cell corner; every pose but the last
  (the goal itself) has identity orientation.
* `MapMetaData.resolution` is a `float32`: the costmap's resolution is
  `(double)0.05f`, so the IO layer must widen the f32, not reparse 0.05.

## Controller (D3): RotationShim + RegulatedPurePursuit

`RotationShim<RegulatedPurePursuit>::mower()` is `FollowPath`. A cycle takes
the robot pose in odom, the (thresholded) speed, the goal checker's xy
tolerance, `costmap_from_plan` (odom <- map) and the local costmap
(`ControlInput`); there is no clock and no tf. `Controller` is nav2_core's
interface (`set_plan`, `compute_velocity_commands`, `set_speed_limit`,
`reset`, `cancel`), `ControllerError` its exception classes.

* Ported line by line and checked cycle by cycle against the real plugins
  (9,377 replayed cycles, 706 of them shim rotations): the plan transform and
  pruning (`max_robot_pose_search_dist`, at least two poses, cut at the window
  extent of 1.4875 m), the carrot by circle/segment intersection, curvature and
  its speed limit, the minimum speed, the approach slow-down, the collision
  check (forward projection, and the rotation step), the costmap-edge
  exception, speed limits; the shim's engage at 0.6 rad / disengage at 0.35,
  the accel-limited rotation and its collision check, `rotate_to_goal_heading`,
  `closed_loop`, `rotate_to_heading_once`, `use_path_orientations`. Branches
  the yaml leaves off (rotate-to-heading, velocity-scaled lookahead, reversing,
  fixed-curvature lookahead, cost regulation) are checked with two other
  parameter sets.
* One known difference: with mower_nav's 0.1 m pose spacing, five remaining
  poses are exactly the yaml's 0.4 m `approach_velocity_scaling_dist`, and
  `length < 0.4` is decided by the last bit (tf2's fused quaternion maths and
  glibc `hypot` upstream). 10 of 7,422 cycles take the other side, one cycle of
  0.26 against ~0.25 m/s near a segment end; the test proves each of them.
* Not ported: publishers (`transformed_plan()` / `carrot()` keep the data),
  dynamic parameters, mutexes, lifecycle. Ported but not reachable here: RPP's
  cancel deceleration (the shim never forwards `cancel`), the unknown-space
  branches (the local costmap does not track unknown), tf errors.
* Upstream behaviour that matters on this robot (all reproduced):
  * the shim acts only after `set_plan`, sampling 0.25 m from the plan's
    *first* pose; mower_nav sends every coverage segment as its own FollowPath,
    so a 90-degree corner segment gets one in-place rotation, while the
    coverage U-turns start at ~0.58 rad, just under the 0.6 threshold; turns
    inside one plan are RPP's alone, and re-sending an unchanged plan mid-run
    re-arms the shim (it turned ~163 degrees back in the oracle);
  * a footprint already touching a lethal cell fails the shim's rotation check
    and RPP's collision check: `NoValidControl`, and after the server's 2 s
    `failure_tolerance` the goal aborts — a row end near the boundary can
    strand the mower;
  * the collision look-ahead is 5 x 5 cm at 0.26 m/s and only the current pose
    below 0.05 m/s; a lethal cell on the path stopped the robot 0.73 m short;
  * the plan is cut at its first pose beyond the window and not picked up
    again; a robot more than 1.49 m from the plan gets `InvalidPath` (after the
    shim has rotated towards it); poses more than 0.25 m apart misbehave, which
    mower_nav's 0.1 m spacing avoids;
  * upstream's shim waits up to 0.1 s for a fresh odom -> base transform and
    silently skips the rotation if it is late; the port uses the cycle's pose
    and never skips;
  * `min_distance_to_obstacle` does nothing in 1.3.13; the shim and RPP share
    `rotate_to_heading_angular_vel` and `max_angular_accel`.

## Checkers (D3) and velocity smoother (D4)

* `checkers.rs`: `SimpleGoalChecker` (xy 0.05, yaw 3.14, stateful: once inside
  5 cm only the yaw is checked until `reset`; with 3.14 that is effectively
  "reached on first entry") and `SimpleProgressChecker` (0.1 m, 20 s: moving
  more than 0.1 m from the baseline re-baselines; otherwise
  `now - baseline <= 20 s`). controller_server resets the goal checker on every
  new or preempting path, the progress checker once per FollowPath goal.
* `smoother.rs`: `VelocitySmoother` OPEN_LOOP at 20 Hz as an explicit
  `tick(now_ns)`: the input callbacks (stamp handling, NaN/inf rejection),
  `smootherTimer` (timeout, the `stopped_` latch, the re-stamped zero),
  velocity clamping, `applyConstraints` and `findEtaConstraint`. With
  `stamp_smoothed_velocity_with_smoothing_time: false` every output keeps the
  input command's stamp; the 1 s timeout counts from that stamp, strictly; on a
  timeout while moving the command becomes a zero stamped `now` and the output
  ramps down at the decel limit (the first ramp message still carries the old
  stamp). The acceleration step is always `accel / 20`. Outputs are
  bit-identical to the binary.
* Not ported: the dynamic-parameter callbacks, `feedback: CLOSED_LOOP`, the
  unstamped publisher, realtime priority, lifecycle. Ported and checked against
  the oracle with other parameter sets but not reached by this yaml:
  `symmetric_yaw_tolerance`, non-stateful goal checking, `scale_velocities`,
  the deadband, `stamp_smoothed_velocity_with_smoothing_time: true`.
* Deliberate deviation: a negative stamp makes upstream's callback throw after
  it has stored the command (the node dies); here it is an error and changes
  nothing.

## Tests

`tests/*_oracle.rs` compare against JSON vectors produced by the real nav2
libraries (`tests/oracle/<component>/oracle.cpp`, built against the installed
`ros-jazzy-nav2-*` 1.3.13 packages):

```bash
docker build -t mower-nav2-oracle:jazzy - < tests/oracle/Dockerfile
docker run -d --name rf-d-oracle -v <repo>:/repo mower-nav2-oracle:jazzy sleep infinity
docker exec rf-d-oracle bash /repo/src/mower_rs/crates/mower_nav_core/tests/oracle/run_oracle.sh   # all components
cargo test -p mower_nav_core
```

Integer results (cells, costs, indices, error kinds) must match exactly. The
nav2 binaries are GCC aarch64 builds, which contract `a + b * c` into FMA
(`-ffp-contract=fast`) and use glibc's libm. Where a last-bit difference can
flip a decision (NavFn's potentials and path steps, the goal checker's
`dx * dx + dy * dy`) the port uses `mul_add` exactly where the shipped
disassembly has `fmadd`, and those results are bit-identical. Elsewhere floats
are compared to 1e-12 relative (`tests/common::TOL`): `Costmap2D::updateOrigin`
is fused upstream and not here, and `cos(22.5 deg)` differs in the last bit
between glibc and macOS. The oracle JSON is read with serde_json's
`float_roundtrip`; without it about one float in ten parses one ulp off.
