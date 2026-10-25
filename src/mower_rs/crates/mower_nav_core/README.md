# mower_nav_core

The ROS-free core of Phase D of [`docs/ROS_FREE_PLAN.md`](../../../../docs/ROS_FREE_PLAN.md)
(section 8, D1–D5): a Rust port of the parts of **nav2 (Jazzy, 1.3.13)** this
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
| `src/controller_server.rs` | `nav2_controller::ControllerServer` (`follow_path`), `nav2_util::SimpleActionServer` | D3 |
| `src/planner_server.rs` | `nav2_planner::PlannerServer` (`compute_path_to_pose`) | D2 |
| `src/navigator.rs` | `nav2_bt_navigator` (`navigate_to_pose`), `nav2_behavior_tree`, BehaviorTree.CPP 4.10 control nodes | D5 |
| `src/nav_stack.rs` | the three servers' threads and loop rates, on one clock | D5 |
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

## Controller server (D3, `follow_path`)

`ControllerServer::mower()` is `controller_server` with that FollowPath, the
`general_goal_checker` and the `progress_checker`: what mower_nav sends every
coverage segment to. It is a state machine: `send_goal` / `cancel_goal` are
the action client's requests, and one `tick(&CycleInput)` is one pass of
`computeControl`'s loop. A pass gets the time, whether the costmap is
current, the robot pose in odom, the odometry twist, `odom <- map`, and the
local costmap. It returns the `cmd_vel` commands, the feedback and the
results of goals that ended.

* Ported line by line:
  * The order of a pass: cancel, then the costmap wait
    (`costmap_update_timeout` 1 s, `ControllerTimedOut`), then preemption,
    then `isGoalReached`, then the progress check, the controller and the
    feedback.
  * The thresholded odometry twist (`min_y_velocity_threshold: 0.5` zeroes
    `y`).
  * `failure_tolerance` (2 s): `NoValidControl` publishes a zero, and after
    more than 2 s since the last valid command the goal ends with
    `PatienceExceeded`.
  * Every exception's error code, then `publishZeroVelocity` (a zero, then the
    controller's `reset`).
* The SimpleActionServer around it is kept, quirks included (module doc):
  * A preempted goal is aborted with an empty result, error code 0.
  * Preemption resets the goal checker, but not the progress checker or the
    patience clock.
  * A second pending goal replaces the first.
  * Cancelling the pending goal also ends the running one; cancelling the
    running goal while one is pending is ignored until the preemption.
  * A goal that fails while it is being cancelled ends `Canceled`, with the
    failure's code.
* Deliberate change: the feedback's `distance_to_goal` measures from the plan
  pose closest to the robot *in map*. Upstream compares the odom pose with map
  poses, so its distance shifts with the `map -> odom` offset, and mower_nav
  turns that distance into coverage progress.
* For the wiring (D6):
  * Tick right after `send_goal`, then every 50 ms while `is_executing()`;
    upstream's worker thread starts a goal at once and keeps its own 20 Hz
    from there.
  * `Costmap2DROS::getRobotPose` takes the latest transform without an age
    check, so a stalled `odom -> base_footprint` freezes the pose instead of
    failing. Pass the latest pose, and `None` only when there has never been
    one.
  * The local costmap updates at 2 Hz in the yaml (A4); a pass may get a
    fresher one.
* Not ported: lifecycle, the action server itself, tf, the odometry
  subscriber, the `speed_limit` subscription (`set_speed_limit`), dynamic
  parameters, `controller_id` / `goal_checker_id` / `progress_checker_id`
  (one of each, the ids mower_nav sends), `error_msg` (1.3.13 leaves it
  empty).

Checked two ways:

* `tests/controller_server.rs`: every branch against a scripted controller,
  with exact tick counts for the patience (aborted on the pass at 2.05 s) and
  the progress checker (20.05 s).
* `tests/controller_server_oracle.rs`, black box against the real
  `controller_server`:
  * `tests/oracle/controller_server/scenarios.py` plays
    `scenarios.json` on the real node with mower_nav2's yaml: map, tf,
    `/odom` and a kinematic robot following `/cmd_vel`.
  * The test plays the same scenarios on the port with a simulated clock.
  * The 11 scenarios: reach the end, already at the goal, empty path, cancel,
    preemption by the same path and by an empty one, a replaced pending goal,
    cancels around a pending goal, a wall across the path, no progress.
  * All 15 goals end with the same status and error code. Result times
    agree within 0.06 s, and the robot ends within 1.1 cm of where nav2
    left it.
  * The closed loop also shows the RPP findings above: the wall run stops
    0.77 m short and aborts 2.05 s after the last command; the 1.5 m segment
    ends 4.8 cm short of its end, inside the 5 cm goal tolerance.

## Planner server, navigator and the stack (D2, D5: `navigate_to_pose`)

mower_nav sends one `navigate_to_pose` goal per mission, to the coverage start
(module docs for the details):

* `planner_server.rs`: `compute_path_to_pose` around NavFn. The order of
  `computePlan` (cancel, the costmap wait with `costmap_update_timeout` 1 s,
  preemption, start pose, plugin id, `createPlan`, the empty-path check), its
  error codes 200–208, and the SimpleActionServer around it. One quirk kept:
  a goal that preempts during the costmap wait receives the plan of the goal
  it preempted (upstream assigns the pending goal to a copy of the pointer).
* `navigator.rs` (D5): `bt_navigator` with nav2's default
  `navigate_to_pose_w_replanning_and_recovery.xml`, ported node by node,
  **without Spin, Wait and BackUp** (`docs/ROS_FREE_PLAN.md` §8):
  * The tree plans at 1 Hz, and every new plan preempts the FollowPath goal.
  * A failed plan is retried once after clearing the global costmap, and a
    failed FollowPath once after clearing the local one.
  * Then the whole pipeline is retried after clearing both, up to 6 times.
  * Upstream's outer recovery cycles through clear, Spin, Wait and BackUp;
    here every recovery is the clear. Until the second recovery the two
    trees are the same.
  * The tree as XML: `tests/oracle/navigator/navigate_to_pose_w_replanning_and_clearing.xml`
    (the default with three lines deleted). Setting it as
    `default_nav_to_pose_bt_xml` makes nav2 on the robot behave like the port
    before D6 switches over.

  Quirks kept:
  * The result code is the smallest nonzero of the two error codes. A
    FollowPath code beats a planner code, even a stale one: an old 104 keeps
    the recovery going after a later planner error that no recovery helps.
  * Every clear counts as a recovery.
  * `GoalUpdated` never fires (BehaviorTree.CPP 4.10 resets it every tick),
    so a preempting goal is planned to at the next replan.
* `nav_stack.rs`: the three servers on one clock, each scheduled the way its
  thread runs upstream.
  * Rates: the tree every 10 ms; the controller at 20 Hz from its loop's
    start; the planner as soon as it has a goal. Both servers poll at 100 Hz
    while they wait for a costmap.
  * The loop rates are `WallRate`s, overrun reset included.
  * Results go back to whoever sent the goal. mower_nav's own `follow_path`
    goals (`send_follow_path`) share the one controller server, as they do
    upstream.
  * The costmaps belong to the caller. A cleared costmap must stay not
    current until its next update (the global one updates at 1 Hz, which is
    why every retry after a clear waits up to a second).
* For the wiring (D6): tick at least every 10 ms (`next_tick_ns`). Pass
  `map <- base` and `odom <- base` as the latest transforms, and apply
  `StackOutput::clears` to the costmaps. `StackOutput::log` has what the
  servers would have logged at WARN or ERROR (tolerated `NoValidControl`,
  the exceptions that ended goals, the planner's missed rate).
* Not ported: per-goal `behavior_tree`, the feedback's
  `estimated_time_remaining` (no consumer), the result-callback wake-up
  (a result is seen at the next tick, ≤ 10 ms later), service and goal
  response timeouts.

Checked:

* `tests/planner_server.rs`: the wait, the timeout, every error code, cancel
  (including the quirk that a plan with the robot at the goal succeeds while
  cancelled), and the preemption quirk.
* `tests/navigator.rs`: the tree against scripted servers. It pins down:
  * the exact tick counts (each plan is seen one tick after it was asked
    for, and the 1 Hz timer restarts then);
  * 7 × 2 plans and 19 clears for NO_VALID_PATH;
  * 7 × 2 FollowPath attempts for PATIENCE_EXCEEDED;
  * the stale-code quirk, cancel during planning, preemption, the feedback
    distance.
* `tests/navigator_oracle.rs`, black box against the real `bt_navigator`,
  `planner_server` and `controller_server`, with `behavior_server` for the
  default tree:
  * `tests/oracle/navigator/scenarios.py` plays 9 scenarios on the robot's
    yaml. It records each goal's result, the last feedback, every
    compute_path_to_pose and follow_path goal, `/plan`, and when the two
    costmaps updated.
  * The test replays them on `NavStack`, with the costmaps updating at the
    phases nav2's had.
  * Every status and error code matches, and so does every recovery count
    (19 in the two retry scenarios), plan count and number of
    compute_path_to_pose goals. Result times agree within 0.18 s (35.07 s vs
    35.05 s for the 7-round wall scenario), and the robot ends within 6 mm.
  * The scenarios that never reach a second recovery also match nav2's
    **default** tree: reach, cancel, preempt, goal occupied (206).
  * For comparison, the default tree with its motion recoveries: unreachable
    takes 17.3 s instead of 12.7 s and the robot turns about 180° and backs
    up; the wall takes 47.5 s instead of 35.1 s. Both end with the same code.
  * One count may differ in the wall scenario. A replan reaches nav2's tree
    10–30 ms after it is asked for, and here one tick later, so whether the
    last replan of a round lands before that round's FollowPath failure can
    flip. The port then sends one more preempting FollowPath goal per round
    (43 vs 38 over 7 rounds); the outcome does not change.

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
