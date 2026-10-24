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

Integer results (cells, costs, indices) must match exactly. Floats are compared
to 1e-12 relative: the nav2 binaries are GCC aarch64 builds, which contract
`a + b * c` into FMA (`-ffp-contract=fast`, seen in `Costmap2D::updateOrigin`)
and use glibc's libm (`cos(22.5 deg)` differs in the last bit from macOS), and
this crate does neither.
