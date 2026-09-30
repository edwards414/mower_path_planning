# mower_coverage_core

The coverage path planning library of the lawn mower. It is a plain Rust
library crate with no ROS, PyO3 or colcon packaging.

| Module | What it does |
| --- | --- |
| `safe_map_filter` | Drops small 4-connected safe-map fragments (`filter_safe_components_rs`) |
| `path_validator` | Checks points and Bresenham segments against the safe grid (`validate_path_rs`) |
| `connector_planner` | Boundary-aware 8-connected A* connector with no corner cutting (`plan_connector_rs`) |
| `zigzag` | Legacy generator, pinned by the oracle; not used by the node (its rotated branch is broken) |
| `spiral` | Onion-layer spiral for each connected component (`plan_spiral_coverage_rs`) |
| `boustrophedon` | The zigzag planner the node uses: straight lanes at any angle, cells, optimised cell order, optional angle search (`plan_boustrophedon_rs`); also `coverage_ratio_rs`, `simplify_path_rs` |
| `cell_decomposition` | Row-sweep boustrophedon cell decomposition (`decompose`, `CoverageCell`); ported from Python, not used by the node (`boustrophedon` builds its own cells in the sweep frame, at any angle) |
| `types` | `SafeMap`, `ValidationResult` and the world/grid conversions |

## Who uses it

The only consumer is the mower_rs coverage node
`src/mower_rs/crates/mower_coverage` (ROS node `boustrophedon_coverage`). It
links this crate through a cargo path dependency. That node is the only
coverage planner. It runs as its own binary, or as the `coverage` module of
`mower_rsd`.

`cell_decomposition` is not wired into the node yet. It keeps the BCD work
for the planned per-cell coverage optimisation.

These functions are ports of the Python planner that used to live in
`src/mower_mission` (`coverage/`, `path_generators/`). That Python code was
removed after commit `dff480b`.

## Tests

```sh
cd src/mower_coverage_core
cargo test --locked
```

- `tests/python_reference_oracle.rs` replays the inputs in
  `tests/data/python_reference_oracle.json` and requires the outputs recorded
  from the Python implementation at `dff480b`. Integers and bools must match
  exactly and floats must be within 1e-9. The inputs are the old conftest
  fixtures, the old parity-test inputs and 16 seeded random maps. The file
  header lists the caveats, including that the rotated-zigzag cases pin the
  current, known-poor behaviour. `tests/data/gen_python_reference_oracle.py`
  regenerates the file byte for byte from a checkout of `dff480b` (see its
  docstring); a deliberate behaviour change replaces the affected cases.
- `tests/{path_validator,safe_map_filter,connector_planner,zigzag,spiral,cell_decomposition,backend_api,backend_parity}.rs`
  are ports of the former `mower_mission/test/test_*.py` unit tests. Each
  file's header names the pytest file it came from and any test that has no
  Rust equivalent.

No ROS installation is needed. CI runs the same command (job
`coverage-core-tests` in `.github/workflows/build.yml`).
