# Spiral / Zigzag Coverage Pattern Plan

> **Note (2026-09-30):** the Python implementation this plan describes
> (`coverage_node.py`, `path_generators/`, `coverage/`) was removed on
> 2026-09-30. The logic now lives in Rust:
>
> - `coverage/path_validator.py` → `src/mower_coverage_core/src/path_validator.rs`
> - `coverage/safe_map_filter.py` → `src/mower_coverage_core/src/safe_map_filter.rs`
> - `coverage/connector_planner.py` → `src/mower_coverage_core/src/connector_planner.rs`
> - `path_generators/zigzag.py` → `src/mower_coverage_core/src/zigzag.rs`
> - `path_generators/spiral.py` (legacy name `speiral.py`) → `src/mower_coverage_core/src/spiral.rs`
> - `coverage/cell_decomposition.py` → `src/mower_coverage_core/src/cell_decomposition.rs` (ported, not wired into the node yet)
> - `coverage/types.py`: `CoverageCell` → `src/mower_coverage_core/src/cell_decomposition.rs`; `SpiralCoveragePlan` / `SpiralSegment` / `CoverageSegment` / `ConnectorSegment` / `RoutePlan` were not ported (the Rust spiral returns `(points, split_points, invalid_segments)`); `SafeMap` / `ValidationResult` (from `coverage/path_validator.py`) → `src/mower_coverage_core/src/types.rs`
> - `coverage_node.py` → `src/mower_rs/crates/mower_coverage/src/lib.rs` (`mower_rs`'s `mower_coverage`; the node name is still `boustrophedon_coverage`, with the same services and parameters)
>
> The decisions below are kept as written; file names refer to the Python code
> of that time.

## Decision

Use `zigzag` as the UI-facing and generator-facing name for the
back-and-forth coverage pattern.

`boustrophedon` and `zigzag` describe the same back-and-forth coverage family
in this project.  To avoid duplicate implementations, keep `zigzag.py` and
delete `boustrophedon.py`.

The ROS node/service name `boustrophedon_coverage` can remain for now because
it is part of the existing launch and service wiring.

## Coverage Modes

Supported user-facing modes:

- `zigzag`: back-and-forth strip coverage.
- `spiral`: onion-layer spiral coverage.

## Qt Behavior

The Qt coverage mode selector shows:

- `zigzag`
- `spiral`

When the user presses `生成Coverage Path`, Qt first applies the current
coverage parameters to ROS:

- `coverage_pattern`
- `strip_width_m`
- `waypoint_spacing_m`
- `unknown_as_obstacle`
- `inflate_radius_m`

After parameter application succeeds, Qt calls:

```text
/generate_coverage_path
```

## Backend Behavior

`coverage_node.py` reads:

```python
coverage_pattern
```

Pattern selection:

```python
if pattern == "spiral":
    use_spiral()
else:
    use_zigzag()
```

The rest of the safety pipeline stays shared:

1. Build `safe_map` from inflated zone and risk maps.
2. Filter safe components.
3. Generate the selected coverage pattern.
4. Detect unsafe direct point-to-point segments.
5. Repair unsafe transitions with `ConnectorPlanner`.
6. Run final path validation.
7. Publish only if the final path is safe.

## File Decision

Keep:

- `src/mower_mission/mower_mission/path_generators/zigzag.py`
- `src/mower_mission/mower_mission/path_generators/speiral.py`

Delete:

- `src/mower_mission/mower_mission/path_generators/boustrophedon.py`

Reason:

- Qt uses `zigzag`.
- `coverage_pattern` uses `zigzag`.
- The back-and-forth implementation lives in `zigzag.py`.
- Keeping both `boustrophedon.py` and `zigzag.py` would duplicate the same
  behavior and make future fixes easy to apply in the wrong file.
