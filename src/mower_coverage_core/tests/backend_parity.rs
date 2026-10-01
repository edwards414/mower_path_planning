//! Port of mower_mission/test/test_backend_parity.py (commit dff480b).
//!
//! The Python file compared the Python backend with the Rust backend (through
//! the removed PyO3 module) on the same inputs. The Python side of every one
//! of those inputs was recorded in tests/data/python_reference_oracle.json
//! under ids starting with `parity/`, so each test below runs the Rust
//! function on that recorded input and requires the Python output exactly
//! (floats within 1e-9), which is stronger than the original equal-count /
//! equal-validity checks, and then repeats the original's Rust-only
//! assertions. Test names: `<python class>_<python test>` without `Test` /
//! `test_`, in snake case.

mod common;

use common::*;
use mower_coverage_core::path_validator::validate_path_rs;
use mower_coverage_core::spiral::plan_spiral_coverage_rs;
use mower_coverage_core::zigzag::generate_coverage_zigzag_path_rs;

/// `(points, split_points, invalid_segments)`
type Plan = (Vec<(f64, f64)>, Vec<(f64, f64)>, Vec<(usize, usize)>);

fn zigzag_rs(map: &str, angle_deg: f64) -> Plan {
    let m = oracle_map(map);
    let (h, w) = m.grid.dim();
    generate_coverage_zigzag_path_rs(m.grid.view(), 0.2, 0.1, m.res, h, w, m.ox, m.oy, angle_deg)
}

fn spiral_rs(map: &str) -> Plan {
    let m = oracle_map(map);
    let (h, w) = m.grid.dim();
    plan_spiral_coverage_rs(m.grid.view(), 0.4, 0.2, m.res, h, w, m.ox, m.oy)
}

// ── PathValidator parity ───────────────────────────────────────────────────────

#[test]
fn path_validator_parity_is_point_safe_open() {
    for x in ["0.05", "0.55", "0.95"] {
        for y in ["0.05", "0.55", "0.95"] {
            assert_case("validate", &format!("parity/validate/point_open/{x}_{y}"), check_validate);
        }
    }
}

#[test]
fn path_validator_parity_is_point_unsafe() {
    assert_case("validate", "parity/validate/point_unsafe", check_validate);
    let m = oracle_map("open10_hole55");
    assert!(!validate_path_rs(&[(0.55, 0.55)], &m.sm()).valid);
}

#[test]
fn path_validator_parity_segment_crossing_wall() {
    assert_case("validate", "parity/validate/segment_crossing_wall", check_validate);
}

#[test]
fn path_validator_parity_full_path_open_grid() {
    assert_case("validate", "parity/validate/full_path_open_grid", check_validate);
}

#[test]
fn path_validator_parity_out_of_bounds_point() {
    assert_case("validate", "parity/validate/out_of_bounds_point", check_validate);
}

#[test]
fn path_validator_parity_negative_origin() {
    assert_case("validate", "parity/validate/negative_origin", check_validate);
}

// ── SafeMapFilter parity ───────────────────────────────────────────────────────

#[test]
fn safe_map_filter_parity_single_component() {
    assert_case("filter", "parity/filter/single_component", check_filter);
}

#[test]
fn safe_map_filter_parity_removes_tiny_island() {
    assert_case("filter", "parity/filter/removes_tiny_island", check_filter);
}

#[test]
fn safe_map_filter_parity_keeps_largest_only() {
    assert_case("filter", "parity/filter/keeps_largest_only", check_filter);
}

#[test]
fn safe_map_filter_parity_empty_grid() {
    assert_case("filter", "parity/filter/empty_grid", check_filter);
}

// ── ConnectorPlanner parity ────────────────────────────────────────────────────

fn connector_path(id: &str) -> Option<Vec<(f64, f64)>> {
    let c = case("connector", id);
    let m = case_map(c);
    mower_coverage_core::connector_planner::plan_connector_rs(
        point(&c["start"]),
        point(&c["end"]),
        &m.sm(),
        num(&c["boundary_weight"]),
    )
}

#[test]
fn connector_planner_parity_open_grid_both_find_path() {
    let id = "parity/connector/open_grid";
    assert_case("connector", id, check_connector);
    let path = connector_path(id).expect("rust connector path");
    let result = validate_path_rs(&path, &oracle_map("fixture_open").sm());
    assert!(result.valid, "rust connector invalid: {}", result.message);
}

#[test]
fn connector_planner_parity_disconnected_both_return_none() {
    let id = "parity/connector/disconnected";
    assert_case("connector", id, check_connector);
    assert!(connector_path(id).is_none());
}

#[test]
fn connector_planner_parity_wall_with_gap() {
    let id = "parity/connector/wall_with_gap";
    assert_case("connector", id, check_connector);
    let path = connector_path(id).expect("path through the gap");
    assert!(validate_path_rs(&path, &oracle_map("fixture_narrow_passage").sm()).valid);
}

#[test]
fn connector_planner_parity_unsafe_start_returns_none() {
    let id = "parity/connector/unsafe_start";
    assert_case("connector", id, check_connector);
    assert!(connector_path(id).is_none());
}

#[test]
fn connector_planner_parity_no_corner_cutting() {
    let id = "parity/connector/no_corner_cutting";
    assert_case("connector", id, check_connector);
    if let Some(path) = connector_path(id) {
        assert!(validate_path_rs(&path, &oracle_map("corner5").sm()).valid);
    }
}

// ── Zigzag generator parity (strip 0.2, spacing 0.1) ──────────────────────────

#[test]
fn zigzag_parity_empty_map_both_empty() {
    assert_case("zigzag", "parity/zigzag/empty_map", check_zigzag);
    let (pts, sp, inv) = zigzag_rs("empty10", 0.0);
    assert!(pts.is_empty() && sp.is_empty() && inv.is_empty());
}

#[test]
fn zigzag_parity_open_grid_same_points() {
    assert_case("zigzag", "parity/zigzag/open_grid", check_zigzag);
}

#[test]
fn zigzag_parity_rotated_open_grid_runs_for_both_backends() {
    assert_case("zigzag", "parity/zigzag/rotated_open_grid", check_zigzag);
    let (pts, sp, inv) = zigzag_rs("fixture_open", 45.0);
    assert!(pts.len() >= 2);
    assert!(!sp.is_empty());
    assert!(inv.is_empty());
}

#[test]
fn zigzag_parity_all_rust_waypoints_on_safe_cells() {
    // Waypoints must not land on obstacle cells (segments may still be invalid).
    assert_case("zigzag", "parity/zigzag/obstacle", check_zigzag);
    let m = oracle_map("fixture_rectangle_obstacle");
    let (pts, _, _) = zigzag_rs("fixture_rectangle_obstacle", 0.0);
    for &(x, y) in &pts {
        let (row, col) = pt_to_cell(x, y);
        if in_grid(&m.grid, row, col) {
            assert!(m.grid[[row as usize, col as usize]], "waypoint ({x:.3},{y:.3}) on unsafe cell");
        }
    }
}

#[test]
fn zigzag_parity_invalid_segments_match() {
    assert_case("zigzag", "parity/zigzag/wall", check_zigzag);
}

#[test]
fn zigzag_parity_split_points_subset_of_points() {
    assert_case("zigzag", "parity/zigzag/open_grid", check_zigzag);
    let (pts, sp, _) = zigzag_rs("fixture_open", 0.0);
    for s in &sp {
        assert!(pts.contains(s), "split point {s:?} not in points");
    }
}

// ── Spiral generator parity (strip 0.4, spacing 0.2) ──────────────────────────

#[test]
fn spiral_parity_empty_map_both_empty() {
    assert_case("spiral", "parity/spiral/empty_map", check_spiral);
    let (pts, sp, _) = spiral_rs("empty10");
    assert!(pts.is_empty() && sp.is_empty());
}

#[test]
fn spiral_parity_open_grid_nonempty() {
    assert_case("spiral", "parity/spiral/open_grid", check_spiral);
    let (pts, sp, _) = spiral_rs("fixture_open");
    assert!(pts.len() >= 2);
    assert!(!sp.is_empty());
}

#[test]
fn spiral_parity_all_rust_points_safe() {
    let (pts, _, _) = spiral_rs("fixture_open");
    assert!(!pts.is_empty());
    let result = validate_path_rs(&pts, &oracle_map("fixture_open").sm());
    assert!(result.valid, "rust spiral has unsafe path: {}", result.message);
}

#[test]
fn spiral_parity_two_islands_split_points_match_count() {
    // Full equality with the recorded Python output (was: equal counts, 2).
    assert_case("spiral", "parity/spiral/two_islands", check_spiral);
    assert_eq!(spiral_rs("fixture_two_islands").1.len(), 2);
}

#[test]
fn spiral_parity_invalid_segs_count_not_worse() {
    // Full equality with the recorded Python output (was: rust <= python + 2).
    let c = case("spiral", "parity/spiral/two_islands");
    let (_, _, inv) = spiral_rs("fixture_two_islands");
    assert!(inv.len() <= pairs(&c["invalid_segments"]).len() + 2);
    assert_eq!(inv, pairs(&c["invalid_segments"]));
}
