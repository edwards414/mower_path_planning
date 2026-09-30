//! Port of mower_mission/test/test_connector_planner.py (commit dff480b).
//! Test names are the Python ones without the `test_` prefix. The Python
//! `plan_connector` defaulted boundary_weight to 0.2; the Rust function takes
//! it explicitly, so these tests pass 0.2.

mod common;

use common::*;
use mower_coverage_core::connector_planner::plan_connector_rs;
use mower_coverage_core::path_validator::validate_path_rs;

const BOUNDARY_WEIGHT: f64 = 0.2;

// ── Basic reachability ─────────────────────────────────────────────────────────

#[test]
fn straight_path_returns_nonempty() {
    let grid = ones(20, 20);
    let path = plan_connector_rs((0.05, 0.05), (1.95, 1.95), &safe_map(&grid), BOUNDARY_WEIGHT);
    let path = path.expect("path on an open grid");
    assert!(path.len() >= 2);
}

#[test]
fn start_equals_end_returns_two_point_path() {
    let grid = ones(20, 20);
    let path = plan_connector_rs((0.55, 0.55), (0.55, 0.55), &safe_map(&grid), BOUNDARY_WEIGHT);
    let path = path.expect("same-cell path");
    assert!(!path.is_empty());
}

#[test]
fn start_on_unsafe_cell_returns_none() {
    let mut grid = ones(20, 20);
    grid[[0, 0]] = false;
    assert!(plan_connector_rs((0.05, 0.05), (1.95, 1.95), &safe_map(&grid), BOUNDARY_WEIGHT).is_none());
}

#[test]
fn end_on_unsafe_cell_returns_none() {
    let mut grid = ones(20, 20);
    grid[[19, 19]] = false;
    assert!(plan_connector_rs((0.05, 0.05), (1.95, 1.95), &safe_map(&grid), BOUNDARY_WEIGHT).is_none());
}

#[test]
fn disconnected_returns_none() {
    // Left and right halves separated by a full-height wall.
    let mut grid = ones(20, 20);
    set(&mut grid, 0..20, 10..11, false);
    assert!(plan_connector_rs((0.55, 1.05), (1.55, 1.05), &safe_map(&grid), BOUNDARY_WEIGHT).is_none());
}

// ── Path quality ───────────────────────────────────────────────────────────────

#[test]
fn path_avoids_wall() {
    // Opposite sides of a wall with a gap at rows 18-19.
    let mut grid = ones(20, 20);
    set(&mut grid, 0..18, 10..11, false);
    let path = plan_connector_rs((0.55, 0.55), (1.55, 0.55), &safe_map(&grid), BOUNDARY_WEIGHT);
    let path = path.expect("should find a path through the gap");
    assert!(path.len() >= 3);
}

#[test]
fn all_path_points_are_safe() {
    // Square obstacle in the centre. (Python skipped the check when no path was
    // found; a path exists here, so the port asserts it.)
    let mut grid = ones(30, 30);
    set(&mut grid, 10..20, 10..20, false);
    let path = plan_connector_rs((0.55, 1.55), (2.95, 1.55), &safe_map(&grid), BOUNDARY_WEIGHT)
        .expect("path around the obstacle");
    for &(x, y) in &path {
        let (row, col) = pt_to_cell(x, y);
        assert!(grid[[row as usize, col as usize]], "point ({x:.2},{y:.2}) on unsafe cell");
    }
}

#[test]
fn connector_passes_path_validator() {
    // Narrow wall with a gap at the bottom. (Python skipped when no path.)
    let mut grid = ones(30, 30);
    set(&mut grid, 5..25, 14..16, false);
    set(&mut grid, 24..30, 14..16, true);
    let sm = safe_map(&grid);
    let path = plan_connector_rs((0.55, 1.55), (2.55, 1.55), &sm, BOUNDARY_WEIGHT).expect("path");
    let result = validate_path_rs(&path, &sm);
    assert!(result.valid, "connector path invalid: {}", result.message);
}

#[test]
fn connector_does_not_cut_diagonal_obstacle_corner() {
    let mut grid = ones(5, 5);
    grid[[1, 2]] = false;
    grid[[2, 1]] = false;
    let sm = safe_map(&grid);
    // start row=1,col=1; end row=3,col=3
    let path = plan_connector_rs((0.15, 0.15), (0.35, 0.35), &sm, BOUNDARY_WEIGHT).expect("path");
    let result = validate_path_rs(&path, &sm);
    assert!(result.valid, "connector cut a blocked corner: {}", result.message);
}

// ── U-shape integration ────────────────────────────────────────────────────────

#[test]
fn u_shape_connector_between_arms() {
    // Open-top U: the connector between the arm tops must go around the bottom.
    let mut grid = zeros(30, 30);
    set(&mut grid, 0..20, 5..10, true); // left arm
    set(&mut grid, 0..20, 20..25, true); // right arm
    set(&mut grid, 20..25, 5..25, true); // connecting bottom
    let sm = safe_map(&grid);

    let start = (OX + 7.5 * RES, OY + 1.5 * RES); // col 7, row 1
    let end = (OX + 22.5 * RES, OY + 1.5 * RES); // col 22, row 1
    let path = plan_connector_rs(start, end, &sm, BOUNDARY_WEIGHT).expect("should find path through the bottom");
    let result = validate_path_rs(&path, &sm);
    assert!(result.valid, "U-shape connector invalid: {}", result.message);
}
