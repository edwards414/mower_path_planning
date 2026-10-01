//! Port of mower_mission/test/test_path_validator.py (commit dff480b).
//! One `#[test]` per Python test function, same names without the `test_`
//! prefix. RES = 0.1, origin (0, 0).

mod common;

use common::*;
use mower_coverage_core::path_validator::{is_point_safe_rs, is_segment_safe_rs, validate_path_rs};

// ── is_point_safe ──────────────────────────────────────────────────────────────

#[test]
fn point_inside_safe_cell() {
    let grid = ones(10, 10);
    assert!(is_point_safe_rs(0.05, 0.05, &safe_map(&grid))); // centre of cell (0,0)
}

#[test]
fn point_on_unsafe_cell() {
    let mut grid = ones(10, 10);
    grid[[2, 3]] = false;
    // cell (row=2, col=3) -> world x=0.35, y=0.25 (centre)
    assert!(!is_point_safe_rs(0.35, 0.25, &safe_map(&grid)));
}

#[test]
fn point_outside_grid_returns_false() {
    let grid = ones(10, 10);
    let sm = safe_map(&grid);
    assert!(!is_point_safe_rs(-0.5, 0.05, &sm));
    assert!(!is_point_safe_rs(0.05, 5.0, &sm));
}

// ── is_segment_safe ────────────────────────────────────────────────────────────

#[test]
fn segment_within_safe_area() {
    let grid = ones(20, 20);
    assert!(is_segment_safe_rs((0.05, 0.05), (1.95, 1.95), &safe_map(&grid)));
}

#[test]
fn segment_crossing_obstacle() {
    // A segment that crosses a vertical wall of unsafe cells at col 10.
    let mut grid = ones(20, 20);
    set(&mut grid, 0..20, 10..11, false);
    assert!(!is_segment_safe_rs((0.5, 1.0), (1.5, 1.0), &safe_map(&grid)));
}

#[test]
fn segment_along_safe_corridor() {
    // Horizontal segment through a single safe row.
    let mut grid = zeros(10, 20);
    set(&mut grid, 5..6, 0..20, true);
    assert!(is_segment_safe_rs((0.05, 0.55), (1.95, 0.55), &safe_map(&grid)));
}

// ── validate_path ──────────────────────────────────────────────────────────────

#[test]
fn validate_path_all_valid_rectangle() {
    let grid = ones(10, 10);
    let points = [(0.05, 0.05), (0.05, 0.95), (0.95, 0.95), (0.95, 0.05)];
    let result = validate_path_rs(&points, &safe_map(&grid));
    assert!(result.valid);
    assert!(result.invalid_points.is_empty());
    assert!(result.invalid_segments.is_empty());
}

#[test]
fn validate_path_detects_crossing() {
    // A segment that jumps across a wall must appear in invalid_segments.
    let mut grid = ones(20, 20);
    set(&mut grid, 0..20, 10..11, false);
    let points = [(0.55, 1.05), (1.45, 1.05)];
    let result = validate_path_rs(&points, &safe_map(&grid));
    assert!(!result.valid);
    assert!(result.invalid_segments.contains(&(0, 1)));
}

#[test]
fn validate_path_detects_unsafe_point() {
    let mut grid = ones(10, 10);
    grid[[5, 5]] = false;
    let unsafe_world = (0.55, 0.55); // cell row=5, col=5
    let points = [(0.05, 0.05), unsafe_world, (0.95, 0.05)];
    let result = validate_path_rs(&points, &safe_map(&grid));
    assert!(!result.valid);
    assert!(result.invalid_points.contains(&1));
}

fn u_arms_grid() -> ndarray::Array2<bool> {
    let mut grid = zeros(30, 20);
    set(&mut grid, 0..26, 2..8, true); // left arm
    set(&mut grid, 0..26, 12..18, true); // right arm
    grid
}

#[test]
fn validate_path_u_shape_no_crossing() {
    // A path inside the left arm only is valid.
    let grid = u_arms_grid();
    let left_arm_pts: Vec<(f64, f64)> = (0..26).map(|row| (0.25 + 0.05, row as f64 * RES + 0.05)).collect();
    let result = validate_path_rs(&left_arm_pts, &safe_map(&grid));
    assert!(result.valid, "{}", result.message);
}

#[test]
fn validate_path_u_shape_crossing_detected() {
    // A direct connector between the arms crosses empty space.
    let grid = u_arms_grid();
    let result = validate_path_rs(&[(0.5, 1.05), (1.45, 1.05)], &safe_map(&grid));
    assert!(!result.valid);
    assert!(result.invalid_segments.contains(&(0, 1)));
}

/// A point in the one-cell band left of or below the grid is outside, not
/// row/column 0 (the Python reference truncated towards zero and passed it).
#[test]
fn points_just_left_of_or_below_the_grid_are_unsafe() {
    let g = ones(10, 10);
    let sm = safe_map(&g);
    for p in [(-0.05, 0.5), (0.5, -0.05), (-0.01, -0.01)] {
        let r = validate_path_rs(&[p], &sm);
        assert!(!r.valid, "{p:?} passed");
    }
    assert!(validate_path_rs(&[(0.0, 0.0), (0.99, 0.99)], &sm).valid);
}
