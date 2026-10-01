//! Port of mower_mission/test/test_backend_api.py (commit dff480b).
//!
//! The Python file tested the pure-Python `PythonBackend` contract; the same
//! contract now belongs to the Rust functions the coverage node calls
//! (`generate_coverage_zigzag_path_rs`, `plan_spiral_coverage_rs`). Test names
//! are the Python ones without the `test_` prefix (`python_backend_` dropped).

mod common;

use common::*;
use mower_coverage_core::spiral::plan_spiral_coverage_rs;
use mower_coverage_core::zigzag::generate_coverage_zigzag_path_rs;
use ndarray::Array2;

// _planning_kwargs: strip 0.4, res 0.1, origin (0, 0).
const STRIP: f64 = 0.4;

fn zigzag(grid: &Array2<bool>, waypoint_spacing_m: f64, angle_deg: f64) -> Vec<(f64, f64)> {
    let (h, w) = grid.dim();
    generate_coverage_zigzag_path_rs(grid.view(), STRIP, waypoint_spacing_m, RES, h, w, OX, OY, angle_deg).0
}

fn spiral(grid: &Array2<bool>, waypoint_spacing_m: f64) -> Vec<(f64, f64)> {
    let (h, w) = grid.dim();
    plan_spiral_coverage_rs(grid.view(), STRIP, waypoint_spacing_m, RES, h, w, OX, OY).0
}

#[test]
fn accepts_documented_zigzag_parameters() {
    let grid = ones(30, 30);
    let axis_points = zigzag(&grid, 0.1, 0.0);
    let rotated_points = zigzag(&grid, 0.1, 45.0);
    assert!(!axis_points.is_empty());
    assert!(!rotated_points.is_empty());
    assert_ne!(axis_points, rotated_points);
}

#[test]
fn waypoint_spacing_controls_zigzag_sampling_density() {
    let grid = ones(30, 30);
    let dense = zigzag(&grid, 0.1, 0.0);
    let sparse = zigzag(&grid, 0.4, 0.0);
    assert!(dense.len() > sparse.len());
}

#[test]
fn waypoint_spacing_controls_spiral_sampling_density() {
    let grid = ones(30, 30);
    let dense = spiral(&grid, 0.1);
    let sparse = spiral(&grid, 0.4);
    assert!(dense.len() > sparse.len());
}
