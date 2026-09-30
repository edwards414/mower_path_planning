//! Port of mower_mission/test/test_zigzag_generator.py and
//! test_zigzag_segment_boundaries.py (commit dff480b). Test names are the
//! Python ones without the `test_` prefix. RES = 0.1, origin (0, 0),
//! STRIP = 0.2, SPACING = 0.1 unless a test says otherwise.
//!
//! Not ported: `test_returns_three_tuple` (the Rust signature returns the
//! `(points, split_points, invalid_segments)` tuple, so it is a compile-time
//! guarantee).

mod common;

use common::*;
use mower_coverage_core::path_validator::validate_path_rs;
use mower_coverage_core::types::SafeMap;
use mower_coverage_core::zigzag::generate_coverage_zigzag_path_rs;
use ndarray::Array2;

const STRIP: f64 = 0.2;
const SPACING: f64 = 0.1;

type Plan = (Vec<(f64, f64)>, Vec<(f64, f64)>, Vec<(usize, usize)>);

fn call(grid: &Array2<bool>, angle_deg: f64) -> Plan {
    let (h, w) = grid.dim();
    generate_coverage_zigzag_path_rs(grid.view(), STRIP, SPACING, RES, h, w, OX, OY, angle_deg)
}

fn call_with_spacing(grid: &Array2<bool>, strip_width_m: f64, waypoint_spacing_m: f64) -> Plan {
    let (h, w) = grid.dim();
    generate_coverage_zigzag_path_rs(grid.view(), strip_width_m, waypoint_spacing_m, RES, h, w, OX, OY, 0.0)
}

// ── Return contract ────────────────────────────────────────────────────────────

#[test]
fn empty_map_returns_empty() {
    let (pts, split_pts, invalid_segs) = call(&zeros(10, 10), 0.0);
    assert!(pts.is_empty());
    assert!(split_pts.is_empty());
    assert!(invalid_segs.is_empty());
}

#[test]
fn rectangle_returns_nonempty() {
    let (pts, split_pts, _) = call(&ones(20, 20), 0.0);
    assert!(pts.len() >= 2);
    assert!(!split_pts.is_empty());
}

#[test]
fn accepts_uint8_safe_map() {
    // Python accepted a uint8 grid; the Rust API takes bool, so the caller
    // converts (non-zero = safe) and the result contract must hold.
    let grid = Array2::<u8>::ones((20, 20)).mapv(|v| v != 0);
    let (pts, split_pts, invalid_segs) = call(&grid, 0.0);
    assert!(pts.len() >= 2);
    assert!(!split_pts.is_empty());
    assert!(invalid_segs.is_empty());
}

#[test]
fn rotated_angle_returns_nonempty() {
    let grid = ones(20, 20);
    let (pts, split_pts, invalid_segs) = call(&grid, 45.0);
    assert!(pts.len() >= 2);
    assert!(!split_pts.is_empty());
    assert!(invalid_segs.is_empty());
    assert_ne!(pts, call(&grid, 0.0).0);
}

#[test]
fn rotated_empty_map_returns_empty() {
    let (pts, split_pts, invalid_segs) = call(&zeros(10, 10), 180.0);
    assert!(pts.is_empty());
    assert!(split_pts.is_empty());
    assert!(invalid_segs.is_empty());
}

// ── Waypoint safety ────────────────────────────────────────────────────────────

#[test]
fn all_waypoints_on_safe_cells() {
    let grid = ones(20, 20);
    let (pts, _, _) = call(&grid, 0.0);
    for &(x, y) in &pts {
        let (r, c) = pt_to_cell(x, y);
        assert!(in_grid(&grid, r, c), "point ({x:.2},{y:.2}) -> cell ({r},{c}) outside the grid");
        assert!(grid[[r as usize, c as usize]], "point ({x:.2},{y:.2}) -> cell ({r},{c}) is unsafe");
    }
}

#[test]
fn risk_hole_no_unsafe_waypoints() {
    let mut grid = ones(20, 20);
    set(&mut grid, 8..12, 8..12, false); // interior obstacle
    let (pts, _, _) = call(&grid, 0.0);
    for &(x, y) in &pts {
        let (r, c) = pt_to_cell(x, y);
        if in_grid(&grid, r, c) {
            assert!(grid[[r as usize, c as usize]], "point ({x:.2},{y:.2}) lands on unsafe cell ({r},{c})");
        }
    }
}

// ── Split points ───────────────────────────────────────────────────────────────

#[test]
fn split_points_nonempty_for_valid_map() {
    let (_, split_pts, _) = call(&ones(20, 20), 0.0);
    assert!(!split_pts.is_empty());
}

#[test]
fn split_points_are_in_point_list() {
    // Every split point must also appear (exactly) in the main points list.
    let (pts, split_pts, _) = call(&ones(20, 20), 0.0);
    for sp in &split_pts {
        assert!(pts.contains(sp), "split point {sp:?} not found in points");
    }
}

#[test]
fn tight_adjacent_strips_do_not_use_tiny_u_turns() {
    // Very tight strips must not produce physically impossible U-turns.
    let (pts, split_pts, invalid_segs) = call(&ones(20, 20), 0.0);
    assert!(invalid_segs.is_empty());
    assert!(pts.contains(&split_pts[0]));
    // pytest.approx default: rel 1e-6
    assert!((split_pts[0].0 - 0.15).abs() <= 1e-6 * 0.15);
    assert!((split_pts[0].1 - 1.95).abs() <= 1e-6 * 1.95);
}

#[test]
fn wide_adjacent_strips_use_u_turn_points() {
    // Adjacent reverse-direction strips are joined by a rounded U-turn.
    let (pts, split_pts, invalid_segs) = call_with_spacing(&ones(30, 30), 0.8, 0.2);
    assert!(invalid_segs.is_empty());
    assert!(pts.contains(&split_pts[0]));
    let approx_first_lane_end =
        (split_pts[0].0 - 0.45).abs() <= 1e-6 * 0.45 && (split_pts[0].1 - 2.95).abs() <= 1e-6 * 2.95;
    assert!(!approx_first_lane_end, "first split point is the plain lane end, no U-turn");
    assert!(
        pts.iter().any(|&(x, y)| 0.45 < x && x < 1.25 && y > 2.4),
        "expected an interior U-turn arc between the first two strips"
    );
}

// ── Invalid segment detection ──────────────────────────────────────────────────

#[test]
fn separated_runs_produce_invalid_segments() {
    // A full-height wall forces cross-gap transitions.
    let mut grid = ones(20, 20);
    set(&mut grid, 0..20, 10..11, false);
    let (pts, _, invalid_segs) = call(&grid, 0.0);
    if pts.len() >= 2 {
        assert!(!invalid_segs.is_empty(), "expected invalid segments when zigzag must cross a vertical wall");
    }
}

#[test]
fn clear_map_has_no_invalid_segments() {
    let (_, _, invalid_segs) = call(&ones(20, 20), 0.0);
    assert!(invalid_segs.is_empty());
}

// ── test_zigzag_segment_boundaries.py ─────────────────────────────────────────

#[test]
fn segment_end_stops_before_first_unsafe_cell() {
    let safe = ndarray::array![[false], [true], [true], [false]];
    let (points, _split_points, invalid_segments) =
        generate_coverage_zigzag_path_rs(safe.view(), 1.0, 1.0, 1.0, 4, 1, 0.0, 0.0, 0.0);

    assert_eq!(points, vec![(0.5, 1.5), (0.5, 2.5)]);
    assert!(invalid_segments.is_empty());

    let sm = SafeMap { grid: safe.view(), resolution: 1.0, origin_x: 0.0, origin_y: 0.0 };
    assert!(validate_path_rs(&points, &sm).valid);
}
