//! Port of mower_mission/test/test_spiral_generator.py (commit dff480b).
//! Test names are the Python ones without the `test_` prefix. RES = 0.1,
//! origin (0, 0), STRIP = 0.2, SPACING = 0.1.
//!
//! The Python `plan_spiral_coverage` returned a rich `SpiralCoveragePlan`
//! (typed segments, coverage_mask, debug counters) that its own 3-tuple
//! wrapper threw away; the Rust function returns only
//! `(points, split_points, invalid_segments)`. Tests of that plan object are
//! ported onto the 3-tuple as follows:
//! - one spiral segment per component == one split point per component
//!   (`split_points[k]` is the last point of component k's spiral);
//! - inter-component A* bridges can never succeed (components are the
//!   4-connected components of the same grid the 4-connected A* runs on), so
//!   `points` is the concatenation of the component spirals;
//! - `coverage_mask` is rebuilt by `coverage_mask()` below with the Python
//!   `_mark_covered_cells` rule, per component spiral (all waypoints are on
//!   safe cells, which `all_waypoints_on_safe_cells` and the oracle check).
//!
//! Not ported: `test_returns_three_tuple` (compile-time guarantee of the Rust
//! signature) and `test_plan_returns_spiral_coverage_plan` (isinstance check
//! of the Python-only plan type). The "transition types <= {bridge, invalid}"
//! assertion of `test_two_disconnected_islands_both_planned` has no Rust
//! counterpart (no typed segments).

mod common;

use common::*;
use mower_coverage_core::spiral::plan_spiral_coverage_rs;
use ndarray::Array2;

const STRIP: f64 = 0.2;
const SPACING: f64 = 0.1;

type Plan = (Vec<(f64, f64)>, Vec<(f64, f64)>, Vec<(usize, usize)>);

fn call(grid: &Array2<bool>) -> Plan {
    let (h, w) = grid.dim();
    plan_spiral_coverage_rs(grid.view(), STRIP, SPACING, RES, h, w, OX, OY)
}

fn max_segment_distance(points: &[(f64, f64)]) -> f64 {
    points
        .windows(2)
        .map(|p| (p[1].0 - p[0].0).hypot(p[1].1 - p[0].1))
        .fold(0.0, f64::max)
}

/// Python `_line_cells` (Bresenham, end cell included).
fn line_cells(start: (i64, i64), end: (i64, i64)) -> Vec<(i64, i64)> {
    let (r0, c0) = start;
    let (r1, c1) = end;
    let dr = (r1 - r0).abs();
    let dc = (c1 - c0).abs();
    let sr = if r1 > r0 { 1 } else { -1 };
    let sc = if c1 > c0 { 1 } else { -1 };
    let (mut r, mut c) = (r0, c0);
    let mut cells = Vec::new();
    if dc > dr {
        let mut err = dc / 2;
        while c != c1 {
            cells.push((r, c));
            err -= dr;
            if err < 0 {
                r += sr;
                err += dc;
            }
            c += sc;
        }
    } else {
        let mut err = dr / 2;
        while r != r1 {
            cells.push((r, c));
            err -= dc;
            if err < 0 {
                c += sc;
                err += dr;
            }
            r += sr;
        }
    }
    cells.push((r1, c1));
    cells
}

/// 4-connected component label of every safe cell (`usize::MAX` = unsafe).
fn component_labels(grid: &Array2<bool>) -> Array2<usize> {
    let (h, w) = grid.dim();
    let mut labels = Array2::from_elem((h, w), usize::MAX);
    let mut next = 0;
    for start in grid.indexed_iter().filter(|(_, &safe)| safe).map(|(rc, _)| rc) {
        if labels[start] != usize::MAX {
            continue;
        }
        labels[start] = next;
        let mut stack = vec![start];
        while let Some((r, c)) = stack.pop() {
            let neighbours = [(r.wrapping_sub(1), c), (r + 1, c), (r, c.wrapping_sub(1)), (r, c + 1)];
            for (nr, nc) in neighbours {
                if nr < h && nc < w && grid[[nr, nc]] && labels[[nr, nc]] == usize::MAX {
                    labels[[nr, nc]] = next;
                    stack.push((nr, nc));
                }
            }
        }
        next += 1;
    }
    labels
}

/// Rebuild the Python `SpiralCoveragePlan.coverage_mask`: for each component
/// spiral, mark the safe cells within `ceil(strip / (2 res))` cells of every
/// cell on its polyline. A new spiral starts where the component changes (the
/// unbridged jump between components is not part of any spiral).
fn coverage_mask(grid: &Array2<bool>, points: &[(f64, f64)]) -> Array2<bool> {
    let (h, w) = (grid.nrows() as i64, grid.ncols() as i64);
    let labels = component_labels(grid);
    let label_of = |(r, c): (i64, i64)| labels[[r as usize, c as usize]];
    let radius = ((STRIP / (2.0 * RES)).ceil() as i64).max(0);
    let mut mask = zeros(grid.nrows(), grid.ncols());
    let mut prev: Option<(i64, i64)> = None;
    for &(x, y) in points {
        let cell = pt_to_cell(x, y);
        let line = match prev {
            Some(p) if label_of(p) == label_of(cell) => line_cells(p, cell),
            _ => vec![cell],
        };
        for (row, col) in line {
            for r in (row - radius).max(0)..=(row + radius).min(h - 1) {
                for c in (col - radius).max(0)..=(col + radius).min(w - 1) {
                    let inside = radius == 0 || (r - row).pow(2) + (c - col).pow(2) <= radius * radius;
                    if grid[[r as usize, c as usize]] && inside {
                        mask[[r as usize, c as usize]] = true;
                    }
                }
            }
        }
        prev = Some(cell);
    }
    mask
}

// ── Return contract ────────────────────────────────────────────────────────────

#[test]
fn empty_map_returns_empty() {
    let (pts, split_pts, invalid_segs) = call(&zeros(10, 10));
    assert!(pts.is_empty());
    assert!(split_pts.is_empty());
    assert!(invalid_segs.is_empty());
}

#[test]
fn rectangle_returns_nonempty() {
    let (pts, split_pts, _) = call(&ones(20, 20));
    assert!(pts.len() >= 2);
    assert!(!split_pts.is_empty());
}

#[test]
fn accepts_uint8_safe_map() {
    // Python accepted a uint8 grid; the Rust API takes bool (non-zero = safe).
    let grid = Array2::<u8>::ones((20, 20)).mapv(|v| v != 0);
    let (pts, split_pts, invalid_segs) = call(&grid);
    assert!(pts.len() >= 2);
    assert!(!split_pts.is_empty());
    assert!(invalid_segs.is_empty());
}

#[test]
fn spiral_order_is_not_row_major_scan() {
    let (pts, _, _) = call(&ones(20, 20));
    let cells: Vec<(i64, i64)> = pts.iter().take(80).map(|&(x, y)| pt_to_cell(x, y)).collect();
    let rows: std::collections::BTreeSet<i64> = cells.iter().map(|c| c.0).collect();
    let cols: std::collections::BTreeSet<i64> = cells.iter().map(|c| c.1).collect();
    assert!(rows.len() > 1);
    assert!(cols.len() > 1);
}

#[test]
fn spiral_uses_strip_centerlines_not_every_cell() {
    let grid = ones(20, 20);
    let (pts, _, _) = call(&grid);
    assert!(pts.len() < count_true(&grid));
}

// ── Waypoint safety ────────────────────────────────────────────────────────────

#[test]
fn all_waypoints_on_safe_cells() {
    let grid = ones(20, 20);
    let (pts, _, _) = call(&grid);
    for &(x, y) in &pts {
        let (r, c) = pt_to_cell(x, y);
        assert!(in_grid(&grid, r, c), "point ({x:.2},{y:.2}) -> cell ({r},{c}) outside the grid");
        assert!(grid[[r as usize, c as usize]], "point ({x:.2},{y:.2}) -> cell ({r},{c}) is unsafe");
    }
}

#[test]
fn center_hole_avoided() {
    let mut grid = ones(20, 20);
    set(&mut grid, 8..12, 8..12, false); // central hole
    let (pts, _, _) = call(&grid);
    for &(x, y) in &pts {
        let (r, c) = pt_to_cell(x, y);
        if in_grid(&grid, r, c) {
            assert!(grid[[r as usize, c as usize]], "point ({x:.2},{y:.2}) lands on unsafe cell ({r},{c})");
        }
    }
}

#[test]
fn obstacle_no_unsafe_waypoints() {
    let mut grid = ones(30, 30);
    set(&mut grid, 5..10, 5..25, false); // horizontal obstacle band
    let (pts, _, _) = call(&grid);
    for &(x, y) in &pts {
        let (r, c) = pt_to_cell(x, y);
        if in_grid(&grid, r, c) {
            assert!(grid[[r as usize, c as usize]], "point ({x:.2},{y:.2}) on unsafe cell ({r},{c})");
        }
    }
}

// ── Split points ───────────────────────────────────────────────────────────────

#[test]
fn split_points_nonempty_for_valid_map() {
    let (_, split_pts, _) = call(&ones(20, 20));
    assert!(!split_pts.is_empty());
}

#[test]
fn split_points_are_in_point_list() {
    let (pts, split_pts, _) = call(&ones(20, 20));
    for sp in &split_pts {
        assert!(pts.contains(sp), "split point {sp:?} not found in points");
    }
}

// ── Invalid segment detection ──────────────────────────────────────────────────

#[test]
fn connected_obstacle_transitions_are_bridged_safely() {
    // A connected safe region routes around an obstacle instead of jumping it.
    let mut grid = ones(20, 20);
    set(&mut grid, 5..15, 8..12, false);
    let (pts, _, invalid_segs) = call(&grid);
    assert!(invalid_segs.is_empty());
    assert!(max_segment_distance(&pts) <= STRIP * 1.75);
}

#[test]
fn disconnected_transition_reported() {
    // Disconnected safe islands still produce an invalid transition segment.
    let mut grid = ones(20, 20);
    set(&mut grid, 0..20, 8..12, false);
    let (pts, _, invalid_segs) = call(&grid);
    if pts.len() >= 2 {
        assert!(!invalid_segs.is_empty());
    }
}

#[test]
fn concave_region_avoids_long_greedy_jumps() {
    // C-shaped map: no long same-layer straight jumps.
    let mut grid = zeros(60, 60);
    set(&mut grid, 5..55, 5..55, true);
    set(&mut grid, 15..45, 20..55, false);
    let (pts, _, invalid_segs) = call(&grid);
    assert!(invalid_segs.is_empty());
    assert!(max_segment_distance(&pts) <= STRIP * 1.75);
}

#[test]
fn clear_rectangle_no_invalid_segments() {
    let (_, _, invalid_segs) = call(&ones(10, 10));
    assert!(invalid_segs.is_empty());
}

// ── plan_spiral_coverage (SpiralCoveragePlan API), ported onto the 3-tuple ────

/// Two 8x8 safe islands separated by a 4-column unsafe gap.
fn two_islands() -> Array2<bool> {
    let mut grid = zeros(10, 22);
    set(&mut grid, 1..9, 1..9, true); // left island
    set(&mut grid, 1..9, 13..21, true); // right island
    grid
}

/// Index of the last point of the first island's spiral: the only step of the
/// path between the left island (x <= 0.9) and the right one (x >= 1.3).
fn island_jump(pts: &[(f64, f64)]) -> usize {
    let jumps: Vec<usize> =
        (0..pts.len().saturating_sub(1)).filter(|&i| (pts[i].0 < 1.1) != (pts[i + 1].0 < 1.1)).collect();
    assert_eq!(jumps.len(), 1, "expected exactly one jump between the islands");
    jumps[0]
}

#[test]
fn single_component_no_bridge() {
    // One component: exactly one spiral segment (one split point, the last
    // point) and hence no bridge.
    let (pts, split_pts, invalid_segs) = call(&ones(10, 10));
    assert_eq!(split_pts.len(), 1);
    assert_eq!(split_pts.last(), pts.last());
    assert!(invalid_segs.is_empty());
}

#[test]
fn two_islands_all_safe_cells_covered_by_strip_mask() {
    let grid = two_islands();
    let (pts, _, _) = call(&grid);
    let covered = coverage_mask(&grid, &pts);
    let uncovered: Vec<(usize, usize)> =
        grid.indexed_iter().filter(|&(rc, &safe)| safe && !covered[rc]).map(|(rc, _)| rc).collect();
    assert!(uncovered.is_empty(), "uncovered cells: {uncovered:?}");
}

#[test]
fn two_disconnected_islands_both_planned() {
    // Both islands get a spiral segment even without a safe bridge.
    let (pts, split_pts, invalid_segs) = call(&two_islands());
    assert_eq!(split_pts.len(), 2, "expected 2 spiral segments");
    // The unbridged jump between the islands is reported as invalid.
    let jump = island_jump(&pts);
    assert!(invalid_segs.contains(&(jump, jump + 1)));
}

#[test]
fn bridge_waypoints_all_safe() {
    // Bridge points are part of `points`; every point must be on a safe cell.
    let grid = two_islands();
    let (pts, _, _) = call(&grid);
    for &(x, y) in &pts {
        let (r, c) = pt_to_cell(x, y);
        if in_grid(&grid, r, c) {
            assert!(grid[[r as usize, c as usize]], "point ({x},{y}) on unsafe cell ({r},{c})");
        }
    }
}

#[test]
fn coverage_mask_matches_visited_points() {
    let grid = ones(10, 10);
    let (pts, _, _) = call(&grid);
    let covered = coverage_mask(&grid, &pts);
    for &(x, y) in &pts {
        let (r, c) = pt_to_cell(x, y);
        if in_grid(&grid, r, c) {
            assert!(covered[[r as usize, c as usize]], "point ({x},{y}) -> ({r},{c}) not in coverage mask");
        }
    }
}

#[test]
fn split_points_are_ends_of_spiral_segments() {
    let (pts, split_pts, _) = call(&two_islands());
    assert_eq!(split_pts.len(), 2);
    for end in &split_pts {
        assert!(pts.contains(end), "spiral-segment end {end:?} not in points");
    }
    // The last spiral ends the path; the first ends right before the second
    // island's spiral starts (in the other island).
    assert_eq!(split_pts.last(), pts.last());
    // (A spiral revisits its loop start, so the split point can also occur
    // earlier in the path; the segment end is the last point before the jump.)
    assert_eq!(pts[island_jump(&pts)], split_pts[0]);
}
