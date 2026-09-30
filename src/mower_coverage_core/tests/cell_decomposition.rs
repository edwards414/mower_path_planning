//! Port of mower_mission/test/test_cell_decomposition.py (commit dff480b).
//! Test names are the Python ones without the `test_` prefix. RES = 0.1,
//! origin (0, 0). The full outputs are also pinned by the `decompose/...`
//! cases of python_reference_oracle.rs.

mod common;

use std::collections::HashSet;

use common::*;
use mower_coverage_core::cell_decomposition::{decompose, CoverageCell};

// ── Basic decomposition ────────────────────────────────────────────────────────

#[test]
fn rectangle_single_cell() {
    let grid = ones(20, 20);
    let (cells, _graph) = decompose(&safe_map(&grid));
    assert_eq!(cells.len(), 1);
}

#[test]
fn empty_grid_no_cells() {
    let grid = zeros(10, 10);
    let (cells, graph) = decompose(&safe_map(&grid));
    assert!(cells.is_empty());
    assert!(graph.is_empty());
}

#[test]
fn cells_cover_safe_area() {
    // The union of all cell masks equals the safe grid exactly.
    let mut grid = ones(30, 30);
    set(&mut grid, 10..20, 10..20, false); // central hole
    let (cells, _) = decompose(&safe_map(&grid));
    let mut union = zeros(30, 30);
    for cell in &cells {
        union.zip_mut_with(&cell.mask, |u, &m| *u |= m);
    }
    assert_eq!(union, grid);
}

#[test]
fn cells_nonempty() {
    let grid = ones(20, 20);
    let (cells, _) = decompose(&safe_map(&grid));
    for cell in &cells {
        assert!(cell.mask.iter().any(|&v| v), "cell {} has an empty mask", cell.cell_id);
        assert!(cell.area_m2 > 0.0);
    }
}

// ── Critical events ────────────────────────────────────────────────────────────

#[test]
fn obstacle_causes_multiple_cells() {
    // An internal obstacle that splits the free intervals gives 3+ cells.
    let mut grid = ones(20, 20);
    set(&mut grid, 5..15, 8..12, false); // rows 5-14, cols 8-11
    let (cells, _) = decompose(&safe_map(&grid));
    assert!(cells.len() >= 3);
}

#[test]
fn u_shape_has_multiple_cells() {
    // Vertical U: the row sweep sees two separate arms, so 2+ cells.
    let mut grid = zeros(30, 20);
    set(&mut grid, 0..20, 0..5, true); // left arm
    set(&mut grid, 0..20, 15..20, true); // right arm
    set(&mut grid, 20..25, 0..20, true); // bottom connecting bar
    let (cells, _) = decompose(&safe_map(&grid));
    assert!(cells.len() >= 2);
}

#[test]
fn disconnected_cells_not_adjacent() {
    // Two separated blocks: one cell each, no edges.
    let mut grid = zeros(10, 20);
    set(&mut grid, 0..10, 0..5, true); // left block
    set(&mut grid, 0..10, 15..20, true); // right block (gap cols 5-14)
    let (cells, graph) = decompose(&safe_map(&grid));
    assert_eq!(cells.len(), 2);
    for (cid, neighbours) in &graph {
        assert!(neighbours.is_empty(), "cell {cid} has unexpected neighbours {neighbours:?}");
    }
}

#[test]
fn obstacle_cells_are_adjacent() {
    // Cells created by split/merge events appear in each other's graph.
    let mut grid = ones(20, 20);
    set(&mut grid, 5..15, 8..12, false);
    let (_, graph) = decompose(&safe_map(&grid));
    let total_edges: usize = graph.values().map(Vec::len).sum();
    assert!(total_edges > 0, "no adjacency edges found after split/merge events");
}

// ── Cell metadata ──────────────────────────────────────────────────────────────

#[test]
fn cell_bbox_contains_all_mask_pixels() {
    let mut grid = ones(20, 20);
    set(&mut grid, 5..15, 8..12, false);
    let (cells, _) = decompose(&safe_map(&grid));
    for cell in &cells {
        let (rmin, cmin, rmax, cmax) = cell.bbox;
        for ((r, c), _) in cell.mask.indexed_iter().filter(|(_, &v)| v) {
            assert!(r >= rmin, "cell {}: row below bbox", cell.cell_id);
            assert!(r <= rmax, "cell {}: row above bbox", cell.cell_id);
            assert!(c >= cmin, "cell {}: col below bbox", cell.cell_id);
            assert!(c <= cmax, "cell {}: col above bbox", cell.cell_id);
        }
    }
}

#[test]
fn cell_area_matches_pixel_count() {
    // area_m2 == pixel_count * resolution^2
    let grid = ones(10, 10);
    let (cells, _) = decompose(&safe_map(&grid));
    for cell in &cells {
        let expected = count_true(&cell.mask) as f64 * RES * RES;
        assert!((cell.area_m2 - expected).abs() < 1e-9, "cell {}: area {} != {}", cell.cell_id, cell.area_m2, expected);
    }
}

#[test]
fn cell_centroid_within_world_bbox() {
    let mut grid = ones(30, 30);
    set(&mut grid, 10..20, 10..20, false);
    let (cells, _) = decompose(&safe_map(&grid));
    for cell in &cells {
        let (rmin, cmin, rmax, cmax) = cell.bbox;
        let x_lo = OX + cmin as f64 * RES;
        let x_hi = OX + (cmax + 1) as f64 * RES;
        let y_lo = OY + rmin as f64 * RES;
        let y_hi = OY + (rmax + 1) as f64 * RES;
        let (cx, cy) = cell.centroid_xy;
        assert!(x_lo <= cx && cx <= x_hi, "cell {} centroid x={cx:.3} outside [{x_lo:.3}, {x_hi:.3}]", cell.cell_id);
        assert!(y_lo <= cy && cy <= y_hi, "cell {} centroid y={cy:.3} outside [{y_lo:.3}, {y_hi:.3}]", cell.cell_id);
    }
}

#[test]
fn single_cell_empty_graph() {
    let grid = ones(20, 20);
    let (cells, graph) = decompose(&safe_map(&grid));
    assert_eq!(cells.len(), 1);
    assert_eq!(graph[&cells[0].cell_id], Vec::<usize>::new());
}

// ── Additional (Rust port): CoverageCell semantics from coverage/types.py ─────

#[test]
fn coverage_cell_equality_and_hash_use_cell_id_only() {
    // Python CoverageCell: @dataclass(eq=False) with __eq__/__hash__ on cell_id.
    let mut grid = ones(20, 20);
    set(&mut grid, 5..15, 8..12, false);
    let (cells, _) = decompose(&safe_map(&grid));
    let mut other: CoverageCell = cells[1].clone();
    other.area_m2 += 1.0;
    other.valid = false;
    assert_eq!(other, cells[1]);
    assert_ne!(cells[0], cells[1]);
    let ids: HashSet<&CoverageCell> = cells.iter().chain(std::iter::once(&other)).collect();
    assert_eq!(ids.len(), cells.len());
}

#[test]
fn cells_are_disjoint_and_ids_follow_creation_order() {
    let mut grid = ones(20, 20);
    set(&mut grid, 5..15, 8..12, false);
    let (cells, graph) = decompose(&safe_map(&grid));
    let mut seen = zeros(20, 20);
    for cell in &cells {
        for (rc, _) in cell.mask.indexed_iter().filter(|(_, &v)| v) {
            assert!(!seen[rc], "cell {} overlaps another cell at {rc:?}", cell.cell_id);
            seen[rc] = true;
        }
        assert!(cell.valid);
        assert!(cell.invalid_reason.is_empty());
        assert_eq!(cell.entry_candidates.len(), 1);
        assert_eq!(cell.exit_candidates.len(), 1);
    }
    let ids: Vec<usize> = cells.iter().map(|c| c.cell_id).collect();
    assert_eq!(ids, (0..cells.len()).collect::<Vec<_>>());
    assert_eq!(graph.keys().copied().collect::<Vec<_>>(), ids);
}
