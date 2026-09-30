use std::collections::{BinaryHeap, HashMap, VecDeque};
use std::cmp::Reverse;

use ndarray::{Array2, ArrayView2};
use ordered_float::OrderedFloat;

use crate::types::{SafeMap, world_to_grid, grid_to_world};

// 8-connected neighbours: (Δrow, Δcol, move_cost)
const NEIGHBOURS: [(i64, i64, f64); 8] = [
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, std::f64::consts::SQRT_2),
    (-1,  1, std::f64::consts::SQRT_2),
    ( 1, -1, std::f64::consts::SQRT_2),
    ( 1,  1, std::f64::consts::SQRT_2),
];

// ── Internal helpers ───────────────────────────────────────────────────────────

fn clamp(r: i64, c: i64, h: usize, w: usize) -> (usize, usize) {
    let r = r.clamp(0, h as i64 - 1) as usize;
    let c = c.clamp(0, w as i64 - 1) as usize;
    (r, c)
}

fn cuts_corner(r0: usize, c0: usize, r1: usize, c1: usize, grid: ArrayView2<bool>) -> bool {
    let dr = (r1 as i64 - r0 as i64).abs();
    let dc = (c1 as i64 - c0 as i64).abs();
    if dr == 1 && dc == 1 {
        return !grid[[r0, c1]] || !grid[[r1, c0]];
    }
    false
}

fn is_boundary_cell(r: usize, c: usize, grid: ArrayView2<bool>) -> bool {
    let h = grid.nrows() as i64;
    let w = grid.ncols() as i64;
    for &(dr, dc, _) in &NEIGHBOURS[..4] {
        let nr = r as i64 + dr;
        let nc = c as i64 + dc;
        if nr < 0 || nr >= h || nc < 0 || nc >= w {
            return true;
        }
        if !grid[[nr as usize, nc as usize]] {
            return true;
        }
    }
    false
}

fn boundary_distance(grid: ArrayView2<bool>) -> Array2<f64> {
    let h = grid.nrows();
    let w = grid.ncols();
    let mut dist = Array2::from_elem((h, w), f64::INFINITY);
    let mut queue: VecDeque<(usize, usize)> = VecDeque::new();

    for r in 0..h {
        for c in 0..w {
            if grid[[r, c]] && is_boundary_cell(r, c, grid) {
                dist[[r, c]] = 0.0;
                queue.push_back((r, c));
            }
        }
    }

    while let Some((r, c)) = queue.pop_front() {
        for &(dr, dc, _) in &NEIGHBOURS[..4] {
            let nr = r as i64 + dr;
            let nc = c as i64 + dc;
            if nr < 0 || nr >= h as i64 || nc < 0 || nc >= w as i64 {
                continue;
            }
            let (nr, nc) = (nr as usize, nc as usize);
            if !grid[[nr, nc]] {
                continue;
            }
            let nd = dist[[r, c]] + 1.0;
            if nd < dist[[nr, nc]] {
                dist[[nr, nc]] = nd;
                queue.push_back((nr, nc));
            }
        }
    }

    // unsafe cells keep infinity
    dist
}

fn reconstruct(
    came_from: &HashMap<(usize, usize), (usize, usize)>,
    r0: usize, c0: usize,
    r1: usize, c1: usize,
    sm: &SafeMap,
) -> Vec<(f64, f64)> {
    let mut cells: Vec<(usize, usize)> = Vec::new();
    let mut cur = (r1, c1);
    while let Some(&prev) = came_from.get(&cur) {
        cells.push(cur);
        cur = prev;
    }
    cells.push((r0, c0));
    cells.reverse();
    cells.iter().map(|&(r, c)| grid_to_world(r, c, sm)).collect()
}

pub fn plan_connector_rs(
    start: (f64, f64),
    end: (f64, f64),
    sm: &SafeMap,
    boundary_weight: f64,
) -> Option<Vec<(f64, f64)>> {
    // Check start/end safety
    let h = sm.grid.nrows();
    let w = sm.grid.ncols();

    let (r0i, c0i) = world_to_grid(start.0, start.1, sm);
    let (r1i, c1i) = world_to_grid(end.0, end.1, sm);

    // Bounds check for start/end
    if r0i < 0 || r0i >= h as i64 || c0i < 0 || c0i >= w as i64 { return None; }
    if r1i < 0 || r1i >= h as i64 || c1i < 0 || c1i >= w as i64 { return None; }
    if !sm.grid[[r0i as usize, c0i as usize]] { return None; }
    if !sm.grid[[r1i as usize, c1i as usize]] { return None; }

    let (r0, c0) = clamp(r0i, c0i, h, w);
    let (r1, c1) = clamp(r1i, c1i, h, w);

    if r0 == r1 && c0 == c1 {
        return Some(vec![start, end]);
    }

    let bdist = boundary_distance(sm.grid);

    let heuristic = |r: usize, c: usize| -> f64 {
        ((r1 as f64 - r as f64).powi(2) + (c1 as f64 - c as f64).powi(2)).sqrt()
    };

    // (f, g, row, col)
    type HeapKey = Reverse<(OrderedFloat<f64>, OrderedFloat<f64>, usize, usize)>;
    let mut open: BinaryHeap<HeapKey> = BinaryHeap::new();
    let init_h = heuristic(r0, c0);
    open.push(Reverse((OrderedFloat(init_h), OrderedFloat(0.0), r0, c0)));

    let mut g_score: HashMap<(usize, usize), f64> = HashMap::new();
    g_score.insert((r0, c0), 0.0);
    let mut came_from: HashMap<(usize, usize), (usize, usize)> = HashMap::new();

    while let Some(Reverse((_, of_g, r, c))) = open.pop() {
        let g = of_g.0;

        if (r, c) == (r1, c1) {
            return Some(reconstruct(&came_from, r0, c0, r1, c1, sm));
        }

        if g > *g_score.get(&(r, c)).unwrap_or(&f64::INFINITY) + f64::EPSILON {
            continue;
        }

        for &(dr, dc, cost) in &NEIGHBOURS {
            let nr = r as i64 + dr as i64;
            let nc = c as i64 + dc as i64;
            if nr < 0 || nr >= h as i64 || nc < 0 || nc >= w as i64 {
                continue;
            }
            let (nr, nc) = (nr as usize, nc as usize);
            if !sm.grid[[nr, nc]] {
                continue;
            }
            if cuts_corner(r, c, nr, nc, sm.grid) {
                continue;
            }
            let bc = boundary_weight * bdist[[nr, nc]].min(8.0);
            let new_g = g + cost + bc;
            if new_g < *g_score.get(&(nr, nc)).unwrap_or(&f64::INFINITY) {
                g_score.insert((nr, nc), new_g);
                came_from.insert((nr, nc), (r, c));
                let f = new_g + heuristic(nr, nc);
                open.push(Reverse((OrderedFloat(f), OrderedFloat(new_g), nr, nc)));
            }
        }
    }

    None
}
