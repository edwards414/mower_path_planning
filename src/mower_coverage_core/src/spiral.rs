use std::collections::{BinaryHeap, HashMap, HashSet, VecDeque};
use std::cmp::Reverse;

use ndarray::{Array2, ArrayView2};
use numpy::PyReadonlyArray2;
use pyo3::prelude::*;

use crate::path_validator::validate_path_rs;
use crate::types::SafeMap;

// ── Connected components (4-connected, sorted largest first) ──────────────────

fn connected_components_4(safe: ArrayView2<bool>) -> Vec<Array2<bool>> {
    let h = safe.nrows();
    let w = safe.ncols();
    let mut visited = Array2::<bool>::default((h, w));
    let mut components: Vec<Array2<bool>> = Vec::new();

    for r in 0..h {
        for c in 0..w {
            if !safe[[r, c]] || visited[[r, c]] { continue; }
            let mut comp = Array2::<bool>::default((h, w));
            let mut dq: VecDeque<(usize, usize)> = VecDeque::new();
            visited[[r, c]] = true;
            dq.push_back((r, c));
            while let Some((cr, cc)) = dq.pop_front() {
                comp[[cr, cc]] = true;
                for (dr, dc) in [(-1i32, 0), (1, 0), (0, -1i32), (0, 1)] {
                    let nr = cr as i32 + dr;
                    let nc = cc as i32 + dc;
                    if nr < 0 || nr >= h as i32 || nc < 0 || nc >= w as i32 { continue; }
                    let (nr, nc) = (nr as usize, nc as usize);
                    if safe[[nr, nc]] && !visited[[nr, nc]] {
                        visited[[nr, nc]] = true;
                        dq.push_back((nr, nc));
                    }
                }
            }
            components.push(comp);
        }
    }
    components.sort_by(|a, b| {
        b.iter().filter(|&&v| v).count().cmp(&a.iter().filter(|&&v| v).count())
    });
    components
}

// ── BFS distance transform ────────────────────────────────────────────────────

fn bfs_dist(comp_mask: ArrayView2<bool>) -> Array2<i32> {
    let h = comp_mask.nrows();
    let w = comp_mask.ncols();
    let mut dist = Array2::from_elem((h, w), -1i32);
    let mut dq: VecDeque<(usize, usize)> = VecDeque::new();

    for r in 0..h {
        for c in 0..w {
            if !comp_mask[[r, c]] { continue; }
            let on_boundary =
                r == 0 || r == h - 1 || c == 0 || c == w - 1
                || !comp_mask[[r - 1, c]]
                || !comp_mask[[r + 1, c]]
                || !comp_mask[[r, c - 1]]
                || !comp_mask[[r, c + 1]];
            if on_boundary {
                dist[[r, c]] = 0;
                dq.push_back((r, c));
            }
        }
    }

    while let Some((r, c)) = dq.pop_front() {
        for (dr, dc) in [(-1i32, 0), (1, 0), (0, -1i32), (0, 1)] {
            let nr = r as i32 + dr;
            let nc = c as i32 + dc;
            if nr < 0 || nr >= h as i32 || nc < 0 || nc >= w as i32 { continue; }
            let (nr, nc) = (nr as usize, nc as usize);
            if comp_mask[[nr, nc]] && dist[[nr, nc]] == -1 {
                dist[[nr, nc]] = dist[[r, c]] + 1;
                dq.push_back((nr, nc));
            }
        }
    }
    dist
}

// ── Spiral layer masks ────────────────────────────────────────────────────────

fn iter_spiral_layer_masks(
    comp_mask: ArrayView2<bool>,
    dist: &Array2<i32>,
    layer_step: usize,
    max_dist: i32,
) -> Vec<Array2<bool>> {
    let h = comp_mask.nrows();
    let w = comp_mask.ncols();
    let mut layer_masks: Vec<Array2<bool>> = Vec::new();
    let mut used: HashSet<i32> = HashSet::new();

    let mut layer_start = 0i32;
    while layer_start <= max_dist {
        let layer_end = (layer_start + layer_step as i32 - 1).min(max_dist);
        let target = (layer_start + layer_end) / 2;

        // Collect unique distances in band
        let mut band_dists: Vec<i32> = dist
            .iter()
            .copied()
            .filter(|&d| d >= layer_start && d <= layer_end)
            .collect::<std::collections::BTreeSet<_>>()
            .into_iter()
            .collect();

        if band_dists.is_empty() {
            layer_start += layer_step as i32;
            continue;
        }

        // Pick closest to target
        band_dists.sort_by_key(|&d| (d - target).abs());
        let target_dist = band_dists[0];

        if !used.insert(target_dist) {
            layer_start += layer_step as i32;
            continue;
        }

        // Build layer mask
        let mut lm = Array2::<bool>::default((h, w));
        for r in 0..h {
            for c in 0..w {
                if comp_mask[[r, c]] && dist[[r, c]] >= target_dist {
                    lm[[r, c]] = true;
                }
            }
        }
        if lm.iter().any(|&v| v) {
            layer_masks.push(lm);
        }

        layer_start += layer_step as i32;
    }
    layer_masks
}

// ── Boundary loop tracing ─────────────────────────────────────────────────────

type Vertex = (i32, i32);
type InsideCell = (usize, usize);

fn choose_next_boundary_edge(
    prev: Vertex,
    cur: Vertex,
    candidates: &[(Vertex, InsideCell)],
) -> (Vertex, InsideCell) {
    let in_vec = (cur.0 - prev.0, cur.1 - prev.1);

    let turn_cost = |cand: &(Vertex, InsideCell)| -> (i32, i32, Vertex) {
        let out_vec = (cand.0.0 - cur.0, cand.0.1 - cur.1);
        let dot = in_vec.0 * out_vec.0 + in_vec.1 * out_vec.1;
        let cross = in_vec.0 * out_vec.1 - in_vec.1 * out_vec.0;
        let rank = if dot > 0 { 0 } else if cross != 0 { 1 } else { 2 };
        (rank, cross.abs(), cand.0)
    };

    candidates.iter().min_by_key(|c| turn_cost(c)).copied().unwrap()
}

fn boundary_cell_loops(mask: ArrayView2<bool>) -> Vec<Vec<(usize, usize)>> {
    let h = mask.nrows();
    let w = mask.ncols();

    let is_inside = |r: i32, c: i32| -> bool {
        r >= 0 && r < h as i32 && c >= 0 && c < w as i32 && mask[[r as usize, c as usize]]
    };

    // Build edge adjacency: start_vertex -> [(end_vertex, inside_cell)]
    let mut edges_by_start: HashMap<Vertex, Vec<(Vertex, InsideCell)>> = HashMap::new();
    let mut all_edges: Vec<(Vertex, Vertex, InsideCell)> = Vec::new();

    macro_rules! add_edge {
        ($s:expr, $e:expr, $cell:expr) => {{
            all_edges.push(($s, $e, $cell));
            edges_by_start
                .entry($s)
                .or_insert_with(Vec::<(Vertex, InsideCell)>::new)
                .push(($e, $cell));
        }};
    }

    for r in 0..h {
        for c in 0..w {
            if !mask[[r, c]] { continue; }
            let ri = r as i32;
            let ci = c as i32;

            if !is_inside(ri - 1, ci) {
                add_edge!((ri, ci), (ri, ci + 1), (r, c));
            }
            if !is_inside(ri, ci + 1) {
                add_edge!((ri, ci + 1), (ri + 1, ci + 1), (r, c));
            }
            if !is_inside(ri + 1, ci) {
                add_edge!((ri + 1, ci + 1), (ri + 1, ci), (r, c));
            }
            if !is_inside(ri, ci - 1) {
                add_edge!((ri + 1, ci), (ri, ci), (r, c));
            }
        }
    }

    let mut visited: HashSet<(Vertex, Vertex)> = HashSet::new();
    let mut loops: Vec<Vec<(usize, usize)>> = Vec::new();

    for &(start, end, inside_cell) in &all_edges {
        let edge_key = (start, end);
        if visited.contains(&edge_key) { continue; }

        let first_start = start;
        let mut cur_start = start;
        let mut cur_end = end;
        let mut cur_cell = inside_cell;
        let mut loop_cells: Vec<(usize, usize)> = Vec::new();

        loop {
            if visited.contains(&(cur_start, cur_end)) { break; }
            visited.insert((cur_start, cur_end));
            loop_cells.push(cur_cell);

            let candidates: Vec<(Vertex, InsideCell)> = edges_by_start
                .get(&cur_end)
                .map(|v| v.iter().filter(|&&(ne, _)| !visited.contains(&(cur_end, ne))).copied().collect())
                .unwrap_or_default();

            if candidates.is_empty() { break; }

            let prev_end = cur_start;
            let (next_end, next_cell) = choose_next_boundary_edge(prev_end, cur_end, &candidates);

            cur_start = cur_end;
            cur_end = next_end;
            cur_cell = next_cell;

            if cur_start == first_start && cur_end == end { break; }
        }

        // Deduplicate consecutive
        let mut loop_dedup: Vec<(usize, usize)> = Vec::new();
        for cell in loop_cells {
            if loop_dedup.last().map_or(true, |&last| last != cell) {
                loop_dedup.push(cell);
            }
        }

        if loop_dedup.len() > 1 && loop_dedup.last() != loop_dedup.first() {
            loop_dedup.push(loop_dedup[0]);
        }
        if !loop_dedup.is_empty() {
            loops.push(loop_dedup);
        }
    }

    loops
}

// ── Path helpers ──────────────────────────────────────────────────────────────

fn cell_dist(a: (usize, usize), b: (usize, usize)) -> usize {
    (a.0 as i64 - b.0 as i64).unsigned_abs() as usize
        + (a.1 as i64 - b.1 as i64).unsigned_abs() as usize
}

fn sort_cell_paths(
    mut paths: Vec<Vec<(usize, usize)>>,
    previous_cell: Option<(usize, usize)>,
) -> Vec<Vec<(usize, usize)>> {
    match previous_cell {
        None => { paths.sort_by(|a, b| b.len().cmp(&a.len())); paths }
        Some(prev) => {
            paths.sort_by_key(|path| path.iter().map(|&c| cell_dist(c, prev)).min().unwrap_or(usize::MAX));
            paths
        }
    }
}

fn rotate_path_near_previous(
    path: Vec<(usize, usize)>,
    previous_cell: Option<(usize, usize)>,
) -> Vec<(usize, usize)> {
    if path.is_empty() { return vec![]; }

    let closed = path.len() > 1 && path[0] == *path.last().unwrap();
    let base: Vec<(usize, usize)> = if closed { path[..path.len()-1].to_vec() } else { path.clone() };
    if base.is_empty() { return vec![]; }

    let start_idx = match previous_cell {
        None => (0..base.len()).min_by_key(|&i| base[i]).unwrap_or(0),
        Some(prev) => (0..base.len()).min_by_key(|&i| cell_dist(base[i], prev)).unwrap_or(0),
    };

    let mut rotated: Vec<_> = base[start_idx..].iter().chain(base[..start_idx].iter()).copied().collect();

    if closed && !rotated.is_empty() {
        rotated.push(rotated[0]);
    } else if let Some(prev) = previous_cell {
        if rotated.len() > 1 {
            let rev: Vec<_> = rotated.iter().rev().copied().collect();
            if cell_dist(rev[0], prev) < cell_dist(rotated[0], prev) {
                rotated = rev;
            }
        }
    }
    rotated
}

fn line_cells(start: (usize, usize), end: (usize, usize)) -> Vec<(usize, usize)> {
    let (r0, c0) = (start.0 as i64, start.1 as i64);
    let (r1, c1) = (end.0 as i64, end.1 as i64);
    let dr = (r1 - r0).abs();
    let dc = (c1 - c0).abs();
    let sr: i64 = if r1 > r0 { 1 } else { -1 };
    let sc: i64 = if c1 > c0 { 1 } else { -1 };
    let (mut r, mut c) = (r0, c0);
    let mut cells = Vec::new();

    if dc > dr {
        let mut err = dc / 2;
        while c != c1 {
            cells.push((r as usize, c as usize));
            err -= dr;
            if err < 0 { r += sr; err += dc; }
            c += sc;
        }
    } else {
        let mut err = dr / 2;
        while r != r1 {
            cells.push((r as usize, c as usize));
            err -= dc;
            if err < 0 { c += sc; err += dr; }
            r += sr;
        }
    }
    cells.push((r1 as usize, c1 as usize));
    cells
}

fn cell_segment_safe(start: (usize, usize), end: (usize, usize), mask: ArrayView2<bool>) -> bool {
    let h = mask.nrows() as i64;
    let w = mask.ncols() as i64;
    for (r, c) in line_cells(start, end) {
        if r as i64 >= h || c as i64 >= w { return false; }
        if !mask[[r, c]] { return false; }
    }
    true
}

fn append_cell_path(target: &mut Vec<(usize, usize)>, cells: &[(usize, usize)]) {
    for &cell in cells {
        if target.last().map_or(true, |&last| last != cell) {
            target.push(cell);
        }
    }
}

fn sample_cells_by_spacing(
    ordered_cells: &[(usize, usize)],
    spacing_cells: usize,
    mask: ArrayView2<bool>,
) -> Vec<(usize, usize)> {
    if ordered_cells.is_empty() { return vec![]; }

    let mut sampled = vec![ordered_cells[0]];
    let mut chunk = vec![ordered_cells[0]];
    let mut travelled: usize = 0;
    let mut previous = ordered_cells[0];

    for &cell in &ordered_cells[1..] {
        travelled += cell_dist(previous, cell);
        chunk.push(cell);
        previous = cell;

        if travelled >= spacing_cells {
            let target = *chunk.last().unwrap();
            if cell_segment_safe(*sampled.last().unwrap(), target, mask) {
                if sampled.last().unwrap() != &target {
                    sampled.push(target);
                }
            } else {
                for &c in &chunk[1..] {
                    if sampled.last().unwrap() != &c {
                        sampled.push(c);
                    }
                }
            }
            chunk = vec![*sampled.last().unwrap()];
            travelled = 0;
        }
    }

    if let Some(&last) = chunk.last() {
        if sampled.last().unwrap() != &last {
            let target = last;
            if cell_segment_safe(*sampled.last().unwrap(), target, mask) {
                sampled.push(target);
            } else {
                for &c in &chunk[1..] {
                    if sampled.last().unwrap() != &c {
                        sampled.push(c);
                    }
                }
            }
        }
    }

    sampled
}

// ── 4-connected A* ────────────────────────────────────────────────────────────

fn astar_4(
    start: (usize, usize),
    goal: (usize, usize),
    safe: ArrayView2<bool>,
) -> Option<Vec<(usize, usize)>> {
    let h = safe.nrows();
    let w = safe.ncols();
    if start == goal { return Some(vec![start]); }

    let heuristic = |r: usize, c: usize| -> i32 {
        (r as i32 - goal.0 as i32).abs() + (c as i32 - goal.1 as i32).abs()
    };

    let mut open: BinaryHeap<Reverse<(i32, i32, usize, usize)>> = BinaryHeap::new();
    open.push(Reverse((heuristic(start.0, start.1), 0, start.0, start.1)));
    let mut g_score: HashMap<(usize, usize), i32> = HashMap::new();
    g_score.insert(start, 0);
    let mut came_from: HashMap<(usize, usize), (usize, usize)> = HashMap::new();

    while let Some(Reverse((_, cost, r, c))) = open.pop() {
        if (r, c) == goal {
            let mut path: Vec<(usize, usize)> = Vec::new();
            let mut cur = (r, c);
            while let Some(&prev) = came_from.get(&cur) {
                path.push(cur);
                cur = prev;
            }
            path.push(start);
            path.reverse();
            return Some(path);
        }

        if cost > *g_score.get(&(r, c)).unwrap_or(&i32::MAX) {
            continue;
        }

        for (dr, dc) in [(-1i32, 0), (1, 0), (0, -1i32), (0, 1)] {
            let nr = r as i32 + dr;
            let nc = c as i32 + dc;
            if nr < 0 || nr >= h as i32 || nc < 0 || nc >= w as i32 { continue; }
            let (nr, nc) = (nr as usize, nc as usize);
            if !safe[[nr, nc]] { continue; }
            let ng = cost + 1;
            if ng < *g_score.get(&(nr, nc)).unwrap_or(&i32::MAX) {
                g_score.insert((nr, nc), ng);
                came_from.insert((nr, nc), (r, c));
                open.push(Reverse((ng + heuristic(nr, nc), ng, nr, nc)));
            }
        }
    }
    None
}

// ── Per-component onion-layer spiral ──────────────────────────────────────────

fn onion_layer_spiral(
    comp_mask: ArrayView2<bool>,
    strip_width_m: f64,
    waypoint_spacing_m: f64,
    res: f64,
    origin_x: f64,
    origin_y: f64,
) -> Vec<(f64, f64)> {
    if !comp_mask.iter().any(|&v| v) { return vec![]; }

    let dist = bfs_dist(comp_mask);
    let layer_step = ((strip_width_m / res).round() as usize).max(1);
    let spacing_cells = ((waypoint_spacing_m / res).round() as usize).max(1);
    let max_dist = comp_mask.iter()
        .zip(dist.iter())
        .filter(|(&m, _)| m)
        .map(|(_, &d)| d)
        .max()
        .unwrap_or(0);

    let layer_masks = iter_spiral_layer_masks(comp_mask, &dist, layer_step, max_dist);

    let mut path_cells: Vec<(usize, usize)> = Vec::new();
    let mut previous_cell: Option<(usize, usize)> = None;

    for layer_mask in &layer_masks {
        let raw_loops = boundary_cell_loops(layer_mask.view());
        let sorted_loops = sort_cell_paths(raw_loops, previous_cell);

        for loop_cells in sorted_loops {
            let ordered = rotate_path_near_previous(loop_cells, previous_cell);
            let sampled = sample_cells_by_spacing(&ordered, spacing_cells, comp_mask);
            if sampled.is_empty() { continue; }

            if let Some(prev) = previous_cell {
                if prev != sampled[0] {
                    if let Some(bridge) = astar_4(prev, sampled[0], comp_mask) {
                        append_cell_path(&mut path_cells, &bridge);
                    }
                }
            }

            append_cell_path(&mut path_cells, &sampled);
            previous_cell = path_cells.last().copied();
        }
    }

    path_cells.iter().map(|&(r, c)| {
        (origin_x + (c as f64 + 0.5) * res, origin_y + (r as f64 + 0.5) * res)
    }).collect()
}

// ── Nearest entry ─────────────────────────────────────────────────────────────

fn nearest_entry(
    last_pt: (f64, f64),
    comp_mask: ArrayView2<bool>,
    res: f64,
    origin_x: f64,
    origin_y: f64,
) -> (usize, usize) {
    let lr = ((last_pt.1 - origin_y) / res) as i64;
    let lc = ((last_pt.0 - origin_x) / res) as i64;
    let h = comp_mask.nrows();
    let w = comp_mask.ncols();

    let mut best_dist = i64::MAX;
    let mut best_rc = (0usize, 0usize);

    for r in 0..h {
        for c in 0..w {
            if !comp_mask[[r, c]] { continue; }
            let d = (r as i64 - lr).abs() + (c as i64 - lc).abs();
            if d < best_dist {
                best_dist = d;
                best_rc = (r, c);
            }
        }
    }
    best_rc
}

// ── Invalid segment detection ─────────────────────────────────────────────────

fn find_invalid_segs(
    points: &[(f64, f64)],
    safe: ArrayView2<bool>,
    res: f64,
    origin_x: f64,
    origin_y: f64,
) -> Vec<(usize, usize)> {
    if points.len() < 2 { return vec![]; }
    let sm = SafeMap { grid: safe, resolution: res, origin_x, origin_y };
    validate_path_rs(points, &sm).invalid_segments
}

// ── Public entrypoint ─────────────────────────────────────────────────────────

pub fn plan_spiral_coverage_rs(
    safe: ArrayView2<bool>,
    strip_width_m: f64,
    waypoint_spacing_m: f64,
    res: f64,
    h: usize,
    w: usize,
    origin_x: f64,
    origin_y: f64,
) -> (Vec<(f64, f64)>, Vec<(f64, f64)>, Vec<(usize, usize)>) {
    if !safe.iter().any(|&v| v) {
        return (vec![], vec![], vec![]);
    }

    let components = connected_components_4(safe);
    let mut all_points: Vec<(f64, f64)> = Vec::new();
    let mut split_points: Vec<(f64, f64)> = Vec::new();
    let mut spiral_last: Option<(f64, f64)> = None;

    for (_comp_id, comp_mask) in components.iter().enumerate() {
        let pts = onion_layer_spiral(
            comp_mask.view(), strip_width_m, waypoint_spacing_m, res, origin_x, origin_y,
        );
        if pts.is_empty() { continue; }

        // Inter-component bridge
        if let Some(last_pt) = spiral_last {
            let goal_rc = nearest_entry(last_pt, comp_mask.view(), res, origin_x, origin_y);
            let start_r = ((last_pt.1 - origin_y) / res) as i64;
            let start_c = ((last_pt.0 - origin_x) / res) as i64;
            if start_r >= 0 && start_r < h as i64 && start_c >= 0 && start_c < w as i64 {
                let start_rc = (start_r as usize, start_c as usize);
                if let Some(bridge) = astar_4(start_rc, goal_rc, safe) {
                    for &(r, c) in &bridge {
                        let wp = (origin_x + (c as f64 + 0.5) * res, origin_y + (r as f64 + 0.5) * res);
                        all_points.push(wp);
                    }
                }
                // If A* bridge fails (genuinely disconnected regions), emit nothing.
                // Emitting a direct fallback waypoint creates a known-invalid segment
                // that the ConnectorPlanner also cannot resolve, causing planning failure.
            }
        }

        all_points.extend_from_slice(&pts);
        if let Some(&last) = pts.last() {
            split_points.push(last);
            spiral_last = Some(last);
        }
    }

    let invalid_segs = find_invalid_segs(&all_points, safe, res, origin_x, origin_y);
    (all_points, split_points, invalid_segs)
}

// ── PyO3 binding ──────────────────────────────────────────────────────────────

#[pyfunction]
pub fn py_generate_coverage_spiral_path(
    grid: PyReadonlyArray2<bool>,
    strip_width_m: f64,
    waypoint_spacing_m: f64,
    res: f64,
    h: usize,
    w: usize,
    origin_x: f64,
    origin_y: f64,
) -> (Vec<(f64, f64)>, Vec<(f64, f64)>, Vec<(usize, usize)>) {
    plan_spiral_coverage_rs(
        grid.as_array(), strip_width_m, waypoint_spacing_m, res, h, w, origin_x, origin_y,
    )
}

pub fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(py_generate_coverage_spiral_path, m)?)?;
    Ok(())
}
