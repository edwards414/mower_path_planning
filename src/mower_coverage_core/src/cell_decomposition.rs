//! Boustrophedon Cell Decomposition (row-sweep variant).
//!
//! Sweeps along rows (the y axis). Each contiguous safe run in a row is a free
//! interval. Critical events (enter, exit, split, merge) define the cell
//! boundaries.
//!
//! Port of the Python `mower_mission/coverage/cell_decomposition.py` and the
//! `CoverageCell` type from `coverage/types.py` (commit dff480b). Nothing calls
//! it yet, exactly as with the Python original: it keeps the BCD work for the
//! planned per-cell coverage optimisation. `tests/cell_decomposition.rs` and
//! `tests/python_reference_oracle.rs` pin it to the Python behaviour.

use std::collections::{BTreeMap, BTreeSet};
use std::hash::{Hash, Hasher};

use ndarray::{Array2, ArrayView1};

use crate::types::SafeMap;

/// A BCD cell with its occupancy mask and planning metadata.
///
/// Like the Python dataclass (`eq=False` plus a custom `__eq__`/`__hash__`),
/// equality and hashing use `cell_id` only.
#[derive(Debug, Clone)]
pub struct CoverageCell {
    pub cell_id: usize,
    /// Same shape as the safe-map grid; true for the cells of this BCD cell.
    pub mask: Array2<bool>,
    /// `(row_min, col_min, row_max, col_max)`, inclusive.
    pub bbox: (usize, usize, usize, usize),
    pub area_m2: f64,
    /// World `(x, y)` of the mean cell centre.
    pub centroid_xy: (f64, f64),
    /// World `(x, y)` candidates; `decompose` sets the middle of the first strip.
    pub entry_candidates: Vec<(f64, f64)>,
    /// World `(x, y)` candidates; `decompose` sets the middle of the last strip.
    pub exit_candidates: Vec<(f64, f64)>,
    pub valid: bool,
    pub invalid_reason: String,
}

impl PartialEq for CoverageCell {
    fn eq(&self, other: &Self) -> bool {
        self.cell_id == other.cell_id
    }
}

impl Eq for CoverageCell {}

impl Hash for CoverageCell {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.cell_id.hash(state);
    }
}

/// Adjacency of the decomposition: `cell_id -> sorted adjacent cell_ids`.
/// Every returned cell has an entry (possibly empty), in `cell_id` order.
pub type CellGraph = BTreeMap<usize, Vec<usize>>;

#[derive(Debug, Clone, Copy)]
struct Strip {
    row: usize,
    col_start: usize,
    col_end: usize,
}

/// `(col_start, col_end)` inclusive pairs for each contiguous safe run.
fn free_intervals(row_data: ArrayView1<bool>) -> Vec<(usize, usize)> {
    let mut intervals = Vec::new();
    let mut in_iv = false;
    let mut start = 0usize;
    for (c, &v) in row_data.iter().enumerate() {
        if v && !in_iv {
            start = c;
            in_iv = true;
        } else if !v && in_iv {
            intervals.push((start, c - 1));
            in_iv = false;
        }
    }
    if in_iv {
        intervals.push((start, row_data.len() - 1));
    }
    intervals
}

fn overlaps(a: (usize, usize), b: (usize, usize)) -> bool {
    a.0 <= b.1 && b.0 <= a.1
}

fn link(adjacency: &mut BTreeMap<usize, BTreeSet<usize>>, a: usize, b: usize) {
    adjacency.entry(a).or_default().insert(b);
    adjacency.entry(b).or_default().insert(a);
}

/// Row-sweep BCD: decompose the safe map into topologically simple cells.
///
/// Returns the cells in `cell_id` order and the cell graph. Adjacency is
/// established at split and merge critical events (and between the siblings
/// of a split).
pub fn decompose(safe_map: &SafeMap) -> (Vec<CoverageCell>, CellGraph) {
    let grid = safe_map.grid;
    let (h, w) = grid.dim();

    // cell_strips[cid] = strips of cell `cid`; ids are allocated in order.
    let mut cell_strips: Vec<Vec<Strip>> = Vec::new();
    let mut adjacency: BTreeMap<usize, BTreeSet<usize>> = BTreeMap::new();

    let mut prev_ivs: Vec<(usize, usize)> = Vec::new();
    let mut prev_active: Vec<usize> = Vec::new(); // cell_id per prev interval

    for r in 0..h {
        let curr_ivs = free_intervals(grid.row(r));

        // Overlap maps for this row transition.
        let mut prev_to_curr: Vec<Vec<usize>> = vec![Vec::new(); prev_ivs.len()];
        let mut curr_to_prev: Vec<Vec<usize>> = vec![Vec::new(); curr_ivs.len()];
        for (pi, &piv) in prev_ivs.iter().enumerate() {
            for (ci, &civ) in curr_ivs.iter().enumerate() {
                if overlaps(piv, civ) {
                    prev_to_curr[pi].push(ci);
                    curr_to_prev[ci].push(pi);
                }
            }
        }

        let mut curr_active: Vec<usize> = Vec::with_capacity(curr_ivs.len());
        for (ci, &civ) in curr_ivs.iter().enumerate() {
            let prev_list = &curr_to_prev[ci];
            let continues = prev_list.len() == 1 && prev_to_curr[prev_list[0]].len() == 1;
            let cid = if continues {
                prev_active[prev_list[0]] // CONTINUE 1-to-1
            } else {
                // ENTER (no parent), SPLIT child or MERGE child
                cell_strips.push(Vec::new());
                cell_strips.len() - 1
            };
            curr_active.push(cid);
            cell_strips[cid].push(Strip { row: r, col_start: civ.0, col_end: civ.1 });
        }

        // Adjacency: SPLIT (1 prev -> N curr): parent <-> child, child <-> child.
        for (pi, curr_list) in prev_to_curr.iter().enumerate() {
            if curr_list.len() >= 2 {
                let parent = prev_active[pi];
                let children: Vec<usize> = curr_list.iter().map(|&ci| curr_active[ci]).collect();
                for &ch in &children {
                    link(&mut adjacency, ch, parent);
                }
                for (i, &ca) in children.iter().enumerate() {
                    for &cb in &children[i + 1..] {
                        link(&mut adjacency, ca, cb);
                    }
                }
            }
        }

        // Adjacency: MERGE (N prev -> 1 curr): child <-> each parent.
        for (ci, prev_list) in curr_to_prev.iter().enumerate() {
            if prev_list.len() >= 2 {
                let child = curr_active[ci];
                for &pi in prev_list {
                    link(&mut adjacency, child, prev_active[pi]);
                }
            }
        }

        prev_ivs = curr_ivs;
        prev_active = curr_active;
    }

    // ── Build CoverageCell objects ────────────────────────────────────────────
    let res = safe_map.resolution;
    let ox = safe_map.origin_x;
    let oy = safe_map.origin_y;

    let mut cells: Vec<CoverageCell> = Vec::with_capacity(cell_strips.len());
    for (cid, strips) in cell_strips.iter().enumerate() {
        let (Some(fs), Some(ls)) = (strips.first(), strips.last()) else {
            continue;
        };

        let mut mask = Array2::<bool>::default((h, w));
        for s in strips {
            for c in s.col_start..=s.col_end {
                mask[[s.row, c]] = true;
            }
        }

        let (mut rmin, mut cmin) = (usize::MAX, usize::MAX);
        let (mut rmax, mut cmax) = (0usize, 0usize);
        let (mut row_sum, mut col_sum, mut count) = (0usize, 0usize, 0usize);
        for ((r, c), &v) in mask.indexed_iter() {
            if !v {
                continue;
            }
            rmin = rmin.min(r);
            rmax = rmax.max(r);
            cmin = cmin.min(c);
            cmax = cmax.max(c);
            row_sum += r;
            col_sum += c;
            count += 1;
        }
        if count == 0 {
            continue;
        }

        // Same operation order as the Python (float(sum) * res * res, and the
        // numpy mean of integer indices, which is exact here).
        let area_m2 = count as f64 * res * res;
        let col_mean = col_sum as f64 / count as f64;
        let row_mean = row_sum as f64 / count as f64;
        let cx = ox + (col_mean + 0.5) * res;
        let cy = oy + (row_mean + 0.5) * res;

        let entry_xy = (
            ox + ((fs.col_start + fs.col_end) as f64 / 2.0 + 0.5) * res,
            oy + (fs.row as f64 + 0.5) * res,
        );
        let exit_xy = (
            ox + ((ls.col_start + ls.col_end) as f64 / 2.0 + 0.5) * res,
            oy + (ls.row as f64 + 0.5) * res,
        );

        cells.push(CoverageCell {
            cell_id: cid,
            mask,
            bbox: (rmin, cmin, rmax, cmax),
            area_m2,
            centroid_xy: (cx, cy),
            entry_candidates: vec![entry_xy],
            exit_candidates: vec![exit_xy],
            valid: true,
            invalid_reason: String::new(),
        });
    }

    let cell_graph: CellGraph = cells
        .iter()
        .map(|cell| {
            let neighbours = adjacency
                .get(&cell.cell_id)
                .map(|set| set.iter().copied().collect())
                .unwrap_or_default();
            (cell.cell_id, neighbours)
        })
        .collect();

    (cells, cell_graph)
}
