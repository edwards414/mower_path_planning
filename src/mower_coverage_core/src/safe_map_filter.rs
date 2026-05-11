use std::collections::VecDeque;

use ndarray::{Array2, ArrayView2};
use numpy::{PyArray2, PyReadonlyArray2, ToPyArray};
use pyo3::prelude::*;

const NEIGHBOURS_8: [(i32, i32); 8] =
    [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)];

// ── Internal ───────────────────────────────────────────────────────────────────

pub fn connected_components_8(safe: ArrayView2<bool>) -> Vec<Vec<(usize, usize)>> {
    let h = safe.nrows();
    let w = safe.ncols();
    let mut seen = Array2::<bool>::default((h, w));
    let mut components: Vec<Vec<(usize, usize)>> = Vec::new();

    for r in 0..h {
        for c in 0..w {
            if !safe[[r, c]] || seen[[r, c]] {
                continue;
            }
            let mut cells: Vec<(usize, usize)> = Vec::new();
            let mut queue: VecDeque<(usize, usize)> = VecDeque::new();
            seen[[r, c]] = true;
            queue.push_back((r, c));

            while let Some((rr, cc)) = queue.pop_front() {
                cells.push((rr, cc));
                for &(dr, dc) in &NEIGHBOURS_8 {
                    let nr = rr as i32 + dr;
                    let nc = cc as i32 + dc;
                    if nr < 0 || nr >= h as i32 || nc < 0 || nc >= w as i32 {
                        continue;
                    }
                    let (nr, nc) = (nr as usize, nc as usize);
                    if seen[[nr, nc]] || !safe[[nr, nc]] {
                        continue;
                    }
                    seen[[nr, nc]] = true;
                    queue.push_back((nr, nc));
                }
            }
            components.push(cells);
        }
    }
    // Sort largest first (matches Python behaviour)
    components.sort_by(|a, b| b.len().cmp(&a.len()));
    components
}

pub fn filter_safe_components_rs(
    safe: ArrayView2<bool>,
    resolution: f64,
    min_area_m2: f64,
    keep_largest_only: bool,
) -> (Array2<bool>, Vec<usize>, Vec<usize>) {
    let h = safe.nrows();
    let w = safe.ncols();
    let components = connected_components_8(safe);
    let all_sizes: Vec<usize> = components.iter().map(|c| c.len()).collect();

    if components.is_empty() {
        return (Array2::default((h, w)), vec![], vec![]);
    }

    let min_cells = ((min_area_m2 / (resolution * resolution)).ceil() as usize).max(1);
    let mut kept: Vec<&Vec<(usize, usize)>> =
        components.iter().filter(|c| c.len() >= min_cells).collect();

    if keep_largest_only && !kept.is_empty() {
        kept = vec![kept[0]];
    }

    let mut filtered = Array2::<bool>::default((h, w));
    for comp in &kept {
        for &(r, c) in comp.iter() {
            filtered[[r, c]] = true;
        }
    }

    let mut kept_sizes: Vec<usize> = kept.iter().map(|c| c.len()).collect();
    kept_sizes.sort_by(|a, b| b.cmp(a));

    (filtered, all_sizes, kept_sizes)
}

// ── PyO3 bindings ──────────────────────────────────────────────────────────────

#[pyfunction]
pub fn py_filter_safe_components(
    py: Python<'_>,
    grid: PyReadonlyArray2<bool>,
    resolution: f64,
    min_area_m2: f64,
    keep_largest_only: bool,
) -> PyResult<(Py<PyArray2<bool>>, Vec<usize>, Vec<usize>)> {
    let (filtered, all_sizes, kept_sizes) =
        filter_safe_components_rs(grid.as_array(), resolution, min_area_m2, keep_largest_only);
    Ok((filtered.to_pyarray_bound(py).unbind(), all_sizes, kept_sizes))
}

pub fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(py_filter_safe_components, m)?)?;
    Ok(())
}
