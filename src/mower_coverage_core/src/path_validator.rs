use numpy::PyReadonlyArray2;
use pyo3::prelude::*;

use crate::types::{SafeMap, ValidationResult, world_to_grid};

// ── Internal helpers ───────────────────────────────────────────────────────────

/// Enumerate all integer grid cells on the line (r0,c0)→(r1,c1).
/// Endpoint (r1,c1) is always included.  Matches Python `_bresenham`.
pub fn bresenham(r0: i64, c0: i64, r1: i64, c1: i64) -> Vec<(i64, i64)> {
    let mut cells = Vec::new();
    let dr = (r1 - r0).abs();
    let dc = (c1 - c0).abs();
    let sr: i64 = if r1 > r0 { 1 } else { -1 };
    let sc: i64 = if c1 > c0 { 1 } else { -1 };
    let (mut r, mut c) = (r0, c0);

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

fn in_bounds(r: i64, c: i64, h: i64, w: i64) -> bool {
    r >= 0 && r < h && c >= 0 && c < w
}

pub fn is_point_safe_rs(x: f64, y: f64, sm: &SafeMap) -> bool {
    let (r, c) = world_to_grid(x, y, sm);
    let h = sm.grid.nrows() as i64;
    let w = sm.grid.ncols() as i64;
    in_bounds(r, c, h, w) && sm.grid[[r as usize, c as usize]]
}

pub fn is_segment_safe_rs(p0: (f64, f64), p1: (f64, f64), sm: &SafeMap) -> bool {
    let (r0, c0) = world_to_grid(p0.0, p0.1, sm);
    let (r1, c1) = world_to_grid(p1.0, p1.1, sm);
    let h = sm.grid.nrows() as i64;
    let w = sm.grid.ncols() as i64;
    for (r, c) in bresenham(r0, c0, r1, c1) {
        if !in_bounds(r, c, h, w) || !sm.grid[[r as usize, c as usize]] {
            return false;
        }
    }
    true
}

pub fn validate_path_rs(points: &[(f64, f64)], sm: &SafeMap) -> ValidationResult {
    let invalid_points: Vec<usize> = points
        .iter()
        .enumerate()
        .filter(|(_, &(x, y))| !is_point_safe_rs(x, y, sm))
        .map(|(i, _)| i)
        .collect();

    let invalid_segments: Vec<(usize, usize)> = (0..points.len().saturating_sub(1))
        .filter(|&i| !is_segment_safe_rs(points[i], points[i + 1], sm))
        .map(|i| (i, i + 1))
        .collect();

    let valid = invalid_points.is_empty() && invalid_segments.is_empty();
    let msg = format!(
        "{} unsafe point(s), {} unsafe segment(s)",
        invalid_points.len(),
        invalid_segments.len()
    );
    ValidationResult { valid, invalid_points, invalid_segments, message: msg }
}

// ── PyO3 bindings ──────────────────────────────────────────────────────────────

#[pyfunction]
pub fn py_is_point_safe(
    x: f64,
    y: f64,
    grid: PyReadonlyArray2<bool>,
    resolution: f64,
    origin_x: f64,
    origin_y: f64,
) -> bool {
    let sm = SafeMap { grid: grid.as_array(), resolution, origin_x, origin_y };
    is_point_safe_rs(x, y, &sm)
}

#[pyfunction]
pub fn py_is_segment_safe(
    p0: (f64, f64),
    p1: (f64, f64),
    grid: PyReadonlyArray2<bool>,
    resolution: f64,
    origin_x: f64,
    origin_y: f64,
) -> bool {
    let sm = SafeMap { grid: grid.as_array(), resolution, origin_x, origin_y };
    is_segment_safe_rs(p0, p1, &sm)
}

#[pyfunction]
pub fn py_validate_path(
    points: Vec<(f64, f64)>,
    grid: PyReadonlyArray2<bool>,
    resolution: f64,
    origin_x: f64,
    origin_y: f64,
) -> (bool, Vec<usize>, Vec<(usize, usize)>, String) {
    let sm = SafeMap { grid: grid.as_array(), resolution, origin_x, origin_y };
    let r = validate_path_rs(&points, &sm);
    (r.valid, r.invalid_points, r.invalid_segments, r.message)
}

pub fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(py_is_point_safe, m)?)?;
    m.add_function(wrap_pyfunction!(py_is_segment_safe, m)?)?;
    m.add_function(wrap_pyfunction!(py_validate_path, m)?)?;
    Ok(())
}
