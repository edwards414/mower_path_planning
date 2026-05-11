use ndarray::ArrayView2;

pub struct SafeMap<'a> {
    pub grid: ArrayView2<'a, bool>,
    pub resolution: f64,
    pub origin_x: f64,
    pub origin_y: f64,
}

pub struct ValidationResult {
    pub valid: bool,
    pub invalid_points: Vec<usize>,
    pub invalid_segments: Vec<(usize, usize)>,
    pub message: String,
}

/// Convert world (x, y) → grid (row, col).  Matches Python `int(...)` truncation.
#[inline]
pub fn world_to_grid(x: f64, y: f64, sm: &SafeMap) -> (i64, i64) {
    let col = ((x - sm.origin_x) / sm.resolution) as i64;
    let row = ((y - sm.origin_y) / sm.resolution) as i64;
    (row, col)
}

/// Convert grid (row, col) to world centre of that cell.
#[inline]
pub fn grid_to_world(r: usize, c: usize, sm: &SafeMap) -> (f64, f64) {
    let wx = sm.origin_x + (c as f64 + 0.5) * sm.resolution;
    let wy = sm.origin_y + (r as f64 + 0.5) * sm.resolution;
    (wx, wy)
}
