use numpy::PyReadonlyArray2;
use pyo3::prelude::*;

use crate::path_validator::validate_path_rs;
use crate::types::SafeMap;

const MIN_U_TURN_RADIUS_M: f64 = 0.35;
const SAME_POINT_TOL: f64 = 1e-9;

// ── Internal helpers ───────────────────────────────────────────────────────────

fn same_point(a: (f64, f64), b: (f64, f64)) -> bool {
    (a.0 - b.0).abs() <= SAME_POINT_TOL && (a.1 - b.1).abs() <= SAME_POINT_TOL
}

fn append_unique(pts: &mut Vec<(f64, f64)>, p: (f64, f64)) {
    if pts.last().map_or(true, |&last| !same_point(last, p)) {
        pts.push(p);
    }
}

fn extend_unique(pts: &mut Vec<(f64, f64)>, new_pts: &[(f64, f64)]) {
    for &p in new_pts {
        append_unique(pts, p);
    }
}

// ── Safe-run extraction ───────────────────────────────────────────────────────

fn extract_safe_runs(
    safe: &ndarray::ArrayView2<bool>,
    mc: usize,
    h: usize,
) -> Vec<(usize, usize)> {
    let mut runs = Vec::new();
    let mut start: Option<usize> = None;
    for i in 0..h {
        let ok = safe[[i, mc]];
        let is_last = i == h - 1;
        if ok && start.is_none() {
            start = Some(i);
        }
        if (!ok || is_last) && start.is_some() {
            let s = start.unwrap();
            let e = if !ok { i - 1 } else { i };
            runs.push((s, e));
            start = None;
        }
    }
    runs
}

// ── U-turn construction ───────────────────────────────────────────────────────

fn trim_run_to_turn_y(run: &[(f64, f64)], turn_y: f64, keep_below: bool) -> Vec<(f64, f64)> {
    if keep_below {
        run.iter().filter(|&&(_, y)| y <= turn_y).copied().collect()
    } else {
        run.iter().filter(|&&(_, y)| y >= turn_y).copied().collect()
    }
}

fn trim_run_from_turn_y(run: &[(f64, f64)], turn_y: f64, skip_above: bool) -> Vec<(f64, f64)> {
    if skip_above {
        run.iter().filter(|&&(_, y)| y <= turn_y).copied().collect()
    } else {
        run.iter().filter(|&&(_, y)| y >= turn_y).copied().collect()
    }
}

fn with_endpoint(mut pts: Vec<(f64, f64)>, ep: (f64, f64)) -> Vec<(f64, f64)> {
    append_unique(&mut pts, ep);
    pts
}

fn with_startpoint(mut pts: Vec<(f64, f64)>, sp: (f64, f64)) -> Vec<(f64, f64)> {
    if pts.first().map_or(false, |&p| same_point(p, sp)) {
        return pts;
    }
    pts.insert(0, sp);
    pts
}

fn make_u_turn(
    current_run: &[(f64, f64)],
    next_run: &[(f64, f64)],
    sm: &SafeMap,
    waypoint_spacing_m: f64,
    res: f64,
) -> Option<(Vec<(f64, f64)>, Vec<(f64, f64)>, Vec<(f64, f64)>)> {
    if current_run.len() < 2 || next_run.len() < 2 {
        return None;
    }

    let incoming_dy = current_run.last().unwrap().1 - current_run[0].1;
    let outgoing_dy = next_run.last().unwrap().1 - next_run[0].1;

    if incoming_dy.abs() < res || outgoing_dy.abs() < res {
        return None;
    }
    if incoming_dy * outgoing_dy >= 0.0 {
        return None;
    }

    let start = *current_run.last().unwrap();
    let end = next_run[0];
    let lane_gap = (end.0 - start.0).abs();
    if lane_gap < res * 0.5 {
        return None;
    }
    if (start.1 - end.1).abs() > (waypoint_spacing_m.max(res)) * 1.5 {
        return None;
    }

    let radius = lane_gap / 2.0;
    if radius < MIN_U_TURN_RADIUS_M {
        return None;
    }

    let boundary_y = (start.1 + end.1) / 2.0;
    let top_turn = incoming_dy > 0.0;
    let center_y = if top_turn { boundary_y - radius } else { boundary_y + radius };

    let current_prefix = with_endpoint(
        trim_run_to_turn_y(current_run, center_y, top_turn),
        (start.0, center_y),
    );
    let next_suffix = with_startpoint(
        trim_run_from_turn_y(next_run, center_y, top_turn),
        (end.0, center_y),
    );

    let samples =
        (((std::f64::consts::PI * radius) / waypoint_spacing_m.max(res)).ceil() as usize + 1)
            .max(3);

    let mut turn_points: Vec<(f64, f64)> = Vec::with_capacity(samples);
    for idx in 0..samples {
        let t = idx as f64 / (samples - 1) as f64;
        let x = start.0 + (end.0 - start.0) * t;
        let arc_offset = radius * (std::f64::consts::PI * t).sin();
        let y = if top_turn { center_y + arc_offset } else { center_y - arc_offset };
        turn_points.push((x, y));
    }

    // Validate the arc
    let mut validation_pts: Vec<(f64, f64)> = Vec::new();
    if let Some(&last) = current_prefix.last() {
        validation_pts.push(last);
    }
    validation_pts.extend_from_slice(&turn_points);
    if next_suffix.len() > 1 {
        validation_pts.push(next_suffix[1]);
    }

    if !validate_path_rs(&validation_pts, sm).valid {
        return None;
    }

    Some((current_prefix, turn_points, next_suffix))
}

// ── Path builder ──────────────────────────────────────────────────────────────

fn build_path_with_u_turns(
    runs: &[Vec<(f64, f64)>],
    sm: &SafeMap,
    waypoint_spacing_m: f64,
    res: f64,
) -> (Vec<(f64, f64)>, Vec<(f64, f64)>) {
    if runs.is_empty() {
        return (vec![], vec![]);
    }

    let mut points: Vec<(f64, f64)> = Vec::new();
    let mut split_points: Vec<(f64, f64)> = Vec::new();
    let mut current_run = runs[0].clone();

    for next_run_raw in &runs[1..] {
        let next_run = next_run_raw.clone();
        match make_u_turn(&current_run, &next_run, sm, waypoint_spacing_m, res) {
            None => {
                extend_unique(&mut points, &current_run);
                if let Some(&last) = current_run.last() {
                    split_points.push(last);
                }
                current_run = next_run;
            }
            Some((prefix, turn_pts, suffix)) => {
                extend_unique(&mut points, &prefix);
                extend_unique(&mut points, &turn_pts);
                if let Some(&last) = turn_pts.last() {
                    split_points.push(last);
                }
                current_run = suffix;
            }
        }
    }

    extend_unique(&mut points, &current_run);
    if let Some(&last) = current_run.last() {
        split_points.push(last);
    }

    (points, split_points)
}

// ── Invalid segment detection ─────────────────────────────────────────────────

fn find_invalid_segments(
    points: &[(f64, f64)],
    sm: &SafeMap,
) -> Vec<(usize, usize)> {
    if points.len() < 2 {
        return vec![];
    }
    validate_path_rs(points, sm).invalid_segments
}

// ── Main zigzag ───────────────────────────────────────────────────────────────

pub fn generate_coverage_zigzag_path_rs(
    safe: ndarray::ArrayView2<bool>,
    strip_width_m: f64,
    waypoint_spacing_m: f64,
    res: f64,
    h: usize,
    w: usize,
    origin_x: f64,
    origin_y: f64,
    angle_deg: f64,
) -> (Vec<(f64, f64)>, Vec<(f64, f64)>, Vec<(usize, usize)>) {
    let sm = SafeMap { grid: safe, resolution: res, origin_x, origin_y };

    if angle_deg.abs() < 1e-6 {
        // ── Axis-aligned case ─────────────────────────────────────────────────
        let strip_cols = ((strip_width_m / res).round() as usize).max(1);
        let midcols: Vec<usize> = (strip_cols / 2..w).step_by(strip_cols).collect();

        let spacing = waypoint_spacing_m;
        let mut runs: Vec<Vec<(f64, f64)>> = Vec::new();
        let mut reverse = false;

        for &mc in &midcols {
            let segments = extract_safe_runs(&safe, mc, h);
            let segs: Vec<_> = if reverse {
                segments.iter().rev().cloned().collect()
            } else {
                segments
            };

            for (s, e) in segs {
                let y0 = origin_y + (s as f64 + 0.5) * res;
                let y1 = origin_y + (e as f64 + 0.5) * res;
                let x = origin_x + (mc as f64 + 0.5) * res;
                let step = spacing.max(res);

                let ys: Vec<f64> = if y1 >= y0 {
                    let mut v: Vec<f64> = Vec::new();
                    let mut y = y0;
                    while y < y1 {
                        v.push(y);
                        y += step;
                    }
                    v.push(y1);
                    if reverse { v.into_iter().rev().collect() } else { v }
                } else {
                    let mut v: Vec<f64> = Vec::new();
                    let mut y = y0;
                    while y > y1 {
                        v.push(y);
                        y -= step;
                    }
                    v.push(y1);
                    if reverse { v.into_iter().rev().collect() } else { v }
                };

                runs.push(ys.into_iter().map(|y| (x, y)).collect());
            }
            reverse = !reverse;
        }

        let eff_spacing = spacing.max(res);
        let (points, split_points) = build_path_with_u_turns(&runs, &sm, eff_spacing, res);
        let invalid_segments = find_invalid_segments(&points, &sm);
        return (points, split_points, invalid_segments);
    }

    // ── Rotated case ──────────────────────────────────────────────────────────
    let angle_rad = angle_deg.to_radians();
    let center_x = origin_x + w as f64 * res / 2.0;
    let center_y = origin_y + h as f64 * res / 2.0;

    let c = (-angle_rad).cos();
    let s = (-angle_rad).sin();

    // Collect rotated coords of all safe cells
    let mut all_rot: Vec<(f64, f64)> = Vec::new();
    for r in 0..h {
        for col in 0..w {
            if !safe[[r, col]] { continue; }
            let wx = origin_x + (col as f64 + 0.5) * res;
            let wy = origin_y + (r as f64 + 0.5) * res;
            let dx = wx - center_x;
            let dy = wy - center_y;
            let rx = c * dx - s * dy + center_x;
            let ry = s * dx + c * dy + center_y;
            all_rot.push((rx, ry));
        }
    }

    if all_rot.is_empty() {
        return (vec![], vec![], vec![]);
    }

    let min_x = all_rot.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
    let max_x = all_rot.iter().map(|p| p.0).fold(f64::NEG_INFINITY, f64::max);
    let width_rot = max_x - min_x;
    let n_strips = ((width_rot / strip_width_m).floor() as usize).max(1);

    let strip_centers: Vec<f64> = if n_strips == 1 {
        vec![(min_x + max_x) / 2.0]
    } else {
        (0..n_strips)
            .map(|i| {
                min_x + strip_width_m / 2.0
                    + i as f64 * (width_rot - strip_width_m) / (n_strips - 1) as f64
            })
            .collect()
    };

    let mut points: Vec<(f64, f64)> = Vec::new();
    let mut split_points: Vec<(f64, f64)> = Vec::new();
    let mut reverse = false;

    for &scx in &strip_centers {
        let mut candidates: Vec<(f64, f64)> = all_rot
            .iter()
            .filter(|&&(rx, _)| (rx - scx).abs() <= strip_width_m / 2.0)
            .copied()
            .collect();

        if candidates.is_empty() {
            reverse = !reverse;
            continue;
        }

        candidates.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
        if reverse {
            candidates.reverse();
        }

        let min_dist = waypoint_spacing_m.max(res) * 0.5;
        let mut prev: Option<(f64, f64)> = None;
        let mut strip_last: Option<(f64, f64)> = None;

        for pt in candidates {
            if let Some(p) = prev {
                let d = ((pt.0 - p.0).powi(2) + (pt.1 - p.1).powi(2)).sqrt();
                if d < min_dist { continue; }
            }
            prev = Some(pt);
            strip_last = Some(pt);
            points.push(pt);
        }
        if let Some(last) = strip_last {
            split_points.push(last);
        }
        reverse = !reverse;
    }

    // Rotate back
    let ci = angle_rad.cos();
    let si = angle_rad.sin();
    let rotate_back = |p: (f64, f64)| -> (f64, f64) {
        let dx = p.0 - center_x;
        let dy = p.1 - center_y;
        (ci * dx - si * dy + center_x, si * dx + ci * dy + center_y)
    };
    let points: Vec<(f64, f64)> = points.iter().map(|&p| rotate_back(p)).collect();
    let split_points: Vec<(f64, f64)> = split_points.iter().map(|&p| rotate_back(p)).collect();

    let invalid_segments = find_invalid_segments(&points, &sm);
    (points, split_points, invalid_segments)
}

// ── PyO3 binding ──────────────────────────────────────────────────────────────

#[pyfunction]
#[pyo3(signature = (grid, strip_width_m, waypoint_spacing_m, res, h, w, origin_x, origin_y, angle_deg=0.0))]
pub fn py_generate_coverage_zigzag_path(
    grid: PyReadonlyArray2<bool>,
    strip_width_m: f64,
    waypoint_spacing_m: f64,
    res: f64,
    h: usize,
    w: usize,
    origin_x: f64,
    origin_y: f64,
    angle_deg: f64,
) -> (Vec<(f64, f64)>, Vec<(f64, f64)>, Vec<(usize, usize)>) {
    let safe = grid.as_array();
    generate_coverage_zigzag_path_rs(
        safe, strip_width_m, waypoint_spacing_m, res, h, w, origin_x, origin_y, angle_deg,
    )
}

pub fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(py_generate_coverage_zigzag_path, m)?)?;
    Ok(())
}
