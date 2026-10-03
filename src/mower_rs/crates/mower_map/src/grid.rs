//! ROS-free occupancy-grid maths of the map manager: the helpers of
//! `nav_map_fusion.py`, `map_safety.py` and `image_mask_import.py` plus the
//! pure parts of `map_manage_node.py` (risk / zone / channel rasterisation),
//! each written against the numpy + OpenCV semantics of the Python node so the
//! resulting `int8` data is identical.

use std::f64::consts::PI;

use base64::Engine;

use crate::raster::{self, Border, Img};

/// OccupancyGrid geometry the helpers need (origin yaw already extracted).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Geometry {
    pub width: u32,
    pub height: u32,
    pub resolution: f64,
    pub origin_x: f64,
    pub origin_y: f64,
    pub origin_yaw: f64,
}

impl Geometry {
    pub fn cells(&self) -> usize {
        self.width as usize * self.height as usize
    }
}

/// `yaw_from_quaternion(q)`.
pub fn yaw_from_quaternion(x: f64, y: f64, z: f64, w: f64) -> f64 {
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    siny_cosp.atan2(cosy_cosp)
}

/// `world_to_grid_index`: bounded (column, row) of a world point, rejecting
/// out-of-bounds points instead of clamping them.
pub fn world_to_grid_index(x: f64, y: f64, g: &Geometry) -> Result<(i32, i32), String> {
    let values = [x, y, g.origin_x, g.origin_y, g.origin_yaw, g.resolution];
    if !values.iter().all(|v| v.is_finite()) {
        return Err("grid coordinates must be finite".into());
    }
    if g.resolution <= 0.0 || g.width == 0 || g.height == 0 {
        return Err("grid geometry must be positive".into());
    }
    let dx = x - g.origin_x;
    let dy = y - g.origin_y;
    let (sin_yaw, cos_yaw) = g.origin_yaw.sin_cos();
    let local_x = cos_yaw * dx + sin_yaw * dy;
    let local_y = -sin_yaw * dx + cos_yaw * dy;
    let column = (local_x / g.resolution).floor();
    let row = (local_y / g.resolution).floor();
    if !(0.0 <= column && column < g.width as f64 && 0.0 <= row && row < g.height as f64) {
        return Err("channel point is outside the navigation map".into());
    }
    Ok((column as i32, row as i32))
}

/// `union_free_space_grids(base, channel)`: 0 where either is 0, else 100.
pub fn union_free_space_grids(base: &[i8], channel: Option<&[i8]>) -> Result<Vec<i8>, String> {
    if base.is_empty() {
        return Err("base_grid must be a non-empty 2-D array".into());
    }
    match channel {
        None => Ok(base.iter().map(|&b| if b == 0 { 0 } else { 100 }).collect()),
        Some(ch) => {
            if ch.len() != base.len() {
                return Err("base and channel grids must have the same shape".into());
            }
            Ok(base.iter().zip(ch).map(|(&b, &c)| if b == 0 || c == 0 { 0 } else { 100 }).collect())
        }
    }
}

/// `fuse_navigation_grid`: block base obstacles / unknowns, every base cell
/// overlapping an occupied risk cell, and base cells outside the risk grid.
pub fn fuse_navigation_grid(
    base: &[i8],
    bg: &Geometry,
    risk: Option<(&[i8], &Geometry)>,
) -> Result<Vec<i8>, String> {
    if base.is_empty() || base.len() != bg.cells() {
        return Err("base_grid must be a non-empty 2-D array".into());
    }
    if !bg.resolution.is_finite() || bg.resolution <= 0.0 {
        return Err("base_resolution must be finite and > 0".into());
    }
    let base_blocked: Vec<bool> = base.iter().map(|&b| b != 0).collect();
    let Some((risk, rg)) = risk else {
        return Ok(base_blocked.iter().map(|&b| if b { 100 } else { 0 }).collect());
    };
    if risk.is_empty() || risk.len() != rg.cells() {
        return Err("risk_grid must be a non-empty 2-D array".into());
    }
    if !rg.resolution.is_finite() || rg.resolution <= 0.0 {
        return Err("risk_resolution must be finite and > 0".into());
    }
    let values = [bg.origin_x, bg.origin_y, bg.origin_yaw, rg.origin_x, rg.origin_y, rg.origin_yaw];
    if !values.iter().all(|v| v.is_finite()) {
        return Err("grid origins and yaws must be finite".into());
    }
    let d = rg.origin_yaw - bg.origin_yaw;
    let yaw_delta = d.sin().atan2(d.cos());
    if yaw_delta.abs() > 1e-9 {
        return Err("base and risk grid orientations must match".into());
    }

    // Express the risk origin in the base grid's local coordinates.
    let delta_x = rg.origin_x - bg.origin_x;
    let delta_y = rg.origin_y - bg.origin_y;
    let (sin_yaw, cos_yaw) = bg.origin_yaw.sin_cos();
    let risk_local_x = cos_yaw * delta_x + sin_yaw * delta_y;
    let risk_local_y = -sin_yaw * delta_x + cos_yaw * delta_y;

    let (bw, bh) = (bg.width as usize, bg.height as usize);
    let (rw, rh) = (rg.width as usize, rg.height as usize);
    let bres = bg.resolution;
    let rres = rg.resolution;
    let base_x0: Vec<f64> = (0..bw).map(|i| i as f64 * bres).collect();
    let base_x1: Vec<f64> = base_x0.iter().map(|x| x + bres).collect();
    let base_y0: Vec<f64> = (0..bh).map(|i| i as f64 * bres).collect();
    let base_y1: Vec<f64> = base_y0.iter().map(|y| y + bres).collect();

    let risk_x1 = risk_local_x + rw as f64 * rres;
    let risk_y1 = risk_local_y + rh as f64 * rres;
    let tolerance = 1e-9
        * [1.0, bres.abs(), rres.abs(), risk_local_x.abs(), risk_local_y.abs(), risk_x1.abs(), risk_y1.abs()]
            .iter()
            .cloned()
            .fold(f64::MIN, f64::max);
    let outside_x: Vec<bool> = (0..bw).map(|c| base_x0[c] < risk_local_x - tolerance || base_x1[c] > risk_x1 + tolerance).collect();
    let outside_y: Vec<bool> = (0..bh).map(|r| base_y0[r] < risk_local_y - tolerance || base_y1[r] > risk_y1 + tolerance).collect();
    let index_tolerance = tolerance / rres;

    let clip = |v: f64, hi: usize| -> usize {
        let v = v as i64; // numpy float -> int64 truncates toward zero, values are already floored/ceiled
        v.max(0).min(hi as i64) as usize
    };
    let col0: Vec<usize> = (0..bw).map(|c| clip(((base_x0[c] - risk_local_x) / rres + index_tolerance).floor(), rw)).collect();
    let col1: Vec<usize> = (0..bw).map(|c| clip(((base_x1[c] - risk_local_x) / rres - index_tolerance).ceil(), rw)).collect();
    let row0: Vec<usize> = (0..bh).map(|r| clip(((base_y0[r] - risk_local_y) / rres + index_tolerance).floor(), rh)).collect();
    let row1: Vec<usize> = (0..bh).map(|r| clip(((base_y1[r] - risk_local_y) / rres - index_tolerance).ceil(), rh)).collect();

    // Integral image of the blocked risk cells, padded with a zero row/column.
    let iw = rw + 1;
    let mut integral = vec![0i64; (rh + 1) * iw];
    for r in 0..rh {
        let mut row_sum = 0i64;
        for c in 0..rw {
            row_sum += (risk[r * rw + c] != 0) as i64;
            integral[(r + 1) * iw + (c + 1)] = integral[r * iw + (c + 1)] + row_sum;
        }
    }
    let at = |r: usize, c: usize| integral[r * iw + c];

    let mut out = vec![0i8; base.len()];
    for r in 0..bh {
        for c in 0..bw {
            let overlap = at(row1[r], col1[c]) - at(row0[r], col1[c]) - at(row1[r], col0[c]) + at(row0[r], col0[c]);
            let blocked = base_blocked[r * bw + c] || outside_y[r] || outside_x[c] || overlap > 0;
            out[r * bw + c] = if blocked { 100 } else { 0 };
        }
    }
    Ok(out)
}

fn img_from_mask(data: &[i8], w: u32, h: u32, pred: impl Fn(i8) -> bool) -> Img {
    Img { w: w as i32, h: h as i32, data: data.iter().map(|&v| pred(v) as u8).collect() }
}

/// `erode_free_space_grid`: 0 only where free space has the clearance; the
/// map exterior counts as obstacle (explicit zero border).
pub fn erode_free_space_grid(data: &[i8], w: u32, h: u32, resolution_m: f64, inflate_radius_m: f64) -> Result<Vec<i8>, String> {
    if data.is_empty() || data.len() != (w as usize) * (h as usize) {
        return Err("free-space grid must be a non-empty 2D array".into());
    }
    if !resolution_m.is_finite() || resolution_m <= 0.0 {
        return Err("resolution_m must be finite and positive".into());
    }
    if !inflate_radius_m.is_finite() || inflate_radius_m < 0.0 {
        return Err("inflate_radius_m must be finite and non-negative".into());
    }
    let radius_cells = (inflate_radius_m / resolution_m).ceil() as i32;
    let mut free = img_from_mask(data, w, h, |v| v == 0);
    if radius_cells > 0 {
        free = raster::erode(&free, &raster::ellipse_kernel(radius_cells), Border::Constant(0));
    }
    Ok(free.data.iter().map(|&v| if v == 1 { 0 } else { 100 }).collect())
}

/// `int(math.ceil(inflate_r_m / resolution))` clamped at 0.
pub fn radius_cells(inflate_r_m: f64, resolution: f64) -> i32 {
    ((inflate_r_m / resolution).ceil() as i32).max(0)
}

/// `_create_risk_map_inflated` data: dilate the occupied (100) cells with the
/// ellipse kernel; `None` when the radius rounds to no cells (the caller then
/// reuses the raw risk map).
pub fn dilate_risk(data: &[i8], w: u32, h: u32, r_cells: i32) -> Option<Vec<i8>> {
    if r_cells <= 0 {
        return None;
    }
    let binary = img_from_mask(data, w, h, |v| v == 100);
    let inflated = raster::dilate(&binary, &raster::ellipse_kernel(r_cells));
    Some(inflated.data.iter().map(|&v| if v == 1 { 100 } else { 0 }).collect())
}

/// `_create_chennal_map_inflated` data: erode the free (0) cells with the
/// default (exterior-ignoring) border; `None` when the radius rounds to none.
pub fn erode_channel(data: &[i8], w: u32, h: u32, r_cells: i32) -> Option<Vec<i8>> {
    if r_cells <= 0 {
        return None;
    }
    let binary = img_from_mask(data, w, h, |v| v == 0);
    let eroded = raster::erode(&binary, &raster::ellipse_kernel(r_cells), Border::Ignore);
    Some(eroded.data.iter().map(|&v| if v == 1 { 0 } else { 100 }).collect())
}

/// `decode_u8_mask`: strict base64 of a row-major `width*height` byte mask.
pub fn decode_u8_mask(encoded: &str, width: u32, height: u32, field_name: &str, optional: bool) -> Result<Vec<u8>, String> {
    let expected = width as usize * height as usize;
    if encoded.is_empty() {
        if optional {
            return Ok(vec![0; expected]);
        }
        return Err(format!("{field_name} is required"));
    }
    let raw = base64::engine::general_purpose::STANDARD
        .decode(encoded)
        .map_err(|_| format!("{field_name} must be valid base64"))?;
    if raw.len() != expected {
        return Err(format!("{field_name} length {} does not match width*height {expected}", raw.len()));
    }
    Ok(raw)
}

pub struct ImagePlacement {
    pub resolution: f64,
    pub robot_x: f64,
    pub robot_y: f64,
    pub robot_yaw: f64,
    pub start_x: f64,
    pub start_y: f64,
    pub image_heading: f64,
}

pub struct RasterizedMasks {
    pub free_grid: Vec<i8>,
    pub risk_grid: Vec<i8>,
    pub width: u32,
    pub height: u32,
    pub min_x: f64,
    pub min_y: f64,
}

/// `rasterize_image_masks`: image-local masks -> axis-aligned map-frame grids.
pub fn rasterize_image_masks(free_mask: &[u8], risk_mask: &[u8], width: u32, height: u32, p: &ImagePlacement) -> RasterizedMasks {
    let (width, height) = (width as usize, height as usize);
    let theta = p.robot_yaw - p.image_heading;
    let (sin_t, cos_t) = theta.sin_cos();
    let local_to_map = |x: f64, y: f64| -> (f64, f64) {
        let dx = x - p.start_x;
        let dy = y - p.start_y;
        (p.robot_x + cos_t * dx - sin_t * dy, p.robot_y + sin_t * dx + cos_t * dy)
    };
    let world_w = width as f64 * p.resolution;
    let world_h = height as f64 * p.resolution;
    let corners = [local_to_map(0.0, 0.0), local_to_map(world_w, 0.0), local_to_map(0.0, world_h), local_to_map(world_w, world_h)];
    let min_x = corners.iter().map(|c| c.0).fold(f64::INFINITY, f64::min);
    let max_x = corners.iter().map(|c| c.0).fold(f64::NEG_INFINITY, f64::max);
    let min_y = corners.iter().map(|c| c.1).fold(f64::INFINITY, f64::min);
    let max_y = corners.iter().map(|c| c.1).fold(f64::NEG_INFINITY, f64::max);
    let out_w = (((max_x - min_x) / p.resolution).ceil() as i64).max(1) as usize;
    let out_h = (((max_y - min_y) / p.resolution).ceil() as i64).max(1) as usize;

    let mut free_grid = vec![100i8; out_w * out_h];
    let mut risk_grid = vec![0i8; out_w * out_h];
    for out_r in 0..out_h {
        let map_y = min_y + (out_r as f64 + 0.5) * p.resolution;
        for out_c in 0..out_w {
            let map_x = min_x + (out_c as f64 + 0.5) * p.resolution;
            let dx = map_x - p.robot_x;
            let dy = map_y - p.robot_y;
            let local_x = p.start_x + cos_t * dx + sin_t * dy;
            let local_y = p.start_y - sin_t * dx + cos_t * dy;
            let src_c = (local_x / p.resolution).floor();
            let src_r = ((world_h - local_y) / p.resolution).floor();
            if src_c < 0.0 || src_c >= width as f64 || src_r < 0.0 || src_r >= height as f64 {
                continue;
            }
            let idx = src_r as usize * width + src_c as usize;
            if free_mask[idx] == 255 {
                free_grid[out_r * out_w + out_c] = 0;
            }
            if risk_mask[idx] == 255 {
                risk_grid[out_r * out_w + out_c] = 100;
            }
        }
    }
    RasterizedMasks { free_grid, risk_grid, width: out_w as u32, height: out_h as u32, min_x, min_y }
}

/// `_clip_free_grid_to_collected`: keep an image cell free only where the
/// nearest collected-free-space cell is free too.
pub fn clip_free_grid_to_collected(
    free_grid: &[i8],
    img_w: u32,
    img_h: u32,
    origin_x: f64,
    origin_y: f64,
    resolution: f64,
    collected: &[i8],
    cg: &Geometry,
) -> Vec<i8> {
    let (iw, ih) = (img_w as usize, img_h as usize);
    let (cw, ch) = (cg.width as i64, cg.height as i64);
    let mut out = vec![100i8; iw * ih];
    for r in 0..ih {
        let world_y = origin_y + (r as f64 + 0.5) * resolution;
        let row = ((world_y - cg.origin_y) / cg.resolution).floor() as i64;
        for c in 0..iw {
            let world_x = origin_x + (c as f64 + 0.5) * resolution;
            let col = ((world_x - cg.origin_x) / cg.resolution).floor() as i64;
            let inside = row >= 0 && row < ch && col >= 0 && col < cw;
            let collected_free = inside && collected[(row * cw + col) as usize] == 0;
            if free_grid[r * iw + c] == 0 && collected_free {
                out[r * iw + c] = 0;
            }
        }
    }
    out
}

/// `_image_safe_grid`: the safe area of an image zone inside collected free
/// space. The image is eroded by `outline_inset_m` from its own outline and
/// kept only where `collected_safe` (the collected free space already eroded
/// by `inflate_radius_m`) is free: the drawn outline lies on grass and only
/// needs the blade's reach, the real edges keep their full clearance.
#[allow(clippy::too_many_arguments)]
pub fn image_safe_grid(
    image_free: &[i8],
    img_w: u32,
    img_h: u32,
    origin_x: f64,
    origin_y: f64,
    resolution: f64,
    outline_inset_m: f64,
    collected_safe: &[i8],
    cg: &Geometry,
) -> Result<Vec<i8>, String> {
    let inset = erode_free_space_grid(image_free, img_w, img_h, resolution, outline_inset_m)?;
    Ok(clip_free_grid_to_collected(&inset, img_w, img_h, origin_x, origin_y, resolution, collected_safe, cg))
}

/// `_generate_risk_map` data: every polygon filled (>= 3 vertices) and
/// outlined (closed when its ends are within 1 m), clamped into the base grid.
pub fn risk_map_data(g: &Geometry, zones: &[Vec<(f64, f64)>]) -> Vec<i8> {
    let (w, h) = (g.width as i32, g.height as i32);
    let mut risk = vec![0i8; g.cells()];
    for zone in zones {
        let poly: Vec<(i32, i32)> = zone
            .iter()
            .map(|&(px, py)| {
                let x = ((px - g.origin_x) / g.resolution) as i64;
                let y = ((py - g.origin_y) / g.resolution) as i64;
                (x.max(0).min((w - 1) as i64) as i32, y.max(0).min((h - 1) as i64) as i32)
            })
            .collect();
        if poly.len() < 2 {
            continue;
        }
        let mut temp = Img::zeros(w, h);
        if poly.len() >= 3 {
            raster::fill_poly(&mut temp, &poly, 1);
        }
        let mut closed = false;
        if poly.len() >= 3 {
            let (first, last) = (poly[0], poly[poly.len() - 1]);
            let close_threshold_cells = ((1.0 / g.resolution).ceil() as i64).max(1);
            let dx = (first.0 - last.0) as f64;
            let dy = (first.1 - last.1) as f64;
            closed = (dx * dx + dy * dy).sqrt() <= close_threshold_cells as f64;
        }
        raster::polylines(&mut temp, &poly, closed, 1, 1);
        for (r, &t) in risk.iter_mut().zip(&temp.data) {
            if t == 1 {
                *r = 100;
            }
        }
    }
    risk
}

pub struct ZoneRaster {
    pub geometry: Geometry,
    /// Union free-space data (0 inside any polygon, 100 elsewhere).
    pub free_space: Vec<i8>,
    /// Per accepted zone: (marker id, its own mask data).
    pub zones: Vec<(i32, Vec<i8>)>,
    /// Marker ids skipped for having fewer than three vertices, with the count.
    pub skipped: Vec<(i32, usize)>,
}

/// The geometry + raster part of `_create_zone_maps_and_freespace`.
/// `None` mirrors the Python early returns (no markers / no points / no
/// polygon with three vertices); the caller logs the matching warning.
pub fn zone_maps_and_freespace(zones: &[(i32, Vec<(f64, f64)>)]) -> Result<ZoneRaster, &'static str> {
    if zones.is_empty() {
        return Err("沒有區域數據");
    }
    let all: Vec<(f64, f64)> = zones.iter().flat_map(|(_, pts)| pts.iter().cloned()).collect();
    if all.is_empty() {
        return Err("沒有有效的點數據");
    }
    let min_x = all.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
    let max_x = all.iter().map(|p| p.0).fold(f64::NEG_INFINITY, f64::max);
    let min_y = all.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
    let max_y = all.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);
    let resolution = 0.05;
    let margin = 1.0;
    let map_width = max_x - min_x + 2.0 * margin;
    let map_height = max_y - min_y + 2.0 * margin;
    let w = (map_width / resolution) as i64;
    let h = (map_height / resolution) as i64;
    let ox = min_x - margin;
    let oy = min_y - margin;
    let geometry = Geometry { width: w.max(0) as u32, height: h.max(0) as u32, resolution, origin_x: ox, origin_y: oy, origin_yaw: 0.0 };
    let mut mask = Img::zeros(w as i32, h as i32);
    let mut out_zones = Vec::new();
    let mut skipped = Vec::new();
    for (id, pts) in zones {
        let poly: Vec<(i32, i32)> = pts.iter().map(|&(px, py)| (((px - ox) / resolution) as i64 as i32, ((py - oy) / resolution) as i64 as i32)).collect();
        if poly.len() < 3 {
            skipped.push((*id, poly.len()));
            continue;
        }
        let mut zone_mask = Img::zeros(w as i32, h as i32);
        raster::fill_poly(&mut zone_mask, &poly, 1);
        raster::fill_poly(&mut mask, &poly, 1);
        out_zones.push((*id, zone_mask.data.iter().map(|&v| if v == 1 { 0 } else { 100 }).collect()));
    }
    if out_zones.is_empty() {
        return Err("沒有可建立自由空間的有效多邊形");
    }
    let free_space = mask.data.iter().map(|&v| if v == 1 { 0 } else { 100 }).collect();
    Ok(ZoneRaster { geometry, free_space, zones: out_zones, skipped })
}

/// The raster part of `_generate_chennal_map`: each path segment drawn as a
/// thick line of `chennal_width` into an all-occupied grid. Errors carry the
/// Python messages (frame mismatch, off-map point).
pub fn chennal_map_data(g: &Geometry, base_frame: &str, paths: &[(String, Vec<(f64, f64)>)], chennal_width: f64) -> Result<Vec<i8>, String> {
    let (w, h) = (g.width as i32, g.height as i32);
    let mut data = vec![100u8; g.cells()];
    for (frame, pts) in paths {
        let marker_frame = if frame.is_empty() { base_frame } else { frame.as_str() };
        if marker_frame != base_frame {
            return Err(format!("通道路徑 frame 與基礎地圖不一致: {marker_frame} != {base_frame}"));
        }
        let mut path_points = Vec::with_capacity(pts.len());
        for &(x, y) in pts {
            path_points.push(world_to_grid_index(x, y, g)?);
        }
        let width_px = ((chennal_width / g.resolution).ceil() as i64).max(1) as i32;
        for seg in path_points.windows(2) {
            let mut temp = Img::zeros(w, h);
            raster::line(&mut temp, seg[0], seg[1], 1, width_px);
            for (d, &t) in data.iter_mut().zip(&temp.data) {
                if t == 1 {
                    *d = 0;
                }
            }
        }
    }
    Ok(data.iter().map(|&v| v as i8).collect())
}

/// `math.isclose(a, b, rel_tol=0.0, abs_tol=1e-9)`.
pub fn isclose_abs(a: f64, b: f64) -> bool {
    a == b || (a - b).abs() <= 1e-9
}

/// Wrapped yaw difference as `_navigation_base_with_channel` computes it.
pub fn yaw_delta(a: f64, b: f64) -> f64 {
    let d = a - b;
    d.sin().atan2(d.cos())
}

#[allow(dead_code)]
pub const TWO_PI: f64 = 2.0 * PI;

#[cfg(test)]
mod tests {
    use super::*;

    fn geom(w: u32, h: u32, res: f64, ox: f64, oy: f64, yaw: f64) -> Geometry {
        Geometry { width: w, height: h, resolution: res, origin_x: ox, origin_y: oy, origin_yaw: yaw }
    }

    #[test]
    fn world_to_grid_handles_rotated_origins_and_rejects_outside_points() {
        assert_eq!(world_to_grid_index(-0.5, 1.5, &geom(3, 3, 1.0, 1.0, 1.0, PI / 2.0)).unwrap(), (0, 1));
        let e = world_to_grid_index(-0.01, 0.0, &geom(3, 3, 1.0, 0.0, 0.0, 0.0)).unwrap_err();
        assert!(e.contains("outside"));
        for v in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(world_to_grid_index(v, 0.0, &geom(3, 3, 1.0, 0.0, 0.0, 0.0)).unwrap_err().contains("finite"));
        }
    }

    #[test]
    fn channel_free_space_is_unioned_into_the_navigation_base() {
        let base = [0, 100, -1, 100, 100, 0];
        let channel = [100, 0, 100, 100, 0, 100];
        assert_eq!(union_free_space_grids(&base, Some(&channel)).unwrap(), vec![0, 0, 100, 100, 0, 0]);
        assert!(union_free_space_grids(&[0, 0, 0, 0], Some(&[0, 0])).unwrap_err().contains("same shape"));
    }

    #[test]
    fn fusion_blocks_base_obstacles_unknowns_and_risk() {
        let base = [100, 0, 0, 0, -1, 0];
        let risk = [0, 100, 0, 0, 0, 0];
        let g = geom(3, 2, 0.5, -1.0, 2.0, 0.0);
        assert_eq!(fuse_navigation_grid(&base, &g, Some((&risk, &g))).unwrap(), vec![100, 100, 0, 0, 100, 0]);
    }

    #[test]
    fn mismatched_extent_is_resampled_conservatively() {
        let base = [0i8; 8];
        let risk = [100, 0, 0, 0];
        let bg = geom(4, 2, 1.0, 0.0, 0.0, 0.0);
        let rg = geom(2, 2, 1.0, 1.0, 0.0, 0.0);
        assert_eq!(fuse_navigation_grid(&base, &bg, Some((&risk, &rg))).unwrap(), vec![100, 100, 0, 100, 100, 0, 0, 100]);
    }

    #[test]
    fn coarser_base_cell_blocks_on_any_risk_overlap() {
        let risk = [0, 0, 0, 100];
        let bg = geom(1, 1, 1.0, 0.0, 0.0, 0.0);
        let rg = geom(2, 2, 0.5, 0.0, 0.0, 0.0);
        assert_eq!(fuse_navigation_grid(&[0], &bg, Some((&risk, &rg))).unwrap(), vec![100]);
    }

    #[test]
    fn orientation_mismatch_is_rejected_instead_of_dropping_risk() {
        let bg = geom(1, 1, 1.0, 0.0, 0.0, 0.0);
        let rg = geom(1, 1, 1.0, 0.0, 0.0, 0.1);
        assert!(fuse_navigation_grid(&[0], &bg, Some((&[0], &rg))).unwrap_err().contains("orientations must match"));
    }

    #[test]
    fn free_space_erosion_treats_map_exterior_as_occupied() {
        let out = erode_free_space_grid(&vec![0i8; 41 * 41], 41, 41, 0.1, 0.75).unwrap();
        for r in 0..41 {
            for c in 0..41 {
                let v = out[r * 41 + c];
                if r < 8 || r >= 33 || c < 8 || c >= 33 {
                    assert_eq!(v, 100, "({r},{c})");
                }
            }
        }
        assert_eq!(out[20 * 41 + 20], 0);
        let narrow = erode_free_space_grid(&vec![0i8; 100], 10, 10, 0.1, 0.75).unwrap();
        assert!(narrow.iter().all(|&v| v != 0));
    }

    #[test]
    fn image_zone_keeps_the_blade_inset_from_its_outline_and_the_full_margin_from_real_edges() {
        // collected free space: world [0, 4]^2 at 0.05 m, and its 0.75 m twin
        let cg = geom(100, 100, 0.05, -0.5, -0.5, 0.0);
        let collected: Vec<i8> = (0..100 * 100)
            .map(|i| {
                let (x, y) = (-0.5 + ((i % 100) as f64 + 0.5) * 0.05, -0.5 + ((i / 100) as f64 + 0.5) * 0.05);
                if (0.0..4.0).contains(&x) && (0.0..4.0).contains(&y) { 0 } else { 100 }
            })
            .collect();
        let collected_safe = erode_free_space_grid(&collected, 100, 100, 0.05, 0.75).unwrap();
        // an all-free 8 m image at 0.1 m from (2, 2): its outline at x = 2 and
        // y = 2 lies on collected grass, its far side beyond the collected edge
        let image = vec![0i8; 80 * 80];
        let extent = |grid: &[i8]| {
            let cols: Vec<usize> = (0..80 * 80).filter(|&i| grid[i] == 0).map(|i| i % 80).collect();
            let rows: Vec<usize> = (0..80 * 80).filter(|&i| grid[i] == 0).map(|i| i / 80).collect();
            let edge = |v: &[usize]| (2.0 + *v.iter().min().unwrap() as f64 * 0.1, 2.0 + (*v.iter().max().unwrap() + 1) as f64 * 0.1);
            (edge(&cols), edge(&rows))
        };

        let safe = image_safe_grid(&image, 80, 80, 2.0, 2.0, 0.1, 0.3, &collected_safe, &cg).unwrap();
        let ((x0, x1), (y0, y1)) = extent(&safe);
        for (lo, hi) in [(x0, x1), (y0, y1)] {
            assert!((lo - 2.3).abs() < 0.051, "0.3 m from the drawn outline, got {lo}");
            assert!((hi - 3.25).abs() < 0.11, "0.75 m from the collected edge, got {hi}");
        }
        // before: the clipped image eroded by 0.75 m everywhere
        let clipped = clip_free_grid_to_collected(&image, 80, 80, 2.0, 2.0, 0.1, &collected, &cg);
        let old = erode_free_space_grid(&clipped, 80, 80, 0.1, 0.75).unwrap();
        let ((old_x0, _), _) = extent(&old);
        assert!((old_x0 - 2.8).abs() < 0.051, "{old_x0}");
    }

    #[test]
    fn decode_u8_mask_roundtrips_and_rejects_wrong_size() {
        let enc = base64::engine::general_purpose::STANDARD.encode([255u8, 0, 0, 255]);
        assert_eq!(decode_u8_mask(&enc, 2, 2, "free_mask_data", false).unwrap(), vec![255, 0, 0, 255]);
        let enc = base64::engine::general_purpose::STANDARD.encode([255u8, 0]);
        assert!(decode_u8_mask(&enc, 2, 2, "free_mask_data", false).unwrap_err().contains("width*height"));
        assert_eq!(decode_u8_mask("", 2, 2, "risk_mask_data", true).unwrap(), vec![0; 4]);
        assert_eq!(decode_u8_mask("", 2, 2, "free_mask_data", false).unwrap_err(), "free_mask_data is required");
        assert!(decode_u8_mask("!!", 1, 1, "free_mask_data", false).unwrap_err().contains("valid base64"));
    }

    #[test]
    fn rasterize_image_masks_flips_image_y_to_map_y() {
        let free = [255, 0, 0, 255];
        let risk = [0, 255, 0, 0];
        let p = ImagePlacement { resolution: 0.5, robot_x: 0.0, robot_y: 0.0, robot_yaw: 0.0, start_x: 0.0, start_y: 0.0, image_heading: 0.0 };
        let r = rasterize_image_masks(&free, &risk, 2, 2, &p);
        assert!((r.min_x).abs() < 1e-9 && (r.min_y).abs() < 1e-9);
        assert_eq!((r.width, r.height), (2, 2));
        assert_eq!(r.free_grid, vec![100, 0, 0, 100]);
        assert_eq!(r.risk_grid, vec![0, 0, 0, 100]);
    }

    #[test]
    fn yaw_from_quaternion_reads_map_heading() {
        let s = 2f64.sqrt() / 2.0;
        assert!((yaw_from_quaternion(0.0, 0.0, s, s) - PI / 2.0).abs() < 1e-9);
    }
}

#[cfg(test)]
mod oracle_tests {
    use super::*;
    use serde_json::Value;

    fn i8s(v: &Value) -> Vec<i8> {
        v.as_array().unwrap().iter().map(|x| x.as_i64().unwrap() as i8).collect()
    }

    fn report(name: &str, w: usize, got: &[i8], want: &Value) -> Option<String> {
        let bytes: Vec<u8> = got.iter().map(|&v| v as u8).collect();
        if crate::raster::tests::same(&bytes, want) {
            return None;
        }
        if let Value::Array(_) = want {
            let want = i8s(want);
            let diffs: Vec<(usize, usize, i8, i8)> = got.iter().zip(&want).enumerate().filter(|(_, (a, b))| a != b).map(|(i, (a, b))| (i % w, i / w, *a, *b)).collect();
            return Some(format!("{name}: {} pixels differ, first {:?}", diffs.len(), &diffs[..diffs.len().min(12)]));
        }
        Some(format!("{name}: checksum differs"))
    }

    /// `tests/map_oracle.json` (map_oracle.py): the node's own risk, channel
    /// and fusion code on the geometries of the differential run.
    #[test]
    fn matches_node_oracle() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/map_oracle.json");
        let Ok(text) = std::fs::read_to_string(path) else {
            eprintln!("no {path}; skipped");
            return;
        };
        let cases: Vec<Value> = serde_json::from_str(&text).unwrap();
        let mut failures = Vec::new();
        for case in &cases {
            let name = case["name"].as_str().unwrap();
            match case["op"].as_str().unwrap() {
                "risk" => {
                    let g = Geometry { width: case["w"].as_u64().unwrap() as u32, height: case["h"].as_u64().unwrap() as u32, resolution: case["res"].as_f64().unwrap(), origin_x: case["ox"].as_f64().unwrap(), origin_y: case["oy"].as_f64().unwrap(), origin_yaw: 0.0 };
                    let zones: Vec<Vec<(f64, f64)>> = case["zones"].as_array().unwrap().iter().map(|z| z.as_array().unwrap().iter().map(|p| (p[0].as_f64().unwrap(), p[1].as_f64().unwrap())).collect()).collect();
                    let got = risk_map_data(&g, &zones);
                    failures.extend(report(name, g.width as usize, &got, &case["out"]));
                }
                "channel" => {
                    let g = Geometry { width: case["w"].as_u64().unwrap() as u32, height: case["h"].as_u64().unwrap() as u32, resolution: case["res"].as_f64().unwrap(), origin_x: case["ox"].as_f64().unwrap(), origin_y: case["oy"].as_f64().unwrap(), origin_yaw: 0.0 };
                    let paths: Vec<(String, Vec<(f64, f64)>)> = case["paths"].as_array().unwrap().iter().map(|z| (String::new(), z.as_array().unwrap().iter().map(|p| (p[0].as_f64().unwrap(), p[1].as_f64().unwrap())).collect())).collect();
                    for (pts, want) in paths.iter().zip(case["grid_points"].as_array().unwrap()) {
                        let got: Vec<Vec<i64>> = pts.1.iter().map(|&(x, y)| { let (c, r) = world_to_grid_index(x, y, &g).unwrap(); vec![c as i64, r as i64] }).collect();
                        let want: Vec<Vec<i64>> = want.as_array().unwrap().iter().map(|p| p.as_array().unwrap().iter().map(|v| v.as_i64().unwrap()).collect()).collect();
                        if got != want { failures.push(format!("{name}: grid points {got:?} vs {want:?}")); }
                    }
                    let got = chennal_map_data(&g, "map", &paths, case["width"].as_f64().unwrap()).unwrap();
                    failures.extend(report(name, g.width as usize, &got, &case["out"]));
                }
                "fuse" => {
                    let bg = Geometry { width: case["bw"].as_u64().unwrap() as u32, height: case["bh"].as_u64().unwrap() as u32, resolution: case["bres"].as_f64().unwrap(), origin_x: case["box"].as_f64().unwrap(), origin_y: case["boy"].as_f64().unwrap(), origin_yaw: 0.0 };
                    let rg = Geometry { width: case["rw"].as_u64().unwrap() as u32, height: case["rh"].as_u64().unwrap() as u32, resolution: case["rres"].as_f64().unwrap(), origin_x: case["rox"].as_f64().unwrap(), origin_y: case["roy"].as_f64().unwrap(), origin_yaw: 0.0 };
                    let got = fuse_navigation_grid(&i8s(&case["base"]), &bg, Some((&i8s(&case["risk"]), &rg))).unwrap();
                    failures.extend(report(name, bg.width as usize, &got, &case["out"]));
                }
                other => panic!("{other}"),
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }
}
