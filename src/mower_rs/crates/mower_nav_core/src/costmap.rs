//! `nav2_costmap_2d` the way `nav2_no_map_params.yaml` configures it: two
//! costmaps whose only layer is a `StaticLayer`.
//!
//! * global (`/map_grid_global`, frame `map`, `track_unknown_space: true`,
//!   not rolling): the master grid takes the map's size, resolution and origin
//!   and every cell is the interpreted map value -> [`static_costmap`];
//! * local (`/map_grid`, frame `odom`, `track_unknown_space` left at false,
//!   3 x 3 m rolling window at 0.05 m): every update re-centres the window on
//!   the robot (grid-aligned, [`Costmap2D::update_origin`]) and resamples the
//!   whole window from the map through the `map <- odom` transform ->
//!   [`RollingWindow`].
//!
//! Upstream: `src/costmap_2d.cpp`, `src/layered_costmap.cpp`,
//! `plugins/static_layer.cpp`, `include/nav2_costmap_2d/cost_values.hpp`.

use crate::geometry::Transform2D;

pub const NO_INFORMATION: u8 = 255;
pub const LETHAL_OBSTACLE: u8 = 254;
pub const INSCRIBED_INFLATED_OBSTACLE: u8 = 253;
pub const MAX_NON_OBSTACLE: u8 = 252;
pub const FREE_SPACE: u8 = 0;

/// `nav_msgs/OccupancyGrid` as `StaticLayer::processMap` reads it: the
/// origin's position only (its orientation is ignored upstream too), and the
/// row-major `int8` data.
#[derive(Debug, Clone, PartialEq)]
pub struct OccupancyGrid {
    pub width: u32,
    pub height: u32,
    pub resolution: f64,
    pub origin_x: f64,
    pub origin_y: f64,
    pub data: Vec<i8>,
}

/// `nav2_costmap_2d::Costmap2D`: the cell storage and its coordinate maths.
#[derive(Debug, Clone, PartialEq)]
pub struct Costmap2D {
    size_x: u32,
    size_y: u32,
    resolution: f64,
    origin_x: f64,
    origin_y: f64,
    default_value: u8,
    data: Vec<u8>,
}

impl Costmap2D {
    /// The constructor: every cell starts at `default_value`.
    pub fn new(size_x: u32, size_y: u32, resolution: f64, origin_x: f64, origin_y: f64, default_value: u8) -> Self {
        Self {
            size_x,
            size_y,
            resolution,
            origin_x,
            origin_y,
            default_value,
            data: vec![default_value; size_x as usize * size_y as usize],
        }
    }

    pub fn size_x(&self) -> u32 {
        self.size_x
    }
    pub fn size_y(&self) -> u32 {
        self.size_y
    }
    pub fn resolution(&self) -> f64 {
        self.resolution
    }
    pub fn origin_x(&self) -> f64 {
        self.origin_x
    }
    pub fn origin_y(&self) -> f64 {
        self.origin_y
    }
    pub fn default_value(&self) -> u8 {
        self.default_value
    }
    /// Row-major cells, `index = my * size_x + mx` (`getCharMap`).
    pub fn data(&self) -> &[u8] {
        &self.data
    }

    /// `getSizeInMetersX`: `(size_x - 1 + 0.5) * resolution`, upstream's
    /// half-cell-short definition (it is what the rolling window centres on).
    pub fn size_in_meters_x(&self) -> f64 {
        (self.size_x as f64 - 1.0 + 0.5) * self.resolution
    }
    pub fn size_in_meters_y(&self) -> f64 {
        (self.size_y as f64 - 1.0 + 0.5) * self.resolution
    }

    pub fn index(&self, mx: u32, my: u32) -> usize {
        my as usize * self.size_x as usize + mx as usize
    }

    /// `getCost(mx, my)`. Upstream does not bounds-check; this panics instead.
    pub fn cost(&self, mx: u32, my: u32) -> u8 {
        self.data[self.index(mx, my)]
    }

    pub fn set_cost(&mut self, mx: u32, my: u32, cost: u8) {
        let i = self.index(mx, my);
        self.data[i] = cost;
    }

    /// `mapToWorld`: the cell centre.
    pub fn map_to_world(&self, mx: u32, my: u32) -> (f64, f64) {
        (
            self.origin_x + (mx as f64 + 0.5) * self.resolution,
            self.origin_y + (my as f64 + 0.5) * self.resolution,
        )
    }

    /// `worldToMap`.
    pub fn world_to_map(&self, wx: f64, wy: f64) -> Option<(u32, u32)> {
        if wx < self.origin_x || wy < self.origin_y {
            return None;
        }
        // static_cast<unsigned int>: truncation toward zero, both are >= 0 here.
        let mx = ((wx - self.origin_x) / self.resolution) as u32;
        let my = ((wy - self.origin_y) / self.resolution) as u32;
        if mx < self.size_x && my < self.size_y {
            Some((mx, my))
        } else {
            None
        }
    }

    /// `worldToMapEnforceBounds`.
    pub fn world_to_map_enforce_bounds(&self, wx: f64, wy: f64) -> (i32, i32) {
        let mx = if wx < self.origin_x {
            0
        } else if wx > self.resolution * self.size_x as f64 + self.origin_x {
            self.size_x as i32 - 1
        } else {
            ((wx - self.origin_x) / self.resolution) as i32
        };
        let my = if wy < self.origin_y {
            0
        } else if wy > self.resolution * self.size_y as f64 + self.origin_y {
            self.size_y as i32 - 1
        } else {
            ((wy - self.origin_y) / self.resolution) as i32
        };
        (mx, my)
    }

    /// `resetMaps`: every cell back to the default value.
    pub fn reset_maps(&mut self) {
        self.data.fill(self.default_value);
    }

    /// `resetMap(x0, y0, xn, yn)`: the half-open window back to the default.
    pub fn reset_map(&mut self, x0: u32, y0: u32, xn: u32, yn: u32) {
        let v = self.default_value;
        for y in y0..yn {
            let row = y as usize * self.size_x as usize;
            self.data[row + x0 as usize..row + xn as usize].fill(v);
        }
    }

    /// `updateOrigin`: move the window by whole cells (so it stays
    /// grid-aligned), keep the overlapping cells, reset the rest.
    pub fn update_origin(&mut self, new_origin_x: f64, new_origin_y: f64) {
        // static_cast<int>: truncation toward zero, also for negative moves.
        let cell_ox = ((new_origin_x - self.origin_x) / self.resolution) as i32;
        let cell_oy = ((new_origin_y - self.origin_y) / self.resolution) as i32;
        let new_grid_ox = self.origin_x + cell_ox as f64 * self.resolution;
        let new_grid_oy = self.origin_y + cell_oy as f64 * self.resolution;

        let size_x = self.size_x as i32;
        let size_y = self.size_y as i32;
        let lower_left_x = cell_ox.max(0).min(size_x);
        let lower_left_y = cell_oy.max(0).min(size_y);
        let upper_right_x = (cell_ox + size_x).max(0).min(size_x);
        let upper_right_y = (cell_oy + size_y).max(0).min(size_y);
        let cell_size_x = (upper_right_x - lower_left_x) as usize;
        let cell_size_y = (upper_right_y - lower_left_y) as usize;

        // copyMapRegion copies nothing when the windows do not overlap.
        let overlap = cell_size_x > 0 && cell_size_y > 0;
        let mut local = vec![0u8; cell_size_x * cell_size_y];
        if overlap {
            for y in 0..cell_size_y {
                let src = (lower_left_y as usize + y) * self.size_x as usize + lower_left_x as usize;
                local[y * cell_size_x..(y + 1) * cell_size_x].copy_from_slice(&self.data[src..src + cell_size_x]);
            }
        }

        self.reset_maps();
        self.origin_x = new_grid_ox;
        self.origin_y = new_grid_oy;

        if !overlap {
            return;
        }
        let start_x = (lower_left_x - cell_ox) as usize;
        let start_y = (lower_left_y - cell_oy) as usize;
        for y in 0..cell_size_y {
            let dst = (start_y + y) * self.size_x as usize + start_x;
            self.data[dst..dst + cell_size_x].copy_from_slice(&local[y * cell_size_x..(y + 1) * cell_size_x]);
        }
    }
}

/// The `StaticLayer` parameters that shape the cost of a map cell. The
/// defaults are `Costmap2DROS`'s: `lethal_cost_threshold` 100,
/// `unknown_cost_value` 255 (the `int8` -1 read as `unsigned char`),
/// `trinary_costmap` true.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StaticLayerParams {
    pub track_unknown_space: bool,
    pub lethal_threshold: u8,
    pub unknown_cost_value: u8,
    pub trinary_costmap: bool,
}

impl StaticLayerParams {
    pub fn new(track_unknown_space: bool) -> Self {
        Self { track_unknown_space, lethal_threshold: 100, unknown_cost_value: 255, trinary_costmap: true }
    }
}

/// `StaticLayer::interpretValue` on the map value as `unsigned char`.
pub fn interpret_value(p: &StaticLayerParams, value: u8) -> u8 {
    if p.track_unknown_space && value == p.unknown_cost_value {
        NO_INFORMATION
    } else if !p.track_unknown_space && value == p.unknown_cost_value {
        FREE_SPACE
    } else if value >= p.lethal_threshold {
        LETHAL_OBSTACLE
    } else if p.trinary_costmap {
        FREE_SPACE
    } else {
        let scale = value as f64 / p.lethal_threshold as f64;
        // double -> unsigned char on return: truncation.
        (scale * LETHAL_OBSTACLE as f64) as u8
    }
}

/// The static layer's own grid (`processMap`): the map's geometry, every cell
/// interpreted. For the non-rolling global costmap this is also the master
/// grid after an update (`updateWithTrueOverwrite` over the whole map).
pub fn static_costmap(grid: &OccupancyGrid, params: &StaticLayerParams) -> Costmap2D {
    let default = if params.track_unknown_space { NO_INFORMATION } else { FREE_SPACE };
    let mut map = Costmap2D::new(grid.width, grid.height, grid.resolution, grid.origin_x, grid.origin_y, default);
    for (cell, &v) in map.data.iter_mut().zip(grid.data.iter()) {
        *cell = interpret_value(params, v as u8);
    }
    map
}

/// The local costmap: a `rolling_window: true` `LayeredCostmap` whose single
/// plugin is a `StaticLayer` on `/map_grid`.
#[derive(Debug, Clone, PartialEq)]
pub struct RollingWindow {
    costmap: Costmap2D,
}

impl RollingWindow {
    /// `Costmap2DROS::on_configure`: `width` / `height` are integer metres,
    /// the grid is `(unsigned int)(width / resolution)` cells, the origin
    /// starts at (`origin_x`, `origin_y`) (default 0, 0) and the default cost
    /// is FREE_SPACE because the local costmap does not track unknown space.
    pub fn new(width_m: i32, height_m: i32, resolution: f64, origin_x: f64, origin_y: f64) -> Self {
        let size_x = (width_m as f64 / resolution) as u32;
        let size_y = (height_m as f64 / resolution) as u32;
        Self { costmap: Costmap2D::new(size_x, size_y, resolution, origin_x, origin_y, FREE_SPACE) }
    }

    /// This robot's `local_costmap`: 3 x 3 m at 0.05 m.
    pub fn local_costmap() -> Self {
        Self::new(3, 3, 0.05, 0.0, 0.0)
    }

    pub fn costmap(&self) -> &Costmap2D {
        &self.costmap
    }

    /// One `LayeredCostmap::updateMap(robot_x, robot_y, _)` cycle with the
    /// robot pose in the window's frame (odom). `map` is the static layer's
    /// grid ([`static_costmap`] with `track_unknown_space: false`) and
    /// `map_from_window` is `lookupTransform(map_frame, global_frame)`.
    pub fn update(&mut self, robot_x: f64, robot_y: f64, map: &Costmap2D, map_from_window: &Transform2D) {
        let c = &mut self.costmap;
        let new_origin_x = robot_x - c.size_in_meters_x() / 2.0;
        let new_origin_y = robot_y - c.size_in_meters_y() / 2.0;
        c.update_origin(new_origin_x, new_origin_y);

        // StaticLayer::updateBounds for a rolling window: the window around the robot.
        let half_w = c.size_in_meters_x() / 2.0;
        let half_h = c.size_in_meters_y() / 2.0;
        let (min_x, min_y, max_x, max_y) = (robot_x - half_w, robot_y - half_h, robot_x + half_w, robot_y + half_h);
        let (x0, y0) = c.world_to_map_enforce_bounds(min_x, min_y);
        let (xn, yn) = c.world_to_map_enforce_bounds(max_x, max_y);
        let x0 = x0.max(0);
        let xn = (c.size_x() as i32).min(xn + 1);
        let y0 = y0.max(0);
        let yn = (c.size_y() as i32).min(yn + 1);
        if xn < x0 || yn < y0 {
            return;
        }
        c.reset_map(x0 as u32, y0 as u32, xn as u32, yn as u32);

        // StaticLayer::updateCosts, rolling branch (use_maximum false).
        for i in x0..xn {
            for j in y0..yn {
                let (wx, wy) = c.map_to_world(i as u32, j as u32);
                let (px, py) = map_from_window.apply_point(wx, wy);
                if let Some((mx, my)) = map.world_to_map(px, py) {
                    let v = map.cost(mx, my);
                    c.set_cost(i as u32, j as u32, v);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interpret_matches_static_layer() {
        let local = StaticLayerParams::new(false);
        let global = StaticLayerParams::new(true);
        assert_eq!(interpret_value(&global, (-1i8) as u8), NO_INFORMATION);
        assert_eq!(interpret_value(&local, (-1i8) as u8), FREE_SPACE);
        assert_eq!(interpret_value(&local, 100), LETHAL_OBSTACLE);
        assert_eq!(interpret_value(&local, 99), FREE_SPACE);
        // other negative int8 values are >= 100 as unsigned char
        assert_eq!(interpret_value(&global, (-2i8) as u8), LETHAL_OBSTACLE);
    }

    #[test]
    fn local_window_is_60_cells() {
        let w = RollingWindow::local_costmap();
        assert_eq!((w.costmap().size_x(), w.costmap().size_y()), (60, 60));
        assert!((w.costmap().size_in_meters_x() - 2.975).abs() < 1e-12);
    }

    #[test]
    fn update_origin_keeps_overlap() {
        let mut c = Costmap2D::new(4, 4, 1.0, 0.0, 0.0, 0);
        c.set_cost(2, 2, 7);
        c.update_origin(1.2, 1.9);
        assert_eq!((c.origin_x(), c.origin_y()), (1.0, 1.0));
        assert_eq!(c.cost(1, 1), 7);
        assert_eq!(c.data().iter().filter(|&&v| v != 0).count(), 1);
        c.update_origin(-0.5, 1.0); // -1.5 cells truncates to -1
        assert_eq!(c.origin_x(), 0.0);
        assert_eq!(c.cost(2, 1), 7);
    }
}
