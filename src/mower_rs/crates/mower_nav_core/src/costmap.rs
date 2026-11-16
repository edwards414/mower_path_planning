//! `nav2_costmap_2d` the way `nav2_no_map_params.yaml` configures it: two
//! costmaps on the static, fail-closed maps `map_manage` publishes.
//!
//! * global (`/map_grid_global`, frame `map`, `track_unknown_space: true`,
//!   not rolling, plugins `static_layer` then `inflation_layer`): the master
//!   grid takes the map's size, resolution and origin, every cell is the
//!   interpreted map value ([`static_costmap`]) and the [`InflationLayer`]
//!   then raises the cost of the free cells within `inflation_radius` of a
//!   lethal one -> [`GlobalLayers`], [`global_costmap`];
//! * local (`/map_grid`, frame `odom`, `track_unknown_space` left at false,
//!   3 x 3 m rolling window at 0.05 m, `static_layer` only): every update
//!   re-centres the window on the robot (grid-aligned,
//!   [`Costmap2D::update_origin`]) and resamples the whole window from the
//!   map through the `map <- odom` transform -> [`RollingWindow`].
//!
//! Upstream (nav2 1.3.13, the version `deploy/image/apt/runtime-apt.lock`
//! pins): `src/costmap_2d.cpp`, `src/layered_costmap.cpp`,
//! `src/costmap_2d_ros.cpp`, `plugins/static_layer.cpp`,
//! `plugins/inflation_layer.cpp`,
//! `include/nav2_costmap_2d/inflation_layer.hpp`,
//! `include/nav2_costmap_2d/cost_values.hpp`.

use crate::footprint::{calculate_min_and_max_distances, robot_footprint};
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

/// `robot_radius` of the global costmap in `nav2_no_map_params.yaml`.
///
/// `/map_grid_global` is already configuration space (`map_manage` eroded the
/// free space by the robot's safety radius), so the robot is a point on it.
/// The radius only sets the inflation layer's inscribed band, and that must
/// stay below one cell: a free cell next to the boundary, where coverage
/// paths start and end, must not become INSCRIBED (an obstacle to NavFn).
/// The local costmap keeps `robot_radius: 0.5`
/// (`rpp::CostmapContext::local_costmap`).
pub const GLOBAL_ROBOT_RADIUS: f64 = 0.02;

/// The `InflationLayer` parameters. `Default` is upstream's
/// (`InflationLayer::onInitialize`): radius 0.55 m, scaling factor 10, unknown
/// cells neither inflated nor inflated around.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InflationLayerParams {
    pub enabled: bool,
    /// Metres from a lethal cell up to which cells get a cost.
    pub inflation_radius: f64,
    /// The decay of the cost beyond the inscribed radius, per metre.
    pub cost_scaling_factor: f64,
    /// Give unknown cells any inflation cost, not only INSCRIBED.
    pub inflate_unknown: bool,
    /// Inflate around unknown cells as around lethal ones.
    pub inflate_around_unknown: bool,
}

impl Default for InflationLayerParams {
    fn default() -> Self {
        Self { enabled: true, inflation_radius: 0.55, cost_scaling_factor: 10.0, inflate_unknown: false, inflate_around_unknown: false }
    }
}

impl InflationLayerParams {
    /// `global_costmap.inflation_layer` in `nav2_no_map_params.yaml`: a cost
    /// gradient over the 0.5 m next to the boundary, nothing lethal, so NavFn
    /// keeps off the edge of the configuration space where there is room.
    pub fn mower_global() -> Self {
        Self { inflation_radius: 0.5, cost_scaling_factor: 4.0, ..Self::default() }
    }
}

/// `InflationLayer::CellData`: a cell waiting for its cost, and the obstacle
/// cell the inflation that reached it started at.
#[derive(Debug, Clone, Copy, PartialEq)]
struct CellData {
    x: u32,
    y: u32,
    src_x: u32,
    src_y: u32,
}

/// `nav2_costmap_2d::InflationLayer`: around every LETHAL cell of the master
/// grid, cells get a cost by their distance to it (in cells, times the
/// resolution): INSCRIBED (253) up to the inscribed radius of the footprint,
/// then `252 * exp(-cost_scaling_factor * (distance - inscribed_radius))`
/// truncated, out to `inflation_radius`. A cell keeps `max(old, new)`; an
/// unknown cell stays unknown unless the new cost is INSCRIBED or more (or
/// any cost, with `inflate_unknown`).
///
/// The distances are not an exact distance transform: the inflation spreads
/// from each obstacle through 4-connected neighbours, each cell taking the
/// obstacle of the first inflation that reaches it, in upstream's order
/// (bins by integer distance level, [`Self::update_costs`]). The port keeps
/// that order, so the costs are upstream's cell for cell.
#[derive(Debug, Clone, PartialEq)]
pub struct InflationLayer {
    params: InflationLayerParams,
    inscribed_radius: f64,
    /// The resolution the caches below were computed for (`matchSize`).
    resolution: Option<f64>,
    cell_inflation_radius: u32,
    /// `cell_inflation_radius + 2`: the side of the two caches.
    cache_length: usize,
    /// `hypot(i, j)` in cells at `i * cache_length + j`.
    cached_distances: Vec<f64>,
    cached_costs: Vec<u8>,
    /// `distance_matrix_` for non-negative offsets (it is symmetric): the
    /// rank of `i * i + j * j` among the squared distances that occur, which
    /// is the bin a cell at that offset from its obstacle waits in.
    distance_levels: Vec<usize>,
    /// `inflation_cells_`: one bin per distance level. Kept between updates
    /// as upstream keeps it (every bin is emptied as it is processed).
    inflation_cells: Vec<Vec<CellData>>,
    seen: Vec<bool>,
}

impl InflationLayer {
    /// The layer for a footprint with this inscribed radius
    /// (`LayeredCostmap::getInscribedRadius`, `onFootprintChanged`).
    pub fn new(params: InflationLayerParams, inscribed_radius: f64) -> Self {
        Self {
            params,
            inscribed_radius,
            resolution: None,
            cell_inflation_radius: 0,
            cache_length: 0,
            cached_distances: Vec::new(),
            cached_costs: Vec::new(),
            distance_levels: Vec::new(),
            inflation_cells: Vec::new(),
            seen: Vec::new(),
        }
    }

    /// The layer for a `robot_radius` and no `footprint`: the inscribed
    /// radius is the padded 16-point circle's ([`robot_footprint`]), which for
    /// 0.02 m is 0.0294 m (the padding's 0.01 counts), not 0.02.
    pub fn for_robot_radius(params: InflationLayerParams, robot_radius: f64) -> Self {
        let (inscribed, _) = calculate_min_and_max_distances(&robot_footprint(robot_radius));
        Self::new(params, inscribed)
    }

    /// This robot's `global_costmap.inflation_layer`.
    pub fn mower_global() -> Self {
        Self::for_robot_radius(InflationLayerParams::mower_global(), GLOBAL_ROBOT_RADIUS)
    }

    pub fn params(&self) -> &InflationLayerParams {
        &self.params
    }

    pub fn inscribed_radius(&self) -> f64 {
        self.inscribed_radius
    }

    /// `inflation_radius` in cells (`Costmap2D::cellDistance`: rounded up),
    /// for the resolution of the last [`Self::match_size`].
    pub fn cell_inflation_radius(&self) -> u32 {
        self.cell_inflation_radius
    }

    /// `computeCost` for a distance in cells at `resolution`.
    pub fn compute_cost(&self, distance: f64, resolution: f64) -> u8 {
        if distance == 0.0 {
            LETHAL_OBSTACLE
        } else if distance * resolution <= self.inscribed_radius {
            INSCRIBED_INFLATED_OBSTACLE
        } else {
            // `distance * resolution_ - inscribed_radius_` in one expression:
            // fused, as the shipped aarch64 build contracts it (see the
            // README on floats; `no_mower_cost_is_near_a_rounding_edge`
            // shows it cannot change a cost of this robot's table).
            let factor = (-1.0 * self.params.cost_scaling_factor * distance.mul_add(resolution, -self.inscribed_radius)).exp();
            // double -> unsigned char: truncation.
            ((INSCRIBED_INFLATED_OBSTACLE - 1) as f64 * factor) as u8
        }
    }

    /// `matchSize` + `computeCaches` + `generateIntegerDistances`: the caches
    /// for a master grid of this resolution. [`Self::update_costs`] calls it.
    pub fn match_size(&mut self, resolution: f64) {
        if self.resolution == Some(resolution) {
            return;
        }
        self.resolution = Some(resolution);
        // Costmap2D::cellDistance
        self.cell_inflation_radius = (self.params.inflation_radius / resolution).ceil().max(0.0) as u32;
        if self.cell_inflation_radius == 0 {
            return;
        }
        let n = self.cell_inflation_radius as usize + 2;
        self.cache_length = n;
        self.cached_distances = vec![0.0; n * n];
        self.cached_costs = vec![0; n * n];
        for i in 0..n {
            for j in 0..n {
                // hypot(i, j) of two small integers: i * i + j * j is exact,
                // so the correctly rounded root is the correctly rounded hypot.
                let d = ((i * i + j * j) as f64).sqrt();
                self.cached_distances[i * n + j] = d;
                self.cached_costs[i * n + j] = self.compute_cost(d, resolution);
            }
        }
        // generateIntegerDistances: the points within r = radius + 2 cells,
        // sorted by squared distance; a point's level is the rank of its
        // squared distance. Only that rank matters, so the quadrant is enough.
        let r = n;
        let mut squares: Vec<usize> = Vec::new();
        for x in 0..=r {
            for y in 0..=r {
                if x * x + y * y <= r * r {
                    squares.push(x * x + y * y);
                }
            }
        }
        squares.sort_unstable();
        squares.dedup();
        self.distance_levels = vec![0; (r + 1) * (r + 1)];
        for x in 0..=r {
            for y in 0..=r {
                if let Ok(level) = squares.binary_search(&(x * x + y * y)) {
                    self.distance_levels[x * (r + 1) + y] = level;
                }
            }
        }
        self.inflation_cells.clear();
        self.inflation_cells.resize(squares.len(), Vec::new());
    }

    /// `updateCosts(master_grid, min_i, min_j, max_i, max_j)`: inflate the
    /// half-open window of the master grid, from the obstacles in it and up
    /// to `cell_inflation_radius` cells around it.
    pub fn update_costs(&mut self, master: &mut Costmap2D, min_i: i32, min_j: i32, max_i: i32, max_j: i32) {
        self.match_size(master.resolution);
        if !self.params.enabled || self.cell_inflation_radius == 0 {
            return;
        }
        let size_x = master.size_x as usize;
        let size_y = master.size_y as usize;
        if self.seen.len() != size_x * size_y {
            self.seen = vec![false; size_x * size_y];
        }
        self.seen.fill(false);

        let (base_min_i, base_min_j, base_max_i, base_max_j) = (min_i, min_j, max_i, max_j);
        let r = self.cell_inflation_radius as i32;
        let min_i = (min_i - r).max(0);
        let min_j = (min_j - r).max(0);
        let max_i = (max_i + r).min(size_x as i32);
        let max_j = (max_j + r).min(size_y as i32);

        // the obstacles, at distance 0
        for j in min_j..max_j {
            for i in min_i..max_i {
                let cost = master.data[j as usize * size_x + i as usize];
                if cost == LETHAL_OBSTACLE || (self.params.inflate_around_unknown && cost == NO_INFORMATION) {
                    self.inflation_cells[0].push(CellData { x: i as u32, y: j as u32, src_x: i as u32, src_y: j as u32 });
                }
            }
        }

        // by increasing distance; a bin grows while it is being processed
        for level in 0..self.inflation_cells.len() {
            let mut k = 0;
            while k < self.inflation_cells[level].len() {
                let cell = self.inflation_cells[level][k];
                k += 1;
                let (mx, my) = (cell.x as usize, cell.y as usize);
                let index = my * size_x + mx;
                if self.seen[index] {
                    continue;
                }
                self.seen[index] = true;

                let cost = self.cached_costs[self.cache_index(cell.x, cell.y, cell.src_x, cell.src_y)];
                let old_cost = master.data[index];
                // only inside the given bounds
                if (mx as i32) >= base_min_i && (my as i32) >= base_min_j && (mx as i32) < base_max_i && (my as i32) < base_max_j {
                    let over_unknown =
                        if self.params.inflate_unknown { cost > FREE_SPACE } else { cost >= INSCRIBED_INFLATED_OBSTACLE };
                    master.data[index] = if old_cost == NO_INFORMATION && over_unknown { cost } else { old_cost.max(cost) };
                }

                // the four neighbours, in upstream's order
                if mx > 0 {
                    self.enqueue(index - 1, cell.x - 1, cell.y, cell.src_x, cell.src_y);
                }
                if my > 0 {
                    self.enqueue(index - size_x, cell.x, cell.y - 1, cell.src_x, cell.src_y);
                }
                if mx < size_x - 1 {
                    self.enqueue(index + 1, cell.x + 1, cell.y, cell.src_x, cell.src_y);
                }
                if my < size_y - 1 {
                    self.enqueue(index + size_x, cell.x, cell.y + 1, cell.src_x, cell.src_y);
                }
            }
            // this level is done (upstream frees it here too)
            self.inflation_cells[level] = Vec::new();
        }
    }

    fn cache_index(&self, mx: u32, my: u32, src_x: u32, src_y: u32) -> usize {
        mx.abs_diff(src_x) as usize * self.cache_length + my.abs_diff(src_y) as usize
    }

    /// `enqueue`: a cell not yet seen waits in the bin of its distance to the
    /// obstacle, if that is within the inflation radius.
    fn enqueue(&mut self, index: usize, mx: u32, my: u32, src_x: u32, src_y: u32) {
        if self.seen[index] {
            return;
        }
        let distance = self.cached_distances[self.cache_index(mx, my, src_x, src_y)];
        if distance > self.cell_inflation_radius as f64 {
            return;
        }
        let level = self.distance_levels[mx.abs_diff(src_x) as usize * (self.cache_length + 1) + my.abs_diff(src_y) as usize];
        self.inflation_cells[level].push(CellData { x: mx, y: my, src_x, src_y });
    }

    /// Cells left waiting after an update. Upstream expects none ("The
    /// inflation list must be empty at the beginning of inflation").
    pub fn pending(&self) -> usize {
        self.inflation_cells.iter().map(Vec::len).sum()
    }
}

/// The global costmap's plugins, `static_layer` then `inflation_layer`, as
/// `LayeredCostmap::updateMap` runs them on a costmap that is not rolling.
///
/// Upstream updates the master grid only when a layer reports bounds, and
/// then (here, where the static layer's bounds are the whole map) all of it:
/// the window is reset, the static layer overwrites it with its own grid and
/// the inflation layer inflates it ([`Self::repaint`]). An update with no
/// bounds leaves the master grid alone, so what NavFn's `clearRobotCell`
/// wrote into it stays until the next repaint. Which updates repaint
/// ([`Self::update`], from the two `updateBounds`):
///
/// * the first update (the footprint was set at configure:
///   `need_reinflation_`), and the first after a reset
///   (`clear_entirely_global_costmap`: `has_updated_data_` and
///   `need_reinflation_`): once;
/// * the update that takes a new map (`has_updated_data_`) **and the one
///   after it**: the inflation layer adds the bounds of the previous update
///   to its own (`last_min_x_` ...), and only the `need_reinflation_` path
///   clears them. So a cell NavFn cleared in the second after a new map is
///   painted over again one update later.
#[derive(Debug, Clone, PartialEq)]
pub struct GlobalLayers {
    static_layer: StaticLayerParams,
    inflation: InflationLayer,
    /// `StaticLayer::has_updated_data_`: a map or a reset the next update
    /// reports bounds for.
    has_updated_data: bool,
    /// `InflationLayer::need_reinflation_`.
    need_reinflation: bool,
    /// The inflation layer's `last_min_x_` ... hold the previous update's
    /// bounds (the whole map) rather than "no expansion".
    last_bounds: bool,
}

impl GlobalLayers {
    /// The layers after `Costmap2DROS::on_configure`, which sets the
    /// footprint (`onFootprintChanged`: `need_reinflation_`).
    pub fn new(static_layer: StaticLayerParams, inflation: InflationLayer) -> Self {
        Self { static_layer, inflation, has_updated_data: false, need_reinflation: true, last_bounds: false }
    }

    /// `global_costmap` in `nav2_no_map_params.yaml`.
    pub fn mower() -> Self {
        Self::new(StaticLayerParams::new(true), InflationLayer::mower_global())
    }

    /// The static layer's own grid for a `/map_grid_global` (`processMap`);
    /// tell the layers with [`Self::new_map`] when it replaces the last one.
    pub fn static_layer(&self, grid: &OccupancyGrid) -> Costmap2D {
        static_costmap(grid, &self.static_layer)
    }

    /// The static layer took a map (`processMap`: `has_updated_data_`).
    pub fn new_map(&mut self) {
        self.has_updated_data = true;
    }

    /// `Costmap2DROS::resetLayers`, for a clear of the whole costmap: both
    /// layers' `reset()`.
    pub fn reset(&mut self) {
        self.has_updated_data = true;
        self.need_reinflation = true;
    }

    /// One `LayeredCostmap::updateMap` with `layer` the static layer's grid:
    /// repaints the master grid if a layer reports bounds (see the type's
    /// doc for when), and says whether it did.
    pub fn update(&mut self, layer: &Costmap2D, master: &mut Costmap2D) -> bool {
        // StaticLayer::updateBounds: the whole map, once per new data
        let static_bounds = std::mem::take(&mut self.has_updated_data);
        // InflationLayer::updateBounds
        let repaint = if self.need_reinflation {
            self.need_reinflation = false;
            self.last_bounds = false;
            true
        } else {
            let previous = std::mem::replace(&mut self.last_bounds, static_bounds);
            previous || static_bounds
        };
        if repaint {
            self.repaint(layer, master);
        }
        repaint
    }

    /// The master grid becomes the static layer's grid (`resetMap` +
    /// `updateWithTrueOverwrite` over the whole map), inflated.
    pub fn repaint(&mut self, layer: &Costmap2D, master: &mut Costmap2D) {
        master.clone_from(layer);
        self.inflation.update_costs(master, 0, 0, layer.size_x as i32, layer.size_y as i32);
    }
}

/// The global costmap's master grid after the update that takes `grid`: what
/// `planner_server` plans on.
pub fn global_costmap(grid: &OccupancyGrid) -> Costmap2D {
    let mut layers = GlobalLayers::mower();
    let layer = layers.static_layer(grid);
    let mut master = layer.clone();
    layers.new_map();
    layers.update(&layer, &mut master);
    master
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
    /// `(double)0.05f`, the resolution of every map `map_manage` publishes.
    const RES: f64 = 0.05f32 as f64;

    fn free_map(w: u32, h: u32, lethal: &[(u32, u32)], unknown: &[(u32, u32)]) -> Costmap2D {
        let mut m = Costmap2D::new(w, h, RES, 0.0, 0.0, NO_INFORMATION);
        m.data.fill(FREE_SPACE);
        for &(x, y) in lethal {
            m.set_cost(x, y, LETHAL_OBSTACLE);
        }
        for &(x, y) in unknown {
            m.set_cost(x, y, NO_INFORMATION);
        }
        m
    }

    fn inflate(layer: &mut InflationLayer, m: &mut Costmap2D) {
        let (w, h) = (m.size_x() as i32, m.size_y() as i32);
        layer.update_costs(m, 0, 0, w, h);
        assert_eq!(layer.pending(), 0);
    }

    #[test]
    fn mower_global_inflation_parameters() {
        let layer = InflationLayer::mower_global();
        assert_eq!(*layer.params(), InflationLayerParams { enabled: true, inflation_radius: 0.5, cost_scaling_factor: 4.0, inflate_unknown: false, inflate_around_unknown: false });
        // the 16-point circle of radius 0.02 padded by 0.01 per axis: its
        // closest edge, well below one 0.05 m cell
        assert!((layer.inscribed_radius() - 0.029_404_946_143_704_71).abs() < 1e-15, "{}", layer.inscribed_radius());
        let mut layer = layer;
        layer.match_size(RES);
        assert_eq!(layer.cell_inflation_radius(), 10);
    }

    /// 252 * exp(-4 * (d * 0.05 - 0.0294049...)) by hand (python3, math.exp),
    /// for d cells straight and diagonal.
    #[test]
    fn mower_costs_by_distance() {
        let mut m = free_map(31, 31, &[(15, 15)], &[]);
        inflate(&mut InflationLayer::mower_global(), &mut m);
        assert_eq!(m.cost(15, 15), LETHAL_OBSTACLE);
        let straight = [232, 190, 155, 127, 104, 85, 69, 57, 46, 38];
        for (k, &c) in straight.iter().enumerate() {
            let d = k as u32 + 1;
            for (x, y) in [(15 + d, 15), (15 - d, 15), (15, 15 + d), (15, 15 - d)] {
                assert_eq!(m.cost(x, y), c, "{d} cells from the obstacle at ({x}, {y})");
            }
        }
        assert_eq!(m.cost(16, 16), 213); // sqrt(2) cells, 0.0707 m
        assert_eq!(m.cost(17, 16), 181); // sqrt(5)
        assert_eq!(m.cost(22, 22), 39); // sqrt(98) = 9.899
        assert_eq!(m.cost(21, 23), 38); // 6-8-10: exactly on the radius
        // beyond the radius: untouched
        assert_eq!(m.cost(26, 15), FREE_SPACE);
        assert_eq!(m.cost(23, 22), FREE_SPACE); // sqrt(113) = 10.63
        assert_eq!(m.cost(22, 23), FREE_SPACE);
        // nothing but the obstacle is an obstacle to NavFn (253 or more)
        assert_eq!(m.data().iter().filter(|&&c| c >= INSCRIBED_INFLATED_OBSTACLE).count(), 1);
    }

    /// Whatever libm or contraction computes the table, no cost of this
    /// robot's is within 1e-3 of the next integer, so none can come out one off.
    #[test]
    fn no_mower_cost_is_near_a_rounding_edge() {
        let layer = InflationLayer::mower_global();
        for i in 0..12u32 {
            for j in 0..12u32 {
                let d = ((i * i + j * j) as f64).sqrt();
                if d == 0.0 {
                    continue;
                }
                let v = 252.0 * (-4.0 * (d * RES - layer.inscribed_radius())).exp();
                assert!((v - v.round()).abs() > 1e-3, "({i}, {j}): {v}");
                assert_eq!(layer.compute_cost(d, RES), v as u8);
            }
        }
    }

    /// The cell takes the nearest obstacle's cost, and never less than it had.
    #[test]
    fn inflation_keeps_the_maximum() {
        let mut m = free_map(40, 5, &[(5, 2), (12, 2)], &[]);
        m.set_cost(8, 2, 200);
        m.set_cost(9, 2, 100);
        inflate(&mut InflationLayer::mower_global(), &mut m);
        assert_eq!(m.cost(6, 2), 232);
        assert_eq!(m.cost(7, 2), 190);
        assert_eq!(m.cost(8, 2), 200); // 3 cells from (5, 2) is 155: kept
        assert_eq!(m.cost(9, 2), 155); // 3 cells from (12, 2): raised
        assert_eq!(m.cost(22, 2), 38);
        assert_eq!(m.cost(23, 2), FREE_SPACE);
    }

    /// `inflate_unknown: false`, `inflate_around_unknown: false` (the
    /// defaults, which the yaml keeps): unknown cells are no source, and stay
    /// unknown unless they are within the inscribed radius.
    #[test]
    fn unknown_cells() {
        let cells = |m: &Costmap2D| (9..=13).map(|x| m.cost(x, 2)).collect::<Vec<_>>();
        let map = || free_map(30, 5, &[(10, 2)], &[(11, 2), (12, 2)]);

        let mut m = map();
        inflate(&mut InflationLayer::mower_global(), &mut m);
        // the inflation still spreads through the unknown cells
        assert_eq!(cells(&m), [232, 254, 255, 255, 155]);

        // a radius of 0.5 m: the first unknown cell is inside the inscribed band
        let mut m = map();
        inflate(&mut InflationLayer::for_robot_radius(InflationLayerParams::mower_global(), 0.5), &mut m);
        assert_eq!(cells(&m)[..3], [253, 254, 253]);
        assert_eq!(m.cost(20, 2), 253); // 0.5 m <= 0.50015 m
        assert_eq!(m.cost(21, 2), FREE_SPACE);

        let mut m = map();
        inflate(&mut InflationLayer::new(InflationLayerParams { inflate_unknown: true, ..InflationLayerParams::mower_global() }, 0.0294), &mut m);
        assert_eq!(cells(&m), [232, 254, 232, 190, 155]);

        // around unknown: an unknown cell is an obstacle cell, and reads LETHAL after
        let mut m = map();
        inflate(&mut InflationLayer::new(InflationLayerParams { inflate_around_unknown: true, ..InflationLayerParams::mower_global() }, 0.0294), &mut m);
        assert_eq!(cells(&m), [232, 254, 254, 254, 232]);
    }

    #[test]
    fn disabled_or_zero_radius_does_nothing() {
        for params in [
            InflationLayerParams { enabled: false, ..InflationLayerParams::mower_global() },
            InflationLayerParams { inflation_radius: 0.0, ..InflationLayerParams::mower_global() },
        ] {
            let mut m = free_map(9, 9, &[(4, 4)], &[]);
            let before = m.clone();
            inflate(&mut InflationLayer::new(params, 0.0294), &mut m);
            assert_eq!(m, before);
        }
    }

    /// Only the window is written, but obstacles up to the radius outside it count.
    #[test]
    fn a_window_takes_the_obstacles_around_it() {
        let mut m = free_map(40, 9, &[(3, 4)], &[]);
        let mut layer = InflationLayer::mower_global();
        layer.update_costs(&mut m, 6, 0, 40, 9);
        assert_eq!(layer.pending(), 0);
        assert_eq!(m.cost(4, 4), FREE_SPACE);
        assert_eq!(m.cost(5, 4), FREE_SPACE);
        assert_eq!(m.cost(6, 4), 155);
        assert_eq!(m.cost(13, 4), 38);
        // an obstacle more than the radius outside the window is not looked at
        let mut m = free_map(40, 9, &[(3, 4)], &[]);
        layer.update_costs(&mut m, 14, 0, 40, 9);
        assert!(m.data().iter().all(|&c| c == FREE_SPACE || c == LETHAL_OBSTACLE));
    }

    /// The layer follows the master grid's resolution and size between updates.
    #[test]
    fn caches_follow_the_resolution() {
        let mut layer = InflationLayer::mower_global();
        let mut m = free_map(31, 31, &[(15, 15)], &[]);
        inflate(&mut layer, &mut m);
        let mut coarse = Costmap2D::new(21, 21, 0.1, 0.0, 0.0, FREE_SPACE);
        coarse.set_cost(10, 10, LETHAL_OBSTACLE);
        inflate(&mut layer, &mut coarse);
        assert_eq!(layer.cell_inflation_radius(), 5);
        assert_eq!(coarse.cost(11, 10), 190); // 0.1 m
        assert_eq!(coarse.cost(15, 10), 38);
        assert_eq!(coarse.cost(16, 10), FREE_SPACE);
    }

    /// `global_costmap`: the static layer's cells, inflated; free cells next
    /// to the boundary stay below INSCRIBED, so NavFn can still reach them.
    #[test]
    fn global_costmap_inflates_the_static_layer() {
        let (w, h) = (40u32, 30u32);
        let data = (0..h).flat_map(|j| (0..w).map(move |i| if i == 0 || j == 0 || i == w - 1 || j == h - 1 { 100 } else if (i, j) == (20, 15) { -1 } else { 0 })).collect();
        let grid = OccupancyGrid { width: w, height: h, resolution: RES, origin_x: -1.0, origin_y: -0.75, data };
        let m = global_costmap(&grid);
        assert_eq!((m.size_x(), m.size_y(), m.origin_x(), m.origin_y()), (w, h, -1.0, -0.75));
        assert_eq!(m.cost(0, 7), LETHAL_OBSTACLE);
        assert_eq!(m.cost(1, 7), 232);
        assert_eq!(m.cost(1, 1), 232);
        assert_eq!(m.cost(10, 15), 38);
        assert_eq!(m.cost(11, 15), FREE_SPACE);
        assert_eq!(m.cost(20, 15), NO_INFORMATION);
        let layers = GlobalLayers::mower();
        let layer = layers.static_layer(&grid);
        for (a, b) in m.data().iter().zip(layer.data()) {
            assert!(a >= b && (*b != FREE_SPACE || *a < INSCRIBED_INFLATED_OBSTACLE), "{a} over {b}");
        }
        // a repaint starts from the static layer again: no cost builds up,
        // and a hole NavFn's clearRobotCell left is painted over
        let mut layers = layers;
        let mut again = m.clone();
        again.set_cost(1, 7, FREE_SPACE);
        layers.repaint(&layer, &mut again);
        assert_eq!(again, m);
    }

    /// Which updates repaint the master grid (seen as: is a cell NavFn
    /// cleared painted over).
    #[test]
    fn global_layers_repaint_once_at_start_and_after_a_reset_and_twice_for_a_new_map() {
        let (w, h) = (30u32, 20u32);
        let data: Vec<i8> = (0..h).flat_map(|j| (0..w).map(move |i| if i == 0 || j == 0 || i == w - 1 || j == h - 1 { 100 } else { 0 })).collect();
        let grid = OccupancyGrid { width: w, height: h, resolution: RES, origin_x: 0.0, origin_y: 0.0, data };
        let mut layers = GlobalLayers::mower();
        let layer = layers.static_layer(&grid);
        let mut master = layer.clone();
        // one update: whether it repainted, and whether a cleared cell is back
        let update = |layers: &mut GlobalLayers, master: &mut Costmap2D| {
            master.set_cost(1, 1, FREE_SPACE);
            let repainted = layers.update(&layer, master);
            assert_eq!(master.cost(1, 1), if repainted { 232 } else { FREE_SPACE });
            repainted
        };
        let run = |layers: &mut GlobalLayers, master: &mut Costmap2D, n: usize| (0..n).map(|_| update(layers, master)).collect::<Vec<_>>();
        let (l, m) = (&mut layers, &mut master);

        // the first map, there at the first update: the footprint's reinflation
        l.new_map();
        assert_eq!(run(l, m, 4), [true, false, false, false]);
        // a new map: the update that takes it and the one after
        l.new_map();
        assert_eq!(run(l, m, 4), [true, true, false, false]);
        // a clear of the whole costmap: once
        l.reset();
        assert_eq!(run(l, m, 3), [true, false, false]);
        // a new map every update, then none
        l.new_map();
        assert!(update(l, m));
        l.new_map();
        assert_eq!(run(l, m, 3), [true, true, false]);
        // a new map in the update of a clear: the reinflation path, once
        l.new_map();
        l.reset();
        assert_eq!(run(l, m, 3), [true, false, false]);
        // a clear in the update after a new map: still that one repaint
        l.new_map();
        assert!(update(l, m));
        l.reset();
        assert_eq!(run(l, m, 3), [true, false, false]);
    }
}
