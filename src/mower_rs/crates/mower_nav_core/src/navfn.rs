//! `nav2_navfn_planner::NavFn`: the navigation function the NavFn planner
//! computes on the global costmap, and the gradient descent that reads a path
//! off it.
//!
//! * [`NavFn::set_costmap`] translates costmap costs: 0..=252 -> `50 + 0.8 c`,
//!   253 / 254 -> 254 (obstacle), 255 (unknown) -> 253 with `allow_unknown`,
//!   else obstacle;
//! * [`NavFn::calc_nav_fn_dijkstra`] seeds potential 0 at the goal cell and
//!   spreads it with the planar-wave update over three priority buffers of at
//!   most [`PRIORITYBUFSIZE`] cells (a cell that does not fit is dropped, as
//!   upstream), for at most `max(nx * ny / 20, nx + ny)` cycles;
//! * [`NavFn::calc_path`] descends the gradient from the start cell in
//!   half-cell steps, following the 8-neighbour minimum where a neighbour is
//!   unreached.
//!
//! Note the naming: the planner seeds NavFn's *goal* with the robot's start
//! and asks for a path from NavFn's *start*, the robot's goal.
//!
//! The arithmetic is upstream's to the bit: `f32` where upstream uses
//! `float`, `f64` where a double literal promotes, and a fused multiply-add
//! exactly where the shipped aarch64 `libnav2_navfn_planner.so` has an
//! `fmadd` (GCC contracts `a + b * c` by default). glibc's `hypotf` is
//! `(float)sqrt((double)x * x + (double)y * y)` and its `hypot` of small
//! integers is the correctly rounded root, which is what [`hypotf`] and
//! [`hypot_int`] compute.
//!
//! Where upstream reads or writes outside its arrays (a cell index the caller
//! put past the grid), this port reads `POT_HIGH` and drops the write
//! instead; the planner reaches that only for a start or goal that rounds
//! past the top row (the top half-row of the map, or the right half of its
//! top-right cell), see `planner.rs`.
//!
//! Upstream: `src/navfn.cpp`, `include/nav2_navfn_planner/navfn.hpp`.

/// `COST_UNKNOWN_ROS`: the costmap's NO_INFORMATION.
pub const COST_UNKNOWN_ROS: u8 = 255;
/// `COST_OBS`: forbidden cells, also the map border.
pub const COST_OBS: u8 = 254;
/// `COST_OBS_ROS`: the costmap's INSCRIBED_INFLATED_OBSTACLE, an obstacle here.
pub const COST_OBS_ROS: u8 = 253;
/// `COST_NEUTRAL`: the cost of free space.
pub const COST_NEUTRAL: u8 = 50;
/// `COST_FACTOR`, a double.
pub const COST_FACTOR: f64 = 0.8;
/// `POT_HIGH` (1.0e10, exactly representable as `f32`): unreached.
pub const POT_HIGH: f32 = 1.0e10;
/// `PRIORITYBUFSIZE`: the capacity of each priority buffer.
pub const PRIORITYBUFSIZE: usize = 10000;
/// `terminal_checking_interval`: cycles between cancel checks.
pub const TERMINAL_CHECKING_INTERVAL: i32 = 5000;
/// `INVSQRT2`, a double literal: upstream's 9 digits, not `FRAC_1_SQRT_2`.
#[allow(clippy::approx_constant)]
const INVSQRT2: f64 = 0.707106781;

/// `nav2_core::PlannerCancelled`, thrown when the cancel checker says so.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Cancelled;

/// What upstream's `RCLCPP_DEBUG` after a propagation reports: cycles run,
/// cells taken from the current buffer, and the largest current buffer.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct PropagationStats {
    pub cycles: i32,
    pub visited: i32,
    pub max_buffer: i32,
}

/// glibc's `hypotf` for finite arguments: the squares are exact in double,
/// so this is one rounding of the sum, one of the root and one to float.
pub fn hypotf(x: f32, y: f32) -> f32 {
    let (x, y) = (x as f64, y as f64);
    (x * x + y * y).sqrt() as f32
}

/// glibc's `hypot` on the `int` cell offsets A* passes it: the sum of squares
/// is exact, and glibc returns its correctly rounded root.
pub fn hypot_int(a: i32, b: i32) -> f64 {
    let (a, b) = (a as f64, b as f64);
    (a * a + b * b).sqrt()
}

/// The `push_cur` / `push_next` / `push_over` macros.
fn push(buf: &mut Vec<i32>, pending: &mut [bool], costarr: &[u8], n: i32) {
    if n >= 0 && (n as usize) < pending.len() {
        let i = n as usize;
        if !pending[i] && costarr[i] < COST_OBS && buf.len() < PRIORITYBUFSIZE {
            buf.push(n);
            pending[i] = true;
        }
    }
}

/// `NavFn`: the cost, potential and gradient arrays of one grid, row-major
/// with `index = y * nx + x`.
#[derive(Debug, Clone)]
pub struct NavFn {
    nx: i32,
    ny: i32,
    ns: i32,
    costarr: Vec<u8>,
    potarr: Vec<f32>,
    pending: Vec<bool>,
    nobs: i32,
    cur_p: Vec<i32>,
    next_p: Vec<i32>,
    over_p: Vec<i32>,
    cur_t: f32,
    pri_inc: f32,
    goal: [i32; 2],
    start: [i32; 2],
    gradx: Vec<f32>,
    grady: Vec<f32>,
    pathx: Vec<f32>,
    pathy: Vec<f32>,
    last_path_cost: f32,
    path_step: f32,
    stats: PropagationStats,
}

impl NavFn {
    /// The constructor: `setNavArr(nx, ny)`, `priInc = 2 * COST_NEUTRAL`,
    /// `pathStep = 0.5`.
    pub fn new(nx: i32, ny: i32) -> Self {
        let mut nav = Self {
            nx: 0,
            ny: 0,
            ns: 0,
            costarr: Vec::new(),
            potarr: Vec::new(),
            pending: Vec::new(),
            nobs: 0,
            cur_p: Vec::with_capacity(PRIORITYBUFSIZE),
            next_p: Vec::with_capacity(PRIORITYBUFSIZE),
            over_p: Vec::with_capacity(PRIORITYBUFSIZE),
            cur_t: COST_OBS as f32,
            pri_inc: 2.0 * COST_NEUTRAL as f32,
            goal: [0, 0],
            start: [0, 0],
            gradx: Vec::new(),
            grady: Vec::new(),
            pathx: Vec::new(),
            pathy: Vec::new(),
            last_path_cost: 0.0,
            path_step: 0.5,
            stats: PropagationStats::default(),
        };
        nav.set_nav_arr(nx, ny);
        nav
    }

    pub fn nx(&self) -> i32 {
        self.nx
    }
    pub fn ny(&self) -> i32 {
        self.ny
    }
    /// The translated costs (`costarr`), with the border set to COST_OBS once
    /// a propagation has run.
    pub fn costarr(&self) -> &[u8] {
        &self.costarr
    }
    /// The navigation function (`potarr`): `POT_HIGH` where unreached.
    pub fn potarr(&self) -> &[f32] {
        &self.potarr
    }
    /// `nobs`: obstacle cells counted by the last `setupNavFn`.
    pub fn nobs(&self) -> i32 {
        self.nobs
    }
    pub fn goal(&self) -> [i32; 2] {
        self.goal
    }
    pub fn start(&self) -> [i32; 2] {
        self.start
    }
    pub fn stats(&self) -> PropagationStats {
        self.stats
    }
    /// `getPathX`: path points in sub-cell grid coordinates.
    pub fn path_x(&self) -> &[f32] {
        &self.pathx
    }
    pub fn path_y(&self) -> &[f32] {
        &self.pathy
    }
    /// `getPathLen`: `npath`, also after a failed `calcPath`.
    pub fn path_len(&self) -> i32 {
        self.pathx.len() as i32
    }
    /// `getLastPathCost`: the start cell's potential after an A* run (0
    /// before any; upstream leaves it uninitialised).
    pub fn last_path_cost(&self) -> f32 {
        self.last_path_cost
    }

    /// `setNavArr`: resize, every cost 0 and nothing pending. Upstream leaves
    /// the potential and gradient arrays uninitialised; `setupNavFn` fills
    /// them before anything reads them.
    pub fn set_nav_arr(&mut self, nx: i32, ny: i32) {
        self.nx = nx;
        self.ny = ny;
        self.ns = nx * ny;
        let ns = self.ns.max(0) as usize;
        self.costarr = vec![0; ns];
        self.potarr = vec![0.0; ns];
        self.pending = vec![false; ns];
        self.gradx = vec![0.0; ns];
        self.grady = vec![0.0; ns];
    }

    /// `setGoal`: the cell the potential is seeded at.
    pub fn set_goal(&mut self, goal: [i32; 2]) {
        self.goal = goal;
    }

    /// `setStart`: the cell `calcPath` descends from.
    pub fn set_start(&mut self, start: [i32; 2]) {
        self.start = start;
    }

    /// `setCostmap(cmap, isROS, allow_unknown)`. `cmap` is row-major like
    /// `Costmap2D::getCharMap` and at least `nx * ny` long. The non-ROS
    /// (PGM) branch makes a 7-cell border of obstacles and always passes
    /// unknown.
    pub fn set_costmap(&mut self, cmap: &[u8], is_ros: bool, allow_unknown: bool) {
        for i in 0..self.ny {
            for j in 0..self.nx {
                let k = (i * self.nx + j) as usize;
                self.costarr[k] = COST_OBS;
                if !is_ros && (i < 7 || i > self.ny - 8 || j < 7 || j > self.nx - 8) {
                    continue; // don't do borders
                }
                let v = cmap[k] as i32;
                if v < COST_OBS_ROS as i32 {
                    // int = 50 + 0.8 * v: one fmadd, then truncation
                    let mut v = (v as f64).mul_add(COST_FACTOR, COST_NEUTRAL as f64) as i32;
                    if v >= COST_OBS as i32 {
                        v = COST_OBS as i32 - 1;
                    }
                    self.costarr[k] = v as u8;
                } else if v == COST_UNKNOWN_ROS as i32 && (allow_unknown || !is_ros) {
                    self.costarr[k] = COST_OBS - 1;
                }
            }
        }
    }

    /// `calcNavFnDijkstra`: `setupNavFn(true)` and a Dijkstra propagation;
    /// `at_start` stops it as soon as the start cell has a potential. Returns
    /// whether it ended before the cycle limit.
    pub fn calc_nav_fn_dijkstra(
        &mut self,
        cancel: &mut dyn FnMut() -> bool,
        at_start: bool,
    ) -> Result<bool, Cancelled> {
        self.setup_nav_fn(true);
        let cycles = (self.nx * self.ny / 20).max(self.nx + self.ny);
        self.prop_nav_fn_dijkstra(cycles, cancel, at_start)
    }

    /// `calcNavFnAstar`: `setupNavFn(true)` and an A* propagation, which
    /// always stops at the start cell. Returns whether the start was reached.
    pub fn calc_nav_fn_astar(&mut self, cancel: &mut dyn FnMut() -> bool) -> Result<bool, Cancelled> {
        self.setup_nav_fn(true);
        let cycles = (self.nx * self.ny / 20).max(self.nx + self.ny);
        self.prop_nav_fn_astar(cycles, cancel)
    }

    /// `setupNavFn`: every potential POT_HIGH, the border an obstacle, the
    /// buffers empty, the goal seeded (`initCost(goal, 0)`), `nobs` counted.
    fn setup_nav_fn(&mut self, keepit: bool) {
        let (nx, ny) = (self.nx as usize, self.ny as usize);
        for i in 0..self.ns as usize {
            self.potarr[i] = POT_HIGH;
            if !keepit {
                self.costarr[i] = COST_NEUTRAL;
            }
            self.gradx[i] = 0.0;
            self.grady[i] = 0.0;
        }
        if self.ns > 0 {
            for i in 0..nx {
                self.costarr[i] = COST_OBS;
                self.costarr[(ny - 1) * nx + i] = COST_OBS;
            }
            for i in 0..ny {
                self.costarr[i * nx] = COST_OBS;
                self.costarr[i * nx + nx - 1] = COST_OBS;
            }
        }

        self.cur_t = COST_OBS as f32;
        self.cur_p.clear();
        self.next_p.clear();
        self.over_p.clear();
        self.pending.fill(false);

        let k = self.goal[0] + self.goal[1] * self.nx;
        self.init_cost(k, 0.0);

        self.nobs = self.costarr.iter().filter(|&&c| c >= COST_OBS).count() as i32;
    }

    /// `initCost`: potential `v` at `k`, its four neighbours into the
    /// current buffer.
    fn init_cost(&mut self, k: i32, v: f32) {
        if k >= 0 && (k as usize) < self.potarr.len() {
            self.potarr[k as usize] = v;
        }
        for n in [k + 1, k - 1, k - self.nx, k + self.nx] {
            push(&mut self.cur_p, &mut self.pending, &self.costarr, n);
        }
    }

    /// The potential at `i`, POT_HIGH outside the grid.
    fn pot(&self, i: i32) -> f32 {
        if i >= 0 && (i as usize) < self.potarr.len() {
            self.potarr[i as usize]
        } else {
            POT_HIGH
        }
    }

    /// The planar-wave potential of cell `n` from its two lowest neighbours
    /// (`updateCell` / `updateCellAstar` up to the push). `n` is an interior
    /// cell: only cells cheaper than COST_OBS are ever pushed, and the border
    /// is COST_OBS. Returns the new potential if it lowers the cell's, with
    /// the left / right / up / down potentials it was computed from.
    fn planar_wave(&self, n: usize) -> Option<(f32, [f32; 4])> {
        let nx = self.nx as usize;
        let l = self.potarr[n - 1];
        let r = self.potarr[n + 1];
        let u = self.potarr[n - nx];
        let d = self.potarr[n + nx];

        let tc = if l < r { l } else { r };
        let mut ta = if u < d { u } else { d };

        if self.costarr[n] >= COST_OBS {
            return None; // don't propagate into obstacles
        }
        let hf = self.costarr[n] as f32;
        let mut dc = tc - ta;
        if dc < 0.0 {
            dc = -dc;
            ta = tc;
        }
        let pot = if dc >= hf {
            ta + hf
        } else {
            // -0.2301 * div * div + 0.5307 * div + 0.7040 in double, then
            // pot = ta + hf * v in float; both fused as shipped
            let div = (dc / hf) as f64;
            let v = (div.mul_add(div * -0.2301, div * 0.5307) + 0.7040) as f32;
            hf.mul_add(v, ta)
        };
        (pot < self.potarr[n]).then_some((pot, [l, r, u, d]))
    }

    /// `INVSQRT2 * cost` of the four neighbours, in double then float.
    fn edge_costs(&self, n: usize) -> [f32; 4] {
        let nx = self.nx as usize;
        [n - 1, n + 1, n - nx, n + nx].map(|m| (INVSQRT2 * self.costarr[m] as f64) as f32)
    }

    /// Push each neighbour whose potential exceeds `pot` plus its edge cost:
    /// into the next buffer below the threshold, the overflow one above.
    fn push_neighbours(&mut self, n: i32, pot: f32, lrud: [f32; 4], edge: [f32; 4]) {
        let buf = if pot < self.cur_t { &mut self.next_p } else { &mut self.over_p };
        let nb = [n - 1, n + 1, n - self.nx, n + self.nx];
        for k in 0..4 {
            if lrud[k] > pot + edge[k] {
                push(buf, &mut self.pending, &self.costarr, nb[k]);
            }
        }
    }

    /// `updateCell`.
    fn update_cell(&mut self, n: i32) {
        let i = n as usize;
        if let Some((pot, lrud)) = self.planar_wave(i) {
            let edge = self.edge_costs(i);
            self.potarr[i] = pot;
            self.push_neighbours(n, pot, lrud, edge);
        }
    }

    /// `updateCellAstar`: the pushes compare against the potential plus the
    /// Euclidean distance to the start times COST_NEUTRAL.
    fn update_cell_astar(&mut self, n: i32) {
        let i = n as usize;
        if let Some((pot, lrud)) = self.planar_wave(i) {
            let edge = self.edge_costs(i);
            let x = n % self.nx;
            let y = n / self.nx;
            let dist = (hypot_int(x - self.start[0], y - self.start[1]) * COST_NEUTRAL as f64) as f32;
            self.potarr[i] = pot;
            self.push_neighbours(n, pot + dist, lrud, edge);
        }
    }

    /// One cycle: clear the pending flags of the current buffer, update its
    /// cells, make the next buffer current, and when that is empty raise the
    /// threshold and make the overflow buffer current.
    fn cycle(&mut self, astar: bool) {
        let mut cur = std::mem::take(&mut self.cur_p);
        for &n in &cur {
            self.pending[n as usize] = false;
        }
        for &n in &cur {
            if astar {
                self.update_cell_astar(n);
            } else {
                self.update_cell(n);
            }
        }
        cur.clear();
        self.cur_p = std::mem::replace(&mut self.next_p, cur);
        if self.cur_p.is_empty() {
            self.cur_t += self.pri_inc;
            std::mem::swap(&mut self.cur_p, &mut self.over_p);
        }
    }

    /// `propNavFnDijkstra`.
    fn prop_nav_fn_dijkstra(
        &mut self,
        cycles: i32,
        cancel: &mut dyn FnMut() -> bool,
        at_start: bool,
    ) -> Result<bool, Cancelled> {
        let mut nwv = 0;
        let mut nc = 0;
        let mut cycle = 0;
        let start_cell = self.start[1] * self.nx + self.start[0];

        while cycle < cycles {
            if cycle % TERMINAL_CHECKING_INTERVAL == 0 && cancel() {
                return Err(Cancelled);
            }
            if self.cur_p.is_empty() && self.next_p.is_empty() {
                break;
            }
            nc += self.cur_p.len() as i32;
            nwv = nwv.max(self.cur_p.len() as i32);
            self.cycle(false);
            if at_start && self.pot(start_cell) < POT_HIGH {
                break;
            }
            cycle += 1;
        }
        self.stats = PropagationStats { cycles: cycle, visited: nc, max_buffer: nwv };
        Ok(cycle < cycles)
    }

    /// `propNavFnAstar`: the threshold starts raised by the goal-start
    /// distance.
    fn prop_nav_fn_astar(&mut self, cycles: i32, cancel: &mut dyn FnMut() -> bool) -> Result<bool, Cancelled> {
        let mut nwv = 0;
        let mut nc = 0;
        let mut cycle = 0;

        let dist = (hypot_int(self.goal[0] - self.start[0], self.goal[1] - self.start[1]) * COST_NEUTRAL as f64) as f32;
        self.cur_t += dist;
        let start_cell = self.start[1] * self.nx + self.start[0];

        while cycle < cycles {
            if cycle % TERMINAL_CHECKING_INTERVAL == 0 && cancel() {
                return Err(Cancelled);
            }
            if self.cur_p.is_empty() && self.next_p.is_empty() {
                break;
            }
            nc += self.cur_p.len() as i32;
            nwv = nwv.max(self.cur_p.len() as i32);
            self.cycle(true);
            if self.pot(start_cell) < POT_HIGH {
                break;
            }
            cycle += 1;
        }
        self.last_path_cost = self.pot(start_cell);
        self.stats = PropagationStats { cycles: cycle, visited: nc, max_buffer: nwv };
        Ok(self.pot(start_cell) < POT_HIGH)
    }

    /// `calcPath(n, st)`: at most `n` steps from `st` (default: the start
    /// cell) down the potential; returns the number of points (the last is
    /// the goal cell itself) or 0 when it fails: off the grid, an unreached
    /// cell, a zero gradient, or out of steps.
    pub fn calc_path(&mut self, n: i32, st: Option<[i32; 2]>) -> i32 {
        let st = st.unwrap_or(self.start);
        let nx = self.nx;
        let ns = self.ns;
        let mut stc = st[1] * nx + st[0];
        let mut dx = 0.0f32;
        let mut dy = 0.0f32;
        self.pathx.clear();
        self.pathy.clear();

        for _ in 0..n {
            // check if near goal (round() on float: ties away, nx * round(dy) a float product)
            let nearest = (stc + dx.round() as i32 + (nx as f32 * dy.round()) as i32).min(nx * self.ny - 1).max(0);
            if self.pot(nearest) < COST_NEUTRAL as f32 {
                self.pathx.push(self.goal[0] as f32);
                self.pathy.push(self.goal[1] as f32);
                return self.path_len();
            }

            if stc < nx || stc > ns - nx {
                return 0; // would be out of bounds
            }

            self.pathx.push((stc % nx) as f32 + dx);
            self.pathy.push((stc / nx) as f32 + dy);
            let np = self.pathx.len();
            let oscillation =
                np > 2 && self.pathx[np - 1] == self.pathx[np - 3] && self.pathy[np - 1] == self.pathy[np - 3];

            let stcnx = stc + nx;
            let stcpx = stc - nx;
            let around = [stc, stc + 1, stc - 1, stcnx, stcnx + 1, stcnx - 1, stcpx, stcpx + 1, stcpx - 1];
            if around.iter().any(|&i| self.pot(i) >= POT_HIGH) || oscillation {
                // follow the grid: the lowest of the eight neighbours, compared
                // against the running minimum truncated to int
                let mut minc = stc;
                let mut minp = self.pot(stc) as i32;
                for sti in [stcpx - 1, stcpx, stcpx + 1, stc - 1, stc + 1, stcnx - 1, stcnx, stcnx + 1] {
                    if self.pot(sti) < minp as f32 {
                        minp = self.pot(sti) as i32;
                        minc = sti;
                    }
                }
                stc = minc;
                dx = 0.0;
                dy = 0.0;
                if self.pot(stc) >= POT_HIGH {
                    return 0; // no path found, high potential
                }
            } else {
                self.grad_cell(stc);
                self.grad_cell(stc + 1);
                self.grad_cell(stcnx);
                self.grad_cell(stcnx + 1);

                // (1.0 - d) * g0 + d * g1: a double product fused with the
                // float product d * g1 widened, as shipped
                let (s, snx) = (stc as usize, stcnx as usize);
                let omdx = 1.0 - dx as f64;
                let omdy = 1.0 - dy as f64;
                let gx = &self.gradx;
                let gy = &self.grady;
                let x1 = omdx.mul_add(gx[s] as f64, (dx * gx[s + 1]) as f64) as f32;
                let x2 = omdx.mul_add(gx[snx] as f64, (dx * gx[snx + 1]) as f64) as f32;
                let x = omdy.mul_add(x1 as f64, (dy * x2) as f64) as f32;
                let y1 = omdx.mul_add(gy[s] as f64, (dx * gy[s + 1]) as f64) as f32;
                let y2 = omdx.mul_add(gy[snx] as f64, (dx * gy[snx + 1]) as f64) as f32;
                let y = omdy.mul_add(y1 as f64, (dy * y2) as f64) as f32;

                if x == 0.0 && y == 0.0 {
                    return 0; // zero gradient
                }

                let ss = self.path_step / hypotf(x, y);
                dx = x.mul_add(ss, dx);
                dy = y.mul_add(ss, dy);

                if dx > 1.0 {
                    stc += 1;
                    dx -= 1.0;
                }
                if dx < -1.0 {
                    stc -= 1;
                    dx += 1.0;
                }
                if dy > 1.0 {
                    stc += nx;
                    dy -= 1.0;
                }
                if dy < -1.0 {
                    stc -= nx;
                    dy += 1.0;
                }
            }
        }
        0 // out of cycles
    }

    /// `gradCell`: the normalised potential gradient at `n` (positive right
    /// and down), cached in `gradx` / `grady`; returns the inverse norm, 1 for
    /// a cached cell, 0 on the top or bottom row. An unreached cell points at
    /// its reached neighbours with magnitude COST_OBS.
    fn grad_cell(&mut self, n: i32) -> f32 {
        let i = n as usize;
        if self.gradx[i] + self.grady[i] > 0.0 {
            return 1.0;
        }
        if n < self.nx || n > self.ns - self.nx {
            return 0.0;
        }

        let nx = self.nx;
        let cv = self.pot(n);
        let mut dx = 0.0f32;
        let mut dy = 0.0f32;
        if cv >= POT_HIGH {
            if self.pot(n - 1) < POT_HIGH {
                dx = -(COST_OBS as f32);
            } else if self.pot(n + 1) < POT_HIGH {
                dx = COST_OBS as f32;
            }
            if self.pot(n - nx) < POT_HIGH {
                dy = -(COST_OBS as f32);
            } else if self.pot(n + nx) < POT_HIGH {
                dy = COST_OBS as f32;
            }
        } else {
            if self.pot(n - 1) < POT_HIGH {
                dx += self.pot(n - 1) - cv;
            }
            if self.pot(n + 1) < POT_HIGH {
                dx += cv - self.pot(n + 1);
            }
            if self.pot(n - nx) < POT_HIGH {
                dy += self.pot(n - nx) - cv;
            }
            if self.pot(n + nx) < POT_HIGH {
                dy += cv - self.pot(n + nx);
            }
        }

        let mut norm = hypotf(dx, dy);
        if norm > 0.0 {
            // (float)(1.0 / (double)norm) == 1.0f / norm (innocuous double rounding)
            norm = 1.0 / norm;
            self.gradx[i] = norm * dx;
            self.grady[i] = norm * dy;
        }
        norm
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn never() -> impl FnMut() -> bool {
        || false
    }

    #[test]
    fn cost_translation() {
        let mut nav = NavFn::new(4, 1);
        nav.set_costmap(&[0, 252, 253, 255], true, true);
        assert_eq!(nav.costarr(), &[50, 251, 254, 253]);
        nav.set_costmap(&[0, 1, 254, 255], true, false);
        assert_eq!(nav.costarr(), &[50, 50, 254, 254]);
    }

    #[test]
    fn open_grid_reaches_start_and_descends() {
        let (nx, ny) = (20, 15);
        let mut nav = NavFn::new(nx, ny);
        nav.set_costmap(&vec![0u8; (nx * ny) as usize], true, true);
        nav.set_goal([3, 4]);
        nav.set_start([15, 10]);
        assert_eq!(nav.calc_nav_fn_dijkstra(&mut never(), true), Ok(true));
        assert!(nav.potarr()[(10 * nx + 15) as usize] < POT_HIGH);
        let len = nav.calc_path(4 * nx, None);
        assert!(len > 0);
        assert_eq!(nav.path_x()[len as usize - 1], 3.0);
        assert_eq!(nav.path_y()[len as usize - 1], 4.0);
        // the first point is the start cell itself
        assert_eq!((nav.path_x()[0], nav.path_y()[0]), (15.0, 10.0));
    }

    #[test]
    fn walled_off_goal_is_unreached() {
        let (nx, ny) = (12, 12);
        let mut cmap = vec![0u8; (nx * ny) as usize];
        for y in 0..ny {
            cmap[(y * nx + 6) as usize] = 254;
        }
        let mut nav = NavFn::new(nx, ny);
        nav.set_costmap(&cmap, true, true);
        nav.set_goal([2, 5]);
        nav.set_start([9, 5]);
        nav.calc_nav_fn_dijkstra(&mut never(), true).unwrap();
        assert_eq!(nav.potarr()[(5 * nx + 9) as usize], POT_HIGH);
        assert_eq!(nav.calc_path(48, None), 0);
    }

    #[test]
    fn cancel_is_checked_on_the_first_cycle() {
        let mut nav = NavFn::new(10, 10);
        nav.set_costmap(&[0u8; 100], true, true);
        nav.set_goal([2, 2]);
        nav.set_start([7, 7]);
        assert_eq!(nav.calc_nav_fn_dijkstra(&mut || true, true), Err(Cancelled));
    }

    #[test]
    fn hypot_helpers() {
        assert_eq!(hypotf(3.0, 4.0), 5.0);
        assert_eq!(hypot_int(-5, 12), 13.0);
    }
}
