//! The robot footprint and `nav2_costmap_2d::FootprintCollisionChecker`.
//!
//! The costmaps here set `robot_radius: 0.5` and no `footprint`, so
//! `Costmap2DROS` uses the 16-point circle of `makeFootprintFromRadius`,
//! padded by `footprint_padding` (default 0.01, declared as a *float*) with
//! `padFootprint`'s per-axis sign rule. That padded polygon is what
//! `getRobotFootprint()` hands RPP's collision checker, and what
//! `LayeredCostmap::setFootprint` derives the inscribed / circumscribed radius
//! from.
//!
//! Upstream: `src/footprint.cpp`, `src/costmap_math.cpp`,
//! `src/footprint_collision_checker.cpp`, `nav2_util/line_iterator.hpp`.

use std::f64::consts::PI;

use crate::costmap::{Costmap2D, LETHAL_OBSTACLE};

/// Polygon vertices in the robot frame.
pub type Footprint = Vec<(f64, f64)>;

/// `Costmap2DROS`'s `footprint_padding` default, a `float` parameter.
pub const DEFAULT_FOOTPRINT_PADDING: f32 = 0.01;

/// `makeFootprintFromRadius`: 16 points around the circle.
pub fn make_footprint_from_radius(radius: f64) -> Footprint {
    let n = 16;
    (0..n)
        .map(|i| {
            let angle = i as f64 * 2.0 * PI / n as f64;
            (angle.cos() * radius, angle.sin() * radius)
        })
        .collect()
}

fn sign0(x: f64) -> f64 {
    if x < 0.0 {
        -1.0
    } else if x > 0.0 {
        1.0
    } else {
        0.0
    }
}

/// `padFootprint`: each coordinate moves outwards by `padding` along its own
/// sign, so cos(pi / 2) * r = 3e-17 still gets the full padding.
pub fn pad_footprint(footprint: &mut Footprint, padding: f64) {
    for p in footprint.iter_mut() {
        p.0 += sign0(p.0) * padding;
        p.1 += sign0(p.1) * padding;
    }
}

/// What `Costmap2DROS::getRobotFootprint()` returns for a `robot_radius`.
pub fn robot_footprint(radius: f64) -> Footprint {
    let mut fp = make_footprint_from_radius(radius);
    pad_footprint(&mut fp, DEFAULT_FOOTPRINT_PADDING as f64);
    fp
}

/// `distanceToLine` (costmap_math.cpp).
pub fn distance_to_line(px: f64, py: f64, x0: f64, y0: f64, x1: f64, y1: f64) -> f64 {
    let a = px - x0;
    let b = py - y0;
    let c = x1 - x0;
    let d = y1 - y0;
    let dot = a * c + b * d;
    let len_sq = c * c + d * d;
    let param = dot / len_sq;
    let (xx, yy) = if param < 0.0 {
        (x0, y0)
    } else if param > 1.0 {
        (x1, y1)
    } else {
        (x0 + param * c, y0 + param * d)
    };
    distance(px, py, xx, yy)
}

/// `distance` (costmap_math.hpp): `hypot`.
fn distance(x0: f64, y0: f64, x1: f64, y1: f64) -> f64 {
    (x1 - x0).hypot(y1 - y0)
}

/// `calculateMinAndMaxDistances`: (inscribed, circumscribed) radius.
pub fn calculate_min_and_max_distances(footprint: &[(f64, f64)]) -> (f64, f64) {
    let mut min_dist = f64::MAX;
    let mut max_dist = 0.0f64;
    if footprint.len() <= 2 {
        return (min_dist, max_dist);
    }
    for i in 0..footprint.len() - 1 {
        let vertex_dist = distance(0.0, 0.0, footprint[i].0, footprint[i].1);
        let edge_dist = distance_to_line(0.0, 0.0, footprint[i].0, footprint[i].1, footprint[i + 1].0, footprint[i + 1].1);
        min_dist = min_dist.min(vertex_dist.min(edge_dist));
        max_dist = max_dist.max(vertex_dist.max(edge_dist));
    }
    let last = footprint[footprint.len() - 1];
    let first = footprint[0];
    let vertex_dist = distance(0.0, 0.0, last.0, last.1);
    let edge_dist = distance_to_line(0.0, 0.0, last.0, last.1, first.0, first.1);
    min_dist = min_dist.min(vertex_dist.min(edge_dist));
    max_dist = max_dist.max(vertex_dist.max(edge_dist));
    (min_dist, max_dist)
}

/// `nav2_util::LineIterator`: Bresenham from (x0, y0) to (x1, y1), both ends
/// included.
#[derive(Debug, Clone)]
pub struct LineIterator {
    x: i32,
    y: i32,
    curpixel: i32,
    xinc1: i32,
    xinc2: i32,
    yinc1: i32,
    yinc2: i32,
    den: i32,
    num: i32,
    numadd: i32,
    numpixels: i32,
}

impl LineIterator {
    pub fn new(x0: i32, y0: i32, x1: i32, y1: i32) -> Self {
        let deltax = (x1 - x0).abs();
        let deltay = (y1 - y0).abs();
        let (mut xinc1, mut xinc2) = if x1 >= x0 { (1, 1) } else { (-1, -1) };
        let (mut yinc1, mut yinc2) = if y1 >= y0 { (1, 1) } else { (-1, -1) };
        let (den, num, numadd, numpixels);
        if deltax >= deltay {
            xinc1 = 0;
            yinc2 = 0;
            den = deltax;
            num = deltax / 2;
            numadd = deltay;
            numpixels = deltax;
        } else {
            xinc2 = 0;
            yinc1 = 0;
            den = deltay;
            num = deltay / 2;
            numadd = deltax;
            numpixels = deltay;
        }
        Self { x: x0, y: y0, curpixel: 0, xinc1, xinc2, yinc1, yinc2, den, num, numadd, numpixels }
    }

    pub fn is_valid(&self) -> bool {
        self.curpixel <= self.numpixels
    }

    pub fn advance(&mut self) {
        self.num += self.numadd;
        if self.num >= self.den {
            self.num -= self.den;
            self.x += self.xinc1;
            self.y += self.yinc1;
        }
        self.x += self.xinc2;
        self.y += self.yinc2;
        self.curpixel += 1;
    }

    pub fn x(&self) -> i32 {
        self.x
    }

    pub fn y(&self) -> i32 {
        self.y
    }
}

impl Iterator for LineIterator {
    type Item = (i32, i32);

    fn next(&mut self) -> Option<(i32, i32)> {
        if !self.is_valid() {
            return None;
        }
        let p = (self.x, self.y);
        self.advance();
        Some(p)
    }
}

/// `FootprintCollisionChecker<Costmap2D *>`.
pub struct FootprintCollisionChecker<'a> {
    costmap: &'a Costmap2D,
}

impl<'a> FootprintCollisionChecker<'a> {
    pub fn new(costmap: &'a Costmap2D) -> Self {
        Self { costmap }
    }

    /// `pointCost`: `getCost(x, y)` (the line cells come from in-bounds ends).
    pub fn point_cost(&self, x: i32, y: i32) -> f64 {
        self.costmap.cost(x as u32, y as u32) as f64
    }

    /// `lineCost`: the highest cell cost along the line, stopping at lethal.
    pub fn line_cost(&self, x0: i32, x1: i32, y0: i32, y1: i32) -> f64 {
        let mut line_cost = 0.0f64;
        for (x, y) in LineIterator::new(x0, y0, x1, y1) {
            let point_cost = self.point_cost(x, y);
            if point_cost == LETHAL_OBSTACLE as f64 {
                return point_cost;
            }
            if line_cost < point_cost {
                line_cost = point_cost;
            }
        }
        line_cost
    }

    /// `footprintCost`: the outline only (the interior is not scanned);
    /// any vertex off the map is lethal.
    pub fn footprint_cost(&self, footprint: &[(f64, f64)]) -> f64 {
        let Some((mut x0, mut y0)) = self.costmap.world_to_map(footprint[0].0, footprint[0].1) else {
            return LETHAL_OBSTACLE as f64;
        };
        let (xstart, ystart) = (x0, y0);
        let mut footprint_cost = 0.0f64;
        let (mut x1, mut y1) = (x0, y0);
        for p in &footprint[1..] {
            let Some((nx, ny)) = self.costmap.world_to_map(p.0, p.1) else {
                return LETHAL_OBSTACLE as f64;
            };
            x1 = nx;
            y1 = ny;
            footprint_cost = self.line_cost(x0 as i32, x1 as i32, y0 as i32, y1 as i32).max(footprint_cost);
            x0 = x1;
            y0 = y1;
            if footprint_cost == LETHAL_OBSTACLE as f64 {
                return footprint_cost;
            }
        }
        self.line_cost(xstart as i32, x1 as i32, ystart as i32, y1 as i32).max(footprint_cost)
    }

    /// `footprintCostAtPose`: the footprint rotated by `theta` and moved to
    /// (x, y).
    pub fn footprint_cost_at_pose(&self, x: f64, y: f64, theta: f64, footprint: &[(f64, f64)]) -> f64 {
        let cos_th = theta.cos();
        let sin_th = theta.sin();
        let oriented: Footprint =
            footprint.iter().map(|p| (x + (p.0 * cos_th - p.1 * sin_th), y + (p.0 * sin_th + p.1 * cos_th))).collect();
        self.footprint_cost(&oriented)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn padded_circle_radii() {
        let fp = robot_footprint(0.5);
        assert_eq!(fp.len(), 16);
        // the point at 90 deg has x = 3e-17 > 0 and is padded outwards too
        assert!(fp[4].0 > 0.009 && fp[4].1 > 0.5);
        let (inscribed, circumscribed) = calculate_min_and_max_distances(&fp);
        assert!(inscribed > 0.49 && inscribed < 0.51, "{inscribed}");
        assert!(circumscribed > 0.51 && circumscribed < 0.52, "{circumscribed}");
    }

    #[test]
    fn line_iterator_includes_both_ends() {
        let cells: Vec<_> = LineIterator::new(0, 0, 3, 1).collect();
        assert_eq!(cells.first(), Some(&(0, 0)));
        assert_eq!(cells.last(), Some(&(3, 1)));
        assert_eq!(cells.len(), 4);
        assert_eq!(LineIterator::new(2, 2, 2, 2).count(), 1);
    }

    #[test]
    fn footprint_off_the_map_is_lethal() {
        let c = Costmap2D::new(10, 10, 0.1, 0.0, 0.0, 0);
        let checker = FootprintCollisionChecker::new(&c);
        let fp = robot_footprint(0.2);
        assert_eq!(checker.footprint_cost_at_pose(0.5, 0.5, 0.0, &fp), 0.0);
        assert_eq!(checker.footprint_cost_at_pose(0.1, 0.5, 0.0, &fp), LETHAL_OBSTACLE as f64);
    }
}
