//! ROS-free geometry helpers (port of `utils/path_record_utils.py` and the
//! polygon helpers of `path_record_node.py`).

pub type Xy = (f64, f64);

/// Douglas-Peucker: indices of the points to keep.
pub fn douglas_peucker(points: &[Xy], epsilon: f64) -> Vec<usize> {
    if points.len() < 3 {
        return (0..points.len()).collect();
    }
    let (start, end) = (points[0], points[points.len() - 1]);
    let mut max_dist = 0.0;
    let mut max_index = 0;
    for (i, p) in points.iter().enumerate().take(points.len() - 1).skip(1) {
        let d = point_to_line_distance(*p, start, end);
        if d > max_dist {
            max_dist = d;
            max_index = i;
        }
    }
    if max_dist > epsilon {
        let left = douglas_peucker(&points[..=max_index], epsilon);
        let right = douglas_peucker(&points[max_index..], epsilon);
        let mut result = left;
        result.extend(right.iter().skip(1).map(|i| max_index + i));
        result
    } else {
        vec![0, points.len() - 1]
    }
}

/// Distance from `point` to the infinite line through `a` and `b` (the
/// segment length when a == b), as in the Python original.
pub fn point_to_line_distance(point: Xy, a: Xy, b: Xy) -> f64 {
    let (x0, y0) = point;
    let (x1, y1) = a;
    let (x2, y2) = b;
    let len_sq = (x2 - x1).powi(2) + (y2 - y1).powi(2);
    if len_sq == 0.0 {
        return ((x0 - x1).powi(2) + (y0 - y1).powi(2)).sqrt();
    }
    ((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1).abs() / len_sq.sqrt()
}

/// Keep only the Douglas-Peucker vertices of a recorded trace.
pub fn simplify(points: &[Xy], epsilon: f64) -> Vec<Xy> {
    if points.len() < 3 {
        return points.to_vec();
    }
    douglas_peucker(points, epsilon).into_iter().map(|i| points[i]).collect()
}

/// Ray-casting point-in-polygon test.
pub fn point_in_polygon(x: f64, y: f64, polygon: &[Xy]) -> bool {
    let n = polygon.len();
    if n < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = polygon[i];
        let (xj, yj) = polygon[j];
        if (yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Minimum distance from a point to the polygon's edges.
pub fn min_dist_to_polygon(x: f64, y: f64, polygon: &[Xy]) -> f64 {
    let n = polygon.len();
    let mut min = f64::INFINITY;
    for i in 0..n {
        let (x1, y1) = polygon[i];
        let (x2, y2) = polygon[(i + 1) % n];
        let (dx, dy) = (x2 - x1, y2 - y1);
        let seg_len_sq = dx * dx + dy * dy;
        let d = if seg_len_sq == 0.0 {
            ((x - x1).powi(2) + (y - y1).powi(2)).sqrt()
        } else {
            let t = (((x - x1) * dx + (y - y1) * dy) / seg_len_sq).clamp(0.0, 1.0);
            ((x - (x1 + t * dx)).powi(2) + (y - (y1 + t * dy)).powi(2)).sqrt()
        };
        if d < min {
            min = d;
        }
    }
    min
}

/// Inside the zone or within `proximity_m` of its boundary.
pub fn point_near_polygon(x: f64, y: f64, polygon: &[Xy], proximity_m: f64) -> bool {
    point_in_polygon(x, y, polygon) || min_dist_to_polygon(x, y, polygon) <= proximity_m
}

/// Shoelace area of an XY polygon (open or closed vertex list).
pub fn polygon_area_m2(pts: &[Xy]) -> f64 {
    let n = pts.len();
    if n < 3 {
        return 0.0;
    }
    let mut area = 0.0;
    for i in 0..n {
        let (x1, y1) = pts[i];
        let (x2, y2) = pts[(i + 1) % n];
        area += x1 * y2 - x2 * y1;
    }
    area.abs() / 2.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn douglas_peucker_keeps_corners_only() {
        let pts: Vec<Xy> = (0..=10).map(|i| (i as f64, 0.0)).chain((1..=5).map(|i| (10.0, i as f64))).collect();
        let kept = simplify(&pts, 0.1);
        assert_eq!(kept, vec![(0.0, 0.0), (10.0, 0.0), (10.0, 5.0)]);
        // a noisy point beyond epsilon survives
        let mut noisy = pts.clone();
        noisy[5] = (5.0, 0.5);
        assert!(simplify(&noisy, 0.1).contains(&(5.0, 0.5)));
        assert_eq!(simplify(&pts[..2], 0.1).len(), 2);
    }

    #[test]
    fn polygon_tests_match_the_python_helpers() {
        let square = vec![(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)];
        assert!(point_in_polygon(2.0, 2.0, &square));
        assert!(!point_in_polygon(5.0, 2.0, &square));
        assert!((min_dist_to_polygon(5.0, 2.0, &square) - 1.0).abs() < 1e-12);
        assert!((min_dist_to_polygon(6.0, 6.0, &square) - 8f64.sqrt()).abs() < 1e-12);
        assert!(point_near_polygon(4.9, 2.0, &square, 1.0));
        assert!(!point_near_polygon(5.1, 2.0, &square, 1.0));
        assert!((polygon_area_m2(&square) - 16.0).abs() < 1e-12);
        assert_eq!(polygon_area_m2(&square[..2]), 0.0);
        assert!(!point_in_polygon(1.0, 1.0, &square[..2]));
    }
}
