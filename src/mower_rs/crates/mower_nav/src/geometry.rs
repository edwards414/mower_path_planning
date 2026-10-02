//! ROS-free helpers of `nav_action_server.py`: dispatch-id canonicalisation,
//! path validation, coverage-path splitting and the covariance gates.

use r2r::geometry_msgs::msg::{Pose, PoseStamped};
use r2r::nav_msgs::msg::Path;

pub fn path_frame_id(path: &Path) -> &str {
    if path.header.frame_id.is_empty() {
        "map"
    } else {
        &path.header.frame_id
    }
}

/// One canonical UUID token (32 lower-case hex digits) or None.
pub fn canonical_dispatch_id(value: &str) -> Option<String> {
    uuid::Uuid::parse_str(value.trim()).ok().map(|u| u.simple().to_string())
}

fn quaternion_norm(p: &Pose) -> f64 {
    (p.orientation.x.powi(2) + p.orientation.y.powi(2) + p.orientation.z.powi(2) + p.orientation.w.powi(2)).sqrt()
}

/// Reject paths that could produce fake success or undefined Nav2 math.
pub fn navigation_path_block_reason(path: &Path) -> Option<String> {
    if path.poses.len() < 2 {
        return Some("navigation path requires at least two poses".into());
    }
    let path_frame = path_frame_id(path);
    if path_frame != "map" {
        return Some(format!("navigation path frame must be map, got {path_frame}"));
    }
    for (index, stamped) in path.poses.iter().enumerate() {
        let pose_frame = if stamped.header.frame_id.is_empty() { path_frame } else { &stamped.header.frame_id };
        if pose_frame != path_frame {
            return Some(format!("navigation pose {index} frame does not match path frame"));
        }
        let p = &stamped.pose;
        let values = [p.position.x, p.position.y, p.position.z, p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w];
        if !values.iter().all(|v| v.is_finite()) {
            return Some(format!("navigation pose {index} contains a non-finite value"));
        }
        if (quaternion_norm(p) - 1.0).abs() > 1e-3 {
            return Some(format!("navigation pose {index} has an invalid quaternion"));
        }
    }
    None
}

fn new_path_like(path: &Path) -> Path {
    let mut p = Path::default();
    p.header.frame_id = path_frame_id(path).to_string();
    p.header.stamp = path.header.stamp.clone();
    p
}

fn pose_distance(a: &PoseStamped, b: &PoseStamped) -> f64 {
    ((b.pose.position.x - a.pose.position.x).powi(2) + (b.pose.position.y - a.pose.position.y).powi(2)).sqrt()
}

/// Cumulative 2D path distance in metres.
pub fn path_distance(path: &Path) -> f64 {
    path.poses.windows(2).map(|w| pose_distance(&w[0], &w[1])).sum()
}

/// Reject segments that Nav2 can accept without producing any motion.
pub fn coverage_segments_block_reason(split_paths: &[Path], minimum_distance_m: f64) -> Option<String> {
    if split_paths.is_empty() {
        return Some("coverage split produced no executable segments".into());
    }
    for (i, p) in split_paths.iter().enumerate() {
        let index = i + 1;
        if p.poses.len() < 2 {
            return Some(format!("coverage segment {index} has fewer than two poses"));
        }
        let distance = path_distance(p);
        if !distance.is_finite() {
            return Some(format!("coverage segment {index} has a non-finite distance"));
        }
        if distance <= minimum_distance_m {
            return Some(format!(
                "coverage segment {index} is only {distance:.3} m; planner split points must be coalesced above the 0.05 m goal tolerance plus one 0.10 m mission-map cell"
            ));
        }
    }
    None
}

/// Reject malformed or overly uncertain 3x3 IMU covariance matrices.
pub fn imu_covariance_block_reason(covariance: &[f64], max_sigma: f64, label: &str) -> Option<String> {
    let invalid = || Some(format!("IMU {label} covariance is unavailable or invalid"));
    if covariance.len() < 9 {
        return invalid();
    }
    let v = &covariance[..9];
    if !v.iter().all(|x| x.is_finite()) {
        return invalid();
    }
    let m = |r: usize, c: usize| v[r * 3 + c];
    let entry_scale = v.iter().map(|x| x.abs()).fold(1e-12, f64::max);
    let symmetry_tolerance = 1e-6 * entry_scale;
    for r in 0..3 {
        for c in (r + 1)..3 {
            if (m(r, c) - m(c, r)).abs() > symmetry_tolerance {
                return invalid();
            }
        }
    }
    let max_variance = max_sigma.max(0.0).powi(2);
    let diagonal = [m(0, 0), m(1, 1), m(2, 2)];
    if diagonal.iter().any(|d| *d <= 0.0) {
        return invalid();
    }
    let minor_epsilon = 1e-9 * entry_scale.powi(2);
    let determinant_epsilon = 1e-9 * entry_scale.powi(3);
    let minors = [
        diagonal[0] * diagonal[1] - m(0, 1).powi(2),
        diagonal[0] * diagonal[2] - m(0, 2).powi(2),
        diagonal[1] * diagonal[2] - m(1, 2).powi(2),
    ];
    let determinant = m(0, 0) * (m(1, 1) * m(2, 2) - m(1, 2) * m(2, 1)) - m(0, 1) * (m(1, 0) * m(2, 2) - m(1, 2) * m(2, 0))
        + m(0, 2) * (m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0));
    if minors.iter().any(|x| *x < -minor_epsilon) || determinant < -determinant_epsilon {
        return invalid();
    }
    let eigenvalue_upper_bound = (0..3)
        .map(|r| m(r, r) + (0..3).filter(|c| *c != r).map(|c| m(r, c).abs()).sum::<f64>())
        .fold(f64::NEG_INFINITY, f64::max);
    if eigenvalue_upper_bound > max_variance {
        return Some(format!("IMU {label} uncertainty exceeds {:.3}", max_sigma.max(0.0)));
    }
    None
}

/// Largest / smallest eigenvalue of the horizontal 2x2 covariance block;
/// `None` when it is not a valid symmetric block.
pub fn horizontal_eigen(var_x: f64, var_y: f64, cov_xy: f64, cov_yx: f64) -> Option<(f64, f64)> {
    if ![var_x, var_y, cov_xy, cov_yx].iter().all(|v| v.is_finite()) || var_x < 0.0 || var_y < 0.0 || (cov_xy - cov_yx).abs() > 1e-6 {
        return None;
    }
    let cross = 0.5 * (cov_xy + cov_yx);
    let discriminant = ((var_x - var_y).powi(2) + 4.0 * cross.powi(2)).max(0.0).sqrt();
    Some((0.5 * (var_x + var_y + discriminant), 0.5 * (var_x + var_y - discriminant)))
}

fn angle_diff(a: f64, b: f64) -> f64 {
    ((a - b).sin().atan2((a - b).cos())).abs()
}

fn segment_heading(a: &PoseStamped, b: &PoseStamped) -> f64 {
    (b.pose.position.y - a.pose.position.y).atan2(b.pose.position.x - a.pose.position.x)
}

/// Insert poses so that consecutive poses are at most `max_step_m` apart
/// (inserted poses face along their stretch; the given poses are kept).
///
/// Nav2's controller trims the plan to the poses near the robot inside its
/// local costmap. The planner's lane-to-lane connectors arrive as 2-3 poses
/// a metre or more apart, and one that leads back past the mower (a long
/// lane joined to a shorter one) was trimmed to nothing: "Resulting plan has
/// 0 poses in it" and the mission failed. Lanes, at ~0.2 m spacing, never did.
pub fn densify_path(path: &Path, max_step_m: f64) -> Path {
    if !(max_step_m > 0.0) || path.poses.len() < 2 {
        return path.clone();
    }
    let mut out = new_path_like(path);
    out.poses.push(path.poses[0].clone());
    for w in path.poses.windows(2) {
        let (a, b) = (&w[0], &w[1]);
        let steps = (pose_distance(a, b) / max_step_m).ceil() as usize;
        if steps > 1 {
            let yaw = segment_heading(a, b);
            for k in 1..steps {
                let t = k as f64 / steps as f64;
                let mut p = b.clone();
                p.pose.position.x = a.pose.position.x + (b.pose.position.x - a.pose.position.x) * t;
                p.pose.position.y = a.pose.position.y + (b.pose.position.y - a.pose.position.y) * t;
                p.pose.position.z = a.pose.position.z + (b.pose.position.z - a.pose.position.z) * t;
                p.pose.orientation.x = 0.0;
                p.pose.orientation.y = 0.0;
                p.pose.orientation.z = (yaw / 2.0).sin();
                p.pose.orientation.w = (yaw / 2.0).cos();
                out.poses.push(p);
            }
        }
        out.poses.push(b.clone());
    }
    out
}

/// Split a coverage path at the planner's split points, always preserving
/// the final segment. The coverage planner mirrors this, the turn split and
/// the 0.15 m refusal in mower_coverage_core/src/nav_split.rs so it never
/// sends a path this server refuses: keep the two in step.
pub fn split_path_by_coverage_points(path: &Path, split_points: &[Pose], tolerance_m: f64) -> Vec<Path> {
    if path.poses.len() < 2 {
        return Vec::new();
    }
    let is_split = |x: f64, y: f64| split_points.iter().any(|p| ((x - p.position.x).powi(2) + (y - p.position.y).powi(2)).sqrt() <= tolerance_m);
    let mut segments = Vec::new();
    let mut current = new_path_like(path);
    let n = path.poses.len();
    for (idx, pose) in path.poses.iter().enumerate() {
        current.poses.push(pose.clone());
        let internal = idx > 0 && idx < n - 1;
        if internal && is_split(pose.pose.position.x, pose.pose.position.y) {
            if current.poses.len() > 1 {
                segments.push(current);
            }
            current = new_path_like(path);
            current.poses.push(pose.clone());
        }
    }
    if current.poses.len() > 1 {
        segments.push(current);
    }
    segments
}

/// Split a path at sharp turns so the controller can re-align heading.
pub fn split_path_by_turn_angle(path: &Path, turn_angle_rad: f64, min_segment_length_m: f64) -> Vec<Path> {
    if path.poses.len() < 3 || turn_angle_rad <= 0.0 {
        return if path.poses.len() >= 2 { vec![path.clone()] } else { Vec::new() };
    }
    let mut segments = Vec::new();
    let mut current = new_path_like(path);
    current.poses.push(path.poses[0].clone());
    let mut current_distance = 0.0;
    let n = path.poses.len();
    for idx in 1..n - 1 {
        let (prev, pose, next) = (&path.poses[idx - 1], &path.poses[idx], &path.poses[idx + 1]);
        current_distance += pose_distance(prev, pose);
        current.poses.push(pose.clone());
        let turn = angle_diff(segment_heading(prev, pose), segment_heading(pose, next));
        if turn >= turn_angle_rad && current_distance >= min_segment_length_m {
            if current.poses.len() > 1 {
                segments.push(current);
            }
            current = new_path_like(path);
            current.poses.push(pose.clone());
            current_distance = 0.0;
        }
    }
    current.poses.push(path.poses[n - 1].clone());
    if current.poses.len() > 1 {
        segments.push(current);
    }
    segments
}

/// Split a path into shorter consecutive chunks (0 disables).
pub fn split_path_by_max_distance(path: &Path, max_distance_m: f64) -> Vec<Path> {
    if path.poses.len() < 2 {
        return Vec::new();
    }
    if max_distance_m <= 0.0 {
        return vec![path.clone()];
    }
    let mut segments = Vec::new();
    let mut current = new_path_like(path);
    current.poses.push(path.poses[0].clone());
    let mut current_distance = 0.0;
    let n = path.poses.len();
    for idx in 1..n {
        current_distance += pose_distance(&path.poses[idx - 1], &path.poses[idx]);
        current.poses.push(path.poses[idx].clone());
        if current_distance >= max_distance_m && idx != n - 1 {
            if current.poses.len() > 1 {
                segments.push(current);
            }
            current = new_path_like(path);
            current.poses.push(path.poses[idx].clone());
            current_distance = 0.0;
        }
    }
    if current.poses.len() > 1 {
        segments.push(current);
    }
    segments
}

pub fn split_paths_by_turn_angle(paths: &[Path], turn_angle_rad: f64, min_segment_length_m: f64) -> Vec<Path> {
    paths.iter().flat_map(|p| split_path_by_turn_angle(p, turn_angle_rad, min_segment_length_m)).collect()
}

pub fn split_paths_by_max_distance(paths: &[Path], max_distance_m: f64) -> Vec<Path> {
    paths.iter().flat_map(|p| split_path_by_max_distance(p, max_distance_m)).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn densify_fills_sparse_connectors_and_keeps_the_given_poses() {
        let sparse = path(&[(6.519, 11.284), (7.171, 8.871)]);
        let dense = densify_path(&sparse, 0.1);
        let n = dense.poses.len();
        assert_eq!(n, 26, "2.50 m at <= 0.1 m: 25 steps, 24 inserted poses + the two ends");
        assert_eq!(dense.poses[0].pose.position, sparse.poses[0].pose.position);
        assert_eq!(dense.poses[n - 1].pose.position, sparse.poses[1].pose.position);
        for w in dense.poses.windows(2) {
            assert!(pose_distance(&w[0], &w[1]) <= 0.1 + 1e-9);
        }
        let mid = &dense.poses[n / 2].pose.orientation;
        let yaw = 2.0 * mid.z.atan2(mid.w);
        assert!((yaw - (8.871f64 - 11.284).atan2(7.171 - 6.519)).abs() < 1e-9, "inserted poses face along the stretch");
        assert!((path_distance(&dense) - path_distance(&sparse)).abs() < 1e-9, "same path, more poses");
        // already dense enough: unchanged
        let lane = path(&[(0.0, 0.0), (0.0, 0.05), (0.0, 0.1)]);
        assert_eq!(densify_path(&lane, 0.1).poses.len(), 3);
    }

    fn path(points: &[(f64, f64)]) -> Path {
        let mut p = Path::default();
        p.header.frame_id = "map".into();
        for (x, y) in points {
            let mut ps = PoseStamped::default();
            ps.header.frame_id = "map".into();
            ps.pose.position.x = *x;
            ps.pose.position.y = *y;
            ps.pose.orientation.w = 1.0;
            p.poses.push(ps);
        }
        p
    }

    #[test]
    fn dispatch_ids_are_canonical_uuids() {
        assert_eq!(canonical_dispatch_id(" 3F2504E0-4F89-11D3-9A0C-0305E82C3301 ").as_deref(), Some("3f2504e04f8911d39a0c0305e82c3301"));
        assert_eq!(canonical_dispatch_id("3f2504e04f8911d39a0c0305e82c3301").as_deref(), Some("3f2504e04f8911d39a0c0305e82c3301"));
        assert!(canonical_dispatch_id("").is_none());
        assert!(canonical_dispatch_id("not-a-uuid").is_none());
    }

    #[test]
    fn path_validation_matches_python_rules() {
        assert_eq!(navigation_path_block_reason(&path(&[(0.0, 0.0)])).as_deref(), Some("navigation path requires at least two poses"));
        let mut p = path(&[(0.0, 0.0), (1.0, 0.0)]);
        assert!(navigation_path_block_reason(&p).is_none());
        p.header.frame_id = "odom".into();
        assert_eq!(navigation_path_block_reason(&p).as_deref(), Some("navigation path frame must be map, got odom"));
        let mut p = path(&[(0.0, 0.0), (1.0, 0.0)]);
        p.poses[1].header.frame_id = "base_link".into();
        assert_eq!(navigation_path_block_reason(&p).as_deref(), Some("navigation pose 1 frame does not match path frame"));
        let mut p = path(&[(0.0, 0.0), (1.0, 0.0)]);
        p.poses[0].pose.position.x = f64::NAN;
        assert_eq!(navigation_path_block_reason(&p).as_deref(), Some("navigation pose 0 contains a non-finite value"));
        let mut p = path(&[(0.0, 0.0), (1.0, 0.0)]);
        p.poses[1].pose.orientation.w = 0.5;
        assert_eq!(navigation_path_block_reason(&p).as_deref(), Some("navigation pose 1 has an invalid quaternion"));
        let mut p = path(&[(0.0, 0.0), (1.0, 0.0)]);
        p.header.frame_id.clear();
        assert!(navigation_path_block_reason(&p).is_none());
    }

    #[test]
    fn splitting_by_points_turns_and_distance() {
        let p = path(&[(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (2.0, 1.0), (2.0, 2.0), (3.0, 2.0)]);
        let mut sp = Pose::default();
        sp.position.x = 2.0;
        let segs = split_path_by_coverage_points(&p, &[sp], 0.1);
        assert_eq!(segs.len(), 2);
        assert_eq!(segs[0].poses.len(), 3);
        assert_eq!(segs[1].poses.len(), 4);
        assert_eq!(segs[1].poses[0].pose.position.x, 2.0);
        // turn splitting at the two 90 degree corners (min length 0.25)
        let turned = split_paths_by_turn_angle(&[p.clone()], 0.8, 0.25);
        assert_eq!(turned.len(), 3);
        assert_eq!(turned[0].poses.len(), 3);
        assert_eq!(turned[1].poses.len(), 3);
        assert_eq!(turned[2].poses.len(), 2);
        // max distance 0 disables chopping; 1.5 m chops
        assert_eq!(split_paths_by_max_distance(&[p.clone()], 0.0).len(), 1);
        let chopped = split_path_by_max_distance(&p, 1.5);
        assert_eq!(chopped.len(), 3);
        assert!((path_distance(&p) - 5.0).abs() < 1e-12);
        assert!(coverage_segments_block_reason(&turned, 0.15).is_none());
        assert_eq!(coverage_segments_block_reason(&[], 0.15).as_deref(), Some("coverage split produced no executable segments"));
        let tiny = path(&[(0.0, 0.0), (0.1, 0.0)]);
        assert!(coverage_segments_block_reason(&[tiny], 0.15).unwrap().starts_with("coverage segment 1 is only 0.100 m"));
        // 0.20 m is above the 0.15 m floor
        let short = path(&[(0.0, 0.0), (0.05, 0.0), (0.05, 0.05), (0.1, 0.05), (0.1, 0.0)]);
        assert!(coverage_segments_block_reason(&[short], 0.15).is_none());
    }

    #[test]
    fn imu_covariance_gate() {
        let good = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01];
        assert!(imu_covariance_block_reason(&good, 0.35, "orientation").is_none());
        assert_eq!(imu_covariance_block_reason(&good[..8], 0.35, "orientation").as_deref(), Some("IMU orientation covariance is unavailable or invalid"));
        let asym = [0.01, 0.5, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01];
        assert!(imu_covariance_block_reason(&asym, 0.35, "orientation").is_some());
        let big = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        assert_eq!(imu_covariance_block_reason(&big, 0.35, "orientation").as_deref(), Some("IMU orientation uncertainty exceeds 0.350"));
        let zero = [0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01];
        assert!(imu_covariance_block_reason(&zero, 0.35, "orientation").is_some());
        assert_eq!(horizontal_eigen(0.0004, 0.0004, 0.0, 0.0), Some((0.0004, 0.0004)));
        assert!(horizontal_eigen(0.0004, 0.0004, 0.1, 0.0).is_none());
    }
}
