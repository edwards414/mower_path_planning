//! Differential test of the navsat_transform maths against the real
//! `robot_localization::NavSatTransform`, and of the UTM projection against
//! GeographicLib 2.3 (the library robot_localization itself links).

mod common;

use common::*;
use mower_localize_core::config::navsat_config;
use mower_localize_core::msgs::*;
use mower_localize_core::navsat::NavSatTransformCore;
use mower_localize_core::tf::{Quaternion, Transform, Vector3};
use mower_localize_core::utm::{utmups_forward, utmups_reverse, ZONE_STANDARD};

/// 1e-6 m is the accuracy the plan asks for; the port actually lands far below.
const METRE_TOL: f64 = 1e-6;
/// Degrees per metre at the equator, for turning a lat/lon error into metres.
const DEG_PER_M: f64 = 1.0 / 111_320.0;

#[test]
fn utm_matches_geographiclib() {
    let data = load("navsat.json");
    let points = data["utm"].as_array().unwrap();
    assert!(points.len() >= 12);

    let mut max_fwd = 0.0f64;
    let mut max_rev = 0.0f64;
    for p in points {
        let lat = f(&p["lat"]);
        let lon = f(&p["lon"]);
        let (zone, northp, x, y, gamma, k) =
            utmups_forward(lat, lon, ZONE_STANDARD).expect("forward");
        assert_eq!(
            zone,
            p["zone"].as_i64().unwrap() as i32,
            "zone at {lat},{lon}"
        );
        assert_eq!(northp, p["northp"].as_bool().unwrap(), "hemisphere");
        let (ex, ey) = (f(&p["x"]), f(&p["y"]));
        max_fwd = max_fwd.max((x - ex).abs()).max((y - ey).abs());
        assert!(
            (x - ex).abs() < METRE_TOL && (y - ey).abs() < METRE_TOL,
            "UTM forward at {lat},{lon}: ({x}, {y}) vs ({ex}, {ey})"
        );
        // The meridian convergence feeds the datum heading, so it matters too.
        assert!(
            (gamma - f(&p["gamma"])).abs() < 1e-11,
            "convergence at {lat},{lon}"
        );
        assert!((k - f(&p["k"])).abs() < 1e-12, "scale at {lat},{lon}");

        let (rlat, rlon, _, _) = utmups_reverse(zone, northp, ex, ey).expect("reverse");
        let err = ((rlat - f(&p["rev_lat"])).abs() + (rlon - f(&p["rev_lon"])).abs()) / DEG_PER_M;
        max_rev = max_rev.max(err);
        assert!(err < METRE_TOL, "UTM reverse at {lat},{lon}: {err} m");
    }
    println!(
        "utm: {} points, max forward error {max_fwd:.3e} m, max reverse error {max_rev:.3e} m",
        points.len()
    );
}

#[test]
fn navsat_transform_matches_robot_localization() {
    let data = load("navsat.json");
    let mut core = NavSatTransformCore::new(navsat_config());
    for t in data["transforms"].as_array().unwrap() {
        let origin = vec_f(&t["origin"]);
        let rot = vec_f(&t["rotation"]);
        core.transforms.insert(
            t["target"].as_str().unwrap(),
            t["source"].as_str().unwrap(),
            Transform::from_parts(
                &Quaternion::new(rot[0], rot[1], rot[2], rot[3]),
                Vector3::new(origin[0], origin[1], origin[2]),
            ),
        );
    }

    let steps = data["steps"].as_array().unwrap();
    let mut max_pos = 0.0f64;
    let mut max_ll = 0.0f64;
    let mut max_cov = 0.0f64;
    let mut gps_odoms = 0usize;
    let mut filtered = 0usize;

    for (i, step) in steps.iter().enumerate() {
        let stamp = step["stamp_ns"].as_i64().unwrap();

        let pos = vec_f(&step["odom"]["position"]);
        let q = vec_f(&step["odom"]["yaw_quat"]);
        let mut odom = Odometry {
            header: Header {
                frame_id: "map".to_string(),
                stamp_ns: stamp,
            },
            child_frame_id: "base_footprint".to_string(),
            ..Default::default()
        };
        odom.pose.position = Vector3::new(pos[0], pos[1], pos[2]);
        odom.pose.orientation = Quaternion::new(q[0], q[1], q[2], q[3]);
        for k in 0..6 {
            odom.pose.covariance[k][k] = 0.1 * (k as f64 + 1.0);
        }
        core.odom_callback(&odom);

        let iq = vec_f(&step["imu_quat"]);
        let imu = Imu {
            header: Header {
                frame_id: "imu_link".to_string(),
                stamp_ns: stamp,
            },
            orientation: Quaternion::new(iq[0], iq[1], iq[2], iq[3]),
            ..Default::default()
        };
        core.imu_callback(&imu);

        let mut fix = NavSatFix {
            header: Header {
                frame_id: "gps_link".to_string(),
                stamp_ns: stamp,
            },
            status: STATUS_FIX,
            latitude: f(&step["fix"]["lat"]),
            longitude: f(&step["fix"]["lon"]),
            altitude: f(&step["fix"]["alt"]),
            position_covariance: ZERO_MAT3,
        };
        fix.position_covariance[0][0] = 0.04;
        fix.position_covariance[1][1] = 0.04;
        fix.position_covariance[2][2] = 0.09;
        core.gps_fix_callback(&fix);

        core.compute_transform();

        assert_eq!(
            core.transform_good,
            step["transform_good"].as_bool().unwrap(),
            "step {i}: transform_good"
        );
        assert_eq!(
            core.utm_zone,
            step["utm_zone"].as_i64().unwrap() as i32,
            "step {i}: utm zone"
        );
        assert_eq!(core.northp, step["northp"].as_bool().unwrap(), "step {i}");
        assert_close(
            core.utm_meridian_convergence,
            f(&step["meridian_convergence"]),
            1e-12,
            &format!("step {i}: meridian convergence"),
        );

        let cwt = &step["cartesian_world_transform"];
        let o = vec_f(&cwt["origin"]);
        for (k, expected) in o.iter().enumerate() {
            let got = match k {
                0 => core.cartesian_world_transform.origin.x,
                1 => core.cartesian_world_transform.origin.y,
                _ => core.cartesian_world_transform.origin.z,
            };
            max_pos = max_pos.max((got - expected).abs());
            assert!(
                (got - expected).abs() < METRE_TOL,
                "step {i}: cartesian_world_transform origin[{k}]: {got} vs {expected}"
            );
        }

        let gps_odom = core.prepare_gps_odometry();
        assert_eq!(
            gps_odom.is_some(),
            step["have_gps_odom"].as_bool().unwrap(),
            "step {i}: gps odometry availability"
        );
        if let Some(odom) = gps_odom {
            gps_odoms += 1;
            let expected = vec_f(&step["gps_odom_position"]);
            for (k, e) in expected.iter().enumerate() {
                let got = match k {
                    0 => odom.pose.position.x,
                    1 => odom.pose.position.y,
                    _ => odom.pose.position.z,
                };
                max_pos = max_pos.max((got - e).abs());
                assert!(
                    (got - e).abs() < METRE_TOL,
                    "step {i}: /odometry/gps position[{k}]: {got} vs {e}"
                );
            }
            let cov = vec_f(&step["gps_odom_cov"]);
            for r in 0..6 {
                for c in 0..6 {
                    max_cov = max_cov.max(assert_close(
                        odom.pose.covariance[r][c],
                        cov[r * 6 + c],
                        1e-9,
                        &format!("step {i}: /odometry/gps covariance[{r}][{c}]"),
                    ));
                }
            }
        }

        let filtered_fix = core.prepare_filtered_gps();
        assert_eq!(
            filtered_fix.is_some(),
            step["have_filtered_gps"].as_bool().unwrap(),
            "step {i}: filtered gps availability"
        );
        if let Some(got) = filtered_fix {
            filtered += 1;
            let e = &step["filtered_gps"];
            let dlat = (got.latitude - f(&e["lat"])).abs() / DEG_PER_M;
            let dlon = (got.longitude - f(&e["lon"])).abs() / DEG_PER_M;
            max_ll = max_ll.max(dlat).max(dlon);
            assert!(
                dlat < METRE_TOL && dlon < METRE_TOL,
                "step {i}: /gps/filtered"
            );
            assert_close(
                got.altitude,
                f(&e["alt"]),
                1e-9,
                &format!("step {i}: /gps/filtered altitude"),
            );
            let cov = vec_f(&e["cov"]);
            for r in 0..3 {
                for c in 0..3 {
                    max_cov = max_cov.max(assert_close(
                        got.position_covariance[r][c],
                        cov[r * 3 + c],
                        1e-9,
                        &format!("step {i}: /gps/filtered covariance[{r}][{c}]"),
                    ));
                }
            }
        }
    }

    println!(
        "navsat: {} steps, {gps_odoms} /odometry/gps and {filtered} /gps/filtered messages, \
         max position error {max_pos:.3e} m, max lat/lon error {max_ll:.3e} m, \
         max covariance error {max_cov:.3e}",
        steps.len()
    );
}
