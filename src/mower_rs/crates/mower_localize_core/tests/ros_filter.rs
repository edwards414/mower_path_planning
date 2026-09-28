//! Differential test of the whole filter pipeline — the `RosFilter`
//! preprocessing (`preparePose`/`prepareTwist`/`prepareAcceleration`, the
//! callbacks, the measurement queue, `integrateMeasurements`) plus the EKF —
//! against the real robot_localization, for both instances in
//! `dual_ekf_navsat_params.yaml`.
//!
//! The messages, the covariances and the sensor frames are exactly the ones the
//! oracle fed to the C++ implementation (`tests/oracle/oracle.cpp`).

mod common;

use common::oracle_config::*;
use common::*;
use mower_localize_core::msgs::*;
use mower_localize_core::prepare::{RosFilterCore, SensorConfig};
use mower_localize_core::tf::{Quaternion, Transform, Vector3};
use serde_json::Value;

const TOL: f64 = 1e-9;

fn transforms_into(core: &mut RosFilterCore, data: &Value) {
    for t in data["transforms"].as_array().unwrap() {
        let origin = vec_f(&t["origin"]);
        let rot = vec_f(&t["rotation"]);
        let transform = Transform::from_parts(
            &Quaternion::new(rot[0], rot[1], rot[2], rot[3]),
            Vector3::new(origin[0], origin[1], origin[2]),
        );
        core.transforms.insert(
            t["target"].as_str().unwrap(),
            t["source"].as_str().unwrap(),
            transform,
        );
    }
}

fn odom_message(input: &Value) -> Odometry {
    let lin = vec_f(&input["twist_linear"]);
    let ang = vec_f(&input["twist_angular"]);
    let mut msg = Odometry {
        header: Header {
            frame_id: input["frame_id"].as_str().unwrap().to_string(),
            stamp_ns: input["stamp_ns"].as_i64().unwrap(),
        },
        child_frame_id: input["child_frame_id"].as_str().unwrap().to_string(),
        ..Default::default()
    };
    msg.twist.linear = Vector3::new(lin[0], lin[1], lin[2]);
    msg.twist.angular = Vector3::new(ang[0], ang[1], ang[2]);
    // The covariance entries the oracle sets (twist.covariance[0, 7, 14, 35]).
    msg.twist.covariance[0][0] = 0.002;
    msg.twist.covariance[1][1] = 0.002;
    msg.twist.covariance[2][2] = 0.01;
    msg.twist.covariance[5][5] = 0.004;
    msg.pose.covariance[0][0] = 0.01;
    msg.pose.covariance[1][1] = 0.01;
    msg.pose.covariance[5][5] = 0.02;
    msg
}

fn imu_message(input: &Value) -> Imu {
    let rpy = vec_f(&input["rpy"]);
    let av = vec_f(&input["angular_velocity"]);
    let la = vec_f(&input["linear_acceleration"]);
    let mut msg = Imu {
        header: Header {
            frame_id: input["frame_id"].as_str().unwrap().to_string(),
            stamp_ns: input["stamp_ns"].as_i64().unwrap(),
        },
        orientation: Quaternion::from_rpy(rpy[0], rpy[1], rpy[2]),
        angular_velocity: Vector3::new(av[0], av[1], av[2]),
        linear_acceleration: Vector3::new(la[0], la[1], la[2]),
        ..Default::default()
    };
    for i in 0..3 {
        msg.orientation_covariance[i][i] = 0.01;
        msg.angular_velocity_covariance[i][i] = 0.005;
        msg.linear_acceleration_covariance[i][i] = 0.02;
    }
    msg
}

fn gps_message(input: &Value) -> Odometry {
    let p = vec_f(&input["position"]);
    let mut msg = Odometry {
        header: Header {
            frame_id: input["frame_id"].as_str().unwrap().to_string(),
            stamp_ns: input["stamp_ns"].as_i64().unwrap(),
        },
        child_frame_id: String::new(),
        ..Default::default()
    };
    msg.pose.position = Vector3::new(p[0], p[1], p[2]);
    msg.pose.orientation = Quaternion::identity();
    msg.pose.covariance[0][0] = 0.5;
    msg.pose.covariance[0][1] = 0.05;
    msg.pose.covariance[1][0] = 0.05;
    msg.pose.covariance[1][1] = 0.5;
    msg.pose.covariance[2][2] = 1.0;
    msg
}

fn run(file: &str, map_instance: bool) {
    let data = load(file);
    let config = if map_instance {
        ekf_map_config()
    } else {
        ekf_odom_config()
    };
    let mut core = build_filter(&config);
    core.filter
        .set_process_noise_covariance(mat15(&data["process_noise_covariance"]));
    core.filter.sensor_timeout_ns = data["sensor_timeout_ns"].as_i64().unwrap();
    core.last_diff_time = 1.7e9;
    transforms_into(&mut core, &data);

    let odom_cfg: SensorConfig = if map_instance {
        map_ekf_odom0()
    } else {
        odom_ekf_odom0()
    };
    let imu_cfg: SensorConfig = if map_instance {
        map_ekf_imu0()
    } else {
        odom_ekf_imu0()
    };
    let gps_cfg = map_ekf_odom1();

    let steps = data["steps"].as_array().unwrap();
    let mut max_meas_err = 0.0f64;
    let mut max_state_err = 0.0f64;
    let mut max_cov_err = 0.0f64;
    let mut prepared_count = 0usize;

    for (i, step) in steps.iter().enumerate() {
        let input = &step["input"];
        match input["type"].as_str().unwrap() {
            "odom" => core.odometry_callback(&odom_message(input), &odom_cfg),
            "imu" => core.imu_callback(&imu_message(input), &imu_cfg),
            "gps" => {
                if map_instance {
                    core.odometry_callback(&gps_message(input), &gps_cfg)
                }
            }
            other => panic!("unknown input {other}"),
        }

        // The prepared measurements now sitting in the queue.
        let expected = step["prepared"].as_array().unwrap();
        let queued = core.queued();
        assert_eq!(
            queued.len(),
            expected.len(),
            "step {i}: queue length after a {} message",
            input["type"]
        );
        for (m, e) in queued.iter().zip(expected.iter()) {
            assert_eq!(m.time_ns, e["time_ns"].as_i64().unwrap(), "step {i}: stamp");
            assert_eq!(
                m.update_vector,
                bools15(&e["update_vector"]),
                "step {i} ({}): update vector",
                m.topic_name
            );
            let em = arr15(&e["measurement"]);
            let ec = mat15(&e["covariance"]);
            for k in 0..15 {
                max_meas_err = max_meas_err.max(assert_close(
                    m.measurement[k],
                    em[k],
                    TOL,
                    &format!("step {i} ({}) measurement[{k}]", m.topic_name),
                ));
                for l in 0..15 {
                    max_meas_err = max_meas_err.max(assert_close(
                        m.covariance[k][l],
                        ec[k][l],
                        TOL,
                        &format!(
                            "step {i} ({}) measurement covariance[{k}][{l}]",
                            m.topic_name
                        ),
                    ));
                }
            }
            prepared_count += 1;
        }

        let now = step["integrate_time_ns"].as_i64().unwrap();
        core.integrate_measurements(now);
        core.differentiate_measurements(now);

        let state = arr15(&step["state"]);
        let cov = mat15(&step["estimate_error_covariance"]);
        for k in 0..15 {
            max_state_err = max_state_err.max(assert_close(
                core.filter.state[k],
                state[k],
                TOL,
                &format!("step {i}: state[{k}]"),
            ));
            for l in 0..15 {
                max_cov_err = max_cov_err.max(assert_close(
                    core.filter.estimate_error_covariance[k][l],
                    cov[k][l],
                    TOL,
                    &format!("step {i}: covariance[{k}][{l}]"),
                ));
            }
        }
        assert_eq!(
            core.filter.last_measurement_time_ns,
            step["last_measurement_time_ns"].as_i64().unwrap(),
            "step {i}: last measurement time"
        );
        let aa = vec_f(&step["angular_acceleration"]);
        assert_close(core.angular_acceleration.x, aa[0], TOL, "angular accel x");
        assert_close(core.angular_acceleration.y, aa[1], TOL, "angular accel y");
        assert_close(core.angular_acceleration.z, aa[2], TOL, "angular accel z");
    }

    println!(
        "{file}: {} steps, {prepared_count} prepared measurements, \
         max measurement error {max_meas_err:.3e}, max state error {max_state_err:.3e}, \
         max covariance error {max_cov_err:.3e}",
        steps.len()
    );
}

#[test]
fn odom_instance_matches_robot_localization() {
    run("ros_filter_odom.json", false);
}

#[test]
fn map_instance_matches_robot_localization() {
    run("ros_filter_map.json", true);
}
