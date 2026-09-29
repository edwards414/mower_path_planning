//! `mower_nav2/config/dual_ekf_navsat_params.yaml` is the one source of the
//! EKF and navsat_transform settings: the launch files pass it to the C++
//! nodes and to mower_localize alike, and `config.rs` resolves it the way
//! robot_localization's `declare_parameter` calls do.
//!
//! These tests read the file the robot runs with and check two things:
//! every key in it is consumed and reported back at its value (nothing is
//! silently dropped), and it is still the configuration the oracle vectors
//! were generated with (`common/oracle_config.rs` = `tests/oracle/oracle.cpp`).

mod common;

use common::oracle_config::*;
use mower_localize_core::config::{EkfSettings, NavSatSettings, ParamValue, Params};
use mower_localize_core::prepare::SensorConfig;

const NODES: [&str; 3] = ["ekf_filter_node_odom", "ekf_filter_node_map", "navsat_transform"];

fn production_yaml() -> serde_yaml::Value {
    let path = format!(
        "{}/../../../mower_nav2/config/dual_ekf_navsat_params.yaml",
        env!("CARGO_MANIFEST_DIR")
    );
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    serde_yaml::from_str(&text).unwrap_or_else(|e| panic!("{path}: {e}"))
}

/// A yaml scalar or sequence as rcl's parser types it: integers stay
/// integers, a sequence takes its elements' type and must not mix them.
fn convert(key: &str, v: &serde_yaml::Value) -> ParamValue {
    use serde_yaml::Value as Y;
    match v {
        Y::Bool(b) => ParamValue::Bool(*b),
        Y::Number(n) if n.is_i64() => ParamValue::Integer(n.as_i64().unwrap()),
        Y::Number(n) => ParamValue::Double(n.as_f64().unwrap()),
        Y::String(s) => ParamValue::String(s.clone()),
        Y::Sequence(items) => {
            let values: Vec<ParamValue> = items.iter().map(|i| convert(key, i)).collect();
            match values.first() {
                Some(ParamValue::Bool(_)) => ParamValue::BoolArray(
                    values
                        .iter()
                        .map(|v| match v {
                            ParamValue::Bool(b) => *b,
                            other => panic!("{key}: mixed sequence ({other:?}); rcl rejects it"),
                        })
                        .collect(),
                ),
                Some(ParamValue::Double(_)) => ParamValue::DoubleArray(
                    values
                        .iter()
                        .map(|v| match v {
                            ParamValue::Double(d) => *d,
                            other => panic!("{key}: mixed sequence ({other:?}); rcl rejects it"),
                        })
                        .collect(),
                ),
                other => panic!("{key}: unsupported sequence {other:?}"),
            }
        }
        other => panic!("{key}: unsupported yaml value {other:?}"),
    }
}

fn section(doc: &serde_yaml::Value, node: &str) -> Params {
    let map = doc[node]["ros__parameters"]
        .as_mapping()
        .unwrap_or_else(|| panic!("no {node}.ros__parameters section"));
    map.iter()
        .map(|(k, v)| {
            let key = k.as_str().expect("string key").to_string();
            let value = convert(&key, v);
            (key, value)
        })
        .collect()
}

#[test]
fn every_key_in_the_yaml_is_applied_and_reported_back() {
    let doc = production_yaml();
    for node in NODES {
        let params = section(&doc, node);
        let (warnings, reported) = if node == "navsat_transform" {
            let r = NavSatSettings::from_params(node, &params).expect(node);
            (r.warnings, r.parameters)
        } else {
            let r = EkfSettings::from_params(node, &params).expect(node);
            (r.warnings, r.parameters)
        };
        assert!(warnings.is_empty(), "{node}: {warnings:#?}");
        for (key, value) in &params {
            let live = reported
                .iter()
                .find(|(k, _)| k == key)
                .unwrap_or_else(|| panic!("{node}: {key} is in the yaml but not consumed"));
            assert_eq!(&live.1, value, "{node}: {key}");
        }
        println!("{node}: {} keys, all consumed", params.len());
    }
}

fn assert_sensor(node: &str, got: &SensorConfig, oracle: &SensorConfig) {
    assert_eq!(got.update_vector, oracle.update_vector, "{node} {}", got.topic_name);
    assert_eq!(got.differential, oracle.differential, "{node} {}", got.topic_name);
    assert_eq!(got.relative, oracle.relative, "{node} {}", got.topic_name);
    assert_eq!(got.rejection_threshold, oracle.rejection_threshold, "{node} {}", got.topic_name);
    assert_eq!(got.pose_use_child_frame, oracle.pose_use_child_frame, "{node} {}", got.topic_name);
    assert_eq!(
        got.remove_gravitational_acceleration, oracle.remove_gravitational_acceleration,
        "{node} {}",
        got.topic_name
    );
}

#[test]
fn the_yaml_is_still_the_configuration_the_oracle_verified() {
    let doc = production_yaml();
    for (node, oracle, odoms, imu) in [
        ("ekf_filter_node_odom", ekf_odom_config(), vec![odom_ekf_odom0()], odom_ekf_imu0()),
        (
            "ekf_filter_node_map",
            ekf_map_config(),
            vec![map_ekf_odom0(), map_ekf_odom1()],
            map_ekf_imu0(),
        ),
    ] {
        let s = EkfSettings::from_params(node, &section(&doc, node)).unwrap().settings;
        assert_eq!(s.frequency, oracle.frequency, "{node}");
        // The oracle runs with a 50 ms sensor timeout.
        assert_eq!(s.build_filter().filter.sensor_timeout_ns, 50_000_000, "{node}");
        assert_eq!(s.two_d_mode, oracle.two_d_mode, "{node}");
        assert_eq!(s.publish_tf, oracle.publish_tf, "{node}");
        assert_eq!(s.map_frame, oracle.map_frame, "{node}");
        assert_eq!(s.odom_frame, oracle.odom_frame, "{node}");
        assert_eq!(s.base_link_frame, oracle.base_link_frame, "{node}");
        assert_eq!(s.world_frame, oracle.world_frame, "{node}");
        assert_eq!(s.process_noise_covariance, Some(oracle.process_noise_covariance), "{node}");
        assert!(s.initial_estimate_covariance.is_none() && !s.predict_to_current_time);
        assert!(!s.dynamic_process_noise_covariance && !s.permit_corrected_publication);
        assert_eq!(s.gravitational_acceleration, 9.80665);
        assert_eq!(s.odoms.len(), odoms.len(), "{node}");
        for (got, want) in s.odoms.iter().zip(&odoms) {
            assert_sensor(node, &got.config, want);
            assert_eq!(got.queue_size, 10);
        }
        assert_eq!(s.imus.len(), 1, "{node}");
        assert_sensor(node, &s.imus[0].config, &imu);
        assert_eq!(s.imus[0].queue_size, 10);
    }

    let nav = NavSatSettings::from_params("navsat_transform", &section(&doc, "navsat_transform"))
        .unwrap()
        .settings;
    let oracle = navsat_config();
    let c = &nav.config;
    assert_eq!(c.magnetic_declination_radians, oracle.magnetic_declination_radians);
    assert_eq!(c.yaw_offset, oracle.yaw_offset);
    assert_eq!(c.zero_altitude, oracle.zero_altitude);
    assert_eq!(c.publish_filtered_gps, oracle.publish_filtered_gps);
    assert_eq!(c.use_odometry_yaw, oracle.use_odometry_yaw);
    assert_eq!(c.wait_for_datum, oracle.wait_for_datum);
    assert_eq!(c.broadcast_cartesian_transform, oracle.broadcast_cartesian_transform);
    assert_eq!(
        c.broadcast_cartesian_transform_as_parent_frame,
        oracle.broadcast_cartesian_transform_as_parent_frame
    );
    // The health gate wants /odometry/gps within 0.30 s of a fix, and the
    // datum waits for the map EKF's heading to settle.
    assert_eq!(nav.frequency, 30.0);
    assert_eq!(nav.delay, 3.0);
}
