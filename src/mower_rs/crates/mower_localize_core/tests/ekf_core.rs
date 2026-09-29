//! Differential test of the filter core against the real
//! `robot_localization::Ekf` (Jazzy 3.8.3): the same measurements, including
//! out-of-sequence and duplicate timestamps, fed to `processMeasurement`.
//!
//! Vectors come from `tests/oracle/oracle.cpp`; see `tests/oracle/run_oracle.sh`.

mod common;

use common::*;
use mower_localize_core::{ekf::Ekf, measurement::Measurement};

const TOL: f64 = 1e-9;

#[test]
fn ekf_core_matches_robot_localization() {
    let data = load("ekf_core.json");
    let mut filter = Ekf::new();
    filter.set_process_noise_covariance(mat15(&data["process_noise_covariance"]));
    filter.sensor_timeout_ns = data["sensor_timeout_ns"].as_i64().unwrap();

    let steps = data["steps"].as_array().unwrap();
    assert!(steps.len() > 100, "expected a long sequence");

    let mut max_state_err = 0.0f64;
    let mut max_cov_err = 0.0f64;

    for (i, step) in steps.iter().enumerate() {
        let mut m = Measurement::new(
            step["topic"].as_str().unwrap(),
            step["time_ns"].as_i64().unwrap(),
        );
        m.measurement = arr15(&step["measurement"]);
        m.covariance = mat15(&step["covariance"]);
        m.update_vector = bools15(&step["update_vector"]);
        m.mahalanobis_thresh = f(&step["mahalanobis"]);

        filter.process_measurement(&m);

        let state = arr15(&step["state"]);
        let cov = mat15(&step["estimate_error_covariance"]);
        for k in 0..15 {
            max_state_err = max_state_err.max(assert_close(
                filter.state[k],
                state[k],
                TOL,
                &format!("step {i} ({}) state[{k}]", m.topic_name),
            ));
            for l in 0..15 {
                max_cov_err = max_cov_err.max(assert_close(
                    filter.estimate_error_covariance[k][l],
                    cov[k][l],
                    TOL,
                    &format!("step {i} ({}) covariance[{k}][{l}]", m.topic_name),
                ));
            }
        }
        assert_eq!(
            filter.last_measurement_time_ns,
            step["last_measurement_time_ns"].as_i64().unwrap(),
            "step {i}: last measurement time"
        );
    }

    println!(
        "ekf core: {} steps, max state error {max_state_err:.3e}, max covariance error {max_cov_err:.3e}",
        steps.len()
    );
}
