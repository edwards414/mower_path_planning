//! `robot_localization/measurement.hpp`, minus the ROS types.

use crate::filter_common::{Mat15, Vec15, STATE_SIZE, TWIST_SIZE, ZERO_MAT15};

/// One measurement ready for the filter: everything is already expressed in
/// the filter's state layout by the preprocessing in [`crate::prepare`].
#[derive(Clone, Debug)]
pub struct Measurement {
    pub topic_name: String,
    /// Measurement timestamp, nanoseconds (the message stamp).
    pub time_ns: i64,
    pub measurement: Vec15,
    pub covariance: Mat15,
    pub update_vector: [bool; STATE_SIZE],
    pub mahalanobis_thresh: f64,
    pub latest_control: [f64; TWIST_SIZE],
    pub latest_control_time_ns: i64,
}

impl Measurement {
    pub fn new(topic_name: &str, time_ns: i64) -> Self {
        Self {
            topic_name: topic_name.to_string(),
            time_ns,
            measurement: [0.0; STATE_SIZE],
            covariance: ZERO_MAT15,
            update_vector: [false; STATE_SIZE],
            // robot_localization's default rejection threshold.
            mahalanobis_thresh: f64::MAX,
            latest_control: [0.0; TWIST_SIZE],
            latest_control_time_ns: 0,
        }
    }
}
