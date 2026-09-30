//! Shared state between the subscription tasks and the publishing timers.

use std::collections::VecDeque;
use std::time::{Instant, SystemTime};

use mower_rs_common::round_to;
use r2r::nav_msgs::msg::Odometry;
use r2r::sensor_msgs::msg::{BatteryState, Imu, NavSatFix};
use serde_json::Value;

/// Latest message of one source plus when it arrived (monotonic), mirroring
/// `telemetry_node._Sample`.
pub struct Sample<T> {
    pub msg: Option<T>,
    pub t: Option<Instant>,
    pub count: u64,
    window: VecDeque<Instant>,
}

impl<T> Default for Sample<T> {
    fn default() -> Self {
        Self { msg: None, t: None, count: 0, window: VecDeque::new() }
    }
}

impl<T> Sample<T> {
    pub fn set(&mut self, msg: T, now: Instant) {
        self.msg = Some(msg);
        self.t = Some(now);
        self.count += 1;
        self.window.push_back(now);
        while let Some(front) = self.window.front() {
            if now.duration_since(*front).as_secs_f64() > 2.0 {
                self.window.pop_front();
            } else {
                break;
            }
        }
    }

    /// Seconds since the last sample, rounded to 3 decimals; `None` before
    /// the first one.
    pub fn age_s(&self, now: Instant) -> Option<f64> {
        self.t.map(|t| round_to(now.duration_since(t).as_secs_f64(), 3))
    }

    /// Arrival rate over the last two seconds, rounded to 0.1 Hz.
    pub fn rate_hz(&self) -> f64 {
        if self.window.len() < 2 {
            return 0.0;
        }
        let span = self
            .window
            .back()
            .unwrap()
            .duration_since(*self.window.front().unwrap())
            .as_secs_f64();
        if span > 0.0 {
            round_to((self.window.len() - 1) as f64 / span, 1)
        } else {
            0.0
        }
    }
}

#[derive(Default)]
pub struct State {
    /// Heartbeat: when the liveness source topic last delivered a message.
    pub heartbeat_last: Option<Instant>,
    /// Telemetry inputs.
    pub fix: Sample<NavSatFix>,
    pub filtered: Sample<NavSatFix>,
    pub imu: Sample<Imu>,
    pub odom: Sample<Odometry>,
    pub base: Sample<Value>,
    pub battery: Sample<BatteryState>,
    pub aon_battery: Sample<BatteryState>,
    pub link: Sample<Value>,
    pub link_mtime: Option<SystemTime>,
    /// Latest `host` block from `host::Sampler` (sampled outside the lock).
    pub host: Value,
    /// robot_info inputs and its last published snapshot (embedded in the
    /// telemetry document as `info`).
    pub firmware_running: Option<Value>,
    pub nav_running: bool,
    pub last_moving: Option<Instant>,
    pub info: Option<Value>,
    pub led_updating: Option<bool>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn sample_rate_window() {
        let mut s: Sample<u8> = Sample::default();
        let t0 = Instant::now();
        assert_eq!(s.rate_hz(), 0.0);
        assert_eq!(s.age_s(t0), None);
        for i in 0..11 {
            s.set(i, t0 + Duration::from_millis(100 * i as u64));
        }
        assert_eq!(s.rate_hz(), 10.0);
        assert_eq!(s.count, 11);
        assert_eq!(s.age_s(t0 + Duration::from_millis(1250)), Some(0.25));
        // samples older than two seconds fall out of the window
        s.set(99, t0 + Duration::from_secs(10));
        assert_eq!(s.rate_hz(), 0.0);
    }
}
