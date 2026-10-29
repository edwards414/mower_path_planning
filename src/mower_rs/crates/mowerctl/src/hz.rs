//! Receive-rate statistics, the arithmetic of `ros2 topic hz` (ros2topic
//! verb/hz.py, ROSTopicHz): the intervals between arrivals, at most `window`
//! of them; a report at most once a second of message time and only after a
//! new message; rate = 1 / mean interval, std dev over the intervals, and
//! "window" = intervals + 1 messages.
//!
//! This is the subscribing mode (E8-1). E8-2 adds a counting mode that reads
//! the bus's `published` counters instead; it reports a rate and a window
//! but no min / max / std dev (`Report`'s fields stay the JSON names).

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use serde_json::{json, Value};

pub const DEFAULT_WINDOW: usize = 10_000;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Report {
    pub rate_hz: f64,
    pub min_s: f64,
    pub max_s: f64,
    pub std_s: f64,
    /// messages the statistic spans (intervals + 1)
    pub window: usize,
}

impl Report {
    /// `ros2 topic hz`'s fields on one line.
    pub fn line(&self) -> String {
        format!(
            "average rate: {:.3}  min: {:.3}s max: {:.3}s std dev: {:.5}s window: {}",
            self.rate_hz, self.min_s, self.max_s, self.std_s, self.window
        )
    }

    pub fn json(&self, t: f64, topic: &str) -> Value {
        json!({"t": (t * 1e3).round() / 1e3, "topic": topic, "rate_hz": self.rate_hz, "min_s": self.min_s,
               "max_s": self.max_s, "std_s": self.std_s, "window": self.window})
    }
}

pub struct Hz {
    window: usize,
    intervals: VecDeque<Duration>,
    last: Option<Instant>,
    last_reported: Option<Instant>,
    pub messages: u64,
}

impl Hz {
    pub fn new(window: usize) -> Self {
        Hz { window: window.max(1), intervals: VecDeque::new(), last: None, last_reported: None, messages: 0 }
    }

    pub fn on_message(&mut self, at: Instant) {
        self.messages += 1;
        if let Some(prev) = self.last {
            // arrivals are monotonic; a reordered one counts as no gap
            self.intervals.push_back(at.saturating_duration_since(prev));
            if self.intervals.len() > self.window {
                self.intervals.pop_front();
            }
        }
        self.last = Some(at);
    }

    /// The statistic, if one is due: hz.py prints nothing for the first
    /// message after the start, and then at most once per second of
    /// arrivals, and only when a new one came.
    pub fn report(&mut self) -> Option<Report> {
        let last = self.last?;
        if self.intervals.is_empty() {
            return None;
        }
        match self.last_reported {
            None => {
                self.last_reported = Some(last);
                return None;
            }
            Some(t) if last < t + Duration::from_secs(1) => return None,
            Some(_) => {}
        }
        self.last_reported = Some(last);
        Some(self.stats())
    }

    fn stats(&self) -> Report {
        let n = self.intervals.len() as f64;
        let secs: Vec<f64> = self.intervals.iter().map(Duration::as_secs_f64).collect();
        let mean = secs.iter().sum::<f64>() / n;
        let std = (secs.iter().map(|d| (d - mean) * (d - mean)).sum::<f64>() / n).sqrt();
        Report {
            rate_hz: if mean > 0.0 { 1.0 / mean } else { 0.0 },
            min_s: secs.iter().cloned().fold(f64::INFINITY, f64::min),
            max_s: secs.iter().cloned().fold(0.0, f64::max),
            std_s: std,
            window: self.intervals.len() + 1,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ms(n: u64) -> Duration {
        Duration::from_millis(n)
    }

    #[test]
    fn reports_follow_hz_py_once_a_second_of_arrivals() {
        let t0 = Instant::now();
        let mut hz = Hz::new(DEFAULT_WINDOW);
        assert_eq!(hz.report(), None, "nothing yet");
        hz.on_message(t0);
        assert_eq!(hz.report(), None, "one message: no interval");
        hz.on_message(t0 + ms(100));
        assert_eq!(hz.report(), None, "the first due report only starts the clock");
        for k in 2..=10 {
            hz.on_message(t0 + ms(100 * k));
        }
        assert_eq!(hz.report(), None, "0.9 s of arrivals since the clock started");
        hz.on_message(t0 + ms(1100));
        let r = hz.report().expect("1 s later");
        assert!((r.rate_hz - 10.0).abs() < 1e-9, "{r:?}");
        assert!((r.min_s - 0.1).abs() < 1e-9 && (r.max_s - 0.1).abs() < 1e-9 && r.std_s < 1e-9, "{r:?}");
        assert_eq!(r.window, 12);
        assert_eq!(hz.report(), None, "no new message, no report");
        assert_eq!(hz.messages, 12);
    }

    #[test]
    fn the_statistic_spans_the_window_only() {
        let t0 = Instant::now();
        let mut hz = Hz::new(3);
        // intervals 0.5, 0.1, 0.2, 0.3: the window keeps the last three
        let mut t = t0;
        hz.on_message(t);
        for d in [500, 100, 200, 300] {
            t += ms(d);
            hz.on_message(t);
        }
        let r = hz.stats();
        assert_eq!(r.window, 4);
        assert!((r.rate_hz - 5.0).abs() < 1e-9, "{r:?}");
        assert!((r.min_s - 0.1).abs() < 1e-9 && (r.max_s - 0.3).abs() < 1e-9, "{r:?}");
        let std = ((0.01f64 + 0.0 + 0.01) / 3.0).sqrt();
        assert!((r.std_s - std).abs() < 1e-9, "{r:?}");
        assert_eq!(
            r.line(),
            format!("average rate: 5.000  min: 0.100s max: 0.300s std dev: {std:.5}s window: 4")
        );
        let j = r.json(1.23456, "/x");
        assert_eq!(j["topic"], "/x");
        assert_eq!(j["window"], 4);
        assert_eq!(j["t"], 1.235);
    }
}
