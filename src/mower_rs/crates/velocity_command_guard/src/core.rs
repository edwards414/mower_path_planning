//! The guard's decision logic, free of ROS so every rule has a test vector.
//!
//! Faithful port of `mower_bringup/velocity_command_guard.py`; the comments
//! that explain a rule are kept next to it. Two clocks are involved, exactly
//! as in the Python node: `ros_ns` is the node clock (wall time in
//! production, what stamps are compared against and what outgoing stamps
//! carry), `mono` is a steady clock used for receipt timeouts.

use std::time::Instant;

#[derive(Debug, Clone, PartialEq)]
pub struct Limits {
    pub command_timeout_s: f64,
    pub max_input_age_s: f64,
    pub max_future_skew_s: f64,
    pub max_linear_x: f64,
    pub max_angular_z: f64,
    pub require_command_session: bool,
    pub command_clock_period_s: f64,
}

impl Limits {
    /// Clamp raw parameter values the way the Python node does.
    pub fn new(
        command_timeout_s: f64,
        max_input_age_s: f64,
        max_future_skew_s: f64,
        max_linear_x: f64,
        max_angular_z: f64,
        require_command_session: bool,
        command_clock_period_s: f64,
    ) -> Result<Self, String> {
        let limits = Self {
            command_timeout_s: command_timeout_s.max(0.05),
            max_input_age_s: max_input_age_s.max(0.0),
            max_future_skew_s: max_future_skew_s.max(0.0),
            max_linear_x: max_linear_x.max(0.0),
            max_angular_z: max_angular_z.max(0.0),
            require_command_session,
            command_clock_period_s: command_clock_period_s.max(0.02),
        };
        let all_finite = [
            limits.command_timeout_s,
            limits.max_input_age_s,
            limits.max_future_skew_s,
            limits.max_linear_x,
            limits.max_angular_z,
            limits.command_clock_period_s,
        ]
        .iter()
        .all(|v| v.is_finite());
        if !all_finite {
            return Err("velocity guard safety limits must be finite".to_string());
        }
        Ok(limits)
    }

    /// Watchdog period: `min(0.05, command_timeout_s / 2)`.
    pub fn watchdog_period_s(&self) -> f64 {
        (self.command_timeout_s / 2.0).min(0.05)
    }
}

/// An incoming `geometry_msgs/TwistStamped`, reduced to what the guard reads.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Command {
    pub stamp_sec: i32,
    pub stamp_nanosec: u32,
    pub session_matches: bool,
    pub linear: [f64; 3],
    pub angular: [f64; 3],
}

impl Command {
    fn source_stamp_ns(&self) -> Option<i64> {
        if self.stamp_sec == 0 && self.stamp_nanosec == 0 {
            None
        } else {
            Some(self.stamp_sec as i64 * 1_000_000_000 + self.stamp_nanosec as i64)
        }
    }

    fn moving(&self) -> bool {
        self.linear[0].abs() > 1e-9 || self.angular[2].abs() > 1e-9
    }
}

/// What the node has to do with a command.
#[derive(Debug, Clone, PartialEq)]
pub enum Action {
    /// Publish a zero and log the reason (rate limited by the caller).
    Reject(&'static str),
    /// Publish this forward/yaw pair with a fresh robot stamp.
    Forward { linear_x: f64, angular_z: f64 },
}

pub struct Guard {
    limits: Limits,
    nonzero_active: bool,
    last_nonzero_received_at: Option<Instant>,
    last_source_stamp_ns: Option<i64>,
}

impl Guard {
    pub fn new(limits: Limits) -> Self {
        Self { limits, nonzero_active: false, last_nonzero_received_at: None, last_source_stamp_ns: None }
    }

    /// Advance a trusted stamped ordering barrier before payload checks.
    fn apply_timestamp_barrier(&mut self, command: &Command, ros_ns: i64) -> (Option<&'static str>, Option<i64>, bool) {
        let Some(stamp_ns) = command.source_stamp_ns() else {
            return (None, None, false);
        };
        let age_s = (ros_ns - stamp_ns) as f64 / 1e9;
        if age_s < -self.limits.max_future_skew_s {
            return (Some("velocity timestamp is too far in the future"), Some(stamp_ns), false);
        }
        let previous = self.last_source_stamp_ns;
        if let Some(previous) = previous {
            if stamp_ns < previous {
                return (Some("velocity source timestamp moved backward"), Some(stamp_ns), false);
            }
        }
        let is_new = previous.map(|p| stamp_ns > p).unwrap_or(true);
        if is_new {
            // A newer malformed/overspeed command still represents a newer
            // stop barrier. A queued older valid motion must never resume after
            // the guard rejected it and published zero.
            self.last_source_stamp_ns = Some(stamp_ns);
        }
        (None, Some(stamp_ns), is_new)
    }

    fn block_reason(
        &self,
        command: &Command,
        source_stamp_ns: Option<i64>,
        timestamp_is_new: bool,
        ros_ns: i64,
        mono: Instant,
    ) -> Option<&'static str> {
        if self.limits.require_command_session {
            if !command.session_matches {
                return Some("manual velocity command session is missing or stale");
            }
            if source_stamp_ns.is_none() {
                return Some("manual velocity requires the robot command clock");
            }
        }
        let values = command.linear.iter().chain(command.angular.iter());
        if !values.clone().all(|v| v.is_finite()) {
            return Some("velocity contains NaN or infinity");
        }
        let unsupported = [command.linear[1], command.linear[2], command.angular[0], command.angular[1]];
        if unsupported.iter().any(|v| v.abs() > 1e-9) {
            return Some("unsupported lateral or non-yaw velocity component");
        }
        if command.linear[0].abs() > self.limits.max_linear_x {
            return Some("linear velocity exceeds robot safety limit");
        }
        if command.angular[2].abs() > self.limits.max_angular_z {
            return Some("angular velocity exceeds robot safety limit");
        }
        // Zero stamps request receipt-time semantics for trusted local sources.
        // The untrusted Flutter path requires a robot-issued nonzero stamp.
        if let Some(stamp_ns) = source_stamp_ns {
            let age_s = (ros_ns - stamp_ns) as f64 / 1e9;
            if age_s > self.limits.max_input_age_s {
                return Some("velocity timestamp is stale");
            }
            if command.moving() && !timestamp_is_new {
                let replay_expired = match self.last_nonzero_received_at {
                    None => true,
                    Some(received_at) => mono.duration_since(received_at).as_secs_f64() > self.limits.command_timeout_s,
                };
                if !self.nonzero_active || replay_expired {
                    return Some("replayed velocity cannot resume stopped motion");
                }
            }
        }
        None
    }

    /// Decide what to do with `command` received now.
    pub fn on_command(&mut self, command: &Command, ros_ns: i64, mono: Instant) -> Action {
        let (barrier_reason, source_stamp_ns, timestamp_is_new) = self.apply_timestamp_barrier(command, ros_ns);
        let reason = barrier_reason.or_else(|| self.block_reason(command, source_stamp_ns, timestamp_is_new, ros_ns, mono));
        if let Some(reason) = reason {
            // A rejection is a stop boundary even when its source stamp is
            // untrusted (for example, far in the future). Advance only to the
            // robot's own clock so any command already queued before this
            // rejection cannot become a fresh resume command afterward.
            if self.last_source_stamp_ns.map(|p| ros_ns > p).unwrap_or(true) {
                self.last_source_stamp_ns = Some(ros_ns);
            }
            self.nonzero_active = false;
            self.last_nonzero_received_at = None;
            return Action::Reject(reason);
        }
        let linear_x = command.linear[0];
        let angular_z = command.angular[2];
        let moving = linear_x.abs() > 1e-9 || angular_z.abs() > 1e-9;
        if !moving {
            self.nonzero_active = false;
            self.last_nonzero_received_at = None;
        } else {
            let refresh_freshness = match source_stamp_ns {
                None => true,
                Some(_) => timestamp_is_new,
            };
            self.nonzero_active = true;
            if refresh_freshness {
                self.last_nonzero_received_at = Some(mono);
            }
        }
        Action::Forward { linear_x, angular_z }
    }

    /// Steady-clock receipt timeout: returns true when a zero must be forced.
    pub fn watchdog(&mut self, mono: Instant) -> bool {
        let Some(received_at) = self.last_nonzero_received_at else { return false };
        if !self.nonzero_active || mono.duration_since(received_at).as_secs_f64() <= self.limits.command_timeout_s {
            return false;
        }
        self.nonzero_active = false;
        self.last_nonzero_received_at = None;
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    const NS: i64 = 1_000_000_000;

    fn limits(session: bool) -> Limits {
        Limits::new(0.20, 0.25, 0.05, 0.50, 1.00, session, 0.05).unwrap()
    }

    fn cmd(stamp_ns: i64, lin: f64, ang: f64) -> Command {
        Command {
            stamp_sec: (stamp_ns / NS) as i32,
            stamp_nanosec: (stamp_ns % NS) as u32,
            session_matches: true,
            linear: [lin, 0.0, 0.0],
            angular: [0.0, 0.0, ang],
        }
    }

    fn forward(lin: f64, ang: f64) -> Action {
        Action::Forward { linear_x: lin, angular_z: ang }
    }

    #[test]
    fn limits_are_clamped_and_must_be_finite() {
        let l = Limits::new(0.01, -1.0, -1.0, -2.0, -3.0, false, 0.001).unwrap();
        assert_eq!((l.command_timeout_s, l.max_input_age_s, l.max_future_skew_s), (0.05, 0.0, 0.0));
        assert_eq!((l.max_linear_x, l.max_angular_z, l.command_clock_period_s), (0.0, 0.0, 0.02));
        assert_eq!(l.watchdog_period_s(), 0.025);
        // as in Python, max(floor, nan) keeps the floor, so only +inf can
        // survive the clamp and is what the finiteness check rejects
        assert_eq!(Limits::new(f64::NAN, 0.25, 0.05, 0.5, 1.0, false, 0.05).unwrap().command_timeout_s, 0.05);
        assert!(Limits::new(f64::INFINITY, 0.25, 0.05, 0.5, 1.0, false, 0.05).is_err());
        assert!(Limits::new(0.2, 0.25, f64::INFINITY, 0.5, 1.0, false, 0.05).is_err());
        assert!(Limits::new(0.2, 0.25, 0.05, f64::INFINITY, 1.0, false, 0.05).is_err());
        assert_eq!(limits(false).watchdog_period_s(), 0.05);
    }

    #[test]
    fn zero_stamp_local_source_uses_receipt_time_and_watchdog() {
        let mut g = Guard::new(limits(false));
        let t0 = Instant::now();
        let now = 1_000 * NS;
        assert_eq!(g.on_command(&cmd(0, 0.3, 0.0), now, t0), forward(0.3, 0.0));
        // fresh again 100 ms later: still moving, watchdog quiet
        assert_eq!(g.on_command(&cmd(0, 0.3, 0.1), now, t0 + Duration::from_millis(100)), forward(0.3, 0.1));
        assert!(!g.watchdog(t0 + Duration::from_millis(250)));
        // 201 ms after the last nonzero: forced zero, once
        assert!(g.watchdog(t0 + Duration::from_millis(302)));
        assert!(!g.watchdog(t0 + Duration::from_millis(303)));
        // an explicit zero clears the activity, so the watchdog stays quiet
        assert_eq!(g.on_command(&cmd(0, 0.0, 0.0), now, t0 + Duration::from_millis(400)), forward(0.0, 0.0));
        assert!(!g.watchdog(t0 + Duration::from_secs(5)));
    }

    #[test]
    fn payload_rules_fail_closed() {
        let mut g = Guard::new(limits(false));
        let t0 = Instant::now();
        let now = 1_000 * NS;
        let mut nan = cmd(0, 0.1, 0.0);
        nan.angular[2] = f64::NAN;
        assert_eq!(g.on_command(&nan, now, t0), Action::Reject("velocity contains NaN or infinity"));
        let mut lateral = cmd(0, 0.1, 0.0);
        lateral.linear[1] = 0.2;
        assert_eq!(g.on_command(&lateral, now, t0), Action::Reject("unsupported lateral or non-yaw velocity component"));
        assert_eq!(g.on_command(&cmd(0, 0.51, 0.0), now, t0), Action::Reject("linear velocity exceeds robot safety limit"));
        assert_eq!(g.on_command(&cmd(0, -0.5, 0.0), now, t0), forward(-0.5, 0.0));
        assert_eq!(g.on_command(&cmd(0, 0.0, 1.01), now, t0), Action::Reject("angular velocity exceeds robot safety limit"));
        assert_eq!(g.on_command(&cmd(0, 0.0, -1.0), now, t0), forward(0.0, -1.0));
    }

    #[test]
    fn stamped_commands_must_be_fresh_ordered_and_not_from_the_future() {
        let mut g = Guard::new(limits(false));
        let t0 = Instant::now();
        let now = 1_000 * NS;
        // 251 ms old: stale
        assert_eq!(g.on_command(&cmd(now - 251_000_000, 0.2, 0.0), now, t0), Action::Reject("velocity timestamp is stale"));
        // 100 ms old: fine (and newer than the rejection barrier? no: the
        // rejection advanced the barrier to `now`, so this older stamp moved backward)
        assert_eq!(g.on_command(&cmd(now - 100_000_000, 0.2, 0.0), now, t0), Action::Reject("velocity source timestamp moved backward"));
        // stamps after the barrier flow again
        let later = now + 10_000_000;
        assert_eq!(g.on_command(&cmd(later, 0.2, 0.0), later, t0), forward(0.2, 0.0));
        // 60 ms in the future: too far (skew limit 50 ms)
        assert_eq!(g.on_command(&cmd(later + 60_000_000, 0.2, 0.0), later, t0), Action::Reject("velocity timestamp is too far in the future"));
        // 40 ms in the future is tolerated, but the rejection above moved the
        // barrier to `later`, and this stamp is newer than that: forwarded
        assert_eq!(g.on_command(&cmd(later + 40_000_000, 0.2, 0.0), later, t0), forward(0.2, 0.0));
        // same stamp replayed while motion is active and fresh: allowed
        assert_eq!(g.on_command(&cmd(later + 40_000_000, 0.2, 0.0), later, t0 + Duration::from_millis(100)), forward(0.2, 0.0));
        // replayed after the receipt timeout: cannot resume stopped motion
        assert!(g.watchdog(t0 + Duration::from_millis(250)));
        assert_eq!(
            g.on_command(&cmd(later + 40_000_000, 0.2, 0.0), later, t0 + Duration::from_millis(260)),
            Action::Reject("replayed velocity cannot resume stopped motion")
        );
    }

    #[test]
    fn replay_keeps_moving_only_while_fresh() {
        let mut g = Guard::new(limits(false));
        let t0 = Instant::now();
        let now = 1_000 * NS;
        assert_eq!(g.on_command(&cmd(now, 0.2, 0.0), now, t0), forward(0.2, 0.0));
        // replays do not refresh the receipt time...
        for ms in [50u64, 100, 150] {
            assert_eq!(g.on_command(&cmd(now, 0.2, 0.0), now + ms as i64 * 1_000_000, t0 + Duration::from_millis(ms)), forward(0.2, 0.0));
        }
        // ...so 201 ms after the original receipt the watchdog still fires
        assert!(g.watchdog(t0 + Duration::from_millis(201)));
        // and a newer stamp is needed to move again (the rejection moved the
        // barrier to the robot clock, so "newer" means after that instant)
        assert_eq!(
            g.on_command(&cmd(now, 0.2, 0.0), now + 210_000_000, t0 + Duration::from_millis(210)),
            Action::Reject("replayed velocity cannot resume stopped motion")
        );
        assert_eq!(
            g.on_command(&cmd(now + 205_000_000, 0.2, 0.0), now + 215_000_000, t0 + Duration::from_millis(215)),
            Action::Reject("velocity source timestamp moved backward")
        );
        // every rejection moves the barrier to the robot clock at that
        // moment (now + 215 ms), so the previously accepted stamp is old too
        assert_eq!(
            g.on_command(&cmd(now + 210_000_000, 0.2, 0.0), now + 216_000_000, t0 + Duration::from_millis(216)),
            Action::Reject("velocity source timestamp moved backward")
        );
        assert_eq!(g.on_command(&cmd(now + 220_000_000, 0.2, 0.0), now + 220_000_000, t0 + Duration::from_millis(220)), forward(0.2, 0.0));
    }

    #[test]
    fn manual_session_requires_matching_id_and_robot_clock() {
        let mut g = Guard::new(limits(true));
        let t0 = Instant::now();
        let now = 1_000 * NS;
        let mut wrong = cmd(now, 0.1, 0.0);
        wrong.session_matches = false;
        assert_eq!(g.on_command(&wrong, now, t0), Action::Reject("manual velocity command session is missing or stale"));
        // zero stamp is not acceptable on the untrusted path
        assert_eq!(g.on_command(&cmd(0, 0.1, 0.0), now, t0), Action::Reject("manual velocity requires the robot command clock"));
        // the rejection barrier is `now`; a command stamped after it passes
        assert_eq!(g.on_command(&cmd(now + 1, 0.1, 0.0), now + 1, t0), forward(0.1, 0.0));
    }

    #[test]
    fn rejection_is_a_stop_barrier_for_queued_older_motion() {
        let mut g = Guard::new(limits(false));
        let t0 = Instant::now();
        let now = 1_000 * NS;
        assert_eq!(g.on_command(&cmd(now - 50_000_000, 0.2, 0.0), now, t0), forward(0.2, 0.0));
        // overspeed with a newer stamp: rejected, barrier moves to the robot clock
        assert_eq!(g.on_command(&cmd(now - 10_000_000, 0.9, 0.0), now, t0), Action::Reject("linear velocity exceeds robot safety limit"));
        // an older valid command that was still queued must not resume motion
        assert_eq!(g.on_command(&cmd(now - 20_000_000, 0.2, 0.0), now, t0), Action::Reject("velocity source timestamp moved backward"));
        assert!(!g.watchdog(t0 + Duration::from_secs(1)));
    }
}
