//! `topic_tools throttle messages <in> <rate> <out>`, folded into the node that
//! publishes `<in>`.
//!
//! On the robot two C++ throttles exist only to hand the status nodes a slow
//! copy of a fast topic (`/odom` -> `/odom_slow`, `/odometry/global` ->
//! `/odometry/global_slow`, both 5 Hz; `mower_mission/launch/mission.launch.py`).
//! Each costs a process, a DDS participant and ~1.3 ms per *input* message on
//! the RK3568 (docs/ROS_FREE_PLAN.md section 1). When the source is a mower_rs
//! module, the module can publish the copy itself and the throttle goes away.
//!
//! What is reproduced, from topic_tools 1.3.4 (the version in the image,
//! `src/throttle_node.cpp` + `src/tool_base_node.cpp`):
//!
//! * **Which message.** The *first* message that arrives at least one period
//!   after the last forwarded one, forwarded unchanged. The period restarts at
//!   that arrival, not on a fixed grid, so a late forward shifts the phase and
//!   nothing is ever sent to catch up. When the source period divides the
//!   throttle period exactly (25 Hz `/odom`, 20 Hz `/odometry/global`), jitter
//!   decides between the n-th and the (n+1)-th message, so "5 Hz" comes out
//!   at 4.3-4.5 Hz — for topic_tools as much as for this (src/mower_rs/README.md,
//!   "The slow copies").
//! * **The period.** `rclcpp::Rate(msgs_per_sec).period()`, i.e.
//!   `static_cast<int64_t>(1.0 / rate * 1e9)` ns, compared with `>=`.
//! * **The clock.** Arrival time on the node clock (`this->now()`, since
//!   `use_wall_clock` defaults to false), never the header stamp. With
//!   `use_sim_time:=false` that is the system clock, which is the only clock
//!   the mower_rs modules that use this run on ([`system_now_ns`]). The
//!   "arrival" of an in-process copy is the moment the module publishes the
//!   source message.
//! * **The first message.** The reference time starts at construction, so a
//!   message within one period of start-up is dropped.
//! * **A clock that goes backwards** restarts the period at the new time and
//!   drops that message ("Detected jump back in time").
//! * **QoS** ([`output_qos`]): `rclcpp::QoS(10)` with the source publisher's
//!   reliability and durability and automatic liveliness, as the throttle
//!   *discovers* them on the graph. For a source created with
//!   `rclcpp::SystemDefaultsQoS()` that depends on the RMW: the robot's
//!   rmw_cyclonedds_cpp announces it reliable + volatile, keep last 1, where
//!   FastDDS (the test image) announces transient local. So a caller passes
//!   the source as the robot's throttle sees it, which need not be the QoS
//!   of its own source publisher (mower_base's `/odom`).
//!
//! Not reproduced: `lazy` (off in both launch entries, so topic_tools
//! publishes whether or not anyone listens, as this does), the `bytes` mode,
//! and the runtime QoS-override parameters nobody sets.

use std::time::{SystemTime, UNIX_EPOCH};

use r2r::qos::LivelinessPolicy;
use r2r::{QosProfile, WrappedTypesupport};

/// What [`MessageThrottle::admit`] decided about one message.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Verdict {
    /// Forward it; the period restarts at its arrival.
    Forward,
    /// Drop it: less than one period since the last forwarded message.
    Drop,
    /// Drop it: the clock went backwards, so the period restarts now.
    ClockWentBack,
}

/// The `messages` gate of `topic_tools::ThrottleNode`, without the transport.
#[derive(Clone, Debug)]
pub struct MessageThrottle {
    period_ns: i64,
    last_ns: i64,
}

impl MessageThrottle {
    /// A gate at `msgs_per_sec`, whose reference time is `now_ns` (the
    /// throttle's construction). `None` for a rate that is not a positive
    /// finite number: topic_tools throws on `rate <= 0`, and a module is
    /// better off without its slow copy than failing the fast topic with it.
    pub fn new(msgs_per_sec: f64, now_ns: i64) -> Option<Self> {
        if !(msgs_per_sec.is_finite() && msgs_per_sec > 0.0) {
            return None;
        }
        Some(MessageThrottle { period_ns: period_ns(msgs_per_sec), last_ns: now_ns })
    }

    pub fn period_ns(&self) -> i64 {
        self.period_ns
    }

    /// `ThrottleNode::process_message` for a message that arrives at `now_ns`.
    pub fn admit(&mut self, now_ns: i64) -> Verdict {
        if self.last_ns > now_ns {
            self.last_ns = now_ns;
            // now - last is 0 here, which only a zero period would pass.
            return if self.period_ns <= 0 { Verdict::Forward } else { Verdict::ClockWentBack };
        }
        if now_ns - self.last_ns >= self.period_ns {
            self.last_ns = now_ns;
            Verdict::Forward
        } else {
            Verdict::Drop
        }
    }
}

/// `rclcpp::Rate(rate).period()`: `Duration::from_seconds(1.0 / rate)`, which
/// is `static_cast<int64_t>(RCL_S_TO_NS(seconds))` -- a truncation.
pub fn period_ns(msgs_per_sec: f64) -> i64 {
    (1.0 / msgs_per_sec * 1_000_000_000.0) as i64
}

/// The node clock of a node with `use_sim_time:=false`: `RCL_ROS_TIME` falls
/// back to the system clock (`CLOCK_REALTIME`), which can be stepped, which is
/// why [`MessageThrottle`] handles a clock that goes backwards.
pub fn system_now_ns() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as i64)
        .unwrap_or(0)
}

/// The QoS topic_tools publishes its output with, given the QoS of the one
/// publisher on the source topic (`ToolBaseNode::try_discover_source`):
/// keep last 10, the source's reliability and durability, automatic
/// liveliness. Deadline and lifespan are the source's, which here are the
/// defaults. With a single source there is none of the mixed-publisher
/// fallback to best effort / volatile.
pub fn output_qos(source: &QosProfile) -> QosProfile {
    QosProfile::default()
        .keep_last(10)
        .reliability(source.reliability.clone())
        .durability(source.durability.clone())
        .liveliness(LivelinessPolicy::Automatic)
}

/// A publisher of the throttled copy, fed by the module with every message it
/// publishes on the source topic.
pub struct SlowCopy<T: WrappedTypesupport + 'static> {
    publisher: r2r::Publisher<T>,
    gate: MessageThrottle,
    topic: String,
    logger: String,
}

impl<T: WrappedTypesupport + 'static> SlowCopy<T> {
    /// `None` when `topic` is empty (the switch that turns the copy off), the
    /// rate is not usable or the publisher cannot be created, e.g. for a
    /// topic name rcl refuses (both logged). Never an error: a typo in the
    /// yaml must cost the slow copy, not the module's fast topic with it.
    /// `source_qos` is the source as the throttle this replaces discovers it
    /// on the robot (see the module doc, "QoS").
    pub fn create(
        node: &mut r2r::Node,
        topic: &str,
        msgs_per_sec: f64,
        source_qos: &QosProfile,
    ) -> Option<Self> {
        let logger = node.logger().to_string();
        if topic.is_empty() {
            return None;
        }
        let Some(gate) = MessageThrottle::new(msgs_per_sec, system_now_ns()) else {
            r2r::log_warn!(
                &logger,
                "{topic}: rate {msgs_per_sec} is not a positive number; not publishing it"
            );
            return None;
        };
        let publisher = match node.create_publisher::<T>(topic, output_qos(source_qos)) {
            Ok(publisher) => publisher,
            Err(e) => {
                r2r::log_error!(
                    &logger,
                    "{topic}: cannot create the publisher ({e:?}); not publishing it"
                );
                return None;
            }
        };
        Some(SlowCopy { publisher, gate, topic: topic.to_string(), logger })
    }

    /// Offer the message the module has just published on the source topic.
    /// It goes out unchanged if the period has elapsed since the last one.
    pub fn offer(&mut self, msg: &T) {
        match self.gate.admit(system_now_ns()) {
            Verdict::Forward => {
                if let Err(e) = self.publisher.publish(msg) {
                    r2r::log_error!(&self.logger, "publish {} failed: {:?}", self.topic, e);
                }
            }
            Verdict::Drop => {}
            Verdict::ClockWentBack => r2r::log_warn!(
                &self.logger,
                "{}: detected jump back in time, resetting throttle period to now",
                self.topic
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: i64 = 1_000_000;
    /// An arbitrary epoch-scale start, like a real `CLOCK_REALTIME` reading.
    const T0: i64 = 1_790_000_000 * 1_000_000_000;

    /// The forwarded arrivals of a source that fires at `arrivals`.
    fn forwarded(rate: f64, start: i64, arrivals: impl IntoIterator<Item = i64>) -> Vec<i64> {
        let mut gate = MessageThrottle::new(rate, start).expect("rate");
        arrivals.into_iter().filter(|&t| gate.admit(t) == Verdict::Forward).collect()
    }

    #[test]
    fn the_period_is_rclcpp_rate_period_truncated_to_nanoseconds() {
        assert_eq!(period_ns(5.0), 200_000_000);
        assert_eq!(period_ns(3.0), 333_333_333);
        assert_eq!(period_ns(30.0), 33_333_333);
        assert_eq!(period_ns(0.3), 3_333_333_333);
        assert_eq!(MessageThrottle::new(5.0, 0).unwrap().period_ns(), 200 * MS);
    }

    #[test]
    fn a_rate_topic_tools_would_refuse_turns_the_copy_off() {
        for rate in [0.0, -5.0, f64::NAN, f64::INFINITY] {
            assert!(MessageThrottle::new(rate, T0).is_none(), "{rate}");
        }
    }

    /// `last_time_` starts at construction, not at the first message: a
    /// message within one period of start-up is dropped.
    #[test]
    fn the_first_message_waits_one_period_from_start() {
        let mut gate = MessageThrottle::new(5.0, T0).unwrap();
        assert_eq!(gate.admit(T0), Verdict::Drop);
        assert_eq!(gate.admit(T0 + MS), Verdict::Drop);
        assert_eq!(gate.admit(T0 + 200 * MS - 1), Verdict::Drop);
        assert_eq!(gate.admit(T0 + 200 * MS), Verdict::Forward, ">= period, not >");
        // a source that starts long after the gate: its first message goes
        let mut late = MessageThrottle::new(5.0, T0).unwrap();
        assert_eq!(late.admit(T0 + 3_000 * MS), Verdict::Forward);
        assert_eq!(late.admit(T0 + 3_040 * MS), Verdict::Drop);
    }

    #[test]
    fn a_steady_source_is_divided_exactly() {
        // /odom at 25 Hz for 10 s: every fifth message, 5.0 Hz. The one at
        // t = 0 is inside the first period, so 49 of the 250 go, from 0.2 s.
        let odom = forwarded(5.0, T0, (0..250).map(|k| T0 + k * 40 * MS));
        assert_eq!(odom.len(), 49);
        assert_eq!(odom[0], T0 + 200 * MS);
        assert!(odom.windows(2).all(|w| w[1] - w[0] == 200 * MS));
        // /odometry/global at 20 Hz: every fourth.
        let global = forwarded(5.0, T0, (0..200).map(|k| T0 + k * 50 * MS));
        assert_eq!(global.len(), 49);
        assert_eq!(global[0], T0 + 200 * MS);
        assert!(global.windows(2).all(|w| w[1] - w[0] == 200 * MS));
    }

    /// The period is measured from the last forwarded *arrival*, so when the
    /// source period divides it exactly, arrival jitter decides whether the
    /// fifth or the sixth /odom message goes, and the rate comes out below
    /// 5 Hz. topic_tools does the same with its receive times; nothing here
    /// catches up.
    #[test]
    fn arrival_jitter_on_an_exact_divisor_skips_to_the_next_message() {
        // 25 Hz with every odd message 0.1 ms late. The first forward is
        // message 5 (late); five messages on is an on-time one, 0.1 ms short
        // of the period, so the sixth goes instead -- every time.
        let late = |k: i64| if k % 2 == 1 { MS / 10 } else { 0 };
        let got = forwarded(5.0, T0, (0..250).map(|k| T0 + k * 40 * MS + late(k)));
        assert_eq!(got[0], T0 + 200 * MS + MS / 10);
        assert!(got.windows(2).all(|w| w[1] - w[0] == 240 * MS));
        assert_eq!(got.len(), 41, "4.17 Hz, not 5");
    }

    #[test]
    fn a_late_forward_shifts_the_phase_instead_of_catching_up() {
        let mut gate = MessageThrottle::new(5.0, T0).unwrap();
        assert_eq!(gate.admit(T0 + 200 * MS), Verdict::Forward);
        assert_eq!(gate.admit(T0 + 399 * MS), Verdict::Drop);
        assert_eq!(gate.admit(T0 + 440 * MS), Verdict::Forward);
        // the next one is due at 640, not at 600
        assert_eq!(gate.admit(T0 + 600 * MS), Verdict::Drop);
        assert_eq!(gate.admit(T0 + 639 * MS), Verdict::Drop);
        assert_eq!(gate.admit(T0 + 640 * MS), Verdict::Forward);
    }

    #[test]
    fn a_clock_that_goes_back_restarts_the_period() {
        let mut gate = MessageThrottle::new(5.0, T0).unwrap();
        assert_eq!(gate.admit(T0 + 1_000 * MS), Verdict::Forward);
        // NTP steps the clock back half a second
        assert_eq!(gate.admit(T0 + 500 * MS), Verdict::ClockWentBack);
        assert_eq!(gate.admit(T0 + 650 * MS), Verdict::Drop);
        assert_eq!(gate.admit(T0 + 700 * MS), Verdict::Forward);
        // a forward step is just a long gap
        assert_eq!(gate.admit(T0 + 60_000 * MS), Verdict::Forward);
    }

    /// The gate runs on the system clock, as topic_tools' node clock does with
    /// `use_sim_time:=false`: an epoch-based reading, not a monotonic one.
    #[test]
    fn the_clock_is_the_system_clock() {
        let before = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_nanos() as i64;
        let now = system_now_ns();
        let after = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_nanos() as i64;
        assert!(before <= now && now <= after);
        assert!(now > 1_700_000_000 * 1_000_000_000, "epoch time, not uptime");
    }

    #[test]
    fn the_output_qos_is_topic_tools_qos_10_with_the_source_policies() {
        // a latched source (FastDDS's reading of SystemDefaultsQoS):
        // reliable + transient local, depth 1
        let latched = QosProfile::default().keep_last(1).transient_local().reliable();
        assert_eq!(
            output_qos(&latched),
            QosProfile::default()
                .keep_last(10)
                .reliable()
                .transient_local()
                .liveliness(LivelinessPolicy::Automatic)
        );
        // mower_localize's /odometry/global: rclcpp::QoS(10), reliable, volatile
        let plain = QosProfile::default().keep_last(10).reliable().volatile();
        assert_eq!(
            output_qos(&plain),
            QosProfile::default()
                .keep_last(10)
                .reliable()
                .volatile()
                .liveliness(LivelinessPolicy::Automatic)
        );
        // a best-effort source gives a best-effort copy
        assert_eq!(
            output_qos(&QosProfile::sensor_data()).reliability,
            r2r::qos::ReliabilityPolicy::BestEffort
        );
    }

    /// A slow-copy topic rcl refuses disables the copy instead of failing
    /// the module that publishes the fast topic. Needs a sourced ROS
    /// environment (as every r2r test binary does to link).
    #[test]
    fn a_topic_rcl_refuses_turns_the_copy_off() {
        use r2r::nav_msgs::msg::Odometry;
        let ctx = r2r::Context::create().expect("rcl context");
        let mut node = r2r::Node::create(ctx, "slow_copy_test", "").expect("node");
        let qos = QosProfile::default().keep_last(10).reliable().volatile();
        for bad in ["/odom_slow/", "odometry/global slow", "/odom//slow"] {
            // an rcl error on its own, the thing create() must not pass on
            assert!(node.create_publisher::<Odometry>(bad, qos.clone()).is_err(), "{bad}");
            assert!(SlowCopy::<Odometry>::create(&mut node, bad, 5.0, &qos).is_none(), "{bad}");
        }
        assert!(SlowCopy::<Odometry>::create(&mut node, "", 5.0, &qos).is_none());
        assert!(SlowCopy::<Odometry>::create(&mut node, "/odom_slow", 0.0, &qos).is_none());
        let mut ok = SlowCopy::<Odometry>::create(&mut node, "/odom_slow", 5.0, &qos)
            .expect("a valid topic and rate");
        ok.offer(&Odometry::default());
        let relative = SlowCopy::<Odometry>::create(&mut node, "odometry/global_slow", 5.0, &qos);
        assert!(relative.is_some());
    }
}
