//! The behaviour `mower_base` adds on top of the C++ chain, or keeps apart
//! from it: the arm latch, the control clock vs. ROS time, and the cmd_vel
//! stamp handling at the cycle level. (The driver's own clock wiring, which
//! clock feeds which argument, is tested in `mower_base`.)
//!
//! Everything runs with the production controller parameters
//! (`mower_controller/controllers/diff_drive_controller.yaml`: open loop,
//! 0.25 s timeout, -25 / -50 deceleration), which is what the robot runs.

use mower_base_core::cycle::{
    BaseConfig, BaseCycle, DisarmReason, Event, ResetSignature, StopEdge, Stream, TxFrame,
    TIMEOUT_REPORTS_TO_DISARM,
};
use mower_base_core::diff_drive::{
    receive_command, Command, DiffDriveParams, LimitParams, OdomSample, Received, Twist,
};
use mower_base_core::protocol::{
    build_frame, MOTOR_STATUS, STATUS_FLAG_COMMAND_TIMEOUT, STATUS_FLAG_COMMAND_VALID,
    STATUS_FLAG_DRIVER_ALARM, WHEEL_FEEDBACK_STATUS,
};

const MS: i64 = 1_000_000;
const DT: i64 = 40 * MS;
/// `latch_release_s` (0.5 s) at 25 Hz: a stream at rest re-arms on the
/// 14th cycle counting the one that saw its first stop (13 * 40 ms = 0.52 s
/// after it; 0.48 s is not yet the window).
const RELEASE_CYCLES: usize = 14;
/// A day of uptime: the control clock is CLOCK_MONOTONIC, never 0.
const T0: i64 = 86_400_000 * MS;
/// ROS time is epoch-based and unrelated to the control clock.
const ROS0: i64 = 1_790_000_000_000_000_000;

fn production() -> BaseConfig {
    let axis = |v: f64, a: f64, d: f64| LimitParams {
        has_velocity_limits: true,
        has_acceleration_limits: true,
        has_jerk_limits: true,
        min_velocity: -v,
        max_velocity: v,
        max_acceleration: a,
        max_acceleration_reverse: -a,
        max_deceleration: -d,
        max_deceleration_reverse: d,
        min_jerk: f64::NAN,
        max_jerk: f64::NAN,
    };
    BaseConfig {
        diff_drive: DiffDriveParams {
            wheel_separation: 0.35,
            wheel_radius: 0.09,
            cmd_vel_timeout: 0.25,
            open_loop: true,
            enable_odom_tf: false,
            base_frame_id: "base_footprint".to_string(),
            pose_covariance_diagonal: [0.0; 6],
            twist_covariance_diagonal: [0.0; 6],
            linear: axis(0.5, 1.0, 25.0),
            angular: axis(1.0, 2.0, 50.0),
            ..DiffDriveParams::mower()
        },
        ..BaseConfig::default()
    }
}

/// `(left, right)` permille of the 0x01 in a cycle's bytes.
fn wheels(tx: &TxFrame) -> (i16, i16) {
    let b = &tx.bytes;
    let mut o = 0;
    while o + 6 <= b.len() {
        let len = b[o + 5] as usize;
        if b[o + 3] == 0x01 {
            let p = &b[o + 6..o + 6 + len];
            return (i16::from_le_bytes([p[0], p[1]]), i16::from_le_bytes([p[2], p[3]]));
        }
        o += 6 + len + 2;
    }
    panic!("no 0x01 in {:?}", tx.frames);
}

/// The blade permille of the 0x02 in a cycle's bytes, if there is one.
fn blade_of(tx: &TxFrame) -> Option<i16> {
    let b = &tx.bytes;
    let mut o = 0;
    while o + 6 <= b.len() {
        let len = b[o + 5] as usize;
        if b[o + 3] == 0x02 {
            let p = &b[o + 6..o + 6 + len];
            return Some(i16::from_le_bytes([p[0], p[1]]));
        }
        o += 6 + len + 2;
    }
    None
}

fn feedback(counts: i32) -> Vec<u8> {
    let mut p = [0u8; 24];
    p[12..16].copy_from_slice(&counts.to_le_bytes());
    p[16..20].copy_from_slice(&counts.to_le_bytes());
    build_frame(WHEEL_FEEDBACK_STATUS, 0, &p)
}

/// A base plus a clock, so a test reads as a script.
struct Rig {
    base: BaseCycle,
    t: i64,
    events: Vec<Event>,
    last_odom: Option<OdomSample>,
    /// What the driver reads off the ROS graph each cycle, for both streams:
    /// a publisher is there (the default, from the first cycle on).
    publishers: bool,
    /// the bytes of the last cycle
    last_tx: TxFrame,
}

impl Rig {
    fn new(cfg: BaseConfig) -> Self {
        let (base, _) = BaseCycle::new(cfg, T0).unwrap();
        Rig {
            base,
            t: T0,
            events: Vec::new(),
            last_odom: None,
            publishers: true,
            last_tx: TxFrame::default(),
        }
    }
    /// One cycle 40 ms later with `cmd` (fresh), optionally after a 0x85.
    fn step(&mut self, cmd: Option<(f64, f64)>, with_feedback: bool) -> (i16, i16) {
        self.t += DT;
        if with_feedback {
            self.base.on_rx(&feedback(0), self.t);
        }
        self.base.set_publishers(Stream::CmdVel, self.publishers, self.t);
        self.base.set_publishers(Stream::WheelOverride, self.publishers, self.t);
        let cmd = cmd.map(|(l, a)| Command { twist: Twist::new(l, a), stamp_ns: self.t });
        let (tx, odom, _) = self.base.tick(cmd, self.t, self.t - T0 + ROS0);
        if odom.is_some() {
            self.last_odom = odom;
        }
        self.events.extend(self.base.take_events());
        wheels(&tx.unwrap())
    }
    fn run(&mut self, n: usize, cmd: Option<(f64, f64)>, with_feedback: bool) -> Vec<(i16, i16)> {
        (0..n).map(|_| self.step(cmd, with_feedback)).collect()
    }
    fn armed(&self, stream: Stream) -> Option<StopEdge> {
        self.events.iter().rev().find_map(|e| match e {
            Event::Armed { stream: s, by, .. } if *s == stream => Some(*by),
            _ => None,
        })
    }
    /// The stick released and left at rest: a zero every cycle until
    /// cmd_vel re-arms. Returns how many cycles that took, the first zero's
    /// included: [`RELEASE_CYCLES`] when nothing else holds it.
    fn release(&mut self, with_feedback: bool) -> usize {
        for n in 1..=RELEASE_CYCLES + 50 {
            assert_eq!(self.step(Some((0.0, 0.0)), with_feedback), (0, 0));
            if !self.base.holds(Stream::CmdVel) {
                assert_eq!(self.armed(Stream::CmdVel), Some(StopEdge::Stop));
                return n;
            }
        }
        panic!("cmd_vel not re-armed by 2.5 s of zeros: {:?}", self.events);
    }
    /// The `ReleaseNotHeld` events so far, as `(stream, by, after_s)`.
    fn not_held(&self) -> Vec<(Stream, StopEdge, f64)> {
        self.events
            .iter()
            .filter_map(|e| match e {
                Event::ReleaseNotHeld { stream, by, window_s, after_s } => {
                    assert!((window_s - 0.5).abs() < 1e-6, "{e:?}");
                    Some((*stream, *by, *after_s))
                }
                _ => None,
            })
            .collect()
    }
    /// Seconds since activation.
    fn elapsed(&self) -> f64 {
        (self.t - T0) as f64 / 1e9
    }
}

fn all_zero(frames: &[(i16, i16)]) -> bool {
    frames.iter().all(|w| *w == (0, 0))
}

// ---- the arm latch -------------------------------------------------------

/// A module restart under a live command: the new cycle must not follow it
/// until the stream shows a stop edge — here explicit zeros, held for the
/// release window.
#[test]
fn activation_holds_a_live_command_until_a_zero() {
    let mut rig = Rig::new(production());
    assert_eq!(rig.base.disarmed(), Some(DisarmReason::Activation));
    assert_eq!(rig.base.take_events(), vec![Event::Disarmed(DisarmReason::Activation)]);

    // nav2 / teleop still streaming 0.4 m/s, 2 s long: nothing moves
    let held = rig.run(50, Some((0.4, 0.2)), true);
    assert!(all_zero(&held), "{held:?}");
    assert!(rig.base.holds(Stream::CmdVel));
    assert_eq!(rig.armed(Stream::CmdVel), None);
    // the odometry agrees with the wheels: no invented motion
    let o = rig.last_odom.clone().expect("/odom runs while held");
    assert_eq!((o.x, o.y, o.linear_x), (0.0, 0.0, 0.0));

    // the stop edge: zeros for 0.48 s are not yet the release window,
    // the 14th cycle (0.52 s after the first) is; then a new command moves
    // the wheels at the normal ramp
    let zeros = rig.run(RELEASE_CYCLES - 1, Some((0.0, 0.0)), true);
    assert!(all_zero(&zeros));
    assert!(rig.base.holds(Stream::CmdVel), "{:?}", rig.events);
    rig.step(Some((0.0, 0.0)), true);
    assert_eq!(rig.armed(Stream::CmdVel), Some(StopEdge::Stop));
    assert!(!rig.base.holds(Stream::CmdVel));
    let moving = rig.run(25, Some((0.4, 0.0)), true);
    assert_eq!(moving[0], (73, 73), "first cycle: 1 m/s^2 * 40 ms = 0.04 m/s");
    assert!(moving.last().unwrap().0 > 600);

    // B6: the pose moved by what was commanded since activation, not by
    // the command times the clock reading
    let x = rig.last_odom.unwrap().x;
    assert!(x > 0.0 && x < 0.4, "open-loop pose after 1 s of ramp: {x} m");
}

/// Silence for longer than the release window (and than cmd_vel_timeout)
/// is the other stop edge, counted once the publisher has been there for
/// `publisher_settle_s`: here it is seen on the first cycle (40 ms), so
/// silence runs from 2.04 s.
#[test]
fn activation_rearms_after_silence() {
    let mut rig = Rig::new(production());
    let frames = rig.run(63, None, true); // 2.52 s
    assert!(all_zero(&frames));
    assert!(rig.base.holds(Stream::CmdVel), "2.52 - 2.04 s is not yet > 0.50 s");
    rig.step(None, true); // 2.56 s
    assert_eq!(rig.armed(Stream::CmdVel), Some(StopEdge::Silence));
    assert_eq!(rig.armed(Stream::WheelOverride), Some(StopEdge::Silence));
    assert_eq!(rig.base.disarmed(), None);
    let moving = rig.run(3, Some((0.3, 0.0)), true);
    assert_eq!(moving[0], (73, 73));
}

/// Finding: a restarted node's reader hears nothing from a live nav2 or
/// teleop writer until DDS discovery has matched them, which on a loaded
/// RK3568 has taken 2.7 s. That quiet stretch is not silence: whenever the
/// publisher shows up in the graph and its non-zero stream starts arriving
/// (writer-side matching can lag the graph by a while more), the wheels
/// stay held until the stream itself stops.
#[test]
fn discovery_latency_is_not_silence() {
    // (graph shows the publisher, first message arrives), s after activation
    for (seen_s, first_s) in [(0.3, 0.3), (0.3, 0.5), (1.0, 2.9), (2.7, 2.9), (3.0, 3.0), (2.7, 4.6)] {
        let mut rig = Rig::new(production());
        rig.publishers = false;
        let mut frames = Vec::new();
        while rig.elapsed() < 8.0 {
            let t = rig.elapsed() + 0.04;
            rig.publishers = t >= seen_s - 1e-9;
            let cmd = (t >= first_s - 1e-9).then_some((0.3, 0.0));
            frames.push(rig.step(cmd, true));
        }
        assert!(all_zero(&frames), "seen {seen_s} s, first message {first_s} s: {frames:?}");
        assert!(rig.base.holds(Stream::CmdVel), "seen {seen_s} s, first {first_s} s");
        let seen = rig.events.iter().find_map(|e| match e {
            Event::Publishers { stream: Stream::CmdVel, present: true, after_activation_s } => {
                Some(*after_activation_s)
            }
            _ => None,
        });
        assert!(seen.is_some_and(|s| (s - seen_s).abs() < 0.041), "{:?}", rig.events);

        // the stream stops: re-armed by the zeros, then it drives
        assert_eq!(rig.release(true), RELEASE_CYCLES);
        assert_eq!(rig.step(Some((0.3, 0.0)), true), (73, 73));
    }
}

/// With no publisher in the graph and nothing heard, there is nothing to
/// time: only an explicit stop re-arms. Once a message has arrived, the
/// silence after it counts whether or not the graph shows its writer.
#[test]
fn no_publisher_and_no_message_is_not_silence() {
    let mut rig = Rig::new(production());
    rig.publishers = false;
    let frames = rig.run(250, None, true); // 10 s
    assert!(all_zero(&frames));
    assert!(rig.base.holds(Stream::CmdVel) && rig.base.holds(Stream::WheelOverride));

    // one non-zero message from a writer the graph has not caught up with,
    // then nothing: re-armed 0.5 s after it (the release window, longer
    // than the 0.25 s cmd_vel_timeout)
    rig.step(Some((0.3, 0.0)), true);
    let quiet = rig.run(12, None, true);
    assert!(all_zero(&quiet));
    assert!(rig.base.holds(Stream::CmdVel));
    rig.step(None, true);
    assert_eq!(rig.armed(Stream::CmdVel), Some(StopEdge::Silence));
    assert!(rig.base.holds(Stream::WheelOverride), "the override stream is still unheard");
}

/// A non-zero after a gap shorter than the timeout does not count as
/// silence.
#[test]
fn a_gap_shorter_than_the_timeout_is_not_a_stop_edge() {
    let mut rig = Rig::new(production());
    rig.run(3, Some((0.4, 0.0)), true);
    rig.run(4, None, true); // 160 ms of nothing
    let held = rig.run(10, Some((0.4, 0.0)), true);
    assert!(all_zero(&held), "{held:?}");
    assert!(rig.base.holds(Stream::CmdVel));
}

/// Feedback that was there and stops: the wheels are braked and stay held,
/// even after the feedback comes back, until the stream shows a stop edge.
#[test]
fn feedback_loss_after_presence_disarms() {
    let mut rig = Rig::new(production());
    rig.release(true);
    // (the idle override stream re-arms by silence at 2.56 s)
    let moving = rig.run(55, Some((0.4, 0.0)), true);
    assert!(moving.last().unwrap().0 > 600);
    assert_eq!(rig.base.disarmed(), None);
    rig.events.clear();

    // UART lead pulled: 0x85 stops, the stick is still held
    let lost = rig.run(20, Some((0.4, 0.0)), false);
    // 0.5 s timeout: cycles 1..=12 are at most 480 ms without a frame
    assert!(lost[..12].iter().all(|w| w.0 > 600), "{lost:?}");
    assert_eq!(lost[12], (0, 0), "braked on the cycle the loss is seen (-25 m/s^2)");
    assert!(all_zero(&lost[12..]));
    assert!(matches!(rig.events[0], Event::FeedbackLost { age_s } if (age_s - 0.52).abs() < 1e-9));
    assert_eq!(rig.events[1], Event::Disarmed(DisarmReason::FeedbackLost));

    // lead re-seated with the stick still held: no lurch
    let back = rig.run(20, Some((0.4, 0.0)), true);
    assert!(all_zero(&back), "{back:?}");
    assert!(rig.events.contains(&Event::FeedbackResumed));
    assert!(rig.base.holds(Stream::CmdVel));

    // release, push again
    assert_eq!(rig.release(true), RELEASE_CYCLES);
    let again = rig.run(5, Some((0.4, 0.0)), true);
    assert_eq!(again[0], (73, 73));
}

/// Finding: the lead comes out while the robot is idle, the operator (or a
/// nav2 goal) pushes while it is still out, and the lead is re-seated. Nothing
/// re-arms while the feedback is missing, so the command that started during
/// the outage never ramps up against wheels that cannot move, and on the
/// re-seat it is still held: the wheels see 0/0 until a stop edge. The same
/// for a wheel_override requested during the outage.
#[test]
fn a_command_given_during_the_outage_waits_for_a_stop() {
    let mut rig = Rig::new(production());
    rig.step(Some((0.0, 0.0)), true);
    rig.run(70, None, true); // the override stream too, by silence
    assert_eq!(rig.base.disarmed(), None);
    rig.events.clear();

    // idle pull: disarmed at 0.52 s, and 1 s of silence does not re-arm
    let idle = rig.run(38, None, false);
    assert!(all_zero(&idle));
    assert!(rig.events.contains(&Event::Disarmed(DisarmReason::FeedbackLost)));
    assert!(rig.base.holds(Stream::CmdVel) && rig.base.holds(Stream::WheelOverride));
    assert_eq!(rig.armed(Stream::CmdVel), None);

    // pushed with the lead out: nothing moves, the odometry does not wander
    let x0 = rig.last_odom.clone().unwrap().x;
    let pushed = rig.run(25, Some((0.4, 0.0)), false);
    assert!(all_zero(&pushed), "{pushed:?}");
    assert_eq!(rig.last_odom.clone().unwrap().x, x0);
    assert_eq!(rig.last_odom.clone().unwrap().linear_x, 0.0);

    // re-seated, still pushed: held, no step to full speed
    let back = rig.run(25, Some((0.4, 0.0)), true);
    assert!(all_zero(&back), "{back:?}");
    assert!(rig.events.contains(&Event::FeedbackResumed));
    assert!(rig.base.holds(Stream::CmdVel));

    // released, pushed again: the normal ramp
    assert_eq!(rig.release(true), RELEASE_CYCLES);
    assert_eq!(rig.step(Some((0.4, 0.0)), true), (73, 73));

    // the same through wheel_override: requested during an outage, still
    // streaming on the re-seat, not applied until cancelled (and left
    // cancelled for the release window)
    rig.run(20, Some((0.0, 0.0)), true);
    rig.run(20, None, false);
    for _ in 0..20 {
        rig.base.request_wheel_override(800, 800, 300, rig.t);
        assert_eq!(rig.step(None, false), (0, 0));
    }
    for _ in 0..20 {
        rig.base.request_wheel_override(800, 800, 300, rig.t);
        assert_eq!(rig.step(None, true), (0, 0), "re-seated, override still streaming");
    }
    rig.base.request_wheel_override(0, 0, 0, rig.t);
    rig.run(RELEASE_CYCLES - 2, None, true);
    assert!(rig.base.holds(Stream::WheelOverride), "0.48 s after the cancel");
    rig.step(None, true);
    assert_eq!(rig.armed(Stream::WheelOverride), Some(StopEdge::Stop));
    rig.base.request_wheel_override(800, 800, 300, rig.t);
    assert_eq!(rig.step(None, true), (800, 800));
}

/// An outage that ends with the streams at rest re-arms on the re-seat
/// cycle itself, not during the outage.
#[test]
fn an_idle_outage_rearms_when_the_lead_is_back() {
    let mut rig = Rig::new(production());
    rig.step(Some((0.0, 0.0)), true);
    rig.run(10, None, true);
    rig.run(60, None, false); // 2.4 s out, idle
    assert!(rig.base.holds(Stream::CmdVel));
    rig.events.clear();
    rig.step(None, true);
    assert_eq!(rig.events[0], Event::FeedbackResumed);
    assert_eq!(rig.armed(Stream::CmdVel), Some(StopEdge::Silence));
    assert_eq!(rig.armed(Stream::WheelOverride), Some(StopEdge::Silence));
    assert_eq!(rig.step(Some((0.4, 0.0)), true), (73, 73));
}

/// A board that never sends 0x85 must still drive (not bricked), and says so
/// once in the log.
#[test]
fn feedback_never_seen_does_not_disarm() {
    let mut rig = Rig::new(production());
    rig.release(false);
    let frames = rig.run(75, Some((0.4, 0.0)), false); // 3 s without a frame
    assert!(frames.last().unwrap().0 > 600, "{:?}", frames.last());
    assert!(!rig.base.holds(Stream::CmdVel));
    let reports: Vec<_> = rig
        .events
        .iter()
        .filter(|e| matches!(e, Event::NoFeedbackSinceActivation { .. }))
        .collect();
    assert_eq!(reports.len(), 1, "{:?}", rig.events);
    assert!(!rig.events.contains(&Event::Disarmed(DisarmReason::FeedbackLost)));
}

/// pid_autotune's raw permille bypasses the controller, so it has its own
/// half of the latch: not applied while held, dropped on a disarm, and
/// re-armed only by its own stop edge.
#[test]
fn wheel_override_obeys_the_latch() {
    let mut rig = Rig::new(production());
    rig.release(true); // cmd_vel armed; the override is not

    // an override stream that was running before the restart
    let mut held = Vec::new();
    for _ in 0..20 {
        rig.base.request_wheel_override(400, -400, 300, rig.t);
        held.push(rig.step(None, true));
    }
    assert!(all_zero(&held), "{held:?}");
    assert!(rig.base.holds(Stream::WheelOverride));

    // the stream stops; once it has been quiet for the release window (and
    // its publisher has been known for 2 s) it is armed, and a new request
    // applies
    rig.run(6, None, true);
    assert!(rig.base.holds(Stream::WheelOverride), "publisher seen 0.04 s, settles at 2.04 s");
    rig.run(40, None, true);
    assert_eq!(rig.armed(Stream::WheelOverride), Some(StopEdge::Silence));
    rig.base.request_wheel_override(400, -400, 300, rig.t);
    assert_eq!(rig.step(None, true), (400, -400));
    assert!(rig.events.contains(&Event::Override { active: true }));

    // feedback lost mid-override: dropped at once, and a still-streaming
    // override is not applied again
    let mut lost = Vec::new();
    for _ in 0..20 {
        rig.base.request_wheel_override(400, -400, 300, rig.t);
        lost.push(rig.step(None, false));
    }
    let first_zero = lost.iter().position(|w| *w == (0, 0)).unwrap();
    assert!(all_zero(&lost[first_zero..]), "{lost:?}");
    assert!(rig.base.holds(Stream::WheelOverride));

    // an explicit cancel (ttl 0) is a stop edge too, held for the window
    rig.base.request_wheel_override(0, 0, 0, rig.t);
    rig.run(RELEASE_CYCLES - 2, None, true);
    assert!(rig.base.holds(Stream::WheelOverride));
    rig.step(None, true);
    assert_eq!(rig.armed(Stream::WheelOverride), Some(StopEdge::Stop));
    rig.base.request_wheel_override(-200, 200, 300, rig.t);
    assert_eq!(rig.step(None, true), (-200, 200));
}

/// Parity mode: with the latch off the cycle follows a live command at once,
/// like the C++ chain, and none of the latch's triggers fires: not the
/// feedback loss, not timeout reports, not a board restart.
#[test]
fn the_latch_can_be_turned_off_for_the_parity_tests() {
    let mut rig = Rig::new(BaseConfig { arm_latch: false, ..production() });
    assert_eq!(rig.base.disarmed(), None);
    assert_eq!(rig.step(Some((0.4, 0.0)), true), (73, 73));
    let lost = rig.run(40, Some((0.4, 0.0)), false);
    assert!(lost.last().unwrap().0 > 600, "no latch on feedback loss either");

    let mut board = Board::new(rig.t);
    rig.drive_n(&mut board, 20, Some((0.4, 0.0)));
    board.rx_lead = false;
    rig.drive_n(&mut board, 50, Some((0.4, 0.0)));
    board.rx_lead = true;
    rig.drive_n(&mut board, 60, Some((0.4, 0.0)));
    board.restart(rig.t, 200 * MS);
    board.deaf_until = rig.t + 300 * MS;
    let moving = rig.drive_n(&mut board, 20, Some((0.4, 0.0)));
    assert!(moving.iter().all(|w| w.0 > 600));
    assert!(
        !rig.events.iter().any(|e| matches!(
            e,
            Event::Disarmed(_)
                | Event::Armed { .. }
                | Event::BoardNotReceiving { .. }
                | Event::BoardReset(_)
                | Event::BoardReceiving { .. }
        )),
        "{:?}",
        rig.events
    );
}

// ---- the command path and board restarts ----------------------------------
//
// The first supervised run on the robot (2026-09-28, wheels off the ground,
// an image without these triggers) pulled only the LubanCat TX -> STM32 RX
// lead (40-pin pin 8 -> PA10) with the stick held forward. The STM32 kept
// sending feedback (feedback age 0.00-0.04 s) and reported 0x81 flags 0x03
// with command_age_ms growing 350 -> 7850 at PWM 0; on the re-seat, stick
// still held, the next 0x01 was the live command and the wheels spun up at
// once (pwm 81 on the first sample). The feedback half of the latch cannot
// see that pull; these tests are the board half.

const VALID: u8 = STATUS_FLAG_COMMAND_VALID;
const TIMEOUT: u8 = STATUS_FLAG_COMMAND_TIMEOUT;
/// the `command_timeout_ms` every 0x01 carries
const BOARD_TIMEOUT: i64 = 300 * MS;
/// `UART_STATUS_PERIOD_MS`
const STATUS: i64 = 50 * MS;

/// A hand-made 0x81.
fn motor_status(flags: u8, command_age_ms: u16) -> Vec<u8> {
    let mut p = [0u8; 12];
    p[8..10].copy_from_slice(&command_age_ms.to_le_bytes());
    p[10] = flags;
    build_frame(MOTOR_STATUS, 0, &p)
}

/// The STM32 as far as the latch can see it: `control_update_50hz` in
/// `firmware/Module/Src/motor.cpp` (a 0x01 drives the wheels until
/// `command_timeout_ms` after it arrived; `valid` is set by the first one
/// and cleared only at boot; `timeout = !valid || age > timeout`) and
/// `MotorTask`'s 50 ms status batch, 0x81 then 0x85, with encoder totals
/// that count from 0 at boot. Each direction of the UART is its own lead.
struct Board {
    next_status: i64,
    valid: bool,
    last_cmd: i64,
    cmd: (i16, i16),
    counts: i32,
    /// LubanCat TX -> STM32 RX (40-pin pin 8 -> PA10)
    rx_lead: bool,
    /// STM32 TX -> LubanCat RX
    tx_lead: bool,
    /// sends nothing and takes nothing before this (bootloader, flash erase)
    stalled_until: i64,
    /// takes no 0x01 before this, but sends
    deaf_until: i64,
    /// the bytes on their way to the host
    out: Vec<u8>,
    /// every 0x01 it accepted, `(t, left, right)`
    accepted: Vec<(i64, i16, i16)>,
    /// what the output stage drove at each status, `(t, left, right)`
    applied: Vec<(i64, i16, i16)>,
    /// every 0x81 that went out on the TX lead, `(t, flags, command_age_ms)`
    reports: Vec<(i64, u8, u16)>,
}

impl Board {
    /// A board that has been up for a while; no 0x01 has reached it yet.
    fn new(t: i64) -> Self {
        Board {
            next_status: t + 17 * MS,
            valid: false,
            last_cmd: 0,
            cmd: (0, 0),
            counts: 0,
            rx_lead: true,
            tx_lead: true,
            stalled_until: 0,
            deaf_until: 0,
            out: Vec::new(),
            accepted: Vec::new(),
            applied: Vec::new(),
            reports: Vec::new(),
        }
    }

    /// Run the board up to `t`.
    fn advance(&mut self, t: i64) {
        while self.next_status <= t {
            let s = self.next_status;
            self.next_status += STATUS;
            if s < self.stalled_until {
                continue;
            }
            let timeout = !self.valid || s - self.last_cmd > BOARD_TIMEOUT;
            let (l, r) = if timeout { (0, 0) } else { self.cmd };
            self.applied.push((s, l, r));
            // 1000 permille = 58 rpm = 8600 counts/s = 430 per status period
            self.counts = self.counts.wrapping_add(l as i32 * 43 / 100);
            let age = if self.valid { ((s - self.last_cmd) / MS).min(65535) as u16 } else { 65535 };
            let flags = if self.valid { VALID } else { 0 } | if timeout { TIMEOUT } else { 0 };
            if self.tx_lead {
                self.reports.push((s, flags, age));
                self.out.extend(motor_status(flags, age));
                self.out.extend(feedback(self.counts));
            }
        }
    }

    /// The host wrote `tx` at `t`.
    fn receive(&mut self, tx: &TxFrame, t: i64) {
        if !self.rx_lead || t < self.stalled_until || t < self.deaf_until {
            return;
        }
        let (l, r) = wheels(tx);
        self.valid = true;
        self.last_cmd = t;
        self.cmd = (l, r);
        self.accepted.push((t, l, r));
    }

    /// Reset at `t`: nothing for `silent` (the real one: 500 ms bootloader
    /// grace plus the app's start-up), then the new boot's first status
    /// batch 60 ms after `MotorTask` starts, totals from 0, no command yet.
    fn restart(&mut self, t: i64, silent: i64) {
        self.valid = false;
        self.cmd = (0, 0);
        self.counts = 0;
        self.stalled_until = t + silent;
        self.next_status = t + silent + 60 * MS;
    }

    /// What the wheels were driven with after `t`.
    fn applied_after(&self, t: i64) -> Vec<(i16, i16)> {
        self.applied.iter().filter(|a| a.0 > t).map(|a| (a.1, a.2)).collect()
    }
    fn accepted_after(&self, t: i64) -> Vec<(i16, i16)> {
        self.accepted.iter().filter(|a| a.0 > t).map(|a| (a.1, a.2)).collect()
    }
}

impl Rig {
    /// One cycle 40 ms later against `board`: read what it sent by now,
    /// tick, write the frame to it.
    fn drive(&mut self, board: &mut Board, cmd: Option<(f64, f64)>) -> (i16, i16) {
        self.t += DT;
        board.advance(self.t);
        let rx = std::mem::take(&mut board.out);
        if !rx.is_empty() {
            self.base.on_rx(&rx, self.t);
        }
        self.base.set_publishers(Stream::CmdVel, self.publishers, self.t);
        self.base.set_publishers(Stream::WheelOverride, self.publishers, self.t);
        let cmd = cmd.map(|(l, a)| Command { twist: Twist::new(l, a), stamp_ns: self.t });
        let (tx, odom, _) = self.base.tick(cmd, self.t, self.t - T0 + ROS0);
        if odom.is_some() {
            self.last_odom = odom;
        }
        self.events.extend(self.base.take_events());
        let tx = tx.unwrap();
        board.receive(&tx, self.t);
        let w = wheels(&tx);
        self.last_tx = tx;
        w
    }
    fn drive_n(&mut self, board: &mut Board, n: usize, cmd: Option<(f64, f64)>) -> Vec<(i16, i16)> {
        (0..n).map(|_| self.drive(board, cmd)).collect()
    }
    /// The stick at rest from the activation until the latch lets go of
    /// cmd_vel: a zero every cycle for the release window. The board's
    /// first status after our first frame shows it receiving (two cycles
    /// with a working lead), long before that.
    fn arm(&mut self, board: &mut Board) {
        assert_eq!(self.release_on(board), RELEASE_CYCLES, "{:?}", self.events);
    }
    /// [`Rig::release`] against `board`: zeros until cmd_vel re-arms, and
    /// how many cycles that took.
    fn release_on(&mut self, board: &mut Board) -> usize {
        for n in 1..=RELEASE_CYCLES + 50 {
            assert_eq!(self.drive(board, Some((0.0, 0.0))), (0, 0));
            if !self.base.holds(Stream::CmdVel) {
                assert_eq!(self.armed(Stream::CmdVel), Some(StopEdge::Stop));
                return n;
            }
        }
        panic!("cmd_vel not re-armed by 2.5 s of zeros: {:?}", self.events);
    }
    /// The app's teleop stream: `v` m/s at 10 Hz (a message on the cycles
    /// that cross a 100 ms boundary), `seconds` long; `None` sends nothing.
    fn teleop(&mut self, board: &mut Board, v: Option<f64>, seconds: f64) -> Vec<(i16, i16)> {
        let end = self.t + (seconds * 1e9).round() as i64;
        let mut out = Vec::new();
        while self.t < end {
            let due = (self.t + DT - T0) / (100 * MS) != (self.t - T0) / (100 * MS);
            out.push(self.drive(board, v.filter(|_| due).map(|x| (x, 0.0))));
        }
        out
    }
    /// pid_autotune's `hold(permille, seconds)`: the override every 100 ms
    /// with a 300 ms ttl, on the 25 Hz cycle.
    fn hold(&mut self, board: &mut Board, permille: i32, seconds: f64) -> Vec<(i16, i16)> {
        let end = self.t + (seconds * 1e9) as i64;
        let mut next = self.t;
        let mut out = Vec::new();
        while self.t < end {
            if self.t >= next {
                self.base.request_wheel_override(permille, permille, 300, self.t);
                next += 100 * MS;
            }
            out.push(self.drive(board, None));
        }
        out
    }
    fn count(&self, pred: impl Fn(&Event) -> bool) -> usize {
        self.events.iter().filter(|e| pred(e)).count()
    }
    fn board_not_receiving(&self) -> Vec<u16> {
        self.events
            .iter()
            .filter_map(|e| match e {
                Event::BoardNotReceiving { command_age_ms } => Some(*command_age_ms),
                _ => None,
            })
            .collect()
    }
    fn board_receiving(&self) -> Vec<u16> {
        self.events
            .iter()
            .filter_map(|e| match e {
                Event::BoardReceiving { command_age_ms, .. } => Some(*command_age_ms),
                _ => None,
            })
            .collect()
    }
    fn resets(&self) -> Vec<ResetSignature> {
        self.events
            .iter()
            .filter_map(|e| match e {
                Event::BoardReset(sig) => Some(*sig),
                _ => None,
            })
            .collect()
    }
}

/// The 2026-09-28 pull, TX lead only, stick held: the feedback never stops,
/// the board reports COMMAND_TIMEOUT with a growing age, and the latch
/// disarms on the second report. On the re-seat the board receives 0/0 —
/// this driver's zeros — and the wheels move again only after a release.
#[test]
fn a_pulled_tx_lead_disarms_and_the_reseat_gets_zeros() {
    let mut rig = Rig::new(production());
    let mut board = Board::new(T0);
    rig.arm(&mut board);
    let moving = rig.drive_n(&mut board, 40, Some((0.4, 0.0)));
    assert!(moving.last().unwrap().0 > 600);
    assert!(board.applied.last().unwrap().1 > 600);
    assert!(!rig.base.holds(Stream::CmdVel));
    rig.events.clear();

    board.rx_lead = false;
    let last_landed = board.last_cmd;
    let pulled_at = rig.t;
    let pulled = rig.drive_n(&mut board, 196, Some((0.4, 0.0))); // 7.84 s out

    // the firmware stopped the wheels on its own, 300-350 ms after the last frame
    let stopped = board.applied.iter().find(|a| a.0 > last_landed && a.1 == 0).unwrap().0;
    assert!(stopped - last_landed > BOARD_TIMEOUT && stopped - last_landed <= BOARD_TIMEOUT + STATUS);
    // the feedback never stopped, so this was not a feedback loss
    assert_eq!(rig.count(|e| matches!(e, Event::FeedbackLost { .. })), 0, "{:?}", rig.events);
    assert!(rig.base.telemetry().feedback_age_s < 0.05);
    // disarmed once, on the second timeout report, with the board's age
    let lost = rig.board_not_receiving();
    assert_eq!(lost.len(), 1, "{:?}", rig.events);
    assert!((350..=400).contains(&lost[0]), "command_age_ms {lost:?}");
    assert_eq!(rig.count(|e| *e == Event::Disarmed(DisarmReason::BoardNotReceiving)), 1);
    assert_eq!(rig.base.disarmed(), Some(DisarmReason::BoardNotReceiving));
    // live until the disarm, braked to 0/0 on that cycle and after
    let first_zero = pulled.iter().position(|w| *w == (0, 0)).unwrap();
    assert!(pulled[..first_zero].iter().all(|w| w.0 > 600));
    assert!(all_zero(&pulled[first_zero..]));
    let t_zero = pulled_at + (first_zero as i64 + 1) * DT;
    assert!(
        t_zero - last_landed <= BOARD_TIMEOUT + TIMEOUT_REPORTS_TO_DISARM as i64 * STATUS + DT,
        "zeroed {} ms after the last frame landed",
        (t_zero - last_landed) / MS
    );
    // the board's own view at the end, as the robot's telemetry showed it
    let m = rig.base.telemetry().motor.unwrap();
    assert_eq!(m.flags, VALID | TIMEOUT);
    assert!((7700..7900).contains(&m.command_age_ms), "{}", m.command_age_ms);

    // re-seated with the stick still held: the board gets zeros
    rig.events.clear();
    let reseat = rig.t;
    board.rx_lead = true;
    let back = rig.drive_n(&mut board, 50, Some((0.4, 0.0)));
    assert!(all_zero(&back), "{back:?}");
    let accepted = board.accepted_after(reseat);
    assert_eq!(accepted.len(), 50);
    assert!(all_zero(&accepted), "the first 0x01 after the re-seat must be 0/0: {accepted:?}");
    assert!(all_zero(&board.applied_after(reseat)), "no lurch");
    let receiving = rig.board_receiving();
    assert_eq!(receiving.len(), 1, "{:?}", rig.events);
    assert!(receiving[0] <= 80, "{receiving:?}");
    assert!(rig.base.holds(Stream::CmdVel), "the stick is still held");
    assert_eq!(rig.armed(Stream::CmdVel), None);
    // (the idle override stream re-arms by its silence once the board is back)
    assert_eq!(rig.armed(Stream::WheelOverride), Some(StopEdge::Silence));

    // release, push again: the normal ramp
    assert_eq!(rig.release_on(&mut board), RELEASE_CYCLES);
    assert_eq!(rig.drive(&mut board, Some((0.4, 0.0))), (73, 73));
}

/// Re-arming needs both: the board receiving again *and* each stream's own
/// stop edge. Released (and then silent) while the lead is still out: held
/// until the board reports frames arriving, then re-armed on that cycle. A
/// report that the last frame is still 200 ms old is not "arriving".
#[test]
fn rearming_needs_the_board_back_as_well_as_the_stop() {
    let mut rig = Rig::new(production());
    let mut board = Board::new(T0);
    rig.arm(&mut board);
    rig.drive_n(&mut board, 60, None); // the override stream too, by silence
    rig.drive_n(&mut board, 10, Some((0.3, 0.0)));
    assert_eq!(rig.base.disarmed(), None);
    rig.events.clear();

    board.rx_lead = false;
    rig.drive_n(&mut board, 25, Some((0.3, 0.0)));
    assert_eq!(rig.base.disarmed(), Some(DisarmReason::BoardNotReceiving));

    // released, then nothing for 2 s, the lead still out
    rig.drive_n(&mut board, 5, Some((0.0, 0.0)));
    rig.drive_n(&mut board, 50, None);
    assert!(rig.base.holds(Stream::CmdVel) && rig.base.holds(Stream::WheelOverride));
    assert_eq!(rig.armed(Stream::CmdVel), None);
    assert_eq!(rig.armed(Stream::WheelOverride), None);
    // pushed with the lead out: no ramp builds up behind it
    let pushed = rig.drive_n(&mut board, 25, Some((0.4, 0.0)));
    assert!(all_zero(&pushed));
    rig.drive_n(&mut board, 25, None);

    // a stale "valid, no timeout" report is not the board back
    rig.base.on_rx(&motor_status(VALID, 200), rig.t);
    rig.events.extend(rig.base.take_events());
    assert!(rig.board_receiving().is_empty());

    // the lead is back: the first report of frames arriving re-arms both
    // streams, which are at rest, on that same cycle
    board.rx_lead = true;
    let mut cycles = 0;
    while rig.board_receiving().is_empty() {
        assert_eq!(rig.drive(&mut board, None), (0, 0));
        cycles += 1;
        assert!(cycles < 5);
    }
    assert_eq!(rig.armed(Stream::CmdVel), Some(StopEdge::Silence));
    assert_eq!(rig.armed(Stream::WheelOverride), Some(StopEdge::Silence));
    let order: Vec<_> = rig
        .events
        .iter()
        .filter(|e| matches!(e, Event::BoardReceiving { .. } | Event::Armed { .. }))
        .collect();
    assert!(matches!(order[0], Event::BoardReceiving { .. }), "{order:?}");
    assert_eq!(rig.drive(&mut board, Some((0.4, 0.0))), (73, 73));
}

/// Parity mode, the same pull: the C++ chain has no latch, keeps sending
/// the live command into the cut lead, and on the re-seat that command is
/// the first thing the board gets — the lurch the robot showed.
#[test]
fn without_the_latch_the_same_pull_lurches() {
    let mut rig = Rig::new(BaseConfig { arm_latch: false, ..production() });
    let mut board = Board::new(T0);
    rig.drive_n(&mut board, 40, Some((0.4, 0.0)));
    board.rx_lead = false;
    let pulled = rig.drive_n(&mut board, 100, Some((0.4, 0.0)));
    assert!(pulled.iter().all(|w| w.0 > 600));
    let reseat = rig.t;
    board.rx_lead = true;
    rig.drive_n(&mut board, 5, Some((0.4, 0.0)));
    let landed = board.accepted.iter().find(|a| a.0 > reseat).unwrap();
    assert!(landed.1 > 600, "the first 0x01 after the re-seat is the live command");
    assert!(board.applied_after(landed.0)[0].0 > 600, "wheels driven on the next status");
    assert!(rig.events.iter().all(|e| matches!(e, Event::Publishers { .. })), "{:?}", rig.events);
}

/// A restart that comes back inside `feedback_timeout_s` (a board that
/// boots faster than the 500 ms bootloader grace allows today), stick held:
/// the new boot's first 0x85 has its totals back at zero after 4 s of
/// driving, and the 0x01 frames from then on are zeros.
#[test]
fn a_board_restart_is_seen_in_the_encoder_totals() {
    let mut rig = Rig::new(production());
    let mut board = Board::new(T0);
    rig.arm(&mut board);
    rig.drive_n(&mut board, 100, Some((0.4, 0.0)));
    let before = board.counts;
    assert!(before > 20_000, "{before}");
    rig.events.clear();

    let reset_at = rig.t;
    board.restart(rig.t, 300 * MS);
    let frames = rig.drive_n(&mut board, 25, Some((0.4, 0.0)));
    assert_eq!(rig.count(|e| matches!(e, Event::FeedbackLost { .. })), 0);
    let resets = rig.resets();
    assert_eq!(resets.len(), 1, "{:?}", rig.events);
    match resets[0] {
        ResetSignature::EncoderTotalsRestarted { left, right, prev_left, prev_right } => {
            assert!(left.abs() < 500 && right.abs() < 500, "{left} {right}");
            assert_eq!((prev_left, prev_right), (before, before));
        }
        other => panic!("{other:?}"),
    }
    assert_eq!(rig.count(|e| *e == Event::Disarmed(DisarmReason::BoardReset)), 1);
    let first_zero = frames.iter().position(|w| *w == (0, 0)).unwrap();
    assert!(all_zero(&frames[first_zero..]));
    // The live command that reached the new boot before its first status is
    // the one thing the host cannot take back: it knows of the restart only
    // from that status. At most the first status period plus one read.
    let live = board.applied_after(reset_at).iter().filter(|w| w.0 != 0).count();
    assert!(live <= 2, "{:?}", board.applied_after(reset_at));
    assert_eq!(rig.board_receiving().len(), 1);
    assert!(rig.base.holds(Stream::CmdVel));

    assert_eq!(rig.release_on(&mut board), RELEASE_CYCLES);
    assert_eq!(rig.drive(&mut board, Some((0.4, 0.0))), (73, 73));
}

/// A restart that the encoder totals cannot show (the wheels never turned
/// since the last one) but COMMAND_VALID can: no 0x01 reached the new boot
/// before its first 0x81, which then reports 0x02 — the firmware's "no
/// command yet" (`timeout = !valid || ...`), not the 0x00 the table used
/// to list.
/// Also: the second signature of the same restart is not a second restart.
#[test]
fn a_board_restart_is_seen_in_command_valid() {
    let mut rig = Rig::new(production());
    let mut board = Board::new(T0);
    rig.arm(&mut board);
    rig.drive_n(&mut board, 60, None);
    assert_eq!(rig.base.disarmed(), None);
    rig.events.clear();

    board.restart(rig.t, 200 * MS);
    board.deaf_until = rig.t + 300 * MS;
    rig.drive_n(&mut board, 8, None); // the new boot's first status is at 260 ms
    assert_eq!(rig.resets(), vec![ResetSignature::CommandValidCleared { flags: TIMEOUT }]);
    assert_eq!(rig.base.disarmed(), Some(DisarmReason::BoardReset));
    // at rest: re-armed by silence once the board receives again (80 ms
    // later) and the release window since the disarm is up
    rig.drive_n(&mut board, RELEASE_CYCLES, None);
    assert_eq!(rig.board_receiving().len(), 1, "{:?}", rig.events);
    assert_eq!(rig.base.disarmed(), None, "{:?}", rig.events);

    // both signatures on one boot: after 4 s of driving, restarted with
    // the first 0x01 landing after the first status
    rig.drive_n(&mut board, 100, Some((0.4, 0.0)));
    rig.events.clear();
    board.restart(rig.t, 200 * MS);
    board.deaf_until = rig.t + 300 * MS;
    rig.drive_n(&mut board, 25, Some((0.4, 0.0)));
    assert_eq!(rig.resets(), vec![ResetSignature::CommandValidCleared { flags: TIMEOUT }]);
    assert_eq!(rig.count(|e| matches!(e, Event::Disarmed(_))), 1, "{:?}", rig.events);
}

/// A real restart: 500 ms in the bootloader plus the app's start-up, so the
/// feedback loss (0.52 s) disarms first and the new boot never sees a live
/// command; the encoder totals then name the cause.
#[test]
fn a_real_restart_disarms_on_the_silence_before_the_board_is_back() {
    let mut rig = Rig::new(production());
    let mut board = Board::new(T0);
    rig.arm(&mut board);
    rig.drive_n(&mut board, 100, Some((0.4, 0.0)));
    rig.events.clear();
    let reset_at = rig.t;
    board.restart(rig.t, 520 * MS);
    let frames = rig.drive_n(&mut board, 40, Some((0.4, 0.0)));
    let kinds: Vec<_> = rig
        .events
        .iter()
        .filter(|e| matches!(e, Event::Disarmed(_) | Event::BoardReset(_)))
        .collect();
    assert_eq!(
        kinds,
        vec![
            &Event::Disarmed(DisarmReason::FeedbackLost),
            &Event::BoardReset(match rig.resets()[0] {
                s @ ResetSignature::EncoderTotalsRestarted { .. } => s,
                other => panic!("{other:?}"),
            }),
            &Event::Disarmed(DisarmReason::BoardReset),
        ]
    );
    assert!(all_zero(&board.accepted_after(reset_at + 520 * MS)), "{:?}", board.accepted);
    assert!(all_zero(&board.applied_after(reset_at)));
    assert!(all_zero(&frames[14..]));
    assert!(rig.base.holds(Stream::CmdVel));
    assert_eq!(rig.release_on(&mut board), RELEASE_CYCLES);
}

/// At activation the board's last report is its memory of the time no
/// driver ran: a timeout (valid, with the age of the previous driver's last
/// frame) or "no command yet" (a board that just booted). Neither disarms
/// anything beyond the activation itself, not even a board that stays deaf
/// for the first 400 ms: timeout reports only count once this driver has
/// been writing steadily for 450 ms. (A board still deaf after that is a
/// lead that was out at activation: see
/// `a_lead_already_out_at_activation_is_a_command_path_loss`.)
#[test]
fn nothing_disarms_at_activation_while_the_board_catches_up() {
    // (board had commands before, board deaf for the first ... ms)
    for (had_commands, deaf_ms) in [(false, 0), (true, 0), (true, 400), (false, 400)] {
        let mut rig = Rig::new(production());
        let mut board = Board::new(T0);
        if had_commands {
            board.valid = true;
            board.last_cmd = T0 - 5_000 * MS;
        }
        board.deaf_until = T0 + deaf_ms * MS;
        let rest = rig.drive_n(&mut board, 20, Some((0.0, 0.0)));
        assert!(all_zero(&rest));
        let moving = rig.drive_n(&mut board, 40, Some((0.4, 0.0)));
        assert!(moving.last().unwrap().0 > 600, "{had_commands} {deaf_ms}");
        let disarms: Vec<_> = rig.events.iter().filter(|e| matches!(e, Event::Disarmed(_))).collect();
        assert_eq!(disarms, vec![&Event::Disarmed(DisarmReason::Activation)], "{had_commands} {deaf_ms}");
        assert!(rig.resets().is_empty() && rig.board_not_receiving().is_empty());
    }
}

/// One timeout report is not a lost lead. Nor is the stale copy of it the
/// telemetry keeps, a garbled frame, or the reports of a stall of this very
/// loop (the firmware timeout doing its job; the reports are read once the
/// loop runs again, while its own writing is not yet steady). Two fresh
/// reports in a row are.
#[test]
fn a_single_or_stale_timeout_report_does_not_disarm() {
    let mut rig = Rig::new(production());
    let mut board = Board::new(T0);
    rig.arm(&mut board);
    rig.drive_n(&mut board, 30, Some((0.3, 0.0)));
    rig.events.clear();

    // one report, then several cycles with no 0x81 at all, the stale copy
    // still in the telemetry
    rig.base.on_rx(&motor_status(VALID | TIMEOUT, 340), rig.t);
    board.tx_lead = false;
    rig.drive_n(&mut board, 5, Some((0.3, 0.0)));
    assert_eq!(rig.base.telemetry().motor.unwrap().flags, VALID | TIMEOUT);
    board.tx_lead = true;
    rig.drive_n(&mut board, 3, Some((0.3, 0.0)));
    // single reports between good ones never add up
    for _ in 0..10 {
        rig.base.on_rx(&motor_status(VALID | TIMEOUT, 340), rig.t);
        rig.drive_n(&mut board, 3, Some((0.3, 0.0)));
    }
    // garbled: the CRC drops them before anything sees them
    for _ in 0..3 {
        let mut bad = motor_status(VALID | TIMEOUT, 340);
        let n = bad.len();
        bad[n - 1] ^= 0xFF;
        rig.base.on_rx(&bad, rig.t);
    }
    rig.drive_n(&mut board, 3, Some((0.3, 0.0)));
    assert!(rig.base.crc_errors() >= 3);

    // this loop stalls for 450 ms: the board times out and reports it
    // while nobody reads; the loop comes back and reads them all at once
    board.advance(rig.t + 450 * MS);
    rig.t += 450 * MS - DT;
    let after = rig.drive_n(&mut board, 25, Some((0.3, 0.0)));
    assert!(after.last().unwrap().0 > 400, "{after:?} {:?}", rig.events);
    assert!(rig.board_not_receiving().is_empty(), "{:?}", rig.events);
    assert_eq!(rig.count(|e| matches!(e, Event::Disarmed(_))), 0, "{:?}", rig.events);
    assert_eq!(rig.base.disarmed(), None);

    // two fresh reports in a row while writing steadily: that is a loss
    for _ in 0..TIMEOUT_REPORTS_TO_DISARM {
        rig.base.on_rx(&motor_status(VALID | TIMEOUT, 340), rig.t);
    }
    rig.events.extend(rig.base.take_events());
    assert_eq!(rig.board_not_receiving(), vec![340]);
    assert_eq!(rig.base.disarmed(), Some(DisarmReason::BoardNotReceiving));
}

/// Both leads out and back together, stick held: the feedback loss holds
/// the wheels, and the board receives this driver's zeros. Its first
/// reports after the re-seat still say COMMAND_TIMEOUT — two of them here,
/// the contact bounce eating the first frames — but they describe the
/// outage the feedback loss has already latched, not a second loss of the
/// command path.
#[test]
fn a_reseat_of_both_leads_is_only_the_feedback_loss() {
    let mut rig = Rig::new(production());
    let mut board = Board::new(T0);
    rig.arm(&mut board);
    let moving = rig.drive_n(&mut board, 40, Some((0.4, 0.0)));
    assert!(moving.last().unwrap().0 > 600);
    assert!(!rig.base.holds(Stream::CmdVel));
    rig.events.clear();

    board.rx_lead = false;
    board.tx_lead = false;
    rig.drive_n(&mut board, 50, Some((0.4, 0.0)));
    assert_eq!(rig.base.disarmed(), Some(DisarmReason::FeedbackLost));

    let reseat = rig.t;
    board.rx_lead = true;
    board.tx_lead = true;
    board.deaf_until = reseat + 100 * MS;
    let back = rig.drive_n(&mut board, 30, Some((0.4, 0.0)));
    assert!(all_zero(&back), "{back:?}");
    assert!(!board.accepted_after(reseat).is_empty());
    assert!(all_zero(&board.accepted_after(reseat)));
    let stale = board.reports.iter().filter(|r| r.0 > reseat && r.1 & TIMEOUT != 0).count();
    assert!(stale >= 2, "needs two timeout reports after the re-seat: {:?}", board.reports);
    assert!(rig.board_not_receiving().is_empty(), "{:?}", rig.events);
    let disarms: Vec<_> = rig.events.iter().filter(|e| matches!(e, Event::Disarmed(_))).collect();
    assert_eq!(disarms, vec![&Event::Disarmed(DisarmReason::FeedbackLost)]);
    assert!(rig.base.holds(Stream::CmdVel), "the stick is still held");

    assert_eq!(rig.release_on(&mut board), RELEASE_CYCLES);
    assert_eq!(rig.drive(&mut board, Some((0.4, 0.0))), (73, 73));
}

/// Both leads out at rest, then only the STM32 TX lead back: the feedback
/// returns but the board still hears nothing, and says so in its 0x81. The
/// feedback alone re-arms nothing (a board that reports its command path
/// must show it receiving as well), so a push now goes nowhere. Once the
/// feedback has been back for 450 ms its timeout reports count again and
/// name the lead, once, without a second disarm: the latch already holds.
/// The re-seat of pin 8, stick still held, gets zeros.
#[test]
fn a_partial_reseat_is_caught_by_the_command_path() {
    let mut rig = Rig::new(production());
    let mut board = Board::new(T0);
    rig.arm(&mut board);
    rig.drive_n(&mut board, 60, None);
    assert_eq!(rig.base.disarmed(), None);
    rig.events.clear();

    let pulled = rig.t;
    board.rx_lead = false;
    board.tx_lead = false;
    rig.drive_n(&mut board, 40, None);
    assert_eq!(rig.base.disarmed(), Some(DisarmReason::FeedbackLost));

    board.tx_lead = true;
    let idle = rig.drive_n(&mut board, 3, None);
    assert!(all_zero(&idle));
    assert!(rig.events.contains(&Event::FeedbackResumed));
    assert!(rig.base.holds(Stream::CmdVel), "the feedback alone re-arms nothing: {:?}", rig.events);
    assert!(rig.base.holds(Stream::WheelOverride));
    let mut pushed = Vec::new();
    let mut caught = None;
    for k in 0..25 {
        pushed.push(rig.drive(&mut board, Some((0.4, 0.0))));
        if caught.is_none() && !rig.board_not_receiving().is_empty() {
            caught = Some(k);
        }
    }
    assert!(all_zero(&pushed), "{pushed:?}");
    assert_eq!(rig.board_not_receiving().len(), 1, "{:?}", rig.events);
    // counted from the cycle that saw the feedback again
    let caught_ms = (3 + caught.unwrap() as i64 + 1) * DT / MS;
    assert!((450..=640).contains(&caught_ms), "named {caught_ms} ms after the feedback returned");
    let disarms: Vec<_> = rig.events.iter().filter(|e| matches!(e, Event::Disarmed(_))).collect();
    assert_eq!(disarms, vec![&Event::Disarmed(DisarmReason::FeedbackLost)]);
    assert_eq!(rig.armed(Stream::CmdVel), None);

    let reseat = rig.t;
    board.rx_lead = true;
    let back = rig.drive_n(&mut board, 25, Some((0.4, 0.0)));
    assert!(all_zero(&back), "{back:?}");
    assert!(all_zero(&board.accepted_after(reseat)));
    assert!(all_zero(&board.applied_after(pulled)), "the board never drove");
    assert_eq!(rig.board_receiving().len(), 1, "{:?}", rig.events);
    assert!(rig.base.holds(Stream::CmdVel));
    // the idle override stream re-arms once the board receives
    assert_eq!(rig.armed(Stream::WheelOverride), Some(StopEdge::Silence));

    assert_eq!(rig.release_on(&mut board), RELEASE_CYCLES);
    assert_eq!(rig.drive(&mut board, Some((0.4, 0.0))), (73, 73));
}

/// What waiting for the board costs when both leads come back together at
/// rest: this driver's zeros land within a cycle and the next 0x81 shows
/// them, so the streams re-arm at most about 90 ms after the feedback
/// returned, and nothing but the feedback loss and its recovery is logged.
/// (Silence counts from the disarm, 0.52 s into the outage: an outage
/// shorter than that plus the release window, about 1 s, re-arms when the
/// window is up instead.)
#[test]
fn both_leads_back_at_rest_rearm_as_soon_as_the_board_receives() {
    let mut rig = Rig::new(production());
    let mut board = Board::new(T0);
    rig.arm(&mut board);
    rig.drive_n(&mut board, 60, None);
    for gap_cycles in [30usize, 40, 75] {
        rig.events.clear();
        board.rx_lead = false;
        board.tx_lead = false;
        rig.drive_n(&mut board, gap_cycles, None);
        assert_eq!(rig.base.disarmed(), Some(DisarmReason::FeedbackLost));
        board.rx_lead = true;
        board.tx_lead = true;
        let mut cycles = 0;
        while rig.base.disarmed().is_some() {
            assert_eq!(rig.drive(&mut board, None), (0, 0));
            cycles += 1;
            assert!(cycles <= 3, "gap {gap_cycles}: {:?}", rig.events);
        }
        let kinds: Vec<_> = rig
            .events
            .iter()
            .filter(|e| !matches!(e, Event::Armed { .. } | Event::FeedbackLost { .. }))
            .collect();
        assert_eq!(
            kinds.iter().map(|e| std::mem::discriminant(*e)).collect::<Vec<_>>(),
            [
                Event::Disarmed(DisarmReason::FeedbackLost),
                Event::FeedbackResumed,
                Event::BoardReceiving { command_age_ms: 0, after_s: 0.0 },
            ]
            .iter()
            .map(std::mem::discriminant)
            .collect::<Vec<_>>(),
            "gap {gap_cycles}: {:?}",
            rig.events
        );
        assert_eq!(rig.armed(Stream::CmdVel), Some(StopEdge::Silence));
        assert_eq!(rig.armed(Stream::Blade), None, "an idle blade's hold is not worth a line");
        assert_eq!(rig.drive(&mut board, Some((0.4, 0.0))), (73, 73));
        rig.drive_n(&mut board, 10, Some((0.0, 0.0)));
        rig.drive_n(&mut board, 10, None);
    }
}

/// pid_autotune around the flash save: its open-loop and verify steps run
/// through the override stream without a single disarm; the save's sector
/// erase then stalls the STM32 (~1 s, no status frames, no 0x01 taken) and
/// the board comes back with a COMMAND_TIMEOUT. Whichever half of the latch
/// that trips — the feedback loss for a 1 s erase, the command path for a
/// shorter one with two timeout reports after it — nothing is moving by
/// then, the board shows our frames arriving on the next status, the run's
/// `finally` cancel (0/0, ttl 0) is its override stream's stop edge, and
/// the next run drives the wheels again.
#[test]
fn a_pid_flash_save_does_not_strand_the_autotune() {
    let mut rig = Rig::new(production());
    let mut board = Board::new(T0);
    rig.arm(&mut board);
    rig.drive_n(&mut board, 60, None);
    assert_eq!(rig.base.disarmed(), None);

    for (erase_ms, deaf_after_ms, tripped) in
        [(1000, 0, DisarmReason::FeedbackLost), (400, 100, DisarmReason::BoardNotReceiving)]
    {
        rig.events.clear();
        // run_inner: open loop 0 -> 40 % -> 70 %, then the closed-loop verify
        assert!(all_zero(&rig.hold(&mut board, 0, 0.6)));
        assert_eq!(*rig.hold(&mut board, 400, 2.5).last().unwrap(), (400, 400));
        assert_eq!(*rig.hold(&mut board, 700, 2.5).last().unwrap(), (700, 700));
        assert_eq!(board.applied.last().map(|a| (a.1, a.2)), Some((700, 700)));
        rig.hold(&mut board, 0, 0.3);
        assert_eq!(*rig.hold(&mut board, 500, 3.0).last().unwrap(), (500, 500));
        rig.hold(&mut board, 0, 0.3);
        assert_eq!(rig.count(|e| matches!(e, Event::Disarmed(_))), 0, "{:?}", rig.events);
        // review, then "apply": persist's 0x04 goes out and the erase starts
        rig.drive_n(&mut board, 25, None);
        rig.base.request_pid(mower_base_core::protocol::PidConfig {
            left_kp: 3.0,
            left_ki: 30.0,
            left_kd: 0.0,
            right_kp: 3.0,
            right_ki: 30.0,
            right_kd: 0.0,
            persist_to_flash: true,
            closed_loop_enabled: true,
        });
        rig.drive(&mut board, None);
        board.stalled_until = rig.t + erase_ms * MS;
        board.deaf_until = rig.t + (erase_ms + deaf_after_ms) * MS;
        // wait_pid: until the board is back and its 0x84 shows the save
        rig.drive_n(&mut board, ((erase_ms + deaf_after_ms) * MS / DT) as usize + 5, None);
        assert_eq!(
            rig.count(|e| matches!(e, Event::Disarmed(_))),
            1,
            "erase {erase_ms} ms: {:?}",
            rig.events
        );
        assert!(rig.events.contains(&Event::Disarmed(tripped)), "{:?}", rig.events);
        assert_eq!(rig.board_receiving().len(), 1, "back after the erase: {:?}", rig.events);
        // run()'s finally: override_wheels(0, 0, 0), then nothing. The
        // idle streams have been silent since the disarm, and re-arm by
        // that once the board is back and the window is up; the cancel
        // would, 0.52 s after it, at the latest.
        rig.base.request_wheel_override(0, 0, 0, rig.t);
        rig.drive_n(&mut board, RELEASE_CYCLES - 1, None);
        assert_eq!(rig.base.disarmed(), None, "erase {erase_ms} ms: {:?}", rig.events);
    }

    // the next run: its steps reach the wheels
    rig.hold(&mut board, 0, 0.6);
    assert_eq!(*rig.hold(&mut board, 400, 1.0).last().unwrap(), (400, 400));
    assert_eq!(board.applied.last().map(|a| (a.1, a.2)), Some((400, 400)));
}

/// Review finding: the LubanCat TX lead is already out (or loose) when the
/// driver activates — after a `kill -9` restart during a pull, a container
/// restart, a reboot with a loose connector. The board never shows it
/// receiving, and it reports COMMAND_TIMEOUT from the first status on:
/// flags 0x03 with the previous driver's age, or 0x02 on a board that
/// booted since. Once this driver has been writing steadily for 450 ms that
/// can only mean its frames are not arriving, so the command path loss
/// fires then; the idle streams do not re-arm by silence behind the cut
/// lead, a push goes nowhere, and the re-seat gets zeros.
#[test]
fn a_lead_already_out_at_activation_is_a_command_path_loss() {
    for had_commands in [true, false] {
        let mut rig = Rig::new(production());
        let mut board = Board::new(T0);
        if had_commands {
            board.valid = true;
            board.last_cmd = T0 - 5_000 * MS;
        }
        board.rx_lead = false;

        // idle for 2.8 s: long past the 2.3 s the idle streams need to
        // re-arm by silence
        let idle = rig.drive_n(&mut board, 70, None);
        assert!(all_zero(&idle));
        let lost = rig.board_not_receiving();
        assert_eq!(lost.len(), 1, "had commands {had_commands}: {:?}", rig.events);
        assert!(lost[0] > 300, "{lost:?}");
        assert!(rig.events.contains(&Event::Disarmed(DisarmReason::BoardNotReceiving)));
        let caught = rig.events.iter().position(|e| matches!(e, Event::BoardNotReceiving { .. }));
        assert!(caught.is_some());
        assert!(rig.base.holds(Stream::CmdVel) && rig.base.holds(Stream::WheelOverride));
        assert_eq!(rig.armed(Stream::CmdVel), None, "{:?}", rig.events);

        // pushed for 2 s with the lead out: nothing ramps up behind it
        let pushed = rig.drive_n(&mut board, 50, Some((0.4, 0.0)));
        assert!(all_zero(&pushed), "{pushed:?}");

        // re-seated with the stick still held: zeros, no lurch
        let reseat = rig.t;
        board.rx_lead = true;
        let back = rig.drive_n(&mut board, 25, Some((0.4, 0.0)));
        assert!(all_zero(&back), "{back:?}");
        assert!(!board.accepted_after(reseat).is_empty());
        assert!(all_zero(&board.accepted_after(reseat)));
        assert!(all_zero(&board.applied_after(T0)), "the board never drove");
        assert_eq!(rig.board_receiving().len(), 1, "{:?}", rig.events);
        assert!(rig.base.holds(Stream::CmdVel), "the stick is still held");
        assert_eq!(rig.board_not_receiving().len(), 1, "logged once: {:?}", rig.events);

        assert_eq!(rig.release_on(&mut board), RELEASE_CYCLES);
        assert_eq!(rig.drive(&mut board, Some((0.4, 0.0))), (73, 73));
    }
}

/// The rest of the same finding: a lead that was already out cannot be
/// named before this driver has written steadily for 450 ms, so an
/// explicit stop inside that window must not re-arm cmd_vel behind it — a
/// push after the stop would ramp up against a board that hears nothing
/// and reach it as a step if the contact is made before the command path
/// loss fires. A board that reports its command path re-arms nothing
/// before it shows it receiving; a working one does that on its first
/// status after our first frame, so the stop costs it one status period.
/// (The push here also cuts the stop short, which the release window
/// alone would hold; with the window off, `latch_release_s` 0, the gate
/// alone must.)
#[test]
fn a_stop_before_the_board_shows_it_receiving_rearms_nothing() {
    for release_s in [0.0, 0.5] {
        // pin 8 out when the driver starts (the previous one killed 2 s
        // ago): stop at 80 ms, pushed from 120 ms, contact at 280 ms, stick
        // held
        let mut rig = Rig::new(BaseConfig { latch_release_s: release_s, ..production() });
        let mut board = Board::new(T0);
        board.valid = true;
        board.last_cmd = T0 - 2_000 * MS;
        board.rx_lead = false;
        rig.drive(&mut board, None);
        rig.drive(&mut board, Some((0.0, 0.0)));
        assert!(rig.base.holds(Stream::CmdVel), "re-armed behind the cut lead: {:?}", rig.events);
        let mut frames = rig.drive_n(&mut board, 5, Some((0.4, 0.0)));
        let reseat = rig.t;
        board.rx_lead = true;
        frames.extend(rig.drive_n(&mut board, 25, Some((0.4, 0.0))));
        assert!(all_zero(&frames), "{frames:?}");
        assert!(!board.accepted_after(reseat).is_empty());
        assert!(all_zero(&board.accepted_after(reseat)), "{:?}", board.accepted);
        assert!(all_zero(&board.applied_after(T0)), "the board never drove");
        assert!(
            rig.board_not_receiving().is_empty(),
            "back before it could be named: {:?}",
            rig.events
        );
        assert!(rig.base.holds(Stream::CmdVel), "the stick is still held");
        let n = rig.release_on(&mut board);
        assert_eq!(n, if release_s > 0.0 { RELEASE_CYCLES } else { 1 });
        assert_eq!(rig.drive(&mut board, Some((0.4, 0.0))), (73, 73));

        // a working lead, same start: the board shows our frames arriving
        // on its status after the first one, so the gate costs nothing
        // beyond the window. One zero, then nothing: with the window off
        // it re-arms on the next cycle, with it 0.52 s after the zero.
        let mut rig = Rig::new(BaseConfig { latch_release_s: release_s, ..production() });
        let mut board = Board::new(T0);
        board.valid = true;
        board.last_cmd = T0 - 2_000 * MS;
        rig.drive(&mut board, Some((0.0, 0.0)));
        assert!(
            rig.base.holds(Stream::CmdVel),
            "the board's first status predates our first frame"
        );
        let mut cycles = 1;
        while rig.base.holds(Stream::CmdVel) {
            rig.drive(&mut board, None);
            cycles += 1;
            assert!(cycles <= RELEASE_CYCLES, "{:?}", rig.events);
        }
        assert_eq!(rig.armed(Stream::CmdVel), Some(StopEdge::Stop), "{:?}", rig.events);
        assert_eq!(cycles, if release_s > 0.0 { RELEASE_CYCLES } else { 2 });
        assert_eq!(rig.drive(&mut board, Some((0.4, 0.0))), (73, 73));
    }

    // and without the latch (the C++), the same pull lurches on the contact
    let mut rig = Rig::new(BaseConfig { arm_latch: false, ..production() });
    let mut board = Board::new(T0);
    board.valid = true;
    board.last_cmd = T0 - 2_000 * MS;
    board.rx_lead = false;
    rig.drive(&mut board, None);
    rig.drive(&mut board, Some((0.0, 0.0)));
    rig.drive_n(&mut board, 5, Some((0.4, 0.0)));
    let reseat = rig.t;
    board.rx_lead = true;
    rig.drive(&mut board, Some((0.4, 0.0)));
    assert!(board.accepted_after(reseat)[0].0 > 0, "{:?}", board.accepted);
}

/// Review finding: both leads out at rest, then the STM32 TX lead makes
/// contact first and pin 8 follows 280 ms later — before the board's
/// timeout reports count again (450 ms after the feedback returns). The
/// streams must not re-arm on the feedback alone: the board has to show it
/// receiving too, so a push in between goes nowhere and the second lead
/// gets zeros.
#[test]
fn a_partial_reseat_with_the_second_lead_soon_after_gets_zeros() {
    let mut rig = Rig::new(production());
    let mut board = Board::new(T0);
    rig.arm(&mut board);
    rig.drive_n(&mut board, 60, None);
    assert_eq!(rig.base.disarmed(), None);
    rig.events.clear();

    let pulled = rig.t;
    board.rx_lead = false;
    board.tx_lead = false;
    rig.drive_n(&mut board, 40, None);
    assert_eq!(rig.base.disarmed(), Some(DisarmReason::FeedbackLost));

    // the STM32 TX lead first: feedback back, the board still deaf
    board.tx_lead = true;
    let idle = rig.drive_n(&mut board, 5, None);
    assert!(all_zero(&idle));
    assert!(rig.events.contains(&Event::FeedbackResumed));
    assert!(rig.base.holds(Stream::CmdVel), "the feedback alone does not re-arm: {:?}", rig.events);
    let mut frames = rig.drive_n(&mut board, 2, Some((0.4, 0.0)));

    // pin 8 280 ms after the feedback came back, stick held
    let reseat = rig.t;
    board.rx_lead = true;
    frames.extend(rig.drive_n(&mut board, 25, Some((0.4, 0.0))));
    assert!(all_zero(&frames), "{frames:?}");
    assert!(all_zero(&board.accepted_after(reseat)));
    assert!(all_zero(&board.applied_after(pulled)), "the board never drove");
    assert!(rig.board_not_receiving().is_empty(), "{:?}", rig.events);
    assert_eq!(rig.board_receiving().len(), 1, "{:?}", rig.events);
    assert!(rig.base.holds(Stream::CmdVel));

    assert_eq!(rig.release_on(&mut board), RELEASE_CYCLES);
    assert_eq!(rig.drive(&mut board, Some((0.4, 0.0))), (73, 73));
}

/// Review finding: the blade is a dead-man the app refreshes every 0.2 s
/// (`docs/ROBOT_API.md`). Held through a pull of pin 8, the firmware's own
/// timeout stops it, and on the re-seat it must not restart while the
/// refresh goes on. The link losses hold it — one explicit 0 at the disarm,
/// then nothing applied — until the dead-man is let go (no refresh within
/// the ttl of the last one, or an explicit stop, either for the release
/// window); a fresh press after that runs it again. Without the latch (the
/// C++) it restarts on the re-seat.
#[test]
fn a_held_blade_needs_a_fresh_press_after_a_link_loss() {
    /// `n` cycles, the app refreshing 600 permille / 500 ms every 0.2 s if
    /// `held`; the 0x02 of each cycle.
    fn press(rig: &mut Rig, board: &mut Board, n: usize, held: bool) -> Vec<Option<i16>> {
        (0..n)
            .map(|k| {
                if held && k % 5 == 0 {
                    rig.base.request_blade(600, 500, rig.t);
                }
                rig.drive(board, None);
                blade_of(&rig.last_tx)
            })
            .collect()
    }
    let quiet = |frames: &[Option<i16>]| frames.iter().all(|b| b.is_none());

    let mut rig = Rig::new(production());
    let mut board = Board::new(T0);
    rig.arm(&mut board);
    rig.drive_n(&mut board, 60, None);
    assert_eq!(rig.base.disarmed(), None);
    assert!(!rig.base.holds(Stream::Blade), "not held at activation");

    // pin 8 pulled with the blade held
    assert_eq!(press(&mut rig, &mut board, 25, true).last(), Some(&Some(600)));
    board.rx_lead = false;
    let out = press(&mut rig, &mut board, 50, true);
    assert!(rig.events.contains(&Event::Disarmed(DisarmReason::BoardNotReceiving)));
    let stop = out.iter().position(|b| *b == Some(0)).expect("one explicit 0 at the disarm");
    assert!(out[..stop].iter().all(|b| *b == Some(600)), "{out:?}");
    assert!(quiet(&out[stop + 1..]), "{out:?}");
    assert!(rig.base.holds(Stream::Blade) && !rig.base.blade_active());

    // re-seated, still held: the wheels' idle streams re-arm, the blade not
    board.rx_lead = true;
    let back = press(&mut rig, &mut board, 50, true);
    assert!(quiet(&back), "restarted on the re-seat: {back:?}");
    assert_eq!(rig.board_receiving().len(), 1);
    assert!(!rig.base.holds(Stream::CmdVel) && !rig.base.holds(Stream::WheelOverride));
    assert!(rig.base.holds(Stream::Blade));
    assert_eq!(rig.base.disarmed(), Some(DisarmReason::BoardNotReceiving));

    // let go: re-armed once the last refresh's 500 ms have run out
    let released = press(&mut rig, &mut board, 15, false);
    assert!(quiet(&released));
    assert_eq!(rig.armed(Stream::Blade), Some(StopEdge::Silence));
    assert_eq!(rig.base.disarmed(), None);
    assert_eq!(press(&mut rig, &mut board, 5, true)[0], Some(600), "a fresh press runs it");

    // both leads, and an explicit stop as the release
    board.rx_lead = false;
    board.tx_lead = false;
    let out = press(&mut rig, &mut board, 40, true);
    assert!(rig.events.contains(&Event::Disarmed(DisarmReason::FeedbackLost)));
    assert!(out.contains(&Some(0)) && quiet(&out[out.iter().position(|b| *b == Some(0)).unwrap() + 1..]));
    board.rx_lead = true;
    board.tx_lead = true;
    assert!(quiet(&press(&mut rig, &mut board, 25, true)));
    assert!(rig.base.holds(Stream::Blade));
    rig.base.request_blade(0, 0, rig.t);
    assert!(quiet(&press(&mut rig, &mut board, RELEASE_CYCLES - 2, false)));
    assert!(rig.base.holds(Stream::Blade), "0.48 s after the stop");
    assert!(quiet(&press(&mut rig, &mut board, 1, false)));
    assert_eq!(rig.armed(Stream::Blade), Some(StopEdge::Stop));
    assert_eq!(press(&mut rig, &mut board, 5, true)[0], Some(600));

    // the C++ has no latch: the refresh restarts it on the re-seat
    let mut rig = Rig::new(BaseConfig { arm_latch: false, ..production() });
    let mut board = Board::new(T0);
    press(&mut rig, &mut board, 25, true);
    board.rx_lead = false;
    press(&mut rig, &mut board, 50, true);
    board.rx_lead = true;
    let reseat = rig.t;
    assert_eq!(press(&mut rig, &mut board, 1, true), vec![Some(600)]);
    assert!(board.accepted_after(reseat).len() == 1);
}

// ---- the release window ----------------------------------------------------
//
// The second supervised run (2026-09-29, image ecdd4a8, RUST_BASE, wheels
// off the ground, the app's teleop stream held forward by a script at
// 10 Hz): pin 8 pulled at 38.5 s, disarmed by the command path loss; the
// board received again at 39.6 s and the wheels stayed stopped under the
// held stick — both correct. Pulling the jumper had also reset the USB hub,
// and during that the robot side stalled for ~0.3 s: at 43.05 s
// velocity_command_guard's receipt watchdog forced the velocity to zero
// (then rejected the queued commands as stale), mower_base took that zero
// for the operator's release ("cmd_vel re-armed by an explicit stop, 4.48 s
// after the command path loss", 43.07 s), and 0.25 s later the stick that
// was still held drove the wheels, 73 -> 366 permille. A stop edge must be
// sustained: `latch_release_s` (0.5 s) with no non-zero command.

/// The robot's sequence: a held stream at 10 Hz, the command path lost and
/// recovered, then the guard's one zero with the stream quiet around it
/// (0.2 s before: the stall; 0.24 s after: the stale rejections), and the
/// stick again. Held, and said so once per disarm; the release that
/// follows (one zero, then nothing) re-arms 0.52 s after its zero. With the
/// window off (`latch_release_s` 0, the image that ran) the same script
/// re-arms on the zero and ramps 73, 146, 220, 293, 366 — the robot's log.
#[test]
fn the_guard_zero_after_a_command_path_loss_is_not_a_release() {
    for release_s in [0.5, 0.0] {
        let mut rig = Rig::new(BaseConfig { latch_release_s: release_s, ..production() });
        let mut board = Board::new(T0);
        let n = rig.release_on(&mut board);
        assert_eq!(n, if release_s > 0.0 { RELEASE_CYCLES } else { 2 });
        let moving = rig.teleop(&mut board, Some(0.4), 2.0);
        assert!(moving.last().unwrap().0 > 600);
        rig.events.clear();

        // pin 8 out for 1.1 s with the stick held, then back
        board.rx_lead = false;
        rig.teleop(&mut board, Some(0.4), 1.1);
        assert_eq!(rig.base.disarmed(), Some(DisarmReason::BoardNotReceiving));
        let reseat = rig.t;
        board.rx_lead = true;
        let held = rig.teleop(&mut board, Some(0.4), 3.4);
        assert!(all_zero(&held), "{held:?}");
        assert_eq!(rig.board_receiving().len(), 1, "{:?}", rig.events);
        assert!(rig.base.holds(Stream::CmdVel));

        // the stall: the stick's last message, 0.2 s of nothing (less than
        // cmd_vel_timeout, so not silence), the guard's zero, 0.24 s of
        // nothing, the stick again
        let mut after = vec![rig.drive(&mut board, Some((0.4, 0.0)))];
        after.extend(rig.drive_n(&mut board, 4, None));
        after.push(rig.drive(&mut board, Some((0.0, 0.0))));
        let zero_at = rig.t;
        after.extend(rig.drive_n(&mut board, 5, None));
        after.push(rig.drive(&mut board, Some((0.4, 0.0))));
        after.extend(rig.teleop(&mut board, Some(0.4), 2.0));
        if release_s == 0.0 {
            assert!(rig.events.iter().any(|e| matches!(
                e,
                Event::Armed { stream: Stream::CmdVel, by: StopEdge::Stop, .. }
            )));
            let driven: Vec<i16> = after.iter().map(|w| w.0).filter(|l| *l != 0).take(5).collect();
            assert_eq!(driven, [73, 146, 220, 293, 366], "{after:?}");
            assert!(board.applied_after(zero_at).iter().any(|w| w.0 > 0));
            continue;
        }
        assert!(all_zero(&after), "{after:?}");
        assert!(all_zero(&board.applied_after(reseat)), "the board drove after the re-seat");
        assert!(rig.base.holds(Stream::CmdVel));
        assert_eq!(rig.armed(Stream::CmdVel), None, "{:?}", rig.events);
        let not_held = rig.not_held();
        assert_eq!(not_held.len(), 1, "{:?}", rig.events);
        let (stream, by, after_s) = not_held[0];
        assert_eq!((stream, by), (Stream::CmdVel, StopEdge::Stop));
        assert!((after_s - 0.24).abs() < 1e-6, "{after_s}");

        // the same stall again: still held, and not said again
        rig.drive_n(&mut board, 4, None);
        rig.drive(&mut board, Some((0.0, 0.0)));
        rig.drive_n(&mut board, 5, None);
        assert!(all_zero(&rig.teleop(&mut board, Some(0.4), 1.0)));
        assert_eq!(rig.not_held().len(), 1, "once per disarm: {:?}", rig.events);

        // released: the app sends one zero and stops
        rig.drive(&mut board, Some((0.0, 0.0)));
        rig.drive_n(&mut board, RELEASE_CYCLES - 2, None);
        assert!(rig.base.holds(Stream::CmdVel), "0.48 s after the release");
        rig.drive(&mut board, None);
        assert_eq!(rig.armed(Stream::CmdVel), Some(StopEdge::Stop));
        assert_eq!(rig.drive(&mut board, Some((0.4, 0.0))), (73, 73));
        assert_eq!(rig.not_held().len(), 1);
    }
}

/// Silence needs the release window too, not just cmd_vel_timeout: a
/// stall of the stream between 0.25 and 0.5 s (here after a feedback loss,
/// the stick held through the re-seat) is not a release. A gap longer than
/// the window is.
#[test]
fn a_stall_shorter_than_the_release_window_is_not_a_release() {
    let mut rig = Rig::new(production());
    rig.release(true);
    rig.run(55, Some((0.4, 0.0)), true);
    rig.run(20, Some((0.4, 0.0)), false);
    assert_eq!(rig.base.disarmed(), Some(DisarmReason::FeedbackLost));
    let back = rig.run(20, Some((0.4, 0.0)), true);
    assert!(all_zero(&back));
    rig.events.clear();

    // 0.28 s of nothing: past cmd_vel_timeout (the controller brakes, and
    // the rule without the window re-armed here), then the stick
    let mut frames = rig.run(7, None, true);
    frames.extend(rig.run(10, Some((0.4, 0.0)), true));
    // 0.48 s of nothing, then the stick
    frames.extend(rig.run(12, None, true));
    frames.extend(rig.run(10, Some((0.4, 0.0)), true));
    assert!(all_zero(&frames), "{frames:?}");
    assert!(rig.base.holds(Stream::CmdVel));
    assert_eq!(rig.armed(Stream::CmdVel), None, "{:?}", rig.events);
    let not_held = rig.not_held();
    assert_eq!(not_held.len(), 1, "{:?}", rig.events);
    let (stream, by, after_s) = not_held[0];
    assert_eq!((stream, by), (Stream::CmdVel, StopEdge::Silence));
    assert!((after_s - 0.32).abs() < 1e-6, "{after_s}");

    // a real stop of the stream: more than 0.5 s of nothing
    rig.run(12, None, true);
    assert!(rig.base.holds(Stream::CmdVel), "0.48 s");
    rig.step(None, true);
    assert_eq!(rig.armed(Stream::CmdVel), Some(StopEdge::Silence));
    assert_eq!(rig.step(Some((0.4, 0.0)), true), (73, 73));
}

/// A real release is often one zero and then nothing: that re-arms once
/// the window from the zero is up, with nothing to report.
#[test]
fn a_real_release_is_one_zero_then_silence() {
    let mut rig = Rig::new(production());
    rig.release(true);
    rig.run(55, Some((0.4, 0.0)), true);
    rig.run(20, Some((0.4, 0.0)), false);
    rig.run(20, Some((0.4, 0.0)), true);
    assert!(rig.base.holds(Stream::CmdVel));
    rig.events.clear();

    rig.step(Some((0.0, 0.0)), true);
    let quiet = rig.run(RELEASE_CYCLES - 2, None, true);
    assert!(all_zero(&quiet));
    assert!(rig.base.holds(Stream::CmdVel), "0.48 s after the zero");
    rig.step(None, true);
    assert_eq!(rig.armed(Stream::CmdVel), Some(StopEdge::Stop));
    assert!(rig.not_held().is_empty(), "{:?}", rig.events);
    assert_eq!(rig.step(Some((0.4, 0.0)), true), (73, 73));

    // and a stop the stream keeps sending (a stick at rest at 25 Hz) is
    // one release, not one per zero: the window runs from the first
    rig.run(5, Some((0.4, 0.0)), false);
    rig.run(20, Some((0.4, 0.0)), false);
    rig.run(5, Some((0.4, 0.0)), true);
    assert!(rig.base.holds(Stream::CmdVel));
    assert_eq!(rig.release(true), RELEASE_CYCLES);
}

/// wheel_override gets the same window: pid_autotune's stream (every
/// 100 ms, ttl 300) held through a feedback loss, one cancel, and the stream
/// again 0.2 s later is still held, not applied, said once. Its silence
/// runs against its own ttl too: a writer that refreshes every 0.6 s with a
/// 1 s ttl is holding the wheels the whole time. A cancel left alone for
/// the window re-arms it.
#[test]
fn a_cancel_cut_short_does_not_rearm_the_override() {
    let mut rig = Rig::new(production());
    rig.release(true);
    rig.run(60, None, true);
    assert_eq!(rig.base.disarmed(), None);
    let mut board = Board::new(rig.t);
    assert_eq!(*rig.hold(&mut board, 400, 1.0).last().unwrap(), (400, 400));
    board.tx_lead = false;
    board.rx_lead = false;
    rig.hold(&mut board, 400, 1.2);
    assert_eq!(rig.base.disarmed(), Some(DisarmReason::FeedbackLost));
    board.tx_lead = true;
    board.rx_lead = true;
    assert!(all_zero(&rig.hold(&mut board, 400, 1.0)));
    assert_eq!(rig.board_receiving().len(), 1, "{:?}", rig.events);
    assert!(!rig.base.holds(Stream::CmdVel), "the idle cmd_vel stream re-armed by silence");
    assert!(rig.base.holds(Stream::WheelOverride));
    rig.events.clear();

    rig.base.request_wheel_override(0, 0, 0, rig.t);
    let mut frames = rig.drive_n(&mut board, 5, None);
    frames.extend(rig.hold(&mut board, 400, 1.0));
    assert!(all_zero(&frames), "{frames:?}");
    assert!(rig.base.holds(Stream::WheelOverride));
    assert_eq!(rig.not_held(), vec![(Stream::WheelOverride, StopEdge::Stop, 0.2)]);

    // refreshed every 0.6 s with a 1 s ttl: gaps longer than the window,
    // shorter than the ttl
    for _ in 0..5 {
        rig.base.request_wheel_override(400, 400, 1000, rig.t);
        assert!(all_zero(&rig.drive_n(&mut board, 15, None)));
    }
    assert!(rig.base.holds(Stream::WheelOverride), "{:?}", rig.events);
    assert_eq!(rig.not_held().len(), 1);

    rig.base.request_wheel_override(0, 0, 0, rig.t);
    rig.drive_n(&mut board, RELEASE_CYCLES - 2, None);
    assert!(rig.base.holds(Stream::WheelOverride));
    rig.drive(&mut board, None);
    assert_eq!(rig.armed(Stream::WheelOverride), Some(StopEdge::Stop));
    assert_eq!(*rig.hold(&mut board, 400, 0.3).last().unwrap(), (400, 400));
}

/// The blade, which the window protects the same way: held through a pull
/// of pin 8, one 0 and a refresh 0.2 s later is not the dead-man let go
/// (whoever sent the 0), nor is a 0.3 s stall of a sender whose ttl is
/// shorter than that; the blade stays off. A 0 left alone for the window,
/// or a refresh left to run out for it, is a release.
#[test]
fn a_blade_stop_cut_short_is_not_a_release() {
    fn press(rig: &mut Rig, board: &mut Board, n: usize, ttl_ms: Option<i64>, every: usize) {
        for k in 0..n {
            if let Some(ttl) = ttl_ms.filter(|_| k % every == 0) {
                rig.base.request_blade(600, ttl, rig.t);
            }
            rig.drive(board, None);
            assert_eq!(blade_of(&rig.last_tx).filter(|b| *b != 0), None, "blade restarted");
        }
    }
    for release_s in [0.5, 0.0] {
        let mut rig = Rig::new(BaseConfig { latch_release_s: release_s, ..production() });
        let mut board = Board::new(T0);
        rig.release_on(&mut board);
        rig.drive_n(&mut board, 60, None);
        rig.base.request_blade(600, 500, rig.t);
        rig.drive(&mut board, None);
        assert_eq!(blade_of(&rig.last_tx), Some(600));
        board.rx_lead = false;
        let mut stopped = false;
        for k in 0..30 {
            if k % 5 == 0 {
                rig.base.request_blade(600, 500, rig.t);
            }
            rig.drive(&mut board, None);
            stopped |= blade_of(&rig.last_tx) == Some(0);
        }
        assert!(stopped && rig.base.holds(Stream::Blade));
        board.rx_lead = true;
        press(&mut rig, &mut board, 25, Some(500), 5);
        assert_eq!(rig.board_receiving().len(), 1);
        rig.events.clear();

        // one 0, the refresh 0.2 s later
        rig.base.request_blade(0, 0, rig.t);
        rig.drive_n(&mut board, 5, None);
        if release_s == 0.0 {
            // without the window: the 0 re-armed it, the refresh runs it
            assert_eq!(rig.armed(Stream::Blade), Some(StopEdge::Stop));
            rig.base.request_blade(600, 500, rig.t);
            rig.drive(&mut board, None);
            assert_eq!(blade_of(&rig.last_tx), Some(600));
            continue;
        }
        press(&mut rig, &mut board, 25, Some(500), 5);
        assert!(rig.base.holds(Stream::Blade));
        assert_eq!(rig.not_held(), vec![(Stream::Blade, StopEdge::Stop, 0.2)]);

        // a sender with a 200 ms ttl, refreshing every 0.12 s, stalls 0.32 s
        press(&mut rig, &mut board, 9, Some(200), 3);
        press(&mut rig, &mut board, 8, None, 1);
        press(&mut rig, &mut board, 9, Some(200), 3);
        assert!(rig.base.holds(Stream::Blade), "{:?}", rig.events);
        assert_eq!(rig.not_held().len(), 1, "once per disarm");

        // let go after the last refresh (three cycles ago, 200 ms ttl): the
        // ttl runs out, the window 0.52 s after the refresh re-arms it
        press(&mut rig, &mut board, 9, None, 1);
        assert!(rig.base.holds(Stream::Blade), "0.48 s: {:?}", rig.events);
        press(&mut rig, &mut board, 1, None, 1);
        assert_eq!(rig.armed(Stream::Blade), Some(StopEdge::Silence));
        rig.base.request_blade(600, 500, rig.t);
        rig.drive(&mut board, None);
        assert_eq!(blade_of(&rig.last_tx), Some(600), "a fresh press runs it");
    }
}

// ---- clocks ----------------------------------------------------------------

/// ROS time (the robot's wall clock) as the publisher and the driver both
/// read it, `real_ns` of control-clock time after activation: stepped back
/// 5 s just before the 51st cycle and forward 10 s just before the 101st,
/// each step landing between a message's stamp and its receipt.
fn ros_at(real_ns: i64, stepped: bool) -> i64 {
    let mut ros = ROS0 + real_ns;
    if stepped && real_ns >= 51 * DT - 2 * MS {
        ros -= 5_000 * MS;
    }
    if stepped && real_ns >= 101 * DT - 2 * MS {
        ros += 10_000 * MS;
    }
    ros
}

/// One run of the cmd_vel path end to end in the core: each message is
/// stamped by the (possibly stepped) ROS clock 5 ms before it is received,
/// goes through `receive_command` against the same clock, and the cycle runs
/// on the control clock. `cmd(k)` is the command sent for cycle k.
fn run_through_a_step(
    stepped: bool,
    cycles: i64,
    cmd: impl Fn(i64) -> Option<Twist>,
) -> Vec<(Vec<u8>, Option<(f64, f64, f64)>, String)> {
    let (mut base, _) = BaseCycle::new(production(), T0).unwrap();
    let mut out = Vec::new();
    for k in 1..=cycles {
        let real = k * DT;
        let t = T0 + real;
        let ros = ros_at(real, stepped);
        base.set_publishers(Stream::CmdVel, true, t);
        base.set_publishers(Stream::WheelOverride, true, t);
        if k % 2 == 0 {
            base.on_rx(&feedback((k * 10) as i32), t);
        }
        if k == 20 {
            base.request_blade(600, 1000, t);
        }
        if k == 70 {
            base.request_wheel_override(300, 300, 1000, t);
        }
        let command = cmd(k).and_then(|twist| {
            let stamp = ros_at(real - 5 * MS, stepped);
            match receive_command(twist, stamp, ros, t, 0.25) {
                Received::Accepted { command, .. } => Some(command),
                Received::Stale { .. } => None,
            }
        });
        let (tx, odom, joints) = base.tick(command, t, ros);
        if let Some(o) = &odom {
            assert_eq!(o.stamp_ns, ros, "odom carries ROS time");
        }
        assert_eq!(joints.as_ref().unwrap().stamp_ns, ros);
        let telemetry = base.telemetry().to_json(mower_base_core::seconds(t), base.blade_active());
        out.push((tx.unwrap().bytes, odom.map(|o| (o.x, o.y, o.yaw)), telemetry));
    }
    out
}

/// A wall-clock step while commands flow changes the stamps and nothing
/// else: stepping ROS time back 5 s and then forward 10 s, with a message in
/// flight across each step, gives byte-identical frames, the same odometry
/// and the same telemetry, and the blade and override dead-men last as long.
/// (The in-flight message at the forward step is 10 s old on arrival and
/// ignored, as upstream ignores it; the one before it is 40 ms older than
/// it would have been, well inside the timeout.)
#[test]
fn a_wall_clock_step_under_a_live_command_changes_nothing() {
    let cmd = |k: i64| match k {
        1..=14 => Some(Twist::new(0.0, 0.0)),
        15..=40 => Some(Twist::new(0.4, 0.3)),
        41..=130 => Some(Twist::new(0.2, 0.0)),
        _ => None,
    };
    let steady = run_through_a_step(false, 150, cmd);
    let stepped = run_through_a_step(true, 150, cmd);
    for (k, (a, b)) in steady.iter().zip(&stepped).enumerate() {
        assert_eq!(a, b, "cycle {} differs", k + 1);
    }
    // and the run did what it says: moving through both steps, the override
    // applied, stopped at the end
    assert!(wheels(&TxFrame { bytes: steady[99].0.clone(), frames: vec![] }).0 > 0);
    assert_eq!(wheels(&TxFrame { bytes: steady[70].0.clone(), frames: vec![] }), (300, 300));
    assert_eq!(wheels(&TxFrame { bytes: steady[149].0.clone(), frames: vec![] }), (0, 0));
}

/// The stream stops right after a message that crossed a backward step: it
/// still times out 0.25 s after it arrived. (Upstream ages it from its
/// stamp, which the step put 5 s in the future, and would keep driving on it
/// for 5.25 s.)
#[test]
fn a_backward_step_does_not_extend_the_last_command() {
    let cmd = |k: i64| match k {
        1..=14 => Some(Twist::new(0.0, 0.0)),
        15..=51 => Some(Twist::new(0.3, 0.0)),
        _ => None,
    };
    let steady = run_through_a_step(false, 80, cmd);
    let stepped = run_through_a_step(true, 80, cmd);
    let permille = |run: &Vec<(Vec<u8>, _, _)>, k: usize| {
        wheels(&TxFrame { bytes: run[k - 1].0.clone(), frames: vec![] })
    };
    assert!(permille(&stepped, 57).0 > 500, "0.24 s after the last message");
    assert_eq!(permille(&stepped, 58), (0, 0), "0.28 s after it: timed out, braked");
    for k in 1..=80 {
        assert_eq!(permille(&steady, k), permille(&stepped, k), "cycle {k}");
    }
}

/// `BaseCycle` itself never reads the `stamp` argument for timing: the same
/// control times with a jumping `stamp` give the same bytes.
#[test]
fn the_cycle_times_nothing_on_the_stamp_argument() {
    let run = |jump: bool| {
        let (mut base, _) = BaseCycle::new(production(), T0).unwrap();
        (1..=100i64)
            .map(|k| {
                let t = T0 + k * DT;
                let stamp = if jump && k % 7 == 0 { 0 } else { ROS0 + k * DT };
                base.on_rx(&feedback(k as i32), t);
                let v = if k <= 14 { 0.0 } else { 0.4 };
                let cmd = (k < 60).then_some(Command { twist: Twist::new(v, 0.0), stamp_ns: t });
                base.tick(cmd, t, stamp).0.unwrap().bytes
            })
            .collect::<Vec<_>>()
    };
    let steady = run(false);
    assert_eq!(steady, run(true));
    assert!(wheels(&TxFrame { bytes: steady[30].clone(), frames: vec![] }).0 > 600);
}

// ---- the log events ------------------------------------------------------

#[test]
fn the_cpp_log_transitions_are_reported() {
    let mut rig = Rig::new(production());
    rig.step(Some((0.0, 0.0)), true);
    rig.run(10, Some((0.3, 0.0)), true);
    rig.events.clear();

    // a moving command times out: reported once, not every second after
    rig.run(30, None, true);
    let timeouts = rig.events.iter().filter(|e| matches!(e, Event::CmdVelTimedOut { .. })).count();
    assert_eq!(timeouts, 1, "{:?}", rig.events);
    // a zero that times out brakes nothing and says nothing
    rig.step(Some((0.0, 0.0)), true);
    rig.events.clear();
    rig.run(30, None, true);
    assert!(!rig.events.iter().any(|e| matches!(e, Event::CmdVelTimedOut { .. })));

    // blade dead-man edges
    rig.base.request_blade(600, 200, rig.t);
    rig.run(10, None, true);
    assert!(rig.events.contains(&Event::Blade { running: true, permille: 600 }));
    assert!(rig.events.contains(&Event::Blade { running: false, permille: 600 }));

    // driver alarm, at most every 2 s
    rig.events.clear();
    let mut alarm = [0u8; 12];
    alarm[10] = STATUS_FLAG_DRIVER_ALARM;
    for _ in 0..75 {
        rig.base.on_rx(&build_frame(MOTOR_STATUS, 0, &alarm), rig.t + DT);
        rig.step(None, true);
    }
    let alarms = rig.events.iter().filter(|e| **e == Event::DriverAlarm).count();
    assert_eq!(alarms, 2, "3 s of alarm frames at 25 Hz");
}
