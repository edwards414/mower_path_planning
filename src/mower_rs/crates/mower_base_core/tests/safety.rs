//! The behaviour `mower_base` adds on top of the C++ chain, or keeps apart
//! from it: the arm latch, the control clock vs. ROS time, and the cmd_vel
//! stamp handling at the cycle level. (The driver's own clock wiring, which
//! clock feeds which argument, is tested in `mower_base`.)
//!
//! Everything runs with the production controller parameters
//! (`mower_controller/controllers/diff_drive_controller.yaml`: open loop,
//! 0.25 s timeout, -25 / -50 deceleration), which is what the robot runs.

use mower_base_core::cycle::{
    BaseConfig, BaseCycle, DisarmReason, Event, StopEdge, Stream, TxFrame,
};
use mower_base_core::diff_drive::{
    receive_command, Command, DiffDriveParams, LimitParams, OdomSample, Received, Twist,
};
use mower_base_core::protocol::{
    build_frame, MOTOR_STATUS, STATUS_FLAG_DRIVER_ALARM, WHEEL_FEEDBACK_STATUS,
};

const MS: i64 = 1_000_000;
const DT: i64 = 40 * MS;
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
}

impl Rig {
    fn new(cfg: BaseConfig) -> Self {
        let (base, _) = BaseCycle::new(cfg, T0).unwrap();
        Rig { base, t: T0, events: Vec::new(), last_odom: None, publishers: true }
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
/// until the stream shows a stop edge — here an explicit zero.
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

    // the stop edge, then a new command moves the wheels at the normal ramp
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

/// Silence for longer than cmd_vel_timeout is the other stop edge, counted
/// once the publisher has been there for `publisher_settle_s`: here it is
/// seen on the first cycle (40 ms), so silence runs from 2.04 s.
#[test]
fn activation_rearms_after_silence() {
    let mut rig = Rig::new(production());
    let frames = rig.run(57, None, true); // 2.28 s
    assert!(all_zero(&frames));
    assert!(rig.base.holds(Stream::CmdVel), "2.28 - 2.04 s is not yet > 0.25 s");
    rig.step(None, true); // 2.32 s
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

        // the stream stops: re-armed by the zero, then it drives
        rig.step(Some((0.0, 0.0)), true);
        assert_eq!(rig.armed(Stream::CmdVel), Some(StopEdge::Stop));
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
    // then nothing: re-armed 0.25 s after it
    rig.step(Some((0.3, 0.0)), true);
    let quiet = rig.run(6, None, true);
    assert!(all_zero(&quiet));
    assert!(rig.base.holds(Stream::CmdVel));
    rig.step(None, true);
    assert_eq!(rig.armed(Stream::CmdVel), Some(StopEdge::Silence));
    assert!(rig.base.holds(Stream::WheelOverride), "the override stream is still unheard");
}

/// A zero followed by a new command in a later cycle re-arms at the zero; a
/// non-zero after a gap shorter than the timeout does not count as silence.
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
    rig.step(Some((0.0, 0.0)), true);
    let moving = rig.run(40, Some((0.4, 0.0)), true);
    assert!(moving.last().unwrap().0 > 600);
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
    rig.step(Some((0.0, 0.0)), true);
    assert_eq!(rig.armed(Stream::CmdVel), Some(StopEdge::Stop));
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
    rig.run(60, None, true); // the override stream too, by silence
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
    rig.step(Some((0.0, 0.0)), true);
    assert_eq!(rig.armed(Stream::CmdVel), Some(StopEdge::Stop));
    assert_eq!(rig.step(Some((0.4, 0.0)), true), (73, 73));

    // the same through wheel_override: requested during an outage, still
    // streaming on the re-seat, not applied until cancelled
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
    rig.step(Some((0.0, 0.0)), false);
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
    rig.step(Some((0.0, 0.0)), true); // cmd_vel armed; the override is not

    // an override stream that was running before the restart
    let mut held = Vec::new();
    for _ in 0..20 {
        rig.base.request_wheel_override(400, -400, 300, rig.t);
        held.push(rig.step(None, true));
    }
    assert!(all_zero(&held), "{held:?}");
    assert!(rig.base.holds(Stream::WheelOverride));

    // the stream stops; once it has been quiet for 250 ms (and its
    // publisher has been known for 2 s) it is armed, and a new request applies
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

    // an explicit cancel (ttl 0) is a stop edge too
    rig.base.request_wheel_override(0, 0, 0, rig.t);
    rig.step(None, true);
    assert_eq!(rig.armed(Stream::WheelOverride), Some(StopEdge::Stop));
    rig.base.request_wheel_override(-200, 200, 300, rig.t);
    assert_eq!(rig.step(None, true), (-200, 200));
}

/// Parity mode: with the latch off the cycle follows a live command at once,
/// like the C++ chain.
#[test]
fn the_latch_can_be_turned_off_for_the_parity_tests() {
    let mut rig = Rig::new(BaseConfig { arm_latch: false, ..production() });
    assert_eq!(rig.base.disarmed(), None);
    assert_eq!(rig.step(Some((0.4, 0.0)), true), (73, 73));
    let lost = rig.run(40, Some((0.4, 0.0)), false);
    assert!(lost.last().unwrap().0 > 600, "no latch on feedback loss either");
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
        1 => Some(Twist::new(0.0, 0.0)),
        2..=40 => Some(Twist::new(0.4, 0.3)),
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
        1 => Some(Twist::new(0.0, 0.0)),
        2..=51 => Some(Twist::new(0.3, 0.0)),
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
                let v = if k == 1 { 0.0 } else { 0.4 };
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
