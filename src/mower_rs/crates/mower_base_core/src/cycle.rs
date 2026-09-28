//! The base hardware cycle as a state machine.
//!
//! Port of the ros2_control `read()` / `write()` semantics in
//! `src/mower_hardware/src/mower_system.cpp`, with the
//! `diff_drive_controller` update ([`crate::diff_drive`]) folded in, so the
//! whole base is two calls:
//!
//! ```text
//!   on_rx(bytes, now)                             <- whatever the serial read returned
//!   tick(cmd_vel, now, stamp) -> (tx, odom, js)   <- one 25 Hz control cycle
//! ```
//!
//! `now` is the control clock (monotonic, what ros2_control's steady trigger
//! clock is): every period, age, deadline and throttle is measured on it, so
//! a wall-clock step changes nothing. `stamp` is ROS time and only ever ends
//! up in the header of a published message.
//!
//! Unit conventions, all from `mower_system.cpp`:
//!
//! * 0x85 `measured_rpm` -> joint velocity `rpm * 2*pi / 60` rad/s.
//! * 0x85 `total_counts` -> joint position: the *wrapped* signed 32-bit
//!   difference times `2*pi / counts_per_rev` rad, accumulated. The absolute
//!   counter value is never used, so position starts at 0 on activation and
//!   the first frame after activation contributes nothing.
//! * commanded joint velocity (rad/s) -> 0x01 permille:
//!   `round(clamp(rad_s * 60 / (2*pi) / max_rpm * 1000, -1000, 1000))`.
//! * every 0x01 carries `command_timeout_ms` (300 ms), so the firmware stops
//!   the wheels by itself if this loop stalls or the link dies.
//!
//! Safety paths:
//!
//! * `cmd_vel_timeout` (0.5 s) zeroes the controller reference, and the speed
//!   limiter then ramps the command down at `max_acceleration` rather than
//!   dropping it — that ramp *is* the safety-zero deceleration.
//! * `feedback_timeout_s` (0.5 s) without a 0x85 zeroes the reported joint
//!   velocities (positions are held), so odometry stops inventing motion.
//! * [`BaseCycle::fault`] models the `return_type::ERROR` paths (serial read
//!   or write failure): the cycle latches, stops commanding and emits the
//!   stop burst, which is what `on_deactivate` does on the real stack.
//! * The **arm latch** (not in the C++; `BaseConfig::arm_latch`): after every
//!   activation, and when 0x85 feedback that had been arriving stops for
//!   longer than `feedback_timeout_s`, the cmd_vel reference is held at zero
//!   and `wheel_override` is not applied, each until its own stream shows a
//!   stop edge — see [`BaseCycle::disarmed`]. The C++ chain latched off for
//!   good after a runtime error; this driver is restarted after 2 s instead,
//!   and without the latch it would resume a live nav2 or teleop command on
//!   its own, and so would a re-seated UART lead.

use crate::diff_drive::{Command, DiffDrive, DiffDriveParams, OdomSample, WheelCommand};
use crate::protocol::{self as proto, FrameParser, PidConfig};
use crate::{seconds, TimeNs, TWO_PI};

/// Everything `mower_system.cpp` takes as a `<param>` in the ros2_control URDF,
/// plus the controller parameters.
#[derive(Debug, Clone, PartialEq)]
pub struct BaseConfig {
    /// firmware `wheel_max_rpm`; 1000 permille = this.
    pub max_rpm: f64,
    /// FT-555 16 PPR x4 x 139:1.
    pub counts_per_rev: f64,
    /// Put into every 0x01 / 0x02 so the STM32 stops itself.
    pub command_timeout_ms: u16,
    /// Declare the wheel velocities stale if no 0x85 for this long.
    pub feedback_timeout_s: f64,
    /// `controller_manager.update_rate`; only used for the very first cycle,
    /// where there is no previous tick to measure a period against.
    pub update_rate_hz: f64,
    /// Re-assert the light state this often in case a frame was lost.
    pub led_resend_period_s: f64,
    /// `kOverrideMaxTtlMs`.
    pub override_max_ttl_ms: i64,
    /// `kBladeMaxTtlMs`.
    pub blade_max_ttl_ms: i64,
    /// Hold the wheels after (re)activation and feedback loss until the
    /// command streams show a stop edge. Off only to reproduce the C++
    /// chain, which has no such latch, in the parity tests.
    pub arm_latch: bool,
    pub diff_drive: DiffDriveParams,
}

impl Default for BaseConfig {
    /// The defaults in `mower_system.cpp::on_init` plus
    /// `config/mower_controllers.yaml`.
    fn default() -> Self {
        Self {
            max_rpm: 58.0,
            counts_per_rev: 8896.0,
            command_timeout_ms: 300,
            feedback_timeout_s: 0.5,
            update_rate_hz: 25.0,
            led_resend_period_s: 5.0,
            override_max_ttl_ms: 1000,
            blade_max_ttl_ms: 1000,
            arm_latch: true,
            diff_drive: DiffDriveParams::mower(),
        }
    }
}

/// One wheel joint's hardware-interface state.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Wheel {
    /// rad/s, from the controller
    pub cmd_velocity: f64,
    /// rad, integrated from total counts
    pub pos: f64,
    /// rad/s, from measured rpm
    pub vel: f64,
    last_total_counts: i32,
    have_counts: bool,
}

/// `sensor_msgs/JointState` for the two wheel joints.
#[derive(Debug, Clone, PartialEq)]
pub struct JointStates {
    pub stamp_ns: TimeNs,
    pub names: [&'static str; 2],
    /// rad
    pub positions: [f64; 2],
    /// rad/s
    pub velocities: [f64; 2],
}

/// The bytes one cycle wants written to the port, in order.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct TxFrame {
    pub bytes: Vec<u8>,
    /// `(type, seq)` of each frame in `bytes`, for logs and assertions.
    pub frames: Vec<(u8, u8)>,
}

impl TxFrame {
    fn push(&mut self, frame: Vec<u8>) {
        if frame.len() >= 5 {
            self.frames.push((frame[3], frame[4]));
        }
        self.bytes.extend_from_slice(&frame);
    }
    pub fn is_empty(&self) -> bool {
        self.bytes.is_empty()
    }
}

/// Latest decoded status frames, what `/mower_base/telemetry` is built from.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Telemetry {
    pub wheel: Option<proto::WheelFeedback>,
    pub motor: Option<proto::MotorStatus>,
    pub pid: Option<proto::PidConfigStatus>,
    pub led: Option<proto::Ws2812Status>,
    pub power: Option<proto::PowerStatus>,
    pub charger: Option<proto::ChargerStatus>,
    pub servo: Option<proto::ServoStatus>,
    pub blade: Option<proto::LawerMotorStatus>,
    pub firmware: Option<proto::FirmwareInfo>,
    pub crc_errors: usize,
    pub feedback_age_s: f64,
}

/// A WS2812 request, packed the way `pack_led` packs it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LedRequest {
    pub mode: u8,
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub period_ms: u16,
    /// Bumped per request so an identical repeat is still re-sent.
    pub serial: u8,
}

/// Why the arm latch holds the wheels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DisarmReason {
    /// [`BaseCycle::new`] / [`BaseCycle::clear_fault`].
    Activation,
    /// 0x85 feedback had been arriving and then stopped for longer than
    /// `feedback_timeout_s`.
    FeedbackLost,
}

/// The two motion streams the arm latch holds, each on its own.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Stream {
    CmdVel,
    WheelOverride,
}

/// Which stop edge re-armed a stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StopEdge {
    /// An explicit stop: a finite cmd_vel with linear.x and angular.z both
    /// 0, or a wheel_override of 0/0 or with ttl 0.
    Stop,
    /// Nothing on the stream for longer than `cmd_vel_timeout`.
    Silence,
}

/// Something the driver should log; drained with [`BaseCycle::take_events`].
/// The texts the C++ logged for the same transitions are in the comments.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Event {
    /// The latch now holds both streams.
    Disarmed(DisarmReason),
    /// `stream` showed a stop edge `after_s` s after the disarm: it moves the
    /// wheels again.
    Armed { stream: Stream, reason: DisarmReason, by: StopEdge, after_s: f64 },
    /// "no wheel feedback for %.2f s" — once per loss.
    FeedbackLost { age_s: f64 },
    /// "feedback resumed"
    FeedbackResumed,
    /// Not one 0x85 in `feedback_timeout_s` since activation. Once, and
    /// deliberately *not* latched: a board that never sends feedback is not
    /// a board that stopped sending it.
    NoFeedbackSinceActivation { age_s: f64 },
    /// "driver alarm flag set" (0x81 flags), at most every 2 s.
    DriverAlarm,
    /// "wheel override active (controller command bypassed)" /
    /// "wheel override expired, back to controller command"
    Override { active: bool },
    /// "blade running at %d permille (dead-man held)" / "blade stop"
    Blade { running: bool, permille: i16 },
    /// diff_drive_controller's "Velocity command timed out. Braking.",
    /// once per timeout and only when it brakes a non-zero command
    /// (upstream repeats it every second, idle or not).
    CmdVelTimedOut { linear_x: f64, angular_z: f64 },
}

/// `C++ RCLCPP_WARN_THROTTLE(..., 2000, "driver alarm flag set")`.
const DRIVER_ALARM_LOG_PERIOD_NS: TimeNs = 2_000_000_000;

/// One stream's half of the arm latch; `Some` = held.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Hold {
    reason: DisarmReason,
    since_ns: TimeNs,
    /// the newest message on the stream since `since_ns` was a stop
    stopped: bool,
}

impl Hold {
    /// The stop edge, if the stream has shown one: the newest message since
    /// the disarm was a stop, or nothing has arrived for longer than
    /// `window_ns` (measured from the disarm or the newest message,
    /// whichever is later). A `window_ns` of 0 accepts only explicit stops.
    fn edge(&self, last_ns: Option<TimeNs>, now: TimeNs, window_ns: i64) -> Option<StopEdge> {
        if self.stopped {
            return Some(StopEdge::Stop);
        }
        let from = last_ns.map_or(self.since_ns, |t| t.max(self.since_ns));
        (window_ns > 0 && now - from > window_ns).then_some(StopEdge::Silence)
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct DeadMan {
    /// Absolute deadline; `None` = nothing held.
    until_ns: Option<TimeNs>,
    value: i16,
    active: bool,
}

/// The whole base driver minus the serial port.
#[derive(Debug, Clone)]
pub struct BaseCycle {
    cfg: BaseConfig,
    parser: FrameParser,
    ddc: DiffDrive,
    left: Wheel,
    right: Wheel,
    tx_seq: u8,
    feedback_valid: bool,
    last_feedback_ns: TimeNs,
    feedback_age_s: f64,
    feedback_stale: bool,
    last_tick_ns: Option<TimeNs>,
    faulted: bool,

    /// The arm latch, per stream.
    cmd_hold: Option<Hold>,
    override_hold: Option<Hold>,
    activated_ns: TimeNs,
    /// control-clock time of the last cycle that picked up a cmd_vel
    last_cmd_ns: Option<TimeNs>,
    /// control-clock time of the last wheel_override request
    last_override_ns: Option<TimeNs>,
    no_feedback_reported: bool,
    driver_alarm_logged_ns: Option<TimeNs>,
    events: Vec<Event>,

    telemetry: Telemetry,
    /// `telemetry_pending_`: a new 0x85 arrived since the last publish.
    telemetry_pending: bool,
    firmware_info: Option<proto::FirmwareInfo>,
    firmware_protocol_mismatch: bool,
    shutdown_acked: bool,
    shutdown_request: Option<u8>,

    /// bytes already sequenced inside `on_rx` (the 0x05 shutdown ack), to be
    /// written before anything the next `tick` produces — which is the order
    /// the C++ writes them in, `read()` running before `write()`.
    pending_tx: TxFrame,

    blade: DeadMan,
    wheel_override: DeadMan,
    override_right: i16,
    servo_request: Option<(u16, u16)>,
    led_request: Option<LedRequest>,
    led_sent: Option<LedRequest>,
    led_sent_ns: Option<TimeNs>,
    pid_request: Option<PidConfig>,
}

impl BaseCycle {
    /// `on_configure` + `on_activate` at `now`. The returned frame is
    /// `send_stop()` (wheels and blade to 0) followed by an 0x06 info request,
    /// exactly what `on_activate` writes.
    pub fn new(cfg: BaseConfig, now: TimeNs) -> Result<(Self, TxFrame), String> {
        let ddc = DiffDrive::new(cfg.diff_drive.clone(), now)?;
        let mut me = Self {
            cfg,
            parser: FrameParser::new(),
            ddc,
            left: Wheel::default(),
            right: Wheel::default(),
            tx_seq: 0,
            feedback_valid: false,
            last_feedback_ns: 0,
            feedback_age_s: 0.0,
            feedback_stale: false,
            last_tick_ns: None,
            faulted: false,
            cmd_hold: None,
            override_hold: None,
            activated_ns: now,
            last_cmd_ns: None,
            last_override_ns: None,
            no_feedback_reported: false,
            driver_alarm_logged_ns: None,
            events: Vec::new(),
            telemetry: Telemetry::default(),
            telemetry_pending: false,
            firmware_info: None,
            firmware_protocol_mismatch: false,
            shutdown_acked: false,
            shutdown_request: None,
            pending_tx: TxFrame::default(),
            blade: DeadMan::default(),
            wheel_override: DeadMan::default(),
            override_right: 0,
            servo_request: None,
            led_request: None,
            led_sent: None,
            led_sent_ns: None,
            pid_request: None,
        };
        me.disarm(DisarmReason::Activation, now);
        let mut tx = me.stop_burst();
        tx.push(proto::build_info_request(me.next_seq()));
        Ok((me, tx))
    }

    pub fn config(&self) -> &BaseConfig {
        &self.cfg
    }
    pub fn left(&self) -> Wheel {
        self.left
    }
    pub fn right(&self) -> Wheel {
        self.right
    }
    pub fn telemetry(&self) -> &Telemetry {
        &self.telemetry
    }
    /// `telemetry_pending_`: a 0x85 has been decoded since the last
    /// [`BaseCycle::clear_telemetry_pending`]. `MowerSystem` publishes
    /// `/mower_base/telemetry` only when this and
    /// [`BaseCycle::feedback_valid`] are both true, so one message goes out
    /// per new frame rather than per control cycle.
    pub fn telemetry_pending(&self) -> bool {
        self.telemetry_pending
    }
    /// Call after publishing `/mower_base/telemetry`.
    pub fn clear_telemetry_pending(&mut self) {
        self.telemetry_pending = false;
    }
    /// At least one 0x85 has been decoded since activation (`feedback_valid_`).
    pub fn feedback_valid(&self) -> bool {
        self.feedback_valid
    }
    /// `blade_active_`: the dead-man is still holding the blade. Reported as
    /// `blade.held` in the telemetry JSON.
    pub fn blade_active(&self) -> bool {
        self.blade.active
    }
    pub fn firmware_info(&self) -> Option<proto::FirmwareInfo> {
        self.firmware_info
    }
    /// The running firmware answered with a protocol version this driver does
    /// not speak. Latched; the driver logs an error and keeps going.
    pub fn firmware_protocol_mismatch(&self) -> bool {
        self.firmware_protocol_mismatch
    }
    pub fn crc_errors(&self) -> usize {
        self.parser.crc_errors()
    }
    pub fn feedback_age_s(&self) -> f64 {
        self.feedback_age_s
    }
    pub fn faulted(&self) -> bool {
        self.faulted
    }
    /// The arm latch: `Some(reason)` while either motion stream is held.
    ///
    /// Both streams are disarmed on every activation and when feedback that
    /// had been arriving is lost for longer than `feedback_timeout_s`. While
    /// cmd_vel is held the controller reference is forced to zero (braking
    /// through the limiter, exactly like a cmd_vel timeout); while
    /// wheel_override is held its requests are not applied, and an override
    /// that was running is dropped at the disarm. Each stream is re-armed
    /// by its own stop edge *after* the disarm: an explicit stop (cmd_vel
    /// 0/0; override 0/0 or ttl 0) as the newest message, or no message for
    /// longer than `cmd_vel_timeout`. With `cmd_vel_timeout` 0 only the
    /// explicit stops count.
    pub fn disarmed(&self) -> Option<DisarmReason> {
        self.cmd_hold.or(self.override_hold).map(|h| h.reason)
    }
    /// Whether `stream` is held by the arm latch right now.
    pub fn holds(&self, stream: Stream) -> bool {
        match stream {
            Stream::CmdVel => self.cmd_hold.is_some(),
            Stream::WheelOverride => self.override_hold.is_some(),
        }
    }
    /// What happened since the last call, for the driver to log.
    pub fn take_events(&mut self) -> Vec<Event> {
        std::mem::take(&mut self.events)
    }
    pub fn diff_drive(&self) -> &DiffDrive {
        &self.ddc
    }

    /// The STM32 asked the host to halt (3 s button press). Returns the reason
    /// once; the 0x05 ack is already queued for the next write.
    pub fn take_shutdown_request(&mut self) -> Option<u8> {
        self.shutdown_request.take()
    }

    /// Serial read or write failed: latch fail-closed. Every following tick
    /// emits the stop burst and commands nothing until [`BaseCycle::clear_fault`].
    pub fn fault(&mut self) {
        self.faulted = true;
        self.ddc.halt();
        self.left.cmd_velocity = 0.0;
        self.right.cmd_velocity = 0.0;
    }

    /// The port was reopened at `now`: a re-activation. Wheel positions are
    /// kept (the encoder counter on the STM32 is free-running), but the count
    /// baseline is dropped so a reconnect does not integrate the gap, and the
    /// latch holds the wheels until the command streams show a stop edge.
    pub fn clear_fault(&mut self, now: TimeNs) {
        self.faulted = false;
        self.left.have_counts = false;
        self.right.have_counts = false;
        self.feedback_valid = false;
        self.feedback_stale = false;
        self.no_feedback_reported = false;
        self.activated_ns = now;
        self.ddc.activate(now);
        self.disarm(DisarmReason::Activation, now);
    }

    fn disarm(&mut self, reason: DisarmReason, now: TimeNs) {
        if !self.cfg.arm_latch {
            return;
        }
        let hold = Hold { reason, since_ns: now, stopped: false };
        self.cmd_hold = Some(hold);
        self.override_hold = Some(hold);
        // An override running from before must not come back on re-arm.
        self.wheel_override.until_ns = None;
        self.events.push(Event::Disarmed(reason));
    }

    /// Re-arm each stream that has shown its stop edge.
    fn try_rearm(&mut self, now: TimeNs) {
        let window = self.ddc.cmd_vel_timeout_ns();
        if let Some(hold) = self.cmd_hold {
            // Silence also needs the controller to have timed the stored
            // command out, so re-arming can never hand the limiter a stale
            // non-zero reference: a command stamped in the future is not
            // aged until its stamp passes, and only an explicit zero clears it.
            let edge = hold
                .edge(self.last_cmd_ns, now, window)
                .filter(|e| *e == StopEdge::Stop || self.ddc.command_timed_out());
            if let Some(by) = edge {
                self.cmd_hold = None;
                self.events.push(Event::Armed {
                    stream: Stream::CmdVel,
                    reason: hold.reason,
                    by,
                    after_s: seconds(now - hold.since_ns),
                });
            }
        }
        if let Some(hold) = self.override_hold {
            if let Some(by) = hold.edge(self.last_override_ns, now, window) {
                self.override_hold = None;
                self.events.push(Event::Armed {
                    stream: Stream::WheelOverride,
                    reason: hold.reason,
                    by,
                    after_s: seconds(now - hold.since_ns),
                });
            }
        }
    }

    fn next_seq(&mut self) -> u8 {
        let s = self.tx_seq;
        self.tx_seq = self.tx_seq.wrapping_add(1);
        s
    }

    /// `send_stop()`: wheels to 0 and blade to 0, and drop the blade dead-man
    /// so a held blade cannot coast through the STM32 timeout.
    pub fn stop_burst(&mut self) -> TxFrame {
        self.blade = DeadMan::default();
        let timeout = self.cfg.command_timeout_ms;
        let mut tx = TxFrame::default();
        let s0 = self.next_seq();
        tx.push(proto::build_wheel_speed_command(s0, 0, 0, timeout));
        let s1 = self.next_seq();
        tx.push(proto::build_lawer_motor_command(s1, 0, timeout));
        tx
    }

    /// Bytes that `on_rx` already sequenced and that must go out before the
    /// next tick's frames. `tick` prepends them automatically; take them here
    /// only if the driver writes them the moment they appear.
    pub fn take_pending_tx(&mut self) -> Option<TxFrame> {
        if self.pending_tx.is_empty() {
            None
        } else {
            Some(std::mem::take(&mut self.pending_tx))
        }
    }

    // ---- side channels ---------------------------------------------------

    /// `/mower_base/wheel_override`: drive raw permille for `ttl_ms`,
    /// bypassing the controller's acceleration limits (PID auto-tune steps).
    ///
    /// While the arm latch holds, the request is not applied — it only
    /// tells the latch whether the override stream is at rest.
    pub fn request_wheel_override(
        &mut self,
        left_permille: i32,
        right_permille: i32,
        ttl_ms: i64,
        now: TimeNs,
    ) {
        let ttl = ttl_ms.clamp(0, self.cfg.override_max_ttl_ms);
        let l = left_permille.clamp(-1000, 1000) as i16;
        let r = right_permille.clamp(-1000, 1000) as i16;
        self.last_override_ns = Some(now);
        if let Some(hold) = self.override_hold.as_mut() {
            hold.stopped = ttl == 0 || (l == 0 && r == 0);
            return;
        }
        self.wheel_override.until_ns = Some(now + ttl * 1_000_000);
        self.wheel_override.value = l;
        self.override_right = r;
    }

    /// `/mower_base/blade_command`: dead-man held blade. `permille <= 0` or an
    /// expired ttl sends exactly one explicit 0.
    pub fn request_blade(&mut self, permille: i32, ttl_ms: i64, now: TimeNs) {
        let ttl = ttl_ms.clamp(0, self.cfg.blade_max_ttl_ms);
        let p = permille.clamp(0, 1000) as i16;
        self.blade.value = p;
        self.blade.until_ns = if p > 0 {
            Some(now + ttl * 1_000_000)
        } else {
            None
        };
    }

    /// `/mower_base/servo_command`. `pulse_us` 0 releases; anything else is
    /// clamped into the 500-2500 us range.
    pub fn request_servo(&mut self, pulse_us: i64, hold_ms: i64) {
        if pulse_us < 0 {
            return;
        }
        let pulse = if pulse_us == 0 {
            0
        } else {
            pulse_us.clamp(proto::SERVO_MIN_PULSE_US as i64, proto::SERVO_MAX_PULSE_US as i64)
        };
        self.servo_request = Some((pulse as u16, hold_ms.clamp(0, 65535) as u16));
    }

    /// `/mower_base/led_command`.
    pub fn request_led(&mut self, req: LedRequest) {
        self.led_request = Some(req);
    }

    /// `/mower_base/pid_command`.
    pub fn request_pid(&mut self, cfg: PidConfig) {
        self.pid_request = Some(cfg);
    }

    // ---- rx --------------------------------------------------------------

    /// Feed one serial read's worth of bytes. Partial frames, garbage between
    /// frames and split SOFs are all handled by [`FrameParser`]; a 0x86 with
    /// SHUTDOWN_REQUESTED queues the 0x05 ack (see [`BaseCycle::take_pending_tx`]).
    pub fn on_rx(&mut self, bytes: &[u8], now: TimeNs) {
        let mut frames: Vec<(u8, u8, Vec<u8>)> = Vec::new();
        self.parser.feed(bytes, |t, s, p| frames.push((t, s, p.to_vec())));
        for (t, s, p) in frames {
            self.handle_frame(t, s, &p, now);
        }
        self.telemetry.crc_errors = self.parser.crc_errors();
    }

    fn handle_frame(&mut self, frame_type: u8, seq: u8, payload: &[u8], now: TimeNs) {
        match frame_type {
            proto::WHEEL_FEEDBACK_STATUS => {
                let Some(fb) = proto::decode_wheel_feedback(seq, payload) else {
                    return;
                };
                let rad_per_count = TWO_PI / self.cfg.counts_per_rev;
                integrate(&mut self.left, fb.left_total_counts, fb.left_measured_rpm, rad_per_count);
                integrate(
                    &mut self.right,
                    fb.right_total_counts,
                    fb.right_measured_rpm,
                    rad_per_count,
                );
                self.telemetry.wheel = Some(fb);
                self.last_feedback_ns = now;
                self.feedback_valid = true;
                if self.feedback_stale {
                    self.events.push(Event::FeedbackResumed);
                }
                self.feedback_stale = false;
                self.telemetry_pending = true;
            }
            proto::MOTOR_STATUS => {
                if let Some(ms) = proto::decode_motor_status(payload) {
                    self.telemetry.motor = Some(ms);
                    if ms.flags & proto::STATUS_FLAG_DRIVER_ALARM != 0
                        && self
                            .driver_alarm_logged_ns
                            .map_or(true, |t| now - t >= DRIVER_ALARM_LOG_PERIOD_NS)
                    {
                        self.driver_alarm_logged_ns = Some(now);
                        self.events.push(Event::DriverAlarm);
                    }
                }
            }
            proto::POWER_STATUS => {
                if let Some(ps) = proto::decode_power_status(payload) {
                    self.telemetry.power = Some(ps);
                    self.on_power_status(ps);
                }
            }
            proto::PID_CONFIG_STATUS => {
                if let Some(st) = proto::decode_pid_config_status(payload) {
                    self.telemetry.pid = Some(st);
                }
            }
            proto::WS2812_STATUS => {
                if let Some(st) = proto::decode_ws2812_status(payload) {
                    self.telemetry.led = Some(st);
                }
            }
            proto::FIRMWARE_INFO => {
                if let Some(fi) = proto::decode_firmware_info(payload) {
                    self.on_firmware_info(fi);
                }
            }
            proto::CHARGER_STATUS => {
                if let Some(st) = proto::decode_charger_status(payload) {
                    self.telemetry.charger = Some(st);
                }
            }
            proto::SERVO_STATUS => {
                if let Some(st) = proto::decode_servo_status(payload) {
                    self.telemetry.servo = Some(st);
                }
            }
            proto::LAWER_MOTOR_STATUS => {
                if let Some(st) = proto::decode_lawer_motor_status(payload) {
                    self.telemetry.blade = Some(st);
                }
            }
            _ => {}
        }
    }

    fn on_firmware_info(&mut self, fi: proto::FirmwareInfo) {
        if self.firmware_info == Some(fi) {
            return;
        }
        self.firmware_info = Some(fi);
        self.telemetry.firmware = Some(fi);
        if fi.protocol_version != proto::PROTOCOL_VERSION {
            self.firmware_protocol_mismatch = true;
        }
    }

    fn on_power_status(&mut self, ps: proto::PowerStatus) {
        if !ps.shutdown_requested() {
            // request cleared (cancelled or rail cut)
            self.shutdown_acked = false;
            return;
        }
        if self.shutdown_acked {
            return;
        }
        self.shutdown_acked = true;
        self.shutdown_request = Some(ps.shutdown_reason);
        let seq = self.next_seq();
        self.pending_tx
            .push(proto::build_power_command(seq, proto::POWER_ACTION_HOST_SHUTDOWN_ACK));
    }

    // ---- one control cycle ----------------------------------------------

    /// One `read() -> update() -> write()` cycle at `now` (control clock).
    ///
    /// `cmd_vel` is the newest command the subscription accepted since the
    /// last tick ([`crate::diff_drive::receive_command`]); `None` means none
    /// did, and the stored command ages towards `cmd_vel_timeout`. `stamp`
    /// (ROS time) goes into the /odom and joint-state headers.
    ///
    /// Returns the bytes to write, the `/odom` sample when the publish rate
    /// lets one through, and the joint states (always, matching
    /// `joint_state_broadcaster`, which publishes every update).
    pub fn tick(
        &mut self,
        cmd_vel: Option<Command>,
        now: TimeNs,
        stamp: TimeNs,
    ) -> (Option<TxFrame>, Option<OdomSample>, Option<JointStates>) {
        // tail of read(): stale feedback zeroes the reported velocities
        if self.feedback_valid {
            self.feedback_age_s = seconds(now) - seconds(self.last_feedback_ns);
            if self.feedback_age_s > self.cfg.feedback_timeout_s {
                self.left.vel = 0.0;
                self.right.vel = 0.0;
                if !self.feedback_stale {
                    self.feedback_stale = true;
                    self.events.push(Event::FeedbackLost { age_s: self.feedback_age_s });
                    self.disarm(DisarmReason::FeedbackLost, now);
                }
            }
        } else if !self.no_feedback_reported
            && seconds(now - self.activated_ns) > self.cfg.feedback_timeout_s
        {
            self.no_feedback_reported = true;
            self.events.push(Event::NoFeedbackSinceActivation {
                age_s: seconds(now - self.activated_ns),
            });
        }
        self.telemetry.feedback_age_s = self.feedback_age_s;

        let period_s = match self.last_tick_ns {
            Some(prev) => seconds(now) - seconds(prev),
            None => 1.0 / self.cfg.update_rate_hz,
        };
        self.last_tick_ns = Some(now);

        let joints = Some(JointStates {
            stamp_ns: stamp,
            names: ["left_wheel_joint", "right_wheel_joint"],
            positions: [self.left.pos, self.right.pos],
            velocities: [self.left.vel, self.right.vel],
        });

        if self.faulted {
            // fail closed: no controller update, no odometry, just stop.
            let mut tx = std::mem::take(&mut self.pending_tx);
            let stop = self.stop_burst();
            tx.bytes.extend_from_slice(&stop.bytes);
            tx.frames.extend_from_slice(&stop.frames);
            return (Some(tx), None, joints);
        }

        // controller update, with the arm latch between the reference and
        // the limiter
        if let Some(cmd) = cmd_vel {
            self.last_cmd_ns = Some(now);
            if let Some(hold) = self.cmd_hold.as_mut() {
                let t = cmd.twist;
                hold.stopped = t.linear_x == 0.0 && t.angular_z == 0.0;
            }
        }
        let was_timed_out = self.ddc.command_timed_out();
        self.ddc.update_reference(now, cmd_vel);
        if self.ddc.command_timed_out() && !was_timed_out {
            let last = self.ddc.last_command();
            let finite = last.linear_x.is_finite() && last.angular_z.is_finite();
            if finite && (last.linear_x != 0.0 || last.angular_z != 0.0) {
                self.events.push(Event::CmdVelTimedOut {
                    linear_x: last.linear_x,
                    angular_z: last.angular_z,
                });
            }
        }
        self.try_rearm(now);
        if self.cmd_hold.is_some() {
            self.ddc.hold_zero();
        }
        let out = self.ddc.update_and_write(now, stamp, period_s, self.left.pos, self.right.pos);
        if let Some(WheelCommand { left, right }) = out.wheel {
            self.left.cmd_velocity = left;
            self.right.cmd_velocity = right;
        }

        // write()
        let mut tx = std::mem::take(&mut self.pending_tx);
        let timeout = self.cfg.command_timeout_ms;
        let (l, r) = match self.override_permille(now) {
            Some(pair) => pair,
            None => (
                self.rad_s_to_permille(self.left.cmd_velocity),
                self.rad_s_to_permille(self.right.cmd_velocity),
            ),
        };
        let seq = self.next_seq();
        tx.push(proto::build_wheel_speed_command(seq, l, r, timeout));

        if let Some(frame) = self.blade_frame(now) {
            tx.push(frame);
        }
        if let Some((pulse, hold)) = self.servo_request.take() {
            let seq = self.next_seq();
            tx.push(proto::build_servo_command(seq, pulse, hold));
        }
        if let Some(frame) = self.led_frame(now) {
            tx.push(frame);
        }
        if let Some(cfg) = self.pid_request.take() {
            let seq = self.next_seq();
            tx.push(proto::build_pid_config_command(seq, &cfg));
        }

        (Some(tx), out.odom, joints)
    }

    /// `rad_s_to_permille`.
    pub fn rad_s_to_permille(&self, rad_s: f64) -> i16 {
        let rpm = rad_s * 60.0 / TWO_PI;
        let permille = rpm / self.cfg.max_rpm * 1000.0;
        permille.clamp(-1000.0, 1000.0).round() as i16
    }

    /// `override_permille`: raw permille while the ttl has not expired.
    fn override_permille(&mut self, now: TimeNs) -> Option<(i16, i16)> {
        let active = matches!(self.wheel_override.until_ns, Some(until) if now < until);
        if active != self.wheel_override.active {
            self.events.push(Event::Override { active });
        }
        self.wheel_override.active = active;
        active.then_some((self.wheel_override.value, self.override_right))
    }

    /// `send_blade`: refresh every cycle while held, one explicit 0 on expiry,
    /// nothing at all when idle (the STM32's own timeout keeps it stopped).
    fn blade_frame(&mut self, now: TimeNs) -> Option<Vec<u8>> {
        let active = matches!(self.blade.until_ns, Some(until) if now < until);
        let was_active = self.blade.active;
        self.blade.active = active;
        if active != was_active {
            self.events.push(Event::Blade { running: active, permille: self.blade.value });
        }
        if !active && !was_active {
            return None;
        }
        let permille = if active { self.blade.value } else { 0 };
        let seq = self.next_seq();
        Some(proto::build_lawer_motor_command(seq, permille, self.cfg.command_timeout_ms))
    }

    /// `send_led_if_needed`: on change, and again every `led_resend_period_s`.
    fn led_frame(&mut self, now: TimeNs) -> Option<Vec<u8>> {
        let req = self.led_request?;
        let changed = Some(req) != self.led_sent;
        let stale = match self.led_sent_ns {
            None => true,
            Some(t) => seconds(now) - seconds(t) >= self.cfg.led_resend_period_s,
        };
        if !changed && !stale {
            return None;
        }
        let seq = self.next_seq();
        self.led_sent = Some(req);
        self.led_sent_ns = Some(now);
        Some(proto::build_ws2812_command(
            seq,
            req.mode,
            req.r,
            req.g,
            req.b,
            req.period_ms,
        ))
    }
}

/// 0x85 counts/rpm -> joint position/velocity.
fn integrate(w: &mut Wheel, total: i32, rpm: f64, rad_per_count: f64) {
    if w.have_counts {
        // signed 32-bit difference handles wrap-around
        let d = (total as u32).wrapping_sub(w.last_total_counts as u32) as i32;
        w.pos += d as f64 * rad_per_count;
    }
    w.last_total_counts = total;
    w.have_counts = true;
    w.vel = rpm * TWO_PI / 60.0;
}
