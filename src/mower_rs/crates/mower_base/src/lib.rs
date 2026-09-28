//! mower_base: the whole `ros2_control` chain of the real robot as one node.
//!
//! Phase B of `docs/ROS_FREE_PLAN.md`. What this replaces, all of it in one
//! process and one thread pair:
//!
//! | replaced | by |
//! |---|---|
//! | `controller_manager/ros2_control_node` (the 25 Hz loop, the resource manager, pal_statistics) | the serial thread here |
//! | `mower_hardware::MowerSystem` (`read` / `write`, the side channels, the `mower_hardware_info` node) | [`mower_base_core::cycle::BaseCycle`] + this node |
//! | `diff_drive_controller` (`diff_controller`) | [`mower_base_core::diff_drive`] |
//! | `joint_state_broadcaster` | `Driver::publish_joint_states` |
//! | the two `controller_manager/spawner` processes | nothing: there is no lifecycle to drive |
//! | `topic_tools throttle` `odom_throttle` (`/odom` -> `/odom_slow`, 5 Hz) | [`SlowCopy`], fed every `/odom` this node publishes |
//!
//! `robot_state_publisher` is **not** replaced — it stays, and it is the
//! reason `/joint_states` has to keep coming out at the controller rate.
//!
//! Everything computed comes from the already-verified
//! [`mower_base_core`] crate (protocol, rate limiter, odometry, controller
//! cycle, telemetry JSON), which is checked against the original C++ with
//! generated vectors. This crate is the transport wrapper and nothing else.
//!
//! Threads
//! -------
//! * One `std::thread` runs the 25 Hz serial cycle: a non-blocking read
//!   (`VMIN = 0`, what [`mower_base_core::cycle::BaseCycle`] expects), one
//!   `tick`, one write, then the publishes. It never awaits and never takes
//!   a lock that a `.await` can hold across.
//! * The tokio side only spins the node so the subscriptions run. Each
//!   subscription writes its request into a mutex-guarded slot that the
//!   serial thread empties once per cycle — never a channel, because a
//!   channel that the serial thread does not drain fast enough would deliver
//!   a stale `cmd_vel` late instead of dropping it, which is precisely what
//!   `realtime_tools::RealtimeThreadSafeBox` avoids in the C++ controller.
//!
//! Fail-closed
//! -----------
//! Any serial read or write error latches `BaseCycle::fault`, writes the
//! stop burst (wheels 0, blade 0) if the port still takes bytes, and ends
//! the module with `Err`. As a binary that is exit 1 and launch's
//! `respawn_delay=2.0`; inside `mower_rsd` the supervisor restarts the
//! module alone after 2 s. Either way the firmware's own 300 ms command
//! timeout has already stopped the wheels.

pub mod requests;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use futures::StreamExt;
use mower_base_core::cycle::{BaseCycle, BaseConfig, JointStates, LedRequest};
use mower_base_core::diff_drive::{DiffDriveParams, LimitParams, OdomSample, Twist};
use mower_base_core::protocol::PidConfig;
use mower_base_core::TimeNs;
use mower_rs_common::throttle::SlowCopy;
use mower_rs_common::{params, ModuleCtx, ModuleResult};
use r2r::builtin_interfaces::msg::Time;
use r2r::geometry_msgs::msg::{TransformStamped, TwistStamped};
use r2r::nav_msgs::msg::Odometry;
use r2r::sensor_msgs::msg::JointState;
use r2r::std_msgs::msg::String as StringMsg;
use r2r::tf2_msgs::msg::TFMessage;
use r2r::QosProfile;

/// `rclcpp::Time` in the default (system) clock, as nanoseconds. The C++ loop
/// is driven by the same clock, and `BaseCycle` only ever takes differences,
/// so this is both the control clock and the message stamp.
fn now_ns() -> TimeNs {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as i64)
        .unwrap_or(0)
}

fn stamp(ns: TimeNs) -> Time {
    Time { sec: (ns / 1_000_000_000) as i32, nanosec: (ns % 1_000_000_000) as u32 }
}

/// Requests the subscriptions leave for the serial thread. Every field is
/// "the latest message since the last cycle": a second message in the same
/// 40 ms window replaces the first, exactly as the C++ atomics do.
#[derive(Default)]
struct Slots {
    cmd_vel: Option<Twist>,
    led: Option<LedRequest>,
    pid: Option<PidConfig>,
    /// `(left_permille, right_permille, ttl_ms)`
    wheel_override: Option<(i32, i32, i64)>,
    /// `(pulse_us, hold_ms)`
    servo: Option<(i64, i64)>,
    /// `(permille, ttl_ms)`
    blade: Option<(i32, i64)>,
}

type Shared = Arc<Mutex<Slots>>;

/// Everything the driver needs that is not a `BaseConfig`.
struct Settings {
    device: String,
    baud: u32,
    update_rate_hz: f64,
    telemetry_rate_hz: f64,
    shutdown_command: String,
}

/// Publishers the serial thread owns. r2r publishers are `Send`, and
/// publishing is one `rcl_publish` with no executor involved, so the cycle
/// publishes straight from its own thread like `mower_imu` does.
struct Publishers {
    odom: r2r::Publisher<Odometry>,
    odom_slow: Option<SlowCopy<Odometry>>,
    joint_states: r2r::Publisher<JointState>,
    tf: r2r::Publisher<TFMessage>,
    telemetry: Option<r2r::Publisher<StringMsg>>,
    firmware_info: Option<r2r::Publisher<StringMsg>>,
}

struct Driver {
    cfg: BaseConfig,
    settings: Settings,
    pubs: Publishers,
    slots: Shared,
    logger: String,
    stop: Arc<AtomicBool>,
    odom_template: Odometry,
    joint_template: JointState,
    tf_template: TransformStamped,
}

impl Driver {
    /// The blocking 25 Hz cycle. Returns the error that ended it, if any.
    fn run(&mut self) -> Result<(), String> {
        let mut port = serialport::new(&self.settings.device, self.settings.baud)
            .timeout(Duration::from_millis(0))
            .open()
            .map_err(|e| format!("serial open failed ({}): {e}", self.settings.device))?;
        r2r::log_info!(&self.logger, "opened {}", self.settings.device);

        let (mut base, activation) = BaseCycle::new(self.cfg.clone(), now_ns())?;
        port.write_all(&activation.bytes)
            .map_err(|e| format!("serial write error (activation): {e}"))?;

        let period = Duration::from_secs_f64(1.0 / self.settings.update_rate_hz);
        // One message per new 0x85, throttled only if telemetry_rate_hz is
        // set below the frame rate; 0.8 keeps frame jitter from dropping
        // every other message (MowerSystem::publish_telemetry_if_due).
        let telemetry_period_s = if self.settings.telemetry_rate_hz > 0.0 {
            0.8 / self.settings.telemetry_rate_hz
        } else {
            f64::INFINITY
        };
        let mut telemetry_sent_ns: Option<TimeNs> = None;
        let mut firmware_sent = None;
        let mut next = Instant::now();
        let mut buf = [0u8; 512];
        let mut overruns = 0u64;

        while !self.stop.load(Ordering::Relaxed) {
            // ---- read(): drain the port, VMIN = 0 ------------------------
            let time = now_ns();
            loop {
                let available = match port.bytes_to_read() {
                    Ok(n) => n as usize,
                    Err(e) => return self.fail(&mut base, &mut *port, format!("serial read error: {e}")),
                };
                if available == 0 {
                    break;
                }
                let want = available.min(buf.len());
                let n = match std::io::Read::read(&mut *port, &mut buf[..want]) {
                    Ok(n) => n,
                    Err(e) if e.kind() == std::io::ErrorKind::TimedOut => 0,
                    Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => 0,
                    Err(e) => return self.fail(&mut base, &mut *port, format!("serial read error: {e}")),
                };
                if n == 0 {
                    break;
                }
                base.on_rx(&buf[..n], time);
            }
            // The 0x86 SHUTDOWN_REQUESTED ack is already queued inside the
            // cycle; the host-side command is ours to run.
            if let Some(reason) = base.take_shutdown_request() {
                self.on_shutdown_request(reason);
            }
            self.publish_firmware_info(&base, &mut firmware_sent);
            self.publish_telemetry(&base, time, telemetry_period_s, &mut telemetry_sent_ns)
                .then(|| base.clear_telemetry_pending());

            // ---- the side channels, stamped with this cycle's time -------
            self.apply_requests(&mut base, time);

            // ---- update() + write() -------------------------------------
            let cmd = self.slots.lock().expect("slots").cmd_vel.take();
            let (tx, odom, joints) = base.tick(cmd, time);
            if let Some(tx) = tx {
                if !tx.is_empty() {
                    if let Err(e) = port.write_all(&tx.bytes) {
                        return self.fail(&mut base, &mut *port, format!("serial write error: {e}"));
                    }
                }
            }
            if let Some(js) = joints {
                self.publish_joint_states(&js);
            }
            if let Some(odom) = odom {
                self.publish_odom(&odom);
            }

            // ---- sleep to the next 25 Hz slot ---------------------------
            next += period;
            let now = Instant::now();
            if now >= next {
                overruns += 1;
                if overruns % 100 == 1 {
                    r2r::log_warn!(
                        &self.logger,
                        "control cycle overrun ({} so far, {:.1} ms late)",
                        overruns,
                        (now - next).as_secs_f64() * 1e3
                    );
                }
                next = now;
            } else {
                std::thread::sleep(next - now);
            }
        }
        // Clean stop: on_deactivate's send_stop().
        let stop = base.stop_burst();
        let _ = port.write_all(&stop.bytes);
        Ok(())
    }

    /// Fail closed: latch, try the stop burst, and hand the error up so the
    /// module ends and gets restarted.
    fn fail(
        &self,
        base: &mut BaseCycle,
        port: &mut dyn serialport::SerialPort,
        message: String,
    ) -> Result<(), String> {
        base.fault();
        let stop = base.stop_burst();
        // Best effort: the port is usually already gone, and the firmware's
        // own 300 ms command timeout is the line that actually stops the
        // wheels in that case.
        let _ = port.write_all(&stop.bytes);
        Err(message)
    }

    fn on_shutdown_request(&self, reason: u8) {
        r2r::log_warn!(
            &self.logger,
            "STM32 requests shutdown (reason {}): acking and running '{}'",
            reason,
            self.settings.shutdown_command
        );
        if self.settings.shutdown_command.is_empty() {
            return;
        }
        // Detached, so the cycle keeps acking status frames while the OS halts.
        match std::process::Command::new("sh")
            .arg("-c")
            .arg(format!("{} &", self.settings.shutdown_command))
            .status()
        {
            Ok(status) if status.success() => {}
            Ok(status) => r2r::log_error!(&self.logger, "shutdown command returned {status}"),
            Err(e) => r2r::log_error!(&self.logger, "shutdown command failed: {e}"),
        }
    }

    /// Move whatever the subscriptions left into the cycle. The ttl deadlines
    /// are computed here, on the control clock, which is where `write()`
    /// computes them in the C++.
    fn apply_requests(&self, base: &mut BaseCycle, now: TimeNs) {
        let taken = {
            let mut slots = self.slots.lock().expect("slots");
            Slots {
                cmd_vel: None,
                led: slots.led.take(),
                pid: slots.pid.take(),
                wheel_override: slots.wheel_override.take(),
                servo: slots.servo.take(),
                blade: slots.blade.take(),
            }
        };
        if let Some(led) = taken.led {
            base.request_led(led);
        }
        if let Some(pid) = taken.pid {
            base.request_pid(pid);
        }
        if let Some((l, r, ttl)) = taken.wheel_override {
            base.request_wheel_override(l, r, ttl, now);
        }
        if let Some((pulse, hold)) = taken.servo {
            base.request_servo(pulse, hold);
        }
        if let Some((permille, ttl)) = taken.blade {
            base.request_blade(permille, ttl, now);
        }
    }

    fn publish_joint_states(&self, js: &JointStates) {
        let mut msg = self.joint_template.clone();
        msg.header.stamp = stamp(js.stamp_ns);
        msg.position = js.positions.to_vec();
        msg.velocity = js.velocities.to_vec();
        if let Err(e) = self.pubs.joint_states.publish(&msg) {
            r2r::log_error!(&self.logger, "publish joint_states failed: {:?}", e);
        }
    }

    fn publish_odom(&mut self, odom: &OdomSample) {
        let mut msg = self.odom_template.clone();
        msg.header.stamp = stamp(odom.stamp_ns);
        msg.pose.pose.position.x = odom.x;
        msg.pose.pose.position.y = odom.y;
        msg.pose.pose.orientation.z = odom.qz;
        msg.pose.pose.orientation.w = odom.qw;
        msg.twist.twist.linear.x = odom.linear_x;
        msg.twist.twist.angular.z = odom.angular_z;
        match self.pubs.odom.publish(&msg) {
            // The throttle only ever saw what reached /odom; the copy is this
            // same message, stamp and all.
            Ok(()) => {
                if let Some(slow) = self.pubs.odom_slow.as_mut() {
                    slow.offer(&msg);
                }
            }
            Err(e) => r2r::log_error!(&self.logger, "publish odom failed: {:?}", e),
        }
        if !odom.publish_tf {
            return;
        }
        let mut tf = self.tf_template.clone();
        tf.header.stamp = stamp(odom.stamp_ns);
        tf.transform.translation.x = odom.x;
        tf.transform.translation.y = odom.y;
        tf.transform.rotation.z = odom.qz;
        tf.transform.rotation.w = odom.qw;
        if let Err(e) = self.pubs.tf.publish(&TFMessage { transforms: vec![tf] }) {
            r2r::log_error!(&self.logger, "publish tf failed: {:?}", e);
        }
    }

    /// `publish_telemetry_if_due`. Returns whether a message went out.
    fn publish_telemetry(
        &self,
        base: &BaseCycle,
        now: TimeNs,
        period_s: f64,
        sent_ns: &mut Option<TimeNs>,
    ) -> bool {
        let Some(publisher) = self.pubs.telemetry.as_ref() else {
            return false;
        };
        if !base.feedback_valid() || !base.telemetry_pending() {
            return false;
        }
        if let Some(sent) = *sent_ns {
            if mower_base_core::seconds(now) - mower_base_core::seconds(sent) < period_s {
                return false;
            }
        }
        *sent_ns = Some(now);
        let msg = StringMsg {
            data: base
                .telemetry()
                .to_json(mower_base_core::seconds(now), base.blade_active()),
        };
        if let Err(e) = publisher.publish(&msg) {
            r2r::log_error!(&self.logger, "publish telemetry failed: {:?}", e);
        }
        true
    }

    /// The latched `/mower_base/firmware_info`, published once per distinct
    /// 0x87 (`on_firmware_info`).
    fn publish_firmware_info(
        &self,
        base: &BaseCycle,
        sent: &mut Option<mower_base_core::protocol::FirmwareInfo>,
    ) {
        let Some(info) = base.firmware_info() else { return };
        if *sent == Some(info) {
            return;
        }
        *sent = Some(info);
        r2r::log_info!(
            &self.logger,
            "STM32 firmware {} (protocol {}, built {})",
            info.version_string(),
            info.protocol_version,
            info.build_unix
        );
        if base.firmware_protocol_mismatch() {
            r2r::log_error!(
                &self.logger,
                "firmware speaks protocol {}, this driver expects {}",
                info.protocol_version,
                mower_base_core::protocol::PROTOCOL_VERSION
            );
        }
        if let Some(publisher) = self.pubs.firmware_info.as_ref() {
            let msg = StringMsg { data: info.to_json() };
            if let Err(e) = publisher.publish(&msg) {
                r2r::log_error!(&self.logger, "publish firmware_info failed: {:?}", e);
            }
        }
    }
}

/// One `linear.x` / `angular.z` block of the controller parameters. The
/// `has_*_limits` defaults are `diff_drive_controller`'s own (all true in
/// Jazzy); an unset bound stays NaN, which the limiter reads as "no limit",
/// so a `has_*` of true with NaN bounds and a `has_*` of false are the same
/// limiter.
fn limit_params(node: &r2r::Node, axis: &str, defaults: LimitParams) -> LimitParams {
    let get = |name: &str, default: f64| params::f64(node, &format!("{axis}.{name}"), default);
    LimitParams {
        has_velocity_limits: params::bool(node, &format!("{axis}.has_velocity_limits"), true),
        has_acceleration_limits: params::bool(node, &format!("{axis}.has_acceleration_limits"), true),
        has_jerk_limits: params::bool(node, &format!("{axis}.has_jerk_limits"), true),
        min_velocity: get("min_velocity", defaults.min_velocity),
        max_velocity: get("max_velocity", defaults.max_velocity),
        max_acceleration: get("max_acceleration", defaults.max_acceleration),
        max_acceleration_reverse: get("max_acceleration_reverse", defaults.max_acceleration_reverse),
        max_deceleration: get("max_deceleration", defaults.max_deceleration),
        max_deceleration_reverse: get("max_deceleration_reverse", defaults.max_deceleration_reverse),
        min_jerk: get("min_jerk", defaults.min_jerk),
        max_jerk: get("max_jerk", defaults.max_jerk),
    }
}

/// `mower_controller/controllers/diff_drive_controller.yaml`, the file the
/// robot actually launches (`diff_controller`), as the built-in defaults.
/// `mower_hardware/config/mower_controllers.yaml` is the bench/demo file and
/// is *not* what production runs; the differences that matter are
/// `open_loop`, `enable_odom_tf`, `base_frame_id`, the wheel geometry and
/// `cmd_vel_timeout`.
pub fn production_diff_drive() -> DiffDriveParams {
    DiffDriveParams {
        wheel_separation: 0.35,
        wheel_radius: 0.09,
        wheel_separation_multiplier: 1.0,
        left_wheel_radius_multiplier: 1.0,
        right_wheel_radius_multiplier: 1.0,
        publish_rate: 25.0,
        cmd_vel_timeout: 0.25,
        open_loop: true,
        position_feedback: true,
        // The two EKFs own odom->base_footprint; the controller must not.
        enable_odom_tf: false,
        velocity_rolling_window_size: 10,
        odom_frame_id: "odom".to_string(),
        base_frame_id: "base_footprint".to_string(),
        // Not set in the yaml: diff_drive_controller's own defaults (zeros).
        pose_covariance_diagonal: [0.0; 6],
        twist_covariance_diagonal: [0.0; 6],
        linear: LimitParams {
            has_velocity_limits: true,
            has_acceleration_limits: true,
            has_jerk_limits: true,
            min_velocity: -0.5,
            max_velocity: 0.5,
            max_acceleration: 1.0,
            max_acceleration_reverse: -1.0,
            // A safety zero must not spend another ~0.5 s in the limiter.
            max_deceleration: -25.0,
            max_deceleration_reverse: 25.0,
            min_jerk: f64::NAN,
            max_jerk: f64::NAN,
        },
        angular: LimitParams {
            has_velocity_limits: true,
            has_acceleration_limits: true,
            has_jerk_limits: true,
            min_velocity: -1.0,
            max_velocity: 1.0,
            max_acceleration: 2.0,
            max_acceleration_reverse: -2.0,
            max_deceleration: -50.0,
            max_deceleration_reverse: 50.0,
            min_jerk: f64::NAN,
            max_jerk: f64::NAN,
        },
    }
}

/// `/odom` as `odom_throttle` discovers it on the robot, which is what the
/// QoS of its `/odom_slow` is derived from (`throttle::output_qos`).
/// `diff_drive_controller` creates `/odom` with `rclcpp::SystemDefaultsQoS()`,
/// which the robot's rmw_cyclonedds_cpp announces as reliable + volatile, keep
/// last 1, so the throttle publishes reliable + volatile, keep last 10. Not
/// this node's latched `/odom`: that is FastDDS's reading of the same system
/// defaults, and a copy derived from it would hand a late transient-local
/// joiner (a bag recorder) up to 10 stale samples no throttle ever did.
fn throttled_odom_qos() -> QosProfile {
    QosProfile::default().keep_last(1).reliable().volatile()
}

fn covariance6(node: &r2r::Node, name: &str, default: [f64; 6]) -> [f64; 6] {
    let value = match node.params.lock().unwrap().get(name).map(|p| p.value.clone()) {
        Some(r2r::ParameterValue::DoubleArray(v)) => v,
        Some(r2r::ParameterValue::IntegerArray(v)) => v.into_iter().map(|x| x as f64).collect(),
        _ => return default,
    };
    let mut out = default;
    for (slot, v) in out.iter_mut().zip(value) {
        *slot = v;
    }
    out
}

pub async fn run(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    let mut node = r2r::Node::create(ctx, &m.node_name, &m.namespace)?;
    let logger = node.logger().to_string();

    let d = production_diff_drive();
    let diff_drive = DiffDriveParams {
        wheel_separation: params::f64(&node, "wheel_separation", d.wheel_separation),
        wheel_radius: params::f64(&node, "wheel_radius", d.wheel_radius),
        wheel_separation_multiplier: params::f64(
            &node,
            "wheel_separation_multiplier",
            d.wheel_separation_multiplier,
        ),
        left_wheel_radius_multiplier: params::f64(
            &node,
            "left_wheel_radius_multiplier",
            d.left_wheel_radius_multiplier,
        ),
        right_wheel_radius_multiplier: params::f64(
            &node,
            "right_wheel_radius_multiplier",
            d.right_wheel_radius_multiplier,
        ),
        publish_rate: params::f64(&node, "publish_rate", d.publish_rate),
        cmd_vel_timeout: params::f64(&node, "cmd_vel_timeout", d.cmd_vel_timeout),
        open_loop: params::bool(&node, "open_loop", d.open_loop),
        position_feedback: params::bool(&node, "position_feedback", d.position_feedback),
        enable_odom_tf: params::bool(&node, "enable_odom_tf", d.enable_odom_tf),
        velocity_rolling_window_size: params::i64(
            &node,
            "velocity_rolling_window_size",
            d.velocity_rolling_window_size as i64,
        )
        .max(1) as usize,
        odom_frame_id: params::string(&node, "odom_frame_id", &d.odom_frame_id),
        base_frame_id: params::string(&node, "base_frame_id", &d.base_frame_id),
        pose_covariance_diagonal: covariance6(
            &node,
            "pose_covariance_diagonal",
            d.pose_covariance_diagonal,
        ),
        twist_covariance_diagonal: covariance6(
            &node,
            "twist_covariance_diagonal",
            d.twist_covariance_diagonal,
        ),
        linear: limit_params(&node, "linear.x", d.linear),
        angular: limit_params(&node, "angular.z", d.angular),
    };

    let update_rate_hz = params::f64(&node, "update_rate", 25.0);
    if !(update_rate_hz.is_finite() && update_rate_hz > 0.0) {
        return Err("update_rate must be finite and > 0".into());
    }
    let cfg = BaseConfig {
        max_rpm: params::f64(&node, "max_rpm", 58.0),
        counts_per_rev: params::f64(&node, "counts_per_rev", 8896.0),
        command_timeout_ms: params::i64(&node, "command_timeout_ms", 300).clamp(0, 65535) as u16,
        feedback_timeout_s: params::f64(&node, "feedback_timeout_s", 0.5),
        update_rate_hz,
        led_resend_period_s: params::f64(&node, "led_resend_period_s", 5.0),
        override_max_ttl_ms: params::i64(&node, "override_max_ttl_ms", 1000),
        blade_max_ttl_ms: params::i64(&node, "blade_max_ttl_ms", 1000),
        diff_drive: diff_drive.clone(),
    };
    let settings = Settings {
        device: params::string(&node, "device", "/dev/stmcom"),
        baud: params::i64(&node, "baud", 115200).clamp(1, u32::MAX as i64) as u32,
        update_rate_hz,
        telemetry_rate_hz: params::f64(&node, "telemetry_rate_hz", 20.0),
        // Inside the container there is no systemd; the host watches
        // ~/.mower/host.request (deploy/host/mower-host-request.service).
        shutdown_command: params::string(
            &node,
            "shutdown_command",
            "/usr/local/bin/mower-host-request poweroff",
        ),
    };

    // ---- topics ----------------------------------------------------------
    let cmd_vel_topic = params::string(&node, "cmd_vel_topic", "/drivetrain_guarded_cmd_vel");
    let odom_topic = params::string(&node, "odom_topic", "/odom");
    let joint_states_topic = params::string(&node, "joint_states_topic", "/joint_states");
    let tf_topic = params::string(&node, "tf_topic", "/tf");
    let telemetry_topic = params::string(&node, "telemetry_topic", "/mower_base/telemetry");
    let firmware_info_topic =
        params::string(&node, "firmware_info_topic", "/mower_base/firmware_info");
    let led_topic = params::string(&node, "led_topic", "/mower_base/led_command");
    let pid_topic = params::string(&node, "pid_topic", "/mower_base/pid_command");
    let override_topic = params::string(&node, "override_topic", "/mower_base/wheel_override");
    let servo_topic = params::string(&node, "servo_topic", "/mower_base/servo_command");
    let blade_topic = params::string(&node, "blade_topic", "/mower_base/blade_command");
    // The slow copy of /odom the status nodes read (heartbeat_source_topic and
    // odom_topic of robot_status, heartbeat / robot_info / telemetry), which
    // mission.launch.py's odom_throttle makes when ros2_control publishes
    // /odom and does not start when this node does. Empty turns it off.
    let odom_slow_topic = params::string(&node, "odom_slow_topic", "/odom_slow");
    let odom_slow_rate_hz = params::f64(&node, "odom_slow_rate_hz", 5.0);

    // QoS, one for one with what the chain announces on the graph today
    // (read off `ros2 topic info -v` / the endpoint info in the differential
    // test, not guessed from the upstream sources):
    //   /odom, /joint_states, /tf        reliable + TRANSIENT_LOCAL, keep last 1
    //                                    -- diff_drive_controller and
    //                                    joint_state_broadcaster both latch,
    //                                    so a late joiner gets the last state
    //   cmd_vel (subscription)           reliable-compatible BEST_EFFORT,
    //                                    which is what diff_controller
    //                                    subscribes with; a reliable
    //                                    subscription would refuse a
    //                                    best-effort publisher
    //   /mower_base/telemetry            best effort, keep last 1
    //   /mower_base/firmware_info, led   transient local + reliable, depth 1
    //   pid                              reliable, depth 4
    //   wheel_override, servo, blade     best effort, depth 1
    //   /odom_slow                       what odom_throttle publishes on the
    //                                    robot: reliable + volatile, keep
    //                                    last 10 (`throttled_odom_qos`)
    let latched = QosProfile::default().keep_last(1).transient_local().reliable();
    let best_effort_1 = QosProfile::default().keep_last(1).best_effort();

    let odom_pub = node.create_publisher::<Odometry>(&odom_topic, latched.clone())?;
    let odom_slow = SlowCopy::<Odometry>::create(
        &mut node,
        &odom_slow_topic,
        odom_slow_rate_hz,
        &throttled_odom_qos(),
    );
    let joint_pub = node.create_publisher::<JointState>(&joint_states_topic, latched.clone())?;
    let tf_pub = node.create_publisher::<TFMessage>(&tf_topic, latched.clone())?;
    let telemetry_pub = if telemetry_topic.is_empty() || settings.telemetry_rate_hz <= 0.0 {
        None
    } else {
        Some(node.create_publisher::<StringMsg>(&telemetry_topic, best_effort_1.clone())?)
    };
    let firmware_pub = if firmware_info_topic.is_empty() {
        None
    } else {
        Some(node.create_publisher::<StringMsg>(&firmware_info_topic, latched.clone())?)
    };

    let slots: Shared = Arc::new(Mutex::new(Slots::default()));

    // ---- subscriptions ---------------------------------------------------
    let mut cmd_vel = node.subscribe::<TwistStamped>(&cmd_vel_topic, best_effort_1.clone())?;
    {
        let slots = slots.clone();
        tokio::spawn(async move {
            while let Some(msg) = cmd_vel.next().await {
                // Only the newest command survives to the next cycle: the
                // C++ controller reads one realtime box, not a queue.
                slots.lock().expect("slots").cmd_vel =
                    Some(Twist::new(msg.twist.linear.x, msg.twist.angular.z));
            }
        });
    }
    if !led_topic.is_empty() {
        let mut sub = node.subscribe::<StringMsg>(&led_topic, latched.clone())?;
        let slots = slots.clone();
        let logger = logger.clone();
        tokio::spawn(async move {
            let mut serial: u8 = 0;
            while let Some(msg) = sub.next().await {
                serial = serial.wrapping_add(1);
                match requests::led(&msg.data, serial) {
                    Some(req) => {
                        r2r::log_info!(
                            &logger,
                            "light request: mode {} rgb({},{},{}) period {} ms",
                            req.mode,
                            req.r,
                            req.g,
                            req.b,
                            req.period_ms
                        );
                        slots.lock().expect("slots").led = Some(req);
                    }
                    None => r2r::log_warn!(
                        &logger,
                        "ignoring light request without a valid mode: {}",
                        msg.data
                    ),
                }
            }
        });
    }
    if !pid_topic.is_empty() {
        let mut sub = node.subscribe::<StringMsg>(&pid_topic, QosProfile::default().keep_last(4))?;
        let slots = slots.clone();
        let logger = logger.clone();
        tokio::spawn(async move {
            while let Some(msg) = sub.next().await {
                match requests::pid(&msg.data) {
                    Some(cfg) => {
                        r2r::log_info!(
                            &logger,
                            "pid request: L {:.3}/{:.3}/{:.3} R {:.3}/{:.3}/{:.3} persist={} closed_loop={}",
                            cfg.left_kp, cfg.left_ki, cfg.left_kd,
                            cfg.right_kp, cfg.right_ki, cfg.right_kd,
                            cfg.persist_to_flash as u8, cfg.closed_loop_enabled as u8
                        );
                        slots.lock().expect("slots").pid = Some(cfg);
                    }
                    None => r2r::log_warn!(
                        &logger,
                        "ignoring pid request without left/right kp: {}",
                        msg.data
                    ),
                }
            }
        });
    }
    if !override_topic.is_empty() {
        let mut sub = node.subscribe::<StringMsg>(&override_topic, best_effort_1.clone())?;
        let slots = slots.clone();
        tokio::spawn(async move {
            while let Some(msg) = sub.next().await {
                if let Some(req) = requests::wheel_override(&msg.data) {
                    slots.lock().expect("slots").wheel_override = Some(req);
                }
            }
        });
    }
    if !servo_topic.is_empty() {
        let mut sub = node.subscribe::<StringMsg>(&servo_topic, best_effort_1.clone())?;
        let slots = slots.clone();
        let logger = logger.clone();
        tokio::spawn(async move {
            while let Some(msg) = sub.next().await {
                match requests::servo(&msg.data) {
                    Some(req) => slots.lock().expect("slots").servo = Some(req),
                    None => r2r::log_warn!(
                        &logger,
                        "ignoring servo request without pulse_us: {}",
                        msg.data
                    ),
                }
            }
        });
    }
    if !blade_topic.is_empty() {
        let mut sub = node.subscribe::<StringMsg>(&blade_topic, best_effort_1.clone())?;
        let slots = slots.clone();
        tokio::spawn(async move {
            while let Some(msg) = sub.next().await {
                if let Some(req) = requests::blade(&msg.data) {
                    slots.lock().expect("slots").blade = Some(req);
                }
            }
        });
    }

    // ---- message templates (everything constant, filled once) ------------
    let mut odom_template = Odometry::default();
    odom_template.header.frame_id = diff_drive.odom_frame_id.clone();
    odom_template.child_frame_id = diff_drive.base_frame_id.clone();
    odom_template.pose.pose.orientation.w = 1.0;
    odom_template.pose.covariance = vec![0.0; 36];
    odom_template.twist.covariance = vec![0.0; 36];
    for i in 0..6 {
        odom_template.pose.covariance[6 * i + i] = diff_drive.pose_covariance_diagonal[i];
        odom_template.twist.covariance[6 * i + i] = diff_drive.twist_covariance_diagonal[i];
    }
    let mut joint_template = JointState::default();
    joint_template.name = vec!["left_wheel_joint".to_string(), "right_wheel_joint".to_string()];
    // joint_state_broadcaster fills every array it exports and writes NaN
    // for an interface the hardware does not have; these joints export
    // position and velocity only, so effort is two NaNs, not empty.
    joint_template.effort = vec![f64::NAN, f64::NAN];
    let mut tf_template = TransformStamped::default();
    tf_template.header.frame_id = diff_drive.odom_frame_id.clone();
    tf_template.child_frame_id = diff_drive.base_frame_id.clone();
    tf_template.transform.rotation.w = 1.0;

    r2r::log_info!(
        &logger,
        "device={} baud={} max_rpm={:.1} counts_per_rev={:.0} update_rate={:.0} Hz; \
         {} -> wheels, odom -> {} ({} loop, odom_tf {}), slow copy {}",
        settings.device,
        settings.baud,
        cfg.max_rpm,
        cfg.counts_per_rev,
        update_rate_hz,
        cmd_vel_topic,
        odom_topic,
        if diff_drive.open_loop { "open" } else { "closed" },
        diff_drive.enable_odom_tf,
        if odom_slow.is_some() {
            format!("{odom_slow_topic} at {odom_slow_rate_hz} Hz")
        } else {
            "off".to_string()
        }
    );

    let stop = Arc::new(AtomicBool::new(false));
    let mut driver = Driver {
        cfg,
        settings,
        pubs: Publishers {
            odom: odom_pub,
            odom_slow,
            joint_states: joint_pub,
            tf: tf_pub,
            telemetry: telemetry_pub,
            firmware_info: firmware_pub,
        },
        slots,
        logger: logger.clone(),
        stop: stop.clone(),
        odom_template,
        joint_template,
        tf_template,
    };
    let serial_thread = std::thread::Builder::new()
        .name("mower-base-serial".into())
        .spawn(move || driver.run())?;

    // ---- spin the subscriptions until shutdown ---------------------------
    let running = Arc::new(AtomicBool::new(true));
    let spin = {
        let running = running.clone();
        tokio::task::spawn_blocking(move || {
            while running.load(Ordering::Relaxed) {
                node.spin_once(Duration::from_millis(100));
            }
            drop(node);
        })
    };

    let mut failure: Option<String> = None;
    loop {
        tokio::select! {
            _ = m.shutdown.wait() => break,
            _ = tokio::time::sleep(Duration::from_millis(100)) => {
                if serial_thread.is_finished() {
                    break;
                }
            }
        }
    }
    stop.store(true, Ordering::Relaxed);
    match serial_thread.join() {
        Ok(Err(message)) => {
            r2r::log_fatal!(&logger, "{}", message);
            failure = Some(message);
        }
        Ok(Ok(())) => {}
        Err(_) => {
            r2r::log_fatal!(&logger, "base serial thread panicked");
            failure = Some("base serial thread panicked".to_string());
        }
    }
    running.store(false, Ordering::Relaxed);
    let _ = spin.await;
    match failure {
        Some(message) => Err(message.into()),
        None => Ok(()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The production controller file, not the bench one in mower_hardware.
    #[test]
    fn the_defaults_are_the_launched_diff_controller_yaml() {
        let p = production_diff_drive();
        assert_eq!(p.wheel_separation, 0.35);
        assert_eq!(p.wheel_radius, 0.09);
        assert_eq!(p.publish_rate, 25.0);
        assert_eq!(p.cmd_vel_timeout, 0.25);
        assert_eq!(p.base_frame_id, "base_footprint");
        assert_eq!(p.odom_frame_id, "odom");
        assert!(p.open_loop, "diff_controller runs open loop");
        assert!(!p.enable_odom_tf, "the EKFs own odom->base_footprint");
        assert_eq!(p.pose_covariance_diagonal, [0.0; 6]);
        assert_eq!(p.linear.max_velocity, 0.5);
        assert_eq!(p.linear.max_acceleration, 1.0);
        assert_eq!(p.linear.max_deceleration, -25.0);
        assert_eq!(p.angular.max_velocity, 1.0);
        assert_eq!(p.angular.max_deceleration, -50.0);
        // The limiters must be constructible with those bounds.
        p.linear.limiter().expect("linear limiter");
        p.angular.limiter().expect("angular limiter");
    }

    /// A `has_*_limits` of true with NaN bounds and one of false have to be
    /// the same limiter, otherwise reading Jazzy's defaults would change
    /// behaviour for any bound the yaml leaves out.
    #[test]
    fn nan_bounds_are_the_same_as_no_limits() {
        let mut on = LimitParams::default();
        on.has_velocity_limits = true;
        on.has_acceleration_limits = true;
        on.has_jerk_limits = true;
        let off = LimitParams::default();
        let (mut a, mut b) = (0.9f64, 0.9f64);
        on.limiter().unwrap().limit(&mut a, 0.0, 0.0, 0.04);
        off.limiter().unwrap().limit(&mut b, 0.0, 0.0, 0.04);
        assert_eq!(a, b);
        assert_eq!(a, 0.9);
    }

    /// /odom_slow keeps what odom_throttle publishes it with on the robot
    /// (rmw_cyclonedds_cpp, topic_tools 1.3.4 over diff_drive_controller's
    /// SystemDefaultsQoS /odom): reliable + volatile, keep last 10 -- not a
    /// transient-local copy of this node's latched /odom.
    #[test]
    fn odom_slow_is_as_volatile_as_the_throttle_it_replaces() {
        use mower_rs_common::throttle::output_qos;
        use r2r::qos::{DurabilityPolicy, HistoryPolicy, ReliabilityPolicy};
        let slow = output_qos(&throttled_odom_qos());
        assert_eq!(slow.history, HistoryPolicy::KeepLast);
        assert_eq!(slow.depth, 10);
        assert_eq!(slow.reliability, ReliabilityPolicy::Reliable);
        assert_eq!(slow.durability, DurabilityPolicy::Volatile);
    }

    #[test]
    fn a_cycle_config_built_from_the_defaults_activates() {
        let cfg = BaseConfig { diff_drive: production_diff_drive(), ..BaseConfig::default() };
        let (_, tx) = BaseCycle::new(cfg, 0).expect("activate");
        // send_stop() (0x01 + 0x02) then the 0x06 info request.
        assert_eq!(
            tx.frames.iter().map(|(t, _)| *t).collect::<Vec<_>>(),
            vec![0x01, 0x02, 0x06]
        );
    }
}
