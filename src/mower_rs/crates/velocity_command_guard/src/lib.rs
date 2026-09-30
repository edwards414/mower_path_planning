//! velocity_command_guard: final robot-side validation and watchdog for
//! drivetrain commands (port of `mower_bringup/velocity_command_guard.py`;
//! the rules live in `core.rs`, with a test vector each).
//!
//! Topics are relative (`cmd_vel_in`, `cmd_vel_out`, `command_clock`) and
//! remapped by twist_mux.launch.py, which runs two instances: the manual
//! guard (`require_command_session:=true`, app joystick -> mux) and the
//! final guard (mux -> ros2_control). Safety limits are read once at start
//! and there is no parameter service, so they cannot change at runtime.
//!
//! Clocks: outgoing stamps and stamp checks use the wall clock (the node
//! clock with use_sim_time:=false, which the production launch enforces);
//! receipt timeouts use a steady clock.

pub mod core;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use futures::StreamExt;
use mower_rs_common::{params, ModuleCtx, ModuleResult};
use r2r::builtin_interfaces::msg::Time;
use r2r::geometry_msgs::msg::TwistStamped;
use r2r::std_msgs::msg::Header;
use r2r::QosProfile;

use crate::core::{Action, Command, Guard, Limits};

fn wall_ns() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as i64)
        .unwrap_or(0)
}

fn stamp_now() -> Time {
    let ns = wall_ns();
    Time { sec: (ns / 1_000_000_000) as i32, nanosec: (ns % 1_000_000_000) as u32 }
}

fn twist(linear_x: f64, angular_z: f64) -> TwistStamped {
    let mut msg = TwistStamped::default();
    // Every upstream stamp is replaced with robot time. This preserves
    // ordering across the manual guard -> mux -> final guard boundary
    // without ever trusting or forwarding a remote clock value. A forced
    // zero is also an ordering barrier.
    msg.header.stamp = stamp_now();
    msg.header.frame_id = "base_footprint".to_string();
    msg.twist.linear.x = linear_x;
    msg.twist.angular.z = angular_z;
    msg
}

/// `manual-session-v1:<32 hex>` from the kernel RNG.
fn new_session_id() -> std::io::Result<String> {
    use std::io::Read;
    let mut bytes = [0u8; 16];
    std::fs::File::open("/dev/urandom")?.read_exact(&mut bytes)?;
    Ok(format!("manual-session-v1:{}", bytes.iter().map(|b| format!("{b:02x}")).collect::<String>()))
}

/// One guard instance. `m.node_name` is `velocity_command_guard` (mux ->
/// ros2_control) or `manual_velocity_guard` (app joystick -> mux); the three
/// topic names stay relative so the launch remappings — `-r <node>:cmd_vel_in:=…`
/// whether they come from `launch_ros`'s `remappings=` or from mower_rsd's
/// argv — resolve per node exactly as before.
pub async fn run(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    let mut node = r2r::Node::create(ctx, &m.node_name, &m.namespace)?;
    let logger = node.logger().to_string();

    let limits = Limits::new(
        params::f64(&node, "command_timeout_s", 0.20),
        params::f64(&node, "max_input_age_s", 0.25),
        params::f64(&node, "max_future_skew_s", 0.05),
        params::f64(&node, "max_linear_x_m_s", 0.50),
        params::f64(&node, "max_angular_z_rad_s", 1.00),
        params::bool(&node, "require_command_session", false),
        params::f64(&node, "command_clock_period_s", 0.05),
    )?;
    let session_id = if limits.require_command_session { Some(new_session_id()?) } else { None };

    let publisher = node.create_publisher::<TwistStamped>("cmd_vel_out", QosProfile::default().keep_last(1))?;
    let mut commands = node.subscribe::<TwistStamped>("cmd_vel_in", QosProfile::default().keep_last(1))?;
    let clock_publisher = if session_id.is_some() {
        Some(node.create_publisher::<Header>("command_clock", QosProfile::default().keep_last(1))?)
    } else {
        None
    };
    let mut clock_timer = if session_id.is_some() {
        Some(node.create_wall_timer(Duration::from_secs_f64(limits.command_clock_period_s))?)
    } else {
        None
    };
    let mut watchdog_timer = node.create_wall_timer(Duration::from_secs_f64(limits.watchdog_period_s()))?;

    r2r::log_info!(
        &logger,
        "velocity_command_guard: timeout={}s max_age={}s max_future_skew={}s max_linear_x={}m/s max_angular_z={}rad/s session={}",
        limits.command_timeout_s,
        limits.max_input_age_s,
        limits.max_future_skew_s,
        limits.max_linear_x,
        limits.max_angular_z,
        limits.require_command_session,
    );

    let guard = Arc::new(Mutex::new(Guard::new(limits)));
    // stop before anything can move, as the Python node did in __init__
    publisher.publish(&twist(0.0, 0.0))?;

    if let (Some(clock_publisher), Some(session_id)) = (clock_publisher.clone(), session_id.clone()) {
        let publish_clock = move || {
            let msg = Header { stamp: stamp_now(), frame_id: session_id.clone() };
            let _ = clock_publisher.publish(&msg);
        };
        publish_clock();
        let mut timer = clock_timer.take().expect("clock timer");
        tokio::spawn(async move {
            while timer.tick().await.is_ok() {
                publish_clock();
            }
        });
    }

    {
        let guard = guard.clone();
        let publisher = publisher.clone();
        let logger = logger.clone();
        tokio::spawn(async move {
            let mut last_rejection_log: Option<Instant> = None;
            while let Some(msg) = commands.next().await {
                let command = Command {
                    stamp_sec: msg.header.stamp.sec,
                    stamp_nanosec: msg.header.stamp.nanosec,
                    session_matches: session_id.as_deref() == Some(msg.header.frame_id.as_str()),
                    linear: [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z],
                    angular: [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z],
                };
                let now = Instant::now();
                let action = guard.lock().unwrap().on_command(&command, wall_ns(), now);
                match action {
                    Action::Forward { linear_x, angular_z } => {
                        if let Err(e) = publisher.publish(&twist(linear_x, angular_z)) {
                            r2r::log_error!(&logger, "publish cmd_vel_out failed: {:?}", e);
                        }
                    }
                    Action::Reject(reason) => {
                        if let Err(e) = publisher.publish(&twist(0.0, 0.0)) {
                            r2r::log_error!(&logger, "publish cmd_vel_out failed: {:?}", e);
                        }
                        if last_rejection_log.map(|t| now.duration_since(t).as_secs_f64() >= 1.0).unwrap_or(true) {
                            last_rejection_log = Some(now);
                            r2r::log_error!(&logger, "Rejected drivetrain command: {}", reason);
                        }
                    }
                }
            }
        });
    }

    {
        let guard = guard.clone();
        let publisher = publisher.clone();
        let logger = logger.clone();
        tokio::spawn(async move {
            while watchdog_timer.tick().await.is_ok() {
                if guard.lock().unwrap().watchdog(Instant::now()) {
                    let _ = publisher.publish(&twist(0.0, 0.0));
                    r2r::log_error!(&logger, "Drivetrain command receipt timeout; forced velocity to zero");
                }
            }
        });
    }

    let running = Arc::new(AtomicBool::new(true));
    let spin = {
        let running = running.clone();
        tokio::task::spawn_blocking(move || {
            while running.load(Ordering::Relaxed) {
                node.spin_once(Duration::from_millis(20));
            }
            drop(node);
        })
    };
    m.shutdown.wait().await;
    // destroy_node() published a final zero in the Python node
    let _ = publisher.publish(&twist(0.0, 0.0));
    running.store(false, Ordering::Relaxed);
    let _ = spin.await;
    Ok(())
}
