//! The guard's IO on the in-process bus (step E2 of docs/ROS_FREE_PLAN.md):
//! what `lib.rs`'s r2r IO does, on the module's bus scope and clock.
//!
//! - **Names:** the same relative names, resolved as rcl resolves them on the
//!   node with the process's `-r` rules (`ModuleCtx::resolve_topic`), so that
//!   the final guard reads `/cmd_vel_guard_input` and writes
//!   `/drivetrain_guarded_cmd_vel`, and the manual guard reads
//!   `/app_joy_cmd`, writes `/joy_cmd` and its command clock
//!   `/manual_command_clock`, as before.
//! - **QoS:** what the r2r IO hands r2r, reliable, volatile, keep last 1.
//! - **Rules:** `core.rs`, unchanged: the same limits from the same
//!   parameters, the same barrier, rejections and replay rule.
//! - **Clocks:** the module's (`m.clock`): its wall time for the stamps the
//!   guard writes and checks, its monotonic time for the receipt timeout, and
//!   wall timers in rcl's schedule for the watchdog and the command clock
//!   (first tick one period after the start; the clock's first frame goes out
//!   at once, as on r2r). In production that is the system clock and
//!   CLOCK_MONOTONIC, what the r2r IO reads.
//! - **Session** (the manual guard, `require_command_session`): a fresh
//!   `manual-session-v1:<32 hex>` per start, in every clock frame's
//!   `frame_id`; a command is from this session when its `frame_id` is that.
//! - **Stops:** a zero before anything can move, a zero from the watchdog, a
//!   zero for every rejection, and a last zero at shutdown, after the
//!   command and watchdog tasks have stopped, so that nothing they still had
//!   in hand can follow it.
//!
//! E2 step 6 moves the final guard, step 7 the manual guard.
//!
//! The r2r node stays for the parameters, the /rosout logs and the ROS graph.
//! It has no endpoint, so it is not spun.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use mower_bus::{Clock, Mono, Qos};
use mower_rs_common::{ModuleCtx, ModuleResult};
use r2r::builtin_interfaces::msg::Time;
use r2r::geometry_msgs::msg::TwistStamped;
use r2r::std_msgs::msg::Header;

use crate::core::{Action, Command, Guard};

/// The r2r IO's `QosProfile::default().keep_last(1)`: R V 1.
const ONE: Qos = Qos::default().keep_last(1);

/// The module's clock as the guard reads time: wall nanoseconds for stamps,
/// and the monotonic clock as the `Instant` `core.rs` takes for receipt
/// timeouts (an `Instant` at the start, carried forward by the clock's
/// monotonic time, so that a manual clock drives it in tests).
#[derive(Clone)]
struct Clocks {
    clock: Clock,
    start: (Instant, Mono),
}

impl Clocks {
    fn new(clock: Clock) -> Self {
        let start = (Instant::now(), clock.mono());
        Clocks { clock, start }
    }

    fn wall_ns(&self) -> i64 {
        self.clock.now().ns()
    }

    fn mono(&self) -> Instant {
        self.start.0 + self.clock.mono().saturating_duration_since(self.start.1)
    }

    fn stamp(&self) -> Time {
        let (sec, nanosec) = self.clock.now().to_parts();
        Time { sec, nanosec }
    }

    /// `lib.rs`'s `twist`: a fresh robot stamp, `base_footprint`. Every
    /// upstream stamp is replaced with robot time, and a forced zero is an
    /// ordering barrier too.
    fn twist(&self, linear_x: f64, angular_z: f64) -> TwistStamped {
        let mut msg = TwistStamped::default();
        msg.header.stamp = self.stamp();
        msg.header.frame_id = "base_footprint".to_string();
        msg.twist.linear.x = linear_x;
        msg.twist.angular.z = angular_z;
        msg
    }
}

/// One guard instance on the bus, with its node `node` (parameters, log).
pub(crate) async fn run_node(node: r2r::Node, m: ModuleCtx) -> ModuleResult {
    let logger = node.logger().to_string();
    let limits = crate::read_limits(&node)?;
    let session_id = if limits.require_command_session { Some(crate::new_session_id()?) } else { None };
    let clocks = Clocks::new(m.clock.clone());

    let publisher = m.bus.publisher::<TwistStamped>(&m.resolve_topic("cmd_vel_out")?, ONE)?;
    let mut commands = m.bus.subscribe::<TwistStamped>(&m.resolve_topic("cmd_vel_in")?, ONE)?;
    let clock_publisher =
        if session_id.is_some() { Some(m.bus.publisher::<Header>(&m.resolve_topic("command_clock")?, ONE)?) } else { None };
    let clock_timer = if session_id.is_some() {
        Some(m.bus.wall_timer(&m.clock, Duration::from_secs_f64(limits.command_clock_period_s))?)
    } else {
        None
    };
    let mut watchdog_timer = m.bus.wall_timer(&m.clock, Duration::from_secs_f64(limits.watchdog_period_s()))?;

    crate::log_limits(&logger, &limits);

    let guard = Arc::new(Mutex::new(Guard::new(limits)));
    // stop before anything can move, as the Python node did in __init__
    publisher.publish(&clocks.twist(0.0, 0.0))?;

    // the robot's command clock: a frame at once, then one per period
    let ticking = match (clock_publisher, clock_timer, session_id.clone()) {
        (Some(clock_publisher), Some(mut timer), Some(session_id)) => {
            let clocks = clocks.clone();
            let publish_clock = move || {
                let msg = Header { stamp: clocks.stamp(), frame_id: session_id.clone() };
                let _ = clock_publisher.publish(&msg);
            };
            publish_clock();
            Some(tokio::spawn(async move {
                while timer.tick().await.is_ok() {
                    publish_clock();
                }
            }))
        }
        _ => None,
    };

    let forwarding = {
        let (guard, publisher, logger, clocks) = (guard.clone(), publisher.clone(), logger.clone(), clocks.clone());
        tokio::spawn(async move {
            let mut last_rejection_log: Option<Instant> = None;
            while let Some(msg) = commands.recv().await {
                let command = Command {
                    stamp_sec: msg.header.stamp.sec,
                    stamp_nanosec: msg.header.stamp.nanosec,
                    session_matches: session_id.as_deref() == Some(msg.header.frame_id.as_str()),
                    linear: [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z],
                    angular: [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z],
                };
                let now = clocks.mono();
                let action = guard.lock().unwrap().on_command(&command, clocks.wall_ns(), now);
                match action {
                    Action::Forward { linear_x, angular_z } => {
                        if let Err(e) = publisher.publish(&clocks.twist(linear_x, angular_z)) {
                            r2r::log_error!(&logger, "publish cmd_vel_out failed: {:?}", e);
                        }
                    }
                    Action::Reject(reason) => {
                        if let Err(e) = publisher.publish(&clocks.twist(0.0, 0.0)) {
                            r2r::log_error!(&logger, "publish cmd_vel_out failed: {:?}", e);
                        }
                        if last_rejection_log.map(|t| now.duration_since(t).as_secs_f64() >= 1.0).unwrap_or(true) {
                            last_rejection_log = Some(now);
                            r2r::log_error!(&logger, "Rejected drivetrain command: {}", reason);
                        }
                    }
                }
            }
        })
    };

    let watching = {
        let (guard, publisher, logger, clocks) = (guard.clone(), publisher.clone(), logger.clone(), clocks.clone());
        tokio::spawn(async move {
            while watchdog_timer.tick().await.is_ok() {
                if guard.lock().unwrap().watchdog(clocks.mono()) {
                    let _ = publisher.publish(&clocks.twist(0.0, 0.0));
                    r2r::log_error!(&logger, "Drivetrain command receipt timeout; forced velocity to zero");
                }
            }
        })
    };

    m.shutdown.wait().await;
    forwarding.abort();
    watching.abort();
    let _ = forwarding.await;
    let _ = watching.await;
    if let Some(ticking) = ticking {
        ticking.abort();
        let _ = ticking.await;
    }
    // destroy_node() published a final zero in the Python node
    let _ = publisher.publish(&clocks.twist(0.0, 0.0));
    drop(node);
    Ok(())
}
