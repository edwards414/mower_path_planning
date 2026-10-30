//! twist_mux, the velocity multiplexer in front of the final guard, as a
//! module of mower_rsd (step E4 of docs/ROS_FREE_PLAN.md): module `mux`,
//! node `twist_mux`, launch switch `rust_mux` (compose `RUST_MUX`, off by
//! default and only with `rust_daemon`).
//!
//! - [`core`]: twist_mux 4.5.0's selection rules, a pure function of when
//!   each input and lock last had a message.
//! - [`config`]: the handles twist_mux builds from its parameters
//!   (twist_mux_topics.yaml, the file the C++ node reads too), in its order
//!   and with its log lines.
//! - [`diag`]: what it publishes on `/diagnostics`.
//! - [`run`]: the module, on the bus.
//!
//! Every rule was measured on the real twist_mux by `tools/mux_oracle.py`;
//! `tests/replay.rs` replays what it recorded, and the oracle runs this module
//! next to the C++ node on the same input scripts.
//!
//! **On the bus.** Each input and lock is a bus reader of its topic as rcl
//! resolves it on the node (`SystemDefaultsQoS`, R V 1 on Cyclone, as
//! twist_mux's readers); the output `cmd_vel_out` (robot.launch.py's
//! `twist_mux:cmd_vel_out:=/cmd_vel_guard_input`) and `/diagnostics` are bus
//! writers, `R V 1`. What crosses to DDS, the bridge carries: nav2's
//! `/nav_cmd_vel` and the manual guard's `/joy_cmd` in, the output to the
//! final guard and `/diagnostics` out.
//!
//! - **A forwarded input is the message as it arrived:** the same `Arc`,
//!   so stamp, frame and twist are bit for bit what its source sent.
//! - **One task, one message at a time.** The inputs are taken in handle
//!   order, then the locks, then the two diagnostics timers: the order in
//!   which messages that are ready together are handled.
//! - **The clock.** Arrivals are timed on `m.clock.mono()`, not the wall
//!   clock (decision D1): a wall-clock step changes no decision. Both
//!   diagnostics timers are on it too, so the two statuses of a second come
//!   together, twist_mux's first; twist_mux's are on two clocks and drift
//!   apart on a slewed one.
//! - **Two muxes are one too many.** Every 2 s (and at the start) the module
//!   looks at the DDS graph for writers of its output that are not the bus
//!   bridge's. While there is one, a C++ twist_mux the launch should not have
//!   started, it forwards nothing (it logs an ERROR once), so the final guard
//!   never gets two copies; the guard and the base then stop the wheels.
//! - **A restart** (mower_rsd starts a failed module again after 2 s) begins
//!   from scratch, as twist_mux does: every input expired, every lock
//!   locked.
//! - **The r2r node** `twist_mux` stays for the parameters, the log and the
//!   graph; nothing of its own is on DDS.

pub mod config;
pub mod core;
pub mod diag;

use std::collections::BTreeMap;
use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;
use std::task::Poll;
use std::time::Duration;

use futures::Stream;
use mower_bus::{Qos, Subscriber, Time};
use mower_rs_common::{ModuleCtx, ModuleResult};
use r2r::builtin_interfaces::msg::Time as RosTime;
use r2r::diagnostic_msgs::msg::{DiagnosticArray, DiagnosticStatus, KeyValue};
use r2r::geometry_msgs::msg::TwistStamped;
use r2r::std_msgs::msg::{Bool, Header};

pub use crate::config::{from_params, Config, ConfigError, Log, Value};
pub use crate::core::{timeout_ns, Handle, Mux};

/// How often the module looks for a second writer of its output.
pub const INTERLOCK_PERIOD: Duration = Duration::from_secs(2);

/// twist_mux's own diagnostics timer (`DIAGNOSTICS_PERIOD`).
const MUX_DIAGNOSTICS_PERIOD: Duration = Duration::from_secs(1);

/// The module. Parameters, log and graph presence on its r2r node; inputs,
/// output and `/diagnostics` on `m.bus`.
pub async fn run(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    let node = r2r::Node::create(ctx, &m.node_name, &m.namespace)?;
    run_node(node, m).await
}

/// The node's parameter overrides as plain values (rcl's names, the nested
/// yaml keys joined with dots).
pub fn params_of(node: &r2r::Node) -> BTreeMap<String, Value> {
    use r2r::ParameterValue as P;
    let params = node.params.lock().unwrap_or_else(|e| e.into_inner());
    params
        .iter()
        .filter_map(|(name, p)| {
            let v = match &p.value {
                P::NotSet => return None,
                P::Bool(b) => Value::Bool(*b),
                P::Integer(i) => Value::Integer(*i),
                P::Double(d) => Value::Double(*d),
                P::String(s) => Value::String(s.clone()),
                other => Value::Other(format!("{other:?}")),
            };
            Some((name.clone(), v))
        })
        .collect()
}

/// Everything after `Node::create`, so that a test can hand in a node whose
/// parameters it has set itself.
pub async fn run_node(node: r2r::Node, m: ModuleCtx) -> ModuleResult {
    let logger = node.logger().to_string();
    let params = params_of(&node);
    let config = config::from_params(&params)?;
    for line in &config.logs {
        match line {
            Log::Info(text) => r2r::log_info!(&logger, "{text}"),
            Log::Warn(text) => r2r::log_warn!(&logger, "{text}"),
        }
    }
    // diagnostic_updater's own two parameters
    let updater_period = match params.get("diagnostic_updater.period") {
        None => 1.0,
        Some(Value::Double(p)) if p.is_finite() && *p > 0.0 => *p,
        Some(other) => return Err(format!("diagnostic_updater.period must be a positive double, not {other:?}").into()),
    };
    let status_node = match params.get("diagnostic_updater.use_fqn") {
        None | Some(Value::Bool(false)) => node.name()?,
        Some(Value::Bool(true)) => node.fully_qualified_name()?,
        Some(other) => return Err(format!("diagnostic_updater.use_fqn must be a bool, not {other:?}").into()),
    };

    // as twist_mux builds them: the readers in handle order, then the
    // output, then the diagnostics, which announce the start at once
    let mut inputs = Vec::new();
    for h in &config.inputs {
        inputs.push(m.bus.subscribe::<TwistStamped>(&m.resolve_topic(&h.topic)?, Qos::system_default())?);
    }
    let mut locks = Vec::new();
    for h in &config.locks {
        locks.push(m.bus.subscribe::<Bool>(&m.resolve_topic(&h.topic)?, Qos::system_default())?);
    }
    let out_name = m.resolve_topic("cmd_vel_out")?;
    let out = m.bus.publisher::<TwistStamped>(&out_name, Qos::default().keep_last(1))?;
    let diagnostics = m.bus.publisher::<DiagnosticArray>(&m.resolve_topic("/diagnostics")?, Qos::default().keep_last(1))?;
    diagnostics.publish_owned(array(m.clock.now(), diag::starting_up(&status_node)))?;
    // both timers from the same instant: twist_mux's tick first, then the
    // updater's, in the same turn
    let mut mux_timer = m.bus.wall_timer(&m.clock, MUX_DIAGNOSTICS_PERIOD)?;
    let mut updater_timer = m.bus.wall_timer(&m.clock, Duration::from_secs_f64(updater_period))?;
    let mut interlock = m.clock.interval(INTERLOCK_PERIOD);

    let mut mux = Mux::new(&config.inputs, &config.locks);
    // twist_mux hands the updater its status at its first tick; until then
    // the updater's status has no handles
    let mut lock_priority: Option<u8> = None;
    let mut foreign: Option<Vec<String>> = None;
    let mut graph_error_logged = false;
    r2r::log_info!(
        &logger,
        "mux: {} inputs and {} locks to {out_name}, on the bus (step E4; tools/mux_oracle.py)",
        config.inputs.len(),
        config.locks.len()
    );

    loop {
        tokio::select! {
            biased;
            () = m.shutdown.wait() => break,
            next = next_of(&mut inputs) => {
                let Some((i, msg)) = next else { break };
                if mux.on_input(i, m.clock.mono()) && foreign.is_none() {
                    out.publish_arc(msg)?;
                }
            }
            next = next_of(&mut locks) => {
                let Some((j, msg)) = next else { break };
                mux.on_lock(j, msg.data, m.clock.mono());
            }
            tick = mux_timer.tick() => {
                if tick.is_err() {
                    break;
                }
                let now = m.clock.mono();
                let priority = mux.lock_priority(now);
                lock_priority = Some(priority);
                diagnostics.publish_owned(array(m.clock.now(), diag::status(&status_node, &mux, priority, now)))?;
            }
            tick = updater_timer.tick() => {
                if tick.is_err() {
                    break;
                }
                let status = match lock_priority {
                    Some(priority) => diag::status(&status_node, &mux, priority, m.clock.mono()),
                    None => diag::status_without_handles(&status_node),
                };
                diagnostics.publish_owned(array(m.clock.now(), status))?;
            }
            _ = interlock.tick() => {
                match node.get_publishers_info_by_topic(&out_name, false) {
                    Ok(writers) => {
                        let others: Vec<String> = writers
                            .iter()
                            .filter(|w| !is_bridge(&w.node_name, &w.node_namespace))
                            .map(|w| format!("{}/{}", w.node_namespace.trim_end_matches('/'), w.node_name))
                            .collect();
                        match (&foreign, others.is_empty()) {
                            (None, false) => {
                                r2r::log_error!(
                                    &logger,
                                    "mux: {out_name} has another writer ({}): two muxes would feed the final guard \
                                     (rust_mux with a C++ twist_mux still running?). Forwarding nothing while it is \
                                     there.",
                                    others.join(", ")
                                );
                                foreign = Some(others);
                            }
                            (Some(_), true) => {
                                r2r::log_info!(&logger, "mux: {out_name} has no other writer any more: forwarding again");
                                foreign = None;
                            }
                            _ => {}
                        }
                        graph_error_logged = false;
                    }
                    Err(e) if !graph_error_logged => {
                        r2r::log_warn!(&logger, "mux: cannot look at the writers of {out_name}: {e}");
                        graph_error_logged = true;
                    }
                    Err(_) => {}
                }
            }
        }
    }
    drop(node);
    Ok(())
}

/// The bus bridge's DDS writer: mower_rsd's bridge node, or a standalone
/// binary's (its namespace).
fn is_bridge(node: &str, namespace: &str) -> bool {
    (node == mower_bridge::NODE && matches!(namespace, "" | "/"))
        || namespace.trim_end_matches('/') == mower_bridge::STANDALONE_NAMESPACE
}

/// The next message of the first reader that has one, in their order, with
/// its index; `None` once a reader's scope is closed. Pending forever with no
/// readers. Cancel-safe: a reader's message is taken only when returned.
fn next_of<T: mower_bus::Message>(readers: &mut [Subscriber<T>]) -> impl Future<Output = Option<(usize, Arc<T>)>> + '_ {
    std::future::poll_fn(move |cx| {
        for (i, reader) in readers.iter_mut().enumerate() {
            match Pin::new(reader).poll_next(cx) {
                Poll::Ready(Some(msg)) => return Poll::Ready(Some((i, msg))),
                Poll::Ready(None) => return Poll::Ready(None),
                Poll::Pending => {}
            }
        }
        Poll::Pending
    })
}

/// diagnostic_updater's array: the wall clock at the publish, an empty
/// frame.
fn array(stamp: Time, status: diag::Status) -> DiagnosticArray {
    let (sec, nanosec) = stamp.to_parts();
    DiagnosticArray {
        header: Header { stamp: RosTime { sec, nanosec }, frame_id: String::new() },
        status: vec![DiagnosticStatus {
            level: status.level,
            name: status.name,
            message: status.message,
            hardware_id: status.hardware_id,
            values: status.values.into_iter().map(|(key, value)| KeyValue { key, value }).collect(),
        }],
    }
}
