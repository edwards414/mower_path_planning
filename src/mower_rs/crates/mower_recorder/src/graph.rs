//! `graph_snapshot` (graph_snapshot_node.py): every `period_s` the DDS graph
//! as JSON on `/graph_snapshot`, so that it lands in the bag, and the node
//! diff between ticks on `/graph_events` and in the log; a watched node that
//! leaves raises the fault. The JSON and the watch rule are
//! `mower_recorder_core::graph`'s; the graph is the r2r node's
//! (`get_node_names_and_namespaces`, `get_topic_names_and_types`, the
//! endpoints of each topic, `get_service_names_and_types`: vendor/r2r patch
//! 3), in rmw's order, which is by name.
//!
//! - **Parameters** as record.launch.py passes them (mower_rsd.yaml):
//!   `period_s` 2.0, `watch_nodes` `[map_manage, boustrophedon_coverage]`
//!   (no leading slash, so the fault never fires: design 1.12 #1),
//!   `fault_topic` `/mower_recorder/fault`. The code's defaults are the
//!   Python node's (`watch_nodes` `['']`: none).
//! - **First tick** one period after the start of serving, as rclpy's
//!   `create_timer`; the first emits no event.
//! - **The stamp** is the process's wall clock (`use_sim_time` is false),
//!   taken after the graph was read, as the Python takes it.
//! - **Off the runtime:** the graph is read on a blocking thread
//!   (`spawn_blocking`): rmw's graph cache is walked once per topic for each
//!   count, which must not hold up the other modules' tasks on a worker.

use std::collections::BTreeSet;
use std::sync::{Arc, Mutex, MutexGuard};

use mower_bus::{BusError, Publisher, Qos, Timer};
use mower_recorder_core::graph::{self, Event, Service, Topic, Watch};
use mower_rs_common::{params, ModuleCtx, ModuleResult};
use r2r::std_msgs::msg::{Bool, String as Text};

use crate::owner::Owner;

/// The node: parameters, log, graph presence and the graph it reads on r2r;
/// the three writers on `m.bus`.
pub async fn run(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    let node = r2r::Node::create(ctx, &m.node_name, &m.namespace)?;
    run_node(node, m).await
}

/// What the node publishes while it serves.
struct Active {
    snapshot: Publisher<Text>,
    events: Publisher<Text>,
    fault: Publisher<Bool>,
    timer: Timer,
    watch: Watch,
}

struct Settings {
    period_s: f64,
    watch_nodes: Vec<String>,
    snapshot_topic: String,
    events_topic: String,
    fault_topic: String,
}

/// Everything after `Node::create`, so that a test can hand in a node with
/// parameters of its own.
pub async fn run_node(node: r2r::Node, m: ModuleCtx) -> ModuleResult {
    let logger = node.logger().to_string();
    let period_s = params::f64(&node, "period_s", 2.0);
    if !(period_s.is_finite() && period_s > 0.0) {
        return Err(format!("period_s must be a positive number of seconds, not {period_s}").into());
    }
    let s = Settings {
        period_s,
        watch_nodes: crate::params::strings(&node, "watch_nodes", &[""]),
        snapshot_topic: m.resolve_topic("/graph_snapshot")?,
        events_topic: m.resolve_topic("/graph_events")?,
        fault_topic: m.resolve_topic(&params::string(&node, "fault_topic", "/mower_recorder/fault"))?,
    };
    let mut owner = Owner::new(&m, &logger, "/graph_snapshot, /graph_events and the fault")?;
    let mut active: Option<Active> = None;
    // shared with the blocking thread that reads the graph
    let node = Arc::new(Mutex::new(node));
    enum Next {
        Stop,
        Look,
        Tick(Result<(), BusError>),
    }
    loop {
        let next = match active.as_mut() {
            None => tokio::select! {
                biased;
                () = m.shutdown.wait() => Next::Stop,
                () = owner.tick() => Next::Look,
            },
            Some(a) => tokio::select! {
                biased;
                () = m.shutdown.wait() => Next::Stop,
                () = owner.tick() => Next::Look,
                tick = a.timer.tick() => Next::Tick(tick.map(|_| ())),
            },
        };
        match next {
            Next::Stop => break,
            Next::Look => {
                if !owner.may_serve(&lock(&node)) {
                    active = None;
                } else if active.is_none() {
                    active = Some(start(&m, &s, &logger)?);
                }
            }
            // the scope closed: the module is being stopped
            Next::Tick(Err(_)) => break,
            Next::Tick(Ok(())) => {
                let reader = node.clone();
                let read = tokio::task::spawn_blocking(move || read_graph(&lock(&reader))).await;
                match (read, active.as_mut()) {
                    (Ok(Ok(graph)), Some(a)) => a.publish(graph, &m, &logger)?,
                    (Ok(Err(e)), _) => r2r::log_warn!(&logger, "graph_snapshot: cannot read the graph: {e}"),
                    (Err(e), _) => r2r::log_warn!(&logger, "graph_snapshot: the graph read did not finish: {e}"),
                    (Ok(Ok(_)), None) => {}
                }
            }
        }
    }
    drop(active);
    Ok(())
}

fn start(m: &ModuleCtx, s: &Settings, logger: &str) -> Result<Active, BusError> {
    let active = Active {
        snapshot: m.bus.publisher::<Text>(&s.snapshot_topic, Qos::default())?,
        events: m.bus.publisher::<Text>(&s.events_topic, Qos::default())?,
        fault: m.bus.publisher::<Bool>(&s.fault_topic, Qos::default())?,
        timer: m.bus.wall_timer(&m.clock, std::time::Duration::from_secs_f64(s.period_s))?,
        watch: Watch::new(&s.watch_nodes),
    };
    let mut watched: Vec<&String> = s.watch_nodes.iter().filter(|w| !w.is_empty()).collect();
    watched.sort();
    watched.dedup();
    let shown: Vec<String> = watched.iter().map(|w| py_str_repr(w)).collect();
    r2r::log_info!(
        logger,
        "graph_snapshot up (period={}s, watch=[{}])",
        mower_pyfmt::repr::float(s.period_s),
        shown.join(", ")
    );
    Ok(active)
}

fn lock(node: &Mutex<r2r::Node>) -> MutexGuard<'_, r2r::Node> {
    node.lock().unwrap_or_else(|e| e.into_inner())
}

/// What one read of the graph gives: the nodes, the topics, the services.
pub type Graph = (BTreeSet<String>, Vec<Topic>, Vec<Service>);

impl Active {
    /// `_tick` after the reads: the snapshot, then the node diff.
    fn publish(&mut self, graph: Graph, m: &ModuleCtx, logger: &str) -> Result<(), BusError> {
        let (nodes, topics, services) = graph;
        let (sec, nanosec) = m.clock.now().to_parts();
        self.snapshot.publish_owned(Text { data: graph::snapshot_json(sec, nanosec, &nodes, &topics, &services) })?;
        for event in self.watch.tick(&nodes) {
            match event {
                Event::Disappeared(gone) => {
                    r2r::log_warn!(logger, "NODE DISAPPEARED: {gone}");
                    self.events.publish_owned(Text { data: graph::event_json("node_disappeared", &gone) })?;
                }
                Event::Fault(gone) => {
                    r2r::log_error!(logger, "WATCHED NODE DOWN: {gone} -> fault");
                    self.fault.publish_owned(Bool { data: true })?;
                }
                Event::Appeared(new) => {
                    self.events.publish_owned(Text { data: graph::event_json("node_appeared", &new) })?;
                }
            }
        }
        Ok(())
    }
}

/// The graph as rclpy's three calls and its counts give it: the nodes by
/// full name, the topics and the services in rmw's order (by name).
pub fn read_graph(node: &r2r::Node) -> r2r::Result<Graph> {
    let nodes: BTreeSet<String> =
        node.get_node_names_and_namespaces()?.iter().map(|(name, ns)| graph::full_name(name, ns)).collect();
    let mut named: Vec<(String, Vec<String>)> = node.get_topic_names_and_types()?.into_iter().collect();
    named.sort_by(|a, b| a.0.cmp(&b.0));
    let mut topics = Vec::with_capacity(named.len());
    for (name, types) in named {
        // rclpy's count_publishers / count_subscribers (rcl_count_*)
        let pubs = node.count_publishers(&name).unwrap_or(0) as i64;
        let subs = node.count_subscribers(&name).unwrap_or(0) as i64;
        topics.push(Topic { name, types, pubs, subs });
    }
    let services =
        node.get_service_names_and_types()?.into_iter().map(|(name, types)| Service { name, types }).collect();
    Ok((nodes, topics, services))
}

/// `repr(str)` of a node name, for the start line.
fn py_str_repr(s: &str) -> String {
    if s.contains('\'') && !s.contains('"') {
        format!("\"{s}\"")
    } else {
        format!("'{}'", s.replace('\\', "\\\\").replace('\'', "\\'"))
    }
}
