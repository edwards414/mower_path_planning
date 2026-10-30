//! One owner of `/mower_recorder/*` (design 4.2, R6). The Python recorder
//! (deploy/docker-compose.yaml's `recorder` service, or a data-collection
//! run) and this module must not both serve the app's recorder endpoints:
//! two latched statuses, two bag lists, two answers to a command.
//!
//! The Python recorder is recognised by its server of
//! `/mower_recorder/start`: a DDS server of that name that is not a bus
//! bridge's (this module's own reaches DDS through the bridge). Each of the
//! three instances looks [`SETTLE`] after its start and every [`PERIOD`]
//! from then on. While there is such a server it holds none of its
//! endpoints, and says so in an ERROR, once; when the server is gone it
//! makes them and says that too. The look is `get_publishers_info_by_topic`
//! of the server's `rr<name>Reply` writer, as mower_bus_ros looks for a
//! service's server.

use std::time::Duration;

use mower_bus::Interval;
use mower_rs_common::{ModuleCtx, NameError};

/// The service whose foreign server means "another recorder runs".
pub const START_SERVICE: &str = "/mower_recorder/start";

/// How often an instance looks again.
pub const PERIOD: Duration = Duration::from_secs(10);

/// Before the first look: time for discovery to show a recorder that already
/// runs. Nothing of the instance is on the bus until then.
pub const SETTLE: Duration = Duration::from_secs(2);

/// The bus bridge's DDS endpoints: mower_rsd's bridge node, or a standalone
/// binary's (its namespace). As mower_mux's interlock tells them apart.
pub fn is_bridge(node: &str, namespace: &str) -> bool {
    (node == mower_bridge::NODE && matches!(namespace, "" | "/"))
        || namespace.trim_end_matches('/') == mower_bridge::STANDALONE_NAMESPACE
}

/// The DDS servers of `service` that are not a bus bridge's, as full node
/// names.
pub fn foreign_servers(node: &r2r::Node, service: &str) -> r2r::Result<Vec<String>> {
    let writers = node.get_publishers_info_by_topic(&format!("rr{service}Reply"), true)?;
    let mut nodes: Vec<String> = writers
        .iter()
        .filter(|w| !is_bridge(&w.node_name, &w.node_namespace))
        .map(|w| mower_recorder_core::graph::full_name(&w.node_name, &w.node_namespace))
        .collect();
    nodes.sort();
    nodes.dedup();
    Ok(nodes)
}

/// One instance's look-out.
pub struct Owner {
    service: String,
    logger: String,
    /// What the instance does not serve while a foreign recorder runs, for the log.
    what: &'static str,
    interval: Interval,
    foreign: Option<Vec<String>>,
    error_logged: bool,
}

impl Owner {
    /// The first look [`SETTLE`] after now, on `m`'s clock.
    pub fn new(m: &ModuleCtx, logger: &str, what: &'static str) -> Result<Owner, NameError> {
        Ok(Owner {
            service: m.resolve_service(START_SERVICE)?,
            logger: logger.to_string(),
            what,
            interval: m.clock.interval_at(m.clock.mono() + SETTLE, PERIOD),
            foreign: None,
            error_logged: false,
        })
    }

    /// The next look is due.
    pub async fn tick(&mut self) {
        self.interval.tick().await;
    }

    /// Whether the instance may serve, from the graph as `node` sees it now.
    /// A graph that cannot be read keeps the last answer (yes at first),
    /// with a WARN once.
    pub fn may_serve(&mut self, node: &r2r::Node) -> bool {
        match foreign_servers(node, &self.service) {
            Ok(others) => {
                self.error_logged = false;
                match (&self.foreign, others.is_empty()) {
                    (None, false) => {
                        r2r::log_error!(
                            &self.logger,
                            "{} is served on DDS by {}, not through this module's bus bridge: another recorder owns \
                             /mower_recorder/* (the Python recorder: RUST_RECORDER=true puts deploy/docker-compose.yaml's \
                             recorder service to sleep, and mower-data-collection.sh refuses to start while it is on). \
                             Not serving {} while it is there; looking again every {} s.",
                            self.service,
                            others.join(", "),
                            self.what,
                            PERIOD.as_secs()
                        );
                        self.foreign = Some(others);
                    }
                    (Some(_), true) => {
                        r2r::log_info!(&self.logger, "{} has no other server any more: serving {}", self.service, self.what);
                        self.foreign = None;
                    }
                    _ => {}
                }
            }
            Err(e) => {
                if !self.error_logged {
                    r2r::log_warn!(&self.logger, "cannot look for another server of {}: {e}", self.service);
                    self.error_logged = true;
                }
            }
        }
        self.foreign.is_none()
    }
}
