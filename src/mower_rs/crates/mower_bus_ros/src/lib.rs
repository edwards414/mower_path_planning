//! The bridge between the in-process bus (`mower_bus`) and DDS (r2r), for
//! the names that straddle the boundary while ROS nodes stay outside
//! `mower_rsd` (docs/ROS_FREE_PLAN.md Phase E; which names, and until when:
//! docs/BUS_TOPICS.md 5.3). README.md has the rules this keeps and why.
//!
//! A module gives a [`RosBridge`] its own r2r node and its bus scope,
//! declares what to forward, and runs it until shutdown:
//!
//! ```text
//! let mut bridge = RosBridge::new(r2r::Node::create(ctx, &m.node_name, &m.namespace)?, m.bus.clone());
//! bridge.to_ros::<Odometry>("/odom", QosProfile::system_default())?;                       // bus -> DDS
//! bridge.from_ros::<TwistStamped>("/app_joy_cmd", QosProfile::default().keep_last(10))?;   // DDS -> bus
//! bridge.service_to_ros::<Trigger::Service>("/save_map")?;                                 // DDS clients -> bus server
//! bridge.service_from_ros::<GetState::Service>("/bt_navigator/get_state")?;                // bus clients -> DDS server
//! bridge.run(m.shutdown.wait()).await?;
//! ```

use std::fmt;
use std::future::Future;
use std::pin::Pin;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use mower_bus::{BusError, Scope};
use r2r::QosProfile;

mod claims;
mod log;
mod node;
mod qos;
mod service;
mod stats;
mod topic;

pub use qos::{bus_qos, UnsupportedQos};
pub use stats::{BridgeStats, EntryStats};

use claims::Claims;
use log::Log;
use node::{Commands, NodeHandle};

/// What can be forwarded: an r2r message type, which is also a bus message
/// (r2r derives `Clone, Serialize, Deserialize`), so both sides carry the
/// same type and nothing is converted.
pub trait BridgeMessage: r2r::WrappedTypesupport + mower_bus::Message {}
impl<T: r2r::WrappedTypesupport + mower_bus::Message> BridgeMessage for T {}

/// An r2r service type (`…::srv::<Name>::Service`) whose request and response
/// are bus messages.
pub trait BridgeService:
    r2r::WrappedServiceTypeSupport<Request: BridgeMessage, Response: BridgeMessage> + 'static
{
}
impl<S> BridgeService for S
where
    S: r2r::WrappedServiceTypeSupport + 'static,
    S::Request: BridgeMessage,
    S::Response: BridgeMessage,
{
}

/// Which way a name crosses; named after the [`RosBridge`] method.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Direction {
    /// Bus writers to a DDS writer.
    ToRos,
    /// DDS writers to a bus writer.
    FromRos,
    /// DDS clients to the bus server.
    ServiceToRos,
    /// Bus clients to the DDS server.
    ServiceFromRos,
}

impl Direction {
    /// Topics and services are separate namespaces, on DDS as on the bus.
    fn kind(self) -> Kind {
        match self {
            Direction::ToRos | Direction::FromRos => Kind::Topic,
            Direction::ServiceToRos | Direction::ServiceFromRos => Kind::Service,
        }
    }
}

impl fmt::Display for Direction {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Direction::ToRos => "to_ros",
            Direction::FromRos => "from_ros",
            Direction::ServiceToRos => "service_to_ros",
            Direction::ServiceFromRos => "service_from_ros",
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub(crate) enum Kind {
    Topic,
    Service,
}

/// Why declaring or running a bridge failed.
#[derive(Debug)]
pub enum BridgeError {
    /// `name` is bridged already, by this bridge or another one in the
    /// process: one direction, and one bridge, per name (README.md).
    AlreadyBridged { name: String, existing: Direction, requested: Direction },
    /// The QoS has no bus equivalent ([`bus_qos`]).
    UnsupportedQos { name: String, reason: &'static str },
    /// The bus refused the endpoint: a relative name, another type on the
    /// name, a service served already, or a closed scope.
    Bus(BusError),
    /// r2r refused the DDS endpoint.
    Ros(r2r::Error),
    /// The spin thread or a forwarder panicked (a bug); the module should
    /// fail so that it is restarted.
    Panicked(String),
}

impl fmt::Display for BridgeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BridgeError::AlreadyBridged { name, existing, requested } => {
                write!(f, "{requested} {name}: already bridged {existing}")
            }
            BridgeError::UnsupportedQos { name, reason } => write!(f, "{name}: the QoS has no bus equivalent: {reason}"),
            BridgeError::Bus(e) => write!(f, "bus: {e}"),
            BridgeError::Ros(e) => write!(f, "ros: {e}"),
            BridgeError::Panicked(what) => write!(f, "panicked: {what}"),
        }
    }
}

impl std::error::Error for BridgeError {}

impl From<BusError> for BridgeError {
    fn from(e: BusError) -> Self {
        BridgeError::Bus(e)
    }
}

impl From<r2r::Error> for BridgeError {
    fn from(e: r2r::Error) -> Self {
        BridgeError::Ros(e)
    }
}

/// One forwarder: everything it does for one name, until the bridge's
/// scope closes.
pub(crate) type Forwarder = Pin<Box<dyn Future<Output = ()> + Send>>;

/// Forwards declared names between the bus and DDS (README.md).
///
/// Declarations create both endpoints at once, so a bad name, a type that
/// differs from the bus's or a QoS without a bus equivalent fails at the
/// call; only `to_ros`'s DDS writer waits for [`RosBridge::run`], because it
/// exists only while a bus writer does.
pub struct RosBridge {
    node: r2r::Node,
    scope: Scope,
    logger: String,
    handle: NodeHandle,
    commands: Commands,
    forwarders: Vec<Forwarder>,
    claims: Claims,
    stats: BridgeStats,
}

impl RosBridge {
    /// A bridge that owns `node` and forwards for `scope`, the module's
    /// scope: its endpoints are those of the scope, and closing the scope
    /// ends the bridge.
    pub fn new(node: r2r::Node, scope: Scope) -> Self {
        let logger = node.logger().to_string();
        let (handle, commands) = node::queue();
        RosBridge {
            node,
            scope,
            logger,
            handle,
            commands,
            forwarders: Vec::new(),
            claims: Claims::default(),
            stats: BridgeStats::default(),
        }
    }

    /// The counters of every name this bridge forwards, also while it runs.
    pub fn stats(&self) -> BridgeStats {
        self.stats.clone()
    }

    /// Forward bus topic `name` to DDS. `qos` is the profile its source, the
    /// bus writer, had as an r2r publisher: the DDS writer uses it, the
    /// bus reader its [`bus_qos`].
    pub fn to_ros<T: BridgeMessage>(&mut self, name: &str, qos: QosProfile) -> Result<(), BridgeError> {
        self.declare(name, Direction::ToRos, &qos, |bridge, bus_qos, env| {
            topic::to_ros::<T>(&bridge.scope, name, qos.clone(), bus_qos, env)
        })
    }

    /// Forward DDS topic `name` to the bus. `qos` is the profile its source,
    /// the DDS writer, has: the DDS reader uses it, the bus writer its
    /// [`bus_qos`].
    pub fn from_ros<T: BridgeMessage>(&mut self, name: &str, qos: QosProfile) -> Result<(), BridgeError> {
        self.declare(name, Direction::FromRos, &qos, |bridge, bus_qos, env| {
            topic::from_ros::<T>(&bridge.scope, &mut bridge.node, name, qos.clone(), bus_qos, env)
        })
    }

    /// Serve bus service `name` on DDS: DDS clients call the bus server.
    pub fn service_to_ros<S: BridgeService>(&mut self, name: &str) -> Result<(), BridgeError> {
        self.declare(name, Direction::ServiceToRos, &QosProfile::services_default(), |bridge, _, env| {
            service::service_to_ros::<S>(&bridge.scope, &mut bridge.node, name, env.counters, env.log)
        })
    }

    /// Serve DDS service `name` on the bus: bus clients call the DDS server.
    pub fn service_from_ros<S: BridgeService>(&mut self, name: &str) -> Result<(), BridgeError> {
        self.declare(name, Direction::ServiceFromRos, &QosProfile::services_default(), |bridge, _, env| {
            service::service_from_ros::<S>(&bridge.scope, &mut bridge.node, name, env.counters, env.log)
        })
    }

    /// The checks every declaration shares, in the order that touches
    /// nothing before a refusal: the QoS, the name's claim, then `make`'s
    /// endpoints. A failed `make` gives the claim back.
    fn declare(
        &mut self, name: &str, direction: Direction, qos: &QosProfile,
        make: impl FnOnce(&mut Self, mower_bus::Qos, topic::Env) -> Result<Forwarder, BridgeError>,
    ) -> Result<(), BridgeError> {
        let bus_qos = bus_qos(qos).map_err(|e| BridgeError::UnsupportedQos { name: name.to_string(), reason: e.reason })?;
        self.claims.claim(name, direction)?;
        let env = topic::Env {
            node: self.handle.clone(),
            counters: self.stats.register(name, direction),
            log: Log::new(&self.logger, format!("{direction} {name}")),
        };
        match make(self, bus_qos, env) {
            Ok(forwarder) => {
                self.forwarders.push(forwarder);
                Ok(())
            }
            Err(e) => {
                self.claims.release(name, direction);
                self.stats.unregister(name, direction);
                Err(e)
            }
        }
    }

    /// Spin the node and forward until `shutdown` completes or the bridge's
    /// scope closes. Before returning it ends every forwarder (their bus
    /// endpoints go with them), stops the spin thread, which drops the node
    /// and with it the DDS endpoints, and then gives the names back.
    ///
    /// Await it to the end: dropped half-way, it still stops the spin
    /// thread, but without waiting for it.
    pub async fn run(self, shutdown: impl Future<Output = ()>) -> Result<(), BridgeError> {
        let RosBridge { node, scope, logger, handle, commands, forwarders, claims, stats: _ } = self;
        r2r::log_info!(&logger, "mower_bus_ros: forwarding {} names between the bus and DDS", forwarders.len());
        let running = Arc::new(AtomicBool::new(true));
        let stop = StopSpin(running.clone());
        let mut spin = tokio::task::spawn_blocking(move || node::spin(node, commands, running));
        let mut tasks = tokio::task::JoinSet::new();
        for forwarder in forwarders {
            tasks.spawn(forwarder);
        }
        let mut shutdown = std::pin::pin!(shutdown);
        // The scope can close with no forwarder of its own to notice (a
        // bridge may declare nothing), so wait for that directly.
        let mut closed = std::pin::pin!(scope.closed());
        let mut failure = None;
        let mut spun = None;
        loop {
            tokio::select! {
                () = &mut shutdown => break,
                () = &mut closed => break,
                // The spin thread stops only when told to, so ending now is a
                // panic (r2r's spin_once unwraps its own locks). Nothing
                // would cross to or from DDS any more.
                ended = &mut spin => {
                    failure = Some(BridgeError::Panicked(match &ended {
                        Err(e) => format!("the spin thread: {e}"),
                        Ok(()) => "the spin thread stopped".to_string(),
                    }));
                    spun = Some(ended);
                    break;
                }
                // A forwarder ends on its own only when the scope closes.
                Some(ended) = tasks.join_next() => if let Err(e) = ended {
                    failure = Some(BridgeError::Panicked(format!("a forwarder: {e}")));
                    break;
                },
            }
        }
        // Forwarders first: they hold the r2r handles, and their bus
        // endpoints must not outlive the bridge.
        tasks.shutdown().await;
        drop(handle);
        drop(stop);
        let spun = match spun {
            Some(spun) => spun,
            None => spin.await,
        };
        // The names are free again only once their DDS endpoints are gone.
        drop(claims);
        match (failure, spun) {
            (Some(failure), _) => Err(failure),
            (None, Err(e)) => Err(BridgeError::Panicked(format!("the spin thread: {e}"))),
            (None, Ok(())) => Ok(()),
        }
    }
}

/// Stops the spin thread when `run` ends, however it ends.
struct StopSpin(Arc<AtomicBool>);

impl Drop for StopSpin {
    fn drop(&mut self) {
        self.0.store(false, Ordering::Release);
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::*;

    /// The process's r2r context, on the DDS tests' domain and RMW
    /// (tests/dds.rs has why).
    fn context() -> r2r::Context {
        static CONTEXT: std::sync::OnceLock<r2r::Context> = std::sync::OnceLock::new();
        CONTEXT
            .get_or_init(|| {
                std::env::set_var("ROS_DOMAIN_ID", "77");
                std::env::set_var("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp");
                r2r::Context::create().expect("an rcl context (source /opt/ros/jazzy/setup.bash)")
            })
            .clone()
    }

    #[test]
    fn run_fails_once_its_spin_thread_has_died() {
        let runtime = tokio::runtime::Builder::new_multi_thread().worker_threads(2).enable_all().build().unwrap();
        runtime.block_on(async {
            let bus = mower_bus::Bus::new();
            let node = r2r::Node::create(context(), "spin_dies_bridge", "").unwrap();
            let mut bridge = RosBridge::new(node, bus.scope("bridge"));
            bridge.to_ros::<r2r::std_msgs::msg::String>("/bridge_unit/spin_dies", QosProfile::default()).unwrap();
            let handle = bridge.handle.clone();
            let (_keep_running, never) = tokio::sync::oneshot::channel::<()>();
            let run = tokio::spawn(bridge.run(async move {
                let _ = never.await;
            }));
            // What a panic inside r2r's spin_once would do: the thread ends.
            assert!(handle.call(|_| panic!("a panic on the spin thread, on purpose")).await.is_none());
            let ended = tokio::time::timeout(Duration::from_secs(20), run).await.expect("run returned").unwrap();
            assert!(matches!(ended, Err(BridgeError::Panicked(ref what)) if what.starts_with("the spin thread")), "{ended:?}");
        });
    }
}
