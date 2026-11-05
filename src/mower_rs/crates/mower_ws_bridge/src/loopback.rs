//! What the loopback listener serves beyond the public policy (E8-2): what
//! `mowerctl` needs to see a robot without DDS.
//!
//! - **Who gets it:** the listener bound to 127.0.0.1:`--loopback-port`, which
//!   has no gate, with `--loopback-policy` on the bus backend. Nothing else:
//!   lib.rs's `Access` hands a [`Loopback`] to that listener alone. A relayed
//!   phone arrives on 127.0.0.1:`--port` from a loopback peer address too, so
//!   the peer address is never looked at; only the listener decides.
//! - **Read, not write:** [`LoopbackPolicy`] adds names to read. It has no key
//!   for a topic to publish or a service to call, and refuses any other key,
//!   so what the listener may write stays the public policy's.
//! - **`/mowerd/{topics,services,actions,modules,bridge}`** are answered here
//!   from the bus's introspection and the process's [`Registry`]. They touch
//!   neither DDS nor any module.
//! - **Topics** are read with the same known-names rule as every client
//!   (bus_hub.rs), so a typo leaves nothing behind. A name the public policy
//!   does not allow is read in the scope `LOOPBACK_SCOPE`, which the manifest
//!   check does not look at. For a name only DDS has, the bus bridge opens a
//!   DDS reader while the bus has that reader, as for any bus reader
//!   (`topic info` shows the direction).
//! - **Parameter reads:** parameter services stay on r2r nodes until E9
//!   (mower_bridge's manifest, "Not listed"), so they have no bus name, and
//!   bridging them would put new DDS endpoints on the graph at all times.
//!   [`ParamReads`] instead keeps an r2r node of its own only while such a
//!   call is in hand; the graph has nothing new while nobody asks.

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::time::{Duration, Instant};

use mower_bus::{Bus, Durability, Qos, Reliability};
use mower_rs_common::Registry;
use serde::Deserialize;
use serde_json::{json, Map, Value};

use crate::bus_hub::nul_at;
use crate::config::{glob_match, Policy};
use crate::r2r_hub::R2rHub;

pub const TOPICS: &str = "/mowerd/topics";
pub const SERVICES: &str = "/mowerd/services";
pub const ACTIONS: &str = "/mowerd/actions";
pub const MODULES: &str = "/mowerd/modules";
pub const BRIDGE: &str = "/mowerd/bridge";

/// The only services [`LoopbackPolicy::parameter_services`] may name, each
/// with the only type it may have: the parameter services that only read.
const PARAMETER_READS: [(&str, &str); 4] = [
    ("/list_parameters", "rcl_interfaces/srv/ListParameters"),
    ("/get_parameters", "rcl_interfaces/srv/GetParameters"),
    ("/describe_parameters", "rcl_interfaces/srv/DescribeParameters"),
    ("/get_parameter_types", "rcl_interfaces/srv/GetParameterTypes"),
];

/// How long the node of parameter reads stays after the last one.
const PARAM_LINGER: Duration = Duration::from_secs(10);
/// That node's name; it is on the graph only around a parameter read.
const PARAM_NODE: &str = "mower_ws_bridge_params";

/// `--loopback-policy`: what the loopback listener may read on top of the
/// public policy (src/mower_bringup/config/ws_bridge_loopback.yaml).
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LoopbackPolicy {
    /// Topics it may subscribe to on top of the public policy's (globs, as
    /// `topics_sub`). Only names the bus knows are read (bus_hub.rs).
    #[serde(default)]
    topics_sub: Vec<String>,
    /// Parameter reads, glob -> type, e.g. `/*/get_parameters`.
    #[serde(default)]
    parameter_services: BTreeMap<String, String>,
}

impl LoopbackPolicy {
    pub fn from_yaml(text: &str) -> Result<Self, String> {
        let policy: LoopbackPolicy = serde_yaml::from_str(text).map_err(|e| format!("loopback policy: {e}"))?;
        for (glob, ty) in &policy.parameter_services {
            let allowed = PARAMETER_READS.iter().any(|(suffix, t)| glob.ends_with(suffix) && ty == t);
            if !allowed {
                return Err(format!("loopback policy: {glob} ({ty}) is not a parameter service that only reads"));
            }
        }
        Ok(policy)
    }
}

/// ROS's rule for an absolute name: tokens of letters, digits and underscores,
/// none starting with a digit. A glob lets a client spell the middle of a
/// name, so what reaches rcl (or a NUL its strings cannot hold) is checked.
fn is_ros_name(name: &str) -> bool {
    let Some(rest) = name.strip_prefix('/') else { return false };
    rest.split('/').all(|token| {
        !token.is_empty()
            && !token.starts_with(|c: char| c.is_ascii_digit())
            && token.chars().all(|c| c.is_ascii_alphanumeric() || c == '_')
    })
}

/// The loopback listener's wider view.
pub struct Loopback {
    policy: LoopbackPolicy,
    public: Arc<Policy>,
    bus: Bus,
    registry: Registry,
    started: Instant,
    params: ParamReads,
}

impl Loopback {
    /// `context`: the r2r context parameter reads make their node on; without
    /// one (the tests without DDS) they answer that no server does.
    pub fn new(
        policy: LoopbackPolicy, public: Arc<Policy>, bus: Bus, registry: Registry, context: Option<r2r::Context>,
        namespace: &str, logger: &str,
    ) -> Loopback {
        let params = ParamReads {
            context,
            namespace: namespace.to_string(),
            logger: logger.to_string(),
            state: Arc::default(),
        };
        Loopback { policy, public, bus, registry, started: Instant::now(), params }
    }

    /// Whether the loopback listener may read `topic` although the public
    /// policy does not allow it.
    pub fn may_subscribe(&self, topic: &str) -> bool {
        self.policy.topics_sub.iter().any(|g| glob_match(g, topic))
    }

    /// The type of `service` if it is a parameter read this policy allows.
    pub fn parameter_type(&self, service: &str) -> Option<&str> {
        if !is_ros_name(service) {
            return None;
        }
        self.policy.parameter_services.iter().find(|(g, _)| glob_match(g, service)).map(|(_, t)| t.as_str())
    }

    /// A parameter read, through the node of [`ParamReads`].
    pub async fn read_parameters(&self, service: &str, service_type: &str, args: Value, timeout: Duration) -> Result<Value, String> {
        if let Some(at) = nul_at(&args) {
            return Err(format!("{service}: request failed: {at} holds a NUL character"));
        }
        self.params.call(service, service_type, args, timeout).await
    }

    /// Stop the node of parameter reads, if it is there.
    pub fn shutdown(&self) {
        self.params.shutdown();
    }

    /// The answer to `/mowerd/*`; `None` for any other name. `args` may hold
    /// `names`, to answer for those alone (`topics`, `services`, `actions`).
    pub fn answer(&self, service: &str, args: &Value) -> Option<Value> {
        let names = args.get("names").and_then(Value::as_array).map(|names| names.iter().filter_map(Value::as_str).collect::<Vec<_>>());
        Some(match service {
            TOPICS => self.topics(names.as_deref()),
            SERVICES => self.services(names.as_deref()),
            ACTIONS => self.actions(names.as_deref()),
            MODULES => self.modules(),
            BRIDGE => self.bridge(),
            _ => return None,
        })
    }

    fn uptime_s(&self) -> f64 {
        self.started.elapsed().as_secs_f64()
    }

    /// `uptime_s` is the clock a rate is taken on: `published` over two
    /// answers, on the bridge's own clock, whatever the transport did.
    fn topics(&self, names: Option<&[&str]>) -> Value {
        let topics = match names {
            Some(names) => names.iter().filter_map(|n| self.bus.topic(n)).collect(),
            None => self.bus.topics(),
        };
        let rows: Vec<Value> = topics
            .into_iter()
            .map(|t| {
                let ty = t.ros_type.clone().or_else(|| t.type_name.map(str::to_string));
                let endpoints = |scopes: &[String], qos: &[Qos]| -> Vec<Value> {
                    scopes.iter().zip(qos).map(|(scope, q)| json!({"scope": scope, "qos": qos_json(*q)})).collect()
                };
                json!({
                    "name": t.name,
                    "type": ty,
                    "writers": endpoints(&t.publishers, &t.publisher_qos),
                    "readers": endpoints(&t.subscribers, &t.subscriber_qos),
                    "retained": t.retained,
                    "published": t.published,
                    "age_s": t.last_published.map(|at| at.elapsed().as_secs_f64()),
                })
            })
            .collect();
        json!({"uptime_s": self.uptime_s(), "topics": rows})
    }

    fn services(&self, names: Option<&[&str]>) -> Value {
        let rows: Vec<Value> = self
            .bus
            .services()
            .into_iter()
            .filter(|s| names.is_none_or(|names| names.contains(&s.name.as_str())))
            .map(|s| {
                json!({
                    "name": s.name,
                    "type": s.ros_type,
                    "server": s.server,
                    "clients": s.clients,
                    "calls": s.calls,
                    // what the app and this listener may call
                    "callable": self.public.service_type(&s.name).is_some(),
                })
            })
            .collect();
        json!({"services": rows})
    }

    fn actions(&self, names: Option<&[&str]>) -> Value {
        let rows: Vec<Value> = self
            .bus
            .actions()
            .into_iter()
            .filter(|a| names.is_none_or(|names| names.contains(&a.name.as_str())))
            .map(|a| {
                json!({
                    "name": a.name,
                    "type": a.ros_type,
                    "server": a.server,
                    "clients": a.clients,
                    "goals": a.goals,
                    "active": a.active,
                })
            })
            .collect();
        json!({"actions": rows})
    }

    /// The supervisor's slots, with each instance's endpoints on the bus; the
    /// scopes no slot names are listed too (a standalone binary has no
    /// supervisor; `LOOPBACK_SCOPE` while a debugging reader lives).
    fn modules(&self) -> Value {
        let mut counts = self.endpoint_counts();
        let rows: Vec<Value> = self
            .registry
            .slots()
            .into_iter()
            .map(|slot| {
                let module = slot.label.split('/').next().unwrap_or_default().to_string();
                let endpoints = counts.remove(&slot.node).unwrap_or_default();
                json!({
                    "label": slot.label,
                    "module": module,
                    "node": slot.node,
                    "state": slot.state.as_str(),
                    "starts": slot.starts,
                    "failures": slot.failures,
                    "up_s": slot.started_at.map(|at| at.elapsed().as_secs_f64()),
                    "last_failure": slot.last_failure.map(|(reason, at)| json!({"reason": reason, "age_s": at.elapsed().as_secs_f64()})),
                    "endpoints": endpoints,
                })
            })
            .collect();
        let others: Vec<Value> = counts.into_iter().map(|(scope, endpoints)| json!({"scope": scope, "endpoints": endpoints})).collect();
        json!({"modules": rows, "other_scopes": others})
    }

    fn endpoint_counts(&self) -> BTreeMap<String, Map<String, Value>> {
        let mut counts: BTreeMap<String, BTreeMap<&str, u64>> = BTreeMap::new();
        let mut add = |scopes: &[String], what: &'static str| {
            for scope in scopes {
                *counts.entry(scope.clone()).or_default().entry(what).or_default() += 1;
            }
        };
        for t in self.bus.topics() {
            add(&t.publishers, "publishes");
            add(&t.subscribers, "subscribes");
        }
        for s in self.bus.services() {
            add(s.server.as_slice(), "serves");
            add(&s.clients, "calls");
        }
        for a in self.bus.actions() {
            add(a.server.as_slice(), "serves_actions");
            add(&a.clients, "calls_actions");
        }
        counts.into_iter().map(|(scope, c)| (scope, c.into_iter().map(|(k, v)| (k.to_string(), json!(v))).collect())).collect()
    }

    fn bridge(&self) -> Value {
        match self.registry.bridge() {
            Some(entries) => json!({"running": true, "entries": entries}),
            None => json!({"running": false, "entries": []}),
        }
    }
}

fn qos_json(q: Qos) -> Value {
    let reliability = match q.reliability {
        Reliability::Reliable => "reliable",
        Reliability::BestEffort => "best_effort",
    };
    let durability = match q.durability {
        Durability::Volatile => "volatile",
        Durability::TransientLocal => "transient_local",
    };
    json!({"depth": q.depth, "reliability": reliability, "durability": durability})
}

/// Parameter reads through an r2r node of their own, spun by an [`R2rHub`],
/// which is there from the first call until [`PARAM_LINGER`] after the last.
struct ParamReads {
    context: Option<r2r::Context>,
    namespace: String,
    logger: String,
    state: Arc<Mutex<ParamState>>,
}

#[derive(Default)]
struct ParamState {
    hub: Option<R2rHub>,
    calls: usize,
    /// Moves with every call that ends, so that only the last one's retiring
    /// counts.
    generation: u64,
    /// The module is ending: no node may start behind its back, because rmw
    /// aborts a process that exits with a thread in `rcl_wait`.
    closed: bool,
}

fn lock(state: &Mutex<ParamState>) -> MutexGuard<'_, ParamState> {
    state.lock().unwrap_or_else(PoisonError::into_inner)
}

impl ParamReads {
    async fn call(&self, service: &str, service_type: &str, args: Value, timeout: Duration) -> Result<Value, String> {
        let hub = self.enter(service)?;
        let result = hub.call_service(service, service_type, args, timeout).await;
        self.leave();
        result
    }

    /// The hub, started if there is none, with this call counted.
    fn enter(&self, service: &str) -> Result<R2rHub, String> {
        let Some(context) = &self.context else {
            return Err(format!("{service}: service unavailable"));
        };
        let mut state = lock(&self.state);
        if state.closed {
            return Err(format!("{service}: service unavailable"));
        }
        if state.hub.is_none() {
            let node = r2r::Node::create(context.clone(), PARAM_NODE, &self.namespace).map_err(|e| format!("{service}: {e:?}"))?;
            state.hub = Some(R2rHub::start(node).map_err(|e| format!("{service}: {e}"))?);
            r2r::log_info!(&self.logger, "parameter reads: node {} up", PARAM_NODE);
        }
        state.calls += 1;
        Ok(state.hub.clone().expect("just started"))
    }

    /// One call less. The hub goes `PARAM_LINGER` after the last call, unless
    /// another came meanwhile. Its node's thread is joined off the runtime
    /// and under the lock, so that neither a new read nor the module's end
    /// finds a node half gone (a join takes at most one 100 ms spin).
    fn leave(&self) {
        let generation = {
            let mut state = lock(&self.state);
            state.calls -= 1;
            state.generation += 1;
            state.generation
        };
        let (state, logger) = (self.state.clone(), self.logger.clone());
        tokio::spawn(async move {
            tokio::time::sleep(PARAM_LINGER).await;
            let _ = tokio::task::spawn_blocking(move || {
                let mut state = lock(&state);
                if state.calls == 0 && state.generation == generation {
                    if let Some(hub) = state.hub.take() {
                        hub.shutdown();
                        r2r::log_info!(&logger, "parameter reads: node {} gone", PARAM_NODE);
                    }
                }
            })
            .await;
        });
    }

    /// The module ends: the node goes now and no read starts another.
    fn shutdown(&self) {
        let mut state = lock(&self.state);
        state.closed = true;
        if let Some(hub) = state.hub.take() {
            hub.shutdown();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SHIPPED: &str = include_str!("../../../../mower_bringup/config/ws_bridge_loopback.yaml");

    #[test]
    fn the_shipped_file_reads_everything_the_bus_knows_and_writes_nothing_more() {
        let p = LoopbackPolicy::from_yaml(SHIPPED).unwrap();
        assert_eq!(p.topics_sub, ["*"]);
        let added: Vec<&str> = p.parameter_services.keys().map(String::as_str).collect();
        assert_eq!(added, ["/*/describe_parameters", "/*/get_parameter_types", "/*/get_parameters", "/*/list_parameters"]);
    }

    #[test]
    fn a_policy_can_name_no_write() {
        // the keys of the public policy that write, and anything else, are refused
        for bad in [
            "topics_pub: ['/cmd_vel']\n",
            "services:\n  /system/restart: std_srvs/srv/Trigger\n",
            "parameter_services:\n  '/*/set_parameters': rcl_interfaces/srv/SetParameters\n",
            "parameter_services:\n  '/*/set_parameters_atomically': rcl_interfaces/srv/SetParametersAtomically\n",
            // a read's name with a writing type, a writing name with a read's type
            "parameter_services:\n  '/*/get_parameters': rcl_interfaces/srv/SetParameters\n",
            "parameter_services:\n  '/*/set_parameters': rcl_interfaces/srv/GetParameters\n",
            "parameter_services:\n  '/*': rcl_interfaces/srv/GetParameters\n",
            "parameter_services:\n  /system/update: std_srvs/srv/Trigger\n",
        ] {
            assert!(LoopbackPolicy::from_yaml(bad).is_err(), "{bad}");
        }
        assert!(LoopbackPolicy::from_yaml("topics_sub: ['*']\n").is_ok());
    }

    #[test]
    fn a_name_a_glob_lets_through_is_still_a_ros_name() {
        for good in ["/map_manage/get_parameters", "/a/b_c/d1/get_parameters", "/_x/get_parameters"] {
            assert!(is_ros_name(good), "{good}");
        }
        for bad in ["", "/", "//get_parameters", "map/get_parameters", "/a b/get_parameters", "/a\u{0}b/get_parameters", "/1a/get_parameters", "/a//get_parameters", "/a/", "/é/get_parameters"] {
            assert!(!is_ros_name(bad), "{bad:?}");
        }
    }
}
