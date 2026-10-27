//! The policy: from the instances that run in this process and their flags,
//! which names cross between the bus and DDS, which way, with which QoS
//! (README.md, "The policy"). A pure function of the manifest, the
//! instances and the process's `-r` rules: no DDS, no bus.

use std::collections::BTreeMap;
use std::fmt;

use mower_bus::{Durability, Qos, Reliability};
use mower_bus_ros::{BridgeError, Direction, RosBridge};
use mower_rs_common::{NameKind, Remaps};

use crate::manifest::{MODULES, OUTSIDE};
use crate::types::{Endpoint, Iface, Instance, Kind, Module, Profile};

/// An instance that runs in this process.
#[derive(Clone, Debug)]
pub struct Running {
    pub module: &'static Module,
    pub instance: &'static Instance,
    /// The node name rcl gives it here, which its names resolve under: the
    /// production name in mower_rsd; standalone, the binary's after
    /// `__node:=`.
    pub node: String,
    /// Whether its IO is on the bus.
    pub on_bus: bool,
}

impl Running {
    /// mower_rsd: every instance of the modules in `--modules`, on the bus as
    /// `on_bus` says. Ids the manifest does not have (`bridge`, `agent`) are
    /// outside in E2.
    pub fn rsd<S: AsRef<str>>(ids: &[S], on_bus: impl Fn(&str) -> bool) -> Vec<Running> {
        let mut running: Vec<Running> = Vec::new();
        for id in ids {
            let Some(module) = crate::module(id.as_ref()) else { continue };
            if running.iter().any(|r| r.module.id == module.id) {
                continue;
            }
            running.extend(module.instances.iter().map(|instance| Running {
                module,
                instance,
                node: instance.node.to_string(),
                on_bus: on_bus(module.id),
            }));
        }
        running
    }

    /// The binary of module `id`, whose node rcl names `node`: the instance
    /// of that name; every instance for a binary named after its module
    /// (`mower_localize`, `localize`, runs all three); a module's only
    /// instance under any name. On the bus whatever its flag says: a binary
    /// runs a bridge because its module moved.
    pub fn standalone(id: &str, node: &str) -> Result<Vec<Running>, String> {
        let module = crate::module(id).ok_or_else(|| format!("no module {id:?} in the manifest"))?;
        let running = |instance: &'static Instance, node: &str| Running {
            module,
            instance,
            node: node.to_string(),
            on_bus: true,
        };
        if let Some(instance) = module.instances.iter().find(|i| i.node == node) {
            return Ok(vec![running(instance, node)]);
        }
        match module.instances {
            [only] => Ok(vec![running(only, node)]),
            all if node == module.id => Ok(all.iter().map(|i| running(i, i.node)).collect()),
            all => Err(format!(
                "node {node:?} is none of module {id}'s instances ({})",
                all.iter().map(|i| i.node).collect::<Vec<_>>().join(", ")
            )),
        }
    }
}

/// One name to forward, and which way.
#[derive(Clone, Debug)]
pub struct Declaration {
    pub direction: Direction,
    pub name: String,
    /// The type, and for a topic the profile the bridge's DDS endpoint uses.
    pub iface: Iface,
    /// Whom it is for, for the log.
    pub why: String,
}

impl Declaration {
    /// Make the bridge forward it.
    pub fn declare(&self, bridge: &mut RosBridge) -> Result<(), BridgeError> {
        match (&self.iface, self.direction) {
            (Iface::Topic(t, qos), Direction::ToRos) => t.declare_to_ros(bridge, &self.name, *qos),
            (Iface::Topic(t, qos), Direction::FromRos) => t.declare_from_ros(bridge, &self.name, *qos),
            (Iface::Service(s), Direction::ServiceToRos) => s.declare_to_ros(bridge, &self.name),
            (Iface::Service(s), Direction::ServiceFromRos) => s.declare_from_ros(bridge, &self.name),
            (iface, direction) => unreachable!("{direction} for {iface:?}: the policy pairs topics with topic directions"),
        }
    }
}

/// `to_ros /battery_state sensor_msgs/msg/BatteryState R V 10`
impl fmt::Display for Declaration {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match &self.iface {
            Iface::Topic(t, qos) => write!(f, "{} {} {} {qos}", self.direction, self.name, t.ros_type()),
            Iface::Service(s) => write!(f, "{} {} {}", self.direction, self.name, s.ros_type()),
        }
    }
}

/// What the bridge of a process declares: topics first, then services, each
/// sorted by name.
#[derive(Clone, Debug, Default)]
pub struct Plan {
    pub declarations: Vec<Declaration>,
    /// What the policy left out, and why.
    pub notes: Vec<String>,
}

impl Plan {
    /// Nothing to forward: no instance of the process is on the bus.
    pub fn is_empty(&self) -> bool {
        self.declarations.is_empty()
    }

    pub fn declare(&self, bridge: &mut RosBridge) -> Result<(), BridgeError> {
        self.declarations.iter().try_for_each(|d| d.declare(bridge))
    }

    /// One line per declaration, as tests and logs write them.
    pub fn lines(&self) -> Vec<String> {
        self.declarations.iter().map(Declaration::to_string).collect()
    }

    /// The plan for a bridge whose node is created as `node` in `namespace`.
    /// rcl applies the process's `-r` rules to the bridge's names as to any
    /// node's, and the policy has resolved them already, as the module's node
    /// did: a rule that matches a resolved name again (a chain, a swap, a rule
    /// for the module's node name on a standalone bridge) would put the
    /// bridge's DDS endpoint on another name than the module's. Such a name is
    /// an error (README.md, "The policy").
    pub fn check_names(&self, remaps: &Remaps, node: &str, namespace: &str) -> Result<(), PlanError> {
        let problems: Vec<String> = self
            .declarations
            .iter()
            .filter_map(|d| match remaps.resolve(name_kind(d.direction), node, namespace, &d.name) {
                Ok(again) if again == d.name => None,
                Ok(again) => Some(format!(
                    "{d}: a -r rule renames {} again on the bridge's node {node}, to {again}, so the bridge would \
                     not carry it on the module's name",
                    d.name
                )),
                Err(e) => Some(format!("{d}: {e}")),
            })
            .collect();
        if problems.is_empty() {
            Ok(())
        } else {
            Err(PlanError { problems })
        }
    }
}

/// The rules a declaration's name meets: rcl_action resolves an action's name
/// as a topic's.
fn name_kind(direction: Direction) -> NameKind {
    match direction {
        Direction::ServiceToRos | Direction::ServiceFromRos => NameKind::Service,
        Direction::ToRos | Direction::FromRos | Direction::ActionToRos | Direction::ActionFromRos => NameKind::Topic,
    }
}

/// Why no bridge can be planned for the process: every problem found.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PlanError {
    pub problems: Vec<String>,
}

impl fmt::Display for PlanError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "no bus bridge for this set of modules: {}", self.problems.join("; "))
    }
}

impl std::error::Error for PlanError {}

/// Where an endpoint is, seen from this process.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Side {
    /// An instance of this process whose module is on the bus.
    Bus,
    /// An instance of this process, on its r2r node: its module is not on
    /// the bus, or the endpoint stays on r2r.
    R2r,
    /// An instance that does not run in this process.
    Elsewhere,
    /// A node outside the typed modules.
    Outside,
}

/// One endpoint of the world, under its name in this process.
struct End {
    owner: &'static str,
    endpoint: &'static Endpoint,
    side: Side,
}

impl End {
    fn on_bus(&self) -> bool {
        self.side == Side::Bus
    }

    fn profile(&self) -> Profile {
        self.endpoint.profile().expect("a topic endpoint")
    }
}

/// `robot_status, gps (r2r), nav2 (outside)`
fn owners(ends: &[&End]) -> String {
    let mut labels: Vec<String> = ends
        .iter()
        .map(|e| match e.side {
            Side::Bus => e.owner.to_string(),
            Side::R2r => format!("{} (r2r)", e.owner),
            Side::Elsewhere => format!("{} (not here)", e.owner),
            Side::Outside => format!("{} (outside)", e.owner),
        })
        .collect();
    labels.dedup();
    labels.join(", ")
}

/// What the bridge must declare for one name.
#[derive(Default)]
struct Outcome {
    declarations: Vec<Declaration>,
    notes: Vec<String>,
}

/// The declarations for the instances `running` of this process, whose
/// names resolve with `remaps` (README.md, "The policy"):
///
/// - a topic an instance on the bus publishes goes to DDS, with its writers'
///   QoS, whether or not anything outside reads it, so that the DDS graph
///   keeps every topic it has today;
/// - a topic an instance on the bus subscribes to comes from DDS when it has a
///   writer that is not on the bus here (an r2r module, a module that does not
///   run here, a node outside), or no known writer at all;
/// - a service an instance on the bus serves is served on DDS; one it calls
///   whose server is not on the bus here is called on DDS;
/// - a name that would need both directions is an error (docs/BUS_TOPICS.md
///   5.5 d), except where the bus side of it is writers that never publish,
///   or endpoints that are not in production, which are then left out.
pub fn plan(running: &[Running], remaps: &Remaps) -> Result<Plan, PlanError> {
    let mut problems = Vec::new();
    // (is a service, name) -> every endpoint of that name
    let mut names: BTreeMap<(bool, String), Vec<End>> = BTreeMap::new();
    for module in MODULES {
        for instance in module.instances {
            let here = running.iter().find(|r| r.instance.node == instance.node);
            for endpoint in instance.endpoints {
                let (name, side) = match here {
                    Some(r) => match remaps.resolve(endpoint.kind.name_kind(), &r.node, "", endpoint.code) {
                        Ok(name) if r.on_bus && !endpoint.stays_on_r2r => (name, Side::Bus),
                        Ok(name) => (name, Side::R2r),
                        Err(e) => {
                            problems.push(format!("{} {} {}: {e}", r.node, endpoint.kind.verb(), endpoint.code));
                            continue;
                        }
                    },
                    None => (endpoint.name.to_string(), Side::Elsewhere),
                };
                let end = End { owner: instance.node, endpoint, side };
                names.entry((!endpoint.kind.is_topic(), name)).or_default().push(end);
            }
        }
    }
    for outside in OUTSIDE {
        for endpoint in outside.endpoints {
            let end = End { owner: outside.node, endpoint, side: Side::Outside };
            names.entry((!endpoint.kind.is_topic(), endpoint.name.to_string())).or_default().push(end);
        }
    }

    let mut plan = Plan::default();
    for ((service, name), ends) in &names {
        let ends: Vec<&End> = ends.iter().collect();
        let outcome = match same_type(name, &ends) {
            Err(problem) => Err(problem),
            Ok(()) if *service => service_outcome(name, &ends),
            Ok(()) => topic_outcome(name, &ends),
        };
        match outcome {
            Ok(outcome) => {
                plan.declarations.extend(outcome.declarations);
                plan.notes.extend(outcome.notes);
            }
            Err(problem) => problems.push(problem),
        }
    }
    if problems.is_empty() {
        Ok(plan)
    } else {
        Err(PlanError { problems })
    }
}

/// One name has one type on the bus (mower_bus README, "Type checking").
fn same_type(name: &str, ends: &[&End]) -> Result<(), String> {
    let first = &ends[0].endpoint.iface;
    let same = ends.iter().all(|e| match (&e.endpoint.iface, first) {
        (Iface::Topic(a, _), Iface::Topic(b, _)) => a == b,
        (Iface::Service(a), Iface::Service(b)) => a == b,
        _ => false,
    });
    if same {
        Ok(())
    } else {
        let types: Vec<String> = ends.iter().map(|e| format!("{} {}", e.owner, e.endpoint.ros_type())).collect();
        Err(format!("{name}: the manifest gives it more than one type ({})", types.join(", ")))
    }
}

fn topic_outcome(name: &str, ends: &[&End]) -> Result<Outcome, String> {
    match topic_directions(name, ends) {
        Ok(outcome) => Ok(outcome),
        Err(conflict) => {
            // Without the endpoints production does not have: they are
            // bridged only where that costs no second direction.
            let production: Vec<&End> = ends.iter().copied().filter(|e| e.endpoint.only.is_none()).collect();
            if production.len() == ends.len() {
                return Err(conflict);
            }
            let mut outcome = topic_directions(name, &production).map_err(|_| conflict)?;
            for e in ends.iter().filter(|e| e.endpoint.only.is_some() && e.on_bus()) {
                outcome.notes.push(format!(
                    "{name}: not bridged for {}, which {} it only with {}: that would need both directions",
                    e.owner,
                    e.endpoint.kind.verb(),
                    e.endpoint.only.unwrap_or_default()
                ));
            }
            Ok(outcome)
        }
    }
}

fn topic_directions(name: &str, ends: &[&End]) -> Result<Outcome, String> {
    let of = |kind: Kind, bus: bool| -> Vec<&End> {
        ends.iter().copied().filter(|e| e.endpoint.kind == kind && e.on_bus() == bus).collect()
    };
    let bus_writers = of(Kind::Publish, true);
    let bus_readers = of(Kind::Subscribe, true);
    // A writer that never publishes has nothing to forward to the bus.
    let dds_writers: Vec<&End> = of(Kind::Publish, false).into_iter().filter(|e| !e.endpoint.silent).collect();
    let live_bus_writer = bus_writers.iter().any(|e| !e.endpoint.silent);
    let mut to_ros = !bus_writers.is_empty();
    // No live writer on the bus and none known elsewhere: whoever writes it
    // is on DDS (a tool, a node the manifest does not know).
    let from_ros = !bus_readers.is_empty() && (!dds_writers.is_empty() || !live_bus_writer);
    let mut outcome = Outcome::default();
    if to_ros && from_ros {
        if live_bus_writer {
            return Err(format!(
                "{name}: written on the bus by {} and on DDS by {}, and read on the bus by {}: that needs both \
                 directions, and the bridge's DDS reader would take back its own samples (docs/BUS_TOPICS.md 5.5 d)",
                owners(&bus_writers),
                owners(&dds_writers),
                owners(&bus_readers)
            ));
        }
        to_ros = false;
        outcome.notes.push(format!("{name}: only DDS -> bus: its bus writers ({}) never publish", owners(&bus_writers)));
    }
    let msg_type = match ends[0].endpoint.iface {
        Iface::Topic(t, _) => t,
        Iface::Service(_) => unreachable!("a topic"),
    };
    if to_ros {
        outcome.declarations.push(Declaration {
            direction: Direction::ToRos,
            name: name.to_string(),
            iface: Iface::Topic(msg_type, to_ros_profile(name, &bus_writers)?),
            why: format!("from {}", owners(&bus_writers)),
        });
    }
    if from_ros {
        let sources = if dds_writers.is_empty() { "a writer the manifest does not know".to_string() } else { owners(&dds_writers) };
        outcome.declarations.push(Declaration {
            direction: Direction::FromRos,
            name: name.to_string(),
            iface: Iface::Topic(msg_type, from_ros_profile(name, &dds_writers, &bus_readers)?),
            why: format!("for {}, from {sources}", owners(&bus_readers)),
        });
    }
    Ok(outcome)
}

/// The DDS writer stands in for the bus writers: their profile when they
/// agree (docs/BUS_TOPICS.md 5.5 a, c). Otherwise the deepest history, and
/// reliable if any is: such a writer matches every reader one of them did.
/// One writer cannot keep two durabilities.
fn to_ros_profile(name: &str, writers: &[&End]) -> Result<Profile, String> {
    let profiles: Vec<Profile> = writers.iter().map(|e| e.profile()).collect();
    if profiles.windows(2).all(|w| w[0] == w[1]) {
        return Ok(profiles[0]);
    }
    let qos: Vec<Qos> = profiles.iter().map(|p| p.bus()).collect();
    if qos.windows(2).any(|w| w[0].durability != w[1].durability) {
        return Err(format!("{name}: its bus writers ({}) differ in durability, which one DDS writer cannot be", owners(writers)));
    }
    Ok(Profile::KeepLast(Qos {
        depth: qos.iter().map(|q| q.depth).max().unwrap_or(1),
        reliability: if qos.iter().any(|q| q.reliability == Reliability::Reliable) {
            Reliability::Reliable
        } else {
            Reliability::BestEffort
        },
        durability: qos[0].durability,
    }))
}

/// The DDS reader stands in for the bus readers, so it keeps the pairs DDS
/// has today, (a DDS writer, the module's own r2r reader):
///
/// - best effort if any bus reader is. That matches every writer one of them
///   did, the test harnesses' best-effort writers included
///   (localize_compare.py), and a reliable writer still matches it;
/// - transient local where a bus reader is and every DDS writer is, so that
///   the writers' latched values reach the bus and are retained there for late
///   joiners; volatile otherwise, as the readers were. `/rosout`, which rcl
///   writes `R TL 1000`, comes in volatile for nav's `R V 100`, so a
///   restarted bridge does not replay ten seconds of logs;
/// - the deepest bus reader's depth: `/tf_static`, with two writers
///   (robot_state_publisher's frames and navsat's `utm`), comes in `R TL 100`,
///   so the bus keeps both for a late joiner.
///
/// Writers that differ in durability while a reader wants history cannot be
/// matched by one reader that does what each reader did.
fn from_ros_profile(name: &str, writers: &[&End], readers: &[&End]) -> Result<Profile, String> {
    let latched = |q: &Qos| q.durability == Durability::TransientLocal;
    let src: Vec<Qos> = writers.iter().map(|e| e.profile().bus()).collect();
    let dst: Vec<Qos> = readers.iter().map(|e| e.profile().bus()).collect();
    let wants_history = dst.iter().any(latched);
    if wants_history && src.iter().any(latched) && !src.iter().all(latched) {
        return Err(format!(
            "{name}: its DDS writers ({}) differ in durability, and {} wants history: no one DDS reader serves both",
            owners(writers),
            owners(readers)
        ));
    }
    Ok(Profile::KeepLast(Qos {
        depth: dst.iter().map(|q| q.depth).max().unwrap_or(1),
        reliability: if dst.iter().any(|q| q.reliability == Reliability::BestEffort) {
            Reliability::BestEffort
        } else {
            Reliability::Reliable
        },
        durability: if wants_history && src.iter().all(latched) { Durability::TransientLocal } else { Durability::Volatile },
    }))
}

fn service_outcome(name: &str, ends: &[&End]) -> Result<Outcome, String> {
    let of = |kind: Kind, bus: bool| -> Vec<&End> {
        ends.iter().copied().filter(|e| e.endpoint.kind == kind && e.on_bus() == bus).collect()
    };
    let bus_servers = of(Kind::Serve, true);
    let other_servers = of(Kind::Serve, false);
    let bus_callers = of(Kind::Call, true);
    let to_ros = !bus_servers.is_empty();
    let from_ros = !bus_callers.is_empty() && (!other_servers.is_empty() || bus_servers.is_empty());
    if to_ros && from_ros {
        return Err(format!(
            "{name}: served on the bus by {} and also by {}, and called on the bus by {}: both directions \
             (docs/BUS_TOPICS.md 5.5 d)",
            owners(&bus_servers),
            owners(&other_servers),
            owners(&bus_callers)
        ));
    }
    let srv_type = match ends[0].endpoint.iface {
        Iface::Service(s) => s,
        Iface::Topic(..) => unreachable!("a service"),
    };
    let mut outcome = Outcome::default();
    if to_ros {
        outcome.declarations.push(Declaration {
            direction: Direction::ServiceToRos,
            name: name.to_string(),
            iface: Iface::Service(srv_type),
            why: format!("served by {}", owners(&bus_servers)),
        });
    }
    if from_ros {
        let server = if other_servers.is_empty() { "a server the manifest does not know".to_string() } else { owners(&other_servers) };
        outcome.declarations.push(Declaration {
            direction: Direction::ServiceFromRos,
            name: name.to_string(),
            iface: Iface::Service(srv_type),
            why: format!("for {}, served by {server}", owners(&bus_callers)),
        });
    }
    Ok(outcome)
}
