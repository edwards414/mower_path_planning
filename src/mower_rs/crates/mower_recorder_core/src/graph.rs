//! `/graph_snapshot` and `/graph_events` (mower_recorder
//! `graph_snapshot_node.py`): the JSON of each 2 s tick and the node diff
//! between ticks, with the watch rule kept as it is (the launch passes the
//! names without their leading slash, so it never fires: design 1.12 #1).

use std::collections::BTreeSet;

use mower_pyfmt::json::{self, JsonOptions};
use mower_pyfmt::Py;

/// `_full_name(name, namespace)`
pub fn full_name(name: &str, namespace: &str) -> String {
    if namespace == "/" {
        format!("/{name}")
    } else {
        format!("{}/{name}", namespace.trim_end_matches('/'))
    }
}

pub struct Topic {
    pub name: String,
    pub types: Vec<String>,
    pub pubs: i64,
    pub subs: i64,
}

pub struct Service {
    pub name: String,
    pub types: Vec<String>,
}

fn strs(v: &[String]) -> Py {
    Py::List(v.iter().map(|s| Py::str(s)).collect())
}

/// One tick's `/graph_snapshot` (`json.dumps(snap, ensure_ascii=False)`):
/// the nodes sorted, topics and services in the order the graph gives them.
pub fn snapshot_json(sec: i32, nanosec: u32, nodes: &BTreeSet<String>, topics: &[Topic], services: &[Service]) -> String {
    let snap = Py::dict([
        ("stamp", Py::dict([("sec", Py::Int(sec as i64)), ("nanosec", Py::Int(nanosec as i64))])),
        ("nodes", Py::List(nodes.iter().map(|n| Py::str(n)).collect())),
        (
            "topics",
            Py::List(
                topics
                    .iter()
                    .map(|t| {
                        Py::dict([("name", Py::str(&t.name)), ("types", strs(&t.types)), ("pubs", Py::Int(t.pubs)), ("subs", Py::Int(t.subs))])
                    })
                    .collect(),
            ),
        ),
        ("services", Py::List(services.iter().map(|s| Py::dict([("name", Py::str(&s.name)), ("types", strs(&s.types))])).collect())),
    ]);
    json::dumps(&snap, &JsonOptions::unicode()).expect("no dates")
}

/// A `/graph_events` message (`json.dumps(..., ensure_ascii=False)`).
pub fn event_json(kind: &str, target: &str) -> String {
    json::dumps(&Py::dict([("event", Py::str(kind)), ("target", Py::str(target))]), &JsonOptions::unicode()).expect("no dates")
}

/// What a tick found.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Event {
    /// Logged at WARN as `NODE DISAPPEARED: <node>`.
    Disappeared(String),
    Appeared(String),
    /// A watched node left: `WATCHED NODE DOWN: <node> -> fault`, and
    /// `Bool(true)` on the fault topic.
    Fault(String),
}

/// The node diff between ticks (`_tick`'s second half).
pub struct Watch {
    prev: BTreeSet<String>,
    watch: BTreeSet<String>,
}

impl Watch {
    /// `watch_nodes`, empty names dropped.
    pub fn new(watch_nodes: &[String]) -> Watch {
        Watch { prev: BTreeSet::new(), watch: watch_nodes.iter().filter(|w| !w.is_empty()).cloned().collect() }
    }

    /// The events of a tick, in the Python's order: every node gone (sorted,
    /// each followed by its fault if watched), then every node new. The
    /// first tick (no previous set) has none.
    pub fn tick(&mut self, nodes: &BTreeSet<String>) -> Vec<Event> {
        let mut ev = Vec::new();
        if !self.prev.is_empty() {
            for gone in self.prev.difference(nodes) {
                ev.push(Event::Disappeared(gone.clone()));
                if self.watch.contains(gone) {
                    ev.push(Event::Fault(gone.clone()));
                }
            }
            for new in nodes.difference(&self.prev) {
                ev.push(Event::Appeared(new.clone()));
            }
        }
        self.prev = nodes.clone();
        ev
    }
}
