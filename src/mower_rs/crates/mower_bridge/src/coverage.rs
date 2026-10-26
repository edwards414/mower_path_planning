//! The manifest check (README.md, "The check"): every endpoint an instance
//! creates on the bus must be in the manifest, with the same type and QoS.
//! The policy plans from the manifest, so an endpoint the manifest lacks is a
//! name nothing bridges, and without the check nothing would say so.

use std::collections::BTreeSet;
use std::sync::Arc;
use std::time::Duration;

use mower_bus::{Bus, Qos};
use mower_rs_common::{ModuleCtx, Remaps};

use crate::policy::Running;
use crate::types::{Iface, Kind, Profile, ShowQos};

/// When the first check runs: by then the instances have made their
/// endpoints (pid_autotune's telemetry reader, made per tuning session, is
/// seen by a later one).
const FIRST_CHECK: Duration = Duration::from_secs(2);
const CHECK_PERIOD: Duration = Duration::from_secs(10);

/// Whose a scope on the bus is.
enum Owner<'a> {
    /// The bridge's own, or a test's.
    Skip,
    Instance(&'a Running),
    /// No instance of the manifest.
    Unknown,
}

/// A bus endpoint as introspection shows it.
struct Seen<'a> {
    scope: &'a str,
    kind: Kind,
    name: &'a str,
    /// The message's or the request's Rust type; `None` on a name no typed
    /// endpoint has fixed yet.
    rust_type: Option<&'static str>,
    response_type: Option<&'static str>,
    qos: Option<Qos>,
}

impl Seen<'_> {
    fn gap(&self, problem: &str) -> String {
        let ty = match self.rust_type {
            Some(rust) => mower_bus::ros_type_name(rust).unwrap_or_else(|| rust.to_string()),
            None => "untyped".to_string(),
        };
        let qos = self.qos.map(|q| format!(", {}", ShowQos(q))).unwrap_or_default();
        format!("{} {} {} ({ty}{qos}) on the bus, but {problem}", self.scope, self.kind.verb(), self.name)
    }
}

/// The endpoints the instances `running` made on `bus` that the manifest
/// does not have with the same type and QoS, one line each, sorted. The
/// endpoints of `bridge_scopes` are the bridge's own. Each line names the
/// scope, the endpoint and what is wrong: no such endpoint in the manifest,
/// another type or QoS there, an endpoint the manifest keeps on r2r, a module
/// whose flag is still false, or a scope that is no instance at all.
pub fn gaps(bus: &Bus, running: &[Running], remaps: &Remaps, bridge_scopes: &[&str]) -> Vec<String> {
    check_bus(bus, remaps, &|scope| {
        if bridge_scopes.contains(&scope) {
            Owner::Skip
        } else if let Some(r) = running.iter().find(|r| r.node == scope) {
            Owner::Instance(r)
        } else {
            Owner::Unknown
        }
    })
}

/// [`gaps`] two seconds after the start and every ten from then on, for as
/// long as the process runs; `report` gets each gap once.
pub async fn watch(
    bus: Bus, running: Vec<Running>, remaps: Arc<Remaps>, bridge_scopes: Vec<String>,
    report: impl Fn(&str) + Send + 'static,
) {
    let scopes: Vec<&str> = bridge_scopes.iter().map(String::as_str).collect();
    let mut reported = BTreeSet::new();
    tokio::time::sleep(FIRST_CHECK).await;
    loop {
        for gap in gaps(&bus, &running, &remaps, &scopes) {
            if reported.insert(gap.clone()) {
                report(&gap);
            }
        }
        tokio::time::sleep(CHECK_PERIOD).await;
    }
}

/// For a module's own tests: what [`gaps`] says about the endpoints the
/// module made on its scope `m.bus` (endpoints of other scopes, the test's
/// own, are not looked at). The instance is the manifest's of the node name
/// rcl would give `m.node_name` with `m.remaps`, and counts as on the bus.
pub fn gaps_of(m: &ModuleCtx) -> Vec<String> {
    let node = m.remaps.node_name(&m.node_name);
    let Some((module, instance)) = crate::instance(&node) else {
        return vec![format!("{node} is no instance of the manifest (crates/mower_bridge/src/manifest.rs)")];
    };
    let running = Running { module, instance, node, on_bus: true };
    let scope = m.bus.name();
    check_bus(m.bus.bus(), &m.remaps, &|s| if s == scope { Owner::Instance(&running) } else { Owner::Skip })
}

/// [`gaps_of`], for a test: "this instance's bus endpoints are in the
/// manifest".
#[track_caller]
pub fn assert_in_manifest(m: &ModuleCtx) {
    let gaps = gaps_of(m);
    assert!(gaps.is_empty(), "endpoints the bridge would not carry:\n{}", gaps.join("\n"));
}

fn check_bus<'a>(bus: &Bus, remaps: &Remaps, owner: &dyn Fn(&str) -> Owner<'a>) -> Vec<String> {
    let mut gaps = Vec::new();
    for topic in bus.topics() {
        let writers = topic.publishers.iter().zip(&topic.publisher_qos).map(|(s, q)| (Kind::Publish, s, *q));
        let readers = topic.subscribers.iter().zip(&topic.subscriber_qos).map(|(s, q)| (Kind::Subscribe, s, *q));
        for (kind, scope, qos) in writers.chain(readers) {
            let seen = Seen { scope, kind, name: &topic.name, rust_type: topic.type_name, response_type: None, qos: Some(qos) };
            gaps.extend(check(&seen, remaps, owner));
        }
    }
    for service in bus.services() {
        let server = service.server.iter().map(|s| (Kind::Serve, s));
        let clients = service.clients.iter().map(|s| (Kind::Call, s));
        for (kind, scope) in server.chain(clients) {
            let seen = Seen {
                scope,
                kind,
                name: &service.name,
                rust_type: Some(service.request_type),
                response_type: Some(service.response_type),
                qos: None,
            };
            gaps.extend(check(&seen, remaps, owner));
        }
    }
    gaps.sort();
    gaps.dedup();
    gaps
}

fn check<'a>(seen: &Seen<'_>, remaps: &Remaps, owner: &dyn Fn(&str) -> Owner<'a>) -> Option<String> {
    let running = match owner(seen.scope) {
        Owner::Skip => return None,
        Owner::Unknown => return Some(seen.gap("its scope is no instance of the manifest, so nothing bridges it")),
        Owner::Instance(r) => r,
    };
    if !running.on_bus {
        return Some(seen.gap(&format!(
            "module {} is not on the bus (crates/mower_bridge/src/on_bus.rs), so nothing bridges it",
            running.module.id
        )));
    }
    let endpoint = running.instance.endpoints.iter().find(|e| {
        e.kind == seen.kind
            && remaps.resolve(e.kind.name_kind(), &running.node, "", e.code).is_ok_and(|name| name == seen.name)
    });
    let Some(endpoint) = endpoint else {
        return Some(seen.gap(&format!("the manifest has no such endpoint for {}, so nothing bridges it", running.node)));
    };
    if endpoint.stays_on_r2r {
        return Some(seen.gap("the manifest keeps it on the module's r2r node"));
    }
    match endpoint.iface {
        Iface::Topic(t, profile) => {
            if seen.rust_type != Some(t.rust_type()) {
                return Some(seen.gap(&format!("the manifest has it as {}", t.ros_type())));
            }
            if seen.qos != Some(profile.bus()) {
                let manifest = match profile {
                    Profile::SystemDefault => format!("{profile} ({})", ShowQos(profile.bus())),
                    Profile::KeepLast(_) => profile.to_string(),
                };
                return Some(seen.gap(&format!("the manifest has it as {manifest}")));
            }
        }
        Iface::Service(s) => {
            if seen.rust_type != Some(s.request_type()) || seen.response_type != Some(s.response_type()) {
                return Some(seen.gap(&format!("the manifest has it as {}", s.ros_type())));
            }
        }
    }
    None
}
