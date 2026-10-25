//! The in-process message bus of the mower daemon (docs/ROS_FREE_PLAN.md,
//! step E1): typed topics, services, type-erased (JSON) access and a clock
//! that tests and replay can drive. README.md has the model and the semantics
//! every part keeps; this file holds the two handles everything hangs off,
//! [`Bus`] and [`Scope`].
//!
//! No r2r, no ROS: what crosses to DDS goes through `mower_bus_ros`.

use std::fmt;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use serde::de::DeserializeOwned;
use serde::Serialize;

mod clock;
mod codec;
mod erased;
mod error;
mod introspect;
mod qos;
mod service;
mod topic;

pub use clock::{Clock, Elapsed, Interval, ManualClock, MissedTick, Mono, Sleep, Time, Timeout};
pub use erased::{ErasedMsg, ErasedSubscriber, JsonPublisher};
pub use error::{BusError, RecvError};
pub use introspect::{ros_type_name, ServiceInfo, TopicInfo};
pub use qos::{Durability, Qos, Reliability};
pub use service::{Client, Request, ResponseFuture, Service};
pub use topic::{Publisher, Subscriber};

/// What can travel on the bus. Blanket-implemented, so r2r's generated
/// message types (which derive `Clone, Serialize, Deserialize`) qualify as
/// they are, and so will ROS-free mirrors of them.
pub trait Message: Clone + Send + Sync + Serialize + DeserializeOwned + 'static {}
impl<T> Message for T where T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static {}

/// The process-wide registry of topics and services. Cheap to clone; every
/// clone is the same bus.
#[derive(Clone, Default)]
pub struct Bus {
    inner: Arc<BusInner>,
}

#[derive(Default)]
struct BusInner {
    topics: topic::Registry,
    services: service::Registry,
    next_scope: AtomicU64,
}

impl Bus {
    pub fn new() -> Self {
        Self::default()
    }

    /// A new, open scope. `name` is the module's node name; it only labels
    /// the scope in introspection, so it need not be unique (a restarted
    /// module gets a fresh scope with the same name).
    pub fn scope(&self, name: &str) -> Scope {
        let id = self.inner.next_scope.fetch_add(1, Ordering::Relaxed) + 1;
        Scope {
            inner: Arc::new(ScopeInner {
                bus: self.clone(),
                name: name.to_string(),
                id,
                closed: AtomicBool::new(false),
                hooks: Mutex::new(Vec::new()),
            }),
        }
    }

    /// Every topic the bus has seen, sorted by name.
    pub fn topics(&self) -> Vec<TopicInfo> {
        self.inner.topics.info()
    }

    /// Every service name the bus has seen, sorted by name.
    pub fn services(&self) -> Vec<ServiceInfo> {
        self.inner.services.info()
    }

    pub(crate) fn topic_registry(&self) -> &topic::Registry {
        &self.inner.topics
    }

    pub(crate) fn service_registry(&self) -> &service::Registry {
        &self.inner.services
    }
}

impl fmt::Debug for Bus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Bus").finish_non_exhaustive()
    }
}

/// One module instance's view of the bus: what an `r2r::Node` is today.
///
/// Every endpoint is created through a scope, and [`Scope::close`] ends all
/// of them at once (README.md, "Model"). Cheap to clone; clones are the same
/// scope.
///
/// The registries must never hold a `Scope` (that would be a reference
/// cycle through [`Bus`]); they keep `Scope::id` and [`Scope::name`].
#[derive(Clone)]
pub struct Scope {
    inner: Arc<ScopeInner>,
}

struct ScopeInner {
    bus: Bus,
    name: String,
    id: u64,
    closed: AtomicBool,
    hooks: Mutex<Vec<Box<dyn FnOnce() + Send>>>,
}

impl Scope {
    pub fn name(&self) -> &str {
        &self.inner.name
    }

    pub fn bus(&self) -> &Bus {
        &self.inner.bus
    }

    pub fn is_closed(&self) -> bool {
        self.inner.closed.load(Ordering::Acquire)
    }

    /// End everything created through this scope (idempotent): subscriber
    /// streams return `None`, publishes fail with [`BusError::Closed`], the
    /// scope's latched history is dropped, its services go away and the
    /// calls waiting on them fail. Creating an endpoint on a closed scope
    /// fails with [`BusError::Closed`].
    pub fn close(&self) {
        if self.inner.closed.swap(true, Ordering::AcqRel) {
            return;
        }
        let hooks = std::mem::take(&mut *self.inner.hooks.lock().unwrap_or_else(|e| e.into_inner()));
        for hook in hooks {
            hook();
        }
    }

    /// Unique per scope for the life of the bus.
    pub(crate) fn id(&self) -> u64 {
        self.inner.id
    }

    /// Run `hook` once when the scope closes, outside any registry lock and
    /// in registration order. Fails (dropping the hook unrun) when the scope
    /// is already closed, which is how endpoint creation learns it must fail
    /// with `Closed`. Endpoints are created at module start, so hooks of
    /// endpoints dropped early simply wait for the close (keep them `Weak`).
    pub(crate) fn on_close(&self, hook: Box<dyn FnOnce() + Send>) -> Result<(), BusError> {
        let mut hooks = self.inner.hooks.lock().unwrap_or_else(|e| e.into_inner());
        if self.is_closed() {
            return Err(BusError::Closed { scope: self.inner.name.clone() });
        }
        hooks.push(hook);
        Ok(())
    }

    /// A writer on topic `name` (README.md, "Topics").
    pub fn publisher<T: Message>(&self, name: &str, qos: Qos) -> Result<Publisher<T>, BusError> {
        topic::publisher(self, name, qos)
    }

    /// A reader of topic `name` (README.md, "Topics").
    pub fn subscribe<T: Message>(&self, name: &str, qos: Qos) -> Result<Subscriber<T>, BusError> {
        topic::subscribe(self, name, qos)
    }

    /// A reader of topic `name` that does not know its type (README.md,
    /// "Erased access"). Works before any typed endpoint has fixed the type.
    pub fn subscribe_erased(&self, name: &str, qos: Qos) -> Result<ErasedSubscriber, BusError> {
        erased::subscribe(self, name, qos)
    }

    /// A writer on topic `name` that takes JSON. Creating it works before the
    /// type is known (it still fails on an invalid name or a closed scope);
    /// each publish decodes with the channel's codec and fails with
    /// [`BusError::UnknownName`] while no typed endpoint has fixed the type.
    pub fn publisher_json(&self, name: &str, qos: Qos) -> Result<JsonPublisher, BusError> {
        erased::publisher(self, name, qos)
    }

    /// The server side of service `name` (README.md, "Services").
    pub fn service<Req: Message, Resp: Message>(&self, name: &str) -> Result<Service<Req, Resp>, BusError> {
        service::serve(self, name)
    }

    /// A client of service `name` (README.md, "Services").
    pub fn client<Req: Message, Resp: Message>(&self, name: &str) -> Result<Client<Req, Resp>, BusError> {
        service::client(self, name)
    }

    /// Call service `name` with a JSON request, using the server's codec.
    pub async fn call_json(&self, name: &str, request: serde_json::Value) -> Result<serde_json::Value, BusError> {
        service::call_json(self, name, request).await
    }
}

impl fmt::Debug for Scope {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Scope")
            .field("name", &self.inner.name)
            .field("id", &self.inner.id)
            .field("closed", &self.is_closed())
            .finish()
    }
}

/// An absolute ROS name: `/` then one or more `/`-separated tokens of
/// `[A-Za-z0-9_]`, none empty and none starting with a digit. Relative and
/// private (`~`) names are the caller's to resolve (README.md, "Model").
pub(crate) fn check_name(name: &str) -> Result<(), BusError> {
    let invalid = |reason| Err(BusError::InvalidName { name: name.to_string(), reason });
    let Some(rest) = name.strip_prefix('/') else {
        return invalid("not absolute (resolve remappings and namespaces first)");
    };
    if rest.is_empty() {
        return invalid("empty");
    }
    for token in rest.split('/') {
        let Some(first) = token.chars().next() else {
            return invalid("empty token");
        };
        if first.is_ascii_digit() {
            return invalid("a token starts with a digit");
        }
        if !token.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') {
            return invalid("characters other than [A-Za-z0-9_/]");
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names_are_absolute_ros_names() {
        for ok in ["/odom", "/robot/info", "/map_manage/get_parameters", "/a_1/b2"] {
            assert_eq!(check_name(ok), Ok(()), "{ok}");
        }
        for bad in ["odom", "", "/", "//odom", "/odom/", "/robot//info", "/1abc", "/a-b", "~/x", "/a b"] {
            assert!(matches!(check_name(bad), Err(BusError::InvalidName { .. })), "{bad}");
        }
    }

    #[test]
    fn closing_a_scope_runs_its_hooks_once_and_refuses_new_ones() {
        let bus = Bus::new();
        let scope = bus.scope("m");
        let ran = Arc::new(AtomicU64::new(0));
        for _ in 0..2 {
            let ran = ran.clone();
            scope.on_close(Box::new(move || { ran.fetch_add(1, Ordering::SeqCst); })).unwrap();
        }
        assert!(!scope.is_closed());
        scope.clone().close();
        scope.close();
        assert!(scope.is_closed());
        assert_eq!(ran.load(Ordering::SeqCst), 2);
        let late = scope.on_close(Box::new(|| panic!("must not run")));
        assert_eq!(late, Err(BusError::Closed { scope: "m".into() }));
        // a fresh scope with the same name is independent
        let again = bus.scope("m");
        assert!(!again.is_closed());
        assert_ne!(again.id(), scope.id());
    }
}
