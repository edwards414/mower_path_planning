//! The in-process message bus of the mower daemon (docs/ROS_FREE_PLAN.md,
//! step E1): typed topics, services, type-erased (JSON) access and a clock
//! that tests and replay can drive. README.md has the model and the semantics
//! every part keeps; this file holds the two handles everything hangs off,
//! [`Bus`] and [`Scope`].
//!
//! No r2r, no ROS: what crosses to DDS goes through `mower_bus_ros`.

use std::any::Any;
use std::fmt;
use std::panic::{catch_unwind, resume_unwind, AssertUnwindSafe};
use std::sync::atomic::{AtomicU64, AtomicU8, Ordering};
use std::sync::{Arc, Condvar, Mutex, MutexGuard, PoisonError};
use std::thread::ThreadId;
use std::time::Duration;

use serde::de::DeserializeOwned;
use serde::Serialize;
use tokio::sync::Notify;

mod clock;
mod codec;
mod erased;
mod error;
mod introspect;
mod qos;
mod service;
mod timer;
mod topic;

pub use clock::{Clock, Elapsed, Interval, ManualClock, MissedTick, Mono, Sleep, Time, Timeout};
pub use erased::{ErasedMsg, ErasedSubscriber, JsonPublisher};
pub use error::{BusError, RecvError};
pub use introspect::{ros_type_name, ServiceInfo, TopicInfo};
pub use qos::{Durability, Qos, Reliability};
pub use service::{Client, Request, ResponseFuture, Service};
pub use timer::Timer;
pub use topic::{Publisher, PublisherChanges, Subscriber};

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
                state: AtomicU8::new(State::Open as u8),
                life: Mutex::new(Life { hooks: Vec::new(), children: Vec::new(), closer: None }),
                finished: Condvar::new(),
                closed: Notify::new(),
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
    /// A `State`. It only moves forward and is written only with `life`
    /// locked (`set_state` takes the guard), so a read under that lock is
    /// exact and one without it is a state the scope has reached:
    /// `is_closed` and a parent pruning its children take no lock for it.
    state: AtomicU8,
    life: Mutex<Life>,
    /// Signalled when a close finishes, for a `close` that waits for the one
    /// under way.
    finished: Condvar,
    /// Woken when a close finishes: `Scope::closed`, and through it timers.
    closed: Notify,
}

/// What closing a scope has to end. No user code ever runs under this
/// lock: hooks run, and refused hooks drop, after it is released.
struct Life {
    /// Run once by the close, in registration order, after the children.
    hooks: Vec<Hook>,
    /// Closed by the close, before the hooks. Held strongly: a child whose
    /// handles are all gone may still have endpoints in tasks it left
    /// running, and only its parent can still reach it to end them. A child
    /// holds nothing of its parent, so there is no cycle. The children whose
    /// close has finished are let go when a child is added.
    children: Vec<Scope>,
    /// The thread running the close, while it is closing: a close re-entered
    /// on that thread must not wait for itself.
    closer: Option<ThreadId>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
enum State {
    Open,
    /// A `close` is ending the children and running the hooks. Nothing new
    /// can attach any more.
    Closing,
    /// Everything has ended.
    Closed,
}

impl ScopeInner {
    fn state(&self) -> State {
        match self.state.load(Ordering::Acquire) {
            0 => State::Open,
            1 => State::Closing,
            _ => State::Closed,
        }
    }

    /// Move to `state`, with `life` locked (the guard is the proof).
    fn set_state(&self, _locked: &mut Life, state: State) {
        self.state.store(state as u8, Ordering::Release);
    }
}

/// What a scope's close runs for one endpoint (see `Scope::on_close`).
pub(crate) type Hook = Box<dyn FnOnce() + Send>;

/// A poisoned lock still holds consistent data: no scope lock is held while
/// user code runs.
fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(PoisonError::into_inner)
}

impl Scope {
    pub fn name(&self) -> &str {
        &self.inner.name
    }

    pub fn bus(&self) -> &Bus {
        &self.inner.bus
    }

    /// Whether the scope is closed or being closed: true from the moment a
    /// [`Scope::close`] starts, and never false again.
    ///
    /// Once it is true, nothing new can be made through the scope (endpoints
    /// and timers fail with [`BusError::Closed`], children come out closed)
    /// and its timers' ticks fail. Endpoints made earlier may still be ending
    /// while the close runs: that they all have ended is what `close()`
    /// returning, or [`Scope::closed`] resolving, says. Takes no lock, so a
    /// dedicated std thread can check it in every turn of its loop.
    pub fn is_closed(&self) -> bool {
        self.inner.state() != State::Open
    }

    /// End everything created through this scope (idempotent): subscriber
    /// streams return `None`, publishes fail with [`BusError::Closed`], the
    /// scope's latched history is dropped, its services go away and the
    /// calls waiting on them fail, its timers' ticks fail and its child
    /// scopes close. Creating an endpoint on a closed scope fails with
    /// [`BusError::Closed`].
    ///
    /// Returns once all of that is done. A close that finds another one
    /// under way on another thread waits for it to finish, so returning
    /// always means every endpoint of the scope has ended.
    ///
    /// The one exception is a close re-entered on the thread running the
    /// close: drop glue or a waker that the close runs (see below) and that
    /// closes this scope again, or its parent. That close returns at once,
    /// before the scope has finished closing, and the close under way
    /// finishes the job. Closing scopes from such user code across threads
    /// is not supported: thread A closing a child whose drop glue closes its
    /// parent, while thread B closes that parent and so waits for the child,
    /// deadlocks.
    ///
    /// # Panics
    ///
    /// A close frees messages and wakes tasks, which runs user code (drop
    /// glue, wakers). If some of it panics, the close still ends everything
    /// else, finishes, and wakes [`Scope::closed`]; it then re-raises the
    /// first panic. A close that only waited for another one does not.
    pub fn close(&self) {
        let inner = &*self.inner;
        let me = std::thread::current().id();
        let (children, hooks) = {
            let mut life = lock(&inner.life);
            match inner.state() {
                State::Open => {}
                // Re-entered from user code the close under way runs on this
                // very thread: waiting would be waiting for itself.
                State::Closing if life.closer == Some(me) => return,
                State::Closing => {
                    let life = inner.finished.wait_while(life, |_| inner.state() != State::Closed);
                    drop(life.unwrap_or_else(PoisonError::into_inner));
                    return;
                }
                State::Closed => return,
            }
            inner.set_state(&mut life, State::Closing);
            life.closer = Some(me);
            (std::mem::take(&mut life.children), std::mem::take(&mut life.hooks))
        };
        // Finishes the close when dropped, so that it finishes even if
        // something unwinds past the `catch_unwind`s below.
        let finish = FinishClose(inner);
        let mut first_panic: Option<Box<dyn Any + Send>> = None;
        let mut contain = |ran: std::thread::Result<()>| {
            if let Err(panic) = ran {
                first_panic.get_or_insert(panic);
            }
        };
        for child in children {
            contain(catch_unwind(AssertUnwindSafe(|| child.close())));
        }
        for hook in hooks {
            contain(catch_unwind(AssertUnwindSafe(hook)));
        }
        drop(finish);
        if let Some(panic) = first_panic {
            resume_unwind(panic);
        }
    }

    /// Resolves once the scope is closed: once a [`Scope::close`] has
    /// finished, so that every endpoint of the scope has ended by then. At
    /// once if that has happened already.
    ///
    /// Needs no runtime. Dropped before it resolves (the losing branch of a
    /// `select!`), it leaves nothing behind, so a loop can poll a fresh one in
    /// every turn.
    pub async fn closed(&self) {
        let mut notified = std::pin::pin!(self.inner.closed.notified());
        // On the waiting list before the look, so that a close finishing in
        // between still wakes it.
        notified.as_mut().enable();
        if self.inner.state() == State::Closed {
            return;
        }
        notified.await;
    }

    /// A new scope on the same bus that closes when this one closes, named
    /// `name` (its node name). If this scope is closed or closing, the child
    /// is created closed.
    ///
    /// For a module that runs several nodes (`mower_localize::run_all`) or
    /// opens endpoints per session (ws_bridge in E3): whoever closes the
    /// module's scope ends them all, also those whose handles were all
    /// dropped while their endpoints live on in tasks. A child can be closed
    /// on its own, before its parent. The parent holds it until its close
    /// has finished; a child holds nothing of its parent.
    pub fn child(&self, name: &str) -> Scope {
        let child = self.inner.bus.scope(name);
        let mut life = lock(&self.inner.life);
        if self.inner.state() != State::Open {
            drop(life);
            child.close(); // it has nothing to end yet
            return child;
        }
        // Let go of the children whose close has finished. One still closing
        // on another thread stays: this scope's close must wait for it.
        let finished: Vec<Scope> = life.children.extract_if(.., |child| child.inner.state() == State::Closed).collect();
        life.children.push(child.clone());
        drop(life);
        // Possibly the last handles: freed with no lock held.
        drop(finished);
        child
    }

    /// An rcl / r2r wall timer on `clock`, tied to this scope: its first tick
    /// comes one period after creation, after an overrun it fires once and
    /// keeps its phase (`clock.interval_at(clock.mono() + period, period)`
    /// with [`MissedTick::Skip`]), and its [`Timer::tick`] fails with
    /// [`BusError::Closed`] once the scope closes. `Closed` at once if it is
    /// closed already.
    ///
    /// Panics if `period` is zero, as [`Clock::interval_at`] does.
    pub fn wall_timer(&self, clock: &Clock, period: Duration) -> Result<Timer, BusError> {
        if self.is_closed() {
            return Err(self.closed_error());
        }
        Ok(timer::wall_timer(self.clone(), clock, period))
    }

    /// Unique per scope for the life of the bus.
    pub(crate) fn id(&self) -> u64 {
        self.inner.id
    }

    pub(crate) fn closed_error(&self) -> BusError {
        BusError::Closed { scope: self.inner.name.clone() }
    }

    /// Run `hook` once when the scope closes, outside any lock and in
    /// registration order. Fails (dropping the hook unrun) once a close has
    /// started, which is how endpoint creation learns it must fail with
    /// `Closed`: nothing attaches to a closing scope. Endpoints are mostly
    /// created at module start, so hooks of endpoints dropped early simply
    /// wait for the close (keep them `Weak`; README.md, "Model", has the
    /// exceptions).
    pub(crate) fn on_close(&self, hook: Hook) -> Result<(), BusError> {
        let mut life = lock(&self.inner.life);
        if self.inner.state() != State::Open {
            drop(life);
            // With no lock held: it may own the last reference to what it was
            // to end.
            drop(hook);
            return Err(self.closed_error());
        }
        life.hooks.push(hook);
        Ok(())
    }

    #[cfg(test)]
    pub(crate) fn hook_count(&self) -> usize {
        lock(&self.inner.life).hooks.len()
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

/// Finishes a close when dropped: the scope is closed, and whoever waits for
/// that (a second `close`, `closed()`, timers) is woken.
struct FinishClose<'a>(&'a ScopeInner);

impl Drop for FinishClose<'_> {
    fn drop(&mut self) {
        let mut life = lock(&self.0.life);
        self.0.set_state(&mut life, State::Closed);
        life.closer = None;
        drop(life);
        self.0.finished.notify_all();
        self.0.closed.notify_waiters();
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

    #[test]
    fn a_hook_offered_while_the_scope_closes_is_refused_and_dropped_unrun() {
        let bus = Bus::new();
        let scope = bus.scope("m");
        let late = Arc::new(Mutex::new(None));
        let (s, l) = (scope.clone(), late.clone());
        scope
            .on_close(Box::new(move || {
                // what an endpoint created on another thread meanwhile does
                assert!(s.is_closed(), "closing counts as closed");
                *l.lock().unwrap() = Some(s.on_close(Box::new(|| panic!("must not run"))));
            }))
            .unwrap();
        scope.close();
        assert_eq!(*late.lock().unwrap(), Some(Err(BusError::Closed { scope: "m".into() })));
        assert_eq!(scope.hook_count(), 0);
    }

    #[test]
    fn a_parent_holds_its_children_until_they_have_closed_and_no_child_holds_its_parent() {
        let bus = Bus::new();
        let parent = bus.scope("localize");
        let child = parent.child("ekf_filter_node_odom");
        let grandchild = child.child("tf");
        assert_eq!(Arc::strong_count(&parent.inner), 1, "a child holds nothing of its parent");
        assert_eq!(Arc::strong_count(&child.inner), 2, "the parent holds its child");
        assert_eq!(child.name(), "ekf_filter_node_odom");
        assert!(Arc::ptr_eq(&child.bus().inner, &bus.inner), "on the same bus");

        child.close();
        assert!(grandchild.is_closed());
        let next = parent.child("ekf_filter_node_map");
        assert_eq!(Arc::strong_count(&child.inner), 1, "a closed child is let go when the next one is added");
        drop(parent);
        assert!(!next.is_closed(), "a parent dropped unclosed closes nothing, like any scope");
    }

    #[test]
    fn closed_children_are_let_go_and_dropped_ones_are_kept_until_their_parent_closes() {
        let bus = Bus::new();
        let bridge = bus.scope("ws_bridge");
        let mut leftovers = Vec::new();
        for i in 0..100 {
            let session = bridge.child("session");
            if i % 2 == 0 {
                session.close(); // a session that ended
            } else {
                // one that dropped its handle and left an endpoint in a task
                leftovers.push(session.publisher::<u32>("/app_joy", Qos::default()).unwrap());
            }
        }
        let live = bridge.child("session");
        assert_eq!(lock(&bridge.inner.life).children.len(), 51, "the 50 closed ones went when one was added");
        assert_eq!(bridge.hook_count(), 0, "children take no hook");
        bridge.close();
        assert!(live.is_closed());
        for leftover in &leftovers {
            assert_eq!(leftover.publish(&1), Err(BusError::Closed { scope: "session".into() }));
        }
    }
}
