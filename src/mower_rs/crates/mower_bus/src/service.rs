//! Services: one server per name, requests handed out one at a time,
//! answered through the `Request` itself. Contract: README.md, "Services".
//!
//! Requirements beyond the README:
//! - no async runtime needed (the response future can be awaited on any
//!   executor; `respond` can be called from a std thread);
//! - `Service`, `Client` and `ResponseFuture` are `Send + Unpin`; `Client` is
//!   `Clone`; `ResponseFuture` is `'static` (it can be spawned);
//! - the request's JSON path (`call_json`) uses the codecs the server
//!   registered (`crate::codec::Codec` for the request and response types);
//! - a `Request` dropped unanswered -> the caller gets `NoResponse`; the
//!   server's scope closing (or its `Service` dropping) -> queued and
//!   in-flight callers get `ServiceUnavailable`, and the name is free for a
//!   new server;
//! - closing a client's scope fails its in-flight calls with `Closed`; the
//!   server may still answer, the answer is dropped;
//! - `request` checks for a server at call time: none -> outer `Err(
//!   ServiceUnavailable)` (a ROS 2 request to a missing server is lost
//!   silently and its caller waits forever).
//!
//! How the parts fit:
//! - The registry keeps one `Entry` per name for the bus's lifetime: the
//!   codecs the first endpoint fixed, the live `Server`, the live clients
//!   (for introspection) and the tasks in `wait_for_service`.
//! - A `Server` is what one `Service` serves. Everything in it is erased (an
//!   `AnyMsg` request in, an `AnyMsg` answer out), so a JSON call reaches a
//!   typed server through the same queue; `Request` and `ResponseFuture`
//!   downcast at the two ends. It holds every call it has not answered,
//!   queued or handed out, so that ending it can fail them all.
//! - A call's answer travels in a oneshot. The caller also watches one
//!   `CloseSignal` per calling scope, fired by a single close hook of that
//!   scope: `call_json` has no endpoint to hang a hook on, and one hook per
//!   call would pile up for the scope's lifetime.
//! - No lock is held while waking a task, sending an answer or dropping a
//!   message or a waker (a waker can hold the last reference to a task,
//!   whose drop runs anything). Locks nest in two places only: an entry's
//!   state over its clients' close signals (introspection), and the caller
//!   table over the scope's hook list (`Registry::caller`).

use std::collections::{hash_map, BTreeMap, HashMap, VecDeque};
use std::fmt;
use std::future::Future;
use std::marker::PhantomData;
use std::pin::Pin;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, Weak};
use std::task::{ready, Context, Poll, Waker};

use futures::channel::oneshot;
use futures::{Stream, StreamExt};

use crate::codec::{AnyMsg, Codec};
use crate::{check_name, ros_type_name, BusError, Message, Scope, ServiceInfo};

/// How a call ends, before the response is typed again.
type Answer = Result<AnyMsg, BusError>;

fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(|e| e.into_inner())
}

fn closed(scope: &Scope) -> BusError {
    BusError::Closed { scope: scope.name().to_string() }
}

/// The service half of the bus registry.
#[derive(Default)]
pub(crate) struct Registry {
    /// One entry per name, kept for the bus's lifetime: the types it fixed
    /// stay fixed after every endpoint is gone (README.md, "Model").
    entries: Mutex<BTreeMap<String, Arc<Entry>>>,
    /// The close signal of each scope that makes calls, by scope id. The
    /// scope's close hook owns it while the scope is open, hence `Weak`;
    /// dead ones are pruned whenever one is added.
    callers: Mutex<HashMap<u64, Weak<CloseSignal>>>,
}

impl Registry {
    pub(crate) fn info(&self) -> Vec<ServiceInfo> {
        let entries: Vec<Arc<Entry>> = lock(&self.entries).values().cloned().collect();
        entries.iter().map(|entry| entry.info()).collect()
    }

    /// The entry of `name`, created for `Req` / `Resp` if the name is new.
    fn entry<Req: Message, Resp: Message>(&self, name: &str) -> Result<Arc<Entry>, BusError> {
        let mut entries = lock(&self.entries);
        if let Some(entry) = entries.get(name) {
            entry.check::<Req, Resp>()?;
            return Ok(entry.clone());
        }
        let entry = Arc::new(Entry::new(name, Codec::of::<Req>(), Codec::of::<Resp>()));
        entries.insert(name.to_string(), entry.clone());
        Ok(entry)
    }

    fn get(&self, name: &str) -> Option<Arc<Entry>> {
        lock(&self.entries).get(name).cloned()
    }

    /// `scope`'s close signal, made together with its close hook on first
    /// use: one hook per calling scope, however many clients it creates and
    /// JSON calls it makes.
    fn caller(&self, scope: &Scope) -> Result<Arc<CloseSignal>, BusError> {
        let mut callers = lock(&self.callers);
        if let Some(signal) = callers.get(&scope.id()).and_then(Weak::upgrade) {
            return Ok(signal);
        }
        let signal = Arc::new(CloseSignal::new(scope.name()));
        // Strong on purpose: with a `Weak`, a scope whose calls had all
        // ended would get a new signal, and one more hook, on its next call.
        let fire = signal.clone();
        scope.on_close(Box::new(move || fire.fire()))?;
        callers.retain(|_, signal| signal.strong_count() > 0);
        callers.insert(scope.id(), Arc::downgrade(&signal));
        Ok(signal)
    }
}

pub(crate) fn serve<Req: Message, Resp: Message>(scope: &Scope, name: &str) -> Result<Service<Req, Resp>, BusError> {
    check_name(name)?;
    // Before touching the registry: a closed scope's attempt fixes no type.
    if scope.is_closed() {
        return Err(closed(scope));
    }
    let entry = scope.bus().service_registry().entry::<Req, Resp>(name)?;
    let server = Arc::new(Server { scope: scope.name().to_string(), state: Mutex::default() });
    let waiting = {
        let mut state = lock(&entry.state);
        if let Some(live) = &state.server {
            return Err(BusError::ServiceTaken { name: name.to_string(), server: live.scope.clone() });
        }
        state.server = Some(server.clone());
        state.waiting.take()
    };
    let (weak_entry, weak_server) = (Arc::downgrade(&entry), Arc::downgrade(&server));
    let hooked = scope.on_close(Box::new(move || {
        if let (Some(entry), Some(server)) = (weak_entry.upgrade(), weak_server.upgrade()) {
            server.end(&entry, true);
        }
    }));
    if hooked.is_err() {
        // The scope closed while the server registered: undo it as the
        // close would have.
        server.end(&entry, true);
    }
    // Taken out of the set above, so woken either way to look again.
    for waker in waiting {
        waker.wake();
    }
    hooked.map(|()| Service { entry, server, _t: PhantomData })
}

pub(crate) fn client<Req: Message, Resp: Message>(scope: &Scope, name: &str) -> Result<Client<Req, Resp>, BusError> {
    check_name(name)?;
    if scope.is_closed() {
        return Err(closed(scope));
    }
    let registry = scope.bus().service_registry();
    let entry = registry.entry::<Req, Resp>(name)?;
    let caller = registry.caller(scope)?;
    let id = entry.next_id();
    lock(&entry.state).clients.insert(id, caller.clone());
    Ok(Client { inner: Arc::new(ClientInner { entry, caller, id }), _t: PhantomData })
}

/// Errors, in order: `InvalidName`, `UnknownName` (no endpoint ever declared
/// the name), `Json` (the request does not decode), then those of a typed
/// call (`Closed`, `ServiceUnavailable`, `NoResponse`).
pub(crate) async fn call_json(scope: &Scope, name: &str, request: serde_json::Value) -> Result<serde_json::Value, BusError> {
    check_name(name)?;
    let registry = scope.bus().service_registry();
    let entry = registry.get(name).ok_or_else(|| BusError::UnknownName { name: name.to_string() })?;
    let json_error = |error| BusError::Json { name: entry.name.clone(), error };
    let request = (entry.request.from_json)(request).map_err(json_error)?;
    if scope.is_closed() {
        return Err(closed(scope));
    }
    let caller = registry.caller(scope)?;
    let response = entry.call(&caller, request)?.await?;
    let json = (entry.response.to_json)(response.as_ref());
    json.unwrap_or_else(|| Err("value/codec type mismatch".into())).map_err(json_error)
}

/// Everything the bus knows about one service name.
struct Entry {
    name: String,
    request: Codec,
    response: Codec,
    /// Calls handed to a live server since the bus started, answered or not.
    /// One refused at call time never reached a server and is not counted.
    calls: AtomicU64,
    /// Ids of this name's calls and clients.
    next_id: AtomicU64,
    state: Mutex<EntryState>,
}

#[derive(Default)]
struct EntryState {
    server: Option<Arc<Server>>,
    /// Live clients by id (so in creation order), each with its scope's
    /// close signal, which carries the scope's name.
    clients: BTreeMap<u64, Arc<CloseSignal>>,
    /// Tasks in `wait_for_service`, woken when a server registers.
    waiting: WakerSet,
}

impl Entry {
    fn new(name: &str, request: Codec, response: Codec) -> Self {
        Entry {
            name: name.to_string(),
            request,
            response,
            calls: AtomicU64::new(0),
            next_id: AtomicU64::new(0),
            state: Mutex::default(),
        }
    }

    /// `TypeMismatch` unless `Req` and `Resp` are the types the name carries.
    fn check<Req: Message, Resp: Message>(&self) -> Result<(), BusError> {
        let mismatch = |existing: &Codec, requested| BusError::TypeMismatch {
            name: self.name.clone(),
            existing: existing.type_name,
            requested,
        };
        if !self.request.is::<Req>() {
            return Err(mismatch(&self.request, std::any::type_name::<Req>()));
        }
        if !self.response.is::<Resp>() {
            return Err(mismatch(&self.response, std::any::type_name::<Resp>()));
        }
        Ok(())
    }

    fn next_id(&self) -> u64 {
        self.next_id.fetch_add(1, Ordering::Relaxed) + 1
    }

    fn unavailable(&self) -> BusError {
        BusError::ServiceUnavailable { name: self.name.clone() }
    }

    /// Queue `request` at the live server for a caller in the scope of
    /// `caller`. Refused at once when that scope is closed or no server is
    /// live.
    fn call(self: &Arc<Self>, caller: &Arc<CloseSignal>, request: AnyMsg) -> Result<Call, BusError> {
        if caller.is_closed() {
            return Err(caller.closed());
        }
        let server = lock(&self.state).server.clone().ok_or_else(|| self.unavailable())?;
        let (reply, answer) = oneshot::channel();
        if server.push(Pending { id: self.next_id(), request, reply }).is_err() {
            // It ended since we looked.
            return Err(self.unavailable());
        }
        self.calls.fetch_add(1, Ordering::Relaxed);
        Ok(Call { entry: self.clone(), answer, caller: caller.clone(), watch: None })
    }

    fn info(&self) -> ServiceInfo {
        let state = lock(&self.state);
        ServiceInfo {
            name: self.name.clone(),
            request_type: self.request.type_name,
            response_type: self.response.type_name,
            ros_type: ros_type_name(self.request.type_name),
            server: state.server.as_ref().map(|server| server.scope.clone()),
            // A client whose scope has closed is not live, dropped or not.
            clients: state.clients.values().filter(|c| !c.is_closed()).map(|c| c.scope.clone()).collect(),
            calls: self.calls.load(Ordering::Relaxed),
        }
    }
}

/// What one `Service` serves: the calls it has not answered yet.
struct Server {
    /// The serving scope's name.
    scope: String,
    state: Mutex<ServerState>,
}

#[derive(Default)]
struct ServerState {
    /// It takes no more calls: its `Service` dropped or its scope closed.
    ended: bool,
    /// Its scope closed: a late answer gets `Closed`.
    scope_closed: bool,
    queue: VecDeque<Pending>,
    /// Calls handed out as a `Request`, not answered yet, by call id.
    in_flight: HashMap<u64, oneshot::Sender<Answer>>,
    /// The `Service`'s task, while it waits for a call.
    waker: Option<Waker>,
}

/// A call waiting in a server's queue.
struct Pending {
    id: u64,
    request: AnyMsg,
    reply: oneshot::Sender<Answer>,
}

impl Server {
    /// Queue a call; it comes back if the server has ended.
    fn push(&self, call: Pending) -> Result<(), Pending> {
        let waker = {
            let mut state = lock(&self.state);
            if state.ended {
                return Err(call);
            }
            state.queue.push_back(call);
            state.waker.take()
        };
        if let Some(waker) = waker {
            waker.wake();
        }
        Ok(())
    }

    /// The next queued call, which is in flight from now on; `None` once the
    /// server has ended.
    fn poll_request(&self, cx: &mut Context<'_>) -> Poll<Option<(u64, AnyMsg)>> {
        let mut state = lock(&self.state);
        if state.ended {
            return Poll::Ready(None);
        }
        let Some(Pending { id, request, reply }) = state.queue.pop_front() else {
            let replaced = match &state.waker {
                Some(waker) if waker.will_wake(cx.waker()) => None,
                _ => state.waker.replace(cx.waker().clone()),
            };
            drop(state);
            drop(replaced);
            return Poll::Pending;
        };
        state.in_flight.insert(id, reply);
        Poll::Ready(Some((id, request)))
    }

    /// Stop serving: free the name (unless a newer server holds it already),
    /// end the `Service`'s stream and fail every call still held with
    /// `ServiceUnavailable`. Idempotent; `scope_closed` sticks, so a
    /// `Service` dropped before its scope closes still refuses late answers
    /// with `Closed` after the close.
    fn end(&self, entry: &Entry, scope_closed: bool) {
        let unregistered = {
            let mut state = lock(&entry.state);
            if state.server.as_ref().is_some_and(|live| std::ptr::eq(Arc::as_ptr(live), self)) {
                state.server.take()
            } else {
                None
            }
        };
        drop(unregistered);
        let (queue, in_flight, waker) = {
            let mut state = lock(&self.state);
            state.scope_closed |= scope_closed;
            if state.ended {
                return;
            }
            state.ended = true;
            (std::mem::take(&mut state.queue), std::mem::take(&mut state.in_flight), state.waker.take())
        };
        for reply in queue.into_iter().map(|call| call.reply).chain(in_flight.into_values()) {
            let _ = reply.send(Err(entry.unavailable()));
        }
        if let Some(waker) = waker {
            waker.wake();
        }
    }
}

/// The server side of one service name: a stream of requests.
///
/// Dropping it stops serving: the name is free for a new server, and the
/// calls it still holds, queued or handed out, fail with
/// `ServiceUnavailable`.
pub struct Service<Req: Message, Resp: Message> {
    entry: Arc<Entry>,
    server: Arc<Server>,
    _t: PhantomData<fn(Req) -> Resp>,
}

impl<Req: Message, Resp: Message> Service<Req, Resp> {
    pub fn name(&self) -> &str {
        &self.entry.name
    }

    /// The next request; `None` once the scope is closed.
    pub async fn recv(&mut self) -> Option<Request<Req, Resp>> {
        self.next().await
    }
}

impl<Req: Message, Resp: Message> Stream for Service<Req, Resp> {
    type Item = Request<Req, Resp>;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Request<Req, Resp>>> {
        let this = self.get_mut();
        let Some((id, request)) = ready!(this.server.poll_request(cx)) else {
            return Poll::Ready(None);
        };
        let (entry, server) = (this.entry.clone(), this.server.clone());
        let responder = Responder { entry, server, id: Some(id), _t: PhantomData };
        Poll::Ready(Some(Request { message: unerase(request), responder }))
    }
}

impl<Req: Message, Resp: Message> Drop for Service<Req, Resp> {
    fn drop(&mut self) {
        self.server.end(&self.entry, false);
    }
}

impl<Req: Message, Resp: Message> fmt::Debug for Service<Req, Resp> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Service").field("name", &self.entry.name).field("scope", &self.server.scope).finish()
    }
}

/// One call, as the server sees it (r2r's `ServiceRequest`). Dropping it
/// unanswered fails the call with `NoResponse`.
pub struct Request<Req: Message, Resp: Message> {
    pub message: Req,
    responder: Responder<Resp>,
}

impl<Req: Message, Resp: Message> Request<Req, Resp> {
    /// Answer the call. `Ok` even if nobody waits for the answer any more
    /// (the caller gave up, or the `Service` was dropped and failed the call
    /// already): the answer is dropped. `Closed` if the server's scope has
    /// closed.
    pub fn respond(self, response: Resp) -> Result<(), BusError> {
        let mut responder = self.responder;
        responder.answer(Ok(Arc::new(response)))
    }
}

impl<Req: Message + fmt::Debug, Resp: Message> fmt::Debug for Request<Req, Resp> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Request")
            .field("service", &self.responder.entry.name)
            .field("message", &self.message)
            .finish_non_exhaustive()
    }
}

/// The answering half of a `Request`, kept apart so that `message` can
/// still be moved out of a request.
struct Responder<Resp> {
    entry: Arc<Entry>,
    server: Arc<Server>,
    /// The call's id; `None` once answered.
    id: Option<u64>,
    _t: PhantomData<fn(Resp)>,
}

impl<Resp> Responder<Resp> {
    fn answer(&mut self, answer: Answer) -> Result<(), BusError> {
        let Some(id) = self.id.take() else {
            return Ok(());
        };
        let (reply, scope_closed) = {
            let mut state = lock(&self.server.state);
            (state.in_flight.remove(&id), state.scope_closed)
        };
        match reply {
            // The send fails only when the caller gave up.
            Some(reply) => {
                let _ = reply.send(answer);
                Ok(())
            }
            // The server ended and failed the call already.
            None if scope_closed => Err(BusError::Closed { scope: self.server.scope.clone() }),
            None => Ok(()),
        }
    }
}

impl<Resp> Drop for Responder<Resp> {
    fn drop(&mut self) {
        if self.id.is_some() {
            let _ = self.answer(Err(BusError::NoResponse { name: self.entry.name.clone() }));
        }
    }
}

/// A client of one service name. Clones are the same client (one entry in
/// [`crate::Bus::services`], gone with the last clone); a call in flight
/// does not need the client any more.
pub struct Client<Req: Message, Resp: Message> {
    inner: Arc<ClientInner>,
    _t: PhantomData<fn(Req) -> Resp>,
}

/// A client's registration, shared by its clones.
struct ClientInner {
    entry: Arc<Entry>,
    /// The client's scope's close signal.
    caller: Arc<CloseSignal>,
    id: u64,
}

impl Drop for ClientInner {
    fn drop(&mut self) {
        let removed = lock(&self.entry.state).clients.remove(&self.id);
        drop(removed);
    }
}

impl<Req: Message, Resp: Message> Clone for Client<Req, Resp> {
    fn clone(&self) -> Self {
        Client { inner: self.inner.clone(), _t: PhantomData }
    }
}

impl<Req: Message, Resp: Message> Client<Req, Resp> {
    pub fn name(&self) -> &str {
        &self.inner.entry.name
    }

    /// Send `msg` (cloned) and get the future of the answer: r2r's
    /// `client.request(&req)?.await?`. Outer error: `Closed`, or
    /// `ServiceUnavailable` when no server is live at call time.
    pub fn request(&self, msg: &Req) -> Result<ResponseFuture<Resp>, BusError> {
        let call = self.inner.entry.call(&self.inner.caller, Arc::new(msg.clone()))?;
        Ok(ResponseFuture { call, _t: PhantomData })
    }

    /// Whether a server is live now. Says nothing about the client's own
    /// scope: once that has closed, `request` fails with `Closed` anyway.
    pub fn is_available(&self) -> bool {
        lock(&self.inner.entry.state).server.is_some()
    }

    /// Resolve once a server is live (at once if one is). `Closed` if the
    /// client's scope closes first.
    pub async fn wait_for_service(&self) -> Result<(), BusError> {
        WaitForService { client: &self.inner, closing: None, waiting: None }.await
    }
}

impl<Req: Message, Resp: Message> fmt::Debug for Client<Req, Resp> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let ClientInner { entry, caller, .. } = &*self.inner;
        f.debug_struct("Client").field("name", &entry.name).field("scope", &caller.scope).finish()
    }
}

/// `Client::wait_for_service`. A future of its own so that a wait given up
/// (dropped by a timeout: modules poll for a missing server in a loop)
/// takes its wakers back out.
struct WaitForService<'a> {
    client: &'a ClientInner,
    closing: Option<u64>,
    waiting: Option<u64>,
}

impl Future for WaitForService<'_> {
    type Output = Result<(), BusError>;

    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = &mut *self;
        let client = this.client;
        if !client.caller.watch(&mut this.closing, cx.waker()) {
            return Poll::Ready(Err(client.caller.closed()));
        }
        let mut state = lock(&client.entry.state);
        if state.server.is_some() {
            return Poll::Ready(Ok(()));
        }
        let replaced = state.waiting.register(&mut this.waiting, cx.waker());
        drop(state);
        drop(replaced);
        Poll::Pending
    }
}

impl Drop for WaitForService<'_> {
    fn drop(&mut self) {
        self.client.caller.unwatch(&mut self.closing);
        if let Some(key) = self.waiting.take() {
            let removed = lock(&self.client.entry.state).waiting.remove(key);
            drop(removed);
        }
    }
}

/// The answer to one call: `ServiceUnavailable`, `NoResponse` or `Closed`
/// when there will be none. Once the caller's scope has closed it resolves
/// to `Closed`, whatever the server said, so a stale task of an ended module
/// does not act on it. Dropping it abandons the call: the server still gets
/// the request, and its answer is dropped.
pub struct ResponseFuture<Resp> {
    call: Call,
    _t: PhantomData<fn() -> Resp>,
}

impl<Resp: Message> Future for ResponseFuture<Resp> {
    type Output = Result<Resp, BusError>;

    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        Pin::new(&mut self.call).poll(cx).map(|answer| answer.map(unerase))
    }
}

impl<Resp> fmt::Debug for ResponseFuture<Resp> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ResponseFuture").field("service", &self.call.entry.name).finish_non_exhaustive()
    }
}

/// A call in flight, its answer still erased: what `ResponseFuture` and
/// `call_json` wait on.
struct Call {
    entry: Arc<Entry>,
    answer: oneshot::Receiver<Answer>,
    /// The calling scope's close signal, and this call's key in it.
    caller: Arc<CloseSignal>,
    watch: Option<u64>,
}

impl Future for Call {
    type Output = Answer;

    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Answer> {
        let this = &mut *self;
        // Watch the scope before looking at the answer, so that a close in
        // between still wakes this task. Once the scope has closed, whatever
        // the server says no longer matters.
        if !this.caller.watch(&mut this.watch, cx.waker()) {
            return Poll::Ready(Err(this.caller.closed()));
        }
        let answer = ready!(Pin::new(&mut this.answer).poll(cx));
        this.caller.unwatch(&mut this.watch);
        // Canceled: the server side went away without a word.
        Poll::Ready(answer.unwrap_or_else(|_| Err(this.entry.unavailable())))
    }
}

impl Drop for Call {
    fn drop(&mut self) {
        self.caller.unwatch(&mut self.watch);
    }
}

/// Fired once, when a scope that makes calls closes: its calls in flight and
/// its `wait_for_service`s wake up and end with `Closed`.
struct CloseSignal {
    scope: String,
    state: Mutex<CloseState>,
}

#[derive(Default)]
struct CloseState {
    fired: bool,
    watchers: WakerSet,
}

impl CloseSignal {
    fn new(scope: &str) -> Self {
        CloseSignal { scope: scope.to_string(), state: Mutex::default() }
    }

    fn is_closed(&self) -> bool {
        lock(&self.state).fired
    }

    fn closed(&self) -> BusError {
        BusError::Closed { scope: self.scope.clone() }
    }

    /// Have `waker` woken when the scope closes; `false` if it has.
    fn watch(&self, key: &mut Option<u64>, waker: &Waker) -> bool {
        let mut state = lock(&self.state);
        if state.fired {
            return false;
        }
        let replaced = state.watchers.register(key, waker);
        drop(state);
        drop(replaced);
        true
    }

    fn unwatch(&self, key: &mut Option<u64>) {
        if let Some(key) = key.take() {
            let removed = lock(&self.state).watchers.remove(key);
            drop(removed);
        }
    }

    fn fire(&self) {
        let watchers = {
            let mut state = lock(&self.state);
            state.fired = true;
            state.watchers.take()
        };
        for waker in watchers {
            waker.wake();
        }
    }
}

/// Tasks waiting for one event, each under its own key, so that a waiter
/// that gives up takes its waker back out instead of leaving it to pile up.
///
/// It lives under a lock, so a waker it lets go of is handed back to be
/// dropped once the lock is released.
#[derive(Default)]
struct WakerSet {
    last_key: u64,
    wakers: HashMap<u64, Waker>,
}

impl WakerSet {
    /// Add or refresh the waker under `key`, picking a key on first use.
    /// Returns the waker it replaces.
    #[must_use = "drop the replaced waker once the lock is released"]
    fn register(&mut self, key: &mut Option<u64>, waker: &Waker) -> Option<Waker> {
        let key = *key.get_or_insert_with(|| {
            self.last_key += 1;
            self.last_key
        });
        match self.wakers.entry(key) {
            hash_map::Entry::Occupied(slot) if slot.get().will_wake(waker) => None,
            hash_map::Entry::Occupied(mut slot) => Some(slot.insert(waker.clone())),
            hash_map::Entry::Vacant(slot) => {
                slot.insert(waker.clone());
                None
            }
        }
    }

    /// Take the waker under `key` out.
    #[must_use = "drop the removed waker once the lock is released"]
    fn remove(&mut self, key: u64) -> Option<Waker> {
        self.wakers.remove(&key)
    }

    /// Empty the set, to wake everyone once the lock is released.
    fn take(&mut self) -> Vec<Waker> {
        self.wakers.drain().map(|(_, waker)| waker).collect()
    }
}

/// A message back in its own type: moved out of the `Arc` when nothing else
/// holds it (always so for the bus's own requests and answers), else cloned.
fn unerase<T: Message>(msg: AnyMsg) -> T {
    let msg = msg.downcast::<T>().expect("a name's types are fixed, and every endpoint is checked against them");
    Arc::unwrap_or_clone(msg)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Bus;

    fn noop_cx() -> Context<'static> {
        Context::from_waker(futures::task::noop_waker_ref())
    }

    fn hooks(scope: &Scope) -> usize {
        scope.hook_count()
    }

    #[test]
    fn waits_and_calls_given_up_leave_no_waker_behind() {
        let bus = Bus::new();
        let caller = bus.scope("caller");
        let client = caller.client::<u32, u32>("/x").unwrap();
        let mut cx = noop_cx();
        for _ in 0..3 {
            let mut wait = Box::pin(client.wait_for_service());
            assert!(wait.as_mut().poll(&mut cx).is_pending());
        }
        assert!(lock(&client.inner.entry.state).waiting.wakers.is_empty());
        assert!(lock(&client.inner.caller.state).watchers.wakers.is_empty());

        let server = bus.scope("server");
        let _srv = server.service::<u32, u32>("/x").unwrap();
        for _ in 0..3 {
            let mut call = client.request(&1).unwrap();
            assert!(Pin::new(&mut call).poll(&mut cx).is_pending());
        }
        assert!(lock(&client.inner.caller.state).watchers.wakers.is_empty());
    }

    /// A waker that records, when its last clone drops, whether every lock
    /// a service endpoint takes was free at that moment.
    struct DropProbe {
        entry: Arc<Entry>,
        caller: Arc<CloseSignal>,
        server: Option<Arc<Server>>,
        freed: Arc<Mutex<Vec<bool>>>,
    }

    impl std::task::Wake for DropProbe {
        fn wake(self: Arc<Self>) {}
    }

    impl Drop for DropProbe {
        fn drop(&mut self) {
            let free = self.entry.state.try_lock().is_ok()
                && self.caller.state.try_lock().is_ok()
                && self.server.as_ref().is_none_or(|server| server.state.try_lock().is_ok());
            self.freed.lock().unwrap().push(free);
        }
    }

    #[test]
    fn wakers_replaced_or_given_back_are_dropped_with_no_lock_held() {
        let bus = Bus::new();
        let (serving, calling) = (bus.scope("server"), bus.scope("caller"));
        let mut srv = serving.service::<u32, u32>("/x").unwrap();
        let client = calling.client::<u32, u32>("/x").unwrap();
        let lonely = calling.client::<u32, u32>("/nobody_serves").unwrap();
        let freed = Arc::new(Mutex::new(Vec::new()));
        // Each probe's last clone ends up in the bus: the test drops its own
        // right after the first poll.
        let probe = |client: &Client<u32, u32>, server: Option<&Arc<Server>>| {
            let (entry, caller) = (client.inner.entry.clone(), client.inner.caller.clone());
            Waker::from(Arc::new(DropProbe { entry, caller, server: server.cloned(), freed: freed.clone() }))
        };
        let freed_now = || std::mem::take(&mut *freed.lock().unwrap());
        let mut noop = noop_cx();

        // the waiting server's waker, replaced by the next poll
        let waker = probe(&client, Some(&srv.server));
        assert!(Pin::new(&mut srv).poll_next(&mut Context::from_waker(&waker)).is_pending());
        drop(waker);
        assert!(Pin::new(&mut srv).poll_next(&mut noop).is_pending());
        assert_eq!(freed_now(), [true], "Server::poll_request");

        // a wait for a missing server: its waker in the entry's set, replaced
        // and then given back by the drop
        for drop_it in [false, true] {
            let mut wait = WaitForService { client: &lonely.inner, closing: None, waiting: None };
            let waker = probe(&lonely, None);
            assert!(Pin::new(&mut wait).poll(&mut Context::from_waker(&waker)).is_pending());
            drop(waker);
            if drop_it {
                drop(wait);
            } else {
                assert!(Pin::new(&mut wait).poll(&mut noop).is_pending());
            }
            assert_eq!(freed_now(), [true], "the entry's waiting set (dropped: {drop_it})");
        }

        // a wait that found the server: its waker only in the scope's close
        // signal, replaced and then given back by the drop
        for drop_it in [false, true] {
            let mut wait = WaitForService { client: &client.inner, closing: None, waiting: None };
            let waker = probe(&client, None);
            assert!(Pin::new(&mut wait).poll(&mut Context::from_waker(&waker)).is_ready());
            drop(waker);
            if drop_it {
                drop(wait);
            } else {
                assert!(Pin::new(&mut wait).poll(&mut noop).is_ready());
            }
            assert_eq!(freed_now(), [true], "the close signal's watchers (dropped: {drop_it})");
        }
    }

    #[test]
    fn a_scope_gets_one_close_hook_however_many_clients_and_json_calls_it_makes() {
        let bus = Bus::new();
        let bridge = bus.scope("bridge");
        let a = bridge.client::<u32, u32>("/a").unwrap();
        let b = bridge.client::<u32, u32>("/b").unwrap();
        for _ in 0..5 {
            let unavailable = futures::executor::block_on(bridge.call_json("/a", serde_json::json!(1)));
            assert_eq!(unavailable, Err(BusError::ServiceUnavailable { name: "/a".into() }));
        }
        assert!(Arc::ptr_eq(&a.inner.caller, &b.inner.caller));
        assert_eq!(hooks(&bridge), 1);
        // a server takes one per service
        let server = bus.scope("server");
        let _a = server.service::<u32, u32>("/a").unwrap();
        let _c = server.service::<u32, u32>("/c").unwrap();
        assert_eq!(hooks(&server), 2);
    }

    #[test]
    fn closed_scopes_leave_the_caller_table_and_dropped_clients_their_entry() {
        let bus = Bus::new();
        for _ in 0..10 {
            let scope = bus.scope("restarted");
            let client = scope.client::<u32, u32>("/x").unwrap();
            scope.close();
            drop(client);
        }
        let live = bus.scope("live");
        let client = live.client::<u32, u32>("/x").unwrap();
        assert_eq!(lock(&bus.service_registry().callers).len(), 1);
        assert_eq!(lock(&client.inner.entry.state).clients.len(), 1);
    }
}
