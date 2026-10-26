//! Actions: one server per name, goals that run until their server ends
//! them, each with cancel requests, feedback and one result. Contract:
//! README.md, "Actions".
//!
//! Requirements beyond the README:
//! - no async runtime needed, as for services: the futures and streams can
//!   be polled on any executor, and the server's calls (answers, feedback,
//!   the terminal transitions) work from a std thread;
//! - the handles are `Send + Unpin`, the futures and streams `'static` (they
//!   can be spawned), and `ActionClient`, `ActionClientGoal` and
//!   `ActionServerGoal` are `Clone`, as r2r's are;
//! - the API is r2r's as mower_nav and mower_coverage use it: the same type
//!   names, the same two-level `Result`s, the same triple from an accepted
//!   goal request (handle, result future, feedback stream), so that their
//!   port is mechanical.
//!
//! How the parts fit:
//! - Built natively, not from bus services and topics the way rcl_action
//!   builds actions from DDS ones: the bus has no runtime to serve a goal, a
//!   cancel and a result service on, and the two ends of a goal want one
//!   state they share.
//! - The registry keeps one `Entry` per name for the bus's lifetime: the
//!   codecs of the three types the first endpoint fixed, the live `Server`,
//!   the live clients (for introspection) and the tasks in
//!   `wait_for_server` and `wait_for_no_server`.
//! - A `Server` is what one `ActionServer` serves: the goal requests it has
//!   not handed out, those it handed out that nobody answered yet, and the
//!   goals it accepted that have not ended. Dropping the `ActionServer`
//!   frees the name and fails the queued requests; its scope's close ends
//!   the rest, and loses the goals (their clients get `ServiceUnavailable`).
//! - A `Goal` is what the two ends of one accepted goal share: its status,
//!   its end (the result, or why there is none), the feedback queued for
//!   the client and the cancel requests queued for the server. The server
//!   holds its goals by `Weak`, so that nothing forms a cycle through the
//!   registry.
//! - A client watches its scope through the `CloseSignal` the scope's
//!   service clients use too (service.rs): one close hook per calling scope,
//!   however many goals it sends.
//! - As everywhere in the bus, no lock is held while waking a task, sending
//!   an answer or dropping a message or a waker, and several of those in a
//!   row are each a call of their own (`crate::wake::UserCode`). Locks nest
//!   in one place only: an entry's state over its clients' close signals
//!   (introspection).

use std::collections::{BTreeMap, HashMap, VecDeque};
use std::fmt;
use std::future::Future;
use std::marker::PhantomData;
use std::pin::Pin;
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError, Weak};
use std::task::{ready, Context, Poll, Waker};

use futures::channel::oneshot;
use futures::{Stream, StreamExt};

use crate::codec::{AnyMsg, Codec};
use crate::service::CloseSignal;
use crate::wake::{UserCode, WakerSet};
use crate::{check_name, ros_type_name, ActionInfo, BusError, Message, Scope};

/// How many feedback messages a goal keeps for its client: the depth of
/// rcl_action's feedback topic (`rmw_qos_profile_default`, KEEP_LAST 10).
const FEEDBACK_DEPTH: usize = 10;

/// What a goal request gets back: the accepted goal, or why not.
type GoalAnswer = Result<Arc<Goal>, BusError>;

/// What a cancel request gets back.
type CancelAnswer = Result<(), BusError>;

/// A goal's end, before the result is typed again: the result with its
/// terminal status, or why there is none.
type End = Result<(GoalStatus, AnyMsg), BusError>;

fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(PoisonError::into_inner)
}

fn closed(scope: &str) -> BusError {
    BusError::Closed { scope: scope.to_string() }
}

/// Run the batch from drop glue: re-raise its first panic, unless the
/// thread is unwinding already, where a second panic would abort.
fn finish_in_drop(user: UserCode) {
    if std::thread::panicking() {
        drop(user);
    } else {
        user.finish();
    }
}

/// An action's three message types: what r2r's `WrappedActionTypeSupport`
/// is to an r2r action, without the ROS part. Implemented for every
/// `(Goal, Result, Feedback)` tuple of bus messages, so a module names its
/// action once,
/// `type WaypointAction = (Waypoint::Goal, Waypoint::Result, Waypoint::Feedback);`,
/// and uses it where r2r takes `Waypoint::Action`.
pub trait Action: 'static {
    type Goal: Message;
    type Result: Message;
    type Feedback: Message;
}

impl<G: Message, R: Message, F: Message> Action for (G, R, F) {
    type Goal = G;
    type Result = R;
    type Feedback = F;
}

/// One goal of an action name. The bus numbers them from 1 up, unique on
/// their name for the bus's lifetime; rcl_action's goal ids are random UUIDs
/// that the clients pick.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct GoalId(u64);

impl GoalId {
    pub fn get(self) -> u64 {
        self.0
    }
}

impl fmt::Display for GoalId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

/// A goal's state: `action_msgs/GoalStatus`, as r2r names it.
///
/// On the bus a goal is `Executing` from its acceptance on (r2r's `accept`
/// executes at once, so `Accepted` never shows), `Canceling` once its server
/// accepted a cancel request, and then `Succeeded`, `Canceled` or `Aborted`.
/// A goal that ended without a result, because its server went away
/// (README.md, "Actions"), is `Unknown`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GoalStatus {
    Unknown,
    Accepted,
    Executing,
    Canceling,
    Succeeded,
    Canceled,
    Aborted,
}

impl GoalStatus {
    /// `Succeeded`, `Canceled` or `Aborted`: what a result carries.
    pub fn is_terminal(self) -> bool {
        matches!(self, GoalStatus::Succeeded | GoalStatus::Canceled | GoalStatus::Aborted)
    }
}

impl fmt::Display for GoalStatus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            GoalStatus::Unknown => "unknown",
            GoalStatus::Accepted => "accepted",
            GoalStatus::Executing => "executing",
            GoalStatus::Canceling => "canceling",
            GoalStatus::Succeeded => "succeeded",
            GoalStatus::Canceled => "canceled",
            GoalStatus::Aborted => "aborted",
        })
    }
}

/// The action half of the bus registry.
#[derive(Default)]
pub(crate) struct Registry {
    /// One entry per name, kept for the bus's lifetime: the types it fixed
    /// stay fixed after every endpoint is gone (README.md, "Model").
    entries: Mutex<BTreeMap<String, Arc<Entry>>>,
}

impl Registry {
    pub(crate) fn info(&self) -> Vec<ActionInfo> {
        let entries: Vec<Arc<Entry>> = lock(&self.entries).values().cloned().collect();
        entries.iter().map(|entry| entry.info()).collect()
    }

    /// The entry of `name`, created for `A`'s types if the name is new.
    fn entry<A: Action>(&self, name: &str) -> Result<Arc<Entry>, BusError> {
        let mut entries = lock(&self.entries);
        if let Some(entry) = entries.get(name) {
            entry.check::<A>()?;
            return Ok(entry.clone());
        }
        let entry = Arc::new(Entry::new::<A>(name));
        entries.insert(name.to_string(), entry.clone());
        Ok(entry)
    }
}

pub(crate) fn serve<A: Action>(scope: &Scope, name: &str) -> Result<ActionServer<A>, BusError> {
    check_name(name)?;
    // Before touching the registry: a closed scope's attempt fixes no type.
    if scope.is_closed() {
        return Err(closed(scope.name()));
    }
    let entry = scope.bus().action_registry().entry::<A>(name)?;
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
            server.close(&entry);
        }
    }));
    if hooked.is_err() {
        // The scope closed while the server registered: undo it as the
        // close would have.
        server.close(&entry);
    }
    // As for a service: woken either way, and a waker's panic goes on once
    // the `ActionServer` exists, so that unwinding drops it.
    let mut user = UserCode::default();
    for waker in waiting {
        user.wake(waker);
    }
    let served = hooked.map(|()| ActionServer { entry, server, _t: PhantomData });
    user.finish();
    served
}

pub(crate) fn client<A: Action>(scope: &Scope, name: &str) -> Result<ActionClient<A>, BusError> {
    check_name(name)?;
    if scope.is_closed() {
        return Err(closed(scope.name()));
    }
    let entry = scope.bus().action_registry().entry::<A>(name)?;
    let caller = scope.bus().service_registry().caller(scope)?;
    let id = entry.next_client.fetch_add(1, Ordering::Relaxed) + 1;
    lock(&entry.state).clients.insert(id, caller.clone());
    Ok(ActionClient { inner: Arc::new(ClientInner { entry, caller, id }), _t: PhantomData })
}

/// Everything the bus knows about one action name.
struct Entry {
    name: String,
    goal: Codec,
    result: Codec,
    feedback: Codec,
    /// Goal requests handed to a live server since the bus started,
    /// answered or not. One refused at send time is not counted.
    sent: AtomicU64,
    /// Goals accepted that have not ended, whichever server accepted them.
    active: AtomicUsize,
    next_goal: AtomicU64,
    next_client: AtomicU64,
    state: Mutex<EntryState>,
}

#[derive(Default)]
struct EntryState {
    server: Option<Arc<Server>>,
    /// Live clients by id, each with its scope's close signal, which carries
    /// the scope's name.
    clients: BTreeMap<u64, Arc<CloseSignal>>,
    /// Tasks in `wait_for_server`, woken when a server registers.
    waiting: WakerSet,
    /// Tasks in `wait_for_no_server`, woken when the server goes.
    leaving: WakerSet,
}

impl Entry {
    fn new<A: Action>(name: &str) -> Self {
        Entry {
            name: name.to_string(),
            goal: Codec::of::<A::Goal>(),
            result: Codec::of::<A::Result>(),
            feedback: Codec::of::<A::Feedback>(),
            sent: AtomicU64::new(0),
            active: AtomicUsize::new(0),
            next_goal: AtomicU64::new(0),
            next_client: AtomicU64::new(0),
            state: Mutex::default(),
        }
    }

    /// `TypeMismatch` unless `A`'s three types are the ones the name carries.
    fn check<A: Action>(&self) -> Result<(), BusError> {
        let mismatch = |existing: &Codec, requested| BusError::TypeMismatch {
            name: self.name.clone(),
            existing: existing.type_name,
            requested,
        };
        if !self.goal.is::<A::Goal>() {
            return Err(mismatch(&self.goal, std::any::type_name::<A::Goal>()));
        }
        if !self.result.is::<A::Result>() {
            return Err(mismatch(&self.result, std::any::type_name::<A::Result>()));
        }
        if !self.feedback.is::<A::Feedback>() {
            return Err(mismatch(&self.feedback, std::any::type_name::<A::Feedback>()));
        }
        Ok(())
    }

    fn unavailable(&self) -> BusError {
        BusError::ServiceUnavailable { name: self.name.clone() }
    }

    fn info(&self) -> ActionInfo {
        let state = lock(&self.state);
        ActionInfo {
            name: self.name.clone(),
            goal_type: self.goal.type_name,
            result_type: self.result.type_name,
            feedback_type: self.feedback.type_name,
            ros_type: ros_type_name(self.goal.type_name),
            server: state.server.as_ref().map(|server| server.scope.clone()),
            // A client whose scope has closed is not live, dropped or not.
            clients: state.clients.values().filter(|c| !c.is_closed()).map(|c| c.scope.clone()).collect(),
            goals: self.sent.load(Ordering::Relaxed),
            active: self.active.load(Ordering::Relaxed),
        }
    }
}

/// What one `ActionServer` serves, and what its scope's close ends.
struct Server {
    /// The serving scope's name.
    scope: String,
    state: Mutex<ServerState>,
}

#[derive(Default)]
struct ServerState {
    /// It takes no more goal requests: its `ActionServer` dropped, or its
    /// scope closed.
    ended: bool,
    /// Its scope closed: what it held has ended with it, and a late answer
    /// gets `Closed`.
    scope_closed: bool,
    queue: VecDeque<PendingGoal>,
    /// Requests handed out as an `ActionServerGoalRequest`, not answered yet.
    in_flight: HashMap<GoalId, oneshot::Sender<GoalAnswer>>,
    /// The goals it accepted that have not ended. Their handles hold them.
    goals: HashMap<GoalId, Weak<Goal>>,
    /// The `ActionServer`'s task, while it waits for a request.
    waker: Option<Waker>,
}

/// A goal request waiting in a server's queue.
struct PendingGoal {
    id: GoalId,
    goal: AnyMsg,
    reply: oneshot::Sender<GoalAnswer>,
}

impl Server {
    /// Queue a request; it comes back if the server takes no more.
    fn push(&self, request: PendingGoal) -> Result<(), PendingGoal> {
        let waker = {
            let mut state = lock(&self.state);
            if state.ended {
                return Err(request);
            }
            state.queue.push_back(request);
            state.waker.take()
        };
        if let Some(waker) = waker {
            waker.wake();
        }
        Ok(())
    }

    /// The next queued request, which is in flight from now on; `None` once
    /// the server takes no more.
    fn poll_request(&self, cx: &mut Context<'_>) -> Poll<Option<(GoalId, AnyMsg)>> {
        let mut state = lock(&self.state);
        if state.ended {
            return Poll::Ready(None);
        }
        let Some(PendingGoal { id, goal, reply }) = state.queue.pop_front() else {
            let replaced = match &state.waker {
                Some(waker) if waker.will_wake(cx.waker()) => None,
                _ => state.waker.replace(cx.waker().clone()),
            };
            drop(state);
            drop(replaced);
            return Poll::Pending;
        };
        state.in_flight.insert(id, reply);
        Poll::Ready(Some((id, goal)))
    }

    /// The answer channel of request `id`, handed out earlier, if it is
    /// still waiting; and whether the scope has closed.
    fn take_in_flight(&self, id: GoalId) -> (Option<oneshot::Sender<GoalAnswer>>, bool) {
        let mut state = lock(&self.state);
        (state.in_flight.remove(&id), state.scope_closed)
    }

    /// Accept request `goal.id` as `goal`: the scope's close loses it from
    /// now on, and it counts as active. The request's answer channel comes
    /// back, to be sent on with no lock held. `Err` once the scope has
    /// closed: the close failed the request already.
    fn adopt(&self, goal: &Arc<Goal>) -> Result<Option<oneshot::Sender<GoalAnswer>>, ()> {
        let mut state = lock(&self.state);
        if state.scope_closed {
            return Err(());
        }
        goal.entry.active.fetch_add(1, Ordering::Relaxed);
        state.goals.insert(goal.id, Arc::downgrade(goal));
        Ok(state.in_flight.remove(&goal.id))
    }

    /// An ended goal leaves the server.
    fn forget(&self, id: GoalId) {
        lock(&self.state).goals.remove(&id);
    }

    /// Free the name, unless a newer server holds it already. Returns the
    /// tasks in `wait_for_no_server` to wake, with no lock held, when it was
    /// this server that held the name.
    fn unregister(&self, entry: &Entry) -> Vec<Waker> {
        let (unregistered, leaving) = {
            let mut state = lock(&entry.state);
            if state.server.as_ref().is_some_and(|live| std::ptr::eq(Arc::as_ptr(live), self)) {
                (state.server.take(), state.leaving.take())
            } else {
                (None, Vec::new())
            }
        };
        drop(unregistered);
        leaving
    }

    /// Its `ActionServer` was dropped, the scope still open: free the name,
    /// end the stream and fail the queued requests with
    /// `ServiceUnavailable`. The requests it handed out are the module's to
    /// answer still, and the goals it accepted run on (README.md,
    /// "Actions").
    fn stop(&self, entry: &Entry) {
        let leaving = self.unregister(entry);
        let ended = {
            let mut state = lock(&self.state);
            if state.ended {
                None
            } else {
                state.ended = true;
                Some((std::mem::take(&mut state.queue), state.waker.take()))
            }
        };
        let mut user = UserCode::default();
        if let Some((queue, waker)) = ended {
            fail(&mut user, entry, queue, HashMap::new());
            if let Some(waker) = waker {
                user.wake(waker);
            }
        }
        // Woken by the end that unregistered it, even when it had ended.
        for waker in leaving {
            user.wake(waker);
        }
        finish_in_drop(user);
    }

    /// Its scope closed: free the name and end everything it holds. The
    /// requests queued or handed out fail with `ServiceUnavailable`, and so
    /// do the goals it accepted that have not ended (they are lost).
    /// Idempotent.
    fn close(&self, entry: &Entry) {
        let leaving = self.unregister(entry);
        let (queue, in_flight, goals, waker) = {
            let mut state = lock(&self.state);
            state.scope_closed = true;
            state.ended = true;
            let goals: Vec<Weak<Goal>> = state.goals.drain().map(|(_, goal)| goal).collect();
            (std::mem::take(&mut state.queue), std::mem::take(&mut state.in_flight), goals, state.waker.take())
        };
        let mut user = UserCode::default();
        fail(&mut user, entry, queue, in_flight);
        for goal in goals.iter().filter_map(Weak::upgrade) {
            goal.lose(entry.unavailable()).run(&mut user);
        }
        if let Some(waker) = waker {
            user.wake(waker);
        }
        for waker in leaving {
            user.wake(waker);
        }
        user.finish();
    }
}

/// Fail requests a server will not answer with `ServiceUnavailable`, each
/// send and each drop a call of its own: a send wakes the client's task, a
/// goal's drop runs its drop glue.
fn fail(
    user: &mut UserCode, entry: &Entry, queue: VecDeque<PendingGoal>, in_flight: HashMap<GoalId, oneshot::Sender<GoalAnswer>>,
) {
    for PendingGoal { goal, reply, .. } in queue {
        user.run(|| {
            let _ = reply.send(Err(entry.unavailable()));
        });
        user.run(|| drop(goal));
    }
    for reply in in_flight.into_values() {
        user.run(|| {
            let _ = reply.send(Err(entry.unavailable()));
        });
    }
}

/// One accepted goal: what its server's handle and its client's handles
/// share.
struct Goal {
    id: GoalId,
    entry: Arc<Entry>,
    state: Mutex<GoalState>,
}

struct GoalState {
    status: GoalStatus,
    /// It reached a terminal status, or was lost; set once.
    ended: bool,
    /// Why a lost goal has no result: `ServiceUnavailable` (its server's
    /// scope closed) or `NoResponse` (its server dropped its last handle).
    lost: Option<BusError>,
    /// The end, until the result future takes it.
    end: Option<End>,
    result_waker: Option<Waker>,
    /// Feedback for the client, oldest first, up to `FEEDBACK_DEPTH`.
    feedback: VecDeque<AnyMsg>,
    /// Feedback pushed out of the full queue.
    feedback_dropped: u64,
    /// A client reads feedback: its stream lives, or will once it reads the
    /// goal response.
    feedback_wanted: bool,
    feedback_waker: Option<Waker>,
    /// Cancel requests the server has not taken yet, oldest first.
    cancels: VecDeque<oneshot::Sender<CancelAnswer>>,
    /// Those it took and has not answered, by key.
    cancels_in_flight: HashMap<u64, oneshot::Sender<CancelAnswer>>,
    next_cancel: u64,
    /// The server takes cancel requests: its `CancelRequests` lives.
    cancels_taken: bool,
    cancel_waker: Option<Waker>,
}

/// What a change of a goal leaves to do once its lock is released.
#[derive(Default)]
#[must_use = "run it once the lock is released"]
struct Wakeups {
    answers: Vec<(oneshot::Sender<CancelAnswer>, CancelAnswer)>,
    wakers: Vec<Waker>,
}

impl Wakeups {
    /// Answers first (they wake the clients' cancel futures), then the
    /// tasks that wait on the goal, each a call of its own.
    fn run(self, user: &mut UserCode) {
        for (reply, answer) in self.answers {
            user.run(|| {
                let _ = reply.send(answer);
            });
        }
        for waker in self.wakers {
            user.wake(waker);
        }
    }
}

impl GoalState {
    fn new() -> Self {
        GoalState {
            status: GoalStatus::Executing,
            ended: false,
            lost: None,
            end: None,
            result_waker: None,
            feedback: VecDeque::new(),
            feedback_dropped: 0,
            feedback_wanted: true,
            feedback_waker: None,
            cancels: VecDeque::new(),
            cancels_in_flight: HashMap::new(),
            next_cancel: 0,
            cancels_taken: true,
            cancel_waker: None,
        }
    }

    /// End the goal as `status` with `end`: the result future, the feedback
    /// stream (once drained) and the server's cancel stream finish, and the
    /// cancel requests the server has not taken are answered with
    /// `refusal`. `in_flight`: those it took as well (a lost goal: nobody
    /// will answer them).
    fn end(&mut self, status: GoalStatus, end: End, refusal: &BusError, in_flight: bool) -> Wakeups {
        self.status = status;
        self.ended = true;
        self.end = Some(end);
        let mut answers: Vec<_> = self.cancels.drain(..).map(|reply| (reply, Err(refusal.clone()))).collect();
        if in_flight {
            answers.extend(self.cancels_in_flight.drain().map(|(_, reply)| (reply, Err(refusal.clone()))));
        }
        let wakers = [self.result_waker.take(), self.feedback_waker.take(), self.cancel_waker.take()];
        Wakeups { answers, wakers: wakers.into_iter().flatten().collect() }
    }
}

/// Store `waker` in `slot` unless it is there already; the waker it
/// replaces is returned, to be dropped with no lock held.
fn register(slot: &mut Option<Waker>, waker: &Waker) -> Option<Waker> {
    match slot {
        Some(old) if old.will_wake(waker) => None,
        _ => slot.replace(waker.clone()),
    }
}

impl Goal {
    fn new(entry: Arc<Entry>, id: GoalId) -> Self {
        Goal { id, entry, state: Mutex::new(GoalState::new()) }
    }

    fn terminated(&self) -> BusError {
        BusError::GoalTerminated { name: self.entry.name.clone() }
    }

    fn cancel_rejected(&self) -> BusError {
        BusError::CancelRejected { name: self.entry.name.clone() }
    }

    fn no_response(&self) -> BusError {
        BusError::NoResponse { name: self.entry.name.clone() }
    }

    /// The goal ends without a result, for `why`; nothing happens if it has
    /// ended already.
    fn lose(&self, why: BusError) -> Wakeups {
        let mut state = lock(&self.state);
        if state.ended {
            return Wakeups::default();
        }
        state.lost = Some(why.clone());
        self.entry.active.fetch_sub(1, Ordering::Relaxed);
        state.end(GoalStatus::Unknown, Err(why.clone()), &why, true)
    }

    /// A client's cancel request. Answered at once when the goal has ended,
    /// when it is canceling already (rcl_action answers that as terminated,
    /// without asking its server) or when its server takes no cancel
    /// requests; queued for the server otherwise.
    fn request_cancel(&self) -> oneshot::Receiver<CancelAnswer> {
        let (reply, answer) = oneshot::channel();
        let (now, waker) = {
            let mut state = lock(&self.state);
            if let Some(lost) = &state.lost {
                (Some((reply, Err(lost.clone()))), None)
            } else if state.ended || state.status == GoalStatus::Canceling {
                (Some((reply, Err(self.terminated()))), None)
            } else if !state.cancels_taken {
                (Some((reply, Err(self.cancel_rejected()))), None)
            } else {
                state.cancels.push_back(reply);
                (None, state.cancel_waker.take())
            }
        };
        // Nobody waits on `answer` yet: this send wakes no task.
        if let Some((reply, answer)) = now {
            let _ = reply.send(answer);
        }
        if let Some(waker) = waker {
            waker.wake();
        }
        answer
    }

    /// Nobody will read the goal's feedback: drop what is queued, and what
    /// comes from now on.
    fn unwant_feedback(&self) {
        let (queued, waker) = {
            let mut state = lock(&self.state);
            state.feedback_wanted = false;
            (std::mem::take(&mut state.feedback), state.feedback_waker.take())
        };
        let mut user = UserCode::default();
        user.run(|| drop(waker));
        for msg in queued {
            user.run(|| drop(msg));
        }
        finish_in_drop(user);
    }
}

/// The server side of one action name: a stream of goal requests (r2r's
/// `create_action_server` stream).
///
/// Dropping it stops taking goals: the name is free for a new server, and
/// the requests still queued fail with `ServiceUnavailable`. The requests it
/// handed out and the goals they became are not ended by it: they end with
/// their own handles or the scope.
pub struct ActionServer<A: Action> {
    entry: Arc<Entry>,
    server: Arc<Server>,
    _t: PhantomData<fn() -> A>,
}

impl<A: Action> ActionServer<A> {
    pub fn name(&self) -> &str {
        &self.entry.name
    }

    /// The next goal request; `None` once the scope is closed.
    pub async fn recv(&mut self) -> Option<ActionServerGoalRequest<A>> {
        self.next().await
    }
}

impl<A: Action> Stream for ActionServer<A> {
    type Item = ActionServerGoalRequest<A>;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<ActionServerGoalRequest<A>>> {
        let this = self.get_mut();
        let Some((id, goal)) = ready!(this.server.poll_request(cx)) else {
            return Poll::Ready(None);
        };
        let responder = GoalResponder { entry: this.entry.clone(), server: this.server.clone(), id: Some(id) };
        Poll::Ready(Some(ActionServerGoalRequest { id, goal: unerase(goal), responder, _t: PhantomData }))
    }
}

impl<A: Action> Drop for ActionServer<A> {
    fn drop(&mut self) {
        self.server.stop(&self.entry);
    }
}

impl<A: Action> fmt::Debug for ActionServer<A> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ActionServer").field("name", &self.entry.name).field("scope", &self.server.scope).finish()
    }
}

/// A goal request, as the server sees it (r2r's `ActionServerGoalRequest`):
/// accept or reject it. Dropping it unanswered fails the client's request
/// with `NoResponse`.
pub struct ActionServerGoalRequest<A: Action> {
    pub id: GoalId,
    pub goal: A::Goal,
    responder: GoalResponder,
    _t: PhantomData<fn() -> A>,
}

impl<A: Action> ActionServerGoalRequest<A> {
    /// Accept the goal: it is `Executing` from now on, and its client gets
    /// its handle, result future and feedback stream. Returns the server's
    /// handle of the goal and the stream of its cancel requests.
    ///
    /// Works also when the client gave up waiting (the goal then runs with
    /// nobody to read its feedback or result), and after the `ActionServer`
    /// was dropped. `Closed` once the server's scope has closed: the client
    /// got `ServiceUnavailable` already.
    pub fn accept(self) -> Result<(ActionServerGoal<A>, CancelRequests), BusError> {
        let ActionServerGoalRequest { id, goal, mut responder, .. } = self;
        let (handle, shared) = responder.accept()?;
        Ok((ActionServerGoal { id, goal, handle, _t: PhantomData }, CancelRequests { goal: shared }))
    }

    /// Refuse the goal: the client gets `GoalRejected`. `Ok` even if nobody
    /// waits for the answer any more; `Closed` once the server's scope has
    /// closed.
    pub fn reject(self) -> Result<(), BusError> {
        let mut responder = self.responder;
        let rejected = BusError::GoalRejected { name: responder.entry.name.clone() };
        responder.answer(Err(rejected))
    }
}

impl<A: Action> fmt::Debug for ActionServerGoalRequest<A>
where
    A::Goal: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ActionServerGoalRequest")
            .field("action", &self.responder.entry.name)
            .field("id", &self.id)
            .field("goal", &self.goal)
            .finish_non_exhaustive()
    }
}

/// The answering half of an `ActionServerGoalRequest`, kept apart so that
/// `goal` can still be moved out of a request.
struct GoalResponder {
    entry: Arc<Entry>,
    server: Arc<Server>,
    /// The request's goal id; `None` once answered.
    id: Option<GoalId>,
}

impl GoalResponder {
    fn accept(&mut self) -> Result<(Arc<ServerGoal>, Arc<Goal>), BusError> {
        let id = self.id.take().expect("a goal request is answered once");
        let goal = Arc::new(Goal::new(self.entry.clone(), id));
        let Ok(reply) = self.server.adopt(&goal) else {
            return Err(closed(&self.server.scope));
        };
        let handle = Arc::new(ServerGoal { goal: goal.clone(), server: self.server.clone() });
        let delivered = match reply {
            // The send fails only when the client gave up waiting.
            Some(reply) => reply.send(Ok(goal.clone())).is_ok(),
            None => false,
        };
        if !delivered {
            goal.unwant_feedback();
        }
        Ok((handle, goal))
    }

    fn answer(&mut self, answer: GoalAnswer) -> Result<(), BusError> {
        let Some(id) = self.id.take() else {
            return Ok(());
        };
        match self.server.take_in_flight(id) {
            // The send fails only when the client gave up waiting.
            (Some(reply), _) => {
                let _ = reply.send(answer);
                Ok(())
            }
            // The scope's close failed the request already.
            (None, true) => Err(closed(&self.server.scope)),
            (None, false) => Ok(()),
        }
    }
}

impl Drop for GoalResponder {
    fn drop(&mut self) {
        if self.id.is_some() {
            let _ = self.answer(Err(BusError::NoResponse { name: self.entry.name.clone() }));
        }
    }
}

/// The server's handle of an accepted goal (r2r's `ActionServerGoal`): it
/// publishes feedback and ends the goal with `succeed`, `abort` or `cancel`.
/// Clones are the same handle.
///
/// The first of the three ends the goal with that status and result, from
/// `Executing` or `Canceling`; a later call fails with `GoalTerminated`.
/// Dropping the last clone before the goal ended loses it: its client gets
/// `NoResponse` (README.md, "Actions").
pub struct ActionServerGoal<A: Action> {
    pub id: GoalId,
    pub goal: A::Goal,
    handle: Arc<ServerGoal>,
    _t: PhantomData<fn() -> A>,
}

impl<A: Action> Clone for ActionServerGoal<A> {
    fn clone(&self) -> Self {
        ActionServerGoal { id: self.id, goal: self.goal.clone(), handle: self.handle.clone(), _t: PhantomData }
    }
}

impl<A: Action> ActionServerGoal<A> {
    /// Whether a cancel request was accepted and the goal has not ended.
    /// `Closed` once the server's scope has closed.
    pub fn is_cancelling(&self) -> Result<bool, BusError> {
        self.handle.is_cancelling()
    }

    /// Queue `msg` for the client's feedback stream. Dropped, with `Ok`,
    /// when no client reads it. `GoalTerminated` once the goal has ended,
    /// `Closed` once the server's scope has closed.
    pub fn publish_feedback(&self, msg: A::Feedback) -> Result<(), BusError> {
        self.handle.feedback(Arc::new(msg))
    }

    /// End the goal `Succeeded` with `msg` (see the type's docs).
    pub fn succeed(&mut self, msg: A::Result) -> Result<(), BusError> {
        self.handle.finish(GoalStatus::Succeeded, Arc::new(msg))
    }

    /// End the goal `Aborted` with `msg`.
    pub fn abort(&mut self, msg: A::Result) -> Result<(), BusError> {
        self.handle.finish(GoalStatus::Aborted, Arc::new(msg))
    }

    /// End the goal `Canceled` with `msg`, whether or not a cancel request
    /// was accepted (as r2r allows; rcl_action reaches CANCELED only from
    /// CANCELING).
    pub fn cancel(&mut self, msg: A::Result) -> Result<(), BusError> {
        self.handle.finish(GoalStatus::Canceled, Arc::new(msg))
    }
}

impl<A: Action> fmt::Debug for ActionServerGoal<A> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ActionServerGoal")
            .field("action", &self.handle.goal.entry.name)
            .field("id", &self.id)
            .finish_non_exhaustive()
    }
}

/// What the clones of one `ActionServerGoal` share.
struct ServerGoal {
    goal: Arc<Goal>,
    server: Arc<Server>,
}

impl ServerGoal {
    /// Refused once the goal has ended: `Closed` if its server's scope lost
    /// it (the only way it can be lost while a handle lives), else
    /// `GoalTerminated`.
    fn refusal(&self, state: &GoalState) -> BusError {
        match state.lost {
            Some(_) => closed(&self.server.scope),
            None => self.goal.terminated(),
        }
    }

    fn is_cancelling(&self) -> Result<bool, BusError> {
        let state = lock(&self.goal.state);
        match state.lost {
            Some(_) => Err(closed(&self.server.scope)),
            None => Ok(state.status == GoalStatus::Canceling),
        }
    }

    fn feedback(&self, msg: AnyMsg) -> Result<(), BusError> {
        let mut msg = Some(msg);
        let (outcome, waker, pushed_out) = {
            let mut state = lock(&self.goal.state);
            if state.ended {
                (Err(self.refusal(&state)), None, None)
            } else if !state.feedback_wanted {
                (Ok(()), None, None)
            } else {
                let pushed_out = if state.feedback.len() >= FEEDBACK_DEPTH {
                    state.feedback_dropped += 1;
                    state.feedback.pop_front()
                } else {
                    None
                };
                state.feedback.extend(msg.take());
                (Ok(()), state.feedback_waker.take(), pushed_out)
            }
        };
        // With no lock held: the client's wake-up, then what goes (a
        // message pushed out of the full queue, or one nobody reads).
        let mut user = UserCode::default();
        if let Some(waker) = waker {
            user.wake(waker);
        }
        user.run(|| drop(pushed_out));
        user.run(|| drop(msg));
        user.finish();
        outcome
    }

    fn finish(&self, status: GoalStatus, result: AnyMsg) -> Result<(), BusError> {
        let mut result = Some(result);
        let (outcome, wakeups) = {
            let mut state = lock(&self.goal.state);
            if state.ended {
                (Err(self.refusal(&state)), Wakeups::default())
            } else {
                self.goal.entry.active.fetch_sub(1, Ordering::Relaxed);
                let end = Ok((status, result.take().expect("not taken yet")));
                // Cancel requests the server took and has not answered stay
                // its to answer; an accept now tells the client the goal
                // has ended.
                (Ok(()), state.end(status, end, &self.goal.terminated(), false))
            }
        };
        if outcome.is_ok() {
            self.server.forget(self.goal.id);
        }
        let mut user = UserCode::default();
        wakeups.run(&mut user);
        // Refused: the result is freed here, with no lock held.
        user.run(|| drop(result));
        user.finish();
        outcome
    }
}

impl Drop for ServerGoal {
    /// The last handle is gone: a goal that has not ended is lost, and its
    /// client gets `NoResponse`.
    fn drop(&mut self) {
        let wakeups = self.goal.lose(self.goal.no_response());
        self.server.forget(self.goal.id);
        let mut user = UserCode::default();
        wakeups.run(&mut user);
        finish_in_drop(user);
    }
}

/// The cancel requests of one goal, as its server sees them (the stream
/// r2r's `accept` returns). It ends when the goal ends.
///
/// Dropping it refuses cancel requests from then on: the client gets
/// `CancelRejected` (r2r would let rcl_action report them accepted without
/// asking the server).
pub struct CancelRequests {
    goal: Arc<Goal>,
}

impl CancelRequests {
    /// The next cancel request; `None` once the goal has ended.
    pub async fn recv(&mut self) -> Option<ActionServerCancelRequest> {
        self.next().await
    }
}

impl Stream for CancelRequests {
    type Item = ActionServerCancelRequest;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<ActionServerCancelRequest>> {
        let this = self.get_mut();
        let mut state = lock(&this.goal.state);
        if let Some(reply) = state.cancels.pop_front() {
            state.next_cancel += 1;
            let key = state.next_cancel;
            state.cancels_in_flight.insert(key, reply);
            drop(state);
            let responder = CancelResponder { goal: this.goal.clone(), key: Some(key) };
            return Poll::Ready(Some(ActionServerCancelRequest { id: this.goal.id, responder }));
        }
        if state.ended {
            return Poll::Ready(None);
        }
        let replaced = register(&mut state.cancel_waker, cx.waker());
        drop(state);
        drop(replaced);
        Poll::Pending
    }
}

impl Drop for CancelRequests {
    fn drop(&mut self) {
        let (queued, waker) = {
            let mut state = lock(&self.goal.state);
            state.cancels_taken = false;
            (std::mem::take(&mut state.cancels), state.cancel_waker.take())
        };
        let mut user = UserCode::default();
        for reply in queued {
            user.run(|| {
                let _ = reply.send(Err(self.goal.cancel_rejected()));
            });
        }
        user.run(|| drop(waker));
        finish_in_drop(user);
    }
}

impl fmt::Debug for CancelRequests {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CancelRequests").field("action", &self.goal.entry.name).field("goal", &self.goal.id).finish()
    }
}

/// One cancel request, as the server sees it (r2r's
/// `ActionServerCancelRequest`). Dropping it unanswered fails the client's
/// request with `NoResponse`.
pub struct ActionServerCancelRequest {
    pub id: GoalId,
    responder: CancelResponder,
}

impl ActionServerCancelRequest {
    /// Accept: the goal is `Canceling` from now on, and the client's cancel
    /// resolves `Ok`. The server still ends the goal itself, with `cancel`
    /// (or `succeed` / `abort`). If the goal has ended meanwhile, the client
    /// gets `GoalTerminated` instead.
    pub fn accept(self) {
        let mut responder = self.responder;
        responder.answer(Some(true));
    }

    /// Refuse: the client gets `CancelRejected`, and the goal runs on.
    pub fn reject(self) {
        let mut responder = self.responder;
        responder.answer(Some(false));
    }
}

impl fmt::Debug for ActionServerCancelRequest {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ActionServerCancelRequest")
            .field("action", &self.responder.goal.entry.name)
            .field("id", &self.id)
            .finish_non_exhaustive()
    }
}

struct CancelResponder {
    goal: Arc<Goal>,
    /// The request's key in the goal's `cancels_in_flight`; `None` once
    /// answered.
    key: Option<u64>,
}

impl CancelResponder {
    /// `None`: dropped unanswered.
    fn answer(&mut self, accepted: Option<bool>) {
        let Some(key) = self.key.take() else {
            return;
        };
        let answered = {
            let mut state = lock(&self.goal.state);
            // `None`: the goal was lost, which answered the request.
            match state.cancels_in_flight.remove(&key) {
                Some(reply) => {
                    let answer = match accepted {
                        Some(true) if state.ended => Err(self.goal.terminated()),
                        Some(true) => {
                            state.status = GoalStatus::Canceling;
                            Ok(())
                        }
                        Some(false) => Err(self.goal.cancel_rejected()),
                        None => Err(self.goal.no_response()),
                    };
                    Some((reply, answer))
                }
                None => None,
            }
        };
        if let Some((reply, answer)) = answered {
            let _ = reply.send(answer);
        }
    }
}

impl Drop for CancelResponder {
    fn drop(&mut self) {
        if self.key.is_some() {
            self.answer(None);
        }
    }
}

/// A client of one action name (r2r's `ActionClient`). Clones are the same
/// client (one entry in [`crate::Bus::actions`], gone with the last clone);
/// a goal does not need the client any more.
pub struct ActionClient<A: Action> {
    inner: Arc<ClientInner>,
    _t: PhantomData<fn() -> A>,
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

impl<A: Action> Clone for ActionClient<A> {
    fn clone(&self) -> Self {
        ActionClient { inner: self.inner.clone(), _t: PhantomData }
    }
}

impl<A: Action> ActionClient<A> {
    pub fn name(&self) -> &str {
        &self.inner.entry.name
    }

    /// Send a goal request and get the future of the server's answer: r2r's
    /// `client.send_goal_request(goal)?.await?`. Outer error: `Closed`, or
    /// `ServiceUnavailable` when no server is live now.
    pub fn send_goal_request(&self, goal: A::Goal) -> Result<GoalResponseFuture<A>, BusError> {
        let ClientInner { entry, caller, .. } = &*self.inner;
        if caller.is_closed() {
            return Err(caller.closed());
        }
        let server = lock(&entry.state).server.clone().ok_or_else(|| entry.unavailable())?;
        let id = GoalId(entry.next_goal.fetch_add(1, Ordering::Relaxed) + 1);
        let (reply, answer) = oneshot::channel();
        if let Err(refused) = server.push(PendingGoal { id, goal: Arc::new(goal), reply }) {
            // It stopped since we looked; the goal goes here, with no lock
            // held.
            drop(refused);
            return Err(entry.unavailable());
        }
        entry.sent.fetch_add(1, Ordering::Relaxed);
        Ok(GoalResponseFuture { entry: entry.clone(), answer, caller: caller.clone(), watch: None, _t: PhantomData })
    }

    /// Whether a server is live now. Says nothing about the client's own
    /// scope: once that has closed, `send_goal_request` fails with `Closed`
    /// anyway.
    pub fn is_available(&self) -> bool {
        lock(&self.inner.entry.state).server.is_some()
    }

    /// Resolve once a server is live (at once if one is): r2r's
    /// `Node::is_available(&client)`. `Closed` if the client's scope closes
    /// first. The future holds the client, as r2r's does, so it is
    /// `'static` and can outlive this handle.
    pub fn wait_for_server(&self) -> impl Future<Output = Result<(), BusError>> + Send + Unpin + 'static {
        WaitForServer { client: self.inner.clone(), served: true, closing: None, waiting: None }
    }

    /// Resolve once no server is live (at once if none is): `is_available`
    /// turning false, for one who follows the server's presence (the
    /// bridge's DDS action server). A server replaced before the waiting
    /// task runs again is not seen to go. `Closed` if the client's scope
    /// closes first.
    pub fn wait_for_no_server(&self) -> impl Future<Output = Result<(), BusError>> + Send + Unpin + 'static {
        WaitForServer { client: self.inner.clone(), served: false, closing: None, waiting: None }
    }
}

impl<A: Action> fmt::Debug for ActionClient<A> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let ClientInner { entry, caller, .. } = &*self.inner;
        f.debug_struct("ActionClient").field("name", &entry.name).field("scope", &caller.scope).finish()
    }
}

/// `ActionClient::wait_for_server` and `ActionClient::wait_for_no_server`:
/// `WaitForService` of services, for the same reason (a wait given up takes
/// its wakers back out).
struct WaitForServer {
    client: Arc<ClientInner>,
    /// What it waits for: a live server (`wait_for_server`), or none.
    served: bool,
    closing: Option<u64>,
    waiting: Option<u64>,
}

impl WaitForServer {
    /// The tasks waiting for what this one waits for.
    fn set<'s>(&self, state: &'s mut EntryState) -> &'s mut WakerSet {
        if self.served {
            &mut state.waiting
        } else {
            &mut state.leaving
        }
    }
}

impl Future for WaitForServer {
    type Output = Result<(), BusError>;

    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = &mut *self;
        let client = &*this.client;
        if !client.caller.watch(&mut this.closing, cx.waker()) {
            return Poll::Ready(Err(client.caller.closed()));
        }
        let mut state = lock(&client.entry.state);
        if state.server.is_some() == this.served {
            return Poll::Ready(Ok(()));
        }
        let mut key = this.waiting;
        let replaced = this.set(&mut state).register(&mut key, cx.waker());
        this.waiting = key;
        drop(state);
        drop(replaced);
        Poll::Pending
    }
}

impl Drop for WaitForServer {
    fn drop(&mut self) {
        self.client.caller.unwatch(&mut self.closing);
        if let Some(key) = self.waiting.take() {
            let mut state = lock(&self.client.entry.state);
            let removed = self.set(&mut state).remove(key);
            drop(state);
            drop(removed);
        }
    }
}

/// The server's answer to a goal request: the goal's handle, its result
/// future and its feedback stream once accepted, as r2r's goal future gives
/// them. Else `GoalRejected`, `NoResponse` (the request was dropped
/// unanswered), `ServiceUnavailable` (the server ended before answering) or
/// `Closed` (the client's scope closed).
///
/// Dropping it gives the request up, as dropping r2r's future does: the
/// server still gets it, and if it accepts it the goal runs with nobody to
/// read its feedback and result.
pub struct GoalResponseFuture<A: Action> {
    entry: Arc<Entry>,
    answer: oneshot::Receiver<GoalAnswer>,
    /// The client's scope's close signal, and this future's key in it.
    caller: Arc<CloseSignal>,
    watch: Option<u64>,
    _t: PhantomData<fn() -> A>,
}

impl<A: Action> Future for GoalResponseFuture<A> {
    type Output = Result<(ActionClientGoal<A>, ResultFuture<A>, FeedbackStream<A>), BusError>;

    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = &mut *self;
        // As a service call: once the scope has closed, whatever the server
        // says no longer matters (an accepted goal is let go by the drop).
        if !this.caller.watch(&mut this.watch, cx.waker()) {
            return Poll::Ready(Err(this.caller.closed()));
        }
        let answer = ready!(Pin::new(&mut this.answer).poll(cx));
        this.caller.unwatch(&mut this.watch);
        let goal = match answer {
            Ok(Ok(goal)) => goal,
            Ok(Err(e)) => return Poll::Ready(Err(e)),
            // The server side went away without a word.
            Err(oneshot::Canceled) => return Poll::Ready(Err(this.entry.unavailable())),
        };
        let caller = &this.caller;
        let handle = ActionClientGoal { id: goal.id, goal: goal.clone(), caller: caller.clone(), _t: PhantomData };
        let result = ResultFuture { goal: goal.clone(), caller: caller.clone(), watch: None, done: false, _t: PhantomData };
        let feedback = FeedbackStream { goal, caller: caller.clone(), watch: None, _t: PhantomData };
        Poll::Ready(Ok((handle, result, feedback)))
    }
}

impl<A: Action> Drop for GoalResponseFuture<A> {
    fn drop(&mut self) {
        self.caller.unwatch(&mut self.watch);
        // An accepted goal whose answer nobody read: its feedback goes. Closed
        // first, so that an acceptance racing this drop either comes in now
        // or finds the channel closed, and drops the feedback itself.
        self.answer.close();
        if let Ok(Some(Ok(goal))) = self.answer.try_recv() {
            goal.unwant_feedback();
        }
    }
}

impl<A: Action> fmt::Debug for GoalResponseFuture<A> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("GoalResponseFuture").field("action", &self.entry.name).finish_non_exhaustive()
    }
}

/// The client's handle of an accepted goal (r2r's `ActionClientGoal`).
/// Clones are the same handle. Dropping every client handle of a goal
/// leaves the goal to run; nothing tells its server.
pub struct ActionClientGoal<A: Action> {
    pub id: GoalId,
    goal: Arc<Goal>,
    caller: Arc<CloseSignal>,
    _t: PhantomData<fn() -> A>,
}

impl<A: Action> Clone for ActionClientGoal<A> {
    fn clone(&self) -> Self {
        ActionClientGoal { id: self.id, goal: self.goal.clone(), caller: self.caller.clone(), _t: PhantomData }
    }
}

impl<A: Action> ActionClientGoal<A> {
    /// The goal's status now: `Executing`, `Canceling`, the terminal status
    /// once it has ended, `Unknown` once it was lost. `Closed` once the
    /// client's scope has closed.
    pub fn get_status(&self) -> Result<GoalStatus, BusError> {
        if self.caller.is_closed() {
            return Err(self.caller.closed());
        }
        Ok(lock(&self.goal.state).status)
    }

    /// Ask the server to cancel the goal: r2r's `goal.cancel()?.await`. The
    /// future resolves `Ok` once the server accepted, else with
    /// `CancelRejected`, `GoalTerminated` (the goal has ended, or is
    /// canceling already), `NoResponse` (the request was dropped
    /// unanswered), `ServiceUnavailable` (the goal was lost with its
    /// server) or `Closed`. Outer error: `Closed`.
    pub fn cancel(&self) -> Result<CancelFuture, BusError> {
        if self.caller.is_closed() {
            return Err(self.caller.closed());
        }
        let answer = self.goal.request_cancel();
        Ok(CancelFuture { entry: self.goal.entry.clone(), answer, caller: self.caller.clone(), watch: None })
    }
}

impl<A: Action> fmt::Debug for ActionClientGoal<A> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ActionClientGoal")
            .field("action", &self.goal.entry.name)
            .field("id", &self.id)
            .finish_non_exhaustive()
    }
}

/// A goal's result (r2r's result future): `(status, result)` with a
/// terminal status once the server ended the goal. `ServiceUnavailable` if
/// the server's scope closed first, `NoResponse` if the server dropped the
/// goal without ending it, and `Closed` once the client's scope has closed,
/// whatever the server did.
pub struct ResultFuture<A: Action> {
    goal: Arc<Goal>,
    caller: Arc<CloseSignal>,
    watch: Option<u64>,
    done: bool,
    _t: PhantomData<fn() -> A>,
}

impl<A: Action> Future for ResultFuture<A> {
    type Output = Result<(GoalStatus, A::Result), BusError>;

    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = &mut *self;
        assert!(!this.done, "ResultFuture polled after it completed");
        if !this.caller.watch(&mut this.watch, cx.waker()) {
            this.done = true;
            return Poll::Ready(Err(this.caller.closed()));
        }
        let (end, replaced) = {
            let mut state = lock(&this.goal.state);
            match state.end.take() {
                Some(end) => (Some(end), None),
                None => (None, register(&mut state.result_waker, cx.waker())),
            }
        };
        drop(replaced);
        let Some(end) = end else {
            return Poll::Pending;
        };
        this.caller.unwatch(&mut this.watch);
        this.done = true;
        Poll::Ready(end.map(|(status, result)| (status, unerase(result))))
    }
}

impl<A: Action> Drop for ResultFuture<A> {
    fn drop(&mut self) {
        self.caller.unwatch(&mut self.watch);
        let waker = lock(&self.goal.state).result_waker.take();
        drop(waker);
    }
}

impl<A: Action> fmt::Debug for ResultFuture<A> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ResultFuture")
            .field("action", &self.goal.entry.name)
            .field("goal", &self.goal.id)
            .finish_non_exhaustive()
    }
}

/// A goal's feedback, in the order its server published it (r2r's feedback
/// stream). It keeps the newest 10 messages; when it is full the oldest is
/// dropped, as a DDS reader of rcl_action's feedback topic drops it. It ends
/// once the goal has ended or was lost and what was queued before is read,
/// and at once when the client's scope closes.
///
/// Dropping it drops what is queued, and the server's later feedback.
pub struct FeedbackStream<A: Action> {
    goal: Arc<Goal>,
    caller: Arc<CloseSignal>,
    watch: Option<u64>,
    _t: PhantomData<fn() -> A>,
}

impl<A: Action> FeedbackStream<A> {
    /// Feedback lost to the full queue.
    pub fn dropped(&self) -> u64 {
        lock(&self.goal.state).feedback_dropped
    }
}

impl<A: Action> Stream for FeedbackStream<A> {
    type Item = A::Feedback;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<A::Feedback>> {
        let this = self.get_mut();
        if !this.caller.watch(&mut this.watch, cx.waker()) {
            this.goal.unwant_feedback();
            return Poll::Ready(None);
        }
        let mut state = lock(&this.goal.state);
        if let Some(msg) = state.feedback.pop_front() {
            drop(state);
            return Poll::Ready(Some(unerase(msg)));
        }
        if state.ended {
            drop(state);
            this.caller.unwatch(&mut this.watch);
            return Poll::Ready(None);
        }
        let replaced = register(&mut state.feedback_waker, cx.waker());
        drop(state);
        drop(replaced);
        Poll::Pending
    }
}

impl<A: Action> Drop for FeedbackStream<A> {
    fn drop(&mut self) {
        self.caller.unwatch(&mut self.watch);
        self.goal.unwant_feedback();
    }
}

impl<A: Action> fmt::Debug for FeedbackStream<A> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("FeedbackStream")
            .field("action", &self.goal.entry.name)
            .field("goal", &self.goal.id)
            .finish_non_exhaustive()
    }
}

/// The server's answer to a cancel request (see `ActionClientGoal::cancel`).
/// Dropping it gives the request up: the server still gets it.
pub struct CancelFuture {
    entry: Arc<Entry>,
    answer: oneshot::Receiver<CancelAnswer>,
    caller: Arc<CloseSignal>,
    watch: Option<u64>,
}

impl Future for CancelFuture {
    type Output = Result<(), BusError>;

    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = &mut *self;
        if !this.caller.watch(&mut this.watch, cx.waker()) {
            return Poll::Ready(Err(this.caller.closed()));
        }
        let answer = ready!(Pin::new(&mut this.answer).poll(cx));
        this.caller.unwatch(&mut this.watch);
        Poll::Ready(answer.unwrap_or_else(|_| Err(this.entry.unavailable())))
    }
}

impl Drop for CancelFuture {
    fn drop(&mut self) {
        self.caller.unwatch(&mut self.watch);
    }
}

impl fmt::Debug for CancelFuture {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CancelFuture").field("action", &self.entry.name).finish_non_exhaustive()
    }
}

/// A message back in its own type: moved out of the `Arc` when nothing else
/// holds it (always so for the bus's own goals, results and feedback), else
/// cloned.
fn unerase<T: Message>(msg: AnyMsg) -> T {
    let msg = msg.downcast::<T>().expect("an action's types are fixed, and every endpoint is checked against them");
    Arc::unwrap_or_clone(msg)
}

#[cfg(test)]
mod tests {
    use std::cell::RefCell;
    use std::task::Wake;

    use serde::{Deserialize, Serialize};

    use super::*;
    use crate::Bus;

    type Act = (u32, u32, u32);

    fn noop_cx() -> Context<'static> {
        Context::from_waker(futures::task::noop_waker_ref())
    }

    /// The locks that one goal's endpoints take.
    #[derive(Clone)]
    struct Locks {
        entry: Arc<Entry>,
        server: Arc<Server>,
        goal: Option<Arc<Goal>>,
        caller: Arc<CloseSignal>,
    }

    impl Locks {
        fn of<A: Action>(srv: &ActionServer<A>, client: &ActionClient<A>) -> Locks {
            Locks { entry: srv.entry.clone(), server: srv.server.clone(), goal: None, caller: client.inner.caller.clone() }
        }

        fn with_goal(&self, goal: &Arc<Goal>) -> Locks {
            Locks { goal: Some(goal.clone()), ..self.clone() }
        }

        fn free(&self) -> bool {
            self.entry.state.try_lock().is_ok()
                && self.server.state.try_lock().is_ok()
                && self.goal.as_ref().is_none_or(|goal| goal.state.try_lock().is_ok())
                && !self.caller.is_locked()
        }
    }

    /// A waker that records, each time it is woken and when its last clone
    /// drops, whether every lock was free at that moment.
    struct Probe {
        locks: Locks,
        seen: Arc<Mutex<Vec<(&'static str, bool)>>>,
    }

    impl Wake for Probe {
        fn wake(self: Arc<Self>) {
            self.wake_by_ref();
        }

        fn wake_by_ref(self: &Arc<Self>) {
            self.seen.lock().unwrap().push(("wake", self.locks.free()));
        }
    }

    impl Drop for Probe {
        fn drop(&mut self) {
            self.seen.lock().unwrap().push(("drop", self.locks.free()));
        }
    }

    /// Every event `seen` recorded since the last look, all with the locks
    /// free, and at least one of `kind`.
    fn all_free(seen: &Mutex<Vec<(&'static str, bool)>>, kind: &str, what: &str) {
        let events = std::mem::take(&mut *seen.lock().unwrap());
        assert!(events.iter().any(|(k, _)| *k == kind), "{what}: no {kind} in {events:?}");
        assert!(events.iter().all(|(_, free)| *free), "{what}: {events:?}");
    }

    fn probe(locks: &Locks, seen: &Arc<Mutex<Vec<(&'static str, bool)>>>) -> Waker {
        Waker::from(Arc::new(Probe { locks: locks.clone(), seen: seen.clone() }))
    }

    type ServerSide = (ActionServerGoal<Act>, CancelRequests);
    type ClientSide = (ActionClientGoal<Act>, ResultFuture<Act>, FeedbackStream<Act>);

    /// Accept the next goal of `srv`, sent through `client`, and return both
    /// ends.
    fn accepted(srv: &mut ActionServer<Act>, client: &ActionClient<Act>, goal: u32) -> (ServerSide, ClientSide) {
        let mut response = client.send_goal_request(goal).unwrap();
        let Poll::Ready(Some(request)) = Pin::new(&mut *srv).poll_next(&mut noop_cx()) else {
            panic!("the request is not handed out");
        };
        let server_side = request.accept().unwrap();
        let Poll::Ready(Ok(client_side)) = Pin::new(&mut response).poll(&mut noop_cx()) else {
            panic!("the acceptance does not reach the client");
        };
        (server_side, client_side)
    }

    #[test]
    fn wakers_are_woken_and_dropped_with_no_lock_held() {
        let bus = Bus::new();
        let (serving, calling) = (bus.scope("server"), bus.scope("caller"));
        let mut srv = serving.action_server::<Act>("/a").unwrap();
        let client = calling.action_client::<Act>("/a").unwrap();
        let locks = Locks::of(&srv, &client);
        let seen = Arc::new(Mutex::new(Vec::new()));
        let mut noop = noop_cx();

        // the waiting server: its waker replaced by the next poll, then woken
        // by a request
        assert!(Pin::new(&mut srv).poll_next(&mut Context::from_waker(&probe(&locks, &seen))).is_pending());
        assert!(Pin::new(&mut srv).poll_next(&mut noop).is_pending());
        all_free(&seen, "drop", "the server's waker, replaced");
        assert!(Pin::new(&mut srv).poll_next(&mut Context::from_waker(&probe(&locks, &seen))).is_pending());
        let mut response = client.send_goal_request(1).unwrap();
        all_free(&seen, "wake", "the server's waker, woken by a request");

        // the client's response future, woken by the acceptance
        assert!(Pin::new(&mut response).poll(&mut Context::from_waker(&probe(&locks, &seen))).is_pending());
        let Poll::Ready(Some(request)) = Pin::new(&mut srv).poll_next(&mut noop) else { panic!("no request") };
        let (mut server_goal, mut cancels) = request.accept().unwrap();
        all_free(&seen, "wake", "the response future, woken by the acceptance");
        let Poll::Ready(Ok((handle, mut result, mut feedback))) = Pin::new(&mut response).poll(&mut noop) else {
            panic!("not accepted")
        };
        let locks = locks.with_goal(&handle.goal);

        // the feedback stream, woken by feedback; the cancel stream, woken by
        // a cancel request
        assert!(Pin::new(&mut feedback).poll_next(&mut Context::from_waker(&probe(&locks, &seen))).is_pending());
        server_goal.publish_feedback(7).unwrap();
        all_free(&seen, "wake", "the feedback stream");
        assert!(Pin::new(&mut cancels).poll_next(&mut Context::from_waker(&probe(&locks, &seen))).is_pending());
        let mut canceling = handle.cancel().unwrap();
        all_free(&seen, "wake", "the cancel stream");
        let Poll::Ready(Some(cancel)) = Pin::new(&mut cancels).poll_next(&mut noop) else { panic!("no cancel") };
        assert!(Pin::new(&mut canceling).poll(&mut Context::from_waker(&probe(&locks, &seen))).is_pending());
        cancel.reject();
        all_free(&seen, "wake", "the cancel future, woken by the answer");
        assert_eq!(Pin::new(&mut canceling).poll(&mut noop), Poll::Ready(Err(BusError::CancelRejected { name: "/a".into() })));

        // the result future and both streams, woken by the end
        assert_eq!(Pin::new(&mut feedback).poll_next(&mut noop), Poll::Ready(Some(7)));
        let waker = probe(&locks, &seen);
        assert!(Pin::new(&mut result).poll(&mut Context::from_waker(&waker)).is_pending());
        assert!(Pin::new(&mut feedback).poll_next(&mut Context::from_waker(&waker)).is_pending());
        assert!(Pin::new(&mut cancels).poll_next(&mut Context::from_waker(&waker)).is_pending());
        drop(waker);
        server_goal.succeed(9).unwrap();
        all_free(&seen, "wake", "the end");
        assert_eq!(Pin::new(&mut result).poll(&mut noop), Poll::Ready(Ok((GoalStatus::Succeeded, 9))));

        // a wait for a server, woken by its registration
        let lonely = calling.action_client::<Act>("/b").unwrap();
        let mut wait = Box::pin(lonely.wait_for_server());
        let locks_b = Locks { entry: lonely.inner.entry.clone(), ..locks.clone() };
        assert!(wait.as_mut().poll(&mut Context::from_waker(&probe(&locks_b, &seen))).is_pending());
        let b = serving.action_server::<Act>("/b").unwrap();
        all_free(&seen, "wake", "wait_for_server");
        drop(wait);

        // a wait for no server, woken by the server's going
        let mut gone = Box::pin(lonely.wait_for_no_server());
        assert!(gone.as_mut().poll(&mut Context::from_waker(&probe(&locks_b, &seen))).is_pending());
        drop(b);
        all_free(&seen, "wake", "wait_for_no_server");
        drop(gone);

        // a goal of a closing client scope: its result future, woken through
        // the scope's close signal
        let ((_g2, _c2), (_h2, mut result2, _f2)) = accepted(&mut srv, &client, 2);
        assert!(Pin::new(&mut result2).poll(&mut Context::from_waker(&probe(&locks, &seen))).is_pending());
        calling.close();
        all_free(&seen, "wake", "the client scope's close");
        assert_eq!(Pin::new(&mut result2).poll(&mut noop), Poll::Ready(Err(BusError::Closed { scope: "caller".into() })));
    }

    #[test]
    fn a_servers_close_wakes_what_waits_on_its_goals_with_no_lock_held() {
        let bus = Bus::new();
        let (serving, calling) = (bus.scope("server"), bus.scope("caller"));
        let mut srv = serving.action_server::<Act>("/a").unwrap();
        let client = calling.action_client::<Act>("/a").unwrap();
        let seen = Arc::new(Mutex::new(Vec::new()));
        let ((_server_goal, mut cancels), (handle, mut result, mut feedback)) = accepted(&mut srv, &client, 1);
        let locks = Locks::of(&srv, &client).with_goal(&handle.goal);
        let mut queued = Box::pin(client.send_goal_request(2).unwrap());
        let mut canceling = handle.cancel().unwrap();
        let waker = probe(&locks, &seen);
        let mut cx = Context::from_waker(&waker);
        assert!(Pin::new(&mut result).poll(&mut cx).is_pending());
        assert!(Pin::new(&mut feedback).poll_next(&mut cx).is_pending());
        assert!(queued.as_mut().poll(&mut cx).is_pending());
        assert!(Pin::new(&mut canceling).poll(&mut cx).is_pending());
        drop(waker);
        let _ = &mut cancels;

        serving.close();
        all_free(&seen, "wake", "the server scope's close");
        let unavailable = BusError::ServiceUnavailable { name: "/a".into() };
        assert_eq!(Pin::new(&mut result).poll(&mut noop_cx()), Poll::Ready(Err(unavailable.clone())));
        assert_eq!(Pin::new(&mut canceling).poll(&mut noop_cx()), Poll::Ready(Err(unavailable.clone())));
        assert!(matches!(queued.as_mut().poll(&mut noop_cx()), Poll::Ready(Err(e)) if e == unavailable));
    }

    #[test]
    fn waits_and_futures_given_up_leave_no_waker_behind() {
        let bus = Bus::new();
        let (serving, calling) = (bus.scope("server"), bus.scope("caller"));
        let lonely = calling.action_client::<Act>("/nobody_serves").unwrap();
        let mut cx = noop_cx();
        for _ in 0..3 {
            let mut wait = Box::pin(lonely.wait_for_server());
            assert!(wait.as_mut().poll(&mut cx).is_pending());
        }
        assert!(lock(&lonely.inner.entry.state).waiting.is_empty());

        let mut srv = serving.action_server::<Act>("/a").unwrap();
        let client = calling.action_client::<Act>("/a").unwrap();
        for _ in 0..3 {
            let mut wait = Box::pin(client.wait_for_no_server());
            assert!(wait.as_mut().poll(&mut cx).is_pending());
        }
        assert!(lock(&client.inner.entry.state).leaving.is_empty());
        for i in 0..3 {
            let mut response = client.send_goal_request(i).unwrap();
            assert!(Pin::new(&mut response).poll(&mut cx).is_pending());
        }
        let ((_server_goal, _cancels), (handle, result, feedback)) = {
            // the three requests given up are still queued; take them first
            for _ in 0..3 {
                let Poll::Ready(Some(request)) = Pin::new(&mut srv).poll_next(&mut cx) else { panic!("queued") };
                request.reject().unwrap();
            }
            accepted(&mut srv, &client, 9)
        };
        let (mut result, mut feedback) = (Box::pin(result), Box::pin(feedback));
        assert!(result.as_mut().poll(&mut cx).is_pending());
        assert!(feedback.as_mut().poll_next(&mut cx).is_pending());
        let mut canceling = handle.cancel().unwrap();
        assert!(Pin::new(&mut canceling).poll(&mut cx).is_pending());
        drop((result, feedback, canceling));
        let state = lock(&handle.goal.state);
        assert!(state.result_waker.is_none() && state.feedback_waker.is_none(), "the goal's slots are empty");
        drop(state);
        assert!(!client.inner.caller.is_watched(), "and so is the scope's close signal");
    }

    thread_local! {
        static WATCHED: RefCell<Option<Locks>> = const { RefCell::new(None) };
        /// One entry per `Tracked` freed while watching: were the locks free?
        static FREED: RefCell<Vec<bool>> = const { RefCell::new(Vec::new()) };
    }

    /// A message with drop glue of its own (user code, as far as the bus
    /// knows).
    #[derive(Clone, Debug, Serialize, Deserialize)]
    struct Tracked(u32);

    impl Drop for Tracked {
        fn drop(&mut self) {
            WATCHED.with(|watched| {
                if let Some(locks) = &*watched.borrow() {
                    let free = locks.free();
                    FREED.with(|freed| freed.borrow_mut().push(free));
                }
            });
        }
    }

    type Tracking = (Tracked, Tracked, Tracked);

    /// Run `f` while watching `locks`, and how each `Tracked` freed meanwhile
    /// went.
    fn freed_during(locks: &Locks, f: impl FnOnce()) -> Vec<bool> {
        WATCHED.with(|w| *w.borrow_mut() = Some(locks.clone()));
        f();
        let unwatched = WATCHED.with(|w| w.borrow_mut().take());
        drop(unwatched);
        FREED.with(|f| f.take())
    }

    #[test]
    fn messages_are_freed_with_no_lock_held() {
        let bus = Bus::new();
        let (serving, calling) = (bus.scope("server"), bus.scope("caller"));
        let mut srv = serving.action_server::<Tracking>("/a").unwrap();
        let client = calling.action_client::<Tracking>("/a").unwrap();
        let locks = Locks::of(&srv, &client);
        let mut cx = noop_cx();
        let accept = |srv: &mut ActionServer<Tracking>| {
            let response = client.send_goal_request(Tracked(0)).unwrap();
            let Poll::Ready(Some(request)) = Pin::new(srv).poll_next(&mut noop_cx()) else { panic!("queued") };
            let (goal, cancels) = request.accept().unwrap();
            (goal, cancels, response)
        };

        // feedback pushed out of the full queue
        let (mut goal, _cancels, mut response) = accept(&mut srv);
        let Poll::Ready(Ok((handle, result, feedback))) = Pin::new(&mut response).poll(&mut cx) else { panic!() };
        let locks = locks.with_goal(&handle.goal);
        let freed = freed_during(&locks, || {
            for i in 0..FEEDBACK_DEPTH as u32 + 2 {
                goal.publish_feedback(Tracked(i)).unwrap();
            }
        });
        assert_eq!(freed, [true, true], "the two oldest");
        assert_eq!(feedback.dropped(), 2);
        // what the stream still held, when it is dropped
        assert_eq!(freed_during(&locks, || drop(feedback)), vec![true; FEEDBACK_DEPTH]);
        // feedback nobody reads any more, and a result or feedback refused
        // after the end
        assert_eq!(freed_during(&locks, || goal.publish_feedback(Tracked(1)).unwrap()), [true]);
        goal.succeed(Tracked(2)).unwrap();
        let freed = freed_during(&locks, || {
            assert!(goal.succeed(Tracked(3)).is_err());
            assert!(goal.publish_feedback(Tracked(4)).is_err());
        });
        assert_eq!(freed, [true, true]);
        drop((handle, result));

        // a client scope's close: what its stream held goes at its next poll
        let (goal, _cancels, mut response) = accept(&mut srv);
        let Poll::Ready(Ok((handle, _result, mut feedback))) = Pin::new(&mut response).poll(&mut cx) else { panic!() };
        let locks = locks.with_goal(&handle.goal);
        goal.publish_feedback(Tracked(5)).unwrap();
        let other = bus.scope("other").action_client::<Tracking>("/a").unwrap();
        let (_gone, _c, response) = accept(&mut srv); // by `client` again, then abandoned
        drop(response);
        let queued = other.send_goal_request(Tracked(6)).unwrap();
        calling.close();
        let freed = freed_during(&locks, || assert!(Pin::new(&mut feedback).poll_next(&mut cx).is_ready()));
        assert_eq!(freed, [true]);

        // goal requests the server will not answer: failed by its close
        let freed = freed_during(&locks, || serving.close());
        assert_eq!(freed, [true], "the queued goal");
        drop(queued);
        drop(goal);
    }

    #[test]
    fn a_server_and_its_goals_are_freed_once_their_handles_are_gone() {
        let bus = Bus::new();
        let (serving, calling) = (bus.scope("server"), bus.scope("caller"));
        let mut srv = serving.action_server::<Act>("/a").unwrap();
        let client = calling.action_client::<Act>("/a").unwrap();
        let ((server_goal, cancels), (handle, result, feedback)) = accepted(&mut srv, &client, 1);
        let (server, goal) = (Arc::downgrade(&srv.server), Arc::downgrade(&handle.goal));
        drop(srv);
        assert!(server.upgrade().is_some(), "its goal still holds it");
        drop((server_goal, cancels));
        assert!(server.upgrade().is_none(), "nothing holds it any more, the registry included");
        drop((handle, result, feedback));
        assert!(goal.upgrade().is_none());
        assert_eq!(bus.actions()[0].active, 0);
    }
}
