//! Action forwarders: `action_to_ros` (DDS clients send goals to the bus
//! server) and `action_from_ros` (bus clients send goals to a DDS server).
//! Contract: README.md, "Actions".
//!
//! Each goal is forwarded in a future of its own in the forwarder's task, as
//! service calls are: one long goal must not hold up the others. That
//! future carries everything of its goal across: the answer to the request,
//! the feedback, the cancel requests (each in a future of its own, so that
//! the server takes its time with one without holding up the rest) and the
//! result.
//!
//! The rules are the services' (service.rs), for the same reasons:
//! - `action_to_ros`: the DDS server exists only while the bus has a server
//!   ("Graph presence"). A goal request in hand that the bus refuses at
//!   once, because no server is live, waits up to `bus_server` for one and
//!   is sent once more. One that a server has taken is never sent again. ROS
//!   has no error reply, so a goal request the bus does not answer is left
//!   unanswered on DDS, and so is the result of a goal the bus lost: the ROS
//!   client sees what it sees of a crashed server.
//! - `action_from_ros`: a DDS server is probed before a goal goes to it and
//!   watched while it holds the goal; its answers to a goal request and to a
//!   cancel request are given up at `dds_call`. The bus client then gets
//!   `NoResponse`, and for a goal held by a server that went away, the goal
//!   is lost on the bus (also `NoResponse`). Its bus server exists only while
//!   the graph shows a DDS server that the bridge's client is matched with,
//!   as `service_from_ros`'s does.
//!
//! r2r converts feedback with no lock held, and a result or a goal request
//! under the server's or client's lock: the first is guarded, the others are
//! checked first (convert.rs).

use std::future::Future;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::Duration;

use futures::future::{BoxFuture, FutureExt};
use futures::stream::{FuturesUnordered, Stream, StreamExt};
use mower_bus::{
    Action, ActionClient, ActionClientGoal, ActionServer, ActionServerCancelRequest, ActionServerGoal,
    ActionServerGoalRequest, BusError, CancelRequests, FeedbackStream, GoalStatus, ResultFuture, Scope,
};

use crate::convert::{convertible, guarded};
use crate::log::{Failure, Log};
use crate::node::{NodeHandle, DWELL, PRESENCE_PERIOD};
use crate::service::{DdsServer, Probed};
use crate::stats::Counters;
use crate::{topic, BridgeAction, BridgeError, Forwarder};

/// The bus side of r2r action `A`: its three message types.
pub(crate) type BusAction<A> = (
    <A as r2r::WrappedActionTypeSupport>::Goal,
    <A as r2r::WrappedActionTypeSupport>::Result,
    <A as r2r::WrappedActionTypeSupport>::Feedback,
);

/// How long the cancel requests of a goal that has ended may still take to
/// be answered by the other side before the goal is ended on this one; and,
/// in `action_to_ros`, how long after that DDS cancel requests of the goal
/// are still refused by the bridge rather than left to r2r.
const CANCEL_DRAIN: Duration = Duration::from_secs(1);

/// What the goals of one forwarder share.
struct Env {
    counters: Arc<Counters>,
    log: Log,
}

impl<A: BridgeAction> Probed for r2r::ActionClient<A> {
    fn probe(&self) -> Option<BoxFuture<'static, bool>> {
        let registered = r2r::Node::is_available(self).ok()?;
        Some(async move { registered.await.is_ok() }.boxed())
    }
}

// ---------------------------------------------------------------------------
// action_to_ros: DDS clients, the bus server
// ---------------------------------------------------------------------------

/// `action_to_ros`: a DDS action server whose goals become goals of the bus
/// server, and which exists only while the bus has a server. A goal request
/// in hand that finds no bus server waits up to `wait` for one
/// (`ServiceTimeouts`), while the DDS server still exists.
pub(crate) fn action_to_ros<A: BridgeAction>(
    scope: &Scope, name: &str, env: topic::Env, wait: Duration,
) -> Result<Forwarder, BridgeError> {
    // The name and the types are checked now; the DDS server comes with a
    // bus server, once the bridge runs.
    let client = scope.action_client::<BusAction<A>>(name)?;
    let topic::Env { node, counters, log } = env;
    let env = Arc::new(Env { counters, log });
    let forwarder = ActionToRos::<A> { name: name.to_string(), client, node, env, wait, running: Arc::default() };
    Ok(Box::pin(forwarder.run()))
}

struct ActionToRos<A: BridgeAction> {
    name: String,
    client: ActionClient<BusAction<A>>,
    node: NodeHandle,
    env: Arc<Env>,
    wait: Duration,
    running: Arc<Running>,
}

impl<A: BridgeAction> ActionToRos<A> {
    /// The DDS server while the bus has a server, made and destroyed on the
    /// spin thread, and its goals forwarded meanwhile, as `service_to_ros`
    /// does with a service. A DDS client then sees what it saw of the
    /// module's own r2r server: none while the module is down
    /// (`wait_for_server` waits), one as soon as it is up.
    async fn run(self) {
        let mut dds: Option<r2r::ActionServer<A>> = None;
        // Goal requests until they are answered; then the goals they became.
        let (mut requests, mut goals) = (FuturesUnordered::new(), FuturesUnordered::new());
        // When the DDS server last came or went.
        let mut changed: Option<tokio::time::Instant> = None;
        loop {
            tokio::select! {
                followed = self.presence_changed(dds.is_some(), changed) => {
                    if followed.is_err() {
                        // The bridge's scope closed.
                        return;
                    }
                    match (self.client.is_available(), dds.take()) {
                        (true, None) => {
                            let name = self.name.clone();
                            match self.node.call(move |node| node.create_action_server::<A>(&name)).await {
                                Some(Ok(server)) => {
                                    dds = Some(server);
                                    self.env.counters.set_dds_server(true);
                                }
                                Some(Err(e)) => self.env.log.error(Failure::Dds, || format!("creating the DDS action server failed: {e}")),
                                // The bridge is stopping.
                                None => return,
                            }
                            self.env.counters.resynced();
                            changed = Some(tokio::time::Instant::now());
                        }
                        (false, Some(server)) => {
                            let (requests, goals) = (std::mem::take(&mut requests), std::mem::take(&mut goals));
                            if self.withdraw(server, requests, goals).await.is_none() {
                                // The bridge is stopping.
                                return;
                            }
                            self.env.counters.resynced();
                            changed = Some(tokio::time::Instant::now());
                        }
                        // It came back while the dwell ran.
                        (_, same) => dds = same,
                    }
                }
                request = next_dds_goal(&mut dds) => match request {
                    Some(request) => {
                        let (client, env, running) = (self.client.clone(), self.env.clone(), self.running.clone());
                        requests.push(to_bus::<A>(client, request, env, self.wait, running));
                    }
                    // Only with the node: the bridge is stopping.
                    None => return,
                },
                Some(accepted) = requests.next() => if let Some(accepted) = accepted {
                    goals.push(carry_to_ros::<A>(accepted, self.env.clone()));
                },
                Some(()) = goals.next() => {}
            }
        }
    }

    /// Once the bus is not as the DDS server is, and `DWELL` after the last
    /// change. While there is a DDS server (`served`), that is once there is
    /// no bus server and no goal of the one that went runs on the bus any
    /// more; while there is none, once there is a bus server. `Closed`: the
    /// bridge's scope closed.
    async fn presence_changed(&self, served: bool, changed: Option<tokio::time::Instant>) -> Result<(), BusError> {
        if served {
            self.client.wait_for_no_server().await?;
            // A server that dropped its ActionServer with its scope open
            // leaves its goals running: they keep their DDS server until they
            // end, as with an r2r server whose node lives on.
            self.running.none().await;
        } else {
            self.client.wait_for_server().await?;
        }
        if let Some(changed) = changed {
            tokio::time::sleep_until(changed + DWELL).await;
        }
        Ok(())
    }

    /// The bus server is gone, and none of its goals runs any more: so is the
    /// DDS server, as the module's own r2r server went with its node. The goal
    /// requests in hand, and those r2r has queued, can be answered by nothing
    /// any more: none was taken by a live server, and one sent to the next
    /// server would run a goal for a client that gets no answer. They are
    /// lost, as with a crashed r2r server, and so are the cancel requests of
    /// ended goals still within their last refusals. `None`: the bridge is
    /// stopping.
    async fn withdraw(
        &self, mut server: r2r::ActionServer<A>, mut requests: FuturesUnordered<impl Future<Output = Option<Accepted<A>>>>,
        goals: FuturesUnordered<impl Future<Output = ()>>,
    ) -> Option<()> {
        while let Some(Some(_)) = requests.next().now_or_never() {}
        let mut lost = requests.len();
        drop(requests);
        drop(goals);
        while let Some(Some(_)) = server.next().now_or_never() {
            lost += 1;
        }
        for _ in 0..lost {
            self.env.counters.unanswered();
        }
        if lost > 0 {
            self.env.log.warn(Failure::BusUnavailable, || {
                format!("the bus action server went away, so the {lost} DDS goal requests in hand get no answer")
            });
        }
        self.env.counters.set_dds_server(false);
        // Awaited, so that the next DDS server never lives next to this one.
        self.node.call(move |node| node.destroy_action_server(server)).await
    }
}

/// The next goal request of the DDS server; while there is none, nothing.
async fn next_dds_goal<A: BridgeAction>(server: &mut Option<r2r::ActionServer<A>>) -> Option<r2r::ActionServerGoalRequest<A>> {
    match server {
        Some(server) => server.next().await,
        None => std::future::pending().await,
    }
}

/// The goals an `action_to_ros` forwarder carries whose bus goal has not
/// ended: its DDS server waits for them once the bus server is gone.
#[derive(Default)]
struct Running {
    goals: AtomicUsize,
    none: tokio::sync::Notify,
}

impl Running {
    fn enter(self: &Arc<Self>) -> RunningGoal {
        self.goals.fetch_add(1, Ordering::SeqCst);
        RunningGoal(self.clone())
    }

    /// Once no goal runs (at once if none does).
    async fn none(&self) {
        loop {
            let notified = self.none.notified();
            let mut notified = std::pin::pin!(notified);
            // Registered before the count is read, so that the last goal's
            // end in between is not missed.
            notified.as_mut().enable();
            if self.goals.load(Ordering::SeqCst) == 0 {
                return;
            }
            notified.await;
        }
    }
}

/// One goal in `Running`, until it is dropped: when the bus goal ends, or
/// with the future that carries it.
struct RunningGoal(Arc<Running>);

impl Drop for RunningGoal {
    fn drop(&mut self) {
        if self.0.goals.fetch_sub(1, Ordering::SeqCst) == 1 {
            self.0.none.notify_waiters();
        }
    }
}

/// A goal both sides accepted, as `to_bus` hands it to `carry_to_ros`.
struct Accepted<A: BridgeAction> {
    bus_goal: ActionClientGoal<BusAction<A>>,
    result: ResultFuture<BusAction<A>>,
    feedback: FeedbackStream<BusAction<A>>,
    dds_goal: r2r::ActionServerGoal<A>,
    dds_cancels: Box<dyn Stream<Item = r2r::ActionServerCancelRequest> + Send + Unpin>,
    running: RunningGoal,
}

/// One DDS goal request: send it to the bus server and answer it with what
/// the server says. An accepted goal is handed on, to be carried across to
/// its end.
async fn to_bus<A: BridgeAction>(
    client: ActionClient<BusAction<A>>, request: r2r::ActionServerGoalRequest<A>, env: Arc<Env>, wait: Duration,
    running: Arc<Running>,
) -> Option<Accepted<A>> {
    let mut waited = "";
    let answer = match client.send_goal_request(request.goal.clone()) {
        Ok(answer) => answer.await,
        // No server was live, so nothing ran for it: wait for one (a module
        // restarting), and send it once more.
        Err(BusError::ServiceUnavailable { .. }) => match tokio::time::timeout(wait, client.wait_for_server()).await {
            Ok(Ok(())) => match client.send_goal_request(request.goal.clone()) {
                Ok(answer) => answer.await,
                Err(e) => Err(e),
            },
            // The bridge's scope closed.
            Ok(Err(e)) => Err(e),
            Err(_) => {
                waited = ", and none came in time";
                Err(BusError::ServiceUnavailable { name: client.name().to_string() })
            }
        },
        Err(e) => Err(e),
    };
    let (bus_goal, result, feedback) = match answer {
        Ok(accepted) => accepted,
        Err(BusError::GoalRejected { .. }) => {
            match request.reject() {
                Ok(()) => {
                    env.counters.forwarded();
                    env.log.ok();
                }
                Err(e) => {
                    env.counters.unanswered();
                    env.log.error(Failure::Dds, || format!("rejecting the DDS goal failed: {e}"));
                }
            }
            return None;
        }
        // A server that took the request and ended (ServiceUnavailable from
        // the answer) or dropped it (NoResponse) may have acted on it: it is
        // not sent again, and the DDS request goes unanswered.
        Err(e) => {
            env.counters.unanswered();
            env.log.warn(Failure::of(&e), || {
                format!("the bus goal request failed ({e}{waited}), so the ROS client gets no answer")
            });
            return None;
        }
    };
    // The bus goal runs from now on, and counts until it ends.
    let running = running.enter();
    let (dds_goal, dds_cancels) = match request.accept() {
        Ok(accepted) => accepted,
        // An r2r error, or the DDS server destroyed meanwhile.
        Err(e) => {
            env.counters.unanswered();
            env.log.error(Failure::Dds, || format!("accepting the DDS goal failed ({e}), so the bus goal is canceled"));
            // Nobody on DDS can see the goal: it must not run for nobody.
            if let Ok(cancel) = bus_goal.cancel() {
                let _ = cancel.await;
            }
            return None;
        }
    };
    env.counters.forwarded();
    env.log.ok();
    Some(Accepted { bus_goal, result, feedback, dds_goal, dds_cancels: Box::new(dds_cancels), running })
}

/// An accepted goal: the bus server's feedback and result go to the DDS
/// goal, the DDS client's cancel requests to the bus server.
async fn carry_to_ros<A: BridgeAction>(accepted: Accepted<A>, env: Arc<Env>) {
    let Accepted { bus_goal, mut result, mut feedback, mut dds_goal, mut dds_cancels, running } = accepted;
    let env = &*env;
    let mut cancels = FuturesUnordered::new();
    let (mut feedback_open, mut cancels_open, mut cancel_accepted) = (true, true, false);
    let end = loop {
        tokio::select! {
            biased;
            Some(accepted) = cancels.next() => cancel_accepted |= accepted,
            msg = feedback.next(), if feedback_open => match msg {
                Some(msg) => publish_on_dds::<A>(&dds_goal, msg, env),
                None => feedback_open = false,
            },
            end = &mut result => break end,
            request = dds_cancels.next(), if cancels_open => match request {
                Some(request) => cancels.push(cancel_on_bus::<A>(bus_goal.clone(), DdsCancel::new(request), env)),
                None => cancels_open = false,
            },
        }
    };
    // The feedback published before the end goes first; the stream ends at
    // once now that the goal has.
    while let Some(msg) = feedback.next().await {
        publish_on_dds::<A>(&dds_goal, msg, env);
    }
    // The cancel requests in flight have the bus server's answers by now, or
    // soon: refusals, since an acceptance after the end is GoalTerminated on
    // the bus (one that came before it was taken above). Those that come
    // meanwhile are refused at once, and those still unanswered after the
    // drain are rejected as they go (`DdsCancel`).
    let _ = tokio::time::timeout(CANCEL_DRAIN, async {
        loop {
            tokio::select! {
                biased;
                accepted = cancels.next() => match accepted {
                    Some(accepted) => cancel_accepted |= accepted,
                    None => return,
                },
                request = dds_cancels.next(), if cancels_open => match request {
                    Some(request) => DdsCancel::new(request).reject(),
                    None => cancels_open = false,
                },
            }
        }
    })
    .await;
    drop(cancels);
    match end {
        // r2r converts the result under the server's lock (convert.rs).
        Ok((status, msg)) => match convertible(&msg) {
            Err(why) => {
                env.counters.invalid();
                env.counters.unanswered();
                env.log.error(Failure::Invalid, || {
                    format!("r2r cannot convert the bus server's result ({why}), so the ROS client gets no result")
                });
            }
            Ok(()) => {
                // r2r forgets a goal once it has applied an accepted cancel
                // request, at its next spin, and then only `cancel` delivers
                // a result (mower_nav's `terminate` falls back to it for
                // that). So a goal whose cancel was accepted on DDS ends
                // CANCELED there, whatever the bus server ended it with, as
                // it did with mower_nav's r2r server; the result message is
                // the server's.
                let delivered = match status {
                    _ if cancel_accepted => dds_goal.cancel(msg),
                    GoalStatus::Succeeded => dds_goal.succeed(msg),
                    GoalStatus::Canceled => dds_goal.cancel(msg),
                    _ => dds_goal.abort(msg),
                };
                if let Err(e) = delivered {
                    env.counters.unanswered();
                    env.log.error(Failure::Dds, || format!("ending the DDS goal failed: {e}"));
                }
            }
        },
        // Lost: the bus server went away, or dropped the goal, before it
        // ended. Its outcome is unknown, and ROS has no word for that.
        Err(e) => {
            env.counters.unanswered();
            env.log.warn(Failure::of(&e), || format!("the bus goal was lost ({e}), so the ROS client gets no result"));
        }
    }
    // The goal is over on both sides: it no longer holds the DDS server.
    drop(running);
    // r2r hands out no cancel request for a goal it has ended with `succeed`
    // or `abort`, and answers those itself. One ended with `cancel` is still
    // cancelable in rcl until r2r applies the acceptance, at its next spin,
    // and a lost goal stays cancelable: what comes meanwhile is refused, as
    // the bus server would refuse it.
    let _ = tokio::time::timeout(CANCEL_DRAIN, async {
        while let Some(request) = dds_cancels.next().await {
            DdsCancel::new(request).reject();
        }
    })
    .await;
}

/// Feedback to DDS. r2r converts it with no lock held, so a value it cannot
/// convert is caught as the conversion fails (convert.rs), and dropped.
fn publish_on_dds<A: BridgeAction>(dds_goal: &r2r::ActionServerGoal<A>, msg: A::Feedback, env: &Env) {
    match guarded(|| dds_goal.publish_feedback(msg)) {
        Ok(Ok(())) => {}
        Ok(Err(e)) => env.log.error(Failure::Dds, || format!("publishing feedback on DDS failed: {e}")),
        Err(why) => {
            env.counters.invalid();
            env.log.error(Failure::Invalid, || format!("r2r cannot convert the bus server's feedback ({why}), so it is dropped"));
        }
    }
}

/// A DDS cancel request in the bridge's hands, rejected unless answered. r2r
/// sends no CancelGoal response at all once a request it handed out for one
/// is dropped unanswered (`send_completed_cancel_requests` skips it), so its
/// client would wait forever: a request the bridge gives up, at the end of
/// the drain or when `run` aborts the forwarder, is rejected instead, as
/// every other refusal is.
struct DdsCancel(Option<r2r::ActionServerCancelRequest>);

impl DdsCancel {
    fn new(request: r2r::ActionServerCancelRequest) -> Self {
        DdsCancel(Some(request))
    }

    fn accept(mut self) {
        self.answer(true);
    }

    fn reject(mut self) {
        self.answer(false);
    }

    fn answer(&mut self, accepted: bool) {
        match self.0.take() {
            Some(request) if accepted => request.accept(),
            Some(request) => request.reject(),
            None => {}
        }
    }
}

impl Drop for DdsCancel {
    fn drop(&mut self) {
        self.answer(false);
    }
}

/// One DDS cancel request: ask the bus server, and answer with what it says;
/// `true` if it was accepted. Every refusal is a reject on DDS, which has no
/// other word for it.
async fn cancel_on_bus<A: BridgeAction>(
    bus_goal: ActionClientGoal<BusAction<A>>, request: DdsCancel, env: &Env,
) -> bool {
    let answer = match bus_goal.cancel() {
        Ok(answer) => answer.await,
        Err(e) => Err(e),
    };
    match answer {
        Ok(()) => {
            request.accept();
            return true;
        }
        Err(BusError::CancelRejected { .. } | BusError::GoalTerminated { .. }) => {}
        Err(e) => {
            env.log.warn(Failure::of(&e), || format!("the bus cancel request failed ({e}), so the ROS client's is rejected"));
        }
    }
    request.reject();
    false
}

// ---------------------------------------------------------------------------
// action_from_ros: bus clients, the DDS server
// ---------------------------------------------------------------------------

/// `action_from_ros`: a bus action server whose goals become goals of the
/// DDS server. Its answers to a goal request or a cancel request are given
/// up at `deadline` (`ServiceTimeouts`).
///
/// The bus server exists only while the graph shows a DDS server that the
/// bridge's client is matched with, looked at every `PRESENCE_PERIOD` on the
/// spin thread (`handle`), as for `service_from_ros`: a bus client finds
/// what an r2r client found, no server while there is none
/// (`ServiceUnavailable` at `send_goal_request`, `wait_for_server` waits,
/// mower_coverage says "Navigation action server unavailable"), one as soon
/// as there is. What the graph shows is never the bridge's own: one
/// direction per name. A server that goes takes the bus server with it; the
/// goal requests it queued fail with `ServiceUnavailable`, and the goals it
/// handed on end with their DDS server, which is watched.
///
/// The graph shows the server's goal reply writer before the bridge's client
/// is matched with all of the server (`rcl_action_server_is_available`: its
/// three services and two topics), and a goal sent then would come back
/// `NoResponse`, which mower_coverage tracks with correlated cancels. So the
/// bus server comes only once a probe finds the server matched, as an r2r
/// client's `is_available` did.
pub(crate) fn action_from_ros<A: BridgeAction>(
    scope: &Scope, node: &mut r2r::Node, handle: NodeHandle, name: &str, counters: Arc<Counters>, log: Log,
    deadline: Duration,
) -> Result<Forwarder, BridgeError> {
    // The name, the types and a bus server already there are checked now;
    // the bus server itself comes with a DDS server.
    drop(scope.action_server::<BusAction<A>>(name)?);
    let dds = Arc::new(DdsServer::new(node.create_action_client::<A>(name)?));
    let env = Arc::new(Env { counters, log });
    let (scope, name) = (scope.clone(), name.to_string());
    // A server's writer: clients write `rq<name>/_action/send_goalRequest`
    // and read this.
    let replies = format!("rr{name}/_action/send_goalReply");
    Ok(Box::pin(async move {
        let mut serving: Option<Serving<BusAction<A>>> = None;
        let mut goals = FuturesUnordered::new();
        // Polled with the goals, so that a probe (up to 500 ms) holds none
        // of them up.
        let first = look_at_graph(handle.clone(), dds.clone(), replies.clone(), false, tokio::time::Instant::now());
        let mut look = Box::pin(first);
        loop {
            tokio::select! {
                seen = &mut look => {
                    env.counters.resynced();
                    match seen {
                        Seen::Stopping => return,
                        Seen::Failed(e) => env.log.error(Failure::Dds, || format!("looking at the graph failed: {e}")),
                        Seen::NoServer => {
                            if serving.take().is_some() {
                                env.counters.set_bus_server(false);
                            }
                            // A server that comes next is probed afresh.
                            dds.forget();
                        }
                        // Not matched yet (within the probe's 500 ms): the
                        // next look probes again.
                        Seen::Served | Seen::Unmatched => {}
                        Seen::Matched => match Serving::new(&scope, &name) {
                            Ok(server) => {
                                serving = Some(server);
                                env.counters.set_bus_server(true);
                            }
                            // The bridge's scope closed.
                            Err(BusError::Closed { .. }) => return,
                            Err(e) => env.log.error(Failure::BusOther, || format!("serving on the bus failed: {e}")),
                        },
                    }
                    let next = tokio::time::Instant::now() + PRESENCE_PERIOD;
                    look.set(look_at_graph(handle.clone(), dds.clone(), replies.clone(), serving.is_some(), next));
                }
                request = next_goal(&mut serving) => match request {
                    Some((request, own)) => goals.push(to_dds::<A>(dds.clone(), request, env.clone(), deadline, own)),
                    // The bridge's scope closed.
                    None => return,
                },
                Some(()) = goals.next() => {}
            }
        }
    }))
}

/// What a look at the graph found (`action_from_ros`).
enum Seen {
    /// The bridge is stopping.
    Stopping,
    Failed(r2r::Error),
    /// No DDS server.
    NoServer,
    /// One, and the bus server is there already.
    Served,
    /// One, and the bridge's client is matched with all of it.
    Matched,
    /// One the bridge's client is not matched with yet.
    Unmatched,
}

/// At `at`, look at the graph on the spin thread for the server's goal
/// reply writer; with no bus server (`served` false), probe a server shown.
async fn look_at_graph<A: BridgeAction>(
    handle: NodeHandle, dds: Arc<DdsServer<r2r::ActionClient<A>>>, replies: String, served: bool, at: tokio::time::Instant,
) -> Seen {
    tokio::time::sleep_until(at).await;
    match handle.call(move |node| node.get_publishers_info_by_topic(&replies, true)).await {
        None => Seen::Stopping,
        Some(Err(e)) => Seen::Failed(e),
        Some(Ok(servers)) if servers.is_empty() => Seen::NoServer,
        Some(Ok(_)) if served => Seen::Served,
        Some(Ok(_)) if dds.available().await => Seen::Matched,
        Some(Ok(_)) => Seen::Unmatched,
    }
}

/// An `action_from_ros` bus server, in a child scope of its own, as
/// `service_from_ros`'s is: the bridge's scope keeps no close hook of it,
/// however often the DDS server comes and goes (mower_bus README, "Hooks of
/// dropped endpoints"). Unlike a service's calls, the goals a server handed
/// on run on after it is dropped, and end with their DDS server: closing the
/// scope would lose them. So the scope closes only once the server and every
/// goal of it are gone, each of which holds it (`OwnScope`).
struct Serving<A: Action> {
    server: ActionServer<A>,
    own: Arc<OwnScope>,
}

impl<A: Action> Serving<A> {
    fn new(scope: &Scope, name: &str) -> Result<Self, BusError> {
        let own = Arc::new(OwnScope(scope.child(scope.name())));
        let server = own.0.action_server::<A>(name)?;
        Ok(Serving { server, own })
    }
}

/// A child scope, closed when the last of those who hold it lets go.
struct OwnScope(Scope);

impl Drop for OwnScope {
    fn drop(&mut self) {
        self.0.close();
    }
}

/// The next goal request of the bus server, with its server's scope; while
/// there is no server, nothing.
async fn next_goal<A: Action>(serving: &mut Option<Serving<A>>) -> Option<(ActionServerGoalRequest<A>, Arc<OwnScope>)> {
    match serving {
        Some(serving) => Some((serving.server.next().await?, serving.own.clone())),
        None => std::future::pending().await,
    }
}

/// One bus goal request: send it to the DDS server, if there is one, answer
/// it with what that server says, and once accepted carry the goal across to
/// its end. Dropping `request` unanswered gives the bus client
/// `NoResponse`: while no DDS server is matched, once none is during the
/// request, when the answer does not come by `deadline`, and when r2r cannot
/// convert the goal. `own` is its bus server's scope, held until the goal is
/// done.
async fn to_dds<A: BridgeAction>(
    dds: Arc<DdsServer<r2r::ActionClient<A>>>, request: ActionServerGoalRequest<BusAction<A>>, env: Arc<Env>,
    deadline: Duration, own: Arc<OwnScope>,
) {
    let _own = own;
    // r2r converts the goal request under its client's lock (convert.rs).
    if let Err(why) = convertible(&request.goal) {
        env.counters.invalid();
        env.counters.unanswered();
        return env.log.error(Failure::Invalid, || {
            format!("r2r cannot convert the bus goal ({why}), so the bus client gets NoResponse")
        });
    }
    if !dds.available().await {
        env.counters.unanswered();
        return env.log.warn(Failure::DdsUnavailable, || {
            "no DDS action server is matched, so the bus client gets NoResponse".into()
        });
    }
    let response = match dds.client.send_goal_request(request.goal.clone()) {
        Ok(response) => response,
        Err(e) => {
            env.counters.unanswered();
            return env.log.error(Failure::Dds, || format!("sending the DDS goal request failed: {e}"));
        }
    };
    let mut response = std::pin::pin!(response);
    let answer = match tokio::time::timeout(deadline, dds.watch(response.as_mut())).await {
        Ok(Some(answer)) => answer,
        Ok(None) => {
            env.counters.unanswered();
            return env.log.warn(Failure::DdsLost, || {
                "the DDS action server went away during a goal request, so the bus client gets NoResponse".into()
            });
        }
        Err(_) => {
            env.counters.unanswered();
            drop(request);
            env.log.warn(Failure::DdsTimeout, || {
                format!("the DDS action server did not answer a goal request within {deadline:?}, so the bus client gets NoResponse")
            });
            // A goal it accepts later would run for nobody: cancel it, as
            // mower_coverage cancels a goal accepted after its own timeout.
            // One more `deadline` for that answer.
            if let Ok(Some(Ok((late, _result, _feedback)))) = tokio::time::timeout(deadline, dds.watch(response)).await {
                if let Ok(cancel) = late.cancel() {
                    let _ = tokio::time::timeout(deadline, dds.watch(cancel)).await;
                }
            }
            return;
        }
    };
    let (dds_goal, dds_result, dds_feedback) = match answer {
        Ok(accepted) => accepted,
        Err(r2r::Error::RCL_RET_ACTION_GOAL_REJECTED) => {
            // `Closed` only: the bridge's scope closed meanwhile.
            let _ = request.reject();
            env.counters.forwarded();
            env.log.ok();
            return;
        }
        Err(e) => {
            env.counters.unanswered();
            return env.log.error(Failure::Dds, || format!("the DDS goal request failed: {e}"));
        }
    };
    let (bus_goal, bus_cancels) = match request.accept() {
        Ok(accepted) => accepted,
        // The bridge's scope closed, and its close gave the bus client
        // ServiceUnavailable. The DDS goal runs on, as the goal of a direct
        // r2r client whose node goes does (README.md, "action_from_ros").
        Err(_) => {
            env.counters.unanswered();
            return;
        }
    };
    env.counters.forwarded();
    env.log.ok();
    carry_from_ros::<A>(&dds, dds_goal, dds_result, dds_feedback, bus_goal, bus_cancels, &env, deadline).await;
}

/// An accepted goal: the DDS server's feedback and result go to the bus
/// goal, the bus client's cancel requests to the DDS server.
///
/// r2r's result future never completes when the DDS server goes away, so it
/// is watched like a pending service call: once no server is matched, the
/// bus goal is lost (dropped, so its client gets `NoResponse`). A server
/// replaced while it holds the goal stays matched, and the goal ends only
/// when the bus client gives up on it, as with a direct r2r client
/// (README.md, "Actions").
#[allow(clippy::too_many_arguments)]
async fn carry_from_ros<A: BridgeAction>(
    dds: &DdsServer<r2r::ActionClient<A>>, dds_goal: r2r::ActionClientGoal<A>,
    dds_result: impl Future<Output = r2r::Result<(r2r::GoalStatus, A::Result)>>,
    mut dds_feedback: impl Stream<Item = A::Feedback> + Unpin, mut bus_goal: ActionServerGoal<BusAction<A>>,
    mut bus_cancels: CancelRequests, env: &Env, deadline: Duration,
) {
    let mut result = std::pin::pin!(dds.watch(dds_result));
    let mut cancels = FuturesUnordered::new();
    let (mut feedback_open, mut cancels_open) = (true, true);
    let end = loop {
        tokio::select! {
            biased;
            Some(()) = cancels.next() => {}
            msg = dds_feedback.next(), if feedback_open => match msg {
                Some(msg) => publish_on_bus::<A>(&bus_goal, msg, env),
                None => feedback_open = false,
            },
            end = &mut result => break end,
            request = bus_cancels.next(), if cancels_open => match request {
                Some(request) => cancels.push(cancel_on_dds::<A>(dds, dds_goal.clone(), request, env, deadline)),
                // The bus goal was lost with the bridge's scope; the DDS goal
                // runs on, as it would if a direct r2r client's node went.
                None => cancels_open = false,
            },
        }
    };
    // What r2r has queued of the feedback goes first.
    while let Some(Some(msg)) = dds_feedback.next().now_or_never() {
        publish_on_bus::<A>(&bus_goal, msg, env);
    }
    // DDS orders a cancel answer and the result on separate entities, so an
    // acceptance the server sent before the result can come after it. It
    // reaches the bus client as an acceptance only while the bus goal has
    // not ended: the answers in flight get a moment first.
    let _ = tokio::time::timeout(CANCEL_DRAIN, async { while cancels.next().await.is_some() {} }).await;
    // Ending the bus goal answers the cancel requests the bridge has not
    // taken with GoalTerminated.
    let ended = match end {
        Some(Ok((r2r::GoalStatus::Succeeded, msg))) => bus_goal.succeed(msg),
        Some(Ok((r2r::GoalStatus::Canceled, msg))) => bus_goal.cancel(msg),
        Some(Ok((r2r::GoalStatus::Aborted, msg))) => bus_goal.abort(msg),
        lost => {
            // The goal goes before its cancel stream: the requests that are
            // still queued get NoResponse, as a lost goal's do, and not the
            // CancelRejected of a stream dropped under a live goal.
            drop(bus_goal);
            env.counters.unanswered();
            return match lost {
                // UNKNOWN: the server that answered did not know the goal.
                // No terminal status is proof of anything then.
                Some(Ok((status, _))) => env.log.warn(Failure::DdsLost, || {
                    format!("the DDS result of a goal has status {status}, so the bus goal is lost")
                }),
                Some(Err(e)) => {
                    env.log.error(Failure::Dds, || format!("the DDS result could not be read ({e}), so the bus goal is lost"))
                }
                None => env.log.warn(Failure::DdsLost, || {
                    "the DDS action server went away during a goal, so the bus goal is lost".into()
                }),
            };
        }
    };
    // `Closed` only: the bridge's scope closed and lost the goal already.
    if ended.is_err() {
        env.counters.unanswered();
    }
    // The cancel requests still in flight get the DDS server's answer when
    // it comes, by `deadline`, as a direct r2r client got it: a rejection is
    // CancelRejected, an acceptance GoalTerminated now that the bus goal has
    // ended.
    while cancels.next().await.is_some() {}
}

fn publish_on_bus<A: BridgeAction>(bus_goal: &ActionServerGoal<BusAction<A>>, msg: A::Feedback, env: &Env) {
    // `Closed` only: the bridge's scope closed; the goal is lost already.
    if let Err(e) = bus_goal.publish_feedback(msg) {
        env.log.warn(Failure::of(&e), || format!("publishing feedback on the bus failed: {e}"));
    }
}

/// One bus cancel request: ask the DDS server, and answer with what it says.
/// Dropped unanswered (the bus client gets `NoResponse`) when the server
/// goes away or does not answer by `deadline`: r2r's cancel future never
/// completes then.
async fn cancel_on_dds<A: BridgeAction>(
    dds: &DdsServer<r2r::ActionClient<A>>, dds_goal: r2r::ActionClientGoal<A>, request: ActionServerCancelRequest,
    env: &Env, deadline: Duration,
) {
    let answer = match dds_goal.cancel() {
        Ok(answer) => tokio::time::timeout(deadline, dds.watch(answer)).await,
        Err(e) => {
            env.log.error(Failure::Dds, || format!("sending the DDS cancel request failed: {e}"));
            return request.reject();
        }
    };
    match answer {
        Ok(Some(Ok(()))) => request.accept(),
        // GoalCancelRejected, GoalCancelAlreadyTerminated (also the answer
        // for a goal that is canceling already) or GoalCancelUnknownGoalID.
        Ok(Some(Err(_))) => request.reject(),
        Ok(None) => env.log.warn(Failure::DdsLost, || {
            "the DDS action server went away during a cancel request, so the bus client gets NoResponse".into()
        }),
        Err(_) => env.log.warn(Failure::DdsTimeout, || {
            format!("the DDS action server did not answer a cancel request within {deadline:?}, so the bus client gets NoResponse")
        }),
    }
}
