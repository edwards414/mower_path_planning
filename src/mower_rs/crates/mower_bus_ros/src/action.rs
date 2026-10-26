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
//! - `action_to_ros`: a goal request the bus refuses at once, because no
//!   server is live, waits up to `bus_server` for one and is sent once more.
//!   One that a server has taken is never sent again. ROS has no error
//!   reply, so a goal request the bus does not answer is left unanswered on
//!   DDS, and so is the result of a goal the bus lost: the ROS client sees
//!   what it sees of a crashed server.
//! - `action_from_ros`: a DDS server is probed before a goal goes to it and
//!   watched while it holds the goal; its answers to a goal request and to a
//!   cancel request are given up at `dds_call`. The bus client then gets
//!   `NoResponse`, and for a goal held by a server that went away, the goal
//!   is lost on the bus (also `NoResponse`). Its bus server exists only while
//!   the graph shows a DDS server, as `service_from_ros`'s does ("Graph
//!   presence").

use std::future::Future;
use std::sync::Arc;
use std::time::Duration;

use futures::future::{BoxFuture, FutureExt};
use futures::stream::{FuturesUnordered, Stream, StreamExt};
use mower_bus::{
    Action, ActionClient, ActionClientGoal, ActionServer, ActionServerCancelRequest, ActionServerGoal,
    ActionServerGoalRequest, BusError, CancelRequests, FeedbackStream, GoalStatus, ResultFuture, Scope,
};

use crate::log::{Failure, Log};
use crate::node::NodeHandle;
use crate::service::{DdsServer, Probed, PRESENCE_PERIOD};
use crate::stats::Counters;
use crate::{BridgeAction, BridgeError, Forwarder};

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
/// server. A goal the bus refuses because no server is live waits up to
/// `wait` for one (`ServiceTimeouts`).
pub(crate) fn action_to_ros<A: BridgeAction>(
    scope: &Scope, node: &mut r2r::Node, name: &str, counters: Arc<Counters>, log: Log, wait: Duration,
) -> Result<Forwarder, BridgeError> {
    let client = scope.action_client::<BusAction<A>>(name)?;
    let mut requests = node.create_action_server::<A>(name)?;
    let env = Arc::new(Env { counters, log });
    Ok(Box::pin(async move {
        let mut goals = FuturesUnordered::new();
        loop {
            tokio::select! {
                request = requests.next() => match request {
                    Some(request) => goals.push(to_bus::<A>(client.clone(), request, env.clone(), wait)),
                    None => return,
                },
                Some(()) = goals.next() => {}
            }
        }
    }))
}

/// One DDS goal request: send it to the bus server, answer it with what the
/// server says, and once accepted carry the goal across to its end.
async fn to_bus<A: BridgeAction>(
    client: ActionClient<BusAction<A>>, request: r2r::ActionServerGoalRequest<A>, env: Arc<Env>, wait: Duration,
) {
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
            return;
        }
        // A server that took the request and ended (ServiceUnavailable from
        // the answer) or dropped it (NoResponse) may have acted on it: it is
        // not sent again, and the DDS request goes unanswered.
        Err(e) => {
            env.counters.unanswered();
            return env.log.warn(Failure::of(&e), || {
                format!("the bus goal request failed ({e}{waited}), so the ROS client gets no answer")
            });
        }
    };
    let (dds_goal, dds_cancels) = match request.accept() {
        Ok(accepted) => accepted,
        Err(e) => {
            env.counters.unanswered();
            env.log.error(Failure::Dds, || format!("accepting the DDS goal failed ({e}), so the bus goal is canceled"));
            // Nobody on DDS can see the goal: it must not run for nobody.
            if let Ok(cancel) = bus_goal.cancel() {
                let _ = cancel.await;
            }
            return;
        }
    };
    env.counters.forwarded();
    env.log.ok();
    carry_to_ros::<A>(bus_goal, result, feedback, dds_goal, dds_cancels, &env).await;
}

/// An accepted goal: the bus server's feedback and result go to the DDS
/// goal, the DDS client's cancel requests to the bus server.
async fn carry_to_ros<A: BridgeAction>(
    bus_goal: ActionClientGoal<BusAction<A>>, mut result: ResultFuture<BusAction<A>>,
    mut feedback: FeedbackStream<BusAction<A>>, mut dds_goal: r2r::ActionServerGoal<A>,
    mut dds_cancels: impl Stream<Item = r2r::ActionServerCancelRequest> + Unpin, env: &Env,
) {
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
        Ok((status, msg)) => {
            // r2r forgets a goal once it has applied an accepted cancel
            // request, at its next spin, and then only `cancel` delivers a
            // result (mower_nav's `terminate` falls back to it for that). So
            // a goal whose cancel was accepted on DDS ends CANCELED there,
            // whatever the bus server ended it with, as it did with
            // mower_nav's r2r server; the result message is the server's.
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
        // Lost: the bus server went away, or dropped the goal, before it
        // ended. Its outcome is unknown, and ROS has no word for that.
        Err(e) => {
            env.counters.unanswered();
            env.log.warn(Failure::of(&e), || format!("the bus goal was lost ({e}), so the ROS client gets no result"));
        }
    }
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

fn publish_on_dds<A: BridgeAction>(dds_goal: &r2r::ActionServerGoal<A>, msg: A::Feedback, env: &Env) {
    if let Err(e) = dds_goal.publish_feedback(msg) {
        env.log.error(Failure::Dds, || format!("publishing feedback on DDS failed: {e}"));
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
/// The bus server exists only while the graph shows a DDS server, looked at
/// every `PRESENCE_PERIOD` on the spin thread (`handle`), as for
/// `service_from_ros`: a bus client finds what an r2r client found, no server
/// while there is none (`ServiceUnavailable` at `send_goal_request`,
/// `wait_for_server` waits, mower_coverage says "Navigation action server
/// unavailable"), one as soon as there is. What the graph shows is never the
/// bridge's own: one direction per name. A server that goes takes the bus
/// server with it; the goal requests it queued fail with
/// `ServiceUnavailable`, and the goals it handed on end with their DDS
/// server, which is watched.
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
        let mut server: Option<ActionServer<BusAction<A>>> = None;
        let mut goals = FuturesUnordered::new();
        let mut look = tokio::time::interval(PRESENCE_PERIOD);
        look.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
        loop {
            tokio::select! {
                _ = look.tick() => {
                    let replies = replies.clone();
                    let shown = handle.call(move |node| node.get_publishers_info_by_topic(&replies, true)).await;
                    env.counters.resynced();
                    match shown {
                        // The bridge is stopping.
                        None => return,
                        Some(Ok(servers)) if servers.is_empty() => {
                            if server.take().is_some() {
                                env.counters.set_bus_server(false);
                            }
                        }
                        Some(Ok(_)) if server.is_some() => {}
                        Some(Ok(_)) => match scope.action_server::<BusAction<A>>(&name) {
                            Ok(serving) => {
                                server = Some(serving);
                                env.counters.set_bus_server(true);
                            }
                            // The bridge's scope closed.
                            Err(BusError::Closed { .. }) => return,
                            Err(e) => env.log.error(Failure::BusOther, || format!("serving on the bus failed: {e}")),
                        },
                        Some(Err(e)) => env.log.error(Failure::Dds, || format!("looking at the graph failed: {e}")),
                    }
                }
                request = next_goal(&mut server) => match request {
                    Some(request) => goals.push(to_dds::<A>(dds.clone(), request, env.clone(), deadline)),
                    // The bridge's scope closed.
                    None => return,
                },
                Some(()) = goals.next() => {}
            }
        }
    }))
}

/// The next goal request of `server`; while there is no server, nothing.
async fn next_goal<A: Action>(server: &mut Option<ActionServer<A>>) -> Option<ActionServerGoalRequest<A>> {
    match server {
        Some(server) => server.next().await,
        None => std::future::pending().await,
    }
}

/// One bus goal request: send it to the DDS server, if there is one, answer
/// it with what that server says, and once accepted carry the goal across to
/// its end. Dropping `request` unanswered gives the bus client
/// `NoResponse`: while no DDS server is matched, once none is during the
/// request, and when the answer does not come by `deadline`.
async fn to_dds<A: BridgeAction>(
    dds: Arc<DdsServer<r2r::ActionClient<A>>>, request: ActionServerGoalRequest<BusAction<A>>, env: Arc<Env>,
    deadline: Duration,
) {
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
        // The bridge's scope closed: nobody on the bus can see the goal.
        Err(_) => {
            env.counters.unanswered();
            if let Ok(cancel) = dds_goal.cancel() {
                let _ = tokio::time::timeout(deadline, dds.watch(cancel)).await;
            }
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
    let _ = tokio::time::timeout(CANCEL_DRAIN, async { while cancels.next().await.is_some() {} }).await;
    // Ending the bus goal answers the cancel requests the bridge has not
    // taken with GoalTerminated.
    let ended = match end {
        Some(Ok((r2r::GoalStatus::Succeeded, msg))) => bus_goal.succeed(msg),
        Some(Ok((r2r::GoalStatus::Canceled, msg))) => bus_goal.cancel(msg),
        Some(Ok((r2r::GoalStatus::Aborted, msg))) => bus_goal.abort(msg),
        // UNKNOWN: the server that answered did not know the goal. No
        // terminal status is proof of anything then.
        Some(Ok((status, _))) => {
            env.counters.unanswered();
            return env.log.warn(Failure::DdsLost, || {
                format!("the DDS result of a goal has status {status}, so the bus goal is lost")
            });
        }
        Some(Err(e)) => {
            env.counters.unanswered();
            return env.log.error(Failure::Dds, || format!("the DDS result could not be read ({e}), so the bus goal is lost"));
        }
        None => {
            env.counters.unanswered();
            return env.log.warn(Failure::DdsLost, || {
                "the DDS action server went away during a goal, so the bus goal is lost".into()
            });
        }
    };
    // `Closed` only: the bridge's scope closed and lost the goal already.
    if ended.is_err() {
        env.counters.unanswered();
    }
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
