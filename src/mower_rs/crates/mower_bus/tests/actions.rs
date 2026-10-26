//! Actions on the in-process bus (README.md, "Actions"): goals accepted and
//! rejected, cancel requests, feedback and results, what closing a scope or
//! dropping a handle does to them, concurrent goals, introspection, a client
//! written the way mower_coverage's is against a server written the way
//! mower_nav's is, on tokio and with no runtime at all.

use std::any::type_name;
use std::collections::HashSet;
use std::future::Future;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::pin::Pin;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::task::{Context, Poll, Waker};
use std::thread;
use std::time::Duration;

use futures::task::ArcWake;
use futures::{FutureExt, Stream, StreamExt};
use mower_bus::{
    ActionClient, ActionClientGoal, ActionInfo, ActionServer, ActionServerCancelRequest, ActionServerGoal,
    ActionServerGoalRequest, Bus, BusError, CancelFuture, CancelRequests, FeedbackStream, GoalResponseFuture, GoalStatus,
    ResultFuture, Scope,
};

/// Laid out like r2r's generated action types, for `ros_type`; the fields
/// are mower_interface/action/Waypoint's that the tests need.
#[allow(non_snake_case)]
mod mower_interface {
    pub mod action {
        pub mod Waypoint {
            use serde::{Deserialize, Serialize};

            #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
            pub struct Goal {
                pub dispatch_id: String,
                pub zone_id: i32,
            }

            #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
            pub struct Result {
                pub success: bool,
            }

            #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
            pub struct Feedback {
                pub feedback: String,
            }
        }
    }
}
use mower_interface::action::Waypoint;

/// What a module writes once, where r2r takes `Waypoint::Action`.
type WaypointAction = (Waypoint::Goal, Waypoint::Result, Waypoint::Feedback);

const NAME: &str = "/nav_action_follow_path";

fn goal(dispatch_id: &str) -> Waypoint::Goal {
    Waypoint::Goal { dispatch_id: dispatch_id.into(), zone_id: -1 }
}

fn result(success: bool) -> Waypoint::Result {
    Waypoint::Result { success }
}

fn feedback(text: &str) -> Waypoint::Feedback {
    Waypoint::Feedback { feedback: text.into() }
}

fn unavailable() -> BusError {
    BusError::ServiceUnavailable { name: NAME.into() }
}

fn no_response() -> BusError {
    BusError::NoResponse { name: NAME.into() }
}

fn terminated() -> BusError {
    BusError::GoalTerminated { name: NAME.into() }
}

fn closed(scope: &str) -> BusError {
    BusError::Closed { scope: scope.into() }
}

/// Fail the test instead of hanging it.
async fn within<F: Future>(fut: F) -> F::Output {
    tokio::time::timeout(Duration::from_secs(5), fut).await.expect("timed out")
}

/// A waker that records being woken: checks wake-ups without timing.
#[derive(Default)]
struct Woken(AtomicBool);

impl ArcWake for Woken {
    fn wake_by_ref(arc_self: &Arc<Self>) {
        arc_self.0.store(true, Ordering::SeqCst);
    }
}

impl Woken {
    fn new() -> Arc<Self> {
        Arc::default()
    }

    /// Whether it was woken since the last look.
    fn take(&self) -> bool {
        self.0.swap(false, Ordering::SeqCst)
    }
}

fn poll_with<F: Future + Unpin>(fut: &mut F, woken: &Arc<Woken>) -> Poll<F::Output> {
    let waker = futures::task::waker(woken.clone());
    Pin::new(fut).poll(&mut Context::from_waker(&waker))
}

fn poll_next_with<S: Stream + Unpin>(stream: &mut S, woken: &Arc<Woken>) -> Poll<Option<S::Item>> {
    let waker = futures::task::waker(woken.clone());
    Pin::new(stream).poll_next(&mut Context::from_waker(&waker))
}

/// What a client holds of an accepted goal.
type ClientSide = (ActionClientGoal<WaypointAction>, ResultFuture<WaypointAction>, FeedbackStream<WaypointAction>);

/// What a server holds of an accepted goal.
type ServerSide = (ActionServerGoal<WaypointAction>, CancelRequests);

/// Send a goal, accept it on `srv` and return both ends, with no runtime:
/// everything is ready at once.
fn accept_now(srv: &mut ActionServer<WaypointAction>, client: &ActionClient<WaypointAction>, dispatch_id: &str) -> (ServerSide, ClientSide) {
    let response = client.send_goal_request(goal(dispatch_id)).unwrap();
    let request = srv.next().now_or_never().flatten().expect("the request is handed out");
    let server_side = request.accept().unwrap();
    let client_side = response.now_or_never().expect("answered at once").expect("accepted");
    (server_side, client_side)
}

/// Send a goal, accept it on `srv` and return both ends.
async fn accept_one(
    srv: &mut ActionServer<WaypointAction>, client: &ActionClient<WaypointAction>, dispatch_id: &str,
) -> (ServerSide, ClientSide) {
    let response = client.send_goal_request(goal(dispatch_id)).unwrap();
    let request = within(srv.next()).await.expect("the request reaches the server");
    assert_eq!(request.goal, goal(dispatch_id));
    let server_side = request.accept().unwrap();
    let client_side = within(response).await.expect("the goal is accepted");
    (server_side, client_side)
}

// ---------------------------------------------------------------------------
// Goals, feedback and results
// ---------------------------------------------------------------------------

#[tokio::test]
async fn an_accepted_goal_gets_its_feedback_in_order_and_then_its_result() {
    let bus = Bus::new();
    let (nav, coverage) = (bus.scope("nav_action_server"), bus.scope("boustrophedon_coverage"));
    let mut srv = nav.action_server::<WaypointAction>(NAME).unwrap();
    let client = coverage.action_client::<WaypointAction>(NAME).unwrap();
    assert_eq!((srv.name(), client.name()), (NAME, NAME));

    let ((mut server_goal, _cancels), (handle, result_future, mut feedbacks)) = accept_one(&mut srv, &client, "a").await;
    assert_eq!(server_goal.id, handle.id, "both ends name the goal alike");
    assert_eq!(server_goal.goal, goal("a"));
    assert_eq!(handle.get_status(), Ok(GoalStatus::Executing), "accepted is executing at once, as in r2r");
    assert_eq!(server_goal.is_cancelling(), Ok(false));
    for text in ["one", "two", "three"] {
        server_goal.publish_feedback(feedback(text)).unwrap();
    }
    server_goal.succeed(result(true)).unwrap();
    // Everything published before the end comes first, then the stream ends.
    let all: Vec<String> = within(feedbacks.by_ref().map(|f| f.feedback).collect::<Vec<_>>()).await;
    assert_eq!(all, ["one", "two", "three"]);
    assert_eq!(within(result_future).await, Ok((GoalStatus::Succeeded, result(true))));
    assert_eq!(handle.get_status(), Ok(GoalStatus::Succeeded));
}

#[tokio::test]
async fn a_rejected_goal_fails_its_request_with_goal_rejected() {
    let bus = Bus::new();
    let mut srv = bus.scope("nav").action_server::<WaypointAction>(NAME).unwrap();
    let client = bus.scope("coverage").action_client::<WaypointAction>(NAME).unwrap();
    let response = client.send_goal_request(goal("reused")).unwrap();
    let request = within(srv.next()).await.unwrap();
    assert_eq!(request.reject(), Ok(()));
    assert!(matches!(within(response).await, Err(BusError::GoalRejected { name }) if name == NAME));
    assert_eq!(bus.actions()[0].active, 0);
}

#[tokio::test]
async fn each_terminal_transition_reaches_the_client_and_only_the_first_one_counts() {
    let bus = Bus::new();
    let mut srv = bus.scope("nav").action_server::<WaypointAction>(NAME).unwrap();
    let client = bus.scope("coverage").action_client::<WaypointAction>(NAME).unwrap();
    type End = fn(&mut ActionServerGoal<WaypointAction>, Waypoint::Result) -> Result<(), BusError>;
    let ends: [(End, GoalStatus); 3] = [
        (ActionServerGoal::succeed, GoalStatus::Succeeded),
        (ActionServerGoal::abort, GoalStatus::Aborted),
        // with no cancel request: r2r allows it, rcl_action would not
        (ActionServerGoal::cancel, GoalStatus::Canceled),
    ];
    for (end, status) in ends {
        let ((mut server_goal, mut cancels), (handle, result_future, mut feedbacks)) = accept_one(&mut srv, &client, "x").await;
        end(&mut server_goal, result(status == GoalStatus::Succeeded)).unwrap();
        assert_eq!(within(result_future).await, Ok((status, result(status == GoalStatus::Succeeded))));
        assert_eq!(handle.get_status(), Ok(status));
        // Ended: nothing ends it again, no feedback, no cancel.
        assert_eq!(server_goal.succeed(result(true)), Err(terminated()));
        assert_eq!(server_goal.abort(result(false)), Err(terminated()));
        assert_eq!(server_goal.cancel(result(false)), Err(terminated()));
        assert_eq!(server_goal.publish_feedback(feedback("late")), Err(terminated()));
        assert_eq!(server_goal.is_cancelling(), Ok(false));
        assert_eq!(within(handle.cancel().unwrap()).await, Err(terminated()));
        assert!(within(feedbacks.next()).await.is_none());
        assert!(within(cancels.next()).await.is_none(), "the cancel stream ends with the goal");
    }
}

#[tokio::test]
async fn feedback_beyond_the_queue_drops_the_oldest_and_counts_it() {
    let bus = Bus::new();
    let mut srv = bus.scope("nav").action_server::<WaypointAction>(NAME).unwrap();
    let client = bus.scope("coverage").action_client::<WaypointAction>(NAME).unwrap();
    let ((server_goal, _cancels), (_handle, _result, mut feedbacks)) = accept_one(&mut srv, &client, "a").await;
    for i in 0..13 {
        server_goal.publish_feedback(feedback(&i.to_string())).unwrap();
    }
    assert_eq!(feedbacks.dropped(), 3);
    let first = within(feedbacks.next()).await.unwrap();
    assert_eq!(first.feedback, "3", "the newest 10 are kept, as a DDS reader of the feedback topic keeps them");
}

// ---------------------------------------------------------------------------
// Cancel requests
// ---------------------------------------------------------------------------

#[tokio::test]
async fn an_accepted_cancel_makes_the_goal_canceling_until_its_server_ends_it() {
    let bus = Bus::new();
    let mut srv = bus.scope("nav").action_server::<WaypointAction>(NAME).unwrap();
    let client = bus.scope("coverage").action_client::<WaypointAction>(NAME).unwrap();
    let ((mut server_goal, mut cancels), (handle, result_future, _feedbacks)) = accept_one(&mut srv, &client, "a").await;

    let canceling = handle.cancel().unwrap();
    let request: ActionServerCancelRequest = within(cancels.next()).await.unwrap();
    assert_eq!(request.id, handle.id);
    request.accept();
    assert_eq!(within(canceling).await, Ok(()));
    assert_eq!(handle.get_status(), Ok(GoalStatus::Canceling));
    assert_eq!(server_goal.is_cancelling(), Ok(true), "r2r fails here: it forgets the goal on an accepted cancel");
    // A second request is answered at once, as rcl_action answers it, and
    // never reaches the server.
    assert_eq!(within(handle.cancel().unwrap()).await, Err(terminated()));
    server_goal.cancel(result(false)).unwrap();
    assert_eq!(within(result_future).await, Ok((GoalStatus::Canceled, result(false))));
    assert!(within(cancels.next()).await.is_none(), "the second request was never handed out");
}

#[tokio::test]
async fn a_goal_canceling_can_still_succeed_or_abort() {
    let bus = Bus::new();
    let mut srv = bus.scope("nav").action_server::<WaypointAction>(NAME).unwrap();
    let client = bus.scope("coverage").action_client::<WaypointAction>(NAME).unwrap();
    for (status, success) in [(GoalStatus::Succeeded, true), (GoalStatus::Aborted, false)] {
        let ((mut server_goal, mut cancels), (handle, result_future, _f)) = accept_one(&mut srv, &client, "a").await;
        let canceling = handle.cancel().unwrap();
        within(cancels.next()).await.unwrap().accept();
        within(canceling).await.unwrap();
        // rcl_action allows these from CANCELING; r2r refuses them there.
        if success {
            server_goal.succeed(result(true)).unwrap();
        } else {
            server_goal.abort(result(false)).unwrap();
        }
        assert_eq!(within(result_future).await, Ok((status, result(success))));
    }
}

#[tokio::test]
async fn a_rejected_cancel_leaves_the_goal_running() {
    let bus = Bus::new();
    let mut srv = bus.scope("nav").action_server::<WaypointAction>(NAME).unwrap();
    let client = bus.scope("coverage").action_client::<WaypointAction>(NAME).unwrap();
    let ((mut server_goal, mut cancels), (handle, result_future, _feedbacks)) = accept_one(&mut srv, &client, "a").await;
    let canceling = handle.cancel().unwrap();
    within(cancels.next()).await.unwrap().reject();
    assert_eq!(within(canceling).await, Err(BusError::CancelRejected { name: NAME.into() }));
    assert_eq!(handle.get_status(), Ok(GoalStatus::Executing));
    // asked again, the server is asked again
    let again = handle.cancel().unwrap();
    within(cancels.next()).await.unwrap().accept();
    assert_eq!(within(again).await, Ok(()));
    server_goal.cancel(result(false)).unwrap();
    assert_eq!(within(result_future).await, Ok((GoalStatus::Canceled, result(false))));
}

#[test]
fn a_cancel_sent_before_the_server_looks_at_its_cancel_requests_waits_for_it() {
    let bus = Bus::new();
    let (nav, coverage) = (bus.scope("nav"), bus.scope("coverage"));
    let mut srv = nav.action_server::<WaypointAction>(NAME).unwrap();
    let client = coverage.action_client::<WaypointAction>(NAME).unwrap();
    let woken = Woken::new();
    let mut response = client.send_goal_request(goal("a")).unwrap();
    let Poll::Ready(Some(request)) = poll_next_with(&mut srv, &woken) else { panic!("not handed out") };
    let (_server_goal, mut cancels) = request.accept().unwrap();
    let Poll::Ready(Ok((handle, _result, _f))) = poll_with(&mut response, &woken) else { panic!("not accepted") };

    // The client cancels at once; the server has not polled its stream yet.
    let mut first = handle.cancel().unwrap();
    let mut second = handle.cancel().unwrap();
    assert!(poll_with(&mut first, &woken).is_pending());
    let Poll::Ready(Some(cancel)) = poll_next_with(&mut cancels, &woken) else { panic!("the queued cancel is lost") };
    let Poll::Ready(Some(also)) = poll_next_with(&mut cancels, &woken) else { panic!("the second is lost") };
    assert!(poll_next_with(&mut cancels, &woken).is_pending());
    cancel.accept();
    assert!(woken.take(), "the answer wakes the client");
    assert_eq!(poll_with(&mut first, &woken), Poll::Ready(Ok(())));
    // Both were taken while the goal executed, so both reach the server, as
    // they would through rcl_action.
    also.accept();
    assert_eq!(poll_with(&mut second, &woken), Poll::Ready(Ok(())));
}

#[tokio::test]
async fn cancel_requests_are_rejected_once_the_server_drops_its_cancel_stream_and_fail_if_dropped_unanswered() {
    let bus = Bus::new();
    let mut srv = bus.scope("nav").action_server::<WaypointAction>(NAME).unwrap();
    let client = bus.scope("coverage").action_client::<WaypointAction>(NAME).unwrap();
    let ((_server_goal, mut cancels), (handle, _result, _feedbacks)) = accept_one(&mut srv, &client, "a").await;
    let dropped = handle.cancel().unwrap();
    drop(within(cancels.next()).await.unwrap());
    assert_eq!(within(dropped).await, Err(no_response()));
    let queued = handle.cancel().unwrap();
    drop(cancels);
    let rejected = BusError::CancelRejected { name: NAME.into() };
    assert_eq!(within(queued).await, Err(rejected.clone()), "queued when the stream went");
    assert_eq!(within(handle.cancel().unwrap()).await, Err(rejected), "sent after it went");
    assert_eq!(handle.get_status(), Ok(GoalStatus::Executing));
}

#[tokio::test]
async fn a_server_that_ends_the_goal_before_answering_a_cancel_tells_the_client_it_has_ended() {
    let bus = Bus::new();
    let mut srv = bus.scope("nav").action_server::<WaypointAction>(NAME).unwrap();
    let client = bus.scope("coverage").action_client::<WaypointAction>(NAME).unwrap();
    let ((mut server_goal, mut cancels), (handle, result_future, _f)) = accept_one(&mut srv, &client, "a").await;
    let taken = handle.cancel().unwrap();
    let request = within(cancels.next()).await.unwrap();
    let queued = handle.cancel().unwrap();
    server_goal.succeed(result(true)).unwrap();
    assert_eq!(within(queued).await, Err(terminated()), "a request not taken yet is answered by the end");
    request.accept();
    assert_eq!(within(taken).await, Err(terminated()), "an accept after the end");
    assert_eq!(within(result_future).await, Ok((GoalStatus::Succeeded, result(true))));
}

// ---------------------------------------------------------------------------
// Requests and handles dropped, servers and clients that go away
// ---------------------------------------------------------------------------

#[tokio::test]
async fn a_goal_request_dropped_unanswered_fails_with_no_response() {
    let bus = Bus::new();
    let mut srv = bus.scope("nav").action_server::<WaypointAction>(NAME).unwrap();
    let client = bus.scope("coverage").action_client::<WaypointAction>(NAME).unwrap();
    let response = client.send_goal_request(goal("a")).unwrap();
    drop(within(srv.next()).await.unwrap());
    assert!(matches!(within(response).await, Err(e) if e == no_response()));

    // Moving the goal out drops the rest of the request, unanswered.
    let response = client.send_goal_request(goal("b")).unwrap();
    let ActionServerGoalRequest { goal: moved, .. } = within(srv.next()).await.unwrap();
    assert_eq!(moved, goal("b"));
    assert!(matches!(within(response).await, Err(e) if e == no_response()));
}

/// A goal whose server drops it without ending it may still be running
/// (mower_nav's task could have died with nav2 driving), so its client
/// gets no terminal status, which mower_coverage would take as proof, but
/// an error: its "uncertain" path.
#[tokio::test]
async fn a_server_goal_dropped_before_it_ended_is_lost_for_its_client_with_no_response() {
    let bus = Bus::new();
    let mut srv = bus.scope("nav").action_server::<WaypointAction>(NAME).unwrap();
    let client = bus.scope("coverage").action_client::<WaypointAction>(NAME).unwrap();
    let ((server_goal, cancels), (handle, result_future, mut feedbacks)) = accept_one(&mut srv, &client, "a").await;
    let clone = server_goal.clone();
    server_goal.publish_feedback(feedback("before")).unwrap();
    let pending_cancel = handle.cancel().unwrap();
    drop(server_goal);
    assert_eq!(handle.get_status(), Ok(GoalStatus::Executing), "a clone still holds it");
    drop((clone, cancels));
    assert_eq!(within(result_future).await, Err(no_response()));
    assert_eq!(handle.get_status(), Ok(GoalStatus::Unknown));
    assert_eq!(within(pending_cancel).await, Err(no_response()));
    assert_eq!(within(handle.cancel().unwrap()).await, Err(no_response()));
    assert_eq!(within(feedbacks.next()).await, Some(feedback("before")), "what came before is still read");
    assert!(within(feedbacks.next()).await.is_none());
    assert_eq!(bus.actions()[0].active, 0);
}

/// What a client sees when its server's module restarts mid-goal: the goal
/// is lost, with an error that is not a terminal status, at once (r2r's
/// client would wait for a result that never comes). The restarted server
/// serves the next goal.
#[tokio::test]
async fn closing_the_servers_scope_loses_its_goals_and_fails_its_requests() {
    let bus = Bus::new();
    let (nav, coverage) = (bus.scope("nav"), bus.scope("coverage"));
    let mut srv = nav.action_server::<WaypointAction>(NAME).unwrap();
    let client = coverage.action_client::<WaypointAction>(NAME).unwrap();
    let ((mut server_goal, mut cancels), (handle, result_future, mut feedbacks)) = accept_one(&mut srv, &client, "a").await;
    server_goal.publish_feedback(feedback("before")).unwrap();
    let taken_cancel = handle.cancel().unwrap();
    let cancel_in_hand = within(cancels.next()).await.unwrap();
    let handed_out = client.send_goal_request(goal("b")).unwrap();
    let request_in_hand = within(srv.next()).await.unwrap();
    let queued = client.send_goal_request(goal("c")).unwrap();
    let rejected_later = client.send_goal_request(goal("d")).unwrap();

    nav.close();
    assert_eq!(within(result_future).await, Err(unavailable()));
    assert_eq!(handle.get_status(), Ok(GoalStatus::Unknown));
    assert_eq!(within(taken_cancel).await, Err(unavailable()));
    assert_eq!(within(handle.cancel().unwrap()).await, Err(unavailable()));
    assert_eq!(within(feedbacks.next()).await, Some(feedback("before")));
    assert!(within(feedbacks.next()).await.is_none());
    for response in [handed_out, queued, rejected_later] {
        assert!(matches!(within(response).await, Err(e) if e == unavailable()));
    }
    // The server's side refuses everything with `Closed`.
    assert_eq!(server_goal.succeed(result(true)), Err(closed("nav")));
    assert_eq!(server_goal.publish_feedback(feedback("late")), Err(closed("nav")));
    assert_eq!(server_goal.is_cancelling(), Err(closed("nav")));
    cancel_in_hand.accept(); // answered by the close already: nothing happens
    assert!(matches!(request_in_hand.accept(), Err(e) if e == closed("nav")));
    assert!(within(srv.next()).await.is_none());
    assert!(within(cancels.next()).await.is_none());
    assert!(!client.is_available());
    assert_eq!(client.send_goal_request(goal("e")).unwrap_err(), unavailable());
    let info = &bus.actions()[0];
    assert_eq!((info.server.as_deref(), info.active), (None, 0));

    // The restarted module serves the name again.
    let restarted = bus.scope("nav");
    let mut srv = restarted.action_server::<WaypointAction>(NAME).unwrap();
    within(client.wait_for_server()).await.unwrap();
    let ((mut server_goal, _c), (_h, result_future, _f)) = accept_one(&mut srv, &client, "f").await;
    server_goal.succeed(result(true)).unwrap();
    assert_eq!(within(result_future).await, Ok((GoalStatus::Succeeded, result(true))));
}

#[tokio::test]
async fn dropping_the_action_server_stops_new_goals_but_not_those_it_handed_out_or_accepted() {
    let bus = Bus::new();
    let nav = bus.scope("nav");
    let mut srv = nav.action_server::<WaypointAction>(NAME).unwrap();
    let client = bus.scope("coverage").action_client::<WaypointAction>(NAME).unwrap();
    let ((mut running, _cancels), (_handle, running_result, _f)) = accept_one(&mut srv, &client, "a").await;
    let handed_out = client.send_goal_request(goal("b")).unwrap();
    let in_hand = within(srv.next()).await.unwrap();
    let queued = client.send_goal_request(goal("c")).unwrap();

    drop(srv);
    assert!(matches!(within(queued).await, Err(e) if e == unavailable()));
    assert!(!client.is_available());
    // A new server can take the name while the old one's goals run on.
    let mut next = bus.scope("nav_again").action_server::<WaypointAction>(NAME).unwrap();
    let (late_goal, _late_cancels) = in_hand.accept().unwrap();
    let (_late_handle, late_result, _late_f) = within(handed_out).await.unwrap();
    drop(late_goal);
    assert_eq!(within(late_result).await, Err(no_response()));
    running.succeed(result(true)).unwrap();
    assert_eq!(within(running_result).await, Ok((GoalStatus::Succeeded, result(true))));
    let ((mut fresh, _c), (_h, fresh_result, _f)) = accept_one(&mut next, &client, "d").await;
    fresh.abort(result(false)).unwrap();
    assert_eq!(within(fresh_result).await, Ok((GoalStatus::Aborted, result(false))));
}

/// A client's scope closing abandons its goals, as a ROS client process that
/// dies does: they run on, unknown to their server, and the client's own
/// futures end with `Closed`.
#[tokio::test]
async fn closing_the_clients_scope_abandons_its_goals() {
    let bus = Bus::new();
    let (nav, coverage) = (bus.scope("nav"), bus.scope("coverage"));
    let mut srv = nav.action_server::<WaypointAction>(NAME).unwrap();
    let client = coverage.action_client::<WaypointAction>(NAME).unwrap();
    let ((mut server_goal, mut cancels), (handle, result_future, mut feedbacks)) = accept_one(&mut srv, &client, "a").await;
    let waiting = client.send_goal_request(goal("b")).unwrap();
    let pending_cancel = handle.cancel().unwrap();
    let cancel_in_hand = within(cancels.next()).await.unwrap();

    coverage.close();
    assert_eq!(within(result_future).await, Err(closed("coverage")));
    assert!(within(feedbacks.next()).await.is_none());
    assert_eq!(within(pending_cancel).await, Err(closed("coverage")));
    assert!(matches!(within(waiting).await, Err(e) if e == closed("coverage")));
    assert_eq!(handle.get_status(), Err(closed("coverage")));
    assert_eq!(handle.cancel().unwrap_err(), closed("coverage"));
    assert_eq!(client.send_goal_request(goal("c")).unwrap_err(), closed("coverage"));
    assert_eq!(within(client.wait_for_server()).await, Err(closed("coverage")));

    // The server's goal runs on; nothing told it. The request sent before
    // the close still reaches it, as a service call does.
    cancel_in_hand.reject();
    assert_eq!(server_goal.is_cancelling(), Ok(false));
    server_goal.publish_feedback(feedback("nobody reads this")).unwrap();
    let late = within(srv.next()).await.unwrap();
    assert_eq!(late.goal, goal("b"));
    let (mut late_goal, _late_cancels) = late.accept().unwrap();
    assert_eq!(bus.actions()[0].active, 2);
    server_goal.succeed(result(true)).unwrap();
    late_goal.abort(result(false)).unwrap();
    assert_eq!(bus.actions()[0].active, 0);
}

/// rcl_action cannot cancel a goal before the server accepts it (there is no
/// handle yet, and the server does not know the id); neither can the bus.
/// A client that gives up waiting leaves the goal to the server, which can
/// still accept it, as with r2r: mower_coverage keeps the response future to
/// adopt and cancel such a goal.
#[tokio::test]
async fn a_client_that_gives_up_before_the_answer_leaves_the_goal_to_run_without_feedback() {
    let bus = Bus::new();
    let mut srv = bus.scope("nav").action_server::<WaypointAction>(NAME).unwrap();
    let client = bus.scope("coverage").action_client::<WaypointAction>(NAME).unwrap();
    drop(client.send_goal_request(goal("given up")).unwrap());
    let request = within(srv.next()).await.unwrap();
    let (mut server_goal, _cancels) = request.accept().unwrap();
    for _ in 0..20 {
        server_goal.publish_feedback(feedback("dropped at once")).unwrap();
    }
    assert_eq!(bus.actions()[0].active, 1);
    server_goal.succeed(result(true)).unwrap();
    assert_eq!(bus.actions()[0].active, 0);

    // One whose answer arrived but was never read: the same.
    let response = client.send_goal_request(goal("unread")).unwrap();
    let (server_goal, _cancels) = within(srv.next()).await.unwrap().accept().unwrap();
    drop(response);
    server_goal.publish_feedback(feedback("dropped at once")).unwrap();
}

// ---------------------------------------------------------------------------
// Several goals
// ---------------------------------------------------------------------------

/// mower_nav serves both action names with a task per goal request, and
/// admits one goal at a time by its own state, not the action's: the bus
/// itself runs any number at once.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn goals_run_concurrently_and_each_gets_its_own_cancel_feedback_and_result() {
    const GOALS: usize = 4;
    let bus = Bus::new();
    let mut srv = bus.scope("nav").action_server::<WaypointAction>(NAME).unwrap();
    let serving = tokio::spawn(async move {
        let mut tasks = Vec::new();
        while let Some(request) = srv.next().await {
            tasks.push(tokio::spawn(async move {
                let id = request.goal.dispatch_id.clone();
                let (mut server_goal, mut cancels) = request.accept().unwrap();
                server_goal.publish_feedback(feedback(&format!("{id} started"))).unwrap();
                // goal "1" waits to be canceled; the others end on their own
                if id == "1" {
                    let cancel = cancels.next().await.unwrap();
                    cancel.accept();
                    server_goal.cancel(result(false)).unwrap();
                } else {
                    tokio::task::yield_now().await;
                    server_goal.succeed(result(true)).unwrap();
                }
            }));
        }
        for task in tasks {
            task.await.unwrap();
        }
    });
    let scopes: Vec<Scope> = (0..2).map(|i| bus.scope(&format!("client{i}"))).collect();
    let mut goals = Vec::new();
    for i in 0..GOALS {
        let client = scopes[i % 2].action_client::<WaypointAction>(NAME).unwrap();
        goals.push(within(client.send_goal_request(goal(&i.to_string())).unwrap()).await.unwrap());
    }
    let ids: HashSet<_> = goals.iter().map(|(handle, _, _)| handle.id).collect();
    assert_eq!(ids.len(), GOALS, "every goal has an id of its own");
    let (handle, _, _) = &goals[1];
    within(handle.cancel().unwrap()).await.unwrap();
    for (i, (_handle, result_future, mut feedbacks)) in goals.into_iter().enumerate() {
        assert_eq!(within(feedbacks.next()).await, Some(feedback(&format!("{i} started"))));
        let expected = if i == 1 { (GoalStatus::Canceled, result(false)) } else { (GoalStatus::Succeeded, result(true)) };
        assert_eq!(within(result_future).await, Ok(expected));
    }
    assert_eq!(bus.actions()[0].active, 0, "nothing is left running");
    drop(scopes);
    serving.abort();
}

// ---------------------------------------------------------------------------
// Endpoints, names, types and introspection
// ---------------------------------------------------------------------------

#[test]
fn a_second_server_is_refused_until_the_first_drops_or_its_scope_closes() {
    let bus = Bus::new();
    let (first, second, third) = (bus.scope("first"), bus.scope("second"), bus.scope("third"));
    let taken = |server: &str| BusError::ServiceTaken { name: NAME.into(), server: server.into() };
    let srv = first.action_server::<WaypointAction>(NAME).unwrap();
    assert_eq!(second.action_server::<WaypointAction>(NAME).unwrap_err(), taken("first"));
    drop(srv);
    let _srv = second.action_server::<WaypointAction>(NAME).unwrap();
    assert_eq!(third.action_server::<WaypointAction>(NAME).unwrap_err(), taken("second"));
    second.close();
    let _srv = third.action_server::<WaypointAction>(NAME).unwrap();
    // A service or a topic of the same name is another namespace.
    let _service = third.service::<Waypoint::Goal, Waypoint::Result>(NAME).unwrap();
}

#[test]
fn the_first_endpoint_fixes_an_actions_three_types_for_the_bus_lifetime() {
    let bus = Bus::new();
    let scope = bus.scope("m");
    let mismatch = |existing, requested| BusError::TypeMismatch { name: NAME.into(), existing, requested };
    let client = scope.action_client::<WaypointAction>(NAME).unwrap();
    assert_eq!(
        scope.action_server::<(u32, Waypoint::Result, Waypoint::Feedback)>(NAME).unwrap_err(),
        mismatch(type_name::<Waypoint::Goal>(), type_name::<u32>()),
    );
    assert_eq!(
        scope.action_client::<(Waypoint::Goal, u32, Waypoint::Feedback)>(NAME).unwrap_err(),
        mismatch(type_name::<Waypoint::Result>(), type_name::<u32>()),
    );
    drop(client);
    assert_eq!(
        scope.action_server::<(Waypoint::Goal, Waypoint::Result, u32)>(NAME).unwrap_err(),
        mismatch(type_name::<Waypoint::Feedback>(), type_name::<u32>()),
    );
    assert!(scope.action_server::<WaypointAction>(NAME).is_ok());
}

#[test]
fn action_names_must_be_absolute_and_closed_scopes_make_no_endpoints() {
    let bus = Bus::new();
    let scope = bus.scope("m");
    assert!(matches!(scope.action_server::<WaypointAction>("nav_action"), Err(BusError::InvalidName { .. })));
    assert!(matches!(scope.action_client::<WaypointAction>("/nav_action/"), Err(BusError::InvalidName { .. })));
    let gone = bus.scope("gone");
    gone.close();
    assert_eq!(gone.action_server::<WaypointAction>(NAME).unwrap_err(), closed("gone"));
    assert_eq!(gone.action_client::<WaypointAction>(NAME).unwrap_err(), closed("gone"));
    assert!(bus.actions().is_empty(), "nothing fixed a type");
}

#[tokio::test]
async fn a_goal_with_no_live_server_fails_at_once_and_wait_for_server_waits_for_one() {
    let bus = Bus::new();
    let coverage = bus.scope("coverage");
    let client = coverage.action_client::<WaypointAction>(NAME).unwrap();
    assert!(!client.is_available());
    assert_eq!(client.send_goal_request(goal("a")).unwrap_err(), unavailable());
    let mut wait = Box::pin(client.wait_for_server());
    let woken = Woken::new();
    assert!(poll_with(&mut wait, &woken).is_pending());
    // The wait holds its client, as r2r's `is_available` future does: it can
    // be spawned, and outlive the handle it came from.
    let spawned = tokio::spawn(coverage.action_client::<WaypointAction>(NAME).unwrap().wait_for_server());
    tokio::task::yield_now().await;
    assert!(!spawned.is_finished());
    let _srv = bus.scope("nav").action_server::<WaypointAction>(NAME).unwrap();
    assert!(woken.take(), "the server's registration wakes the waiter");
    assert_eq!(poll_with(&mut wait, &woken), Poll::Ready(Ok(())));
    assert_eq!(within(spawned).await.unwrap(), Ok(()));
    assert!(client.is_available());
    assert_eq!(client.wait_for_server().now_or_never(), Some(Ok(())));
}

#[test]
fn introspection_lists_each_actions_types_server_clients_and_goals() {
    let bus = Bus::new();
    let (nav, coverage, tool) = (bus.scope("nav_action_server"), bus.scope("boustrophedon_coverage"), bus.scope("tool"));
    let mut srv = nav.action_server::<WaypointAction>(NAME).unwrap();
    let client = coverage.action_client::<WaypointAction>(NAME).unwrap();
    let _clone = client.clone();
    let _other = tool.action_client::<WaypointAction>(NAME).unwrap();
    let _plain = tool.action_client::<(u32, u32, u32)>("/count").unwrap();
    {
        let ((_g, _c), _client_side) = accept_now(&mut srv, &client, "a");
        drop(client.send_goal_request(goal("b")).unwrap());
        assert!(client.send_goal_request(goal("c")).is_ok());
        let info = bus.actions();
        let count = ActionInfo {
            name: "/count".into(),
            goal_type: "u32",
            result_type: "u32",
            feedback_type: "u32",
            ros_type: None,
            server: None,
            clients: vec!["tool".into()],
            goals: 0,
            active: 0,
        };
        let waypoint = ActionInfo {
            name: NAME.into(),
            goal_type: type_name::<Waypoint::Goal>(),
            result_type: type_name::<Waypoint::Result>(),
            feedback_type: type_name::<Waypoint::Feedback>(),
            ros_type: Some("mower_interface/action/Waypoint".into()),
            server: Some("nav_action_server".into()),
            // a client and its clone are one client
            clients: vec!["boustrophedon_coverage".into(), "tool".into()],
            goals: 3,
            active: 1,
        };
        assert_eq!(info, vec![count, waypoint]);
    }
    tool.close();
    assert_eq!(bus.actions()[1].clients, ["boustrophedon_coverage"]);
}

// ---------------------------------------------------------------------------
// Runtimes and panics
// ---------------------------------------------------------------------------

#[test]
fn a_goal_served_from_a_std_thread_completes_without_any_runtime() {
    let bus = Bus::new();
    let (nav, coverage) = (bus.scope("nav"), bus.scope("coverage"));
    let mut srv = nav.action_server::<WaypointAction>(NAME).unwrap();
    let serving = thread::spawn(move || {
        while let Some(request) = futures::executor::block_on(srv.next()) {
            let (mut server_goal, mut cancels) = request.accept().unwrap();
            // each goal on a thread of its own, which waits for a cancel
            thread::spawn(move || {
                server_goal.publish_feedback(feedback("waiting for a cancel")).unwrap();
                let cancel = futures::executor::block_on(cancels.next()).unwrap();
                cancel.accept();
                server_goal.cancel(result(false)).unwrap();
            });
        }
    });
    let client = coverage.action_client::<WaypointAction>(NAME).unwrap();
    for i in 0..3 {
        let (handle, result_future, mut feedbacks) =
            futures::executor::block_on(client.send_goal_request(goal(&i.to_string())).unwrap()).unwrap();
        assert_eq!(futures::executor::block_on(feedbacks.next()), Some(feedback("waiting for a cancel")));
        assert_eq!(futures::executor::block_on(handle.cancel().unwrap()), Ok(()));
        assert_eq!(futures::executor::block_on(result_future), Ok((GoalStatus::Canceled, result(false))));
    }
    nav.close();
    serving.join().unwrap();
}

/// A waker that panics when woken.
struct Exploding;

impl ArcWake for Exploding {
    fn wake_by_ref(_: &Arc<Self>) {
        panic!("waker exploded");
    }
}

fn exploding() -> Waker {
    futures::task::waker(Arc::new(Exploding))
}

#[test]
fn a_client_whose_waker_panics_keeps_no_other_client_asleep_when_the_server_closes() {
    let bus = Bus::new();
    let (nav, coverage) = (bus.scope("nav"), bus.scope("coverage"));
    let mut srv = nav.action_server::<WaypointAction>(NAME).unwrap();
    let client = coverage.action_client::<WaypointAction>(NAME).unwrap();
    let ((_g1, _c1), (_h1, mut broken, _f1)) = accept_now(&mut srv, &client, "a");
    let ((_g2, _c2), (_h2, mut fine, mut feedbacks)) = accept_now(&mut srv, &client, "b");
    let mut queued = client.send_goal_request(goal("c")).unwrap();
    assert!(Pin::new(&mut broken).poll(&mut Context::from_waker(&exploding())).is_pending());
    let woken = Woken::new();
    assert!(poll_with(&mut fine, &woken).is_pending());
    assert!(poll_next_with(&mut feedbacks, &woken).is_pending());
    let queued_woken = Woken::new();
    assert!(poll_with(&mut queued, &queued_woken).is_pending());

    assert!(catch_unwind(AssertUnwindSafe(|| nav.close())).is_err(), "the waker's panic goes on");
    assert!(woken.take() && queued_woken.take(), "the others were woken");
    assert_eq!(poll_with(&mut fine, &woken), Poll::Ready(Err(unavailable())));
    assert_eq!(poll_next_with(&mut feedbacks, &woken), Poll::Ready(None));
    assert!(matches!(poll_with(&mut queued, &queued_woken), Poll::Ready(Err(e)) if e == unavailable()));
    assert!(nav.is_closed());
}

#[test]
fn handles_are_send_and_unpin_and_the_futures_and_streams_can_be_spawned() {
    fn send_unpin<T: Send + Unpin>() {}
    fn spawnable<T: Send + Unpin + 'static>() {}
    fn cloneable<T: Clone>() {}
    fn shared<T: Send + Sync>() {}
    fn spawnable_future<F: Future + Send + Unpin + 'static>(_: F) {}

    send_unpin::<ActionServer<WaypointAction>>();
    spawnable::<ActionServerGoalRequest<WaypointAction>>();
    spawnable::<ActionServerGoal<WaypointAction>>();
    spawnable::<CancelRequests>();
    spawnable::<ActionServerCancelRequest>();
    send_unpin::<ActionClient<WaypointAction>>();
    spawnable::<GoalResponseFuture<WaypointAction>>();
    spawnable::<ActionClientGoal<WaypointAction>>();
    spawnable::<ResultFuture<WaypointAction>>();
    spawnable::<FeedbackStream<WaypointAction>>();
    spawnable::<CancelFuture>();
    cloneable::<ActionClient<WaypointAction>>();
    cloneable::<ActionClientGoal<WaypointAction>>();
    cloneable::<ActionServerGoal<WaypointAction>>();
    // mower_coverage keeps goal handles in an `Arc<Mutex<_>>` across tasks
    shared::<Mutex<ActionClientGoal<WaypointAction>>>();
    shared::<Mutex<ActionServerGoal<WaypointAction>>>();
    let bus = Bus::new();
    let client = bus.scope("m").action_client::<WaypointAction>(NAME).unwrap();
    spawnable_future(client.wait_for_server());
}

// ---------------------------------------------------------------------------
// mower_coverage's client against mower_nav's server
// ---------------------------------------------------------------------------

/// mower_nav's server, cut down to what its action sees
/// (crates/mower_nav/src/lib.rs, `handle_goal_request` and `Execution`): a
/// task per request; admission (a reused dispatch id is rejected); a cancel
/// request accepted for the reserved goal, noted in a flag; and
/// `terminate`, which tries the intended transition first and falls back to
/// `cancel`. Each goal runs until the test tells it how to end, or until a
/// cancel was accepted.
struct NavLike {
    ends: tokio::sync::mpsc::UnboundedSender<Outcome>,
    accepted: Arc<AtomicU64>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Outcome {
    Succeeded,
    Aborted,
}

fn nav_like(scope: &Scope, accept_after: Duration) -> NavLike {
    let mut requests = scope.action_server::<WaypointAction>(NAME).unwrap();
    let (ends, end_rx) = tokio::sync::mpsc::unbounded_channel::<Outcome>();
    let end_rx = Arc::new(tokio::sync::Mutex::new(end_rx));
    let accepted = Arc::new(AtomicU64::new(0));
    let seen = Arc::new(Mutex::new(HashSet::new()));
    {
        let accepted = accepted.clone();
        tokio::spawn(async move {
            while let Some(req) = requests.next().await {
                let (seen, end_rx, accepted) = (seen.clone(), end_rx.clone(), accepted.clone());
                tokio::spawn(handle_goal_request(req, seen, end_rx, accepted, accept_after));
            }
        });
    }
    NavLike { ends, accepted }
}

async fn handle_goal_request(
    req: ActionServerGoalRequest<WaypointAction>, seen: Arc<Mutex<HashSet<String>>>,
    ends: Arc<tokio::sync::Mutex<tokio::sync::mpsc::UnboundedReceiver<Outcome>>>, accepted: Arc<AtomicU64>,
    accept_after: Duration,
) {
    if !seen.lock().unwrap().insert(req.goal.dispatch_id.clone()) {
        let _ = req.reject();
        return;
    }
    tokio::time::sleep(accept_after).await;
    let (goal_handle, mut cancel_requests) = match req.accept() {
        Ok(v) => v,
        Err(_) => return,
    };
    accepted.fetch_add(1, Ordering::SeqCst);
    let action_cancel_accepted = Arc::new(AtomicBool::new(false));
    {
        let accepted = action_cancel_accepted.clone();
        tokio::spawn(async move {
            while let Some(request) = cancel_requests.next().await {
                accepted.store(true, Ordering::SeqCst);
                request.accept();
            }
        });
    }
    let goal = Mutex::new(goal_handle);
    let terminate = |outcome: Option<Outcome>| {
        let mut goal = goal.lock().unwrap();
        let first = match outcome {
            Some(Outcome::Succeeded) => goal.succeed(result(true)),
            Some(Outcome::Aborted) => goal.abort(result(false)),
            None if action_cancel_accepted.load(Ordering::SeqCst) => goal.cancel(result(false)),
            None => goal.abort(result(false)),
        };
        if first.is_err() {
            let _ = goal.cancel(result(false));
        }
    };
    loop {
        if action_cancel_accepted.load(Ordering::SeqCst) {
            return terminate(None);
        }
        let next = { ends.lock().await.try_recv() };
        match next {
            Ok(outcome) => return terminate(Some(outcome)),
            // The test let this server go (its module "restarted").
            Err(tokio::sync::mpsc::error::TryRecvError::Disconnected) => return,
            Err(tokio::sync::mpsc::error::TryRecvError::Empty) => {}
        }
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

type Handle = Arc<Mutex<ActionClientGoal<WaypointAction>>>;
type Slot = tokio::sync::watch::Receiver<Option<Result<(GoalStatus, Waypoint::Result), String>>>;

/// mower_coverage's `send_follow_path` up to the acceptance, and
/// `Handles::adopt` (crates/mower_coverage/src/lib.rs), with the bus in
/// place of r2r: the r2r calls swapped one for one.
async fn send_follow_path(client: &ActionClient<WaypointAction>, dispatch_id: &str) -> Result<(Handle, Slot), String> {
    let available = tokio::time::timeout(Duration::from_secs(2), client.wait_for_server()).await.is_ok_and(|r| r.is_ok());
    if !available {
        return Err("Navigation action server unavailable".into());
    }
    let fut = match client.send_goal_request(goal(dispatch_id)) {
        Ok(f) => f,
        Err(e) => return Err(format!("Navigation action goal dispatch failed: {e:?}")),
    };
    let mut boxed: futures::future::BoxFuture<'static, _> = Box::pin(fut);
    let accepted = match tokio::time::timeout(Duration::from_secs(3), &mut boxed).await {
        Ok(r) => r,
        Err(_) => {
            // a late acceptance is adopted and canceled
            tokio::spawn(async move {
                if let Ok((goal, result, _feedback)) = boxed.await {
                    let (handle, _slot) = adopt(goal, result);
                    let cancel = handle.lock().unwrap().cancel().ok();
                    if let Some(fut) = cancel {
                        let _ = fut.await;
                    }
                }
            });
            return Err("acceptance timed out".into());
        }
    };
    let (goal, result, _feedback) = match accepted {
        Ok(v) => v,
        Err(BusError::GoalRejected { .. }) => return Err("Navigation action goal rejected".into()),
        Err(e) => return Err(format!("Navigation action goal response failed ({e:?}); dispatch outcome is uncertain")),
    };
    Ok(adopt(goal, result))
}

fn adopt(goal: ActionClientGoal<WaypointAction>, result: ResultFuture<WaypointAction>) -> (Handle, Slot) {
    let handle: Handle = Arc::new(Mutex::new(goal));
    let (tx, rx) = tokio::sync::watch::channel(None);
    tokio::spawn(async move {
        let outcome = result.await.map_err(|e| format!("{e:?}"));
        let _ = tx.send(Some(outcome));
    });
    (handle, rx)
}

/// `_proven_action_result`: only SUCCEEDED / CANCELED / ABORTED prove a
/// terminal state.
async fn proven(mut slot: Slot) -> Result<(GoalStatus, Waypoint::Result), String> {
    let outcome = within(slot.wait_for(Option::is_some)).await.unwrap().clone().unwrap();
    match outcome {
        Ok((status, r)) if matches!(status, GoalStatus::Succeeded | GoalStatus::Canceled | GoalStatus::Aborted) => Ok((status, r)),
        Ok((status, _)) => Err(format!("action result status {status} is not terminal proof")),
        Err(e) => Err(e),
    }
}

#[tokio::test(start_paused = true)]
async fn a_client_written_like_mower_coverage_drives_a_server_written_like_mower_nav() {
    let bus = Bus::new();
    let coverage = bus.scope("boustrophedon_coverage");
    let client = coverage.action_client::<WaypointAction>(NAME).unwrap();
    assert_eq!(send_follow_path(&client, "early").await.unwrap_err(), "Navigation action server unavailable");
    let nav = bus.scope("nav_action_server");
    let server = nav_like(&nav, Duration::ZERO);

    // A leg that succeeds.
    let (_handle, slot) = send_follow_path(&client, "leg1").await.unwrap();
    server.ends.send(Outcome::Succeeded).unwrap();
    assert_eq!(proven(slot).await, Ok((GoalStatus::Succeeded, result(true))));

    // A reused dispatch id is rejected.
    assert_eq!(send_follow_path(&client, "leg1").await.unwrap_err(), "Navigation action goal rejected");

    // A leg canceled through the action: the ack, then a proven CANCELED.
    let (handle, slot) = send_follow_path(&client, "leg2").await.unwrap();
    let Some(ack) = handle.lock().unwrap().cancel().ok() else { panic!("the action cancel could not be sent") };
    assert_eq!(within(ack).await, Ok(()));
    assert_eq!(proven(slot).await, Ok((GoalStatus::Canceled, result(false))));

    // The nav module restarts mid-leg: no proof, coverage's uncertain path,
    // and the action cancel fails at once (its fallback goes through the
    // dispatch services).
    let (handle, slot) = send_follow_path(&client, "leg3").await.unwrap();
    nav.close();
    let lost = proven(slot).await.unwrap_err();
    assert!(lost.contains("ServiceUnavailable"), "{lost}");
    let Some(ack) = handle.lock().unwrap().cancel().ok() else { panic!("the action cancel could not be sent") };
    assert_eq!(within(ack).await, Err(unavailable()));
    let nav = bus.scope("nav_action_server");
    let server = nav_like(&nav, Duration::ZERO);
    let (_handle, slot) = send_follow_path(&client, "leg4").await.unwrap();
    server.ends.send(Outcome::Aborted).unwrap();
    assert_eq!(proven(slot).await, Ok((GoalStatus::Aborted, result(false))));

    // An acceptance that comes after coverage's 3 s: the late goal is
    // adopted and canceled.
    nav.close();
    let nav = bus.scope("nav_action_server");
    let slow = nav_like(&nav, Duration::from_secs(4));
    assert_eq!(send_follow_path(&client, "leg5").await.unwrap_err(), "acceptance timed out");
    tokio::time::sleep(Duration::from_secs(2)).await;
    assert_eq!(slow.accepted.load(Ordering::SeqCst), 1, "the server accepted it late");
    for _ in 0..100 {
        if bus.actions()[0].active == 0 {
            break;
        }
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    assert_eq!(bus.actions()[0].active, 0, "the late goal was canceled");
}

// ---------------------------------------------------------------------------
// Soak
// ---------------------------------------------------------------------------

/// Goals race server restarts, cancels and closes of the client scopes:
/// every future resolves, with an outcome the README allows.
///
/// Ignored by default because it soaks for 3 s by design. Run it with
/// `cargo test -p mower_bus --test actions -- --ignored`.
#[test]
#[ignore = "soaks for 3 s; run with --ignored"]
fn goals_racing_server_restarts_cancels_and_client_closes_always_resolve_with_an_allowed_outcome() {
    const HANG: Duration = Duration::from_secs(10);
    let rt = tokio::runtime::Builder::new_multi_thread().worker_threads(4).enable_time().build().unwrap();
    let bus = Bus::new();
    let stop = Arc::new(AtomicBool::new(false));
    let ended = Arc::new(AtomicU64::new(0));

    // A server that restarts; its goals end in every way, some from
    // spawned tasks, some dropped, some only when canceled.
    let server = {
        let (bus, stop) = (bus.clone(), stop.clone());
        rt.spawn(async move {
            let mut round = 0u64;
            while !stop.load(Ordering::SeqCst) {
                round += 1;
                let scope = bus.scope("server");
                let mut srv = scope.action_server::<WaypointAction>(NAME).unwrap_or_else(|e| panic!("round {round}: {e}"));
                let serving = tokio::spawn(async move {
                    let mut n = 0u64;
                    while let Some(req) = srv.next().await {
                        n += 1;
                        match n % 7 {
                            0 => drop(req),
                            1 => {
                                let _ = req.reject();
                            }
                            _ => {
                                let Ok((mut g, mut cancels)) = req.accept() else { continue };
                                tokio::spawn(async move {
                                    let _ = g.publish_feedback(feedback("f"));
                                    match n % 7 {
                                        2 => drop(g),
                                        3 => {
                                            if let Some(c) = cancels.next().await {
                                                c.accept();
                                                let _ = g.cancel(result(false));
                                            }
                                        }
                                        4 => {
                                            if let Some(c) = cancels.next().await {
                                                c.reject();
                                            }
                                            let _ = g.abort(result(false));
                                        }
                                        _ => {
                                            tokio::task::yield_now().await;
                                            let _ = g.succeed(result(true));
                                        }
                                    }
                                });
                            }
                        }
                    }
                });
                tokio::time::sleep(Duration::from_micros(300 + round % 700)).await;
                scope.close();
                tokio::time::timeout(HANG, serving).await.expect("server stream ended after close").unwrap();
            }
        })
    };

    let clients: Vec<_> = (0..6)
        .map(|c| {
            let (bus, stop, ended) = (bus.clone(), stop.clone(), ended.clone());
            rt.spawn(async move {
                let mut k = 0u64;
                while !stop.load(Ordering::SeqCst) {
                    let scope = bus.scope(&format!("client{c}"));
                    let client = scope.action_client::<WaypointAction>(NAME).unwrap();
                    for _ in 0..30 {
                        k += 1;
                        if k.is_multiple_of(29) {
                            let s = scope.clone();
                            tokio::spawn(async move { s.close() });
                        }
                        let fut = match client.send_goal_request(goal(&k.to_string())) {
                            Ok(fut) => fut,
                            Err(BusError::ServiceUnavailable { .. }) | Err(BusError::Closed { .. }) => {
                                tokio::task::yield_now().await;
                                continue;
                            }
                            Err(e) => panic!("outer error {e}"),
                        };
                        let (handle, result_future, mut feedbacks) = match tokio::time::timeout(HANG, fut).await.expect("a goal request hung") {
                            Ok(goal) => goal,
                            Err(BusError::ServiceUnavailable { .. })
                            | Err(BusError::NoResponse { .. })
                            | Err(BusError::GoalRejected { .. })
                            | Err(BusError::Closed { .. }) => continue,
                            Err(e) => panic!("goal response error {e}"),
                        };
                        // Cancel every other goal; the goals that wait for a cancel
                        // end when their server's scope closes.
                        if k.is_multiple_of(2) {
                            if let Ok(cancel) = handle.cancel() {
                                match tokio::time::timeout(HANG, cancel).await.expect("a cancel hung") {
                                    Ok(())
                                    | Err(BusError::CancelRejected { .. })
                                    | Err(BusError::GoalTerminated { .. })
                                    | Err(BusError::ServiceUnavailable { .. })
                                    | Err(BusError::NoResponse { .. })
                                    | Err(BusError::Closed { .. }) => {}
                                    Err(e) => panic!("cancel error {e}"),
                                }
                            }
                        }
                        match tokio::time::timeout(HANG, result_future).await.expect("a result hung") {
                            Ok((status, _)) => {
                                assert!(status.is_terminal(), "{status}");
                                ended.fetch_add(1, Ordering::Relaxed);
                            }
                            Err(BusError::ServiceUnavailable { .. })
                            | Err(BusError::NoResponse { .. })
                            | Err(BusError::Closed { .. }) => {}
                            Err(e) => panic!("result error {e}"),
                        }
                        // the stream ends once the goal has
                        tokio::time::timeout(HANG, async { while feedbacks.next().await.is_some() {} })
                            .await
                            .expect("a feedback stream hung");
                        if stop.load(Ordering::SeqCst) {
                            break;
                        }
                    }
                    scope.close();
                }
            })
        })
        .collect();

    thread::sleep(Duration::from_secs(3));
    stop.store(true, Ordering::SeqCst);
    rt.block_on(async {
        tokio::time::timeout(HANG, server).await.expect("server loop hung").unwrap();
        for c in clients {
            tokio::time::timeout(HANG, c).await.expect("client hung").unwrap();
        }
    });
    assert!(ended.load(Ordering::Relaxed) > 100, "{}", ended.load(Ordering::Relaxed));
    assert_eq!(bus.actions()[0].active, 0, "no goal is left running");
}
