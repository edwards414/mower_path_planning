//! The action forwarders against real DDS, in one process (README.md,
//! "Actions"): the bridge's node and a peer node that stands in for the CI
//! tools' rclpy clients (`nav_progress_check.py`, `nav_core_check.py`) and
//! for the rclpy server of `coverage_cancel_check.py`, all on the process's
//! one participant, as in mower_rsd.
//!
//! - **Isolation:** `ROS_DOMAIN_ID` 89, set before the process's r2r context
//!   exists, so that these run next to `dds.rs` (77) and other checkouts'
//!   tests on the same host; names of their own for every test.
//! - **RMW:** rmw_cyclonedds_cpp, the robot's.
//! - **Waiting:** discovery is polled for, never slept for. An absence (a
//!   result that never comes) is asserted only after the bridge's stats say
//!   it let the goal go.

use std::future::Future;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

use futures::{FutureExt, StreamExt};
use mower_bus::{ActionServer, Bus, BusError, GoalStatus, Scope};
use mower_bus_ros::{BridgeError, BridgeStats, Direction, EntryStats, RosBridge, ServiceTimeouts};
use r2r::mower_interface::action::Waypoint;
use r2r::std_srvs::srv::SetBool;
use r2r::QosProfile;

/// For anything that waits on DDS. Generous: CI runners are small and
/// shared, and a pass never waits this long.
const WAIT: Duration = Duration::from_secs(20);

/// The bus side of `Waypoint::Action`.
type WaypointAction = (Waypoint::Goal, Waypoint::Result, Waypoint::Feedback);

/// The process's r2r context, on domain 89 and rmw_cyclonedds_cpp.
fn context() -> r2r::Context {
    static CONTEXT: OnceLock<r2r::Context> = OnceLock::new();
    CONTEXT
        .get_or_init(|| {
            assert!(
                rmw_installed("rmw_cyclonedds_cpp"),
                "these tests run on the robot's RMW: apt-get install ros-jazzy-rmw-cyclonedds-cpp"
            );
            // rcl reads both when the context is created, and only then.
            std::env::set_var("ROS_DOMAIN_ID", "89");
            std::env::set_var("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp");
            r2r::Context::create().expect("an rcl context (source /opt/ros/jazzy/setup.bash)")
        })
        .clone()
}

fn rmw_installed(rmw: &str) -> bool {
    let prefixes = std::env::var("AMENT_PREFIX_PATH").unwrap_or_default();
    prefixes.split(':').any(|prefix| std::path::Path::new(prefix).join(format!("lib/lib{rmw}.so")).exists())
}

/// Run one test on a runtime of its own, within a bound that only a hang
/// reaches.
fn within(test: impl Future<Output = ()>) {
    let runtime = tokio::runtime::Builder::new_multi_thread().worker_threads(2).enable_all().build().unwrap();
    runtime.block_on(async {
        tokio::time::timeout(Duration::from_secs(120), test).await.expect("the test hung");
    });
}

/// A node of the process that is not the bridge: spun on a thread of its
/// own, its endpoints made between two spins. Dropping it takes the node
/// off the graph, as a ROS process that exits.
struct Peer {
    node: Arc<Mutex<r2r::Node>>,
    running: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl Peer {
    fn new(name: &str) -> Peer {
        let node = Arc::new(Mutex::new(r2r::Node::create(context(), name, "").unwrap()));
        let running = Arc::new(AtomicBool::new(true));
        let thread = {
            let (node, running) = (node.clone(), running.clone());
            std::thread::spawn(move || {
                while running.load(Ordering::Acquire) {
                    node.lock().unwrap().spin_once(Duration::from_millis(5));
                    // Let `with` in between: the lock is not fair.
                    std::thread::sleep(Duration::from_millis(1));
                }
            })
        };
        Peer { node, running, thread: Some(thread) }
    }

    fn with<R>(&self, f: impl FnOnce(&mut r2r::Node) -> R) -> R {
        f(&mut self.node.lock().unwrap())
    }

    /// A DDS action client on this node, once its server is matched.
    async fn action_client(&self, name: &str) -> r2r::ActionClient<Waypoint::Action> {
        let client = self.with(|n| n.create_action_client::<Waypoint::Action>(name)).unwrap();
        let available = r2r::Node::is_available(&client).unwrap();
        tokio::time::timeout(WAIT, available).await.expect("timed out waiting for the action server").unwrap();
        client
    }
}

impl Drop for Peer {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Release);
        if let Some(thread) = self.thread.take() {
            thread.join().unwrap();
        }
    }
}

fn new_bridge(node: &str, scope: &Scope) -> RosBridge {
    RosBridge::new(r2r::Node::create(context(), node, "").unwrap(), scope.clone())
}

/// A bridge running on the test's runtime, as a module would run it.
struct Running {
    stats: BridgeStats,
    stop: Option<tokio::sync::oneshot::Sender<()>>,
    task: tokio::task::JoinHandle<Result<(), BridgeError>>,
}

fn start(bridge: RosBridge) -> Running {
    let stats = bridge.stats();
    let (stop, stopped) = tokio::sync::oneshot::channel::<()>();
    let task = tokio::spawn(bridge.run(stopped.map(|_| ())));
    Running { stats, stop: Some(stop), task }
}

impl Running {
    fn action(&self, name: &str) -> EntryStats {
        self.stats.action(name).expect("a bridged action")
    }

    /// Trip the shutdown and wait for `run` to return.
    async fn stop(mut self) -> Result<(), BridgeError> {
        let _ = self.stop.take().expect("not stopped yet").send(());
        tokio::time::timeout(WAIT, self.task).await.expect("run returned").expect("run did not panic")
    }
}

async fn eventually(what: &str, mut done: impl FnMut() -> bool) {
    let deadline = Instant::now() + WAIT;
    while !done() {
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

async fn wait<F: Future>(what: &str, fut: F) -> F::Output {
    match tokio::time::timeout(WAIT, fut).await {
        Ok(output) => output,
        Err(_) => panic!("timed out waiting for {what}"),
    }
}

fn goal(dispatch_id: &str) -> Waypoint::Goal {
    Waypoint::Goal { dispatch_id: dispatch_id.into(), zone_id: 7, ..Default::default() }
}

fn result(success: bool) -> Waypoint::Result {
    Waypoint::Result { success }
}

fn feedback(text: &str) -> Waypoint::Feedback {
    Waypoint::Feedback { feedback: text.into() }
}

// ---------------------------------------------------------------------------
// action_to_ros: DDS clients (the CI tools) drive a bus server (mower_nav)
// ---------------------------------------------------------------------------

/// How a bus server ends each goal it accepts, by its dispatch id.
fn serve_on_bus(scope: &Scope, name: &str) -> tokio::task::JoinHandle<()> {
    let mut requests: ActionServer<WaypointAction> = scope.action_server(name).unwrap();
    tokio::spawn(async move {
        while let Some(request) = requests.next().await {
            let id = request.goal.dispatch_id.clone();
            if id.starts_with("reject") {
                let _ = request.reject();
                continue;
            }
            assert_eq!(request.goal.zone_id, 7, "the goal crosses whole");
            let (mut goal, mut cancels) = request.accept().unwrap();
            tokio::spawn(async move {
                for text in ["one", "two", "three"] {
                    goal.publish_feedback(feedback(&format!("{id} {text}"))).unwrap();
                }
                match id.as_str() {
                    // waits for a cancel request, and takes it
                    i if i.starts_with("cancelable") => {
                        let cancel = cancels.next().await.unwrap();
                        cancel.accept();
                        goal.cancel(result(false)).unwrap();
                    }
                    // refuses cancel requests, then ends on its own
                    i if i.starts_with("stubborn") => {
                        cancels.next().await.unwrap().reject();
                        goal.succeed(result(true)).unwrap();
                    }
                    // takes the cancel request and succeeds anyway
                    i if i.starts_with("too_late") => {
                        cancels.next().await.unwrap().accept();
                        goal.succeed(result(true)).unwrap();
                    }
                    // runs until its scope closes
                    i if i.starts_with("forever") => while cancels.next().await.is_some() {},
                    _ => goal.succeed(result(true)).unwrap(),
                }
            });
        }
    })
}

#[test]
fn action_to_ros_carries_a_dds_goal_to_the_bus_server_and_its_feedback_and_result_back() {
    within(async {
        let name = "/bridge_action_test/follow_path";
        let bus = Bus::new();
        let mut bridge = new_bridge("follow_path_bridge", &bus.scope("bridge"));
        bridge.action_to_ros::<Waypoint::Action>(name).unwrap();
        let running = start(bridge);
        let _nav = serve_on_bus(&bus.scope("nav_action_server"), name);
        let tool = Peer::new("follow_path_tool");
        let client = tool.action_client(name).await;

        let (dds_goal, result_future, feedbacks) = wait("the goal", client.send_goal_request(goal("a")).unwrap()).await.unwrap();
        let texts: Vec<String> = wait("the feedback", feedbacks.take(3).map(|f| f.feedback).collect()).await;
        assert_eq!(texts, ["a one", "a two", "a three"], "in order, and all of it");
        assert_eq!(wait("the result", result_future).await.unwrap(), (r2r::GoalStatus::Succeeded, result(true)));
        eventually("the status on DDS", || matches!(dds_goal.get_status(), Ok(r2r::GoalStatus::Succeeded))).await;
        assert_eq!((running.action(name).forwarded, running.action(name).unanswered), (1, 0));
        assert_eq!(running.action(name).direction, Direction::ActionToRos);

        // Several at once, each with its own feedback and result.
        let goals = ["b", "c", "d"].map(|id| client.send_goal_request(goal(id)).unwrap());
        let accepted = wait("three goals", futures::future::join_all(goals)).await;
        for ((_goal, result_future, mut feedbacks), id) in accepted.into_iter().map(Result::unwrap).zip(["b", "c", "d"]) {
            assert_eq!(wait("feedback", feedbacks.next()).await.unwrap().feedback, format!("{id} one"));
            assert_eq!(wait("a result", result_future).await.unwrap().0, r2r::GoalStatus::Succeeded);
        }
        running.stop().await.unwrap();
    });
}

#[test]
fn action_to_ros_rejects_on_dds_what_the_bus_server_rejects() {
    within(async {
        let name = "/bridge_action_test/rejecting";
        let bus = Bus::new();
        let mut bridge = new_bridge("rejecting_bridge", &bus.scope("bridge"));
        bridge.action_to_ros::<Waypoint::Action>(name).unwrap();
        let running = start(bridge);
        let _nav = serve_on_bus(&bus.scope("nav_action_server"), name);
        let tool = Peer::new("rejecting_tool");
        let client = tool.action_client(name).await;
        let refused = wait("the answer", client.send_goal_request(goal("reject me")).unwrap()).await;
        assert!(matches!(refused, Err(r2r::Error::RCL_RET_ACTION_GOAL_REJECTED)), "rejected, as the CI tools expect");
        assert_eq!((running.action(name).forwarded, running.action(name).unanswered), (1, 0));
        running.stop().await.unwrap();
    });
}

#[test]
fn a_dds_cancel_reaches_the_bus_server_and_its_answer_comes_back() {
    within(async {
        let name = "/bridge_action_test/canceled";
        let bus = Bus::new();
        let mut bridge = new_bridge("canceled_bridge", &bus.scope("bridge"));
        bridge.action_to_ros::<Waypoint::Action>(name).unwrap();
        let running = start(bridge);
        let _nav = serve_on_bus(&bus.scope("nav_action_server"), name);
        let tool = Peer::new("canceled_tool");
        let client = tool.action_client(name).await;

        // Accepted by the bus server, which ends the goal CANCELED.
        let (dds_goal, result_future, _f) = wait("the goal", client.send_goal_request(goal("cancelable")).unwrap()).await.unwrap();
        wait("the cancel", dds_goal.cancel().unwrap()).await.expect("the cancel is accepted");
        assert_eq!(wait("the result", result_future).await.unwrap(), (r2r::GoalStatus::Canceled, result(false)));

        // Refused by the bus server: the goal runs on and succeeds.
        let (dds_goal, result_future, _f) = wait("the goal", client.send_goal_request(goal("stubborn")).unwrap()).await.unwrap();
        assert!(matches!(wait("the cancel", dds_goal.cancel().unwrap()).await, Err(r2r::Error::GoalCancelRejected)));
        assert_eq!(wait("the result", result_future).await.unwrap(), (r2r::GoalStatus::Succeeded, result(true)));

        // Accepted, but the bus server succeeds anyway: r2r takes only a
        // CANCELED result for a goal whose cancel it accepted.
        let (dds_goal, result_future, _f) = wait("the goal", client.send_goal_request(goal("too_late")).unwrap()).await.unwrap();
        wait("the cancel", dds_goal.cancel().unwrap()).await.expect("the cancel is accepted");
        assert_eq!(wait("the result", result_future).await.unwrap(), (r2r::GoalStatus::Canceled, result(true)));
        assert_eq!(running.action(name).unanswered, 0);
        running.stop().await.unwrap();
    });
}

/// A DDS goal sent while the bus server's module restarts waits for the
/// restarted server, as a service call does, and one sent while none comes
/// in time is left unanswered.
#[test]
fn a_dds_goal_sent_while_the_bus_server_restarts_is_served_once_it_is_back() {
    within(async {
        let name = "/bridge_action_test/restarting";
        let bus = Bus::new();
        let timeouts = ServiceTimeouts { bus_server: Duration::from_secs(3), ..ServiceTimeouts::default() };
        let mut bridge = new_bridge("restarting_bridge", &bus.scope("bridge")).with_service_timeouts(timeouts);
        bridge.action_to_ros::<Waypoint::Action>(name).unwrap();
        let running = start(bridge);
        let tool = Peer::new("restarting_tool");
        // The DDS server is advertised from the declaration on, bus server or not.
        let client = tool.action_client(name).await;

        let sent = client.send_goal_request(goal("a")).unwrap();
        tokio::time::sleep(Duration::from_millis(300)).await;
        let _nav = serve_on_bus(&bus.scope("nav_action_server"), name);
        let (_goal, result_future, _f) = wait("the goal", sent).await.unwrap();
        assert_eq!(wait("the result", result_future).await.unwrap().0, r2r::GoalStatus::Succeeded);
        assert_eq!(running.action(name).unanswered, 0);
        running.stop().await.unwrap();

        // No server, and none within the wait: no answer on DDS.
        let name = "/bridge_action_test/never_served";
        let timeouts = ServiceTimeouts { bus_server: Duration::from_millis(300), ..ServiceTimeouts::default() };
        let mut bridge = new_bridge("never_served_bridge", &bus.scope("bridge")).with_service_timeouts(timeouts);
        bridge.action_to_ros::<Waypoint::Action>(name).unwrap();
        let running = start(bridge);
        let client = tool.action_client(name).await;
        let mut sent = Box::pin(client.send_goal_request(goal("b")).unwrap());
        eventually("the goal request to be given up", || running.action(name).unanswered == 1).await;
        tokio::time::sleep(Duration::from_millis(200)).await;
        assert!((&mut sent).now_or_never().is_none(), "the DDS request ends unanswered");
        running.stop().await.unwrap();
    });
}

/// The bus server's module restarts mid-goal: the bus goal is lost, which
/// ROS cannot say, so the DDS goal gets no result, as when a direct r2r
/// server's node goes. The restarted server serves the next goal.
#[test]
fn a_bus_goal_lost_mid_goal_leaves_the_dds_goal_without_a_result() {
    within(async {
        let name = "/bridge_action_test/lost";
        let bus = Bus::new();
        let mut bridge = new_bridge("lost_bridge", &bus.scope("bridge"));
        bridge.action_to_ros::<Waypoint::Action>(name).unwrap();
        let running = start(bridge);
        let nav = bus.scope("nav_action_server");
        let _first = serve_on_bus(&nav, name);
        let tool = Peer::new("lost_tool");
        let client = tool.action_client(name).await;
        let (_goal, result_future, _f) = wait("the goal", client.send_goal_request(goal("forever")).unwrap()).await.unwrap();
        let mut result_future = Box::pin(result_future);

        nav.close();
        eventually("the lost goal to be counted", || running.action(name).unanswered == 1).await;
        tokio::time::sleep(Duration::from_millis(200)).await;
        assert!((&mut result_future).now_or_never().is_none(), "no result on DDS");

        let _second = serve_on_bus(&bus.scope("nav_action_server"), name);
        let (_goal, result_future, _f) = wait("the next goal", client.send_goal_request(goal("a")).unwrap()).await.unwrap();
        assert_eq!(wait("its result", result_future).await.unwrap().0, r2r::GoalStatus::Succeeded);
        running.stop().await.unwrap();
    });
}

// ---------------------------------------------------------------------------
// action_from_ros: bus clients (mower_coverage) drive a DDS server
// ---------------------------------------------------------------------------

/// What a DDS server on `peer` does with each goal, by its dispatch id, and
/// the cancel requests it saw.
struct DdsServer {
    _task: tokio::task::JoinHandle<()>,
    cancels: Arc<AtomicUsize>,
}

fn serve_on_dds(peer: &Peer, name: &str, accept_after: Duration) -> DdsServer {
    let mut requests = peer.with(|n| n.create_action_server::<Waypoint::Action>(name)).unwrap();
    let cancels = Arc::new(AtomicUsize::new(0));
    let seen = cancels.clone();
    let task = tokio::spawn(async move {
        while let Some(request) = requests.next().await {
            let id = request.goal.dispatch_id.clone();
            if id.starts_with("reject") {
                let _ = request.reject();
                continue;
            }
            let seen = seen.clone();
            tokio::spawn(async move {
                tokio::time::sleep(accept_after).await;
                let (mut goal, mut cancels) = request.accept().unwrap();
                for text in ["one", "two"] {
                    goal.publish_feedback(feedback(&format!("{id} {text}"))).unwrap();
                }
                match id.as_str() {
                    i if i.starts_with("cancelable") => {
                        let cancel = cancels.next().await.unwrap();
                        seen.fetch_add(1, Ordering::SeqCst);
                        cancel.accept();
                        goal.cancel(result(false)).unwrap();
                    }
                    i if i.starts_with("stubborn") => {
                        cancels.next().await.unwrap().reject();
                        seen.fetch_add(1, Ordering::SeqCst);
                        goal.abort(result(false)).unwrap();
                    }
                    i if i.starts_with("forever") => {
                        while let Some(cancel) = cancels.next().await {
                            seen.fetch_add(1, Ordering::SeqCst);
                            cancel.reject();
                        }
                    }
                    _ => {
                        // Let the feedback land before the result: DDS
                        // orders neither against the other.
                        tokio::time::sleep(Duration::from_millis(200)).await;
                        goal.succeed(result(true)).unwrap();
                    }
                }
            });
        }
    });
    DdsServer { _task: task, cancels }
}

/// Send a bus goal until the bridge's DDS client has matched the server,
/// once the bridge serves the name on the bus (it does while the graph shows
/// the DDS server): before the match, requests come back `NoResponse` at
/// once, never anything else.
async fn send_until_matched(
    client: &mower_bus::ActionClient<WaypointAction>, dispatch_id: &str,
) -> Result<(mower_bus::ActionClientGoal<WaypointAction>, mower_bus::ResultFuture<WaypointAction>, mower_bus::FeedbackStream<WaypointAction>), BusError> {
    wait("the bridge's bus server", client.wait_for_server()).await.unwrap();
    let deadline = Instant::now() + WAIT;
    loop {
        match wait("the goal response", client.send_goal_request(goal(dispatch_id)).unwrap()).await {
            Err(BusError::NoResponse { .. }) => assert!(Instant::now() < deadline, "the DDS server never matched"),
            other => return other,
        }
    }
}

#[test]
fn action_from_ros_carries_a_bus_goal_to_the_dds_server_and_its_feedback_and_result_back() {
    within(async {
        let name = "/bridge_action_test/navigate";
        let nav2 = Peer::new("navigate_nav2");
        let server = serve_on_dds(&nav2, name, Duration::ZERO);
        let bus = Bus::new();
        let mut bridge = new_bridge("navigate_bridge", &bus.scope("bridge"));
        bridge.action_from_ros::<Waypoint::Action>(name).unwrap();
        let running = start(bridge);
        let client = bus.scope("boustrophedon_coverage").action_client::<WaypointAction>(name).unwrap();
        eventually("the bridge's bus server, which comes with the DDS one", || client.is_available()).await;
        assert!(running.action(name).bus_server);

        let (handle, result_future, feedbacks) = send_until_matched(&client, "a").await.unwrap();
        assert_eq!(handle.get_status(), Ok(GoalStatus::Executing));
        let texts: Vec<String> = wait("the feedback", feedbacks.map(|f| f.feedback).collect()).await;
        assert_eq!(texts, ["a one", "a two"], "in order, all of it, then the stream ends with the goal");
        assert_eq!(wait("the result", result_future).await, Ok((GoalStatus::Succeeded, result(true))));

        // Rejected on DDS: rejected on the bus.
        let refused = wait("the answer", client.send_goal_request(goal("reject me")).unwrap()).await;
        assert!(matches!(refused, Err(BusError::GoalRejected { .. })), "{refused:?}");

        // Cancel requests: accepted, then the server ends the goal CANCELED;
        // rejected, and the goal runs on.
        let (handle, result_future, _f) = wait("the goal", client.send_goal_request(goal("cancelable")).unwrap()).await.unwrap();
        assert_eq!(wait("the cancel", handle.cancel().unwrap()).await, Ok(()));
        assert_eq!(wait("the result", result_future).await, Ok((GoalStatus::Canceled, result(false))));
        let (handle, result_future, _f) = wait("the goal", client.send_goal_request(goal("stubborn")).unwrap()).await.unwrap();
        let rejected = wait("the cancel", handle.cancel().unwrap()).await;
        assert!(matches!(rejected, Err(BusError::CancelRejected { .. })), "{rejected:?}");
        assert_eq!(wait("the result", result_future).await, Ok((GoalStatus::Aborted, result(false))));
        assert_eq!(server.cancels.load(Ordering::SeqCst), 2);
        let stats = running.action(name);
        assert_eq!((stats.direction, stats.unanswered), (Direction::ActionFromRos, 0));
        running.stop().await.unwrap();
    });
}

/// The bridge serves the name on the bus only while the graph shows a DDS
/// server, so a bus client finds what an r2r client found: no server while
/// there is none. mower_coverage then says "Navigation action server
/// unavailable" after its 2 s wait, as it did against a missing rclpy server
/// (coverage_cancel_check.py), instead of sending a goal that ends in
/// `NoResponse` and its correlated-cancel tracking.
#[test]
fn action_from_ros_serves_on_the_bus_only_while_the_graph_shows_a_dds_server() {
    within(async {
        let name = "/bridge_action_test/presence";
        let bus = Bus::new();
        let mut bridge = new_bridge("presence_bridge", &bus.scope("bridge"));
        bridge.action_from_ros::<Waypoint::Action>(name).unwrap();
        let running = start(bridge);
        let client = bus.scope("boustrophedon_coverage").action_client::<WaypointAction>(name).unwrap();
        eventually("the bridge's first look at the graph", || running.action(name).resyncs > 0).await;
        assert!(!client.is_available() && !running.action(name).bus_server);
        let refused = client.send_goal_request(goal("a"));
        assert!(matches!(refused, Err(BusError::ServiceUnavailable { .. })), "refused at once: {refused:?}");
        assert!(tokio::time::timeout(Duration::from_secs(1), client.wait_for_server()).await.is_err(), "no server to wait for");
        assert_eq!(running.action(name).unanswered, 0, "nothing was sent anywhere");

        // A DDS server comes: so does the bus server, and a goal goes through.
        let nav2 = Peer::new("presence_nav2");
        let _server = serve_on_dds(&nav2, name, Duration::ZERO);
        wait("the bus server", client.wait_for_server()).await.unwrap();
        assert!(running.action(name).bus_server);
        let (_handle, result_future, _f) = send_until_matched(&client, "a").await.unwrap();
        assert_eq!(wait("the result", result_future).await, Ok((GoalStatus::Succeeded, result(true))));

        // It goes with its process: the bus server goes too.
        drop(nav2);
        eventually("the bus server to go", || !client.is_available()).await;
        assert!(!running.action(name).bus_server);
        let refused = client.send_goal_request(goal("b"));
        assert!(matches!(refused, Err(BusError::ServiceUnavailable { .. })), "{refused:?}");
        running.stop().await.unwrap();
    });
}

#[test]
fn action_from_ros_loses_a_goal_whose_dds_server_goes() {
    within(async {
        let name = "/bridge_action_test/nobody";
        let bus = Bus::new();
        let mut bridge = new_bridge("nobody_bridge", &bus.scope("bridge"));
        bridge.action_from_ros::<Waypoint::Action>(name).unwrap();
        let running = start(bridge);
        let client = bus.scope("boustrophedon_coverage").action_client::<WaypointAction>(name).unwrap();

        // A server that takes the goal, then goes away with its process.
        let nav2 = Peer::new("nobody_nav2");
        let _server = serve_on_dds(&nav2, name, Duration::ZERO);
        let (handle, result_future, _f) = send_until_matched(&client, "forever").await.unwrap();
        drop(nav2);
        let lost = wait("the lost goal", result_future).await;
        assert!(matches!(lost, Err(BusError::NoResponse { .. })), "never a terminal status: {lost:?}");
        assert_eq!(handle.get_status(), Ok(GoalStatus::Unknown));
        assert!(running.action(name).unanswered >= 1);
        running.stop().await.unwrap();
    });
}

/// mower_coverage keeps a goal request that its server accepts late
/// (coverage_cancel_check.py's scenario B accepts after 3.5 s). Within
/// `dds_call` the bridge waits for the answer; past it the bus client gets
/// `NoResponse`, and a goal the DDS server accepts later is canceled.
#[test]
fn a_late_dds_acceptance_reaches_the_bus_client_within_the_deadline_and_is_canceled_after_it() {
    within(async {
        let name = "/bridge_action_test/late";
        let nav2 = Peer::new("late_nav2");
        let server = serve_on_dds(&nav2, name, Duration::from_millis(1500));
        let bus = Bus::new();
        let timeouts = ServiceTimeouts { dds_call: Duration::from_secs(4), ..ServiceTimeouts::default() };
        let mut bridge = new_bridge("late_bridge", &bus.scope("bridge")).with_service_timeouts(timeouts);
        bridge.action_from_ros::<Waypoint::Action>(name).unwrap();
        let running = start(bridge);
        let client = bus.scope("boustrophedon_coverage").action_client::<WaypointAction>(name).unwrap();
        let (_handle, result_future, _f) = send_until_matched(&client, "a").await.unwrap();
        assert_eq!(wait("the result", result_future).await, Ok((GoalStatus::Succeeded, result(true))));
        running.stop().await.unwrap();

        // Given up at 1 s, accepted at 1.5 s: within the second second the
        // bridge waits for a late answer, to cancel it.
        let timeouts = ServiceTimeouts { dds_call: Duration::from_secs(1), ..ServiceTimeouts::default() };
        let mut bridge = new_bridge("too_late_bridge", &bus.scope("bridge")).with_service_timeouts(timeouts);
        bridge.action_from_ros::<Waypoint::Action>(name).unwrap();
        let running = start(bridge);
        wait("the bridge's bus server", client.wait_for_server()).await.unwrap();
        let started = Instant::now();
        // Matched or not, a request comes back NoResponse now.
        let mut answered = false;
        while server.cancels.load(Ordering::SeqCst) == 0 {
            let answer = wait("the answer", client.send_goal_request(goal("forever")).unwrap()).await;
            assert!(matches!(answer, Err(BusError::NoResponse { .. })), "{answer:?}");
            answered = true;
            eventually_or(Duration::from_secs(4), || server.cancels.load(Ordering::SeqCst) > 0).await;
            assert!(started.elapsed() < WAIT, "the late goal was never canceled");
        }
        assert!(answered);
        running.stop().await.unwrap();
    });
}

/// Like `eventually`, but gives up quietly after `limit`.
async fn eventually_or(limit: Duration, mut done: impl FnMut() -> bool) {
    let deadline = Instant::now() + limit;
    while !done() && Instant::now() < deadline {
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

// ---------------------------------------------------------------------------
// One direction per name, and the lifecycle
// ---------------------------------------------------------------------------

#[test]
fn an_action_crosses_once_in_one_direction_and_is_a_namespace_of_its_own() {
    within(async {
        let name = "/bridge_action_test/one_way";
        let bus = Bus::new();
        let mut bridge = new_bridge("action_one_way_bridge", &bus.scope("bridge"));
        bridge.action_to_ros::<Waypoint::Action>(name).unwrap();
        let refused = |result: Result<(), BridgeError>, requested: Direction| match result {
            Err(BridgeError::AlreadyBridged { existing: Direction::ActionToRos, requested: r, .. }) => assert_eq!(r, requested),
            other => panic!("{requested} {name} must be refused, got {other:?}"),
        };
        refused(bridge.action_from_ros::<Waypoint::Action>(name), Direction::ActionFromRos);
        refused(bridge.action_to_ros::<Waypoint::Action>(name), Direction::ActionToRos);
        let mut other = new_bridge("action_one_way_other", &bus.scope("other"));
        refused(other.action_from_ros::<Waypoint::Action>(name), Direction::ActionFromRos);
        // A topic or a service of the same name is another namespace, as on
        // DDS (`<name>/_action/...`) and on the bus.
        bridge.service_to_ros::<SetBool::Service>(name).unwrap();
        bridge.to_ros::<r2r::std_msgs::msg::String>(name, QosProfile::default()).unwrap();
        // Refused by the bus (a relative name): the name stays free.
        assert!(matches!(other.action_to_ros::<Waypoint::Action>("one_way_relative"), Err(BridgeError::Bus(BusError::InvalidName { .. }))));
        assert!(other.stats().action("one_way_relative").is_none());

        // A bridge that has run gives its names back.
        let running = start(bridge);
        running.stop().await.unwrap();
        other.action_from_ros::<Waypoint::Action>(name).unwrap();
    });
}

#[test]
fn run_ends_the_forwarders_and_takes_the_dds_action_server_with_the_node() {
    within(async {
        let name = "/bridge_action_test/lifecycle";
        let bus = Bus::new();
        let bridge_scope = bus.scope("bridge");
        let mut bridge = new_bridge("action_lifecycle_bridge", &bridge_scope);
        bridge.action_to_ros::<Waypoint::Action>(name).unwrap();
        bridge.action_from_ros::<Waypoint::Action>("/bridge_action_test/lifecycle_from").unwrap();
        let running = start(bridge);
        let _nav = serve_on_bus(&bus.scope("nav_action_server"), name);
        let tool = Peer::new("action_lifecycle_tool");
        let client = tool.action_client(name).await;
        let (_goal, result_future, _f) = wait("a goal", client.send_goal_request(goal("a")).unwrap()).await.unwrap();
        wait("its result", result_future).await.unwrap();
        let _from = serve_on_dds(&tool, "/bridge_action_test/lifecycle_from", Duration::ZERO);
        let coverage = bus.scope("boustrophedon_coverage").action_client::<WaypointAction>("/bridge_action_test/lifecycle_from").unwrap();
        eventually("the bridge's bus server", || coverage.is_available()).await;

        bridge_scope.close();
        tokio::time::timeout(WAIT, running.task).await.expect("run returned").expect("run did not panic").unwrap();
        assert!(!coverage.is_available(), "the bridge's bus server went with its scope");
        // The DDS action server went with the node: a new one can take the
        // name, and a client finds it.
        let mut again = new_bridge("action_lifecycle_again", &bus.scope("again"));
        again.action_to_ros::<Waypoint::Action>(name).unwrap();
        let running = start(again);
        let matched = r2r::Node::is_available(&client).unwrap();
        wait("the new DDS server to match", matched).await.unwrap();
        let (_goal, result_future, _f) = wait("a goal", client.send_goal_request(goal("b")).unwrap()).await.unwrap();
        assert_eq!(wait("its result", result_future).await.unwrap().0, r2r::GoalStatus::Succeeded);
        running.stop().await.unwrap();
    });
}
