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

/// A bus server that takes the first cancel request of each goal, ends the
/// goal SUCCEEDED, and answers the cancel request (a rejection, all that is
/// left after the end) `answer_after` later, or never: a server slow with a
/// cancel.
fn slow_cancel_server(scope: &Scope, name: &str, answer_after: Option<Duration>) -> tokio::task::JoinHandle<()> {
    let mut requests: ActionServer<WaypointAction> = scope.action_server(name).unwrap();
    tokio::spawn(async move {
        while let Some(request) = requests.next().await {
            let (mut goal, mut cancels) = request.accept().unwrap();
            tokio::spawn(async move {
                let cancel = cancels.next().await.unwrap();
                goal.succeed(result(true)).unwrap();
                match answer_after {
                    Some(after) => {
                        tokio::time::sleep(after).await;
                        cancel.reject();
                    }
                    // Held until the test's runtime goes.
                    None => std::future::pending::<()>().await,
                }
            });
        }
    })
}

/// r2r sends no answer at all to a DDS cancel request that the bridge drops
/// unanswered, and the ROS client would wait for one forever. So every
/// request the bridge takes gets one, however late the bus server's own
/// comes: after the end it can only be a refusal, and the bridge waits 1 s
/// for it, then rejects. A request that comes after the result r2r answers
/// itself.
#[test]
fn a_dds_cancel_the_bus_server_answers_late_or_never_is_still_rejected() {
    within(async {
        let bus = Bus::new();
        let mut bridge = new_bridge("slow_cancel_bridge", &bus.scope("bridge"));
        let servers = [
            ("/bridge_action_test/cancel_answered_in_the_drain", Some(Duration::from_millis(300))),
            ("/bridge_action_test/cancel_answered_after_the_drain", Some(Duration::from_secs(3))),
            ("/bridge_action_test/cancel_never_answered", None),
        ];
        for (name, _) in servers {
            bridge.action_to_ros::<Waypoint::Action>(name).unwrap();
        }
        let running = start(bridge);
        let nav = bus.scope("nav_action_server");
        let _nav: Vec<_> = servers.iter().map(|(name, after)| slow_cancel_server(&nav, name, *after)).collect();
        let tool = Peer::new("slow_cancel_tool");

        for (name, _) in servers {
            let client = tool.action_client(name).await;
            let (dds_goal, result_future, _f) = wait("the goal", client.send_goal_request(goal("a")).unwrap()).await.unwrap();
            let cancel = dds_goal.cancel().unwrap();
            assert_eq!(wait("the result", result_future).await.unwrap(), (r2r::GoalStatus::Succeeded, result(true)));
            let answer = wait("the cancel's answer", cancel).await;
            assert!(matches!(answer, Err(r2r::Error::GoalCancelRejected)), "{name}: {answer:?}");
            // The goal has ended on DDS: r2r answers by itself.
            let again = wait("the next cancel's answer", dds_goal.cancel().unwrap()).await;
            assert!(matches!(again, Err(r2r::Error::GoalCancelAlreadyTerminated)), "{name}: {again:?}");
            assert_eq!((running.action(name).forwarded, running.action(name).unanswered), (1, 0));
        }
        running.stop().await.unwrap();
    });
}

/// A cancel request that the bus server still holds when the bridge stops is
/// rejected too: the forwarder rejects it as `run` aborts it, and `run`
/// spins the node once more so that r2r sends the answer before the node
/// goes.
#[test]
fn a_dds_cancel_in_flight_when_the_bridge_stops_is_rejected() {
    within(async {
        let name = "/bridge_action_test/cancel_at_stop";
        let bus = Bus::new();
        let mut bridge = new_bridge("cancel_at_stop_bridge", &bus.scope("bridge"));
        bridge.action_to_ros::<Waypoint::Action>(name).unwrap();
        let running = start(bridge);
        // Takes each goal and its first cancel request, and holds both: the
        // goal runs, the request waits for an answer.
        let (hold, mut held) = tokio::sync::mpsc::unbounded_channel();
        let mut requests: ActionServer<WaypointAction> = bus.scope("nav_action_server").action_server(name).unwrap();
        let _nav = tokio::spawn(async move {
            while let Some(request) = requests.next().await {
                let (goal, mut cancels) = request.accept().unwrap();
                let hold = hold.clone();
                tokio::spawn(async move {
                    let cancel = cancels.next().await.unwrap();
                    let _ = hold.send((goal, cancel));
                });
            }
        });
        let tool = Peer::new("cancel_at_stop_tool");
        let client = tool.action_client(name).await;
        let (dds_goal, _result, _f) = wait("the goal", client.send_goal_request(goal("a")).unwrap()).await.unwrap();
        let cancel = dds_goal.cancel().unwrap();
        let _in_hand = wait("the bus server to hold the cancel request", held.recv()).await.unwrap();

        running.stop().await.unwrap();
        let answer = wait("the cancel's answer", cancel).await;
        assert!(matches!(answer, Err(r2r::Error::GoalCancelRejected)), "{answer:?}");
    });
}

/// Whether the graph shows `name`'s DDS action server: a writer of any of
/// its three services and two topics, as `ros2 action list` would see it.
fn shown(peer: &Peer, name: &str) -> bool {
    let writers = ["rr", "rr", "rr", "rt", "rt"]
        .iter()
        .zip(["send_goalReply", "cancel_goalReply", "get_resultReply", "feedback", "status"])
        .map(|(prefix, entity)| format!("{prefix}{name}/_action/{entity}"));
    peer.with(|node| writers.into_iter().any(|writer| !node.get_publishers_info_by_topic(&writer, true).unwrap().is_empty()))
}

/// A DDS client waits for a server, as rclpy's `wait_for_server(1.0)`: true
/// if r2r finds one within a second.
async fn found_within_a_second(client: &r2r::ActionClient<Waypoint::Action>) -> bool {
    tokio::time::timeout(Duration::from_secs(1), r2r::Node::is_available(client).unwrap()).await.is_ok()
}

/// The DDS action server exists only while the bus has a server, as the
/// module's own r2r server lived and died with its node. While the module is
/// down a DDS client finds none (`wait_for_server` false, nothing on the
/// graph); it finds one as soon as the module is up, none once its scope
/// closes (mower_rsd ends a failed module so, and starts it again 2 s later),
/// and one again after.
#[test]
fn action_to_ros_serves_on_dds_only_while_the_bus_has_a_server() {
    within(async {
        let name = "/bridge_action_test/followed";
        let bus = Bus::new();
        let mut bridge = new_bridge("followed_bridge", &bus.scope("bridge"));
        bridge.action_to_ros::<Waypoint::Action>(name).unwrap();
        let running = start(bridge);
        let tool = Peer::new("followed_tool");
        let client = tool.with(|n| n.create_action_client::<Waypoint::Action>(name)).unwrap();
        assert!(!found_within_a_second(&client).await, "no server while the bus has none");
        assert!(!shown(&tool, name) && !running.action(name).dds_server, "nothing on the graph");

        // The module is up: so is the DDS server, which serves its goals.
        let nav = bus.scope("nav_action_server");
        let _first = serve_on_bus(&nav, name);
        wait("the DDS server", r2r::Node::is_available(&client).unwrap()).await.unwrap();
        assert!(shown(&tool, name) && running.action(name).dds_server);
        let (_goal, result_future, _f) = wait("a goal", client.send_goal_request(goal("a")).unwrap()).await.unwrap();
        assert_eq!(wait("its result", result_future).await.unwrap().0, r2r::GoalStatus::Succeeded);

        // It fails: its scope closes, and the DDS server goes too.
        nav.close();
        eventually("the DDS server to leave the graph", || !shown(&tool, name)).await;
        assert!(!running.action(name).dds_server);
        assert!(!found_within_a_second(&client).await);

        // Started again: it serves again.
        let _second = serve_on_bus(&bus.scope("nav_action_server"), name);
        wait("the DDS server again", r2r::Node::is_available(&client).unwrap()).await.unwrap();
        let (_goal, result_future, _f) = wait("a goal", client.send_goal_request(goal("b")).unwrap()).await.unwrap();
        assert_eq!(wait("its result", result_future).await.unwrap(), (r2r::GoalStatus::Succeeded, result(true)));
        let stats = running.action(name);
        assert_eq!((stats.forwarded, stats.unanswered, stats.resyncs), (2, 0, 3), "came, went and came back");
        running.stop().await.unwrap();
    });
}

/// nav's module ends mid-goal (its scope closes): the bus goal is lost, and
/// the DDS server goes with the bus server, as nav's r2r server went with its
/// node. The DDS goal ends as it did then: no result, and no answer to a
/// cancel while the module is down. The restarted module's server does not
/// know it (`GoalCancelUnknownGoalID`), as nav's restarted r2r server did
/// not, and serves the next goal.
#[test]
fn a_goal_running_when_the_module_restarts_ends_as_with_its_r2r_server_gone() {
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
        let (dds_goal, result_future, _f) = wait("the goal", client.send_goal_request(goal("forever")).unwrap()).await.unwrap();
        let mut result_future = Box::pin(result_future);

        nav.close();
        eventually("the DDS server to leave the graph", || !shown(&tool, name)).await;
        assert_eq!(running.action(name).unanswered, 1, "the lost goal");
        let cancel = tokio::time::timeout(Duration::from_secs(1), dds_goal.cancel().unwrap()).await;
        assert!(cancel.is_err(), "no answer to a cancel: {cancel:?}");
        assert!((&mut result_future).now_or_never().is_none(), "no result");

        let _second = serve_on_bus(&bus.scope("nav_action_server"), name);
        wait("the restarted module's DDS server", r2r::Node::is_available(&client).unwrap()).await.unwrap();
        let unknown = wait("the cancel's answer", dds_goal.cancel().unwrap()).await;
        assert!(matches!(unknown, Err(r2r::Error::GoalCancelUnknownGoalID)), "{unknown:?}");
        let (_goal, next_result, _f) = wait("the next goal", client.send_goal_request(goal("a")).unwrap()).await.unwrap();
        assert_eq!(wait("its result", next_result).await.unwrap().0, r2r::GoalStatus::Succeeded);
        assert!((&mut result_future).now_or_never().is_none(), "never a result for the lost goal");
        running.stop().await.unwrap();
    });
}

/// A bus server that drops its `ActionServer` with its scope open leaves its
/// goals running (mower_bus README, "Actions"): the DDS server stays for
/// them, as an r2r server stayed while its node lived, and the DDS goal gets
/// its result. Once they have ended, the DDS server goes.
#[test]
fn the_dds_server_stays_while_goals_of_a_dropped_bus_server_run() {
    within(async {
        let name = "/bridge_action_test/dropped";
        let bus = Bus::new();
        let mut bridge = new_bridge("dropped_bridge", &bus.scope("bridge"));
        bridge.action_to_ros::<Waypoint::Action>(name).unwrap();
        let running = start(bridge);
        let nav = bus.scope("nav_action_server");
        let mut requests: ActionServer<WaypointAction> = nav.action_server(name).unwrap();
        let tool = Peer::new("dropped_tool");
        let client = tool.action_client(name).await;
        let sent = client.send_goal_request(goal("a")).unwrap();
        let (mut bus_goal, _cancels) = wait("the bus goal request", requests.next()).await.unwrap().accept().unwrap();
        let (_goal, result_future, _f) = wait("the DDS goal", sent).await.unwrap();

        drop(requests);
        // Longer than the bridge's 200 ms between two changes of its server.
        tokio::time::sleep(Duration::from_millis(600)).await;
        assert!(shown(&tool, name) && running.action(name).dds_server, "the DDS server stays for the goal");
        bus_goal.succeed(result(true)).unwrap();
        assert_eq!(wait("the result", result_future).await.unwrap(), (r2r::GoalStatus::Succeeded, result(true)));
        eventually("the DDS server to go once the goal has ended", || !shown(&tool, name)).await;
        assert!(!running.action(name).dds_server);
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

/// Send a bus goal once the bridge serves the name on the bus. It does only
/// once its DDS client is matched with all of the server, so the first goal
/// reaches the server: never `NoResponse` for want of a match, as before the
/// bus server waited for one (a goal mower_coverage would then have tracked
/// with correlated cancels).
async fn send_once_served(
    client: &mower_bus::ActionClient<WaypointAction>, dispatch_id: &str,
) -> Result<(mower_bus::ActionClientGoal<WaypointAction>, mower_bus::ResultFuture<WaypointAction>, mower_bus::FeedbackStream<WaypointAction>), BusError> {
    wait("the bridge's bus server", client.wait_for_server()).await.unwrap();
    let answer = wait("the goal response", client.send_goal_request(goal(dispatch_id)).unwrap()).await;
    assert!(!matches!(answer, Err(BusError::NoResponse { .. })), "the first goal: {answer:?}");
    answer
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

        let (handle, result_future, feedbacks) = send_once_served(&client, "a").await.unwrap();
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
        let (_handle, result_future, _f) = send_once_served(&client, "a").await.unwrap();
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

/// The graph shows a DDS server's goal reply writer before the bridge's
/// client is matched with all of the server, which rcl_action needs (its
/// three services and two topics): only a goal service on the graph, as here,
/// is what a server coming up shows first. The bus server waits for the
/// whole, as an r2r client's `is_available` did, so no goal is sent that
/// would come back `NoResponse`.
#[test]
fn a_dds_server_not_matched_whole_gets_no_bus_server() {
    within(async {
        let name = "/bridge_action_test/half";
        let bus = Bus::new();
        let mut bridge = new_bridge("half_bridge", &bus.scope("bridge"));
        bridge.action_from_ros::<Waypoint::Action>(name).unwrap();
        let running = start(bridge);
        let client = bus.scope("boustrophedon_coverage").action_client::<WaypointAction>(name).unwrap();
        let nav2 = Peer::new("half_nav2");
        type SendGoal = <Waypoint::Action as r2r::WrappedActionTypeSupport>::SendGoal;
        let goal_service = format!("{name}/_action/send_goal");
        let half = nav2.with(|n| n.create_service::<SendGoal>(&goal_service, QosProfile::services_default())).unwrap();
        // Each look finds the writer and probes, in vain, for 500 ms.
        eventually("three looks at the half server", || running.action(name).resyncs >= 3).await;
        assert!(!client.is_available() && !running.action(name).bus_server, "no bus server for half a server");

        // The whole server comes: so does the bus server.
        nav2.with(|n| n.destroy_service(half));
        let _server = serve_on_dds(&nav2, name, Duration::ZERO);
        let (_handle, result_future, _f) = send_once_served(&client, "a").await.unwrap();
        assert_eq!(wait("the result", result_future).await, Ok((GoalStatus::Succeeded, result(true))));
        assert_eq!(running.action(name).unanswered, 0);
        running.stop().await.unwrap();
    });
}

/// Each time a DDS server comes, the bus server comes once the bridge's
/// client is matched with it, and the first goal is the server's; each time
/// it goes, so does the bus server, and what a probe found of it counts for
/// nothing after (`DdsServer::forget`).
#[test]
fn every_dds_server_that_comes_is_matched_before_its_first_goal() {
    within(async {
        let name = "/bridge_action_test/rounds";
        let bus = Bus::new();
        let mut bridge = new_bridge("rounds_bridge", &bus.scope("bridge"));
        bridge.action_from_ros::<Waypoint::Action>(name).unwrap();
        let running = start(bridge);
        let client = bus.scope("boustrophedon_coverage").action_client::<WaypointAction>(name).unwrap();
        for round in 0..4 {
            let nav2 = Peer::new(&format!("rounds_nav2_{round}"));
            let _server = serve_on_dds(&nav2, name, Duration::ZERO);
            let (_handle, result_future, _f) = send_once_served(&client, "a").await.unwrap();
            assert_eq!(wait("the result", result_future).await, Ok((GoalStatus::Succeeded, result(true))), "round {round}");
            drop(nav2);
            wait("the bus server to go", client.wait_for_no_server()).await.unwrap();
        }
        assert_eq!(running.action(name).unanswered, 0);
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
        let (handle, result_future, _f) = send_once_served(&client, "forever").await.unwrap();
        drop(nav2);
        let lost = wait("the lost goal", result_future).await;
        assert!(matches!(lost, Err(BusError::NoResponse { .. })), "never a terminal status: {lost:?}");
        assert_eq!(handle.get_status(), Ok(GoalStatus::Unknown));
        assert!(running.action(name).unanswered >= 1);
        running.stop().await.unwrap();
    });
}

/// The cancel requests of a goal lost with its DDS server are lost with it:
/// each gets `NoResponse`, never a refusal, since nobody refused it. That
/// holds for those the bridge sent to DDS, for those that waited in the
/// goal's queue while the bridge gave the ones at DDS their last second
/// (the goal goes before its cancel stream), and for those after.
#[test]
fn the_cancel_requests_of_a_goal_lost_with_its_dds_server_get_no_response() {
    within(async {
        let name = "/bridge_action_test/lost_cancels";
        let bus = Bus::new();
        let mut bridge = new_bridge("lost_cancels_bridge", &bus.scope("bridge"));
        bridge.action_from_ros::<Waypoint::Action>(name).unwrap();
        let running = start(bridge);
        let client = bus.scope("boustrophedon_coverage").action_client::<WaypointAction>(name).unwrap();
        let nav2 = Peer::new("lost_cancels_nav2");
        let _server = serve_on_dds(&nav2, name, Duration::ZERO);
        let (handle, result_future, _f) = send_once_served(&client, "forever").await.unwrap();

        drop(nav2);
        // A cancel request every 20 ms until the goal is lost, and a few
        // more after.
        let mut result_future = Box::pin(result_future);
        let (mut lost, mut after, mut answers) = (None, 0, Vec::new());
        let deadline = Instant::now() + WAIT;
        while after < 5 {
            assert!(Instant::now() < deadline, "the goal was never lost");
            answers.push(tokio::spawn(handle.cancel().unwrap()));
            match lost {
                None => lost = (&mut result_future).now_or_never(),
                Some(_) => after += 1,
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        assert!(matches!(lost, Some(Err(BusError::NoResponse { .. }))), "{lost:?}");
        for answer in answers {
            let answer = wait("a cancel's answer", answer).await.unwrap();
            assert!(matches!(answer, Err(BusError::NoResponse { .. })), "{answer:?}");
        }
        running.stop().await.unwrap();
    });
}

/// A DDS server that takes the first cancel request of each goal, ends the
/// goal SUCCEEDED, and answers the request after the result: an acceptance
/// or a rejection, after the milliseconds the goal's dispatch id names
/// ("accept 300").
fn serve_on_dds_answering_cancels_after_the_result(peer: &Peer, name: &str) -> tokio::task::JoinHandle<()> {
    let mut requests = peer.with(|n| n.create_action_server::<Waypoint::Action>(name)).unwrap();
    tokio::spawn(async move {
        while let Some(request) = requests.next().await {
            let (answer, after) = request.goal.dispatch_id.split_once(' ').expect("\"accept <ms>\" or \"reject <ms>\"");
            let (accept, after) = (answer == "accept", Duration::from_millis(after.parse().unwrap()));
            let (mut goal, mut cancels) = request.accept().unwrap();
            tokio::spawn(async move {
                let cancel = cancels.next().await.unwrap();
                goal.succeed(result(true)).unwrap();
                tokio::time::sleep(after).await;
                if accept {
                    cancel.accept();
                } else {
                    cancel.reject();
                }
            });
        }
    })
}

/// The DDS server's answer to a cancel request reaches the bus client also
/// when it comes after the result, up to `dds_call`, as it reached a direct
/// r2r client. DDS orders the two on separate entities, so the bridge ends
/// the bus goal only once the answers in flight are in, or 1 s has passed:
/// an acceptance by then is one on the bus; a later one finds the bus goal
/// ended and reads `GoalTerminated`.
#[test]
fn a_dds_cancel_answer_that_comes_after_the_result_still_reaches_the_bus_client() {
    within(async {
        let name = "/bridge_action_test/late_cancel_answer";
        let nav2 = Peer::new("late_cancel_answer_nav2");
        let _server = serve_on_dds_answering_cancels_after_the_result(&nav2, name);
        let bus = Bus::new();
        let timeouts = ServiceTimeouts { dds_call: Duration::from_secs(8), ..ServiceTimeouts::default() };
        let mut bridge = new_bridge("late_cancel_answer_bridge", &bus.scope("bridge")).with_service_timeouts(timeouts);
        bridge.action_from_ros::<Waypoint::Action>(name).unwrap();
        let running = start(bridge);
        let client = bus.scope("boustrophedon_coverage").action_client::<WaypointAction>(name).unwrap();
        let rejected = || BusError::CancelRejected { name: name.into() };
        let terminated = || BusError::GoalTerminated { name: name.into() };
        for (id, expected) in [("accept 300", Ok(())), ("reject 2500", Err(rejected())), ("accept 2500", Err(terminated()))] {
            let (handle, result_future, _f) = send_once_served(&client, id).await.unwrap();
            let cancel = handle.cancel().unwrap();
            assert_eq!(wait("the result", result_future).await, Ok((GoalStatus::Succeeded, result(true))), "{id}");
            assert_eq!(wait("the cancel's answer", cancel).await, expected, "{id}");
        }
        assert_eq!(running.action(name).unanswered, 0);
        running.stop().await.unwrap();
    });
}

/// mower_coverage keeps a goal request that its server accepts late
/// (coverage_cancel_check.py's scenario B accepts after 4 s). Within
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
        let (_handle, result_future, _f) = send_once_served(&client, "a").await.unwrap();
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
        // The server answers after the deadline now: NoResponse.
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

/// What r2r cannot convert (here a NUL in a string) is dropped and counted,
/// as for topics and services (README.md, "Values r2r cannot convert"): a
/// bus server's feedback never reaches DDS, and its goal goes on; a bus
/// client's goal never reaches the DDS server, and the client gets
/// `NoResponse`. Neither panics the bridge.
#[test]
fn what_r2r_cannot_convert_is_dropped_and_counted() {
    within(async {
        let (to_ros, from_ros) = ("/bridge_action_test/invalid_to", "/bridge_action_test/invalid_from");
        let bus = Bus::new();
        let mut bridge = new_bridge("invalid_bridge", &bus.scope("bridge"));
        bridge.action_to_ros::<Waypoint::Action>(to_ros).unwrap();
        bridge.action_from_ros::<Waypoint::Action>(from_ros).unwrap();
        let running = start(bridge);
        let mut requests: ActionServer<WaypointAction> = bus.scope("nav_action_server").action_server(to_ros).unwrap();
        let _nav = tokio::spawn(async move {
            while let Some(request) = requests.next().await {
                let (mut goal, _cancels) = request.accept().unwrap();
                goal.publish_feedback(feedback("a\0b")).unwrap();
                goal.publish_feedback(feedback("fine")).unwrap();
                goal.succeed(result(true)).unwrap();
            }
        });
        let tool = Peer::new("invalid_tool");
        let client = tool.action_client(to_ros).await;
        let (_goal, result_future, mut feedbacks) = wait("the goal", client.send_goal_request(goal("a")).unwrap()).await.unwrap();
        assert_eq!(wait("the feedback", feedbacks.next()).await.unwrap().feedback, "fine");
        assert_eq!(wait("the result", result_future).await.unwrap(), (r2r::GoalStatus::Succeeded, result(true)));
        assert_eq!(running.action(to_ros).invalid, 1);

        let _server = serve_on_dds(&tool, from_ros, Duration::ZERO);
        let coverage = bus.scope("boustrophedon_coverage").action_client::<WaypointAction>(from_ros).unwrap();
        wait("the bus server", coverage.wait_for_server()).await.unwrap();
        let dropped = wait("the answer", coverage.send_goal_request(goal("a\0b")).unwrap()).await;
        assert!(matches!(dropped, Err(BusError::NoResponse { .. })), "{dropped:?}");
        assert_eq!((running.action(from_ros).invalid, running.action(from_ros).unanswered), (1, 1));
        let (_handle, result_future, _f) = wait("the next goal", coverage.send_goal_request(goal("b")).unwrap()).await.unwrap();
        assert_eq!(wait("its result", result_future).await, Ok((GoalStatus::Succeeded, result(true))));
        running.stop().await.unwrap();
    });
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
