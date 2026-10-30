//! The final guard on the bus (E2 step 6): the module's real `run_on_bus`, on a scope of a `Bus` and a manual clock,
//! with mower_rsd's `-r` rules; twist_mux's output and mower_base's input are another scope of the same bus, as the
//! bridge and the base reach it in mower_rsd.
//!
//! - **r2r:** the guard keeps a node for its parameters and logs, on ROS domain 90; no message goes through DDS. One
//!   guard at a time: two nodes of one name must not share the process's rosout writer.
//! - **Time:** the wall clock the guard stamps and checks with, and the monotonic one of its receipt timeout and its
//!   watchdog timer, are the manual clock's; a tick is one `advance`. Each test runs on one thread, so what the guard
//!   does with a message is done once the test yields (`settle`).

use std::sync::{Arc, Mutex, MutexGuard, OnceLock, PoisonError};
use std::time::{Duration, SystemTime};

use mower_bus::{Bus, Clock, ManualClock, Publisher, Qos, Scope, Subscriber, Time};
use mower_rs_common::{ModuleCtx, ModuleResult, Shutdown};
use r2r::geometry_msgs::msg::TwistStamped;
use r2r::{Parameter, ParameterValue};

/// For anything that waits in real time. A pass never waits this long.
const WAIT: Duration = Duration::from_secs(10);

/// The guard's endpoints: reliable, volatile, keep last 1.
const ONE: Qos = Qos::default().keep_last(1);

fn context() -> r2r::Context {
    static CONTEXT: OnceLock<r2r::Context> = OnceLock::new();
    CONTEXT
        .get_or_init(|| {
            // rcl reads it when the context is created, and only then.
            std::env::set_var("ROS_DOMAIN_ID", "90");
            r2r::Context::create().expect("an rcl context (source /opt/ros/jazzy/setup.bash)")
        })
        .clone()
}

fn ms(n: u64) -> Duration {
    Duration::from_millis(n)
}

/// Let the module's tasks run what they were woken for.
async fn settle() {
    for _ in 0..20 {
        tokio::task::yield_now().await;
    }
}

/// One start of a guard, as mower_rsd starts it: its node name, the production rules, a scope of `bus`, the manual
/// clock. It holds the turn until it is dropped.
struct Guard {
    m: ModuleCtx,
    shutdown: Shutdown,
    scope: Scope,
    task: Option<tokio::task::JoinHandle<ModuleResult>>,
    _turn: MutexGuard<'static, ()>,
}

impl Guard {
    fn start(bus: &Bus, clock: Clock, node_name: &str, params: &[(&str, ParameterValue)]) -> Guard {
        static TURN: Mutex<()> = Mutex::new(());
        let turn = TURN.lock().unwrap_or_else(PoisonError::into_inner);
        let node = r2r::Node::create(context(), node_name, "").unwrap();
        for (key, value) in params {
            node.params.lock().unwrap().insert(key.to_string(), Parameter::new(value.clone()));
        }
        let shutdown = Shutdown::new();
        let scope = bus.scope(node_name);
        let m = ModuleCtx::new(node_name, shutdown.clone())
            .with_bus(scope.clone())
            .with_clock(clock)
            .with_remaps(Arc::new(mower_bridge::production_remaps()));
        let task = tokio::spawn(velocity_command_guard::run_on_bus(node, m.clone()));
        Guard { m, shutdown, scope, task: Some(task), _turn: turn }
    }

    /// What mower_rsd does: the module stops when asked, then its scope is closed.
    async fn stop(mut self) -> ModuleResult {
        self.shutdown.trigger();
        let task = self.task.take().unwrap();
        let result = tokio::time::timeout(WAIT, task).await.expect("the guard did not stop").expect("the guard panicked");
        self.scope.close();
        result
    }
}

/// twist_mux's output and the base's input, on the test's scope.
struct Ends {
    _test: Scope,
    mux: Publisher<TwistStamped>,
    base: Subscriber<TwistStamped>,
}

fn ends(bus: &Bus) -> Ends {
    let test = bus.scope("test");
    // what the base asks for: SystemDefaults as CycloneDDS reads it, R V 1
    let base = test.subscribe::<TwistStamped>("/drivetrain_guarded_cmd_vel", Qos::system_default()).unwrap();
    let mux = test.publisher::<TwistStamped>("/cmd_vel_guard_input", ONE).unwrap();
    Ends { _test: test, mux, base }
}

fn manual_clock() -> (Clock, ManualClock) {
    Clock::manual(Time::from_system(SystemTime::now()))
}

fn command(stamp: Time, linear_x: f64, angular_z: f64) -> TwistStamped {
    let mut msg = TwistStamped::default();
    let (sec, nanosec) = stamp.to_parts();
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
    msg.header.frame_id = "twist_mux".to_string();
    msg.twist.linear.x = linear_x;
    msg.twist.angular.z = angular_z;
    msg
}

fn stamp_of(msg: &TwistStamped) -> Time {
    Time::from_parts(msg.header.stamp.sec, msg.header.stamp.nanosec)
}

async fn next(sub: &mut Subscriber<TwistStamped>) -> Arc<TwistStamped> {
    tokio::time::timeout(WAIT, sub.recv()).await.expect("nothing arrived").expect("the test's scope closed")
}

/// Wait until the guard's endpoints are on the bus (it makes them as it starts).
async fn wait_up(ends: &Ends) {
    let deadline = std::time::Instant::now() + WAIT;
    while ends.mux.subscription_count() == 0 || ends.base.publisher_count() == 0 {
        assert!(std::time::Instant::now() < deadline, "the guard's endpoints never came");
        tokio::time::sleep(ms(5)).await;
    }
}

/// The final guard as mower_rsd runs it: twist_mux's `/cmd_vel_guard_input` in, `/drivetrain_guarded_cmd_vel` out
/// (its relative names under the production rules), R V 1 both, every endpoint in the manifest; a zero before
/// anything else; each command forwarded with the robot's stamp and `base_footprint`, the rejected ones as zeros; a
/// last zero when it stops, and nothing of it left on the bus once its scope is closed.
#[tokio::test(flavor = "current_thread")]
async fn the_final_guard_forwards_twist_mux_s_commands_with_the_robot_s_stamp() {
    let bus = Bus::new();
    let (clock, manual) = manual_clock();
    let mut ends = ends(&bus);
    let guard = Guard::start(&bus, clock.clone(), "velocity_command_guard", &[]);
    wait_up(&ends).await;
    mower_bridge::coverage::assert_in_manifest(&guard.m);
    let topics = bus.topics();
    let of = |name: &str| topics.iter().find(|t| t.name == name).unwrap().clone();
    assert_eq!(of("/cmd_vel_guard_input").subscriber_qos, [ONE]);
    assert_eq!(of("/drivetrain_guarded_cmd_vel").publisher_qos, [ONE]);

    // stop before anything can move
    let first = next(&mut ends.base).await;
    assert_eq!((first.twist.linear.x, first.twist.angular.z), (0.0, 0.0));
    assert_eq!(first.header.frame_id, "base_footprint");
    assert_eq!(stamp_of(&first), clock.now());

    // a fresh command: forwarded, restamped with the robot's clock
    manual.advance(ms(10));
    let sent = clock.now() - ms(30);
    ends.mux.publish(&command(sent, 0.3, -0.2)).unwrap();
    let out = next(&mut ends.base).await;
    assert_eq!((out.twist.linear.x, out.twist.angular.z), (0.3, -0.2));
    assert_eq!(stamp_of(&out), clock.now());
    assert_eq!(out.header.frame_id, "base_footprint");

    // 300 ms on: the watchdog's zero, then a command newer than the last but
    // older than max_input_age_s (0.25 s): stale, a zero
    manual.advance(ms(300));
    assert_eq!(next(&mut ends.base).await.twist.linear.x, 0.0, "the watchdog's zero");
    ends.mux.publish(&command(clock.now() - ms(251), 0.3, 0.0)).unwrap();
    let out = next(&mut ends.base).await;
    assert_eq!((out.twist.linear.x, out.twist.angular.z), (0.0, 0.0));
    // the rejection moved the barrier to the robot's clock: an older stamp moves backward, a zero again
    manual.advance(ms(10));
    ends.mux.publish(&command(clock.now() - ms(50), 0.3, 0.0)).unwrap();
    assert_eq!(next(&mut ends.base).await.twist.linear.x, 0.0);
    // overspeed, NaN, lateral: zeros
    for bad in [command(clock.now(), 0.51, 0.0), command(clock.now(), f64::NAN, 0.0), {
        let mut c = command(clock.now(), 0.1, 0.0);
        c.twist.linear.y = 0.1;
        c
    }] {
        manual.advance(ms(10));
        let mut bad = bad;
        let (sec, nanosec) = clock.now().to_parts();
        bad.header.stamp.sec = sec;
        bad.header.stamp.nanosec = nanosec;
        ends.mux.publish(&bad).unwrap();
        assert_eq!(next(&mut ends.base).await.twist.linear.x, 0.0);
    }
    // a zero stamp is a trusted local source's receipt-time command
    manual.advance(ms(10));
    ends.mux.publish(&command(Time::from_ns(0), -0.4, 0.5)).unwrap();
    let out = next(&mut ends.base).await;
    assert_eq!((out.twist.linear.x, out.twist.angular.z), (-0.4, 0.5));

    // a last zero when it stops
    manual.advance(ms(10));
    ends.mux.publish(&command(clock.now(), 0.0, 0.0)).unwrap();
    assert_eq!(next(&mut ends.base).await.twist.linear.x, 0.0);
    guard.stop().await.unwrap();
    let last = next(&mut ends.base).await;
    assert_eq!((last.twist.linear.x, last.twist.angular.z), (0.0, 0.0));
    assert!(ends.base.try_recv().is_err(), "nothing after the last zero");
    assert_eq!(ends.base.publisher_count(), 0);
    assert_eq!(ends.mux.subscription_count(), 0);
}

/// The watchdog on the module's clock: 200 ms after the last fresh non-zero command one zero, and only one; an
/// explicit zero clears it. A timer in rcl's schedule, one period (50 ms) after the start.
#[tokio::test(flavor = "current_thread")]
async fn the_watchdog_forces_one_zero_200_ms_after_the_last_command() {
    let bus = Bus::new();
    let (clock, manual) = manual_clock();
    let mut ends = ends(&bus);
    let guard = Guard::start(&bus, clock.clone(), "velocity_command_guard", &[]);
    wait_up(&ends).await;
    assert_eq!(next(&mut ends.base).await.twist.linear.x, 0.0, "the start's zero");

    ends.mux.publish(&command(clock.now(), 0.2, 0.0)).unwrap();
    assert_eq!(next(&mut ends.base).await.twist.linear.x, 0.2);
    // 150 ms: quiet
    for _ in 0..3 {
        manual.advance(ms(50));
        settle().await;
    }
    assert!(ends.base.try_recv().is_err(), "a zero before the timeout");
    // the next ticks: 200 ms is not past the timeout, 250 ms is
    manual.advance(ms(50));
    settle().await;
    assert!(ends.base.try_recv().is_err(), "a zero at exactly the timeout");
    manual.advance(ms(50));
    let forced = next(&mut ends.base).await;
    assert_eq!((forced.twist.linear.x, forced.twist.angular.z), (0.0, 0.0));
    assert_eq!(stamp_of(&forced), clock.now());
    for _ in 0..10 {
        manual.advance(ms(50));
        settle().await;
    }
    assert!(ends.base.try_recv().is_err(), "a second forced zero");

    // moving again, then an explicit zero: the watchdog stays quiet
    ends.mux.publish(&command(clock.now(), 0.1, 0.0)).unwrap();
    assert_eq!(next(&mut ends.base).await.twist.linear.x, 0.1);
    manual.advance(ms(50));
    settle().await;
    ends.mux.publish(&command(clock.now(), 0.0, 0.0)).unwrap();
    assert_eq!(next(&mut ends.base).await.twist.linear.x, 0.0);
    for _ in 0..10 {
        manual.advance(ms(50));
        settle().await;
    }
    assert!(ends.base.try_recv().is_err(), "a zero after an explicit zero");
    guard.stop().await.unwrap();
}

/// A command still queued when the guard is told to stop is never forwarded after the last zero, and what twist_mux
/// publishes once the guard is gone reaches nothing.
#[tokio::test(flavor = "current_thread")]
async fn nothing_follows_the_last_zero() {
    let bus = Bus::new();
    let (clock, _manual) = manual_clock();
    let mut ends = ends(&bus);
    let guard = Guard::start(&bus, clock.clone(), "velocity_command_guard", &[]);
    wait_up(&ends).await;
    assert_eq!(next(&mut ends.base).await.twist.linear.x, 0.0);
    guard.shutdown.trigger();
    ends.mux.publish(&command(clock.now(), 0.3, 0.0)).unwrap();
    guard.stop().await.unwrap();
    let mut seen = Vec::new();
    while let Ok(msg) = ends.base.try_recv() {
        seen.push(msg.twist.linear.x);
    }
    assert_eq!(seen.last(), Some(&0.0), "{seen:?}");
    ends.mux.publish(&command(clock.now(), 0.3, 0.0)).unwrap();
    settle().await;
    assert!(ends.base.try_recv().is_err());
}

/// E2 step 6 moves the final guard only: the manual guard's bus IO (its session and command clock) comes in step 7,
/// and until then it refuses to start.
#[tokio::test(flavor = "current_thread")]
async fn the_manual_guard_does_not_start_on_the_bus_before_step_7() {
    let bus = Bus::new();
    let (clock, _manual) = manual_clock();
    let params = [("require_command_session", ParameterValue::Bool(true))];
    let mut guard = Guard::start(&bus, clock, "manual_velocity_guard", &params);
    let task = guard.task.take().unwrap();
    let result = tokio::time::timeout(WAIT, task).await.unwrap().unwrap();
    let e = result.unwrap_err().to_string();
    assert!(e.contains("step 7"), "{e}");
    guard.scope.close();
    assert!(bus.topics().iter().all(|t| t.publishers.is_empty() && t.subscribers.is_empty()));
}
