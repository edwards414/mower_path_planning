//! The mux module on a bus of its own, on a manual clock: what it forwards,
//! that it forwards the very message it got, that a wall-clock step changes
//! nothing (decision D1), its /diagnostics, the interlock against a second
//! mux, and its endpoints against mower_bridge's manifest.
//!
//! - **Isolation:** ROS domain 90 (the other crates' DDS tests use 77-89),
//!   the r2r node in a namespace of the tests' own, one test at a time.
//! - **Waiting:** for what the module does, never a sleep that hopes.

use std::sync::{Arc, Mutex, MutexGuard, OnceLock, PoisonError};
use std::time::Duration;

use mower_bus::{Bus, Clock, ManualClock, Publisher, Qos, Scope, Subscriber, Time};
use mower_rs_common::{ModuleCtx, Remaps, Shutdown};
use r2r::diagnostic_msgs::msg::DiagnosticArray;
use r2r::geometry_msgs::msg::TwistStamped;
use r2r::std_msgs::msg::Bool;

const WAIT: Duration = Duration::from_secs(10);
const MS: Duration = Duration::from_millis(1);

fn context() -> r2r::Context {
    static CONTEXT: OnceLock<r2r::Context> = OnceLock::new();
    CONTEXT
        .get_or_init(|| {
            // rcl reads it when the context is created, and only then
            std::env::set_var("ROS_DOMAIN_ID", "90");
            r2r::Context::create().expect("an rcl context (source /opt/ros/jazzy/setup.bash)")
        })
        .clone()
}

fn within(test: impl std::future::Future<Output = ()>) {
    static ONE_AT_A_TIME: Mutex<()> = Mutex::new(());
    let _serial: MutexGuard<'_, ()> = ONE_AT_A_TIME.lock().unwrap_or_else(PoisonError::into_inner);
    let runtime = tokio::runtime::Builder::new_multi_thread().worker_threads(2).enable_all().build().unwrap();
    runtime.block_on(async {
        tokio::time::timeout(Duration::from_secs(60), test).await.expect("the test hung");
    });
}

/// twist_mux_topics.yaml as rcl hands it to the node.
fn production_params() -> Vec<(&'static str, r2r::ParameterValue)> {
    use r2r::ParameterValue::{Bool as B, Double as D, Integer as I, String as S};
    let mut p = vec![("use_stamped", B(true))];
    for (name, topic, timeout, priority) in [
        ("topics.navigation_safety_stop", "/navigation_safety_stop", 0.2, 50),
        ("topics.navigation", "/nav_cmd_vel", 0.5, 10),
        ("topics.app_manual", "/joy_cmd", 0.5, 90),
        ("topics.physical_joystick", "/physical_joy_cmd", 0.5, 100),
        ("topics.keyboard", "/keyboard_cmd_vel", 0.5, 80),
        ("locks.navigation_coordinator", "/navigation_coordinator_lock", 0.3, 50),
    ] {
        let leak = |s: String| -> &'static str { Box::leak(s.into_boxed_str()) };
        p.push((leak(format!("{name}.topic")), S(topic.into())));
        p.push((leak(format!("{name}.timeout")), D(timeout)));
        p.push((leak(format!("{name}.priority")), I(priority)));
    }
    p
}

/// robot.launch.py's rule for the mux, and one for another node that must
/// not apply.
fn production_remaps() -> Arc<Remaps> {
    let argv: Vec<String> = "--ros-args -r twist_mux:cmd_vel_out:=/cmd_vel_guard_input \
         -r velocity_command_guard:cmd_vel_out:=/drivetrain_guarded_cmd_vel"
        .split_whitespace()
        .map(str::to_string)
        .collect();
    Arc::new(Remaps::from_args(&argv).unwrap())
}

struct Running {
    bus: Bus,
    feeder: Scope,
    manual: ManualClock,
    m: ModuleCtx,
    shutdown: Shutdown,
    task: tokio::task::JoinHandle<mower_rs_common::ModuleResult>,
}

impl Running {
    /// The module as mower_rsd starts it, with `params` as its node's
    /// overrides and its node in `ns`.
    async fn start(ns: &str, params: &[(&str, r2r::ParameterValue)]) -> Running {
        let node = r2r::Node::create(context(), "twist_mux", ns).unwrap();
        for (key, value) in params {
            node.params.lock().unwrap().insert(key.to_string(), r2r::Parameter::new(value.clone()));
        }
        let (clock, manual) = Clock::manual(Time::from_ns(1_793_000_000_000_000_000));
        let bus = Bus::new();
        let shutdown = Shutdown::new();
        let m = ModuleCtx::new("twist_mux", shutdown.clone())
            .with_bus(bus.scope("twist_mux"))
            .with_clock(clock.clone())
            .with_remaps(production_remaps());
        let task = tokio::spawn(mower_mux::run_node(node, m.clone()));
        let r = Running { feeder: bus.scope("feeder"), bus, manual, m, shutdown, task };
        // up: the start announced, the interlock's first look taken, and the
        // two diagnostics timers waiting for their first second
        r.wait(|| r.bus.topics().iter().any(|t| t.name == "/diagnostics" && t.published >= 1)).await;
        r.wait(|| r.manual.next_deadline() == Some(mower_bus::Mono::from_ns(1_000_000_000))).await;
        r
    }

    async fn wait(&self, ready: impl Fn() -> bool) {
        let deadline = tokio::time::Instant::now() + WAIT;
        while !ready() {
            assert!(tokio::time::Instant::now() < deadline, "timed out");
            tokio::time::sleep(MS).await;
        }
    }

    fn input(&self, topic: &str) -> Publisher<TwistStamped> {
        self.feeder.publisher::<TwistStamped>(topic, Qos::default()).unwrap()
    }

    /// Publish and let the module take it.
    async fn send(&self, publisher: &Publisher<TwistStamped>, frame: &str) -> Arc<TwistStamped> {
        let mut msg = TwistStamped::default();
        msg.header.frame_id = frame.to_string();
        msg.header.stamp.sec = 7;
        msg.twist.linear.x = 0.25;
        let msg = Arc::new(msg);
        publisher.publish_arc(msg.clone()).unwrap();
        self.settle().await;
        msg
    }

    async fn lock(&self, publisher: &Publisher<Bool>, data: bool) {
        publisher.publish(&Bool { data }).unwrap();
        self.settle().await;
    }

    /// Long enough for the module's one task to take what was just
    /// published (the bus shows no reader's queue; what must arrive is
    /// waited for where it arrives).
    async fn settle(&self) {
        tokio::time::sleep(20 * MS).await;
    }

    async fn stop(self) {
        self.shutdown.trigger();
        self.task.await.unwrap().unwrap();
    }
}

fn drain<T: mower_bus::Message>(reader: &mut Subscriber<T>) -> Vec<Arc<T>> {
    std::iter::from_fn(|| reader.try_recv().ok()).collect()
}

/// The production rules on the bus: the very message goes out (the same
/// `Arc`), a true lock masks nav but not the safety stop, a false one lets
/// nav through, joy wins for its 0.5 s to the nanosecond, and none of it
/// moves when the wall clock steps back 10 s or forward an hour.
#[test]
fn it_forwards_the_winner_as_it_came_on_the_monotonic_clock() {
    within(async {
        let r = Running::start("/e4_mux_test_forward", &production_params()).await;
        let mut out = r.feeder.subscribe::<TwistStamped>("/cmd_vel_guard_input", Qos::default().keep_last(100)).unwrap();
        let (safety, nav, joy) = (r.input("/navigation_safety_stop"), r.input("/nav_cmd_vel"), r.input("/joy_cmd"));
        let lock = r.feeder.publisher::<Bool>("/navigation_coordinator_lock", Qos::default()).unwrap();
        mower_bridge::coverage::assert_in_manifest(&r.m);

        // nothing heard: the lock is locked, nav does not pass
        r.send(&nav, "nav#0").await;
        assert!(drain(&mut out).is_empty());
        r.lock(&lock, true).await;
        let sent = r.send(&safety, "safety#0").await;
        let got = drain(&mut out);
        assert_eq!(got.len(), 1);
        assert!(Arc::ptr_eq(&got[0], &sent), "not the same message");
        r.send(&nav, "nav#1").await;
        assert!(drain(&mut out).is_empty(), "nav under a true lock");

        // the safety stop's 0.2 s pass (on the monotonic clock), and the wall
        // clock steps; nothing that depends on arrivals moves with it
        r.manual.advance(Duration::from_millis(250));
        r.manual.set_wall(Time::from_ns(1_793_000_000_000_000_000 - 10_000_000_000));
        r.lock(&lock, false).await;
        r.manual.set_wall(Time::from_ns(1_793_000_000_000_000_000 + 3_600_000_000_000));
        r.send(&nav, "nav#2").await;
        assert_eq!(drain(&mut out).iter().map(|m| m.header.frame_id.clone()).collect::<Vec<_>>(), ["nav#2"]);

        // joy for exactly 0.5 s, then a nanosecond more
        r.lock(&lock, true).await;
        r.send(&joy, "joy#0").await;
        assert_eq!(drain(&mut out).len(), 1);
        r.manual.advance(Duration::from_millis(250));
        r.lock(&lock, true).await;
        r.manual.advance(Duration::from_millis(250));
        r.send(&safety, "safety#1").await;
        assert!(drain(&mut out).is_empty(), "joy is 0.5 s old: still fresh");
        r.manual.advance(Duration::from_nanos(1));
        r.lock(&lock, true).await;
        r.send(&safety, "safety#2").await;
        assert_eq!(drain(&mut out).iter().map(|m| m.header.frame_id.clone()).collect::<Vec<_>>(), ["safety#2"]);
        r.stop().await;
    });
}

/// diagnostic_updater's "Node starting up" at once, then twist_mux's tick
/// and the updater's at every second, in that order, with the production
/// handles' text.
#[test]
fn it_announces_the_start_then_two_statuses_a_second() {
    within(async {
        let r = Running::start("/e4_mux_test_diag", &production_params()).await;
        // a reader after the start misses it, as on DDS: the start is in the
        // introspection's count
        let mut diag = r.feeder.subscribe::<DiagnosticArray>("/diagnostics", Qos::default().keep_last(100)).unwrap();
        let info = r.bus.topics().into_iter().find(|t| t.name == "/diagnostics").unwrap();
        assert_eq!(info.published, 1, "the start, before any tick");
        assert_eq!(info.publisher_qos, [Qos::default().keep_last(1)]);
        r.manual.advance(Duration::from_millis(999));
        r.settle().await;
        assert!(drain(&mut diag).is_empty());
        r.manual.advance(MS);
        r.wait(|| r.bus.topics().iter().any(|t| t.name == "/diagnostics" && t.published == 3)).await;
        let got = drain(&mut diag);
        assert_eq!(got.len(), 2);
        for array in &got {
            let s = &array.status[0];
            assert_eq!((s.name.as_str(), s.level, s.message.as_str(), s.hardware_id.as_str()),
                       ("twist_mux: Twist mux status", 0, "ok", "none"));
            assert_eq!(s.values.len(), 9);
            assert_eq!(s.values[3].key, "velocity topics.navigation_safety_stop");
            assert_eq!(s.values[3].value, " masked (listening to /navigation_safety_stop @ 0.200000s with priority #50)");
            assert_eq!(s.values[6].value, "50");
            assert_eq!(array.header.stamp.sec, 1_793_000_001);
        }
        r.manual.advance(Duration::from_secs(1));
        r.wait(|| r.bus.topics().iter().any(|t| t.name == "/diagnostics" && t.published == 5)).await;
        assert_eq!(drain(&mut diag).len(), 2);
        r.stop().await;
    });
}

/// A second writer of the output on DDS (a C++ twist_mux the launch should
/// not have started): from the next look on, nothing is forwarded; once it
/// is gone, forwarding comes back.
#[test]
fn it_forwards_nothing_while_another_writer_feeds_the_final_guard() {
    within(async {
        let r = Running::start("/e4_mux_test_interlock", &production_params()).await;
        let mut out = r.feeder.subscribe::<TwistStamped>("/cmd_vel_guard_input", Qos::default().keep_last(100)).unwrap();
        let safety = r.input("/navigation_safety_stop");
        let lock = r.feeder.publisher::<Bool>("/navigation_coordinator_lock", Qos::default()).unwrap();
        r.lock(&lock, true).await;
        r.send(&safety, "safety#0").await;
        assert_eq!(drain(&mut out).len(), 1);

        let mut other = r2r::Node::create(context(), "twist_mux_cpp", "/e4_mux_test_interlock_other").unwrap();
        let writer = other
            .create_publisher::<TwistStamped>("/cmd_vel_guard_input", r2r::QosProfile::default().keep_last(1))
            .unwrap();
        // the next look, 2 s on
        r.manual.advance(Duration::from_secs(2));
        let mut blocked = false;
        for k in 0..200 {
            r.lock(&lock, true).await;
            r.send(&safety, &format!("safety#{}", k + 1)).await;
            if drain(&mut out).is_empty() {
                blocked = true;
                break;
            }
            r.manual.advance(Duration::from_secs(2));
        }
        assert!(blocked, "still forwarding next to another writer");
        drop(writer);
        drop(other);
        let mut back = false;
        for k in 0..200 {
            r.manual.advance(Duration::from_secs(2));
            r.lock(&lock, true).await;
            r.send(&safety, &format!("safety#{}", 300 + k)).await;
            if !drain(&mut out).is_empty() {
                back = true;
                break;
            }
        }
        assert!(back, "not forwarding again once the other writer went");
        r.stop().await;
    });
}

/// What twist_mux refuses, the module refuses at its start (mower_rsd then
/// starts it again 2 s later).
#[test]
fn a_configuration_twist_mux_would_not_take_fails_the_start() {
    within(async {
        let mut params = production_params();
        params.retain(|(k, _)| *k != "use_stamped");
        params.push(("use_stamped", r2r::ParameterValue::Bool(false)));
        let node = r2r::Node::create(context(), "twist_mux", "/e4_mux_test_refused").unwrap();
        for (key, value) in &params {
            node.params.lock().unwrap().insert(key.to_string(), r2r::Parameter::new(value.clone()));
        }
        let m = ModuleCtx::new("twist_mux", Shutdown::new()).with_remaps(production_remaps());
        let e = mower_mux::run_node(node, m).await.unwrap_err();
        assert!(e.to_string().contains("use_stamped is false"), "{e}");
    });
}
