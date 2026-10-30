//! The static_tf module on a bus of its own: what it publishes at the start
//! (the URDF, the fixed joints, latched), the wheels' /tf from /joint_states,
//! and its endpoints against mower_bridge's manifest.
//!
//! - **Isolation:** ROS domain 91 (the mux's tests use 90), the r2r node in a
//!   namespace of the tests' own, one test at a time.

use std::path::PathBuf;
use std::sync::{Mutex, MutexGuard, OnceLock, PoisonError};
use std::time::Duration;

use mower_bus::{Bus, Clock, Qos, Time};
use mower_rs_common::{ModuleCtx, Shutdown};
use r2r::sensor_msgs::msg::JointState;
use r2r::std_msgs::msg::String as Text;
use r2r::tf2_msgs::msg::TFMessage;

const WAIT: Duration = Duration::from_secs(10);

fn context() -> r2r::Context {
    static CONTEXT: OnceLock<r2r::Context> = OnceLock::new();
    CONTEXT
        .get_or_init(|| {
            std::env::set_var("ROS_DOMAIN_ID", "91");
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

fn urdf_text() -> String {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/rsp_capture.json");
    let c: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
    c["urdf"].as_str().unwrap().to_string()
}

fn node(ns: &str, params: &[(&str, r2r::ParameterValue)]) -> r2r::Node {
    let node = r2r::Node::create(context(), "static_tf_publisher", ns).unwrap();
    for (key, value) in params {
        node.params.lock().unwrap().insert(key.to_string(), r2r::Parameter::new(value.clone()));
    }
    node
}

/// The URDF goes out as given and the 7 fixed joints with the start's wall
/// clock, both latched for late joiners; a joint state gives the wheels'
/// transforms with its stamp, the next 40 ms later none (the 50 ms
/// throttle); the endpoints are the manifest's.
#[test]
fn it_publishes_what_robot_state_publisher_publishes() {
    within(async {
        let urdf = urdf_text();
        let n = node("/e4_static_tf_test", &[("robot_description", r2r::ParameterValue::String(urdf.clone()))]);
        // the joint states below are stamped before this, as mower_base's
        // are by the time rsp reads them (a stamp ahead of the wall clock is
        // capped by it: state.rs)
        let start = Time::from_ns(1_793_000_001_123_456_789);
        let (clock, _manual) = Clock::manual(start);
        let bus = Bus::new();
        let shutdown = Shutdown::new();
        let m = ModuleCtx::new("static_tf_publisher", shutdown.clone())
            .with_bus(bus.scope("static_tf_publisher"))
            .with_clock(clock.clone());
        let task = tokio::spawn(mower_static_tf::run_node(n, m.clone()));
        let deadline = tokio::time::Instant::now() + WAIT;
        while !bus.topics().iter().any(|t| t.name == "/tf_static" && t.retained == 1) {
            assert!(tokio::time::Instant::now() < deadline, "no /tf_static");
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        mower_bridge::coverage::assert_in_manifest(&m);

        // late joiners get both latched messages
        let peer = bus.scope("peer");
        let mut tf_static = peer.subscribe::<TFMessage>("/tf_static", Qos::latched().keep_last(100)).unwrap();
        let mut description = peer.subscribe::<Text>("/robot_description", Qos::latched()).unwrap();
        let fixed = tf_static.try_recv().unwrap();
        assert_eq!(fixed.transforms.len(), 7);
        assert!(fixed.transforms.iter().all(|t| t.header.stamp.sec == 1_793_000_001 && t.header.stamp.nanosec == 123_456_789));
        assert_eq!(fixed.transforms[2].header.frame_id, "base_footprint");
        assert_eq!(fixed.transforms[2].child_frame_id, "base_link");
        assert_eq!(description.try_recv().unwrap().data, urdf, "the URDF text as it was given");

        let mut tf = peer.subscribe::<TFMessage>("/tf", Qos::default().keep_last(100)).unwrap();
        let joints = peer.publisher::<JointState>("/joint_states", Qos::system_default()).unwrap();
        let wheels = |stamp_ms: i64, left: f64| {
            let mut js = JointState::default();
            js.header.stamp.sec = (stamp_ms / 1000) as i32;
            js.header.stamp.nanosec = ((stamp_ms % 1000) * 1_000_000) as u32;
            js.name = vec!["left_wheel_joint".into(), "right_wheel_joint".into()];
            js.position = vec![left, -left];
            js
        };
        joints.publish(&wheels(1_793_000_000_200, 0.5)).unwrap();
        let deadline = tokio::time::Instant::now() + WAIT;
        let out = loop {
            if let Ok(m) = tf.try_recv() {
                break m;
            }
            assert!(tokio::time::Instant::now() < deadline, "no /tf");
            tokio::time::sleep(Duration::from_millis(2)).await;
        };
        let pairs: Vec<(&str, &str)> =
            out.transforms.iter().map(|t| (t.header.frame_id.as_str(), t.child_frame_id.as_str())).collect();
        assert_eq!(pairs, [("base_link", "left_wheel_link"), ("base_link", "right_wheel_link")]);
        assert_eq!((out.transforms[0].header.stamp.sec, out.transforms[0].header.stamp.nanosec), (1_793_000_000, 200_000_000));
        joints.publish(&wheels(1_793_000_000_240, 0.6)).unwrap();
        joints.publish(&wheels(1_793_000_000_280, 0.7)).unwrap();
        let deadline = tokio::time::Instant::now() + WAIT;
        let next = loop {
            if let Ok(m) = tf.try_recv() {
                break m;
            }
            assert!(tokio::time::Instant::now() < deadline, "no second /tf");
            tokio::time::sleep(Duration::from_millis(2)).await;
        };
        assert_eq!(next.transforms[0].header.stamp.nanosec, 280_000_000, "the one 40 ms on is throttled");
        shutdown.trigger();
        task.await.unwrap().unwrap();
    });
}

#[test]
fn a_start_without_a_usable_description_fails() {
    within(async {
        for (ns, value) in [("/e4_static_tf_empty", ""), ("/e4_static_tf_broken", "<robot name='x'><link name='a'/>")] {
            let n = node(ns, &[("robot_description", r2r::ParameterValue::String(value.into()))]);
            let m = ModuleCtx::new("static_tf_publisher", Shutdown::new());
            assert!(mower_static_tf::run_node(n, m).await.is_err(), "{value:?}");
        }
    });
}
