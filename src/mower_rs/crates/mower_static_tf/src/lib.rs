//! robot_state_publisher as a module of mower_rsd (step E4 of
//! docs/ROS_FREE_PLAN.md): module `static_tf`, node `static_tf_publisher`,
//! launch switch `rust_static_tf` (compose `RUST_STATIC_TF`, off by default
//! and only with `rust_daemon`).
//!
//! It publishes everything rsp 3.3.4 publishes for the robot (decision D2),
//! from the same `robot_description` (robot.launch.py runs xacro on
//! real_robot.xacro as mower.launch.py does for rsp):
//!
//! - **`/tf_static`**, once at the start, latched (`R TL 1`): one message with
//!   a transform per fixed (and planar) joint, sorted by joint name, all with
//!   one stamp, the wall clock at the start. On this robot the 7 fixed joints
//!   of the URDF, `base_footprint -> base_link` (90° yaw) among them, which the
//!   EKFs and navsat read in-process now. mower_rsd starts localize's three
//!   nodes only once this message is on the bus: navsat's datum needs the
//!   map EKF to have had the IMU's mount from its first sample (the start-up
//!   race of rsp's frames over DDS; mower_rsd's `waits_for_static_tf`).
//! - **`/tf`** (`R V 100`): for each `/joint_states` (`BE V 5`, rsp's
//!   SensorDataQoS) that rsp would publish (`state.rs`: the 50 ms throttle on
//!   the messages' stamps, 12.5 Hz at mower_base's 25 Hz), the moving joints
//!   of it with its stamp: the two wheels.
//! - **`/robot_description`** (`R TL 1`): the URDF text as it was given.
//!
//! The arithmetic is urdfdom's and KDL's, operation for operation (`kdl.rs`);
//! `tests/golden.rs` holds it to what rsp published for the same URDF and the
//! same joint states (`tools/rsp_capture.py`).
//!
//! **Not done:** rsp re-reads `robot_description` when the parameter is set
//! at run time, and serves its parameters; this module reads them once (its
//! r2r node is for the parameters, the log and the graph, like the other
//! modules'), and a new description takes a restart. `frame_prefix`,
//! `publish_frequency` and `ignore_timestamp` are read as rsp reads them.

pub mod kdl;
pub mod state;
pub mod urdf;
pub mod xml;

use futures::StreamExt;
use mower_bus::{Qos, Time};
use mower_rs_common::{params, ModuleCtx, ModuleResult};
use r2r::builtin_interfaces::msg::Time as RosTime;
use r2r::geometry_msgs::msg::{Quaternion, Transform, TransformStamped, Vector3};
use r2r::sensor_msgs::msg::JointState;
use r2r::std_msgs::msg::{Header, String as Text};
use r2r::tf2_msgs::msg::TFMessage;

use crate::state::{Outcome, Throttle};
use crate::urdf::Robot;

/// How the module's transforms are written: rsp's parameters.
#[derive(Clone, Debug, PartialEq)]
pub struct Settings {
    pub publish_frequency: f64,
    pub ignore_timestamp: bool,
    pub frame_prefix: String,
}

impl Default for Settings {
    fn default() -> Self {
        Settings { publish_frequency: 20.0, ignore_timestamp: false, frame_prefix: String::new() }
    }
}

fn stamp(ns: i64) -> RosTime {
    let (sec, nanosec) = Time::from_ns(ns).to_parts();
    RosTime { sec, nanosec }
}

/// rsp's `kdlToTransform` with the stamp and the frames.
fn transform(settings: &Settings, parent: &str, child: &str, f: &kdl::Frame, stamp_ns: i64) -> TransformStamped {
    let ([x, y, z], [qx, qy, qz, qw]) = kdl::to_transform(f);
    TransformStamped {
        header: Header { stamp: stamp(stamp_ns), frame_id: format!("{}{parent}", settings.frame_prefix) },
        child_frame_id: format!("{}{child}", settings.frame_prefix),
        transform: Transform {
            translation: Vector3 { x, y, z },
            rotation: Quaternion { x: qx, y: qy, z: qz, w: qw },
        },
    }
}

/// rsp's `publishFixedTransforms`: one message, the fixed joints by name,
/// one stamp.
pub fn fixed_transforms(robot: &Robot, settings: &Settings, stamp_ns: i64) -> TFMessage {
    TFMessage {
        transforms: robot
            .fixed()
            .map(|j| transform(settings, &j.parent, &j.child, &j.segment.pose(0.0), stamp_ns))
            .collect(),
    }
}

/// rsp's `publishTransforms`: the joints `state.rs` let through.
pub fn moving_transforms(robot: &Robot, settings: &Settings, joints: &[(String, f64)], stamp_ns: i64) -> TFMessage {
    TFMessage {
        transforms: joints
            .iter()
            .filter_map(|(name, q)| robot.moving(name).map(|j| (j, *q)))
            .map(|(j, q)| transform(settings, &j.parent, &j.child, &j.segment.pose(q), stamp_ns))
            .collect(),
    }
}

/// `rclcpp::Time(builtin_interfaces::msg::Time)` in nanoseconds.
fn ns_of(t: &RosTime) -> i64 {
    t.sec as i64 * 1_000_000_000 + t.nanosec as i64
}

/// The module.
pub async fn run(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    let node = r2r::Node::create(ctx, &m.node_name, &m.namespace)?;
    run_node(node, m).await
}

/// Everything after `Node::create`, so that a test can hand in a node whose
/// parameters it has set itself.
pub async fn run_node(node: r2r::Node, m: ModuleCtx) -> ModuleResult {
    let logger = node.logger().to_string();
    let description = params::string(&node, "robot_description", "");
    if description.is_empty() {
        return Err("robot_description is empty: robot.launch.py passes the URDF of real_robot.xacro".into());
    }
    let settings = Settings {
        publish_frequency: params::f64(&node, "publish_frequency", 20.0),
        ignore_timestamp: params::bool(&node, "ignore_timestamp", false),
        frame_prefix: params::string(&node, "frame_prefix", ""),
    };
    if !(settings.publish_frequency > 0.0 && settings.publish_frequency.is_finite()) {
        return Err(format!("publish_frequency must be positive, not {}", settings.publish_frequency).into());
    }
    let robot = urdf::parse(&description).map_err(|e| format!("robot_description: {e}"))?;

    // rsp's broadcasters and publisher: tf2's StaticBroadcasterQoS and
    // DynamicBroadcasterQoS, QoS(1).transient_local()
    let tf_static = m.bus.publisher::<TFMessage>(&m.resolve_topic("/tf_static")?, Qos::latched())?;
    let tf = m.bus.publisher::<TFMessage>(&m.resolve_topic("/tf")?, Qos::default().keep_last(100))?;
    let robot_description = m.bus.publisher::<Text>(&m.resolve_topic("robot_description")?, Qos::latched())?;
    let mut joint_states = m.bus.subscribe::<JointState>(&m.resolve_topic("joint_states")?, Qos::sensor_data())?;

    robot_description.publish_owned(Text { data: description })?;
    let fixed = fixed_transforms(&robot, &settings, m.clock.now().ns());
    let (fixed_count, moving_count) = (fixed.transforms.len(), robot.joints.values().filter(|j| robot.moving(&j.name).is_some()).count());
    tf_static.publish_owned(fixed)?;
    r2r::log_info!(
        &logger,
        "static_tf: robot {} from {}: {fixed_count} fixed joints on /tf_static, {moving_count} moving ones on /tf \
         from /joint_states (step E4; robot_state_publisher's \"Robot initialized\")",
        robot.name,
        robot.root
    );

    let mut throttle = Throttle::new(settings.publish_frequency, settings.ignore_timestamp);
    let mut empty_warned: Option<mower_bus::Mono> = None;
    loop {
        tokio::select! {
            biased;
            () = m.shutdown.wait() => break,
            msg = joint_states.next() => {
                let Some(msg) = msg else { break };
                let js = state::JointState { stamp_ns: ns_of(&msg.header.stamp), name: &msg.name, position: &msg.position };
                let outcome = throttle.on_joint_state(&robot, &js, m.clock.now().ns());
                if throttle.moved_backwards {
                    r2r::log_warn!(&logger, "Moved backwards in time, re-publishing joint transforms!");
                }
                match outcome {
                    Outcome::Publish(joints) => {
                        tf.publish_owned(moving_transforms(&robot, &settings, &joints, js.stamp_ns))?;
                    }
                    Outcome::Throttled => {}
                    Outcome::Invalid { empty_position: false } => {
                        r2r::log_error!(&logger, "Robot state publisher ignored an invalid JointState message");
                    }
                    Outcome::Invalid { empty_position: true } => {
                        // rsp's RCLCPP_WARN_THROTTLE, 10 s
                        let now = m.clock.mono();
                        if empty_warned.is_none_or(|t| now.saturating_duration_since(t).as_secs() >= 10) {
                            empty_warned = Some(now);
                            let first = msg.name.first().map(String::as_str).unwrap_or_default();
                            r2r::log_warn!(
                                &logger,
                                "Robot state publisher ignored a JointState message about joint(s) \"{first}\"(,...) \
                                 whose position member was empty."
                            );
                        }
                    }
                }
            }
        }
    }
    drop(node);
    Ok(())
}
