//! The words the manifest is written in: QoS profiles as constants, r2r
//! types as values, and endpoints. Everything here is `const`, so the
//! manifest is a static table, like mower_rsd's module table.

use std::fmt;

use mower_bus::{Durability, Qos, Reliability};
use mower_bus_ros::{BridgeAction, BridgeError, BridgeMessage, BridgeService, RosBridge};
use mower_rs_common::NameKind;
use r2r::QosProfile;

/// The QoS profile an endpoint's code hands r2r, written the way
/// docs/BUS_TOPICS.md writes it: `R V 10`, `BE V 1`, `R TL 1`, `SD`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Profile {
    /// `QosProfile::system_default()` (mower_base's /odom, /joint_states, /tf
    /// and cmd_vel). The bus reads it as rmw_cyclonedds does, R V 1
    /// (docs/BUS_TOPICS.md 5.5 c); the bridge's DDS endpoint keeps the
    /// profile itself, so it resolves per RMW as the module's own did.
    SystemDefault,
    /// `QosProfile::default().keep_last(depth)` with this reliability and
    /// durability. Deadline, lifespan and liveliness stay r2r's defaults.
    KeepLast(Qos),
}

const fn keep_last(depth: usize, reliability: Reliability, durability: Durability) -> Profile {
    Profile::KeepLast(Qos { depth, reliability, durability })
}

/// `QosProfile::system_default()`.
pub const SD: Profile = Profile::SystemDefault;
/// The guards: `QosProfile::default().keep_last(1)`.
pub const R_V_1: Profile = keep_last(1, Reliability::Reliable, Durability::Volatile);
/// `/mower_base/pid_command`.
pub const R_V_4: Profile = keep_last(4, Reliability::Reliable, Durability::Volatile);
/// `QosProfile::default()`, and rclcpp's `QoS(10)`.
pub const R_V_10: Profile = keep_last(10, Reliability::Reliable, Durability::Volatile);
/// tf2's `TransformBroadcaster` and `TransformListener`, `/rosout` readers.
pub const R_V_100: Profile = keep_last(100, Reliability::Reliable, Durability::Volatile);
/// `QosProfile::default().keep_last(1).best_effort()`.
pub const BE_V_1: Profile = keep_last(1, Reliability::BestEffort, Durability::Volatile);
/// `QosProfile::sensor_data()`.
pub const BE_V_5: Profile = keep_last(5, Reliability::BestEffort, Durability::Volatile);
/// The EKFs' inputs: `SensorDataQoS().keep_last(queue_size)`, queue size 10.
pub const BE_V_10: Profile = keep_last(10, Reliability::BestEffort, Durability::Volatile);
/// Every module's `latched()`, and tf2's `StaticTransformBroadcaster`.
pub const R_TL_1: Profile = keep_last(1, Reliability::Reliable, Durability::TransientLocal);
/// ws_bridge's readers (E3): the depth of 10 its r2r subscriptions have,
/// transient local so that a latched writer's value reaches a late client.
pub const R_TL_10: Profile = keep_last(10, Reliability::Reliable, Durability::TransientLocal);
/// tf2's static listener.
pub const R_TL_100: Profile = keep_last(100, Reliability::Reliable, Durability::TransientLocal);
/// rcl's `/rosout` writer (it also sets a lifespan of 10 s, which only a
/// writer has).
pub const R_TL_1000: Profile = keep_last(1000, Reliability::Reliable, Durability::TransientLocal);

impl Profile {
    /// The r2r profile: what the bridge's DDS endpoint uses.
    pub fn ros(self) -> QosProfile {
        match self {
            Profile::SystemDefault => QosProfile::system_default(),
            Profile::KeepLast(q) => {
                let p = QosProfile::default().keep_last(q.depth);
                let p = match q.reliability {
                    Reliability::Reliable => p.reliable(),
                    Reliability::BestEffort => p.best_effort(),
                };
                match q.durability {
                    Durability::Volatile => p.volatile(),
                    Durability::TransientLocal => p.transient_local(),
                }
            }
        }
    }

    /// What the bus makes of it: `mower_bus_ros::bus_qos`, the conversion the
    /// bridge's bus endpoints use, so that the manifest check compares with
    /// what a converted module asks the bus for.
    pub fn bus(self) -> Qos {
        mower_bus_ros::bus_qos(&self.ros()).expect("every Profile has a bus equivalent")
    }
}

impl fmt::Display for Profile {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Profile::SystemDefault => f.write_str("SD"),
            Profile::KeepLast(q) => fmt_qos(*q, f),
        }
    }
}

/// A bus QoS in docs/BUS_TOPICS.md's notation.
pub(crate) fn fmt_qos(q: Qos, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    let reliability = match q.reliability {
        Reliability::Reliable => "R",
        Reliability::BestEffort => "BE",
    };
    let durability = match q.durability {
        Durability::Volatile => "V",
        Durability::TransientLocal => "TL",
    };
    write!(f, "{reliability} {durability} {}", q.depth)
}

/// [`fmt_qos`] as a value, for messages.
pub(crate) struct ShowQos(pub Qos);

impl fmt::Display for ShowQos {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt_qos(self.0, f)
    }
}

/// An r2r message type as a value: its name, and the bridge's declarations
/// for it.
#[derive(Clone, Copy)]
pub struct MsgType {
    rust: fn() -> &'static str,
    to_ros: fn(&mut RosBridge, &str, QosProfile) -> Result<(), BridgeError>,
    from_ros: fn(&mut RosBridge, &str, QosProfile) -> Result<(), BridgeError>,
}

impl MsgType {
    pub const fn of<T: BridgeMessage>() -> Self {
        MsgType { rust: std::any::type_name::<T>, to_ros: to_ros::<T>, from_ros: from_ros::<T> }
    }

    /// `std::any::type_name`, which bus introspection reports.
    pub fn rust_type(&self) -> &'static str {
        (self.rust)()
    }

    /// The ROS name, e.g. `sensor_msgs/msg/BatteryState`.
    pub fn ros_type(&self) -> String {
        mower_bus::ros_type_name(self.rust_type()).unwrap_or_else(|| self.rust_type().to_string())
    }

    /// `bridge.to_ros::<T>(name, qos)`.
    pub(crate) fn declare_to_ros(self, bridge: &mut RosBridge, name: &str, qos: Profile) -> Result<(), BridgeError> {
        (self.to_ros)(bridge, name, qos.ros())
    }

    /// `bridge.from_ros::<T>(name, qos)`.
    pub(crate) fn declare_from_ros(self, bridge: &mut RosBridge, name: &str, qos: Profile) -> Result<(), BridgeError> {
        (self.from_ros)(bridge, name, qos.ros())
    }
}

fn to_ros<T: BridgeMessage>(bridge: &mut RosBridge, name: &str, qos: QosProfile) -> Result<(), BridgeError> {
    bridge.to_ros::<T>(name, qos)
}

fn from_ros<T: BridgeMessage>(bridge: &mut RosBridge, name: &str, qos: QosProfile) -> Result<(), BridgeError> {
    bridge.from_ros::<T>(name, qos)
}

impl PartialEq for MsgType {
    fn eq(&self, other: &Self) -> bool {
        self.rust_type() == other.rust_type()
    }
}

impl fmt::Debug for MsgType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.ros_type())
    }
}

/// An r2r service type (`…::srv::<Name>::Service`) as a value.
#[derive(Clone, Copy)]
pub struct SrvType {
    request: fn() -> &'static str,
    response: fn() -> &'static str,
    to_ros: fn(&mut RosBridge, &str) -> Result<(), BridgeError>,
    from_ros: fn(&mut RosBridge, &str) -> Result<(), BridgeError>,
}

impl SrvType {
    pub const fn of<S: BridgeService>() -> Self {
        SrvType {
            request: std::any::type_name::<S::Request>,
            response: std::any::type_name::<S::Response>,
            to_ros: service_to_ros::<S>,
            from_ros: service_from_ros::<S>,
        }
    }

    /// The request's `std::any::type_name`, which bus introspection reports.
    pub fn request_type(&self) -> &'static str {
        (self.request)()
    }

    pub fn response_type(&self) -> &'static str {
        (self.response)()
    }

    /// The ROS name, e.g. `std_srvs/srv/Trigger`.
    pub fn ros_type(&self) -> String {
        mower_bus::ros_type_name(self.request_type()).unwrap_or_else(|| self.request_type().to_string())
    }

    /// `bridge.service_to_ros::<S>(name)`.
    pub(crate) fn declare_to_ros(self, bridge: &mut RosBridge, name: &str) -> Result<(), BridgeError> {
        (self.to_ros)(bridge, name)
    }

    /// `bridge.service_from_ros::<S>(name)`.
    pub(crate) fn declare_from_ros(self, bridge: &mut RosBridge, name: &str) -> Result<(), BridgeError> {
        (self.from_ros)(bridge, name)
    }
}

fn service_to_ros<S: BridgeService>(bridge: &mut RosBridge, name: &str) -> Result<(), BridgeError> {
    bridge.service_to_ros::<S>(name)
}

fn service_from_ros<S: BridgeService>(bridge: &mut RosBridge, name: &str) -> Result<(), BridgeError> {
    bridge.service_from_ros::<S>(name)
}

impl PartialEq for SrvType {
    fn eq(&self, other: &Self) -> bool {
        self.request_type() == other.request_type() && self.response_type() == other.response_type()
    }
}

impl fmt::Debug for SrvType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.ros_type())
    }
}

/// An r2r action type (`…::action::<Name>::Action`) as a value. On the bus
/// it is the action `(Goal, Result, Feedback)` (mower_bus README, "Actions").
#[derive(Clone, Copy)]
pub struct ActType {
    goal: fn() -> &'static str,
    result: fn() -> &'static str,
    feedback: fn() -> &'static str,
    to_ros: fn(&mut RosBridge, &str) -> Result<(), BridgeError>,
    from_ros: fn(&mut RosBridge, &str) -> Result<(), BridgeError>,
}

impl ActType {
    pub const fn of<A: BridgeAction>() -> Self {
        ActType {
            goal: std::any::type_name::<A::Goal>,
            result: std::any::type_name::<A::Result>,
            feedback: std::any::type_name::<A::Feedback>,
            to_ros: action_to_ros::<A>,
            from_ros: action_from_ros::<A>,
        }
    }

    /// The goal's `std::any::type_name`, which bus introspection reports.
    pub fn goal_type(&self) -> &'static str {
        (self.goal)()
    }

    pub fn result_type(&self) -> &'static str {
        (self.result)()
    }

    pub fn feedback_type(&self) -> &'static str {
        (self.feedback)()
    }

    /// The ROS name, e.g. `mower_interface/action/Waypoint`.
    pub fn ros_type(&self) -> String {
        mower_bus::ros_type_name(self.goal_type()).unwrap_or_else(|| self.goal_type().to_string())
    }

    /// `bridge.action_to_ros::<A>(name)`.
    pub(crate) fn declare_to_ros(self, bridge: &mut RosBridge, name: &str) -> Result<(), BridgeError> {
        (self.to_ros)(bridge, name)
    }

    /// `bridge.action_from_ros::<A>(name)`.
    pub(crate) fn declare_from_ros(self, bridge: &mut RosBridge, name: &str) -> Result<(), BridgeError> {
        (self.from_ros)(bridge, name)
    }
}

fn action_to_ros<A: BridgeAction>(bridge: &mut RosBridge, name: &str) -> Result<(), BridgeError> {
    bridge.action_to_ros::<A>(name)
}

fn action_from_ros<A: BridgeAction>(bridge: &mut RosBridge, name: &str) -> Result<(), BridgeError> {
    bridge.action_from_ros::<A>(name)
}

impl PartialEq for ActType {
    fn eq(&self, other: &Self) -> bool {
        self.goal_type() == other.goal_type()
            && self.result_type() == other.result_type()
            && self.feedback_type() == other.feedback_type()
    }
}

impl fmt::Debug for ActType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.ros_type())
    }
}

/// What an endpoint does with its name.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Kind {
    Publish,
    Subscribe,
    Serve,
    Call,
    /// An action server.
    ServeAction,
    /// An action client.
    CallAction,
}

/// The namespaces of names: a topic, a service and an action may share a
/// name, on DDS (an action is `<name>/_action/...`) as on the bus.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Space {
    Topic,
    Service,
    Action,
}

impl Kind {
    pub fn is_topic(self) -> bool {
        matches!(self, Kind::Publish | Kind::Subscribe)
    }

    pub fn space(self) -> Space {
        match self {
            Kind::Publish | Kind::Subscribe => Space::Topic,
            Kind::Serve | Kind::Call => Space::Service,
            Kind::ServeAction | Kind::CallAction => Space::Action,
        }
    }

    /// Which `-r` rules resolve its name: rcl_action resolves an action's
    /// with the topic rules (mower_bus README, "Actions").
    pub fn name_kind(self) -> NameKind {
        match self.space() {
            Space::Service => NameKind::Service,
            Space::Topic | Space::Action => NameKind::Topic,
        }
    }

    pub fn verb(self) -> &'static str {
        match self {
            Kind::Publish => "publishes",
            Kind::Subscribe => "subscribes to",
            Kind::Serve => "serves",
            Kind::Call => "calls",
            Kind::ServeAction => "serves action",
            Kind::CallAction => "sends goals to action",
        }
    }
}

/// A topic's type and QoS, a service's type or an action's.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Iface {
    Topic(MsgType, Profile),
    Service(SrvType),
    Action(ActType),
}

/// One topic, service or action end of a module instance, or of a node
/// outside.
#[derive(Clone, Copy, Debug)]
pub struct Endpoint {
    pub kind: Kind,
    /// The name as the code hands it to r2r in production: a literal, or the
    /// production value of [`Endpoint::param`]. A relative one resolves
    /// under the instance's node name with the process's `-r` rules
    /// (`ModuleCtx::resolve_topic`).
    pub code: &'static str,
    /// The absolute name in production (docs/BUS_TOPICS.md 5.4 and 5.7):
    /// `code` resolved with robot.launch.py's rules.
    pub name: &'static str,
    /// The parameter `code` is the production value of, if any.
    pub param: Option<&'static str>,
    pub iface: Iface,
    /// Stays on the module's r2r node when the module moves: navsat's
    /// `/tf_static` writer until E4, the adapter's calls to two parameter
    /// services, and nav's clients of nav2 until E9 (README.md, "The
    /// manifest").
    pub stays_on_r2r: bool,
    /// A writer that is created and never publishes. It needs no forwarding
    /// into the bus.
    pub silent: bool,
    /// Not in production: the endpoint exists only with this configuration
    /// (e.g. `nav_backend: nav_core`). Bridged unless that would need a
    /// second direction on its name.
    pub only: Option<&'static str>,
    /// An outside node's endpoint that this module of mower_rsd takes the
    /// place of (E4: twist_mux, robot_state_publisher). While that module
    /// runs in the process, robot.launch.py does not start the node, and the
    /// policy leaves the endpoint out.
    pub replaced_by: Option<&'static str>,
}

impl Endpoint {
    const fn new(kind: Kind, code: &'static str, iface: Iface) -> Self {
        Endpoint {
            kind,
            code,
            name: code,
            param: None,
            iface,
            stays_on_r2r: false,
            silent: false,
            only: None,
            replaced_by: None,
        }
    }

    /// The production name, when the code's name is relative or remapped.
    pub const fn named(self, name: &'static str) -> Self {
        Endpoint { name, ..self }
    }

    /// The parameter whose production value `code` is.
    pub const fn param(self, key: &'static str) -> Self {
        Endpoint { param: Some(key), ..self }
    }

    pub const fn silent(self) -> Self {
        Endpoint { silent: true, ..self }
    }

    pub const fn stays_on_r2r(self) -> Self {
        Endpoint { stays_on_r2r: true, ..self }
    }

    pub const fn only(self, configuration: &'static str) -> Self {
        Endpoint { only: Some(configuration), ..self }
    }

    /// An outside endpoint that module `id` replaces when it runs in the
    /// process.
    pub const fn replaced_by(self, id: &'static str) -> Self {
        Endpoint { replaced_by: Some(id), ..self }
    }

    /// A topic's QoS profile.
    pub fn profile(&self) -> Option<Profile> {
        match self.iface {
            Iface::Topic(_, profile) => Some(profile),
            Iface::Service(_) | Iface::Action(_) => None,
        }
    }

    /// The ROS type, for messages and tables.
    pub fn ros_type(&self) -> String {
        match &self.iface {
            Iface::Topic(t, _) => t.ros_type(),
            Iface::Service(s) => s.ros_type(),
            Iface::Action(a) => a.ros_type(),
        }
    }
}

pub const fn publishes<T: BridgeMessage>(code: &'static str, qos: Profile) -> Endpoint {
    Endpoint::new(Kind::Publish, code, Iface::Topic(MsgType::of::<T>(), qos))
}

pub const fn subscribes<T: BridgeMessage>(code: &'static str, qos: Profile) -> Endpoint {
    Endpoint::new(Kind::Subscribe, code, Iface::Topic(MsgType::of::<T>(), qos))
}

pub const fn serves<S: BridgeService>(code: &'static str) -> Endpoint {
    Endpoint::new(Kind::Serve, code, Iface::Service(SrvType::of::<S>()))
}

pub const fn calls<S: BridgeService>(code: &'static str) -> Endpoint {
    Endpoint::new(Kind::Call, code, Iface::Service(SrvType::of::<S>()))
}

pub const fn serves_action<A: BridgeAction>(code: &'static str) -> Endpoint {
    Endpoint::new(Kind::ServeAction, code, Iface::Action(ActType::of::<A>()))
}

pub const fn calls_action<A: BridgeAction>(code: &'static str) -> Endpoint {
    Endpoint::new(Kind::CallAction, code, Iface::Action(ActType::of::<A>()))
}

/// One node of a module: what mower_rsd's module table calls an instance.
#[derive(Debug)]
pub struct Instance {
    /// The production node name, as in mower_rsd's module table.
    pub node: &'static str,
    pub endpoints: &'static [Endpoint],
}

/// A mower_rsd module: one or more instances that move together.
#[derive(Debug)]
pub struct Module {
    /// mower_rsd's `--modules` id.
    pub id: &'static str,
    pub instances: &'static [Instance],
}

/// A node outside the typed modules whose endpoints a module depends on.
#[derive(Debug)]
pub struct Outside {
    pub node: &'static str,
    /// The step that removes it (docs/BUS_TOPICS.md 5.3), or why it is there.
    pub until: &'static str,
    pub endpoints: &'static [Endpoint],
}
