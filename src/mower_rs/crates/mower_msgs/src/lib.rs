//! The ROS 2 interface types the crates use, as plain Rust: no r2r, no rcl, no
//! ROS at build or run time. `mower_schema/gen/gen_types.py` writes
//! `src/generated/` from mower_schema's committed `.msg`, `.srv` and `.action`
//! files; nothing in production uses them yet (E9-22 step A).
//!
//! - **The layout is r2r's:** `mower_msgs::std_msgs::msg::String`,
//!   `mower_msgs::std_srvs::srv::Trigger::{Service, Request, Response}`,
//!   `mower_msgs::mower_interface::action::Waypoint::{Action, Goal, Result,
//!   Feedback, FeedbackMessage, SendGoal::{..}, GetResult::{..}}`. So changing an import
//!   is all a user of the fields needs, and `mower_bus::ros_type_name` reads
//!   `std::any::type_name` of these as it reads r2r's.
//! - **So is the behaviour of a value:** the same derives, `#[serde(default)]`, a
//!   `Vec` for every array, fixed ones too (holding their length is the
//!   converter's job, as in r2r), `#[serde(rename = "type")]` on `type_`, an empty
//!   struct for an empty message, and `Default` with the file's defaults and each
//!   fixed array's length. `mower_msgs_ros/tests/parity.rs` holds every type's
//!   JSON, default, CDR and refusals to r2r's while r2r exists, and
//!   `mower_ros_facts/tests/plain_*.rs` to what r2r and rmw wrote, with no ROS.
//! - **One difference:** a constant has its field's type (`u8`, `i32`). r2r's is a
//!   bindgen enum that nothing here can use without a cast, and no crate names one.
//! - **What they do not do:** refuse. As with r2r's, a `Vec` of the wrong length or a
//!   string with a NUL is a value here; only the conversion to C (r2r) and
//!   `mower_cdr` refuse it.
//!
//! The traits replace r2r's `WrappedServiceTypeSupport` and
//! `WrappedActionTypeSupport`, for what a bound needs of them.

use serde::de::DeserializeOwned;
use serde::Serialize;
use std::fmt::Debug;

#[allow(clippy::derivable_impls)]
mod generated;

pub use generated::*;

/// A message type: `NAME` is rosidl's, `pkg/msg/Type`, or for a part of a service
/// or action `pkg/srv/Type_Request`, `pkg/action/Type_SendGoal_Response`, ...
pub trait MessageType: Clone + Debug + Default + PartialEq + Serialize + DeserializeOwned + Send + Sync + 'static {
    const NAME: &'static str;
}

/// A service: the marker `...::srv::<Name>::Service`.
pub trait ServiceType: 'static {
    type Request: MessageType;
    type Response: MessageType;
    /// `pkg/srv/Type`
    const NAME: &'static str;
}

/// An action: the marker `...::action::<Name>::Action`.
pub trait ActionType: 'static {
    type Goal: MessageType;
    type Result: MessageType;
    type Feedback: MessageType;
    type FeedbackMessage: MessageType;
    type SendGoal: ServiceType;
    type GetResult: ServiceType;
    /// `pkg/action/Type`
    const NAME: &'static str;
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn a_default_holds_the_file_s_defaults_and_each_fixed_length() {
        assert_eq!(geometry_msgs::msg::Quaternion::default().w, 1.0);
        assert_eq!(sensor_msgs::msg::NavSatStatus::default().status, -2);
        assert_eq!(mower_interface::srv::ResumeCoverage::Request::default().zone_id, -1);
        assert_eq!(sensor_msgs::msg::NavSatFix::default().position_covariance.len(), 9);
        assert_eq!(nav_msgs::msg::Odometry::default().pose.covariance.len(), 36);
        assert_eq!(unique_identifier_msgs::msg::UUID::default().uuid.len(), 16);
        assert!(geometry_msgs::msg::PoseStamped::default().header.frame_id.is_empty());
    }

    #[test]
    fn a_keyword_field_keeps_its_ros_name_in_json() {
        let m = visualization_msgs::msg::Marker { type_: 7, ..Default::default() };
        let j = serde_json::to_value(&m).unwrap();
        assert_eq!(j["type"], 7);
        assert!(j.get("type_").is_none());
        let back: visualization_msgs::msg::Marker = serde_json::from_value(json!({"type": 3})).unwrap();
        assert_eq!(back.type_, 3);
    }

    #[test]
    fn a_missing_field_is_its_default_and_an_unknown_one_is_ignored() {
        let q: geometry_msgs::msg::Quaternion = serde_json::from_value(json!({"x": 0.5, "nope": 1})).unwrap();
        assert_eq!((q.x, q.w), (0.5, 1.0));
        let e: std_msgs::msg::Empty = serde_json::from_value(json!({"anything": 1})).unwrap();
        assert_eq!(serde_json::to_string(&e).unwrap(), "{}");
    }

    #[test]
    fn constants_have_their_file_s_values() {
        assert_eq!(visualization_msgs::msg::Marker::ADD, 0);
        assert_eq!(sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX, -1);
        assert_eq!(mower_interface::msg::CoverageProgress::STATUS_FAILED, 6);
        assert_eq!(nav2_msgs::action::FollowPath::Result::CONTROLLER_TIMED_OUT, 107);
    }

    #[test]
    fn the_names_are_rosidl_s() {
        fn service<S: ServiceType>() -> (&'static str, &'static str, &'static str) {
            (S::NAME, <S::Request as MessageType>::NAME, <S::Response as MessageType>::NAME)
        }
        assert_eq!(service::<std_srvs::srv::Trigger::Service>(),
            ("std_srvs/srv/Trigger", "std_srvs/srv/Trigger_Request", "std_srvs/srv/Trigger_Response"));
        assert_eq!(service::<mower_interface::action::Waypoint::SendGoal::Service>(), (
            "mower_interface/action/Waypoint_SendGoal",
            "mower_interface/action/Waypoint_SendGoal_Request",
            "mower_interface/action/Waypoint_SendGoal_Response"
        ));
        assert_eq!(<mower_interface::action::Waypoint::Action as ActionType>::NAME, "mower_interface/action/Waypoint");
    }
}
