//! What `Bus::topics` / `Bus::services` report: what `/rosapi/topics` (E3)
//! and `mowerctl topic list/hz` (E8) are built from.

use std::time::Instant;

use crate::Qos;

/// One topic, as the bus sees it now.
#[derive(Clone, Debug, PartialEq)]
pub struct TopicInfo {
    pub name: String,
    /// The Rust type, once a typed endpoint has fixed it.
    pub type_name: Option<&'static str>,
    /// [`ros_type_name`] of `type_name`, when that is an r2r message type.
    pub ros_type: Option<String>,
    /// Scope names, one entry per live writer.
    pub publishers: Vec<String>,
    /// Each live writer's QoS, in the order of `publishers`.
    pub publisher_qos: Vec<Qos>,
    /// Scope names, one entry per live reader (typed and erased).
    pub subscribers: Vec<String>,
    /// Each live reader's QoS, in the order of `subscribers`.
    pub subscriber_qos: Vec<Qos>,
    /// Messages held for late joiners, over all writers.
    pub retained: usize,
    /// Messages published on this name since the bus started.
    pub published: u64,
    /// When the last one was: a std `Instant`, not a reading of the bus
    /// clock (a manual clock does not move it).
    pub last_published: Option<Instant>,
}

/// One service name, as the bus sees it now.
#[derive(Clone, Debug, PartialEq)]
pub struct ServiceInfo {
    pub name: String,
    pub request_type: &'static str,
    pub response_type: &'static str,
    /// e.g. `std_srvs/srv/Trigger` for r2r's `std_srvs::srv::Trigger::Request`.
    pub ros_type: Option<String>,
    /// The serving scope's name, if a server is live.
    pub server: Option<String>,
    /// Scope names, one entry per live client.
    pub clients: Vec<String>,
    /// Calls that reached a live server since the bus started, answered or
    /// not. Calls refused at call time (no server, closed scope, bad JSON)
    /// are not counted.
    pub calls: u64,
}

/// The ROS interface name of an r2r-generated type, from its Rust path
/// (`std::any::type_name`):
///
/// - `…::<pkg>::msg::<Name>` -> `<pkg>/msg/<Name>`
/// - `…::<pkg>::srv::<Name>::{Request,Response}` -> `<pkg>/srv/<Name>`
/// - `…::<pkg>::action::<Name>::<Part>` -> `<pkg>/action/<Name>`
///
/// r2r defines its messages in `r2r::msg_types::generated_msgs::…` and
/// re-exports them, so only the tail of the path is looked at. Anything else
/// (a Rust type of our own, a generic) gives `None`.
pub fn ros_type_name(rust_type: &str) -> Option<String> {
    if rust_type.contains('<') {
        return None;
    }
    let parts: Vec<&str> = rust_type.split("::").collect();
    let n = parts.len();
    let ident = |s: &str| !s.is_empty() && s.chars().all(|c| c.is_ascii_alphanumeric() || c == '_');
    if n >= 3 && parts[n - 2] == "msg" && ident(parts[n - 3]) && ident(parts[n - 1]) {
        return Some(format!("{}/msg/{}", parts[n - 3], parts[n - 1]));
    }
    if n >= 4 && ident(parts[n - 4]) && ident(parts[n - 2]) {
        match (parts[n - 3], parts[n - 1]) {
            ("srv", "Request" | "Response") => return Some(format!("{}/srv/{}", parts[n - 4], parts[n - 2])),
            ("action", _) => return Some(format!("{}/action/{}", parts[n - 4], parts[n - 2])),
            _ => {}
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn r2r_paths_map_to_ros_interface_names() {
        assert_eq!(ros_type_name("r2r::msg_types::generated_msgs::nav_msgs::msg::Odometry").as_deref(), Some("nav_msgs/msg/Odometry"));
        assert_eq!(ros_type_name("r2r::nav_msgs::msg::Odometry").as_deref(), Some("nav_msgs/msg/Odometry"));
        assert_eq!(ros_type_name("r2r::std_srvs::srv::Trigger::Request").as_deref(), Some("std_srvs/srv/Trigger"));
        assert_eq!(ros_type_name("r2r::std_srvs::srv::Trigger::Response").as_deref(), Some("std_srvs/srv/Trigger"));
        assert_eq!(ros_type_name("r2r::mower_interface::action::Waypoint::Goal").as_deref(), Some("mower_interface/action/Waypoint"));
        assert_eq!(ros_type_name("mower_bus::tests::Pose"), None);
        assert_eq!(ros_type_name("alloc::vec::Vec<f64>"), None);
        assert_eq!(ros_type_name("f64"), None);
    }
}
