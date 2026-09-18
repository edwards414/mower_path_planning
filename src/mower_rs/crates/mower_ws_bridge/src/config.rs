//! Bridge policy: which topics a client may subscribe to or publish, which
//! services it may call (with their types, because r2r has no service-type
//! graph query), and the listener addresses.

use serde::Deserialize;
use std::collections::BTreeMap;

#[derive(Debug, Clone, Deserialize)]
pub struct Policy {
    /// Topics clients may subscribe to (fnmatch-style `*` globs, as in
    /// rosbridge's topics_sub_glob). Also the rosapi/topics answer.
    #[serde(default)]
    pub topics_sub: Vec<String>,
    /// Topics clients may advertise / publish.
    #[serde(default)]
    pub topics_pub: Vec<String>,
    /// Services clients may call, with their types.
    #[serde(default)]
    pub services: BTreeMap<String, String>,
}

impl Policy {
    pub fn from_yaml(text: &str) -> Result<Self, String> {
        serde_yaml::from_str(text).map_err(|e| format!("bridge policy: {e}"))
    }

    pub fn may_subscribe(&self, topic: &str) -> bool {
        self.topics_sub.iter().any(|g| glob_match(g, topic))
    }

    pub fn may_publish(&self, topic: &str) -> bool {
        self.topics_pub.iter().any(|g| glob_match(g, topic))
    }

    pub fn service_type(&self, service: &str) -> Option<&str> {
        self.services.get(service).map(String::as_str)
    }
}

/// fnmatch subset: `*` matches any run of characters (including `/`),
/// everything else is literal. rosbridge uses Python's fnmatch, whose `*`
/// also crosses `/`.
pub fn glob_match(pattern: &str, name: &str) -> bool {
    fn rec(p: &[u8], n: &[u8]) -> bool {
        match p.first() {
            None => n.is_empty(),
            Some(b'*') => {
                let rest = &p[1..];
                (0..=n.len()).any(|i| rec(rest, &n[i..]))
            }
            Some(c) => n.first() == Some(c) && rec(&p[1..], &n[1..]),
        }
    }
    rec(pattern.as_bytes(), name.as_bytes())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn globs_follow_fnmatch() {
        assert!(glob_match("/adapter/*", "/adapter/map_layers/map_grid"));
        assert!(glob_match("/*/online", "/robot/online"));
        assert!(glob_match("/*/online", "/mw-2/robot/online"));
        assert!(glob_match("/robot/info", "/robot/info"));
        assert!(!glob_match("/robot/info", "/robot/info2"));
        assert!(!glob_match("/adapter/*", "/adapters/x"));
        assert!(glob_match("*", "/anything"));
    }

    #[test]
    fn policy_parses_and_answers() {
        let p = Policy::from_yaml(
            "topics_sub: ['/adapter/*', '/robot/online']\ntopics_pub: ['/app_joy_cmd']\nservices:\n  /cancel_nav2: std_srvs/srv/Trigger\n",
        )
        .unwrap();
        assert!(p.may_subscribe("/adapter/robot_pose"));
        assert!(!p.may_subscribe("/odom"));
        assert!(p.may_publish("/app_joy_cmd"));
        assert!(!p.may_publish("/cmd_vel"));
        assert_eq!(p.service_type("/cancel_nav2"), Some("std_srvs/srv/Trigger"));
        assert_eq!(p.service_type("/rm_rf"), None);
    }
}
