//! ROS name resolution for bus endpoints (step E2 of docs/ROS_FREE_PLAN.md).
//!
//! The bus takes absolute names only (mower_bus README, "Names"), but the
//! modules name their topics and services the ROS way, relative
//! (`imu/data_raw`), private (`~/status`) or absolute, and the launch files
//! rename them per node with `-r <node>:<from>:=<to>` (docs/BUS_TOPICS.md
//! 5.4). [`Remaps`] does with the process's `--ros-args` what rcl's
//! `rcl_resolve_name` does with them for the module's r2r node:
//!
//! 1. the node's name and namespace after the `__node:=` / `__ns:=` rules;
//! 2. the name expanded against them (`~`, `{node}`, `{ns}`, relative);
//! 3. the first `-r` rule, in argv order, whose node prefix (if any) is the
//!    node's name, whose kind fits (`rostopic://` / `rosservice://`) and whose
//!    expanded `from` equals the expanded name; its `to`, expanded the same
//!    way, is the result.

use std::fmt;

/// A name that rcl would refuse, or an argv rule it could not parse.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NameError(String);

impl fmt::Display for NameError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for NameError {}

/// Topic and service names live in separate namespaces of rules: a
/// `rostopic://` rule never renames a service, and the other way round.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NameKind {
    Topic,
    Service,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RuleKind {
    TopicOrService,
    Topic,
    Service,
    NodeName,
    Namespace,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct Rule {
    /// Only for the node of this name; every node when `None`.
    node: Option<String>,
    kind: RuleKind,
    from: String,
    to: String,
}

/// The remap rules of a process's `--ros-args`, in argv order.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Remaps {
    rules: Vec<Rule>,
}

impl Remaps {
    /// The `-r` / `--remap` rules of every `--ros-args` section of `argv`
    /// (without the program name). A section ends at `--` or at the end.
    pub fn from_args(argv: &[String]) -> Result<Remaps, NameError> {
        let mut rules = Vec::new();
        let mut in_ros = false;
        let mut it = argv.iter();
        while let Some(arg) = it.next() {
            if !in_ros {
                in_ros = arg == "--ros-args";
                continue;
            }
            match arg.as_str() {
                "--" => in_ros = false,
                "-r" | "--remap" => {
                    let rule = it
                        .next()
                        .ok_or_else(|| NameError(format!("{arg} needs a rule")))?;
                    rules.push(parse_rule(rule)?);
                }
                // The other ROS arguments that take a value; their values
                // must not be read as flags.
                "-p" | "--param" | "--params-file" | "-e" | "--enclave" | "--log-level"
                | "--log-file-name" | "--log-config-file" => {
                    it.next();
                }
                // Flags (`--disable-rosout-logs`, ...), and whatever rcl
                // refuses on its own when it reads the same argv.
                _ => {}
            }
        }
        Ok(Remaps { rules })
    }

    /// `node`'s name after the `__node:=` rules (rcl_remap_node_name).
    pub fn node_name(&self, node: &str) -> String {
        self.first(RuleKind::NodeName, node)
            .map_or_else(|| node.to_string(), |r| r.to.clone())
    }

    /// The namespace of the node named `node` (the name after
    /// [`Remaps::node_name`]) after the
    /// `__ns:=` rules, normalised to an absolute namespace (`/` for none).
    pub fn namespace(&self, node: &str, namespace: &str) -> String {
        let ns = self
            .first(RuleKind::Namespace, node)
            .map_or(namespace, |r| r.to.as_str());
        normalise_namespace(ns)
    }

    /// The absolute name rcl gives `name` on the node created as `node` in
    /// `namespace` ("" for the root namespace).
    pub fn resolve(
        &self,
        kind: NameKind,
        node: &str,
        namespace: &str,
        name: &str,
    ) -> Result<String, NameError> {
        let node = self.node_name(node);
        let ns = self.namespace(&node, namespace);
        let expanded = expand(name, &node, &ns)?;
        for rule in &self.rules {
            let fits = match rule.kind {
                RuleKind::TopicOrService => true,
                RuleKind::Topic => kind == NameKind::Topic,
                RuleKind::Service => kind == NameKind::Service,
                RuleKind::NodeName | RuleKind::Namespace => false,
            };
            if !fits || rule.node.as_deref().is_some_and(|n| n != node) {
                continue;
            }
            // `from` was checked when the rule was parsed, so it expands.
            if expand(&rule.from, &node, &ns).is_ok_and(|from| from == expanded) {
                return expand(&rule.to, &node, &ns);
            }
        }
        Ok(expanded)
    }

    fn first(&self, kind: RuleKind, node: &str) -> Option<&Rule> {
        self.rules
            .iter()
            .find(|r| r.kind == kind && r.node.as_deref().is_none_or(|n| n == node))
    }
}

/// `[node:][rostopic://|rosservice://]from:=to`, or `[node:]__node:=name` /
/// `[node:]__ns:=namespace`.
fn parse_rule(rule: &str) -> Result<Rule, NameError> {
    let bad = |why: &str| NameError(format!("remap rule {rule:?}: {why}"));
    let (lhs, to) = rule.split_once(":=").ok_or_else(|| bad("no ':='"))?;
    // The node prefix ends at the first ':' that does not start a "://".
    let (node, from) = match lhs.find(':') {
        Some(i) if !lhs[i..].starts_with("://") => (Some(&lhs[..i]), &lhs[i + 1..]),
        _ => (None, lhs),
    };
    if let Some(node) = node {
        check_token(node).map_err(|e| bad(&format!("node name: {e}")))?;
    }
    let (kind, from) = match from {
        "__node" | "__name" => (RuleKind::NodeName, from),
        "__ns" => (RuleKind::Namespace, from),
        _ => match (
            from.strip_prefix("rostopic://"),
            from.strip_prefix("rosservice://"),
        ) {
            (Some(f), _) => (RuleKind::Topic, f),
            (_, Some(f)) => (RuleKind::Service, f),
            _ => (RuleKind::TopicOrService, from),
        },
    };
    match kind {
        RuleKind::NodeName => check_token(to).map_err(|e| bad(&format!("node name: {e}")))?,
        RuleKind::Namespace => check_namespace(&normalise_namespace(to)).map_err(|e| bad(&e))?,
        _ => {
            // Expanded against a stand-in node, a valid rule side is a valid
            // name; what it expands to on the real node is decided later.
            expand(from, "n", "/").map_err(|e| bad(&e.0))?;
            expand(to, "n", "/").map_err(|e| bad(&e.0))?;
        }
    }
    Ok(Rule {
        node: node.map(str::to_string),
        kind,
        from: from.to_string(),
        to: to.to_string(),
    })
}

/// rcl_expand_topic_name: substitutions, `~`, then the namespace for a
/// relative name. `ns` is normalised (`/` or `/a/b`).
fn expand(name: &str, node: &str, ns: &str) -> Result<String, NameError> {
    if name.is_empty() {
        return Err(NameError("empty name".into()));
    }
    let subst = name
        .replace("{node}", node)
        .replace("{namespace}", ns)
        .replace("{ns}", ns);
    let fqn = if ns == "/" {
        format!("/{node}")
    } else {
        format!("{ns}/{node}")
    };
    let full = if subst == "~" {
        fqn
    } else if let Some(rest) = subst.strip_prefix("~/") {
        format!("{fqn}/{rest}")
    } else if subst.starts_with('/') {
        subst.clone()
    } else if ns == "/" {
        format!("/{subst}")
    } else {
        format!("{ns}/{subst}")
    };
    check_name(&full).map_err(|e| NameError(format!("{name:?}: {e}")))?;
    Ok(full)
}

fn normalise_namespace(ns: &str) -> String {
    let trimmed = ns.trim_end_matches('/');
    if trimmed.is_empty() {
        "/".to_string()
    } else if trimmed.starts_with('/') {
        trimmed.to_string()
    } else {
        format!("/{trimmed}")
    }
}

/// rmw_validate_full_topic_name: `/` followed by tokens joined by `/`.
fn check_name(name: &str) -> Result<(), String> {
    let rest = name
        .strip_prefix('/')
        .ok_or_else(|| format!("{name:?} is not absolute"))?;
    rest.split('/')
        .try_for_each(check_token)
        .map_err(|e| format!("{name:?}: {e}"))
}

/// rmw_validate_namespace: `/`, or a full name.
fn check_namespace(ns: &str) -> Result<(), String> {
    if ns == "/" {
        Ok(())
    } else {
        check_name(ns)
    }
}

/// A name token: `[A-Za-z_][A-Za-z0-9_]*`.
fn check_token(token: &str) -> Result<(), String> {
    let mut chars = token.chars();
    match chars.next() {
        None => Err("empty token".into()),
        Some(c) if !(c.is_ascii_alphabetic() || c == '_') => {
            Err(format!("token {token:?} starts with {c:?}"))
        }
        _ if !chars.all(|c| c.is_ascii_alphanumeric() || c == '_') => {
            Err(format!("bad character in {token:?}"))
        }
        _ => Ok(()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(s: &str) -> Vec<String> {
        s.split_whitespace().map(str::to_string).collect()
    }

    /// The `-r` rules robot.launch.py passes to mower_rsd in production
    /// (docs/BUS_TOPICS.md 5.4), with a `--params-file` in between.
    fn production() -> Remaps {
        Remaps::from_args(&args(
            "--modules imu,guards,localize,gps --ros-args --params-file /x/mower_rsd.yaml \
             -r imu:imu/data_raw:=imu/data \
             -r manual_velocity_guard:cmd_vel_in:=/app_joy_cmd \
             -r manual_velocity_guard:cmd_vel_out:=/joy_cmd \
             -r manual_velocity_guard:command_clock:=/manual_command_clock \
             -r velocity_command_guard:cmd_vel_in:=/cmd_vel_guard_input \
             -r velocity_command_guard:cmd_vel_out:=/drivetrain_guarded_cmd_vel \
             -r ekf_filter_node_odom:odometry/filtered:=odometry/local \
             -r ekf_filter_node_odom:imu:=imu/data \
             -r ekf_filter_node_map:odometry/filtered:=odometry/global \
             -r ekf_filter_node_map:imu:=imu/data \
             -r navsat_transform:imu:=imu/data \
             -r navsat_transform:odometry/filtered:=odometry/global \
             -r navsat_transform:gps/fix:=/fix",
        ))
        .unwrap()
    }

    #[test]
    fn production_names_resolve_as_bus_topics_5_4_lists_them() {
        let r = production();
        let t = |node: &str, name: &str| r.resolve(NameKind::Topic, node, "", name).unwrap();
        assert_eq!(t("imu", "imu/data_raw"), "/imu/data");
        assert_eq!(t("manual_velocity_guard", "cmd_vel_in"), "/app_joy_cmd");
        assert_eq!(t("manual_velocity_guard", "cmd_vel_out"), "/joy_cmd");
        assert_eq!(
            t("manual_velocity_guard", "command_clock"),
            "/manual_command_clock"
        );
        assert_eq!(
            t("velocity_command_guard", "cmd_vel_in"),
            "/cmd_vel_guard_input"
        );
        assert_eq!(
            t("velocity_command_guard", "cmd_vel_out"),
            "/drivetrain_guarded_cmd_vel"
        );
        // The EKFs: parameter value, then the node's rule, then the root.
        assert_eq!(
            t("ekf_filter_node_odom", "odometry/filtered"),
            "/odometry/local"
        );
        assert_eq!(t("ekf_filter_node_odom", "odom"), "/odom");
        assert_eq!(t("ekf_filter_node_odom", "imu"), "/imu/data");
        assert_eq!(
            t("ekf_filter_node_map", "odometry/filtered"),
            "/odometry/global"
        );
        assert_eq!(t("ekf_filter_node_map", "odometry/gps"), "/odometry/gps");
        assert_eq!(t("ekf_filter_node_map", "imu"), "/imu/data");
        assert_eq!(
            t("ekf_filter_node_map", "/odometry/global_slow"),
            "/odometry/global_slow"
        );
        assert_eq!(
            t("navsat_transform", "odometry/filtered"),
            "/odometry/global"
        );
        assert_eq!(t("navsat_transform", "gps/fix"), "/fix");
        assert_eq!(t("navsat_transform", "imu"), "/imu/data");
        assert_eq!(t("navsat_transform", "odometry/gps"), "/odometry/gps");
        assert_eq!(t("navsat_transform", "gps/filtered"), "/gps/filtered");
        assert_eq!(t("gps", "gps/status"), "/gps/status");
        // nav and coverage: relative names, no rule.
        let s = |node: &str, name: &str| r.resolve(NameKind::Service, node, "", name).unwrap();
        assert_eq!(
            s("nav_action_server", "bt_navigator/get_state"),
            "/bt_navigator/get_state"
        );
        assert_eq!(
            t("nav_action_server", "nav_action_follow_path"),
            "/nav_action_follow_path"
        );
        assert_eq!(
            t("boustrophedon_coverage", "nav_action_follow_path"),
            "/nav_action_follow_path"
        );
    }

    #[test]
    fn a_node_prefix_limits_a_rule_to_that_node() {
        let r = production();
        // The IMU's rule is not the localize nodes' rule, and the other way.
        assert_eq!(
            r.resolve(NameKind::Topic, "gps", "", "imu/data_raw")
                .unwrap(),
            "/imu/data_raw"
        );
        assert_eq!(
            r.resolve(NameKind::Topic, "imu", "", "imu").unwrap(),
            "/imu"
        );
        assert_eq!(
            r.resolve(NameKind::Topic, "battery_state", "", "/battery_state")
                .unwrap(),
            "/battery_state"
        );
    }

    #[test]
    fn rules_match_on_expanded_names() {
        let r = Remaps::from_args(&args("--ros-args -r /a/b:=c -r x:=/abs/x -r n:~/p:=q")).unwrap();
        // A relative input equal to an absolute `from` after expansion.
        assert_eq!(r.resolve(NameKind::Topic, "n", "/a", "b").unwrap(), "/a/c");
        assert_eq!(r.resolve(NameKind::Topic, "n", "", "/a/b").unwrap(), "/c");
        assert_eq!(r.resolve(NameKind::Topic, "n", "", "x").unwrap(), "/abs/x");
        assert_eq!(
            r.resolve(NameKind::Topic, "n", "/ns", "x").unwrap(),
            "/abs/x",
            "`from` x is /ns/x there too"
        );
        assert_eq!(r.resolve(NameKind::Topic, "n", "", "~/p").unwrap(), "/q");
        assert_eq!(
            r.resolve(NameKind::Topic, "n", "", "/n/p").unwrap(),
            "/q",
            "~/p of n is /n/p"
        );
    }

    #[test]
    fn the_first_matching_rule_wins() {
        let r =
            Remaps::from_args(&args("--ros-args -r n:a:=first -r a:=second -r a:=third")).unwrap();
        assert_eq!(r.resolve(NameKind::Topic, "n", "", "a").unwrap(), "/first");
        assert_eq!(r.resolve(NameKind::Topic, "m", "", "a").unwrap(), "/second");
    }

    #[test]
    fn rule_kinds_keep_topics_and_services_apart() {
        let r = Remaps::from_args(&args(
            "--ros-args -r rostopic://t:=t2 -r rosservice://s:=s2 -r n:rostopic://u:=u2",
        ))
        .unwrap();
        assert_eq!(r.resolve(NameKind::Topic, "n", "", "t").unwrap(), "/t2");
        assert_eq!(r.resolve(NameKind::Service, "n", "", "t").unwrap(), "/t");
        assert_eq!(r.resolve(NameKind::Service, "n", "", "s").unwrap(), "/s2");
        assert_eq!(r.resolve(NameKind::Topic, "n", "", "s").unwrap(), "/s");
        assert_eq!(r.resolve(NameKind::Topic, "n", "", "u").unwrap(), "/u2");
        assert_eq!(r.resolve(NameKind::Topic, "m", "", "u").unwrap(), "/u");
    }

    #[test]
    fn private_names_and_substitutions_use_the_renamed_node() {
        let r = Remaps::from_args(&args(
            "--ros-args -r __node:=renamed -r renamed:__ns:=/space",
        ))
        .unwrap();
        assert_eq!(r.node_name("original"), "renamed");
        assert_eq!(r.namespace("renamed", ""), "/space");
        assert_eq!(
            r.resolve(NameKind::Topic, "original", "", "~/status")
                .unwrap(),
            "/space/renamed/status"
        );
        assert_eq!(
            r.resolve(NameKind::Topic, "original", "", "~").unwrap(),
            "/space/renamed"
        );
        assert_eq!(
            r.resolve(NameKind::Topic, "original", "", "rel").unwrap(),
            "/space/rel"
        );
        assert_eq!(
            r.resolve(NameKind::Topic, "original", "", "{node}/x")
                .unwrap(),
            "/space/renamed/x"
        );
        // A prefixed __node rule renames only its node.
        let r = Remaps::from_args(&args("--ros-args -r a:__node:=b")).unwrap();
        assert_eq!(r.node_name("a"), "b");
        assert_eq!(r.node_name("c"), "c");
    }

    #[test]
    fn only_ros_args_sections_count() {
        let r = Remaps::from_args(&args(
            "-r not:=this --ros-args -p n:x:=1 --params-file -r -r a:=b -- -r c:=d --ros-args --remap e:=f \
             --disable-rosout-logs",
        ))
        .unwrap();
        assert_eq!(r.resolve(NameKind::Topic, "n", "", "not").unwrap(), "/not");
        // `-r` right after --params-file is that flag's value, not a rule.
        assert_eq!(r.resolve(NameKind::Topic, "n", "", "a").unwrap(), "/b");
        assert_eq!(r.resolve(NameKind::Topic, "n", "", "c").unwrap(), "/c");
        assert_eq!(r.resolve(NameKind::Topic, "n", "", "e").unwrap(), "/f");
    }

    #[test]
    fn bad_names_and_rules_are_refused() {
        let r = Remaps::default();
        for name in ["", "/", "a//b", "a/", "1a", "a b", "~x", "a/{nope}", "a-b"] {
            assert!(
                r.resolve(NameKind::Topic, "n", "", name).is_err(),
                "{name:?}"
            );
        }
        for rule in [
            "a=b",
            "a:=",
            ":=b",
            "1n:a:=b",
            "a:=b-c",
            "n:__node:=1x",
            "__ns:=/1x",
        ] {
            assert!(
                Remaps::from_args(&args(&format!("--ros-args -r {rule}"))).is_err(),
                "{rule:?}"
            );
        }
        assert!(Remaps::from_args(&args("--ros-args -r")).is_err());
    }
}
