//! The handles twist_mux 4.5.0 builds from its parameters, in its order and
//! with its log lines, from the node's parameter overrides as plain values
//! (the module converts r2r's; tests write them by hand).
//!
//! - **Order.** `getTopicHandles("topics")` lists the parameters under
//!   `topics.`, which rclcpp keeps sorted by full name, and takes each
//!   name's prefix up to its last `.` the first time it comes. The handle is
//!   that prefix, `topics.app_manual`: the byte order of the names, not the
//!   order of the yaml. Then the same for `locks.`.
//! - **Each handle** needs `<prefix>.topic` (a string), `.timeout` (a
//!   double: twist_mux throws on an integer) and `.priority` (an integer,
//!   clamped to 0..=255). A missing one, or one of another type, is an error:
//!   twist_mux logs it FATAL and exits; the module returns it, and mower_rsd
//!   starts it again 2 s later.
//! - **`use_stamped`** defaults to true when it is not set (with twist_mux's
//!   INFO line). false would make twist_mux take `geometry_msgs/Twist`, which
//!   nothing on the robot publishes: refused.

use std::collections::BTreeMap;
use std::fmt;

use crate::core::Handle;

/// A parameter's value as rcl has it.
#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    Bool(bool),
    Integer(i64),
    Double(f64),
    String(String),
    /// An array, or anything else twist_mux's handles never take.
    Other(String),
}

impl Value {
    fn kind(&self) -> &'static str {
        match self {
            Value::Bool(_) => "bool",
            Value::Integer(_) => "integer",
            Value::Double(_) => "double",
            Value::String(_) => "string",
            Value::Other(_) => "other",
        }
    }
}

/// A log line twist_mux writes while it reads its configuration.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Log {
    Info(String),
    Warn(String),
}

#[derive(Clone, Debug, PartialEq)]
pub struct Config {
    pub inputs: Vec<Handle>,
    pub locks: Vec<Handle>,
    /// What twist_mux logs on the way, in its order.
    pub logs: Vec<Log>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ConfigError(pub String);

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for ConfigError {}

/// twist_mux's handles from the node's parameters (`name -> value`, the
/// nested yaml keys joined with dots, as rcl names them).
pub fn from_params(params: &BTreeMap<String, Value>) -> Result<Config, ConfigError> {
    let mut logs = Vec::new();
    match params.get("use_stamped") {
        None => logs.push(Log::Info("\"use_stamped\" is not declared as parameter, defaulting to \"true\".".into())),
        Some(Value::Bool(true)) => {}
        Some(Value::Bool(false)) => {
            return Err(ConfigError(
                "use_stamped is false: twist_mux would then mux geometry_msgs/Twist, which nothing on the robot \
                 publishes; this mux takes TwistStamped only"
                    .into(),
            ))
        }
        Some(other) => return Err(ConfigError(format!("use_stamped must be a bool, not {} {other:?}", other.kind()))),
    }
    let inputs = handles(params, "topics", &mut logs)?;
    let locks = handles(params, "locks", &mut logs)?;
    Ok(Config { inputs, locks, logs })
}

/// rclcpp's `list_parameters({kind}, 10)` prefixes, then each handle.
fn handles(params: &BTreeMap<String, Value>, kind: &str, logs: &mut Vec<Log>) -> Result<Vec<Handle>, ConfigError> {
    let dotted = format!("{kind}.");
    let mut prefixes: Vec<&str> = Vec::new();
    // BTreeMap<String, _> iterates in byte order, as rclcpp's std::map does
    for name in params.keys() {
        let Some(rest) = name.strip_prefix(&dotted) else { continue };
        // depth 10: fewer than 10 separators below the prefix
        if rest.matches('.').count() + 1 >= 10 {
            continue;
        }
        if let Some(cut) = name.rfind('.') {
            let prefix = &name[..cut];
            if !prefixes.contains(&prefix) {
                prefixes.push(prefix);
            }
        }
    }
    let mut out = Vec::new();
    for prefix in prefixes {
        let field = |key: &str| params.get(&format!("{prefix}.{key}"));
        let missing = |key: &str| ConfigError(format!("Error parsing params '{kind}':\n\t{prefix}.{key} is not set"));
        let wrong = |key: &str, want: &str, got: &Value| {
            ConfigError(format!("Error parsing params '{kind}':\n\t{prefix}.{key} must be a {want}, not {} {got:?}", got.kind()))
        };
        let topic = match field("topic") {
            Some(Value::String(s)) => s.clone(),
            Some(v) => return Err(wrong("topic", "string", v)),
            None => return Err(missing("topic")),
        };
        let timeout = match field("timeout") {
            Some(Value::Double(d)) => *d,
            Some(v) => return Err(wrong("timeout", "double", v)),
            None => return Err(missing("timeout")),
        };
        let priority = match field("priority") {
            Some(Value::Integer(i)) => *i,
            Some(v) => return Err(wrong("priority", "integer", v)),
            None => return Err(missing("priority")),
        };
        let handle = Handle::new(prefix, &topic, timeout, priority);
        logs.push(Log::Info(format!(
            "Topic handler '{}' subscribed to topic '{}': timeout = {} , priority = {}.",
            handle.name,
            handle.topic,
            if handle.timeout_s() > 0.0 { format!("{:.6}s", handle.timeout_s()) } else { "None".into() },
            handle.priority
        )));
        if handle.priority == 0 {
            logs.push(Log::Warn(if kind == "locks" {
                format!("Lock '{}' with priority 0 <= 0 is redundant. Priority 0 is always locked.", handle.name)
            } else {
                format!(
                    "Topic handler '{}' with priority 0 <= 0 will never be selected. Increase the priority to at least 1.",
                    handle.name
                )
            }));
        }
        out.push(handle);
    }
    Ok(out)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    fn params(pairs: &[(&str, Value)]) -> BTreeMap<String, Value> {
        pairs.iter().map(|(k, v)| (k.to_string(), v.clone())).collect()
    }

    fn handle(prefix: &str, topic: &str, timeout: f64, priority: i64) -> Vec<(String, Value)> {
        vec![
            (format!("{prefix}.topic"), Value::String(topic.into())),
            (format!("{prefix}.timeout"), Value::Double(timeout)),
            (format!("{prefix}.priority"), Value::Integer(priority)),
        ]
    }

    /// twist_mux_topics.yaml as rcl reads it.
    pub(crate) fn production_params() -> BTreeMap<String, Value> {
        let mut p: Vec<(String, Value)> = vec![("use_stamped".into(), Value::Bool(true))];
        p.extend(handle("topics.navigation_safety_stop", "/navigation_safety_stop", 0.2, 50));
        p.extend(handle("topics.navigation", "/nav_cmd_vel", 0.5, 10));
        p.extend(handle("topics.app_manual", "/joy_cmd", 0.5, 90));
        p.extend(handle("topics.physical_joystick", "/physical_joy_cmd", 0.5, 100));
        p.extend(handle("topics.keyboard", "/keyboard_cmd_vel", 0.5, 80));
        p.extend(handle("locks.navigation_coordinator", "/navigation_coordinator_lock", 0.3, 50));
        p.into_iter().collect()
    }

    /// The order and the INFO lines of twist_mux's log for the production
    /// file (e4-design evidence out/twist_mux.log).
    #[test]
    fn the_production_handles_come_in_twist_mux_s_order_with_its_log() {
        let c = from_params(&production_params()).unwrap();
        let names: Vec<&str> = c.inputs.iter().map(|h| h.name.as_str()).collect();
        assert_eq!(
            names,
            ["topics.app_manual", "topics.keyboard", "topics.navigation", "topics.navigation_safety_stop",
             "topics.physical_joystick"]
        );
        assert_eq!(c.locks.iter().map(|h| h.name.as_str()).collect::<Vec<_>>(), ["locks.navigation_coordinator"]);
        let log: Vec<Log> = vec![
            Log::Info("Topic handler 'topics.app_manual' subscribed to topic '/joy_cmd': timeout = 0.500000s , priority = 90.".into()),
            Log::Info("Topic handler 'topics.keyboard' subscribed to topic '/keyboard_cmd_vel': timeout = 0.500000s , priority = 80.".into()),
            Log::Info("Topic handler 'topics.navigation' subscribed to topic '/nav_cmd_vel': timeout = 0.500000s , priority = 10.".into()),
            Log::Info("Topic handler 'topics.navigation_safety_stop' subscribed to topic '/navigation_safety_stop': timeout = 0.200000s , priority = 50.".into()),
            Log::Info("Topic handler 'topics.physical_joystick' subscribed to topic '/physical_joy_cmd': timeout = 0.500000s , priority = 100.".into()),
            Log::Info("Topic handler 'locks.navigation_coordinator' subscribed to topic '/navigation_coordinator_lock': timeout = 0.300000s , priority = 50.".into()),
        ];
        assert_eq!(c.logs, log);
    }

    /// The generic file of tools/mux_oracle.py, against the C++ log of the
    /// same file: sorted by name, None for timeout 0, the two priority-0
    /// warnings, and use_stamped's default line when it is not set.
    #[test]
    fn ties_timeout_zero_and_priority_zero_log_as_twist_mux_does() {
        let mut p: Vec<(String, Value)> = Vec::new();
        p.extend(handle("topics.bbb", "/g_bbb", 0.5, 50));
        p.extend(handle("topics.aaa", "/g_aaa", 0.5, 50));
        p.extend(handle("topics.zero", "/g_zero", 0.5, 0));
        p.extend(handle("topics.never", "/g_never", 0.0, 40));
        p.extend(handle("topics.low", "/g_low", 0.5, 30));
        p.extend(handle("locks.quiet", "/g_quiet", 0.0, 60));
        p.extend(handle("locks.zerolock", "/g_zerolock", 0.5, 0));
        p.extend(handle("locks.high", "/g_high", 0.3, 45));
        let c = from_params(&p.into_iter().collect()).unwrap();
        let lines: Vec<String> = c.logs.iter().map(|l| format!("{l:?}")).collect();
        assert_eq!(
            lines,
            [
                r#"Info("\"use_stamped\" is not declared as parameter, defaulting to \"true\".")"#,
                r#"Info("Topic handler 'topics.aaa' subscribed to topic '/g_aaa': timeout = 0.500000s , priority = 50.")"#,
                r#"Info("Topic handler 'topics.bbb' subscribed to topic '/g_bbb': timeout = 0.500000s , priority = 50.")"#,
                r#"Info("Topic handler 'topics.low' subscribed to topic '/g_low': timeout = 0.500000s , priority = 30.")"#,
                r#"Info("Topic handler 'topics.never' subscribed to topic '/g_never': timeout = None , priority = 40.")"#,
                r#"Info("Topic handler 'topics.zero' subscribed to topic '/g_zero': timeout = 0.500000s , priority = 0.")"#,
                r#"Warn("Topic handler 'topics.zero' with priority 0 <= 0 will never be selected. Increase the priority to at least 1.")"#,
                r#"Info("Topic handler 'locks.high' subscribed to topic '/g_high': timeout = 0.300000s , priority = 45.")"#,
                r#"Info("Topic handler 'locks.quiet' subscribed to topic '/g_quiet': timeout = None , priority = 60.")"#,
                r#"Info("Topic handler 'locks.zerolock' subscribed to topic '/g_zerolock': timeout = 0.500000s , priority = 0.")"#,
                r#"Warn("Lock 'locks.zerolock' with priority 0 <= 0 is redundant. Priority 0 is always locked.")"#,
            ]
        );
    }

    /// rclcpp sorts full names, not prefixes: `a-b.x` comes before `a.x`
    /// ('-' < '.'), though the prefix `a` sorts before `a-b`.
    #[test]
    fn prefixes_come_in_the_order_of_the_full_names() {
        let mut p: Vec<(String, Value)> = vec![("use_stamped".into(), Value::Bool(true))];
        p.extend(handle("topics.a", "/a", 0.5, 1));
        p.extend(handle("topics.a-b", "/ab", 0.5, 1));
        let c = from_params(&p.into_iter().collect()).unwrap();
        assert_eq!(c.inputs.iter().map(|h| h.name.as_str()).collect::<Vec<_>>(), ["topics.a-b", "topics.a"]);
    }

    #[test]
    fn a_missing_or_mistyped_field_is_an_error() {
        let base = production_params();
        let mut p = base.clone();
        p.remove("topics.navigation.priority");
        assert!(from_params(&p).unwrap_err().0.contains("topics.navigation.priority is not set"));
        let mut p = base.clone();
        p.insert("topics.navigation.timeout".into(), Value::Integer(1));
        assert!(from_params(&p).unwrap_err().0.contains("must be a double"), "twist_mux throws on an integer timeout");
        let mut p = base.clone();
        p.insert("locks.navigation_coordinator.priority".into(), Value::Double(50.0));
        assert!(from_params(&p).is_err());
        let mut p = base.clone();
        p.insert("topics.navigation.topic".into(), Value::Integer(3));
        assert!(from_params(&p).is_err());
        let mut p = base.clone();
        p.insert("use_stamped".into(), Value::Bool(false));
        assert!(from_params(&p).unwrap_err().0.contains("use_stamped is false"));
        let mut p = base;
        p.insert("use_stamped".into(), Value::String("true".into()));
        assert!(from_params(&p).is_err());
        // no handles at all is what twist_mux accepts too
        let empty = from_params(&params(&[])).unwrap();
        assert!(empty.inputs.is_empty() && empty.locks.is_empty());
    }

    /// Unrelated parameters of the node are not handles, and a priority out
    /// of range is clamped as twist_mux clamps it.
    #[test]
    fn other_parameters_are_ignored_and_priorities_clamped() {
        let mut p = production_params();
        p.insert("diagnostic_updater.period".into(), Value::Double(1.0));
        p.insert("qos_overrides./parameter_events.publisher.depth".into(), Value::Integer(1000));
        p.insert("topicsx.a.topic".into(), Value::String("/x".into()));
        p.insert("topics.physical_joystick.priority".into(), Value::Integer(300));
        let c = from_params(&p).unwrap();
        assert_eq!(c.inputs.len(), 5);
        assert_eq!(c.inputs[4].priority, 255);
    }
}
