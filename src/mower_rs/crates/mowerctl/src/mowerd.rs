//! The bridge's own view (E8-2): `/mowerd/{topics,services,actions,modules,
//! bridge}`, which ws_bridge answers on its loopback listener alone, from the
//! bus and the process's registry, with no DDS.
//!
//! Everywhere else they are refused like any service not allowed: the pairing
//! gate (`--pair`), a loopback listener without a loopback policy, the r2r
//! backend, an image older than E8-2. So a command with another source falls
//! back to it ([`read`]); one without says so ([`require`]).

use std::time::Duration;

use serde_json::{json, Value};

use crate::conn::{service_error, Conn};
use crate::format;
use crate::Fail;

pub const TOPICS: &str = "/mowerd/topics";
pub const SERVICES: &str = "/mowerd/services";
pub const ACTIONS: &str = "/mowerd/actions";
pub const MODULES: &str = "/mowerd/modules";
pub const BRIDGE: &str = "/mowerd/bridge";

/// `service`'s answer; `None` when this listener does not answer it.
pub async fn read(c: &mut Conn, service: &str, args: Value, timeout: Duration) -> Result<Option<Value>, Fail> {
    let reply = c.call(service, args, timeout).await?;
    let values = reply.doc.get("values").cloned().unwrap_or(Value::Null);
    if reply.doc.get("result").and_then(Value::as_bool) == Some(false) {
        let text = values.as_str().map(str::to_string).unwrap_or_else(|| values.to_string());
        return if text.ends_with("is not allowed") { Ok(None) } else { Err(service_error(service, &text)) };
    }
    Ok(Some(values))
}

/// [`read`], for a command that has no other source.
pub async fn require(c: &mut Conn, service: &str, args: Value, timeout: Duration) -> Result<Value, Fail> {
    read(c, service, args, timeout).await?.ok_or_else(|| {
        Fail::missing(format!(
            "{service} is not answered on this listener: it takes ws_bridge's loopback listener (127.0.0.1:9091), \
             on the bus backend, with --loopback-policy"
        ))
    })
}

/// `{"names": [...]}`, what `/mowerd/topics` and `/mowerd/services` answer for.
pub fn named(names: &[String]) -> Value {
    json!({ "names": names })
}

/// The rows of `key` in an answer.
pub fn rows<'a>(values: &'a Value, key: &str) -> &'a [Value] {
    values.get(key).and_then(Value::as_array).map(Vec::as_slice).unwrap_or(&[])
}

pub fn text<'a>(row: &'a Value, key: &str) -> &'a str {
    row.get(key).and_then(Value::as_str).unwrap_or("")
}

pub fn number(row: &Value, key: &str) -> u64 {
    row.get(key).and_then(Value::as_u64).unwrap_or(0)
}

pub fn seconds(row: &Value, key: &str) -> Option<f64> {
    row.get(key).and_then(Value::as_f64)
}

/// When `topic`'s latest message was published, on the bridge's clock, in the
/// `/mowerd/topics` answer `values`: `uptime_s` where it has none yet. A rate
/// over two such times is the mean interval between messages, as `ros2 topic
/// hz` takes it, with no error from where the two reads fell between messages.
pub fn last_published(values: &Value, topic: &Value) -> f64 {
    seconds(values, "uptime_s").unwrap_or(0.0) - seconds(topic, "age_s").unwrap_or(0.0)
}

/// `R V 10`, `BE TL 1`: the manifest's way of writing a QoS.
pub fn qos(endpoint: &Value) -> String {
    let q = endpoint.get("qos").cloned().unwrap_or(Value::Null);
    let reliability = if text(&q, "reliability") == "best_effort" { "BE" } else { "R" };
    let durability = if text(&q, "durability") == "transient_local" { "TL" } else { "V" };
    format!("{reliability} {durability} {}", number(&q, "depth"))
}

/// A topic's writer and reader counts make it live: the bus never forgets a
/// name, and `ros2 topic list` would not show one with no endpoint.
pub fn is_live(topic: &Value) -> bool {
    !rows(topic, "writers").is_empty() || !rows(topic, "readers").is_empty()
}

/// `name`'s row in `/mowerd/bridge`'s answer; a topic and a service may share
/// a name, so `of_kind` picks by direction.
pub fn bridged<'a>(bridge: &'a Value, name: &str, of_kind: impl Fn(&str) -> bool) -> Option<&'a Value> {
    rows(bridge, "entries").iter().find(|e| text(e, "name") == name && of_kind(text(e, "direction")))
}

pub fn is_topic_direction(direction: &str) -> bool {
    matches!(direction, "to_ros" | "from_ros")
}

pub fn is_service_direction(direction: &str) -> bool {
    direction.starts_with("service_")
}

/// One bridged name, on one line.
pub fn bridge_line(e: &Value) -> String {
    let mut line = format!("{}: forwarded {}", text(e, "direction"), number(e, "forwarded"));
    for key in ["skipped", "stale", "unanswered", "dropped", "invalid"] {
        let n = number(e, key);
        if n > 0 {
            line.push_str(&format!(", {key} {n}"));
        }
    }
    let on: Vec<&str> = ["dds_writer", "dds_reader", "dds_server", "bus_writer", "bus_server"]
        .into_iter()
        .filter(|k| e.get(*k).and_then(Value::as_bool) == Some(true))
        .collect();
    if !on.is_empty() {
        line.push_str(&format!(" ({})", on.join(", ")));
    }
    line
}

/// Rows as aligned columns, ready to print.
pub fn table(rows: Vec<Vec<String>>) -> Vec<String> {
    format::table(&rows).into_iter().map(|l| l.trim_end().to_string()).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn qos_reads_as_the_manifest_writes_it() {
        let e = json!({"scope": "gps", "qos": {"depth": 10, "reliability": "reliable", "durability": "volatile"}});
        assert_eq!(qos(&e), "R V 10");
        let e = json!({"qos": {"depth": 1, "reliability": "best_effort", "durability": "transient_local"}});
        assert_eq!(qos(&e), "BE TL 1");
    }

    #[test]
    fn a_bridge_line_shows_what_is_not_zero() {
        let e = json!({"name": "/fix", "direction": "to_ros", "forwarded": 12, "skipped": 3, "stale": 0, "dds_writer": true, "bus_server": false});
        assert_eq!(bridge_line(&e), "to_ros: forwarded 12, skipped 3 (dds_writer)");
    }
}
