//! `mowerctl param list | get`. Read-only: there is no `param set` (E8 design
//! D2).
//!
//! The loopback listener's policy allows the parameter reads of every node,
//! which go to the node's own `/<node>/list_parameters` and `get_parameters`.
//! Those are DDS services until E9, so this one command does touch DDS, for
//! as long as the read lasts (mower_ws_bridge's loopback.rs). Only nodes that
//! serve them answer: map_manage, boustrophedon_coverage, the three of
//! localize, and the C++ and Python nodes.

use serde_json::{json, Value};

use crate::cli::{self, ConnOpts, CALL_TIMEOUT};
use crate::conn::Conn;
use crate::format::Out;
use crate::{block_on, Fail};

/// rcl_interfaces/msg/ParameterType
const NOT_SET: u64 = 0;

pub fn main(args: Vec<String>) -> Result<(), Fail> {
    let Some((sub, rest)) = args.split_first() else {
        return Err(Fail::usage("param: list or get?"));
    };
    match sub.as_str() {
        "list" | "get" => Cmd::parse(sub, rest.to_vec()).and_then(|c| block_on(c.run(&mut Out::stdout()))),
        other => Err(Fail::usage(format!("param {other}: no such command (list, get)"))),
    }
}

#[derive(Debug)]
struct Cmd {
    conn: ConnOpts,
    get: bool,
    node: String,
    names: Vec<String>,
    json: bool,
}

/// `/<node>/<service>`, however the node was written.
fn service_of(node: &str, service: &str) -> String {
    format!("/{}/{service}", node.trim_matches('/'))
}

/// A parameter's value as `ros2 param get` would show it: the field its
/// `type` names. `None` when it is not set.
fn shown(value: &Value) -> Option<Value> {
    // r2r's JSON names the field `type_`, rcl's `type`
    let ty = value.get("type").or_else(|| value.get("type_")).and_then(Value::as_u64).unwrap_or(NOT_SET);
    let field = match ty {
        1 => "bool_value",
        2 => "integer_value",
        3 => "double_value",
        4 => "string_value",
        5 => "byte_array_value",
        6 => "bool_array_value",
        7 => "integer_array_value",
        8 => "double_array_value",
        9 => "string_array_value",
        _ => return None,
    };
    value.get(field).cloned()
}

impl Cmd {
    fn parse(sub: &str, args: Vec<String>) -> Result<Self, Fail> {
        let mut c = Cmd { conn: ConnOpts::default(), get: sub == "get", node: String::new(), names: Vec::new(), json: false };
        let mut it = args.into_iter();
        while let Some(a) = it.next() {
            match a.as_str() {
                "--json" => c.json = true,
                _ if c.conn.take(&a, &mut it)? => {}
                _ if a.starts_with('-') => return Err(cli::unknown("param", &a)),
                _ if c.node.is_empty() => c.node = a,
                _ => c.names.push(a),
            }
        }
        if c.node.is_empty() {
            return Err(Fail::usage(format!("param {sub}: which node?")));
        }
        if c.get && c.names.is_empty() {
            return Err(Fail::usage("param get: which parameter(s)?"));
        }
        Ok(c)
    }

    async fn run(self, out: &mut Out) -> Result<(), Fail> {
        let timeout = self.conn.timeout_or(CALL_TIMEOUT);
        let mut c = Conn::open(&self.conn).await?;
        let values = if self.get {
            c.call_values(&service_of(&self.node, "get_parameters"), json!({"names": self.names}), timeout).await
        } else {
            c.call_values(&service_of(&self.node, "list_parameters"), json!({"prefixes": self.names, "depth": 0}), timeout).await
        };
        c.close().await;
        let values = values?;
        if self.json {
            return out.line(&values.to_string());
        }
        if !self.get {
            let names = values.get("result").and_then(|r| r.get("names")).and_then(Value::as_array).cloned().unwrap_or_default();
            for name in names.iter().filter_map(Value::as_str) {
                out.line(name)?;
            }
            return Ok(());
        }
        let answers = values.get("values").and_then(Value::as_array).cloned().unwrap_or_default();
        let mut unset = Vec::new();
        // a server may answer an unknown name with nothing at all
        for (i, name) in self.names.iter().enumerate() {
            match answers.get(i).and_then(shown) {
                Some(v) => out.line(&format!("{name}: {v}"))?,
                None => unset.push(name.as_str()),
            }
        }
        if unset.is_empty() {
            Ok(())
        } else {
            Err(Fail::missing(format!("{}: not set: {}", self.node, unset.join(", "))))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fake_bridge::FakeBridge;

    fn v(list: &[&str]) -> Vec<String> {
        list.iter().map(|s| s.to_string()).collect()
    }

    fn run_on(url: &str, sub: &str, args: &[&str]) -> (Result<(), Fail>, String) {
        let mut a = v(args);
        a.extend(v(&["--url", url]));
        let (mut out, buf) = Out::buffer();
        let r = block_on(Cmd::parse(sub, a).unwrap().run(&mut out));
        let text = buf.lock().unwrap().clone();
        (r, text)
    }

    #[test]
    fn arguments() {
        assert_eq!(service_of("map_manage", "get_parameters"), "/map_manage/get_parameters");
        assert_eq!(service_of("/map_manage/", "list_parameters"), "/map_manage/list_parameters");
        assert!(Cmd::parse("get", v(&["/n"])).is_err());
        assert!(Cmd::parse("list", v(&[])).is_err());
        assert!(Cmd::parse("list", v(&["/n", "--bogus"])).is_err());
        // no writing (D2)
        assert!(main(v(&["set", "/n", "a", "1"])).unwrap_err().msg.contains("no such command"));
    }

    #[test]
    fn values_print_by_their_type() {
        assert_eq!(shown(&json!({"type": 3, "double_value": 2.5, "integer_value": 0})), Some(json!(2.5)));
        assert_eq!(shown(&json!({"type_": 4, "string_value": "x"})), Some(json!("x")));
        assert_eq!(shown(&json!({"type": 7, "integer_array_value": [1, 2]})), Some(json!([1, 2])));
        assert_eq!(shown(&json!({"type": 0})), None);
    }

    #[test]
    fn list_and_get_call_the_nodes_services() {
        let b = FakeBridge::loopback();
        let (r, text) = run_on(&b.url, "list", &["map_manage"]);
        assert_eq!(r, Ok(()));
        assert_eq!(text, "free_space_resolution\nrobot_radius\n");
        let (r, text) = run_on(&b.url, "get", &["/map_manage", "robot_radius", "no_such"]);
        assert_eq!(r.unwrap_err().code, crate::EXIT_MISSING);
        assert_eq!(text, "robot_radius: 0.35\n");
        let calls = b.seen_ops("call_service");
        assert_eq!(calls[0]["service"], "/map_manage/list_parameters");
        assert_eq!(calls[0]["args"], json!({"prefixes": [], "depth": 0}));
        assert_eq!(calls[1]["service"], "/map_manage/get_parameters");
        assert_eq!(calls[1]["args"], json!({"names": ["robot_radius", "no_such"]}));
        // a node that serves none, and a listener that does not allow it
        assert_eq!(run_on(&b.url, "list", &["battery_state"]).0.unwrap_err().code, crate::EXIT_MISSING);
        let refused = FakeBridge::start(None);
        assert_eq!(run_on(&refused.url, "list", &["map_manage"]).0.unwrap_err().code, crate::EXIT_MISSING);
    }
}
