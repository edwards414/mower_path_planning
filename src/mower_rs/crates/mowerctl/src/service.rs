//! `mowerctl service list | call`.
//!
//! No call of today's bridge lists services, so `list` prints what the
//! listener's policy allows: the policy built into this binary (the
//! `ws_bridge.yaml` of the same source tree, which on the robot is the image
//! the bridge runs from), or `--policy FILE`. Whether a server is up shows
//! on a call. E8-2's `/mowerd/services` answers from the bus instead.
//!
//! `call` goes through the bridge like the app's calls, so only what the
//! policy allows can be called at all. On top of that mowerctl stays
//! read-only by default: a service that is not a known query may change
//! the robot's state (start a mission, restart the stack, update it) and is
//! only called with `--yes`.

use std::time::Duration;

use serde_json::{json, Value};

use crate::cli::{self, ConnOpts, CALL_TIMEOUT};
use crate::conn::{service_error, Conn};
use crate::format::{self, Out};
use crate::{block_on, Fail};

/// src/mower_bringup/config/ws_bridge.yaml, the policy of the source tree
/// this binary was built from.
const BUILT_IN_POLICY: &str = include_str!("../../../../mower_bringup/config/ws_bridge.yaml");

/// Answered by ws_bridge itself, outside its policy (mower_ws_bridge client.rs).
const NATIVE: [(&str, &str); 2] = [("/rosapi/get_time", "rosapi/srv/GetTime"), ("/rosapi/topics", "rosapi/srv/Topics")];

/// Services that only report: mowerctl calls them without `--yes`.
const QUERIES: [&str; 4] = ["/rosapi/topics", "/rosapi/get_time", "/check_nav_status", "/coverage_progress_status"];
/// ... and every parameter read (the E8-2 loopback policy allows them).
const QUERY_SUFFIXES: [&str; 4] = ["/get_parameters", "/list_parameters", "/describe_parameters", "/get_parameter_types"];

pub fn is_query(service: &str) -> bool {
    QUERIES.contains(&service) || QUERY_SUFFIXES.iter().any(|s| service.ends_with(s) && service.len() > s.len())
}

/// `args`: the subcommand first (main.rs appends the leading options).
pub fn main(args: Vec<String>) -> Result<(), Fail> {
    let Some((sub, rest)) = args.split_first() else {
        return Err(Fail::usage("service: list or call?"));
    };
    let rest = rest.to_vec();
    match sub.as_str() {
        "list" => list(rest, &mut Out::stdout()),
        "call" => Call::parse(rest).and_then(|c| block_on(c.run(&mut Out::stdout()))),
        other => Err(Fail::usage(format!("service {other}: no such command (list, call)"))),
    }
}

/// The services a policy file allows, with their types, sorted by name.
pub fn policy_services(yaml: &str) -> Result<Vec<(String, String)>, String> {
    let doc: Value = serde_yaml::from_str(yaml).map_err(|e| format!("not a ws_bridge policy: {e}"))?;
    let mut out: Vec<(String, String)> = match doc.get("services") {
        None | Some(Value::Null) => Vec::new(),
        Some(Value::Object(map)) => map.iter().map(|(k, v)| (k.clone(), v.as_str().unwrap_or("").to_string())).collect(),
        Some(_) => return Err("`services` is not a name: type map".into()),
    };
    out.sort();
    Ok(out)
}

fn list(args: Vec<String>, out: &mut Out) -> Result<(), Fail> {
    let mut json_out = false;
    let mut policy: Option<String> = None;
    let mut it = args.into_iter();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--json" => json_out = true,
            "--policy" => policy = Some(cli::value(&a, &mut it)?),
            // accepted for symmetry with the other commands; nothing to connect to
            _ if ConnOpts::default().take(&a, &mut it)? => {}
            _ => return Err(cli::unknown("service list", &a)),
        }
    }
    let (text, source) = match &policy {
        Some(path) => (std::fs::read_to_string(path).map_err(|e| Fail::failed(format!("{path}: {e}")))?, path.clone()),
        None => (BUILT_IN_POLICY.to_string(), "the policy built into mowerctl".to_string()),
    };
    let mut services = policy_services(&text).map_err(|e| Fail::failed(format!("{source}: {e}")))?;
    services.extend(NATIVE.iter().map(|(n, t)| (n.to_string(), t.to_string())));
    services.sort();
    if json_out {
        let rows: Vec<Value> = services.iter().map(|(n, t)| json!({"name": n, "type": t, "query": is_query(n)})).collect();
        return out.line(&Value::Array(rows).to_string());
    }
    eprintln!("services the bridge's policy allows ({source}); a call shows whether one is served");
    let rows: Vec<Vec<String>> = services
        .into_iter()
        .map(|(n, t)| {
            let note = if is_query(&n) { "" } else { "needs --yes" };
            vec![n, t, note.to_string()]
        })
        .collect();
    for line in format::table(&rows) {
        out.line(line.trim_end())?;
    }
    Ok(())
}

#[derive(Debug)]
struct Call {
    conn: ConnOpts,
    name: String,
    args: Value,
    yes: bool,
    raw: bool,
}

impl Call {
    fn parse(args: Vec<String>) -> Result<Self, Fail> {
        let mut c = Call { conn: ConnOpts::default(), name: String::new(), args: json!({}), yes: false, raw: false };
        let mut positional = Vec::new();
        let mut it = args.into_iter();
        while let Some(a) = it.next() {
            match a.as_str() {
                "--yes" | "-y" => c.yes = true,
                "--raw" => c.raw = true,
                _ if c.conn.take(&a, &mut it)? => {}
                // a JSON argument may start with '-' only as a number, never an object
                _ if !a.starts_with('-') => positional.push(a),
                _ => return Err(cli::unknown("service call", &a)),
            }
        }
        let mut positional = positional.into_iter();
        c.name = positional.next().ok_or_else(|| Fail::usage("service call: which service?"))?;
        if let Some(text) = positional.next() {
            c.args = serde_json::from_str(&text).map_err(|e| Fail::usage(format!("service call: ARGS is not JSON ({e})")))?;
            if !c.args.is_object() {
                return Err(Fail::usage("service call: ARGS must be a JSON object (the request's fields)"));
            }
        }
        if let Some(extra) = positional.next() {
            return Err(cli::unknown("service call", &extra));
        }
        Ok(c)
    }

    async fn run(self, out: &mut Out) -> Result<(), Fail> {
        if !self.yes && !is_query(&self.name) {
            return Err(Fail::failed(format!(
                "{} is not a known query and may change the robot's state; add --yes to call it",
                self.name
            )));
        }
        let timeout: Duration = self.conn.timeout_or(CALL_TIMEOUT);
        let mut c = Conn::open(&self.conn).await?;
        let reply = c.call(&self.name, self.args.clone(), timeout).await;
        c.close().await;
        let reply = reply?;
        if self.raw {
            out.line(&reply.text)?;
        }
        let values = reply.doc.get("values").cloned().unwrap_or(Value::Null);
        if reply.doc.get("result").and_then(Value::as_bool) == Some(false) {
            return Err(service_error(&self.name, values.as_str().unwrap_or(&values.to_string())));
        }
        if !self.raw {
            out.line(&serde_json::to_string_pretty(&values).unwrap_or_else(|_| values.to_string()))?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fake_bridge::FakeBridge;
    use crate::gate::Pairing;

    fn v(list: &[&str]) -> Vec<String> {
        list.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn the_built_in_policy_is_the_trees_and_parses() {
        let services = policy_services(BUILT_IN_POLICY).unwrap();
        assert!(services.contains(&("/check_nav_status".to_string(), "std_srvs/srv/Trigger".to_string())));
        assert!(services.contains(&("/system/restart".to_string(), "std_srvs/srv/Trigger".to_string())));
        assert!(services.windows(2).all(|w| w[0] <= w[1]));
        assert!(policy_services("topics_sub: ['/a']\n").unwrap().is_empty());
        assert!(policy_services("services: [1, 2]\n").is_err());
    }

    #[test]
    fn queries_run_without_yes_and_everything_else_needs_it() {
        for q in ["/rosapi/topics", "/rosapi/get_time", "/check_nav_status", "/coverage_progress_status", "/map_manage/get_parameters", "/boustrophedon_coverage/list_parameters"] {
            assert!(is_query(q), "{q}");
        }
        // what the app's buttons do, and the parameter write it uses
        for s in ["/system/restart", "/system/update", "/system/check_update", "/zone_exec_path", "/cancel_nav2", "/boustrophedon_coverage/set_parameters", "/get_parameters"] {
            assert!(!is_query(s), "{s}");
        }
        // every query a policy names is one of its services or a native one
        let allowed = policy_services(BUILT_IN_POLICY).unwrap();
        for q in QUERIES {
            assert!(NATIVE.iter().any(|(n, _)| *n == q) || allowed.iter().any(|(n, _)| n == q), "{q} is not in ws_bridge.yaml");
        }
        let e = block_on(Call::parse(v(&["/system/restart"])).unwrap().run(&mut Out::stdout())).unwrap_err();
        assert_eq!(e.code, crate::EXIT_FAILED);
        assert!(e.msg.contains("--yes"), "{e:?}");
    }

    #[test]
    fn call_arguments() {
        let c = Call::parse(v(&["/zone_exec_path", "{\"zone_id\": 2}", "--yes", "--url", "ws://h:1"])).unwrap();
        assert_eq!(c.name, "/zone_exec_path");
        assert_eq!(c.args, json!({"zone_id": 2}));
        assert!(c.yes && !c.raw);
        assert_eq!(Call::parse(v(&["/check_nav_status"])).unwrap().args, json!({}));
        assert!(Call::parse(v(&[])).is_err());
        assert!(Call::parse(v(&["/s", "[1]"])).unwrap_err().msg.contains("JSON object"));
        assert!(Call::parse(v(&["/s", "{"])).unwrap_err().msg.contains("not JSON"));
        assert!(Call::parse(v(&["/s", "{}", "extra"])).is_err());
        assert!(main(v(&["list", "--policy", "/nonexistent/ws_bridge.yaml"])).is_err());
    }

    fn call(url: &str, args: &[&str]) -> (Result<(), Fail>, String) {
        let mut a = v(args);
        a.extend(v(&["--url", url]));
        let (mut out, buf) = Out::buffer();
        let r = block_on(Call::parse(a).unwrap().run(&mut out));
        let text = buf.lock().unwrap().clone();
        (r, text)
    }

    #[test]
    fn calls_go_through_the_bridge_and_its_errors_map_to_exit_codes() {
        let b = FakeBridge::start(None);
        let (r, text) = call(&b.url, &["/check_nav_status"]);
        assert_eq!(r, Ok(()));
        assert_eq!(serde_json::from_str::<Value>(&text).unwrap(), json!({"message": "{\"nav_state\":\"idle\"}", "success": true}));
        assert_eq!(call(&b.url, &["/coverage_progress_status"]).0.unwrap_err().code, crate::EXIT_MISSING);
        assert_eq!(call(&b.url, &["/rm_rf", "--yes"]).0.unwrap_err().code, crate::EXIT_MISSING);
        assert_eq!(call(&b.url, &["/slow", "--yes", "--timeout", "0.3"]).0.unwrap_err().code, crate::EXIT_TIMEOUT);
        assert_eq!(call(&b.url, &["/system/restart"]).0.unwrap_err().code, crate::EXIT_FAILED);
        assert!(b.seen_ops("call_service").iter().all(|d| d["service"] != "/system/restart"), "refused before it was sent");
        let (r, text) = call(&b.url, &["/system/restart", "--yes", "--raw"]);
        assert_eq!(r, Ok(()));
        assert!(text.contains("\"op\":\"service_response\""), "{text}");
        let sent = b.seen_ops("call_service");
        let restart = sent.iter().find(|d| d["service"] == "/system/restart").unwrap();
        assert_eq!(restart["args"], json!({}));
        assert!(restart["id"].as_str().unwrap().starts_with("mowerctl:call:"));
    }

    #[test]
    fn the_gate_is_passed_with_a_pairing_file_and_refused_without() {
        let pairing = r#"{"robot_id": "MW-ABC123", "secret": "mfrggzdf"}"#;
        let b = FakeBridge::start(Some(Pairing::parse(pairing).unwrap()));
        let path = std::env::temp_dir().join(format!("mowerctl-pair-{}.json", std::process::id()));
        std::fs::write(&path, pairing).unwrap();
        let path = path.to_str().unwrap().to_string();
        let (r, text) = call(&b.url, &["/check_nav_status", "--pair", &path]);
        assert_eq!(r, Ok(()), "{text}");
        let e = call(&b.url, &["/check_nav_status"]).0.unwrap_err();
        assert_eq!(e.code, crate::EXIT_CONNECT);
        assert!(e.msg.contains("401") && e.msg.contains("pairing required"), "{e:?}");
        let h = b.headers.lock().unwrap()[0].clone();
        let get = |k: &str| h.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone()).unwrap_or_default();
        assert_eq!(get("x-mower-robot"), "MW-ABC123");
        assert_eq!(get("x-mower-client"), "mowerctl");
        assert_eq!(get("x-mower-nonce").len(), 32);
        assert!((get("x-mower-time").parse::<i64>().unwrap() - crate::gate::now_unix()).abs() < 60);
        let _ = std::fs::remove_file(&path);
    }
}
