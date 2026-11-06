//! `mowerctl node list`, `module list`: what mower_rsd's supervisor knows
//! (`/mowerd/modules`). A node here is a bus scope: mower_rsd names each
//! instance's after its ROS node.

use serde_json::Value;

use crate::cli::{self, ConnOpts, DEFAULT_TIMEOUT};
use crate::conn::Conn;
use crate::format::Out;
use crate::mowerd::{self, number, rows, seconds, text, MODULES};
use crate::{block_on, Fail};

#[derive(Debug)]
pub struct List {
    conn: ConnOpts,
    verbose: bool,
    json: bool,
}

/// `args`: the subcommand first (main.rs appends the leading options).
pub fn main(command: &str, args: Vec<String>) -> Result<(), Fail> {
    let Some((sub, rest)) = args.split_first() else {
        return Err(Fail::usage(format!("{command}: list?")));
    };
    if sub != "list" {
        return Err(Fail::usage(format!("{command} {sub}: no such command (list)")));
    }
    let list = List::parse(command, rest.to_vec())?;
    block_on(async {
        match command {
            "module" => list.modules(&mut Out::stdout()).await,
            _ => list.nodes(&mut Out::stdout()).await,
        }
    })
}

/// `3 pub, 2 sub, 1 srv`: a scope's endpoints on the bus.
fn endpoints_line(endpoints: Option<&Value>) -> String {
    let Some(endpoints) = endpoints else { return String::new() };
    let mut parts = Vec::new();
    for (key, short) in [("publishes", "pub"), ("subscribes", "sub"), ("serves", "srv"), ("calls", "cli"), ("serves_actions", "act"), ("calls_actions", "act cli")] {
        let n = number(endpoints, key);
        if n > 0 {
            parts.push(format!("{n} {short}"));
        }
    }
    parts.join(", ")
}

/// A number of seconds for people.
fn ago(s: f64) -> String {
    match s {
        s if s < 90.0 => format!("{s:.0} s"),
        s if s < 5400.0 => format!("{:.0} min", s / 60.0),
        s => format!("{:.1} h", s / 3600.0),
    }
}

impl List {
    fn parse(command: &str, args: Vec<String>) -> Result<Self, Fail> {
        let mut c = List { conn: ConnOpts::default(), verbose: false, json: false };
        let mut it = args.into_iter();
        while let Some(a) = it.next() {
            match a.as_str() {
                "-v" | "--verbose" => c.verbose = true,
                "--json" => c.json = true,
                _ if c.conn.take(&a, &mut it)? => {}
                _ => return Err(cli::unknown(&format!("{command} list"), &a)),
            }
        }
        Ok(c)
    }

    async fn answer(&self) -> Result<Value, Fail> {
        let mut c = Conn::open(&self.conn).await?;
        let values = mowerd::require(&mut c, MODULES, serde_json::json!({}), self.conn.timeout_or(DEFAULT_TIMEOUT)).await;
        c.close().await;
        values
    }

    async fn nodes(&self, out: &mut Out) -> Result<(), Fail> {
        let values = self.answer().await?;
        if self.json {
            return out.line(&values.to_string());
        }
        let mut lines: Vec<Vec<String>> = rows(&values, "modules")
            .iter()
            .map(|m| vec![text(m, "node").to_string(), text(m, "state").to_string(), endpoints_line(m.get("endpoints"))])
            .collect();
        lines.extend(rows(&values, "other_scopes").iter().map(|s| vec![text(s, "scope").to_string(), "-".to_string(), endpoints_line(s.get("endpoints"))]));
        lines.sort();
        if self.verbose {
            for line in mowerd::table(lines) {
                out.line(&line)?;
            }
        } else {
            for line in lines {
                out.line(&line[0])?;
            }
        }
        Ok(())
    }

    async fn modules(&self, out: &mut Out) -> Result<(), Fail> {
        let values = self.answer().await?;
        if self.json {
            return out.line(&values.to_string());
        }
        let lines: Vec<Vec<String>> = rows(&values, "modules")
            .iter()
            .map(|m| {
                let failure = m.get("last_failure").filter(|f| !f.is_null()).map(|f| {
                    format!("last failure {} ago: {}", seconds(f, "age_s").map(ago).unwrap_or_default(), text(f, "reason"))
                });
                vec![
                    text(m, "label").to_string(),
                    text(m, "state").to_string(),
                    format!("up {}", seconds(m, "up_s").map(ago).unwrap_or_else(|| "-".to_string())),
                    format!("starts {}", number(m, "starts")),
                    format!("failures {}", number(m, "failures")),
                    failure.unwrap_or_default(),
                ]
            })
            .collect();
        if lines.is_empty() {
            eprintln!("no supervisor in this process (a standalone mower_ws_bridge)");
        }
        for line in mowerd::table(lines) {
            out.line(&line)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fake_bridge::FakeBridge;

    fn with_url(args: &[&str], url: &str) -> Vec<String> {
        let mut a: Vec<String> = args.iter().map(|s| s.to_string()).collect();
        a.extend(["--url".to_string(), url.to_string()]);
        a
    }

    #[test]
    fn modules_and_nodes_print_the_supervisors_slots() {
        let b = FakeBridge::loopback();
        let (mut out, buf) = Out::buffer();
        block_on(List::parse("module", with_url(&[], &b.url)).unwrap().modules(&mut out)).unwrap();
        let text = buf.lock().unwrap().clone();
        assert!(text.contains("gps/gps  running"), "{text}");
        assert!(text.contains("imu/imu  restarting"), "{text}");
        assert!(text.contains("failures 1"), "{text}");
        assert!(text.contains("imu/imu: no such device"), "{text}");

        let (mut out, buf) = Out::buffer();
        block_on(List::parse("node", with_url(&[], &b.url)).unwrap().nodes(&mut out)).unwrap();
        let names = buf.lock().unwrap().clone();
        assert!(names.lines().any(|l| l == "gps"), "{names}");
        assert!(names.lines().any(|l| l == "nav_action_server"), "a scope no slot names: {names}");

        let (mut out, buf) = Out::buffer();
        block_on(List::parse("node", with_url(&["-v"], &b.url)).unwrap().nodes(&mut out)).unwrap();
        assert!(buf.lock().unwrap().contains("1 pub"), "{}", buf.lock().unwrap());
    }

    #[test]
    fn they_need_the_loopback_listeners_view() {
        let b = FakeBridge::start(None);
        let e = block_on(List::parse("module", with_url(&[], &b.url)).unwrap().modules(&mut Out::buffer().0)).unwrap_err();
        assert_eq!(e.code, crate::EXIT_MISSING);
        assert!(e.msg.contains("loopback"), "{e:?}");
        assert!(main("node", vec!["info".to_string()]).unwrap_err().msg.contains("no such command"));
    }

    #[test]
    fn seconds_read_for_people() {
        assert_eq!(ago(12.4), "12 s");
        assert_eq!(ago(600.0), "10 min");
        assert_eq!(ago(7200.0), "2.0 h");
    }
}
