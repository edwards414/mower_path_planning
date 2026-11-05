//! `mowerctl graph`: everything `/mowerd/*` knows, at once (E8-2), what
//! `graph_dump.py` gave over DDS and cannot give once DDS has gone.
//!
//! It shows the bus, which holds every topic of a mower_rs module. The names
//! only DDS has (nav2's own) are not on it until E9 takes them out (E8 design
//! §2.7). `--sample S` adds each topic's rate, from its `published` counter
//! over S seconds: no subscription, so no reader and no DDS traffic.

use std::collections::BTreeMap;
use std::time::Duration;

use serde_json::{json, Value};

use crate::cli::{self, ConnOpts, DEFAULT_TIMEOUT};
use crate::conn::Conn;
use crate::format::Out;
use crate::mowerd::{self, number, rows, seconds, text, ACTIONS, BRIDGE, MODULES, SERVICES, TOPICS};
use crate::{block_on, Fail};

#[derive(Debug)]
pub struct Cmd {
    conn: ConnOpts,
    sample: Option<Duration>,
    json: bool,
}

pub fn main(args: Vec<String>) -> Result<(), Fail> {
    Cmd::parse(args).and_then(|c| block_on(c.run(&mut Out::stdout())))
}

/// Each topic's rate between two `/mowerd/topics` answers: the messages
/// between them over the time between the latest of each (`last_published`).
fn rates(first: &Value, second: &Value) -> BTreeMap<String, f64> {
    let before: BTreeMap<&str, (u64, f64)> =
        rows(first, "topics").iter().map(|t| (text(t, "name"), (number(t, "published"), mowerd::last_published(first, t)))).collect();
    let started = seconds(first, "uptime_s").unwrap_or(0.0);
    rows(second, "topics")
        .iter()
        .map(|t| {
            let (count, since) = before.get(text(t, "name")).copied().unwrap_or((0, started));
            let (messages, dt) = (number(t, "published").saturating_sub(count), mowerd::last_published(second, t) - since);
            (text(t, "name").to_string(), if dt > 0.0 { messages as f64 / dt } else { 0.0 })
        })
        .collect()
}

impl Cmd {
    fn parse(args: Vec<String>) -> Result<Self, Fail> {
        let mut c = Cmd { conn: ConnOpts::default(), sample: None, json: false };
        let mut it = args.into_iter();
        while let Some(a) = it.next() {
            match a.as_str() {
                "--json" => c.json = true,
                "--sample" => c.sample = Some(cli::seconds(&a, &cli::value(&a, &mut it)?)?),
                _ if c.conn.take(&a, &mut it)? => {}
                _ => return Err(cli::unknown("graph", &a)),
            }
        }
        Ok(c)
    }

    async fn run(self, out: &mut Out) -> Result<(), Fail> {
        let timeout = self.conn.timeout_or(DEFAULT_TIMEOUT);
        let mut c = Conn::open(&self.conn).await?;
        let doc = self.gather(&mut c, timeout).await;
        c.close().await;
        let doc = doc?;
        if self.json {
            return out.line(&doc.to_string());
        }
        let (topics, services, actions) = (rows(&doc, "topics"), rows(&doc, "services"), rows(&doc, "actions"));
        out.line(&format!(
            "{} topics, {} services, {} actions, {} module instances, {} bridged names",
            topics.len(),
            services.len(),
            actions.len(),
            rows(&doc, "modules").len(),
            rows(&doc, "bridge").len()
        ))?;
        let lines = topics
            .iter()
            .map(|t| {
                let rate = seconds(t, "rate_hz").map(|r| format!("{r:.2} Hz")).unwrap_or_default();
                vec![
                    text(t, "name").to_string(),
                    format!("[{}]", t.get("type").and_then(Value::as_str).unwrap_or("?")),
                    format!("{} writers", rows(t, "writers").len()),
                    format!("{} readers", rows(t, "readers").len()),
                    format!("published {}", number(t, "published")),
                    rate,
                ]
            })
            .collect();
        for line in mowerd::table(lines) {
            out.line(&line)?;
        }
        Ok(())
    }

    /// One document: `/mowerd/*`'s answers under their names, with `rate_hz`
    /// on each topic when sampled.
    async fn gather(&self, c: &mut Conn, timeout: Duration) -> Result<Value, Fail> {
        let mut topics = mowerd::require(c, TOPICS, json!({}), timeout).await?;
        if let Some(sample) = self.sample {
            tokio::time::sleep(sample).await;
            let second = mowerd::require(c, TOPICS, json!({}), timeout).await?;
            let found = rates(&topics, &second);
            topics = second;
            if let Some(rows) = topics.get_mut("topics").and_then(Value::as_array_mut) {
                for t in rows {
                    if let Some(rate) = found.get(text(t, "name")) {
                        t["rate_hz"] = json!(rate);
                    }
                }
            }
        }
        let services = mowerd::require(c, SERVICES, json!({}), timeout).await?;
        let actions = mowerd::require(c, ACTIONS, json!({}), timeout).await?;
        let modules = mowerd::require(c, MODULES, json!({}), timeout).await?;
        let bridge = mowerd::require(c, BRIDGE, json!({}), timeout).await?;
        let mut doc = json!({
            "uptime_s": topics["uptime_s"],
            "topics": topics["topics"],
            "services": services["services"],
            "actions": actions["actions"],
            "modules": modules["modules"],
            "other_scopes": modules["other_scopes"],
            "bridge_running": bridge["running"],
            "bridge": bridge["entries"],
        });
        if let Some(sample) = self.sample {
            doc["sample_s"] = json!(sample.as_secs_f64());
        }
        Ok(doc)
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
    fn rates_come_from_two_answers_of_the_counters() {
        let a = json!({"uptime_s": 10.0, "topics": [{"name": "/a", "published": 100, "age_s": 0.0}, {"name": "/b", "published": 5, "age_s": 3.0}, {"name": "/slow", "published": 1, "age_s": 0.5}]});
        let b = json!({"uptime_s": 12.0, "topics": [{"name": "/a", "published": 120, "age_s": 0.0}, {"name": "/b", "published": 5, "age_s": 5.0}, {"name": "/new", "published": 4, "age_s": 0.0}, {"name": "/slow", "published": 2, "age_s": 0.5}]});
        let r = rates(&a, &b);
        assert_eq!(r["/a"], 10.0);
        assert_eq!(r["/b"], 0.0);
        assert_eq!(r["/new"], 2.0);
        // one message 2 s after the last, whatever the two reads missed: 0.5 Hz
        assert_eq!(r["/slow"], 0.5);
    }

    #[test]
    fn the_document_holds_all_of_it_and_no_subscription_was_made() {
        let b = FakeBridge::loopback();
        let (mut out, buf) = Out::buffer();
        block_on(Cmd::parse(with_url(&["--json", "--sample", "0.3"], &b.url)).unwrap().run(&mut out)).unwrap();
        let doc: Value = serde_json::from_str(&buf.lock().unwrap()).unwrap();
        let names: Vec<&str> = rows(&doc, "topics").iter().map(|t| text(t, "name")).collect();
        assert!(names.contains(&"/fix"), "{doc}");
        assert!(rows(&doc, "services").iter().any(|s| text(s, "name") == "/check_nav_status"), "{doc}");
        assert!(rows(&doc, "actions").iter().any(|a| text(a, "name") == "/nav_action"), "{doc}");
        assert!(rows(&doc, "modules").iter().any(|m| text(m, "label") == "imu/imu"), "{doc}");
        assert_eq!(rows(&doc, "bridge")[0]["name"], "/fix");
        assert_eq!(doc["sample_s"], 0.3);
        let fix = rows(&doc, "topics").iter().find(|t| text(t, "name") == "/fix").unwrap();
        // the fake's /fix counts at 40 Hz
        assert!((seconds(fix, "rate_hz").unwrap() - 40.0).abs() < 15.0, "{fix}");
        assert!(b.seen_ops("subscribe").is_empty());

        let (mut out, buf) = Out::buffer();
        block_on(Cmd::parse(with_url(&[], &b.url)).unwrap().run(&mut out)).unwrap();
        assert!(buf.lock().unwrap().contains("1 module") || buf.lock().unwrap().contains("2 module"), "{}", buf.lock().unwrap());
    }

    #[test]
    fn it_needs_the_loopback_listeners_view() {
        let b = FakeBridge::start(None);
        let e = block_on(Cmd::parse(with_url(&["--json"], &b.url)).unwrap().run(&mut Out::buffer().0)).unwrap_err();
        assert_eq!(e.code, crate::EXIT_MISSING);
    }
}
