//! `mowerctl bridge`: what the bus bridge carries to and from DDS, name by
//! name, with its counters (`/mowerd/bridge`). It goes when E9 takes the
//! bridge out.

use serde_json::{json, Value};

use crate::cli::{self, ConnOpts, DEFAULT_TIMEOUT};
use crate::conn::Conn;
use crate::format::Out;
use crate::mowerd::{self, rows, text, BRIDGE};
use crate::{block_on, Fail};

#[derive(Debug)]
pub struct Cmd {
    conn: ConnOpts,
    name: Option<String>,
    json: bool,
}

pub fn main(args: Vec<String>) -> Result<(), Fail> {
    Cmd::parse(args).and_then(|c| block_on(c.run(&mut Out::stdout())))
}

impl Cmd {
    fn parse(args: Vec<String>) -> Result<Self, Fail> {
        let mut c = Cmd { conn: ConnOpts::default(), name: None, json: false };
        let mut it = args.into_iter();
        while let Some(a) = it.next() {
            match a.as_str() {
                "--json" => c.json = true,
                _ if c.conn.take(&a, &mut it)? => {}
                _ if !a.starts_with('-') && c.name.is_none() => c.name = Some(a),
                _ => return Err(cli::unknown("bridge", &a)),
            }
        }
        Ok(c)
    }

    async fn run(self, out: &mut Out) -> Result<(), Fail> {
        let mut c = Conn::open(&self.conn).await?;
        let values = mowerd::require(&mut c, BRIDGE, json!({}), self.conn.timeout_or(DEFAULT_TIMEOUT)).await;
        c.close().await;
        let values = values?;
        let entries: Vec<&Value> = rows(&values, "entries").iter().filter(|e| self.name.as_deref().is_none_or(|n| text(e, "name") == n)).collect();
        if let (Some(name), true) = (&self.name, entries.is_empty()) {
            return Err(Fail::missing(format!("{name}: the bus bridge does not carry it")));
        }
        if self.json {
            return out.line(&if self.name.is_some() { json!(entries) } else { values }.to_string());
        }
        if values.get("running").and_then(Value::as_bool) != Some(true) {
            return out.line("no bus bridge runs in this process");
        }
        let lines = entries.iter().map(|e| vec![text(e, "name").to_string(), mowerd::bridge_line(e)]).collect();
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

    fn run_on(url: &str, args: &[&str]) -> (Result<(), Fail>, String) {
        let mut a: Vec<String> = args.iter().map(|s| s.to_string()).collect();
        a.extend(["--url".to_string(), url.to_string()]);
        let (mut out, buf) = Out::buffer();
        let r = block_on(Cmd::parse(a).unwrap().run(&mut out));
        let text = buf.lock().unwrap().clone();
        (r, text)
    }

    #[test]
    fn it_shows_each_bridged_name_with_its_counters() {
        let b = FakeBridge::loopback();
        let (r, text) = run_on(&b.url, &[]);
        assert_eq!(r, Ok(()));
        assert_eq!(text, "/fix  to_ros: forwarded 12, skipped 3 (dds_writer)\n");
        assert_eq!(run_on(&b.url, &["/no_such"]).0.unwrap_err().code, crate::EXIT_MISSING);
        let (r, text) = run_on(&b.url, &["/fix", "--json"]);
        assert_eq!(r, Ok(()));
        assert_eq!(serde_json::from_str::<Value>(&text).unwrap()[0]["forwarded"], 12);
        assert_eq!(run_on(&FakeBridge::start(None).url, &[]).0.unwrap_err().code, crate::EXIT_MISSING);
    }
}
