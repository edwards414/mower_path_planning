//! `mowerctl topic list | info | echo | hz`.
//!
//! What a listener serves is what its policy allows and the graph has:
//! `/rosapi/topics` (answered by the bridge itself) for `list` and `info`, a
//! `subscribe` for `echo` and `hz`. Today the loopback listener applies the
//! public listener's policy, so these are the app's topics. E8-2 gives the
//! loopback its own policy and the bridge's own introspection (`/mowerd/*`):
//! `info` then grows writers, readers, QoS and counters, and `hz` a mode that
//! reads the counters without subscribing.

use std::time::{Duration, Instant};

use serde_json::{json, Value};

use crate::cli::{self, ConnOpts, DEFAULT_TIMEOUT};
use crate::conn::{self, subscribe_error, Conn, Frame};
use crate::format::{self, Out};
use crate::hz::{Hz, DEFAULT_WINDOW};
use crate::{block_on, Fail};

/// `args`: the subcommand first, then its arguments (main.rs appends the
/// connection options given before `topic`).
pub fn main(args: Vec<String>) -> Result<(), Fail> {
    let Some((sub, rest)) = args.split_first() else {
        return Err(Fail::usage("topic: list, info, echo or hz?"));
    };
    let rest = rest.to_vec();
    match sub.as_str() {
        "list" => List::parse(rest).and_then(|c| block_on(c.run(&mut Out::stdout()))),
        "info" => Info::parse(rest).and_then(|c| block_on(c.run(&mut Out::stdout()))),
        "echo" => Echo::parse(rest).and_then(|c| block_on(c.run(&mut Out::stdout()))),
        "hz" => HzCmd::parse(rest).and_then(|c| block_on(c.run(&mut Out::stdout()))),
        other => Err(Fail::usage(format!("topic {other}: no such command (list, info, echo, hz)"))),
    }
}

/// `/rosapi/topics`: (name, type) pairs, sorted by name as the bridge sends
/// them.
async fn listed(c: &mut Conn, timeout: Duration) -> Result<Vec<(String, String)>, Fail> {
    let values = c.call_values("/rosapi/topics", json!({}), timeout).await?;
    let names = values.get("topics").and_then(Value::as_array).cloned().unwrap_or_default();
    let types = values.get("types").and_then(Value::as_array).cloned().unwrap_or_default();
    Ok(names
        .iter()
        .enumerate()
        .filter_map(|(i, n)| Some((n.as_str()?.to_string(), types.get(i).and_then(Value::as_str).unwrap_or("").to_string())))
        .collect())
}

#[derive(Debug)]
struct List {
    conn: ConnOpts,
    types: bool,
    json: bool,
}

impl List {
    fn parse(args: Vec<String>) -> Result<Self, Fail> {
        let mut c = List { conn: ConnOpts::default(), types: false, json: false };
        let mut it = args.into_iter();
        while let Some(a) = it.next() {
            match a.as_str() {
                "-v" | "-t" | "--verbose" | "--show-types" => c.types = true,
                "--json" => c.json = true,
                _ if c.conn.take(&a, &mut it)? => {}
                _ => return Err(cli::unknown("topic list", &a)),
            }
        }
        Ok(c)
    }

    async fn run(self, out: &mut Out) -> Result<(), Fail> {
        let mut c = Conn::open(&self.conn).await?;
        let topics = listed(&mut c, self.conn.timeout_or(DEFAULT_TIMEOUT)).await?;
        c.close().await;
        if self.json {
            let (names, types): (Vec<_>, Vec<_>) = topics.into_iter().unzip();
            return out.line(&json!({"topics": names, "types": types}).to_string());
        }
        if self.types {
            let rows: Vec<Vec<String>> = topics.into_iter().map(|(n, t)| vec![n, format!("[{t}]")]).collect();
            for line in format::table(&rows) {
                out.line(&line)?;
            }
        } else {
            for (n, _) in topics {
                out.line(&n)?;
            }
        }
        Ok(())
    }
}

#[derive(Debug)]
struct Info {
    conn: ConnOpts,
    name: String,
    json: bool,
}

impl Info {
    fn parse(args: Vec<String>) -> Result<Self, Fail> {
        let mut c = Info { conn: ConnOpts::default(), name: String::new(), json: false };
        let mut it = args.into_iter();
        while let Some(a) = it.next() {
            match a.as_str() {
                "--json" => c.json = true,
                _ if c.conn.take(&a, &mut it)? => {}
                _ if !a.starts_with('-') && c.name.is_empty() => c.name = a,
                _ => return Err(cli::unknown("topic info", &a)),
            }
        }
        if c.name.is_empty() {
            return Err(Fail::usage("topic info: which topic?"));
        }
        Ok(c)
    }

    async fn run(self, out: &mut Out) -> Result<(), Fail> {
        let mut c = Conn::open(&self.conn).await?;
        let topics = listed(&mut c, self.conn.timeout_or(DEFAULT_TIMEOUT)).await?;
        c.close().await;
        let Some((name, ty)) = topics.into_iter().find(|(n, _)| *n == self.name) else {
            return Err(Fail::missing(format!(
                "{}: not listed by this listener (nothing publishes it, or its policy does not serve it)",
                self.name
            )));
        };
        if self.json {
            out.line(&json!({"name": name, "type": ty}).to_string())
        } else {
            out.line(&format!("Topic: {name}"))?;
            out.line(&format!("Type: {ty}"))
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum EchoFormat {
    Lines,
    Yaml,
    Raw,
}

#[derive(Debug)]
struct Echo {
    conn: ConnOpts,
    name: String,
    count: Option<u64>,
    throttle_ms: Option<u64>,
    field: Option<String>,
    no_arr: bool,
    format: EchoFormat,
}

impl Echo {
    fn parse(args: Vec<String>) -> Result<Self, Fail> {
        let mut c = Echo {
            conn: ConnOpts::default(),
            name: String::new(),
            count: None,
            throttle_ms: None,
            field: None,
            no_arr: false,
            format: EchoFormat::Lines,
        };
        let mut it = args.into_iter();
        while let Some(a) = it.next() {
            match a.as_str() {
                "-n" | "--count" => c.count = Some(cli::count(&a, &cli::value(&a, &mut it)?)?),
                "--once" => c.count = Some(1),
                "--throttle" => c.throttle_ms = Some(cli::count(&a, &cli::value(&a, &mut it)?)?),
                "--field" => c.field = Some(cli::value(&a, &mut it)?),
                "--no-arr" => c.no_arr = true,
                "--yaml" => c.format = EchoFormat::Yaml,
                "--raw" => c.format = EchoFormat::Raw,
                _ if c.conn.take(&a, &mut it)? => {}
                _ if !a.starts_with('-') && c.name.is_empty() => c.name = a,
                _ => return Err(cli::unknown("topic echo", &a)),
            }
        }
        if c.name.is_empty() {
            return Err(Fail::usage("topic echo: which topic?"));
        }
        if c.format == EchoFormat::Raw && (c.field.is_some() || c.no_arr) {
            return Err(Fail::usage("topic echo: --raw prints the frames as they are, without --field or --no-arr"));
        }
        Ok(c)
    }

    /// One message as this command prints it.
    fn render(&self, f: &Frame) -> Result<String, Fail> {
        if self.format == EchoFormat::Raw {
            return Ok(f.text.clone());
        }
        if self.format == EchoFormat::Lines && self.field.is_none() && !self.no_arr {
            // the bytes the app got; the parsed value only if the frame
            // could not be read that way
            let text = format::raw_member(&f.text, "msg").map(str::to_string);
            let text = text.unwrap_or_else(|| f.doc.get("msg").cloned().unwrap_or(Value::Null).to_string());
            return Ok(format::echo_line(f.unix, &self.name, &text));
        }
        let msg = f.doc.get("msg").cloned().unwrap_or(Value::Null);
        let mut v = match &self.field {
            Some(path) => format::field(&msg, path)
                .cloned()
                .ok_or_else(|| Fail::failed(format!("{}: its messages have no field {path:?}", self.name)))?,
            None => msg,
        };
        if self.no_arr {
            format::drop_arrays(&mut v);
        }
        Ok(match (self.format, &self.field) {
            (EchoFormat::Yaml, _) => format!("{}\n---", format::yaml(&v)),
            // a field alone, like `ros2 topic echo --field`
            (_, Some(_)) => v.to_string(),
            _ => format::echo_line(f.unix, &self.name, &v.to_string()),
        })
    }

    async fn run(self, out: &mut Out) -> Result<(), Fail> {
        let mut c = Conn::open(&self.conn).await?;
        let id = c.subscribe(&self.name, self.throttle_ms).await?;
        let deadline = self.conn.timeout.map(|t| Instant::now() + t);
        let mut printed = 0u64;
        // one listener for the whole run, so a Ctrl-C between two frames
        // is not lost (and ends the run cleanly, closing the WebSocket)
        let mut sigint = std::pin::pin!(tokio::signal::ctrl_c());
        let outcome = loop {
            let frame = tokio::select! {
                f = c.recv_until(deadline) => f,
                _ = &mut sigint => break Ok(()),
            };
            match frame {
                Ok(Some(f)) if f.is_publish_of(&self.name) => {
                    if let Err(e) = self.render(&f).and_then(|text| out.line(&text)) {
                        break Err(e);
                    }
                    printed += 1;
                    if Some(printed) == self.count {
                        break Ok(());
                    }
                }
                Ok(Some(f)) => {
                    if let Some(msg) = f.error_for(&id) {
                        break Err(subscribe_error(&self.name, msg));
                    }
                }
                Ok(None) => break Err(Fail::timeout(format!(
                    "{}: {printed} of {} message(s) within {} s",
                    self.name,
                    self.count.unwrap_or(1),
                    self.conn.timeout.unwrap_or_default().as_secs_f64()
                ))),
                Err(e) => break Err(e),
            }
        };
        c.close().await;
        match outcome {
            // --timeout without -n: a duration, fine once something came
            Err(e) if e.code == crate::EXIT_TIMEOUT && self.count.is_none() && printed > 0 => Ok(()),
            other => other,
        }
    }
}

#[derive(Debug)]
struct HzCmd {
    conn: ConnOpts,
    names: Vec<String>,
    window: usize,
    duration: Option<Duration>,
    json: bool,
}

impl HzCmd {
    fn parse(args: Vec<String>) -> Result<Self, Fail> {
        let mut c = HzCmd { conn: ConnOpts::default(), names: Vec::new(), window: DEFAULT_WINDOW, duration: None, json: false };
        let mut it = args.into_iter();
        while let Some(a) = it.next() {
            match a.as_str() {
                "--window" | "-w" => c.window = cli::count(&a, &cli::value(&a, &mut it)?)? as usize,
                "--duration" => c.duration = Some(cli::seconds(&a, &cli::value(&a, &mut it)?)?),
                "--json" => c.json = true,
                _ if c.conn.take(&a, &mut it)? => {}
                _ if !a.starts_with('-') => {
                    if !c.names.contains(&a) {
                        c.names.push(a);
                    }
                }
                _ => return Err(cli::unknown("topic hz", &a)),
            }
        }
        if c.names.is_empty() {
            return Err(Fail::usage("topic hz: which topic(s)?"));
        }
        Ok(c)
    }

    async fn run(self, out: &mut Out) -> Result<(), Fail> {
        let mut c = Conn::open(&self.conn).await?;
        let mut ids = Vec::new();
        for name in &self.names {
            ids.push(c.subscribe(name, None).await?);
        }
        let mut stats: Vec<Hz> = self.names.iter().map(|_| Hz::new(self.window)).collect();
        let deadline = self.duration.map(|d| Instant::now() + d);
        let mut tick = tokio::time::interval(Duration::from_millis(250));
        let several = self.names.len() > 1;
        let mut sigint = std::pin::pin!(tokio::signal::ctrl_c());
        let outcome: Result<(), Fail> = 'run: loop {
            tokio::select! {
                f = c.recv_until(deadline) => match f {
                    Ok(Some(f)) => {
                        if let Some(i) = self.names.iter().position(|n| f.is_publish_of(n)) {
                            stats[i].on_message(f.at);
                        } else if let Some(i) = ids.iter().position(|id| f.error_for(id).is_some()) {
                            break 'run Err(subscribe_error(&self.names[i], f.error_for(&ids[i]).unwrap_or("")));
                        }
                    }
                    Ok(None) => break 'run Ok(()),
                    Err(e) => break 'run Err(e),
                },
                _ = tick.tick() => {
                    for (i, s) in stats.iter_mut().enumerate() {
                        let Some(r) = s.report() else { continue };
                        let text = if self.json {
                            r.json(conn::unix_now(), &self.names[i]).to_string()
                        } else if several {
                            format!("{}  {}", self.names[i], r.line())
                        } else {
                            r.line()
                        };
                        if let Err(e) = out.line(&text) {
                            break 'run Err(e);
                        }
                    }
                }
                _ = &mut sigint => break 'run Ok(()),
            }
        };
        c.close().await;
        outcome?;
        if self.duration.is_some() {
            let silent: Vec<&str> = self.names.iter().zip(&stats).filter(|(_, s)| s.messages < 2).map(|(n, _)| n.as_str()).collect();
            if !silent.is_empty() {
                return Err(Fail::timeout(format!("fewer than two messages, so no rate, on {}", silent.join(", "))));
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fake_bridge::{FakeBridge, FIX_FRAME};

    fn v(list: &[&str]) -> Vec<String> {
        list.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn echo_options_parse_and_conflicts_are_refused() {
        let e = Echo::parse(v(&["/fix", "-n", "3", "--throttle", "500", "--field", "status.status", "--url", "ws://h:1"])).unwrap();
        assert_eq!((e.name.as_str(), e.count, e.throttle_ms), ("/fix", Some(3), Some(500)));
        assert_eq!(e.field.as_deref(), Some("status.status"));
        assert_eq!(e.conn.url.as_deref(), Some("ws://h:1"));
        assert!(Echo::parse(v(&["/fix", "--raw", "--no-arr"])).is_err());
        assert!(Echo::parse(v(&[])).is_err());
        assert!(Echo::parse(v(&["/a", "/b"])).is_err(), "one topic");
        assert!(Echo::parse(v(&["/a", "--bogus"])).unwrap_err().msg.contains("unknown option --bogus"));
    }

    #[test]
    fn echo_renders_lines_fields_arrays_yaml_and_raw() {
        let text = r#"{"msg":{"data":[1,2],"header":{"frame_id":"f"}},"op":"publish","topic":"/t"}"#.to_string();
        let f = Frame::parse(text.clone(), 1761550000.5, Instant::now()).unwrap();
        let echo = |args: &[&str]| Echo::parse(v(args)).unwrap().render(&f);
        assert_eq!(echo(&["/t"]).unwrap(), r#"{"t":1761550000.5,"topic":"/t","msg":{"data":[1,2],"header":{"frame_id":"f"}}}"#);
        assert_eq!(echo(&["/t", "--no-arr"]).unwrap(), r#"{"t":1761550000.5,"topic":"/t","msg":{"data":"<2 items>","header":{"frame_id":"f"}}}"#);
        assert_eq!(echo(&["/t", "--field", "header.frame_id"]).unwrap(), r#""f""#);
        assert_eq!(echo(&["/t", "--field", "data.1"]).unwrap(), "2");
        assert_eq!(echo(&["/t", "--yaml", "--field", "header"]).unwrap(), "frame_id: f\n---");
        assert_eq!(echo(&["/t", "--raw"]).unwrap(), text);
        assert_eq!(echo(&["/t", "--field", "nope"]).unwrap_err().code, crate::EXIT_FAILED);
    }

    #[test]
    fn subcommands_and_their_arguments() {
        assert!(main(v(&[])).unwrap_err().msg.contains("list, info, echo or hz"));
        assert!(main(v(&["pub", "/x"])).unwrap_err().msg.contains("no such command"), "no topic pub (D2)");
        assert!(Info::parse(v(&[])).is_err());
        let l = List::parse(v(&["-v", "--json", "--timeout", "2"])).unwrap();
        assert!(l.types && l.json);
        assert_eq!(l.conn.timeout, Some(Duration::from_secs(2)));
        let h = HzCmd::parse(v(&["/a", "/b", "/a", "--window", "50", "--duration", "3", "--json"])).unwrap();
        assert_eq!(h.names, v(&["/a", "/b"]));
        assert_eq!((h.window, h.duration, h.json), (50, Some(Duration::from_secs(3)), true));
        assert!(HzCmd::parse(v(&["--json"])).is_err());
    }

    fn with_url(args: &[&str], url: &str) -> Vec<String> {
        let mut a = v(args);
        a.extend(v(&["--url", url]));
        a
    }

    #[test]
    fn list_and_info_read_rosapi_topics() {
        let b = FakeBridge::start(None);
        let list = |args: &[&str]| {
            let (mut out, buf) = Out::buffer();
            block_on(List::parse(with_url(args, &b.url)).unwrap().run(&mut out)).unwrap();
            let text = buf.lock().unwrap().clone();
            text
        };
        assert_eq!(list(&[]), "/fix\n/robot/online\n");
        assert_eq!(list(&["-v"]), "/fix           [sensor_msgs/msg/NavSatFix]\n/robot/online  [std_msgs/msg/Bool]\n");
        assert_eq!(list(&["--json"]), "{\"topics\":[\"/fix\",\"/robot/online\"],\"types\":[\"sensor_msgs/msg/NavSatFix\",\"std_msgs/msg/Bool\"]}\n");
        let (mut out, buf) = Out::buffer();
        block_on(Info::parse(with_url(&["/robot/online"], &b.url)).unwrap().run(&mut out)).unwrap();
        assert_eq!(*buf.lock().unwrap(), "Topic: /robot/online\nType: std_msgs/msg/Bool\n");
        let e = block_on(Info::parse(with_url(&["/odom"], &b.url)).unwrap().run(&mut Out::buffer().0)).unwrap_err();
        assert_eq!(e.code, crate::EXIT_MISSING);
        assert!(b.seen_ops("subscribe").is_empty(), "list and info only call /rosapi/topics");
    }

    #[test]
    fn echo_prints_json_lines_and_subscribes_without_a_type() {
        let b = FakeBridge::start(None);
        let (mut out, buf) = Out::buffer();
        block_on(Echo::parse(with_url(&["/robot/online", "-n", "3", "--throttle", "100"], &b.url)).unwrap().run(&mut out)).unwrap();
        let text = buf.lock().unwrap().clone();
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines.len(), 3, "{text}");
        for l in &lines {
            assert!(l.starts_with("{\"t\":"), "{l}");
            let d: Value = serde_json::from_str(l).unwrap();
            assert_eq!(d["topic"], "/robot/online");
            assert_eq!(d["msg"], json!({"data": true}));
            assert!(d["t"].as_f64().unwrap() > 1.7e9);
        }
        let subs = b.seen_ops("subscribe");
        assert_eq!(subs.len(), 1);
        assert_eq!(subs[0]["topic"], "/robot/online");
        assert_eq!(subs[0]["throttle_rate"], 100);
        assert!(subs[0].get("type").is_none(), "no type hint, so the bridge refuses an unknown name");
    }

    #[test]
    fn echo_keeps_the_frames_message_bytes() {
        let b = FakeBridge::start(None);
        let (mut out, buf) = Out::buffer();
        block_on(Echo::parse(with_url(&["/fix", "-n", "1"], &b.url)).unwrap().run(&mut out)).unwrap();
        let line = buf.lock().unwrap().trim_end().to_string();
        let start = FIX_FRAME.find("\"msg\":").unwrap() + 6;
        let end = FIX_FRAME.find(",\"op\":").unwrap();
        assert!(line.ends_with(&format!(",\"topic\":\"/fix\",\"msg\":{}}}", &FIX_FRAME[start..end])), "{line}");
        let (mut out, buf) = Out::buffer();
        block_on(Echo::parse(with_url(&["/fix", "--once", "--raw"], &b.url)).unwrap().run(&mut out)).unwrap();
        assert_eq!(buf.lock().unwrap().trim_end(), FIX_FRAME);
    }

    #[test]
    fn echo_and_hz_exit_codes() {
        let b = FakeBridge::start(None);
        let echo = |args: &[&str]| block_on(Echo::parse(with_url(args, &b.url)).unwrap().run(&mut Out::buffer().0));
        assert_eq!(echo(&["/odom"]).unwrap_err().code, crate::EXIT_MISSING);
        assert_eq!(echo(&["/odmo", "-n", "1"]).unwrap_err().code, crate::EXIT_MISSING);
        assert_eq!(echo(&["/quiet", "-n", "1", "--timeout", "0.3"]).unwrap_err().code, crate::EXIT_TIMEOUT);
        // --timeout without -n is a duration: fine once something came
        assert_eq!(echo(&["/robot/online", "--timeout", "0.3"]), Ok(()));
        let hz = |args: &[&str]| block_on(HzCmd::parse(with_url(args, &b.url)).unwrap().run(&mut Out::buffer().0));
        assert_eq!(hz(&["/quiet", "--duration", "0.3"]).unwrap_err().code, crate::EXIT_TIMEOUT);
        assert_eq!(hz(&["/robot/online", "/odom", "--duration", "0.3"]).unwrap_err().code, crate::EXIT_MISSING);
    }

    #[test]
    fn hz_reports_the_rate_like_ros2() {
        let b = FakeBridge::start(None);
        let (mut out, buf) = Out::buffer();
        // the fake sends 60 messages 25 ms apart
        block_on(HzCmd::parse(with_url(&["/robot/online", "--duration", "1.6", "--json"], &b.url)).unwrap().run(&mut out)).unwrap();
        let text = buf.lock().unwrap().clone();
        let reports: Vec<Value> = text.lines().map(|l| serde_json::from_str(l).unwrap()).collect();
        let r = reports.last().unwrap_or_else(|| panic!("no report: {text:?}"));
        assert_eq!(r["topic"], "/robot/online");
        assert!((r["rate_hz"].as_f64().unwrap() - 40.0).abs() < 8.0, "{text}");
        assert!(r["window"].as_u64().unwrap() >= 30, "{text}");
        assert!(r["min_s"].as_f64().unwrap() <= r["max_s"].as_f64().unwrap());
    }
}
