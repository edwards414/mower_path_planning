//! `mowerctl ping`: `/rosapi/get_time` round trips, as the studio measures
//! its link (every 3 s), and how far the robot's clock is from this host's
//! (the pairing gate refuses a client more than 60 s off).

use std::time::{Duration, Instant};

use serde_json::{json, Value};

use crate::cli::{self, ConnOpts, DEFAULT_TIMEOUT};
use crate::conn::{unix_now, Conn};
use crate::format::Out;
use crate::{block_on, Fail};

const GATE_SKEW_S: f64 = 60.0;

#[derive(Debug)]
struct Ping {
    conn: ConnOpts,
    count: u64,
    interval: Duration,
}

pub fn main(args: Vec<String>) -> Result<(), Fail> {
    let mut p = Ping { conn: ConnOpts::default(), count: 4, interval: Duration::from_secs(1) };
    let mut it = args.into_iter();
    while let Some(a) = it.next() {
        match a.as_str() {
            "-c" | "--count" => p.count = cli::count(&a, &cli::value(&a, &mut it)?)?,
            "-i" | "--interval" => p.interval = cli::seconds(&a, &cli::value(&a, &mut it)?)?,
            _ if p.conn.take(&a, &mut it)? => {}
            _ => return Err(cli::unknown("ping", &a)),
        }
    }
    block_on(p.run(&mut Out::stdout()))
}

/// `{"time": {"sec", "nanosec"}}` as unix seconds.
fn robot_time(values: &Value) -> Option<f64> {
    let t = values.get("time")?;
    Some(t.get("sec")?.as_f64()? + t.get("nanosec")?.as_f64()? * 1e-9)
}

/// The robot's clock minus this host's, from one round trip: the answer is
/// taken to be stamped half way through it.
pub fn offset_s(sent_unix: f64, rtt_s: f64, robot_unix: f64) -> f64 {
    robot_unix - (sent_unix + rtt_s / 2.0)
}

impl Ping {
    async fn run(self, out: &mut Out) -> Result<(), Fail> {
        let timeout = self.conn.timeout_or(DEFAULT_TIMEOUT);
        let mut c = Conn::open(&self.conn).await?;
        let (mut rtts, mut offsets) = (Vec::new(), Vec::new());
        for i in 0..self.count {
            if i > 0 {
                tokio::time::sleep(self.interval).await;
            }
            let (sent_unix, sent) = (unix_now(), Instant::now());
            let values = c.call_values("/rosapi/get_time", json!({}), timeout).await?;
            let rtt = sent.elapsed().as_secs_f64();
            let robot = robot_time(&values).ok_or_else(|| Fail::failed(format!("/rosapi/get_time answered {values}")))?;
            let offset = offset_s(sent_unix, rtt, robot);
            out.line(&format!("{}: rtt {:.2} ms, robot clock {:+.3} s", c.url, rtt * 1e3, offset))?;
            rtts.push(rtt);
            offsets.push(offset);
        }
        c.close().await;
        let n = rtts.len() as f64;
        let (min, max) = rtts.iter().fold((f64::INFINITY, 0.0f64), |(lo, hi), r| (lo.min(*r), hi.max(*r)));
        let offset = offsets.iter().sum::<f64>() / n;
        out.line(&format!(
            "{} answers, rtt min/avg/max {:.2}/{:.2}/{:.2} ms, robot clock {:+.3} s from this host",
            rtts.len(),
            min * 1e3,
            rtts.iter().sum::<f64>() / n * 1e3,
            max * 1e3,
            offset
        ))?;
        if offset.abs() > GATE_SKEW_S {
            eprintln!("mowerctl: the clocks are more than {GATE_SKEW_S} s apart: the pairing gate refuses clients of this host");
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_robot_time_and_the_offset() {
        let v = json!({"time": {"sec": 1761550000, "nanosec": 500_000_000}});
        assert_eq!(robot_time(&v), Some(1761550000.5));
        assert_eq!(robot_time(&json!({"time": {"sec": 1}})), None);
        // sent at 100.0, answered after 0.2 s, robot said 100.6: 0.5 s ahead
        assert!((offset_s(100.0, 0.2, 100.6) - 0.5).abs() < 1e-12);
    }

    #[test]
    fn ping_reports_each_round_trip_and_the_offset() {
        let b = crate::fake_bridge::FakeBridge::start(None);
        let (mut out, buf) = Out::buffer();
        let p = Ping { conn: ConnOpts { url: Some(b.url.clone()), ..Default::default() }, count: 2, interval: Duration::from_millis(50) };
        block_on(p.run(&mut out)).unwrap();
        let text = buf.lock().unwrap().clone();
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines.len(), 3, "{text}");
        assert!(lines[0].starts_with(&format!("{}: rtt ", b.url)), "{text}");
        assert!(lines[2].starts_with("2 answers, rtt min/avg/max "), "{text}");
        // the fake answers with this host's clock
        let offset: f64 = lines[2].split("robot clock ").nth(1).unwrap().split(' ').next().unwrap().parse().unwrap();
        assert!(offset.abs() < 0.5, "{text}");
    }

    #[test]
    fn options() {
        let v = |l: &[&str]| l.iter().map(|s| s.to_string()).collect::<Vec<_>>();
        assert!(main(v(&["-c", "0"])).is_err());
        assert!(main(v(&["--bogus"])).unwrap_err().msg.contains("unknown option"));
    }
}
