//! Command-line parsing: the connection options every bridge command shares,
//! and small helpers for the rest. Hand-rolled like the crates' other
//! parsers (mower_ws_bridge's parse_args); `--flag=value` is accepted too.

use std::time::Duration;

use crate::Fail;

/// ws_bridge's loopback listener: no gate, bound to 127.0.0.1 only.
pub const LOOPBACK_URL: &str = "ws://127.0.0.1:9091";
/// The pairing gate on the robot itself (ws_bridge binds 127.0.0.1:9090
/// whenever its public address is a LAN or VPN one).
pub const GATE_URL: &str = "ws://127.0.0.1:9090";
/// Default answer timeout.
pub const DEFAULT_TIMEOUT: Duration = Duration::from_secs(5);
/// A service call waits longer: ws_bridge itself gives up after 30 s.
pub const CALL_TIMEOUT: Duration = Duration::from_secs(35);

/// Options that pick and open the connection.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ConnOpts {
    pub url: Option<String>,
    pub pair: Option<String>,
    pub timeout: Option<Duration>,
}

impl ConnOpts {
    /// `--url`, `--pair` or `--timeout` with its value: Ok(true) when `flag`
    /// was one of them.
    pub fn take(&mut self, flag: &str, it: &mut impl Iterator<Item = String>) -> Result<bool, Fail> {
        match flag {
            "--url" => self.url = Some(value(flag, it)?),
            "--pair" => self.pair = Some(value(flag, it)?),
            "--timeout" => self.timeout = Some(seconds(flag, &value(flag, it)?)?),
            _ => return Ok(false),
        }
        Ok(true)
    }

    /// The URL to connect to: `--url`, else `$MOWERCTL_URL`, else the gate
    /// with `--pair` and the loopback listener without.
    pub fn url(&self) -> String {
        self.url_with(std::env::var("MOWERCTL_URL").ok())
    }

    fn url_with(&self, env: Option<String>) -> String {
        if let Some(u) = &self.url {
            return u.clone();
        }
        if let Some(u) = env.filter(|u| !u.is_empty()) {
            return u;
        }
        if self.pair.is_some() { GATE_URL } else { LOOPBACK_URL }.to_string()
    }

    pub fn timeout_or(&self, default: Duration) -> Duration {
        self.timeout.unwrap_or(default)
    }
}

/// The value after `flag`.
pub fn value(flag: &str, it: &mut impl Iterator<Item = String>) -> Result<String, Fail> {
    it.next().ok_or_else(|| Fail::usage(format!("{flag} needs a value")))
}

/// A positive number of seconds.
pub fn seconds(flag: &str, text: &str) -> Result<Duration, Fail> {
    match text.parse::<f64>() {
        Ok(s) if s.is_finite() && s > 0.0 && s < 1e9 => Ok(Duration::from_secs_f64(s)),
        _ => Err(Fail::usage(format!("{flag} wants a positive number of seconds, not {text:?}"))),
    }
}

/// A positive integer.
pub fn count(flag: &str, text: &str) -> Result<u64, Fail> {
    match text.parse::<u64>() {
        Ok(n) if n > 0 => Ok(n),
        _ => Err(Fail::usage(format!("{flag} wants a positive integer, not {text:?}"))),
    }
}

/// `--flag=value` -> `--flag value`, so every parser sees one shape.
pub fn expand_equals(argv: Vec<String>) -> Vec<String> {
    let mut out = Vec::with_capacity(argv.len());
    for a in argv {
        match a.strip_prefix("--").and_then(|rest| rest.split_once('=')) {
            Some((name, v)) if !name.is_empty() => {
                out.push(format!("--{name}"));
                out.push(v.to_string());
            }
            _ => out.push(a),
        }
    }
    out
}

/// The connection options in front of the command word, and the rest.
pub fn split_leading_options(argv: &[String]) -> Result<(Vec<String>, Vec<String>), Fail> {
    let mut lead = Vec::new();
    let mut i = 0;
    while i < argv.len() && matches!(argv[i].as_str(), "--url" | "--pair" | "--timeout") {
        let v = argv.get(i + 1).ok_or_else(|| Fail::usage(format!("{} needs a value", argv[i])))?;
        lead.push(argv[i].clone());
        lead.push(v.clone());
        i += 2;
    }
    Ok((lead, argv[i..].to_vec()))
}

/// An argument that is not an option of the command.
pub fn unknown(command: &str, arg: &str) -> Fail {
    if arg.starts_with('-') {
        Fail::usage(format!("{command}: unknown option {arg}"))
    } else {
        Fail::usage(format!("{command}: unexpected argument {arg:?}"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v(list: &[&str]) -> Vec<String> {
        list.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn equals_forms_are_split_and_the_rest_left_alone() {
        assert_eq!(
            expand_equals(v(&["--url=ws://h:1/x=y", "-n", "3", "topic", "{\"a\":\"b=c\"}", "--", "--=x"])),
            v(&["--url", "ws://h:1/x=y", "-n", "3", "topic", "{\"a\":\"b=c\"}", "--", "--=x"])
        );
    }

    #[test]
    fn leading_connection_options_are_split_off() {
        let (lead, rest) = split_leading_options(&v(&["--pair", "p.json", "--timeout", "2", "topic", "list", "--url", "u"])).unwrap();
        assert_eq!(lead, v(&["--pair", "p.json", "--timeout", "2"]));
        assert_eq!(rest, v(&["topic", "list", "--url", "u"]));
        let (lead, rest) = split_leading_options(&v(&["--help"])).unwrap();
        assert!(lead.is_empty());
        assert_eq!(rest, v(&["--help"]));
        assert!(split_leading_options(&v(&["--url"])).is_err());
    }

    #[test]
    fn the_url_defaults_to_the_loopback_listener_or_with_a_pairing_to_the_gate() {
        let mut o = ConnOpts::default();
        assert_eq!(o.url_with(None), LOOPBACK_URL);
        assert_eq!(o.url_with(Some(String::new())), LOOPBACK_URL);
        assert_eq!(o.url_with(Some("ws://127.0.0.1:19091".into())), "ws://127.0.0.1:19091");
        o.pair = Some("pair.json".into());
        assert_eq!(o.url_with(None), GATE_URL);
        o.url = Some("ws://10.0.0.5:9090".into());
        assert_eq!(o.url_with(Some("ws://ignored".into())), "ws://10.0.0.5:9090");
    }

    #[test]
    fn connection_options_are_taken_with_their_values() {
        let mut o = ConnOpts::default();
        let mut it = v(&["ws://a:1", "1.5", "f.json"]).into_iter();
        assert_eq!(o.take("--url", &mut it), Ok(true));
        assert_eq!(o.take("--timeout", &mut it), Ok(true));
        assert_eq!(o.take("--pair", &mut it), Ok(true));
        assert_eq!(o.take("-n", &mut it), Ok(false));
        assert_eq!(o, ConnOpts { url: Some("ws://a:1".into()), pair: Some("f.json".into()), timeout: Some(Duration::from_millis(1500)) });
        assert!(o.take("--url", &mut it).is_err(), "a missing value");
        assert!(seconds("--timeout", "0").is_err());
        assert!(seconds("--timeout", "-1").is_err());
        assert!(seconds("--timeout", "nan").is_err());
        assert!(count("-n", "0").is_err());
        assert_eq!(count("-n", "7"), Ok(7));
    }
}
