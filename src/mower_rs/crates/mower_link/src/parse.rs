//! The modem's answers, `/proc/net/wireless` and `ip -j -4 addr`, read the way
//! `deploy/host/mower-link-status.py` reads them. The patterns are its own,
//! with Python's `\s` spelled out and `\d` as ASCII digits (a modem does not
//! answer in other scripts; Python would take them).

use std::sync::OnceLock;

use mower_pyfmt::{json, Py};
use regex::Regex;

use crate::{py, Raised};

/// Python's `\s` on a `str`: `str.isspace()`.
const S: &str = r"[\t\n\x0b\x0c\r\x1c-\x1f \x{85}\x{a0}\x{1680}\x{2000}-\x{200a}\x{2028}\x{2029}\x{202f}\x{205f}\x{3000}]*";

macro_rules! pattern {
    ($p:expr) => {{
        static RE: OnceLock<Regex> = OnceLock::new();
        RE.get_or_init(|| Regex::new(&$p.replace("{S}", S)).expect("the pattern compiles"))
    }};
}

#[derive(Debug, Clone, PartialEq)]
pub struct Csq {
    pub csq: i64,
    pub ber: i64,
    pub rssi_dbm: Option<i64>,
}

/// `+CSQ: 15,99`; 3GPP 27.007: 0 = -113 dBm, 31 = -51 dBm, 99 = unknown.
/// Python's ints have no limit; a number outside i64 here reads as no answer.
pub fn csq(text: &str) -> Option<Csq> {
    let m = pattern!(r"\+CSQ:{S}([0-9]+),([0-9]+)").captures(text)?;
    let csq: i64 = m[1].parse().ok()?;
    let ber: i64 = m[2].parse().ok()?;
    let rssi_dbm = if csq == 99 { None } else { Some(csq.checked_mul(2)?.checked_sub(113)?) };
    Some(Csq { csq, ber, rssi_dbm })
}

/// `+CREG: 0,1`: registered is home (1) or roaming (5), as the digits stand.
pub fn creg(text: &str) -> Option<bool> {
    let m = pattern!(r"\+CREG:{S}[0-9]+,([0-9]+)").captures(text)?;
    Some(matches!(&m[1], "1" | "5"))
}

#[derive(Debug, Clone, PartialEq)]
pub struct Cops {
    pub operator: String,
    pub tech: Option<String>,
}

/// `+COPS: 0,0,"Chunghwa",7`: a technology other than 0, 2, 7 stays its digits.
pub fn cops(text: &str) -> Option<Cops> {
    let m = pattern!(r#"\+COPS:{S}[0-9]+,[0-9]+,"([^"]*)",?([0-9]+)?"#).captures(text)?;
    let tech = m.get(2).map(|t| {
        match t.as_str() {
            "0" => "GSM",
            "2" => "UTRAN",
            "7" => "LTE",
            other => other,
        }
        .to_string()
    });
    Some(Cops { operator: m[1].to_string(), tech })
}

#[derive(Debug, Clone, PartialEq)]
pub struct Qnwinfo {
    pub band: String,
    pub plmn: String,
}

/// Quectel: `+QNWINFO: "FDD LTE","46692","LTE BAND 7",3050`.
pub fn qnwinfo(text: &str) -> Option<Qnwinfo> {
    let m = pattern!(r#"\+QNWINFO:{S}"([^"]*)","([0-9]*)","([^"]*)",([0-9]+)"#).captures(text)?;
    Some(Qnwinfo { band: m[3].to_string(), plmn: m[2].to_string() })
}

#[derive(Debug, Clone, PartialEq)]
pub struct Qcsq {
    pub rsrp_dbm: i64,
    pub sinr_db: f64,
    pub rsrq_db: i64,
}

/// Quectel: `+QCSQ: "LTE",<rssi>,<rsrp>,<sinr>,<rsrq>`; sinr is value/5 - 20 dB.
/// rsrp and rsrq are ints here and floats from +CPSI: part of the contract.
pub fn qcsq(text: &str) -> Option<Qcsq> {
    let m = pattern!(r#"\+QCSQ:{S}"LTE",(-?[0-9]+),(-?[0-9]+),(-?[0-9]+),(-?[0-9]+)"#).captures(text)?;
    let raw_sinr: i64 = m[3].parse().ok()?;
    Some(Qcsq {
        rsrp_dbm: m[2].parse().ok()?,
        sinr_db: py::round1(raw_sinr as f64 / 5.0 - 20.0),
        rsrq_db: m[4].parse().ok()?,
    })
}

#[derive(Debug, Clone, PartialEq)]
pub struct Cpsi {
    pub band: String,
    pub plmn: String,
    pub rsrq_db: f64,
    pub rsrp_dbm: f64,
    pub sinr_db: f64,
}

/// SIMCom (SIM7600):
/// `+CPSI: LTE,Online,466-92,0x639D,134534710,430,EUTRAN-BAND7,3050,5,5,-103,-998,-722,13`.
/// After the band: EARFCN, DL/UL bandwidth, then RSRQ, RSRP and RSSI in 0.1
/// dB(m) units and RSSNR in dB.
pub fn cpsi(text: &str) -> Option<Cpsi> {
    let m = pattern!(
        r"\+CPSI:{S}LTE,[^,]*,([0-9]+-[0-9]+),[^,]*,[^,]*,[^,]*,([^,]*),-?[0-9]+,-?[0-9]+,-?[0-9]+,(-?[0-9]+),(-?[0-9]+),(-?[0-9]+),(-?[0-9]+)"
    )
    .captures(text)?;
    let tenths = |s: &str| -> Option<f64> { Some(py::round1(s.parse::<i64>().ok()? as f64 / 10.0)) };
    Some(Cpsi {
        band: m[2].to_string(),
        plmn: m[1].replace('-', ""),
        rsrq_db: tenths(&m[3])?,
        rsrp_dbm: tenths(&m[4])?,
        sinr_db: py::float(&m[6])?,
    })
}

/// One row of `/proc/net/wireless`.
#[derive(Debug, Clone, PartialEq)]
pub struct Wifi {
    pub iface: String,
    pub link: f64,
    pub rssi_dbm: f64,
}

/// The rows after the two header lines. A short row is skipped; a number that
/// does not parse raises, in Python's `except OSError` as here: nothing gets
/// written.
pub fn wireless(text: &str) -> Result<Vec<Wifi>, Raised> {
    let text = py::universal_newlines(text);
    let mut out = Vec::new();
    for line in text.split_inclusive('\n').skip(2) {
        let parts: Vec<&str> = py::split(line).collect();
        if parts.len() < 4 {
            continue;
        }
        let number = |s: &str| {
            let s = s.trim_end_matches('.');
            py::float(s).ok_or_else(|| Raised(format!("could not convert string to float: {}", py::repr_str(s))))
        };
        out.push(Wifi {
            iface: parts[0].trim_end_matches(':').to_string(),
            link: number(parts[2])?,
            rssi_dbm: number(parts[3])?,
        });
    }
    Ok(out)
}

/// One interface of `ip -j -4 addr`.
#[derive(Debug, Clone, PartialEq)]
pub struct Iface {
    pub name: String,
    pub up: bool,
    /// The first `inet` address's `local`, as JSON gave it; `Py::None` without one.
    pub addr: Py,
}

/// Interfaces left out, by prefix (so `lo` also takes `loop1`).
pub const SKIP_IFACES: [&str; 5] = ["lo", "docker", "veth", "br-", "dummy"];

fn raised_get(what: &str) -> Raised {
    Raised(format!("{what} has no attribute 'get'"))
}

/// What iterating a JSON value gives: a list its items; an empty dict or string
/// nothing; a dict's keys and a string's characters are strings, and Python's
/// `.get` on those raises; a number, a bool or null do not iterate.
fn items_of(v: &Py) -> Result<&[Py], Raised> {
    match v {
        Py::List(items) => Ok(items),
        Py::Dict(d) if d.is_empty() => Ok(&[]),
        Py::Str(s) if s.is_empty() => Ok(&[]),
        Py::Dict(_) | Py::Str(_) => Err(raised_get("'str' object")),
        other => Err(Raised(format!("'{}' object is not iterable", other.type_name()))),
    }
}

/// `ip`'s stdout. What Python catches (`OSError`, `ValueError`,
/// `SubprocessError`) is an empty list: no `ip`, a timeout, not UTF-8, not JSON;
/// JSON of another shape raises past that `except`, and nothing gets written.
pub fn ifaces(stdout: Option<&[u8]>) -> Result<Vec<Iface>, Raised> {
    let Some(stdout) = stdout else {
        return Ok(Vec::new());
    };
    let Ok(text) = std::str::from_utf8(stdout) else {
        return Ok(Vec::new());
    };
    let doc = match json::loads(if text.is_empty() { "[]" } else { text }) {
        Ok(doc) => doc,
        Err(_) => return Ok(Vec::new()),
    };
    let mut out = Vec::new();
    for it in items_of(&doc)? {
        if !matches!(it, Py::Dict(_)) {
            return Err(raised_get(&format!("'{}' object", it.type_name())));
        }
        let name = match it.get("ifname") {
            None => "",
            Some(Py::Str(s)) => s.as_str(),
            Some(other) => return Err(Raised(format!("'{}' object has no attribute 'startswith'", other.type_name()))),
        };
        if SKIP_IFACES.iter().any(|p| name.starts_with(p)) {
            continue;
        }
        let mut addr = None;
        if let Some(info) = it.get("addr_info") {
            for a in items_of(info)? {
                if !matches!(a, Py::Dict(_)) {
                    return Err(raised_get(&format!("'{}' object", a.type_name())));
                }
                if addr.is_none() && matches!(a.get("family"), Some(Py::Str(f)) if f == "inet") {
                    addr = Some(a.get("local").cloned().unwrap_or(Py::None));
                }
            }
        }
        let up = matches!(it.get("operstate"), Some(Py::Str(s)) if s == "UP" || s == "UNKNOWN");
        out.push(Iface { name: name.to_string(), up, addr: addr.unwrap_or(Py::None) });
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_simcom_reply_captured_on_the_robot() {
        // test_lte_uplink.py's reply, 2026-10-07
        let reply = "AT+CPSI?\r\n+CPSI: LTE,Online,466-92,0x639D,134534710,430,EUTRAN-BAND7,3050,5,5,-103,-998,-722,13\r\n\r\nOK\r\n";
        assert_eq!(
            cpsi(reply),
            Some(Cpsi { band: "EUTRAN-BAND7".into(), plmn: "46692".into(), rsrq_db: -10.3, rsrp_dbm: -99.8, sinr_db: 13.0 })
        );
        assert_eq!(cpsi("+CPSI: NO SERVICE,Online\r\nOK"), None);
        assert_eq!(cpsi("ERROR"), None);
    }

    #[test]
    fn whitespace_after_the_colon_is_pythons() {
        assert_eq!(creg("+CREG:\u{1c}\u{3000} 0,5"), Some(true));
        assert_eq!(creg("+CREG:\u{200b}0,5"), None, "U+200B is not whitespace");
    }

    #[test]
    fn numbers_too_long_for_i64_read_as_no_answer() {
        assert_eq!(csq("+CSQ: 99999999999999999999,0"), None);
        assert_eq!(csq("+CSQ: 9223372036854775807,0"), None);
    }

    #[test]
    fn only_a_digit_answers() {
        assert_eq!(creg("+CREG: ٠,١"), None);
    }
}
