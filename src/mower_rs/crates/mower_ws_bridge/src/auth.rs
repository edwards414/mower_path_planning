//! Pairing gate (port of `mower_mission/identity.py` verify_mac + NonceCache).
//!
//! Every connection on the public listener must carry `X-Mower-Robot`,
//! `X-Mower-Client`, `X-Mower-Time`, `X-Mower-Nonce` and `X-Mower-Mac`, where
//! the MAC is HMAC-SHA256 over `robot_id\nclient\ntime\nnonce` keyed with the
//! base32 pairing secret from `identity.json`. Time skew is limited to
//! ±60 s and a nonce is accepted once within the skew window.

use std::collections::HashMap;
use std::time::{SystemTime, UNIX_EPOCH};

use hmac::{Hmac, Mac};
use sha2::Sha256;

pub const MAC_MAX_SKEW_S: i64 = 60;
const HEADER_PREFIX: &str = "x-mower-";

#[derive(Debug, Clone)]
pub struct Identity {
    pub robot_id: String,
    pub name: Option<String>,
    secret: Vec<u8>,
}

impl Identity {
    /// `identity.json` from the state dir, or `None` (development robot).
    pub fn load(state_dir: &str) -> Result<Option<Self>, String> {
        let Some(doc) = mower_rs_common::read_state_json(state_dir, "identity.json") else {
            return Ok(None);
        };
        let robot_id = doc.get("robot_id").and_then(|v| v.as_str()).unwrap_or("").to_string();
        let valid = robot_id.len() == 9
            && robot_id.starts_with("MW-")
            && robot_id[3..].chars().all(|c| c.is_ascii_uppercase() || c.is_ascii_digit());
        let secret = doc.get("secret").and_then(|v| v.as_str()).unwrap_or("");
        if !valid || secret.is_empty() {
            return Ok(None);
        }
        Ok(Some(Self {
            robot_id,
            name: doc.get("name").and_then(|v| v.as_str()).map(str::to_string),
            secret: secret_bytes(secret)?,
        }))
    }

    pub fn compute_mac(&self, client: &str, t: i64, nonce: &str) -> String {
        let mut mac = Hmac::<Sha256>::new_from_slice(&self.secret).expect("hmac accepts any key length");
        mac.update(format!("{}\n{}\n{}\n{}", self.robot_id, client, t, nonce).as_bytes());
        hex::encode(mac.finalize().into_bytes())
    }
}

/// `identity.secret_bytes`: upper-case, pad to a multiple of 8, base32 decode.
fn secret_bytes(secret: &str) -> Result<Vec<u8>, String> {
    let mut s = secret.trim().to_ascii_uppercase();
    while s.len() % 8 != 0 {
        s.push('=');
    }
    data_encoding::BASE32.decode(s.as_bytes()).map_err(|e| format!("identity secret is not base32: {e}"))
}

/// Remembers nonces for the skew window so a captured hand-shake cannot be
/// replayed.
#[derive(Default)]
pub struct NonceCache {
    seen: HashMap<String, i64>,
}

impl NonceCache {
    pub fn check_and_add(&mut self, nonce: &str, now: i64) -> bool {
        let ttl = 2 * MAC_MAX_SKEW_S;
        self.seen.retain(|_, at| now - *at <= ttl);
        if self.seen.contains_key(nonce) {
            return false;
        }
        self.seen.insert(nonce.to_string(), now);
        true
    }
}

pub fn now_unix() -> i64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs() as i64).unwrap_or(0)
}

/// Validate the pairing headers of one connection. `headers` are
/// (lower-cased name, value) pairs. Returns the client id or the reason.
pub fn verify_mac(
    identity: &Identity,
    headers: &[(String, String)],
    nonces: &mut NonceCache,
    now: i64,
) -> Result<String, String> {
    let get = |name: &str| {
        let key = format!("{HEADER_PREFIX}{name}");
        headers
            .iter()
            .find(|(k, _)| k.eq_ignore_ascii_case(&key))
            .map(|(_, v)| v.trim().to_string())
            .unwrap_or_default()
    };
    let (robot_id, client, t_raw, nonce, mac) = (get("robot"), get("client"), get("time"), get("nonce"), get("mac"));
    if robot_id.is_empty() || client.is_empty() || t_raw.is_empty() || nonce.is_empty() || mac.is_empty() {
        return Err("missing pairing headers".into());
    }
    if robot_id != identity.robot_id {
        return Err(format!("wrong robot ({robot_id})"));
    }
    let t: i64 = t_raw.parse().map_err(|_| "bad time".to_string())?;
    if (now - t).abs() > MAC_MAX_SKEW_S {
        return Err(format!("clock skew {} s", now - t));
    }
    if !(16..=64).contains(&nonce.len()) || !nonce.chars().all(|c| c.is_ascii_hexdigit()) {
        return Err("bad nonce".into());
    }
    let expected = identity.compute_mac(&client, t, &nonce);
    if !constant_time_eq(expected.as_bytes(), mac.to_ascii_lowercase().as_bytes()) {
        return Err("bad mac".into());
    }
    if !nonces.check_and_add(&nonce.to_ascii_lowercase(), now) {
        return Err("replayed nonce".into());
    }
    Ok(client)
}

fn constant_time_eq(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    a.iter().zip(b).fold(0u8, |acc, (x, y)| acc | (x ^ y)) == 0
}

#[cfg(test)]
mod tests {
    use super::*;

    fn identity() -> Identity {
        // secret "MFRGGZDF" = base32("abcde")
        Identity { robot_id: "MW-ABC123".into(), name: None, secret: secret_bytes("mfrggzdf").unwrap() }
    }

    fn headers(id: &Identity, client: &str, t: i64, nonce: &str, mac: Option<&str>) -> Vec<(String, String)> {
        let mac = mac.map(str::to_string).unwrap_or_else(|| id.compute_mac(client, t, nonce));
        vec![
            ("x-mower-robot".into(), id.robot_id.clone()),
            ("X-Mower-Client".into(), client.into()),
            ("x-mower-time".into(), t.to_string()),
            ("x-mower-nonce".into(), nonce.into()),
            ("x-mower-mac".into(), mac),
        ]
    }

    #[test]
    fn secret_decodes_like_python() {
        assert_eq!(secret_bytes("mfrggzdf").unwrap(), b"abcde");
        assert_eq!(secret_bytes("MFRGGZDF").unwrap(), b"abcde");
        assert!(secret_bytes("not base32!").is_err());
    }

    #[test]
    fn mac_matches_the_python_reference() {
        // hmac.new(b"abcde", b"MW-ABC123\nphone-1\n1700000000\n0123456789abcdef", sha256).hexdigest()
        let id = identity();
        assert_eq!(
            id.compute_mac("phone-1", 1_700_000_000, "0123456789abcdef"),
            "58cd0b71d4bc46a9f6d2c9873d7141680542cc4c50967a073f59eedc2b13be15".to_string()
        );
    }

    #[test]
    fn verify_accepts_once_and_rejects_replays_skew_and_bad_macs() {
        let id = identity();
        let mut nonces = NonceCache::default();
        let now = 1_700_000_000;
        let h = headers(&id, "phone-1", now - 30, "0123456789abcdef", None);
        assert_eq!(verify_mac(&id, &h, &mut nonces, now), Ok("phone-1".into()));
        assert_eq!(verify_mac(&id, &h, &mut nonces, now), Err("replayed nonce".into()));
        let h = headers(&id, "phone-1", now - 61, "0123456789abcdef01", None);
        assert!(verify_mac(&id, &h, &mut nonces, now).unwrap_err().starts_with("clock skew"));
        let h = headers(&id, "phone-1", now, "0123456789abcdef02", Some("00"));
        assert_eq!(verify_mac(&id, &h, &mut nonces, now), Err("bad mac".into()));
        let mut h = headers(&id, "phone-1", now, "0123456789abcdef03", None);
        h[0].1 = "MW-OTHER1".into();
        assert!(verify_mac(&id, &h, &mut nonces, now).unwrap_err().starts_with("wrong robot"));
        let h = headers(&id, "phone-1", now, "zz", None);
        assert_eq!(verify_mac(&id, &h, &mut nonces, now), Err("bad nonce".into()));
        assert_eq!(verify_mac(&id, &[], &mut nonces, now), Err("missing pairing headers".into()));
        // upper-case MAC hex is accepted (compare_digest on mac.lower())
        let mut h = headers(&id, "phone-1", now, "0123456789abcdef04", None);
        h[4].1 = h[4].1.to_ascii_uppercase();
        assert_eq!(verify_mac(&id, &h, &mut nonces, now), Ok("phone-1".into()));
    }
}
