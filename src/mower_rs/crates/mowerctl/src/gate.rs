//! The pairing gate's hand-shake from the client side: the X-Mower-* headers
//! the app sends (mower_ws_bridge/src/auth.rs verifies them, as
//! tools/probe_app.py and mower_agent compute them). The MAC is HMAC-SHA256
//! over `robot_id\nclient\ntime\nnonce`, keyed with the base32 pairing
//! secret; the gate accepts a time within ±60 s of its own clock and each
//! nonce once.

use hmac::{Hmac, Mac};
use sha2::Sha256;

/// The client name the gate logs ("paired client mowerctl from ...").
pub const CLIENT: &str = "mowerctl";

#[derive(Debug, Clone)]
pub struct Pairing {
    pub robot_id: String,
    secret: Vec<u8>,
}

impl Pairing {
    /// A pairing file: `{"robot_id": "MW-XXXXXX", "secret": "<base32>"}`.
    /// The robot's own `~/.mower/identity.json` has the same two fields.
    pub fn load(path: &str) -> Result<Self, String> {
        let text = std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?;
        Self::parse(&text).map_err(|e| format!("{path}: {e}"))
    }

    pub fn parse(text: &str) -> Result<Self, String> {
        let doc: serde_json::Value = serde_json::from_str(text).map_err(|e| format!("not JSON: {e}"))?;
        let field = |k: &str| doc.get(k).and_then(|v| v.as_str()).map(str::trim).filter(|s| !s.is_empty());
        let robot_id = field("robot_id").ok_or("no \"robot_id\"")?.to_string();
        let secret = secret_bytes(field("secret").ok_or("no \"secret\"")?)?;
        Ok(Pairing { robot_id, secret })
    }

    pub fn mac(&self, client: &str, t: i64, nonce: &str) -> String {
        let mut mac = Hmac::<Sha256>::new_from_slice(&self.secret).expect("hmac accepts any key length");
        mac.update(format!("{}\n{}\n{}\n{}", self.robot_id, client, t, nonce).as_bytes());
        hex::encode(mac.finalize().into_bytes())
    }

    /// The five headers for one connection at unix time `t`.
    pub fn headers(&self, t: i64, nonce: &str) -> Vec<(&'static str, String)> {
        vec![
            ("X-Mower-Robot", self.robot_id.clone()),
            ("X-Mower-Client", CLIENT.to_string()),
            ("X-Mower-Time", t.to_string()),
            ("X-Mower-Nonce", nonce.to_string()),
            ("X-Mower-Mac", self.mac(CLIENT, t, nonce)),
        ]
    }
}

/// `identity.secret_bytes`: upper-case, pad to a multiple of 8, base32.
fn secret_bytes(secret: &str) -> Result<Vec<u8>, String> {
    let mut s = secret.trim().to_ascii_uppercase();
    while s.len() % 8 != 0 {
        s.push('=');
    }
    data_encoding::BASE32.decode(s.as_bytes()).map_err(|e| format!("secret is not base32: {e}"))
}

/// 16 random bytes as hex, the length the app and probe_app.py send.
pub fn nonce() -> Result<String, String> {
    use std::io::Read;
    let mut b = [0u8; 16];
    std::fs::File::open("/dev/urandom")
        .and_then(|mut f| f.read_exact(&mut b))
        .map_err(|e| format!("/dev/urandom: {e}"))?;
    Ok(hex::encode(b))
}

pub fn now_unix() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs() as i64)
        .unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_mac_matches_the_python_and_bridge_reference() {
        // mower_ws_bridge auth.rs: hmac.new(b"abcde", b"MW-ABC123\nphone-1\n1700000000\n0123456789abcdef", sha256)
        let p = Pairing::parse(r#"{"robot_id": "MW-ABC123", "secret": "mfrggzdf"}"#).unwrap();
        assert_eq!(p.mac("phone-1", 1_700_000_000, "0123456789abcdef"), "58cd0b71d4bc46a9f6d2c9873d7141680542cc4c50967a073f59eedc2b13be15");
        let h = p.headers(1_700_000_000, "0123456789abcdef");
        let names: Vec<&str> = h.iter().map(|(k, _)| *k).collect();
        assert_eq!(names, ["X-Mower-Robot", "X-Mower-Client", "X-Mower-Time", "X-Mower-Nonce", "X-Mower-Mac"]);
        assert_eq!(h[0].1, "MW-ABC123");
        assert_eq!(h[1].1, "mowerctl");
        assert_eq!(h[2].1, "1700000000");
        assert_eq!(h[4].1, p.mac("mowerctl", 1_700_000_000, "0123456789abcdef"));
    }

    #[test]
    fn an_identity_file_is_a_pairing_file_and_bad_ones_say_why() {
        // the robot's identity.json carries a name as well
        let p = Pairing::parse(r#"{"robot_id": "MW-E2TEST", "name": "x", "secret": "MUZC25TFOJUWM6JNNZXXILLBFVZGKYLMFVZWKY3SMV2CC"}"#).unwrap();
        assert_eq!(p.robot_id, "MW-E2TEST");
        assert!(Pairing::parse("{}").unwrap_err().contains("robot_id"));
        assert!(Pairing::parse(r#"{"robot_id": "MW-A"}"#).unwrap_err().contains("secret"));
        assert!(Pairing::parse(r#"{"robot_id": "MW-A", "secret": "not base32!"}"#).unwrap_err().contains("base32"));
        assert!(Pairing::parse("nope").unwrap_err().contains("JSON"));
    }

    #[test]
    fn nonces_are_32_hex_digits_and_differ() {
        let (a, b) = (nonce().unwrap(), nonce().unwrap());
        assert_eq!(a.len(), 32);
        assert!(a.chars().all(|c| c.is_ascii_hexdigit()));
        assert_ne!(a, b);
    }
}
