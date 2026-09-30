//! `mrelay1` framing between the robot and the backend hub (port of
//! `mower_mission/relay_protocol.py`, docs/BACKEND_ARCHITECTURE.md §5).
//!
//! Cloudflare caps one WebSocket message at 1 MiB while rosbridge map
//! messages run to tens of MB, so every rosbridge message is cut into chunks
//! here and glued back together on the phone (and the other way round). The
//! hub only adds or strips the 8-byte session id.
//!
//! ```text
//! robot <-> hub :  [type][sid: 8 bytes][payload]
//! app   <-> hub :  [type][payload]
//! ```
//!
//! type: 0x01 text (last chunk), 0x11 text (more follows),
//!       0x02 binary (last chunk), 0x12 binary (more follows).

pub const SUBPROTOCOL: &str = "mrelay1";
pub const SID_BYTES: usize = 8;
pub const T_TEXT: u8 = 0x01;
pub const T_TEXT_MORE: u8 = 0x11;
pub const T_BIN: u8 = 0x02;
pub const T_BIN_MORE: u8 = 0x12;
pub const MORE_BIT: u8 = 0x10;
pub const CHUNK_SIZE: usize = 512 * 1024;
pub const MAX_MESSAGE: usize = 64 * 1024 * 1024;

/// One rosbridge message on the local side.
#[derive(Clone, Debug, PartialEq)]
pub enum Message {
    Text(String),
    Binary(Vec<u8>),
}

/// `sid_bytes`: the 8-byte session id of its hex form.
pub fn sid_bytes(sid_hex: &str) -> Result<[u8; SID_BYTES], String> {
    let raw = hex::decode(sid_hex).map_err(|_| "bad session id".to_string())?;
    raw.try_into().map_err(|_| "bad session id".to_string())
}

/// `encode_frames`: robot->hub frames for one rosbridge message.
pub fn encode_frames(sid_hex: &str, message: &Message, chunk_size: usize) -> Result<Vec<Vec<u8>>, String> {
    let (payload, final_t, more_t): (&[u8], u8, u8) = match message {
        Message::Text(s) => (s.as_bytes(), T_TEXT, T_TEXT_MORE),
        Message::Binary(b) => (b.as_slice(), T_BIN, T_BIN_MORE),
    };
    let sid = sid_bytes(sid_hex)?;
    let mut out = Vec::new();
    if payload.is_empty() {
        let mut f = vec![final_t];
        f.extend_from_slice(&sid);
        out.push(f);
        return Ok(out);
    }
    let mut start = 0usize;
    while start < payload.len() {
        let end = (start + chunk_size).min(payload.len());
        let last = start + chunk_size >= payload.len();
        let mut f = vec![if last { final_t } else { more_t }];
        f.extend_from_slice(&sid);
        f.extend_from_slice(&payload[start..end]);
        out.push(f);
        start += chunk_size;
    }
    Ok(out)
}

/// `decode_frame`: (sid_hex, type, payload) of one hub->robot frame.
pub fn decode_frame(frame: &[u8]) -> Option<(String, u8, &[u8])> {
    if frame.len() < 1 + SID_BYTES || !matches!(frame[0], T_TEXT | T_TEXT_MORE | T_BIN | T_BIN_MORE) {
        return None;
    }
    Some((hex::encode(&frame[1..1 + SID_BYTES]), frame[0], &frame[1 + SID_BYTES..]))
}

/// Glue chunk payloads of one session back into rosbridge messages.
pub struct Reassembler {
    max: usize,
    parts: Vec<u8>,
}

impl Reassembler {
    pub fn new(max_message: usize) -> Self {
        Reassembler { max: max_message, parts: Vec::new() }
    }

    /// The complete message when this chunk finishes one, else `None`;
    /// `Err` when the message grows past the limit (state reset).
    pub fn feed(&mut self, frame_type: u8, payload: &[u8]) -> Result<Option<Message>, String> {
        if self.parts.len() + payload.len() > self.max {
            self.parts.clear();
            return Err("relayed message too large".into());
        }
        self.parts.extend_from_slice(payload);
        if frame_type & MORE_BIT != 0 {
            return Ok(None);
        }
        let data = std::mem::take(&mut self.parts);
        if frame_type == T_TEXT {
            Ok(Some(Message::Text(String::from_utf8_lossy(&data).into_owned())))
        } else {
            Ok(Some(Message::Binary(data)))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SID: &str = "0123456789abcdef";

    #[test]
    fn text_message_becomes_one_final_frame() {
        let frames = encode_frames(SID, &Message::Text("{\"op\":\"x\"}".into()), CHUNK_SIZE).unwrap();
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0][0], T_TEXT);
        assert_eq!(&frames[0][1..9], &sid_bytes(SID).unwrap());
        assert_eq!(&frames[0][9..], b"{\"op\":\"x\"}");
        let (sid, t, payload) = decode_frame(&frames[0]).unwrap();
        assert_eq!((sid.as_str(), t, payload), (SID, T_TEXT, &b"{\"op\":\"x\"}"[..]));
    }

    #[test]
    fn large_message_is_chunked_and_reassembled() {
        let text: String = "x".repeat(CHUNK_SIZE * 2 + 5);
        let frames = encode_frames(SID, &Message::Text(text.clone()), CHUNK_SIZE).unwrap();
        assert_eq!(frames.len(), 3);
        assert_eq!(frames[0][0], T_TEXT_MORE);
        assert_eq!(frames[1][0], T_TEXT_MORE);
        assert_eq!(frames[2][0], T_TEXT);
        assert_eq!(frames[2].len(), 1 + SID_BYTES + 5);
        let mut r = Reassembler::new(MAX_MESSAGE);
        let mut out = None;
        for f in &frames {
            let (_, t, p) = decode_frame(f).unwrap();
            out = r.feed(t, p).unwrap();
        }
        assert_eq!(out, Some(Message::Text(text)));
    }

    #[test]
    fn binary_and_empty_messages() {
        let frames = encode_frames(SID, &Message::Binary(vec![1, 2, 3]), 2).unwrap();
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0][0], T_BIN_MORE);
        assert_eq!(frames[1][0], T_BIN);
        let empty = encode_frames(SID, &Message::Text(String::new()), CHUNK_SIZE).unwrap();
        assert_eq!(empty, vec![{
            let mut f = vec![T_TEXT];
            f.extend_from_slice(&sid_bytes(SID).unwrap());
            f
        }]);
        let mut r = Reassembler::new(MAX_MESSAGE);
        assert_eq!(r.feed(T_BIN, &[]).unwrap(), Some(Message::Binary(Vec::new())));
    }

    #[test]
    fn malformed_frames_and_oversize() {
        assert!(decode_frame(&[T_TEXT, 1, 2]).is_none());
        assert!(decode_frame(&[0x03, 0, 0, 0, 0, 0, 0, 0, 0, 1]).is_none());
        assert!(sid_bytes("zz").is_err());
        assert!(sid_bytes("0011").is_err());
        let mut r = Reassembler::new(10);
        assert!(r.feed(T_TEXT_MORE, &[0; 6]).unwrap().is_none());
        assert_eq!(r.feed(T_TEXT, &[0; 6]).unwrap_err(), "relayed message too large");
        // state was reset
        assert_eq!(r.feed(T_TEXT, b"ok").unwrap(), Some(Message::Text("ok".into())));
    }
}
