//! Serial record/replay log.
//!
//! A length-prefixed binary log of `(monotonic ns, direction, bytes)`. It is
//! deliberately dumber than a rosbag: a capture can be produced by anything
//! that can see the byte stream (a `socat` pty tee in front of ros2_control,
//! or `strace -e read,write` on the `ros2_control_node`), and replaying it
//! needs nothing but this crate. See the crate README for how to capture one
//! on the robot without touching ros2_control.
//!
//! ```text
//!   magic   "MWRSER01"                      8 bytes
//!   record  t_ns   i64 little endian        8 bytes
//!           dir    0 = rx, 1 = tx           1 byte
//!           len    u32 little endian        4 bytes
//!           bytes  len bytes
//!   ...
//! ```
//!
//! `t_ns` is a monotonic nanosecond count on the *same* clock the replay
//! feeds to [`crate::cycle::BaseCycle`]; rx records are what the port
//! returned from one `read()`, tx records are what one cycle wrote.

/// Which way the bytes went.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// STM32 -> host
    Rx,
    /// host -> STM32
    Tx,
}

impl Direction {
    pub fn as_byte(self) -> u8 {
        match self {
            Direction::Rx => 0,
            Direction::Tx => 1,
        }
    }
    pub fn from_byte(b: u8) -> Option<Self> {
        match b {
            0 => Some(Direction::Rx),
            1 => Some(Direction::Tx),
            _ => None,
        }
    }
}

/// One `read()` or one cycle's write.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Record {
    pub t_ns: i64,
    pub dir: Direction,
    pub bytes: Vec<u8>,
}

pub const MAGIC: &[u8; 8] = b"MWRSER01";

/// Serialise a whole log.
pub fn encode(records: &[Record]) -> Vec<u8> {
    let mut out = Vec::with_capacity(MAGIC.len() + records.len() * 32);
    out.extend_from_slice(MAGIC);
    for r in records {
        out.extend_from_slice(&r.t_ns.to_le_bytes());
        out.push(r.dir.as_byte());
        out.extend_from_slice(&(r.bytes.len() as u32).to_le_bytes());
        out.extend_from_slice(&r.bytes);
    }
    out
}

/// Parse a whole log. Fails closed: a truncated or corrupt log is an error,
/// never a silently short replay.
pub fn decode(data: &[u8]) -> Result<Vec<Record>, String> {
    if data.len() < MAGIC.len() || &data[..MAGIC.len()] != MAGIC {
        return Err("not a mower serial log (bad magic)".to_string());
    }
    let mut out = Vec::new();
    let mut i = MAGIC.len();
    while i < data.len() {
        if i + 13 > data.len() {
            return Err(format!("truncated record header at byte {i}"));
        }
        let t_ns = i64::from_le_bytes(data[i..i + 8].try_into().unwrap());
        let dir = Direction::from_byte(data[i + 8])
            .ok_or_else(|| format!("bad direction byte {} at {}", data[i + 8], i + 8))?;
        let len = u32::from_le_bytes(data[i + 9..i + 13].try_into().unwrap()) as usize;
        i += 13;
        if i + len > data.len() {
            return Err(format!("truncated record payload at byte {i}"));
        }
        out.push(Record {
            t_ns,
            dir,
            bytes: data[i..i + len].to_vec(),
        });
        i += len;
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip() {
        let records = vec![
            Record { t_ns: 0, dir: Direction::Rx, bytes: vec![] },
            Record { t_ns: 40_000_000, dir: Direction::Tx, bytes: vec![0xA5, 0x5A, 1] },
            Record { t_ns: -5, dir: Direction::Rx, bytes: (0..=255u8).collect() },
        ];
        assert_eq!(decode(&encode(&records)).unwrap(), records);
    }

    #[test]
    fn rejects_garbage() {
        assert!(decode(b"nope").is_err());
        let mut good = encode(&[Record { t_ns: 1, dir: Direction::Tx, bytes: vec![1, 2, 3] }]);
        good.pop();
        assert!(decode(&good).is_err());
    }
}
