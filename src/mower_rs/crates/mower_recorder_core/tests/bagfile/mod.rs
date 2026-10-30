//! What the recording tests share: the E5-0 fixtures and a small MCAP
//! reader (records, chunks expanded), independent of mower_mcap's writer.
#![allow(dead_code)]

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

pub fn golden() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../../deploy/test/golden/recorder")
}

pub fn config_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../mower_recorder/config")
}

pub const ZSTD_MAGIC: [u8; 4] = [0x28, 0xb5, 0x2f, 0xfd];

pub fn unzst(path: &Path) -> Vec<u8> {
    zstd::decode_all(std::fs::File::open(path).unwrap_or_else(|e| panic!("{}: {e}", path.display()))).unwrap()
}

/// The `<base>_<n>.mcap.zst` files of a fixture bag dir, decompressed.
pub fn originals(dir: &Path) -> Vec<Vec<u8>> {
    let base = dir.file_name().unwrap().to_str().unwrap().to_string();
    let mut out = Vec::new();
    for n in 0.. {
        let p = dir.join(format!("{base}_{n}.mcap.zst"));
        if !p.exists() {
            break;
        }
        out.push(unzst(&p));
    }
    out
}

pub struct Channel {
    pub id: u16,
    pub schema_id: u16,
    pub topic: String,
    pub encoding: String,
    pub metadata: Vec<(String, String)>,
}

impl Channel {
    pub fn meta(&self, k: &str) -> &str {
        self.metadata.iter().find(|(a, _)| a == k).map(|(_, v)| v.as_str()).unwrap_or("")
    }
}

pub struct Schema {
    pub id: u16,
    pub name: String,
    pub encoding: String,
    pub data: Vec<u8>,
}

#[derive(Clone, PartialEq, Eq, Debug)]
pub struct Message {
    pub channel: u16,
    pub sequence: u32,
    pub log_time: u64,
    pub publish_time: u64,
    pub data: Vec<u8>,
}

pub struct Mcap {
    pub library: String,
    pub schemas: BTreeMap<u16, Schema>,
    pub channels: BTreeMap<u16, Channel>,
    /// in file order, payloads as stored
    pub messages: Vec<Message>,
}

impl Mcap {
    pub fn topic(&self, m: &Message) -> &str {
        &self.channels[&m.channel].topic
    }
    pub fn type_of(&self, c: &Channel) -> &str {
        &self.schemas[&c.schema_id].name
    }
}

struct Rd<'a> {
    b: &'a [u8],
    o: usize,
}

impl Rd<'_> {
    fn take(&mut self, n: usize) -> &[u8] {
        let s = &self.b[self.o..self.o + n];
        self.o += n;
        s
    }
    fn u16(&mut self) -> u16 {
        u16::from_le_bytes(self.take(2).try_into().unwrap())
    }
    fn u32(&mut self) -> u32 {
        u32::from_le_bytes(self.take(4).try_into().unwrap())
    }
    fn u64(&mut self) -> u64 {
        u64::from_le_bytes(self.take(8).try_into().unwrap())
    }
    fn str(&mut self) -> String {
        let n = self.u32() as usize;
        String::from_utf8(self.take(n).to_vec()).unwrap()
    }
    fn rest(&mut self) -> Vec<u8> {
        let s = self.b[self.o..].to_vec();
        self.o = self.b.len();
        s
    }
}

/// The top-level records (op, body), the footer's magic excluded.
fn top(file: &[u8]) -> Vec<(u8, &[u8])> {
    assert_eq!(&file[..8], b"\x89MCAP0\r\n");
    let mut out = Vec::new();
    let mut o = 8;
    while o + 9 <= file.len() - 8 {
        let op = file[o];
        let n = u64::from_le_bytes(file[o + 1..o + 9].try_into().unwrap()) as usize;
        out.push((op, &file[o + 9..o + 9 + n]));
        o += 9 + n;
    }
    out
}

/// A chunk's records, decompressed.
fn chunk_records(body: &[u8]) -> Vec<(u8, Vec<u8>)> {
    let clen = u32::from_le_bytes(body[28..32].try_into().unwrap()) as usize;
    let comp = std::str::from_utf8(&body[32..32 + clen]).unwrap();
    let data = &body[32 + clen + 8..];
    let recs = if comp == "zstd" { zstd::decode_all(data).unwrap() } else { data.to_vec() };
    let mut out = Vec::new();
    let mut i = 0;
    while i < recs.len() {
        let op = recs[i];
        let m = u64::from_le_bytes(recs[i + 1..i + 9].try_into().unwrap()) as usize;
        out.push((op, recs[i + 9..i + 9 + m].to_vec()));
        i += 9 + m;
    }
    out
}

pub fn read(file: &[u8]) -> Mcap {
    let mut m = Mcap { library: String::new(), schemas: BTreeMap::new(), channels: BTreeMap::new(), messages: Vec::new() };
    let on = |m: &mut Mcap, op: u8, body: &[u8]| {
        let mut r = Rd { b: body, o: 0 };
        match op {
            0x01 => {
                let _profile = r.str();
                m.library = r.str();
            }
            0x03 => {
                let id = r.u16();
                let s = Schema { id, name: r.str(), encoding: r.str(), data: { let n = r.u32() as usize; r.take(n).to_vec() } };
                m.schemas.entry(id).or_insert(s);
            }
            0x04 => {
                let id = r.u16();
                let schema_id = r.u16();
                let topic = r.str();
                let encoding = r.str();
                let n = r.u32() as usize;
                let end = r.o + n;
                let mut metadata = Vec::new();
                while r.o < end {
                    metadata.push((r.str(), r.str()));
                }
                m.channels.entry(id).or_insert(Channel { id, schema_id, topic, encoding, metadata });
            }
            0x05 => {
                let msg = Message { channel: r.u16(), sequence: r.u32(), log_time: r.u64(), publish_time: r.u64(), data: r.rest() };
                m.messages.push(msg);
            }
            _ => {}
        }
    };
    for (op, body) in top(file) {
        if op == 0x06 {
            for (iop, b) in chunk_records(body) {
                on(&mut m, iop, &b);
            }
        } else {
            on(&mut m, op, body);
        }
    }
    m
}

/// Every record of a file, a chunk's records in place of the chunk (the
/// chunk's own header kept), payloads and chunk data decompressed.
pub fn records(file: &[u8]) -> Vec<(u8, Vec<u8>)> {
    let mut out = Vec::new();
    for (op, body) in top(file) {
        if op == 0x06 {
            let clen = u32::from_le_bytes(body[28..32].try_into().unwrap()) as usize;
            out.push((0x06, body[..32 + clen].to_vec()));
            for (iop, mut b) in chunk_records(body) {
                if iop == 0x05 && b[22..].starts_with(&ZSTD_MAGIC) {
                    let raw = zstd::decode_all(&b[22..]).unwrap();
                    b.truncate(22);
                    b.extend_from_slice(&raw);
                }
                out.push((iop | 0x80, b));
            }
        } else {
            out.push((op, body.to_vec()));
        }
    }
    out
}

/// A record with the fields that hold file offsets or compressed sizes
/// zeroed: what a zstd chunk of another length moves (A4).
fn normalized(op: u8, body: &[u8]) -> Vec<u8> {
    let mut b = body.to_vec();
    let zero = |b: &mut Vec<u8>, r: std::ops::Range<usize>| b[r].iter_mut().for_each(|x| *x = 0);
    match op {
        0x08 => {
            zero(&mut b, 16..32);
            let n = u32::from_le_bytes(b[32..36].try_into().unwrap()) as usize;
            for i in 0..n / 10 {
                let at = 36 + i * 10 + 2;
                zero(&mut b, at..at + 8);
            }
            let clen_at = 36 + n + 8;
            let clen = u32::from_le_bytes(b[clen_at..clen_at + 4].try_into().unwrap()) as usize;
            let cs = clen_at + 4 + clen;
            zero(&mut b, cs..cs + 8);
        }
        0x0d => zero(&mut b, 0..8),
        0x0e => zero(&mut b, 1..9),
        0x02 => zero(&mut b, 0..20),
        _ => {}
    }
    b
}

/// The same records once decompressed, offsets apart: only zstd chunk
/// bytes differ (A4).
pub fn differs_only_in_zstd(want: &[u8], got: &[u8]) -> bool {
    let (a, b) = (records(want), records(got));
    let norm = |v: &[(u8, Vec<u8>)]| v.iter().map(|(op, body)| (*op, normalized(*op, body))).collect::<Vec<_>>();
    a.len() == b.len() && norm(&a) == norm(&b)
}

/// (starting_time, duration, message_count) of each `files:` entry of a
/// `metadata.yaml`.
pub fn files_of(metadata_yaml: &str) -> Vec<(i64, i64, u64)> {
    let mut out = Vec::new();
    let files = metadata_yaml.split("\n  files:\n").nth(1).unwrap();
    let mut cur: Vec<i64> = Vec::new();
    for line in files.lines() {
        let t = line.trim();
        for key in ["nanoseconds_since_epoch: ", "nanoseconds: ", "message_count: "] {
            if let Some(v) = t.strip_prefix(key) {
                cur.push(v.parse().unwrap());
            }
        }
        if cur.len() == 3 {
            out.push((cur[0], cur[1], cur[2] as u64));
            cur.clear();
        }
    }
    out
}

pub fn tmp(name: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("mower_recorder_core_{name}_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).unwrap();
    d
}

/// The files under `dir`, relative, sorted (directories with a final `/`
/// when empty).
pub fn tree(dir: &Path) -> Vec<String> {
    fn walk(base: &Path, d: &Path, out: &mut Vec<String>) {
        let mut entries: Vec<_> = std::fs::read_dir(d).unwrap().flatten().collect();
        entries.sort_by_key(|e| e.file_name());
        if entries.is_empty() && d != base {
            out.push(format!("{}/", d.strip_prefix(base).unwrap().display()));
        }
        for e in entries {
            let p = e.path();
            if p.is_dir() {
                walk(base, &p, out);
            } else {
                out.push(p.strip_prefix(base).unwrap().display().to_string());
            }
        }
    }
    let mut out = Vec::new();
    walk(dir, dir, &mut out);
    out.sort();
    out
}
