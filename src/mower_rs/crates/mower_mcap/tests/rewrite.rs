//! G1 of the E5 design: every bag of the E5-0 recordings
//! (deploy/test/golden/recorder) is written again by `BagWriter` from what
//! is in it, with the options its `ros2 bag record` line had, and compared
//! with what rosbag2 wrote: every `.mcap` file and `metadata.yaml`.
//!
//! The replay gives the writer what rosbag2 had: the topics as file 0's
//! channels (name, type, offered QoS, hash) in channel order, which is the
//! order rosbag2 created them; the messages in file order with their
//! receive and source times, per-message zstd undone (the writer compresses
//! again); the header's `library` string (the one field that is the
//! writer's choice, D1). Two things are not in the files and are taken from
//! `metadata.yaml`: where a size split fell (rosbag2's cache thread makes
//! that a message or more late, F2S), and, in snapshot mode, the messages
//! that went into the buffer and were never written (they still count in
//! the file's `message_count` and times).
//!
//! Where a file differs, the test names the first record that does, with
//! the payloads decompressed, so a zstd byte difference (allowed, A4) is
//! told apart from a layout difference.

use std::path::{Path, PathBuf};

use mower_mcap::{BagOptions, BagWriter, Preset, Qos, TopicMetadata};

fn golden() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../../deploy/test/golden/recorder")
}

struct Case {
    /// the bag dir, under golden()
    dir: &'static str,
    preset: Preset,
    message_compression: bool,
    max_size: u64,
    max_duration: u64,
    snapshot: Option<u64>,
}

const MIB: u64 = 1024 * 1024;

/// The `ros2 bag record` options of each bag (mower_recorder record_profile.py
/// and the fixture's profile; the action side capture used the defaults).
fn cases() -> Vec<Case> {
    let full = |dir, size, dur| Case { dir, preset: Preset::None, message_compression: true, max_size: size, max_duration: dur, snapshot: None };
    let snap = |dir| Case { dir, preset: Preset::None, message_compression: false, max_size: 0, max_duration: 0, snapshot: Some(256 * MIB) };
    vec![
        full("F1/MW-E5FIX_20261030T081851/bag", 512 * MIB, 3600),
        snap("F1/MW-E5FIX_20261030T081851/snapshots"),
        Case { dir: "F1/actions", preset: Preset::None, message_compression: false, max_size: 0, max_duration: 0, snapshot: None },
        full("F2/MW-E5FIX_20261030T080903/bag", MIB, 10),
        snap("F2/MW-E5FIX_20261030T080903/snapshots"),
        full("F2S/MW-E5FIX_20261030T081247/bag", MIB, 3600),
        snap("F2S/MW-E5FIX_20261030T081247/snapshots"),
        Case { dir: "F3/MW-E5FIX_20261030T080906/bag", preset: Preset::ZstdFast, message_compression: false, max_size: 512 * MIB, max_duration: 3600, snapshot: None },
        Case { dir: "F3/MW-E5FIX_20261030T080906/camera", preset: Preset::None, message_compression: false, max_size: 256 * MIB, max_duration: 0, snapshot: None },
    ]
}

const ZSTD_MAGIC: [u8; 4] = [0x28, 0xb5, 0x2f, 0xfd];

fn library(file: &[u8]) -> String {
    // magic, then the Header record: op, u64 length, profile, library
    let mut o = 8 + 9;
    let rd = |o: &mut usize| {
        let n = u32::from_le_bytes(file[*o..*o + 4].try_into().unwrap()) as usize;
        let s = String::from_utf8(file[*o + 4..*o + 4 + n].to_vec()).unwrap();
        *o += 4 + n;
        s
    };
    let _profile = rd(&mut o);
    rd(&mut o)
}

/// (starting_time, duration, message_count) of each `files:` entry.
fn files_of(metadata_yaml: &str) -> Vec<(i64, i64, u64)> {
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

/// Every record of a file, a chunk's records in place of the chunk (the
/// chunk's own header kept), payloads and chunk data decompressed.
fn records(file: &[u8]) -> Vec<(u8, Vec<u8>)> {
    let mut out = Vec::new();
    let mut o = 8;
    while o + 9 <= file.len() - 8 {
        let op = file[o];
        let n = u64::from_le_bytes(file[o + 1..o + 9].try_into().unwrap()) as usize;
        let body = &file[o + 9..o + 9 + n];
        if op == 0x06 {
            let clen = u32::from_le_bytes(body[28..32].try_into().unwrap()) as usize;
            let comp = std::str::from_utf8(&body[32..32 + clen]).unwrap().to_string();
            let data = &body[32 + clen + 8..];
            let recs = if comp == "zstd" { zstd::decode_all(data).unwrap() } else { data.to_vec() };
            out.push((0x06, body[..32 + clen].to_vec()));
            let mut i = 0;
            while i < recs.len() {
                let iop = recs[i];
                let m = u64::from_le_bytes(recs[i + 1..i + 9].try_into().unwrap()) as usize;
                let mut b = recs[i + 9..i + 9 + m].to_vec();
                if iop == 0x05 && b[22..].starts_with(&ZSTD_MAGIC) {
                    let raw = zstd::decode_all(&b[22..]).unwrap();
                    b.truncate(22);
                    b.extend_from_slice(&raw);
                }
                out.push((iop | 0x80, b));
                i += 9 + m;
            }
        } else {
            out.push((op, body.to_vec()));
        }
        o += 9 + n;
    }
    out
}

/// A record with the fields that hold file offsets or compressed sizes
/// zeroed: what a zstd chunk of another length moves (A4).
fn normalized(op: u8, body: &[u8]) -> Vec<u8> {
    let mut b = body.to_vec();
    let zero = |b: &mut Vec<u8>, r: std::ops::Range<usize>| b[r].iter_mut().for_each(|x| *x = 0);
    match op {
        // ChunkIndex: chunk_start_offset, chunk_length, the message index
        // offsets, compressed_size
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

fn first_difference(want: &[u8], got: &[u8]) -> String {
    let (a, b) = (records(want), records(got));
    let norm = |v: &[(u8, Vec<u8>)]| v.iter().map(|(op, body)| (*op, normalized(*op, body))).collect::<Vec<_>>();
    if a != b && a.len() == b.len() && norm(&a) == norm(&b) {
        return "the same records once decompressed, offsets apart: a zstd chunk compressed to other bytes (A4)".into();
    }
    for (i, (x, y)) in a.iter().zip(&b).enumerate() {
        if x != y {
            return format!(
                "record #{i}: rosbag2 op {:#04x} ({} bytes), ours op {:#04x} ({} bytes){}",
                x.0,
                x.1.len(),
                y.0,
                y.1.len(),
                if x.0 == y.0 { format!("\n rosbag2 {:02x?}\n ours    {:02x?}", &x.1[..x.1.len().min(96)], &y.1[..y.1.len().min(96)]) } else { String::new() }
            );
        }
    }
    if a.len() != b.len() {
        return format!("{} records vs {}", a.len(), b.len());
    }
    "the same records after decompression: only zstd bytes differ (A4)".into()
}

struct Outcome {
    files: usize,
    identical: usize,
    zstd_only: usize,
}

fn rewrite(case: &Case, tmp: &Path) -> Outcome {
    let dir = golden().join(case.dir);
    let base = dir.file_name().unwrap().to_str().unwrap().to_string();
    let mut originals = Vec::new();
    for n in 0.. {
        let p = dir.join(format!("{base}_{n}.mcap.zst"));
        if !p.exists() {
            break;
        }
        originals.push(zstd::decode_all(std::fs::File::open(p).unwrap()).unwrap());
    }
    assert!(!originals.is_empty(), "{}", case.dir);
    let metadata_yaml = std::fs::read_to_string(dir.join("metadata.yaml")).unwrap();
    let mut opts = BagOptions::new(&library(&originals[0]));
    opts.preset = case.preset;
    opts.message_compression = case.message_compression;
    opts.max_bagfile_duration = case.max_duration;
    opts.snapshot_cache_size = case.snapshot;
    // a size split is replayed where it fell (rosbag2's lags); the rule
    // itself has its own test
    let force_splits = case.max_size > 0 && case.max_size < 512 * MIB;
    opts.max_bagfile_size = if force_splits { 0 } else { case.max_size };
    let out_dir = tmp.join(&base);
    let mut w = BagWriter::create(&out_dir, opts).unwrap();

    // the topics, in file 0's channel order
    let summary = mcap::Summary::read(&originals[0]).unwrap().unwrap();
    let mut channels: Vec<_> = summary.channels.values().cloned().collect();
    channels.sort_by_key(|c| c.id);
    for (i, ch) in channels.iter().enumerate() {
        assert_eq!(ch.id as usize, i + 1);
        let schema = ch.schema.as_ref().unwrap();
        w.create_topic(TopicMetadata {
            name: ch.topic.clone(),
            type_name: schema.name.clone(),
            serialization_format: ch.message_encoding.clone(),
            offered_qos_profiles: Qos::parse_list(&ch.metadata["offered_qos_profiles"]).unwrap(),
            type_description_hash: ch.metadata["topic_type_hash"].clone(),
        })
        .unwrap();
        // the QoS text round-trips through the parser
        assert_eq!(Qos::dump_list(&Qos::parse_list(&ch.metadata["offered_qos_profiles"]).unwrap()), ch.metadata["offered_qos_profiles"]);
    }
    let files = files_of(&metadata_yaml);
    assert_eq!(files.len(), originals.len());
    for (k, file) in originals.iter().enumerate() {
        if k > 0 && force_splits {
            w.split().unwrap();
        }
        let mut written = 0u64;
        for m in mcap::MessageStream::new(file).unwrap() {
            let m = m.unwrap();
            let data = if case.message_compression { zstd::decode_all(&m.data[..]).unwrap() } else { m.data.to_vec() };
            w.write(&m.channel.topic, m.log_time as i64, m.publish_time as i64, &data).unwrap();
            written += 1;
        }
        if case.snapshot.is_some() {
            if k + 1 < originals.len() {
                assert!(w.take_snapshot().unwrap());
            } else {
                // what was buffered after the last trigger and dropped at stop
                let (start, duration, count) = files[k];
                let extra = count - written;
                for i in 0..extra {
                    let t = if extra == 1 { start } else { start + (duration as i128 * i as i128 / (extra as i128 - 1)) as i64 };
                    w.write(&channels[0].topic, t, t, &[]).unwrap();
                }
            }
        }
    }
    w.close().unwrap();

    let mut outcome = Outcome { files: 0, identical: 0, zstd_only: 0 };
    for (k, want) in originals.iter().enumerate() {
        let got = std::fs::read(out_dir.join(format!("{base}_{k}.mcap"))).unwrap();
        outcome.files += 1;
        if *want == got {
            outcome.identical += 1;
            continue;
        }
        let why = first_difference(want, &got);
        assert!(why.contains("(A4)"), "{} file {k}: {why}", case.dir);
        outcome.zstd_only += 1;
    }
    assert!(!out_dir.join(format!("{base}_{}.mcap", originals.len())).exists(), "{}: an extra file", case.dir);
    let ours = std::fs::read_to_string(out_dir.join("metadata.yaml")).unwrap();
    assert!(ours == metadata_yaml, "{}: metadata.yaml differs\n--- rosbag2\n{metadata_yaml}\n--- ours\n{ours}", case.dir);
    outcome
}

#[test]
fn every_e5_0_bag_is_written_again_as_rosbag2_wrote_it() {
    // MOWER_MCAP_REWRITE_KEEP=<dir> keeps the rewritten bags there, for the
    // readers (mower-check-run, ros2 bag info) to look at
    let keep = std::env::var_os("MOWER_MCAP_REWRITE_KEEP").map(PathBuf::from);
    let tmp = keep.clone().unwrap_or_else(|| std::env::temp_dir().join(format!("mower_mcap_rewrite_{}", std::process::id())));
    std::fs::create_dir_all(&tmp).unwrap();
    let mut report = Vec::new();
    for case in cases().iter() {
        let t = tmp.join(case.dir.replace('/', "_"));
        std::fs::create_dir_all(&t).unwrap();
        let o = rewrite(case, &t);
        let line = format!("{}: {} files, {} identical, {} differ only in zstd bytes", case.dir, o.files, o.identical, o.zstd_only);
        eprintln!("{line}");
        report.push(line);
    }
    if keep.is_none() {
        std::fs::remove_dir_all(&tmp).unwrap();
    }
    // every bag but the zstd_fast one is byte for byte rosbag2's
    let exact: usize = report.iter().filter(|l| l.ends_with(" 0 differ only in zstd bytes")).count();
    assert!(exact >= cases().len() - 1, "{}", report.join("\n"));
}
