//! Every bag of the E5-0 recordings (deploy/test/golden/recorder) recorded
//! again through `recording` (the writer thread, its queue and the feed the
//! DDS side calls), and compared with what rosbag2 wrote, as
//! mower_mcap/tests/rewrite.rs compares the bare writer: every `.mcap` file
//! and `metadata.yaml` byte for byte, except the `zstd_fast` chunks (A4).
//!
//! The feed gets what rosbag2 had: the topics when they appeared (file 0's
//! channels in channel order, which is the order rosbag2 created them, with
//! their offered QoS and type hash), then the messages in file order with
//! their receive and source times, per-message zstd undone (the writer
//! compresses again), the snapshot trigger where F1 had it, and the
//! header's `library` (D1). Two things are not in the files: where a size
//! split fell (rosbag2's lags, F2S: forced here with `split`, and the
//! natural rule checked apart, A13), and in snapshot mode the messages
//! buffered and never written (they count in `files[].message_count`).

mod bagfile;

use std::path::Path;

use bagfile::*;
use mower_mcap::{BagOptions, Preset, Qos, TopicMetadata};
use mower_recorder_core::recording::{Options, Recording};

#[derive(Clone, Copy)]
struct Case {
    dir: &'static str,
    preset: Preset,
    message_compression: bool,
    max_size: u64,
    max_duration: u64,
    snapshot: Option<u64>,
    hidden: bool,
}

const MIB: u64 = 1024 * 1024;

/// The options of each bag: the groups as `profile::recorder_specs` makes
/// them for the fixture's profile (checked in `the_options_are_the_profiles`).
fn cases() -> Vec<Case> {
    let full = |dir, size, dur| Case { dir, preset: Preset::None, message_compression: true, max_size: size, max_duration: dur, snapshot: None, hidden: false };
    let snap = |dir| Case { dir, preset: Preset::None, message_compression: false, max_size: 0, max_duration: 0, snapshot: Some(256 * MIB), hidden: false };
    vec![
        full("F1/MW-E5FIX_20261030T081851/bag", 512 * MIB, 3600),
        snap("F1/MW-E5FIX_20261030T081851/snapshots"),
        // the side recorder with --include-hidden-topics and the defaults
        Case { dir: "F1/actions", preset: Preset::None, message_compression: false, max_size: 0, max_duration: 0, snapshot: None, hidden: true },
        full("F2/MW-E5FIX_20261030T080903/bag", MIB, 10),
        snap("F2/MW-E5FIX_20261030T080903/snapshots"),
        full("F2S/MW-E5FIX_20261030T081247/bag", MIB, 3600),
        snap("F2S/MW-E5FIX_20261030T081247/snapshots"),
        Case { dir: "F3/MW-E5FIX_20261030T080906/bag", preset: Preset::ZstdFast, message_compression: false, max_size: 512 * MIB, max_duration: 3600, snapshot: None, hidden: false },
        Case { dir: "F3/MW-E5FIX_20261030T080906/camera", preset: Preset::None, message_compression: false, max_size: 256 * MIB, max_duration: 0, snapshot: None, hidden: false },
    ]
}

/// Whether the size splits are replayed where rosbag2's fell.
#[derive(Clone, Copy, PartialEq)]
enum Splits {
    AsRosbag2,
    ByTheRule,
}

/// Feeds a fixture bag into a new recording at `out`; the caller ends it.
fn feed(case: &Case, out: &Path, splits: Splits) -> (Recording, u64) {
    let dir = golden().join(case.dir);
    let originals = originals(&dir);
    assert!(!originals.is_empty(), "{}", case.dir);
    let metadata_yaml = std::fs::read_to_string(dir.join("metadata.yaml")).unwrap();
    let file0 = read(&originals[0]);
    let mut opts = BagOptions::new(&file0.library);
    opts.preset = case.preset;
    opts.message_compression = case.message_compression;
    opts.max_bagfile_duration = case.max_duration;
    opts.snapshot_cache_size = case.snapshot;
    let force_splits = splits == Splits::AsRosbag2 && case.max_size > 0 && case.max_size < 512 * MIB;
    opts.max_bagfile_size = if force_splits { 0 } else { case.max_size };
    let mut o = Options::new("replay", out, opts);
    o.include_hidden = case.hidden;
    let rec = Recording::start(o).unwrap();
    let f = rec.feed();

    let channels: Vec<_> = file0.channels.values().collect();
    for (i, ch) in channels.iter().enumerate() {
        assert_eq!(ch.id as usize, i + 1);
        let t = TopicMetadata {
            name: ch.topic.clone(),
            type_name: file0.type_of(ch).to_string(),
            serialization_format: ch.encoding.clone(),
            offered_qos_profiles: Qos::parse_list(ch.meta("offered_qos_profiles")).unwrap(),
            type_description_hash: ch.meta("topic_type_hash").to_string(),
        };
        assert!(f.topic_appeared(t), "{} {}", case.dir, ch.topic);
    }
    let files = files_of(&metadata_yaml);
    assert_eq!(files.len(), originals.len());
    let mut fed = 0u64;
    for (k, file) in originals.iter().enumerate() {
        if k > 0 && force_splits {
            rec.split().unwrap();
        }
        let m = read(file);
        let mut written = 0u64;
        for msg in &m.messages {
            let data = if case.message_compression { zstd::decode_all(&msg.data[..]).unwrap() } else { msg.data.clone() };
            assert!(f.message(m.topic(msg), msg.log_time as i64, msg.publish_time as i64, data));
            written += 1;
        }
        fed += written;
        if case.snapshot.is_some() {
            if k + 1 < originals.len() {
                assert!(rec.take_snapshot().unwrap());
            } else {
                // what was buffered after the last trigger and dropped at stop
                let (start, duration, count) = files[k];
                let extra = count - written;
                for i in 0..extra {
                    let t = if extra == 1 { start } else { start + (duration as i128 * i as i128 / (extra as i128 - 1)) as i64 };
                    assert!(f.message(&channels[0].topic, t, t, Vec::new()));
                }
                fed += extra;
            }
        }
    }
    (rec, fed)
}

struct Outcome {
    files: usize,
    identical: usize,
    zstd_only: usize,
}

/// Our bag at `out` against rosbag2's.
fn compare(case: &Case, out: &Path) -> Outcome {
    let dir = golden().join(case.dir);
    let base = dir.file_name().unwrap().to_str().unwrap().to_string();
    let originals = originals(&dir);
    let mut o = Outcome { files: 0, identical: 0, zstd_only: 0 };
    for (k, want) in originals.iter().enumerate() {
        let got = std::fs::read(out.join(format!("{base}_{k}.mcap"))).unwrap();
        o.files += 1;
        if *want == got {
            o.identical += 1;
        } else {
            assert!(case.preset == Preset::ZstdFast && differs_only_in_zstd(want, &got), "{} file {k} differs beyond zstd bytes", case.dir);
            o.zstd_only += 1;
        }
    }
    assert!(!out.join(format!("{base}_{}.mcap", originals.len())).exists(), "{}: an extra file", case.dir);
    let want = std::fs::read_to_string(dir.join("metadata.yaml")).unwrap();
    let ours = std::fs::read_to_string(out.join("metadata.yaml")).unwrap();
    assert!(ours == want, "{}: metadata.yaml differs\n--- rosbag2\n{want}\n--- ours\n{ours}", case.dir);
    o
}

fn out_dir(tmp: &Path, case: &Case, tag: &str) -> std::path::PathBuf {
    let base = Path::new(case.dir).file_name().unwrap().to_owned();
    let d = tmp.join(format!("{}_{tag}", case.dir.replace('/', "_")));
    std::fs::create_dir_all(&d).unwrap();
    d.join(base)
}

#[test]
fn every_e5_0_bag_is_recorded_again_as_rosbag2_wrote_it() {
    let tmp = tmp("replay");
    let mut report = Vec::new();
    for case in cases() {
        let out = out_dir(&tmp, &case, "stop");
        let (rec, fed) = feed(&case, &out, Splits::AsRosbag2);
        let fin = rec.stop().unwrap();
        // everything fed was recorded (A12), nothing dropped
        let s = fin.stats;
        assert_eq!((s.queued, s.written, s.dropped_full, s.dropped_closed, s.dropped_unknown, s.write_errors), (fed, fed, 0, 0, 0, 0), "{}", case.dir);
        let o = compare(&case, &out);
        let line = format!("{}: {} files, {} identical, {} differ only in zstd bytes", case.dir, o.files, o.identical, o.zstd_only);
        eprintln!("{line}");
        report.push((case.preset, o));
    }
    // 15 files; all but the zstd_fast one byte for byte
    assert_eq!(report.iter().map(|(_, o)| o.files).sum::<usize>(), 15);
    for (preset, o) in &report {
        if *preset != Preset::ZstdFast {
            assert_eq!(o.identical, o.files);
        }
    }
    std::fs::remove_dir_all(&tmp).unwrap();
}

/// R5: a recording that is dropped, or dropped by a panic's unwinding,
/// finalizes as a stop does.
#[test]
fn a_dropped_or_panicking_recording_is_finalized() {
    let tmp = tmp("drop");
    let cases = cases();
    // F2: five files split by duration, per-message zstd
    let case = &cases[3];
    let out = out_dir(&tmp, case, "drop");
    let (rec, _) = feed(case, &out, Splits::AsRosbag2);
    drop(rec);
    assert_eq!(compare(case, &out).identical, 5);
    // F2S (forced split) and its never-triggered snapshot group, ended by a
    // panic in the code holding them
    for case in [&cases[5], &cases[6]] {
        let out = out_dir(&tmp, case, "panic");
        let o2 = out.clone();
        let c = *case;
        let r = std::thread::spawn(move || {
            let (_rec, _) = feed(&c, &o2, Splits::AsRosbag2);
            panic!("a bug elsewhere");
        })
        .join();
        assert!(r.is_err());
        let o = compare(case, &out);
        assert_eq!(o.identical, o.files);
    }
    std::fs::remove_dir_all(&tmp).unwrap();
}

/// A13: with the size rule itself (flushed bytes >= --max-bag-size, checked
/// before each write), F2S splits where its first chunk past 1 MiB was
/// flushed; rosbag2's cache thread let one more message into the old file.
/// The messages are the same, in the same order.
#[test]
fn the_size_split_by_the_rule() {
    let tmp = tmp("a13");
    let case = &cases()[5];
    let out = out_dir(&tmp, case, "rule");
    let (rec, fed) = feed(case, &out, Splits::ByTheRule);
    let fin = rec.stop().unwrap();
    assert_eq!(fin.metadata.files.len(), 2);
    let flat = |files: Vec<Vec<u8>>| -> (Vec<usize>, Vec<(String, u64, u64, Vec<u8>)>) {
        let mut per = Vec::new();
        let mut all = Vec::new();
        for f in files {
            let m = read(&f);
            per.push(m.messages.len());
            for msg in &m.messages {
                all.push((m.topic(msg).to_string(), msg.log_time, msg.publish_time, zstd::decode_all(&msg.data[..]).unwrap()));
            }
        }
        (per, all)
    };
    let (theirs_per, theirs) = flat(originals(&golden().join(case.dir)));
    let ours_files: Vec<Vec<u8>> = (0..2).map(|k| std::fs::read(out.join(format!("bag_{k}.mcap"))).unwrap()).collect();
    let (ours_per, ours) = flat(ours_files);
    assert_eq!(theirs.len() as u64, fed);
    assert!(ours == theirs, "the same messages in the same order");
    // one message earlier than rosbag2
    assert_eq!(ours_per[0] + 1, theirs_per[0], "{ours_per:?} vs {theirs_per:?}");
    assert_eq!(fin.metadata.files.iter().map(|f| f.message_count as usize).collect::<Vec<_>>(), ours_per);
    std::fs::remove_dir_all(&tmp).unwrap();
}

/// Hidden names are refused unless the group includes them, as rosbag2
/// skips them without --include-hidden-topics; and the cases' options are
/// what the profiles give.
#[test]
fn hidden_names_and_the_options_are_the_profiles() {
    let tmp = tmp("hidden");
    let rec = Recording::start(Options::new("full", &tmp.join("bag"), BagOptions::new("t"))).unwrap();
    let f = rec.feed();
    let m = read(&originals(&golden().join("F1/actions"))[0]);
    for ch in m.channels.values() {
        assert!(!f.wants(&ch.topic));
        let t = mower_recorder_core::recording::topic_metadata(&ch.topic, m.type_of(ch), &[], ch.meta("topic_type_hash"));
        assert!(!f.topic_appeared(t), "{}", ch.topic);
    }
    let fin = rec.stop().unwrap();
    assert!(fin.metadata.topics_with_message_count.is_empty());

    use mower_recorder_core::profile;
    let cfg = |p: &Path| profile::load_config(Some(p), &mut Vec::new());
    let runs = [
        (cfg(&config_dir().join("record_topics.yaml")), [0, 1]),
        (cfg(&golden().join("F2/record_topics_split.yaml")), [3, 4]),
        (cfg(&golden().join("F2S/record_topics_split.yaml")), [5, 6]),
        (cfg(&config_dir().join("data_collection.yaml")), [7, 8]),
    ];
    let cases = cases();
    for (c, idx) in runs {
        let specs = profile::recorder_specs(&c, Path::new("/r"), None, "t").unwrap();
        assert_eq!(specs.len(), 2);
        for (s, i) in specs.iter().zip(idx) {
            let b = s.bag.as_ref().unwrap();
            let k = &cases[i];
            assert_eq!((b.preset, b.message_compression, b.max_bagfile_size, b.max_bagfile_duration, b.snapshot_cache_size), (k.preset, k.message_compression, k.max_size, k.max_duration, k.snapshot), "{}", k.dir);
        }
    }
    std::fs::remove_dir_all(&tmp).unwrap();
}
