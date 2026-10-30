//! The split rules on their own (the fixtures replay size splits where
//! rosbag2's lagging cache thread put them): rosbag2 checks before each
//! message (`should_split_bagfile`) whether the bytes libmcap has written
//! reach `--max-bag-size`, which only grows when a chunk is flushed, or
//! whether the message is more than `--max-bag-duration` after the file's
//! first.

use mower_mcap::{BagOptions, BagWriter, Qos, TopicMetadata};

fn topic() -> TopicMetadata {
    TopicMetadata {
        name: "/x".into(),
        type_name: "std_msgs/msg/String".into(),
        serialization_format: "cdr".into(),
        offered_qos_profiles: vec![Qos::new("keep_last", 10, "reliable", "volatile")],
        type_description_hash: mower_schema::type_hash("std_msgs/msg/String").unwrap().into(),
    }
}

fn messages_per_file(dir: &std::path::Path) -> Vec<u64> {
    let mut out = Vec::new();
    for n in 0.. {
        let p = dir.join(format!("b_{n}.mcap"));
        if !p.exists() {
            return out;
        }
        let bytes = std::fs::read(p).unwrap();
        out.push(mcap::MessageStream::new(&bytes).unwrap().count() as u64);
    }
    unreachable!()
}

#[test]
fn a_size_split_comes_at_the_message_after_a_flush_reaches_the_limit() {
    let tmp = std::env::temp_dir().join(format!("mower_mcap_rules_size_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&tmp);
    std::fs::create_dir_all(&tmp).unwrap();
    let mut o = BagOptions::new("test");
    o.max_bagfile_size = 1024 * 1024;
    let mut w = BagWriter::create(&tmp.join("b"), o).unwrap();
    w.create_topic(topic()).unwrap();
    // 100 000-byte payloads: a chunk closes after the 8th message (768 KiB),
    // so 1 MiB is first reached when the 16th closes the second chunk
    let payload = vec![7u8; 100_000];
    for i in 0..20i64 {
        w.write("/x", 1_000_000_000 + i, 0, &payload).unwrap();
    }
    let m = w.close().unwrap();
    assert_eq!(messages_per_file(&tmp.join("b")), [16, 4]);
    assert_eq!(m.files.iter().map(|f| f.message_count).collect::<Vec<_>>(), [16, 4]);
    std::fs::remove_dir_all(&tmp).unwrap();
}

#[test]
fn a_duration_split_comes_at_the_first_message_past_the_limit() {
    let tmp = std::env::temp_dir().join(format!("mower_mcap_rules_dur_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&tmp);
    std::fs::create_dir_all(&tmp).unwrap();
    let mut o = BagOptions::new("test");
    o.max_bagfile_duration = 10;
    let mut w = BagWriter::create(&tmp.join("b"), o).unwrap();
    w.create_topic(topic()).unwrap();
    let s = 1_000_000_000i64;
    // 10 s exactly is not past the limit; 10 s + 1 ns is
    for t in [0, 5 * s, 10 * s, 10 * s + 1, 15 * s, 20 * s + 1, 20 * s + 2] {
        w.write("/x", 100 * s + t, 0, b"\0\x01\0\0\x01\0\0\0").unwrap();
    }
    let m = w.close().unwrap();
    assert_eq!(messages_per_file(&tmp.join("b")), [3, 3, 1]);
    assert_eq!(m.files[1].starting_time, 110 * s + 1);
    std::fs::remove_dir_all(&tmp).unwrap();
}
