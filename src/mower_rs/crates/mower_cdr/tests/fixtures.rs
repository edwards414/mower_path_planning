//! Every payload rosbag2 recorded off the wire in the E5-0 recordings
//! (deploy/test/golden/recorder), decoded with its schema and encoded again,
//! must come back byte for byte: rmw's own bytes, padding included, on every
//! recorded type with real values.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

fn golden() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../../deploy/test/golden/recorder")
}

fn mcaps(dir: &Path, out: &mut Vec<PathBuf>) {
    let mut entries: Vec<_> = std::fs::read_dir(dir).unwrap().map(|e| e.unwrap().path()).collect();
    entries.sort();
    for p in entries {
        if p.is_dir() {
            mcaps(&p, out);
        } else if p.to_string_lossy().ends_with(".mcap.zst") {
            out.push(p);
        }
    }
}

const ZSTD_MAGIC: [u8; 4] = [0x28, 0xb5, 0x2f, 0xfd];

#[test]
fn every_recorded_payload_round_trips() {
    let mut files = Vec::new();
    mcaps(&golden(), &mut files);
    let mut per_type: BTreeMap<String, usize> = BTreeMap::new();
    let mut zstd_framed = 0;
    for f in &files {
        let bytes = zstd::decode_all(std::fs::File::open(f).unwrap()).unwrap();
        for m in mcap::MessageStream::new(&bytes).unwrap() {
            let m = m.unwrap();
            let schema = m.channel.schema.as_ref().unwrap();
            if schema.encoding != "ros2msg" {
                continue; // the action side capture: no definition to decode with
            }
            let raw = if m.data.starts_with(&ZSTD_MAGIC) {
                zstd_framed += 1;
                zstd::decode_all(&m.data[..]).unwrap()
            } else {
                m.data.to_vec()
            };
            let ctx = || format!("{} {} {} @{}", f.display(), m.channel.topic, schema.name, m.log_time);
            let value = mower_cdr::decode(&schema.name, &raw).unwrap_or_else(|e| panic!("{}: {e}", ctx()));
            let again = mower_cdr::encode(&schema.name, &value).unwrap_or_else(|e| panic!("{}: {e}", ctx()));
            assert!(again == raw, "{}: re-encoded {} bytes differ from rmw's {}", ctx(), again.len(), raw.len());
            *per_type.entry(schema.name.clone()).or_default() += 1;
        }
    }
    eprintln!("{} files, {zstd_framed} zstd-framed payloads, per type: {per_type:?}", files.len());
    // every type that had a message (no fixture has a /plan message)
    assert!(per_type.len() >= 13, "{per_type:?}");
    assert!(zstd_framed > 10_000);
}
