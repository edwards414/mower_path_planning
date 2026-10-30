//! mower_schema against what rosbag2 wrote in the E5-0 recordings
//! (deploy/test/golden/recorder): every Schema record's name, encoding and
//! text, byte for byte, and every channel's type hash.

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

#[test]
fn every_schema_and_hash_rosbag2_wrote_is_reproduced() {
    let mut files = Vec::new();
    mcaps(&golden(), &mut files);
    assert!(files.len() >= 12, "fixtures missing under {}", golden().display());
    let mut checked = std::collections::BTreeSet::new();
    let mut channels = 0;
    for f in &files {
        let bytes = zstd::decode_all(std::fs::File::open(f).unwrap()).unwrap();
        let summary = mcap::Summary::read(&bytes).unwrap().expect("a summary");
        for schema in summary.schemas.values() {
            let def = mower_schema::rosbag2_definition(&schema.name);
            assert_eq!(schema.encoding, def.encoding, "{}: {}", f.display(), schema.name);
            assert!(
                schema.data.as_ref() == def.text.as_bytes(),
                "{}: {} differs\n--- rosbag2\n{}\n--- mower_schema\n{}",
                f.display(),
                schema.name,
                String::from_utf8_lossy(&schema.data),
                def.text
            );
            checked.insert(schema.name.clone());
        }
        for ch in summary.channels.values() {
            let schema = ch.schema.as_ref().unwrap();
            let hash = &ch.metadata["topic_type_hash"];
            // action types are not in the table; their hash is the publisher's
            if let Some(h) = mower_schema::type_hash(&schema.name) {
                assert_eq!(hash, h, "{}: {} {}", f.display(), ch.topic, schema.name);
            } else {
                assert!(schema.name.contains("/action/") || schema.name == "action_msgs/msg/GoalStatusArray");
            }
            channels += 1;
        }
    }
    // the recorded types: F1-F3 and the action side capture
    for t in [
        "std_msgs/msg/String", "std_msgs/msg/Bool", "geometry_msgs/msg/PoseStamped", "nav_msgs/msg/Odometry",
        "nav_msgs/msg/OccupancyGrid", "nav_msgs/msg/Path", "sensor_msgs/msg/NavSatFix", "sensor_msgs/msg/Imu",
        "sensor_msgs/msg/BatteryState", "sensor_msgs/msg/CompressedImage", "sensor_msgs/msg/CameraInfo",
        "tf2_msgs/msg/TFMessage", "diagnostic_msgs/msg/DiagnosticArray", "rcl_interfaces/msg/Log",
        "nav2_msgs/action/NavigateToPose_FeedbackMessage", "action_msgs/msg/GoalStatusArray",
    ] {
        assert!(checked.contains(t), "no fixture has {t}");
    }
    eprintln!("{} files, {} schemas, {} channels", files.len(), checked.len(), channels);
}

#[test]
fn the_mower_interface_copy_is_the_repo_s() {
    let repo = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../mower_interface/msg/CoverageProgress.msg");
    let copy = mower_schema::lookup("mower_interface/msg/CoverageProgress").unwrap().text;
    assert_eq!(std::fs::read_to_string(repo).unwrap(), copy, "rerun gen/gen_schema.py");
}
