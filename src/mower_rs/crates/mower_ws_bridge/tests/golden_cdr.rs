//! The bytes rmw writes for the corpus's values
//! (`deploy/test/golden/ros_facts/json_corpus_cdr.txt`): with them,
//! `mower_ros_facts/tests/json_corpus.rs` holds the frames to `mower_cdr` with
//! no r2r. `MOWER_GOLDEN=write` writes the file (tools/oracle/regen.sh facts);
//! here it is compared. `map_layer` is a captured frame of 200 kB, with a
//! file of its own. The services' requests and responses have a file of their own
//! (`json_corpus_srv_cdr.txt`, E9-22), which `plain_corpus.rs` replays on the
//! plain types.

#[path = "golden/fixtures.rs"]
#[allow(dead_code)]
mod fixtures;

use fixtures::{Corpus, Msg, Srv};
use serde_json::Value;
use std::path::Path;

/// The robot's RMW, as `mower_cdr/tests/rmw_oracle.rs` says why.
fn robot_rmw() {
    let lib = std::env::var("AMENT_PREFIX_PATH").unwrap_or_default().split(':').any(|p| {
        Path::new(p).join("lib/librmw_cyclonedds_cpp.so").exists()
    });
    assert!(lib, "needs the robot's RMW: apt-get install ros-jazzy-rmw-cyclonedds-cpp");
    std::env::set_var("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp");
}

#[derive(Default)]
struct Cdr {
    lines: Vec<String>,
    services: Vec<String>,
}

impl Cdr {
    fn add<T: Msg>(&mut self, case: &str, value: &T) {
        let ty = mower_bus::ros_type_name(std::any::type_name::<T>()).expect("a message type");
        let hex: String = value.to_serialized_bytes().unwrap().iter().map(|b| format!("{b:02x}")).collect();
        self.lines.push(format!("{case}\t{ty}\t{hex}"));
    }
}

impl Corpus for Cdr {
    fn publish<T: Msg>(&mut self, case: &str, _topic: &str, value: T) {
        if fixtures::own_file(case).is_none() {
            self.add(case, &value);
        }
    }

    fn service<S: Srv>(&mut self, case: &str, _service: &str, _id: &str, args: Value, response: S::Response) {
        let ty = mower_bus::ros_type_name(std::any::type_name::<S::Request>()).expect("a service type");
        let request: S::Request = serde_json::from_value(args).unwrap_or_else(|e| panic!("{case}: {e}"));
        let hex = |bytes: Vec<u8>| -> String { bytes.iter().map(|b| format!("{b:02x}")).collect() };
        self.services.push(format!("{case}/request\t{ty}_Request\t{}", hex(r2r::WrappedTypesupport::to_serialized_bytes(&request).unwrap())));
        self.services.push(format!("{case}/response\t{ty}_Response\t{}", hex(r2r::WrappedTypesupport::to_serialized_bytes(&response).unwrap())));
    }

    fn inbound<T: Msg>(&mut self, case: &str, _topic: &str, frame: Value) {
        let value: T = serde_json::from_value(frame).unwrap_or_else(|e| panic!("{case}: {e}"));
        self.add(case, &value);
    }
}

#[test]
fn rmw_writes_these_bytes_for_the_corpus_s_values() {
    robot_rmw();
    let mut cdr = Cdr::default();
    fixtures::corpus(&mut cdr);
    for (file, mut lines) in [("json_corpus_cdr.txt", cdr.lines), ("json_corpus_srv_cdr.txt", cdr.services)] {
        lines.sort();
        let text = lines.join("\n") + "\n";
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../../deploy/test/golden/ros_facts").join(file);
        if std::env::var("MOWER_GOLDEN").as_deref() == Ok("write") {
            std::fs::write(&path, &text).unwrap();
            continue;
        }
        assert!(
            std::fs::read_to_string(&path).unwrap_or_default() == text,
            "{} is not what rmw writes for the corpus (tools/oracle/regen.sh facts rewrites it)",
            path.display()
        );
    }
}
