//! E3's JSON corpus on the plain types, with no ROS (E9-22): its own `fixtures.rs`
//! compiles here, whose imports name r2r, with `r2r` meaning `mower_msgs`. Every case
//! must give the same text, byte for byte, and the same bytes as rmw wrote for it
//! (`json_corpus_cdr.txt`, `json_corpus_srv_cdr.txt`). This is the comparison
//! `tests/json_corpus.rs` could not make of the services: the request as the
//! server's type reads it, and the response the app gets, typed.

// `fixtures.rs` says `use r2r::...`: in this test crate that name is this crate,
// which re-exports mower_msgs and answers for r2r's two traits it names.
extern crate self as r2r;

pub use mower_msgs::ServiceType as WrappedServiceTypeSupport;
pub use mower_msgs::{
    builtin_interfaces, geometry_msgs, mower_interface, rcl_interfaces, sensor_msgs, std_msgs, std_srvs,
};
pub use mower_msgs::MessageType as WrappedTypesupport;

#[path = "../../mower_ws_bridge/tests/golden/fixtures.rs"]
#[allow(dead_code)]
mod fixtures;

use fixtures::{Corpus, Msg, Srv};
use mower_msgs::MessageType;
use serde_json::{json, Value};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

fn golden(name: &str) -> BTreeMap<String, String> {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../mower_ws_bridge/tests/golden");
    let text = std::fs::read_to_string(dir.join(name)).unwrap_or_else(|e| panic!("{name}: {e}"));
    fixtures::parse_golden(&text)
}

fn facts(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../../deploy/test/golden/ros_facts").join(name)
}

/// The corpus's cases with bytes: case -> (type, bytes rmw wrote).
fn vectors(file: &str) -> BTreeMap<String, (String, Vec<u8>)> {
    let text = std::fs::read_to_string(facts(file)).unwrap();
    text.lines()
        .map(|line| {
            let mut cols = line.split('\t');
            let (case, ty, hex) = (cols.next().unwrap(), cols.next().unwrap(), cols.next().unwrap());
            let bytes = (0..hex.len()).step_by(2).map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap()).collect();
            (case.to_string(), (ty.to_string(), bytes))
        })
        .collect()
}

#[derive(Default)]
struct Check {
    publish: BTreeMap<String, String>,
    inbound: BTreeMap<String, String>,
    request: BTreeMap<String, String>,
    response: BTreeMap<String, String>,
    vectors: BTreeMap<String, (String, Vec<u8>)>,
    wrong: Vec<String>,
    seen: BTreeMap<&'static str, Vec<String>>,
}

impl Check {
    fn compare(&mut self, file: &'static str, case: &str, golden: Option<&str>, ours: &str) {
        match golden {
            Some(golden) if golden == ours => {}
            Some(golden) => self.wrong.push(format!(
                "{file} {case}:\n  golden: {}\n  plain:  {}",
                golden.chars().take(400).collect::<String>(),
                ours.chars().take(400).collect::<String>()
            )),
            None => self.wrong.push(format!("{file}: no line for case {case}")),
        }
        self.seen.entry(file).or_default().push(case.to_string());
    }

    /// The case's bytes, if it has any, against `mower_cdr` of the plain value.
    fn bytes<T: MessageType>(&mut self, case: &str, value: &T) {
        match self.vectors.remove(case) {
            Some((ty, rmw)) if ty == T::NAME => {
                let ours = mower_cdr::to_vec(T::NAME, value).unwrap_or_else(|e| panic!("{case}: {e}"));
                if ours != mower_cdr::pad4(rmw) {
                    self.wrong.push(format!("{case} ({ty}): the plain value's bytes are not rmw's"));
                }
            }
            Some((ty, _)) => self.wrong.push(format!("{case}: the vector is a {ty}, the case a {}", T::NAME)),
            None => {
                if fixtures::own_file(case).is_none() {
                    self.wrong.push(format!("{case}: no frozen bytes"));
                }
            }
        }
    }
}

impl Corpus for Check {
    fn publish<T: Msg>(&mut self, case: &str, topic: &str, value: T) {
        let msg = serde_json::to_value(&value).unwrap();
        let frame = json!({"op": "publish", "topic": topic, "msg": msg}).to_string();
        let golden = fixtures::own_file(case).map(|t| t.trim_end().to_string()).or_else(|| self.publish.get(case).cloned());
        self.compare("publish.txt", case, golden.as_deref(), &frame);
        self.bytes(case, &value);
    }

    fn service<S: Srv>(&mut self, case: &str, service: &str, id: &str, args: Value, response: S::Response) {
        // what the server is handed: the type's fields, its defaults filled in
        let request: S::Request = serde_json::from_value(args).unwrap_or_else(|e| panic!("{case}: {e}"));
        let text = serde_json::to_value(&request).unwrap().to_string();
        let golden = self.request.get(case).cloned();
        self.compare("request.txt", case, golden.as_deref(), &text);
        // and what the app gets back
        let values = serde_json::to_value(&response).unwrap();
        let frame = json!({"id": id, "op": "service_response", "result": true, "service": service, "values": values}).to_string();
        let golden = self.response.get(case).cloned();
        self.compare("service_response.txt", case, golden.as_deref(), &frame);
        self.bytes(&format!("{case}/request"), &request);
        self.bytes(&format!("{case}/response"), &response);
    }

    fn inbound<T: Msg>(&mut self, case: &str, _topic: &str, frame: Value) {
        let value: T = serde_json::from_value(frame).unwrap_or_else(|e| panic!("{case}: {e}"));
        let text = serde_json::to_value(&value).unwrap().to_string();
        let golden = self.inbound.get(case).cloned();
        self.compare("inbound.txt", case, golden.as_deref(), &text);
        self.bytes(case, &value);
    }
}

#[test]
fn every_case_of_the_corpus_is_the_same_on_the_plain_types() {
    let mut check = Check {
        publish: golden("publish.txt"),
        inbound: golden("inbound.txt"),
        request: golden("request.txt"),
        response: golden("service_response.txt"),
        vectors: vectors("json_corpus_cdr.txt").into_iter().chain(vectors("json_corpus_srv_cdr.txt")).collect(),
        ..Default::default()
    };
    fixtures::corpus(&mut check);
    assert!(check.wrong.is_empty(), "{}", check.wrong.join("\n"));
    assert!(check.vectors.is_empty(), "frozen bytes of no case: {:?}", check.vectors.keys().collect::<Vec<_>>());
    // and every line of the files was a case: nothing was skipped
    for (file, lines) in [
        ("publish.txt", &check.publish),
        ("inbound.txt", &check.inbound),
        ("request.txt", &check.request),
        ("service_response.txt", &check.response),
    ] {
        let seen = check.seen.get(file).unwrap_or_else(|| panic!("{file}: no case"));
        for case in lines.keys() {
            assert!(seen.contains(case), "{file}: {case} is no case of the corpus");
        }
    }
    assert!(check.seen["request.txt"].len() >= 15, "{:?}", check.seen["request.txt"]);
}
