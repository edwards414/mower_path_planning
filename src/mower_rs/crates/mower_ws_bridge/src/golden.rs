//! The golden corpus (tests/golden) without DDS: each case through the bus
//! (an erased reader's `to_json`, a `JsonClient`, a `JsonPublisher`) and
//! ws_bridge's own frame functions. The bytes must be the golden ones, which
//! are what r2r's untyped path, ws_bridge's r2r backend, gives
//! (mower_bus_ros's tests/dds.rs checks that side and writes the files).

use futures::StreamExt;
use mower_bus::{Bus, Qos, Scope};
use serde_json::{json, Value};

use crate::client::service_response_frame;
use crate::entry::publish_frame;

#[path = "../tests/golden/fixtures.rs"]
mod fixtures;

/// What the bus gave, case by case, for each golden file.
#[derive(Default)]
struct Found {
    publish: Vec<(String, String)>,
    service_response: Vec<(String, String)>,
    request: Vec<(String, String)>,
    inbound: Vec<(String, String)>,
}

/// The cases on a bus of their own: `module` stands for the typed modules,
/// `bridge` for ws_bridge's scope. Each case has a name of its own.
struct BusCorpus {
    module: Scope,
    bridge: Scope,
    found: Found,
}

fn name(case: &str) -> String {
    format!("/golden/{case}")
}

impl fixtures::Corpus for BusCorpus {
    fn publish<T: fixtures::Msg>(&mut self, case: &str, topic: &str, value: T) {
        let publisher = self.module.publisher::<T>(&name(case), Qos::latched()).unwrap();
        publisher.publish(&value).unwrap();
        let mut reader = self.bridge.subscribe_erased(&name(case), Qos::latched().keep_last(10)).unwrap();
        let msg = reader.try_recv().expect("the retained value");
        let frame = publish_frame(topic, &msg.to_json().unwrap());
        self.found.publish.push((case.to_string(), frame.to_string()));
    }

    fn service<S: fixtures::Srv>(&mut self, case: &str, service: &str, id: &str, args: Value, response: S::Response) {
        let mut server = self.module.service::<S::Request, S::Response>(&name(case)).unwrap();
        let client = self.bridge.client_json(&name(case)).unwrap();
        let serve = async {
            let request = server.next().await.expect("a request");
            let decoded = request.message.clone();
            request.respond(response).unwrap();
            decoded
        };
        let (answer, decoded) = futures::executor::block_on(futures::future::join(client.call(args), serve));
        let frame = service_response_frame(service, Ok(answer.unwrap()), Some(json!(id)));
        self.found.service_response.push((case.to_string(), frame));
        self.found.request.push((case.to_string(), serde_json::to_value(&decoded).unwrap().to_string()));
    }

    fn inbound<T: fixtures::Msg>(&mut self, case: &str, _topic: &str, frame: Value) {
        let mut module_reader = self.module.subscribe::<T>(&name(case), Qos::default()).unwrap();
        self.bridge.publisher_json(&name(case), Qos::default()).unwrap().publish(&frame).unwrap();
        let decoded = module_reader.try_recv().expect("decoded");
        self.found.inbound.push((case.to_string(), serde_json::to_value(&*decoded).unwrap().to_string()));
    }
}

#[test]
fn the_bus_gives_the_golden_corpus_byte_for_byte() {
    let bus = Bus::new();
    let mut corpus = BusCorpus { module: bus.scope("module"), bridge: bus.scope("mower_ws_bridge"), found: Found::default() };
    fixtures::corpus(&mut corpus);
    let mut found = corpus.found;
    let mut problems = Vec::new();
    found.publish.retain(|(case, text)| match fixtures::own_file(case) {
        Some(golden) => {
            if golden != text {
                problems.push(format!("{case}: the bus's frame is not the captured one"));
            }
            false
        }
        None => true,
    });
    for (file, golden, texts) in [
        ("publish.txt", include_str!("../tests/golden/publish.txt"), &found.publish),
        ("service_response.txt", include_str!("../tests/golden/service_response.txt"), &found.service_response),
        ("request.txt", include_str!("../tests/golden/request.txt"), &found.request),
        ("inbound.txt", include_str!("../tests/golden/inbound.txt"), &found.inbound),
    ] {
        problems.extend(fixtures::golden_differences(file, golden, texts));
    }
    assert!(problems.is_empty(), "the bus's frames are not the golden ones:\n{}", problems.join("\n"));
}

/// The frames' bytes rest on `serde_json::Value` sorting its keys. Should a
/// dependency ever unify serde_json's `preserve_order` feature into the
/// build, the keys would keep their insertion order and every frame would
/// change: this fails first.
#[test]
fn a_json_value_sorts_its_keys() {
    assert_eq!(json!({"b": 1, "a": 2}).to_string(), r#"{"a":2,"b":1}"#);
    assert_eq!(publish_frame("/t", &json!({"z": 0.1f32, "a": -0.0})).as_ref(), r#"{"msg":{"a":-0.0,"z":0.10000000149011612},"op":"publish","topic":"/t"}"#);
}
