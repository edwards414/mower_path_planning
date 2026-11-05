//! What mower_agent says to the backend and in its log, as the Python fleet
//! agent did (E9-07, from test_controller_configuration.py; see
//! test_support/source_check.rs). The constants, the relay framing and the
//! signed headers are the unit test `rclpy_parity` in src/lib.rs; these are
//! the endpoint paths and messages of code that talks to the backend, the
//! gate and MediaMTX over the network.

#[path = "../../../test_support/source_check.rs"]
mod source_check;

use source_check::Source;

#[test]
fn the_backend_endpoints_and_the_log_lines_are_the_python_agent_s() {
    let lib = Source::new("mower_agent/src/lib.rs", include_str!("../src/lib.rs"));
    for text in [
        "/v1/robots/register",
        "/v1/relay/robot/",
        "/turn",
        "/v3/config/global/patch",
        "webrtcICEServers2",
        "MOWER_BACKEND_URL not set: agent idle (development mode)",
        "MOWER_PROVISION_TOKEN not set: skipping registration",
        "relay refused us: HTTP",
        "reconnecting in",
        "gate refused",
        "gate unreachable",
        "no such session",
        "session ended",
        "relay lost",
        "not relayed",
        "camera server unreachable",
        "not implemented yet",
        "another device_key holds this robot_id",
        "bad provision token",
    ] {
        lib.says(text);
    }
}

/// E8-2: ws_bridge's loopback listener (no gate) reads what the phones may
/// not: every session of a phone goes to the pairing gate, and only the
/// agent's own watch of /robot/info and /robot/telemetry to that listener.
#[test]
fn a_relayed_session_goes_to_the_gate_and_never_to_the_loopback_listener() {
    let lib = Source::new("mower_agent/src/lib.rs", include_str!("../src/lib.rs"));
    let session = lib.body("open_session");
    session.has("agent.gate_url.as_str()");
    session.lacks("rosbridge_url");
    lib.body("watch_topics_forever").has("self.rosbridge_url.as_str()");
    // the field, its value at the start and the watch: no other use
    assert_eq!(lib.count("rosbridge_url"), 3);
}
