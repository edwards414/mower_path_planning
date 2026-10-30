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
