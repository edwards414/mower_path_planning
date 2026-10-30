//! Rules of mower_imu that only its source shows (E9-07, from
//! test_controller_configuration.py; see test_support/source_check.rs). The
//! 88-byte backlog, the 200 ms freshness of a sample's parts and a port that
//! dies are the unit test `serial_rules` in src/lib.rs.

#[path = "../../../test_support/source_check.rs"]
mod source_check;

use source_check::Source;

fn lib() -> Source {
    Source::new("mower_imu/src/lib.rs", include_str!("../src/lib.rs"))
}

/// When the driver's own loop was held up for longer than a sample's parts
/// stay fresh (a host scheduling gap), what waits on the port is stale: it is
/// dropped unread, with the parts. A source check: a test cannot stall the
/// driver's thread.
#[test]
fn a_host_timing_gap_drops_the_backlog() {
    let serial = lib().body("run(&self");
    serial.has("if poll_at.duration_since(last_poll_at) > COMPONENT_TIMEOUT {");
    serial.before("if poll_at.duration_since(last_poll_at) > COMPONENT_TIMEOUT {", "port.read(");
    serial.says("Discarded IMU serial backlog after a host timing gap");
}

/// What the driver logs when it drops a backlog or a sample, as
/// wit_ros2_imu.py does (the drops themselves are `serial_rules`).
#[test]
fn the_drops_are_logged() {
    let serial = lib().body("run(&self");
    serial.says("Discarded oversized IMU serial backlog");
    serial.says("Dropping IMU orientation frame because acceleration/gyro components are stale");
}
