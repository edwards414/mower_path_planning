//! A rule of mower_gps that only its source shows (E9-07, from
//! test_controller_configuration.py; see test_support/source_check.rs). A
//! receiver that dies, goes quiet or never starts is the unit test
//! `receiver_rules` in src/lib.rs.

#[path = "../../../test_support/source_check.rs"]
mod source_check;

use source_check::Source;

/// End of file on the port -- what a hung-up tty reads -- ends the driver
/// like a read error, never as a quiet loop. A source check: on a pseudo
/// terminal the serialport crate reports the hangup as an error first, which
/// `receiver_rules::a_receiver_that_is_unplugged_fails_the_module` runs.
#[test]
fn end_of_file_on_the_port_ends_the_driver() {
    let read = Source::new("mower_gps/src/lib.rs", include_str!("../src/lib.rs")).body("read_frames");
    read.has("Ok(0) => Err(");
    read.says("GPS serial port returned EOF (receiver unplugged?)");
}
