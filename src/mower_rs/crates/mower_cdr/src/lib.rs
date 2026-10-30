//! CDR for ROS 2 messages without ROS, guided by the message's schema
//! (`mower_schema`), byte for byte what rmw writes on the robot.
//!
//! - **The form:** XCDR1 little endian, as rmw_cyclonedds serializes for ROS 2:
//!   the encapsulation header `00 01 00 00`, then the fields in declaration
//!   order, each primitive aligned to its size counted from the byte after
//!   the header, a string as `u32` length + 1, its bytes and a NUL, a
//!   sequence as a `u32` count and its elements, a fixed array as its
//!   elements alone. [`to_vec`] and [`encode`] pad the result with zeros to
//!   a multiple of 4, as every sample rosbag2 records off the wire is (E5
//!   design 2.1); [`decode`] takes either form.
//! - **Two ways in:** a typed value through serde ([`to_vec`]: the bus's r2r
//!   types, whose fixed arrays are `Vec`s and need the schema to drop their
//!   length), or a dynamic [`Value`] ([`encode`]). [`decode`] gives a
//!   `Value`, which keeps every float bit (NaN included, which JSON cannot),
//!   and [`from_value`] turns it into a typed value: what E8's replay needs.
//! - **What r2r refuses is refused** ([`Error::FixedLength`],
//!   [`Error::Bound`], [`Error::Nul`]): a fixed array of the wrong length,
//!   a bounded sequence or string over its bound, a NUL in a string. r2r
//!   asserts on the first two and panics on the third when it converts a
//!   value to its C message (r2r_msg_gen `copy_to_native`), so such a value
//!   never reached DDS; mower_bus_ros drops it with a count
//!   (`convert.rs`). `wstring` is not supported (no recorded type has one).
//!
//! `tests/rmw_oracle.rs` holds this to rmw itself (r2r's
//! `to_serialized_bytes`) on every type of `mower_schema`'s table, and
//! `tests/fixtures.rs` to every payload of the E5-0 recordings.

mod de;
mod decode;
mod encode;
mod error;
mod ser;
mod value;

pub use de::from_value;
pub use decode::decode;
pub use encode::encode;
pub use error::Error;
pub use ser::to_vec;
pub use value::Value;

/// The encapsulation header of little-endian plain CDR.
pub const HEADER: [u8; 4] = [0, 1, 0, 0];

/// Zero-pads a serialized message to a multiple of 4 bytes, the form rosbag2
/// records (rmw's own `serialize` leaves it unpadded).
pub fn pad4(mut bytes: Vec<u8>) -> Vec<u8> {
    while bytes.len() % 4 != 0 {
        bytes.push(0);
    }
    bytes
}

pub(crate) fn schema(type_name: &str) -> Result<&'static mower_schema::MsgDef, Error> {
    mower_schema::message(type_name).map_err(|e| Error::Schema(e.to_string()))
}
