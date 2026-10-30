//! rosbag2's MCAP bags, written without ROS: what `ros2 bag record -s mcap`
//! (rosbag2 Jazzy 0.26.11 with libmcap 1.3.1, the robot image's) writes,
//! byte for byte, for the recorder that replaces mower_recorder (E5).
//!
//! - [`McapWriter`]: libmcap 1.3.1's writer with rosbag2's options:
//!   chunking, chunk compression, the lazily written Schema and Channel
//!   records, the message and chunk indexes, Statistics and the summary
//!   with its CRC (`writer.rs` says how).
//! - [`BagWriter`]: rosbag2 on top: the bag directory, `<dir>_<n>.mcap`
//!   files, the open-time and final `rosbag2` metadata records of each file,
//!   `metadata.yaml`, the size and duration splits, per-message zstd
//!   (`--compression-mode message`), the `zstd_fast` preset and snapshot
//!   mode (`bag.rs`).
//! - [`BagMetadata`], [`TopicMetadata`], [`Qos`]: rosbag2's metadata and its
//!   YAML (yaml-cpp's dialect), the channels' `offered_qos_profiles`.
//! - [`stdmap`]: libstdc++'s `unordered_map` iteration order, which four of
//!   the orders on disk follow.
//!
//! Schemas come from `mower_schema`. The only field of a file that does not
//! depend on the input is the header's `library`, which is the caller's
//! (D1: the recorder names itself; a test passes libmcap's to compare whole
//! files). `tests/rewrite.rs` rewrites every E5-0 recording
//! (deploy/test/golden/recorder) from its messages and compares the result
//! with what rosbag2 wrote (the design's G1).

mod bag;
mod crc32;
mod meta;
mod records;
pub mod stdmap;
mod writer;

pub use bag::{BagOptions, BagWriter, Preset};
pub use meta::{BagMetadata, FileInformation, Qos, TopicInformation, TopicMetadata, INFINITE};
pub use writer::{zstd_compress_message, ChunkCompression, McapWriter, WriterOptions, DEFAULT_CHUNK_SIZE};
