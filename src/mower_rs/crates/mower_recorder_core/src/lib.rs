//! The Rust recorder's core (E5-2, E5-3b), without ROS: everything of
//! `mower_recorder` that is not a node, byte for byte what the Python
//! recorder writes and sends, so that the `bag` module (E5-3) only wires it
//! to the bus, DDS and the clock. Nothing calls it yet; nothing the robot
//! records changes.
//!
//! - [`profile`]: the profile YAMLs, the groups and their `ros2 bag record`
//!   lines and `mower_mcap::BagOptions`, the hidden-name rule, the disk guard.
//! - [`run`]: the run id, `run_metadata.yaml`, `/mower_recorder/status`, the
//!   rear light, the reply texts, the parameter-dump file names.
//! - [`dc_metadata`], [`extrinsics`]: data_collection's spec 1.6 metadata,
//!   `calib/`, and the camera extrinsics (camera_extrinsics.py), bit for bit.
//! - [`store`]: `/mower_recorder/bags`, the commands and their replies, the
//!   R2 settings (both precedences), the upload gate.
//! - [`manifest`]: the upload (run_manifest.py): the file walk, readiness,
//!   the state file, the HEAD pass, `_manifest.json` last, the marker, delete
//!   and verify; [`s3`]: the R2 client, SigV4 by hand over ureq (D8).
//! - [`mediamtx`], [`graph`], [`params`]: the video PATCHes, `/graph_*`, and
//!   `ros2 param dump`'s format.
//! - [`recording`] (E5-3b): one group of a run, a writer thread with its
//!   `mower_mcap::BagWriter` fed through a byte-bounded queue, and the
//!   reader-QoS rule and `qos_overrides.yaml` the sources subscribe with.
//! - [`manager`] (E5-3b): recorder_manager's run logic (start, stop,
//!   snapshot, the 2 s tick, the fault, the metadata and parameter dump)
//!   behind small traits for what the `bag` module provides.
//!
//! The JSON and YAML go through `mower_pyfmt` (Python's json and PyYAML).
//! Where a Python exception's text would reach a reply or `upload_error`,
//! the Rust text says the same thing differently (A9).

pub mod dc_metadata;
pub mod extrinsics;
pub mod graph;
pub mod manager;
pub mod manifest;
pub mod mediamtx;
pub mod params;
pub mod profile;
pub mod recording;
pub mod run;
pub mod s3;
pub mod store;
pub mod util;
