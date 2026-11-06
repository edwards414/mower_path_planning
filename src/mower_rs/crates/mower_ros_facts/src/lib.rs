//! No code: the tests of the frozen ROS facts (E9-06), in a crate of their own
//! so that they build and run with no ROS at all. `tests/`:
//!
//! - `resolver.rs`: rcl's resolution of every manifest name, against
//!   `mower_rs_common::names`;
//! - `cdr_vectors.rs`: the bytes rmw wrote for every `mower_schema` type,
//!   against `mower_cdr`;
//! - `recorded_payloads.rs`: every payload rosbag2 recorded in E5-0, the same;
//! - `json_corpus.rs`: E3's JSON corpus, against `mower_cdr`'s reading of the
//!   bytes rmw wrote for the same values;
//! - `frozen.rs`: the graph, the parameters, the log: what they must not hold,
//!   how they agree with each other, and that no file changed behind
//!   `SHA256SUMS`.
//!
//! What the files are and how they are made: `deploy/test/golden/ros_facts/README.md`.
