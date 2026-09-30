//! Coverage path planning core: safe-map filtering, zigzag / spiral
//! generation, path validation, A* connectors and boustrophedon cell
//! decomposition.
//!
//! A plain Rust library used by the mower_rs coverage node
//! (`src/mower_rs/crates/mower_coverage`, node `boustrophedon_coverage`).
//! The functions are ports of the former Python implementation in
//! `mower_mission` (removed after commit dff480b); `tests/` pins them to the
//! recorded Python outputs.

pub mod types;
pub mod path_validator;
pub mod safe_map_filter;
pub mod connector_planner;
pub mod zigzag;
pub mod spiral;
pub mod cell_decomposition;
pub mod boustrophedon;
