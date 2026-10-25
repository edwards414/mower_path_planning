//! ROS-free port of the nav2 (Jazzy 1.3.13) plugins this mower runs, as
//! configured by `mower_nav2/config/nav2_no_map_params.yaml`: Phase D of
//! `docs/ROS_FREE_PLAN.md`.
//!
//! Pure core: no ROS, no I/O, no clock. Every entry point takes explicit
//! poses, grids and timestamps, so the same code runs live, in shadow mode,
//! in replay and under test. See `README.md` for what is ported and what is
//! deliberately not.

pub mod checkers;
pub mod costmap;
pub mod footprint;
pub mod geometry;
pub mod navfn;
pub mod path_handler;
pub mod planner;
pub mod rpp;
pub mod shim;
pub mod smoother;
