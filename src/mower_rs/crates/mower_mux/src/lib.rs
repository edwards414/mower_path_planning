//! twist_mux, the velocity multiplexer in front of the final guard, as a
//! module of mower_rsd (step E4 of docs/ROS_FREE_PLAN.md).
//!
//! - [`core`]: twist_mux 4.5.0's selection rules, a pure function of when
//!   each input and lock last had a message.
//! - [`config`]: the handles twist_mux builds from its parameters
//!   (twist_mux_topics.yaml), in its order and with its log lines.
//! - [`diag`]: what it publishes on `/diagnostics`.
//!
//! Every rule was measured on the real twist_mux by `tools/mux_oracle.py`;
//! `tests/replay.rs` replays what it recorded.

pub mod config;
pub mod core;
pub mod diag;

pub use crate::config::{from_params, Config, ConfigError, Log, Value};
pub use crate::core::{timeout_ns, Handle, Mux};
