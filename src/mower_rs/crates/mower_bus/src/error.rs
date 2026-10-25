//! The bus's two error types.

use std::fmt;

/// Why creating or using an endpoint failed.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BusError {
    /// Not an absolute ROS name (see `check_name`).
    InvalidName { name: String, reason: &'static str },
    /// `name` already carries another type; the first typed endpoint fixed it.
    TypeMismatch { name: String, existing: &'static str, requested: &'static str },
    /// The endpoint's scope is closed (the module instance ended).
    Closed { scope: String },
    /// Another live server already serves `name`.
    ServiceTaken { name: String, server: String },
    /// No server for `name`, or it went away before answering.
    ServiceUnavailable { name: String },
    /// The server dropped the request without answering.
    NoResponse { name: String },
    /// Erased access to a name whose type no typed endpoint has fixed yet.
    UnknownName { name: String },
    /// JSON encoding or decoding with the name's codec failed.
    Json { name: String, error: String },
}

impl fmt::Display for BusError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BusError::InvalidName { name, reason } => write!(f, "invalid bus name {name:?}: {reason}"),
            BusError::TypeMismatch { name, existing, requested } => {
                write!(f, "{name} carries {existing}, not {requested}")
            }
            BusError::Closed { scope } => write!(f, "bus scope {scope:?} is closed"),
            BusError::ServiceTaken { name, server } => write!(f, "service {name} is already served by {server:?}"),
            BusError::ServiceUnavailable { name } => write!(f, "service {name} is not available"),
            BusError::NoResponse { name } => write!(f, "service {name} dropped the request without answering"),
            BusError::UnknownName { name } => write!(f, "{name}: type not known yet (no typed endpoint)"),
            BusError::Json { name, error } => write!(f, "{name}: JSON: {error}"),
        }
    }
}

impl std::error::Error for BusError {}

/// Why a non-async receive returned without a message.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RecvError {
    /// `try_recv`: nothing queued.
    Empty,
    /// `recv_timeout`: nothing arrived in time.
    Timeout,
    /// The receiver's scope is closed; nothing will arrive any more.
    Closed,
}

impl fmt::Display for RecvError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            RecvError::Empty => "no message queued",
            RecvError::Timeout => "timed out waiting for a message",
            RecvError::Closed => "bus scope closed",
        })
    }
}

impl std::error::Error for RecvError {}
