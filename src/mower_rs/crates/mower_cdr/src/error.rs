use std::fmt;

/// Why a value could not be serialized or a buffer decoded. `path` is the
/// field, dotted from the root (`pose.covariance`, `points[3].x`).
#[derive(Debug, Clone, PartialEq)]
pub enum Error {
    /// The type is not in mower_schema's table, or its `.msg` did not parse.
    Schema(String),
    /// The value does not have the shape the schema says (a missing or
    /// misnamed field, a number where a string belongs, ...).
    Shape { path: String, message: String },
    /// A fixed-size array with another length.
    FixedLength { path: String, want: usize, got: usize },
    /// A bounded sequence or string over its bound.
    Bound { path: String, bound: usize, got: usize },
    /// A NUL inside a string.
    Nul { path: String },
    /// `decode`: the buffer is not this type's CDR.
    Malformed { offset: usize, message: String },
    /// A serde error raised by the value's own Serialize / Deserialize.
    Custom(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Schema(m) => write!(f, "schema: {m}"),
            Error::Shape { path, message } => write!(f, "{path}: {message}"),
            Error::FixedLength { path, want, got } => write!(f, "{path}: fixed size {want}, got {got}"),
            Error::Bound { path, bound, got } => write!(f, "{path}: bounded by {bound}, got {got}"),
            Error::Nul { path } => write!(f, "{path}: NUL in a string"),
            Error::Malformed { offset, message } => write!(f, "malformed CDR at byte {offset}: {message}"),
            Error::Custom(m) => f.write_str(m),
        }
    }
}

impl std::error::Error for Error {}

impl serde::ser::Error for Error {
    fn custom<T: fmt::Display>(msg: T) -> Self {
        Error::Custom(msg.to_string())
    }
}

impl serde::de::Error for Error {
    fn custom<T: fmt::Display>(msg: T) -> Self {
        Error::Custom(msg.to_string())
    }
}

pub(crate) fn shape(path: &str, message: impl Into<String>) -> Error {
    Error::Shape { path: if path.is_empty() { "(root)".into() } else { path.to_string() }, message: message.into() }
}

pub(crate) fn join(path: &str, field: &str) -> String {
    if path.is_empty() {
        field.to_string()
    } else {
        format!("{path}.{field}")
    }
}
