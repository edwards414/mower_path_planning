//! What the Python recorder writes with the Python standard library and
//! PyYAML, reproduced byte for byte so that the Rust recorder writes the
//! same files and messages (E5 design 4.4, decision D2):
//!
//! - [`json::dumps`]: CPython 3.12's `json.dumps` (`ensure_ascii`, `indent`,
//!   `sort_keys`, the default separators, `allow_nan`), for the status, bags,
//!   command_result, rear light and graph messages, `_manifest.json`,
//!   `.upload_state.json` and `.uploaded`;
//! - [`yaml::safe_dump`] / [`yaml::dump`]: PyYAML 6.0.1's block-style
//!   emitter (indent 2, width 80), for `run_metadata.yaml`, `params/*.yaml`
//!   and the calib files;
//! - [`repr::float`]: `repr(float)`, which both of them use.
//!
//! The value model is [`Py`]: what `json.loads` / `yaml.safe_load` return for
//! these documents, with dicts in insertion order. `tests/vectors.rs` holds
//! all of it to vectors the real Python wrote (`tests/oracle/pyfmt_gen.py`,
//! run in the robot image).

pub mod json;
pub mod repr;
pub mod yaml;

/// A Python value of the kinds the recorder emits.
#[derive(Debug, Clone, PartialEq)]
pub enum Py {
    None,
    Bool(bool),
    Int(i64),
    Float(f64),
    Str(String),
    List(Vec<Py>),
    /// A `dict` in insertion order. Keys are strings: that is all the
    /// recorder writes, and `json.dumps` turns other keys into strings anyway.
    Dict(Vec<(String, Py)>),
}

impl Py {
    pub fn str(s: &str) -> Py {
        Py::Str(s.to_string())
    }
    /// A dict from `(key, value)` pairs, in that order.
    pub fn dict<K: Into<String>>(items: impl IntoIterator<Item = (K, Py)>) -> Py {
        Py::Dict(items.into_iter().map(|(k, v)| (k.into(), v)).collect())
    }
    /// `d.get(key)` on a dict.
    pub fn get(&self, key: &str) -> Option<&Py> {
        match self {
            Py::Dict(items) => items.iter().find(|(k, _)| k == key).map(|(_, v)| v),
            _ => None,
        }
    }
}

impl From<&str> for Py {
    fn from(s: &str) -> Py {
        Py::Str(s.to_string())
    }
}
impl From<String> for Py {
    fn from(s: String) -> Py {
        Py::Str(s)
    }
}
impl From<i64> for Py {
    fn from(v: i64) -> Py {
        Py::Int(v)
    }
}
impl From<f64> for Py {
    fn from(v: f64) -> Py {
        Py::Float(v)
    }
}
impl From<bool> for Py {
    fn from(v: bool) -> Py {
        Py::Bool(v)
    }
}
impl<T: Into<Py>> From<Option<T>> for Py {
    fn from(v: Option<T>) -> Py {
        v.map_or(Py::None, Into::into)
    }
}
