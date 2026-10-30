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
//! - [`repr::float`]: `repr(float)`, which both of them use;
//! - [`yaml::safe_load`]: PyYAML 6.0.1's `yaml.safe_load` (SafeLoader, YAML
//!   1.1 resolution), for the profiles, the operator's session and
//!   calibration files, and the files the recorder reads back
//!   (`run_metadata.yaml` on a rename, rosbag2's `metadata.yaml`);
//! - [`json::loads`]: CPython 3.12's `json.loads`.
//!
//! The value model is [`Py`]: what `json.loads` / `yaml.safe_load` return for
//! these documents, with dicts in insertion order. `tests/vectors.rs` holds
//! the emitters to vectors the real Python wrote (`tests/oracle/pyfmt_gen.py`),
//! `tests/load_vectors.rs` the loaders (`tests/oracle/pyload_gen.py`), both
//! generated in the robot image.

pub mod json;
mod load;
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
    /// A `datetime.date`: what `yaml.safe_load` makes of a date-only YAML
    /// timestamp (`2026-10-30`).
    Date { year: i32, month: u32, day: u32 },
    /// A `datetime.datetime` from a YAML timestamp with a time
    /// (`construct_yaml_timestamp`): the fraction cut or padded to
    /// microseconds; `offset_minutes` is the zone (`Z` = 0, `+hh[:mm]`),
    /// `None` for a naive one.
    DateTime {
        year: i32,
        month: u32,
        day: u32,
        hour: u32,
        minute: u32,
        second: u32,
        microsecond: u32,
        offset_minutes: Option<i32>,
    },
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
    /// `type(v).__name__`
    pub fn type_name(&self) -> &'static str {
        match self {
            Py::None => "NoneType",
            Py::Bool(_) => "bool",
            Py::Int(_) => "int",
            Py::Float(_) => "float",
            Py::Str(_) => "str",
            Py::List(_) => "list",
            Py::Dict(_) => "dict",
            Py::Date { .. } => "date",
            Py::DateTime { .. } => "datetime",
        }
    }
    /// `d.isoformat()` of a date, `d.isoformat(' ')` of a datetime (what
    /// PyYAML's SafeRepresenter writes); `None` for other values.
    pub fn isoformat(&self) -> Option<String> {
        match *self {
            Py::Date { year, month, day } => Some(format!("{year:04}-{month:02}-{day:02}")),
            Py::DateTime { year, month, day, hour, minute, second, microsecond, offset_minutes } => {
                let mut s = format!("{year:04}-{month:02}-{day:02} {hour:02}:{minute:02}:{second:02}");
                if microsecond != 0 {
                    s.push_str(&format!(".{microsecond:06}"));
                }
                if let Some(off) = offset_minutes {
                    let sign = if off < 0 { '-' } else { '+' };
                    let a = off.unsigned_abs();
                    s.push_str(&format!("{sign}{:02}:{:02}", a / 60, a % 60));
                }
                Some(s)
            }
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
