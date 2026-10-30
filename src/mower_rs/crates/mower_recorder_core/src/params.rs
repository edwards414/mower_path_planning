//! `params/<node>.yaml`: what `ros2 param dump <node>` prints (ros2param
//! `verb/dump.py`, Jazzy): the names sorted, a dotted name nested
//! (`insert_dict`), under `{node: {ros__parameters: ...}}`, through
//! `yaml.dump(..., default_flow_style=False)` and `print` (one more `\n`).

use mower_pyfmt::yaml::{self, YamlOptions};
use mower_pyfmt::Py;

/// A parameter's value as `get_parameters` returns it.
#[derive(Debug, Clone, PartialEq)]
pub enum ParamValue {
    NotSet,
    Bool(bool),
    Integer(i64),
    Double(f64),
    String(String),
    BoolArray(Vec<bool>),
    IntegerArray(Vec<i64>),
    DoubleArray(Vec<f64>),
    StringArray(Vec<String>),
}

impl ParamValue {
    /// `parameter_value_to_python`
    pub fn to_py(&self) -> Py {
        match self {
            ParamValue::NotSet => Py::None,
            ParamValue::Bool(b) => Py::Bool(*b),
            ParamValue::Integer(i) => Py::Int(*i),
            ParamValue::Double(d) => Py::Float(*d),
            ParamValue::String(s) => Py::str(s),
            ParamValue::BoolArray(v) => Py::List(v.iter().map(|b| Py::Bool(*b)).collect()),
            ParamValue::IntegerArray(v) => Py::List(v.iter().map(|i| Py::Int(*i)).collect()),
            ParamValue::DoubleArray(v) => Py::List(v.iter().map(|d| Py::Float(*d)).collect()),
            ParamValue::StringArray(v) => Py::List(v.iter().map(|s| Py::str(s)).collect()),
        }
    }
}

/// `insert_dict(dictionary, key, value)`; `Err` where the Python would raise
/// (a name that is also the prefix of a dotted one, the other way round).
fn insert(d: &mut Vec<(String, Py)>, key: &str, value: Py) -> Result<(), String> {
    match key.split_once('.') {
        Some((head, rest)) => {
            if !d.iter().any(|(k, _)| k == head) {
                d.push((head.to_string(), Py::Dict(Vec::new())));
            }
            match d.iter_mut().find(|(k, _)| k == head).map(|(_, v)| v) {
                Some(Py::Dict(sub)) => insert(sub, rest, value),
                _ => Err(format!("'{}' object does not support item assignment", "str")),
            }
        }
        None => {
            match d.iter_mut().find(|(k, _)| k == key) {
                Some(slot) => slot.1 = value,
                None => d.push((key.to_string(), value)),
            }
            Ok(())
        }
    }
}

/// The dump of `node` (its full name, `/map_manage`) with these parameters.
pub fn dump(node: &str, params: &[(String, ParamValue)]) -> Result<String, String> {
    let mut sorted: Vec<&(String, ParamValue)> = params.iter().collect();
    sorted.sort_by(|a, b| a.0.cmp(&b.0));
    let mut ros = Vec::new();
    for (name, v) in sorted {
        insert(&mut ros, name, v.to_py())?;
    }
    let doc = Py::dict([(node, Py::dict([("ros__parameters", Py::Dict(ros))]))]);
    Ok(yaml::dump(&doc, &YamlOptions::default()) + "\n")
}
