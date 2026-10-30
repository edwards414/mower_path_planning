//! What the tests share: the vectors file and its typed values.
#![allow(dead_code)]

use mower_pyfmt::Py;
use serde_json::Value;

pub fn vectors() -> Value {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/vectors/extrinsics.json");
    serde_json::from_str(&std::fs::read_to_string(path).expect("tests/vectors/extrinsics.json")).unwrap()
}

/// A float as the generator wrote it (`repr`).
pub fn f(v: &Value) -> f64 {
    match v.as_str().expect("a repr string") {
        "nan" => f64::NAN,
        "inf" => f64::INFINITY,
        "-inf" => f64::NEG_INFINITY,
        s => s.parse().unwrap(),
    }
}

pub fn fs(v: &Value) -> Vec<f64> {
    v.as_array().unwrap().iter().map(f).collect()
}

/// A typed value (tags N, B, I, F, S, L, D).
pub fn decode(v: &Value) -> Py {
    let a = v.as_array().expect("typed value is an array");
    match a[0].as_str().unwrap() {
        "N" => Py::None,
        "B" => Py::Bool(a[1].as_bool().unwrap()),
        "I" => Py::Int(a[1].as_str().unwrap().parse().unwrap()),
        "F" => Py::Float(f(&a[1])),
        "S" => Py::Str(a[1].as_str().unwrap().to_string()),
        "L" => Py::List(a[1].as_array().unwrap().iter().map(decode).collect()),
        "D" => Py::Dict(a[1].as_array().unwrap().iter().map(|kv| (kv[0].as_str().unwrap().to_string(), decode(&kv[1]))).collect()),
        t => panic!("unknown tag {t}"),
    }
}

/// Equality with floats bit for bit (any NaN equals any NaN).
pub fn same(a: &Py, b: &Py) -> bool {
    match (a, b) {
        (Py::Float(x), Py::Float(y)) => x.to_bits() == y.to_bits() || (x.is_nan() && y.is_nan()),
        (Py::List(x), Py::List(y)) => x.len() == y.len() && x.iter().zip(y).all(|(p, q)| same(p, q)),
        (Py::Dict(x), Py::Dict(y)) => x.len() == y.len() && x.iter().zip(y).all(|((k, p), (l, q))| k == l && same(p, q)),
        _ => a == b,
    }
}

pub fn bits_eq(a: f64, b: f64) -> bool {
    a.to_bits() == b.to_bits() || (a.is_nan() && b.is_nan())
}

pub fn strings(v: &Value) -> Vec<String> {
    v.as_array().unwrap().iter().map(|s| s.as_str().unwrap().to_string()).collect()
}
