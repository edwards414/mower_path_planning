//! Helpers shared by the oracle tests.
#![allow(dead_code)]

use serde_json::Value;

pub fn load(name: &str) -> Value {
    let path = format!("{}/tests/data/{}", env!("CARGO_MANIFEST_DIR"), name);
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("cannot read {path}: {e} (run tests/oracle/run_oracle.sh)"));
    serde_json::from_str(&text).unwrap_or_else(|e| panic!("cannot parse {path}: {e}"))
}

pub fn f(v: &Value) -> f64 {
    v.as_f64().expect("number")
}

pub fn vec_f(v: &Value) -> Vec<f64> {
    v.as_array().expect("array").iter().map(f).collect()
}

pub fn vec_b(v: &Value) -> Vec<bool> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|b| b.as_bool().expect("bool"))
        .collect()
}

pub fn mat15(v: &Value) -> [[f64; 15]; 15] {
    let rows = v.as_array().expect("array");
    let mut m = [[0.0f64; 15]; 15];
    for (i, r) in rows.iter().enumerate() {
        let cols = vec_f(r);
        m[i][..15].copy_from_slice(&cols[..15]);
    }
    m
}

pub fn arr15(v: &Value) -> [f64; 15] {
    let values = vec_f(v);
    let mut a = [0.0f64; 15];
    a.copy_from_slice(&values[..15]);
    a
}

pub fn bools15(v: &Value) -> [bool; 15] {
    let values = vec_b(v);
    let mut a = [false; 15];
    a.copy_from_slice(&values[..15]);
    a
}

/// Relative comparison with an absolute floor of `tol` for values near zero.
pub fn close(a: f64, b: f64, tol: f64) -> bool {
    if a == b {
        return true;
    }
    (a - b).abs() <= tol * b.abs().max(1.0)
}

#[track_caller]
pub fn assert_close(a: f64, b: f64, tol: f64, what: &str) -> f64 {
    let err = if b.abs() > 1.0 {
        (a - b).abs() / b.abs()
    } else {
        (a - b).abs()
    };
    assert!(
        close(a, b, tol),
        "{what}: rust {a:.17e} vs oracle {b:.17e} (err {err:.3e} > {tol:.1e})"
    );
    err
}
