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
    v.as_f64().unwrap_or_else(|| panic!("number expected, got {v}"))
}

pub fn vec_f(v: &Value) -> Vec<f64> {
    v.as_array().expect("array").iter().map(f).collect()
}

/// Floats from the oracle agree to this. The nav2 binaries are GCC builds for
/// aarch64, which contract `a + b * c` into FMA (`-ffp-contract=fast`) and use
/// glibc's libm; this crate does neither, so the last bit can differ.
pub const TOL: f64 = 1e-12;

pub fn close(a: f64, b: f64) -> bool {
    a == b || (a - b).abs() <= TOL * (1.0 + a.abs().max(b.abs()))
}

#[track_caller]
pub fn assert_close(a: f64, b: f64, what: &str) {
    assert!(close(a, b), "{what}: {a} vs {b}");
}
