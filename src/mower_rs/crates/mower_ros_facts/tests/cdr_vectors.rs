//! E5's G3 frozen: for every type of `mower_schema`'s table, the bytes rmw
//! wrote for values of `random_value` (NaN, infinities, -0.0, integer limits,
//! empty and non-ASCII strings, empty and longer arrays), which
//! `mower_cdr/tests/rmw_oracle.rs` writes and keeps honest while r2r exists.
//! `mower_cdr` must write the same bytes and read the same value back, with no
//! rmw here. (The bytes rmw wrote are unpadded; `mower_cdr` pads to 4, as what
//! rosbag2 records off the wire is.)
//!
//! Each vector also has r2r's JSON of the value: what E9-22's generated types
//! have to serialize to, for all of the table, not only the types of E3's
//! corpus. It must be what `mower_cdr` reads from the bytes, written by the rule
//! in `support/json.rs`.

#[path = "../../mower_cdr/tests/support/random_value.rs"]
#[allow(dead_code)]
mod random_value;

#[path = "support/json.rs"]
mod json;

use random_value::{random, Rng};
use std::collections::BTreeMap;
use std::path::Path;

fn unhex(s: &str) -> Vec<u8> {
    assert!(s.len() % 2 == 0, "odd hex length");
    (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
}

#[test]
fn mower_cdr_writes_and_reads_what_rmw_wrote() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../../deploy/test/golden/ros_facts/cdr_vectors.txt");
    let text = std::fs::read_to_string(path).unwrap();
    let mut per_type: BTreeMap<String, usize> = BTreeMap::new();
    let mut bytes_total = 0;
    for line in text.lines() {
        let mut cols = line.split('\t');
        let (name, index, hex, r2r_json) = (cols.next().unwrap(), cols.next().unwrap(), cols.next().unwrap(), cols.next().unwrap());
        let index: u32 = index.parse().unwrap();
        let rmw = unhex(hex);
        let value = random(&mut Rng::for_vector(name, index), name, 0);
        let ours = mower_cdr::encode(name, &value).unwrap_or_else(|e| panic!("{name} #{index}: encode: {e}"));
        assert!(ours == mower_cdr::pad4(rmw.clone()), "{name} #{index}: mower_cdr does not write what rmw wrote");
        let back = mower_cdr::decode(name, &rmw).unwrap_or_else(|e| panic!("{name} #{index}: decode: {e}"));
        assert!(back == value, "{name} #{index}: mower_cdr does not read what rmw wrote");
        let ours = json::to_json(&back).to_string();
        assert!(ours == r2r_json, "{name} #{index}: not r2r's JSON\n  r2r:    {r2r_json}\n  schema: {ours}");
        *per_type.entry(name.to_string()).or_default() += 1;
        bytes_total += rmw.len();
    }
    let names: Vec<&str> = per_type.keys().map(String::as_str).collect();
    assert_eq!(names, mower_schema::names(), "the vectors are not of this table's types");
    assert!(per_type.values().all(|n| *n == per_type.values().next().copied().unwrap()), "{per_type:?}");
    eprintln!("{} types, {} vectors, {bytes_total} bytes of CDR", per_type.len(), text.lines().count());
}
