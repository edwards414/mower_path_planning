//! Replays `tests/vectors/pyload.json`, what the real PyYAML 6.0.1
//! `yaml.safe_load` and CPython 3.12 `json.loads` made of each text
//! (`tests/oracle/pyload_gen.py`, run in the robot image): the same value
//! (floats bit for bit, types exact), an error where Python raised, an error
//! where Python returned a value mower_pyfmt has no type for (the documented
//! gaps), CPython's error message for JSON, and `safe_dump` of the loaded
//! value where the generator kept one. The `refs` load every output of
//! `tests/vectors/pyfmt.json` back.

use mower_pyfmt::json;
use mower_pyfmt::yaml::{self, YamlOptions};
use mower_pyfmt::Py;
use serde_json::Value;

/// pyload.json's typed values: floats as their IEEE bits in hex.
fn decode(v: &Value) -> Py {
    let a = v.as_array().expect("typed value is an array");
    match a[0].as_str().unwrap() {
        "N" => Py::None,
        "B" => Py::Bool(a[1].as_bool().unwrap()),
        "I" => Py::Int(a[1].as_str().unwrap().parse().unwrap()),
        "F" => Py::Float(f64::from_bits(u64::from_str_radix(a[1].as_str().unwrap(), 16).unwrap())),
        "S" => Py::Str(a[1].as_str().unwrap().to_string()),
        "L" => Py::List(a[1].as_array().unwrap().iter().map(decode).collect()),
        "D" => Py::Dict(
            a[1].as_array().unwrap().iter().map(|kv| (kv[0].as_str().unwrap().to_string(), decode(&kv[1]))).collect(),
        ),
        "Da" => {
            let f = a[1].as_array().unwrap();
            Py::Date { year: f[0].as_i64().unwrap() as i32, month: f[1].as_u64().unwrap() as u32, day: f[2].as_u64().unwrap() as u32 }
        }
        "Dt" => {
            let f = a[1].as_array().unwrap();
            let u = |i: usize| f[i].as_u64().unwrap() as u32;
            Py::DateTime {
                year: f[0].as_i64().unwrap() as i32,
                month: u(1),
                day: u(2),
                hour: u(3),
                minute: u(4),
                second: u(5),
                microsecond: u(6),
                offset_minutes: f[7].as_i64().map(|o| o as i32),
            }
        }
        t => panic!("unknown tag {t}"),
    }
}

/// pyfmt.json's typed values: floats as `repr`.
fn decode_repr(v: &Value) -> Py {
    let a = v.as_array().unwrap();
    match a[0].as_str().unwrap() {
        "F" => Py::Float(match a[1].as_str().unwrap() {
            "nan" => f64::NAN,
            "inf" => f64::INFINITY,
            "-inf" => f64::NEG_INFINITY,
            s => s.parse().unwrap(),
        }),
        "L" => Py::List(a[1].as_array().unwrap().iter().map(decode_repr).collect()),
        "D" => Py::Dict(
            a[1].as_array().unwrap().iter().map(|kv| (kv[0].as_str().unwrap().to_string(), decode_repr(&kv[1]))).collect(),
        ),
        _ => decode(v),
    }
}

/// The same value: types exact, floats bit for bit, dict order kept.
fn same(a: &Py, b: &Py) -> bool {
    match (a, b) {
        (Py::Float(x), Py::Float(y)) => x.to_bits() == y.to_bits(),
        (Py::List(x), Py::List(y)) => x.len() == y.len() && x.iter().zip(y).all(|(p, q)| same(p, q)),
        (Py::Dict(x), Py::Dict(y)) => x.len() == y.len() && x.iter().zip(y).all(|(p, q)| p.0 == q.0 && same(&p.1, &q.1)),
        _ => a == b,
    }
}

/// The value with every dict's keys sorted (what a `sort_keys` dump loads
/// back as).
fn sorted_keys(v: &Py) -> Py {
    match v {
        Py::List(x) => Py::List(x.iter().map(sorted_keys).collect()),
        Py::Dict(x) => {
            let mut items: Vec<(String, Py)> = x.iter().map(|(k, v)| (k.clone(), sorted_keys(v))).collect();
            items.sort_by(|a, b| a.0.cmp(&b.0));
            Py::Dict(items)
        }
        other => other.clone(),
    }
}

/// One output of a pyfmt.json case loaded back with the matching loader.
fn load_out(case: &Value, combo: &str) -> Result<Py, String> {
    let text = case["out"][combo].as_str().unwrap();
    if combo.starts_with('y') {
        yaml::safe_load(text).map_err(|e| e.to_string())
    } else {
        json::loads(text)
    }
}

fn load(path: &str) -> Value {
    serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap()
}

#[test]
fn yaml_and_json_loads_as_python_does() {
    // PYLOAD_VECTORS / PYFMT_VECTORS=<file> replay another generator run (a
    // bigger random one: pyfmt_gen.py --random 2000, then pyload_gen.py --pyfmt)
    let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/vectors/");
    let custom = std::env::var("PYLOAD_VECTORS").is_ok();
    let doc = load(&std::env::var("PYLOAD_VECTORS").unwrap_or_else(|_| format!("{dir}pyload.json")));
    let fmt = load(&std::env::var("PYFMT_VECTORS").unwrap_or_else(|_| format!("{dir}pyfmt.json")));
    let mut failures = Vec::new();
    let mut n = [0usize; 6];
    let unicode = YamlOptions { allow_unicode: true, sort_keys: false };
    for c in doc["yaml"].as_array().unwrap() {
        let (id, text) = (c["id"].as_str().unwrap(), c["text"].as_str().unwrap());
        let got = yaml::safe_load(text);
        if let Some(v) = c.get("v") {
            n[0] += 1;
            let want = decode(v);
            match &got {
                Ok(g) if same(g, &want) => {}
                other => failures.push(format!("yaml {id} {text:?}\n  want {want:?}\n  got  {other:?}")),
            }
            if let (Some(d), Ok(g)) = (c.get("dump"), &got) {
                n[1] += 1;
                let mine = yaml::safe_dump(g, &unicode);
                if mine != d.as_str().unwrap() {
                    failures.push(format!("dump {id} {text:?}\n  want {:?}\n  got  {mine:?}", d.as_str().unwrap()));
                }
            }
        } else {
            n[2] += 1;
            if let Ok(g) = &got {
                let why = c.get("err").or(c.get("gap")).unwrap();
                failures.push(format!("yaml {id} {text:?}: Python {why}, we loaded {g:?}"));
            }
        }
    }
    for c in doc["json"].as_array().unwrap() {
        let (id, text) = (c["id"].as_str().unwrap(), c["text"].as_str().unwrap());
        let got = json::loads(text);
        n[3] += 1;
        if let Some(v) = c.get("v") {
            let want = decode(v);
            match &got {
                Ok(g) if same(g, &want) => {}
                other => failures.push(format!("json {id} {text:?}\n  want {want:?}\n  got  {other:?}")),
            }
        } else if let Some(e) = c.get("err") {
            if got.as_ref().err().map(|s| s.as_str()) != e.as_str() {
                failures.push(format!("json {id} {text:?}\n  want error {e}\n  got  {got:?}"));
            }
        } else if got.is_ok() {
            failures.push(format!("json {id} {text:?}: Python {}, we loaded {got:?}", c["gap"]));
        }
    }
    let cases: std::collections::HashMap<&str, &Value> =
        fmt["cases"].as_array().unwrap().iter().map(|c| (c["id"].as_str().unwrap(), c)).collect();
    for r in doc["refs"].as_array().unwrap() {
        let cid = r["case"].as_str().unwrap();
        let case = cases[cid];
        let orig = decode_repr(&case["v"]);
        let combos = |k: &str| -> Vec<String> {
            r.get(k).map_or(Vec::new(), |v| match v {
                Value::Array(a) => a.iter().map(|c| c.as_str().unwrap().to_string()).collect(),
                Value::Object(o) => o.keys().cloned().collect(),
                _ => panic!(),
            })
        };
        for (kind, want) in [("same", Some(orig.clone())), ("sorted", Some(sorted_keys(&orig)))] {
            for combo in combos(kind) {
                n[4] += 1;
                match load_out(case, &combo) {
                    Ok(g) if same(&g, want.as_ref().unwrap()) => n[5] += 1,
                    g => failures.push(format!("ref {cid} [{combo}] {kind}\n  want {want:?}\n  got  {g:?}")),
                }
            }
        }
        for combo in combos("other") {
            n[4] += 1;
            let o = &r["other"][&combo];
            match (o.get("v").map(decode), load_out(case, &combo)) {
                (Some(w), Ok(g)) if same(&g, &w) => n[5] += 1,
                (None, Err(_)) => n[5] += 1,
                (w, g) => failures.push(format!("ref {cid} [{combo}]\n  want {w:?} {o}\n  got  {g:?}")),
            }
        }
    }
    assert!(
        failures.is_empty(),
        "{} failures; first ones:\n{}",
        failures.len(),
        failures.iter().take(25).cloned().collect::<Vec<_>>().join("\n")
    );
    assert!(custom || (n[0] > 1000 && n[3] > 90 && n[4] > 4000), "{n:?}");
    eprintln!(
        "yaml: {} values ({} dumps), {} errors and gaps; json: {} cases; refs: {} of {} as Python",
        n[0], n[1], n[2], n[3], n[5], n[4]
    );
}
