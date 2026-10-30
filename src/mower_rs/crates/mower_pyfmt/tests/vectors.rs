//! Replays `tests/vectors/pyfmt.json`, which the real Python 3.12 and PyYAML
//! 6.0.1 wrote (`tests/oracle/pyfmt_gen.py`, run in the robot image): every
//! case, every output combo, byte for byte. The `real/` cases are files and
//! messages the Python recorder wrote in the E5-0 fixture runs.

use mower_pyfmt::json::{self, JsonOptions};
use mower_pyfmt::yaml::{self, YamlOptions};
use mower_pyfmt::Py;
use serde_json::Value;

fn decode(v: &Value) -> Py {
    let a = v.as_array().expect("typed value is an array");
    let tag = a[0].as_str().expect("tag");
    match tag {
        "N" => Py::None,
        "B" => Py::Bool(a[1].as_bool().unwrap()),
        "I" => Py::Int(a[1].as_str().unwrap().parse().unwrap()),
        "F" => Py::Float(match a[1].as_str().unwrap() {
            "nan" => f64::NAN,
            "inf" => f64::INFINITY,
            "-inf" => f64::NEG_INFINITY,
            s => s.parse().unwrap(),
        }),
        "S" => Py::Str(a[1].as_str().unwrap().to_string()),
        "L" => Py::List(a[1].as_array().unwrap().iter().map(decode).collect()),
        "D" => Py::Dict(
            a[1].as_array()
                .unwrap()
                .iter()
                .map(|kv| (kv[0].as_str().unwrap().to_string(), decode(&kv[1])))
                .collect(),
        ),
        t => panic!("unknown tag {t}"),
    }
}

/// The combo names of the generator's `COMBOS`.
fn render(combo: &str, v: &Py) -> String {
    let j = |ensure_ascii, indent, sort_keys| json::dumps(v, &JsonOptions { ensure_ascii, indent, sort_keys });
    let y = |allow_unicode, sort_keys| yaml::safe_dump(v, &YamlOptions { allow_unicode, sort_keys });
    match combo {
        "j" => j(true, None, false),
        "ju" => j(false, None, false),
        "js" => j(true, None, true),
        "ji2u" => j(false, Some(2), false),
        "ji1s" => j(true, Some(1), true),
        "ji0" => j(true, Some(0), false),
        "y" => y(false, true),
        "yu" => y(true, false),
        "yus" => y(true, true),
        "ya" => y(false, false),
        c => panic!("unknown combo {c}"),
    }
}

#[test]
fn python_vectors() {
    // PYFMT_VECTORS=<file> replays another generator run instead (for example
    // a bigger random one: pyfmt_gen.py --random 3000 --seed 7 --all-combos)
    let path = std::env::var("PYFMT_VECTORS")
        .unwrap_or_else(|_| concat!(env!("CARGO_MANIFEST_DIR"), "/tests/vectors/pyfmt.json").to_string());
    let doc: Value = serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
    let custom = std::env::var("PYFMT_VECTORS").is_ok();
    let cases = doc["cases"].as_array().unwrap();
    let (mut outputs, mut failures) = (0usize, Vec::new());
    for case in cases {
        let id = case["id"].as_str().unwrap();
        let v = decode(&case["v"]);
        for (combo, want) in case["out"].as_object().unwrap() {
            let want = want.as_str().unwrap();
            let got = render(combo, &v);
            outputs += 1;
            if got != want {
                failures.push(format!("{id} [{combo}]\n  want {want:?}\n  got  {got:?}"));
            }
            if combo.starts_with('y') {
                let opts = YamlOptions { allow_unicode: combo == "yu" || combo == "yus", sort_keys: combo != "yu" && combo != "ya" };
                assert_eq!(yaml::dump(&v, &opts), got, "{id}: dump == safe_dump");
            }
        }
    }
    let shown: Vec<&String> = failures.iter().take(20).collect();
    assert!(
        failures.is_empty(),
        "{} of {outputs} outputs differ from Python ({} cases); first ones:\n{}",
        failures.len(),
        cases.len(),
        shown.iter().map(|s| s.as_str()).collect::<Vec<_>>().join("\n")
    );
    assert!(custom || (cases.len() > 600 && outputs > 4000), "{} cases, {outputs} outputs", cases.len());
    eprintln!("{} cases, {outputs} outputs equal to Python's", cases.len());
}
