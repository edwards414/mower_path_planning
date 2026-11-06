//! E3's JSON corpus (`mower_ws_bridge/tests/golden/`) with no r2r: the frames
//! are r2r's text of typed values, and `json_corpus_cdr.txt` is what rmw wrote
//! for the same values (`mower_ws_bridge/tests/golden_cdr.rs` writes it). So a
//! frame must be what `mower_cdr` reads from those bytes, written the way the
//! bridge writes JSON: floats to 17 significant digits at most, `f32` widened
//! to `f64`, NaN and the infinities as `null`, keys sorted.
//!
//! - Messages (`publish.txt`, `inbound.txt`): compared byte for byte, values
//!   included.
//! - Services (`request.txt`, `service_response.txt`): checked here for being
//!   the canonical form of their own JSON, and `SHA256SUMS` holds their values.
//!   `plain_corpus.rs` makes the typed comparison, on E9-22's types.
//!
//! Field names and JSON kinds are those of `mower_schema`'s `.msg` files: what
//! E9-22's generated types have to serialize to.

#[path = "support/json.rs"]
mod support_json;

use serde_json::{json, Value as Json};
use support_json::to_json;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

fn golden(name: &str) -> String {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../mower_ws_bridge/tests/golden");
    std::fs::read_to_string(dir.join(name)).unwrap_or_else(|e| panic!("{name}: {e}"))
}

fn facts(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../../deploy/test/golden/ros_facts").join(name)
}

fn cases(file: &str) -> BTreeMap<String, String> {
    file.lines().filter_map(|l| l.split_once('\t')).map(|(c, t)| (c.to_string(), t.to_string())).collect()
}

/// The corpus's cases with bytes: case -> (type, bytes rmw wrote).
fn vectors() -> BTreeMap<String, (String, Vec<u8>)> {
    let text = std::fs::read_to_string(facts("json_corpus_cdr.txt")).unwrap();
    text.lines()
        .map(|line| {
            let mut cols = line.split('\t');
            let (case, ty, hex) = (cols.next().unwrap(), cols.next().unwrap(), cols.next().unwrap());
            let bytes = (0..hex.len()).step_by(2).map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap()).collect();
            (case.to_string(), (ty.to_string(), bytes))
        })
        .collect()
}

fn message(case: &str, vectors: &BTreeMap<String, (String, Vec<u8>)>) -> Json {
    let (ty, bytes) = vectors.get(case).unwrap_or_else(|| panic!("{case}: not in json_corpus_cdr.txt"));
    to_json(&mower_cdr::decode(ty, bytes).unwrap_or_else(|e| panic!("{case} ({ty}): {e}")))
}

#[test]
fn a_frame_the_app_receives_is_what_mower_cdr_reads_from_rmw_s_bytes() {
    let vectors = vectors();
    let frames = cases(&golden("publish.txt"));
    assert!(frames.len() >= 14, "{}", frames.len());
    let mut wrong = Vec::new();
    for (case, frame) in &frames {
        let parsed: Json = serde_json::from_str(frame).unwrap_or_else(|e| panic!("{case}: {e}"));
        let topic = parsed["topic"].as_str().unwrap_or_else(|| panic!("{case}: no topic"));
        let ours = json!({"op": "publish", "topic": topic, "msg": message(case, &vectors)}).to_string();
        if &ours != frame {
            wrong.push(format!("{case}:\n  golden: {frame}\n  cdr:    {ours}"));
        }
    }
    assert!(wrong.is_empty(), "{}", wrong.join("\n"));
}

#[test]
fn a_message_the_app_sends_is_what_mower_cdr_reads_from_rmw_s_bytes() {
    let vectors = vectors();
    let inbound = cases(&golden("inbound.txt"));
    assert!(inbound.len() >= 4, "{}", inbound.len());
    for (case, text) in &inbound {
        assert_eq!(&message(case, &vectors).to_string(), text, "{case}");
    }
}

/// Every vector belongs to a message case: nothing stale.
#[test]
fn the_vectors_are_the_corpus_s_message_cases() {
    let mut messages: Vec<String> = cases(&golden("publish.txt")).into_keys().collect();
    messages.extend(cases(&golden("inbound.txt")).into_keys());
    messages.sort();
    let mut have: Vec<String> = vectors().into_keys().collect();
    have.sort();
    assert_eq!(have, messages);
}

/// `text` with every number outside a string replaced by `0`. serde_json's
/// parser is off by an ulp at times without its `float_roundtrip` feature
/// (which would then change what the whole workspace parses), so the numbers
/// of a reparsed text are not compared here: the message cases compare them
/// above, and SHA256SUMS holds the services'.
fn numbers_masked(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    let (mut in_string, mut escaped, mut in_number) = (false, false, false);
    for c in text.chars() {
        if in_string {
            out.push(c);
            match (escaped, c) {
                (true, _) => escaped = false,
                (false, '\\') => escaped = true,
                (false, '"') => in_string = false,
                _ => {}
            }
            continue;
        }
        if c.is_ascii_digit() || matches!(c, '-' | '+' | '.' | 'e' | 'E') && (in_number || c == '-') {
            if !in_number {
                out.push('0');
                in_number = true;
            }
            continue;
        }
        in_number = false;
        in_string = c == '"';
        out.push(c);
    }
    out
}

/// Keys sorted, no white space, r2r's text as serde_json writes it: the form a
/// reader of the corpus may rely on. `map_layer` has its own file (a captured
/// frame, 200 kB); it gets this check, and SHA256SUMS holds its bytes.
#[test]
fn every_text_is_the_canonical_form_of_its_json() {
    let canonical = |what: String, text: &str| -> Json {
        let parsed: Json = serde_json::from_str(text).unwrap_or_else(|e| panic!("{what}: {e}"));
        assert_eq!(numbers_masked(&parsed.to_string()), numbers_masked(text), "{what}");
        parsed
    };
    for file in ["publish.txt", "inbound.txt", "request.txt", "service_response.txt"] {
        let map = cases(&golden(file));
        assert!(!map.is_empty(), "{file}");
        for (case, text) in map {
            canonical(format!("{file} {case}"), &text);
        }
    }
    let frame = canonical("map_grid.frame".to_string(), &golden("map_grid.frame"));
    assert!(frame["msg"]["data"].is_string());
}
