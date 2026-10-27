//! Output: JSON lines, YAML, tables, and the stdout that ends quietly when
//! its reader goes away (`mowerctl topic echo X | head -3`).

use std::io::Write;

use serde_json::Value;

use crate::Fail;

/// Where a command's output goes: stdout, flushed line by line (a closed
/// pipe ends the command with exit 0), or a buffer for the tests.
pub struct Out {
    sink: Sink,
}

enum Sink {
    Stdout(std::io::Stdout),
    #[cfg(test)]
    Buffer(std::sync::Arc<std::sync::Mutex<String>>),
}

impl Out {
    pub fn stdout() -> Self {
        Out { sink: Sink::Stdout(std::io::stdout()) }
    }

    /// Output collected into the returned buffer.
    #[cfg(test)]
    pub fn buffer() -> (Self, std::sync::Arc<std::sync::Mutex<String>>) {
        let buf = std::sync::Arc::new(std::sync::Mutex::new(String::new()));
        (Out { sink: Sink::Buffer(buf.clone()) }, buf)
    }

    pub fn line(&mut self, text: &str) -> Result<(), Fail> {
        match &mut self.sink {
            Sink::Stdout(out) => {
                let mut lock = out.lock();
                match writeln!(lock, "{text}").and_then(|_| lock.flush()) {
                    Ok(()) => Ok(()),
                    Err(e) if e.kind() == std::io::ErrorKind::BrokenPipe => Err(Fail::stdout_closed()),
                    Err(e) => Err(Fail::failed(format!("stdout: {e}"))),
                }
            }
            #[cfg(test)]
            Sink::Buffer(buf) => {
                let mut b = buf.lock().unwrap();
                b.push_str(text);
                b.push('\n');
                Ok(())
            }
        }
    }
}

impl Fail {
    /// Not a failure: the reader of stdout has gone.
    pub fn stdout_closed() -> Fail {
        Fail { code: 0, msg: String::new() }
    }
}

/// `{"t": <arrival, unix s>, "topic": <name>, "msg": <message>}` in that key
/// order, `msg` being the message's JSON text as the frame carried it.
pub fn echo_line(t: f64, topic: &str, msg_text: &str) -> String {
    let t = (t * 1e6).round() / 1e6;
    format!("{{\"t\":{},\"topic\":{},\"msg\":{}}}", Value::from(t), Value::from(topic), msg_text)
}

/// The text of the top-level member `key` of the JSON object `text`, as it
/// stands there: a message printed from it is the bytes ws_bridge sent the
/// app, which parsing and printing again need not give back (a float read
/// one ulp off, an escape written another way). None if `text` is not an
/// object with that member.
pub fn raw_member<'a>(text: &'a str, key: &str) -> Option<&'a str> {
    let b = text.as_bytes();
    let mut i = skip_ws(b, 0);
    if b.get(i) != Some(&b'{') {
        return None;
    }
    i += 1;
    loop {
        i = skip_ws(b, i);
        let (k0, k1) = string_span(b, i)?;
        let name: String = serde_json::from_str(&text[k0..k1]).ok()?;
        i = skip_ws(b, k1);
        if b.get(i) != Some(&b':') {
            return None;
        }
        i = skip_ws(b, i + 1);
        let end = value_end(b, i)?;
        if name == key {
            return Some(&text[i..end]);
        }
        i = skip_ws(b, end);
        match b.get(i) {
            Some(b',') => i += 1,
            _ => return None,
        }
    }
}

fn skip_ws(b: &[u8], mut i: usize) -> usize {
    while i < b.len() && matches!(b[i], b' ' | b'\t' | b'\n' | b'\r') {
        i += 1;
    }
    i
}

/// [start, end) of the string at `i`, quotes included. Structural bytes are
/// ASCII and never part of a UTF-8 sequence, so the indices are char
/// boundaries.
fn string_span(b: &[u8], i: usize) -> Option<(usize, usize)> {
    if b.get(i) != Some(&b'"') {
        return None;
    }
    let mut j = i + 1;
    while j < b.len() {
        match b[j] {
            b'\\' => j += 2,
            b'"' => return Some((i, j + 1)),
            _ => j += 1,
        }
    }
    None
}

/// The end of the value at `i`.
fn value_end(b: &[u8], i: usize) -> Option<usize> {
    match *b.get(i)? {
        b'"' => string_span(b, i).map(|(_, e)| e),
        b'{' | b'[' => {
            let mut depth = 0usize;
            let mut j = i;
            while j < b.len() {
                match b[j] {
                    b'"' => {
                        j = string_span(b, j)?.1;
                        continue;
                    }
                    b'{' | b'[' => depth += 1,
                    b'}' | b']' => {
                        depth -= 1;
                        if depth == 0 {
                            return Some(j + 1);
                        }
                    }
                    _ => {}
                }
                j += 1;
            }
            None
        }
        _ => {
            let mut j = i;
            while j < b.len() && !matches!(b[j], b',' | b'}' | b']' | b' ' | b'\t' | b'\n' | b'\r') {
                j += 1;
            }
            (j > i).then_some(j)
        }
    }
}

/// `--no-arr`: every array becomes the string `"<N items>"`.
pub fn drop_arrays(v: &mut Value) {
    match v {
        Value::Array(items) => *v = Value::String(format!("<{} items>", items.len())),
        Value::Object(map) => map.values_mut().for_each(drop_arrays),
        _ => {}
    }
}

/// `--field a.b.0.c`: object keys, and array indices as numbers.
pub fn field<'a>(v: &'a Value, path: &str) -> Option<&'a Value> {
    path.split('.').filter(|p| !p.is_empty()).try_fold(v, |cur, part| match cur {
        Value::Object(map) => map.get(part),
        Value::Array(items) => part.parse::<usize>().ok().and_then(|i| items.get(i)),
        _ => None,
    })
}

/// A value as YAML, without serde_yaml's leading `---`.
pub fn yaml(v: &Value) -> String {
    let text = serde_yaml::to_string(v).unwrap_or_else(|e| format!("# not printable as YAML: {e}\n"));
    text.strip_prefix("---\n").unwrap_or(&text).trim_end().to_string()
}

/// Rows as aligned columns (two spaces between them; the last unpadded).
pub fn table(rows: &[Vec<String>]) -> Vec<String> {
    let cols = rows.iter().map(Vec::len).max().unwrap_or(0);
    let widths: Vec<usize> = (0..cols)
        .map(|c| rows.iter().filter_map(|r| r.get(c)).map(|s| s.chars().count()).max().unwrap_or(0))
        .collect();
    rows.iter()
        .map(|r| {
            let mut line = String::new();
            for (c, cell) in r.iter().enumerate() {
                if c + 1 < r.len() {
                    line.push_str(&format!("{cell:<w$}  ", w = widths[c]));
                } else {
                    line.push_str(cell);
                }
            }
            line
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn an_echo_line_carries_the_frames_message_bytes() {
        // a frame as ws_bridge builds it (serde_json::Value: keys sorted, an
        // f32 widened, NaN as null, an i8 status -1)
        let frame = r#"{"msg":{"altitude":null,"header":{"frame_id":"gps_link","stamp":{"nanosec":4294967,"sec":1761550000}},"latitude":25.033000012345678,"longitude":121.5654,"position_covariance":[0.000196,0.0,0.0],"status":{"service":15,"status":-1},"voltage":24.299999237060547},"op":"publish","topic":"/fix"}"#;
        let start = frame.find("\"msg\":").unwrap() + 6;
        let end = frame.find(",\"op\":").unwrap();
        let msg = raw_member(frame, "msg").unwrap();
        assert_eq!(msg, &frame[start..end]);
        assert_eq!(echo_line(1761550000.1234567, "/fix", msg), format!("{{\"t\":1761550000.123457,\"topic\":\"/fix\",\"msg\":{msg}}}"));
        let back: Value = serde_json::from_str(&echo_line(1.0, "/fix", msg)).unwrap();
        let doc: Value = serde_json::from_str(frame).unwrap();
        assert_eq!(back["msg"], doc["msg"]);
    }

    #[test]
    fn raw_members_are_the_text_as_it_stands() {
        // forms a parse and print would not give back, kept as they are
        let text = "{ \"op\" : \"publish\", \"msg\" : {\"a\": 1E5, \"s\": \"\\u00e9 } ] \\\" ,\", \"l\": [[1, {\"x\": []}], 2.50]} , \"topic\":\"/t\" }";
        assert_eq!(raw_member(text, "msg"), Some("{\"a\": 1E5, \"s\": \"\\u00e9 } ] \\\" ,\", \"l\": [[1, {\"x\": []}], 2.50]}"));
        assert_eq!(raw_member(text, "topic"), Some("\"/t\""));
        assert_eq!(raw_member(text, "op"), Some("\"publish\""));
        assert_eq!(raw_member("{\"n\":-1.5e-3,\"b\":true}", "n"), Some("-1.5e-3"));
        assert_eq!(raw_member("{\"n\":null}", "n"), Some("null"));
        // a key spelled with an escape is still the key
        assert_eq!(raw_member("{\"m\\u0073g\":7}", "msg"), Some("7"));
        for bad in ["", "[1]", "{\"msg\"}", "{\"msg\":", "{\"msg\":{\"a\":1", "{\"op\":1 \"msg\":2}", "{\"a\":1}"] {
            assert_eq!(raw_member(bad, "msg"), None, "{bad}");
        }
    }

    #[test]
    fn arrays_fields_and_yaml() {
        let mut v = json!({"data": [1, 2, 3], "pose": {"covariance": vec![0.0; 36], "position": {"x": 1.5}}, "name": "a"});
        assert_eq!(field(&v, "pose.position.x"), Some(&json!(1.5)));
        assert_eq!(field(&v, "data.1"), Some(&json!(2)));
        assert_eq!(field(&v, "data.9"), None);
        assert_eq!(field(&v, "pose.nope"), None);
        assert_eq!(field(&v, "name.x"), None);
        drop_arrays(&mut v);
        assert_eq!(v, json!({"data": "<3 items>", "pose": {"covariance": "<36 items>", "position": {"x": 1.5}}, "name": "a"}));
        assert_eq!(yaml(&json!({"b": 1, "a": {"c": "x"}})), "a:\n  c: x\nb: 1");
    }

    #[test]
    fn tables_align_every_column_but_the_last() {
        let rows = vec![vec!["/a".to_string(), "std_msgs/msg/Bool".to_string()], vec!["/longer".to_string(), "x".to_string()]];
        assert_eq!(table(&rows), vec!["/a       std_msgs/msg/Bool", "/longer  x"]);
        assert!(table(&[]).is_empty());
    }
}
