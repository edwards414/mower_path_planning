//! CPython 3.12's `json.dumps` (Lib/json/encoder.py `JSONEncoder`,
//! `_make_iterencode`, `py_encode_basestring(_ascii)`; the C accelerator in
//! Modules/_json.c writes the same text).

use crate::{repr, Py};

/// The `json.dumps` keywords the recorder uses. `allow_nan` is always on (the
/// default) and the separators are always the defaults: `", "` and `": "`
/// without `indent`, `","` and `": "` with it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct JsonOptions {
    pub ensure_ascii: bool,
    /// `indent=N`: a newline and N spaces per level (0 still breaks lines).
    pub indent: Option<usize>,
    pub sort_keys: bool,
}

impl Default for JsonOptions {
    /// `json.dumps(obj)`: `ensure_ascii=True`, no indent, insertion order.
    fn default() -> Self {
        JsonOptions { ensure_ascii: true, indent: None, sort_keys: false }
    }
}

impl JsonOptions {
    /// `json.dumps(obj, ensure_ascii=False)`
    pub fn unicode() -> Self {
        JsonOptions { ensure_ascii: false, ..Default::default() }
    }
}

/// `json.dumps(value, ...)` with the given options.
pub fn dumps(value: &Py, opts: &JsonOptions) -> String {
    let mut out = String::new();
    encode(value, opts, 0, &mut out);
    out
}

/// `encoder.py` `_make_iterencode`: `_iterencode_list`, `_iterencode_dict`
/// and the scalar cases.
fn encode(value: &Py, opts: &JsonOptions, level: usize, out: &mut String) {
    match value {
        Py::None => out.push_str("null"),
        Py::Bool(true) => out.push_str("true"),
        Py::Bool(false) => out.push_str("false"),
        Py::Int(i) => out.push_str(&i.to_string()),
        Py::Float(f) => out.push_str(&float(*f)),
        Py::Str(s) => encode_str(s, opts.ensure_ascii, out),
        Py::List(items) => {
            if items.is_empty() {
                out.push_str("[]");
                return;
            }
            out.push('[');
            let inner = level + 1;
            for (i, item) in items.iter().enumerate() {
                if i > 0 {
                    out.push(',');
                    if opts.indent.is_none() {
                        out.push(' ');
                    }
                }
                newline_indent(opts, inner, out);
                encode(item, opts, inner, out);
            }
            newline_indent(opts, level, out);
            out.push(']');
        }
        Py::Dict(items) => {
            if items.is_empty() {
                out.push_str("{}");
                return;
            }
            let mut refs: Vec<&(String, Py)> = items.iter().collect();
            if opts.sort_keys {
                // sorted(dct.items()): keys are unique, so by key
                refs.sort_by(|a, b| a.0.cmp(&b.0));
            }
            out.push('{');
            let inner = level + 1;
            for (i, (k, v)) in refs.into_iter().enumerate() {
                if i > 0 {
                    out.push(',');
                    if opts.indent.is_none() {
                        out.push(' ');
                    }
                }
                newline_indent(opts, inner, out);
                encode_str(k, opts.ensure_ascii, out);
                out.push_str(": ");
                encode(v, opts, inner, out);
            }
            newline_indent(opts, level, out);
            out.push('}');
        }
    }
}

fn newline_indent(opts: &JsonOptions, level: usize, out: &mut String) {
    if let Some(n) = opts.indent {
        out.push('\n');
        for _ in 0..n * level {
            out.push(' ');
        }
    }
}

/// `floatstr` with `allow_nan=True`: `NaN`, `Infinity`, `-Infinity`, else
/// `float.__repr__`.
pub fn float(f: f64) -> String {
    if f.is_nan() {
        "NaN".to_string()
    } else if f == f64::INFINITY {
        "Infinity".to_string()
    } else if f == f64::NEG_INFINITY {
        "-Infinity".to_string()
    } else {
        repr::float(f)
    }
}

/// `py_encode_basestring_ascii` (`ensure_ascii=True`: everything outside
/// `' '..='~'` as `\uXXXX`, lower-case hex, astral characters as a surrogate
/// pair) or `py_encode_basestring` (only `"`, `\` and the C0 controls).
/// Both use the short escapes `\" \\ \n \r \t \b \f`.
pub fn encode_str(s: &str, ensure_ascii: bool, out: &mut String) {
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            '\u{08}' => out.push_str("\\b"),
            '\u{0c}' => out.push_str("\\f"),
            c if (c as u32) < 0x20 => push_u(out, c as u32),
            c if ensure_ascii && !(' '..='~').contains(&c) => {
                let n = c as u32;
                if n < 0x10000 {
                    push_u(out, n);
                } else {
                    let v = n - 0x10000;
                    push_u(out, 0xd800 | ((v >> 10) & 0x3ff));
                    push_u(out, 0xdc00 | (v & 0x3ff));
                }
            }
            c => out.push(c),
        }
    }
    out.push('"');
}

fn push_u(out: &mut String, n: u32) {
    out.push_str(&format!("\\u{:04x}", n));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shapes() {
        let v = Py::dict([
            ("b", Py::List(vec![Py::Int(1), Py::Float(2.0), Py::None])),
            ("a", Py::str("é\u{1F600}\u{7f}\n")),
            ("c", Py::Dict(vec![])),
        ]);
        assert_eq!(
            dumps(&v, &JsonOptions::default()),
            "{\"b\": [1, 2.0, null], \"a\": \"\\u00e9\\ud83d\\ude00\\u007f\\n\", \"c\": {}}"
        );
        assert_eq!(
            dumps(&v, &JsonOptions::unicode()),
            "{\"b\": [1, 2.0, null], \"a\": \"é\u{1F600}\u{7f}\\n\", \"c\": {}}"
        );
        let o = JsonOptions { ensure_ascii: true, indent: Some(1), sort_keys: true };
        assert_eq!(
            dumps(&v, &o),
            "{\n \"a\": \"\\u00e9\\ud83d\\ude00\\u007f\\n\",\n \"b\": [\n  1,\n  2.0,\n  null\n ],\n \"c\": {}\n}"
        );
        assert_eq!(dumps(&Py::Float(f64::NAN), &JsonOptions::default()), "NaN");
    }
}
