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

/// What `json.dumps` raises for a value JSON has no form for: a date or a
/// datetime (`TypeError: Object of type date is not JSON serializable`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct JsonError(pub String);

impl std::fmt::Display for JsonError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for JsonError {}

/// `json.dumps(value, ...)` with the given options.
pub fn dumps(value: &Py, opts: &JsonOptions) -> Result<String, JsonError> {
    let mut out = String::new();
    encode(value, opts, 0, &mut out)?;
    Ok(out)
}

/// `encoder.py` `_make_iterencode`: `_iterencode_list`, `_iterencode_dict`
/// and the scalar cases.
fn encode(value: &Py, opts: &JsonOptions, level: usize, out: &mut String) -> Result<(), JsonError> {
    match value {
        Py::Date { .. } | Py::DateTime { .. } => {
            return Err(JsonError(format!("Object of type {} is not JSON serializable", value.type_name())));
        }
        Py::None => out.push_str("null"),
        Py::Bool(true) => out.push_str("true"),
        Py::Bool(false) => out.push_str("false"),
        Py::Int(i) => out.push_str(&i.to_string()),
        Py::Float(f) => out.push_str(&float(*f)),
        Py::Str(s) => encode_str(s, opts.ensure_ascii, out),
        Py::List(items) => {
            if items.is_empty() {
                out.push_str("[]");
                return Ok(());
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
                encode(item, opts, inner, out)?;
            }
            newline_indent(opts, level, out);
            out.push(']');
        }
        Py::Dict(items) => {
            if items.is_empty() {
                out.push_str("{}");
                return Ok(());
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
                encode(v, opts, inner, out)?;
            }
            newline_indent(opts, level, out);
            out.push('}');
        }
    }
    Ok(())
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

// ---- json.loads ------------------------------------------------------------

/// `json.loads(text)` (CPython 3.12, the C scanner `Modules/_json.c`):
/// dicts keep their first key's place and its last value, `NaN` and
/// `Infinity` are floats, ints are exact. The error is `str()` of the
/// `JSONDecodeError`: `Expecting value: line 1 column 1 (char 0)`.
///
/// Gaps: an int outside i64 is an error (Python's ints have no limit); a lone
/// surrogate escape (`"\ud800"`), which a Python `str` holds and a Rust one
/// cannot, becomes U+FFFD.
pub fn loads(text: &str) -> Result<Py, String> {
    let s: Vec<char> = text.chars().collect();
    if s.first() == Some(&'\u{feff}') {
        return Err(decode_error(&s, "Unexpected UTF-8 BOM (decode using utf-8-sig)", 0));
    }
    let p = Scan { s: &s };
    let idx = p.skip_ws(0);
    let (value, end) = match p.scan_once(idx) {
        Ok(v) => v,
        Err(Fail::Stop(at)) => return Err(decode_error(&s, "Expecting value", at)),
        Err(Fail::Error(msg, at)) => return Err(decode_error(&s, &msg, at)),
    };
    let end = p.skip_ws(end);
    if end != s.len() {
        return Err(decode_error(&s, "Extra data", end));
    }
    Ok(value)
}

/// `JSONDecodeError.__str__`: the message, the line and column of `pos`
/// (1-based) and `pos` itself.
fn decode_error(s: &[char], msg: &str, pos: usize) -> String {
    let lineno = s[..pos].iter().filter(|&&c| c == '\n').count() + 1;
    let colno = match s[..pos].iter().rposition(|&c| c == '\n') {
        Some(nl) => pos - nl,
        None => pos + 1,
    };
    format!("{msg}: line {lineno} column {colno} (char {pos})")
}

enum Fail {
    /// StopIteration(idx): no value starts at idx ("Expecting value")
    Stop(usize),
    Error(String, usize),
}

struct Scan<'a> {
    s: &'a [char],
}

fn is_ws(c: char) -> bool {
    matches!(c, ' ' | '\t' | '\n' | '\r')
}

impl Scan<'_> {
    fn at(&self, i: usize) -> Option<char> {
        self.s.get(i).copied()
    }

    fn skip_ws(&self, mut i: usize) -> usize {
        while i < self.s.len() && is_ws(self.s[i]) {
            i += 1;
        }
        i
    }

    fn word(&self, i: usize, w: &str) -> bool {
        w.chars().enumerate().all(|(k, c)| self.at(i + k) == Some(c))
    }

    /// `scan_once_unicode`
    fn scan_once(&self, idx: usize) -> Result<(Py, usize), Fail> {
        let Some(c) = self.at(idx) else {
            return Err(Fail::Stop(idx));
        };
        match c {
            '"' => return self.scanstring(idx + 1).map(|(s, e)| (Py::Str(s), e)),
            '{' => return self.object(idx + 1),
            '[' => return self.array(idx + 1),
            'n' if self.word(idx, "null") => return Ok((Py::None, idx + 4)),
            't' if self.word(idx, "true") => return Ok((Py::Bool(true), idx + 4)),
            'f' if self.word(idx, "false") => return Ok((Py::Bool(false), idx + 5)),
            'N' if self.word(idx, "NaN") => return Ok((Py::Float(f64::NAN), idx + 3)),
            'I' if self.word(idx, "Infinity") => return Ok((Py::Float(f64::INFINITY), idx + 8)),
            '-' if self.word(idx, "-Infinity") => return Ok((Py::Float(f64::NEG_INFINITY), idx + 9)),
            _ => {}
        }
        self.number(idx)
    }

    /// `_match_number_unicode`
    fn number(&self, start: usize) -> Result<(Py, usize), Fail> {
        let digit = |i: usize| matches!(self.at(i), Some('0'..='9'));
        let mut idx = start;
        if self.at(idx) == Some('-') {
            idx += 1;
            if idx >= self.s.len() {
                return Err(Fail::Stop(start));
            }
        }
        match self.at(idx) {
            Some('1'..='9') => {
                idx += 1;
                while digit(idx) {
                    idx += 1;
                }
            }
            Some('0') => idx += 1,
            _ => return Err(Fail::Stop(start)),
        }
        let mut is_float = false;
        if self.at(idx) == Some('.') && digit(idx + 1) {
            is_float = true;
            idx += 2;
            while digit(idx) {
                idx += 1;
            }
        }
        if idx + 1 < self.s.len() && matches!(self.at(idx), Some('e' | 'E')) {
            let e_start = idx;
            idx += 1;
            if idx + 1 < self.s.len() && matches!(self.at(idx), Some('-' | '+')) {
                idx += 1;
            }
            while digit(idx) {
                idx += 1;
            }
            if digit(idx - 1) {
                is_float = true;
            } else {
                idx = e_start;
            }
        }
        let text: String = self.s[start..idx].iter().collect();
        if is_float {
            Ok((Py::Float(text.parse().unwrap_or(f64::NAN)), idx))
        } else {
            match text.parse::<i64>() {
                Ok(v) => Ok((Py::Int(v), idx)),
                Err(_) => Err(Fail::Error("an int outside i64 (Python's ints have no limit)".into(), start)),
            }
        }
    }

    /// `scanstring_unicode` with strict=True; `end` is just past the quote.
    fn scanstring(&self, mut end: usize) -> Result<(String, usize), Fail> {
        let s = self.s;
        let len = s.len();
        let begin = end - 1;
        let mut out = String::new();
        let e = |m: &str, at| Fail::Error(m.to_string(), at);
        loop {
            let mut next = end;
            let mut c = '\0';
            while next < len {
                c = s[next];
                if c == '"' || c == '\\' {
                    break;
                }
                if (c as u32) <= 0x1f {
                    return Err(e("Invalid control character at", next));
                }
                next += 1;
            }
            if next >= len || !(c == '"' || c == '\\') {
                return Err(e("Unterminated string starting at", begin));
            }
            out.extend(&s[end..next]);
            next += 1;
            if c == '"' {
                end = next;
                break;
            }
            if next == len {
                return Err(e("Unterminated string starting at", begin));
            }
            let esc = s[next];
            let ch: u32 = if esc != 'u' {
                end = next + 1;
                match esc {
                    '"' => '"' as u32,
                    '\\' => '\\' as u32,
                    '/' => '/' as u32,
                    'b' => 8,
                    'f' => 12,
                    'n' => 10,
                    'r' => 13,
                    't' => 9,
                    _ => return Err(e("Invalid \\escape", end - 2)),
                }
            } else {
                next += 1;
                end = next + 4;
                if end >= len {
                    return Err(e("Invalid \\uXXXX escape", next - 1));
                }
                let hex = |from: usize, to: usize| -> Option<u32> {
                    let mut v = 0u32;
                    for &d in &s[from..to] {
                        v = (v << 4) | d.to_digit(16).filter(|_| d.is_ascii())?;
                    }
                    Some(v)
                };
                let mut c1 = hex(next, end).ok_or_else(|| e("Invalid \\uXXXX escape", end - 5))?;
                next = end;
                if (0xD800..=0xDBFF).contains(&c1) && end + 6 < len && s[next] == '\\' && s[next + 1] == 'u' {
                    end += 6;
                    let c2 = hex(next + 2, end).ok_or_else(|| e("Invalid \\uXXXX escape", end - 5))?;
                    if (0xDC00..=0xDFFF).contains(&c2) {
                        c1 = 0x10000 + (((c1 - 0xD800) << 10) | (c2 - 0xDC00));
                    } else {
                        end -= 6;
                    }
                }
                c1
            };
            out.push(char::from_u32(ch).unwrap_or('\u{fffd}'));
        }
        Ok((out, end))
    }

    /// `_parse_object_unicode`; `idx` is just past the brace.
    fn object(&self, mut idx: usize) -> Result<(Py, usize), Fail> {
        let mut items: Vec<(String, Py)> = Vec::new();
        let e = |m: &str, at| Fail::Error(m.to_string(), at);
        idx = self.skip_ws(idx);
        if self.at(idx) != Some('}') {
            loop {
                if self.at(idx) != Some('"') {
                    return Err(e("Expecting property name enclosed in double quotes", idx));
                }
                let (key, next) = self.scanstring(idx + 1)?;
                idx = self.skip_ws(next);
                if self.at(idx) != Some(':') {
                    return Err(e("Expecting ':' delimiter", idx));
                }
                idx = self.skip_ws(idx + 1);
                let (value, next) = self.scan_once(idx)?;
                match items.iter_mut().find(|(k, _)| *k == key) {
                    Some(slot) => slot.1 = value,
                    None => items.push((key, value)),
                }
                idx = self.skip_ws(next);
                if self.at(idx) == Some('}') {
                    break;
                }
                if self.at(idx) != Some(',') {
                    return Err(e("Expecting ',' delimiter", idx));
                }
                idx = self.skip_ws(idx + 1);
            }
        }
        Ok((Py::Dict(items), idx + 1))
    }

    /// `_parse_array_unicode`; `idx` is just past the bracket.
    fn array(&self, mut idx: usize) -> Result<(Py, usize), Fail> {
        let mut items = Vec::new();
        idx = self.skip_ws(idx);
        if self.at(idx) != Some(']') {
            loop {
                let (value, next) = self.scan_once(idx)?;
                items.push(value);
                idx = self.skip_ws(next);
                if self.at(idx) == Some(']') {
                    break;
                }
                if self.at(idx) != Some(',') {
                    return Err(Fail::Error("Expecting ',' delimiter".into(), idx));
                }
                idx = self.skip_ws(idx + 1);
            }
        }
        Ok((Py::List(items), idx + 1))
    }
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
            dumps(&v, &JsonOptions::default()).unwrap(),
            "{\"b\": [1, 2.0, null], \"a\": \"\\u00e9\\ud83d\\ude00\\u007f\\n\", \"c\": {}}"
        );
        assert_eq!(
            dumps(&v, &JsonOptions::unicode()).unwrap(),
            "{\"b\": [1, 2.0, null], \"a\": \"é\u{1F600}\u{7f}\\n\", \"c\": {}}"
        );
        let o = JsonOptions { ensure_ascii: true, indent: Some(1), sort_keys: true };
        assert_eq!(
            dumps(&v, &o).unwrap(),
            "{\n \"a\": \"\\u00e9\\ud83d\\ude00\\u007f\\n\",\n \"b\": [\n  1,\n  2.0,\n  null\n ],\n \"c\": {}\n}"
        );
        assert_eq!(dumps(&Py::Float(f64::NAN), &JsonOptions::default()).unwrap(), "NaN");
        let d = Py::List(vec![Py::Date { year: 2026, month: 10, day: 30 }]);
        assert_eq!(dumps(&d, &JsonOptions::default()).unwrap_err().0, "Object of type date is not JSON serializable");
    }
}
