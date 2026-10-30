//! What `mower_flash.py` gets from Python itself and the Rust port must do the
//! same way: `json.load` of the manifest (CPython 3.12's C scanner, its
//! accepted extensions and its error messages), `json.dump(..., indent=2)` of
//! the report and `json.dumps` of `app-info`, `str()` / `repr()` of whatever a
//! manifest holds (the `bundled:` line prints three of its values), and the
//! comparisons and conversions `firmware_matches` applies to them.
//!
//! `tests/pyjson_vectors.rs` holds all of it to vectors the real Python wrote
//! (`tests/oracle/pyfmt_gen.py`). Known gap: a lone UTF-16 surrogate in a
//! manifest string (`"\ud800"`), which a Python `str` can hold and a Rust
//! `String` cannot, becomes U+FFFD.

/// A value `json.load` returns.
#[derive(Debug, Clone, PartialEq)]
pub enum Py {
    None,
    Bool(bool),
    /// An integer literal, exact (Python ints have no size limit): its digits,
    /// `-0` written `0`.
    Int(String),
    Float(f64),
    Str(String),
    List(Vec<Py>),
    /// Insertion order; a repeated key keeps its first place and its last
    /// value, as a `dict` does.
    Dict(Vec<(String, Py)>),
}

impl Py {
    pub fn int(v: i64) -> Py {
        Py::Int(v.to_string())
    }
    pub fn str(s: &str) -> Py {
        Py::Str(s.to_string())
    }
    /// `type(v).__name__`
    pub fn type_name(&self) -> &'static str {
        match self {
            Py::None => "NoneType",
            Py::Bool(_) => "bool",
            Py::Int(_) => "int",
            Py::Float(_) => "float",
            Py::Str(_) => "str",
            Py::List(_) => "list",
            Py::Dict(_) => "dict",
        }
    }
    /// `d.get(key)` on a dict.
    pub fn get(&self, key: &str) -> Option<&Py> {
        match self {
            Py::Dict(items) => items.iter().find(|(k, _)| k == key).map(|(_, v)| v),
            _ => None,
        }
    }
}

// ---- json.loads ------------------------------------------------------------

/// `json.loads(text)`; the error is the line a traceback ends with,
/// `json.decoder.JSONDecodeError: Expecting value: line 1 column 1 (char 0)`.
pub fn loads(text: &str) -> Result<Py, String> {
    let s: Vec<char> = text.chars().collect();
    if s.first() == Some(&'\u{feff}') {
        return Err(decode_error(&s, "Unexpected UTF-8 BOM (decode using utf-8-sig)", 0));
    }
    let mut p = Scanner { s: &s };
    let idx = p.skip_ws(0);
    let (value, end) = match p.scan_once(idx) {
        Ok(v) => v,
        Err(Fail::Stop(at)) => return Err(decode_error(&s, "Expecting value", at)),
        Err(Fail::Error(msg, at)) => return Err(decode_error(&s, msg, at)),
    };
    let end = p.skip_ws(end);
    if end != s.len() {
        return Err(decode_error(&s, "Extra data", end));
    }
    Ok(value)
}

fn decode_error(s: &[char], msg: &str, pos: usize) -> String {
    let lineno = s[..pos].iter().filter(|&&c| c == '\n').count() + 1;
    let colno = match s[..pos].iter().rposition(|&c| c == '\n') {
        Some(nl) => pos - nl,
        None => pos + 1,
    };
    format!("json.decoder.JSONDecodeError: {msg}: line {lineno} column {colno} (char {pos})")
}

enum Fail {
    /// StopIteration(idx): no value starts at idx ("Expecting value")
    Stop(usize),
    Error(&'static str, usize),
}

struct Scanner<'a> {
    s: &'a [char],
}

fn is_ws(c: char) -> bool {
    matches!(c, ' ' | '\t' | '\n' | '\r')
}

impl Scanner<'_> {
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

    /// `scan_once_unicode` of `Modules/_json.c`.
    fn scan_once(&mut self, idx: usize) -> Result<(Py, usize), Fail> {
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

    /// `_match_number_unicode`.
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
            let canonical = if text == "-0" { "0".to_string() } else { text };
            Ok((Py::Int(canonical), idx))
        }
    }

    /// `scanstring_unicode` with strict=True; `end` is just past the quote.
    fn scanstring(&self, mut end: usize) -> Result<(String, usize), Fail> {
        let s = self.s;
        let len = s.len();
        let begin = end - 1;
        let mut out = String::new();
        loop {
            let mut next = end;
            let mut c = '\0';
            while next < len {
                c = s[next];
                if c == '"' || c == '\\' {
                    break;
                }
                if (c as u32) <= 0x1f {
                    return Err(Fail::Error("Invalid control character at", next));
                }
                next += 1;
            }
            if next >= len || !(c == '"' || c == '\\') {
                return Err(Fail::Error("Unterminated string starting at", begin));
            }
            out.extend(&s[end..next]);
            next += 1;
            if c == '"' {
                end = next;
                break;
            }
            if next == len {
                return Err(Fail::Error("Unterminated string starting at", begin));
            }
            let esc = s[next];
            let ch: u32;
            if esc != 'u' {
                end = next + 1;
                ch = match esc {
                    '"' => '"' as u32,
                    '\\' => '\\' as u32,
                    '/' => '/' as u32,
                    'b' => 8,
                    'f' => 12,
                    'n' => 10,
                    'r' => 13,
                    't' => 9,
                    _ => return Err(Fail::Error("Invalid \\escape", end - 2)),
                };
            } else {
                next += 1;
                end = next + 4;
                if end >= len {
                    return Err(Fail::Error("Invalid \\uXXXX escape", next - 1));
                }
                let mut c1 = 0u32;
                for &d in &s[next..end] {
                    c1 = (c1 << 4) | d.to_digit(16).filter(|_| d.is_ascii()).ok_or(Fail::Error("Invalid \\uXXXX escape", end - 5))?;
                }
                next = end;
                if (0xD800..=0xDBFF).contains(&c1) && end + 6 < len && s[next] == '\\' && s[next + 1] == 'u' {
                    let mut c2 = 0u32;
                    end += 6;
                    for &d in &s[next + 2..end] {
                        c2 = (c2 << 4) | d.to_digit(16).filter(|_| d.is_ascii()).ok_or(Fail::Error("Invalid \\uXXXX escape", end - 5))?;
                    }
                    if (0xDC00..=0xDFFF).contains(&c2) {
                        c1 = 0x10000 + (((c1 - 0xD800) << 10) | (c2 - 0xDC00));
                    } else {
                        end -= 6;
                    }
                }
                ch = c1;
            }
            out.push(char::from_u32(ch).unwrap_or('\u{fffd}'));
        }
        Ok((out, end))
    }

    /// `_parse_object_unicode`; `idx` is just past the brace.
    fn object(&mut self, mut idx: usize) -> Result<(Py, usize), Fail> {
        let mut items: Vec<(String, Py)> = Vec::new();
        idx = self.skip_ws(idx);
        if self.at(idx) != Some('}') {
            loop {
                if self.at(idx) != Some('"') {
                    return Err(Fail::Error("Expecting property name enclosed in double quotes", idx));
                }
                let (key, next) = self.scanstring(idx + 1)?;
                idx = self.skip_ws(next);
                if self.at(idx) != Some(':') {
                    return Err(Fail::Error("Expecting ':' delimiter", idx));
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
                    return Err(Fail::Error("Expecting ',' delimiter", idx));
                }
                idx = self.skip_ws(idx + 1);
            }
        }
        Ok((Py::Dict(items), idx + 1))
    }

    /// `_parse_array_unicode`; `idx` is just past the bracket.
    fn array(&mut self, mut idx: usize) -> Result<(Py, usize), Fail> {
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
                    return Err(Fail::Error("Expecting ',' delimiter", idx));
                }
                idx = self.skip_ws(idx + 1);
            }
        }
        Ok((Py::List(items), idx + 1))
    }
}

// ---- json.dumps ------------------------------------------------------------

/// `json.dumps(v)` (`indent=None`) or `json.dump(v, f, indent=n)`: ASCII only,
/// `NaN` / `Infinity` allowed, keys in insertion order.
pub fn dumps(v: &Py, indent: Option<usize>) -> String {
    let mut out = String::new();
    encode(v, indent, 0, &mut out);
    out
}

fn encode(v: &Py, indent: Option<usize>, level: usize, out: &mut String) {
    match v {
        Py::None => out.push_str("null"),
        Py::Bool(b) => out.push_str(if *b { "true" } else { "false" }),
        Py::Int(s) => out.push_str(s),
        Py::Float(f) => out.push_str(&if f.is_nan() {
            "NaN".to_string()
        } else if f.is_infinite() {
            (if *f > 0.0 { "Infinity" } else { "-Infinity" }).to_string()
        } else {
            float_repr(*f)
        }),
        Py::Str(s) => encode_str(s, out),
        Py::List(items) if items.is_empty() => out.push_str("[]"),
        Py::Dict(items) if items.is_empty() => out.push_str("{}"),
        Py::List(items) => {
            out.push('[');
            for (i, item) in items.iter().enumerate() {
                separator(indent, level + 1, i == 0, out);
                encode(item, indent, level + 1, out);
            }
            close(indent, level, out);
            out.push(']');
        }
        Py::Dict(items) => {
            out.push('{');
            for (i, (k, item)) in items.iter().enumerate() {
                separator(indent, level + 1, i == 0, out);
                encode_str(k, out);
                out.push_str(": ");
                encode(item, indent, level + 1, out);
            }
            close(indent, level, out);
            out.push('}');
        }
    }
}

fn separator(indent: Option<usize>, level: usize, first: bool, out: &mut String) {
    match indent {
        Some(n) => {
            if !first {
                out.push(',');
            }
            out.push('\n');
            out.push_str(&" ".repeat(n * level));
        }
        None => {
            if !first {
                out.push_str(", ");
            }
        }
    }
}

fn close(indent: Option<usize>, level: usize, out: &mut String) {
    if let Some(n) = indent {
        out.push('\n');
        out.push_str(&" ".repeat(n * level));
    }
}

/// `encode_basestring_ascii`.
fn encode_str(s: &str, out: &mut String) {
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            '\u{8}' => out.push_str("\\b"),
            '\u{c}' => out.push_str("\\f"),
            ' '..='~' => out.push(c),
            _ => {
                let mut units = [0u16; 2];
                for u in c.encode_utf16(&mut units) {
                    out.push_str(&format!("\\u{u:04x}"));
                }
            }
        }
    }
    out.push('"');
}

// ---- str() / repr() ----------------------------------------------------------

/// `repr(float)`: the shortest digits that read back the same, fixed notation
/// for 1e-4 <= |x| < 1e16, otherwise `1e+16` / `1.5e-05`.
pub fn float_repr(x: f64) -> String {
    if x.is_nan() {
        return "nan".to_string();
    }
    if x.is_infinite() {
        return (if x > 0.0 { "inf" } else { "-inf" }).to_string();
    }
    if x == 0.0 {
        return (if x.is_sign_negative() { "-0.0" } else { "0.0" }).to_string();
    }
    let sign = if x < 0.0 { "-" } else { "" };
    let e = format!("{:e}", x.abs()); // shortest round-trip digits
    let (mant, exp) = e.split_once('e').unwrap_or((&e, "0"));
    let digits: String = mant.chars().filter(|c| *c != '.').collect();
    let decpt = exp.parse::<i32>().unwrap_or(0) + 1;
    if decpt <= -4 || decpt > 16 {
        let mut m = digits[..1].to_string();
        if digits.len() > 1 {
            m.push('.');
            m.push_str(&digits[1..]);
        }
        let e = decpt - 1;
        format!("{sign}{m}e{}{:02}", if e < 0 { '-' } else { '+' }, e.abs())
    } else if decpt <= 0 {
        format!("{sign}0.{}{digits}", "0".repeat((-decpt) as usize))
    } else if decpt as usize >= digits.len() {
        format!("{sign}{digits}{}.0", "0".repeat(decpt as usize - digits.len()))
    } else {
        let (a, b) = digits.split_at(decpt as usize);
        format!("{sign}{a}.{b}")
    }
}

/// `str(v)`.
pub fn py_str(v: &Py) -> String {
    match v {
        Py::Str(s) => s.clone(),
        other => py_repr(other),
    }
}

/// `repr(v)`.
pub fn py_repr(v: &Py) -> String {
    match v {
        Py::None => "None".to_string(),
        Py::Bool(b) => (if *b { "True" } else { "False" }).to_string(),
        Py::Int(s) => s.clone(),
        Py::Float(f) => float_repr(*f),
        Py::Str(s) => str_repr(s),
        Py::List(items) => format!("[{}]", items.iter().map(py_repr).collect::<Vec<_>>().join(", ")),
        Py::Dict(items) => format!(
            "{{{}}}",
            items.iter().map(|(k, v)| format!("{}: {}", str_repr(k), py_repr(v))).collect::<Vec<_>>().join(", ")
        ),
    }
}

/// `Py_UNICODE_ISPRINTABLE` for the code points a manifest could plausibly
/// hold: controls, format characters, separators other than the space,
/// private use. (Unassigned code points count as printable here.)
fn printable(c: char) -> bool {
    let u = c as u32;
    !(u < 0x20
        || (0x7F..=0xA0).contains(&u)
        || u == 0xAD
        || u == 0x061C
        || u == 0x1680
        || u == 0x180E
        || (0x2000..=0x200F).contains(&u)
        || (0x2028..=0x202F).contains(&u)
        || (0x205F..=0x206F).contains(&u)
        || u == 0x3000
        || (0xE000..=0xF8FF).contains(&u)
        || u == 0xFEFF
        || (0xFFF9..=0xFFFB).contains(&u)
        || (0xFDD0..=0xFDEF).contains(&u)
        || u & 0xFFFE == 0xFFFE // the noncharacters U+xFFFE / U+xFFFF of every plane
        || (0xF0000..=0x10FFFF).contains(&u))
}

/// `repr(str)`: single quotes unless the text has one and no double quote.
pub fn str_repr(s: &str) -> String {
    let quote = if s.contains('\'') && !s.contains('"') { '"' } else { '\'' };
    let mut out = String::new();
    out.push(quote);
    for c in s.chars() {
        match c {
            _ if c == quote || c == '\\' => {
                out.push('\\');
                out.push(c);
            }
            '\t' => out.push_str("\\t"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            _ if (c as u32) < 0x20 || c as u32 == 0x7F => out.push_str(&format!("\\x{:02x}", c as u32)),
            _ if (c as u32) < 0x7F => out.push(c),
            _ if printable(c) => out.push(c),
            _ if (c as u32) <= 0xFF => out.push_str(&format!("\\x{:02x}", c as u32)),
            _ if (c as u32) <= 0xFFFF => out.push_str(&format!("\\u{:04x}", c as u32)),
            _ => out.push_str(&format!("\\U{:08x}", c as u32)),
        }
    }
    out.push(quote);
    out
}

// ---- what firmware_matches does with manifest values -------------------------

/// `n == v` for a Python int `n`: ints exactly, floats and bools by value.
pub fn eq_int(n: i64, v: &Py) -> bool {
    match v {
        Py::Int(s) => s.parse::<i128>().is_ok_and(|x| x == n as i128),
        Py::Float(f) => f.is_finite() && f.fract() == 0.0 && f.abs() < 1e30 && *f as i128 == n as i128,
        Py::Bool(b) => n == *b as i64,
        _ => false,
    }
}

/// `bool(v)`.
pub fn truthy(v: &Py) -> bool {
    match v {
        Py::None => false,
        Py::Bool(b) => *b,
        Py::Int(s) => s != "0",
        Py::Float(f) => *f != 0.0,
        Py::Str(s) => !s.is_empty(),
        Py::List(items) => !items.is_empty(),
        Py::Dict(items) => !items.is_empty(),
    }
}

/// `list(v)`; the error is the traceback's last line.
pub fn to_list(v: &Py) -> Result<Vec<Py>, String> {
    match v {
        Py::List(items) => Ok(items.clone()),
        Py::Str(s) => Ok(s.chars().map(|c| Py::Str(c.to_string())).collect()),
        Py::Dict(items) => Ok(items.iter().map(|(k, _)| Py::Str(k.clone())).collect()),
        other => Err(format!("TypeError: '{}' object is not iterable", other.type_name())),
    }
}

/// `v[:8]` (Python 3.12: a slice is hashable, so a dict raises KeyError).
pub fn head8(v: &Py) -> Result<Py, String> {
    match v {
        Py::Str(s) => Ok(Py::Str(s.chars().take(8).collect())),
        Py::List(items) => Ok(Py::List(items.iter().take(8).cloned().collect())),
        Py::Dict(_) => Err("KeyError: slice(None, 8, None)".to_string()),
        other => Err(format!("TypeError: '{}' object is not subscriptable", other.type_name())),
    }
}

/// `d[key]` on what `json.load` returned.
pub fn getitem<'a>(d: &'a Py, key: &str) -> Result<&'a Py, String> {
    match d {
        Py::Dict(_) => d.get(key).ok_or_else(|| format!("KeyError: {}", str_repr(key))),
        Py::List(_) => Err("TypeError: list indices must be integers or slices, not str".to_string()),
        Py::Str(_) => Err("TypeError: string indices must be integers, not 'str'".to_string()),
        other => Err(format!("TypeError: '{}' object is not subscriptable", other.type_name())),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn report_shape() {
        let v = Py::Dict(vec![
            ("time".into(), Py::int(0)),
            ("semver".into(), Py::List(vec![Py::int(0), Py::int(6), Py::int(0)])),
            ("e".into(), Py::List(vec![])),
            ("error".into(), Py::None),
        ]);
        assert_eq!(
            dumps(&v, Some(2)),
            "{\n  \"time\": 0,\n  \"semver\": [\n    0,\n    6,\n    0\n  ],\n  \"e\": [],\n  \"error\": null\n}"
        );
        assert_eq!(dumps(&v, None), "{\"time\": 0, \"semver\": [0, 6, 0], \"e\": [], \"error\": null}");
    }

    #[test]
    fn floats_like_python() {
        for (x, want) in [(1e16, "1e+16"), (1e15, "1000000000000000.0"), (0.0001, "0.0001"), (1e-5, "1e-05"), (1.5, "1.5"), (123.0, "123.0"), (-0.0, "-0.0"), (0.1, "0.1")] {
            assert_eq!(float_repr(x), want);
        }
    }

    #[test]
    fn errors_like_python() {
        assert_eq!(loads("not json\n").unwrap_err(), "json.decoder.JSONDecodeError: Expecting value: line 1 column 1 (char 0)");
        assert_eq!(loads("[1,]").unwrap_err(), "json.decoder.JSONDecodeError: Expecting value: line 1 column 4 (char 3)");
        assert_eq!(
            loads("{\"a\":1,}").unwrap_err(),
            "json.decoder.JSONDecodeError: Expecting property name enclosed in double quotes: line 1 column 8 (char 7)"
        );
    }
}
