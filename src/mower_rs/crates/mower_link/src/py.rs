//! The bits of Python's behaviour the host tools' output depends on, shared
//! with `mowerctl pair`. `tests/golden.rs` and mowerctl's tests hold them to
//! what the Python tools really did (`deploy/test/golden/host/`).

/// `str.isspace()` (CPython's table): Rust's `char::is_whitespace` plus the
/// four separators `\x1c`..`\x1f`.
pub fn is_space(c: char) -> bool {
    c.is_whitespace() || ('\u{1c}'..='\u{1f}').contains(&c)
}

/// `s.strip()`
pub fn strip(s: &str) -> &str {
    s.trim_matches(is_space)
}

/// `s.split()`
pub fn split(s: &str) -> impl Iterator<Item = &str> {
    s.split(is_space).filter(|p| !p.is_empty())
}

/// What `open(path)` in text mode makes of line ends: `\r\n` and `\r` are `\n`.
pub fn universal_newlines(s: &str) -> String {
    s.replace("\r\n", "\n").replace('\r', "\n")
}

/// `os.path.join(dir, name)` for a plain `name`.
pub fn join(dir: &str, name: &str) -> String {
    if dir.is_empty() || dir.ends_with('/') {
        format!("{dir}{name}")
    } else {
        format!("{dir}/{name}")
    }
}

/// `os.path.dirname(path)`
pub fn dirname(path: &str) -> String {
    let Some(i) = path.rfind('/') else {
        return String::new();
    };
    let head = &path[..=i];
    if head.chars().all(|c| c == '/') {
        head.to_string()
    } else {
        head.trim_end_matches('/').to_string()
    }
}

/// `os.path.basename(path)`
pub fn basename(path: &str) -> &str {
    path.rfind('/').map_or(path, |i| &path[i + 1..])
}

/// `float(s)` for what a kernel or a modem prints. Not its `1_000` form, nor
/// digits outside ASCII.
pub fn float(s: &str) -> Option<f64> {
    strip(s).parse().ok()
}

/// `round(x, 1)`: both correctly round the exact binary value, ties to even.
pub fn round1(x: f64) -> f64 {
    format!("{x:.1}").parse().unwrap_or(x)
}

/// `repr(s)` of a `str`, for what a path can hold: the quote it does not
/// contain, the usual escapes, `\xNN` for the C0 and DEL controls.
pub fn repr_str(s: &str) -> String {
    let quote = if s.contains('\'') && !s.contains('"') { '"' } else { '\'' };
    let mut out = String::with_capacity(s.len() + 2);
    out.push(quote);
    for c in s.chars() {
        match c {
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if c == quote => {
                out.push('\\');
                out.push(c);
            }
            c if (c as u32) < 0x20 || c == '\u{7f}' => out.push_str(&format!("\\x{:02x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push(quote);
    out
}

/// The text of `strerror(errno)`, without the " (os error N)" Rust adds.
pub fn strerror(errno: i32) -> String {
    let text = std::io::Error::from_raw_os_error(errno).to_string();
    match text.rfind(" (os error ") {
        Some(i) => text[..i].to_string(),
        None => text,
    }
}

/// `str(OSError)`: `[Errno 21] Is a directory: '/dev/lte_at'`.
pub fn os_error(errno: i32, filename: Option<&str>) -> String {
    match filename {
        Some(f) => format!("[Errno {errno}] {}: {}", strerror(errno), repr_str(f)),
        None => format!("[Errno {errno}] {}", strerror(errno)),
    }
}

/// `str()` of an `io::Error` from an operation on `filename`.
pub fn io_error(e: &std::io::Error, filename: &str) -> String {
    match e.raw_os_error() {
        Some(errno) => os_error(errno, Some(filename)),
        None => format!("{e}: {}", repr_str(filename)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn strip_and_split_know_pythons_whitespace() {
        assert_eq!(strip(" \t\u{1c}a b\u{3000}\r\n"), "a b");
        assert_eq!(split(" a\u{1f}b  c\n").collect::<Vec<_>>(), ["a", "b", "c"]);
    }

    #[test]
    fn paths_join_and_split_like_os_path() {
        assert_eq!(join("a", "b"), "a/b");
        assert_eq!(join("a/", "b"), "a/b");
        assert_eq!(join("", "b"), "b");
        assert_eq!(dirname("a/b"), "a");
        assert_eq!(dirname("a//b"), "a");
        assert_eq!(dirname("/b"), "/");
        assert_eq!(dirname("//b"), "//");
        assert_eq!(dirname("b"), "");
        assert_eq!(basename("a/b"), "b");
        assert_eq!(basename("b"), "b");
    }

    #[test]
    fn errors_read_like_str_of_an_oserror() {
        assert_eq!(os_error(21, Some("/dev/lte_at")), "[Errno 21] Is a directory: '/dev/lte_at'");
        assert_eq!(os_error(5, None), "[Errno 5] Input/output error");
        assert_eq!(repr_str("it's"), "\"it's\"");
        assert_eq!(repr_str("a'b\"c\n"), "'a\\'b\"c\\n'");
    }

    #[test]
    fn newlines_and_rounding() {
        assert_eq!(universal_newlines("a\r\nb\rc\n"), "a\nb\nc\n");
        assert_eq!(round1(-0.04).to_string(), "-0");
        assert_eq!(round1(2.25), 2.2);
        assert_eq!(round1(2.35), 2.4);
    }
}
