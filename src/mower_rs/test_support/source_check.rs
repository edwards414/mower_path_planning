//! Source checks for the rules no behavioural test reaches (E9-07).
//!
//! Most of the rules src/mower_mission/test/*.py used to grep from the Rust
//! sources are now tests that run the code. What is left is about the shape
//! of the code itself -- something is absent (no parameter handler), happens
//! before something else, or sits where only a running rcl graph or the
//! robot's hardware reaches it -- and each crate checks it in its own
//! `tests/source_rules.rs`, next to the code, which says for every rule why
//! it is checked in the source. They include this file with
//! `#[path = "../../../test_support/source_check.rs"] mod source_check;`, so
//! it is no crate and has no dependencies.
//!
//! A check compares code as code: comments are left out and whitespace only
//! separates words, so that reformatting a line does not fail it; string
//! literals are compared as written.

#![allow(dead_code)]

/// One source file, or the body of one fn in it.
pub struct Source {
    name: String,
    text: String,
    code: String,
    literals: Vec<String>,
}

#[derive(Clone, Copy, PartialEq)]
enum Kind {
    Code,
    Literal,
    Comment,
}

fn is_ident(c: char) -> bool {
    c.is_alphanumeric() || c == '_'
}

/// The kind of every char of `text`: code, inside a string or char literal
/// (quotes included), or inside a comment.
fn kinds(text: &str) -> Vec<(usize, char, Kind)> {
    let chars: Vec<(usize, char)> = text.char_indices().collect();
    let mut out = Vec::with_capacity(chars.len());
    let mut i = 0;
    let push = |out: &mut Vec<(usize, char, Kind)>, from: usize, to: usize, kind: Kind| {
        for &(at, c) in &chars[from..to.min(chars.len())] {
            out.push((at, c, kind));
        }
    };
    while i < chars.len() {
        let c = chars[i].1;
        let next = chars.get(i + 1).map(|p| p.1);
        let prev_ident = i > 0 && is_ident(chars[i - 1].1);
        if c == '/' && next == Some('/') {
            let end = (i..chars.len()).find(|&j| chars[j].1 == '\n').unwrap_or(chars.len());
            push(&mut out, i, end, Kind::Comment);
            i = end;
        } else if c == '/' && next == Some('*') {
            let end = (i + 2..chars.len()).find(|&j| chars[j].1 == '*' && chars.get(j + 1).map(|p| p.1) == Some('/')).map_or(chars.len(), |j| j + 2);
            push(&mut out, i, end, Kind::Comment);
            i = end;
        } else if c == 'r' && !prev_ident && matches!(next, Some('"') | Some('#')) {
            // r"..." or r#"..."#
            let hashes = (i + 1..chars.len()).take_while(|&j| chars[j].1 == '#').count();
            let open = i + 1 + hashes;
            if chars.get(open).map(|p| p.1) != Some('"') {
                out.push((chars[i].0, c, Kind::Code));
                i += 1;
                continue;
            }
            let mut end = chars.len();
            let mut j = open + 1;
            while j < chars.len() {
                if chars[j].1 == '"' && (1..=hashes).all(|h| chars.get(j + h).map(|p| p.1) == Some('#')) {
                    end = j + hashes + 1;
                    break;
                }
                j += 1;
            }
            push(&mut out, i, end, Kind::Literal);
            i = end;
        } else if c == '"' {
            let mut j = i + 1;
            while j < chars.len() && chars[j].1 != '"' {
                j += if chars[j].1 == '\\' { 2 } else { 1 };
            }
            let end = (j + 1).min(chars.len());
            push(&mut out, i, end, Kind::Literal);
            i = end;
        } else if c == '\'' {
            // a char literal ('x', '\n', '\'', '"'), or a lifetime ('a)
            let end = if next == Some('\\') {
                (i + 2..chars.len()).find(|&j| chars[j].1 == '\'' && j > i + 2).map(|j| j + 1)
            } else if chars.get(i + 2).map(|p| p.1) == Some('\'') {
                Some(i + 3)
            } else {
                None
            };
            match end {
                Some(end) => {
                    push(&mut out, i, end, Kind::Literal);
                    i = end;
                }
                None => {
                    out.push((chars[i].0, c, Kind::Code));
                    i += 1;
                }
            }
        } else {
            out.push((chars[i].0, c, Kind::Code));
            i += 1;
        }
    }
    out
}

/// Code without comments, whitespace dropped except a single space between
/// two words, literals as written; and the literals, quotes included.
fn normalise(text: &str) -> (String, Vec<String>) {
    let mut code = String::new();
    let mut literals: Vec<String> = Vec::new();
    let mut in_literal = false;
    let mut pending_space = false;
    for (_, c, kind) in kinds(text) {
        if kind == Kind::Literal {
            if !in_literal {
                literals.push(String::new());
                if pending_space && code.ends_with(is_ident) {
                    code.push(' ');
                }
                pending_space = false;
                in_literal = true;
            }
            code.push(c);
            literals.last_mut().expect("a literal").push(c);
            continue;
        }
        in_literal = false;
        if kind == Kind::Comment || c.is_whitespace() {
            pending_space = true;
            continue;
        }
        if pending_space && code.ends_with(is_ident) && is_ident(c) {
            code.push(' ');
        }
        pending_space = false;
        code.push(c);
    }
    (code, literals)
}

impl Source {
    pub fn new(name: &str, text: &str) -> Source {
        let (code, literals) = normalise(text);
        Source { name: name.to_string(), text: text.to_string(), code, literals }
    }

    fn norm(needle: &str) -> String {
        normalise(needle).0
    }

    /// Where the code of `needle` first occurs.
    pub fn find(&self, needle: &str) -> Option<usize> {
        self.code.find(&Self::norm(needle))
    }

    /// How often the code of `needle` occurs.
    pub fn count(&self, needle: &str) -> usize {
        self.code.matches(&Self::norm(needle)).count()
    }

    /// The code has `needle`.
    pub fn has(&self, needle: &str) {
        assert!(self.find(needle).is_some(), "{}: no `{needle}` in its code", self.name);
    }

    /// The code has no `needle`.
    pub fn lacks(&self, needle: &str) {
        assert!(self.find(needle).is_none(), "{}: `{needle}` in its code", self.name);
    }

    /// Some string literal of the code has `text` (a message, a name).
    pub fn says(&self, text: &str) {
        assert!(self.literals.iter().any(|l| l.contains(text)), "{}: no string literal with {text:?}", self.name);
    }

    /// `first` occurs, `then` occurs, and the first `first` comes before the
    /// first `then`.
    pub fn before(&self, first: &str, then: &str) {
        let a = self.find(first).unwrap_or_else(|| panic!("{}: no `{first}` in its code", self.name));
        let b = self.find(then).unwrap_or_else(|| panic!("{}: no `{then}` in its code", self.name));
        assert!(a < b, "{}: `{first}` must come before `{then}`", self.name);
    }

    /// The needles occur in this order, each one after the one before it
    /// (searched from where the previous one ends).
    pub fn in_order(&self, needles: &[&str]) {
        let mut from = 0;
        for needle in needles {
            let n = Self::norm(needle);
            let at = self.code[from..].find(&n).unwrap_or_else(|| panic!("{}: no `{needle}` after {:?}", self.name, needles));
            from += at + n.len();
        }
    }

    /// The arguments of every call of `name` (a method or a fn), as code.
    pub fn calls(&self, name: &str) -> Vec<String> {
        let open = format!("{}(", Self::norm(name));
        let mut out = Vec::new();
        let mut from = 0;
        while let Some(at) = self.code[from..].find(&open) {
            let start = from + at + open.len();
            let mut depth = 1usize;
            let mut end = self.code.len();
            for (i, c) in self.code[start..].char_indices() {
                match c {
                    '(' => depth += 1,
                    ')' => {
                        depth -= 1;
                        if depth == 0 {
                            end = start + i;
                            break;
                        }
                    }
                    _ => {}
                }
            }
            out.push(self.code[start..end].to_string());
            from = start;
        }
        out
    }

    /// The first fn called `name`, from `fn` to its closing brace. A `name`
    /// with a `(` is the start of the signature, to pick one of several fns
    /// of that name: `run(ctx` is a module's entry point, `run(&self` a
    /// method.
    pub fn body(&self, name: &str) -> Source {
        let k = kinds(&self.text);
        let code_at = |i: usize| k[i].2 == Kind::Code;
        let start = (0..k.len())
            .find(|&i| {
                code_at(i)
                    && self.text[k[i].0..].starts_with("fn ")
                    && (i == 0 || !is_ident(k[i - 1].1))
                    && {
                        let rest = self.text[k[i].0 + 3..].trim_start();
                        rest.starts_with(name) && (name.contains('(') || rest[name.len()..].starts_with(['(', '<']))
                    }
            })
            .unwrap_or_else(|| panic!("{}: no fn {name}", self.name));
        let open = (start..k.len()).find(|&i| code_at(i) && k[i].1 == '{').unwrap_or_else(|| panic!("{}: fn {name} has no body", self.name));
        let mut depth = 0usize;
        let mut end = k.len() - 1;
        for i in open..k.len() {
            if !code_at(i) {
                continue;
            }
            match k[i].1 {
                '{' => depth += 1,
                '}' => {
                    depth -= 1;
                    if depth == 0 {
                        end = i;
                        break;
                    }
                }
                _ => {}
            }
        }
        let to = k[end].0 + k[end].1.len_utf8();
        Source::new(&format!("{} fn {name}", self.name), &self.text[k[start].0..to])
    }
}
