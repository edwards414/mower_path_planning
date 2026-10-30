//! PyYAML 6.0.1's `yaml.safe_load` of one string, ported module for module:
//! `reader.py` (the printable check, line and column counting), `scanner.py`
//! (tokens, simple keys, indentation, every scalar style and its folding),
//! `parser.py` (the grammar, tag handles), `composer.py` (anchors and
//! aliases), `resolver.py` (YAML 1.1 implicit tags, the same resolver the
//! dumper uses) and `constructor.py` (SafeConstructor: merge keys, the int,
//! float, bool, null and timestamp constructors).
//!
//! The scanner runs to the end of the stream before the parser starts;
//! PyYAML interleaves them, but the scanner never depends on the parser, so
//! the tokens are the same, and `safe_load` reads the whole stream either
//! way (it checks there is no second document), so the same inputs fail.
//!
//! A lone surrogate escape (`"\ud800"`), which a Python `str` holds and a
//! Rust one cannot, becomes U+FFFD, as in `json::loads`.
//!
//! Known gaps, all reported as errors where Python returns a value:
//! - a mapping key that is not a string (`1: a`, `null: a`, `yes: a`);
//! - an int outside i64;
//! - `!!binary` (bytes), `!!set`, `!!omap`, `!!pairs`;
//! - a recursive alias (`&a [*a]`: Python builds a list that holds itself).
//!
//! And one that is not an error: an alias of a collection gives a copy, where
//! Python gives the same object, so `safe_dump` of what was loaded writes the
//! copy out again instead of PyYAML's `&id001` / `*id001`.

use std::collections::HashMap;
use std::fmt;

use crate::yaml::{resolve, Tag};
use crate::Py;

/// Why a document did not load, and where (1-based, as PyYAML prints marks).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct YamlError {
    pub line: usize,
    pub column: usize,
    pub message: String,
}

impl fmt::Display for YamlError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "line {}, column {}: {}", self.line, self.column, self.message)
    }
}

impl std::error::Error for YamlError {}

#[derive(Debug, Clone, Copy, Default)]
struct Mark {
    line: usize,
    column: usize,
}

fn err(mark: Mark, message: impl Into<String>) -> YamlError {
    YamlError { line: mark.line + 1, column: mark.column + 1, message: message.into() }
}

type R<T> = Result<T, YamlError>;

// ---- tokens.py -----------------------------------------------------------------

#[derive(Debug, Clone, PartialEq)]
enum Tok {
    StreamStart,
    StreamEnd,
    Directive { name: String, value: Option<DirValue> },
    DocumentStart,
    DocumentEnd,
    BlockSequenceStart,
    BlockMappingStart,
    BlockEnd,
    FlowSequenceStart,
    FlowMappingStart,
    FlowSequenceEnd,
    FlowMappingEnd,
    Key,
    Value,
    BlockEntry,
    FlowEntry,
    Alias(String),
    Anchor(String),
    Tag(Option<String>, String),
    Scalar { value: String, plain: bool },
}

#[derive(Debug, Clone, PartialEq)]
enum DirValue {
    Yaml(u64, u64),
    Tag(String, String),
}

impl Tok {
    /// The token's `id`, as PyYAML's messages name it.
    fn id(&self) -> &'static str {
        match self {
            Tok::StreamStart => "<stream start>",
            Tok::StreamEnd => "<stream end>",
            Tok::Directive { .. } => "<directive>",
            Tok::DocumentStart => "<document start>",
            Tok::DocumentEnd => "<document end>",
            Tok::BlockSequenceStart => "<block sequence start>",
            Tok::BlockMappingStart => "<block mapping start>",
            Tok::BlockEnd => "<block end>",
            Tok::FlowSequenceStart => "[",
            Tok::FlowMappingStart => "{",
            Tok::FlowSequenceEnd => "]",
            Tok::FlowMappingEnd => "}",
            Tok::Key => "?",
            Tok::Value => ":",
            Tok::BlockEntry => "-",
            Tok::FlowEntry => ",",
            Tok::Alias(_) => "<alias>",
            Tok::Anchor(_) => "<anchor>",
            Tok::Tag(..) => "<tag>",
            Tok::Scalar { .. } => "<scalar>",
        }
    }
}

#[derive(Debug, Clone)]
struct Token {
    tok: Tok,
    start: Mark,
}

// ---- reader.py -------------------------------------------------------------------

/// `Reader.NON_PRINTABLE`: what a document may not contain.
fn printable(c: char) -> bool {
    matches!(c, '\x09' | '\x0A' | '\x0D' | '\x20'..='\x7E' | '\u{85}' | '\u{A0}'..='\u{D7FF}' | '\u{E000}'..='\u{FFFD}'
        | '\u{10000}'..='\u{10FFFF}')
}

fn in_set(c: char, set: &str) -> bool {
    set.contains(c)
}

const BZ: &str = "\0 \t\r\n\u{85}\u{2028}\u{2029}";
const BREAKZ: &str = "\0\r\n\u{85}\u{2028}\u{2029}";
const BREAK: &str = "\r\n\u{85}\u{2028}\u{2029}";

// ---- scanner.py ------------------------------------------------------------------

struct SimpleKey {
    token_number: usize,
    required: bool,
    index: usize,
    line: usize,
    mark: Mark,
    column: usize,
}

struct Scanner {
    buf: Vec<char>,
    pos: usize,
    line: usize,
    column: usize,
    done: bool,
    flow_level: usize,
    tokens: Vec<Token>,
    indent: isize,
    indents: Vec<isize>,
    allow_simple_key: bool,
    possible: HashMap<usize, SimpleKey>,
}

impl Scanner {
    fn new(text: &str) -> R<Scanner> {
        let mut buf: Vec<char> = Vec::with_capacity(text.len() + 1);
        let (mut line, mut column) = (0, 0);
        for (i, c) in text.chars().enumerate() {
            if !printable(c) {
                // ReaderError: the position only (PyYAML gives no line here)
                return Err(YamlError {
                    line: line + 1,
                    column: column + 1,
                    message: format!("unacceptable character #x{:04x}: special characters are not allowed (position {i})", c as u32),
                });
            }
            if c == '\n' {
                line += 1;
                column = 0;
            } else {
                column += 1;
            }
            buf.push(c);
        }
        buf.push('\0');
        let mut s = Scanner {
            buf,
            pos: 0,
            line: 0,
            column: 0,
            done: false,
            flow_level: 0,
            tokens: Vec::new(),
            indent: -1,
            indents: Vec::new(),
            allow_simple_key: true,
            possible: HashMap::new(),
        };
        let m = s.mark();
        s.tokens.push(Token { tok: Tok::StreamStart, start: m });
        Ok(s)
    }

    fn peek(&self, n: usize) -> char {
        self.buf.get(self.pos + n).copied().unwrap_or('\0')
    }

    fn prefix(&self, n: usize) -> String {
        let end = (self.pos + n).min(self.buf.len());
        self.buf[self.pos..end].iter().collect()
    }

    fn forward(&mut self, n: usize) {
        for _ in 0..n {
            if self.pos >= self.buf.len() {
                return;
            }
            let ch = self.buf[self.pos];
            self.pos += 1;
            if in_set(ch, "\n\u{85}\u{2028}\u{2029}") || (ch == '\r' && self.peek(0) != '\n') {
                self.line += 1;
                self.column = 0;
            } else if ch != '\u{FEFF}' {
                self.column += 1;
            }
        }
    }

    fn mark(&self) -> Mark {
        Mark { line: self.line, column: self.column }
    }

    fn push(&mut self, tok: Tok, start: Mark) {
        self.tokens.push(Token { tok, start });
    }

    fn run(mut self) -> R<Vec<Token>> {
        while !self.done {
            self.fetch_more_tokens()?;
        }
        Ok(self.tokens)
    }

    fn fetch_more_tokens(&mut self) -> R<()> {
        self.scan_to_next_token();
        self.stale_possible_simple_keys()?;
        self.unwind_indent(self.column as isize);
        let ch = self.peek(0);
        if ch == '\0' {
            return self.fetch_stream_end();
        }
        if ch == '%' && self.column == 0 {
            return self.fetch_directive();
        }
        if ch == '-' && self.check_document_indicator("---") {
            return self.fetch_document_indicator(Tok::DocumentStart);
        }
        if ch == '.' && self.check_document_indicator("...") {
            return self.fetch_document_indicator(Tok::DocumentEnd);
        }
        match ch {
            '[' => return self.fetch_flow_collection_start(Tok::FlowSequenceStart),
            '{' => return self.fetch_flow_collection_start(Tok::FlowMappingStart),
            ']' => return self.fetch_flow_collection_end(Tok::FlowSequenceEnd),
            '}' => return self.fetch_flow_collection_end(Tok::FlowMappingEnd),
            ',' => return self.fetch_flow_entry(),
            _ => {}
        }
        if ch == '-' && in_set(self.peek(1), BZ) {
            return self.fetch_block_entry();
        }
        if ch == '?' && (self.flow_level > 0 || in_set(self.peek(1), BZ)) {
            return self.fetch_key();
        }
        if ch == ':' && (self.flow_level > 0 || in_set(self.peek(1), BZ)) {
            return self.fetch_value();
        }
        match ch {
            '*' => return self.fetch_anchor(true),
            '&' => return self.fetch_anchor(false),
            '!' => return self.fetch_tag(),
            '|' if self.flow_level == 0 => return self.fetch_block_scalar(false),
            '>' if self.flow_level == 0 => return self.fetch_block_scalar(true),
            '\'' => return self.fetch_flow_scalar(false),
            '"' => return self.fetch_flow_scalar(true),
            _ => {}
        }
        if self.check_plain() {
            return self.fetch_plain();
        }
        Err(err(self.mark(), format!("while scanning for the next token: found character {ch:?} that cannot start any token")))
    }

    fn stale_possible_simple_keys(&mut self) -> R<()> {
        let mut stale = Vec::new();
        for (level, key) in &self.possible {
            if key.line != self.line || self.pos - key.index > 1024 {
                if key.required {
                    return Err(err(key.mark, "while scanning a simple key: could not find expected ':'"));
                }
                stale.push(*level);
            }
        }
        for l in stale {
            self.possible.remove(&l);
        }
        Ok(())
    }

    fn save_possible_simple_key(&mut self) -> R<()> {
        let required = self.flow_level == 0 && self.indent == self.column as isize;
        if self.allow_simple_key {
            self.remove_possible_simple_key()?;
            let key = SimpleKey {
                token_number: self.tokens.len(),
                required,
                index: self.pos,
                line: self.line,
                column: self.column,
                mark: self.mark(),
            };
            self.possible.insert(self.flow_level, key);
        }
        Ok(())
    }

    fn remove_possible_simple_key(&mut self) -> R<()> {
        if let Some(key) = self.possible.remove(&self.flow_level) {
            if key.required {
                return Err(err(key.mark, "while scanning a simple key: could not find expected ':'"));
            }
        }
        Ok(())
    }

    fn unwind_indent(&mut self, column: isize) {
        if self.flow_level > 0 {
            return;
        }
        while self.indent > column {
            let m = self.mark();
            self.indent = self.indents.pop().unwrap_or(-1);
            self.push(Tok::BlockEnd, m);
        }
    }

    fn add_indent(&mut self, column: isize) -> bool {
        if self.indent < column {
            self.indents.push(self.indent);
            self.indent = column;
            true
        } else {
            false
        }
    }

    fn fetch_stream_end(&mut self) -> R<()> {
        self.unwind_indent(-1);
        self.remove_possible_simple_key()?;
        self.allow_simple_key = false;
        self.possible.clear();
        let m = self.mark();
        self.push(Tok::StreamEnd, m);
        self.done = true;
        Ok(())
    }

    fn fetch_directive(&mut self) -> R<()> {
        self.unwind_indent(-1);
        self.remove_possible_simple_key()?;
        self.allow_simple_key = false;
        let start = self.mark();
        let tok = self.scan_directive()?;
        self.push(tok, start);
        Ok(())
    }

    fn check_document_indicator(&self, s: &str) -> bool {
        self.column == 0 && self.prefix(3) == s && in_set(self.peek(3), BZ)
    }

    fn fetch_document_indicator(&mut self, tok: Tok) -> R<()> {
        self.unwind_indent(-1);
        self.remove_possible_simple_key()?;
        self.allow_simple_key = false;
        let start = self.mark();
        self.forward(3);
        self.push(tok, start);
        Ok(())
    }

    fn fetch_flow_collection_start(&mut self, tok: Tok) -> R<()> {
        self.save_possible_simple_key()?;
        self.flow_level += 1;
        self.allow_simple_key = true;
        let start = self.mark();
        self.forward(1);
        self.push(tok, start);
        Ok(())
    }

    fn fetch_flow_collection_end(&mut self, tok: Tok) -> R<()> {
        self.remove_possible_simple_key()?;
        // PyYAML lets flow_level go below 0 on a stray ']' and the parser
        // then fails; here it stays at 0 and the parser fails the same way
        self.flow_level = self.flow_level.saturating_sub(1);
        self.allow_simple_key = false;
        let start = self.mark();
        self.forward(1);
        self.push(tok, start);
        Ok(())
    }

    fn fetch_flow_entry(&mut self) -> R<()> {
        self.allow_simple_key = true;
        self.remove_possible_simple_key()?;
        let start = self.mark();
        self.forward(1);
        self.push(Tok::FlowEntry, start);
        Ok(())
    }

    fn fetch_block_entry(&mut self) -> R<()> {
        if self.flow_level == 0 {
            if !self.allow_simple_key {
                return Err(err(self.mark(), "sequence entries are not allowed here"));
            }
            if self.add_indent(self.column as isize) {
                let m = self.mark();
                self.push(Tok::BlockSequenceStart, m);
            }
        }
        self.allow_simple_key = true;
        self.remove_possible_simple_key()?;
        let start = self.mark();
        self.forward(1);
        self.push(Tok::BlockEntry, start);
        Ok(())
    }

    fn fetch_key(&mut self) -> R<()> {
        if self.flow_level == 0 {
            if !self.allow_simple_key {
                return Err(err(self.mark(), "mapping keys are not allowed here"));
            }
            if self.add_indent(self.column as isize) {
                let m = self.mark();
                self.push(Tok::BlockMappingStart, m);
            }
        }
        self.allow_simple_key = self.flow_level == 0;
        self.remove_possible_simple_key()?;
        let start = self.mark();
        self.forward(1);
        self.push(Tok::Key, start);
        Ok(())
    }

    fn fetch_value(&mut self) -> R<()> {
        if let Some(key) = self.possible.remove(&self.flow_level) {
            self.tokens.insert(key.token_number, Token { tok: Tok::Key, start: key.mark });
            if self.flow_level == 0 && self.add_indent(key.column as isize) {
                self.tokens.insert(key.token_number, Token { tok: Tok::BlockMappingStart, start: key.mark });
            }
            self.allow_simple_key = false;
        } else {
            if self.flow_level == 0 {
                if !self.allow_simple_key {
                    return Err(err(self.mark(), "mapping values are not allowed here"));
                }
                if self.add_indent(self.column as isize) {
                    let m = self.mark();
                    self.push(Tok::BlockMappingStart, m);
                }
            }
            self.allow_simple_key = self.flow_level == 0;
            self.remove_possible_simple_key()?;
        }
        let start = self.mark();
        self.forward(1);
        self.push(Tok::Value, start);
        Ok(())
    }

    fn fetch_anchor(&mut self, alias: bool) -> R<()> {
        self.save_possible_simple_key()?;
        self.allow_simple_key = false;
        let start = self.mark();
        let tok = self.scan_anchor(alias)?;
        self.push(tok, start);
        Ok(())
    }

    fn fetch_tag(&mut self) -> R<()> {
        self.save_possible_simple_key()?;
        self.allow_simple_key = false;
        let start = self.mark();
        let tok = self.scan_tag()?;
        self.push(tok, start);
        Ok(())
    }

    fn fetch_block_scalar(&mut self, folded: bool) -> R<()> {
        self.allow_simple_key = true;
        self.remove_possible_simple_key()?;
        let start = self.mark();
        let tok = self.scan_block_scalar(folded)?;
        self.push(tok, start);
        Ok(())
    }

    fn fetch_flow_scalar(&mut self, double: bool) -> R<()> {
        self.save_possible_simple_key()?;
        self.allow_simple_key = false;
        let start = self.mark();
        let tok = self.scan_flow_scalar(double)?;
        self.push(tok, start);
        Ok(())
    }

    fn fetch_plain(&mut self) -> R<()> {
        self.save_possible_simple_key()?;
        self.allow_simple_key = false;
        let start = self.mark();
        let tok = self.scan_plain()?;
        self.push(tok, start);
        Ok(())
    }

    fn check_plain(&self) -> bool {
        let ch = self.peek(0);
        !in_set(ch, "\0 \t\r\n\u{85}\u{2028}\u{2029}-?:,[]{}#&*!|>'\"%@`")
            || (!in_set(self.peek(1), BZ) && (ch == '-' || (self.flow_level == 0 && in_set(ch, "?:"))))
    }

    fn scan_to_next_token(&mut self) {
        if self.pos == 0 && self.peek(0) == '\u{FEFF}' {
            self.forward(1);
        }
        loop {
            while self.peek(0) == ' ' {
                self.forward(1);
            }
            if self.peek(0) == '#' {
                while !in_set(self.peek(0), BREAKZ) {
                    self.forward(1);
                }
            }
            if !self.scan_line_break().is_empty() {
                if self.flow_level == 0 {
                    self.allow_simple_key = true;
                }
            } else {
                break;
            }
        }
    }

    fn scan_directive(&mut self) -> R<Tok> {
        let start = self.mark();
        self.forward(1);
        let name = self.scan_directive_name(start)?;
        let value = if name == "YAML" {
            Some(self.scan_yaml_directive_value(start)?)
        } else if name == "TAG" {
            Some(self.scan_tag_directive_value(start)?)
        } else {
            while !in_set(self.peek(0), BREAKZ) {
                self.forward(1);
            }
            None
        };
        self.scan_directive_ignored_line(start)?;
        Ok(Tok::Directive { name, value })
    }

    fn name_char(c: char) -> bool {
        c.is_ascii_alphanumeric() || c == '-' || c == '_'
    }

    fn scan_directive_name(&mut self, start: Mark) -> R<String> {
        let mut n = 0;
        while Self::name_char(self.peek(n)) {
            n += 1;
        }
        if n == 0 {
            return Err(err(self.mark(), format!("while scanning a directive (line {}): expected alphabetic or numeric character, but found {:?}", start.line + 1, self.peek(0))));
        }
        let v = self.prefix(n);
        self.forward(n);
        if !in_set(self.peek(0), "\0 \r\n\u{85}\u{2028}\u{2029}") {
            return Err(err(self.mark(), format!("while scanning a directive: expected alphabetic or numeric character, but found {:?}", self.peek(0))));
        }
        Ok(v)
    }

    fn scan_yaml_directive_value(&mut self, _start: Mark) -> R<DirValue> {
        while self.peek(0) == ' ' {
            self.forward(1);
        }
        let major = self.scan_yaml_directive_number()?;
        if self.peek(0) != '.' {
            return Err(err(self.mark(), format!("while scanning a directive: expected a digit or '.', but found {:?}", self.peek(0))));
        }
        self.forward(1);
        let minor = self.scan_yaml_directive_number()?;
        if !in_set(self.peek(0), "\0 \r\n\u{85}\u{2028}\u{2029}") {
            return Err(err(self.mark(), format!("while scanning a directive: expected a digit or ' ', but found {:?}", self.peek(0))));
        }
        Ok(DirValue::Yaml(major, minor))
    }

    fn scan_yaml_directive_number(&mut self) -> R<u64> {
        if !self.peek(0).is_ascii_digit() {
            return Err(err(self.mark(), format!("while scanning a directive: expected a digit, but found {:?}", self.peek(0))));
        }
        let mut n = 0;
        while self.peek(n).is_ascii_digit() {
            n += 1;
        }
        let v = self.prefix(n).parse::<u64>().map_err(|_| err(self.mark(), "a YAML directive number out of range"))?;
        self.forward(n);
        Ok(v)
    }

    fn scan_tag_directive_value(&mut self, start: Mark) -> R<DirValue> {
        while self.peek(0) == ' ' {
            self.forward(1);
        }
        let handle = self.scan_tag_handle("directive", start)?;
        if self.peek(0) != ' ' {
            return Err(err(self.mark(), format!("while scanning a directive: expected ' ', but found {:?}", self.peek(0))));
        }
        while self.peek(0) == ' ' {
            self.forward(1);
        }
        let prefix = self.scan_tag_uri("directive", start)?;
        if !in_set(self.peek(0), "\0 \r\n\u{85}\u{2028}\u{2029}") {
            return Err(err(self.mark(), format!("while scanning a directive: expected ' ', but found {:?}", self.peek(0))));
        }
        Ok(DirValue::Tag(handle, prefix))
    }

    fn scan_directive_ignored_line(&mut self, _start: Mark) -> R<()> {
        while self.peek(0) == ' ' {
            self.forward(1);
        }
        if self.peek(0) == '#' {
            while !in_set(self.peek(0), BREAKZ) {
                self.forward(1);
            }
        }
        if !in_set(self.peek(0), BREAKZ) {
            return Err(err(self.mark(), format!("while scanning a directive: expected a comment or a line break, but found {:?}", self.peek(0))));
        }
        self.scan_line_break();
        Ok(())
    }

    fn scan_anchor(&mut self, alias: bool) -> R<Tok> {
        let what = if alias { "alias" } else { "anchor" };
        self.forward(1);
        let mut n = 0;
        while Self::name_char(self.peek(n)) {
            n += 1;
        }
        if n == 0 {
            return Err(err(self.mark(), format!("while scanning an {what}: expected alphabetic or numeric character, but found {:?}", self.peek(0))));
        }
        let v = self.prefix(n);
        self.forward(n);
        if !in_set(self.peek(0), "\0 \t\r\n\u{85}\u{2028}\u{2029}?:,]}%@`") {
            return Err(err(self.mark(), format!("while scanning an {what}: expected alphabetic or numeric character, but found {:?}", self.peek(0))));
        }
        Ok(if alias { Tok::Alias(v) } else { Tok::Anchor(v) })
    }

    fn scan_tag(&mut self) -> R<Tok> {
        let start = self.mark();
        let ch = self.peek(1);
        let (handle, suffix);
        if ch == '<' {
            handle = None;
            self.forward(2);
            suffix = self.scan_tag_uri("tag", start)?;
            if self.peek(0) != '>' {
                return Err(err(self.mark(), format!("while parsing a tag: expected '>', but found {:?}", self.peek(0))));
            }
            self.forward(1);
        } else if in_set(ch, BZ) {
            handle = None;
            suffix = "!".to_string();
            self.forward(1);
        } else {
            let mut n = 1;
            let mut c = ch;
            let mut use_handle = false;
            while !in_set(c, "\0 \r\n\u{85}\u{2028}\u{2029}") {
                if c == '!' {
                    use_handle = true;
                    break;
                }
                n += 1;
                c = self.peek(n);
            }
            if use_handle {
                handle = Some(self.scan_tag_handle("tag", start)?);
            } else {
                handle = Some("!".to_string());
                self.forward(1);
            }
            suffix = self.scan_tag_uri("tag", start)?;
        }
        if !in_set(self.peek(0), "\0 \r\n\u{85}\u{2028}\u{2029}") {
            return Err(err(self.mark(), format!("while scanning a tag: expected ' ', but found {:?}", self.peek(0))));
        }
        Ok(Tok::Tag(handle, suffix))
    }

    fn scan_block_scalar(&mut self, folded: bool) -> R<Tok> {
        let mut chunks = String::new();
        let start = self.mark();
        self.forward(1);
        let (chomping, increment) = self.scan_block_scalar_indicators(start)?;
        self.scan_block_scalar_ignored_line()?;
        let min_indent = (self.indent + 1).max(1) as usize;
        let (mut breaks, indent);
        if let Some(inc) = increment {
            indent = min_indent + inc - 1;
            breaks = self.scan_block_scalar_breaks(indent);
        } else {
            let (b, max_indent) = self.scan_block_scalar_indentation();
            breaks = b;
            indent = min_indent.max(max_indent);
        }
        let mut line_break = String::new();
        while self.column == indent && self.peek(0) != '\0' {
            chunks.push_str(&breaks.concat());
            let leading_non_space = !in_set(self.peek(0), " \t");
            let mut n = 0;
            while !in_set(self.peek(n), BREAKZ) {
                n += 1;
            }
            chunks.push_str(&self.prefix(n));
            self.forward(n);
            line_break = self.scan_line_break();
            breaks = self.scan_block_scalar_breaks(indent);
            if self.column == indent && self.peek(0) != '\0' {
                if folded && line_break == "\n" && leading_non_space && !in_set(self.peek(0), " \t") {
                    if breaks.is_empty() {
                        chunks.push(' ');
                    }
                } else {
                    chunks.push_str(&line_break);
                }
            } else {
                break;
            }
        }
        if chomping != Some(false) {
            chunks.push_str(&line_break);
        }
        if chomping == Some(true) {
            chunks.push_str(&breaks.concat());
        }
        Ok(Tok::Scalar { value: chunks, plain: false })
    }

    fn scan_block_scalar_indicators(&mut self, _start: Mark) -> R<(Option<bool>, Option<usize>)> {
        let mut chomping = None;
        let mut increment = None;
        let bad0 = |s: &Scanner| err(s.mark(), "while scanning a block scalar: expected indentation indicator in the range 1-9, but found 0");
        let ch = self.peek(0);
        if in_set(ch, "+-") {
            chomping = Some(ch == '+');
            self.forward(1);
            let ch = self.peek(0);
            if ch.is_ascii_digit() {
                let d = ch.to_digit(10).unwrap() as usize;
                if d == 0 {
                    return Err(bad0(self));
                }
                increment = Some(d);
                self.forward(1);
            }
        } else if ch.is_ascii_digit() {
            let d = ch.to_digit(10).unwrap() as usize;
            if d == 0 {
                return Err(bad0(self));
            }
            increment = Some(d);
            self.forward(1);
            let ch = self.peek(0);
            if in_set(ch, "+-") {
                chomping = Some(ch == '+');
                self.forward(1);
            }
        }
        if !in_set(self.peek(0), "\0 \r\n\u{85}\u{2028}\u{2029}") {
            return Err(err(self.mark(), format!("while scanning a block scalar: expected chomping or indentation indicators, but found {:?}", self.peek(0))));
        }
        Ok((chomping, increment))
    }

    fn scan_block_scalar_ignored_line(&mut self) -> R<()> {
        while self.peek(0) == ' ' {
            self.forward(1);
        }
        if self.peek(0) == '#' {
            while !in_set(self.peek(0), BREAKZ) {
                self.forward(1);
            }
        }
        if !in_set(self.peek(0), BREAKZ) {
            return Err(err(self.mark(), format!("while scanning a block scalar: expected a comment or a line break, but found {:?}", self.peek(0))));
        }
        self.scan_line_break();
        Ok(())
    }

    fn scan_block_scalar_indentation(&mut self) -> (Vec<String>, usize) {
        let mut chunks = Vec::new();
        let mut max_indent = 0;
        while in_set(self.peek(0), " \r\n\u{85}\u{2028}\u{2029}") {
            if self.peek(0) != ' ' {
                chunks.push(self.scan_line_break());
            } else {
                self.forward(1);
                if self.column > max_indent {
                    max_indent = self.column;
                }
            }
        }
        (chunks, max_indent)
    }

    fn scan_block_scalar_breaks(&mut self, indent: usize) -> Vec<String> {
        let mut chunks = Vec::new();
        while self.column < indent && self.peek(0) == ' ' {
            self.forward(1);
        }
        while in_set(self.peek(0), BREAK) {
            chunks.push(self.scan_line_break());
            while self.column < indent && self.peek(0) == ' ' {
                self.forward(1);
            }
        }
        chunks
    }

    fn scan_flow_scalar(&mut self, double: bool) -> R<Tok> {
        let start = self.mark();
        let mut chunks = String::new();
        let quote = self.peek(0);
        self.forward(1);
        self.scan_flow_scalar_non_spaces(double, start, &mut chunks)?;
        while self.peek(0) != quote {
            self.scan_flow_scalar_spaces(start, &mut chunks)?;
            self.scan_flow_scalar_non_spaces(double, start, &mut chunks)?;
        }
        self.forward(1);
        Ok(Tok::Scalar { value: chunks, plain: false })
    }

    fn scan_flow_scalar_non_spaces(&mut self, double: bool, start: Mark, chunks: &mut String) -> R<()> {
        loop {
            let mut n = 0;
            while !in_set(self.peek(n), "'\"\\\0 \t\r\n\u{85}\u{2028}\u{2029}") {
                n += 1;
            }
            if n > 0 {
                chunks.push_str(&self.prefix(n));
                self.forward(n);
            }
            let ch = self.peek(0);
            if !double && ch == '\'' && self.peek(1) == '\'' {
                chunks.push('\'');
                self.forward(2);
            } else if (double && ch == '\'') || (!double && in_set(ch, "\"\\")) {
                chunks.push(ch);
                self.forward(1);
            } else if double && ch == '\\' {
                self.forward(1);
                let ch = self.peek(0);
                let rep = match ch {
                    '0' => Some('\0'),
                    'a' => Some('\x07'),
                    'b' => Some('\x08'),
                    't' | '\t' => Some('\x09'),
                    'n' => Some('\x0A'),
                    'v' => Some('\x0B'),
                    'f' => Some('\x0C'),
                    'r' => Some('\x0D'),
                    'e' => Some('\x1B'),
                    ' ' => Some(' '),
                    '"' => Some('"'),
                    '\\' => Some('\\'),
                    '/' => Some('/'),
                    'N' => Some('\u{85}'),
                    '_' => Some('\u{A0}'),
                    'L' => Some('\u{2028}'),
                    'P' => Some('\u{2029}'),
                    _ => None,
                };
                if let Some(r) = rep {
                    chunks.push(r);
                    self.forward(1);
                } else if let Some(len) = match ch {
                    'x' => Some(2),
                    'u' => Some(4),
                    'U' => Some(8),
                    _ => None,
                } {
                    self.forward(1);
                    for k in 0..len {
                        if !self.peek(k).is_ascii_hexdigit() {
                            return Err(err(self.mark(), format!(
                                "while scanning a double-quoted scalar (line {}): expected escape sequence of {len} hexadecimal numbers, but found {:?}",
                                start.line + 1,
                                self.peek(k)
                            )));
                        }
                    }
                    let code = u32::from_str_radix(&self.prefix(len), 16).unwrap();
                    // chr() of a lone surrogate is a str a Rust one cannot
                    // be: U+FFFD, as json::loads does; above 0x10FFFF chr()
                    // raises
                    let c = if (0xD800..=0xDFFF).contains(&code) {
                        '\u{FFFD}'
                    } else {
                        char::from_u32(code)
                            .ok_or_else(|| err(self.mark(), format!("while scanning a double-quoted scalar: chr() of {code:#x} is out of range")))?
                    };
                    chunks.push(c);
                    self.forward(len);
                } else if in_set(ch, BREAK) {
                    self.scan_line_break();
                    self.scan_flow_scalar_breaks(start, chunks)?;
                } else {
                    return Err(err(self.mark(), format!("while scanning a double-quoted scalar: found unknown escape character {ch:?}")));
                }
            } else {
                return Ok(());
            }
        }
    }

    fn scan_flow_scalar_spaces(&mut self, start: Mark, chunks: &mut String) -> R<()> {
        let mut n = 0;
        while in_set(self.peek(n), " \t") {
            n += 1;
        }
        let whitespaces = self.prefix(n);
        self.forward(n);
        let ch = self.peek(0);
        if ch == '\0' {
            return Err(err(self.mark(), format!("while scanning a quoted scalar (line {}): found unexpected end of stream", start.line + 1)));
        } else if in_set(ch, BREAK) {
            let line_break = self.scan_line_break();
            let mut breaks = String::new();
            self.scan_flow_scalar_breaks(start, &mut breaks)?;
            if line_break != "\n" {
                chunks.push_str(&line_break);
            } else if breaks.is_empty() {
                chunks.push(' ');
            }
            chunks.push_str(&breaks);
        } else {
            chunks.push_str(&whitespaces);
        }
        Ok(())
    }

    fn scan_flow_scalar_breaks(&mut self, start: Mark, chunks: &mut String) -> R<()> {
        loop {
            let p = self.prefix(3);
            if (p == "---" || p == "...") && in_set(self.peek(3), BZ) {
                return Err(err(self.mark(), format!("while scanning a quoted scalar (line {}): found unexpected document separator", start.line + 1)));
            }
            while in_set(self.peek(0), " \t") {
                self.forward(1);
            }
            if in_set(self.peek(0), BREAK) {
                let b = self.scan_line_break();
                chunks.push_str(&b);
            } else {
                return Ok(());
            }
        }
    }

    fn scan_plain(&mut self) -> R<Tok> {
        let mut chunks = String::new();
        let indent = (self.indent + 1) as usize;
        let mut spaces: Option<String> = Some(String::new());
        loop {
            let mut n = 0;
            if self.peek(0) == '#' {
                break;
            }
            loop {
                let ch = self.peek(n);
                let colon_end = if self.flow_level > 0 { format!("{BZ},[]{{}}") } else { BZ.to_string() };
                if in_set(ch, BZ)
                    || (ch == ':' && in_set(self.peek(n + 1), &colon_end))
                    || (self.flow_level > 0 && in_set(ch, ",?[]{}"))
                {
                    break;
                }
                n += 1;
            }
            if n == 0 {
                break;
            }
            self.allow_simple_key = false;
            chunks.push_str(spaces.as_deref().unwrap_or(""));
            chunks.push_str(&self.prefix(n));
            self.forward(n);
            spaces = self.scan_plain_spaces();
            match &spaces {
                None => break,
                Some(s) if s.is_empty() => break,
                _ => {}
            }
            if self.peek(0) == '#' || (self.flow_level == 0 && self.column < indent) {
                break;
            }
        }
        Ok(Tok::Scalar { value: chunks, plain: true })
    }

    /// `scan_plain_spaces`: `None` where Python returns nothing (a document
    /// separator follows), else the chunks joined.
    fn scan_plain_spaces(&mut self) -> Option<String> {
        let mut chunks = String::new();
        let mut n = 0;
        while self.peek(n) == ' ' {
            n += 1;
        }
        let whitespaces = self.prefix(n);
        self.forward(n);
        let ch = self.peek(0);
        if in_set(ch, BREAK) {
            let line_break = self.scan_line_break();
            self.allow_simple_key = true;
            let p = self.prefix(3);
            if (p == "---" || p == "...") && in_set(self.peek(3), BZ) {
                return None;
            }
            let mut breaks = String::new();
            while in_set(self.peek(0), " \r\n\u{85}\u{2028}\u{2029}") {
                if self.peek(0) == ' ' {
                    self.forward(1);
                } else {
                    breaks.push_str(&self.scan_line_break());
                    let p = self.prefix(3);
                    if (p == "---" || p == "...") && in_set(self.peek(3), BZ) {
                        return None;
                    }
                }
            }
            if line_break != "\n" {
                chunks.push_str(&line_break);
            } else if breaks.is_empty() {
                chunks.push(' ');
            }
            chunks.push_str(&breaks);
        } else if !whitespaces.is_empty() {
            chunks.push_str(&whitespaces);
        }
        Some(chunks)
    }

    fn scan_tag_handle(&mut self, name: &str, _start: Mark) -> R<String> {
        if self.peek(0) != '!' {
            return Err(err(self.mark(), format!("while scanning a {name}: expected '!', but found {:?}", self.peek(0))));
        }
        let mut n = 1;
        let mut ch = self.peek(n);
        if ch != ' ' {
            while Self::name_char(ch) {
                n += 1;
                ch = self.peek(n);
            }
            if ch != '!' {
                self.forward(n);
                return Err(err(self.mark(), format!("while scanning a {name}: expected '!', but found {ch:?}")));
            }
            n += 1;
        }
        let v = self.prefix(n);
        self.forward(n);
        Ok(v)
    }

    fn scan_tag_uri(&mut self, name: &str, _start: Mark) -> R<String> {
        let mut chunks = String::new();
        let mut n = 0;
        let mut ch = self.peek(n);
        while ch.is_ascii_alphanumeric() || in_set(ch, "-;/?:@&=+$,_.!~*'()[]%") {
            if ch == '%' {
                chunks.push_str(&self.prefix(n));
                self.forward(n);
                n = 0;
                chunks.push_str(&self.scan_uri_escapes(name)?);
            } else {
                n += 1;
            }
            ch = self.peek(n);
        }
        if n > 0 {
            chunks.push_str(&self.prefix(n));
            self.forward(n);
        }
        if chunks.is_empty() {
            return Err(err(self.mark(), format!("while parsing a {name}: expected URI, but found {ch:?}")));
        }
        Ok(chunks)
    }

    fn scan_uri_escapes(&mut self, name: &str) -> R<String> {
        let mut codes = Vec::new();
        while self.peek(0) == '%' {
            self.forward(1);
            for k in 0..2 {
                if !self.peek(k).is_ascii_hexdigit() {
                    return Err(err(self.mark(), format!(
                        "while scanning a {name}: expected URI escape sequence of 2 hexadecimal numbers, but found {:?}",
                        self.peek(k)
                    )));
                }
            }
            codes.push(u8::from_str_radix(&self.prefix(2), 16).unwrap());
            self.forward(2);
        }
        String::from_utf8(codes).map_err(|e| err(self.mark(), format!("while scanning a {name}: {e}")))
    }

    fn scan_line_break(&mut self) -> String {
        let ch = self.peek(0);
        if in_set(ch, "\r\n\u{85}") {
            if self.prefix(2) == "\r\n" {
                self.forward(2);
            } else {
                self.forward(1);
            }
            "\n".to_string()
        } else if in_set(ch, "\u{2028}\u{2029}") {
            self.forward(1);
            ch.to_string()
        } else {
            String::new()
        }
    }
}

// ---- parser.py + composer.py -----------------------------------------------------

const TAG_PREFIX: &str = "tag:yaml.org,2002:";

#[derive(Debug, Clone)]
enum NodeKind {
    Scalar(String),
    Seq(Vec<usize>),
    Map(Vec<(usize, usize)>),
}

#[derive(Debug, Clone)]
struct Node {
    tag: String,
    kind: NodeKind,
    mark: Mark,
}

struct Parser {
    toks: Vec<Token>,
    i: usize,
    handles: HashMap<String, String>,
    nodes: Vec<Node>,
    anchors: HashMap<String, usize>,
}

fn default_handles() -> HashMap<String, String> {
    HashMap::from([("!".to_string(), "!".to_string()), ("!!".to_string(), TAG_PREFIX.to_string())])
}

/// `Resolver.resolve` for a scalar: the implicit tag of a plain one, else
/// `str`.
fn resolve_scalar(value: &str, plain_implicit: bool) -> String {
    if plain_implicit {
        let name = match resolve(value) {
            Tag::Str => "str",
            Tag::Bool => "bool",
            Tag::Int => "int",
            Tag::Float => "float",
            Tag::Null => "null",
            Tag::Merge => "merge",
            Tag::Timestamp => "timestamp",
            Tag::Value => "value",
            Tag::Yaml => "yaml",
        };
        format!("{TAG_PREFIX}{name}")
    } else {
        format!("{TAG_PREFIX}str")
    }
}

impl Parser {
    fn peek(&self) -> &Token {
        &self.toks[self.i.min(self.toks.len() - 1)]
    }

    fn check(&self, f: impl Fn(&Tok) -> bool) -> bool {
        f(&self.peek().tok)
    }

    fn get(&mut self) -> Token {
        let t = self.peek().clone();
        if self.i < self.toks.len() {
            self.i += 1;
        }
        t
    }

    fn add(&mut self, tag: String, kind: NodeKind, mark: Mark) -> usize {
        self.nodes.push(Node { tag, kind, mark });
        self.nodes.len() - 1
    }

    fn empty_scalar(&mut self, mark: Mark) -> usize {
        // process_empty_scalar: implicit (True, False), '' -> null
        let tag = resolve_scalar("", true);
        self.add(tag, NodeKind::Scalar(String::new()), mark)
    }

    /// `get_single_node`: the one document, or `None` for an empty stream.
    fn single_document(&mut self) -> R<Option<usize>> {
        let t = self.get();
        debug_assert_eq!(t.tok, Tok::StreamStart);
        // parse_implicit_document_start
        let root = if !self.check(|t| matches!(t, Tok::Directive { .. } | Tok::DocumentStart | Tok::StreamEnd)) {
            self.handles = default_handles();
            self.parse_node(true, false)?
        } else if let Some(r) = self.explicit_document()? {
            r
        } else {
            return Ok(None);
        };
        self.document_end();
        // another document?
        let at = self.peek().start;
        if self.explicit_document()?.is_some() {
            return Err(err(at, "expected a single document in the stream, but found another document"));
        }
        Ok(Some(root))
    }

    fn document_end(&mut self) {
        if self.check(|t| *t == Tok::DocumentEnd) {
            self.get();
        }
        self.anchors.clear();
    }

    /// `parse_document_start` and `parse_document_content`: `None` at the end
    /// of the stream.
    fn explicit_document(&mut self) -> R<Option<usize>> {
        while self.check(|t| *t == Tok::DocumentEnd) {
            self.get();
        }
        if self.check(|t| *t == Tok::StreamEnd) {
            return Ok(None);
        }
        self.process_directives()?;
        if !self.check(|t| *t == Tok::DocumentStart) {
            let t = self.peek();
            return Err(err(t.start, format!("expected '<document start>', but found {:?}", t.tok.id())));
        }
        self.get();
        if self.check(|t| matches!(t, Tok::Directive { .. } | Tok::DocumentStart | Tok::DocumentEnd | Tok::StreamEnd)) {
            let m = self.peek().start;
            return Ok(Some(self.empty_scalar(m)));
        }
        Ok(Some(self.parse_node(true, false)?))
    }

    fn process_directives(&mut self) -> R<()> {
        let mut version = None;
        self.handles = HashMap::new();
        while let Tok::Directive { name, value } = &self.peek().tok {
            let (name, value, start) = (name.clone(), value.clone(), self.peek().start);
            self.get();
            match (name.as_str(), value) {
                ("YAML", Some(DirValue::Yaml(major, minor))) => {
                    if version.is_some() {
                        return Err(err(start, "found duplicate YAML directive"));
                    }
                    if major != 1 {
                        return Err(err(start, "found incompatible YAML document (version 1.* is required)"));
                    }
                    version = Some((major, minor));
                }
                ("TAG", Some(DirValue::Tag(handle, prefix))) => {
                    if self.handles.contains_key(&handle) {
                        return Err(err(start, format!("duplicate tag handle {handle:?}")));
                    }
                    self.handles.insert(handle, prefix);
                }
                _ => {}
            }
        }
        for (k, v) in default_handles() {
            self.handles.entry(k).or_insert(v);
        }
        Ok(())
    }

    /// `parse_node` with `compose_node`.
    fn parse_node(&mut self, block: bool, indentless_sequence: bool) -> R<usize> {
        if let Tok::Alias(name) = &self.peek().tok {
            let (name, m) = (name.clone(), self.peek().start);
            self.get();
            return self.anchors.get(&name).copied().ok_or_else(|| err(m, format!("found undefined alias {name:?}")));
        }
        let mut anchor = None;
        let mut tag: Option<(Option<String>, String)> = None;
        let mut start = None;
        let mut tag_mark = None;
        if let Tok::Anchor(a) = &self.peek().tok {
            anchor = Some(a.clone());
            start = Some(self.peek().start);
            self.get();
            if let Tok::Tag(h, s) = &self.peek().tok {
                tag = Some((h.clone(), s.clone()));
                tag_mark = Some(self.peek().start);
                self.get();
            }
        } else if let Tok::Tag(h, s) = &self.peek().tok {
            tag = Some((h.clone(), s.clone()));
            start = Some(self.peek().start);
            tag_mark = start;
            self.get();
            if let Tok::Anchor(a) = &self.peek().tok {
                anchor = Some(a.clone());
                self.get();
            }
        }
        let tag: Option<String> = match tag {
            None => None,
            Some((None, suffix)) => Some(suffix),
            Some((Some(handle), suffix)) => match self.handles.get(&handle) {
                Some(prefix) => Some(format!("{prefix}{suffix}")),
                None => return Err(err(tag_mark.unwrap(), format!("while parsing a node: found undefined tag handle {handle:?}"))),
            },
        };
        let start = start.unwrap_or(self.peek().start);
        if let Some(a) = &anchor {
            if self.anchors.contains_key(a) {
                return Err(err(start, format!("found duplicate anchor {a:?}")));
            }
        }
        let nonspecific = tag.is_none() || tag.as_deref() == Some("!");
        let coll_tag = |default: &str| if nonspecific { format!("{TAG_PREFIX}{default}") } else { tag.clone().unwrap() };
        if indentless_sequence && self.check(|t| *t == Tok::BlockEntry) {
            let idx = self.add(coll_tag("seq"), NodeKind::Seq(Vec::new()), start);
            self.register(&anchor, idx);
            let mut items = Vec::new();
            while self.check(|t| *t == Tok::BlockEntry) {
                let t = self.get();
                if !self.check(|t| matches!(t, Tok::BlockEntry | Tok::Key | Tok::Value | Tok::BlockEnd)) {
                    items.push(self.parse_node(true, false)?);
                } else {
                    items.push(self.empty_scalar(t.start));
                }
            }
            self.nodes[idx].kind = NodeKind::Seq(items);
            return Ok(idx);
        }
        if let Tok::Scalar { value, plain } = &self.peek().tok {
            let (value, plain) = (value.clone(), *plain);
            self.get();
            let t = if nonspecific {
                // implicit[0]: a plain scalar without a tag, or a '!' tag
                resolve_scalar(&value, (plain && tag.is_none()) || tag.as_deref() == Some("!"))
            } else {
                tag.clone().unwrap()
            };
            let idx = self.add(t, NodeKind::Scalar(value), start);
            self.register(&anchor, idx);
            return Ok(idx);
        }
        if self.check(|t| *t == Tok::FlowSequenceStart) {
            let idx = self.add(coll_tag("seq"), NodeKind::Seq(Vec::new()), start);
            self.register(&anchor, idx);
            let items = self.flow_sequence()?;
            self.nodes[idx].kind = NodeKind::Seq(items);
            return Ok(idx);
        }
        if self.check(|t| *t == Tok::FlowMappingStart) {
            let idx = self.add(coll_tag("map"), NodeKind::Map(Vec::new()), start);
            self.register(&anchor, idx);
            let items = self.flow_mapping()?;
            self.nodes[idx].kind = NodeKind::Map(items);
            return Ok(idx);
        }
        if block && self.check(|t| *t == Tok::BlockSequenceStart) {
            let idx = self.add(coll_tag("seq"), NodeKind::Seq(Vec::new()), start);
            self.register(&anchor, idx);
            let items = self.block_sequence()?;
            self.nodes[idx].kind = NodeKind::Seq(items);
            return Ok(idx);
        }
        if block && self.check(|t| *t == Tok::BlockMappingStart) {
            let idx = self.add(coll_tag("map"), NodeKind::Map(Vec::new()), start);
            self.register(&anchor, idx);
            let items = self.block_mapping()?;
            self.nodes[idx].kind = NodeKind::Map(items);
            return Ok(idx);
        }
        if anchor.is_some() || tag.is_some() {
            // an empty scalar: implicit = (tag is None or tag == '!', False)
            let t = if nonspecific { resolve_scalar("", true) } else { tag.clone().unwrap() };
            let idx = self.add(t, NodeKind::Scalar(String::new()), start);
            self.register(&anchor, idx);
            return Ok(idx);
        }
        let t = self.peek();
        Err(err(t.start, format!(
            "while parsing a {} node: expected the node content, but found {:?}",
            if block { "block" } else { "flow" },
            t.tok.id()
        )))
    }

    fn register(&mut self, anchor: &Option<String>, idx: usize) {
        if let Some(a) = anchor {
            self.anchors.insert(a.clone(), idx);
        }
    }

    fn block_sequence(&mut self) -> R<Vec<usize>> {
        let open = self.get().start;
        let mut items = Vec::new();
        loop {
            if self.check(|t| *t == Tok::BlockEntry) {
                let t = self.get();
                if !self.check(|t| matches!(t, Tok::BlockEntry | Tok::BlockEnd)) {
                    items.push(self.parse_node(true, false)?);
                } else {
                    items.push(self.empty_scalar(t.start));
                }
                continue;
            }
            if !self.check(|t| *t == Tok::BlockEnd) {
                let t = self.peek();
                return Err(err(t.start, format!(
                    "while parsing a block collection (line {}): expected <block end>, but found {:?}",
                    open.line + 1,
                    t.tok.id()
                )));
            }
            self.get();
            return Ok(items);
        }
    }

    fn block_mapping(&mut self) -> R<Vec<(usize, usize)>> {
        let open = self.get().start;
        let mut items = Vec::new();
        loop {
            if !self.check(|t| *t == Tok::Key) {
                if !self.check(|t| *t == Tok::BlockEnd) {
                    let t = self.peek();
                    return Err(err(t.start, format!(
                        "while parsing a block mapping (line {}): expected <block end>, but found {:?}",
                        open.line + 1,
                        t.tok.id()
                    )));
                }
                self.get();
                return Ok(items);
            }
            let t = self.get();
            let key = if !self.check(|t| matches!(t, Tok::Key | Tok::Value | Tok::BlockEnd)) {
                self.parse_node(true, true)?
            } else {
                self.empty_scalar(t.start)
            };
            let value = if self.check(|t| *t == Tok::Value) {
                let t = self.get();
                if !self.check(|t| matches!(t, Tok::Key | Tok::Value | Tok::BlockEnd)) {
                    self.parse_node(true, true)?
                } else {
                    self.empty_scalar(t.start)
                }
            } else {
                let m = self.peek().start;
                self.empty_scalar(m)
            };
            items.push((key, value));
        }
    }

    fn flow_sequence(&mut self) -> R<Vec<usize>> {
        let open = self.get().start;
        let mut items = Vec::new();
        let mut first = true;
        loop {
            if self.check(|t| *t == Tok::FlowSequenceEnd) {
                self.get();
                return Ok(items);
            }
            if !first {
                if self.check(|t| *t == Tok::FlowEntry) {
                    self.get();
                } else {
                    let t = self.peek();
                    return Err(err(t.start, format!(
                        "while parsing a flow sequence (line {}): expected ',' or ']', but got {:?}",
                        open.line + 1,
                        t.tok.id()
                    )));
                }
            }
            first = false;
            if self.check(|t| *t == Tok::Key) {
                // a single-pair mapping inside the sequence
                let km = self.peek().start;
                let idx = self.add(format!("{TAG_PREFIX}map"), NodeKind::Map(Vec::new()), km);
                let t = self.get();
                let key = if !self.check(|t| matches!(t, Tok::Value | Tok::FlowEntry | Tok::FlowSequenceEnd)) {
                    self.parse_node(false, false)?
                } else {
                    self.empty_scalar(t.start)
                };
                let value = if self.check(|t| *t == Tok::Value) {
                    let t = self.get();
                    if !self.check(|t| matches!(t, Tok::FlowEntry | Tok::FlowSequenceEnd)) {
                        self.parse_node(false, false)?
                    } else {
                        self.empty_scalar(t.start)
                    }
                } else {
                    let m = self.peek().start;
                    self.empty_scalar(m)
                };
                self.nodes[idx].kind = NodeKind::Map(vec![(key, value)]);
                items.push(idx);
            } else if !self.check(|t| *t == Tok::FlowSequenceEnd) {
                items.push(self.parse_node(false, false)?);
            }
        }
    }

    fn flow_mapping(&mut self) -> R<Vec<(usize, usize)>> {
        let open = self.get().start;
        let mut items = Vec::new();
        let mut first = true;
        loop {
            if self.check(|t| *t == Tok::FlowMappingEnd) {
                self.get();
                return Ok(items);
            }
            if !first {
                if self.check(|t| *t == Tok::FlowEntry) {
                    self.get();
                } else {
                    let t = self.peek();
                    return Err(err(t.start, format!(
                        "while parsing a flow mapping (line {}): expected ',' or '}}', but got {:?}",
                        open.line + 1,
                        t.tok.id()
                    )));
                }
            }
            first = false;
            if self.check(|t| *t == Tok::Key) {
                let t = self.get();
                let key = if !self.check(|t| matches!(t, Tok::Value | Tok::FlowEntry | Tok::FlowMappingEnd)) {
                    self.parse_node(false, false)?
                } else {
                    self.empty_scalar(t.start)
                };
                let value = if self.check(|t| *t == Tok::Value) {
                    let t = self.get();
                    if !self.check(|t| matches!(t, Tok::FlowEntry | Tok::FlowMappingEnd)) {
                        self.parse_node(false, false)?
                    } else {
                        self.empty_scalar(t.start)
                    }
                } else {
                    let m = self.peek().start;
                    self.empty_scalar(m)
                };
                items.push((key, value));
            } else if !self.check(|t| *t == Tok::FlowMappingEnd) {
                let key = self.parse_node(false, false)?;
                let m = self.peek().start;
                let value = self.empty_scalar(m);
                items.push((key, value));
            }
        }
    }
}

// ---- constructor.py (SafeConstructor) ----------------------------------------

struct Constructor {
    nodes: Vec<Node>,
    building: Vec<bool>,
}

impl Constructor {
    fn tag_is(&self, idx: usize, name: &str) -> bool {
        self.nodes[idx].tag.strip_prefix(TAG_PREFIX) == Some(name)
    }

    fn scalar(&self, idx: usize) -> R<String> {
        match &self.nodes[idx].kind {
            NodeKind::Scalar(v) => Ok(v.clone()),
            NodeKind::Seq(_) => Err(err(self.nodes[idx].mark, "expected a scalar node, but found sequence")),
            NodeKind::Map(_) => Err(err(self.nodes[idx].mark, "expected a scalar node, but found mapping")),
        }
    }

    /// `flatten_mapping`: merge keys replaced by what they merge, in front.
    fn flatten(&mut self, idx: usize) -> R<()> {
        let NodeKind::Map(items) = self.nodes[idx].kind.clone() else { return Ok(()) };
        let mark = self.nodes[idx].mark;
        let mut merge: Vec<(usize, usize)> = Vec::new();
        let mut rest = Vec::new();
        for (k, v) in items {
            if self.tag_is(k, "merge") {
                match self.nodes[v].kind.clone() {
                    NodeKind::Map(_) => {
                        self.flatten(v)?;
                        if let NodeKind::Map(sub) = &self.nodes[v].kind {
                            merge.extend(sub.iter().copied());
                        }
                    }
                    NodeKind::Seq(subs) => {
                        let mut submerge = Vec::new();
                        for s in subs {
                            if !matches!(self.nodes[s].kind, NodeKind::Map(_)) {
                                return Err(err(self.nodes[s].mark, format!(
                                    "while constructing a mapping (line {}): expected a mapping for merging, but found {}",
                                    mark.line + 1,
                                    kind_id(&self.nodes[s].kind)
                                )));
                            }
                            self.flatten(s)?;
                            if let NodeKind::Map(sub) = &self.nodes[s].kind {
                                submerge.push(sub.clone());
                            }
                        }
                        for sub in submerge.into_iter().rev() {
                            merge.extend(sub);
                        }
                    }
                    NodeKind::Scalar(_) => {
                        return Err(err(self.nodes[v].mark, format!(
                            "while constructing a mapping (line {}): expected a mapping or list of mappings for merging, but found scalar",
                            mark.line + 1
                        )));
                    }
                }
            } else {
                if self.tag_is(k, "value") {
                    self.nodes[k].tag = format!("{TAG_PREFIX}str");
                }
                rest.push((k, v));
            }
        }
        merge.extend(rest);
        self.nodes[idx].kind = NodeKind::Map(merge);
        Ok(())
    }

    fn construct(&mut self, idx: usize) -> R<Py> {
        if self.building[idx] {
            return Err(err(self.nodes[idx].mark, "a recursive alias (Python builds an object that holds itself)"));
        }
        self.building[idx] = true;
        let r = self.construct_inner(idx);
        self.building[idx] = false;
        r
    }

    fn construct_inner(&mut self, idx: usize) -> R<Py> {
        let mark = self.nodes[idx].mark;
        let tag = self.nodes[idx].tag.clone();
        let name = tag.strip_prefix(TAG_PREFIX);
        let bad = |what: &str| err(mark, what.to_string());
        match name {
            Some("null") => {
                self.scalar(idx)?;
                Ok(Py::None)
            }
            Some("bool") => {
                let v = self.scalar(idx)?;
                match v.to_lowercase().as_str() {
                    "yes" | "true" | "on" => Ok(Py::Bool(true)),
                    "no" | "false" | "off" => Ok(Py::Bool(false)),
                    _ => Err(bad(&format!("not a YAML bool: {v:?} (Python: KeyError)"))),
                }
            }
            Some("int") => construct_int(&self.scalar(idx)?).map(Py::Int).map_err(|m| err(mark, m)),
            Some("float") => construct_float(&self.scalar(idx)?).map(Py::Float).map_err(|m| err(mark, m)),
            Some("timestamp") => construct_timestamp(&self.scalar(idx)?).map_err(|m| err(mark, m)),
            Some("str") => Ok(Py::Str(self.scalar(idx)?)),
            Some("seq") => {
                let NodeKind::Seq(items) = self.nodes[idx].kind.clone() else {
                    return Err(bad(&format!("expected a sequence node, but found {}", kind_id(&self.nodes[idx].kind))));
                };
                let mut out = Vec::with_capacity(items.len());
                for i in items {
                    out.push(self.construct(i)?);
                }
                Ok(Py::List(out))
            }
            Some("map") => {
                if !matches!(self.nodes[idx].kind, NodeKind::Map(_)) {
                    return Err(bad(&format!("expected a mapping node, but found {}", kind_id(&self.nodes[idx].kind))));
                }
                self.flatten(idx)?;
                let NodeKind::Map(items) = self.nodes[idx].kind.clone() else { unreachable!() };
                let mut out: Vec<(String, Py)> = Vec::new();
                for (k, v) in items {
                    let key = match self.construct(k)? {
                        Py::Str(s) => s,
                        other => {
                            return Err(err(self.nodes[k].mark, format!(
                                "a mapping key that is not a string (Python: a {} key, which mower_pyfmt's dicts cannot hold)",
                                other.type_name()
                            )))
                        }
                    };
                    let value = self.construct(v)?;
                    match out.iter_mut().find(|(k2, _)| *k2 == key) {
                        Some(slot) => slot.1 = value,
                        None => out.push((key, value)),
                    }
                }
                Ok(Py::Dict(out))
            }
            Some("binary") | Some("set") | Some("omap") | Some("pairs") => {
                Err(bad(&format!("!!{} is not supported (Python builds a value mower_pyfmt has no type for)", name.unwrap())))
            }
            _ => Err(bad(&format!("could not determine a constructor for the tag {tag:?}"))),
        }
    }
}

fn kind_id(k: &NodeKind) -> &'static str {
    match k {
        NodeKind::Scalar(_) => "scalar",
        NodeKind::Seq(_) => "sequence",
        NodeKind::Map(_) => "mapping",
    }
}

/// Python's `int(text, base)` for the ASCII forms: surrounding whitespace,
/// a sign, the base's own `0b`/`0o`/`0x` prefix, digits.
fn py_int(text: &str, base: u32) -> Result<i128, String> {
    let bad = || format!("invalid literal for int() with base {base}: {text:?}");
    let t = text.trim_matches(|c: char| c.is_whitespace());
    let (neg, t) = match t.strip_prefix('-') {
        Some(r) => (true, r),
        None => (false, t.strip_prefix('+').unwrap_or(t)),
    };
    let t = match base {
        2 => t.strip_prefix("0b").or_else(|| t.strip_prefix("0B")).unwrap_or(t),
        8 => t.strip_prefix("0o").or_else(|| t.strip_prefix("0O")).unwrap_or(t),
        16 => t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")).unwrap_or(t),
        _ => t,
    };
    if t.is_empty() || !t.chars().all(|c| c.is_digit(base)) {
        return Err(bad());
    }
    let mut v: i128 = 0;
    for c in t.chars() {
        v = v.checked_mul(base as i128).and_then(|v| v.checked_add(c.to_digit(base).unwrap() as i128)).ok_or_else(too_big)?;
    }
    Ok(if neg { -v } else { v })
}

fn too_big() -> String {
    "an int outside i64 (Python's ints have no limit)".to_string()
}

/// `construct_yaml_int`
fn construct_int(text: &str) -> Result<i64, String> {
    let value: String = text.replace('_', "");
    let first = value.chars().next().ok_or("an empty int (Python: IndexError)")?;
    let sign: i128 = if first == '-' { -1 } else { 1 };
    let value = if first == '+' || first == '-' { &value[1..] } else { &value[..] };
    let v: i128 = if value == "0" {
        0
    } else if let Some(r) = value.strip_prefix("0b") {
        sign * py_int(r, 2)?
    } else if let Some(r) = value.strip_prefix("0x") {
        sign * py_int(r, 16)?
    } else if value.starts_with('0') {
        sign * py_int(value, 8)?
    } else if value.is_empty() {
        return Err("an int with only a sign (Python: IndexError)".into());
    } else if value.contains(':') {
        let mut digits = Vec::new();
        for part in value.split(':') {
            digits.push(py_int(part, 10)?);
        }
        let (mut base, mut v): (i128, i128) = (1, 0);
        for d in digits.into_iter().rev() {
            v = v.checked_add(d.checked_mul(base).ok_or_else(too_big)?).ok_or_else(too_big)?;
            base = base.checked_mul(60).ok_or_else(too_big)?;
        }
        sign * v
    } else {
        sign * py_int(value, 10)?
    };
    i64::try_from(v).map_err(|_| too_big())
}

/// Python's `float(text)` for what a YAML float can hold.
fn py_float(text: &str) -> Result<f64, String> {
    let t = text.trim_matches(|c: char| c.is_whitespace());
    t.parse::<f64>().map_err(|_| format!("could not convert string to float: {text:?}"))
}

/// `construct_yaml_float`
fn construct_float(text: &str) -> Result<f64, String> {
    let value = text.replace('_', "").to_lowercase();
    let first = value.chars().next().ok_or("an empty float (Python: IndexError)")?;
    let sign = if first == '-' { -1.0 } else { 1.0 };
    let value = if first == '+' || first == '-' { &value[1..] } else { &value[..] };
    if value == ".inf" {
        Ok(sign * f64::INFINITY)
    } else if value == ".nan" {
        // nan_value = -inf / inf: the default NaN of the robot's arm64
        Ok(f64::NAN)
    } else if value.contains(':') {
        let mut digits = Vec::new();
        for part in value.split(':') {
            digits.push(py_float(part)?);
        }
        let (mut base, mut v) = (1.0f64, 0.0f64);
        for d in digits.into_iter().rev() {
            v += d * base;
            base *= 60.0;
        }
        Ok(sign * v)
    } else {
        Ok(sign * py_float(value)?)
    }
}

fn days_in_month(year: i32, month: u32) -> u32 {
    match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        _ if (year % 4 == 0 && year % 100 != 0) || year % 400 == 0 => 29,
        _ => 28,
    }
}

/// `construct_yaml_timestamp`: its regexp, then `datetime.date` or
/// `datetime.datetime` with their range checks.
fn construct_timestamp(text: &str) -> Result<Py, String> {
    let fail = || format!("not a timestamp: {text:?} (Python: the regexp does not match)");
    // `$` also matches before a final newline
    let t: Vec<char> = text.strip_suffix('\n').unwrap_or(text).chars().collect();
    let n = t.len();
    let dig = |i: usize| t.get(i).is_some_and(|c| c.is_ascii_digit());
    let num = |a: usize, b: usize| -> u32 { t[a..b].iter().collect::<String>().parse().unwrap() };
    // 1-2 digits from i
    let d12 = |i: usize| -> Option<usize> {
        if !dig(i) {
            None
        } else if dig(i + 1) {
            Some(i + 2)
        } else {
            Some(i + 1)
        }
    };
    if !(dig(0) && dig(1) && dig(2) && dig(3) && t.get(4) == Some(&'-')) {
        return Err(fail());
    }
    let year = num(0, 4) as i32;
    let me = d12(5).ok_or_else(fail)?;
    if t.get(me) != Some(&'-') {
        return Err(fail());
    }
    let month = num(5, me);
    let de = d12(me + 1).ok_or_else(fail)?;
    let day = num(me + 1, de);
    let check_date = || -> Result<(), String> {
        if !(1..=9999).contains(&year) {
            return Err(format!("year {year} is out of range"));
        }
        if !(1..=12).contains(&month) {
            return Err("month must be in 1..12".into());
        }
        if day < 1 || day > days_in_month(year, month) {
            return Err("day is out of range for month".into());
        }
        Ok(())
    };
    if de == n {
        check_date()?;
        return Ok(Py::Date { year, month, day });
    }
    // the time part
    let mut i = de;
    if matches!(t.get(i), Some('T' | 't')) {
        i += 1;
    } else if matches!(t.get(i), Some(' ' | '\t')) {
        while matches!(t.get(i), Some(' ' | '\t')) {
            i += 1;
        }
    } else {
        return Err(fail());
    }
    let he = d12(i).ok_or_else(fail)?;
    let hour = num(i, he);
    if !(t.get(he) == Some(&':') && dig(he + 1) && dig(he + 2) && t.get(he + 3) == Some(&':') && dig(he + 4) && dig(he + 5)) {
        return Err(fail());
    }
    let minute = num(he + 1, he + 3);
    let second = num(he + 4, he + 6);
    i = he + 6;
    let mut microsecond = 0;
    if t.get(i) == Some(&'.') {
        let fs = i + 1;
        i = fs;
        while dig(i) {
            i += 1;
        }
        let mut frac: String = t[fs..i].iter().take(6).collect();
        if !frac.is_empty() {
            while frac.len() < 6 {
                frac.push('0');
            }
            microsecond = frac.parse().unwrap();
        }
    }
    let mut offset = None;
    if i < n {
        let mut j = i;
        while matches!(t.get(j), Some(' ' | '\t')) {
            j += 1;
        }
        match t.get(j) {
            Some('Z') if j + 1 == n => offset = Some(0),
            Some(&s @ ('-' | '+')) => {
                let hend = d12(j + 1).ok_or_else(fail)?;
                let th = num(j + 1, hend) as i32;
                let mut tm = 0;
                let mut end = hend;
                if t.get(hend) == Some(&':') && dig(hend + 1) && dig(hend + 2) {
                    tm = num(hend + 1, hend + 3) as i32;
                    end = hend + 3;
                }
                if end != n {
                    return Err(fail());
                }
                let delta = th * 60 + tm;
                offset = Some(if s == '-' { -delta } else { delta });
            }
            _ => return Err(fail()),
        }
    }
    check_date()?;
    if hour > 23 {
        return Err("hour must be in 0..23".into());
    }
    if minute > 59 {
        return Err("minute must be in 0..59".into());
    }
    if second > 59 {
        return Err("second must be in 0..59".into());
    }
    if let Some(o) = offset {
        if o.abs() >= 24 * 60 {
            return Err("offset must be a timedelta strictly between -timedelta(hours=24) and timedelta(hours=24)".into());
        }
    }
    Ok(Py::DateTime { year, month, day, hour, minute, second, microsecond, offset_minutes: offset })
}

/// `yaml.safe_load(text)`: one document (none at all is `None`).
pub fn safe_load(text: &str) -> Result<Py, YamlError> {
    let toks = Scanner::new(text)?.run()?;
    let mut p = Parser { toks, i: 0, handles: default_handles(), nodes: Vec::new(), anchors: HashMap::new() };
    let root = p.single_document()?;
    let Some(root) = root else { return Ok(Py::None) };
    let n = p.nodes.len();
    let mut c = Constructor { nodes: p.nodes, building: vec![false; n] };
    c.construct(root)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ok(s: &str) -> Py {
        safe_load(s).unwrap_or_else(|e| panic!("{s:?}: {e}"))
    }

    #[test]
    fn styles() {
        assert_eq!(ok("a: 1\nb: [x, 'y', \"z\\n\"]\n"), Py::dict([
            ("a", Py::Int(1)),
            ("b", Py::List(vec![Py::str("x"), Py::str("y"), Py::str("z\n")])),
        ]));
        assert_eq!(ok("- |\n  a\n  b\n- >-\n  c\n  d\n"), Py::List(vec![Py::str("a\nb\n"), Py::str("c d")]));
        assert_eq!(ok(""), Py::None);
        assert_eq!(ok("off"), Py::Bool(false));
        assert_eq!(ok("012"), Py::Int(10));
        assert_eq!(ok("1:30"), Py::Int(90));
        assert_eq!(ok("2026-10-30"), Py::Date { year: 2026, month: 10, day: 30 });
        assert!(safe_load("a: 1\n--- b\n").is_err());
        assert!(safe_load("=").is_err());
        assert_eq!(ok("a: &x {k: 1}\nb:\n  <<: *x\n  j: 2\n"), Py::dict([
            ("a", Py::dict([("k", Py::Int(1))])),
            ("b", Py::dict([("k", Py::Int(1)), ("j", Py::Int(2))])),
        ]));
    }
}
