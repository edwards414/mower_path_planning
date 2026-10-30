//! PyYAML 6.0.1's `yaml.safe_dump` / `yaml.dump` for the values the recorder
//! writes, ported from `yaml/representer.py` (SafeRepresenter),
//! `yaml/serializer.py`, `yaml/resolver.py` (the YAML 1.1 implicit
//! resolvers) and `yaml/emitter.py` (the block-style emitter).
//!
//! Fixed: `default_flow_style=False` (the default since PyYAML 5.1), indent 2,
//! width 80, `\n` line breaks, no explicit document start or end, no
//! encoding (a `str`). Variable: `allow_unicode` and `sort_keys`.
//!
//! The emitter keeps PyYAML's state (`indent`, `column`, `whitespace`,
//! `indention`, `open_ended`, the contexts) and walks the value tree in the
//! order the event state machine does, so each writer below is a line by line
//! port of the Python method with the same name. Strings are handled as
//! sequences of code points, as Python's `str` is, so column counts agree.

use crate::{repr, Py};

/// `yaml.safe_load` (`load.rs`).
pub use crate::load::{safe_load, YamlError};

/// The `safe_dump` keywords the recorder varies.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct YamlOptions {
    /// `allow_unicode=True` writes non-ASCII printable characters as they are;
    /// otherwise they force double quotes and `\x`, `\u` or `\U` escapes.
    pub allow_unicode: bool,
    /// `sort_keys=True` (the default) sorts every mapping by key.
    pub sort_keys: bool,
}

impl Default for YamlOptions {
    /// `yaml.safe_dump(data)`: `allow_unicode=False`, `sort_keys=True`.
    fn default() -> Self {
        YamlOptions { allow_unicode: false, sort_keys: true }
    }
}

/// `yaml.safe_dump(value, **opts)` returned as a string.
pub fn safe_dump(value: &Py, opts: &YamlOptions) -> String {
    let root = represent(value, opts.sort_keys);
    let mut e = Emitter::new(opts.allow_unicode);
    e.emit_document(&root);
    e.out
}

/// `yaml.dump(value, default_flow_style=False, **opts)`. `Dumper` differs
/// from `SafeDumper` only in how it represents Python objects outside this
/// value model (tuples, sets, classes), so for a [`Py`] the text is the same
/// as [`safe_dump`]'s (the vector generator checks this on every case).
/// `ros2 param dump` prints this with one more `\n` (`print(yaml.dump(...))`).
pub fn dump(value: &Py, opts: &YamlOptions) -> String {
    safe_dump(value, opts)
}

// ---- resolver.py -----------------------------------------------------------

/// The tags a plain scalar can resolve to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Tag {
    Str,
    Bool,
    Int,
    Float,
    Null,
    Merge,
    Timestamp,
    Value,
    Yaml,
}

impl Tag {
    /// `prepare_tag` of `tag:yaml.org,2002:<name>`
    fn short(self) -> &'static str {
        match self {
            Tag::Str => "!!str",
            Tag::Bool => "!!bool",
            Tag::Int => "!!int",
            Tag::Float => "!!float",
            Tag::Null => "!!null",
            Tag::Merge => "!!merge",
            Tag::Timestamp => "!!timestamp",
            Tag::Value => "!!value",
            Tag::Yaml => "!!yaml",
        }
    }
}

/// `Resolver.resolve(ScalarNode, value, (True, False))`: the tag a plain
/// scalar with this text would load as. The resolvers are tried by first
/// character in the order they were added (bool, float, int, merge, null,
/// timestamp, value, yaml); every regexp is anchored, and each first-character
/// list covers all the strings its regexp matches, so "does any regexp match
/// the whole text" is the same test. (Python's `$` also matches before a final
/// `\n`; such a text has a line break and is never plain, so it cannot change
/// the output.)
pub(crate) fn resolve(value: &str) -> Tag {
    let s: Vec<char> = value.chars().collect();
    if is_bool(value) {
        Tag::Bool
    } else if is_float(&s) {
        Tag::Float
    } else if is_int(&s) {
        Tag::Int
    } else if value == "<<" {
        Tag::Merge
    } else if matches!(value, "~" | "null" | "Null" | "NULL" | "") {
        Tag::Null
    } else if is_timestamp(&s) {
        Tag::Timestamp
    } else if value == "=" {
        Tag::Value
    } else if matches!(value, "!" | "&" | "*") {
        Tag::Yaml
    } else {
        Tag::Str
    }
}

fn is_bool(v: &str) -> bool {
    matches!(
        v,
        "yes" | "Yes" | "YES" | "no" | "No" | "NO" | "true" | "True" | "TRUE" | "false" | "False"
            | "FALSE" | "on" | "On" | "ON" | "off" | "Off" | "OFF"
    )
}

fn digit(c: Option<&char>) -> bool {
    matches!(c, Some('0'..='9'))
}

/// `[0-9_]*` from `i`
fn digits_us(s: &[char], mut i: usize) -> usize {
    while matches!(s.get(i), Some('0'..='9' | '_')) {
        i += 1;
    }
    i
}

fn sign(s: &[char], i: usize) -> usize {
    if matches!(s.get(i), Some('-' | '+')) {
        i + 1
    } else {
        i
    }
}

/// `(?::[0-5]?[0-9])+` from `i`; the end, or None when not even one group
/// matches. Greedy is exact here: a second digit left behind is never `:`.
fn sexagesimal(s: &[char], mut i: usize) -> Option<usize> {
    let mut groups = 0;
    while s.get(i) == Some(&':') {
        let j = i + 1;
        if matches!(s.get(j), Some('0'..='5')) && digit(s.get(j + 1)) {
            i = j + 2;
        } else if digit(s.get(j)) {
            i = j + 1;
        } else {
            break;
        }
        groups += 1;
    }
    if groups > 0 {
        Some(i)
    } else {
        None
    }
}

/// `(?:[eE][-+][0-9]+)?` from `i`
fn exponent(s: &[char], i: usize) -> usize {
    if matches!(s.get(i), Some('e' | 'E')) && matches!(s.get(i + 1), Some('-' | '+')) && digit(s.get(i + 2)) {
        let mut j = i + 3;
        while digit(s.get(j)) {
            j += 1;
        }
        j
    } else {
        i
    }
}

/// The float resolver:
/// `[-+]?(?:[0-9][0-9_]*)\.[0-9_]*(?:[eE][-+][0-9]+)?`
/// `| \.[0-9][0-9_]*(?:[eE][-+][0-9]+)?`
/// `| [-+]?[0-9][0-9_]*(?::[0-5]?[0-9])+\.[0-9_]*`
/// `| [-+]?\.(?:inf|Inf|INF) | \.(?:nan|NaN|NAN)`
fn is_float(s: &[char]) -> bool {
    let n = s.len();
    let rest = |i: usize| -> String { s[i.min(n)..].iter().collect() };
    // 1
    let i = sign(s, 0);
    if digit(s.get(i)) {
        let j = digits_us(s, i + 1);
        if s.get(j) == Some(&'.') && exponent(s, digits_us(s, j + 1)) == n {
            return true;
        }
    }
    // 2
    if s.first() == Some(&'.') && digit(s.get(1)) && exponent(s, digits_us(s, 2)) == n {
        return true;
    }
    // 3
    if digit(s.get(i)) {
        let j = digits_us(s, i + 1);
        if let Some(k) = sexagesimal(s, j) {
            if s.get(k) == Some(&'.') && digits_us(s, k + 1) == n {
                return true;
            }
        }
    }
    // 4
    if s.get(i) == Some(&'.') && matches!(rest(i + 1).as_str(), "inf" | "Inf" | "INF") {
        return true;
    }
    // 5
    matches!(rest(0).as_str(), ".nan" | ".NaN" | ".NAN")
}

/// The int resolver:
/// `[-+]?0b[0-1_]+ | [-+]?0[0-7_]+ | [-+]?(?:0|[1-9][0-9_]*)`
/// `| [-+]?0x[0-9a-fA-F_]+ | [-+]?[1-9][0-9_]*(?::[0-5]?[0-9])+`
fn is_int(s: &[char]) -> bool {
    let n = s.len();
    let i = sign(s, 0);
    let all = |from: usize, ok: fn(char) -> bool| from < n && s[from..].iter().all(|c| ok(*c));
    if s.get(i) == Some(&'0') {
        if s.get(i + 1) == Some(&'b') && all(i + 2, |c| matches!(c, '0' | '1' | '_')) {
            return true;
        }
        if all(i + 1, |c| matches!(c, '0'..='7' | '_')) {
            return true;
        }
        if i + 1 == n {
            return true;
        }
        if s.get(i + 1) == Some(&'x') && all(i + 2, |c| c.is_ascii_hexdigit() || c == '_') {
            return true;
        }
        return false;
    }
    if matches!(s.get(i), Some('1'..='9')) {
        let j = digits_us(s, i + 1);
        if j == n {
            return true;
        }
        if sexagesimal(s, j) == Some(n) {
            return true;
        }
    }
    false
}

/// The timestamp resolver:
/// `[0-9][0-9][0-9][0-9]-[0-9][0-9]-[0-9][0-9]`
/// `| [0-9][0-9][0-9][0-9]-[0-9][0-9]?-[0-9][0-9]?(?:[Tt]|[ \t]+)[0-9][0-9]?`
/// `  :[0-9][0-9]:[0-9][0-9](?:\.[0-9]*)?(?:[ \t]*(?:Z|[-+][0-9][0-9]?(?::[0-9][0-9])?))?`
/// (re.X drops the blanks outside the character classes.)
fn is_timestamp(s: &[char]) -> bool {
    let n = s.len();
    let d = |i: usize| digit(s.get(i));
    if !(d(0) && d(1) && d(2) && d(3) && s.get(4) == Some(&'-')) {
        return false;
    }
    if n == 10 && d(5) && d(6) && s.get(7) == Some(&'-') && d(8) && d(9) {
        return true;
    }
    // \d\d? greedy: a second digit left behind is never the next separator
    let one_or_two = |i: usize| -> Option<usize> {
        if d(i) && d(i + 1) {
            Some(i + 2)
        } else if d(i) {
            Some(i + 1)
        } else {
            None
        }
    };
    let Some(i) = one_or_two(5) else { return false };
    if s.get(i) != Some(&'-') {
        return false;
    }
    let Some(mut i) = one_or_two(i + 1) else { return false };
    match s.get(i) {
        Some('T' | 't') => i += 1,
        Some(' ' | '\t') => {
            while matches!(s.get(i), Some(' ' | '\t')) {
                i += 1;
            }
        }
        _ => return false,
    }
    let Some(mut i) = one_or_two(i) else { return false };
    for _ in 0..2 {
        if !(s.get(i) == Some(&':') && d(i + 1) && d(i + 2)) {
            return false;
        }
        i += 3;
    }
    if s.get(i) == Some(&'.') {
        i += 1;
        while d(i) {
            i += 1;
        }
    }
    if i == n {
        return true;
    }
    while matches!(s.get(i), Some(' ' | '\t')) {
        i += 1;
    }
    match s.get(i) {
        Some('Z') => i + 1 == n,
        Some('-' | '+') => {
            let Some(mut j) = one_or_two(i + 1) else { return false };
            if s.get(j) == Some(&':') && d(j + 1) && d(j + 2) {
                j += 3;
            }
            j == n
        }
        _ => false,
    }
}

// ---- representer.py + serializer.py ----------------------------------------

/// A node with what the serializer puts in its event: for a scalar, its text,
/// its tag and `implicit = (tag == resolve(plain), tag == resolve(non-plain))`.
enum Node {
    Scalar { text: String, tag: Tag, implicit: (bool, bool) },
    Seq(Vec<Node>),
    Map(Vec<(Node, Node)>),
}

fn scalar(text: String, tag: Tag) -> Node {
    // resolve(..., (False, True)) is always the default scalar tag, str
    let implicit = (resolve(&text) == tag, tag == Tag::Str);
    Node::Scalar { text, tag, implicit }
}

/// `SafeRepresenter.represent_*` (`default_style=None`,
/// `default_flow_style=False`).
fn represent(v: &Py, sort_keys: bool) -> Node {
    match v {
        Py::None => scalar("null".to_string(), Tag::Null),
        Py::Bool(b) => scalar(if *b { "true" } else { "false" }.to_string(), Tag::Bool),
        Py::Int(i) => scalar(i.to_string(), Tag::Int),
        Py::Float(f) => scalar(represent_float(*f), Tag::Float),
        Py::Str(s) => scalar(s.clone(), Tag::Str),
        // represent_date: isoformat(); represent_datetime: isoformat(' ')
        Py::Date { .. } | Py::DateTime { .. } => scalar(v.isoformat().unwrap(), Tag::Timestamp),
        Py::List(items) => Node::Seq(items.iter().map(|x| represent(x, sort_keys)).collect()),
        Py::Dict(items) => {
            let mut refs: Vec<&(String, Py)> = items.iter().collect();
            if sort_keys {
                // sorted(mapping.items()): unique keys, so by key
                refs.sort_by(|a, b| a.0.cmp(&b.0));
            }
            Node::Map(
                refs.into_iter()
                    .map(|(k, x)| (scalar(k.clone(), Tag::Str), represent(x, sort_keys)))
                    .collect(),
            )
        }
    }
}

/// `represent_float`: `.nan`, `.inf`, `-.inf`, else `repr(data).lower()`
/// with `.0` put before an `e` that has no `.` in front of it (`1e+17` ->
/// `1.0e+17`).
fn represent_float(f: f64) -> String {
    if f.is_nan() {
        return ".nan".to_string();
    }
    if f == f64::INFINITY {
        return ".inf".to_string();
    }
    if f == f64::NEG_INFINITY {
        return "-.inf".to_string();
    }
    let v = repr::float(f).to_lowercase();
    if !v.contains('.') && v.contains('e') {
        v.replacen('e', ".0e", 1)
    } else {
        v
    }
}

// ---- emitter.py --------------------------------------------------------------

const BEST_INDENT: i64 = 2;
const BEST_WIDTH: i64 = 80;

fn is_break(c: char) -> bool {
    matches!(c, '\n' | '\u{85}' | '\u{2028}' | '\u{2029}')
}

fn is_blank_or_break(c: char) -> bool {
    matches!(c, '\0' | ' ' | '\t' | '\r' | '\n' | '\u{85}' | '\u{2028}' | '\u{2029}')
}

/// `ScalarAnalysis`
struct Analysis {
    empty: bool,
    multiline: bool,
    allow_flow_plain: bool,
    allow_block_plain: bool,
    allow_single_quoted: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Style {
    Plain,
    Single,
    Double,
}

struct Emitter {
    out: String,
    allow_unicode: bool,
    indents: Vec<Option<i64>>,
    indent: Option<i64>,
    flow_level: usize,
    root_context: bool,
    mapping_context: bool,
    simple_key_context: bool,
    column: i64,
    whitespace: bool,
    indention: bool,
    open_ended: bool,
}

impl Emitter {
    fn new(allow_unicode: bool) -> Self {
        Emitter {
            out: String::new(),
            allow_unicode,
            indents: Vec::new(),
            indent: None,
            flow_level: 0,
            root_context: false,
            mapping_context: false,
            simple_key_context: false,
            column: 0,
            whitespace: true,
            indention: true,
            open_ended: false,
        }
    }

    /// One document between StreamStart and StreamEnd, as `dump` emits it:
    /// `expect_document_start(first=True)` (implicit: no `---`, since the
    /// root event always carries a tag, so `check_empty_document` is false),
    /// the root node, `expect_document_end` (`write_indent`, no `...`), and at
    /// StreamEnd the `...` an open-ended document (a plain root scalar) needs.
    fn emit_document(&mut self, root: &Node) {
        self.expect_node(root, true, false, false);
        self.write_indent();
        if self.open_ended {
            self.write_indicator("...", true, false, false);
            self.write_indent();
        }
    }

    /// `expect_node`; the sequence context is set but read by nobody here.
    fn expect_node(&mut self, node: &Node, root: bool, _sequence: bool, mapping: bool) {
        self.expect_node_ctx(node, root, mapping, false)
    }

    fn expect_node_ctx(&mut self, node: &Node, root: bool, mapping: bool, simple_key: bool) {
        self.root_context = root;
        self.mapping_context = mapping;
        self.simple_key_context = simple_key;
        match node {
            Node::Scalar { text, tag, implicit } => {
                // process_tag, then expect_scalar
                let analysis = self.analyze_scalar(text);
                let style = self.choose_scalar_style(&analysis, implicit.0);
                let tagged = !((style == Style::Plain && implicit.0) || (style != Style::Plain && implicit.1));
                if tagged {
                    self.write_indicator(tag.short(), true, false, false);
                }
                self.increase_indent(true, false);
                let split = !self.simple_key_context;
                let chars: Vec<char> = text.chars().collect();
                match style {
                    Style::Double => self.write_double_quoted(&chars, split),
                    Style::Single => self.write_single_quoted(&chars, split),
                    Style::Plain => self.write_plain(&chars, split),
                }
                self.indent = self.indents.pop().flatten();
            }
            Node::Seq(items) => {
                if self.flow_level > 0 || items.is_empty() {
                    self.expect_flow_sequence(items);
                } else {
                    self.expect_block_sequence(items);
                }
            }
            Node::Map(items) => {
                if self.flow_level > 0 || items.is_empty() {
                    self.expect_flow_mapping(items);
                } else {
                    self.expect_block_mapping(items);
                }
            }
        }
    }

    fn increase_indent(&mut self, flow: bool, indentless: bool) {
        self.indents.push(self.indent);
        match self.indent {
            None => self.indent = Some(if flow { BEST_INDENT } else { 0 }),
            Some(i) if !indentless => self.indent = Some(i + BEST_INDENT),
            Some(_) => {}
        }
    }

    // Flow collections: here only the empty ones, `[]` and `{}`; the general
    // case is ported as well.

    fn expect_flow_sequence(&mut self, items: &[Node]) {
        self.write_indicator("[", true, true, false);
        self.flow_level += 1;
        self.increase_indent(true, false);
        for (i, item) in items.iter().enumerate() {
            if i > 0 {
                self.write_indicator(",", false, false, false);
            }
            if self.column > BEST_WIDTH {
                self.write_indent();
            }
            self.expect_node(item, false, true, false);
        }
        self.indent = self.indents.pop().flatten();
        self.flow_level -= 1;
        self.write_indicator("]", false, false, false);
    }

    fn expect_flow_mapping(&mut self, items: &[(Node, Node)]) {
        self.write_indicator("{", true, true, false);
        self.flow_level += 1;
        self.increase_indent(true, false);
        for (i, (k, v)) in items.iter().enumerate() {
            if i > 0 {
                self.write_indicator(",", false, false, false);
            }
            if self.column > BEST_WIDTH {
                self.write_indent();
            }
            if self.check_simple_key(k) {
                self.expect_node_ctx(k, false, true, true);
                self.write_indicator(":", false, false, false);
                self.expect_node(v, false, false, true);
            } else {
                self.write_indicator("?", true, false, false);
                self.expect_node(k, false, false, true);
                if self.column > BEST_WIDTH {
                    self.write_indent();
                }
                self.write_indicator(":", true, false, false);
                self.expect_node(v, false, false, true);
            }
        }
        self.indent = self.indents.pop().flatten();
        self.flow_level -= 1;
        self.write_indicator("}", false, false, false);
    }

    fn expect_block_sequence(&mut self, items: &[Node]) {
        let indentless = self.mapping_context && !self.indention;
        self.increase_indent(false, indentless);
        for item in items {
            self.write_indent();
            self.write_indicator("-", true, false, true);
            self.expect_node(item, false, true, false);
        }
        self.indent = self.indents.pop().flatten();
    }

    fn expect_block_mapping(&mut self, items: &[(Node, Node)]) {
        self.increase_indent(false, false);
        for (k, v) in items {
            self.write_indent();
            if self.check_simple_key(k) {
                self.expect_node_ctx(k, false, true, true);
                self.write_indicator(":", false, false, false);
                self.expect_node(v, false, false, true);
            } else {
                self.write_indicator("?", true, false, true);
                self.expect_node(k, false, false, true);
                self.write_indent();
                self.write_indicator(":", true, false, true);
                self.expect_node(v, false, false, true);
            }
        }
        self.indent = self.indents.pop().flatten();
    }

    /// `check_simple_key`: a non-empty single-line scalar, or an empty
    /// collection, whose length is under 128. The length counts the prepared
    /// tag as well, which every event here has (`!!str` for a string key):
    /// a string key is simple up to 122 characters.
    fn check_simple_key(&self, node: &Node) -> bool {
        const TAG_LEN: i64 = 5; // "!!str", "!!seq", "!!map", "!!int", ...
        match node {
            Node::Scalar { text, tag, .. } => {
                let a = self.analyze_scalar(text);
                let length = tag.short().chars().count() as i64 + text.chars().count() as i64;
                length < 128 && !a.empty && !a.multiline
            }
            Node::Seq(items) => TAG_LEN < 128 && items.is_empty(),
            Node::Map(items) => TAG_LEN < 128 && items.is_empty(),
        }
    }

    /// `choose_scalar_style` for an event with no requested style.
    fn choose_scalar_style(&self, a: &Analysis, implicit_plain: bool) -> Style {
        if implicit_plain
            && !(self.simple_key_context && (a.empty || a.multiline))
            && ((self.flow_level > 0 && a.allow_flow_plain) || (self.flow_level == 0 && a.allow_block_plain))
        {
            return Style::Plain;
        }
        if a.allow_single_quoted && !(self.simple_key_context && a.multiline) {
            return Style::Single;
        }
        Style::Double
    }

    /// `analyze_scalar`
    fn analyze_scalar(&self, scalar: &str) -> Analysis {
        let s: Vec<char> = scalar.chars().collect();
        if s.is_empty() {
            return Analysis {
                empty: true,
                multiline: false,
                allow_flow_plain: false,
                allow_block_plain: true,
                allow_single_quoted: true,
            };
        }
        let mut block_indicators = false;
        let mut flow_indicators = false;
        let mut line_breaks = false;
        let mut special_characters = false;
        let mut leading_space = false;
        let mut leading_break = false;
        let mut trailing_space = false;
        let mut trailing_break = false;
        let mut break_space = false;
        let mut space_break = false;
        if scalar.starts_with("---") || scalar.starts_with("...") {
            block_indicators = true;
            flow_indicators = true;
        }
        let mut preceded_by_whitespace = true;
        let mut followed_by_whitespace = s.len() == 1 || is_blank_or_break(s[1]);
        let mut previous_space = false;
        let mut previous_break = false;
        let n = s.len();
        let mut index = 0;
        while index < n {
            let ch = s[index];
            if index == 0 {
                if "#,[]{}&*!|>'\"%@`".contains(ch) {
                    flow_indicators = true;
                    block_indicators = true;
                }
                if ch == '?' || ch == ':' {
                    flow_indicators = true;
                    if followed_by_whitespace {
                        block_indicators = true;
                    }
                }
                if ch == '-' && followed_by_whitespace {
                    flow_indicators = true;
                    block_indicators = true;
                }
            } else {
                if ",?[]{}".contains(ch) {
                    flow_indicators = true;
                }
                if ch == ':' {
                    flow_indicators = true;
                    if followed_by_whitespace {
                        block_indicators = true;
                    }
                }
                if ch == '#' && preceded_by_whitespace {
                    flow_indicators = true;
                    block_indicators = true;
                }
            }
            if is_break(ch) {
                line_breaks = true;
            }
            if !(ch == '\n' || ('\x20'..='\x7E').contains(&ch)) {
                let unicode = (ch == '\u{85}'
                    || ('\u{A0}'..='\u{D7FF}').contains(&ch)
                    || ('\u{E000}'..='\u{FFFD}').contains(&ch)
                    || ('\u{10000}'..'\u{10FFFF}').contains(&ch))
                    && ch != '\u{FEFF}';
                if unicode {
                    if !self.allow_unicode {
                        special_characters = true;
                    }
                } else {
                    special_characters = true;
                }
            }
            if ch == ' ' {
                if index == 0 {
                    leading_space = true;
                }
                if index == n - 1 {
                    trailing_space = true;
                }
                if previous_break {
                    break_space = true;
                }
                previous_space = true;
                previous_break = false;
            } else if is_break(ch) {
                if index == 0 {
                    leading_break = true;
                }
                if index == n - 1 {
                    trailing_break = true;
                }
                if previous_space {
                    space_break = true;
                }
                previous_space = false;
                previous_break = true;
            } else {
                previous_space = false;
                previous_break = false;
            }
            index += 1;
            preceded_by_whitespace = is_blank_or_break(ch);
            followed_by_whitespace = index + 1 >= n || is_blank_or_break(s[index + 1]);
        }
        let mut allow_flow_plain = true;
        let mut allow_block_plain = true;
        let mut allow_single_quoted = true;
        if leading_space || leading_break || trailing_space || trailing_break {
            allow_flow_plain = false;
            allow_block_plain = false;
        }
        if break_space {
            allow_flow_plain = false;
            allow_block_plain = false;
            allow_single_quoted = false;
        }
        if space_break || special_characters {
            allow_flow_plain = false;
            allow_block_plain = false;
            allow_single_quoted = false;
        }
        if line_breaks {
            allow_flow_plain = false;
            allow_block_plain = false;
        }
        if flow_indicators {
            allow_flow_plain = false;
        }
        if block_indicators {
            allow_block_plain = false;
        }
        Analysis { empty: false, multiline: line_breaks, allow_flow_plain, allow_block_plain, allow_single_quoted }
    }

    // ---- writers ----

    fn write(&mut self, data: &[char]) {
        self.column += data.len() as i64;
        self.out.extend(data.iter());
    }

    fn write_indicator(&mut self, indicator: &str, need_whitespace: bool, whitespace: bool, indention: bool) {
        if !(self.whitespace || !need_whitespace) {
            self.out.push(' ');
            self.column += 1;
        }
        self.out.push_str(indicator);
        self.column += indicator.chars().count() as i64;
        self.whitespace = whitespace;
        self.indention = self.indention && indention;
        self.open_ended = false;
    }

    fn write_indent(&mut self) {
        let indent = self.indent.unwrap_or(0);
        if !self.indention || self.column > indent || (self.column == indent && !self.whitespace) {
            self.write_line_break(None);
        }
        if self.column < indent {
            self.whitespace = true;
            for _ in self.column..indent {
                self.out.push(' ');
            }
            self.column = indent;
        }
    }

    fn write_line_break(&mut self, data: Option<char>) {
        self.whitespace = true;
        self.indention = true;
        self.column = 0;
        self.out.push(data.unwrap_or('\n'));
    }

    /// `write_plain`
    fn write_plain(&mut self, text: &[char], split: bool) {
        if self.root_context {
            self.open_ended = true;
        }
        if text.is_empty() {
            return;
        }
        if !self.whitespace {
            self.write(&[' ']);
        }
        self.whitespace = false;
        self.indention = false;
        let mut spaces = false;
        let mut breaks = false;
        let (mut start, mut end) = (0usize, 0usize);
        while end <= text.len() {
            let ch = text.get(end).copied();
            if spaces {
                if ch != Some(' ') {
                    if start + 1 == end && self.column > BEST_WIDTH && split {
                        self.write_indent();
                        self.whitespace = false;
                        self.indention = false;
                    } else {
                        self.write(&text[start..end]);
                    }
                    start = end;
                }
            } else if breaks {
                if !ch.is_some_and(is_break) {
                    if text[start] == '\n' {
                        self.write_line_break(None);
                    }
                    for &br in &text[start..end] {
                        self.write_line_break(Some(br));
                    }
                    self.write_indent();
                    self.whitespace = false;
                    self.indention = false;
                    start = end;
                }
            } else if ch.is_none_or(|c| c == ' ' || is_break(c)) {
                self.write(&text[start..end]);
                start = end;
            }
            if let Some(c) = ch {
                spaces = c == ' ';
                breaks = is_break(c);
            }
            end += 1;
        }
    }

    /// `write_single_quoted`
    fn write_single_quoted(&mut self, text: &[char], split: bool) {
        self.write_indicator("'", true, false, false);
        let mut spaces = false;
        let mut breaks = false;
        let (mut start, mut end) = (0usize, 0usize);
        while end <= text.len() {
            let ch = text.get(end).copied();
            if spaces {
                if ch != Some(' ') {
                    if start + 1 == end && self.column > BEST_WIDTH && split && start != 0 && end != text.len() {
                        self.write_indent();
                    } else {
                        self.write(&text[start..end]);
                    }
                    start = end;
                }
            } else if breaks {
                if !ch.is_some_and(is_break) {
                    if text[start] == '\n' {
                        self.write_line_break(None);
                    }
                    for &br in &text[start..end] {
                        self.write_line_break(Some(br));
                    }
                    self.write_indent();
                    start = end;
                }
            } else if ch.is_none_or(|c| c == ' ' || is_break(c) || c == '\'') && start < end {
                self.write(&text[start..end]);
                start = end;
            }
            if ch == Some('\'') {
                self.write(&['\'', '\'']);
                start = end + 1;
            }
            if let Some(c) = ch {
                spaces = c == ' ';
                breaks = is_break(c);
            }
            end += 1;
        }
        self.write_indicator("'", false, false, false);
    }

    /// `write_double_quoted`, with `ESCAPE_REPLACEMENTS` and the `\x`, `\u`,
    /// `\U` escapes (upper-case hex). Astral characters are escaped even
    /// with `allow_unicode`: its printable test stops at U+FFFD.
    fn write_double_quoted(&mut self, text: &[char], split: bool) {
        self.write_indicator("\"", true, false, false);
        let n = text.len() as i64;
        let (mut start, mut end) = (0i64, 0i64);
        // text[a:b] with Python's clamping
        let slice = |a: i64, b: i64| -> &[char] {
            let a = a.clamp(0, n) as usize;
            let b = b.clamp(0, n) as usize;
            if a < b {
                &text[a..b]
            } else {
                &[]
            }
        };
        while end <= n {
            let ch = if end < n { Some(text[end as usize]) } else { None };
            let plain_ok = |c: char| {
                ('\x20'..='\x7E').contains(&c)
                    || (self.allow_unicode
                        && (('\u{A0}'..='\u{D7FF}').contains(&c) || ('\u{E000}'..='\u{FFFD}').contains(&c)))
            };
            if ch.is_none_or(|c| "\"\\\u{85}\u{2028}\u{2029}\u{FEFF}".contains(c) || !plain_ok(c)) {
                if start < end {
                    let d: Vec<char> = slice(start, end).to_vec();
                    self.write(&d);
                    start = end;
                }
                if let Some(c) = ch {
                    let data = match escape(c) {
                        Some(r) => format!("\\{r}"),
                        None if (c as u32) <= 0xFF => format!("\\x{:02X}", c as u32),
                        None if (c as u32) <= 0xFFFF => format!("\\u{:04X}", c as u32),
                        None => format!("\\U{:08X}", c as u32),
                    };
                    let d: Vec<char> = data.chars().collect();
                    self.write(&d);
                    start = end + 1;
                }
            }
            if 0 < end && end < n - 1 && (ch == Some(' ') || start >= end) && self.column + (end - start) > BEST_WIDTH && split {
                let mut d: Vec<char> = slice(start, end).to_vec();
                d.push('\\');
                if start < end {
                    start = end;
                }
                self.write(&d);
                self.write_indent();
                self.whitespace = false;
                self.indention = false;
                if text[start as usize] == ' ' {
                    self.write(&['\\']);
                }
            }
            end += 1;
        }
        self.write_indicator("\"", false, false, false);
    }
}

/// `Emitter.ESCAPE_REPLACEMENTS`
fn escape(c: char) -> Option<char> {
    Some(match c {
        '\0' => '0',
        '\x07' => 'a',
        '\x08' => 'b',
        '\x09' => 't',
        '\x0A' => 'n',
        '\x0B' => 'v',
        '\x0C' => 'f',
        '\x0D' => 'r',
        '\x1B' => 'e',
        '"' => '"',
        '\\' => '\\',
        '\u{85}' => 'N',
        '\u{A0}' => '_',
        '\u{2028}' => 'L',
        '\u{2029}' => 'P',
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn d(v: &Py) -> String {
        safe_dump(v, &YamlOptions::default())
    }

    #[test]
    fn basics() {
        assert_eq!(d(&Py::str("abc")), "abc\n...\n");
        assert_eq!(d(&Py::Int(5)), "5\n...\n");
        assert_eq!(d(&Py::str("")), "''\n");
        assert_eq!(d(&Py::str("off")), "'off'\n");
        assert_eq!(d(&Py::List(vec![])), "[]\n");
        assert_eq!(d(&Py::Float(1e17)), "1.0e+17\n...\n");
        let v = Py::dict([
            ("b", Py::List(vec![Py::Int(1), Py::dict([("x", Py::None), ("y", Py::Dict(vec![]))])])),
            ("a", Py::str("2026-10-27T10:15:00.123456")),
        ]);
        assert_eq!(d(&v), "a: '2026-10-27T10:15:00.123456'\nb:\n- 1\n- x: null\n  y: {}\n");
    }

    #[test]
    fn resolvers() {
        for s in ["1_000", "0x1F", "0o17", "017", "0b101", "12:30:00", "-0", "+5"] {
            assert!(matches!(resolve(s), Tag::Int | Tag::Str), "{s}");
        }
        assert_eq!(resolve("12:30:00"), Tag::Int);
        assert_eq!(resolve("190:20:30.15"), Tag::Float);
        assert_eq!(resolve("1e3"), Tag::Str);
        assert_eq!(resolve("1.0e+3"), Tag::Float);
        assert_eq!(resolve("0o17"), Tag::Str);
        assert_eq!(resolve("2026-10-27"), Tag::Timestamp);
        assert_eq!(resolve("2026-1-2 3:04:05.6 +8"), Tag::Timestamp);
        assert_eq!(resolve(".inf"), Tag::Float);
        assert_eq!(resolve("-.Inf"), Tag::Float);
        assert_eq!(resolve("."), Tag::Str);
        assert_eq!(resolve("+."), Tag::Str);
    }
}
