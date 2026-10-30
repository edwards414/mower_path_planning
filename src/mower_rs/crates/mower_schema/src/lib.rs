//! ROS 2 message definitions without ROS: what the recorder (E5) needs to
//! write rosbag2's MCAP schemas and channels, and what `mower_cdr` needs to
//! lay a message out in CDR.
//!
//! - **The table** ([`lookup`], [`names`]) holds every message type on the
//!   bus (mower_bridge's manifest) and in the record profiles, with the types
//!   they depend on: the `.msg` text exactly as the robot image installs it
//!   and the type's RIHS01 hash. `gen/gen_schema.py` regenerates it from the
//!   image (`msg/`, `src/table.rs`, `msg/PROVENANCE.txt`); nothing here needs
//!   ROS at build or run time.
//! - **rosbag2's definition** ([`rosbag2_definition`]) is the schema text
//!   rosbag2 Jazzy (0.26.11) writes into each MCAP Schema record: the root
//!   `.msg` file, then each dependency once, depth first, sorted by name per
//!   message, behind an 80 `=` delimiter (rosbag2_cpp
//!   local_message_definition_source.cpp, `get_full_text`). A type it does
//!   not know gets what rosbag2 then writes: an empty text with an empty
//!   encoding (SequentialWriter::create_topic swaps the source's `unknown`
//!   for `empty_message_definition_for`), as action types always get in
//!   Jazzy (E5-0 fixture F1, `actions/`).
//! - **Layouts** ([`message`]) are the fields of a type, parsed from its
//!   `.msg` text the way rosidl reads it: constants left out, array kinds and
//!   bounds kept, an empty message given rosidl's
//!   `uint8 structure_needs_at_least_one_member`.
//!
//! Type names are accepted as `pkg/msg/Type` (ROS 2's) or `pkg/Type`
//! (what `.msg` files and rosbag2's delimiters use).

use std::collections::{BTreeSet, HashMap, HashSet};
use std::sync::OnceLock;

mod msg;
mod table;

pub use msg::{parse_msg, ArrayKind, Field, FieldType, MsgDef, ParseError, Primitive};

/// One message type of the table.
#[derive(Debug)]
pub struct TypeInfo {
    /// `pkg/msg/Type`
    pub name: &'static str,
    /// The `.msg` file, byte for byte.
    pub text: &'static str,
    /// `RIHS01_<64 hex>`, from the package's `<Type>.json`.
    pub hash: &'static str,
}

fn index() -> &'static HashMap<&'static str, TypeInfo> {
    static INDEX: OnceLock<HashMap<&'static str, TypeInfo>> = OnceLock::new();
    INDEX.get_or_init(|| {
        table::TYPES
            .iter()
            .map(|&(name, text, hash)| (name, TypeInfo { name, text, hash }))
            .collect()
    })
}

/// `pkg/Type` or `pkg/msg/Type` → `pkg/msg/Type`. `None` for anything else
/// (an action or service type, a name without a package).
pub fn full_name(name: &str) -> Option<String> {
    let parts: Vec<&str> = name.split('/').collect();
    match parts.as_slice() {
        [pkg, ty] if valid_token(pkg) && valid_token(ty) => Some(format!("{pkg}/msg/{ty}")),
        [pkg, "msg", ty] if valid_token(pkg) && valid_token(ty) => Some(format!("{pkg}/msg/{ty}")),
        _ => None,
    }
}

fn valid_token(s: &str) -> bool {
    !s.is_empty() && s.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_')
}

/// The table's entry for a message type.
pub fn lookup(name: &str) -> Option<&'static TypeInfo> {
    index().get(full_name(name)?.as_str())
}

/// Every type in the table, sorted.
pub fn names() -> Vec<&'static str> {
    let mut v: Vec<&'static str> = table::TYPES.iter().map(|t| t.0).collect();
    v.sort_unstable();
    v
}

/// The types the table was generated for (the bus's and the profiles').
pub fn roots() -> &'static [&'static str] {
    table::ROOTS
}

/// The RIHS01 hash of a type in the table.
pub fn type_hash(name: &str) -> Option<&'static str> {
    lookup(name).map(|t| t.hash)
}

/// What rosbag2 stores for a type: the Schema record's `encoding` and `data`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Definition {
    /// `ros2msg`, or empty with an empty text.
    pub encoding: &'static str,
    pub text: String,
}

/// The `=` line rosbag2 puts before every dependency.
pub const DELIMITER: &str = "================================================================================\n";

/// rosbag2's schema for `name` (see the crate docs). The text is built from
/// the table only: a dependency missing from it makes the whole definition
/// unknown, as a missing file does in rosbag2.
pub fn rosbag2_definition(name: &str) -> Definition {
    let unknown = Definition { encoding: "", text: String::new() };
    // rosbag2 treats these as action types and finds nothing for them
    if name.contains("/action/") || name == "action_msgs/msg/GoalStatusArray" {
        return unknown;
    }
    let Some(root) = lookup(name) else { return unknown };
    let mut seen: HashSet<String> = HashSet::new();
    match append_recursive(root, &mut seen, 50) {
        Some(text) => Definition { encoding: "ros2msg", text },
        None => unknown,
    }
}

fn append_recursive(t: &TypeInfo, seen: &mut HashSet<String>, depth: u32) -> Option<String> {
    if depth == 0 {
        return None;
    }
    let pkg = t.name.split('/').next().unwrap_or("");
    let mut out = t.text.to_string();
    for dep in rosbag2_dependencies(t.text, pkg) {
        if seen.insert(dep.clone()) {
            let info = lookup(&dep)?;
            out.push('\n');
            out.push_str(DELIMITER);
            out.push_str("MSG: ");
            out.push_str(&dep);
            out.push('\n');
            out.push_str(&append_recursive(info, seen, depth - 1)?);
        }
    }
    Some(out)
}

const PRIMITIVE_NAMES: [&str; 14] = [
    "bool", "byte", "char", "float32", "float64", "int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64",
    "string",
];

/// The dependencies rosbag2 finds in a `.msg` text (`pkg/Type`, sorted):
/// every match of its `MSG_FIELD_TYPE_REGEX`,
/// `(?:^|\n)\s*([a-zA-Z0-9_/]+)(?:\[[^\]]*\])?\s+`, applied as
/// std::sregex_iterator applies it (`^` only at the start, matches one after
/// another), minus the primitive names; a bare name is in `pkg`.
pub fn rosbag2_dependencies(text: &str, pkg: &str) -> BTreeSet<String> {
    let b = text.as_bytes();
    let word = |c: u8| c.is_ascii_alphanumeric() || c == b'_' || c == b'/';
    let space = |c: u8| matches!(c, b' ' | b'\t' | b'\n' | b'\r' | 0x0b | 0x0c);
    // one attempt with the anchor already consumed: `\s*`, the word, the
    // optional `[...]`, then `\s+` (greedy; nothing to backtrack into: a
    // shorter word is followed by a word character, which neither `[` nor
    // `\s` accepts)
    let attempt = |mut j: usize| -> Option<(usize, usize, usize)> {
        while j < b.len() && space(b[j]) {
            j += 1;
        }
        let w0 = j;
        while j < b.len() && word(b[j]) {
            j += 1;
        }
        if j == w0 {
            return None;
        }
        let w1 = j;
        if j < b.len() && b[j] == b'[' {
            if let Some(k) = b[j + 1..].iter().position(|&c| c == b']') {
                j += k + 2;
            }
        }
        let s0 = j;
        while j < b.len() && space(b[j]) {
            j += 1;
        }
        (j > s0).then_some((w0, w1, j))
    };
    let mut deps = BTreeSet::new();
    let mut i = 0usize;
    while i <= b.len() {
        // `^` matches only at the very start; `\n` anywhere
        let mut m = if i == 0 { attempt(0) } else { None };
        if m.is_none() && i < b.len() && b[i] == b'\n' {
            m = attempt(i + 1);
        }
        match m {
            Some((w0, w1, end)) => {
                let ty = &text[w0..w1];
                if !PRIMITIVE_NAMES.contains(&ty) {
                    deps.insert(if ty.contains('/') { ty.to_string() } else { format!("{pkg}/{ty}") });
                }
                // the next search resumes after the match, where `^` cannot match
                i = end.max(i + 1);
            }
            None => i += 1,
        }
    }
    deps
}

/// The parsed layout of a type in the table (cached).
pub fn message(name: &str) -> Result<&'static MsgDef, ParseError> {
    static CACHE: OnceLock<HashMap<&'static str, Result<MsgDef, ParseError>>> = OnceLock::new();
    let cache = CACHE.get_or_init(|| {
        table::TYPES
            .iter()
            .map(|&(name, text, _)| (name, parse_msg(name, text)))
            .collect()
    });
    let full = full_name(name).ok_or_else(|| ParseError::new(name, 0, "not a message type name"))?;
    match cache.get(full.as_str()) {
        Some(Ok(def)) => Ok(def),
        Some(Err(e)) => Err(e.clone()),
        None => Err(ParseError::new(name, 0, "not in mower_schema's table")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names_are_normalised() {
        assert_eq!(full_name("std_msgs/String").as_deref(), Some("std_msgs/msg/String"));
        assert_eq!(full_name("std_msgs/msg/String").as_deref(), Some("std_msgs/msg/String"));
        assert_eq!(full_name("nav2_msgs/action/NavigateToPose_FeedbackMessage"), None);
        assert_eq!(full_name("String"), None);
        assert!(lookup("geometry_msgs/Pose").is_some());
    }

    #[test]
    fn rosbag2_s_regex_finds_the_first_word_of_each_line() {
        let text = "# comment Header\nstd_msgs/Header header\n  Pose[] poses # x\nfloat64 w 1\nuint8 A=1\nstring<=4 s\n";
        let deps: Vec<String> = rosbag2_dependencies(text, "geometry_msgs").into_iter().collect();
        assert_eq!(deps, ["geometry_msgs/Pose", "std_msgs/Header"]);
        // a lone word swallows the next line's '\n', so that line is not seen
        let deps: Vec<String> = rosbag2_dependencies("Foo\nBar b\nBaz c\n", "p").into_iter().collect();
        assert_eq!(deps, ["p/Baz", "p/Foo"]);
        // the start of the text counts, a type without trailing space does not
        let deps: Vec<String> = rosbag2_dependencies("Point p\nQuat", "g").into_iter().collect();
        assert_eq!(deps, ["g/Point"]);
    }

    #[test]
    fn every_dependency_of_every_type_is_in_the_table() {
        for name in names() {
            let info = lookup(name).unwrap();
            let pkg = name.split('/').next().unwrap();
            for dep in rosbag2_dependencies(info.text, pkg) {
                assert!(lookup(&dep).is_some(), "{name} needs {dep}");
            }
            assert_eq!(rosbag2_definition(name).encoding, "ros2msg", "{name}");
            assert!(info.hash.starts_with("RIHS01_") && info.hash.len() == 71, "{name}");
            message(name).unwrap_or_else(|e| panic!("{name}: {e}"));
        }
    }

    #[test]
    fn pose_stamped_lists_its_dependencies_as_rosbag2_does() {
        // E5 design 2.1, from the live yard2 bag: Pose, Point, Quaternion,
        // then Header, Time
        let d = rosbag2_definition("geometry_msgs/msg/PoseStamped");
        let order: Vec<&str> = d.text.lines().filter_map(|l| l.strip_prefix("MSG: ")).collect();
        assert_eq!(order, ["geometry_msgs/Pose", "geometry_msgs/Point", "geometry_msgs/Quaternion", "std_msgs/Header",
            "builtin_interfaces/Time"]);
        assert!(d.text.starts_with(lookup("geometry_msgs/PoseStamped").unwrap().text));
    }

    #[test]
    fn action_and_unknown_types_get_rosbag2_s_empty_definition() {
        for t in ["nav2_msgs/action/NavigateToPose_FeedbackMessage", "action_msgs/msg/GoalStatusArray", "x_msgs/msg/Y"] {
            assert_eq!(rosbag2_definition(t), Definition { encoding: "", text: String::new() }, "{t}");
        }
    }
}
