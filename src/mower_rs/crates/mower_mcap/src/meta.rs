//! rosbag2's bag metadata (rosbag2_storage `BagMetadata`, version 9) and the
//! YAML it is written as: the two `rosbag2` metadata records of every MCAP
//! file (`YAML::Dump` of the node), `metadata.yaml` (the same node under
//! `rosbag2_bagfile_information:`) and each channel's `offered_qos_profiles`
//! (`YAML::Dump` of the QoS list). The emitter is yaml-cpp 0.8's block style
//! for exactly the nodes rosbag2 builds (rosbag2_storage yaml.hpp, qos.cpp):
//! a nested map indented by 2, a sequence indented by 2 under its key with
//! a map item's first key on the dash's line, an empty sequence as `[]` on
//! its own line, an empty map as `~`, strings plain unless yaml-cpp's
//! `IsValidPlainScalar` refuses them (then double quoted), no final newline.

use std::fmt::Write as _;

/// One QoS profile as rosbag2 stores it (`convert<rclcpp::QoS>::encode`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Qos {
    /// `keep_last`, `keep_all`, `system_default`, `unknown`
    pub history: String,
    pub depth: u64,
    /// `reliable`, `best_effort`, `system_default`, `best_available`, `unknown`
    pub reliability: String,
    /// `volatile`, `transient_local`, `system_default`, `best_available`, `unknown`
    pub durability: String,
    /// (sec, nsec)
    pub deadline: (u64, u64),
    pub lifespan: (u64, u64),
    /// `automatic`, `manual_by_topic`, `system_default`, `best_available`, `unknown`
    pub liveliness: String,
    pub liveliness_lease_duration: (u64, u64),
    pub avoid_ros_namespace_conventions: bool,
}

/// `RMW_DURATION_INFINITE` as rosbag2 writes it.
pub const INFINITE: (u64, u64) = (9_223_372_036, 854_775_807);

impl Qos {
    /// A writer's profile with the defaults rmw reports for what it does not
    /// set: infinite deadline, lifespan and lease, automatic liveliness.
    pub fn new(history: &str, depth: u64, reliability: &str, durability: &str) -> Qos {
        Qos {
            history: history.into(),
            depth,
            reliability: reliability.into(),
            durability: durability.into(),
            deadline: INFINITE,
            lifespan: INFINITE,
            liveliness: "automatic".into(),
            liveliness_lease_duration: INFINITE,
            avoid_ros_namespace_conventions: false,
        }
    }

    fn node(&self) -> Node {
        let t = |(s, n): (u64, u64)| Node::Map(vec![("sec".into(), Node::int(s)), ("nsec".into(), Node::int(n))]);
        Node::Map(vec![
            ("history".into(), Node::str(&self.history)),
            ("depth".into(), Node::int(self.depth)),
            ("reliability".into(), Node::str(&self.reliability)),
            ("durability".into(), Node::str(&self.durability)),
            ("deadline".into(), t(self.deadline)),
            ("lifespan".into(), t(self.lifespan)),
            ("liveliness".into(), Node::str(&self.liveliness)),
            ("liveliness_lease_duration".into(), t(self.liveliness_lease_duration)),
            ("avoid_ros_namespace_conventions".into(), Node::Scalar(self.avoid_ros_namespace_conventions.to_string())),
        ])
    }

    /// The channel metadata `offered_qos_profiles`: `YAML::Dump` of the list
    /// (`[]` when empty).
    pub fn dump_list(profiles: &[Qos]) -> String {
        dump(&Node::Seq(profiles.iter().map(Qos::node).collect()))
    }

    /// Reads back what [`Qos::dump_list`] (or rosbag2) wrote.
    pub fn parse_list(text: &str) -> Result<Vec<Qos>, String> {
        if text.trim() == "[]" || text.trim().is_empty() {
            return Ok(Vec::new());
        }
        let mut out = Vec::new();
        let mut cur: Option<Vec<(String, String)>> = None;
        let mut parent = String::new();
        for line in text.lines() {
            let (item, rest) = match line.strip_prefix("- ") {
                Some(r) => (true, r),
                None => (false, line.strip_prefix("  ").ok_or(format!("bad line {line:?}"))?),
            };
            if item {
                if let Some(c) = cur.take() {
                    out.push(qos_from(c)?);
                }
                cur = Some(Vec::new());
            }
            let c = cur.as_mut().ok_or("a value before the first item")?;
            let nested = rest.starts_with("  ");
            let (k, v) = rest.trim().split_once(':').ok_or(format!("bad line {line:?}"))?;
            let v = v.trim();
            if nested {
                c.push((format!("{parent}.{k}"), v.to_string()));
            } else if v.is_empty() {
                parent = k.to_string();
            } else {
                c.push((k.to_string(), v.to_string()));
            }
        }
        if let Some(c) = cur {
            out.push(qos_from(c)?);
        }
        Ok(out)
    }
}

fn qos_from(kv: Vec<(String, String)>) -> Result<Qos, String> {
    let get = |k: &str| kv.iter().find(|(a, _)| a == k).map(|(_, v)| v.clone()).ok_or(format!("no {k}"));
    let num = |k: &str| get(k)?.parse::<u64>().map_err(|e| format!("{k}: {e}"));
    let t = |k: &str| -> Result<(u64, u64), String> { Ok((num(&format!("{k}.sec"))?, num(&format!("{k}.nsec"))?)) };
    Ok(Qos {
        history: get("history")?,
        depth: num("depth")?,
        reliability: get("reliability")?,
        durability: get("durability")?,
        deadline: t("deadline")?,
        lifespan: t("lifespan")?,
        liveliness: get("liveliness")?,
        liveliness_lease_duration: t("liveliness_lease_duration")?,
        avoid_ros_namespace_conventions: get("avoid_ros_namespace_conventions")? == "true",
    })
}

/// A topic as rosbag2 knows it (`rosbag2_storage::TopicMetadata`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TopicMetadata {
    pub name: String,
    /// `pkg/msg/Type`
    pub type_name: String,
    /// `cdr`
    pub serialization_format: String,
    /// One entry per writer known when the topic was created.
    pub offered_qos_profiles: Vec<Qos>,
    /// `RIHS01_...`
    pub type_description_hash: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TopicInformation {
    pub topic_metadata: TopicMetadata,
    pub message_count: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FileInformation {
    pub path: String,
    /// Nanoseconds since the epoch (`i64::MAX` before the first message).
    pub starting_time: i64,
    pub duration: i64,
    pub message_count: u64,
}

/// `rosbag2_storage::BagMetadata` (version 9; `bag_size` is not written).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BagMetadata {
    pub version: i32,
    pub storage_identifier: String,
    pub duration: i64,
    pub starting_time: i64,
    pub message_count: u64,
    pub topics_with_message_count: Vec<TopicInformation>,
    pub compression_format: String,
    pub compression_mode: String,
    pub relative_file_paths: Vec<String>,
    pub files: Vec<FileInformation>,
    pub custom_data: Vec<(String, String)>,
    pub ros_distro: String,
}

impl BagMetadata {
    fn node(&self) -> Node {
        let ns = |n: i64| Node::Map(vec![("nanoseconds".into(), Node::int(n))]);
        let since = |n: i64| Node::Map(vec![("nanoseconds_since_epoch".into(), Node::int(n))]);
        let topics = self
            .topics_with_message_count
            .iter()
            .map(|t| {
                let m = &t.topic_metadata;
                Node::Map(vec![
                    (
                        "topic_metadata".into(),
                        Node::Map(vec![
                            ("name".into(), Node::str(&m.name)),
                            ("type".into(), Node::str(&m.type_name)),
                            ("serialization_format".into(), Node::str(&m.serialization_format)),
                            ("offered_qos_profiles".into(), Node::Seq(m.offered_qos_profiles.iter().map(Qos::node).collect())),
                            ("type_description_hash".into(), Node::str(&m.type_description_hash)),
                        ]),
                    ),
                    ("message_count".into(), Node::int(t.message_count)),
                ])
            })
            .collect();
        let files = self
            .files
            .iter()
            .map(|f| {
                Node::Map(vec![
                    ("path".into(), Node::str(&f.path)),
                    ("starting_time".into(), since(f.starting_time)),
                    ("duration".into(), ns(f.duration)),
                    ("message_count".into(), Node::int(f.message_count)),
                ])
            })
            .collect();
        let custom = if self.custom_data.is_empty() {
            Node::Null
        } else {
            Node::Map(self.custom_data.iter().map(|(k, v)| (k.clone(), Node::str(v))).collect())
        };
        Node::Map(vec![
            ("version".into(), Node::int(self.version)),
            ("storage_identifier".into(), Node::str(&self.storage_identifier)),
            ("duration".into(), ns(self.duration)),
            ("starting_time".into(), since(self.starting_time)),
            ("message_count".into(), Node::int(self.message_count)),
            ("topics_with_message_count".into(), Node::Seq(topics)),
            ("compression_format".into(), Node::str(&self.compression_format)),
            ("compression_mode".into(), Node::str(&self.compression_mode)),
            ("relative_file_paths".into(), Node::Seq(self.relative_file_paths.iter().map(|p| Node::str(p)).collect())),
            ("files".into(), Node::Seq(files)),
            ("custom_data".into(), custom),
            ("ros_distro".into(), Node::str(&self.ros_distro)),
        ])
    }

    /// The `serialized_metadata` of a `rosbag2` metadata record.
    pub fn to_record_yaml(&self) -> String {
        dump(&self.node())
    }

    /// `metadata.yaml`.
    pub fn to_metadata_yaml(&self) -> String {
        dump(&Node::Map(vec![("rosbag2_bagfile_information".into(), self.node())]))
    }
}

/// The nodes rosbag2 builds.
#[derive(Debug, Clone)]
enum Node {
    Null,
    /// Already formatted (a number, a bool).
    Scalar(String),
    Str(String),
    Seq(Vec<Node>),
    Map(Vec<(String, Node)>),
}

impl Node {
    fn int(v: impl ToString) -> Node {
        Node::Scalar(v.to_string())
    }
    fn str(s: &str) -> Node {
        Node::Str(s.to_string())
    }
}

/// `YAML::Dump(node)` for these nodes.
fn dump(node: &Node) -> String {
    let mut out = String::new();
    match node {
        Node::Map(m) if !m.is_empty() => map_block(&mut out, m, 0, false),
        Node::Seq(s) if !s.is_empty() => seq_block(&mut out, s, 0, false),
        other => out.push_str(&inline(other)),
    }
    out
}

fn indent(out: &mut String, n: usize) {
    out.extend(std::iter::repeat(' ').take(n));
}

/// A map's entries at column `col`; the first on the current line when
/// `first_inline` (a sequence item).
fn map_block(out: &mut String, m: &[(String, Node)], col: usize, first_inline: bool) {
    for (i, (k, v)) in m.iter().enumerate() {
        if i > 0 || !first_inline {
            if !out.is_empty() {
                out.push('\n');
            }
            indent(out, col);
        }
        out.push_str(&scalar(k));
        out.push(':');
        value(out, v, col);
    }
}

fn seq_block(out: &mut String, s: &[Node], col: usize, _first_inline: bool) {
    for item in s {
        if !out.is_empty() {
            out.push('\n');
        }
        indent(out, col);
        out.push('-');
        match item {
            Node::Map(m) if !m.is_empty() => {
                out.push(' ');
                map_block(out, m, col + 2, true);
            }
            Node::Seq(s) if !s.is_empty() => {
                out.push(' ');
                seq_block(out, s, col + 2, true);
            }
            other => {
                out.push(' ');
                out.push_str(&inline(other));
            }
        }
    }
}

/// The value after `key:` of a map at column `col`.
fn value(out: &mut String, v: &Node, col: usize) {
    match v {
        Node::Map(m) if !m.is_empty() => map_block(out, m, col + 2, false),
        Node::Seq(s) if !s.is_empty() => seq_block(out, s, col + 2, false),
        Node::Seq(_) => {
            out.push('\n');
            indent(out, col + 2);
            out.push_str("[]");
        }
        other => {
            out.push(' ');
            out.push_str(&inline(other));
        }
    }
}

fn inline(n: &Node) -> String {
    match n {
        Node::Null => "~".into(),
        Node::Scalar(s) => s.clone(),
        Node::Str(s) => scalar(s),
        Node::Seq(_) => "[]".into(),
        Node::Map(_) => "{}".into(),
    }
}

/// yaml-cpp's string: plain if `IsValidPlainScalar` (block context, UTF-8
/// out), else double quoted.
pub(crate) fn scalar(s: &str) -> String {
    if plain_ok(s) {
        return s.to_string();
    }
    let mut out = String::from("\"");
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\t' => out.push_str("\\t"),
            '\r' => out.push_str("\\r"),
            '\u{8}' => out.push_str("\\b"),
            '\u{c}' => out.push_str("\\f"),
            c if (c as u32) < 0x20 || c == '\u{feff}' || ('\u{7f}'..='\u{9f}').contains(&c) => {
                let _ = write!(out, "\\x{:02X}", c as u32);
            }
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

fn plain_ok(s: &str) -> bool {
    // IsNullString
    if matches!(s, "" | "~" | "null" | "Null" | "NULL") {
        return false;
    }
    let b = s.as_bytes();
    let blank_or_break = |c: Option<&u8>| matches!(c, None | Some(b' ' | b'\t' | b'\n' | b'\r'));
    // Exp::PlainScalar(): not a blank or break, not an indicator, not
    // `-`, `?` or `:` followed by a blank, a break or the end
    let first = b[0];
    if matches!(first, b' ' | b'\t' | b'\n' | b'\r') || b",[]{}#&*!|>'\"%@`".contains(&first) {
        return false;
    }
    if matches!(first, b'-' | b'?' | b':') && blank_or_break(b.get(1)) {
        return false;
    }
    if s.ends_with(' ') {
        return false;
    }
    for (i, &c) in b.iter().enumerate() {
        // EndScalar `:` + blank/break/end; blank or break + `#`; not
        // printable; BOM; break; tab
        if c == b':' && blank_or_break(b.get(i + 1)) {
            return false;
        }
        if matches!(c, b' ' | b'\t' | b'\n' | b'\r') && b.get(i + 1) == Some(&b'#') {
            return false;
        }
        if c == b'\n' || c == b'\r' || c == b'\t' || c < 0x20 || c == 0x7f {
            return false;
        }
    }
    !s.contains('\u{feff}')
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scalars_as_yaml_cpp_writes_them() {
        assert_eq!(scalar("/tf"), "/tf");
        assert_eq!(scalar("RIHS01_ab"), "RIHS01_ab");
        assert_eq!(scalar(""), "\"\"");
        assert_eq!(scalar("~"), "\"~\"");
        assert_eq!(scalar("a: b"), "\"a: b\"");
        assert_eq!(scalar("- x"), "\"- x\"");
        assert_eq!(scalar("x #y"), "\"x #y\"");
        assert_eq!(scalar("true"), "true");
    }

    #[test]
    fn qos_lists_round_trip() {
        let l = vec![Qos::new("keep_last", 1, "reliable", "transient_local"), Qos::new("keep_last", 100, "best_effort", "volatile")];
        let text = Qos::dump_list(&l);
        assert!(text.starts_with("- history: keep_last\n  depth: 1\n  reliability: reliable\n"), "{text}");
        assert!(!text.ends_with('\n'));
        assert_eq!(Qos::parse_list(&text).unwrap(), l);
        assert_eq!(Qos::dump_list(&[]), "[]");
    }
}
