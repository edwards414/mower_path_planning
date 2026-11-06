//! The services and actions the crates name, and the messages only they need
//! (E9-22, feature `interfaces`). `gen/gen_schema.py` copies them from the same
//! image as the rest of the table, and `gen/gen_types.py` writes `mower_msgs` from them.
//!
//! - **Who sees them:** no production crate turns the feature on, so the robot's
//!   binaries do not hold these files, and this crate's object code is byte for
//!   byte what it was without them.
//!   The feature's users are the tests of `mower_msgs_ros` and `mower_ros_facts`.
//!   Even with it on, [`crate::names`], [`crate::lookup`], [`crate::type_hash`] and
//!   [`crate::rosbag2_definition`] do not see them, so what the recorder
//!   would write and what `cdr_vectors.txt` holds does not move. [`crate::message`]
//!   reads them, so `mower_cdr` can lay them out.
//!
//! - **The parts** have rosidl's names: `pkg/srv/T_Request` and `T_Response`;
//!   `pkg/action/T_Goal`, `T_Result`, `T_Feedback`, `T_FeedbackMessage`,
//!   `T_SendGoal_Request`, `T_SendGoal_Response`, `T_GetResult_Request`,
//!   `T_GetResult_Response`. rosidl writes the last five itself, whatever the file says.
//! - **Sections** are split at a line `---`, as rosidl_adapter does.

use std::collections::HashMap;
use std::sync::OnceLock;

use crate::msg::{parse_msg, ArrayKind, Field, FieldType, MsgDef, ParseError, Primitive};
use crate::table;

/// A service or action of the table.
#[derive(Debug, Clone, Copy)]
pub struct Interface {
    /// `pkg/srv/Type` or `pkg/action/Type`
    pub name: &'static str,
    /// The `.srv` or `.action` file, byte for byte.
    pub text: &'static str,
    /// The RIHS01 hash of the type and of each part rosidl makes of it, by
    /// name, from the package's `<Type>.json`.
    pub hashes: &'static [(&'static str, &'static str)],
}

/// Every service and action, sorted.
pub fn interfaces() -> Vec<Interface> {
    table::INTERFACES.iter().map(|&(name, text, hashes)| Interface { name, text, hashes }).collect()
}

/// The roots they were generated for (gen/gen_schema.py `INTERFACE_ROOTS`).
pub fn interface_roots() -> &'static [&'static str] {
    table::INTERFACE_ROOTS
}

/// The RIHS01 hash of anything generated: a message of either table, a service,
/// an action or a part of one.
pub fn any_type_hash(name: &str) -> Option<&'static str> {
    crate::type_hash(name)
        .or_else(|| {
            let full = crate::full_name(name)?;
            table::MORE_TYPES.iter().find(|t| t.0 == full).map(|t| t.2)
        })
        .or_else(|| table::INTERFACES.iter().flat_map(|i| i.2.iter()).find(|h| h.0 == name).map(|h| h.1))
}

fn sections(text: &str) -> Vec<String> {
    let mut out = vec![String::new()];
    for line in text.split('\n') {
        if line.trim_end() == "---" {
            out.push(String::new());
        } else {
            let last = out.last_mut().expect("never empty");
            last.push_str(line);
            last.push('\n');
        }
    }
    out
}

fn single(name: &str, ty: FieldType) -> Field {
    Field { name: name.to_string(), ty, array: ArrayKind::Single }
}

fn nested(name: &str, ty: String) -> Field {
    single(name, FieldType::Message(ty))
}

fn uuid(name: &str) -> Field {
    nested(name, "unique_identifier_msgs/msg/UUID".to_string())
}

/// The layout of a message this module adds, by `pkg/msg/Type`, `pkg/Type` or a
/// part's full name: what [`crate::message`] reads before the recorder's table.
pub(crate) fn layout(name: &str) -> Option<Result<&'static MsgDef, ParseError>> {
    static CACHE: OnceLock<HashMap<String, Result<MsgDef, ParseError>>> = OnceLock::new();
    let cache = CACHE.get_or_init(|| more_layouts().into_iter().collect());
    let full = crate::full_name(name);
    cache.get(full.as_deref().unwrap_or(name)).map(|l| l.as_ref().map_err(Clone::clone))
}

/// Every message type a service or an action makes, and the further messages,
/// with their layouts.
fn more_layouts() -> Vec<(String, Result<MsgDef, ParseError>)> {
    let mut out = Vec::new();
    for &(name, text, _) in table::MORE_TYPES {
        out.push((name.to_string(), parse_msg(name, text)));
    }
    for &(name, text, _) in table::INTERFACES {
        let parts = sections(text);
        let section = |suffix: &str, i: usize| {
            let part = format!("{name}{suffix}");
            let parsed = match parts.get(i) {
                Some(section) => parse_msg(&part, section),
                None => Err(ParseError::new(&part, 0, "the file has no such section")),
            };
            (part, parsed)
        };
        if name.contains("/srv/") {
            out.push(section("_Request", 0));
            out.push(section("_Response", 1));
        } else {
            out.push(section("_Goal", 0));
            out.push(section("_Result", 1));
            out.push(section("_Feedback", 2));
            let made = |suffix: &str, fields: Vec<Field>| {
                let part = format!("{name}{suffix}");
                (part.clone(), Ok(MsgDef { name: part, fields }))
            };
            out.push(made("_FeedbackMessage", vec![uuid("goal_id"), nested("feedback", format!("{name}_Feedback"))]));
            out.push(made("_SendGoal_Request", vec![uuid("goal_id"), nested("goal", format!("{name}_Goal"))]));
            out.push(made("_SendGoal_Response", vec![
                single("accepted", FieldType::Primitive(Primitive::Bool)),
                nested("stamp", "builtin_interfaces/msg/Time".to_string()),
            ]));
            out.push(made("_GetResult_Request", vec![uuid("goal_id")]));
            out.push(made("_GetResult_Response", vec![
                single("status", FieldType::Primitive(Primitive::Int8)),
                nested("result", format!("{name}_Result")),
            ]));
        }
    }
    out
}

/// The name of every message type [`crate::message`] reads, sorted.
pub fn message_names() -> Vec<String> {
    let mut names: Vec<String> = crate::names().into_iter().map(str::to_string).collect();
    names.extend(more_layouts().into_iter().map(|(name, _)| name));
    names.sort();
    names
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_service_has_two_sections_and_an_action_three() {
        for i in interfaces() {
            let want = if i.name.contains("/srv/") { 2 } else { 3 };
            assert_eq!(sections(i.text).len(), want, "{}", i.name);
            assert!(i.hashes.iter().any(|h| h.0 == i.name), "{}", i.name);
            assert!(i.hashes.iter().all(|h| h.1.starts_with("RIHS01_") && h.1.len() == 71), "{}", i.name);
        }
    }

    #[test]
    fn every_layout_parses_and_every_type_it_names_is_known() {
        let names = message_names();
        for name in &names {
            let def = crate::message(name).unwrap_or_else(|e| panic!("{name}: {e}"));
            assert_eq!(&def.name, name);
            for f in &def.fields {
                if let FieldType::Message(t) = &f.ty {
                    assert!(names.contains(t), "{name}.{} is a {t}", f.name);
                }
                assert!(!matches!(f.ty, FieldType::WString { .. }), "{name}.{}: mower_cdr has no wstring", f.name);
            }
        }
        for name in &names {
            assert!(any_type_hash(name).is_some(), "{name} has no hash");
        }
    }

    #[test]
    fn the_parts_of_an_action_are_rosidl_s() {
        let names = |t: &str| -> Vec<String> {
            crate::message(t).unwrap().fields.iter().map(|f| f.name.clone()).collect()
        };
        let a = "mower_interface/action/Waypoint";
        assert_eq!(names(&format!("{a}_SendGoal_Request")), ["goal_id", "goal"]);
        assert_eq!(names(&format!("{a}_SendGoal_Response")), ["accepted", "stamp"]);
        assert_eq!(names(&format!("{a}_GetResult_Request")), ["goal_id"]);
        assert_eq!(names(&format!("{a}_GetResult_Response")), ["status", "result"]);
        assert_eq!(names(&format!("{a}_FeedbackMessage")), ["goal_id", "feedback"]);
        assert_eq!(names("std_srvs/srv/Trigger_Request"), ["structure_needs_at_least_one_member"]);
        assert_eq!(names("std_srvs/srv/Trigger_Response"), ["success", "message"]);
    }

    /// The recorder's table is what it was: none of this reaches what it writes.
    #[test]
    fn the_recorder_s_table_does_not_see_these() {
        assert_eq!(crate::names().len(), 39);
        for t in ["std_msgs/msg/Empty", "rcl_interfaces/msg/ParameterEvent", "action_msgs/msg/GoalStatusArray"] {
            assert!(crate::lookup(t).is_none(), "{t}");
            assert_eq!(crate::rosbag2_definition(t).encoding, "", "{t}");
            assert!(crate::type_hash(t).is_none(), "{t}");
            assert!(crate::message(t).is_ok(), "{t}");
        }
    }
}
