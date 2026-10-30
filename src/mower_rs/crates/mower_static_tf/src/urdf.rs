//! A robot description as urdfdom 4 reads it and kdl_parser turns it into a
//! KDL tree, for what robot_state_publisher does with it: the joints, by
//! name, each with its parent and child link, its KDL segment, and the
//! joint it mimics.
//!
//! - **Read:** the `<link>` and `<joint>` elements directly under `<robot>`
//!   (a `<ros2_control>` block's joints are not joints of the tree). A joint
//!   needs a name, a known type, a parent and a child link; `<origin>` is
//!   optional (`xyz`, `rpy`: zero when not given), so is `<axis>` (x when the
//!   element is not there, zero when it has no `xyz`); revolute and prismatic
//!   need `<limit>`. Numbers are parsed as urdfdom's `strToDouble`
//!   (correctly rounded), lists split on spaces.
//! - **Checked:** unique link and joint names, every joint's links exist, no
//!   link with two parents, exactly one root.
//! - **KDL:** fixed and planar joints are `Joint::None` (kdl_parser's
//!   "unknown type" fallback for planar), floating ones too but rsp leaves
//!   those out; revolute and continuous turn about their axis, prismatic
//!   joints slide along it.

use std::collections::{BTreeMap, BTreeSet};

use crate::kdl::{self, Frame, Segment};
use crate::xml::{self, Element};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointType {
    Fixed,
    Revolute,
    Continuous,
    Prismatic,
    Planar,
    Floating,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Mimic {
    pub joint: String,
    pub multiplier: f64,
    pub offset: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Joint {
    pub name: String,
    pub kind: JointType,
    pub parent: String,
    pub child: String,
    pub segment: Segment,
    pub mimic: Option<Mimic>,
}

/// What rsp keeps of a robot description.
#[derive(Clone, Debug, PartialEq)]
pub struct Robot {
    pub name: String,
    pub root: String,
    /// Every joint, by name (rsp's maps are `std::map`s keyed by it).
    pub joints: BTreeMap<String, Joint>,
}

impl Robot {
    /// rsp's `segments_fixed_`: the fixed and planar joints, by name.
    pub fn fixed(&self) -> impl Iterator<Item = &Joint> {
        self.joints.values().filter(|j| matches!(j.kind, JointType::Fixed | JointType::Planar))
    }

    /// rsp's `segments_`: the joints a position moves.
    pub fn moving(&self, name: &str) -> Option<&Joint> {
        self.joints
            .get(name)
            .filter(|j| matches!(j.kind, JointType::Revolute | JointType::Continuous | JointType::Prismatic))
    }

    /// rsp's `mimic_`: the joints that mimic another, by name.
    pub fn mimics(&self) -> impl Iterator<Item = (&str, &Mimic)> {
        self.joints.values().filter_map(|j| j.mimic.as_ref().map(|m| (j.name.as_str(), m)))
    }
}

/// urdfdom's `strToDouble`.
fn number(s: &str, what: &str) -> Result<f64, String> {
    s.parse::<f64>().map_err(|_| format!("{what}: cannot read {s:?} as a number"))
}

/// urdfdom's `Vector3::init`: three numbers split on spaces.
fn vector(s: &str, what: &str) -> Result<[f64; 3], String> {
    let parts: Vec<f64> =
        s.split(' ').filter(|p| !p.is_empty()).map(|p| number(p, what)).collect::<Result<_, _>>()?;
    match parts[..] {
        [x, y, z] => Ok([x, y, z]),
        _ => Err(format!("{what}: {} numbers in {s:?}, 3 expected", parts.len())),
    }
}

fn pose(e: Option<&Element>, what: &str) -> Result<Frame, String> {
    let (mut xyz, mut rpy) = ([0.0; 3], [0.0; 3]);
    if let Some(e) = e {
        if let Some(v) = e.attribute("xyz") {
            xyz = vector(v, &format!("{what} origin xyz"))?;
        }
        if let Some(v) = e.attribute("rpy") {
            rpy = vector(v, &format!("{what} origin rpy"))?;
        }
    }
    Ok(kdl::origin(xyz, rpy))
}

fn joint(e: &Element) -> Result<Joint, String> {
    let name = e.attribute("name").ok_or("a joint without a name")?.to_string();
    let what = format!("joint {name}");
    let kind = match e.attribute("type").ok_or_else(|| format!("{what} has no type"))? {
        "fixed" => JointType::Fixed,
        "revolute" => JointType::Revolute,
        "continuous" => JointType::Continuous,
        "prismatic" => JointType::Prismatic,
        "planar" => JointType::Planar,
        "floating" => JointType::Floating,
        other => return Err(format!("{what} has no known type [{other}]")),
    };
    let origin = pose(e.first_child("origin"), &what)?;
    let link = |tag: &str| -> Result<String, String> {
        e.first_child(tag)
            .and_then(|c| c.attribute("link"))
            .map(str::to_string)
            .ok_or_else(|| format!("{what} has no {tag} link"))
    };
    let (parent, child) = (link("parent")?, link("child")?);
    let axis = match e.first_child("axis") {
        None => [1.0, 0.0, 0.0],
        Some(a) => match a.attribute("xyz") {
            Some(v) => vector(v, &format!("{what} axis"))?,
            None => [0.0, 0.0, 0.0],
        },
    };
    if matches!(kind, JointType::Revolute | JointType::Prismatic) && e.first_child("limit").is_none() {
        return Err(format!("{what} is of type {kind:?} but it does not specify limits"));
    }
    let kdl_joint = match kind {
        JointType::Revolute | JointType::Continuous => kdl::Joint::rot(&origin, axis),
        JointType::Prismatic => kdl::Joint::trans(&origin, axis),
        JointType::Fixed | JointType::Planar | JointType::Floating => kdl::Joint::None,
    };
    let mimic = match e.first_child("mimic") {
        None => None,
        Some(m) => Some(Mimic {
            joint: m.attribute("joint").ok_or_else(|| format!("{what}: mimic without a joint"))?.to_string(),
            multiplier: m.attribute("multiplier").map(|v| number(v, &what)).transpose()?.unwrap_or(1.0),
            offset: m.attribute("offset").map(|v| number(v, &what)).transpose()?.unwrap_or(0.0),
        }),
    };
    Ok(Joint { name, kind, parent, child, segment: Segment::new(kdl_joint, &origin), mimic })
}

/// The robot of a URDF text (`robot_description`).
pub fn parse(text: &str) -> Result<Robot, String> {
    let doc = xml::parse(text).map_err(|e| e.to_string())?;
    if doc.name != "robot" {
        return Err(format!("the root element is <{}>, not <robot>", doc.name));
    }
    let name = doc.attribute("name").ok_or("the robot has no name")?.to_string();
    let mut links = BTreeSet::new();
    for l in doc.children_named("link") {
        let n = l.attribute("name").ok_or("a link without a name")?;
        if !links.insert(n.to_string()) {
            return Err(format!("link '{n}' is not unique"));
        }
    }
    let mut joints = BTreeMap::new();
    let mut parent_of: BTreeMap<String, String> = BTreeMap::new();
    for e in doc.children_named("joint") {
        let j = joint(e)?;
        for l in [&j.parent, &j.child] {
            if !links.contains(l) {
                return Err(format!("link [{l}] of joint [{}] not found", j.name));
            }
        }
        if let Some(other) = parent_of.insert(j.child.clone(), j.name.clone()) {
            return Err(format!("link [{}] has two parent joints, [{other}] and [{}]", j.child, j.name));
        }
        if joints.insert(j.name.clone(), j).is_some() {
            return Err(format!("joint '{}' is not unique", e.attribute("name").unwrap_or_default()));
        }
    }
    let roots: Vec<&String> = links.iter().filter(|l| !parent_of.contains_key(*l)).collect();
    let root = match roots[..] {
        [one] => one.clone(),
        [] => return Err("no root link found: the robot is not a tree".into()),
        [a, b, ..] => return Err(format!("two root links found: [{a}] and [{b}]")),
    };
    Ok(Robot { name, root, joints })
}

#[cfg(test)]
mod tests {
    use super::*;

    const SMALL: &str = r#"<robot name="t">
        <link name="base"/><link name="wheel"/><link name="cam"/><link name="m"/>
        <joint name="w" type="continuous"><origin xyz="0 0.2 0" rpy="1.5708 0 0"/><parent link="base"/>
          <child link="wheel"/><axis xyz="0 0 1"/></joint>
        <joint name="c" type="fixed"><origin xyz="0.1 0 0.3"/><parent link="base"/><child link="cam"/></joint>
        <joint name="mm" type="continuous"><parent link="wheel"/><child link="m"/>
          <mimic joint="w" multiplier="2"/></joint>
      </robot>"#;

    #[test]
    fn joints_by_name_with_their_links_and_mimics() {
        let r = parse(SMALL).unwrap();
        assert_eq!((r.name.as_str(), r.root.as_str()), ("t", "base"));
        let names: Vec<&str> = r.joints.keys().map(String::as_str).collect();
        assert_eq!(names, ["c", "mm", "w"]);
        assert_eq!(r.fixed().map(|j| j.name.as_str()).collect::<Vec<_>>(), ["c"]);
        assert!(r.moving("w").is_some() && r.moving("c").is_none() && r.moving("nope").is_none());
        let (n, m) = r.mimics().next().unwrap();
        assert_eq!((n, m.joint.as_str(), m.multiplier, m.offset), ("mm", "w", 2.0, 0.0));
        // no axis: x
        assert!(matches!(r.joints["mm"].segment.joint, kdl::Joint::Rot { .. }));
    }

    #[test]
    fn what_urdfdom_refuses_is_an_error() {
        let bad = [
            "<robo name='x'/>",
            "<robot name='x'><link name='a'/><link name='a'/></robot>",
            "<robot name='x'><link name='a'/><link name='b'/></robot>",
            "<robot name='x'><link name='a'/><joint name='j' type='hinge'><parent link='a'/><child link='a'/></joint></robot>",
            "<robot name='x'><link name='a'/><link name='b'/><joint name='j' type='revolute'><parent link='a'/><child link='b'/></joint></robot>",
            "<robot name='x'><link name='a'/><joint name='j' type='fixed'><parent link='a'/><child link='z'/></joint></robot>",
            "<robot name='x'><link name='a'/><link name='b'/><joint name='j' type='fixed'><origin xyz='1 2'/><parent link='a'/><child link='b'/></joint></robot>",
        ];
        for text in bad {
            assert!(parse(text).is_err(), "{text}");
        }
    }
}
