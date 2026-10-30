//! The front camera's extrinsics: the measured YAML turned into the static
//! TF and the derived calib block, a port of mower_recorder's
//! `camera_extrinsics.py` (the module docstring there says what is measured
//! and the frames' conventions).
//!
//! The arithmetic is the Python's, operation for operation, so the floats
//! come out bit for bit as the Python recorder's on the same libm: every
//! `sum()` of three products is CPython 3.12's (it starts from the int 0
//! and then adds with Neumaier compensation, [`py_sum3`]), `math.degrees` /
//! `math.radians` multiply by CPython's constants, and `round(x, n)` is the
//! correctly rounded decimal rounding ([`py_round`]). `tests/vectors.rs`
//! holds it to what the real Python computed in the robot image.
//!
//! Dicts in and out are `mower_pyfmt::Py`, as the Python's are plain dicts.

use mower_pyfmt::{repr, yaml, Py};

pub const SCHEMA: &str = "mower.camera_extrinsics/v1";
pub const VALID_SOURCES: [&str; 3] = ["placeholder", "measured", "cad"];

pub const BASE_FOOTPRINT: &str = "base_footprint";
pub const BASE_LINK: &str = "base_link";
pub const CAMERA_LINK: &str = "camera_front_link";
pub const OPTICAL_FRAME: &str = "camera_front_optical_frame";

/// The fallback geometry when the URDF cannot be read (robot.urdf.xacro,
/// 2026-09-25; camera_extrinsics.py:49-57).
#[allow(clippy::approx_constant)] // the URDF's 1.5708, not pi/2
pub const DEFAULT_BASE_LINK_RPY: Vec3 = [0.0, 0.0, 1.5708];
pub const DEFAULT_BASE_LINK_XYZ: Vec3 = [0.0, 0.0, 0.0];
pub const DEFAULT_WHEEL_JOINTS_IN_BASE_LINK: [Vec3; 2] = [[0.13253, 0.165534, -0.00295275], [-0.10347, 0.165534, -0.00295275]];

/// camera_front_link -> camera_front_optical_frame (REP-103):
/// `(-math.pi / 2.0, 0.0, -math.pi / 2.0)`.
pub const OPTICAL_RPY: Vec3 = [-std::f64::consts::PI / 2.0, 0.0, -std::f64::consts::PI / 2.0];

pub type Vec3 = [f64; 3];
pub type Mat3 = [[f64; 3]; 3];

// ---- what Python does with floats --------------------------------------------------

/// `sum(gen)` of three floats in CPython 3.12 (bltinmodule.c
/// `builtin_sum_impl`): the start value is the int 0, so the first term is
/// `0 + x0` (an int-float add, which turns `-0.0` into `0.0`); the rest goes
/// through the float loop with Neumaier compensation, and the compensation
/// is added at the end when it is finite and not zero.
pub fn py_sum3(x0: f64, x1: f64, x2: f64) -> f64 {
    let mut f = 0.0 + x0;
    let mut c = 0.0f64;
    for x in [x1, x2] {
        let t = f + x;
        if f.abs() >= x.abs() {
            c += (f - t) + x;
        } else {
            c += (x - t) + f;
        }
        f = t;
    }
    if c != 0.0 && c.is_finite() {
        f += c;
    }
    f
}

/// `math.degrees(x)`: `x * (180.0 / pi)` with CPython's constant.
pub fn degrees(x: f64) -> f64 {
    const RAD_TO_DEG: f64 = 180.0 / std::f64::consts::PI;
    x * RAD_TO_DEG
}

/// `math.radians(x)`: `x * (pi / 180.0)` with CPython's constant.
pub fn radians(x: f64) -> f64 {
    const DEG_TO_RAD: f64 = std::f64::consts::PI / 180.0;
    x * DEG_TO_RAD
}

/// `round(x, n)` for a float (floatobject.c `double_round`): the decimal
/// rounding of the exact binary value to `n` places, ties to even, read back
/// correctly rounded; NaN and infinities stay as they are.
pub fn py_round(x: f64, n: usize) -> f64 {
    if !x.is_finite() || n > 323 {
        return x;
    }
    // Rust's fixed-precision formatting is exact and rounds ties to even
    format!("{x:.n$}").parse().expect("a formatted float parses")
}

/// `float(v)`; the error is the TypeError / ValueError text.
pub(crate) fn py_float(v: &Py) -> Result<f64, String> {
    match v {
        Py::Bool(b) => Ok(if *b { 1.0 } else { 0.0 }),
        Py::Int(i) => Ok(*i as f64),
        Py::Float(f) => Ok(*f),
        Py::Str(s) => parse_py_float(s).ok_or_else(|| format!("could not convert string to float: {}", str_repr(s))),
        other => Err(format!("float() argument must be a string or a real number, not '{}'", type_name(other))),
    }
}

/// `float(str)`: surrounding whitespace, underscores between digits,
/// `inf` / `infinity` / `nan` in any case with a sign.
pub fn parse_py_float(s: &str) -> Option<f64> {
    let t = py_strip(s);
    let (sign, body) = match t.strip_prefix('-') {
        Some(r) => (-1.0, r),
        None => (1.0, t.strip_prefix('+').unwrap_or(t)),
    };
    if body.starts_with(['+', '-']) {
        return None;
    }
    let lower = body.to_ascii_lowercase();
    if lower == "inf" || lower == "infinity" {
        return Some(sign * f64::INFINITY);
    }
    if lower == "nan" {
        return Some(f64::NAN.copysign(sign));
    }
    // underscores only between two digits
    let b = body.as_bytes();
    for (i, &c) in b.iter().enumerate() {
        if c == b'_' && !(i > 0 && b[i - 1].is_ascii_digit() && b.get(i + 1).is_some_and(|d| d.is_ascii_digit())) {
            return None;
        }
    }
    let clean: String = body.chars().filter(|&c| c != '_').collect();
    if clean.is_empty() || !clean.bytes().all(|c| c.is_ascii_digit() || matches!(c, b'.' | b'e' | b'E' | b'+' | b'-')) {
        return None;
    }
    clean.parse::<f64>().ok().map(|v| sign * v)
}

/// `str.strip()`: Python's whitespace (`Py_UNICODE_ISSPACE`), which adds
/// U+001C..U+001F to Unicode's White_Space.
pub(crate) fn py_strip(s: &str) -> &str {
    s.trim_matches(|c: char| c.is_whitespace() || ('\u{1c}'..='\u{1f}').contains(&c))
}

pub(crate) fn type_name(v: &Py) -> &'static str {
    match v {
        Py::None => "NoneType",
        Py::Bool(_) => "bool",
        Py::Int(_) => "int",
        Py::Float(_) => "float",
        Py::Str(_) => "str",
        Py::List(_) => "list",
        Py::Dict(_) => "dict",
        #[allow(unreachable_patterns)]
        _ => "object",
    }
}

/// The plain YAML scalar of a value mower_pyfmt dumps (a date: what
/// `str(date)` and `date.isoformat()` give).
fn yaml_scalar(v: &Py) -> String {
    let text = yaml::safe_dump(v, &yaml::YamlOptions { allow_unicode: true, sort_keys: false });
    text.strip_suffix("\n...\n").or_else(|| text.strip_suffix('\n')).unwrap_or(&text).to_string()
}

/// `str(v)`.
pub(crate) fn py_str(v: &Py) -> String {
    match v {
        Py::Str(s) => s.clone(),
        Py::None | Py::Bool(_) | Py::Int(_) | Py::Float(_) | Py::List(_) | Py::Dict(_) => py_repr(v),
        #[allow(unreachable_patterns)]
        other => yaml_scalar(other),
    }
}

/// `repr(v)`. (A value mower_pyfmt has beyond these, a date, is written as
/// its YAML scalar: not Python's `datetime.date(...)`.)
pub(crate) fn py_repr(v: &Py) -> String {
    match v {
        Py::None => "None".into(),
        Py::Bool(b) => (if *b { "True" } else { "False" }).into(),
        Py::Int(i) => i.to_string(),
        Py::Float(f) => repr::float(*f),
        Py::Str(s) => str_repr(s),
        Py::List(items) => format!("[{}]", items.iter().map(py_repr).collect::<Vec<_>>().join(", ")),
        Py::Dict(items) => format!(
            "{{{}}}",
            items.iter().map(|(k, v)| format!("{}: {}", str_repr(k), py_repr(v))).collect::<Vec<_>>().join(", ")
        ),
        #[allow(unreachable_patterns)]
        other => yaml_scalar(other),
    }
}

/// `Py_UNICODE_ISPRINTABLE` for the code points such a file plausibly holds
/// (the same approximation as mower_flash's): controls, format characters,
/// separators other than the space, private use and noncharacters are not.
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
        || (0xD800..=0xDFFF).contains(&u)
        || (0xE000..=0xF8FF).contains(&u)
        || u == 0xFEFF
        || (0xFFF9..=0xFFFB).contains(&u)
        || (0xFDD0..=0xFDEF).contains(&u)
        || u & 0xFFFE == 0xFFFE
        || (0xF0000..=0x10FFFF).contains(&u))
}

/// `repr(str)`: single quotes unless the text has one and no double quote.
pub(crate) fn str_repr(s: &str) -> String {
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

/// `bool(v)`.
pub(crate) fn truthy(v: &Py) -> bool {
    match v {
        Py::None => false,
        Py::Bool(b) => *b,
        Py::Int(i) => *i != 0,
        Py::Float(f) => *f != 0.0,
        Py::Str(s) => !s.is_empty(),
        Py::List(items) => !items.is_empty(),
        Py::Dict(items) => !items.is_empty(),
        #[allow(unreachable_patterns)]
        _ => true,
    }
}

// ---- small rigid-transform helpers (camera_extrinsics.py:63-142) ----------------------

/// `math.cos` / `math.sin` refuse what glibc turns into a NaN from a
/// number (mathmodule.c `math_1`: an infinite angle): "math domain error".
fn domain_checked(x: f64) -> Result<(), String> {
    if x.is_infinite() {
        Err("math domain error".into())
    } else {
        Ok(())
    }
}

/// [`rpy_to_matrix`] where Python raises: an infinite angle is
/// `ValueError: math domain error`.
pub fn try_rpy_to_matrix(roll: f64, pitch: f64, yaw: f64) -> Result<Mat3, String> {
    domain_checked(roll)?;
    domain_checked(pitch)?;
    domain_checked(yaw)?;
    Ok(rpy_to_matrix(roll, pitch, yaw))
}

/// `rpy_to_matrix(roll, pitch, yaw)`. An infinite angle gives NaNs here,
/// where the Python raises; [`try_rpy_to_matrix`] raises.
pub fn rpy_to_matrix(roll: f64, pitch: f64, yaw: f64) -> Mat3 {
    let (cr, sr) = (roll.cos(), roll.sin());
    let (cp, sp) = (pitch.cos(), pitch.sin());
    let (cy, sy) = (yaw.cos(), yaw.sin());
    [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
}

/// The inverse of [`rpy_to_matrix`] (fixed-axis XYZ); pitch in [-pi/2, pi/2].
pub fn matrix_to_rpy(m: &Mat3) -> Vec3 {
    let sp = -m[2][0];
    // max(-1.0, min(1.0, sp)): Python's min/max keep the first of equals and
    // never pick a NaN over a number
    let sp = if sp < 1.0 { sp } else { 1.0 };
    let sp = if sp > -1.0 { sp } else { -1.0 };
    let pitch = sp.asin();
    if sp.abs() < 1.0 - 1e-9 {
        [m[2][1].atan2(m[2][2]), pitch, m[1][0].atan2(m[0][0])]
    } else {
        // gimbal lock: everything into yaw
        [0.0, pitch, (-m[0][1]).atan2(m[1][1])]
    }
}

/// A rotation matrix as `(x, y, z, w)` with `w >= 0`.
pub fn matrix_to_quaternion(m: &Mat3) -> [f64; 4] {
    let tr = m[0][0] + m[1][1] + m[2][2];
    let (x, y, z, w);
    if tr > 0.0 {
        let s = (tr + 1.0).sqrt() * 2.0;
        w = 0.25 * s;
        x = (m[2][1] - m[1][2]) / s;
        y = (m[0][2] - m[2][0]) / s;
        z = (m[1][0] - m[0][1]) / s;
    } else if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
        let s = (1.0 + m[0][0] - m[1][1] - m[2][2]).sqrt() * 2.0;
        w = (m[2][1] - m[1][2]) / s;
        x = 0.25 * s;
        y = (m[0][1] + m[1][0]) / s;
        z = (m[0][2] + m[2][0]) / s;
    } else if m[1][1] > m[2][2] {
        let s = (1.0 + m[1][1] - m[0][0] - m[2][2]).sqrt() * 2.0;
        w = (m[0][2] - m[2][0]) / s;
        x = (m[0][1] + m[1][0]) / s;
        y = 0.25 * s;
        z = (m[1][2] + m[2][1]) / s;
    } else {
        let s = (1.0 + m[2][2] - m[0][0] - m[1][1]).sqrt() * 2.0;
        w = (m[1][0] - m[0][1]) / s;
        x = (m[0][2] + m[2][0]) / s;
        y = (m[1][2] + m[2][1]) / s;
        z = 0.25 * s;
    }
    let n = (x * x + y * y + z * z + w * w).sqrt();
    let q = [x / n, y / n, z / n, w / n];
    if q[3] >= 0.0 {
        q
    } else {
        [-q[0], -q[1], -q[2], -q[3]]
    }
}

pub fn quaternion_to_matrix(q: [f64; 4]) -> Mat3 {
    let [x, y, z, w] = q;
    let n = (x * x + y * y + z * z + w * w).sqrt();
    let (x, y, z, w) = (x / n, y / n, z / n, w / n);
    [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]
}

fn matmul(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut out = [[0.0; 3]; 3];
    for (i, row) in out.iter_mut().enumerate() {
        for (j, v) in row.iter_mut().enumerate() {
            *v = py_sum3(a[i][0] * b[0][j], a[i][1] * b[1][j], a[i][2] * b[2][j]);
        }
    }
    out
}

fn matvec(a: &Mat3, v: &Vec3) -> Vec3 {
    [0, 1, 2].map(|i| py_sum3(a[i][0] * v[0], a[i][1] * v[1], a[i][2] * v[2]))
}

fn transpose(a: &Mat3) -> Mat3 {
    [0, 1, 2].map(|i| [a[0][i], a[1][i], a[2][i]])
}

/// A rigid transform `T_parent_child`: `p_parent = R @ p_child + t`
/// (camera_extrinsics.py:145-183).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transform {
    pub rotation: Mat3,
    pub translation: Vec3,
}

impl Default for Transform {
    fn default() -> Self {
        Transform { rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], translation: [0.0; 3] }
    }
}

impl Transform {
    pub fn new(rotation: Mat3, translation: Vec3) -> Transform {
        Transform { rotation, translation }
    }

    pub fn from_xyz_rpy(xyz: Vec3, rpy: Vec3) -> Transform {
        Transform { rotation: rpy_to_matrix(rpy[0], rpy[1], rpy[2]), translation: xyz }
    }

    /// `Transform.from_xyz_rpy` with Python's domain error for an infinite
    /// angle.
    pub fn try_from_xyz_rpy(xyz: Vec3, rpy: Vec3) -> Result<Transform, String> {
        Ok(Transform { rotation: try_rpy_to_matrix(rpy[0], rpy[1], rpy[2])?, translation: xyz })
    }

    /// `self @ other`.
    pub fn compose(&self, other: &Transform) -> Transform {
        let rot = matmul(&self.rotation, &other.rotation);
        let rt = matvec(&self.rotation, &other.translation);
        Transform { rotation: rot, translation: [0, 1, 2].map(|i| rt[i] + self.translation[i]) }
    }

    pub fn inverse(&self) -> Transform {
        let rt = transpose(&self.rotation);
        let t = matvec(&rt, &self.translation);
        Transform { rotation: rt, translation: [-t[0], -t[1], -t[2]] }
    }

    pub fn apply(&self, point: Vec3) -> Vec3 {
        let r = matvec(&self.rotation, &point);
        [0, 1, 2].map(|i| r[i] + self.translation[i])
    }

    pub fn rotate(&self, vector: Vec3) -> Vec3 {
        matvec(&self.rotation, &vector)
    }

    pub fn rpy(&self) -> Vec3 {
        matrix_to_rpy(&self.rotation)
    }

    pub fn quaternion(&self) -> [f64; 4] {
        matrix_to_quaternion(&self.rotation)
    }
}

// ---- URDF reference geometry (camera_extrinsics.py:186-241) ------------------------------

/// What the conversion needs of the robot: `base_link` in `base_footprint`
/// and the drive-wheel axle's midpoint, with where they came from (`urdf` or
/// `builtin_default`).
#[derive(Debug, Clone, PartialEq)]
pub struct Reference {
    pub base_link_in_base_footprint: Transform,
    pub axle_center_in_base_footprint: Vec3,
    pub source: String,
}

/// One element of the small XML tree [`urdf_reference`] reads.
#[derive(Debug, Default)]
struct Element {
    /// as ElementTree names it: `{namespace}local` when in a namespace
    tag: String,
    attrs: Vec<(String, String)>,
    children: Vec<Element>,
}

impl Element {
    fn get(&self, name: &str) -> Option<&str> {
        self.attrs.iter().find(|(k, _)| k == name).map(|(_, v)| v.as_str())
    }
    /// `find(tag)`: the first direct child with that tag.
    fn find(&self, tag: &str) -> Option<&Element> {
        self.children.iter().find(|c| c.tag == tag)
    }
    /// `iter(tag)`: this element and its descendants, document order.
    fn iter<'a>(&'a self, tag: &str, out: &mut Vec<&'a Element>) {
        if self.tag == tag {
            out.push(self);
        }
        for c in &self.children {
            c.iter(tag, out);
        }
    }
}

fn unescape(s: &str) -> Result<String, String> {
    let mut out = String::new();
    let mut rest = s;
    while let Some(i) = rest.find('&') {
        out.push_str(&rest[..i]);
        let end = rest[i..].find(';').ok_or("an unterminated entity")? + i;
        let ent = &rest[i + 1..end];
        match ent {
            "lt" => out.push('<'),
            "gt" => out.push('>'),
            "amp" => out.push('&'),
            "quot" => out.push('"'),
            "apos" => out.push('\''),
            _ => {
                let code = if let Some(h) = ent.strip_prefix("#x") {
                    u32::from_str_radix(h, 16).ok()
                } else if let Some(d) = ent.strip_prefix('#') {
                    d.parse().ok()
                } else {
                    None
                };
                out.push(code.and_then(char::from_u32).ok_or_else(|| format!("undefined entity &{ent};"))?);
            }
        }
        rest = &rest[end + 1..];
    }
    out.push_str(rest);
    Ok(out)
}

/// A minimal XML reader, enough for a URDF: elements, attributes (entities
/// expanded, whitespace normalised as XML does), namespaces resolved to
/// ElementTree's `{uri}local` tags; comments, processing instructions,
/// CDATA, text and the doctype are skipped.
fn parse_xml(xml: &str) -> Result<Element, String> {
    let mut stack: Vec<(Element, Vec<(String, String)>)> = Vec::new();
    let mut root: Option<Element> = None;
    let mut i = 0;
    let b = xml.as_bytes();
    while i < b.len() {
        if b[i] != b'<' {
            i += 1;
            continue;
        }
        let rest = &xml[i..];
        if rest.starts_with("<!--") {
            i += rest.find("-->").ok_or("an unterminated comment")? + 3;
            continue;
        }
        if rest.starts_with("<?") {
            i += rest.find("?>").ok_or("an unterminated processing instruction")? + 2;
            continue;
        }
        if rest.starts_with("<![CDATA[") {
            i += rest.find("]]>").ok_or("an unterminated CDATA section")? + 3;
            continue;
        }
        if rest.starts_with("<!") {
            i += rest.find('>').ok_or("an unterminated declaration")? + 1;
            continue;
        }
        // a tag: find its end outside quotes
        let mut j = 1;
        let mut quote = None;
        while j < rest.len() {
            let c = rest.as_bytes()[j];
            match quote {
                Some(q) if c == q => quote = None,
                Some(_) => {}
                None if c == b'"' || c == b'\'' => quote = Some(c),
                None if c == b'>' => break,
                None => {}
            }
            j += 1;
        }
        if j >= rest.len() {
            return Err("an unterminated tag".into());
        }
        let inner = &rest[1..j];
        i += j + 1;
        if let Some(name) = inner.strip_prefix('/') {
            let (el, _) = stack.pop().ok_or("a closing tag without its start")?;
            let _ = name;
            match stack.last_mut() {
                Some((parent, _)) => parent.children.push(el),
                None => root = Some(el),
            }
            continue;
        }
        let empty = inner.ends_with('/');
        let inner = inner.trim_end_matches('/');
        let name_end = inner.find(|c: char| c.is_whitespace()).unwrap_or(inner.len());
        let raw_name = &inner[..name_end];
        let mut raw_attrs = Vec::new();
        let mut a = inner[name_end..].trim_start();
        while !a.is_empty() {
            let eq = a.find('=').ok_or("an attribute without a value")?;
            let key = a[..eq].trim().to_string();
            let v = a[eq + 1..].trim_start();
            let q = v.chars().next().ok_or("an attribute without a value")?;
            if q != '"' && q != '\'' {
                return Err("an unquoted attribute".into());
            }
            let close = v[1..].find(q).ok_or("an unterminated attribute")? + 1;
            let value: String = unescape(&v[1..close])?.chars().map(|c| if matches!(c, '\t' | '\n' | '\r') { ' ' } else { c }).collect();
            raw_attrs.push((key, value));
            a = v[close + 1..].trim_start();
        }
        // namespaces in scope: the parent's, then this element's xmlns
        let mut ns: Vec<(String, String)> = stack.last().map(|(_, n)| n.clone()).unwrap_or_default();
        for (k, v) in &raw_attrs {
            if k == "xmlns" {
                ns.push((String::new(), v.clone()));
            } else if let Some(p) = k.strip_prefix("xmlns:") {
                ns.push((p.to_string(), v.clone()));
            }
        }
        let resolve = |qname: &str, is_attr: bool| -> Result<String, String> {
            let (prefix, local) = match qname.split_once(':') {
                Some((p, l)) => (p, l),
                None if is_attr => return Ok(qname.to_string()),
                None => ("", qname),
            };
            if prefix == "xml" {
                return Ok(format!("{{http://www.w3.org/XML/1998/namespace}}{local}"));
            }
            match ns.iter().rev().find(|(p, _)| p == prefix) {
                Some((_, uri)) if uri.is_empty() => Ok(local.to_string()),
                Some((_, uri)) => Ok(format!("{{{uri}}}{local}")),
                None if prefix.is_empty() => Ok(local.to_string()),
                None => Err(format!("unbound prefix {prefix:?}")),
            }
        };
        let tag = resolve(raw_name, false)?;
        let mut attrs = Vec::new();
        for (k, v) in raw_attrs {
            if k == "xmlns" || k.starts_with("xmlns:") {
                continue;
            }
            attrs.push((resolve(&k, true)?, v));
        }
        let el = Element { tag, attrs, children: Vec::new() };
        if empty {
            match stack.last_mut() {
                Some((parent, _)) => parent.children.push(el),
                None => root = Some(el),
            }
        } else {
            stack.push((el, ns));
        }
        if root.is_some() && stack.is_empty() {
            break;
        }
    }
    if !stack.is_empty() {
        return Err("unclosed elements".into());
    }
    root.ok_or_else(|| "no element found".into())
}

fn floats(s: &str) -> Result<Vec<f64>, String> {
    s.split_whitespace().map(|v| parse_py_float(v).ok_or_else(|| format!("could not convert string to float: {}", str_repr(v)))).collect()
}

/// An origin's numbers: xyz may hold more than three (the Python indexes
/// the first three), rpy must hold three (`rpy_to_matrix(*rpy)`).
fn vec3(v: &[f64], what: &str) -> Result<Vec3, String> {
    let ok = if what == "rpy" { v.len() == 3 } else { v.len() >= 3 };
    if !ok {
        return Err(format!("{what}: expected 3 values, got {}", v.len()));
    }
    Ok([v[0], v[1], v[2]])
}

/// `_origin(joint)`: the joint's `<origin>` xyz and rpy (`0 0 0` when absent
/// or empty).
fn origin(joint: &Element) -> Result<(Vec<f64>, Vec<f64>), String> {
    match joint.find("origin") {
        None => Ok((vec![0.0; 3], vec![0.0; 3])),
        Some(o) => {
            let xyz = o.get("xyz").filter(|s| !s.is_empty()).unwrap_or("0 0 0");
            let rpy = o.get("rpy").filter(|s| !s.is_empty()).unwrap_or("0 0 0");
            Ok((floats(xyz)?, floats(rpy)?))
        }
    }
}

/// `urdf_reference(urdf_xml)`: `base_footprint -> base_link` and the two
/// drive wheels (`continuous` joints of `base_link` named `*wheel*`, not
/// `*caster*`) from a processed URDF or a plain-XML xacro. The error texts
/// are the Python's ValueErrors.
pub fn urdf_reference(urdf_xml: &str) -> Result<Reference, String> {
    let root = parse_xml(urdf_xml)?;
    // name -> (parent link, child link, type, (xyz, rpy)), as the Python's dict
    type Joint = (Option<String>, Option<String>, Option<String>, (Vec<f64>, Vec<f64>));
    let mut joints: Vec<(Option<String>, Joint)> = Vec::new();
    let mut found = Vec::new();
    root.iter("joint", &mut found);
    for j in found {
        let (Some(parent), Some(child)) = (j.find("parent"), j.find("child")) else { continue };
        let entry = (parent.get("link").map(str::to_string), child.get("link").map(str::to_string), j.get("type").map(str::to_string), origin(j)?);
        let name = j.get("name").map(str::to_string);
        // a dict keyed by name: a later joint of the same name replaces the
        // value and keeps the first one's place
        match joints.iter_mut().find(|(n, _)| *n == name) {
            Some(slot) => slot.1 = entry,
            None => joints.push((name, entry)),
        }
    }
    let mut t_bf_bl = None;
    let mut wheels = Vec::new();
    for (name, (parent, child, jtype, (xyz, rpy))) in &joints {
        if parent.as_deref() == Some(BASE_FOOTPRINT) && child.as_deref() == Some(BASE_LINK) {
            t_bf_bl = Some(Transform::try_from_xyz_rpy(vec3(xyz, "xyz")?, vec3(rpy, "rpy")?)?);
        } else if parent.as_deref() == Some(BASE_LINK) && jtype.as_deref() == Some("continuous") {
            let n = name.as_deref().ok_or("argument of type 'NoneType' is not iterable")?;
            if n.contains("wheel") && !n.contains("caster") {
                wheels.push(xyz.clone());
            }
        }
    }
    let t_bf_bl = t_bf_bl.ok_or("URDF has no base_footprint -> base_link joint")?;
    if wheels.len() != 2 {
        return Err(format!("expected 2 drive wheel joints, found {}", wheels.len()));
    }
    let (w0, w1) = (vec3(&wheels[0], "xyz")?, vec3(&wheels[1], "xyz")?);
    let mid = [0, 1, 2].map(|i| (w0[i] + w1[i]) / 2.0);
    Ok(Reference { axle_center_in_base_footprint: t_bf_bl.apply(mid), base_link_in_base_footprint: t_bf_bl, source: "urdf".into() })
}

/// `default_reference()`: the built-in geometry.
pub fn default_reference() -> Reference {
    let t_bf_bl = Transform::from_xyz_rpy(DEFAULT_BASE_LINK_XYZ, DEFAULT_BASE_LINK_RPY);
    let [w0, w1] = DEFAULT_WHEEL_JOINTS_IN_BASE_LINK;
    let mid = [0, 1, 2].map(|i| (w0[i] + w1[i]) / 2.0);
    Reference { axle_center_in_base_footprint: t_bf_bl.apply(mid), base_link_in_base_footprint: t_bf_bl, source: "builtin_default".into() }
}

// ---- the YAML model (camera_extrinsics.py:244-291) ----------------------------------------

fn is_number(v: Option<&Py>) -> bool {
    matches!(v, Some(Py::Int(_) | Py::Float(_)))
}

fn num_str(v: &Py) -> String {
    py_str(v)
}

/// `_num`: a number (not a bool), else the error.
fn check_num(section: &Py, key: &str, errors: &mut Vec<String>, where_: &str) {
    let v = section.get(key);
    if !is_number(v) {
        errors.push(format!("{where_}.{key} 必須是數字（目前：{}）", py_repr(v.unwrap_or(&Py::None))));
    }
}

fn as_f64(v: &Py) -> f64 {
    match v {
        Py::Int(i) => *i as f64,
        Py::Float(f) => *f,
        _ => f64::NAN,
    }
}

/// `validate(ext)`: (errors, warnings); errors make the file unusable. The
/// texts are the Python's.
pub fn validate(ext: &Py) -> (Vec<String>, Vec<String>) {
    let (mut errors, mut warnings) = (Vec::new(), Vec::new());
    let Py::Dict(_) = ext else {
        return (vec!["extrinsics YAML 不是 mapping".into()], vec![]);
    };
    match ext.get("schema") {
        None | Some(Py::None) => {}
        Some(Py::Str(s)) if s == SCHEMA => {}
        Some(other) => warnings.push(format!("schema={}，預期 {SCHEMA}", py_repr(other))),
    }
    let source_raw = ext.get("source").cloned().unwrap_or_else(|| Py::str(""));
    let source = py_strip(&py_str(&source_raw)).to_string();
    if !VALID_SOURCES.contains(&source.as_str()) {
        errors.push(format!("source 必須是 {}（目前：{}）", VALID_SOURCES.join("/"), str_repr(&source)));
    } else if source == "placeholder" {
        warnings.push("source=placeholder：外參是佔位值，必須實測後覆寫".into());
    }
    let cam = ext.get("camera");
    let empty = Py::Dict(Vec::new());
    let mut refd = match ext.get("reference") {
        Some(v) if truthy(v) => v,
        _ => &empty,
    };
    let Some(cam @ Py::Dict(_)) = cam else {
        errors.push("缺少 camera 區塊".into());
        return (errors, warnings);
    };
    for key in ["forward_of_axle_m", "left_of_center_m", "height_m", "roll_deg", "pitch_deg", "yaw_deg"] {
        check_num(cam, key, &mut errors, "camera");
    }
    if !matches!(refd, Py::Dict(_)) {
        errors.push("reference 必須是 mapping".into());
        refd = &empty;
    }
    check_num(refd, "axle_height_m", &mut errors, "reference");
    if !errors.is_empty() {
        return (errors, warnings);
    }
    let g = |k: &str| cam.get(k).unwrap();
    let h = as_f64(g("height_m"));
    if !(0.05..=2.5).contains(&h) {
        errors.push(format!("camera.height_m={} 超出合理範圍 0.05～2.5 m", num_str(g("height_m"))));
    }
    let f = as_f64(g("forward_of_axle_m"));
    if !(-1.0..=2.0).contains(&f) {
        warnings.push(format!("camera.forward_of_axle_m={} 異常", num_str(g("forward_of_axle_m"))));
    }
    let p = as_f64(g("pitch_deg"));
    if !(-5.0..=80.0).contains(&p) {
        errors.push(format!("camera.pitch_deg={} 超出 -5～80°（向下為正）", num_str(g("pitch_deg"))));
    }
    if as_f64(g("roll_deg")).abs() > 20.0 || as_f64(g("yaw_deg")).abs() > 45.0 {
        warnings.push("camera.roll_deg／yaw_deg 偏大，請再確認量測".into());
    }
    let ah = refd.get("axle_height_m").unwrap();
    let a = as_f64(ah);
    if !(0.02..=0.5).contains(&a) {
        errors.push(format!("reference.axle_height_m={} 超出 0.02～0.5 m", num_str(ah)));
    }
    match refd.get("axle_center_in_base_footprint_m") {
        None | Some(Py::None) => {}
        Some(Py::List(items)) if items.len() == 3 => {}
        Some(_) => errors.push("reference.axle_center_in_base_footprint_m 必須是 [x, y, z] 或 null".into()),
    }
    (errors, warnings)
}

/// What [`compute`] returns (the Python's result dict).
#[derive(Debug, Clone, PartialEq)]
pub struct Computed {
    /// `str(ext.get('source'))`, not stripped
    pub source: String,
    pub warnings: Vec<String>,
    pub reference_source: String,
    pub axle_center_in_base_footprint: Vec3,
    pub base_footprint_height_m: f64,
    pub camera_height_m: f64,
    pub camera_pitch_deg: f64,
    pub base_footprint_to_camera_link: Transform,
    pub base_link_to_camera_link: Transform,
    pub camera_link_to_optical: Transform,
    pub base_footprint_to_optical: Transform,
}

/// Why [`compute`] failed: the Python's exception type and text.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ComputeError {
    /// `ValueError`: validate()'s errors joined with `; `.
    Invalid(String),
    /// `TypeError` or `ValueError` from `float()` of an axle element, or
    /// `ValueError: math domain error` from an infinite angle.
    Conversion { kind: &'static str, message: String },
}

impl std::fmt::Display for ComputeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ComputeError::Invalid(m) => f.write_str(m),
            ComputeError::Conversion { message, .. } => f.write_str(message),
        }
    }
}

impl std::error::Error for ComputeError {}

/// `compute(ext, reference)`: the measured extrinsics as transforms.
/// `reference` is [`urdf_reference`]'s or [`default_reference`]'s (`None`
/// = the default); an explicit `reference.axle_center_in_base_footprint_m`
/// in the YAML wins for the axle.
pub fn compute(ext: &Py, reference: Option<&Reference>) -> Result<Computed, ComputeError> {
    let (errors, warnings) = validate(ext);
    if !errors.is_empty() {
        return Err(ComputeError::Invalid(errors.join("; ")));
    }
    let default = default_reference();
    let reference = reference.unwrap_or(&default);
    let cam = ext.get("camera").unwrap();
    let empty = Py::Dict(Vec::new());
    let refd = match ext.get("reference") {
        Some(v) if truthy(v) => v,
        _ => &empty,
    };
    let axle: Vec3 = match refd.get("axle_center_in_base_footprint_m") {
        Some(Py::List(items)) => {
            let mut a = [0.0; 3];
            for (k, v) in items.iter().enumerate() {
                a[k] = py_float(v).map_err(|message| ComputeError::Conversion {
                    kind: if matches!(v, Py::Str(_)) { "ValueError" } else { "TypeError" },
                    message,
                })?;
            }
            a
        }
        _ => reference.axle_center_in_base_footprint,
    };
    let f = |k: &str| as_f64(cam.get(k).unwrap());
    let ground_z = axle[2] - as_f64(refd.get("axle_height_m").unwrap());
    let xyz = [axle[0] + f("forward_of_axle_m"), axle[1] + f("left_of_center_m"), ground_z + f("height_m")];
    let rpy = ["roll_deg", "pitch_deg", "yaw_deg"].map(|k| radians(f(k)));
    let t_bf_cam = Transform::try_from_xyz_rpy(xyz, rpy)
        .map_err(|message| ComputeError::Conversion { kind: "ValueError", message })?;
    let t_cam_opt = Transform::from_xyz_rpy([0.0; 3], OPTICAL_RPY);
    let t_bf_bl = reference.base_link_in_base_footprint;
    Ok(Computed {
        source: py_str(ext.get("source").unwrap_or(&Py::None)),
        warnings,
        reference_source: reference.source.clone(),
        axle_center_in_base_footprint: axle,
        base_footprint_height_m: -ground_z,
        camera_height_m: f("height_m"),
        camera_pitch_deg: f("pitch_deg"),
        base_footprint_to_camera_link: t_bf_cam,
        base_link_to_camera_link: t_bf_bl.inverse().compose(&t_bf_cam),
        camera_link_to_optical: t_cam_opt,
        base_footprint_to_optical: t_bf_cam.compose(&t_cam_opt),
    })
}

/// `_round(v, n)`: a list of rounded floats.
fn round_list(v: &[f64], n: usize) -> Py {
    Py::List(v.iter().map(|x| Py::Float(py_round(*x, n))).collect())
}

/// `transform_dict(t, parent, child)`.
pub fn transform_dict(t: &Transform, parent: &str, child: &str) -> Py {
    let rpy = t.rpy();
    Py::dict([
        ("parent", Py::str(parent)),
        ("child", Py::str(child)),
        ("xyz_m", round_list(&t.translation, 6)),
        ("rpy_rad", round_list(&rpy, 6)),
        ("rpy_deg", round_list(&rpy.map(degrees), 4)),
        ("quaternion_xyzw", round_list(&t.quaternion(), 9)),
    ])
}

/// `derived_block(result)`: the summary written into
/// `<run>/calib/extrinsics.yaml`.
pub fn derived_block(r: &Computed) -> Py {
    Py::dict([
        ("note", Py::str("由 mower_recorder 依上方量測值計算；/tf_static 發布的就是這些值")),
        ("reference_source", Py::str(&r.reference_source)),
        ("axle_center_in_base_footprint_m", round_list(&r.axle_center_in_base_footprint, 6)),
        ("base_footprint_height_m", Py::Float(py_round(r.base_footprint_height_m, 6))),
        ("camera_height_m", Py::Float(py_round(r.camera_height_m, 6))),
        ("camera_pitch_deg", Py::Float(py_round(r.camera_pitch_deg, 4))),
        ("base_footprint_to_camera_front_optical_frame", transform_dict(&r.base_footprint_to_optical, BASE_FOOTPRINT, OPTICAL_FRAME)),
        ("base_footprint_to_camera_front_link", transform_dict(&r.base_footprint_to_camera_link, BASE_FOOTPRINT, CAMERA_LINK)),
        (
            "published_tf",
            Py::List(vec![
                transform_dict(&r.base_link_to_camera_link, BASE_LINK, CAMERA_LINK),
                transform_dict(&r.camera_link_to_optical, CAMERA_LINK, OPTICAL_FRAME),
            ]),
        ),
    ])
}

/// `static_tf_arguments(t, parent, child)`: the arguments of
/// `static_transform_publisher` (Jazzy), each number `repr(round(v, 9))`.
pub fn static_tf_arguments(t: &Transform, parent: &str, child: &str) -> Vec<String> {
    let [x, y, z] = t.translation;
    let [qx, qy, qz, qw] = t.quaternion();
    let mut args = Vec::new();
    for (flag, val) in [("--x", x), ("--y", y), ("--z", z), ("--qx", qx), ("--qy", qy), ("--qz", qz), ("--qw", qw)] {
        args.push(flag.to_string());
        args.push(repr::float(py_round(val, 9)));
    }
    args.extend(["--frame-id".to_string(), parent.to_string(), "--child-frame-id".to_string(), child.to_string()]);
    args
}
