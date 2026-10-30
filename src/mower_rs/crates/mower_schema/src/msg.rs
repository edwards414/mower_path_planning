//! A `.msg` file as rosidl reads it (rosidl_adapter's `parser.py`,
//! `parse_message_string`): one field or constant per line, `#` starts a
//! comment, a field may carry a default value, a constant has `=`. Only what a
//! CDR layout needs is kept: the fields in order, each with its type, array
//! kind and bounds.

use std::fmt;

/// A primitive (non-string) field type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Primitive {
    Bool,
    /// `byte` (rosidl `octet`)
    Byte,
    /// `char` (rosidl `uint8`)
    Char,
    Float32,
    Float64,
    Int8,
    Uint8,
    Int16,
    Uint16,
    Int32,
    Uint32,
    Int64,
    Uint64,
}

impl Primitive {
    fn parse(s: &str) -> Option<Primitive> {
        Some(match s {
            "bool" => Primitive::Bool,
            "byte" => Primitive::Byte,
            "char" => Primitive::Char,
            "float32" => Primitive::Float32,
            "float64" => Primitive::Float64,
            "int8" => Primitive::Int8,
            "uint8" => Primitive::Uint8,
            "int16" => Primitive::Int16,
            "uint16" => Primitive::Uint16,
            "int32" => Primitive::Int32,
            "uint32" => Primitive::Uint32,
            "int64" => Primitive::Int64,
            "uint64" => Primitive::Uint64,
            _ => return None,
        })
    }

    /// Size in bytes, which is also its CDR alignment.
    pub fn size(self) -> usize {
        match self {
            Primitive::Bool | Primitive::Byte | Primitive::Char | Primitive::Int8 | Primitive::Uint8 => 1,
            Primitive::Int16 | Primitive::Uint16 => 2,
            Primitive::Float32 | Primitive::Int32 | Primitive::Uint32 => 4,
            Primitive::Float64 | Primitive::Int64 | Primitive::Uint64 => 8,
        }
    }

    /// The `.msg` spelling.
    pub fn name(self) -> &'static str {
        match self {
            Primitive::Bool => "bool",
            Primitive::Byte => "byte",
            Primitive::Char => "char",
            Primitive::Float32 => "float32",
            Primitive::Float64 => "float64",
            Primitive::Int8 => "int8",
            Primitive::Uint8 => "uint8",
            Primitive::Int16 => "int16",
            Primitive::Uint16 => "uint16",
            Primitive::Int32 => "int32",
            Primitive::Uint32 => "uint32",
            Primitive::Int64 => "int64",
            Primitive::Uint64 => "uint64",
        }
    }
}

/// The element type of a field.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FieldType {
    Primitive(Primitive),
    /// `string` or `string<=N`
    String { bound: Option<u32> },
    /// `wstring` or `wstring<=N`
    WString { bound: Option<u32> },
    /// Another message, `pkg/msg/Type`.
    Message(String),
}

/// Single value, `T[N]`, `T[]` or `T[<=N]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArrayKind {
    Single,
    Fixed(u32),
    Unbounded,
    Bounded(u32),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Field {
    pub name: String,
    pub ty: FieldType,
    pub array: ArrayKind,
}

/// A message type's fields, in declaration order (the CDR order).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MsgDef {
    /// `pkg/msg/Type`
    pub name: String,
    pub fields: Vec<Field>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParseError {
    pub type_name: String,
    /// 1-based, 0 when not about a line.
    pub line: usize,
    pub message: String,
}

impl ParseError {
    pub(crate) fn new(type_name: &str, line: usize, message: &str) -> ParseError {
        ParseError { type_name: type_name.to_string(), line, message: message.to_string() }
    }
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.line > 0 {
            write!(f, "{} line {}: {}", self.type_name, self.line, self.message)
        } else {
            write!(f, "{}: {}", self.type_name, self.message)
        }
    }
}

impl std::error::Error for ParseError {}

fn parse_bound(s: &str) -> Option<u32> {
    s.strip_prefix("<=")?.parse().ok()
}

fn is_name(s: &str) -> bool {
    let b = s.as_bytes();
    !b.is_empty() && b[0].is_ascii_alphabetic() && b.iter().all(|c| c.is_ascii_alphanumeric() || *c == b'_')
}

/// Parses a `.msg` text. `type_name` is `pkg/msg/Type`; a bare field type
/// names a message of the same package.
pub fn parse_msg(type_name: &str, text: &str) -> Result<MsgDef, ParseError> {
    let pkg = type_name.split('/').next().unwrap_or("");
    let err = |line: usize, m: &str| ParseError::new(type_name, line, m);
    let mut fields = Vec::new();
    for (n, raw) in text.lines().enumerate() {
        let line_no = n + 1;
        let line = match raw.find('#') {
            Some(i) => &raw[..i],
            None => raw,
        };
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let (type_str, rest) = match line.find(|c: char| c.is_whitespace()) {
            Some(i) => (&line[..i], line[i..].trim_start()),
            None => return Err(err(line_no, "a type without a name")),
        };
        // a constant: `TYPE NAME=VALUE`, nothing on the wire
        let name_end = rest.find(|c: char| c.is_whitespace() || c == '=').unwrap_or(rest.len());
        if rest[name_end..].trim_start().starts_with('=') {
            continue;
        }
        let name = &rest[..name_end];
        if !is_name(name) {
            return Err(err(line_no, &format!("bad field name {name:?}")));
        }
        let (base, array) = match type_str.find('[') {
            Some(i) => {
                let inner = type_str[i + 1..]
                    .strip_suffix(']')
                    .ok_or_else(|| err(line_no, "unterminated array"))?;
                let kind = if inner.is_empty() {
                    ArrayKind::Unbounded
                } else if let Some(b) = parse_bound(inner) {
                    ArrayKind::Bounded(b)
                } else {
                    ArrayKind::Fixed(inner.parse().map_err(|_| err(line_no, "bad array size"))?)
                };
                (&type_str[..i], kind)
            }
            None => (type_str, ArrayKind::Single),
        };
        let ty = if let Some(p) = Primitive::parse(base) {
            FieldType::Primitive(p)
        } else if base == "string" {
            FieldType::String { bound: None }
        } else if base == "wstring" {
            FieldType::WString { bound: None }
        } else if let Some(b) = base.strip_prefix("string") {
            FieldType::String { bound: Some(parse_bound(b).ok_or_else(|| err(line_no, "bad string bound"))?) }
        } else if let Some(b) = base.strip_prefix("wstring") {
            FieldType::WString { bound: Some(parse_bound(b).ok_or_else(|| err(line_no, "bad wstring bound"))?) }
        } else {
            let full = match base.split('/').collect::<Vec<_>>().as_slice() {
                [t] => format!("{pkg}/msg/{t}"),
                [p, t] => format!("{p}/msg/{t}"),
                [p, "msg", t] => format!("{p}/msg/{t}"),
                _ => return Err(err(line_no, &format!("bad type {base:?}"))),
            };
            FieldType::Message(full)
        };
        fields.push(Field { name: name.to_string(), ty, array });
    }
    if fields.is_empty() {
        // rosidl gives an empty structure one member (C and DDS need one)
        fields.push(Field {
            name: "structure_needs_at_least_one_member".to_string(),
            ty: FieldType::Primitive(Primitive::Uint8),
            array: ArrayKind::Single,
        });
    }
    Ok(MsgDef { name: type_name.to_string(), fields })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fields_constants_defaults_and_arrays() {
        let text = "# Header\nuint8 OK=0 # ok\nstring NAME = \"a # b\"\nstd_msgs/Header header\nfloat64 w 1\n\
                    float64[36] covariance\nPoint[] points\nint32[<=3] few\nstring<=8 short\nbyte[] data\n";
        let d = parse_msg("geometry_msgs/msg/X", text).unwrap();
        let f: Vec<(&str, &FieldType, ArrayKind)> = d.fields.iter().map(|f| (f.name.as_str(), &f.ty, f.array)).collect();
        assert_eq!(f, [
            ("header", &FieldType::Message("std_msgs/msg/Header".into()), ArrayKind::Single),
            ("w", &FieldType::Primitive(Primitive::Float64), ArrayKind::Single),
            ("covariance", &FieldType::Primitive(Primitive::Float64), ArrayKind::Fixed(36)),
            ("points", &FieldType::Message("geometry_msgs/msg/Point".into()), ArrayKind::Unbounded),
            ("few", &FieldType::Primitive(Primitive::Int32), ArrayKind::Bounded(3)),
            ("short", &FieldType::String { bound: Some(8) }, ArrayKind::Single),
            ("data", &FieldType::Primitive(Primitive::Byte), ArrayKind::Unbounded),
        ]);
    }

    #[test]
    fn an_empty_message_gets_rosidl_s_member() {
        let d = parse_msg("std_msgs/msg/Empty", "# nothing\n").unwrap();
        assert_eq!(d.fields.len(), 1);
        assert_eq!(d.fields[0].name, "structure_needs_at_least_one_member");
    }
}
