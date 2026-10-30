use mower_schema::{ArrayKind, Field, FieldType, MsgDef, Primitive};

use crate::error::{join, shape};
use crate::{schema, Error, Value, HEADER};

/// The output buffer: the encapsulation header, then the body, aligned from
/// the byte after the header.
pub(crate) struct Buf {
    pub out: Vec<u8>,
}

impl Buf {
    pub fn new() -> Buf {
        Buf { out: HEADER.to_vec() }
    }

    pub fn align(&mut self, n: usize) {
        while (self.out.len() - HEADER.len()) % n != 0 {
            self.out.push(0);
        }
    }

    pub fn u32(&mut self, v: u32) {
        self.align(4);
        self.out.extend_from_slice(&v.to_le_bytes());
    }

    /// A string: its length with the NUL, the bytes, the NUL.
    pub fn string(&mut self, path: &str, s: &str, bound: Option<u32>) -> Result<(), Error> {
        if s.as_bytes().contains(&0) {
            return Err(Error::Nul { path: path.to_string() });
        }
        if let Some(b) = bound {
            if s.len() > b as usize {
                return Err(Error::Bound { path: path.to_string(), bound: b as usize, got: s.len() });
            }
        }
        self.u32(s.len() as u32 + 1);
        self.out.extend_from_slice(s.as_bytes());
        self.out.push(0);
        Ok(())
    }

    /// A primitive's little-endian bytes, aligned to its size.
    pub fn prim_bytes(&mut self, size: usize, bytes: &[u8]) {
        self.align(size);
        self.out.extend_from_slice(bytes);
    }

    pub fn finish(self) -> Vec<u8> {
        crate::pad4(self.out)
    }
}

/// Checks an array's length against its kind and writes the count of a
/// sequence. A single value is not an array (the caller's error).
pub(crate) fn array_header(buf: &mut Buf, path: &str, kind: ArrayKind, len: usize) -> Result<(), Error> {
    match kind {
        ArrayKind::Single => Err(shape(path, "an array where one value belongs")),
        ArrayKind::Fixed(n) => {
            if len != n as usize {
                return Err(Error::FixedLength { path: path.to_string(), want: n as usize, got: len });
            }
            Ok(())
        }
        ArrayKind::Bounded(b) if len > b as usize => {
            Err(Error::Bound { path: path.to_string(), bound: b as usize, got: len })
        }
        ArrayKind::Bounded(_) | ArrayKind::Unbounded => {
            buf.u32(len as u32);
            Ok(())
        }
    }
}

/// Serializes a [`Value`] of `type_name` (`pkg/msg/Type`), padded to 4.
pub fn encode(type_name: &str, value: &Value) -> Result<Vec<u8>, Error> {
    let def = schema(type_name)?;
    let mut buf = Buf::new();
    encode_struct(&mut buf, "", def, value)?;
    Ok(buf.finish())
}

fn encode_struct(buf: &mut Buf, path: &str, def: &MsgDef, value: &Value) -> Result<(), Error> {
    let Value::Struct(fields) = value else {
        return Err(shape(path, format!("{} must be a message", def.name)));
    };
    if fields.len() != def.fields.len() {
        return Err(shape(path, format!("{} has {} fields, the value {}", def.name, def.fields.len(), fields.len())));
    }
    for (f, (name, v)) in def.fields.iter().zip(fields) {
        let p = join(path, &f.name);
        if *name != f.name {
            return Err(shape(&p, format!("found field {name:?}")));
        }
        encode_field(buf, &p, f, v)?;
    }
    Ok(())
}

fn encode_field(buf: &mut Buf, path: &str, f: &Field, v: &Value) -> Result<(), Error> {
    if f.array == ArrayKind::Single {
        return encode_elem(buf, path, &f.ty, v);
    }
    match v {
        Value::Bytes(b) => {
            if !matches!(f.ty, FieldType::Primitive(Primitive::Uint8 | Primitive::Byte | Primitive::Char)) {
                return Err(shape(path, "bytes for an array that is not of uint8, byte or char"));
            }
            array_header(buf, path, f.array, b.len())?;
            buf.out.extend_from_slice(b);
            Ok(())
        }
        Value::Array(items) => {
            array_header(buf, path, f.array, items.len())?;
            for (i, item) in items.iter().enumerate() {
                encode_elem(buf, &format!("{path}[{i}]"), &f.ty, item)?;
            }
            Ok(())
        }
        _ => Err(shape(path, "not an array")),
    }
}

fn encode_elem(buf: &mut Buf, path: &str, ty: &FieldType, v: &Value) -> Result<(), Error> {
    match (ty, v) {
        (FieldType::Primitive(p), v) => {
            let ok = match (p, v) {
                (Primitive::Bool, Value::Bool(b)) => {
                    buf.out.push(*b as u8);
                    true
                }
                (Primitive::Uint8 | Primitive::Byte | Primitive::Char, Value::U8(x)) => {
                    buf.out.push(*x);
                    true
                }
                (Primitive::Int8, Value::I8(x)) => {
                    buf.out.push(*x as u8);
                    true
                }
                (Primitive::Int16, Value::I16(x)) => prim(buf, &x.to_le_bytes()),
                (Primitive::Uint16, Value::U16(x)) => prim(buf, &x.to_le_bytes()),
                (Primitive::Int32, Value::I32(x)) => prim(buf, &x.to_le_bytes()),
                (Primitive::Uint32, Value::U32(x)) => prim(buf, &x.to_le_bytes()),
                (Primitive::Float32, Value::F32(x)) => prim(buf, &x.to_le_bytes()),
                (Primitive::Int64, Value::I64(x)) => prim(buf, &x.to_le_bytes()),
                (Primitive::Uint64, Value::U64(x)) => prim(buf, &x.to_le_bytes()),
                (Primitive::Float64, Value::F64(x)) => prim(buf, &x.to_le_bytes()),
                _ => false,
            };
            if ok {
                Ok(())
            } else {
                Err(shape(path, format!("expected a {}", p.name())))
            }
        }
        (FieldType::String { bound }, Value::Str(s)) => buf.string(path, s, *bound),
        (FieldType::String { .. }, _) => Err(shape(path, "expected a string")),
        (FieldType::WString { .. }, _) => Err(shape(path, "wstring is not supported")),
        (FieldType::Message(name), v) => encode_struct(buf, path, schema(name)?, v),
    }
}

fn prim(buf: &mut Buf, bytes: &[u8]) -> bool {
    buf.prim_bytes(bytes.len(), bytes);
    true
}
