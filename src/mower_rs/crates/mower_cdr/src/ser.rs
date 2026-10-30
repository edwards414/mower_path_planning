//! A serde `Serializer` that writes CDR, walking the schema alongside the
//! value: serde visits a struct's fields in declaration order, which is the
//! `.msg` order, so the schema says at each step whether a `Vec` is a
//! sequence (count first) or a fixed array (no count) and what bounds hold.
//! Field names are checked against the schema's (r2r renames Rust keywords
//! back with `#[serde(rename)]`).

use mower_schema::{ArrayKind, Field, FieldType, MsgDef, Primitive};
use serde::ser::{self, Impossible, Serialize};

use crate::encode::{array_header, Buf};
use crate::error::{join, shape};
use crate::{schema, Error};

/// Serializes a typed message (an r2r type, or any `Serialize` with the
/// same fields) as `type_name` (`pkg/msg/Type`), padded to 4.
pub fn to_vec<T: Serialize + ?Sized>(type_name: &str, value: &T) -> Result<Vec<u8>, Error> {
    let def = schema(type_name)?;
    let mut buf = Buf::new();
    value.serialize(Ser { buf: &mut buf, path: String::new(), expect: Expect::Struct(def) })?;
    Ok(buf.finish())
}

#[derive(Clone, Copy)]
enum Expect<'s> {
    Struct(&'s MsgDef),
    Field(&'s Field),
    Elem(&'s FieldType),
}

struct Ser<'b, 's> {
    buf: &'b mut Buf,
    path: String,
    expect: Expect<'s>,
}

impl<'b, 's> Ser<'b, 's> {
    /// The element type of a single value.
    fn elem(&self) -> Result<&'s FieldType, Error> {
        match self.expect {
            Expect::Elem(t) => Ok(t),
            Expect::Field(f) if f.array == ArrayKind::Single => Ok(&f.ty),
            Expect::Field(_) => Err(shape(&self.path, "one value where an array belongs")),
            Expect::Struct(d) => Err(shape(&self.path, format!("a primitive where {} belongs", d.name))),
        }
    }

    fn prim(self, p: Primitive, bytes: &[u8]) -> Result<(), Error> {
        let ok = match self.elem()? {
            FieldType::Primitive(q) => *q == p || matches!((p, q), (Primitive::Uint8, Primitive::Byte | Primitive::Char)),
            _ => false,
        };
        if !ok {
            return Err(shape(&self.path, format!("a {} does not belong here", p.name())));
        }
        self.buf.prim_bytes(bytes.len(), bytes);
        Ok(())
    }

    fn message(&self) -> Result<&'s MsgDef, Error> {
        match self.expect {
            Expect::Struct(d) => Ok(d),
            _ => match self.elem()? {
                FieldType::Message(name) => schema(name),
                _ => Err(shape(&self.path, "a message where a primitive belongs")),
            },
        }
    }

    fn seq(self, len: Option<usize>) -> Result<SerSeq<'b, 's>, Error> {
        let Expect::Field(f) = self.expect else {
            return Err(shape(&self.path, "an array where one value belongs"));
        };
        let len = len.ok_or_else(|| shape(&self.path, "an array of unknown length"))?;
        array_header(self.buf, &self.path, f.array, len)?;
        Ok(SerSeq { buf: self.buf, path: self.path, ty: &f.ty, i: 0 })
    }
}

/// rosidl's member of an empty message, which r2r leaves out of the struct.
fn synthetic(f: &Field) -> bool {
    f.name == "structure_needs_at_least_one_member"
}

macro_rules! no {
    ($($f:ident($($t:ty),*)),* $(,)?) => {$(
        fn $f(self, $(_: $t),*) -> Result<Self::Ok, Error> {
            Err(shape(&self.path, concat!("unsupported: ", stringify!($f))))
        }
    )*};
}

impl<'b, 's> ser::Serializer for Ser<'b, 's> {
    type Ok = ();
    type Error = Error;
    type SerializeSeq = SerSeq<'b, 's>;
    type SerializeTuple = SerSeq<'b, 's>;
    type SerializeTupleStruct = Impossible<(), Error>;
    type SerializeTupleVariant = Impossible<(), Error>;
    type SerializeMap = Impossible<(), Error>;
    type SerializeStruct = SerStruct<'b, 's>;
    type SerializeStructVariant = Impossible<(), Error>;

    fn serialize_bool(self, v: bool) -> Result<(), Error> {
        self.prim(Primitive::Bool, &[v as u8])
    }
    fn serialize_i8(self, v: i8) -> Result<(), Error> {
        self.prim(Primitive::Int8, &v.to_le_bytes())
    }
    fn serialize_i16(self, v: i16) -> Result<(), Error> {
        self.prim(Primitive::Int16, &v.to_le_bytes())
    }
    fn serialize_i32(self, v: i32) -> Result<(), Error> {
        self.prim(Primitive::Int32, &v.to_le_bytes())
    }
    fn serialize_i64(self, v: i64) -> Result<(), Error> {
        self.prim(Primitive::Int64, &v.to_le_bytes())
    }
    fn serialize_u8(self, v: u8) -> Result<(), Error> {
        self.prim(Primitive::Uint8, &[v])
    }
    fn serialize_u16(self, v: u16) -> Result<(), Error> {
        self.prim(Primitive::Uint16, &v.to_le_bytes())
    }
    fn serialize_u32(self, v: u32) -> Result<(), Error> {
        self.prim(Primitive::Uint32, &v.to_le_bytes())
    }
    fn serialize_u64(self, v: u64) -> Result<(), Error> {
        self.prim(Primitive::Uint64, &v.to_le_bytes())
    }
    fn serialize_f32(self, v: f32) -> Result<(), Error> {
        self.prim(Primitive::Float32, &v.to_le_bytes())
    }
    fn serialize_f64(self, v: f64) -> Result<(), Error> {
        self.prim(Primitive::Float64, &v.to_le_bytes())
    }
    fn serialize_str(self, v: &str) -> Result<(), Error> {
        match self.elem()? {
            FieldType::String { bound } => self.buf.string(&self.path, v, *bound),
            _ => Err(shape(&self.path, "a string does not belong here")),
        }
    }
    fn serialize_bytes(self, v: &[u8]) -> Result<(), Error> {
        let path = self.path.clone();
        let Expect::Field(f) = self.expect else { return Err(shape(&path, "bytes where one value belongs")) };
        if !matches!(f.ty, FieldType::Primitive(Primitive::Uint8 | Primitive::Byte | Primitive::Char)) {
            return Err(shape(&path, "bytes for an array that is not of uint8, byte or char"));
        }
        array_header(self.buf, &path, f.array, v.len())?;
        self.buf.out.extend_from_slice(v);
        Ok(())
    }
    fn serialize_newtype_struct<T: Serialize + ?Sized>(self, _name: &'static str, v: &T) -> Result<(), Error> {
        v.serialize(self)
    }
    fn serialize_seq(self, len: Option<usize>) -> Result<SerSeq<'b, 's>, Error> {
        self.seq(len)
    }
    fn serialize_tuple(self, len: usize) -> Result<SerSeq<'b, 's>, Error> {
        self.seq(Some(len))
    }
    fn serialize_struct(self, _name: &'static str, _len: usize) -> Result<SerStruct<'b, 's>, Error> {
        let def = self.message()?;
        Ok(SerStruct { buf: self.buf, path: self.path, def, i: 0 })
    }
    fn serialize_unit_struct(self, _name: &'static str) -> Result<(), Error> {
        let def = self.message()?;
        SerStruct { buf: self.buf, path: self.path, def, i: 0 }.finish()
    }

    no!(serialize_char(char), serialize_none(), serialize_unit());
    fn serialize_some<T: Serialize + ?Sized>(self, _v: &T) -> Result<(), Error> {
        Err(shape(&self.path, "unsupported: Option"))
    }
    fn serialize_unit_variant(self, _: &'static str, _: u32, _: &'static str) -> Result<(), Error> {
        Err(shape(&self.path, "unsupported: enum"))
    }
    fn serialize_newtype_variant<T: Serialize + ?Sized>(
        self, _: &'static str, _: u32, _: &'static str, _: &T,
    ) -> Result<(), Error> {
        Err(shape(&self.path, "unsupported: enum"))
    }
    fn serialize_tuple_struct(self, _: &'static str, _: usize) -> Result<Self::SerializeTupleStruct, Error> {
        Err(shape(&self.path, "unsupported: tuple struct"))
    }
    fn serialize_tuple_variant(
        self, _: &'static str, _: u32, _: &'static str, _: usize,
    ) -> Result<Self::SerializeTupleVariant, Error> {
        Err(shape(&self.path, "unsupported: enum"))
    }
    fn serialize_map(self, _: Option<usize>) -> Result<Self::SerializeMap, Error> {
        Err(shape(&self.path, "unsupported: map"))
    }
    fn serialize_struct_variant(
        self, _: &'static str, _: u32, _: &'static str, _: usize,
    ) -> Result<Self::SerializeStructVariant, Error> {
        Err(shape(&self.path, "unsupported: enum"))
    }
}

pub(crate) struct SerSeq<'b, 's> {
    buf: &'b mut Buf,
    path: String,
    ty: &'s FieldType,
    i: usize,
}

impl<'b, 's> ser::SerializeSeq for SerSeq<'b, 's> {
    type Ok = ();
    type Error = Error;
    fn serialize_element<T: Serialize + ?Sized>(&mut self, v: &T) -> Result<(), Error> {
        let path = format!("{}[{}]", self.path, self.i);
        self.i += 1;
        v.serialize(Ser { buf: self.buf, path, expect: Expect::Elem(self.ty) })
    }
    fn end(self) -> Result<(), Error> {
        Ok(())
    }
}

impl<'b, 's> ser::SerializeTuple for SerSeq<'b, 's> {
    type Ok = ();
    type Error = Error;
    fn serialize_element<T: Serialize + ?Sized>(&mut self, v: &T) -> Result<(), Error> {
        ser::SerializeSeq::serialize_element(self, v)
    }
    fn end(self) -> Result<(), Error> {
        Ok(())
    }
}

pub(crate) struct SerStruct<'b, 's> {
    buf: &'b mut Buf,
    path: String,
    def: &'s MsgDef,
    i: usize,
}

impl<'b, 's> SerStruct<'b, 's> {
    fn finish(self) -> Result<(), Error> {
        match &self.def.fields[self.i..] {
            [] => Ok(()),
            [f] if self.i == 0 && synthetic(f) => {
                self.buf.out.push(0);
                Ok(())
            }
            [f, ..] => Err(shape(&join(&self.path, &f.name), format!("missing in the {} value", self.def.name))),
        }
    }
}

impl<'b, 's> ser::SerializeStruct for SerStruct<'b, 's> {
    type Ok = ();
    type Error = Error;
    fn serialize_field<T: Serialize + ?Sized>(&mut self, key: &'static str, v: &T) -> Result<(), Error> {
        let Some(f) = self.def.fields.get(self.i) else {
            return Err(shape(&join(&self.path, key), format!("not a field of {}", self.def.name)));
        };
        let path = join(&self.path, &f.name);
        if f.name != key {
            return Err(shape(&path, format!("the value has {key:?} here")));
        }
        self.i += 1;
        v.serialize(Ser { buf: self.buf, path, expect: Expect::Field(f) })
    }
    fn end(self) -> Result<(), Error> {
        self.finish()
    }
}
