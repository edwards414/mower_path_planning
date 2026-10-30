use mower_schema::{ArrayKind, Field, FieldType, MsgDef, Primitive};

use crate::{schema, Error, Value, HEADER};

struct Rd<'a> {
    b: &'a [u8],
    o: usize,
}

impl<'a> Rd<'a> {
    fn bad(&self, message: impl Into<String>) -> Error {
        Error::Malformed { offset: self.o, message: message.into() }
    }

    fn align(&mut self, n: usize) {
        let rel = self.o - HEADER.len();
        self.o += (n - rel % n) % n;
    }

    fn take(&mut self, n: usize) -> Result<&'a [u8], Error> {
        if self.o + n > self.b.len() {
            return Err(self.bad(format!("{n} bytes past the end")));
        }
        let s = &self.b[self.o..self.o + n];
        self.o += n;
        Ok(s)
    }

    fn prim<const N: usize>(&mut self) -> Result<[u8; N], Error> {
        self.align(N);
        Ok(self.take(N)?.try_into().unwrap())
    }

    fn u32(&mut self) -> Result<u32, Error> {
        Ok(u32::from_le_bytes(self.prim::<4>()?))
    }
}

/// Decodes `bytes` (rmw's form, padded to 4 or not) as `type_name`.
pub fn decode(type_name: &str, bytes: &[u8]) -> Result<Value, Error> {
    let def = schema(type_name)?;
    if bytes.len() < 4 || bytes[0] != 0 || bytes[1] != 1 {
        return Err(Error::Malformed { offset: 0, message: "not a little-endian CDR header".into() });
    }
    let mut rd = Rd { b: bytes, o: HEADER.len() };
    let v = struct_(&mut rd, def)?;
    let rest = &bytes[rd.o..];
    if rest.len() >= 4 || rest.iter().any(|&b| b != 0) {
        return Err(Error::Malformed { offset: rd.o, message: format!("{} bytes left over", rest.len()) });
    }
    Ok(v)
}

fn struct_(rd: &mut Rd, def: &MsgDef) -> Result<Value, Error> {
    let mut fields = Vec::with_capacity(def.fields.len());
    for f in &def.fields {
        fields.push((f.name.clone(), field(rd, f)?));
    }
    Ok(Value::Struct(fields))
}

fn field(rd: &mut Rd, f: &Field) -> Result<Value, Error> {
    let n = match f.array {
        ArrayKind::Single => return elem(rd, &f.ty),
        ArrayKind::Fixed(n) => n as usize,
        ArrayKind::Unbounded | ArrayKind::Bounded(_) => {
            let n = rd.u32()? as usize;
            if let ArrayKind::Bounded(b) = f.array {
                if n > b as usize {
                    return Err(rd.bad(format!("{}: {n} elements over the bound {b}", f.name)));
                }
            }
            n
        }
    };
    if let FieldType::Primitive(Primitive::Uint8 | Primitive::Byte | Primitive::Char) = f.ty {
        return Ok(Value::Bytes(rd.take(n)?.to_vec()));
    }
    // a count no buffer could hold is an error, not an allocation
    if n > rd.b.len() {
        return Err(rd.bad(format!("{}: {n} elements", f.name)));
    }
    let mut items = Vec::with_capacity(n);
    for _ in 0..n {
        items.push(elem(rd, &f.ty)?);
    }
    Ok(Value::Array(items))
}

fn elem(rd: &mut Rd, ty: &FieldType) -> Result<Value, Error> {
    Ok(match ty {
        FieldType::Primitive(p) => match p {
            Primitive::Bool => match rd.take(1)?[0] {
                0 => Value::Bool(false),
                1 => Value::Bool(true),
                x => return Err(rd.bad(format!("bool {x}"))),
            },
            Primitive::Uint8 | Primitive::Byte | Primitive::Char => Value::U8(rd.take(1)?[0]),
            Primitive::Int8 => Value::I8(rd.take(1)?[0] as i8),
            Primitive::Int16 => Value::I16(i16::from_le_bytes(rd.prim()?)),
            Primitive::Uint16 => Value::U16(u16::from_le_bytes(rd.prim()?)),
            Primitive::Int32 => Value::I32(i32::from_le_bytes(rd.prim()?)),
            Primitive::Uint32 => Value::U32(u32::from_le_bytes(rd.prim()?)),
            Primitive::Float32 => Value::F32(f32::from_le_bytes(rd.prim()?)),
            Primitive::Int64 => Value::I64(i64::from_le_bytes(rd.prim()?)),
            Primitive::Uint64 => Value::U64(u64::from_le_bytes(rd.prim()?)),
            Primitive::Float64 => Value::F64(f64::from_le_bytes(rd.prim()?)),
        },
        FieldType::String { bound } => {
            let n = rd.u32()? as usize;
            if n == 0 {
                return Err(rd.bad("a string without its NUL"));
            }
            let s = rd.take(n)?;
            if s[n - 1] != 0 || s[..n - 1].contains(&0) {
                return Err(rd.bad("a string not ended by its only NUL"));
            }
            if let Some(b) = bound {
                if n - 1 > *b as usize {
                    return Err(rd.bad(format!("a string over its bound {b}")));
                }
            }
            let text = std::str::from_utf8(&s[..n - 1]).map_err(|_| rd.bad("a string that is not UTF-8"))?;
            Value::Str(text.to_string())
        }
        FieldType::WString { .. } => return Err(rd.bad("wstring is not supported")),
        FieldType::Message(name) => struct_(rd, schema(name)?)?,
    })
}
