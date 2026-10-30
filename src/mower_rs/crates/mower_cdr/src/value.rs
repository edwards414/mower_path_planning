/// A message as data: what [`crate::decode`] returns and [`crate::encode`]
/// takes. Each primitive keeps its schema type, so encoding a decoded value
/// gives the same bytes, and floats keep every bit.
#[derive(Debug, Clone)]
pub enum Value {
    Bool(bool),
    I8(i8),
    U8(u8),
    I16(i16),
    U16(u16),
    I32(i32),
    U32(u32),
    I64(i64),
    U64(u64),
    F32(f32),
    F64(f64),
    Str(String),
    /// An array of `uint8`, `byte` or `char` (an image, a raw buffer).
    Bytes(Vec<u8>),
    /// Any other array, fixed or not.
    Array(Vec<Value>),
    /// A message: its fields in declaration order.
    Struct(Vec<(String, Value)>),
}

impl Value {
    /// The field `name` of a message.
    pub fn get(&self, name: &str) -> Option<&Value> {
        match self {
            Value::Struct(fields) => fields.iter().find(|(n, _)| n == name).map(|(_, v)| v),
            _ => None,
        }
    }
}

/// Equality with floats compared bit for bit (NaN equals the same NaN,
/// `0.0` differs from `-0.0`): "the same value", as a round trip must keep it.
impl PartialEq for Value {
    fn eq(&self, other: &Value) -> bool {
        use Value::*;
        match (self, other) {
            (Bool(a), Bool(b)) => a == b,
            (I8(a), I8(b)) => a == b,
            (U8(a), U8(b)) => a == b,
            (I16(a), I16(b)) => a == b,
            (U16(a), U16(b)) => a == b,
            (I32(a), I32(b)) => a == b,
            (U32(a), U32(b)) => a == b,
            (I64(a), I64(b)) => a == b,
            (U64(a), U64(b)) => a == b,
            (F32(a), F32(b)) => a.to_bits() == b.to_bits(),
            (F64(a), F64(b)) => a.to_bits() == b.to_bits(),
            (Str(a), Str(b)) => a == b,
            (Bytes(a), Bytes(b)) => a == b,
            (Array(a), Array(b)) => a == b,
            (Struct(a), Struct(b)) => a == b,
            _ => false,
        }
    }
}
