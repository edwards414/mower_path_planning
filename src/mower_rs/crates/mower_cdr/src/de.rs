//! A serde `Deserializer` over a [`Value`]: a decoded message becomes a
//! typed one (an r2r type) without a trip through JSON, so NaN and every
//! other float bit survive (E8 design 5.2).

use serde::de::{self, DeserializeOwned, DeserializeSeed, IntoDeserializer, MapAccess, SeqAccess, Visitor};

use crate::{Error, Value};

/// Builds a typed value from a [`Value`] (as [`crate::decode`] returns it).
pub fn from_value<T: DeserializeOwned>(value: &Value) -> Result<T, Error> {
    T::deserialize(ValueDe(value))
}

struct ValueDe<'a>(&'a Value);

impl<'de, 'a> de::Deserializer<'de> for ValueDe<'a> {
    type Error = Error;

    fn deserialize_any<V: Visitor<'de>>(self, v: V) -> Result<V::Value, Error> {
        match self.0 {
            Value::Bool(x) => v.visit_bool(*x),
            Value::I8(x) => v.visit_i8(*x),
            Value::U8(x) => v.visit_u8(*x),
            Value::I16(x) => v.visit_i16(*x),
            Value::U16(x) => v.visit_u16(*x),
            Value::I32(x) => v.visit_i32(*x),
            Value::U32(x) => v.visit_u32(*x),
            Value::I64(x) => v.visit_i64(*x),
            Value::U64(x) => v.visit_u64(*x),
            Value::F32(x) => v.visit_f32(*x),
            Value::F64(x) => v.visit_f64(*x),
            Value::Str(s) => v.visit_str(s),
            Value::Bytes(b) => v.visit_seq(BytesSeq(b.iter())),
            Value::Array(items) => v.visit_seq(ItemsSeq(items.iter())),
            Value::Struct(fields) => v.visit_map(Fields { it: fields.iter(), next: None }),
        }
    }

    // a message's empty struct (r2r leaves out rosidl's one member) is still
    // a map, with that member as an unknown key serde skips
    serde::forward_to_deserialize_any! {
        bool i8 i16 i32 i64 i128 u8 u16 u32 u64 u128 f32 f64 char str string bytes byte_buf option unit
        unit_struct newtype_struct seq tuple tuple_struct map struct enum identifier ignored_any
    }
}

struct BytesSeq<'a>(std::slice::Iter<'a, u8>);

impl<'de, 'a> SeqAccess<'de> for BytesSeq<'a> {
    type Error = Error;
    fn next_element_seed<T: DeserializeSeed<'de>>(&mut self, seed: T) -> Result<Option<T::Value>, Error> {
        match self.0.next() {
            Some(b) => seed.deserialize((*b).into_deserializer()).map(Some),
            None => Ok(None),
        }
    }
    fn size_hint(&self) -> Option<usize> {
        Some(self.0.len())
    }
}

struct ItemsSeq<'a>(std::slice::Iter<'a, Value>);

impl<'de, 'a> SeqAccess<'de> for ItemsSeq<'a> {
    type Error = Error;
    fn next_element_seed<T: DeserializeSeed<'de>>(&mut self, seed: T) -> Result<Option<T::Value>, Error> {
        match self.0.next() {
            Some(v) => seed.deserialize(ValueDe(v)).map(Some),
            None => Ok(None),
        }
    }
    fn size_hint(&self) -> Option<usize> {
        Some(self.0.len())
    }
}

struct Fields<'a> {
    it: std::slice::Iter<'a, (String, Value)>,
    next: Option<&'a Value>,
}

impl<'de, 'a> MapAccess<'de> for Fields<'a> {
    type Error = Error;
    fn next_key_seed<K: DeserializeSeed<'de>>(&mut self, seed: K) -> Result<Option<K::Value>, Error> {
        match self.it.next() {
            Some((name, v)) => {
                self.next = Some(v);
                seed.deserialize(name.as_str().into_deserializer()).map(Some)
            }
            None => Ok(None),
        }
    }
    fn next_value_seed<V: DeserializeSeed<'de>>(&mut self, seed: V) -> Result<V::Value, Error> {
        let v = self.next.take().ok_or_else(|| de::Error::custom("a value without its key"))?;
        seed.deserialize(ValueDe(v))
    }
}
