//! A message as the bridge sends it: what `serde_json::to_value` makes of r2r's
//! typed value, from `mower_schema`'s fields alone. `f32` widened to `f64`, NaN and
//! the infinities `null`, an array of bytes a JSON array, keys sorted (a
//! `serde_json::Value` sorts them).

use mower_cdr::Value;
use serde_json::{json, Map, Number, Value as Json};

fn number(x: f64) -> Json {
    Number::from_f64(x).map(Json::Number).unwrap_or(Json::Null)
}

pub fn to_json(value: &Value) -> Json {
    match value {
        Value::Bool(v) => json!(v),
        Value::I8(v) => json!(v),
        Value::U8(v) => json!(v),
        Value::I16(v) => json!(v),
        Value::U16(v) => json!(v),
        Value::I32(v) => json!(v),
        Value::U32(v) => json!(v),
        Value::I64(v) => json!(v),
        Value::U64(v) => json!(v),
        Value::F32(v) => number(*v as f64),
        Value::F64(v) => number(*v),
        Value::Str(v) => json!(v),
        Value::Bytes(v) => json!(v),
        Value::Array(v) => Json::Array(v.iter().map(to_json).collect()),
        Value::Struct(fields) => {
            let mut map = Map::new();
            for (name, v) in fields {
                map.insert(name.clone(), to_json(v));
            }
            Json::Object(map)
        }
    }
}
