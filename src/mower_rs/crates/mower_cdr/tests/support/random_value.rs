//! Random and edge values of a table type, from a generator of its own: NaN,
//! infinities, -0.0, subnormals, integer limits, empty and non-ASCII strings,
//! empty and longer arrays. `rmw_oracle.rs` compares them with rmw while r2r
//! is here; `mower_ros_facts` replays the same values, with no ROS, against
//! the bytes rmw wrote for them (`deploy/test/golden/ros_facts/cdr_vectors.txt`).
//!
//! The values are a function of this file alone, not of a random-number
//! crate's version: **changing what it generates invalidates the frozen
//! vectors**, and after r2r is gone nothing can write them again.

use mower_cdr::Value;
use mower_schema::{ArrayKind, FieldType, Primitive};

/// SplitMix64.
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Rng {
        Rng(seed)
    }

    /// The generator of value `index` of `type_name` in the frozen vectors:
    /// adding a type or a value moves no other vector.
    pub fn for_vector(type_name: &str, index: u32) -> Rng {
        // FNV-1a
        let mut h = 0xcbf2_9ce4_8422_2325u64;
        for b in type_name.bytes().chain(index.to_le_bytes()) {
            h = (h ^ b as u64).wrapping_mul(0x0000_0100_0000_01b3);
        }
        Rng(h)
    }

    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }

    fn percent(&mut self, p: u64) -> bool {
        self.next() % 100 < p
    }

    /// In `0..n`.
    fn below(&mut self, n: usize) -> usize {
        (self.next() % n as u64) as usize
    }

    fn pick<'a, T>(&mut self, xs: &'a [T]) -> &'a T {
        &xs[self.below(xs.len())]
    }
}

const STRINGS: &[&str] = &["", "a", "map", "/tf", "測試 字串", "émoji 🚀", "with space and \"quotes\"", "line\nbreak",
    "0123456789abcdef0123456789abcdef0123456789abcdef"];

fn prim(rng: &mut Rng, p: Primitive) -> Value {
    let edge = rng.percent(30);
    let bits = rng.next();
    match p {
        Primitive::Bool => Value::Bool(bits & 1 == 1),
        Primitive::Byte | Primitive::Char | Primitive::Uint8 => Value::U8(if edge { *rng.pick(&[0, 1, 255]) } else { bits as u8 }),
        Primitive::Int8 => Value::I8(if edge { *rng.pick(&[i8::MIN, -1, 0, i8::MAX]) } else { bits as i8 }),
        Primitive::Int16 => Value::I16(if edge { *rng.pick(&[i16::MIN, -1, 0, i16::MAX]) } else { bits as i16 }),
        Primitive::Uint16 => Value::U16(if edge { *rng.pick(&[0, u16::MAX]) } else { bits as u16 }),
        Primitive::Int32 => Value::I32(if edge { *rng.pick(&[i32::MIN, -1, 0, i32::MAX]) } else { bits as i32 }),
        Primitive::Uint32 => Value::U32(if edge { *rng.pick(&[0, u32::MAX]) } else { bits as u32 }),
        Primitive::Int64 => Value::I64(if edge { *rng.pick(&[i64::MIN, -1, 0, i64::MAX]) } else { bits as i64 }),
        Primitive::Uint64 => Value::U64(if edge { *rng.pick(&[0, u64::MAX]) } else { bits }),
        Primitive::Float32 => Value::F32(if edge {
            *rng.pick(&[f32::NAN, -f32::NAN, f32::INFINITY, f32::NEG_INFINITY, -0.0, f32::MIN_POSITIVE, 1e-45, f32::MAX])
        } else {
            f32::from_bits(bits as u32)
        }),
        Primitive::Float64 => Value::F64(if edge {
            *rng.pick(&[f64::NAN, f64::from_bits(0x7ff4_0000_0000_0001), f64::INFINITY, f64::NEG_INFINITY, -0.0,
                f64::MIN_POSITIVE, 5e-324, f64::MAX])
        } else if rng.percent(50) {
            (bits >> 12) as f64 * 2e6 / (1u64 << 52) as f64 - 1e6
        } else {
            f64::from_bits(bits)
        }),
    }
}

fn elem(rng: &mut Rng, ty: &FieldType, depth: u32) -> Value {
    match ty {
        FieldType::Primitive(p) => prim(rng, *p),
        FieldType::String { bound } => {
            let mut s = rng.pick(STRINGS).to_string();
            if rng.percent(20) {
                let n = rng.below(300);
                s = (0..n).map(|_| (b' ' + rng.below(95) as u8) as char).collect();
            }
            if let Some(b) = bound {
                s.truncate(*b as usize);
            }
            Value::Str(s)
        }
        FieldType::WString { .. } => unreachable!("no wstring in the table"),
        FieldType::Message(name) => random(rng, name, depth + 1),
    }
}

/// A value of `name`; `depth` 0 for a root message.
pub fn random(rng: &mut Rng, name: &str, depth: u32) -> Value {
    let def = mower_schema::message(name).unwrap();
    let mut fields = Vec::new();
    for f in &def.fields {
        let n = match f.array {
            ArrayKind::Single => {
                fields.push((f.name.clone(), elem(rng, &f.ty, depth)));
                continue;
            }
            ArrayKind::Fixed(n) => n as usize,
            ArrayKind::Bounded(b) => rng.below(b as usize + 1),
            ArrayKind::Unbounded => match &f.ty {
                FieldType::Message(_) if depth > 2 => rng.below(2),
                FieldType::Message(_) => rng.below(4),
                _ => *rng.pick(&[0, 1, 2, 7, 33]),
            },
        };
        let v = if matches!(f.ty, FieldType::Primitive(Primitive::Uint8 | Primitive::Byte | Primitive::Char)) {
            Value::Bytes((0..n).map(|_| rng.next() as u8).collect())
        } else {
            Value::Array((0..n).map(|_| elem(rng, &f.ty, depth)).collect())
        };
        fields.push((f.name.clone(), v));
    }
    Value::Struct(fields)
}
