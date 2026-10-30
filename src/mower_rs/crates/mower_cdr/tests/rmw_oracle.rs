//! mower_cdr against rmw itself (E5 design G3, the property test): for every
//! type in mower_schema's table, random and edge values (NaN, infinities,
//! -0.0, subnormals, integer limits, empty and non-ASCII strings, empty and
//! longer arrays) are made typed through `from_value`, serialized by rmw
//! (r2r's `to_serialized_bytes`, the RMW the environment selects), and must
//! equal mower_cdr's bytes padded to 4, by both ways in (serde on the typed
//! value, and the dynamic value). rmw's unpadded bytes must decode back to
//! the same value, and r2r must read mower_cdr's bytes back.
//!
//! The RMW is the robot's, rmw_cyclonedds_cpp, whatever the environment
//! says: Fast DDS (Jazzy's default) leaves whatever its buffer held in the
//! alignment padding (bytes from an earlier string where Cyclone writes
//! zeros), so its output is not a function of the value.
//!
//! Then what r2r refuses: mower_cdr refuses the same values (fixed-array
//! length, NUL) where r2r panics.

use mower_cdr::Value;
use mower_schema::{ArrayKind, FieldType, Primitive};
use r2r::WrappedTypesupport;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use std::sync::Once;

/// rmw_implementation loads the RMW the first time a type is serialized,
/// from this variable (both tests call this first).
fn robot_rmw() {
    static ONCE: Once = Once::new();
    ONCE.call_once(|| {
        let lib = std::env::var("AMENT_PREFIX_PATH").unwrap_or_default().split(':').any(|p| {
            std::path::Path::new(p).join("lib/librmw_cyclonedds_cpp.so").exists()
        });
        assert!(lib, "the oracle runs on the robot's RMW: apt-get install ros-jazzy-rmw-cyclonedds-cpp");
        std::env::set_var("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp");
    });
}

const STRINGS: &[&str] = &["", "a", "map", "/tf", "測試 字串", "émoji 🚀", "with space and \"quotes\"", "line\nbreak",
    "0123456789abcdef0123456789abcdef0123456789abcdef"];

fn prim(rng: &mut StdRng, p: Primitive) -> Value {
    let edge = rng.gen_bool(0.3);
    match p {
        Primitive::Bool => Value::Bool(rng.gen()),
        Primitive::Byte | Primitive::Char | Primitive::Uint8 => Value::U8(if edge { *pick(rng, &[0, 1, 255]) } else { rng.gen() }),
        Primitive::Int8 => Value::I8(if edge { *pick(rng, &[i8::MIN, -1, 0, i8::MAX]) } else { rng.gen() }),
        Primitive::Int16 => Value::I16(if edge { *pick(rng, &[i16::MIN, -1, 0, i16::MAX]) } else { rng.gen() }),
        Primitive::Uint16 => Value::U16(if edge { *pick(rng, &[0, u16::MAX]) } else { rng.gen() }),
        Primitive::Int32 => Value::I32(if edge { *pick(rng, &[i32::MIN, -1, 0, i32::MAX]) } else { rng.gen() }),
        Primitive::Uint32 => Value::U32(if edge { *pick(rng, &[0, u32::MAX]) } else { rng.gen() }),
        Primitive::Int64 => Value::I64(if edge { *pick(rng, &[i64::MIN, -1, 0, i64::MAX]) } else { rng.gen() }),
        Primitive::Uint64 => Value::U64(if edge { *pick(rng, &[0, u64::MAX]) } else { rng.gen() }),
        Primitive::Float32 => Value::F32(if edge {
            *pick(rng, &[f32::NAN, -f32::NAN, f32::INFINITY, f32::NEG_INFINITY, -0.0, f32::MIN_POSITIVE, 1e-45, f32::MAX])
        } else {
            f32::from_bits(rng.gen())
        }),
        Primitive::Float64 => Value::F64(if edge {
            *pick(rng, &[f64::NAN, f64::from_bits(0x7ff4_0000_0000_0001), f64::INFINITY, f64::NEG_INFINITY, -0.0,
                f64::MIN_POSITIVE, 5e-324, f64::MAX])
        } else if rng.gen_bool(0.5) {
            rng.gen_range(-1e6..1e6)
        } else {
            f64::from_bits(rng.gen())
        }),
    }
}

fn pick<'a, T>(rng: &mut StdRng, xs: &'a [T]) -> &'a T {
    &xs[rng.gen_range(0..xs.len())]
}

fn elem(rng: &mut StdRng, ty: &FieldType, depth: u32) -> Value {
    match ty {
        FieldType::Primitive(p) => prim(rng, *p),
        FieldType::String { bound } => {
            let mut s = pick(rng, STRINGS).to_string();
            if rng.gen_bool(0.2) {
                s = (0..rng.gen_range(0..300)).map(|_| rng.gen_range(b' '..=b'~') as char).collect();
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

fn random(rng: &mut StdRng, name: &str, depth: u32) -> Value {
    let def = mower_schema::message(name).unwrap();
    let mut fields = Vec::new();
    for f in &def.fields {
        let n = match f.array {
            ArrayKind::Single => {
                fields.push((f.name.clone(), elem(rng, &f.ty, depth)));
                continue;
            }
            ArrayKind::Fixed(n) => n as usize,
            ArrayKind::Bounded(b) => rng.gen_range(0..=b as usize),
            ArrayKind::Unbounded => match &f.ty {
                FieldType::Message(_) if depth > 2 => rng.gen_range(0..=1),
                FieldType::Message(_) => rng.gen_range(0..=3),
                _ => *pick(rng, &[0, 1, 2, 7, 33]),
            },
        };
        let v = if matches!(f.ty, FieldType::Primitive(Primitive::Uint8 | Primitive::Byte | Primitive::Char)) {
            Value::Bytes((0..n).map(|_| rng.gen()).collect())
        } else {
            Value::Array((0..n).map(|_| elem(rng, &f.ty, depth)).collect())
        };
        fields.push((f.name.clone(), v));
    }
    Value::Struct(fields)
}

fn check<T>(name: &str, rng: &mut StdRng, n: usize)
where
    T: WrappedTypesupport + serde::Serialize + serde::de::DeserializeOwned,
{
    for i in 0..n {
        let v = random(rng, name, 0);
        let ctx = || format!("{name} #{i}: {v:?}");
        let typed: T = mower_cdr::from_value(&v).unwrap_or_else(|e| panic!("{}: from_value: {e}", ctx()));
        let rmw = typed.to_serialized_bytes().unwrap();
        let want = mower_cdr::pad4(rmw.clone());
        let via_serde = mower_cdr::to_vec(name, &typed).unwrap_or_else(|e| panic!("{}: to_vec: {e}", ctx()));
        assert!(via_serde == want, "{}: serde path\nrmw  {:02x?}\nours {:02x?}", ctx(), want, via_serde);
        let via_value = mower_cdr::encode(name, &v).unwrap_or_else(|e| panic!("{}: encode: {e}", ctx()));
        assert!(via_value == want, "{}: value path", ctx());
        let back = mower_cdr::decode(name, &rmw).unwrap_or_else(|e| panic!("{}: decode: {e}", ctx()));
        assert!(back == v, "{}: decoded {back:?}", ctx());
        let again = T::from_serialized_bytes(&via_serde).unwrap();
        assert!(again.to_serialized_bytes().unwrap() == rmw, "{}: r2r read it back differently", ctx());
    }
}

macro_rules! oracle {
    ($rng:ident, $n:expr; $($t:path => $name:literal),* $(,)?) => {{
        let names: Vec<&str> = vec![$($name),*];
        $( check::<$t>($name, &mut $rng, $n); )*
        names
    }};
}

#[test]
fn mower_cdr_writes_what_rmw_writes_for_every_table_type() {
    robot_rmw();
    let mut rng = StdRng::seed_from_u64(0x0e5_cd2);
    let covered = oracle!(rng, 300;
        r2r::builtin_interfaces::msg::Duration => "builtin_interfaces/msg/Duration",
        r2r::builtin_interfaces::msg::Time => "builtin_interfaces/msg/Time",
        r2r::diagnostic_msgs::msg::DiagnosticArray => "diagnostic_msgs/msg/DiagnosticArray",
        r2r::diagnostic_msgs::msg::DiagnosticStatus => "diagnostic_msgs/msg/DiagnosticStatus",
        r2r::diagnostic_msgs::msg::KeyValue => "diagnostic_msgs/msg/KeyValue",
        r2r::geometry_msgs::msg::Point => "geometry_msgs/msg/Point",
        r2r::geometry_msgs::msg::Pose => "geometry_msgs/msg/Pose",
        r2r::geometry_msgs::msg::PoseStamped => "geometry_msgs/msg/PoseStamped",
        r2r::geometry_msgs::msg::PoseWithCovariance => "geometry_msgs/msg/PoseWithCovariance",
        r2r::geometry_msgs::msg::Quaternion => "geometry_msgs/msg/Quaternion",
        r2r::geometry_msgs::msg::Transform => "geometry_msgs/msg/Transform",
        r2r::geometry_msgs::msg::TransformStamped => "geometry_msgs/msg/TransformStamped",
        r2r::geometry_msgs::msg::Twist => "geometry_msgs/msg/Twist",
        r2r::geometry_msgs::msg::TwistStamped => "geometry_msgs/msg/TwistStamped",
        r2r::geometry_msgs::msg::TwistWithCovariance => "geometry_msgs/msg/TwistWithCovariance",
        r2r::geometry_msgs::msg::Vector3 => "geometry_msgs/msg/Vector3",
        r2r::mower_interface::msg::CoverageProgress => "mower_interface/msg/CoverageProgress",
        r2r::nav_msgs::msg::MapMetaData => "nav_msgs/msg/MapMetaData",
        r2r::nav_msgs::msg::OccupancyGrid => "nav_msgs/msg/OccupancyGrid",
        r2r::nav_msgs::msg::Odometry => "nav_msgs/msg/Odometry",
        r2r::nav_msgs::msg::Path => "nav_msgs/msg/Path",
        r2r::rcl_interfaces::msg::Log => "rcl_interfaces/msg/Log",
        r2r::sensor_msgs::msg::BatteryState => "sensor_msgs/msg/BatteryState",
        r2r::sensor_msgs::msg::CameraInfo => "sensor_msgs/msg/CameraInfo",
        r2r::sensor_msgs::msg::CompressedImage => "sensor_msgs/msg/CompressedImage",
        r2r::sensor_msgs::msg::Imu => "sensor_msgs/msg/Imu",
        r2r::sensor_msgs::msg::JointState => "sensor_msgs/msg/JointState",
        r2r::sensor_msgs::msg::NavSatFix => "sensor_msgs/msg/NavSatFix",
        r2r::sensor_msgs::msg::NavSatStatus => "sensor_msgs/msg/NavSatStatus",
        r2r::sensor_msgs::msg::RegionOfInterest => "sensor_msgs/msg/RegionOfInterest",
        r2r::std_msgs::msg::Bool => "std_msgs/msg/Bool",
        r2r::std_msgs::msg::ColorRGBA => "std_msgs/msg/ColorRGBA",
        r2r::std_msgs::msg::Header => "std_msgs/msg/Header",
        r2r::std_msgs::msg::String => "std_msgs/msg/String",
        r2r::tf2_msgs::msg::TFMessage => "tf2_msgs/msg/TFMessage",
        r2r::visualization_msgs::msg::Marker => "visualization_msgs/msg/Marker",
        r2r::visualization_msgs::msg::MarkerArray => "visualization_msgs/msg/MarkerArray",
        r2r::visualization_msgs::msg::MeshFile => "visualization_msgs/msg/MeshFile",
        r2r::visualization_msgs::msg::UVCoordinate => "visualization_msgs/msg/UVCoordinate",
    );
    assert_eq!(covered, mower_schema::names(), "a table type is not checked here");
}

fn panics(f: impl FnOnce()) -> bool {
    let hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(|_| {}));
    let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)).is_err();
    std::panic::set_hook(hook);
    r
}

#[test]
fn what_r2r_refuses_mower_cdr_refuses() {
    robot_rmw();
    // a fixed array of the wrong length: r2r asserts
    let mut odom = r2r::nav_msgs::msg::Odometry::default();
    odom.pose.covariance = vec![0.0; 35];
    assert!(matches!(
        mower_cdr::to_vec("nav_msgs/msg/Odometry", &odom),
        Err(mower_cdr::Error::FixedLength { want: 36, got: 35, .. })
    ));
    assert!(panics(|| drop(odom.to_serialized_bytes())), "r2r accepted 35 of 36");
    // r2r's Default has fixed arrays at their full length
    let odom = r2r::nav_msgs::msg::Odometry::default();
    let rmw = mower_cdr::pad4(odom.to_serialized_bytes().unwrap());
    assert_eq!(mower_cdr::to_vec("nav_msgs/msg/Odometry", &odom).unwrap(), rmw);
    // a NUL in a string: r2r panics in its CString conversion
    let s = r2r::std_msgs::msg::String { data: "a\0b".into() };
    assert!(matches!(mower_cdr::to_vec("std_msgs/msg/String", &s), Err(mower_cdr::Error::Nul { .. })));
    assert!(panics(|| drop(s.to_serialized_bytes())), "r2r accepted a NUL");
    // a field the type does not have, or a type the table does not know
    assert!(matches!(mower_cdr::to_vec("std_msgs/msg/Bool", &s), Err(mower_cdr::Error::Shape { .. })));
    assert!(matches!(mower_cdr::to_vec("x_msgs/msg/Y", &s), Err(mower_cdr::Error::Schema(_))));
}
