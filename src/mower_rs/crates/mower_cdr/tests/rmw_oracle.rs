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

#[path = "support/random_value.rs"]
mod random_value;

use r2r::WrappedTypesupport;
use random_value::{random, Rng};
use std::path::{Path, PathBuf};
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

fn check<T>(name: &str, rng: &mut Rng, n: usize)
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

/// A type of mower_schema's table, typed through r2r.
struct Entry {
    name: &'static str,
    check: fn(&str, &mut Rng, usize),
    vectors: fn(&str, &mut String),
}

macro_rules! entries {
    ($($t:path => $name:literal),* $(,)?) => {
        vec![$(Entry { name: $name, check: check::<$t>, vectors: vectors_of::<$t> }),*]
    };
}

fn table() -> Vec<Entry> {
    let table = entries!(
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
    let names: Vec<&str> = table.iter().map(|e| e.name).collect();
    assert_eq!(names, mower_schema::names(), "a table type is not checked here");
    table
}

#[test]
fn mower_cdr_writes_what_rmw_writes_for_every_table_type() {
    robot_rmw();
    let mut rng = Rng::new(0x0e5_cd2);
    for entry in table() {
        (entry.check)(entry.name, &mut rng, 300);
    }
}

/// The vectors `mower_ros_facts` replays without ROS, in table order: for each
/// table type, this many values of `random_value`, the bytes rmw writes and
/// r2r's JSON of the typed value, as `serde_json::Value` writes it (what the
/// bridge sends the app). E3's corpus has hand-picked values of the types the app
/// uses; these are every type's, edge values included.
const VECTORS_PER_TYPE: u32 = 10;

fn vectors_of<T>(name: &str, text: &mut String)
where
    T: WrappedTypesupport + serde::Serialize + serde::de::DeserializeOwned,
{
    for i in 0..VECTORS_PER_TYPE {
        let v = random(&mut Rng::for_vector(name, i), name, 0);
        let typed: T = mower_cdr::from_value(&v).unwrap_or_else(|e| panic!("{name} #{i}: from_value: {e}"));
        let hex: String = typed.to_serialized_bytes().unwrap().iter().map(|b| format!("{b:02x}")).collect();
        let json = serde_json::to_value(&typed).unwrap_or_else(|e| panic!("{name} #{i}: to_value: {e}"));
        text.push_str(&format!("{name}\t{i}\t{hex}\t{json}\n"));
    }
}

fn facts(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../../deploy/test/golden/ros_facts").join(name)
}

/// What rmw writes is what `cdr_vectors.txt` holds. `MOWER_GOLDEN=write`
/// writes the file instead (tools/oracle/regen.sh facts); then
/// tools/oracle/facts_manifest.py.
#[test]
fn the_frozen_vectors_are_what_rmw_writes() {
    robot_rmw();
    let mut text = String::new();
    for entry in table() {
        (entry.vectors)(entry.name, &mut text);
    }
    let path = facts("cdr_vectors.txt");
    if std::env::var("MOWER_GOLDEN").as_deref() == Ok("write") {
        std::fs::write(&path, &text).unwrap();
        return;
    }
    let have = std::fs::read_to_string(&path).unwrap_or_default();
    for (want, have) in text.lines().zip(have.lines()) {
        assert!(want == have, "rmw no longer writes this vector of {} (tools/oracle/regen.sh facts rewrites it)", want.split('\t').next().unwrap());
    }
    assert_eq!(text.lines().count(), have.lines().count(), "{} is not this table's (tools/oracle/regen.sh facts)", path.display());
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
