//! The golden corpus of the app's JSON (step E3): for every interface type
//! the app and the Studio use, values whose frames they receive, or the
//! frames they send, with the values that make JSON byte identity hard.
//!
//! - **Where the texts come from:** mower_bus_ros's `tests/dds.rs` includes
//!   this file and runs every case through r2r's untyped path, which is what
//!   mower_ws_bridge's r2r backend sends and decodes (the frame its
//!   `fan_out` and `call_service` build around r2r's `serde_json::Value`).
//!   It checks the texts next to this file, or writes them with
//!   `MOWER_GOLDEN=write`.
//! - **Who checks them:** mower_ws_bridge's own tests include this file too,
//!   and build the same frames through the bus (`ErasedMsg::to_json`, a
//!   `JsonClient`, a `JsonPublisher`) with ws_bridge's frame functions, with
//!   no DDS. The texts must be byte identical.
//! - **The files** (one case per line: its name, a tab, the text):
//!   `publish.txt`, the frames of the messages the app receives;
//!   `service_response.txt`, the frames of the services' answers;
//!   `request.txt`, each app request as r2r decodes it (the JSON of the
//!   typed value the server gets); `inbound.txt`, each message the app
//!   publishes, as r2r decodes it. `map_grid.frame` is a real map layer, the
//!   frame mower_ws_bridge sent on main (fbf5afe) for map_manage's start-up
//!   grid through the adapter: the case `map_layer` publishes its `data`.
//! - **No NUL** in any string: r2r cannot carry one (its conversion
//!   panics), so ws_bridge's bus backend refuses it at the door instead.

use r2r::builtin_interfaces::msg::Time;
use r2r::geometry_msgs::msg::{Point, Pose, PoseStamped, Quaternion, TwistStamped};
use r2r::mower_interface::msg::CoverageProgress;
use r2r::mower_interface::srv::{
    EditZone, GetCoverageProgress, ImportImageMask, PidAutotune, ResumeCoverage, SiteOp, ZoneExecPath, ZoneSequence,
};
use r2r::rcl_interfaces::msg::SetParametersResult;
use r2r::rcl_interfaces::srv::SetParameters;
use r2r::sensor_msgs::msg::{BatteryState, NavSatFix, NavSatStatus};
use r2r::std_msgs::msg::{Bool, Header, String as Text};
use r2r::std_srvs::srv::Trigger;
use serde_json::{json, Value};

/// A message type of the corpus: an r2r type, which is also a bus message.
pub trait Msg: r2r::WrappedTypesupport + mower_bus::Message + PartialEq + std::fmt::Debug {}
impl<T: r2r::WrappedTypesupport + mower_bus::Message + PartialEq + std::fmt::Debug> Msg for T {}

/// A service type of the corpus.
pub trait Srv: r2r::WrappedServiceTypeSupport<Request: Msg, Response: Msg> + 'static {}
impl<S> Srv for S
where
    S: r2r::WrappedServiceTypeSupport + 'static,
    S::Request: Msg,
    S::Response: Msg,
{
}

/// What a corpus case is to its runner.
pub trait Corpus {
    /// The app receives `value` on `topic`.
    fn publish<T: Msg>(&mut self, case: &str, topic: &str, value: T);
    /// The app calls `service` with `args` and the call's `id`; its server
    /// answers `response`.
    fn service<S: Srv>(&mut self, case: &str, service: &str, id: &str, args: Value, response: S::Response);
    /// The app (or the Studio) publishes `frame` (the `msg` of its publish
    /// op) on `topic`.
    fn inbound<T: Msg>(&mut self, case: &str, topic: &str, frame: Value);
}

/// The text of `map_grid.frame`, a frame as the app got it.
pub const MAP_GRID_FRAME: &str = include_str!("map_grid.frame");

/// The map layer `map_grid.frame` carries.
pub fn map_layer() -> Text {
    let frame: Value = serde_json::from_str(MAP_GRID_FRAME).expect("map_grid.frame is a frame");
    Text { data: frame["msg"]["data"].as_str().expect("a String message").to_string() }
}

/// The golden text of a `publish` case that has a file of its own instead
/// of a line of `publish.txt`: `map_layer`'s is `map_grid.frame`, which is
/// only ever compared, never written.
pub fn own_file(case: &str) -> Option<&'static str> {
    (case == "map_layer").then_some(MAP_GRID_FRAME)
}

/// A golden file's content: one line per case, in corpus order, its name, a
/// tab, its text. Only the writer (mower_bus_ros) uses it.
#[allow(dead_code)]
pub fn golden_lines(cases: &[(String, String)]) -> String {
    cases.iter().map(|(case, text)| format!("{case}\t{text}\n")).collect()
}

/// What `golden_lines` wrote: case -> text.
pub fn parse_golden(file: &str) -> std::collections::BTreeMap<String, String> {
    file.lines()
        .filter_map(|line| line.split_once('\t'))
        .map(|(case, text)| (case.to_string(), text.to_string()))
        .collect()
}

/// Where `found` (case, text) differs from golden file `name`'s content
/// `have`, one line each; `name` for the messages.
pub fn golden_differences(name: &str, have: &str, found: &[(String, String)]) -> Vec<String> {
    let have = parse_golden(have);
    let mut problems = Vec::new();
    for (case, text) in found {
        match have.get(case) {
            None => problems.push(format!("{name}: no line for case {case}")),
            Some(golden) if golden != text => problems.push(format!(
                "{name}: case {case} differs at byte {}:\n  golden: {}\n  now:    {}",
                golden.bytes().zip(text.bytes()).take_while(|(a, b)| a == b).count(),
                golden.chars().take(300).collect::<String>(),
                text.chars().take(300).collect::<String>()
            )),
            Some(_) => {}
        }
    }
    for case in have.keys() {
        if !found.iter().any(|(c, _)| c == case) {
            problems.push(format!("{name}: case {case} is no case of the corpus"));
        }
    }
    problems
}

fn stamp(sec: i32, nanosec: u32) -> Time {
    Time { sec, nanosec }
}

fn header(sec: i32, nanosec: u32, frame_id: &str) -> Header {
    Header { stamp: stamp(sec, nanosec), frame_id: frame_id.into() }
}

fn point(x: f64, y: f64, z: f64) -> Point {
    Point { x, y, z }
}

fn pose(position: Point, orientation: Quaternion) -> Pose {
    Pose { position, orientation }
}

fn coverage_progress(frame_id: &str) -> CoverageProgress {
    CoverageProgress {
        header: header(1_793_000_123, 456_000_000, frame_id),
        status: 2,
        status_text: "running 割草中".into(),
        mission_id: "zone-3/2026-10-27T09:00:00Z".into(),
        zone_id: -1,
        current_segment_index: 2,
        total_segments: i32::MAX,
        completed_segments: i32::MIN,
        current_segment_progress: 0.1,
        overall_progress: 1.0 / 3.0,
        current_segment_distance_m: 12.7,
        completed_distance_m: -0.0,
        total_distance_m: f32::INFINITY,
        remaining_distance_m: f32::NAN,
        current_pose: PoseStamped {
            header: header(1_793_000_123, 0, "map"),
            pose: pose(point(1.5, -2.25, 0.0), Quaternion { x: 0.0, y: 0.0, z: 0.707_106_781_186_547_6, w: 0.707_106_781_186_547_6 }),
        },
        current_segment_start: PoseStamped::default(),
        current_segment_goal: PoseStamped { header: header(0, 0, ""), pose: pose(point(1e-300, 1e300, 5e-324), Quaternion::default()) },
        checkpoint_available: true,
        message: "line 1\nline \"2\"\t\\ end".into(),
    }
}

/// Every case, in the files' order.
pub fn corpus(c: &mut impl Corpus) {
    // --- messages the app receives -----------------------------------------
    c.publish(
        "robot_info",
        "/robot/info",
        Text {
            data: r#"{"api_version":7,"name":"割草機 MW-0CFP37","software":{"image":"main","version":"1.4.2"},"uptime_s":3600.25}"#
                .into(),
        },
    );
    // control characters, quotes, backslashes, a 4-byte character, DEL and
    // U+2028, which JSON need not escape (serde_json does not)
    c.publish(
        "string_edges",
        "/gps/status",
        Text { data: "tab\tnewline\ncr\r\u{1}\u{1f} \"q\" \\b\\ 🛰\u{7f}\u{2028}é".into() },
    );
    c.publish("string_empty", "/zone_sequence_status", Text { data: String::new() });
    c.publish("map_layer", "/adapter/map_layers/map_grid", map_layer());
    c.publish("bool_true", "/robot/online", Bool { data: true });
    c.publish("bool_false", "/robot/online", Bool { data: false });
    c.publish("command_clock", "/manual_command_clock", header(1_793_000_000, 999_999_999, "manual-7f3a9c"));
    c.publish("header_limits", "/manual_command_clock", header(i32::MIN, u32::MAX, ""));
    c.publish(
        "robot_pose",
        "/adapter/robot_pose",
        PoseStamped {
            header: header(1_793_000_000, 250_000_000, "map"),
            pose: pose(point(12.345_678_901_234_5, -0.0, 0.1), Quaternion { x: 0.0, y: 0.0, z: 1.0 / 3.0, w: 1.0 }),
        },
    );
    c.publish(
        "pose_edges",
        "/adapter/robot_pose",
        PoseStamped {
            header: header(0, 0, "odom"),
            pose: pose(point(f64::NAN, f64::INFINITY, f64::NEG_INFINITY), Quaternion { x: f64::MAX, y: f64::MIN_POSITIVE, z: 9_007_199_254_740_993.0, w: 2.0 }),
        },
    );
    c.publish(
        "battery",
        "/battery_state",
        BatteryState {
            header: header(1_793_000_000, 1, "battery"),
            voltage: 25.1,
            temperature: f32::NAN,
            current: -1.0 / 3.0,
            charge: 0.1,
            capacity: f32::NAN,
            design_capacity: 20.0,
            percentage: 0.875,
            power_supply_status: 2,
            power_supply_health: 1,
            power_supply_technology: u8::MAX,
            present: true,
            cell_voltage: vec![4.1, 4.15, 3.333_333_3, f32::NEG_INFINITY, -0.0, f32::MAX, f32::MIN_POSITIVE],
            cell_temperature: vec![],
            location: "後輪 rear".into(),
            serial_number: "".into(),
        },
    );
    c.publish("battery_default", "/battery_state", BatteryState::default());
    c.publish(
        "fix",
        "/fix",
        NavSatFix {
            header: header(1_793_000_000, 200_000_000, "gps_link"),
            status: NavSatStatus { status: -1, service: 1 | 8 },
            latitude: 23.695_012_345_678_9,
            longitude: 120.538_765_432_1,
            altitude: f64::NAN,
            position_covariance: vec![0.000_4, 0.0, 0.0, 0.0, 0.000_4, 0.0, 0.0, 0.0, 1e-9],
            position_covariance_type: 2,
        },
    );
    c.publish(
        "fix_limits",
        "/fix",
        NavSatFix {
            status: NavSatStatus { status: i8::MIN, service: u16::MAX },
            position_covariance: vec![-0.0; 9],
            ..Default::default()
        },
    );
    c.publish("coverage_progress", "/coverage_progress", coverage_progress("map"));

    // --- services the app calls: its request, the server's answer ----------
    c.service::<Trigger::Service>(
        "trigger",
        "/check_nav_status",
        "call_1793000000000_41",
        json!({}),
        Trigger::Response { success: true, message: "{\"nav_active\":false,\"state\":\"idle\"}".into() },
    );
    c.service::<Trigger::Service>(
        "trigger_refused",
        "/record_zone_start",
        "call_1793000000001_42",
        json!({}),
        Trigger::Response { success: false, message: "錄製中，請先結束 \"zone 2\"".into() },
    );
    c.service::<ImportImageMask::Service>(
        "import_image_mask",
        "/import_image_mask",
        "call_1793000000002_43",
        json!({
            "zone_id": 3,
            "robot_pose_header": {"stamp": {"sec": 1_793_000_000, "nanosec": 125_000_000}, "frame_id": "map"},
            "robot_pose_map": {
                "position": {"x": 1.5, "y": -2.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.479_425_538_604_203, "w": 0.877_582_561_890_372_8}
            },
            "width": 64,
            "height": 48,
            "resolution_m": 0.05,
            "start_x_m": 0.25,
            "start_y_m": -0.75,
            "image_heading_rad": 1.0,
            "mask_encoding": "base64_u8_row_major",
            "free_mask_data": "AAAA////".repeat(512),
            "risk_mask_data": ""
        }),
        ImportImageMask::Response { success: true, message: "imported".into(), zone_id: 3, area_m2: 7.68 },
    );
    c.service::<EditZone::Service>(
        "edit_zone_delete",
        "/edit_zone",
        "call_1793000000003_44",
        json!({"op": "delete", "kind": "zone", "id": 2, "points": []}),
        EditZone::Response { success: true, message: "zone #2 已刪除".into(), id: 2 },
    );
    c.service::<EditZone::Service>(
        "edit_zone_add",
        "/edit_zone",
        "call_1793000000004_45",
        json!({"op": "add", "kind": "risk", "id": 0, "points": [
            {"x": 0.0, "y": 0.0, "z": 0.0}, {"x": 3.5, "y": 0.1, "z": 0.0}, {"x": 2, "y": -1, "z": 0}
        ]}),
        EditZone::Response { success: true, message: String::new(), id: 7 },
    );
    c.service::<SiteOp::Service>(
        "site_op",
        "/site_op",
        "call_1793000000005_46",
        json!({"op": "rename", "name": "前院", "new_name": "front yard"}),
        SiteOp::Response {
            success: true,
            message: "renamed".into(),
            sites_json: r#"{"active":"front yard","sites":[{"name":"front yard","zones":3}]}"#.into(),
        },
    );
    c.service::<ZoneExecPath::Service>(
        "zone_exec_path",
        "/zone_exec_path",
        "call_1793000000006_47",
        json!({"zone_id": 3}),
        ZoneExecPath::Response { success: true, message: "path ready".into() },
    );
    c.service::<ZoneSequence::Service>(
        "run_zone_sequence",
        "/run_zone_sequence",
        "call_1793000000007_48",
        json!({"zone_ids": [3, 1, 2], "channel_proximity_m": 0.8}),
        ZoneSequence::Response { success: false, message: "navigation is not ready".into() },
    );
    c.service::<GetCoverageProgress::Service>(
        "coverage_progress_status",
        "/coverage_progress_status",
        "call_1793000000008_49",
        json!({}),
        GetCoverageProgress::Response {
            success: true,
            message: String::new(),
            progress: coverage_progress("map"),
            checkpoint: CoverageProgress::default(),
        },
    );
    // the IDL default: a request without zone_id resumes zone -1
    c.service::<ResumeCoverage::Service>(
        "resume_coverage_default",
        "/resume_coverage",
        "call_1793000000009_50",
        json!({}),
        ResumeCoverage::Response { success: false, message: "no checkpoint".into() },
    );
    c.service::<PidAutotune::Service>(
        "pid_autotune",
        "/pid_autotune",
        "call:3",
        json!({"op": "start"}),
        PidAutotune::Response { success: true, message: "started".into() },
    );
    // the app's set_parameters: a ParameterValue with its type and one value
    for (case, name, value) in [
        ("set_parameters_string", "coverage_pattern", json!({"type": 4, "string_value": "zigzag"})),
        ("set_parameters_bool", "boundary_ring", json!({"type": 1, "bool_value": false})),
        ("set_parameters_double", "strip_width_m", json!({"type": 3, "double_value": 0.35})),
    ] {
        c.service::<SetParameters::Service>(
            case,
            "/boustrophedon_coverage/set_parameters",
            "call_1793000000010_51",
            json!({"parameters": [{"name": name, "value": value}]}),
            SetParameters::Response {
                results: vec![SetParametersResult { successful: true, reason: String::new() }],
            },
        );
    }
    c.service::<SetParameters::Service>(
        "set_parameters_refused",
        "/boustrophedon_coverage/set_parameters",
        "call_1793000000011_52",
        json!({"parameters": [{"name": "strip_width_m", "value": {"type": 3, "double_value": -1}}]}),
        SetParameters::Response {
            results: vec![SetParametersResult { successful: false, reason: "strip_width_m must be > 0".into() }],
        },
    );

    // --- messages the app and the Studio publish ---------------------------
    c.inbound::<TwistStamped>(
        "joystick",
        "/app_joy_cmd",
        json!({
            "header": {"stamp": {"sec": 1_793_000_000, "nanosec": 950_000_000}, "frame_id": "manual-7f3a9c"},
            "twist": {"linear": {"x": 0.35, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": -0.6}}
        }),
    );
    // the stop frame with integers where the type has doubles
    c.inbound::<TwistStamped>(
        "joystick_zero_ints",
        "/app_joy_cmd",
        json!({
            "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "manual-session-unavailable"},
            "twist": {"linear": {"x": 0, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": 0}}
        }),
    );
    c.inbound::<Text>("blade", "/mower_base/blade_command", json!({"data": "{\"permille\":600,\"ttl_ms\":500}"}));
    c.inbound::<Text>("recorder_command", "/mower_recorder/command", json!({"data": "{\"cmd\":\"delete\",\"run\":\"錄製 1\"}"}));
}
