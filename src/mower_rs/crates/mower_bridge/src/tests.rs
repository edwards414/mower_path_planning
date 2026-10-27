//! The manifest against the code's sources of names (the launch file, the
//! parameter files), and the policy against docs/BUS_TOPICS.md 5.3, without
//! DDS. tests/dds.rs runs a plan against DDS, tests/resolver.rs the names
//! against rcl.

use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;

use mower_bus::{Bus, Qos};
use mower_rs_common::{ModuleCtx, Remaps, Shutdown};
use r2r::std_msgs::msg::{Bool, String as Text};
use r2r::std_srvs::srv::Trigger;

use crate::types::*;
use crate::*;

/// What robot.launch.py puts in `--modules` with every switch on.
const PRODUCTION_MODULES: [&str; 15] = [
    "base", "status", "guards", "imu", "gps", "localize", "map", "coverage", "nav", "record", "adapter", "battery",
    "pid_autotune", "bridge", "agent",
];

/// The plan of the production process with the modules `on` on the bus.
fn production_plan(on: &[&str]) -> Result<Plan, PlanError> {
    plan(&Running::rsd(&PRODUCTION_MODULES, |id| on.contains(&id)), &production_remaps())
}

fn lines(on: &[&str]) -> Vec<String> {
    production_plan(on).unwrap_or_else(|e| panic!("{on:?}: {e}")).lines()
}

#[track_caller]
fn assert_lines(on: &[&str], expected: &[&str]) {
    let got = lines(on);
    let missing: Vec<&&str> = expected.iter().filter(|l| !got.iter().any(|g| g == *l)).collect();
    let extra: Vec<&String> = got.iter().filter(|g| !expected.contains(&g.as_str())).collect();
    assert!(missing.is_empty() && extra.is_empty(), "{on:?}\nmissing: {missing:#?}\nextra: {extra:#?}");
    assert_eq!(got, expected, "{on:?}: the order");
}

fn repo() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../..")
}

// ---------------------------------------------------------------------------
// The policy, step by step (docs/BUS_TOPICS.md section 6)
// ---------------------------------------------------------------------------

/// Nothing on the bus, nothing to bridge: mower_rsd starts no bridge and the
/// process is what it was before E2.
#[test]
fn with_no_module_on_the_bus_there_is_nothing_to_bridge() {
    let plan = production_plan(&[]).unwrap();
    assert!(plan.is_empty() && plan.notes.is_empty(), "{:?}", plan.lines());
}

/// Steps 1 and 2. BUS_TOPICS 5.3: the app topics go bus -> DDS until E3;
/// "most of its 9 inputs are bridged DDS -> bus until their producers move":
/// all but the two battery topics, which are in-process now. The LED request
/// goes out for mower_base, still on r2r. Each input's DDS reader is what
/// robot_status's own r2r reader was: best effort for its sensor inputs.
#[test]
fn battery_and_status() {
    assert_lines(
        &["battery", "status"],
        &[
            "to_ros /aon_battery_state sensor_msgs/msg/BatteryState R V 10",
            "to_ros /battery_state sensor_msgs/msg/BatteryState R V 10",
            "from_ros /fix sensor_msgs/msg/NavSatFix BE V 5",
            "from_ros /gps/filtered sensor_msgs/msg/NavSatFix BE V 5",
            "from_ros /imu/data sensor_msgs/msg/Imu BE V 5",
            "from_ros /mower_base/firmware_info std_msgs/msg/String R TL 1",
            "to_ros /mower_base/led_command std_msgs/msg/String R TL 1",
            "from_ros /mower_base/telemetry std_msgs/msg/String BE V 1",
            "from_ros /nav_operation_active std_msgs/msg/Bool R TL 1",
            "from_ros /odom_slow nav_msgs/msg/Odometry R V 10",
            "to_ros /robot/info std_msgs/msg/String R TL 1",
            "to_ros /robot/online std_msgs/msg/Bool R TL 1",
            "to_ros /robot/telemetry std_msgs/msg/String R V 10",
            "service_to_ros /system/check_update std_srvs/srv/Trigger",
            "service_to_ros /system/restart std_srvs/srv/Trigger",
            "service_to_ros /system/update std_srvs/srv/Trigger",
        ],
    );
}

/// Step 3. Every map, marker and JSON layer goes out transient local (5.5 a:
/// nav2's StaticLayer, the app's late joiners); the adapter's sources are
/// in-process but for coverage's markers and /odometry/global_slow. The
/// parameter services stay on r2r (E2 decision 2), and so do the adapter's
/// calls to them; /toLL and the lock go through the bridge. Coverage's two
/// silent writers do not make /free_space_inflated and /risk_map_inflated
/// two-way.
#[test]
fn battery_status_map_record_and_adapter() {
    assert_lines(
        &["battery", "status", "map", "record", "adapter"],
        &[
            "to_ros /adapter/coverage_settings std_msgs/msg/String R TL 1",
            "to_ros /adapter/map_datum std_msgs/msg/String R TL 1",
            "to_ros /adapter/map_layers/chennal_map_inflated std_msgs/msg/String R TL 1",
            "to_ros /adapter/map_layers/free_space_inflated std_msgs/msg/String R TL 1",
            "to_ros /adapter/map_layers/map_grid std_msgs/msg/String R TL 1",
            "to_ros /adapter/map_layers/risk_map_inflated std_msgs/msg/String R TL 1",
            "to_ros /adapter/marker_layers/channels std_msgs/msg/String R TL 1",
            "to_ros /adapter/marker_layers/connectors std_msgs/msg/String R TL 1",
            "to_ros /adapter/marker_layers/coverage_path std_msgs/msg/String R TL 1",
            "to_ros /adapter/marker_layers/invalid_segments std_msgs/msg/String R TL 1",
            "to_ros /adapter/marker_layers/risk_zones std_msgs/msg/String R TL 1",
            "to_ros /adapter/marker_layers/zones std_msgs/msg/String R TL 1",
            "to_ros /adapter/robot_pose geometry_msgs/msg/PoseStamped R V 10",
            "to_ros /adapter/zone_summaries std_msgs/msg/String R TL 1",
            "to_ros /aon_battery_state sensor_msgs/msg/BatteryState R V 10",
            "to_ros /battery_state sensor_msgs/msg/BatteryState R V 10",
            "to_ros /channel_path nav_msgs/msg/Path R V 10",
            "to_ros /channel_path_array visualization_msgs/msg/MarkerArray R TL 1",
            "to_ros /chennal_map nav_msgs/msg/OccupancyGrid R TL 1",
            "to_ros /chennal_map_inflated nav_msgs/msg/OccupancyGrid R TL 1",
            "to_ros /chennal_path nav_msgs/msg/Path R V 10",
            "to_ros /chennal_path_array visualization_msgs/msg/MarkerArray R TL 1",
            "from_ros /coverage_connectors visualization_msgs/msg/MarkerArray R TL 1",
            "from_ros /coverage_invalid_segments visualization_msgs/msg/MarkerArray R TL 1",
            "from_ros /coverage_path_markers visualization_msgs/msg/MarkerArray R TL 1",
            "from_ros /fix sensor_msgs/msg/NavSatFix BE V 5",
            "to_ros /free_space nav_msgs/msg/OccupancyGrid R TL 1",
            "to_ros /free_space_inflated nav_msgs/msg/OccupancyGrid R TL 1",
            "from_ros /gps/filtered sensor_msgs/msg/NavSatFix BE V 5",
            "from_ros /imu/data sensor_msgs/msg/Imu BE V 5",
            "to_ros /map_grid nav_msgs/msg/OccupancyGrid R TL 1",
            "to_ros /map_grid_global nav_msgs/msg/OccupancyGrid R TL 1",
            "from_ros /mower_base/firmware_info std_msgs/msg/String R TL 1",
            "to_ros /mower_base/led_command std_msgs/msg/String R TL 1",
            "to_ros /mower_base/rear_light std_msgs/msg/String R TL 1",
            "from_ros /mower_base/telemetry std_msgs/msg/String BE V 1",
            "from_ros /nav_operation_active std_msgs/msg/Bool R TL 1",
            "from_ros /odom_slow nav_msgs/msg/Odometry R V 10",
            "from_ros /odometry/global_slow nav_msgs/msg/Odometry R V 10",
            "to_ros /recorded_path nav_msgs/msg/Path R V 10",
            "to_ros /risk_map nav_msgs/msg/OccupancyGrid R TL 1",
            "to_ros /risk_map_inflated nav_msgs/msg/OccupancyGrid R TL 1",
            "to_ros /risk_zone_list visualization_msgs/msg/MarkerArray R TL 1",
            "to_ros /risk_zone_markers visualization_msgs/msg/Marker R TL 1",
            "to_ros /robot/info std_msgs/msg/String R TL 1",
            "to_ros /robot/online std_msgs/msg/Bool R TL 1",
            "to_ros /robot/telemetry std_msgs/msg/String R V 10",
            "to_ros /site_list std_msgs/msg/String R TL 1",
            "to_ros /zone_list visualization_msgs/msg/MarkerArray R TL 1",
            "to_ros /zone_markers visualization_msgs/msg/Marker R TL 1",
            "service_to_ros /channel_record_end std_srvs/srv/Trigger",
            "service_to_ros /channel_record_start std_srvs/srv/Trigger",
            "service_to_ros /chennal_record_end std_srvs/srv/Trigger",
            "service_to_ros /chennal_record_start std_srvs/srv/Trigger",
            "service_to_ros /create_chennal_map std_srvs/srv/Trigger",
            "service_to_ros /create_free_space std_srvs/srv/Trigger",
            "service_to_ros /create_risk_map std_srvs/srv/Trigger",
            "service_to_ros /edit_zone mower_interface/srv/EditZone",
            "service_to_ros /get_channel_path_list mower_interface/srv/ChannelPathList",
            "service_to_ros /get_channel_route mower_interface/srv/ChannelRoute",
            "service_to_ros /get_chennal_path_list mower_interface/srv/ChennalPathList",
            "service_to_ros /get_record_zone_info std_srvs/srv/Trigger",
            "service_to_ros /get_record_zone_list mower_interface/srv/GetZoneList",
            "service_to_ros /get_risk_zone_list mower_interface/srv/GetZoneList",
            "service_to_ros /get_zone_map_list_srv mower_interface/srv/ZoneMapList",
            "service_to_ros /import_image_mask mower_interface/srv/ImportImageMask",
            "service_to_ros /load_zone_list std_srvs/srv/Trigger",
            "service_from_ros /mission_operation_lock mower_interface/srv/MissionOperationLock",
            "service_to_ros /record_cancel std_srvs/srv/Trigger",
            "service_to_ros /record_zone_end std_srvs/srv/Trigger",
            "service_to_ros /record_zone_start std_srvs/srv/Trigger",
            "service_to_ros /restore_free_space_coverage std_srvs/srv/Trigger",
            "service_to_ros /risk_zone_end std_srvs/srv/Trigger",
            "service_to_ros /risk_zone_start std_srvs/srv/Trigger",
            "service_to_ros /save_zone_list std_srvs/srv/Trigger",
            "service_to_ros /site_op mower_interface/srv/SiteOp",
            "service_to_ros /system/check_update std_srvs/srv/Trigger",
            "service_to_ros /system/restart std_srvs/srv/Trigger",
            "service_to_ros /system/update std_srvs/srv/Trigger",
            "service_from_ros /toLL robot_localization/srv/ToLL",
        ],
    );
    let notes = production_plan(&["battery", "status", "map", "record", "adapter"]).unwrap().notes;
    assert!(notes.is_empty(), "{notes:?}");
}

/// Step 4. 5.3: localize's inputs "cross the bridge only as /odom (base
/// still on r2r) and rsp's /tf_static"; the sensor and odometry topics now
/// only go out (E3, E5, E9). /tf_static is DDS -> bus only, navsat's utm
/// writer staying on r2r (E2 decision 3), and deep enough for rsp's and
/// navsat's frames both.
#[test]
fn and_imu_gps_and_localize() {
    let all = ["battery", "status", "map", "record", "adapter", "imu", "gps", "localize"];
    let got = lines(&all);
    let before = lines(&all[..5]);
    let gone: Vec<&String> = before.iter().filter(|l| !got.contains(l)).collect();
    let new: Vec<&String> = got.iter().filter(|l| !before.contains(l)).collect();
    assert_eq!(
        gone,
        [
            "from_ros /fix sensor_msgs/msg/NavSatFix BE V 5",
            "from_ros /gps/filtered sensor_msgs/msg/NavSatFix BE V 5",
            "from_ros /imu/data sensor_msgs/msg/Imu BE V 5",
            "from_ros /odometry/global_slow nav_msgs/msg/Odometry R V 10",
            "service_from_ros /toLL robot_localization/srv/ToLL",
        ]
    );
    assert_eq!(
        new,
        [
            "to_ros /fix sensor_msgs/msg/NavSatFix R V 10",
            "to_ros /gps/filtered sensor_msgs/msg/NavSatFix R V 10",
            "to_ros /gps/status std_msgs/msg/String R V 10",
            "to_ros /imu/data sensor_msgs/msg/Imu R V 10",
            "from_ros /odom nav_msgs/msg/Odometry BE V 10",
            "to_ros /odometry/global nav_msgs/msg/Odometry R V 10",
            "to_ros /odometry/global_slow nav_msgs/msg/Odometry R V 10",
            "to_ros /odometry/gps nav_msgs/msg/Odometry R V 10",
            "to_ros /odometry/local nav_msgs/msg/Odometry R V 10",
            "to_ros /tf tf2_msgs/msg/TFMessage R V 100",
            "from_ros /tf_static tf2_msgs/msg/TFMessage R TL 100",
            "service_to_ros /datum robot_localization/srv/SetDatum",
            "service_to_ros /fromLL robot_localization/srv/FromLL",
            "service_to_ros /fromLLArray robot_localization/srv/FromLLArray",
            "service_to_ros /toLL robot_localization/srv/ToLL",
        ]
    );
    assert!(production_plan(&all).unwrap().notes.is_empty());
}

/// The table of README.md, "The policy".
#[test]
fn the_readme_counts_the_declarations_of_each_step() {
    use mower_bus_ros::Direction::*;
    let count = |on: &[&str]| {
        let plan = production_plan(on).unwrap();
        [ToRos, FromRos, ServiceToRos, ServiceFromRos].map(|d| plan.declarations.iter().filter(|x| x.direction == d).count())
    };
    let step3 = ["battery", "status", "map", "record", "adapter"];
    assert_eq!(count(&[]), [0, 0, 0, 0]);
    assert_eq!(count(&step3[..2]), [6, 7, 3, 0]);
    assert_eq!(count(&step3), [39, 11, 28, 2]);
    assert_eq!(count(&[&step3[..], &["imu", "gps", "localize"]].concat()), [48, 9, 32, 1]);
}

/// docs/BUS_TOPICS.md, "At a glance": 78 topics have an endpoint in a typed
/// module (the 13 of E2; ws_bridge joined the manifest in E3), and the
/// modules serve 45 services besides the parameter services.
#[test]
fn the_manifest_has_bus_topics_md_s_counts() {
    let mut topics = BTreeSet::new();
    let mut served = BTreeSet::new();
    let typed = MODULES.iter().filter(|m| m.id != "bridge");
    for e in typed.flat_map(|m| m.instances).flat_map(|i| i.endpoints) {
        let parameters = e.name.ends_with("/get_parameters") || e.name.ends_with("/set_parameters");
        if e.kind.is_topic() && e.only.is_none() {
            topics.insert(e.name);
        } else if e.kind == Kind::Serve && !parameters {
            served.insert(e.name);
        }
    }
    assert_eq!((topics.len(), served.len()), (78, 45));
}

/// Steps 5 to 7, for the record: the rest of the order needs no two-way
/// name either, and the drivetrain edges become in-process.
#[test]
fn the_rest_of_the_order_plans_too() {
    let mut on = vec!["battery", "status", "map", "record", "adapter", "imu", "gps", "localize", "coverage", "nav"];
    let step5 = production_plan(&on).unwrap();
    // nav_core.pose_source: tf would read /tf, which goes out from the EKFs
    assert_eq!(
        step5.notes,
        ["/tf: not bridged for nav_action_server, which subscribes to it only with nav_backend: nav_core and \
          nav_core.pose_source: tf: that would need both directions"]
    );
    let step5 = step5.lines();
    for line in [
        "from_ros /joy_cmd geometry_msgs/msg/TwistStamped R V 10",
        "from_ros /rosout rcl_interfaces/msg/Log R V 100",
        "from_ros /physical_joy_cmd geometry_msgs/msg/TwistStamped R V 10",
        "to_ros /nav_operation_active std_msgs/msg/Bool R TL 1",
        "to_ros /navigation_safety_stop geometry_msgs/msg/TwistStamped R V 10",
    ] {
        assert!(step5.iter().any(|l| l == line), "{line} in {step5:#?}");
    }
    assert!(!step5.iter().any(|l| l.contains(" /nav_operation_active ") && l.starts_with("from_ros")));
    // nav keeps its bt_navigator client on r2r with nav2's two actions until E9.
    assert!(!step5.iter().any(|l| l.contains(" /bt_navigator/get_state ")), "{step5:#?}");

    on.extend(["base", "pid_autotune", "guards"]);
    let all = lines(&on);
    for line in [
        "to_ros /odom nav_msgs/msg/Odometry SD",
        "to_ros /tf tf2_msgs/msg/TFMessage R V 100",
        "to_ros /drivetrain_guarded_cmd_vel geometry_msgs/msg/TwistStamped R V 1",
        "from_ros /cmd_vel_guard_input geometry_msgs/msg/TwistStamped R V 1",
        "from_ros /app_joy_cmd geometry_msgs/msg/TwistStamped R V 1",
        "from_ros /mower_base/blade_command std_msgs/msg/String BE V 1",
        "from_ros /tf_static tf2_msgs/msg/TFMessage R TL 100",
    ] {
        assert!(all.iter().any(|l| l == line), "{line} in {all:#?}");
    }
    let from: Vec<&String> = all.iter().filter(|l| l.starts_with("from_ros ")).collect();
    // ws_bridge's three writers (on r2r) among them
    assert_eq!(from.len(), 8, "only the writers outside the bus are left: {from:#?}");
}

/// docs/BUS_TOPICS.md 5.5 d, e: /mower_base/led_command has two latched
/// writers. With its reader on the bus and one writer on r2r, the bridge
/// would have to read back what it writes. Every other combination of flags
/// plans: that is the one order the steps must keep (base and pid_autotune
/// move together, after status).
#[test]
fn every_combination_of_flags_plans_but_a_split_led_command() {
    let ids: Vec<&str> = MODULES.iter().map(|m| m.id).collect();
    let remaps = production_remaps();
    for bits in 0u32..(1 << ids.len()) {
        let on = |id: &str| ids.iter().position(|i| *i == id).is_some_and(|n| bits & (1 << n) != 0);
        let refused = on("base") && on("status") != on("pid_autotune");
        match plan(&Running::rsd(&PRODUCTION_MODULES, on), &remaps) {
            Ok(_) => assert!(!refused, "{bits:#015b}: planned with base on the bus and the LED writers split"),
            Err(e) => {
                assert!(refused, "{bits:#015b}: {e}");
                assert!(e.problems.len() == 1 && e.problems[0].starts_with("/mower_base/led_command: "), "{e}");
            }
        }
    }
}

/// Deterministic: the same input gives the same declarations in the same
/// order, whatever the order of `--modules`.
#[test]
fn the_plan_does_not_depend_on_the_order_of_the_modules() {
    let on = |id: &str| ["status", "map", "localize"].contains(&id);
    let forward = plan(&Running::rsd(&PRODUCTION_MODULES, on), &production_remaps()).unwrap().lines();
    let mut reversed = PRODUCTION_MODULES;
    reversed.reverse();
    let backward = plan(&Running::rsd(&reversed, on), &production_remaps()).unwrap().lines();
    assert_eq!(forward, backward);
}

/// shadow_compare.py's default set, `status,battery,adapter,guards`, with the
/// first three on the bus: map, record and coverage run elsewhere, so the
/// adapter's latched sources come in from DDS, and its calls go out.
#[test]
fn shadow_compare_s_set_plans_the_adapter_s_inputs_from_dds() {
    let running = Running::rsd(&["status", "battery", "adapter", "guards"], |id| id != "guards");
    let got = plan(&running, &production_remaps()).unwrap().lines();
    for source in ["/map_grid", "/free_space_inflated", "/risk_map_inflated", "/chennal_map_inflated"] {
        let line = format!("from_ros {source} nav_msgs/msg/OccupancyGrid R TL 1");
        assert!(got.contains(&line), "{line}");
    }
    let markers = ["/chennal_path_array", "/coverage_connectors", "/coverage_invalid_segments", "/coverage_path_markers"];
    for source in markers.iter().chain(&["/risk_zone_list", "/zone_list"]) {
        let line = format!("from_ros {source} visualization_msgs/msg/MarkerArray R TL 1");
        assert!(got.contains(&line), "{line}");
    }
    for line in [
        "from_ros /odometry/global_slow nav_msgs/msg/Odometry R V 10",
        "service_from_ros /get_zone_map_list_srv mower_interface/srv/ZoneMapList",
        "service_from_ros /toLL robot_localization/srv/ToLL",
    ] {
        assert!(got.iter().any(|l| l == line), "{line} in {got:#?}");
    }
    assert!(!got.iter().any(|l| l.contains("get_parameters")), "they stay on r2r");
    // the guards are in the process, on r2r: their topics are theirs
    assert!(!got.iter().any(|l| l.contains("joy_cmd") || l.contains("cmd_vel")), "{got:#?}");
}

/// A subset of modules: what does not run here is elsewhere, so every input
/// of a moved module comes from DDS and every output goes out.
#[test]
fn modules_that_do_not_run_here_are_elsewhere() {
    let running = Running::rsd(&["status"], |_| true);
    let got = plan(&running, &production_remaps()).unwrap().lines();
    assert!(got.contains(&"from_ros /battery_state sensor_msgs/msg/BatteryState R V 10".to_string()), "{got:#?}");
    assert!(got.contains(&"to_ros /robot/online std_msgs/msg/Bool R TL 1".to_string()));
    assert_eq!(got.len(), 16);
}

/// A moved module in its own binary: every endpoint it has faces DDS. The
/// exceptions are what stays on r2r, and a silent writer of a name the module
/// also reads, which needs only DDS -> bus.
#[test]
fn every_module_plans_standalone_with_every_endpoint_facing_dds() {
    for module in MODULES {
        // one instance at a time, as the guards' binary runs them
        let running: Vec<Running> =
            module.instances.iter().flat_map(|i| Running::standalone(module.id, i.node).unwrap()).collect();
        for r in &running {
            let alone = plan(std::slice::from_ref(r), &Remaps::default()).unwrap_or_else(|e| panic!("{}: {e}", r.node));
            let declared: BTreeSet<(Space, &str)> = alone.declarations.iter().map(|d| (space(d), d.name.as_str())).collect();
            for e in r.instance.endpoints.iter().filter(|e| !e.stays_on_r2r) {
                // without the production rules, the code's names
                let name = Remaps::default().resolve(e.kind.name_kind(), &r.node, "", e.code).unwrap();
                assert!(declared.contains(&(e.kind.space(), name.as_str())), "{} {} {name}", r.node, e.kind.verb());
            }
        }
    }
    // Coverage's grids: it only reads them (its silent writers went with the
    // move, docs/BUS_TOPICS.md 5.8 #2), and sends its goals to whoever serves
    // the action on DDS: coverage_cancel_check.py's server, standalone.
    let coverage = plan(&Running::standalone("coverage", "boustrophedon_coverage").unwrap(), &Remaps::default()).unwrap();
    let lines = coverage.lines();
    assert!(lines.contains(&"from_ros /risk_map_inflated nav_msgs/msg/OccupancyGrid R TL 1".to_string()));
    assert!(!lines.iter().any(|l| l.contains("/free_space_inflated")), "{lines:#?}");
    assert!(coverage.notes.is_empty(), "{:?}", coverage.notes);
    assert!(lines.contains(&"action_from_ros /nav_action_follow_path mower_interface/action/Waypoint".to_string()));
    // nav serves both of its actions to the CI tools' rclpy clients; nav2's
    // stay r2r clients
    let nav = plan(&Running::standalone("nav", "nav_action_server").unwrap(), &Remaps::default()).unwrap().lines();
    let actions: Vec<&String> = nav.iter().filter(|l| l.starts_with("action_")).collect();
    assert_eq!(
        actions,
        ["action_to_ros /nav_action mower_interface/action/Waypoint", "action_to_ros /nav_action_follow_path mower_interface/action/Waypoint"]
    );
}

/// The namespace a declaration's name is in.
fn space(d: &Declaration) -> Space {
    use mower_bus_ros::Direction::*;
    match d.direction {
        ToRos | FromRos => Space::Topic,
        ServiceToRos | ServiceFromRos => Space::Service,
        ActionToRos | ActionFromRos => Space::Action,
    }
}

/// Step 5 in mower_rsd: nav's two action servers are served on DDS as its
/// services are, so that the graph keeps them (the tools, nav_action_client.py);
/// coverage's goals reach nav on the bus, and cross only while nav is on r2r.
#[test]
fn coverage_s_goals_cross_only_while_nav_is_on_r2r() {
    let actions = |on: &[&str]| -> Vec<String> { lines(on).into_iter().filter(|l| l.starts_with("action_")).collect() };
    let step4 = ["battery", "status", "map", "record", "adapter", "imu", "gps", "localize"];
    let served = ["action_to_ros /nav_action mower_interface/action/Waypoint", "action_to_ros /nav_action_follow_path mower_interface/action/Waypoint"];
    assert_eq!(actions(&step4), Vec::<String>::new());
    assert_eq!(actions(&[&step4[..], &["coverage", "nav"]].concat()), served);
    assert_eq!(actions(&[&step4[..], &["nav"]].concat()), served);
    assert_eq!(
        actions(&[&step4[..], &["coverage"]].concat()),
        ["action_from_ros /nav_action_follow_path mower_interface/action/Waypoint"]
    );
}

/// rcl applies the process's rules to the bridge's names too, which the
/// policy resolved already: a rule that would rename one of them again puts
/// the bridge's DDS endpoint on another name than the module's, and the plan
/// refuses it. robot.launch.py's rules rename none, for mower_rsd's bridge
/// and for every standalone one.
#[test]
fn a_rule_that_renames_a_bridged_name_again_is_an_error() {
    // the module's /battery_state is /battery_b; the bridge's /battery_b
    // would be /battery_state
    let swap = ["--ros-args", "-r", "/battery_state:=/battery_b", "-r", "/battery_b:=/battery_state"];
    let swap = Remaps::from_args(&swap.map(str::to_string)).unwrap();
    let e = rsd_plan(&["battery"], &swap).unwrap_err();
    assert!(e.problems.iter().any(|p| p.starts_with("to_ros /battery_b ")), "{e}");
    let alone = Running::standalone("battery", "battery_state").unwrap();
    let e = plan(&alone, &swap).unwrap().check_names(&swap, "battery_state", STANDALONE_NAMESPACE).unwrap_err();
    assert!(e.problems.iter().any(|p| p.starts_with("to_ros /battery_b ")), "{e}");

    let production = production_remaps();
    rsd_plan(&PRODUCTION_MODULES, &production).unwrap();
    for module in MODULES {
        for i in module.instances {
            let alone = Running::standalone(module.id, i.node).unwrap();
            let p = plan(&alone, &production).unwrap_or_else(|e| panic!("{}: {e}", i.node));
            p.check_names(&production, i.node, STANDALONE_NAMESPACE).unwrap_or_else(|e| panic!("{}: {e}", i.node));
        }
    }
}

#[test]
fn a_binary_finds_its_instances_by_node_name() {
    let nodes = |id: &str, node: &str| -> Result<Vec<(String, String)>, String> {
        Ok(Running::standalone(id, node)?.into_iter().map(|r| (r.instance.node.to_string(), r.node)).collect())
    };
    let pair = |a: &str, b: &str| (a.to_string(), b.to_string());
    // launch renames the binary's node (`name=`)
    assert_eq!(nodes("guards", "manual_velocity_guard"), Ok(vec![pair("manual_velocity_guard", "manual_velocity_guard")]));
    assert!(nodes("guards", "velocity_command_guard_2").is_err(), "which guard?");
    // mower_localize runs all three under their own names
    assert_eq!(nodes("localize", "localize").unwrap().len(), 3);
    // a binary started by hand keeps its built-in name
    assert_eq!(nodes("imu", "imu_driver_node"), Ok(vec![pair("imu", "imu_driver_node")]));
    assert_eq!(nodes("bridge", "mower_ws_bridge"), Ok(vec![pair("mower_ws_bridge", "mower_ws_bridge")]));
    assert!(nodes("agent", "mower_agent").is_err(), "no endpoint of its own");
}

// ---------------------------------------------------------------------------
// ws_bridge (step E3)
// ---------------------------------------------------------------------------

/// The production plan of E2 as merged (main fbf5afe, step 5 included),
/// taken from that tree before E3 touched the manifest: every module with
/// its flag, ws_bridge on r2r. 61 to_ros, 9 from_ros, 44 service_to_ros, 2
/// action_to_ros.
const E2_PRODUCTION_PLAN: [&str; 116] = [
    "to_ros /adapter/coverage_settings std_msgs/msg/String R TL 1",
    "to_ros /adapter/map_datum std_msgs/msg/String R TL 1",
    "to_ros /adapter/map_layers/chennal_map_inflated std_msgs/msg/String R TL 1",
    "to_ros /adapter/map_layers/free_space_inflated std_msgs/msg/String R TL 1",
    "to_ros /adapter/map_layers/map_grid std_msgs/msg/String R TL 1",
    "to_ros /adapter/map_layers/risk_map_inflated std_msgs/msg/String R TL 1",
    "to_ros /adapter/marker_layers/channels std_msgs/msg/String R TL 1",
    "to_ros /adapter/marker_layers/connectors std_msgs/msg/String R TL 1",
    "to_ros /adapter/marker_layers/coverage_path std_msgs/msg/String R TL 1",
    "to_ros /adapter/marker_layers/invalid_segments std_msgs/msg/String R TL 1",
    "to_ros /adapter/marker_layers/risk_zones std_msgs/msg/String R TL 1",
    "to_ros /adapter/marker_layers/zones std_msgs/msg/String R TL 1",
    "to_ros /adapter/robot_pose geometry_msgs/msg/PoseStamped R V 10",
    "to_ros /adapter/zone_summaries std_msgs/msg/String R TL 1",
    "to_ros /aon_battery_state sensor_msgs/msg/BatteryState R V 10",
    "to_ros /battery_state sensor_msgs/msg/BatteryState R V 10",
    "to_ros /channel_path nav_msgs/msg/Path R V 10",
    "to_ros /channel_path_array visualization_msgs/msg/MarkerArray R TL 1",
    "to_ros /chennal_map nav_msgs/msg/OccupancyGrid R TL 1",
    "to_ros /chennal_map_inflated nav_msgs/msg/OccupancyGrid R TL 1",
    "to_ros /chennal_path nav_msgs/msg/Path R V 10",
    "to_ros /chennal_path_array visualization_msgs/msg/MarkerArray R TL 1",
    "to_ros /coverage_connectors visualization_msgs/msg/MarkerArray R TL 1",
    "to_ros /coverage_invalid_segments visualization_msgs/msg/MarkerArray R TL 1",
    "to_ros /coverage_path nav_msgs/msg/Path R TL 1",
    "to_ros /coverage_path_markers visualization_msgs/msg/MarkerArray R TL 1",
    "to_ros /coverage_progress mower_interface/msg/CoverageProgress R TL 1",
    "to_ros /coverage_split_points visualization_msgs/msg/Marker R V 1",
    "to_ros /fix sensor_msgs/msg/NavSatFix R V 10",
    "to_ros /free_space nav_msgs/msg/OccupancyGrid R TL 1",
    "to_ros /free_space_inflated nav_msgs/msg/OccupancyGrid R TL 1",
    "to_ros /gps/filtered sensor_msgs/msg/NavSatFix R V 10",
    "to_ros /gps/status std_msgs/msg/String R V 10",
    "to_ros /imu/data sensor_msgs/msg/Imu R V 10",
    "from_ros /joy_cmd geometry_msgs/msg/TwistStamped R V 10",
    "from_ros /keyboard_cmd_vel geometry_msgs/msg/TwistStamped R V 10",
    "to_ros /map_grid nav_msgs/msg/OccupancyGrid R TL 1",
    "to_ros /map_grid_global nav_msgs/msg/OccupancyGrid R TL 1",
    "from_ros /mower_base/firmware_info std_msgs/msg/String R TL 1",
    "to_ros /mower_base/led_command std_msgs/msg/String R TL 1",
    "to_ros /mower_base/rear_light std_msgs/msg/String R TL 1",
    "from_ros /mower_base/telemetry std_msgs/msg/String BE V 1",
    "to_ros /nav_cmd_vel geometry_msgs/msg/TwistStamped R V 10",
    "to_ros /nav_operation_active std_msgs/msg/Bool R TL 1",
    "to_ros /navigation_coordinator_lock std_msgs/msg/Bool R V 10",
    "to_ros /navigation_safety_stop geometry_msgs/msg/TwistStamped R V 10",
    "from_ros /odom nav_msgs/msg/Odometry BE V 10",
    "from_ros /odom_slow nav_msgs/msg/Odometry R V 10",
    "to_ros /odometry/global nav_msgs/msg/Odometry R V 10",
    "to_ros /odometry/global_slow nav_msgs/msg/Odometry R V 10",
    "to_ros /odometry/gps nav_msgs/msg/Odometry R V 10",
    "to_ros /odometry/local nav_msgs/msg/Odometry R V 10",
    "from_ros /physical_joy_cmd geometry_msgs/msg/TwistStamped R V 10",
    "to_ros /plan nav_msgs/msg/Path R V 1",
    "to_ros /recorded_path nav_msgs/msg/Path R V 10",
    "to_ros /risk_map nav_msgs/msg/OccupancyGrid R TL 1",
    "to_ros /risk_map_inflated nav_msgs/msg/OccupancyGrid R TL 1",
    "to_ros /risk_zone_list visualization_msgs/msg/MarkerArray R TL 1",
    "to_ros /risk_zone_markers visualization_msgs/msg/Marker R TL 1",
    "to_ros /robot/info std_msgs/msg/String R TL 1",
    "to_ros /robot/online std_msgs/msg/Bool R TL 1",
    "to_ros /robot/telemetry std_msgs/msg/String R V 10",
    "from_ros /rosout rcl_interfaces/msg/Log R V 100",
    "to_ros /site_list std_msgs/msg/String R TL 1",
    "to_ros /split_path nav_msgs/msg/Path R V 1",
    "to_ros /tf tf2_msgs/msg/TFMessage R V 100",
    "from_ros /tf_static tf2_msgs/msg/TFMessage R TL 100",
    "to_ros /zone_list visualization_msgs/msg/MarkerArray R TL 1",
    "to_ros /zone_markers visualization_msgs/msg/Marker R TL 1",
    "to_ros /zone_sequence_status std_msgs/msg/String R TL 1",
    "service_to_ros /cancel_nav2 std_srvs/srv/Trigger",
    "service_to_ros /cancel_navigation_dispatch mower_interface/srv/CancelNavigationDispatch",
    "service_to_ros /cencel_nav2 std_srvs/srv/Trigger",
    "service_to_ros /channel_record_end std_srvs/srv/Trigger",
    "service_to_ros /channel_record_start std_srvs/srv/Trigger",
    "service_to_ros /check_nav_status std_srvs/srv/Trigger",
    "service_to_ros /chennal_record_end std_srvs/srv/Trigger",
    "service_to_ros /chennal_record_start std_srvs/srv/Trigger",
    "service_to_ros /confirm_navigation_dispatch mower_interface/srv/ConfirmNavigationDispatch",
    "service_to_ros /coverage_progress_status mower_interface/srv/GetCoverageProgress",
    "service_to_ros /create_chennal_map std_srvs/srv/Trigger",
    "service_to_ros /create_free_space std_srvs/srv/Trigger",
    "service_to_ros /create_risk_map std_srvs/srv/Trigger",
    "service_to_ros /datum robot_localization/srv/SetDatum",
    "service_to_ros /edit_zone mower_interface/srv/EditZone",
    "service_to_ros /fromLL robot_localization/srv/FromLL",
    "service_to_ros /fromLLArray robot_localization/srv/FromLLArray",
    "service_to_ros /generate_coverage_path std_srvs/srv/Trigger",
    "service_to_ros /get_channel_path_list mower_interface/srv/ChannelPathList",
    "service_to_ros /get_channel_route mower_interface/srv/ChannelRoute",
    "service_to_ros /get_chennal_path_list mower_interface/srv/ChennalPathList",
    "service_to_ros /get_record_zone_info std_srvs/srv/Trigger",
    "service_to_ros /get_record_zone_list mower_interface/srv/GetZoneList",
    "service_to_ros /get_risk_zone_list mower_interface/srv/GetZoneList",
    "service_to_ros /get_zone_map_list_srv mower_interface/srv/ZoneMapList",
    "service_to_ros /import_image_mask mower_interface/srv/ImportImageMask",
    "service_to_ros /load_zone_list std_srvs/srv/Trigger",
    "service_to_ros /mission_operation_lock mower_interface/srv/MissionOperationLock",
    "service_to_ros /record_cancel std_srvs/srv/Trigger",
    "service_to_ros /record_zone_end std_srvs/srv/Trigger",
    "service_to_ros /record_zone_start std_srvs/srv/Trigger",
    "service_to_ros /restore_free_space_coverage std_srvs/srv/Trigger",
    "service_to_ros /resume_coverage mower_interface/srv/ResumeCoverage",
    "service_to_ros /risk_zone_end std_srvs/srv/Trigger",
    "service_to_ros /risk_zone_start std_srvs/srv/Trigger",
    "service_to_ros /run_zone_sequence mower_interface/srv/ZoneSequence",
    "service_to_ros /save_zone_list std_srvs/srv/Trigger",
    "service_to_ros /site_op mower_interface/srv/SiteOp",
    "service_to_ros /stop_zone_sequence std_srvs/srv/Trigger",
    "service_to_ros /system/check_update std_srvs/srv/Trigger",
    "service_to_ros /system/restart std_srvs/srv/Trigger",
    "service_to_ros /system/update std_srvs/srv/Trigger",
    "service_to_ros /toLL robot_localization/srv/ToLL",
    "service_to_ros /zone_exec_path mower_interface/srv/ZoneExecPath",
    "action_to_ros /nav_action mower_interface/action/Waypoint",
    "action_to_ros /nav_action_follow_path mower_interface/action/Waypoint",
];

/// What the policy notes for the same plan.
const E2_PRODUCTION_NOTE: &str = "/tf: not bridged for nav_action_server, which subscribes to it only with \
                                  nav_backend: nav_core and nav_core.pose_source: tf: that would need both directions";

#[track_caller]
fn assert_same_lines(got: &[String], expected: &[&str], what: &str) {
    let missing: Vec<&&str> = expected.iter().filter(|l| !got.iter().any(|g| g == *l)).collect();
    let extra: Vec<&String> = got.iter().filter(|g| !expected.contains(&g.as_str())).collect();
    assert!(missing.is_empty() && extra.is_empty(), "{what}\nmissing: {missing:#?}\nextra: {extra:#?}");
    assert_eq!(got, expected, "{what}: the order");
}

/// With ws_bridge's flag false, the production plan is E2's line for line,
/// whether ws_bridge runs in mower_rsd (rust_bridge:=true) or does not
/// (rust_bridge:=false: the Python rosbridge, or the separate binary). The
/// `bridge` module and the recorder's nodes outside add no declaration
/// while nothing of theirs is on the bus. A later flag flip (E2 steps 6 and
/// 7) changes this plan on purpose, and the snapshot with it.
#[test]
fn with_ws_bridge_off_the_production_plan_is_e2_s_line_for_line() {
    let remaps = production_remaps();
    let in_rsd = rsd_plan(&PRODUCTION_MODULES, &remaps).unwrap();
    assert_same_lines(&in_rsd.lines(), &E2_PRODUCTION_PLAN, "ws_bridge in mower_rsd, on r2r");
    assert_eq!(in_rsd.notes, [E2_PRODUCTION_NOTE]);
    let without: Vec<&str> = PRODUCTION_MODULES.iter().copied().filter(|id| *id != "bridge").collect();
    let elsewhere = rsd_plan(&without, &remaps).unwrap();
    assert_same_lines(&elsewhere.lines(), &E2_PRODUCTION_PLAN, "ws_bridge not in mower_rsd");
    assert_eq!(elsewhere.notes, [E2_PRODUCTION_NOTE]);
}

/// ws_bridge.yaml as ws_bridge reads it: topics_sub, topics_pub, and the
/// services with their types.
fn ws_bridge_yaml() -> (Vec<String>, Vec<String>, BTreeMap<String, String>) {
    let text = std::fs::read_to_string(repo().join("src/mower_bringup/config/ws_bridge.yaml")).unwrap();
    let doc: serde_yaml::Value = serde_yaml::from_str(&text).unwrap();
    let list = |key: &str| -> Vec<String> {
        doc[key].as_sequence().unwrap().iter().map(|v| v.as_str().unwrap().to_string()).collect()
    };
    let services = doc["services"]
        .as_mapping()
        .unwrap()
        .iter()
        .map(|(k, v)| (k.as_str().unwrap().to_string(), v.as_str().unwrap().to_string()))
        .collect();
    (list("topics_sub"), list("topics_pub"), services)
}

/// The fnmatch subset ws_bridge's allow-list uses (mower_ws_bridge's
/// config.rs): `*` is any run of characters, `/` included, as in Python's
/// fnmatch, which rosbridge used.
fn glob(pattern: &str, name: &str) -> bool {
    fn rec(p: &[u8], n: &[u8]) -> bool {
        match p.first() {
            None => n.is_empty(),
            Some(b'*') => (0..=n.len()).any(|i| rec(&p[1..], &n[i..])),
            Some(c) => n.first() == Some(c) && rec(&p[1..], &n[1..]),
        }
    }
    rec(pattern.as_bytes(), name.as_bytes())
}

/// The `bridge` module is ws_bridge.yaml's allow-list: topics_sub's globs
/// expanded to every name a module or a node outside writes, read R TL 10;
/// topics_pub, written R V 10; the services, called, with the types the yaml
/// gives them (which ws_bridge's r2r backend creates its clients with). The
/// bus backend subscribes only to names of the module or typed on the bus,
/// so a name added to the allow-list needs a line here too.
/// mower_mission's `test_ws_bridge_policy_matches_the_rosbridge_allow_lists`
/// keeps the yaml rosbridge's.
#[test]
fn the_bridge_module_is_ws_bridge_yaml_s_allow_list() {
    let (topics_sub, topics_pub, services) = ws_bridge_yaml();
    let written: BTreeSet<&str> = MODULES
        .iter()
        .flat_map(|m| m.instances)
        .flat_map(|i| i.endpoints)
        .chain(OUTSIDE.iter().flat_map(|o| o.endpoints))
        .filter(|e| e.kind == Kind::Publish && e.only.is_none())
        .map(|e| e.name)
        .collect();
    let reachable: BTreeSet<&str> = written.iter().copied().filter(|n| topics_sub.iter().any(|g| glob(g, n))).collect();

    let (module, bridge) = instance("mower_ws_bridge").expect("ws_bridge's instance");
    assert_eq!(module.id, "bridge");
    let of = |kind: Kind| bridge.endpoints.iter().filter(move |e| e.kind == kind);
    let subscribed: BTreeSet<&str> = of(Kind::Subscribe).map(|e| e.name).collect();
    assert_eq!(subscribed, reachable, "topics_sub's names that have a writer");
    assert!(of(Kind::Subscribe).all(|e| e.profile() == Some(R_TL_10)));
    assert!(topics_pub.iter().all(|t| !t.contains('*')), "topics_pub names topics, it has no globs");
    let published: BTreeSet<&str> = of(Kind::Publish).map(|e| e.name).collect();
    assert_eq!(published, topics_pub.iter().map(String::as_str).collect::<BTreeSet<_>>());
    assert!(of(Kind::Publish).all(|e| e.profile() == Some(R_V_10)));
    let called: BTreeMap<String, String> = of(Kind::Call).map(|e| (e.name.to_string(), e.ros_type())).collect();
    assert_eq!(called, services, "the services, with ws_bridge.yaml's types");
    assert_eq!(bridge.endpoints.len(), subscribed.len() + published.len() + called.len(), "and nothing else");
    // /gps/fix and /*/battery_state have no writer
    assert_eq!((subscribed.len(), published.len(), called.len()), (28, 4, 32));
    assert!(!subscribed.contains("/gps/fix"));
}

/// E3.1's switch: ws_bridge on the bus. The names the app uses that only DDS
/// has (the guards, mower_base, pid_autotune, coverage's parameter node,
/// the recorder) are bridged for it, with the QoS ws_bridge's r2r endpoints
/// had: 5 from_ros, 4 to_ros, 4 service_from_ros. Nothing else changes.
#[test]
fn ws_bridge_on_the_bus_adds_the_thirteen_names_only_dds_has() {
    let on = rsd_plan_with(&PRODUCTION_MODULES, &["bridge"], &production_remaps()).unwrap();
    let lines = on.lines();
    let removed: Vec<&str> = E2_PRODUCTION_PLAN.iter().copied().filter(|l| !lines.iter().any(|g| g == l)).collect();
    assert!(removed.is_empty(), "{removed:#?}");
    let added: Vec<&str> = lines.iter().map(String::as_str).filter(|l| !E2_PRODUCTION_PLAN.contains(l)).collect();
    assert_eq!(
        added,
        [
            "to_ros /app_joy_cmd geometry_msgs/msg/TwistStamped R V 10",
            "from_ros /manual_command_clock std_msgs/msg/Header R V 10",
            "to_ros /mower_base/blade_command std_msgs/msg/String R V 10",
            "to_ros /mower_base/servo_command std_msgs/msg/String R V 10",
            "from_ros /mower_recorder/bags std_msgs/msg/String R TL 10",
            "to_ros /mower_recorder/command std_msgs/msg/String R V 10",
            "from_ros /mower_recorder/command_result std_msgs/msg/String R V 10",
            "from_ros /mower_recorder/status std_msgs/msg/String R TL 10",
            "from_ros /pid_autotune/status std_msgs/msg/String R TL 10",
            "service_from_ros /boustrophedon_coverage/set_parameters rcl_interfaces/srv/SetParameters",
            "service_from_ros /mower_recorder/start std_srvs/srv/Trigger",
            "service_from_ros /mower_recorder/stop std_srvs/srv/Trigger",
            "service_from_ros /pid_autotune mower_interface/srv/PidAutotune",
        ]
    );
    assert_eq!(on.notes, [E2_PRODUCTION_NOTE]);
    // whom each is for, and from where
    let why = |name: &str| on.declarations.iter().find(|d| d.name == name).unwrap().why.clone();
    assert_eq!(why("/pid_autotune/status"), "for mower_ws_bridge, from pid_autotune (r2r)");
    assert_eq!(why("/mower_recorder/status"), "for mower_ws_bridge, from recorder_manager (outside)");
    assert_eq!(why("/boustrophedon_coverage/set_parameters"), "for mower_ws_bridge, served by boustrophedon_coverage (r2r)");
    assert_eq!(why("/app_joy_cmd"), "from mower_ws_bridge");
}

/// Only ws_bridge's flag is set at run time: any other id is on_bus.rs's.
#[test]
fn the_run_time_flags_are_ws_bridge_s_alone() {
    let remaps = production_remaps();
    assert_eq!(rsd_plan_with(&PRODUCTION_MODULES, &[] as &[&str], &remaps).unwrap().lines(), E2_PRODUCTION_PLAN);
    let e = rsd_plan_with(&PRODUCTION_MODULES, &["base"], &remaps).unwrap_err();
    assert_eq!(e.problems, ["base: its flag is on_bus.rs's, not set at run time"]);
    // the bus bridge module's arguments carry them
    let args = rsd_args(&["status", "bridge"], &["bridge"]);
    assert_eq!(args, ["status", "bridge", "--on-bus", "bridge"]);
    assert_eq!(
        crate::run::parse_rsd_args(&args),
        Ok((vec!["status".to_string(), "bridge".to_string()], vec!["bridge".to_string()]))
    );
    assert_eq!(crate::run::parse_rsd_args(&rsd_args(&["status"], &[] as &[&str])), Ok((vec!["status".to_string()], vec![])));
    assert!(crate::run::parse_rsd_args(&["status".to_string(), "--on-bus".to_string()]).is_err());
}

/// Where each other end of `name` is (`kind` of it), with the flags `on`
/// and every module running: whether one is an instance on the bus.
fn on_the_bus(name: &str, kind: Kind, on: &dyn Fn(&str) -> bool) -> bool {
    MODULES.iter().filter(|m| on(m.id)).flat_map(|m| m.instances).flat_map(|i| i.endpoints).any(|e| {
        e.kind == kind && e.name == name && !e.stays_on_r2r && e.only.is_none()
    })
}

/// RUST_BRIDGE=false (the Python rosbridge) or the separate binary: ws_bridge
/// is not in mower_rsd's `--modules`, and every name of its allow-list keeps
/// a DDS face, whatever the flags: a topic it reads is written on DDS or
/// goes out to it, a topic it writes is read on DDS or comes in from it, a
/// service it calls is served on DDS or is served out to it.
#[test]
fn without_ws_bridge_in_the_process_every_name_of_its_allow_list_faces_dds() {
    let without: Vec<&str> = PRODUCTION_MODULES.iter().copied().filter(|id| *id != "bridge").collect();
    let (_, bridge) = instance("mower_ws_bridge").unwrap();
    let flags: [(&str, &dyn Fn(&str) -> bool); 2] = [("on_bus.rs", &on_bus), ("every flag", &|id: &str| id != "bridge")];
    for (what, on) in flags {
        let lines = plan(&Running::rsd(&without, on), &production_remaps()).unwrap().lines();
        let has = |direction: &str, name: &str| lines.iter().any(|l| l.starts_with(&format!("{direction} {name} ")));
        for e in bridge.endpoints {
            let faced = match e.kind {
                Kind::Subscribe => !on_the_bus(e.name, Kind::Publish, on) || has("to_ros", e.name),
                Kind::Publish => !on_the_bus(e.name, Kind::Subscribe, on) || has("from_ros", e.name),
                Kind::Call => !on_the_bus(e.name, Kind::Serve, on) || has("service_to_ros", e.name),
                other => panic!("ws_bridge does not {}", other.verb()),
            };
            assert!(faced, "{what}: {} {} has no DDS face", e.kind.verb(), e.name);
        }
    }
}

/// The separate binary with `--backend bus` (RUST_DAEMON=false): a bus of its
/// own, and every name ws_bridge reaches carried by the bridge next to it,
/// with the QoS its r2r endpoints had.
#[test]
fn ws_bridge_standalone_carries_every_name_it_reaches() {
    let running = Running::standalone("bridge", "mower_ws_bridge").unwrap();
    let alone = plan(&running, &Remaps::default()).unwrap();
    let count = |direction: &str| alone.lines().iter().filter(|l| l.starts_with(&format!("{direction} "))).count();
    assert_eq!((count("from_ros"), count("to_ros"), count("service_from_ros")), (28, 4, 32));
    assert_eq!(alone.declarations.len(), 64);
    let lines = alone.lines();
    for line in [
        "from_ros /adapter/map_layers/map_grid std_msgs/msg/String R TL 10",
        "from_ros /adapter/robot_pose geometry_msgs/msg/PoseStamped R V 10",
        "from_ros /robot/online std_msgs/msg/Bool R TL 10",
        "from_ros /robot/telemetry std_msgs/msg/String R V 10",
        "from_ros /fix sensor_msgs/msg/NavSatFix R V 10",
        "from_ros /coverage_progress mower_interface/msg/CoverageProgress R TL 10",
        "to_ros /app_joy_cmd geometry_msgs/msg/TwistStamped R V 10",
        "service_from_ros /system/update std_srvs/srv/Trigger",
    ] {
        assert!(lines.iter().any(|l| l == line), "{line} in {lines:#?}");
    }
    assert!(alone.notes.is_empty(), "{:?}", alone.notes);
}

/// E2 steps 6 and 7 after E3: base, pid_autotune and the guards on the bus as
/// well. ws_bridge's names that only DDS had are then in the process: the
/// from_ros of /manual_command_clock and /pid_autotune/status and the
/// service_from_ros of /pid_autotune go. The recorder's six and
/// set_parameters stay (until E5, and until the parameter services move).
/// So do the to_ros of the three command topics: ws_bridge writes them on
/// the bus, and the policy gives every topic written on the bus a DDS face
/// (README.md, "The policy"), which carries nothing while no DDS node reads
/// them.
#[test]
fn steps_6_and_7_after_e3_leave_the_recorder_s_names_and_set_parameters_bridged() {
    let e3 = rsd_plan_with(&PRODUCTION_MODULES, &["bridge"], &production_remaps()).unwrap().lines();
    let all = lines(&MODULES.iter().map(|m| m.id).collect::<Vec<_>>());
    let gone: Vec<&String> = e3.iter().filter(|l| !all.contains(l)).collect();
    let came: Vec<&String> = all.iter().filter(|l| !e3.contains(l)).collect();
    for line in [
        "from_ros /manual_command_clock std_msgs/msg/Header R V 10",
        "from_ros /pid_autotune/status std_msgs/msg/String R TL 10",
        "service_from_ros /pid_autotune mower_interface/srv/PidAutotune",
    ] {
        assert!(gone.iter().any(|l| *l == line), "{line} goes: {gone:#?}");
    }
    for line in [
        "to_ros /manual_command_clock std_msgs/msg/Header R V 1",
        "to_ros /pid_autotune/status std_msgs/msg/String R TL 1",
        "service_to_ros /pid_autotune mower_interface/srv/PidAutotune",
    ] {
        assert!(came.iter().any(|l| *l == line), "{line} comes, for DDS: {came:#?}");
    }
    for line in [
        "from_ros /mower_recorder/bags std_msgs/msg/String R TL 10",
        "from_ros /mower_recorder/command_result std_msgs/msg/String R V 10",
        "from_ros /mower_recorder/status std_msgs/msg/String R TL 10",
        "to_ros /mower_recorder/command std_msgs/msg/String R V 10",
        "service_from_ros /mower_recorder/start std_srvs/srv/Trigger",
        "service_from_ros /mower_recorder/stop std_srvs/srv/Trigger",
        "service_from_ros /boustrophedon_coverage/set_parameters rcl_interfaces/srv/SetParameters",
        "to_ros /app_joy_cmd geometry_msgs/msg/TwistStamped R V 10",
        "to_ros /mower_base/blade_command std_msgs/msg/String R V 10",
        "to_ros /mower_base/servo_command std_msgs/msg/String R V 10",
    ] {
        assert!(all.iter().any(|l| l == line), "{line} stays: {all:#?}");
    }
    // nothing of ws_bridge comes in from DDS but the recorder's
    let from: Vec<&String> = all.iter().filter(|l| l.starts_with("from_ros ") && l.contains("/mower_recorder/")).collect();
    assert_eq!(from.len(), 3, "{from:#?}");
}

// ---------------------------------------------------------------------------
// The manifest
// ---------------------------------------------------------------------------

/// mower_rsd's module table, as its own test writes it, without `agent`,
/// which has no endpoint of its own.
#[test]
fn the_manifest_has_mower_rsd_s_modules_and_node_names() {
    let expected = [
        ("base", vec!["mower_base"]),
        ("status", vec!["robot_status"]),
        ("guards", vec!["manual_velocity_guard", "velocity_command_guard"]),
        ("imu", vec!["imu"]),
        ("gps", vec!["gps"]),
        ("localize", vec!["ekf_filter_node_odom", "ekf_filter_node_map", "navsat_transform"]),
        ("map", vec!["map_manage"]),
        ("coverage", vec!["boustrophedon_coverage"]),
        ("nav", vec!["nav_action_server"]),
        ("record", vec!["path_record_node"]),
        ("adapter", vec!["flutter_adapter"]),
        ("battery", vec!["battery_state"]),
        ("pid_autotune", vec!["pid_autotune"]),
        ("bridge", vec!["mower_ws_bridge"]),
    ];
    let got: Vec<(&str, Vec<&str>)> = MODULES.iter().map(|m| (m.id, m.instances.iter().map(|i| i.node).collect())).collect();
    assert_eq!(got, expected);
    let flags: Vec<&str> = ON_BUS.iter().map(|(id, _)| *id).collect();
    assert_eq!(flags, MODULES.iter().map(|m| m.id).collect::<Vec<_>>(), "one flag per module, in order");
}

#[test]
fn every_endpoint_resolves_to_its_production_name_with_the_production_rules() {
    let remaps = production_remaps();
    for module in MODULES {
        for instance in module.instances {
            for e in instance.endpoints {
                let resolved = remaps.resolve(e.kind.name_kind(), instance.node, "", e.code).unwrap();
                assert_eq!(resolved, e.name, "{} {} {}", instance.node, e.kind.verb(), e.code);
                // and without them, a relative name would come out otherwise
                if e.code != e.name {
                    assert!(!e.code.starts_with('/') || e.param.is_some(), "{}: {}", instance.node, e.code);
                }
            }
        }
    }
    for outside in OUTSIDE {
        for e in outside.endpoints {
            assert!(e.code.starts_with('/') && e.code == e.name, "{} {}", outside.node, e.code);
        }
    }
}

#[test]
fn the_manifest_is_consistent() {
    let mut types: BTreeMap<(Space, &str), Vec<String>> = BTreeMap::new();
    let mut servers: BTreeMap<(Space, &str), Vec<&str>> = BTreeMap::new();
    let all = MODULES
        .iter()
        .flat_map(|m| m.instances.iter().map(move |i| (m.id, i.node, i.endpoints)))
        .chain(OUTSIDE.iter().map(|o| ("outside", o.node, o.endpoints)));
    for (id, node, endpoints) in all {
        let mut seen = BTreeSet::new();
        for e in endpoints {
            assert!(seen.insert((e.kind, e.name)), "{node} lists {} {} twice", e.kind.verb(), e.name);
            assert_eq!(e.kind.is_topic(), matches!(e.iface, Iface::Topic(..)), "{node} {}", e.name);
            assert_eq!(e.kind.space() == Space::Action, matches!(e.iface, Iface::Action(_)), "{node} {}", e.name);
            let ros = e.ros_type();
            let r2r_type = ["/msg/", "/srv/", "/action/"].iter().any(|kind| ros.contains(kind));
            assert!(r2r_type, "{node} {}: not an r2r type: {ros}", e.name);
            types.entry((e.kind.space(), e.name)).or_default().push(ros);
            if matches!(e.kind, Kind::Serve | Kind::ServeAction) {
                servers.entry((e.kind.space(), e.name)).or_default().push(node);
            }
            assert!(e.only.is_none() || id == "nav", "{node}: only nav's nav_core endpoints are not in production");
            assert!(!e.silent || e.kind == Kind::Publish, "{node} {}", e.name);
        }
    }
    for ((_, name), mut ros) in types {
        ros.dedup();
        assert_eq!(ros.len(), 1, "{name} has more than one type: {ros:?}");
    }
    for ((_, name), nodes) in servers {
        assert_eq!(nodes.len(), 1, "{name} is served by {nodes:?}");
    }
}

/// robot.launch.py's `_DAEMON_REMAPS` and navsat's `gps/fix` rule, which the
/// launch file builds from `gps_fix_topic` and refuses to run with anything
/// but `/fix`.
#[test]
fn production_remaps_are_robot_launch_py_s() {
    let launch = std::fs::read_to_string(repo().join("src/mower_bringup/launch/robot.launch.py")).unwrap();
    let start = launch.find("_DAEMON_REMAPS = {").expect("_DAEMON_REMAPS");
    let block = &launch[start..start + launch[start..].find("\n}\n").expect("its end")];
    // module -> its rules: `'imu': ['imu:imu/data_raw:=imu/data'],` or a
    // `'guards': [` line with one rule per line below it
    let mut by_module: BTreeMap<String, Vec<String>> = BTreeMap::new();
    let mut module = String::new();
    for line in block.lines().skip(1) {
        let code = line.split('#').next().unwrap_or_default();
        for quoted in code.split('\'').skip(1).step_by(2) {
            if quoted.contains(":=") {
                by_module.entry(module.clone()).or_default().push(quoted.to_string());
            } else {
                module = quoted.to_string();
            }
        }
    }
    assert!(launch.contains("remaps = [r for module in modules for r in _DAEMON_REMAPS.get(module, [])]"));
    let mut argv: Vec<String> = PRODUCTION_MODULES.iter().flat_map(|m| by_module.remove(*m).unwrap_or_default()).collect();
    assert!(by_module.is_empty(), "rules of no module: {by_module:?}");
    assert!(launch.contains("remaps.append('navsat_transform:gps/fix:=' + LaunchConfiguration("));
    assert!(launch.contains("if gps_fix_topic.strip() != '/fix':"));
    argv.push("navsat_transform:gps/fix:=/fix".to_string());
    assert_eq!(argv, PRODUCTION_REMAPS);
}

/// The production value of each name a parameter sets (docs/BUS_TOPICS.md
/// 5.7), from the files robot.launch.py passes mower_rsd, in its order: a
/// later file wins. A parameter no file sets keeps the code's default, which
/// the manifest has as `code`.
#[test]
fn names_from_parameters_are_the_parameter_files_values() {
    let files = [
        "src/mower_bringup/config/mower_rsd.yaml",
        "src/mower_bringup/config/gps.yaml",
        "src/mower_nav2/config/dual_ekf_navsat_params.yaml",
    ];
    let mut values: BTreeMap<(String, String), String> = BTreeMap::new();
    for file in files {
        let text = std::fs::read_to_string(repo().join(file)).unwrap();
        let doc: serde_yaml::Value = serde_yaml::from_str(&text).unwrap();
        for (node, section) in doc.as_mapping().unwrap() {
            let Some(params) = section.get("ros__parameters").and_then(|p| p.as_mapping()) else { continue };
            for (key, value) in params {
                if let (Some(node), Some(key), Some(value)) = (node.as_str(), key.as_str(), value.as_str()) {
                    values.insert((node.to_string(), key.to_string()), value.to_string());
                }
            }
        }
    }
    let mut checked = 0;
    for module in MODULES {
        for instance in module.instances {
            for e in instance.endpoints {
                let Some(param) = e.param else { continue };
                if let Some(value) = values.get(&(instance.node.to_string(), param.to_string())) {
                    assert_eq!(value, e.code, "{} {param} ({file:?})", instance.node, file = files);
                    checked += 1;
                }
            }
        }
    }
    // robot_status (2), gps (2), the EKFs' inputs (5), the slow copies (2),
    // nav, record, adapter
    assert_eq!(checked, 14);
    assert_eq!(values[&("robot_status".into(), "heartbeat_source_topic".into())], "/odom_slow", "one subscription with odom_topic");
}

#[test]
fn profiles_read_as_the_bridge_and_the_bus_read_them() {
    assert_eq!(SD.bus(), Qos::system_default());
    assert_eq!(R_V_10.bus(), Qos::default());
    assert_eq!(BE_V_5.bus(), Qos::sensor_data());
    assert_eq!(R_TL_1.bus(), Qos::latched());
    assert_eq!(R_V_10.ros(), r2r::QosProfile::default());
    assert_eq!(BE_V_5.ros(), r2r::QosProfile::sensor_data());
    assert_eq!(R_TL_1.ros(), r2r::QosProfile::default().keep_last(1).reliable().transient_local());
    assert_eq!(SD.ros(), r2r::QosProfile::system_default());
    let shown: Vec<String> = [SD, R_V_1, BE_V_1, R_TL_100].iter().map(Profile::to_string).collect();
    assert_eq!(shown, ["SD", "R V 1", "BE V 1", "R TL 100"]);
}

// ---------------------------------------------------------------------------
// The check
// ---------------------------------------------------------------------------

/// Some of robot_status's endpoints, as its conversion would make them.
struct Status {
    scope: mower_bus::Scope,
    _online: mower_bus::Publisher<Bool>,
    _telemetry: mower_bus::Subscriber<Text>,
    _update: mower_bus::Service<Trigger::Request, Trigger::Response>,
}

fn converted_status(bus: &Bus) -> Status {
    let scope = bus.scope("robot_status");
    Status {
        _online: scope.publisher::<Bool>("/robot/online", R_TL_1.bus()).unwrap(),
        _telemetry: scope.subscribe::<Text>("/mower_base/telemetry", BE_V_1.bus()).unwrap(),
        _update: scope.service::<Trigger::Request, Trigger::Response>("/system/update").unwrap(),
        scope,
    }
}

#[test]
fn the_check_finds_no_gap_in_a_module_as_the_manifest_has_it() {
    let bus = Bus::new();
    let _status = converted_status(&bus);
    let _bridge = bus.scope(NODE).subscribe::<Bool>("/robot/online", Qos::latched().keep_last(64)).unwrap();
    let running = Running::rsd(&["status"], |_| true);
    assert_eq!(coverage::gaps(&bus, &running, &production_remaps(), &[NODE]), Vec::<String>::new());
}

#[test]
fn the_check_names_every_endpoint_the_bridge_would_not_carry() {
    let bus = Bus::new();
    let status = converted_status(&bus);
    let status = &status.scope;
    // a name the manifest does not have, a QoS it does not, another
    // instance's endpoint, an endpoint that stays on r2r
    let _a = status.publisher::<Bool>("/robot/onlin", R_TL_1.bus()).unwrap();
    let _b = status.publisher::<Text>("/robot/telemetry", Qos::default().keep_last(1)).unwrap();
    let _c = status.client::<Trigger::Request, Trigger::Response>("/system/update").unwrap();
    let _d = bus.scope("navsat_transform").publisher::<r2r::tf2_msgs::msg::TFMessage>("/tf_static", R_TL_1.bus()).unwrap();
    // a module whose flag is false, and a scope nobody knows
    let _e = bus.scope("battery_state").publisher::<Text>("/mower_recorder/command", Qos::default()).unwrap();
    let _f = bus.scope("mower_ws_bridge").subscribe::<Text>("/robot/info", Qos::default()).unwrap();
    let running = Running::rsd(&["status", "localize", "battery"], |id| id != "battery");
    let gaps = coverage::gaps(&bus, &running, &production_remaps(), &[NODE]);
    assert_eq!(
        gaps,
        [
            "battery_state publishes /mower_recorder/command (std_msgs/msg/String, R V 10) on the bus, but module \
             battery is not on the bus (crates/mower_bridge/src/on_bus.rs), so nothing bridges it",
            "mower_ws_bridge subscribes to /robot/info (std_msgs/msg/String, R V 10) on the bus, but its scope is no \
             instance of the manifest, so nothing bridges it",
            "navsat_transform publishes /tf_static (tf2_msgs/msg/TFMessage, R TL 1) on the bus, but the manifest keeps \
             it on the module's r2r node",
            "robot_status calls /system/update (std_srvs/srv/Trigger) on the bus, but the manifest has no such endpoint \
             for robot_status, so nothing bridges it",
            "robot_status publishes /robot/onlin (std_msgs/msg/Bool, R TL 1) on the bus, but the manifest has no such \
             endpoint for robot_status, so nothing bridges it",
            "robot_status publishes /robot/telemetry (std_msgs/msg/String, R V 1) on the bus, but the manifest has it \
             as R V 10",
        ]
    );
}

/// The helper for the modules' own tests: names resolve as the module
/// resolved them, other scopes are the test's.
#[test]
fn a_module_s_test_checks_its_endpoints_against_the_manifest() {
    let argv: Vec<String> = ["--ros-args", "-r", "__node:=manual_velocity_guard", "-r", "cmd_vel_out:=/joy_cmd"]
        .map(str::to_string)
        .to_vec();
    let remaps = std::sync::Arc::new(Remaps::from_args(&argv).unwrap());
    let bus = Bus::new();
    let m = ModuleCtx::new("velocity_command_guard", Shutdown::new())
        .with_bus(bus.scope("velocity_command_guard"))
        .with_remaps(remaps);
    let out = m.resolve_topic("cmd_vel_out").unwrap();
    assert_eq!(out, "/joy_cmd");
    let _out = m.bus.publisher::<r2r::geometry_msgs::msg::TwistStamped>(&out, R_V_1.bus()).unwrap();
    let _test = bus.scope("test").publisher::<Text>("/anything", Qos::default()).unwrap();
    coverage::assert_in_manifest(&m);
    let _debug = m.bus.publisher::<r2r::std_msgs::msg::Header>(&m.resolve_topic("guard/debug").unwrap(), R_V_1.bus());
    assert_eq!(
        coverage::gaps_of(&m),
        ["velocity_command_guard publishes /guard/debug (std_msgs/msg/Header, R V 1) on the bus, but the manifest \
          has no such endpoint for manual_velocity_guard, so nothing bridges it"]
    );
    let unknown = ModuleCtx::new("joystick", Shutdown::new());
    assert_eq!(coverage::gaps_of(&unknown).len(), 1);
}
