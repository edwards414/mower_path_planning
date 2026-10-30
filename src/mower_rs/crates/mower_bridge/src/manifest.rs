//! Every endpoint of every mower_rsd module instance, and of the nodes
//! outside that a module depends on (README.md, "The manifest").
//!
//! - **The module code is the source of truth.** docs/BUS_TOPICS.md sections
//!   2 and 3 cite the lines; the QoS is the profile the code hands r2r.
//! - **Names:** `code` is what the code passes in production (a literal, or
//!   the production value of the parameter named), `name` the absolute name
//!   rcl makes of it with robot.launch.py's rules ([`PRODUCTION_REMAPS`];
//!   docs/BUS_TOPICS.md 5.4, 5.7). The tests check both against rcl, the
//!   parameter files and the launch file.
//! - **Not listed:** parameter services, which stay on r2r in E2 (the
//!   adapter's calls to two parameter services stay there too, and are
//!   listed so marked: `stays_on_r2r`, as are nav's clients of nav2's
//!   actions and of `/bt_navigator/get_state`, until E9). The one exception
//!   is coverage's `/boustrophedon_coverage/set_parameters`, which the app
//!   calls through ws_bridge: listed as a server that stays on r2r (E3
//!   decision 9). Nor the endpoints a module creates only with a
//!   non-production setting, except nav's `nav_core` backend (marked
//!   `only`), which CI runs.
//! - **Actions** are listed like services: nav serves `/nav_action` and
//!   `/nav_action_follow_path` on the bus, and coverage sends its goals to the
//!   second (step 5).
//! - **ws_bridge** (module `bridge`, E3) is what its allow-list reaches
//!   (mower_bringup/config/ws_bridge.yaml): the test
//!   `the_bridge_module_is_ws_bridge_yaml_s_allow_list` keeps the two in step.

use r2r::diagnostic_msgs::msg::DiagnosticArray;
use r2r::geometry_msgs::msg::{PoseStamped, TwistStamped};
use r2r::lifecycle_msgs::srv::GetState;
use r2r::mower_interface::action::Waypoint;
use r2r::mower_interface::msg::CoverageProgress;
use r2r::mower_interface::srv::{
    CancelNavigationDispatch, ChannelPathList, ChannelRoute, ChennalPathList, ConfirmNavigationDispatch, EditZone,
    GetCoverageProgress, GetZoneList, ImportImageMask, MissionOperationLock, PidAutotune, ResumeCoverage, SiteOp,
    ZoneExecPath, ZoneMapList, ZoneSequence,
};
use r2r::nav2_msgs::action::{FollowPath, NavigateToPose};
use r2r::nav_msgs::msg::{OccupancyGrid, Odometry, Path};
use r2r::rcl_interfaces::msg::Log;
use r2r::rcl_interfaces::srv::{GetParameters, SetParameters};
use r2r::robot_localization::srv::{FromLL, FromLLArray, SetDatum, ToLL};
use r2r::sensor_msgs::msg::{BatteryState, Imu, JointState, NavSatFix};
use r2r::std_msgs::msg::{Bool, Header, String as Text};
use r2r::std_srvs::srv::Trigger;
use r2r::tf2_msgs::msg::TFMessage;
use r2r::visualization_msgs::msg::{Marker, MarkerArray};

use crate::types::*;

/// `nav_backend: nav_core` (robot.launch.py `rust_nav_core:=true`, CI's
/// nav_core_check.py).
const NAV_CORE: &str = "nav_backend: nav_core";
/// The same with `nav_core.pose_source: tf`, which only nav_core_check.py sets.
const NAV_CORE_TF: &str = "nav_backend: nav_core and nav_core.pose_source: tf";

/// The `-r` rules robot.launch.py passes mower_rsd in production, in argv
/// order: `_DAEMON_REMAPS` in the order of the modules, then navsat's
/// `gps/fix` rule with the `gps_fix_topic` the launch file pins. With every
/// switch on, E4's (off by default) included: a rule scoped to a node of a
/// module that does not run names nothing.
pub const PRODUCTION_REMAPS: &[&str] = &[
    "manual_velocity_guard:cmd_vel_in:=/app_joy_cmd",
    "manual_velocity_guard:cmd_vel_out:=/joy_cmd",
    "manual_velocity_guard:command_clock:=/manual_command_clock",
    "velocity_command_guard:cmd_vel_in:=/cmd_vel_guard_input",
    "velocity_command_guard:cmd_vel_out:=/drivetrain_guarded_cmd_vel",
    // rust_mux (E4, off by default): twist_mux.launch.py's /cmd_vel_out rule
    "twist_mux:cmd_vel_out:=/cmd_vel_guard_input",
    "imu:imu/data_raw:=imu/data",
    "ekf_filter_node_odom:odometry/filtered:=odometry/local",
    "ekf_filter_node_odom:imu:=imu/data",
    "ekf_filter_node_map:odometry/filtered:=odometry/global",
    "ekf_filter_node_map:imu:=imu/data",
    "navsat_transform:imu:=imu/data",
    "navsat_transform:odometry/filtered:=odometry/global",
    "navsat_transform:gps/fix:=/fix",
];

/// [`PRODUCTION_REMAPS`] as rcl reads them.
pub fn production_remaps() -> mower_rs_common::Remaps {
    let mut argv = vec!["--ros-args".to_string()];
    for rule in PRODUCTION_REMAPS {
        argv.push("-r".to_string());
        argv.push(rule.to_string());
    }
    mower_rs_common::Remaps::from_args(&argv).expect("the production rules parse")
}

/// The modules of mower_rsd, in its module table's order. `agent` has no
/// endpoint of its own (it reaches the graph through ws_bridge's loopback
/// WebSocket) and is not listed.
pub static MODULES: &[Module] = &[
    Module {
        id: "base",
        instances: &[Instance {
            node: "mower_base",
            // The C++ chain's profiles (mower_base/src/lib.rs, "QoS"). Every
            // topic is a parameter whose production value is its default.
            endpoints: &[
                publishes::<Odometry>("/odom", SD).param("odom_topic"),
                // topic_tools' QoS for the slow copy: R V 10 (throttled_odom_qos)
                publishes::<Odometry>("/odom_slow", R_V_10).param("odom_slow_topic"),
                publishes::<JointState>("/joint_states", SD).param("joint_states_topic"),
                // enable_odom_tf is false: the EKFs own odom -> base_footprint
                publishes::<TFMessage>("/tf", SD).param("tf_topic").silent(),
                publishes::<Text>("/mower_base/telemetry", BE_V_1).param("telemetry_topic"),
                publishes::<Text>("/mower_base/firmware_info", R_TL_1).param("firmware_info_topic"),
                subscribes::<TwistStamped>("/drivetrain_guarded_cmd_vel", SD).param("cmd_vel_topic"),
                subscribes::<Text>("/mower_base/led_command", R_TL_1).param("led_topic"),
                subscribes::<Text>("/mower_base/pid_command", R_V_4).param("pid_topic"),
                subscribes::<Text>("/mower_base/wheel_override", BE_V_1).param("override_topic"),
                subscribes::<Text>("/mower_base/servo_command", BE_V_1).param("servo_topic"),
                subscribes::<Text>("/mower_base/blade_command", BE_V_1).param("blade_topic"),
            ],
        }],
    },
    Module {
        id: "status",
        instances: &[Instance {
            node: "robot_status",
            endpoints: &[
                publishes::<Bool>("/robot/online", R_TL_1),
                publishes::<Text>("/robot/info", R_TL_1),
                publishes::<Text>("/robot/telemetry", R_V_10),
                publishes::<Text>("/mower_base/led_command", R_TL_1).param("led_topic"),
                // heartbeat_source_topic is /odom_slow too (mower_rsd.yaml),
                // and the two share one subscription
                subscribes::<Odometry>("/odom_slow", R_V_10).param("odom_topic"),
                subscribes::<NavSatFix>("/fix", BE_V_5).param("gps_fix_topic"),
                subscribes::<NavSatFix>("/gps/filtered", BE_V_5).param("gps_filtered_topic"),
                subscribes::<Imu>("/imu/data", BE_V_5).param("imu_topic"),
                subscribes::<Text>("/mower_base/telemetry", BE_V_1).param("base_telemetry_topic"),
                subscribes::<BatteryState>("/battery_state", R_V_10).param("battery_topic"),
                subscribes::<BatteryState>("/aon_battery_state", R_V_10).param("aon_battery_topic"),
                subscribes::<Text>("/mower_base/firmware_info", R_TL_1).param("firmware_info_topic"),
                subscribes::<Bool>("/nav_operation_active", R_TL_1).param("nav_active_topic"),
                serves::<Trigger::Service>("/system/update"),
                serves::<Trigger::Service>("/system/restart"),
                serves::<Trigger::Service>("/system/check_update"),
            ],
        }],
    },
    Module {
        id: "guards",
        // One code, two nodes; relative names, remapped per node.
        instances: &[
            Instance {
                node: "manual_velocity_guard",
                endpoints: &[
                    publishes::<TwistStamped>("cmd_vel_out", R_V_1).named("/joy_cmd"),
                    subscribes::<TwistStamped>("cmd_vel_in", R_V_1).named("/app_joy_cmd"),
                    // only with require_command_session, which this guard has
                    publishes::<Header>("command_clock", R_V_1).named("/manual_command_clock"),
                ],
            },
            Instance {
                node: "velocity_command_guard",
                endpoints: &[
                    publishes::<TwistStamped>("cmd_vel_out", R_V_1).named("/drivetrain_guarded_cmd_vel"),
                    subscribes::<TwistStamped>("cmd_vel_in", R_V_1).named("/cmd_vel_guard_input"),
                ],
            },
        ],
    },
    Module {
        // E4: twist_mux_topics.yaml's handles, the same file the C++ node
        // reads; rclcpp::SystemDefaultsQoS() readers, R V 1 on Cyclone
        // (measured), and rclcpp::QoS(KeepLast(1)) for the output and
        // diagnostic_updater's /diagnostics. Born on the bus.
        id: "mux",
        instances: &[Instance {
            node: "twist_mux",
            endpoints: &[
                subscribes::<TwistStamped>("/joy_cmd", SD).param("topics.app_manual.topic"),
                subscribes::<TwistStamped>("/keyboard_cmd_vel", SD).param("topics.keyboard.topic"),
                subscribes::<TwistStamped>("/nav_cmd_vel", SD).param("topics.navigation.topic"),
                subscribes::<TwistStamped>("/navigation_safety_stop", SD).param("topics.navigation_safety_stop.topic"),
                subscribes::<TwistStamped>("/physical_joy_cmd", SD).param("topics.physical_joystick.topic"),
                subscribes::<Bool>("/navigation_coordinator_lock", SD).param("locks.navigation_coordinator.topic"),
                publishes::<TwistStamped>("cmd_vel_out", R_V_1).named("/cmd_vel_guard_input"),
                publishes::<DiagnosticArray>("/diagnostics", R_V_1),
            ],
        }],
    },
    Module {
        id: "imu",
        instances: &[Instance {
            node: "imu",
            endpoints: &[publishes::<Imu>("imu/data_raw", R_V_10).named("/imu/data")],
        }],
    },
    Module {
        id: "gps",
        instances: &[Instance {
            node: "gps",
            endpoints: &[
                publishes::<NavSatFix>("/fix", R_V_10).param("fix_topic"),
                publishes::<Text>("gps/status", R_V_10).param("status_topic").named("/gps/status"),
            ],
        }],
    },
    Module {
        // E4: robot_state_publisher 3.3.4's endpoints (measured,
        // tools/rsp_capture.py), robot_description from robot.launch.py.
        // Born on the bus.
        id: "static_tf",
        instances: &[Instance {
            node: "static_tf_publisher",
            endpoints: &[
                publishes::<TFMessage>("/tf_static", R_TL_1),
                publishes::<TFMessage>("/tf", R_V_100),
                publishes::<Text>("robot_description", R_TL_1).named("/robot_description"),
                subscribes::<JointState>("joint_states", BE_V_5).named("/joint_states"),
            ],
        }],
    },
    Module {
        id: "localize",
        // Upstream's names, remapped per node; the EKF inputs are the
        // parameter values of dual_ekf_navsat_params.yaml (odomN, imuN), with
        // the depth of their queue_size.
        instances: &[
            Instance {
                node: "ekf_filter_node_odom",
                endpoints: &[
                    subscribes::<TFMessage>("/tf_static", R_TL_100),
                    subscribes::<Odometry>("odom", BE_V_10).param("odom0").named("/odom"),
                    subscribes::<Imu>("imu", BE_V_10).param("imu0").named("/imu/data"),
                    publishes::<Odometry>("odometry/filtered", R_V_10).named("/odometry/local"),
                    publishes::<TFMessage>("/tf", R_V_100),
                ],
            },
            Instance {
                node: "ekf_filter_node_map",
                endpoints: &[
                    subscribes::<TFMessage>("/tf_static", R_TL_100),
                    subscribes::<Odometry>("odom", BE_V_10).param("odom0").named("/odom"),
                    subscribes::<Odometry>("odometry/gps", BE_V_10).param("odom1").named("/odometry/gps"),
                    subscribes::<Imu>("imu", BE_V_10).param("imu0").named("/imu/data"),
                    publishes::<Odometry>("odometry/filtered", R_V_10).named("/odometry/global"),
                    publishes::<Odometry>("/odometry/global_slow", R_V_10).param("odometry_slow_topic"),
                    publishes::<TFMessage>("/tf", R_V_100),
                ],
            },
            Instance {
                node: "navsat_transform",
                endpoints: &[
                    subscribes::<TFMessage>("/tf_static", R_TL_100),
                    subscribes::<Odometry>("odometry/filtered", BE_V_1).named("/odometry/global"),
                    subscribes::<NavSatFix>("gps/fix", BE_V_1).named("/fix"),
                    subscribes::<Imu>("imu", BE_V_1).named("/imu/data"),
                    publishes::<Odometry>("odometry/gps", R_V_10).named("/odometry/gps"),
                    publishes::<NavSatFix>("gps/filtered", R_V_10).named("/gps/filtered"),
                    // The utm frame, on r2r: /tf_static goes DDS -> bus
                    // (mower_bus_ros README, option 1), or, with E4's
                    // static_tf on the bus, only bus -> DDS, and the EKFs do
                    // not need utm (option A: on the bus with E5 and E9)
                    publishes::<TFMessage>("/tf_static", R_TL_1).stays_on_r2r().unread_on_bus(),
                    serves::<ToLL::Service>("/toLL"),
                    serves::<FromLL::Service>("/fromLL"),
                    serves::<FromLLArray::Service>("/fromLLArray"),
                    serves::<SetDatum::Service>("/datum"),
                ],
            },
        ],
    },
    Module {
        id: "map",
        instances: &[Instance {
            node: "map_manage",
            endpoints: &[
                publishes::<OccupancyGrid>("/free_space", R_TL_1),
                publishes::<OccupancyGrid>("/free_space_inflated", R_TL_1),
                publishes::<OccupancyGrid>("/risk_map", R_TL_1),
                publishes::<OccupancyGrid>("/risk_map_inflated", R_TL_1),
                publishes::<OccupancyGrid>("/chennal_map", R_TL_1),
                publishes::<OccupancyGrid>("/chennal_map_inflated", R_TL_1),
                publishes::<OccupancyGrid>("/map_grid", R_TL_1),
                publishes::<OccupancyGrid>("/map_grid_global", R_TL_1),
                calls::<MissionOperationLock::Service>("/mission_operation_lock"),
                calls::<GetZoneList::Service>("/get_risk_zone_list"),
                calls::<GetZoneList::Service>("/get_record_zone_list"),
                calls::<ChennalPathList::Service>("/get_chennal_path_list"),
                subscribes::<Bool>("/nav_operation_active", R_TL_1),
                serves::<Trigger::Service>("/create_risk_map"),
                serves::<Trigger::Service>("/create_free_space"),
                serves::<Trigger::Service>("/create_chennal_map"),
                serves::<Trigger::Service>("/restore_free_space_coverage"),
                serves::<ImportImageMask::Service>("/import_image_mask"),
                serves::<ZoneMapList::Service>("/get_zone_map_list_srv"),
            ],
        }],
    },
    Module {
        id: "coverage",
        instances: &[Instance {
            node: "boustrophedon_coverage",
            endpoints: &[
                calls::<MissionOperationLock::Service>("/mission_operation_lock"),
                publishes::<Path>("/coverage_path", R_TL_1),
                publishes::<MarkerArray>("/coverage_path_markers", R_TL_1),
                publishes::<MarkerArray>("/coverage_invalid_segments", R_TL_1),
                publishes::<MarkerArray>("/coverage_connectors", R_TL_1),
                // its two writers that never published, /free_space_inflated
                // and /risk_map_inflated, went with the move to the bus
                // (docs/BUS_TOPICS.md 5.8 #2)
                publishes::<Text>("/zone_sequence_status", R_TL_1),
                calls::<ZoneMapList::Service>("/get_zone_map_list_srv"),
                calls::<ConfirmNavigationDispatch::Service>("/confirm_navigation_dispatch"),
                calls::<CancelNavigationDispatch::Service>("/cancel_navigation_dispatch"),
                calls::<Trigger::Service>("/check_nav_status"),
                calls::<GetCoverageProgress::Service>("/coverage_progress_status"),
                calls::<ChannelRoute::Service>("/get_channel_route"),
                subscribes::<Bool>("/nav_operation_active", R_TL_1),
                subscribes::<OccupancyGrid>("/risk_map", R_TL_1),
                subscribes::<OccupancyGrid>("/risk_map_inflated", R_TL_1),
                serves::<Trigger::Service>("/generate_coverage_path"),
                serves::<ZoneExecPath::Service>("/zone_exec_path"),
                serves::<ResumeCoverage::Service>("/resume_coverage"),
                serves::<ZoneSequence::Service>("/run_zone_sequence"),
                serves::<Trigger::Service>("/stop_zone_sequence"),
                calls_action::<Waypoint::Action>("nav_action_follow_path").named("/nav_action_follow_path"),
                // A parameter service, served on its r2r node like the other
                // five (E2 decision 2). Listed because the app calls it
                // through ws_bridge (E3 decision 9).
                serves::<SetParameters::Service>("/boustrophedon_coverage/set_parameters").stays_on_r2r(),
            ],
        }],
    },
    Module {
        id: "nav",
        instances: &[Instance {
            node: "nav_action_server",
            endpoints: &[
                publishes::<Bool>("/nav_operation_active", R_TL_1),
                publishes::<TwistStamped>("/navigation_safety_stop", R_V_10),
                publishes::<Bool>("/navigation_coordinator_lock", R_V_10),
                publishes::<Path>("/split_path", R_V_1),
                publishes::<Marker>("/coverage_split_points", R_V_1),
                publishes::<CoverageProgress>("/coverage_progress", R_TL_1),
                serves_action::<Waypoint::Action>("nav_action").named("/nav_action"),
                serves_action::<Waypoint::Action>("nav_action_follow_path").named("/nav_action_follow_path"),
                // nav2 backend (production): its clients of nav2 stay on r2r
                // until E9
                calls::<GetState::Service>("bt_navigator/get_state").named("/bt_navigator/get_state").stays_on_r2r(),
                calls_action::<NavigateToPose::Action>("navigate_to_pose").named("/navigate_to_pose").stays_on_r2r(),
                calls_action::<FollowPath::Action>("follow_path").named("/follow_path").stays_on_r2r(),
                publishes::<TwistStamped>("/nav_cmd_vel", R_V_10).param("nav_core.cmd_vel_topic").only(NAV_CORE),
                publishes::<Path>("/plan", R_V_1).param("nav_core.plan_topic").only(NAV_CORE),
                subscribes::<Odometry>("/odom", R_V_10).param("nav_core.odom_topic").only(NAV_CORE),
                subscribes::<OccupancyGrid>("/map_grid", R_TL_1).param("nav_core.map_topic").only(NAV_CORE),
                subscribes::<OccupancyGrid>("/map_grid_global", R_TL_1)
                    .param("nav_core.global_map_topic")
                    .only(NAV_CORE),
                subscribes::<TFMessage>("/tf", R_V_100).only(NAV_CORE_TF),
                subscribes::<TFMessage>("/tf_static", R_TL_100).only(NAV_CORE_TF),
                // manual sources; the last two have no publisher unless teleop is on
                subscribes::<TwistStamped>("/joy_cmd", R_V_10),
                subscribes::<TwistStamped>("/physical_joy_cmd", R_V_10),
                subscribes::<TwistStamped>("/keyboard_cmd_vel", R_V_10),
                subscribes::<PoseStamped>("/adapter/robot_pose", R_V_10),
                subscribes::<Imu>("/imu/data", BE_V_5).param("imu_topic"),
                subscribes::<NavSatFix>("/fix", BE_V_5).param("gps_fix_topic"),
                subscribes::<Odometry>("/odometry/gps", R_V_10).param("gps_odometry_topic"),
                // nav2 backend: nav2's warnings for the failure messages
                subscribes::<Log>("/rosout", R_V_100),
                serves::<Trigger::Service>("/cancel_nav2"),
                serves::<Trigger::Service>("/cencel_nav2"),
                serves::<GetCoverageProgress::Service>("/coverage_progress_status"),
                serves::<Trigger::Service>("/check_nav_status"),
                serves::<ConfirmNavigationDispatch::Service>("/confirm_navigation_dispatch"),
                serves::<CancelNavigationDispatch::Service>("/cancel_navigation_dispatch"),
                serves::<MissionOperationLock::Service>("/mission_operation_lock"),
            ],
        }],
    },
    Module {
        id: "record",
        instances: &[Instance {
            node: "path_record_node",
            endpoints: &[
                publishes::<Path>("/recorded_path", R_V_10),
                publishes::<Marker>("/zone_markers", R_TL_1),
                publishes::<MarkerArray>("/zone_list", R_TL_1),
                publishes::<Marker>("/risk_zone_markers", R_TL_1),
                publishes::<MarkerArray>("/risk_zone_list", R_TL_1),
                publishes::<Path>("/chennal_path", R_V_10),
                publishes::<Path>("/channel_path", R_V_10),
                publishes::<MarkerArray>("/chennal_path_array", R_TL_1),
                publishes::<MarkerArray>("/channel_path_array", R_TL_1),
                publishes::<Text>("/site_list", R_TL_1),
                // nothing reads it since the Rust base (docs/BUS_TOPICS.md 5.8 #1)
                publishes::<Text>("/mower_base/rear_light", R_TL_1).param("rear_light_topic"),
                calls::<MissionOperationLock::Service>("/mission_operation_lock"),
                subscribes::<Odometry>("/odometry/global_slow", R_V_10).param("robot_pose_source_topic"),
                subscribes::<Text>("/adapter/map_datum", R_TL_1),
                subscribes::<Bool>("/nav_operation_active", R_TL_1),
                serves::<Trigger::Service>("/record_zone_start"),
                serves::<Trigger::Service>("/record_zone_end"),
                serves::<Trigger::Service>("/risk_zone_start"),
                serves::<Trigger::Service>("/risk_zone_end"),
                serves::<Trigger::Service>("/channel_record_start"),
                serves::<Trigger::Service>("/chennal_record_start"),
                serves::<Trigger::Service>("/channel_record_end"),
                serves::<Trigger::Service>("/chennal_record_end"),
                serves::<Trigger::Service>("/record_cancel"),
                serves::<Trigger::Service>("/save_zone_list"),
                serves::<Trigger::Service>("/load_zone_list"),
                serves::<Trigger::Service>("/get_record_zone_info"),
                serves::<EditZone::Service>("/edit_zone"),
                serves::<GetZoneList::Service>("/get_record_zone_list"),
                serves::<GetZoneList::Service>("/get_risk_zone_list"),
                serves::<ChannelPathList::Service>("/get_channel_path_list"),
                serves::<ChennalPathList::Service>("/get_chennal_path_list"),
                serves::<ChannelRoute::Service>("/get_channel_route"),
                serves::<SiteOp::Service>("/site_op"),
            ],
        }],
    },
    Module {
        id: "adapter",
        instances: &[Instance {
            node: "flutter_adapter",
            // Each relay: its source, then its JSON layer.
            endpoints: &[
                subscribes::<OccupancyGrid>("/map_grid", R_TL_1),
                publishes::<Text>("/adapter/map_layers/map_grid", R_TL_1),
                subscribes::<OccupancyGrid>("/free_space_inflated", R_TL_1),
                publishes::<Text>("/adapter/map_layers/free_space_inflated", R_TL_1),
                subscribes::<OccupancyGrid>("/risk_map_inflated", R_TL_1),
                publishes::<Text>("/adapter/map_layers/risk_map_inflated", R_TL_1),
                subscribes::<OccupancyGrid>("/chennal_map_inflated", R_TL_1),
                publishes::<Text>("/adapter/map_layers/chennal_map_inflated", R_TL_1),
                subscribes::<MarkerArray>("/zone_list", R_TL_1),
                publishes::<Text>("/adapter/marker_layers/zones", R_TL_1),
                subscribes::<MarkerArray>("/risk_zone_list", R_TL_1),
                publishes::<Text>("/adapter/marker_layers/risk_zones", R_TL_1),
                subscribes::<MarkerArray>("/chennal_path_array", R_TL_1),
                publishes::<Text>("/adapter/marker_layers/channels", R_TL_1),
                subscribes::<MarkerArray>("/coverage_path_markers", R_TL_1),
                publishes::<Text>("/adapter/marker_layers/coverage_path", R_TL_1),
                subscribes::<MarkerArray>("/coverage_invalid_segments", R_TL_1),
                publishes::<Text>("/adapter/marker_layers/invalid_segments", R_TL_1),
                subscribes::<MarkerArray>("/coverage_connectors", R_TL_1),
                publishes::<Text>("/adapter/marker_layers/connectors", R_TL_1),
                subscribes::<Odometry>("/odometry/global_slow", R_V_10).param("robot_pose_source_topic"),
                publishes::<PoseStamped>("/adapter/robot_pose", R_V_10),
                // parameter services: called on r2r, where they are served
                calls::<GetParameters::Service>("/boustrophedon_coverage/get_parameters").stays_on_r2r(),
                calls::<GetParameters::Service>("/map_manage/get_parameters").stays_on_r2r(),
                publishes::<Text>("/adapter/coverage_settings", R_TL_1),
                calls::<ZoneMapList::Service>("/get_zone_map_list_srv"),
                publishes::<Text>("/adapter/zone_summaries", R_TL_1),
                calls::<ToLL::Service>("/toLL"),
                publishes::<Text>("/adapter/map_datum", R_TL_1),
            ],
        }],
    },
    Module {
        id: "battery",
        instances: &[Instance {
            node: "battery_state",
            endpoints: &[
                subscribes::<Text>("/mower_base/telemetry", BE_V_1).param("base_telemetry_topic"),
                publishes::<BatteryState>("/battery_state", R_V_10).param("battery_topic"),
                publishes::<BatteryState>("/aon_battery_state", R_V_10).param("aon_battery_topic"),
            ],
        }],
    },
    Module {
        id: "pid_autotune",
        instances: &[Instance {
            node: "pid_autotune",
            endpoints: &[
                publishes::<Text>("/pid_autotune/status", R_TL_1).param("status_topic"),
                publishes::<Text>("/mower_base/pid_command", R_V_4).param("pid_topic"),
                publishes::<Text>("/mower_base/wheel_override", BE_V_1).param("override_topic"),
                publishes::<Text>("/mower_base/led_command", R_TL_1).param("led_topic"),
                calls::<MissionOperationLock::Service>("/mission_operation_lock").param("lock_service"),
                subscribes::<Bool>("/nav_operation_active", R_V_10).param("nav_active_topic"),
                // only during a tuning session
                subscribes::<Text>("/mower_base/telemetry", BE_V_1).param("telemetry_topic"),
                serves::<PidAutotune::Service>("/pid_autotune"),
            ],
        }],
    },
    Module {
        // ws_bridge, the app's door (E3). Its allow-list, ws_bridge.yaml in
        // its order: topics_sub's globs expanded to the names that have a
        // writer here or outside, read as the bus backend reads them (an
        // erased reader per topic, R TL 10, while a client subscribes);
        // topics_pub, written with a JSON writer made at the first publish,
        // R V 10, as the r2r backend's untyped writer was; services, each
        // called with a JSON client. Its flag is mower_rsd's run-time switch
        // (`--backend bus`).
        id: "bridge",
        instances: &[Instance {
            node: "mower_ws_bridge",
            endpoints: &[
                // topics_sub: '/adapter/*'
                subscribes::<Text>("/adapter/map_layers/map_grid", R_TL_10),
                subscribes::<Text>("/adapter/map_layers/free_space_inflated", R_TL_10),
                subscribes::<Text>("/adapter/map_layers/risk_map_inflated", R_TL_10),
                subscribes::<Text>("/adapter/map_layers/chennal_map_inflated", R_TL_10),
                subscribes::<Text>("/adapter/marker_layers/zones", R_TL_10),
                subscribes::<Text>("/adapter/marker_layers/risk_zones", R_TL_10),
                subscribes::<Text>("/adapter/marker_layers/channels", R_TL_10),
                subscribes::<Text>("/adapter/marker_layers/coverage_path", R_TL_10),
                subscribes::<Text>("/adapter/marker_layers/invalid_segments", R_TL_10),
                subscribes::<Text>("/adapter/marker_layers/connectors", R_TL_10),
                // also '/*/robot_pose'
                subscribes::<PoseStamped>("/adapter/robot_pose", R_TL_10),
                subscribes::<Text>("/adapter/coverage_settings", R_TL_10),
                subscribes::<Text>("/adapter/zone_summaries", R_TL_10),
                subscribes::<Text>("/adapter/map_datum", R_TL_10),
                subscribes::<Text>("/site_list", R_TL_10),
                // also '/*/online'
                subscribes::<Bool>("/robot/online", R_TL_10),
                subscribes::<Text>("/robot/info", R_TL_10),
                subscribes::<Text>("/robot/telemetry", R_TL_10),
                subscribes::<Text>("/pid_autotune/status", R_TL_10),
                subscribes::<BatteryState>("/battery_state", R_TL_10),
                subscribes::<NavSatFix>("/fix", R_TL_10),
                // '/gps/fix' has no writer (docs/BUS_TOPICS.md 5.8 #5), nor
                // has any name of '/*/battery_state'
                subscribes::<Text>("/gps/status", R_TL_10),
                subscribes::<Header>("/manual_command_clock", R_TL_10),
                subscribes::<Text>("/mower_recorder/status", R_TL_10),
                subscribes::<Text>("/mower_recorder/bags", R_TL_10),
                subscribes::<Text>("/mower_recorder/command_result", R_TL_10),
                subscribes::<CoverageProgress>("/coverage_progress", R_TL_10),
                subscribes::<Text>("/zone_sequence_status", R_TL_10),
                // topics_pub
                publishes::<TwistStamped>("/app_joy_cmd", R_V_10),
                publishes::<Text>("/mower_recorder/command", R_V_10),
                publishes::<Text>("/mower_base/servo_command", R_V_10),
                publishes::<Text>("/mower_base/blade_command", R_V_10),
                // services
                calls::<Trigger::Service>("/record_zone_start"),
                calls::<Trigger::Service>("/record_zone_end"),
                calls::<Trigger::Service>("/risk_zone_start"),
                calls::<Trigger::Service>("/risk_zone_end"),
                calls::<Trigger::Service>("/channel_record_start"),
                calls::<Trigger::Service>("/channel_record_end"),
                calls::<Trigger::Service>("/record_cancel"),
                calls::<Trigger::Service>("/save_zone_list"),
                calls::<Trigger::Service>("/load_zone_list"),
                calls::<Trigger::Service>("/create_free_space"),
                calls::<Trigger::Service>("/create_risk_map"),
                calls::<Trigger::Service>("/create_chennal_map"),
                calls::<Trigger::Service>("/generate_coverage_path"),
                calls::<Trigger::Service>("/restore_free_space_coverage"),
                calls::<ImportImageMask::Service>("/import_image_mask"),
                calls::<EditZone::Service>("/edit_zone"),
                calls::<SiteOp::Service>("/site_op"),
                calls::<ZoneExecPath::Service>("/zone_exec_path"),
                calls::<ZoneSequence::Service>("/run_zone_sequence"),
                calls::<Trigger::Service>("/stop_zone_sequence"),
                calls::<Trigger::Service>("/cancel_nav2"),
                calls::<Trigger::Service>("/cencel_nav2"),
                calls::<Trigger::Service>("/check_nav_status"),
                calls::<GetCoverageProgress::Service>("/coverage_progress_status"),
                calls::<ResumeCoverage::Service>("/resume_coverage"),
                calls::<Trigger::Service>("/system/update"),
                calls::<Trigger::Service>("/system/restart"),
                calls::<Trigger::Service>("/system/check_update"),
                calls::<PidAutotune::Service>("/pid_autotune"),
                calls::<SetParameters::Service>("/boustrophedon_coverage/set_parameters"),
                calls::<Trigger::Service>("/mower_recorder/start"),
                calls::<Trigger::Service>("/mower_recorder/stop"),
            ],
        }],
    },
];

/// The nodes outside the typed modules that a module's subscription or call
/// can depend on (docs/BUS_TOPICS.md 5.3, "DDS→bus"), with the step that
/// removes them. Their QoS is upstream behaviour, unconfirmed unless said
/// measured (twist_mux and robot_state_publisher were, in the runtime image,
/// for E4: tools/mux_oracle.py, tools/rsp_capture.py).
pub static OUTSIDE: &[Outside] = &[
    Outside {
        // cmd_vel_out, rclcpp::QoS(KeepLast(1)): R V 1, measured
        node: "twist_mux",
        until: "E4",
        // rust_daemon with rust_mux does not start it
        endpoints: &[publishes::<TwistStamped>("/cmd_vel_guard_input", R_V_1).replaced_by("mux")],
    },
    Outside {
        // tf2_ros' StaticBroadcasterQoS and DynamicBroadcasterQoS, R TL 1 and
        // R V 100, measured; rust_daemon with rust_static_tf does not start it
        node: "robot_state_publisher",
        until: "E4",
        endpoints: &[
            publishes::<TFMessage>("/tf_static", R_TL_1).replaced_by("static_tf"),
            publishes::<TFMessage>("/tf", R_V_100).replaced_by("static_tf"),
        ],
    },
    Outside {
        // rcl's /rosout writer is on every node; mower_nav reads nav2's.
        // velocity_smoother writes /nav_cmd_vel for twist_mux (R V 1,
        // measured): with the mux on the bus it comes in from DDS.
        node: "nav2",
        until: "E9",
        endpoints: &[
            publishes::<Log>("/rosout", R_TL_1000),
            publishes::<TwistStamped>("/nav_cmd_vel", R_V_1),
            serves::<GetState::Service>("/bt_navigator/get_state"),
            serves_action::<NavigateToPose::Action>("/navigate_to_pose"),
            serves_action::<FollowPath::Action>("/follow_path"),
        ],
    },
    Outside {
        node: "teleop_twist_joy",
        until: "enable_physical_joystick (off)",
        endpoints: &[publishes::<TwistStamped>("/physical_joy_cmd", R_V_10)],
    },
    Outside {
        node: "mower_teleop",
        until: "enable_keyboard_teleop (off)",
        endpoints: &[publishes::<TwistStamped>("/keyboard_cmd_vel", R_V_10)],
    },
    Outside {
        // The Python recorder (recorder_manager_node.py): its status for the
        // app, latched, and the services the app calls through ws_bridge
        // (docs/BUS_TOPICS.md 2.8, 3.3).
        node: "recorder_manager",
        until: "E5",
        endpoints: &[
            publishes::<Text>("/mower_recorder/status", R_TL_1),
            // "the rear light breathes red"; nothing reads it (docs/BUS_TOPICS.md 5.8 #1)
            publishes::<Text>("/mower_base/rear_light", R_TL_1),
            serves::<Trigger::Service>("/mower_recorder/start"),
            serves::<Trigger::Service>("/mower_recorder/stop"),
            serves::<Trigger::Service>("/mower_recorder/snapshot"),
        ],
    },
    Outside {
        // bag_store_node.py: the bag list, latched, and the answers to the
        // app's commands (docs/BUS_TOPICS.md 2.8).
        node: "bag_store",
        until: "E5",
        endpoints: &[
            publishes::<Text>("/mower_recorder/bags", R_TL_1),
            publishes::<Text>("/mower_recorder/command_result", R_V_10),
            subscribes::<Text>("/mower_recorder/command", R_V_10),
        ],
    },
];

/// The module `id`.
pub fn module(id: &str) -> Option<&'static Module> {
    MODULES.iter().find(|m| m.id == id)
}

/// The instance whose production node name is `node`, with its module.
pub fn instance(node: &str) -> Option<(&'static Module, &'static Instance)> {
    MODULES.iter().find_map(|m| m.instances.iter().find(|i| i.node == node).map(|i| (m, i)))
}
