//! The module table as `mower_rsd --list-modules` prints it, before any
//! ROS: which launch switch hosts each module in the daemon (E9-07, from
//! test_controller_configuration.py). The node names are the unit test's
//! (`module_table_carries_the_production_node_names` in src/main.rs).

use std::process::Command;

/// `(id, switch, nodes)` for every module.
fn modules() -> Vec<(String, String, Vec<String>)> {
    let out = Command::new(env!("CARGO_BIN_EXE_mower_rsd")).arg("--list-modules").output().expect("mower_rsd runs");
    assert!(out.status.success(), "{out:?}");
    String::from_utf8(out.stdout)
        .expect("utf-8")
        .lines()
        .map(|line| {
            let mut words = line.split_whitespace();
            let id = words.next().expect("an id").to_string();
            let switch = words.next().expect("a switch").to_string();
            let nodes = words.collect::<Vec<_>>().join(" ").split(", ").map(str::to_string).collect();
            (id, switch, nodes)
        })
        .collect()
}

fn module(id: &str) -> (String, Vec<String>) {
    let (_, switch, nodes) = modules().into_iter().find(|(m, _, _)| m == id).unwrap_or_else(|| panic!("no module {id}"));
    (switch, nodes)
}

/// rust_localize swaps both EKFs and navsat_transform for one module with
/// their three node names.
#[test]
fn localize_is_the_rust_localize_switch_with_the_three_node_names() {
    assert_eq!(module("localize"), ("rust_localize".to_string(), vec!["ekf_filter_node_odom".into(), "ekf_filter_node_map".into(), "navsat_transform".into()]));
}

/// rust_base replaces the ros2_control chain with one mower_base node.
#[test]
fn base_is_the_rust_base_switch_with_one_mower_base_node() {
    assert_eq!(module("base"), ("rust_base".to_string(), vec!["mower_base".to_string()]));
}

/// The coverage planner has no rclpy version left and so no switch: it is
/// always in the set, as boustrophedon_coverage, and nothing is called
/// rust_coverage.
#[test]
fn coverage_has_no_switch_and_always_runs() {
    assert_eq!(module("coverage"), ("(always)".to_string(), vec!["boustrophedon_coverage".to_string()]));
    for (id, switch, _) in modules() {
        assert_ne!(switch, "rust_coverage", "{id}");
    }
}

/// rust_mux (E4) swaps the C++ twist_mux for the daemon's mux module, under
/// the C++ node's name (twist_mux_topics.yaml's key).
#[test]
fn mux_is_the_rust_mux_switch_with_one_twist_mux_node() {
    assert_eq!(module("mux"), ("rust_mux".to_string(), vec!["twist_mux".to_string()]));
}

/// rust_static_tf (E4) swaps robot_state_publisher for the daemon's
/// static_tf module.
#[test]
fn static_tf_is_the_rust_static_tf_switch_with_one_static_tf_publisher_node() {
    assert_eq!(module("static_tf"), ("rust_static_tf".to_string(), vec!["static_tf_publisher".to_string()]));
}

/// rust_recorder (E5) swaps the Python recorder for the daemon's bag module,
/// under the three nodes' names, in record.launch.py's order; not the zone
/// recorder's rust_record.
#[test]
fn bag_is_the_rust_recorder_switch_with_the_recorder_s_three_nodes() {
    assert_eq!(
        module("bag"),
        ("rust_recorder".to_string(), vec!["graph_snapshot".into(), "recorder_manager".into(), "bag_store".into(), "camera_front".into()])
    );
    assert_eq!(module("record").0, "rust_record");
}
