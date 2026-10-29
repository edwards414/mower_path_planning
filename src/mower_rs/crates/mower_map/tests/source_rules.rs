//! Rules of mower_map that only its source shows (E9-07, from
//! test_controller_configuration.py and test_qos_configuration.py; see
//! test_support/source_check.rs). The services, the latched grids, the lease
//! and the inflation envelope are tests/bus.rs's.

#[path = "../../../test_support/source_check.rs"]
mod source_check;

use source_check::Source;

fn lib() -> Source {
    Source::new("mower_map/src/lib.rs", include_str!("../src/lib.rs"))
}

/// lib.rs and grid.rs, for what may be in either.
fn module() -> Source {
    Source::new("mower_map/src/{lib,grid}.rs", &[include_str!("../src/lib.rs"), include_str!("../src/grid.rs")].join("\n"))
}

/// Every user-facing message of map_manage_node.py is mower_map's too: the
/// Chinese ones the app shows (tests/rclpy_parity.json, which the Python
/// test checks against the Python node), and the English fail-closed ones.
#[test]
fn the_user_facing_messages_are_the_python_node_s() {
    let parity: serde_json::Value = serde_json::from_str(include_str!("rclpy_parity.json")).expect("json");
    let messages = parity["user_messages"].as_array().expect("user_messages");
    assert!(!messages.is_empty());
    let module = module();
    for text in messages {
        module.says(text.as_str().expect("a message"));
    }
    for text in [
        "held occupied until its risk map is ready",
        "cannot fuse",
        "base and channel grid geometries must match",
        "base and risk grid orientations must match",
        "channel point is outside the navigation map",
        "mask_encoding must be base64_u8_row_major",
        "robot_pose_header.frame_id must be map",
        "inflate_radius_m cannot change while navigation is active or unknown",
        "inflate_radius_m must be finite",
    ] {
        module.says(text);
    }
}

/// The module does not start with an inflation radius below the safety
/// envelope or a non-finite one. A source check: `run` reads the radius from
/// the node it creates, which a test cannot hand parameters to (the bus
/// tests start with the default, the envelope itself).
#[test]
fn the_module_does_not_start_below_the_safety_envelope() {
    let run = lib().body("run(ctx");
    run.has("params::f64(&node, \"inflate_radius_m\", MIN_SAFE_INFLATE_RADIUS_M)");
    run.has("if !inflate_radius_m.is_finite() || inflate_radius_m < MIN_SAFE_INFLATE_RADIUS_M {");
    run.says("inflate_radius_m startup value must be finite and >= ");
}

/// No generic parameter handler: the parameter services are the module's
/// own, which validate a change (the envelope, the lease) before taking it.
#[test]
fn the_parameter_services_are_the_module_s_own() {
    lib().lacks("make_parameter_handler");
}

/// An image mission cannot bypass the production clearance: the image
/// zone's inflated mask is the safe map (the collected free space eroded
/// by the inflation radius, intersected with the image), never the raw mask.
/// A source check of what test_qos_configuration.py checked on
/// map_manage_node.py (E9-07: the rule now lives only here).
#[test]
fn an_image_mission_keeps_the_production_clearance() {
    let lib = lib();
    let import = lib.body("import_image_mask");
    import.has("self.image_zone_safe_map(&free_map, &image_map)");
    import.has("zone_map.msg.mask_map_inflated = free_space_inflated.msg.clone();");
    lib.lacks("mask_map_inflated = zone_map.msg.mask_map");
    lib.body("image_zone_safe_map").has("self.create_free_space_inflated(collected, None)?");
}

/// Rebuilding the free space and restoring it after an image mission are
/// transactional: the navigation maps are built and checked before the zone
/// list is replaced, and a restore with nothing collected keeps the image
/// mission. A source check (from test_qos_configuration.py's checks of
/// map_manage_node.py).
#[test]
fn rebuild_and_restore_replace_the_zones_only_once_the_maps_are_built() {
    let lib = lib();
    lib.body("create_free_space_sync").in_order(&["self.build_navigation_maps(", "self.zone_map_list = zone_maps;", "self.collected_free_space = Some("]);
    let restore = lib.body("restore_free_space");
    restore.says("沒有採集的自由空間可還原；保留目前圖片任務");
    restore.in_order(&["self.build_navigation_maps(", "self.zone_map_list = self.free_zone_backup.take()"]);
}
