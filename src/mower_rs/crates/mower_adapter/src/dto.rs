//! ROS message shapes -> Flutter-friendly DTOs (port of
//! `mower_mission/adapters/dto.py`, see docs/flutter_frontend_function_spec.md
//! section 7). Pure functions over the generated r2r message structs so the
//! JSON layout is unit tested without a ROS graph.

use base64::Engine;
use r2r::mower_interface::msg::ZoneMap;
use r2r::nav_msgs::msg::OccupancyGrid;
use r2r::rcl_interfaces::msg::ParameterValue;
use r2r::std_msgs::msg::ColorRGBA;
use r2r::visualization_msgs::msg::MarkerArray;
use serde_json::{json, Map, Value};

/// visualization_msgs/Marker.type -> snake_case label.
fn marker_type_label(type_int: i32) -> String {
    match type_int {
        0 => "arrow",
        1 => "cube",
        2 => "sphere",
        3 => "cylinder",
        4 => "line_strip",
        5 => "line_list",
        6 => "cube_list",
        7 => "sphere_list",
        8 => "points",
        9 => "text_view_facing",
        10 => "mesh_resource",
        11 => "triangle_list",
        other => return format!("type_{other}"),
    }
    .to_string()
}

/// std_msgs/ColorRGBA (rgba 0..1 floats) -> #rrggbb.
pub fn color_rgba_to_hex(color: &ColorRGBA) -> String {
    // Python's round() is round-half-to-even.
    let byte = |v: f32| ((v as f64) * 255.0).round_ties_even().clamp(0.0, 255.0) as u8;
    format!("#{:02x}{:02x}{:02x}", byte(color.r), byte(color.g), byte(color.b))
}

/// nav_msgs/OccupancyGrid -> MapLayer DTO (base64-encoded int8 data).
pub fn occupancy_grid_to_map_layer(name: &str, msg: &OccupancyGrid) -> Value {
    let info = &msg.info;
    // int8 -> uint8 wire representation, as bytes((b & 0xFF) ...) in Python
    let raw: Vec<u8> = msg.data.iter().map(|b| *b as u8).collect();
    json!({
        "name": name,
        "type": "occupancy_grid",
        "resolution": info.resolution as f64,
        "width": info.width,
        "height": info.height,
        "origin": {"x": info.origin.position.x, "y": info.origin.position.y},
        "encoding": "base64",
        "data": base64::engine::general_purpose::STANDARD.encode(raw),
    })
}

/// visualization_msgs/MarkerArray -> MarkerLayer DTO.
pub fn marker_array_to_marker_layer(name: &str, msg: &MarkerArray) -> Value {
    let markers: Vec<Value> = msg
        .markers
        .iter()
        .map(|m| {
            json!({
                "id": m.id,
                "type": marker_type_label(m.type_),
                "color": color_rgba_to_hex(&m.color),
                "points": m.points.iter().map(|p| json!({"x": p.x, "y": p.y})).collect::<Vec<_>>(),
            })
        })
        .collect();
    json!({"name": name, "markers": markers})
}

/// An empty layer, published at start-up so a late app never keeps stale data.
pub fn empty_marker_layer(name: &str) -> Value {
    json!({"name": name, "markers": []})
}

/// mower_interface/ZoneMap[] -> ZoneSummary[].
pub fn zone_map_list_to_summaries(zone_map_list: &[ZoneMap]) -> Value {
    Value::Array(
        zone_map_list
            .iter()
            .map(|zm| {
                let point_count = zm.path.poses.len();
                json!({
                    "zoneId": zm.zone_id,
                    "pointCount": point_count,
                    "hasMap": !zm.mask_map_inflated.data.is_empty(),
                    "hasCoveragePath": point_count > 0,
                })
            })
            .collect(),
    )
}

/// rcl_interfaces/ParameterValue -> JSON (the discriminated `type` field).
pub fn parameter_value_to_json(value: &ParameterValue) -> Value {
    match value.type_ {
        1 => json!(value.bool_value),
        2 => json!(value.integer_value),
        3 => json!(value.double_value),
        4 => json!(value.string_value),
        5 => json!(value.byte_array_value),
        6 => json!(value.bool_array_value),
        7 => json!(value.integer_array_value),
        8 => json!(value.double_array_value),
        9 => json!(value.string_array_value),
        _ => Value::Null,
    }
}

/// Compose the CoverageSettings DTO from the two parameter snapshots.
pub fn params_to_coverage_settings(coverage: &Map<String, Value>, map: &Map<String, Value>) -> Value {
    let get = |m: &Map<String, Value>, k: &str| m.get(k).cloned().unwrap_or(Value::Null);
    json!({
        "stripWidthM": get(coverage, "strip_width_m"),
        "waypointSpacingM": get(coverage, "waypoint_spacing_m"),
        "zigzagAngleDeg": get(coverage, "zigzag_angle_deg"),
        "zigzagAutoAngle": get(coverage, "zigzag_auto_angle"),
        "inflateRadiusM": get(map, "inflate_radius_m"),
        "unknownAsObstacle": get(coverage, "unknown_as_obstacle"),
        "coveragePattern": get(coverage, "coverage_pattern"),
        "boundaryRing": get(coverage, "boundary_ring"),
    })
}

/// Whether a `/toLL` answer is a navsat datum point. navsat_transform answers
/// the default (0, 0, 0) until its datum exists; anything non-finite (a UTM
/// projection without a zone) is no datum either, and must never be latched.
pub fn is_navsat_datum_point(lat: f64, lon: f64) -> bool {
    lat.is_finite() && lon.is_finite() && (lat.abs() >= 1e-6 || lon.abs() >= 1e-6)
}

/// Bearing of the map +X axis clockwise from true north, derived from two
/// toLL samples taken along map +X.
pub fn bearing_to_north(lat0: f64, lon0: f64, lat1: f64, lon1: f64) -> f64 {
    let mlat = 111320.0;
    let mlon = 111320.0 * lat0.to_radians().cos();
    let east = (lon1 - lon0) * mlon;
    let north = (lat1 - lat0) * mlat;
    east.atan2(north)
}

#[cfg(test)]
mod tests {
    use super::*;
    use r2r::geometry_msgs::msg::Point;
    use r2r::visualization_msgs::msg::Marker;

    #[test]
    fn occupancy_grid_layer_matches_python_dto() {
        let mut grid = OccupancyGrid::default();
        grid.info.resolution = 0.05;
        grid.info.width = 4;
        grid.info.height = 3;
        grid.info.origin.position.x = -1.5;
        grid.info.origin.position.y = 2.0;
        grid.data = vec![0, 100, -1, 50, 0, 0, 0, 0, 0, 0, 0, 0];
        let dto = occupancy_grid_to_map_layer("map_grid", &grid);
        assert_eq!(dto["name"], json!("map_grid"));
        assert_eq!(dto["type"], json!("occupancy_grid"));
        assert_eq!(dto["width"], json!(4));
        assert_eq!(dto["height"], json!(3));
        assert_eq!(dto["origin"], json!({"x": -1.5, "y": 2.0}));
        assert_eq!(dto["encoding"], json!("base64"));
        // float32 0.05 widened to double, exactly as Python serialises it
        assert_eq!(dto["resolution"], json!(0.05f32 as f64));
        // bytes 00 64 ff 32 00 ... -> base64
        assert_eq!(dto["data"], json!("AGT/MgAAAAAAAAAA"));
    }

    #[test]
    fn marker_layer_labels_types_and_colors() {
        let mut m = Marker::default();
        m.id = 7;
        m.type_ = 4;
        m.color = ColorRGBA { r: 0.6, g: 0.0, b: 1.0, a: 0.5 };
        m.points = vec![Point { x: 1.0, y: 2.0, z: 9.0 }, Point { x: -0.5, y: 0.25, z: 0.0 }];
        let mut odd = Marker::default();
        odd.type_ = 42;
        odd.color = ColorRGBA { r: 2.0, g: -1.0, b: 0.5, a: 1.0 };
        let arr = MarkerArray { markers: vec![m, odd] };
        let dto = marker_array_to_marker_layer("zones", &arr);
        assert_eq!(dto["name"], json!("zones"));
        let markers = dto["markers"].as_array().unwrap();
        assert_eq!(markers[0]["id"], json!(7));
        assert_eq!(markers[0]["type"], json!("line_strip"));
        assert_eq!(markers[0]["color"], json!("#9900ff"));
        assert_eq!(markers[0]["points"], json!([{"x": 1.0, "y": 2.0}, {"x": -0.5, "y": 0.25}]));
        assert_eq!(markers[1]["type"], json!("type_42"));
        assert_eq!(markers[1]["color"], json!("#ff0080"));
        assert_eq!(empty_marker_layer("risk_zones"), json!({"name": "risk_zones", "markers": []}));
    }

    #[test]
    fn zone_summaries_and_settings() {
        let mut zm = ZoneMap::default();
        zm.zone_id = 3;
        zm.mask_map_inflated.data = vec![0, 0];
        zm.path.poses.push(Default::default());
        zm.path.poses.push(Default::default());
        let empty = ZoneMap::default();
        let s = zone_map_list_to_summaries(&[zm, empty]);
        assert_eq!(s, json!([
            {"zoneId": 3, "pointCount": 2, "hasMap": true, "hasCoveragePath": true},
            {"zoneId": 0, "pointCount": 0, "hasMap": false, "hasCoveragePath": false},
        ]));
        let mut cov = Map::new();
        cov.insert("strip_width_m".into(), json!(0.3));
        cov.insert("coverage_pattern".into(), json!("zigzag"));
        let map = Map::new();
        let dto = params_to_coverage_settings(&cov, &map);
        assert_eq!(dto["stripWidthM"], json!(0.3));
        assert_eq!(dto["coveragePattern"], json!("zigzag"));
        assert_eq!(dto["inflateRadiusM"], Value::Null);
        assert_eq!(dto["boundaryRing"], Value::Null);
        assert_eq!(dto["zigzagAutoAngle"], Value::Null);
    }

    #[test]
    fn parameter_values_follow_the_type_tag() {
        let mut v = ParameterValue::default();
        v.type_ = 3;
        v.double_value = 0.25;
        assert_eq!(parameter_value_to_json(&v), json!(0.25));
        v.type_ = 1;
        v.bool_value = true;
        assert_eq!(parameter_value_to_json(&v), json!(true));
        v.type_ = 4;
        v.string_value = "spiral".into();
        assert_eq!(parameter_value_to_json(&v), json!("spiral"));
        v.type_ = 0;
        assert_eq!(parameter_value_to_json(&v), Value::Null);
    }

    #[test]
    fn bearing_east_is_ninety_degrees() {
        let b = bearing_to_north(25.0, 121.0, 25.0, 121.0001);
        assert!((b.to_degrees() - 90.0).abs() < 1e-6);
        let n = bearing_to_north(25.0, 121.0, 25.0001, 121.0);
        assert!(n.abs() < 1e-9);
    }

    #[test]
    fn only_a_finite_non_zero_to_ll_answer_is_a_navsat_datum() {
        assert!(is_navsat_datum_point(23.694, 120.5377));
        assert!(is_navsat_datum_point(0.0, 118.51), "the check is value-blind beyond (0, 0)");
        // navsat's "no datum yet" answer
        assert!(!is_navsat_datum_point(0.0, 0.0));
        assert!(!is_navsat_datum_point(5e-7, -5e-7));
        // a projection without a UTM zone
        assert!(!is_navsat_datum_point(f64::NAN, f64::NAN));
        assert!(!is_navsat_datum_point(23.694, f64::NAN));
        assert!(!is_navsat_datum_point(f64::INFINITY, 120.0));
    }
}
