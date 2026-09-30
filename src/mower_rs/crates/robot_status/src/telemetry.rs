//! `/robot/telemetry` document builder (port of `telemetry_node.py`).
//!
//! The JSON layout is the dashboard's contract, see the docstring of
//! `mower_mission/telemetry_node.py`. Every block carries `valid` (seen at
//! least once) and `age_s` (seconds since the last sample).

use std::fs;
use std::time::Instant;

use mower_rs_common::{json_number, round_to, unix_time_s};
use r2r::sensor_msgs::msg::BatteryState;
use serde_json::{json, Map, Value};

use crate::state::{Sample, State};

/// `sensor_msgs/BatteryState` power_supply_status values.
fn battery_status_text(status: u8) -> &'static str {
    match status {
        1 => "charging",
        2 => "discharging",
        3 => "not_charging",
        4 => "full",
        _ => "unknown",
    }
}

/// `sensor_msgs/NavSatStatus` status values.
fn fix_status_text(status: i8) -> String {
    match status {
        -1 => "NO FIX".to_string(),
        0 => "3D FIX".to_string(),
        1 => "DGPS".to_string(),
        2 => "RTK FIXED".to_string(),
        other => format!("status {other}"),
    }
}

/// (roll, pitch, yaw) in degrees, ZYX convention (REP-103).
pub fn quat_to_euler_deg(x: f64, y: f64, z: f64, w: f64) -> (f64, f64, f64) {
    let sinr = 2.0 * (w * x + y * z);
    let cosr = 1.0 - 2.0 * (x * x + y * y);
    let roll = sinr.atan2(cosr);
    let sinp = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0);
    let pitch = sinp.asin();
    let siny = 2.0 * (w * z + x * y);
    let cosy = 1.0 - 2.0 * (y * y + z * z);
    let yaw = siny.atan2(cosy);
    (roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees())
}

fn opt_number(v: Option<f64>) -> Value {
    v.map(json_number).unwrap_or(Value::Null)
}

fn valid_block<T>(sample: &Sample<T>, now: Instant) -> Map<String, Value> {
    let mut m = Map::new();
    m.insert("valid".into(), json!(sample.msg.is_some()));
    m.insert("age_s".into(), opt_number(sample.age_s(now)));
    m
}

/// The BatteryState fields the dashboard shows (`telemetry_node.battery_fields`).
pub fn battery_fields(msg: Option<&BatteryState>) -> Value {
    match msg {
        None => json!({
            "present": false, "pct": null, "voltage_v": null, "current_a": null, "status": null,
        }),
        Some(b) => json!({
            "present": b.present,
            "pct": json_number(round_to(b.percentage as f64, 3)),
            "voltage_v": json_number(round_to(b.voltage as f64, 2)),
            "current_a": json_number(round_to(b.current as f64, 2)),
            "status": battery_status_text(b.power_supply_status),
        }),
    }
}

fn gps_block(st: &State, now: Instant) -> Value {
    let mut block = valid_block(&st.fix, now);
    block.insert("rate_hz".into(), json!(st.fix.rate_hz()));
    if let Some(fix) = &st.fix.msg {
        let cov = &fix.position_covariance;
        let (h_acc, v_acc) = if cov.len() == 9 {
            (
                Some(cov[0].max(cov[4]).max(0.0).sqrt()),
                Some(cov[8].max(0.0).sqrt()),
            )
        } else {
            (None, None)
        };
        block.insert("status".into(), json!(fix.status.status));
        block.insert("status_text".into(), json!(fix_status_text(fix.status.status)));
        block.insert("service".into(), json!(fix.status.service));
        block.insert("lat".into(), json_number(fix.latitude));
        block.insert("lon".into(), json_number(fix.longitude));
        block.insert("alt".into(), json_number(fix.altitude));
        block.insert("h_acc_m".into(), opt_number(h_acc.map(|v| round_to(v, 4))));
        block.insert("v_acc_m".into(), opt_number(v_acc.map(|v| round_to(v, 4))));
        block.insert("cov_type".into(), json!(fix.position_covariance_type));
        block.insert("frame_id".into(), json!(fix.header.frame_id));
    }
    block.insert(
        "filtered".into(),
        match &st.filtered.msg {
            None => Value::Null,
            Some(f) => json!({
                "lat": json_number(f.latitude), "lon": json_number(f.longitude),
                "alt": json_number(f.altitude), "age_s": opt_number(st.filtered.age_s(now)),
            }),
        },
    );
    // u-blox NAV-PVT is only available in the gps image (ublox_msgs); the
    // runtime image does not ship it, so the block stays null here.
    block.insert("pvt".into(), Value::Null);
    Value::Object(block)
}

fn imu_block(st: &State, now: Instant) -> Value {
    let mut block = valid_block(&st.imu, now);
    block.insert("rate_hz".into(), json!(st.imu.rate_hz()));
    if let Some(imu) = &st.imu.msg {
        let q = &imu.orientation;
        let (roll, pitch, yaw) = quat_to_euler_deg(q.x, q.y, q.z, q.w);
        let g = &imu.angular_velocity;
        let a = &imu.linear_acceleration;
        block.insert("frame_id".into(), json!(imu.header.frame_id));
        block.insert("roll_deg".into(), json_number(round_to(roll, 2)));
        block.insert("pitch_deg".into(), json_number(round_to(pitch, 2)));
        block.insert("yaw_deg".into(), json_number(round_to(yaw, 2)));
        block.insert(
            "gyro_dps".into(),
            Value::Array([g.x, g.y, g.z].iter().map(|v| json_number(round_to(v.to_degrees(), 3))).collect()),
        );
        block.insert(
            "accel_mps2".into(),
            Value::Array([a.x, a.y, a.z].iter().map(|v| json_number(round_to(*v, 3))).collect()),
        );
        block.insert(
            "has_orientation".into(),
            json!(imu.orientation_covariance.first().map(|c| *c >= 0.0).unwrap_or(false)),
        );
    }
    Value::Object(block)
}

fn odom_block(st: &State, now: Instant) -> Value {
    let mut block = valid_block(&st.odom, now);
    if let Some(odom) = &st.odom.msg {
        let p = &odom.pose.pose.position;
        let q = &odom.pose.pose.orientation;
        let (_, _, yaw) = quat_to_euler_deg(q.x, q.y, q.z, q.w);
        block.insert("x".into(), json_number(round_to(p.x, 3)));
        block.insert("y".into(), json_number(round_to(p.y, 3)));
        block.insert("yaw_deg".into(), json_number(round_to(yaw, 2)));
        block.insert("vx".into(), json_number(round_to(odom.twist.twist.linear.x, 3)));
        block.insert("wz".into(), json_number(round_to(odom.twist.twist.angular.z, 3)));
        block.insert("frame_id".into(), json!(odom.header.frame_id));
    }
    Value::Object(block)
}

fn battery_block(st: &State, now: Instant) -> Value {
    let mut block = valid_block(&st.battery, now);
    if let Value::Object(fields) = battery_fields(st.battery.msg.as_ref()) {
        block.extend(fields);
    }
    block.insert(
        "aon".into(),
        match &st.aon_battery.msg {
            None => Value::Null,
            Some(aon) => battery_fields(Some(aon)),
        },
    );
    Value::Object(block)
}

/// `{valid, age_s}` plus every key of the sampled JSON object.
fn json_block(sample: &Sample<Value>, now: Instant) -> Value {
    let mut block = valid_block(sample, now);
    if let Some(Value::Object(obj)) = &sample.msg {
        for (k, v) in obj {
            block.insert(k.clone(), v.clone());
        }
    }
    Value::Object(block)
}

/// Re-read `<state_dir>/link_status.json` only when the host rewrote it.
pub fn poll_link(st: &mut State, state_dir: &str, now: Instant) {
    let path = format!("{state_dir}/link_status.json");
    let Ok(meta) = fs::metadata(&path) else { return };
    let Ok(mtime) = meta.modified() else { return };
    if st.link_mtime == Some(mtime) {
        return;
    }
    st.link_mtime = Some(mtime);
    if let Some(doc) = mower_rs_common::read_json_file(&path) {
        st.link.set(doc, now);
    }
}

/// One `/robot/telemetry` document.
pub fn build_document(st: &State, robot_id: &str, seq: u64, start: Instant, now: Instant) -> Value {
    let mut host = match &st.host {
        Value::Object(m) => m.clone(),
        _ => Map::new(),
    };
    host.insert("uptime_s".into(), json_number(round_to(now.duration_since(start).as_secs_f64(), 1)));
    json!({
        "time": json_number(round_to(unix_time_s(), 3)),
        "robot_id": robot_id,
        "seq": seq,
        "gps": gps_block(st, now),
        "imu": imu_block(st, now),
        "odom": odom_block(st, now),
        "base": json_block(&st.base, now),
        "battery": battery_block(st, now),
        "link": json_block(&st.link, now),
        "host": Value::Object(host),
        "info": st.info.clone().unwrap_or(Value::Null),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quat_to_euler_yaw_only() {
        let (r, p, y) = quat_to_euler_deg(0.0, 0.0, (45f64.to_radians() / 2.0).sin(), (45f64.to_radians() / 2.0).cos());
        assert!(r.abs() < 1e-9 && p.abs() < 1e-9);
        assert!((y - 45.0).abs() < 1e-9);
    }

    #[test]
    fn battery_fields_map_status_and_absent_pack() {
        assert_eq!(battery_fields(None)["present"], json!(false));
        assert_eq!(battery_fields(None)["pct"], Value::Null);
        let mut b = BatteryState::default();
        b.present = true;
        b.percentage = 0.63;
        b.voltage = 23.456;
        b.current = f32::NAN;
        b.power_supply_status = 2;
        let f = battery_fields(Some(&b));
        assert_eq!(f["status"], json!("discharging"));
        assert_eq!(f["voltage_v"], json!(23.46));
        assert_eq!(f["current_a"], Value::Null);
        assert_eq!(f["pct"], json!(0.63));
    }

    #[test]
    fn empty_state_document_has_every_block() {
        let st = State::default();
        let now = Instant::now();
        let doc = build_document(&st, "lubancat", 1, now, now);
        for key in ["gps", "imu", "odom", "base", "battery", "link", "host"] {
            assert!(doc[key].is_object(), "{key}");
        }
        assert_eq!(doc["gps"]["valid"], json!(false));
        assert_eq!(doc["gps"]["age_s"], Value::Null);
        assert_eq!(doc["gps"]["pvt"], Value::Null);
        assert_eq!(doc["battery"]["aon"], Value::Null);
        assert_eq!(doc["info"], Value::Null);
        assert_eq!(doc["robot_id"], json!("lubancat"));
        // no NaN can reach the wire
        assert!(!serde_json::to_string(&doc).unwrap().contains("NaN"));
    }
}
