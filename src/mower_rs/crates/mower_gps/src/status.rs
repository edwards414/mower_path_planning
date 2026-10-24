//! The `sky` (NAV-SAT) and `relpos` (NAV-RELPOSNED) objects of the
//! `gps/status` JSON. Plain functions of the decoded messages, no r2r, so
//! the layout the dashboard parses is pinned by the tests below.

use std::time::Duration;

use mower_rs_common::json_number;
use mower_ubx::{NavRelPosNed, NavSat};
use serde_json::{json, Value};

/// Older than this, a satellite list or relative position is reported as
/// null rather than as the current state.
pub const MAX_AGE: Duration = Duration::from_secs(5);

fn age_value(age: Duration) -> Value {
    json_number((age.as_secs_f64() * 1000.0).round() / 1000.0)
}

fn opt(x: Option<f64>) -> Value {
    x.map(json_number).unwrap_or(Value::Null)
}

/// `{"age_s", "itow_ms", "num_svs", "num_used", "sats": [[gnss_id, sv_id,
/// cno_dbhz, elev_deg|null, azim_deg|null, flags], ...]}`; one array per
/// satellite rather than an object to keep the message small on the relay.
/// `flags` is the raw NAV-SAT word (bit 3 svUsed, bits 0-2 quality, bits
/// 4-5 health, bit 6 diffCorr).
pub fn sky_json(sat: &NavSat, age: Duration) -> Value {
    let sats: Vec<Value> =
        sat.sats.iter().map(|s| json!([s.gnss_id, s.sv_id, s.cno, s.elev_deg, s.azim_deg, s.flags])).collect();
    json!({
        "age_s": age_value(age),
        "itow_ms": sat.itow_ms,
        "num_svs": sat.sats.len(),
        "num_used": sat.num_used(),
        "sats": sats,
    })
}

/// The base -> rover vector in metres (north / east / down), its accuracy,
/// and the flags the dashboard shows. `valid` is relPosValid: false while
/// no reference station corrections arrive. `heading_deg` /
/// `acc_heading_deg` only when the receiver flags the heading valid;
/// `length_m` / `acc_length_m` are null for a version 0 (M8P) message.
pub fn relpos_json(r: &NavRelPosNed, age: Duration) -> Value {
    json!({
        "age_s": age_value(age),
        "itow_ms": r.itow_ms,
        "ref_station_id": r.ref_station_id,
        "valid": r.rel_pos_valid(),
        "carrier_solution": r.carrier_solution().as_str(),
        "n_m": json_number(r.n_m()),
        "e_m": json_number(r.e_m()),
        "d_m": json_number(r.d_m()),
        "length_m": opt(r.length_m()),
        "heading_deg": opt(r.heading_deg()),
        "acc_n_m": json_number(r.acc_n_m()),
        "acc_e_m": json_number(r.acc_e_m()),
        "acc_d_m": json_number(r.acc_d_m()),
        "acc_length_m": opt(r.acc_length_m()),
        "acc_heading_deg": opt(r.acc_heading_deg()),
        "is_moving": r.is_moving(),
        "ref_pos_miss": r.ref_pos_miss(),
        "ref_obs_miss": r.ref_obs_miss(),
        "flags": r.flags,
    })
}

/// `sky_json` / `relpos_json` of the latest message, or null when there is
/// none or it is older than [`MAX_AGE`].
pub fn latest<T>(slot: &Option<(T, std::time::Instant)>, now: std::time::Instant, f: fn(&T, Duration) -> Value) -> Value {
    match slot {
        Some((msg, at)) => {
            let age = now.saturating_duration_since(*at);
            if age > MAX_AGE {
                Value::Null
            } else {
                f(msg, age)
            }
        }
        None => Value::Null,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mower_ubx::SatInfo;
    use std::time::Instant;

    fn relpos(version: u8, flags: u32) -> NavRelPosNed {
        let v1 = version == 1;
        NavRelPosNed {
            version,
            ref_station_id: 7,
            itow_ms: 1000,
            rel_pos_n_cm: 1234,
            rel_pos_e_cm: -456,
            rel_pos_d_cm: 12,
            rel_pos_hp_n: 56,
            rel_pos_hp_e: -78,
            rel_pos_hp_d: -5,
            rel_pos_length_cm: v1.then_some(1316),
            rel_pos_hp_length: v1.then_some(9),
            rel_pos_heading_1e5: v1.then_some(33_973_000),
            acc_n_0_1mm: 120,
            acc_e_0_1mm: 110,
            acc_d_0_1mm: 200,
            acc_length_0_1mm: v1.then_some(150),
            acc_heading_1e5: v1.then_some(52_000),
            flags,
        }
    }

    #[test]
    fn sky_is_one_array_per_satellite() {
        let sat = NavSat {
            itow_ms: 345_600_000,
            version: 1,
            sats: vec![
                SatInfo { gnss_id: 0, sv_id: 12, cno: 44, elev_deg: Some(56), azim_deg: Some(123), pr_res_dm: 0, flags: 0x1F },
                SatInfo { gnss_id: 6, sv_id: 3, cno: 0, elev_deg: None, azim_deg: None, pr_res_dm: 0, flags: 0x01 },
            ],
        };
        let v = sky_json(&sat, Duration::from_millis(420));
        assert_eq!(
            v,
            json!({"age_s": 0.42, "itow_ms": 345_600_000, "num_svs": 2, "num_used": 1,
                   "sats": [[0, 12, 44, 56, 123, 31], [6, 3, 0, null, null, 1]]})
        );
    }

    #[test]
    fn relpos_carries_metres_and_withholds_what_the_receiver_does_not_vouch_for() {
        // gnssFixOK | diffSoln | relPosValid | carrSoln fixed | headingValid
        let v = relpos_json(&relpos(1, 0x001 | 0x002 | 0x004 | (2 << 3) | 0x100), Duration::from_millis(100));
        assert_eq!(
            v,
            json!({"age_s": 0.1, "itow_ms": 1000, "ref_station_id": 7, "valid": true,
                   "carrier_solution": "fixed",
                   "n_m": 12.3456, "e_m": -4.5678, "d_m": 0.1195,
                   "length_m": 13.1609, "heading_deg": 339.73,
                   "acc_n_m": 0.012, "acc_e_m": 0.011, "acc_d_m": 0.02,
                   "acc_length_m": 0.015, "acc_heading_deg": 0.52,
                   "is_moving": false, "ref_pos_miss": false, "ref_obs_miss": false,
                   "flags": 279})
        );
        // no corrections: invalid, no heading
        let v = relpos_json(&relpos(1, 0x001 | 0x040 | 0x080), Duration::ZERO);
        assert_eq!(v["valid"], json!(false));
        assert_eq!(v["carrier_solution"], json!("none"));
        assert_eq!((v["heading_deg"].clone(), v["acc_heading_deg"].clone()), (Value::Null, Value::Null));
        assert_eq!((v["ref_pos_miss"].clone(), v["ref_obs_miss"].clone()), (json!(true), json!(true)));
        // version 0: no length / heading at all
        let v = relpos_json(&relpos(0, 0x004 | (1 << 3)), Duration::ZERO);
        assert_eq!(v["carrier_solution"], json!("float"));
        for key in ["length_m", "heading_deg", "acc_length_m", "acc_heading_deg"] {
            assert_eq!(v[key], Value::Null, "{key}");
        }
    }

    #[test]
    fn latest_is_null_when_missing_or_stale() {
        let at = Instant::now();
        let none: Option<(NavRelPosNed, Instant)> = None;
        assert_eq!(latest(&none, at, relpos_json), Value::Null);
        let slot = Some((relpos(1, 0), at));
        assert_eq!(latest(&slot, at + Duration::from_secs(1), relpos_json)["age_s"], json!(1.0));
        assert_eq!(latest(&slot, at + MAX_AGE, relpos_json)["age_s"], json!(5.0));
        assert_eq!(latest(&slot, at + MAX_AGE + Duration::from_millis(1), relpos_json), Value::Null);
    }
}
