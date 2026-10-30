//! The NavSatFix contract of the ROS `ublox_gps` driver that navsat_transform
//! and the navigation gate rely on, for every NAV-PVT fix type (E9-07, from
//! test_controller_configuration.py; the unit tests in src/lib.rs cover the
//! carrier solutions and the high-precision position).

use mower_ubx::{Fix, NavPvt};

/// A NAV-PVT payload at 23.69 N 120.54 E, 0.5 m horizontal accuracy.
fn pvt(fix_type: u8, flags: u8) -> NavPvt {
    let mut p = vec![0u8; NavPvt::LEN];
    p[20] = fix_type;
    p[21] = flags;
    p[24..28].copy_from_slice(&1_205_387_000i32.to_le_bytes());
    p[28..32].copy_from_slice(&236_950_000i32.to_le_bytes());
    p[40..44].copy_from_slice(&500u32.to_le_bytes());
    NavPvt::parse(&p).expect("a NAV-PVT")
}

/// 2D, 3D and GNSS + dead reckoning carry a position; no fix, dead
/// reckoning only and time only do not, even with gnssFixOK. Without a
/// position the fix is NO_FIX (-1) with a NaN position, never zeros.
#[test]
fn only_2d_3d_and_gnss_dead_reckoning_fixes_are_positions() {
    for fix_type in 0..=5u8 {
        let position = matches!(fix_type, 2..=4);
        let pvt = pvt(fix_type, NavPvt::FLAG_GNSS_FIX_OK);
        assert_eq!(pvt.has_position(), position, "fix type {fix_type}");
        let fix = Fix::from_epoch(&pvt, None);
        if position {
            assert_eq!(fix.status, 0, "fix type {fix_type}: FIX");
            assert!((fix.latitude_deg - 23.695).abs() < 1e-9 && (fix.longitude_deg - 120.5387).abs() < 1e-9);
        } else {
            assert_eq!(fix.status, -1, "fix type {fix_type}: NO_FIX");
            assert!(fix.latitude_deg.is_nan() && fix.longitude_deg.is_nan() && fix.altitude_m.is_nan(), "fix type {fix_type}");
        }
    }
    // a position fix outside the receiver's masks is no fix either
    assert_eq!(Fix::from_epoch(&pvt(3, 0), None).status, -1);
}

/// GBAS_FIX (2) is reported for an RTK fixed solution only.
#[test]
fn only_an_rtk_fixed_solution_is_gbas() {
    let ok = NavPvt::FLAG_GNSS_FIX_OK;
    assert_eq!(pvt(3, ok | NavPvt::FLAG_CARRIER_FIXED).navsat_status(), 2);
    assert_eq!(pvt(3, ok | NavPvt::FLAG_CARRIER_FLOAT).navsat_status(), 0);
    assert_eq!(pvt(3, ok | NavPvt::FLAG_DIFF_SOLN).navsat_status(), 0);
    assert_eq!(pvt(3, ok).navsat_status(), 0);
}
