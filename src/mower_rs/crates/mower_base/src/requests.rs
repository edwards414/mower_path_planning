//! The five `/mower_base/*_command` JSON topics, parsed the way
//! `mower_hardware::MowerSystem` parses them.
//!
//! The C++ does not use a JSON parser: `json_int` / `json_double` search for
//! `"key"`, skip to the `:` and run `strtol` / `strtod`. That means a missing
//! key falls back to the default, a value that is not a number falls back to
//! the default, and `1.5` read as an integer is `1`. This module reproduces
//! those rules on top of `serde_json`, and every clamp is the C++ clamp.
//!
//! Text that is not JSON: the LED, PID and servo requests are ignored (with
//! the C++ warning, which they share with a missing key), but a
//! `wheel_override` is a cancel and a `blade_command` is a stop. The C++
//! scan reads any key it cannot find as 0, which for these two is the
//! cancel / the stop; this port does not guess which keys the scan would
//! still have found in broken text, and a motion request that cannot be
//! understood must never leave the previous one running.

use mower_base_core::cycle::LedRequest;
use mower_base_core::protocol::{PidConfig, SERVO_MAX_PULSE_US, SERVO_MIN_PULSE_US};
use serde_json::Value;

/// `json_int(s, key, def)`: the first `"key"` anywhere in the document,
/// truncated towards zero like `strtol`.
fn json_int(v: &Value, key: &str, def: i64) -> i64 {
    match find(v, key) {
        Some(Value::Number(n)) => n
            .as_i64()
            .or_else(|| n.as_f64().map(|f| f.trunc() as i64))
            .unwrap_or(def),
        Some(Value::Bool(b)) => {
            // strtol over "true" reads nothing -> default; a JSON bool never
            // appears on these topics, but be explicit rather than lucky.
            let _ = b;
            def
        }
        _ => def,
    }
}

/// `json_double(s, section, key, def)`. `section` is one level of nesting
/// (`"left"` -> `"kp"`); the C++ searches for the key *after* the section
/// name, which for the flat documents these topics carry is the same thing.
fn json_double(v: &Value, section: Option<&str>, key: &str, def: f64) -> f64 {
    let scope = match section {
        Some(s) => match find(v, s) {
            Some(inner) => inner.clone(),
            None => return def,
        },
        None => v.clone(),
    };
    match find(&scope, key) {
        Some(Value::Number(n)) => n.as_f64().filter(|f| f.is_finite()).unwrap_or(def),
        _ => def,
    }
}

/// First value for `key`, searching nested objects depth-first, as the C++
/// substring search effectively does.
fn find<'a>(v: &'a Value, key: &str) -> Option<&'a Value> {
    match v {
        Value::Object(map) => {
            if let Some(found) = map.get(key) {
                return Some(found);
            }
            map.values().find_map(|child| find(child, key))
        }
        Value::Array(items) => items.iter().find_map(|child| find(child, key)),
        _ => None,
    }
}

fn parse(text: &str) -> Option<Value> {
    serde_json::from_str(text).ok()
}

/// `on_led_command`. `serial` distinguishes an identical repeated request so
/// it is re-asserted; the caller bumps it per message, as the C++ static
/// counter does. `None` = ignored, with the same warning the C++ logs.
pub fn led(text: &str, serial: u8) -> Option<LedRequest> {
    let v = parse(text)?;
    let mode = json_int(&v, "mode", -1);
    if !(0..=255).contains(&mode) {
        return None;
    }
    let clamp8 = |x: i64| x.clamp(0, 255) as u8;
    Some(LedRequest {
        mode: mode as u8,
        r: clamp8(json_int(&v, "r", 0)),
        g: clamp8(json_int(&v, "g", 0)),
        b: clamp8(json_int(&v, "b", 0)),
        period_ms: json_int(&v, "period_ms", 0).clamp(0, 65535) as u16,
        serial,
    })
}

/// `on_pid_command`. Both `left.kp` and `right.kp` are required.
pub fn pid(text: &str) -> Option<PidConfig> {
    let v = parse(text)?;
    let nan = f64::NAN;
    let lkp = json_double(&v, Some("left"), "kp", nan);
    let rkp = json_double(&v, Some("right"), "kp", nan);
    if lkp.is_nan() || rkp.is_nan() {
        return None;
    }
    Some(PidConfig {
        left_kp: lkp as f32,
        left_ki: json_double(&v, Some("left"), "ki", 0.0) as f32,
        left_kd: json_double(&v, Some("left"), "kd", 0.0) as f32,
        right_kp: rkp as f32,
        right_ki: json_double(&v, Some("right"), "ki", 0.0) as f32,
        right_kd: json_double(&v, Some("right"), "kd", 0.0) as f32,
        persist_to_flash: json_int(&v, "persist", 0) != 0,
        closed_loop_enabled: json_int(&v, "closed_loop", 1) != 0,
    })
}

/// `on_wheel_override` -> `(left_permille, right_permille, ttl_ms)`. The
/// clamps to +-1000 and to `kOverrideMaxTtlMs` happen in
/// `BaseCycle::request_wheel_override`, which is where the C++ does them too
/// (the ttl one) — here only the message is decoded. `Err` holds the cancel
/// (`0, 0, 0`) to apply when the text is not JSON, for the caller to log.
pub fn wheel_override(text: &str) -> Result<(i32, i32, i64), (i32, i32, i64)> {
    let v = parse(text).ok_or((0, 0, 0))?;
    Ok((
        json_int(&v, "left_permille", 0).clamp(-1000, 1000) as i32,
        json_int(&v, "right_permille", 0).clamp(-1000, 1000) as i32,
        json_int(&v, "ttl_ms", 0),
    ))
}

/// `on_servo_command` -> `(pulse_us, hold_ms)`. A negative or missing
/// `pulse_us` is ignored; 0 releases the servo, anything else is clamped
/// into the mechanical range.
pub fn servo(text: &str) -> Option<(i64, i64)> {
    let v = parse(text)?;
    let pulse = json_int(&v, "pulse_us", -1);
    if pulse < 0 {
        return None;
    }
    let pulse = if pulse == 0 {
        0
    } else {
        pulse.clamp(SERVO_MIN_PULSE_US as i64, SERVO_MAX_PULSE_US as i64)
    };
    Some((pulse, json_int(&v, "hold_ms", 0).clamp(0, 65535)))
}

/// `on_blade_command` -> `(permille, ttl_ms)`. `Err` holds the stop
/// (`0, 0`) to apply when the text is not JSON, for the caller to log.
pub fn blade(text: &str) -> Result<(i32, i64), (i32, i64)> {
    let v = parse(text).ok_or((0, 0))?;
    Ok((
        json_int(&v, "permille", 0).clamp(0, 1000) as i32,
        json_int(&v, "ttl_ms", 0),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn led_needs_a_mode_and_clamps_the_rest() {
        let r = led(r#"{"mode":6,"r":255,"g":180,"b":0,"period_ms":1600}"#, 3).unwrap();
        assert_eq!((r.mode, r.r, r.g, r.b, r.period_ms, r.serial), (6, 255, 180, 0, 1600, 3));
        // out-of-range channels clamp, the period saturates
        let r = led(r#"{"mode":1,"r":999,"g":-5,"period_ms":99999}"#, 0).unwrap();
        assert_eq!((r.r, r.g, r.b, r.period_ms), (255, 0, 0, 65535));
        // no mode, bad mode, not JSON at all: ignored
        assert!(led(r#"{"r":255}"#, 0).is_none());
        assert!(led(r#"{"mode":300}"#, 0).is_none());
        assert!(led("not json", 0).is_none());
    }

    #[test]
    fn pid_needs_both_kp_and_defaults_the_rest() {
        let c = pid(r#"{"left":{"kp":2,"ki":0.6},"right":{"kp":2.5},"persist":1}"#).unwrap();
        assert_eq!(c.left_kp, 2.0);
        assert_eq!(c.left_ki, 0.6f64 as f32);
        assert_eq!(c.left_kd, 0.0);
        assert_eq!(c.right_kp, 2.5);
        assert!(c.persist_to_flash);
        // closed_loop defaults to 1, not 0
        assert!(c.closed_loop_enabled);
        assert!(pid(r#"{"left":{"kp":2}}"#).is_none());
        assert!(pid(r#"{"left":{"ki":2},"right":{"kp":1}}"#).is_none());
    }

    #[test]
    fn override_clamps_permille_and_keeps_the_ttl_for_the_cycle() {
        assert_eq!(
            wheel_override(r#"{"left_permille":4000,"right_permille":-4000,"ttl_ms":300}"#),
            Ok((1000, -1000, 300))
        );
        assert_eq!(wheel_override("{}"), Ok((0, 0, 0)));
    }

    /// Not JSON: a cancel / a stop, never "ignored" (which would leave the
    /// previous override or blade request running until its ttl).
    #[test]
    fn malformed_motion_requests_stop() {
        assert_eq!(
            wheel_override(r#"{"left_permille":400,"right_permille":-400,"ttl_ms":300,}"#),
            Err((0, 0, 0))
        );
        assert_eq!(wheel_override("{left_permille:400}"), Err((0, 0, 0)));
        assert_eq!(blade(r#"{"permille":0,"ttl_ms":0,}"#), Err((0, 0)));
        assert_eq!(blade("{permille:600}"), Err((0, 0)));
        assert_eq!(blade(""), Err((0, 0)));
    }

    #[test]
    fn servo_ignores_a_missing_pulse_and_clamps_the_range() {
        assert_eq!(servo(r#"{"pulse_us":1500,"hold_ms":200}"#), Some((1500, 200)));
        assert_eq!(servo(r#"{"pulse_us":0}"#), Some((0, 0)));
        assert_eq!(servo(r#"{"pulse_us":100}"#), Some((500, 0)));
        assert_eq!(servo(r#"{"pulse_us":9000}"#), Some((2500, 0)));
        assert!(servo(r#"{"hold_ms":10}"#).is_none());
    }

    #[test]
    fn blade_clamps_permille_into_the_forward_range() {
        assert_eq!(blade(r#"{"permille":300,"ttl_ms":500}"#), Ok((300, 500)));
        assert_eq!(blade(r#"{"permille":-5}"#), Ok((0, 0)));
        assert_eq!(blade(r#"{"permille":5000,"ttl_ms":99999}"#), Ok((1000, 99999)));
    }

    #[test]
    fn a_float_read_as_an_integer_truncates_like_strtol() {
        let v: Value = serde_json::from_str(r#"{"ttl_ms":300.9}"#).unwrap();
        assert_eq!(json_int(&v, "ttl_ms", 0), 300);
    }
}
