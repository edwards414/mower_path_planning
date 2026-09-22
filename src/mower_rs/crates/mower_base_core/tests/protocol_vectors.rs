//! Differential test against the original C++ protocol.
//!
//! `tests/vectors/protocol_oracle.json` is emitted by a harness linked
//! against `src/mower_hardware/src/mower_protocol.cpp` itself (see the crate
//! README). Encoders must be byte-identical, decoders field-identical, and
//! the byte-stream parser must agree frame for frame including its CRC error
//! count.

use mower_base_core::protocol::*;
use serde_json::Value;

fn vectors() -> Value {
    let raw = include_str!("vectors/protocol_oracle.json");
    serde_json::from_str(raw).expect("protocol oracle vectors are valid JSON")
}

fn unhex(s: &str) -> Vec<u8> {
    assert!(s.len().is_multiple_of(2), "odd hex string: {s}");
    (0..s.len() / 2)
        .map(|i| u8::from_str_radix(&s[i * 2..i * 2 + 2], 16).expect("hex"))
        .collect()
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

fn i(v: &Value, k: &str) -> i64 {
    v[k].as_i64().unwrap_or_else(|| panic!("missing int {k} in {v}"))
}
fn u(v: &Value, k: &str) -> u64 {
    v[k].as_u64().unwrap_or_else(|| panic!("missing uint {k} in {v}"))
}
fn f(v: &Value, k: &str) -> f64 {
    v[k].as_f64().unwrap_or_else(|| panic!("missing float {k} in {v}"))
}
fn b(v: &Value, k: &str) -> bool {
    v[k].as_bool().unwrap_or_else(|| panic!("missing bool {k} in {v}"))
}

#[test]
fn crc16_matches_oracle() {
    let vs = vectors();
    let cases = vs["crc16"].as_array().unwrap();
    assert!(!cases.is_empty());
    for c in cases {
        let data = unhex(c["data"].as_str().unwrap());
        assert_eq!(
            crc16_ccitt_false(&data) as u64,
            u(c, "crc"),
            "crc16 of {}",
            c["data"]
        );
    }
}

#[test]
fn encoders_are_byte_identical() {
    let vs = vectors();
    let cases = vs["encode"].as_array().unwrap();
    assert!(cases.len() >= 30, "expected a decent number of encode vectors");
    for c in cases {
        let args = c["args"].as_array().unwrap();
        let name = c["fn"].as_str().unwrap();
        let got = match name {
            "wheel_speed_command" => build_wheel_speed_command(
                args[0].as_i64().unwrap() as u8,
                args[1].as_i64().unwrap() as i16,
                args[2].as_i64().unwrap() as i16,
                args[3].as_i64().unwrap() as u16,
            ),
            "lawer_motor_command" => build_lawer_motor_command(
                args[0].as_i64().unwrap() as u8,
                args[1].as_i64().unwrap() as i16,
                args[2].as_i64().unwrap() as u16,
            ),
            "servo_command" => build_servo_command(
                args[0].as_i64().unwrap() as u8,
                args[1].as_i64().unwrap() as u16,
                args[2].as_i64().unwrap() as u16,
            ),
            "power_command" => build_power_command(
                args[0].as_i64().unwrap() as u8,
                args[1].as_i64().unwrap() as u8,
            ),
            "info_request" => build_info_request(args[0].as_i64().unwrap() as u8),
            "ws2812_command" => build_ws2812_command(
                args[0].as_i64().unwrap() as u8,
                args[1].as_i64().unwrap() as u8,
                args[2].as_i64().unwrap() as u8,
                args[3].as_i64().unwrap() as u8,
                args[4].as_i64().unwrap() as u8,
                args[5].as_i64().unwrap() as u16,
            ),
            "pid_config_command" => {
                let g = args[1].as_array().unwrap();
                let cfg = PidConfig {
                    left_kp: g[0].as_f64().unwrap() as f32,
                    left_ki: g[1].as_f64().unwrap() as f32,
                    left_kd: g[2].as_f64().unwrap() as f32,
                    right_kp: g[3].as_f64().unwrap() as f32,
                    right_ki: g[4].as_f64().unwrap() as f32,
                    right_kd: g[5].as_f64().unwrap() as f32,
                    persist_to_flash: args[2].as_bool().unwrap(),
                    closed_loop_enabled: args[3].as_bool().unwrap(),
                };
                build_pid_config_command(args[0].as_i64().unwrap() as u8, &cfg)
            }
            "frame" => build_frame(
                args[0].as_i64().unwrap() as u8,
                args[1].as_i64().unwrap() as u8,
                &unhex(args[2].as_str().unwrap()),
            ),
            other => panic!("unknown encoder {other}"),
        };
        assert_eq!(hex(&got), c["bytes"].as_str().unwrap(), "{name}{}", c["args"]);
    }
}

#[test]
fn frame_parser_matches_oracle_byte_stream() {
    let vs = vectors();
    let cases = vs["streams"].as_array().unwrap();
    assert!(cases.len() >= 15);
    for c in cases {
        let name = c["name"].as_str().unwrap();
        let mut parser = FrameParser::new();
        let mut got: Vec<(u8, u8, String)> = Vec::new();
        for chunk in c["chunks"].as_array().unwrap() {
            let bytes = unhex(chunk.as_str().unwrap());
            parser.feed(&bytes, |t, s, p| got.push((t, s, hex(p))));
        }
        let want: Vec<(u8, u8, String)> = c["frames"]
            .as_array()
            .unwrap()
            .iter()
            .map(|fr| {
                (
                    u(fr, "type") as u8,
                    u(fr, "seq") as u8,
                    fr["payload"].as_str().unwrap().to_string(),
                )
            })
            .collect();
        assert_eq!(got, want, "frames for stream '{name}'");
        assert_eq!(
            parser.crc_errors() as u64,
            u(c, "crc_errors"),
            "crc_errors for stream '{name}'"
        );
    }
}

#[test]
fn decoders_are_field_identical() {
    let vs = vectors();
    let cases = vs["decode"].as_array().unwrap();
    assert!(cases.len() >= 25);
    let mut seen_rejections = 0;
    for c in cases {
        let p = unhex(c["payload"].as_str().unwrap());
        let ok = b(c, "ok");
        let want = &c["fields"];
        if !ok {
            seen_rejections += 1;
            assert!(want.is_null());
        }
        match c["fn"].as_str().unwrap() {
            "wheel_feedback" => {
                let got = decode_wheel_feedback(u(c, "seq") as u8, &p);
                assert_eq!(got.is_some(), ok);
                if let Some(g) = got {
                    assert_eq!(g.left_target_rpm, f(want, "left_target_rpm"));
                    assert_eq!(g.left_measured_rpm, f(want, "left_measured_rpm"));
                    assert_eq!(g.right_target_rpm, f(want, "right_target_rpm"));
                    assert_eq!(g.right_measured_rpm, f(want, "right_measured_rpm"));
                    assert_eq!(g.left_pid_output as i64, i(want, "left_pid_output"));
                    assert_eq!(g.right_pid_output as i64, i(want, "right_pid_output"));
                    assert_eq!(g.left_total_counts as i64, i(want, "left_total_counts"));
                    assert_eq!(g.right_total_counts as i64, i(want, "right_total_counts"));
                    assert_eq!(g.flags as u64, u(want, "flags"));
                    assert_eq!(g.seq as u64, u(want, "seq"));
                }
            }
            "motor_status" => {
                let got = decode_motor_status(&p);
                assert_eq!(got.is_some(), ok);
                if let Some(g) = got {
                    assert_eq!(g.commanded_left_permille as i64, i(want, "commanded_left_permille"));
                    assert_eq!(g.commanded_right_permille as i64, i(want, "commanded_right_permille"));
                    assert_eq!(g.applied_left_pwm as i64, i(want, "applied_left_pwm"));
                    assert_eq!(g.applied_right_pwm as i64, i(want, "applied_right_pwm"));
                    assert_eq!(g.command_age_ms as u64, u(want, "command_age_ms"));
                    assert_eq!(g.flags as u64, u(want, "flags"));
                    assert_eq!(g.last_rx_seq as u64, u(want, "last_rx_seq"));
                }
            }
            "power_status" => {
                let got = decode_power_status(&p);
                assert_eq!(got.is_some(), ok);
                if let Some(g) = got {
                    assert_eq!(g.state as u64, u(want, "state"));
                    assert_eq!(g.flags as u64, u(want, "flags"));
                    assert_eq!(g.shutdown_reason as u64, u(want, "shutdown_reason"));
                    assert_eq!(g.last_rx_seq as u64, u(want, "last_rx_seq"));
                    assert_eq!(g.press_ms as u64, u(want, "press_ms"));
                    assert_eq!(g.shutdown_elapsed_ms as u64, u(want, "shutdown_elapsed_ms"));
                    assert_eq!(g.shutdown_requested(), b(want, "shutdown_requested"));
                }
            }
            "ws2812_status" => {
                let got = decode_ws2812_status(&p);
                assert_eq!(got.is_some(), ok);
                if let Some(g) = got {
                    assert_eq!(g.mode as u64, u(want, "mode"));
                    assert_eq!(g.r as u64, u(want, "r"));
                    assert_eq!(g.g as u64, u(want, "g"));
                    assert_eq!(g.b as u64, u(want, "b"));
                    assert_eq!(g.effect_period_ms as u64, u(want, "effect_period_ms"));
                    assert_eq!(g.flags as u64, u(want, "flags"));
                    assert_eq!(g.last_rx_seq as u64, u(want, "last_rx_seq"));
                }
            }
            "pid_config_status" => {
                let got = decode_pid_config_status(&p);
                assert_eq!(got.is_some(), ok);
                if let Some(g) = got {
                    assert_eq!(g.left_kp as f64, f(want, "left_kp"));
                    assert_eq!(g.left_ki as f64, f(want, "left_ki"));
                    assert_eq!(g.left_kd as f64, f(want, "left_kd"));
                    assert_eq!(g.right_kp as f64, f(want, "right_kp"));
                    assert_eq!(g.right_ki as f64, f(want, "right_ki"));
                    assert_eq!(g.right_kd as f64, f(want, "right_kd"));
                    assert_eq!(g.flags as u64, u(want, "flags"));
                    assert_eq!(g.last_rx_seq as u64, u(want, "last_rx_seq"));
                    assert_eq!(g.flash_diag as u64, u(want, "flash_diag"));
                }
            }
            "firmware_info" => {
                let got = decode_firmware_info(&p);
                assert_eq!(got.is_some(), ok);
                if let Some(g) = got {
                    assert_eq!(g.major as u64, u(want, "major"));
                    assert_eq!(g.minor as u64, u(want, "minor"));
                    assert_eq!(g.patch as u64, u(want, "patch"));
                    assert_eq!(g.protocol_version as u64, u(want, "protocol_version"));
                    assert_eq!(g.git_sha32 as u64, u(want, "git_sha32"));
                    assert_eq!(g.build_unix as u64, u(want, "build_unix"));
                    assert_eq!(g.build_flags as u64, u(want, "build_flags"));
                    assert_eq!(g.dirty(), b(want, "dirty"));
                    assert_eq!(g.unversioned(), b(want, "unversioned"));
                    assert_eq!(g.version_string(), want["version_string"].as_str().unwrap());
                    assert_eq!(g.to_json(), want["json"].as_str().unwrap());
                }
            }
            "charger_status" => {
                let got = decode_charger_status(&p);
                assert_eq!(got.is_some(), ok);
                if let Some(g) = got {
                    assert_eq!(g.voltage_cv as u64, u(want, "voltage_cv"));
                    assert_eq!(g.current_ca as u64, u(want, "current_ca"));
                    assert_eq!(g.temp_c as u64, u(want, "temp_c"));
                    assert_eq!(g.reg3 as u64, u(want, "reg3"));
                    assert_eq!(g.reg4 as u64, u(want, "reg4"));
                    assert_eq!(g.flags as u64, u(want, "flags"));
                    assert_eq!(g.comm_error_count as u64, u(want, "comm_error_count"));
                    assert_eq!(g.age_ms as u64, u(want, "age_ms"));
                    assert_eq!(g.last_exception_code as u64, u(want, "last_exception_code"));
                    assert_eq!(g.online(), b(want, "online"));
                    assert_eq!(g.current_present(), b(want, "current_present"));
                    assert_eq!(g.input_present(), b(want, "input_present"));
                }
            }
            "servo_status" => {
                let got = decode_servo_status(&p);
                assert_eq!(got.is_some(), ok);
                if let Some(g) = got {
                    assert_eq!(g.pulse_us as u64, u(want, "pulse_us"));
                    assert_eq!(g.hold_timeout_ms as u64, u(want, "hold_timeout_ms"));
                    assert_eq!(g.command_age_ms as u64, u(want, "command_age_ms"));
                    assert_eq!(g.flags as u64, u(want, "flags"));
                    assert_eq!(g.last_rx_seq as u64, u(want, "last_rx_seq"));
                    assert_eq!(g.enabled(), b(want, "enabled"));
                    assert_eq!(g.limit_active(), b(want, "limit_active"));
                    assert_eq!(g.output_active(), b(want, "output_active"));
                    assert_eq!(g.timed_out(), b(want, "timed_out"));
                    assert_eq!(g.limit_up(), b(want, "limit_up"));
                    assert_eq!(g.limit_down(), b(want, "limit_down"));
                }
            }
            "lawer_motor_status" => {
                let got = decode_lawer_motor_status(&p);
                assert_eq!(got.is_some(), ok);
                if let Some(g) = got {
                    assert_eq!(g.commanded_permille as i64, i(want, "commanded_permille"));
                    assert_eq!(g.applied_pwm as i64, i(want, "applied_pwm"));
                    assert_eq!(g.command_age_ms as u64, u(want, "command_age_ms"));
                    assert_eq!(g.flags as u64, u(want, "flags"));
                    assert_eq!(g.last_rx_seq as u64, u(want, "last_rx_seq"));
                }
            }
            other => panic!("unknown decoder {other}"),
        }
    }
    assert!(seen_rejections >= 9, "every decoder needs a wrong-length vector");
}
