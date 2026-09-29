//! `/mower_base/telemetry`: the one JSON message the base publishes for the
//! app, the battery node, the PID auto-tune and the parameter dashboard.
//!
//! Byte-for-byte the `std::snprintf` in
//! `mower_hardware::MowerSystem::telemetry_json()`. The consumers parse it
//! with `json.loads` / `serde_json`, so only the *values* have to match —
//! but the C++ format specifiers are reproduced exactly anyway (`%.2f` for
//! rpm, `%.3f` for the two times, `%.4f` for the PID gains), because the
//! differential test compares the two texts field by field and a different
//! number of decimals there would be a real difference in what the auto-tune
//! sees.
//!
//! Rust's `{:.N}` and C's `%.Nf` are both correctly rounded with
//! round-half-to-even, so they agree on every finite double.

use crate::cycle::Telemetry;
use crate::protocol as proto;

fn b(v: bool) -> &'static str {
    if v {
        "true"
    } else {
        "false"
    }
}

impl Telemetry {
    /// `MowerSystem::telemetry_json(now)`.
    ///
    /// * `now_s` is `rclcpp::Time::seconds()` of the cycle that publishes.
    /// * `blade_held` is the driver's dead-man state (`blade_active_`), which
    ///   is not derivable from the status frames.
    ///
    /// A status object that has never been decoded is reported with
    /// `"valid":false` and the zeroed struct, exactly as the C++ does with
    /// its default-constructed members.
    pub fn to_json(&self, now_s: f64, blade_held: bool) -> String {
        let fb = self.wheel.unwrap_or_default();
        let ms = self.motor.unwrap_or_default();
        let pid = self.pid.unwrap_or_default();
        let led = self.led.unwrap_or_default();
        let ps = self.power.unwrap_or_default();
        let ch = self.charger.unwrap_or_default();
        let sv = self.servo.unwrap_or_default();
        let bl = self.blade.unwrap_or_default();

        let mut s = String::with_capacity(1536);
        s.push_str(&format!(
            "{{\"t\":{:.3},\"feedback_age_s\":{:.3},\"crc_errors\":{},",
            now_s, self.feedback_age_s, self.crc_errors
        ));
        s.push_str(&format!(
            "\"wheel\":{{\"left\":{{\"target_rpm\":{:.2},\"measured_rpm\":{:.2},\"pid_output\":{},\"total_counts\":{}}},",
            fb.left_target_rpm, fb.left_measured_rpm, fb.left_pid_output, fb.left_total_counts
        ));
        s.push_str(&format!(
            "\"right\":{{\"target_rpm\":{:.2},\"measured_rpm\":{:.2},\"pid_output\":{},\"total_counts\":{}}},",
            fb.right_target_rpm, fb.right_measured_rpm, fb.right_pid_output, fb.right_total_counts
        ));
        s.push_str(&format!("\"flags\":{},\"seq\":{}}},", fb.flags, fb.seq));
        s.push_str(&format!(
            "\"motor\":{{\"valid\":{},\"cmd_left_permille\":{},\"cmd_right_permille\":{},",
            b(self.motor.is_some()),
            ms.commanded_left_permille,
            ms.commanded_right_permille
        ));
        s.push_str(&format!(
            "\"pwm_left\":{},\"pwm_right\":{},\"command_age_ms\":{},\"flags\":{}}},",
            ms.applied_left_pwm, ms.applied_right_pwm, ms.command_age_ms, ms.flags
        ));
        s.push_str(&format!(
            "\"pid\":{{\"valid\":{},\"left\":{{\"kp\":{:.4},\"ki\":{:.4},\"kd\":{:.4}}},",
            b(self.pid.is_some()),
            pid.left_kp as f64,
            pid.left_ki as f64,
            pid.left_kd as f64
        ));
        s.push_str(&format!(
            "\"right\":{{\"kp\":{:.4},\"ki\":{:.4},\"kd\":{:.4}}},\"flags\":{},\"last_rx_seq\":{},\"flash_diag\":{}}},",
            pid.right_kp as f64,
            pid.right_ki as f64,
            pid.right_kd as f64,
            pid.flags,
            pid.last_rx_seq,
            pid.flash_diag
        ));
        s.push_str(&format!(
            "\"led\":{{\"valid\":{},\"mode\":{},\"r\":{},\"g\":{},\"b\":{},\"period_ms\":{},\"flags\":{}}},",
            b(self.led.is_some()),
            led.mode,
            led.r,
            led.g,
            led.b,
            led.effect_period_ms,
            led.flags
        ));
        s.push_str(&format!(
            "\"power\":{{\"valid\":{},\"state\":{},\"flags\":{},\"shutdown_reason\":{},",
            b(self.power.is_some()),
            ps.state,
            ps.flags,
            ps.shutdown_reason
        ));
        s.push_str(&format!(
            "\"press_ms\":{},\"shutdown_elapsed_ms\":{}}},",
            ps.press_ms, ps.shutdown_elapsed_ms
        ));
        s.push_str(&format!(
            "\"charger\":{{\"valid\":{},\"online\":{},\"current_present\":{},\"input_present\":{},",
            b(self.charger.is_some()),
            b(ch.online()),
            b(ch.current_present()),
            b(ch.input_present())
        ));
        s.push_str(&format!(
            "\"voltage_v\":{:.2},\"current_a\":{:.2},\"temp_c\":{},\"reg3\":{},\"reg4\":{},",
            ch.voltage_cv as f64 / 100.0,
            ch.current_ca as f64 / 100.0,
            ch.temp_c,
            ch.reg3,
            ch.reg4
        ));
        s.push_str(&format!(
            "\"flags\":{},\"comm_errors\":{},\"age_ms\":{}}},",
            ch.flags, ch.comm_error_count, ch.age_ms
        ));
        s.push_str(&format!(
            "\"servo\":{{\"valid\":{},\"pulse_us\":{},\"hold_ms\":{},\"age_ms\":{},\"flags\":{},",
            b(self.servo.is_some()),
            sv.pulse_us,
            sv.hold_timeout_ms,
            sv.command_age_ms,
            sv.flags
        ));
        s.push_str(&format!(
            "\"enabled\":{},\"output\":{},\"limit_active\":{},\"limit_up\":{},\"limit_down\":{},\"timed_out\":{}}},",
            b(sv.enabled()),
            b(sv.output_active()),
            b(sv.limit_active()),
            b(sv.limit_up()),
            b(sv.limit_down()),
            b(sv.timed_out())
        ));
        s.push_str(&format!(
            "\"blade\":{{\"valid\":{},\"cmd_permille\":{},\"pwm\":{},\"age_ms\":{},\"flags\":{},\"held\":{}}}}}",
            b(self.blade.is_some()),
            bl.commanded_permille,
            bl.applied_pwm,
            bl.command_age_ms,
            bl.flags,
            b(blade_held)
        ));
        s
    }
}

/// The `charger` object alone, for logs. Kept next to [`Telemetry::to_json`]
/// so the two stay in step.
pub fn charger_summary(ch: &proto::ChargerStatus) -> String {
    format!(
        "{:.2} V {:.2} A online={} age={} ms",
        ch.voltage_cv as f64 / 100.0,
        ch.current_ca as f64 / 100.0,
        ch.online(),
        ch.age_ms
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::{
        ChargerStatus, LawerMotorStatus, MotorStatus, PidConfigStatus, PowerStatus, ServoStatus,
        WheelFeedback, Ws2812Status, CHARGER_FLAG_CURRENT_PRESENT, CHARGER_FLAG_ONLINE,
        SERVO_FLAG_ENABLED, SERVO_FLAG_LIMIT_UP,
    };

    #[test]
    fn an_empty_telemetry_is_all_invalid_and_parses() {
        let t = Telemetry::default();
        let text = t.to_json(1_758_600_000.5, false);
        let v: serde_json::Value = serde_json::from_str(&text).expect("valid json");
        assert_eq!(v["t"], 1_758_600_000.5);
        for key in ["motor", "pid", "led", "power", "charger", "servo", "blade"] {
            assert_eq!(v[key]["valid"], false, "{key}");
        }
        assert_eq!(v["blade"]["held"], false);
        assert_eq!(v["crc_errors"], 0);
    }

    /// Every field the C++ writes, with the same specifiers. The expected
    /// text is what `snprintf` produces for these inputs.
    #[test]
    fn a_full_telemetry_matches_the_cpp_layout() {
        let t = Telemetry {
            wheel: Some(WheelFeedback {
                left_target_rpm: 12.345,
                left_measured_rpm: 12.0,
                right_target_rpm: -12.345,
                right_measured_rpm: -11.995,
                left_pid_output: 250,
                right_pid_output: -250,
                left_total_counts: 123456,
                right_total_counts: -123456,
                flags: 3,
                seq: 42,
            }),
            motor: Some(MotorStatus {
                commanded_left_permille: 200,
                commanded_right_permille: -200,
                applied_left_pwm: 400,
                applied_right_pwm: -400,
                command_age_ms: 20,
                flags: 1,
                last_rx_seq: 7,
            }),
            pid: Some(PidConfigStatus {
                left_kp: 2.5,
                left_ki: 0.625,
                left_kd: 0.0,
                right_kp: 2.5,
                right_ki: 0.625,
                right_kd: 0.0,
                flags: 9,
                last_rx_seq: 8,
                flash_diag: 0,
            }),
            led: Some(Ws2812Status {
                mode: 6,
                r: 255,
                g: 180,
                b: 0,
                effect_period_ms: 1600,
                flags: 1,
                last_rx_seq: 5,
            }),
            power: Some(PowerStatus {
                state: 0,
                flags: 2,
                shutdown_reason: 0,
                last_rx_seq: 3,
                press_ms: 0,
                shutdown_elapsed_ms: 0,
            }),
            charger: Some(ChargerStatus {
                voltage_cv: 2537,
                current_ca: 125,
                temp_c: 28,
                reg3: 1,
                reg4: 2,
                flags: CHARGER_FLAG_ONLINE | CHARGER_FLAG_CURRENT_PRESENT,
                comm_error_count: 0,
                age_ms: 40,
                last_exception_code: 0,
            }),
            servo: Some(ServoStatus {
                pulse_us: 1500,
                hold_timeout_ms: 0,
                command_age_ms: 15,
                flags: SERVO_FLAG_ENABLED | SERVO_FLAG_LIMIT_UP,
                last_rx_seq: 2,
            }),
            blade: Some(LawerMotorStatus {
                commanded_permille: 300,
                applied_pwm: 600,
                command_age_ms: 10,
                flags: 1,
                last_rx_seq: 4,
            }),
            firmware: None,
            crc_errors: 2,
            feedback_age_s: 0.0415,
        };
        let text = t.to_json(1_000_000.125, true);
        let expected = concat!(
            "{\"t\":1000000.125,\"feedback_age_s\":0.042,\"crc_errors\":2,",
            "\"wheel\":{\"left\":{\"target_rpm\":12.35,\"measured_rpm\":12.00,\"pid_output\":250,\"total_counts\":123456},",
            "\"right\":{\"target_rpm\":-12.35,\"measured_rpm\":-11.99,\"pid_output\":-250,\"total_counts\":-123456},",
            "\"flags\":3,\"seq\":42},",
            "\"motor\":{\"valid\":true,\"cmd_left_permille\":200,\"cmd_right_permille\":-200,",
            "\"pwm_left\":400,\"pwm_right\":-400,\"command_age_ms\":20,\"flags\":1},",
            "\"pid\":{\"valid\":true,\"left\":{\"kp\":2.5000,\"ki\":0.6250,\"kd\":0.0000},",
            "\"right\":{\"kp\":2.5000,\"ki\":0.6250,\"kd\":0.0000},\"flags\":9,\"last_rx_seq\":8,\"flash_diag\":0},",
            "\"led\":{\"valid\":true,\"mode\":6,\"r\":255,\"g\":180,\"b\":0,\"period_ms\":1600,\"flags\":1},",
            "\"power\":{\"valid\":true,\"state\":0,\"flags\":2,\"shutdown_reason\":0,",
            "\"press_ms\":0,\"shutdown_elapsed_ms\":0},",
            "\"charger\":{\"valid\":true,\"online\":true,\"current_present\":true,\"input_present\":false,",
            "\"voltage_v\":25.37,\"current_a\":1.25,\"temp_c\":28,\"reg3\":1,\"reg4\":2,",
            "\"flags\":3,\"comm_errors\":0,\"age_ms\":40},",
            "\"servo\":{\"valid\":true,\"pulse_us\":1500,\"hold_ms\":0,\"age_ms\":15,\"flags\":17,",
            "\"enabled\":true,\"output\":false,\"limit_active\":false,\"limit_up\":true,\"limit_down\":false,\"timed_out\":false},",
            "\"blade\":{\"valid\":true,\"cmd_permille\":300,\"pwm\":600,\"age_ms\":10,\"flags\":1,\"held\":true}}"
        );
        assert_eq!(text, expected);
        serde_json::from_str::<serde_json::Value>(&text).expect("valid json");
    }
}
