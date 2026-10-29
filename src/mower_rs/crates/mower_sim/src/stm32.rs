//! The STM32 base board on its UART (/dev/stmcom), as tools/fake_base.py
//! plays it, with the wheels of the truth world:
//!
//! - 0x01 drives the wheels (permille of max_rpm) with the board's own
//!   command timeout (the 0x01's, 200 ms for 0); 0x02 the blade, 0x03 the
//!   lights, 0x04 the PID gains (a save stalls the board for the flash
//!   erase: nothing sent or taken for `flash_stall_s`), 0x06 asks for 0x87,
//!   0x07 the lift servo; 0x05 is taken and ignored;
//! - every 50 ms 0x85 (wheel feedback: rpm, counts) and 0x81 (what the
//!   wheel driver did with the last 0x01: COMMAND_VALID from the first one
//!   on, COMMAND_TIMEOUT before it and once it is older than its timeout);
//! - every second 0x86, 0x89 (the pack: voltage and current from the truth
//!   world's battery), 0x82, 0x88, 0x83, 0x84 and 0x87;
//! - 0x87 carries the image's bundled firmware identity, so /robot/info
//!   reports the firmware as up to date (fake_base.py sends 0xABCD1234).
//!
//! Faults (control.rs): `mute` is both UART leads pulled (nothing sent,
//! everything received dropped), `deaf` only the host's TX lead (received
//! dropped, sending goes on).
//!
//! Frames go through mower_base_core's own builder and parser.

use mower_base_core::protocol::{self as proto, FrameParser};

use crate::world::World;

/// What 0x87 reports.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FirmwareId {
    pub major: u8,
    pub minor: u8,
    pub patch: u8,
    pub git_sha32: u32,
    pub build_unix: u32,
    pub build_flags: u8,
}

impl FirmwareId {
    /// fake_base.py's board.
    pub const FAKE: FirmwareId = FirmwareId { major: 0, minor: 6, patch: 0, git_sha32: 0xABCD_1234, build_unix: 1_758_600_000, build_flags: 0 };

    /// The image's mower_robot_firmware.json (MOWER_FIRMWARE_MANIFEST):
    /// semver, git_sha32 (or the first 8 hex digits of git_sha), build_unix,
    /// dirty.
    pub fn from_manifest(text: &str) -> Result<FirmwareId, String> {
        let doc: serde_json::Value = serde_json::from_str(text).map_err(|e| format!("not JSON: {e}"))?;
        let semver: Vec<u8> = doc["semver"]
            .as_array()
            .map(|a| a.iter().filter_map(|v| v.as_u64()).map(|v| v.min(255) as u8).collect())
            .unwrap_or_default();
        if semver.len() != 3 {
            return Err("no semver [major, minor, patch]".into());
        }
        let sha32 = match doc["git_sha32"].as_u64() {
            Some(v) => v as u32,
            None => doc["git_sha"]
                .as_str()
                .and_then(|s| s.get(..8))
                .and_then(|s| u32::from_str_radix(s, 16).ok())
                .ok_or("no git_sha32 or git_sha")?,
        };
        let build_unix = doc["build_unix"].as_u64().ok_or("no build_unix")? as u32;
        let dirty = doc["dirty"].as_bool().unwrap_or(false);
        Ok(FirmwareId {
            major: semver[0],
            minor: semver[1],
            patch: semver[2],
            git_sha32: sha32,
            build_unix,
            build_flags: if dirty { proto::FW_BUILD_FLAG_DIRTY } else { 0 },
        })
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Faults {
    pub mute: bool,
    pub deaf: bool,
}

pub const STATUS_PERIOD_S: f64 = 0.050;
pub const SLOW_PERIOD_S: f64 = 1.0;
/// MOTOR_DEFAULT_COMMAND_TIMEOUT_MS, for a 0x01 that carries 0.
pub const DEFAULT_COMMAND_TIMEOUT_S: f64 = 0.200;

pub struct Stm32 {
    parser: FrameParser,
    seq: u8,
    firmware: FirmwareId,
    cmd: (i16, i16),
    last_command_at: Option<f64>,
    command_timeout_s: f64,
    blade_permille: i16,
    blade_last_at: Option<f64>,
    blade_timeout_s: f64,
    servo_pulse: u16,
    led: (u8, u8, u8, u8, u16),
    pid: [f32; 6],
    pid_flags: u8,
    last_rx_seq: [u8; 256],
    flash_stall_s: f64,
    stalled_until: f64,
    stalled: bool,
    next_status: f64,
    next_slow: f64,
    pub frames_rx: u64,
    pub frames_tx: u64,
}

fn i16_at(p: &[u8], i: usize) -> i16 {
    i16::from_le_bytes([p[i], p[i + 1]])
}
fn u16_at(p: &[u8], i: usize) -> u16 {
    u16::from_le_bytes([p[i], p[i + 1]])
}

impl Stm32 {
    pub fn new(firmware: FirmwareId, flash_stall_s: f64) -> Stm32 {
        Stm32 {
            parser: FrameParser::new(),
            seq: 0,
            firmware,
            cmd: (0, 0),
            last_command_at: None,
            command_timeout_s: 0.300,
            blade_permille: 0,
            blade_last_at: None,
            blade_timeout_s: 0.0,
            servo_pulse: 0,
            led: (0, 0, 0, 0, 0),
            pid: [2.0, 0.6, 0.0, 2.0, 0.6, 0.0],
            // CLOSED_LOOP | FLASH_VALID, as a board that booted with saved gains
            pid_flags: proto::PID_FLAG_CLOSED_LOOP | proto::PID_FLAG_FLASH_VALID,
            last_rx_seq: [0; 256],
            flash_stall_s,
            stalled_until: f64::NEG_INFINITY,
            stalled: false,
            next_status: 0.0,
            next_slow: 0.0,
            frames_rx: 0,
            frames_tx: 0,
        }
    }

    fn send(&mut self, faults: Faults, kind: u8, payload: &[u8], out: &mut Vec<u8>) {
        if faults.mute {
            return;
        }
        out.extend_from_slice(&proto::build_frame(kind, self.seq, payload));
        self.seq = self.seq.wrapping_add(1);
        self.frames_tx += 1;
    }

    /// Bytes the host wrote.
    pub fn on_bytes(&mut self, data: &[u8], now: f64, faults: Faults, world: &mut World, out: &mut Vec<u8>) {
        // the sector erase: the CPU runs nothing, UART included, and what
        // arrives meanwhile overruns the DMA ring
        if now < self.stalled_until || faults.mute || faults.deaf {
            return;
        }
        let mut frames = Vec::new();
        self.parser.feed(data, |kind, seq, payload| frames.push((kind, seq, payload.to_vec())));
        for (kind, seq, payload) in frames {
            self.on_frame(kind, seq, &payload, now, faults, world, out);
            if self.stalled_until > now {
                break; // the rest of this read is lost with the erase
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn on_frame(&mut self, kind: u8, seq: u8, p: &[u8], now: f64, faults: Faults, world: &mut World, out: &mut Vec<u8>) {
        self.frames_rx += 1;
        match kind {
            proto::WHEEL_SPEED_COMMAND if p.len() == 8 => {
                let (l, r, timeout) = (i16_at(p, 0), i16_at(p, 2), u16_at(p, 4));
                self.cmd = (l, r);
                world.command(l, r);
                self.last_command_at = Some(now);
                self.command_timeout_s = if timeout == 0 { DEFAULT_COMMAND_TIMEOUT_S } else { timeout as f64 / 1000.0 };
            }
            proto::LAWER_MOTOR_COMMAND if p.len() == 8 => {
                // one-directional: a negative command reads as 0
                self.blade_permille = i16_at(p, 0).clamp(0, 1000);
                self.blade_last_at = Some(now);
                self.blade_timeout_s = u16_at(p, 2) as f64 / 1000.0;
                world.blade = self.blade_permille as f64 / 1000.0;
            }
            proto::WS2812_COMMAND if p.len() == 8 => self.led = (p[0], p[1], p[2], p[3], u16_at(p, 4)),
            proto::PID_CONFIG_COMMAND if p.len() == 28 => {
                for (i, g) in self.pid.iter_mut().enumerate() {
                    *g = f32::from_le_bytes([p[4 * i], p[4 * i + 1], p[4 * i + 2], p[4 * i + 3]]);
                }
                let (persist, closed) = (p[24] != 0, p[25] != 0);
                // CLOSED_LOOP as asked, LAST_APPLY_OK; a save adds FLASH_VALID
                // and LAST_SAVE_OK and stalls the board for the erase
                let mut flags = (self.pid_flags & (proto::PID_FLAG_FLASH_VALID | proto::PID_FLAG_LAST_SAVE_OK))
                    | proto::PID_FLAG_LAST_APPLY_OK
                    | if closed { proto::PID_FLAG_CLOSED_LOOP } else { 0 };
                if persist {
                    flags |= proto::PID_FLAG_FLASH_VALID | proto::PID_FLAG_LAST_SAVE_OK;
                    self.stalled_until = now + self.flash_stall_s;
                }
                self.pid_flags = flags;
            }
            proto::SERVO_COMMAND if p.len() == 8 => self.servo_pulse = u16_at(p, 0),
            _ => {}
        }
        self.last_rx_seq[kind as usize] = seq;
        if kind == proto::INFO_REQUEST {
            self.send_firmware_info(faults, out);
        }
        if kind == proto::PID_CONFIG_COMMAND && self.stalled_until <= now {
            self.send_pid_status(faults, out);
        }
    }

    /// The board's own clock: timeouts and the status stream.
    pub fn tick(&mut self, now: f64, faults: Faults, world: &mut World, out: &mut Vec<u8>) {
        if now < self.stalled_until {
            self.stalled = true;
            return;
        }
        if self.stalled {
            // the save's outcome goes out with the first batch after it
            self.stalled = false;
            self.parser = FrameParser::new();
            self.send_pid_status(faults, out);
        }
        if let Some(t) = self.last_command_at {
            if now - t > self.command_timeout_s {
                world.command(0, 0);
            }
        }
        if let Some(t) = self.blade_last_at {
            if self.blade_timeout_s > 0.0 && now - t > self.blade_timeout_s && self.blade_permille != 0 {
                self.blade_permille = 0;
                world.blade = 0.0;
            }
        }
        if now >= self.next_status {
            self.send_wheel_feedback(faults, world, out);
            self.send_motor_status(now, faults, out);
            self.next_status += STATUS_PERIOD_S;
            if self.next_status < now {
                self.next_status = now + STATUS_PERIOD_S;
            }
        }
        if now >= self.next_slow {
            self.send_power_status(faults, out);
            self.send_charger_status(faults, world, out);
            self.send_blade_status(now, faults, out);
            self.send_servo_status(faults, out);
            self.send_led_status(faults, out);
            self.send_pid_status(faults, out);
            self.send_firmware_info(faults, out);
            self.next_slow = now + SLOW_PERIOD_S;
        }
    }

    pub fn send_wheel_feedback(&mut self, faults: Faults, world: &World, out: &mut Vec<u8>) {
        let mut p = Vec::with_capacity(24);
        let rpm100 = |rpm: f64| ((rpm * 100.0).round().clamp(i16::MIN as f64, i16::MAX as f64) as i16).to_le_bytes();
        p.extend_from_slice(&rpm100(world.left.target_rpm));
        p.extend_from_slice(&rpm100(world.left.rpm));
        p.extend_from_slice(&rpm100(world.right.target_rpm));
        p.extend_from_slice(&rpm100(world.right.rpm));
        p.extend_from_slice(&self.cmd.0.to_le_bytes());
        p.extend_from_slice(&self.cmd.1.to_le_bytes());
        p.extend_from_slice(&world.left.counts(world.wheels.counts_per_rev).to_le_bytes());
        p.extend_from_slice(&world.right.counts(world.wheels.counts_per_rev).to_le_bytes());
        p.extend_from_slice(&[proto::WHEEL_FLAG_OUTPUT_ENABLED | proto::WHEEL_FLAG_CLOSED_LOOP, 0, 0, 0]);
        self.send(faults, proto::WHEEL_FEEDBACK_STATUS, &p, out);
    }

    fn send_motor_status(&mut self, now: f64, faults: Faults, out: &mut Vec<u8>) {
        // motor.cpp control_update_50hz: valid from the first 0x01 until a
        // restart; timeout = !valid || age > command_timeout
        let age_ms = self.last_command_at.map(|t| ((now - t) * 1000.0) as u64);
        let valid = age_ms.is_some();
        let timeout = age_ms.map(|a| a as f64 > self.command_timeout_s * 1000.0).unwrap_or(true);
        let flags = if valid { proto::STATUS_FLAG_COMMAND_VALID } else { 0 } | if timeout { proto::STATUS_FLAG_COMMAND_TIMEOUT } else { 0 };
        let commanded = if valid { self.cmd } else { (0, 0) };
        let applied = if timeout { (0, 0) } else { self.cmd };
        let mut p = Vec::with_capacity(12);
        p.extend_from_slice(&commanded.0.to_le_bytes());
        p.extend_from_slice(&commanded.1.to_le_bytes());
        p.extend_from_slice(&applied.0.saturating_mul(2).to_le_bytes());
        p.extend_from_slice(&applied.1.saturating_mul(2).to_le_bytes());
        p.extend_from_slice(&(age_ms.unwrap_or(u64::MAX).min(65_535) as u16).to_le_bytes());
        p.push(flags);
        p.push(self.last_rx_seq[proto::WHEEL_SPEED_COMMAND as usize]);
        self.send(faults, proto::MOTOR_STATUS, &p, out);
    }

    fn send_blade_status(&mut self, now: f64, faults: Faults, out: &mut Vec<u8>) {
        let age = self.blade_last_at.map(|t| ((now - t) * 1000.0) as u64).unwrap_or(0).min(65_535) as u16;
        let mut p = Vec::with_capacity(8);
        p.extend_from_slice(&self.blade_permille.to_le_bytes());
        p.extend_from_slice(&self.blade_permille.saturating_mul(2).to_le_bytes());
        p.extend_from_slice(&age.to_le_bytes());
        p.push(proto::STATUS_FLAG_COMMAND_VALID);
        p.push(self.last_rx_seq[proto::LAWER_MOTOR_COMMAND as usize]);
        self.send(faults, proto::LAWER_MOTOR_STATUS, &p, out);
    }

    fn send_servo_status(&mut self, faults: Faults, out: &mut Vec<u8>) {
        let mut p = Vec::with_capacity(8);
        p.extend_from_slice(&self.servo_pulse.to_le_bytes());
        p.extend_from_slice(&0u16.to_le_bytes());
        p.extend_from_slice(&10u16.to_le_bytes());
        p.push(if self.servo_pulse != 0 { proto::SERVO_FLAG_ENABLED } else { 0 });
        p.push(self.last_rx_seq[proto::SERVO_COMMAND as usize]);
        self.send(faults, proto::SERVO_STATUS, &p, out);
    }

    fn send_led_status(&mut self, faults: Faults, out: &mut Vec<u8>) {
        let (mode, r, g, b, period) = self.led;
        let mut p = vec![mode, r, g, b];
        p.extend_from_slice(&period.to_le_bytes());
        p.push(0x01);
        p.push(self.last_rx_seq[proto::WS2812_COMMAND as usize]);
        self.send(faults, proto::WS2812_STATUS, &p, out);
    }

    fn send_pid_status(&mut self, faults: Faults, out: &mut Vec<u8>) {
        let mut p = Vec::with_capacity(28);
        for g in self.pid {
            p.extend_from_slice(&g.to_le_bytes());
        }
        p.push(self.pid_flags);
        p.push(self.last_rx_seq[proto::PID_CONFIG_COMMAND as usize]);
        p.extend_from_slice(&0u16.to_le_bytes());
        self.send(faults, proto::PID_CONFIG_STATUS, &p, out);
    }

    fn send_power_status(&mut self, faults: Faults, out: &mut Vec<u8>) {
        let mut p = vec![proto::POWER_STATE_RUNNING, proto::POWER_FLAG_MAIN_POWER_ENABLED, 0, self.last_rx_seq[proto::POWER_COMMAND as usize]];
        p.extend_from_slice(&[0, 0, 0, 0]);
        self.send(faults, proto::POWER_STATUS, &p, out);
    }

    fn send_charger_status(&mut self, faults: Faults, world: &World, out: &mut Vec<u8>) {
        let cv = (world.battery.voltage() * 100.0).round().clamp(0.0, 65_535.0) as u16;
        let ca = (world.battery.current_a * 100.0).round().clamp(0.0, 65_535.0) as u16;
        let mut p = Vec::with_capacity(16);
        for v in [cv, ca, 28, 1, 2] {
            p.extend_from_slice(&v.to_le_bytes());
        }
        p.push(proto::CHARGER_FLAG_ONLINE | proto::CHARGER_FLAG_CURRENT_PRESENT);
        p.push(0);
        p.extend_from_slice(&40u16.to_le_bytes());
        p.extend_from_slice(&[0, 0]);
        self.send(faults, proto::CHARGER_STATUS, &p, out);
    }

    fn send_firmware_info(&mut self, faults: Faults, out: &mut Vec<u8>) {
        let f = self.firmware;
        let mut p = vec![f.major, f.minor, f.patch, proto::PROTOCOL_VERSION];
        p.extend_from_slice(&f.git_sha32.to_le_bytes());
        p.extend_from_slice(&f.build_unix.to_le_bytes());
        p.extend_from_slice(&[f.build_flags, 0, 0, 0]);
        self.send(faults, proto::FIRMWARE_INFO, &p, out);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Wheels;
    use crate::world::{Battery, World};

    fn world() -> World {
        World::new(Wheels::from_params(None).unwrap(), 0.0, Battery::at_rest(24.3, 10.0))
    }

    /// Every frame of `bytes` through mower_base_core's own parser.
    fn frames(bytes: &[u8]) -> Vec<(u8, u8, Vec<u8>)> {
        let mut p = FrameParser::new();
        let mut out = Vec::new();
        p.feed(bytes, |k, s, payload| out.push((k, s, payload.to_vec())));
        assert_eq!(p.crc_errors(), 0);
        out
    }

    fn of(kind: u8, fs: &[(u8, u8, Vec<u8>)]) -> Vec<Vec<u8>> {
        fs.iter().filter(|f| f.0 == kind).map(|f| f.2.clone()).collect()
    }

    #[test]
    fn every_status_decodes_with_mower_base_cores_decoders() {
        let fw = FirmwareId::from_manifest(r#"{"semver": [0, 0, 0], "git_sha": "f3f4630eb97537f93fb454a190ede03e0e346066", "git_sha32": 4092879630, "build_unix": 1792870092, "dirty": false}"#).unwrap();
        let mut b = Stm32::new(fw, 1.0);
        let mut w = world();
        let mut out = Vec::new();
        b.tick(0.0, Faults::default(), &mut w, &mut out);
        let fs = frames(&out);
        let kinds: Vec<u8> = fs.iter().map(|f| f.0).collect();
        assert_eq!(kinds, [0x85, 0x81, 0x86, 0x89, 0x82, 0x88, 0x83, 0x84, 0x87]);
        let seqs: Vec<u8> = fs.iter().map(|f| f.1).collect();
        assert_eq!(seqs, (0..9).collect::<Vec<u8>>(), "one sequence over all frames");
        assert!(proto::decode_wheel_feedback(0, &of(0x85, &fs)[0]).is_some());
        let m = proto::decode_motor_status(&of(0x81, &fs)[0]).unwrap();
        assert_eq!((m.flags, m.command_age_ms), (proto::STATUS_FLAG_COMMAND_TIMEOUT, 65_535), "no 0x01 yet");
        let p = proto::decode_power_status(&of(0x86, &fs)[0]).unwrap();
        assert_eq!(p.flags, proto::POWER_FLAG_MAIN_POWER_ENABLED);
        let c = proto::decode_charger_status(&of(0x89, &fs)[0]).unwrap();
        assert!(c.online() && c.current_present());
        assert!((c.voltage_cv as f64 / 100.0 - w.battery.voltage()).abs() < 0.006, "{c:?}");
        assert_eq!(c.voltage_cv, 2428, "24.3 V at rest less 0.6 A x 40 mOhm");
        assert!(proto::decode_lawer_motor_status(&of(0x82, &fs)[0]).is_some());
        assert!(proto::decode_servo_status(&of(0x88, &fs)[0]).is_some());
        assert!(proto::decode_ws2812_status(&of(0x83, &fs)[0]).is_some());
        let pid = proto::decode_pid_config_status(&of(0x84, &fs)[0]).unwrap();
        assert_eq!((pid.left_kp, pid.right_ki, pid.flags), (2.0, 0.6, 0x03));
        let info = proto::decode_firmware_info(&of(0x87, &fs)[0]).unwrap();
        assert_eq!(info.version_string(), "0.0.0+f3f4630e");
        assert_eq!((info.git_sha32, info.build_unix, info.protocol_version), (4092879630, 1792870092, 1));
        // what /mower_base/firmware_info carries: robot_status compares its
        // git_sha with the manifest's first 8 digits
        assert!(info.to_json().contains("\"git_sha\":\"f3f4630e\""));
    }

    #[test]
    fn wheel_commands_drive_the_world_and_time_out_like_the_board() {
        let mut b = Stm32::new(FirmwareId::FAKE, 1.0);
        let mut w = world();
        let mut out = Vec::new();
        // the host's own encoder, 300 ms timeout as mower_base sends it
        b.on_bytes(&proto::build_wheel_speed_command(7, 500, -250, 300), 1.0, Faults::default(), &mut w, &mut out);
        assert_eq!((w.left.target_rpm, w.right.target_rpm), (29.0, -14.5));
        w.step(0.005);
        b.tick(1.0, Faults::default(), &mut w, &mut out);
        let counts = w.left.counts(8896.0);
        for k in 1..10 {
            w.step(0.005);
            b.tick(1.0 + 0.005 * k as f64, Faults::default(), &mut w, &mut out);
        }
        let fs = frames(&out);
        assert_eq!(of(0x85, &fs).len(), 1, "one 0x85 per 50 ms");
        let m = proto::decode_motor_status(of(0x81, &fs).last().unwrap()).unwrap();
        assert_eq!(m.flags, proto::STATUS_FLAG_COMMAND_VALID);
        assert_eq!((m.commanded_left_permille, m.commanded_right_permille, m.applied_left_pwm, m.last_rx_seq), (500, -250, 1000, 7));
        let fb = proto::decode_wheel_feedback(0, of(0x85, &fs).last().unwrap()).unwrap();
        assert_eq!((fb.left_target_rpm, fb.right_target_rpm), (29.0, -14.5));
        assert!(fb.left_measured_rpm > 0.0 && fb.right_measured_rpm < 0.0);
        assert_eq!(fb.left_total_counts, counts);
        // 0.3 s after the command the board stops the wheels itself
        out.clear();
        b.tick(1.31, Faults::default(), &mut w, &mut out);
        assert_eq!((w.left.target_rpm, w.right.target_rpm), (0.0, 0.0));
        let m = proto::decode_motor_status(of(0x81, &frames(&out)).last().unwrap()).unwrap();
        assert_eq!(m.flags, proto::STATUS_FLAG_COMMAND_VALID | proto::STATUS_FLAG_COMMAND_TIMEOUT);
        assert_eq!((m.applied_left_pwm, m.command_age_ms), (0, 310));
    }

    #[test]
    fn mute_deaf_info_blade_and_the_flash_stall() {
        let mut b = Stm32::new(FirmwareId::FAKE, 1.0);
        let mut w = world();
        let mut out = Vec::new();
        let muted = Faults { mute: true, deaf: false };
        b.tick(0.0, muted, &mut w, &mut out);
        b.on_bytes(&proto::build_wheel_speed_command(1, 900, 900, 300), 0.0, muted, &mut w, &mut out);
        assert!(out.is_empty() && w.left.target_rpm == 0.0, "a pulled cable: nothing out, nothing in");
        let deaf = Faults { mute: false, deaf: true };
        b.on_bytes(&proto::build_wheel_speed_command(1, 900, 900, 300), 0.1, deaf, &mut w, &mut out);
        b.tick(1.1, deaf, &mut w, &mut out);
        assert!(w.left.target_rpm == 0.0 && !out.is_empty(), "the TX lead out: still sending");
        out.clear();
        b.on_bytes(&proto::build_info_request(3), 2.0, Faults::default(), &mut w, &mut out);
        assert_eq!(proto::decode_firmware_info(&of(0x87, &frames(&out))[0]).unwrap().git_sha32, 0xABCD_1234);
        // the blade, dead-man held: it stops by itself after its timeout
        b.on_bytes(&proto::build_lawer_motor_command(4, 600, 500), 2.0, Faults::default(), &mut w, &mut out);
        assert_eq!(w.blade, 0.6);
        b.tick(2.4, Faults::default(), &mut w, &mut out);
        assert_eq!(w.blade, 0.6);
        b.tick(2.6, Faults::default(), &mut w, &mut out);
        assert_eq!(w.blade, 0.0);
        // a PID save: no answer, nothing sent or taken for the erase, then
        // the outcome first
        out.clear();
        let cfg = proto::PidConfig { left_kp: 3.0, persist_to_flash: true, closed_loop_enabled: true, ..Default::default() };
        let mut bytes = proto::build_pid_config_command(9, &cfg);
        bytes.extend(proto::build_wheel_speed_command(10, 300, 300, 300));
        b.on_bytes(&bytes, 3.0, Faults::default(), &mut w, &mut out);
        assert!(out.is_empty());
        assert_eq!(w.left.target_rpm, 0.0, "the 0x01 behind the save was lost with it");
        b.tick(3.5, Faults::default(), &mut w, &mut out);
        b.on_bytes(&proto::build_wheel_speed_command(11, 300, 300, 300), 3.5, Faults::default(), &mut w, &mut out);
        assert!(out.is_empty() && w.left.target_rpm == 0.0);
        b.tick(4.01, Faults::default(), &mut w, &mut out);
        let fs = frames(&out);
        assert_eq!(fs[0].0, proto::PID_CONFIG_STATUS);
        let pid = proto::decode_pid_config_status(&fs[0].2).unwrap();
        assert_eq!(pid.left_kp, 3.0);
        assert_eq!(pid.flags, 0x0F, "closed loop, flash valid, saved, applied");
        assert_eq!(pid.last_rx_seq, 9);
    }

    #[test]
    fn host_frames_split_anywhere_still_arrive() {
        let mut b = Stm32::new(FirmwareId::FAKE, 1.0);
        let mut w = world();
        let mut out = Vec::new();
        let bytes = [proto::build_ws2812_command(2, proto::LED_FLOW, 1, 2, 3, 400), proto::build_servo_command(3, 1500, 0)].concat();
        for chunk in bytes.chunks(3) {
            b.on_bytes(chunk, 0.0, Faults::default(), &mut w, &mut out);
        }
        assert_eq!((b.led, b.servo_pulse), ((proto::LED_FLOW, 1, 2, 3, 400), 1500));
        assert!(FirmwareId::from_manifest("{}").is_err());
        let dirty = FirmwareId::from_manifest(r#"{"semver": [0, 6, 1], "git_sha": "abcdef1234", "build_unix": 5, "dirty": true}"#).unwrap();
        assert_eq!((dirty.git_sha32, dirty.build_flags), (0xabcdef12, proto::FW_BUILD_FLAG_DIRTY));
    }
}
