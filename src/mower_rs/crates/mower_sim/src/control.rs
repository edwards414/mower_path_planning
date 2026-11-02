//! Fault injection through files in a control directory, read every 50 ms
//! (the semantics of fake_base.py's --mute-file / --deaf-file and
//! fake_sensors.py's --mute-file):
//!
//! | file | while it exists |
//! |---|---|
//! | `base_mute` | the STM32 UART with both leads pulled: nothing sent, everything received dropped (its own 300 ms timeout stops the wheels) |
//! | `base_deaf` | only the host's TX lead out: received dropped, the board keeps sending (0x81 COMMAND_TIMEOUT) |
//! | `sensors_mute` | the IMU and the F9P stall: they send nothing and drop what they receive, their ports stay open (a USB endpoint stall) |
//! | `imu_gone` | the IMU unplugged: its pty closed under the driver (EOF / EIO, as on a USB unplug) and `/dev/imu_usb` removed; a fresh pty and link when the file goes |
//! | `gps_gone` | the F9P unplugged the same way (`/dev/gps_rtk`); it comes back power-cycled: factory configuration, the fix walking nosat -> 3d -> float -> fixed (`--reacquire`) |
//! | `hub_gone` | a USB hub reset: both of them unplugged at once |
//! | `gps_mode` | its content, `fixed`, `float`, `3d`, `none` or `nosat`, optionally `:HACC_MM` (`float:400`), is the receiver's solution |
//!
//! Each change is one line for the log (main.rs prints them on stderr as
//! `mower_sim: control: <line>`).

use std::path::PathBuf;

use crate::gnss::GpsMode;
use crate::stm32::Faults;

#[derive(Debug, Default)]
pub struct Control {
    dir: Option<PathBuf>,
    pub base: Faults,
    pub sensors_mute: bool,
    pub imu_gone: bool,
    pub gps_gone: bool,
    pub hub_gone: bool,
    pub gps_mode: Option<GpsMode>,
    mode_text: Option<String>,
}

impl Control {
    pub fn new(dir: Option<PathBuf>) -> Control {
        Control { dir, ..Default::default() }
    }

    /// Whether the IMU is plugged in (no `imu_gone`, no `hub_gone`).
    pub fn imu_plugged(&self) -> bool {
        !(self.imu_gone || self.hub_gone)
    }

    /// Whether the F9P is plugged in (no `gps_gone`, no `hub_gone`).
    pub fn gps_plugged(&self) -> bool {
        !(self.gps_gone || self.hub_gone)
    }

    /// Read the files again; one line per change, for the log.
    pub fn poll(&mut self) -> Vec<String> {
        let Some(dir) = &self.dir else { return Vec::new() };
        let exists = |name: &str| dir.join(name).exists();
        let mut changes = Vec::new();
        let mut flag = |name: &str, slot: &mut bool| {
            let now = exists(name);
            if now != *slot {
                *slot = now;
                changes.push(format!("{name} {}", if now { "on" } else { "off" }));
            }
        };
        flag("base_mute", &mut self.base.mute);
        flag("base_deaf", &mut self.base.deaf);
        flag("sensors_mute", &mut self.sensors_mute);
        flag("imu_gone", &mut self.imu_gone);
        flag("gps_gone", &mut self.gps_gone);
        flag("hub_gone", &mut self.hub_gone);
        // a change of the file's text, so a bad one is reported once
        let text = std::fs::read_to_string(dir.join("gps_mode")).ok().map(|t| t.trim().to_string());
        if text != self.mode_text {
            self.mode_text = text.clone();
            match text {
                None => {
                    self.gps_mode = None;
                    changes.push("gps_mode (file gone: back to the command line's)".to_string());
                }
                Some(t) => match GpsMode::parse(&t) {
                    Some(m) => {
                        self.gps_mode = Some(m);
                        changes.push(format!("gps_mode {}", m.name()));
                    }
                    None => changes.push(format!("gps_mode {t:?} is not fixed, float, 3d, none or nosat (with an optional :HACC_MM): ignored")),
                },
            }
        }
        changes
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gnss::Mode;

    #[test]
    fn files_switch_faults_on_and_off() {
        let dir = std::env::temp_dir().join(format!("mower_sim_control_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let mut c = Control::new(Some(dir.clone()));
        assert!(c.poll().is_empty());
        std::fs::write(dir.join("base_mute"), "").unwrap();
        std::fs::write(dir.join("gps_mode"), "float\n").unwrap();
        assert_eq!(c.poll(), ["base_mute on", "gps_mode float"]);
        assert!(c.base.mute && !c.base.deaf && c.gps_mode == Some(Mode::Float.into()));
        assert!(c.poll().is_empty(), "no change, no line");
        std::fs::remove_file(dir.join("base_mute")).unwrap();
        std::fs::write(dir.join("sensors_mute"), "").unwrap();
        assert_eq!(c.poll(), ["base_mute off", "sensors_mute on"]);
        std::fs::remove_dir_all(&dir).unwrap();
        assert_eq!(c.poll(), ["sensors_mute off", "gps_mode (file gone: back to the command line's)"]);
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("gps_mode"), "rtk-ish").unwrap();
        assert_eq!(c.poll(), ["gps_mode \"rtk-ish\" is not fixed, float, 3d, none or nosat (with an optional :HACC_MM): ignored"]);
        assert!(c.poll().is_empty() && c.gps_mode.is_none(), "said once");
        std::fs::remove_dir_all(&dir).unwrap();
        assert!(Control::new(None).poll().is_empty());
    }

    /// `imu_gone`, `gps_gone` and `hub_gone` unplug the devices (the hub
    /// both); `gps_mode` takes nosat and an hAcc of its own.
    #[test]
    fn gone_files_unplug_and_gps_mode_takes_nosat_and_an_h_acc() {
        let dir = std::env::temp_dir().join(format!("mower_sim_control_gone_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let mut c = Control::new(Some(dir.clone()));
        assert!(c.poll().is_empty() && c.imu_plugged() && c.gps_plugged());
        std::fs::write(dir.join("hub_gone"), "").unwrap();
        assert_eq!(c.poll(), ["hub_gone on"]);
        assert!(!c.imu_plugged() && !c.gps_plugged());
        // the IMU stays out after the hub is back while imu_gone is there
        std::fs::write(dir.join("imu_gone"), "").unwrap();
        std::fs::remove_file(dir.join("hub_gone")).unwrap();
        assert_eq!(c.poll(), ["imu_gone on", "hub_gone off"]);
        assert!(!c.imu_plugged() && c.gps_plugged());
        std::fs::remove_file(dir.join("imu_gone")).unwrap();
        std::fs::write(dir.join("gps_gone"), "").unwrap();
        std::fs::write(dir.join("gps_mode"), "float:400\n").unwrap();
        assert_eq!(c.poll(), ["imu_gone off", "gps_gone on", "gps_mode float:400"]);
        assert!(c.imu_plugged() && !c.gps_plugged());
        assert_eq!(c.gps_mode, Some(GpsMode { mode: Mode::Float, h_acc_mm: Some(400) }));
        std::fs::remove_file(dir.join("gps_gone")).unwrap();
        std::fs::write(dir.join("gps_mode"), "nosat").unwrap();
        assert_eq!(c.poll(), ["gps_gone off", "gps_mode nosat"]);
        assert_eq!(c.gps_mode, Some(Mode::NoSat.into()));
        for bad in ["float:", "float:abc", "float:0", "float:-5", "float:4294968", "bogus:400", ":400"] {
            std::fs::write(dir.join("gps_mode"), bad).unwrap();
            let said = c.poll();
            assert_eq!(said.len(), 1, "{bad}: {said:?}");
            assert!(said[0].ends_with("ignored"), "{bad}: {said:?}");
            assert_eq!(c.gps_mode, Some(Mode::NoSat.into()), "{bad}: the last good one holds");
        }
        std::fs::write(dir.join("gps_mode"), " fixed:30 \n").unwrap();
        assert_eq!(c.poll(), ["gps_mode fixed:30"]);
        std::fs::remove_dir_all(&dir).unwrap();
        assert_eq!(c.poll(), ["gps_mode (file gone: back to the command line's)"]);
    }
}
