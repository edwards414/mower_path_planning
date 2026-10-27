//! Fault injection through files in a control directory, read every 50 ms
//! (the semantics of fake_base.py's --mute-file / --deaf-file and
//! fake_sensors.py's --mute-file):
//!
//! | file | while it exists |
//! |---|---|
//! | `base_mute` | the STM32 UART with both leads pulled: nothing sent, everything received dropped (its own 300 ms timeout stops the wheels) |
//! | `base_deaf` | only the host's TX lead out: received dropped, the board keeps sending (0x81 COMMAND_TIMEOUT) |
//! | `sensors_mute` | a USB hub reset as the robot sees it: the IMU and the F9P send nothing and drop what they receive |
//! | `gps_mode` | its content, `fixed`, `float`, `3d` or `none`, is the receiver's solution |

use std::path::PathBuf;

use crate::gnss::Mode;
use crate::stm32::Faults;

#[derive(Debug, Default)]
pub struct Control {
    dir: Option<PathBuf>,
    pub base: Faults,
    pub sensors_mute: bool,
    pub gps_mode: Option<Mode>,
    mode_text: Option<String>,
}

impl Control {
    pub fn new(dir: Option<PathBuf>) -> Control {
        Control { dir, ..Default::default() }
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
        // a change of the file's text, so a bad one is reported once
        let text = std::fs::read_to_string(dir.join("gps_mode")).ok().map(|t| t.trim().to_string());
        if text != self.mode_text {
            self.mode_text = text.clone();
            match text {
                None => {
                    self.gps_mode = None;
                    changes.push("gps_mode (file gone: back to the command line's)".to_string());
                }
                Some(t) => match Mode::parse(&t) {
                    Some(m) => {
                        self.gps_mode = Some(m);
                        changes.push(format!("gps_mode {}", m.name()));
                    }
                    None => changes.push(format!("gps_mode {t:?} is not fixed, float, 3d or none: ignored")),
                },
            }
        }
        changes
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn files_switch_faults_on_and_off() {
        let dir = std::env::temp_dir().join(format!("mower_sim_control_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let mut c = Control::new(Some(dir.clone()));
        assert!(c.poll().is_empty());
        std::fs::write(dir.join("base_mute"), "").unwrap();
        std::fs::write(dir.join("gps_mode"), "float\n").unwrap();
        assert_eq!(c.poll(), ["base_mute on", "gps_mode float"]);
        assert!(c.base.mute && !c.base.deaf && c.gps_mode == Some(Mode::Float));
        assert!(c.poll().is_empty(), "no change, no line");
        std::fs::remove_file(dir.join("base_mute")).unwrap();
        std::fs::write(dir.join("sensors_mute"), "").unwrap();
        assert_eq!(c.poll(), ["base_mute off", "sensors_mute on"]);
        std::fs::remove_dir_all(&dir).unwrap();
        assert_eq!(c.poll(), ["sensors_mute off", "gps_mode (file gone: back to the command line's)"]);
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("gps_mode"), "rtk-ish").unwrap();
        assert_eq!(c.poll(), ["gps_mode \"rtk-ish\" is not fixed, float, 3d or none: ignored"]);
        assert!(c.poll().is_empty() && c.gps_mode.is_none(), "said once");
        std::fs::remove_dir_all(&dir).unwrap();
        assert!(Control::new(None).poll().is_empty());
    }
}
