//! What twist_mux 4.5.0 publishes on `/diagnostics`, as plain values (the
//! module fills `diagnostic_msgs/DiagnosticArray` from them).
//!
//! - **At the start**, one status: diagnostic_updater publishes it as the task
//!   is added, before twist_mux sets its hardware id. Level 0, message
//!   `Node starting up`, hardware id empty, no values.
//! - **Then two a second**: twist_mux's own 1 s timer and the updater's
//!   (`diagnostic_updater.period`, 1 s) both publish the task. Each is
//!   computed when it is published, with the lock priority of twist_mux's
//!   last tick, which the measurements show is the one of the same second.
//!   Level 0 (OK), message `ok` (the loop time and the data age it would
//!   check are never set in ROS 2), hardware id `none`, and the values below.
//! - **The name** is `<node>: Twist mux status`, the node's fully qualified
//!   name with `diagnostic_updater.use_fqn`.
//! - **Nothing at the end**: the measured twist_mux publishes no "shutting
//!   down" status.

use mower_bus::Mono;

use crate::core::Mux;

/// diagnostic_msgs/DiagnosticStatus OK.
pub const OK: u8 = 0;

/// One DiagnosticStatus.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Status {
    pub name: String,
    pub level: u8,
    pub message: String,
    pub hardware_id: String,
    pub values: Vec<(String, String)>,
}

fn name(node: &str) -> String {
    format!("{node}: Twist mux status")
}

/// diagnostic_updater's status when the task is added.
pub fn starting_up(node: &str) -> Status {
    Status { name: name(node), level: OK, message: "Node starting up".into(), hardware_id: String::new(), values: Vec::new() }
}

/// The task's status at `now`, with `lock_priority` from twist_mux's last
/// tick:
///
/// ```text
/// velocity topics.app_manual   " masked (listening to /joy_cmd @ 0.500000s with priority #90)"
/// lock locks.navigation_coordinator  " locked (listening to /navigation_coordinator_lock @ 0.300000s with priority #50)"
/// current priority             "50"
/// loop time in [sec]           "0"
/// data age in [sec]            "0"
/// ```
///
/// "unmasked" only says an input is not masked, not that it is the winner.
pub fn status(node: &str, mux: &Mux, lock_priority: u8, now: Mono) -> Status {
    let mut values = Vec::new();
    for (i, h) in mux.inputs().enumerate() {
        let masked = if mux.is_masked(i, lock_priority, now) { "masked" } else { "unmasked" };
        values.push((
            format!("velocity {}", h.name),
            format!(" {masked} (listening to {} @ {:.6}s with priority #{})", h.topic, h.timeout_s(), h.priority),
        ));
    }
    for (j, h) in mux.locks().enumerate() {
        let locked = if mux.is_locked(j, now) { "locked" } else { "free" };
        values.push((
            format!("lock {}", h.name),
            format!(" {locked} (listening to {} @ {:.6}s with priority #{})", h.topic, h.timeout_s(), h.priority),
        ));
    }
    values.push(("current priority".into(), lock_priority.to_string()));
    // std::stringstream of the double 0.0, which ROS 2's twist_mux never sets
    values.push(("loop time in [sec]".into(), "0".into()));
    values.push(("data age in [sec]".into(), "0".into()));
    Status { name: name(node), level: OK, message: "ok".into(), hardware_id: "none".into(), values }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{from_params, tests::production_params};

    const MS: u64 = 1_000_000;

    fn production() -> Mux {
        let c = from_params(&production_params()).unwrap();
        Mux::new(&c.inputs, &c.locks)
    }

    /// The first status of e4-design evidence out/probe.json, word for word:
    /// nothing heard, so every input masked and the lock locked.
    #[test]
    fn the_idle_status_is_twist_mux_s_word_for_word() {
        let mux = production();
        let now = Mono::from_ns(10_000 * MS);
        let s = status("twist_mux", &mux, mux.lock_priority(now), now);
        assert_eq!(s.name, "twist_mux: Twist mux status");
        assert_eq!((s.level, s.message.as_str(), s.hardware_id.as_str()), (0, "ok", "none"));
        let expected = [
            ("velocity topics.app_manual", " masked (listening to /joy_cmd @ 0.500000s with priority #90)"),
            ("velocity topics.keyboard", " masked (listening to /keyboard_cmd_vel @ 0.500000s with priority #80)"),
            ("velocity topics.navigation", " masked (listening to /nav_cmd_vel @ 0.500000s with priority #10)"),
            ("velocity topics.navigation_safety_stop",
             " masked (listening to /navigation_safety_stop @ 0.200000s with priority #50)"),
            ("velocity topics.physical_joystick", " masked (listening to /physical_joy_cmd @ 0.500000s with priority #100)"),
            ("lock locks.navigation_coordinator",
             " locked (listening to /navigation_coordinator_lock @ 0.300000s with priority #50)"),
            ("current priority", "50"),
            ("loop time in [sec]", "0"),
            ("data age in [sec]", "0"),
        ];
        let got: Vec<(&str, &str)> = s.values.iter().map(|(k, v)| (k.as_str(), v.as_str())).collect();
        assert_eq!(got, expected);
    }

    /// P5 of the evidence (lock true, safety and joy fresh): both unmasked,
    /// though only joy is forwarded; then with the lock false and nav fresh
    /// (P2): free, priority 0, nav unmasked.
    #[test]
    fn unmasked_is_not_the_winner_and_a_free_lock_reads_free() {
        let mut mux = production();
        let t = |ms: u64| Mono::from_ns(ms * MS);
        mux.on_lock(0, true, t(1000));
        mux.on_input(3, t(1001));
        mux.on_input(0, t(1002));
        let s = status("twist_mux", &mux, mux.lock_priority(t(1010)), t(1010));
        let v = |k: &str| s.values.iter().find(|(key, _)| key == k).unwrap().1.split(' ').nth(1).unwrap().to_string();
        assert_eq!(v("velocity topics.app_manual"), "unmasked");
        assert_eq!(v("velocity topics.navigation_safety_stop"), "unmasked");
        assert_eq!(v("velocity topics.navigation"), "masked");
        mux.on_lock(0, false, t(2000));
        mux.on_input(2, t(2001));
        let s = status("twist_mux", &mux, mux.lock_priority(t(2010)), t(2010));
        assert!(s.values.iter().any(|(k, v)| k == "lock locks.navigation_coordinator" && v.starts_with(" free (")));
        assert!(s.values.iter().any(|(k, v)| k == "current priority" && v == "0"));
        assert!(s.values.iter().any(|(k, v)| k == "velocity topics.navigation" && v.starts_with(" unmasked (")));
    }

    #[test]
    fn the_start_is_diagnostic_updater_s_status_without_values() {
        let s = starting_up("twist_mux");
        assert_eq!(s, Status {
            name: "twist_mux: Twist mux status".into(),
            level: 0,
            message: "Node starting up".into(),
            hardware_id: String::new(),
            values: Vec::new(),
        });
        // a renamed node names its status (tools/mux_oracle.py's C++ copies)
        assert_eq!(starting_up("twist_mux_cpp1").name, "twist_mux_cpp1: Twist mux status");
    }

    /// A timeout of 0 reads 0.000000s in the status (the log says None).
    #[test]
    fn a_timeout_zero_handle_reads_zero_seconds() {
        let mux = Mux::new(&[crate::core::Handle::new("topics.never", "/n", 0.0, 40)], &[]);
        let s = status("m", &mux, 0, Mono::from_ns(0));
        assert_eq!(s.values[0].1, " unmasked (listening to /n @ 0.000000s with priority #40)");
    }
}
