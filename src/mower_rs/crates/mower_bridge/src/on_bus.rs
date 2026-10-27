//! Which modules' topics, services and timers are on the bus.
//!
//! Moving a module is flipping its flag here, in the commit that merges its
//! conversion (README.md, "Moving a module"); docs/BUS_TOPICS.md section 6 has
//! the order. While a flag is false, mower_rsd runs that module exactly as
//! before E2, and with every flag false it starts no bridge at all.

/// One flag per module of the manifest, in its order.
pub const ON_BUS: &[(&str, bool)] = &[
    ("base", false),
    ("status", true),
    ("guards", false),
    ("imu", true),
    ("gps", true),
    ("localize", true),
    ("map", true),
    ("coverage", true),
    ("nav", true),
    ("record", true),
    ("adapter", true),
    ("battery", true),
    ("pid_autotune", false),
];

/// Whether module `id` is on the bus; false for an id the table does not
/// have (`bridge`, `agent`).
pub fn on_bus(id: &str) -> bool {
    ON_BUS.iter().any(|&(module, on)| module == id && on)
}
