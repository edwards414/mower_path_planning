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
    // ws_bridge (step E3): its default backend, r2r. Its flag is RUNTIME's.
    ("bridge", false),
];

/// The modules whose flag mower_rsd sets when it starts, from how it runs
/// them, rather than here: ws_bridge, whose backend is chosen at run time
/// (`--backend bus`, the launch's `ws_bridge_bus`). It is the app's only
/// door, the in-app update's included, so it moves back without a new
/// image.
pub const RUNTIME: &[&str] = &["bridge"];

/// Whether module `id` is on the bus; false for an id the table does not
/// have (`agent`).
pub fn on_bus(id: &str) -> bool {
    ON_BUS.iter().any(|&(module, on)| module == id && on)
}

/// [`on_bus`], with the run-time flags `on_now` (of [`RUNTIME`]) set too.
pub fn on_bus_with<S: AsRef<str>>(on_now: &[S]) -> impl Fn(&str) -> bool + '_ {
    move |id| on_bus(id) || on_now.iter().any(|on| on.as_ref() == id)
}
