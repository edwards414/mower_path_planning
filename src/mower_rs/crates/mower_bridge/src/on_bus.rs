//! Which modules' topics, services and timers are on the bus.
//!
//! Moving a module is flipping its flag here, in the commit that merges its
//! conversion (README.md, "Moving a module"); docs/BUS_TOPICS.md section 6 has
//! the order. While a flag is false, mower_rsd runs that module exactly as
//! before E2, and with every flag false it starts no bridge at all.
//!
//! E2 steps 6 and 7, the drivetrain (mower_base, pid_autotune and the two
//! guards), merge with their flags false: those modules keep their r2r IO
//! next to the bus IO and pick one at start by their own flag
//! ([`instance_on_bus`]), so that the owner switches the drivetrain over
//! after a supervised test, with nothing else changed.

/// One flag per module of the manifest, in its order.
pub const ON_BUS: &[(&str, bool)] = &[
    ("base", false),
    ("status", true),
    // Both guards. Each also has a flag of its own (INSTANCES): they move in
    // two steps.
    ("guards", false),
    // E4: born on the bus
    ("mux", true),
    ("imu", true),
    ("gps", true),
    // E4: born on the bus
    ("static_tf", true),
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
    // E5: born on the bus
    ("bag", true),
];

/// Flags of single instances, by node name, for a module whose instances
/// move in different steps: an instance is on the bus when its module's flag
/// or its own is set. The final guard moves in E2 step 6, with mower_base and
/// pid_autotune, the manual guard in step 7 (docs/BUS_TOPICS.md section 6).
pub const INSTANCES: &[(&str, bool)] = &[("velocity_command_guard", false), ("manual_velocity_guard", false)];

/// The modules whose flag mower_rsd sets when it starts, from how it runs
/// them, rather than here: ws_bridge, whose backend is chosen at run time
/// (`--backend bus`, the launch's `ws_bridge_bus`). It is the app's only
/// door, the in-app update's included, so it moves back without a new
/// image.
pub const RUNTIME: &[&str] = &["bridge"];

/// Whether module `id`, or the instance whose node is `id` ([`INSTANCES`]),
/// is on the bus; false for an id neither table has (`agent`).
pub fn on_bus(id: &str) -> bool {
    ON_BUS.iter().chain(INSTANCES).any(|&(module, on)| module == id && on)
}

/// [`on_bus`], with the run-time flags `on_now` (of [`RUNTIME`]) set too.
pub fn on_bus_with<S: AsRef<str>>(on_now: &[S]) -> impl Fn(&str) -> bool + '_ {
    move |id| on_bus(id) || on_now.iter().any(|on| on.as_ref() == id)
}

/// Whether the instance rcl names `node` runs on the bus: its module's flag,
/// or its own. A module whose flag may still be false reads it at start to
/// pick its IO; [`crate::Running::rsd`] plans with the same two flags, so the
/// module and the bridge agree. False for a node the manifest does not have.
pub fn instance_on_bus(node: &str) -> bool {
    crate::instance(node).is_some_and(|(module, instance)| on_bus(module.id) || on_bus(instance.node))
}
