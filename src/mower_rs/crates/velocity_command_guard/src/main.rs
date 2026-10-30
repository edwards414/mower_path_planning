//! `velocity_command_guard` as its own process (one guard instance, the one
//! launch's `-r __node:=` names). The same code runs as a module of
//! `mower_rsd`; see `lib.rs`. With the instance's flag set (mower_bridge's
//! on_bus.rs) it runs on a bus of its own next to a bridge; otherwise on
//! r2r, as before E2.

fn main() -> std::process::ExitCode {
    mower_bridge::module_main_by_flag("guards", "velocity_command_guard", 2, velocity_command_guard::run)
}
