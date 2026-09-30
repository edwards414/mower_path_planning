//! `velocity_command_guard` as its own process (one guard instance).
//! The same code runs as a module of `mower_rsd`; see `lib.rs`.

fn main() -> std::process::ExitCode {
    mower_rs_common::module_main("velocity_command_guard", 2, velocity_command_guard::run)
}
