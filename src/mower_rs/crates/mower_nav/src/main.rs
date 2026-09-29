//! `mower_nav` as its own process. The same code runs as a module of
//! `mower_rsd`; see `lib.rs`.

fn main() -> std::process::ExitCode {
    mower_rs_common::module_main("nav_action_server", 4, mower_nav::run)
}
