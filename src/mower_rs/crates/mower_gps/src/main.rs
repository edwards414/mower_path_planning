//! `mower_gps` as its own process. The same code runs as a module of
//! `mower_rsd`; see `lib.rs`.

fn main() -> std::process::ExitCode {
    mower_rs_common::module_main("gps_driver_node", 2, mower_gps::run)
}
