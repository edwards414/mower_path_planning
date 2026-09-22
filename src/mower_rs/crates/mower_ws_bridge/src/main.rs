//! `mower_ws_bridge` as its own process. The same code runs as a module of
//! `mower_rsd`; see `lib.rs`.

fn main() -> std::process::ExitCode {
    mower_rs_common::module_main("mower_ws_bridge", 4, mower_ws_bridge::run)
}
