//! `mower_ws_bridge` as its own process. The same code runs as a module of
//! `mower_rsd`; see `lib.rs`.
//!
//! With `--backend bus` it runs as a module moved to the bus does on its own
//! (mower_bridge's `module_main`): on a bus of its own, next to a bridge on
//! which every endpoint of its manifest entry faces DDS.

fn main() -> std::process::ExitCode {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    match mower_ws_bridge::backend_of(&argv) {
        Ok(mower_ws_bridge::Backend::Bus) => mower_bridge::module_main("bridge", "mower_ws_bridge", 4, mower_ws_bridge::run),
        // r2r; a bad --backend is run's to report
        _ => mower_rs_common::module_main("mower_ws_bridge", 4, mower_ws_bridge::run),
    }
}
