//! `mower_recorder` as its own process: all three nodes (`rust_daemon:=false`
//! with `rust_recorder:=true`, mission.launch.py). The same code runs as the
//! `bag` module of `mower_rsd`; see `lib.rs`. Its endpoints are on the bus, so
//! alone it runs with a bridge that carries every one of them to DDS
//! (`mower_bridge::module_main`).

fn main() -> std::process::ExitCode {
    mower_bridge::module_main("bag", "bag", 4, mower_recorder::run_all)
}
