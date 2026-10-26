//! `mower_localize` as its own process: all three localization nodes. The same
//! code runs as the `localize` module of `mower_rsd`; see `lib.rs`. Their topics and
//! services are on the bus (step E2), so alone they run with a bridge that carries
//! every one of them to DDS (`mower_bridge::module_main`).

fn main() -> std::process::ExitCode {
    mower_bridge::module_main("localize", "localize", 4, mower_localize::run_all)
}
