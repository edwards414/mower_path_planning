//! `mower_coverage` as its own process. The same code runs as a module of
//! `mower_rsd`; see `lib.rs`. Its action client, topics, services and clients are on the bus (step E2), so alone it
//! runs with a bridge that carries every one of them to DDS (`mower_bridge::module_main`).

fn main() -> std::process::ExitCode {
    mower_bridge::module_main("coverage", "boustrophedon_coverage", 4, mower_coverage::run)
}
