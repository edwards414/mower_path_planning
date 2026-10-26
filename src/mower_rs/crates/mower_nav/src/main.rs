//! `mower_nav` as its own process. The same code runs as a module of
//! `mower_rsd`; see `lib.rs`. Its action servers, topics, services and timers are on the bus (step E2), so alone it
//! runs with a bridge that carries every one of them to DDS (`mower_bridge::module_main`).

fn main() -> std::process::ExitCode {
    mower_bridge::module_main("nav", "nav_action_server", 4, mower_nav::run)
}
