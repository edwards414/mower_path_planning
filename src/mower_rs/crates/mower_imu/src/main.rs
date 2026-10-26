//! `mower_imu` as its own process. The same code runs as a module of
//! `mower_rsd`; see `lib.rs`. Its topic is on the bus (step E2), so alone it runs with a
//! bridge that carries it to DDS (`mower_bridge::module_main`).

fn main() -> std::process::ExitCode {
    mower_bridge::module_main("imu", "imu_driver_node", 2, mower_imu::run)
}
