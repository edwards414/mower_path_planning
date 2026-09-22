//! `mower_agent` as its own process. The same code runs as a module of
//! `mower_rsd`; see `lib.rs`.

fn main() -> std::process::ExitCode {
    // No ROS node of its own (it reaches the graph through the bridge's
    // loopback WebSocket), so it does not take the r2r context.
    mower_rs_common::module_main("mower_agent", 2, |_ctx, m| mower_agent::run(m))
}
