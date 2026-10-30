//! `mower_base` as its own process. The same code runs as the `base` module
//! of `mower_rsd`; see `lib.rs`. With the module's flag set (mower_bridge's
//! on_bus.rs) it runs on a bus of its own next to a bridge; otherwise on r2r,
//! as before E2.

fn main() -> std::process::ExitCode {
    mower_bridge::module_main_by_flag("base", "mower_base", 2, mower_base::run)
}
