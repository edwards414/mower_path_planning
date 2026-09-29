//! `mower_localize` as its own process: all three localization nodes. The same
//! code runs as the `localize` module of `mower_rsd`; see `lib.rs`.

fn main() -> std::process::ExitCode {
    mower_rs_common::module_main("localize", 4, mower_localize::run_all)
}
