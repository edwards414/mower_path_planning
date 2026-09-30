//! `mower_base` as its own process. The same code runs as the `base` module
//! of `mower_rsd`; see `lib.rs`.

fn main() -> std::process::ExitCode {
    mower_rs_common::module_main("mower_base", 2, mower_base::run)
}
