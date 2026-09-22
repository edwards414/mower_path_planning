//! `mower_coverage` as its own process. The same code runs as a module of
//! `mower_rsd`; see `lib.rs`.

fn main() -> std::process::ExitCode {
    mower_rs_common::module_main("boustrophedon_coverage", 4, mower_coverage::run)
}
