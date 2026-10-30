//! `mower_flash`: see the crate docs and `mower_flash --help`.

fn main() {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    std::process::exit(mower_flash::cli::main(&argv));
}
