use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    // Put memory.x where the linker can find it. The layout must match
    // Module/Inc/boot_shared.h and STM32F411CEUX_FLASH.ld: the app lives
    // behind the 32 KB UART bootloader and leaves sector 7 for settings.
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    fs::write(out.join("memory.x"), include_bytes!("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
}
