//! The binary on a pty against the fake board: `tools/fake_base.py
//! --bootloader-lib`, whose bootloader is the unmodified
//! `firmware/bootloader/Src/bl_protocol.c`, driven by `tools/flash_check.py`,
//! which holds every run to the scenario table and to the G1 goldens
//! (`deploy/test/golden/firmware_sync/`): the transcripts and reports
//! `firmware/tools/mower_flash.py` produced in the same scenarios. Python
//! itself is not needed for this half (no pyserial): CI's flasher step runs
//! both tools side by side.
//!
//! Needs Linux, `python3` and `cc`; skipped without them.

use std::path::Path;
use std::process::Command;

#[test]
fn matches_the_python_goldens_on_the_fake_board() {
    if !cfg!(target_os = "linux") {
        eprintln!("skipped: the pty test runs on Linux only");
        return;
    }
    let have = |c: &str| Command::new(c).arg("--version").output().is_ok_and(|o| o.status.success());
    if !have("python3") || !have("cc") {
        eprintln!("skipped: needs python3 and cc");
        return;
    }
    let repo = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../..").canonicalize().unwrap();
    let work = std::env::temp_dir().join(format!("mower_flash_pty_{}", std::process::id()));
    let out = Command::new("python3")
        .arg(repo.join("src/mower_rs/tools/flash_check.py"))
        .args(["check", "--impl", &format!("rust={}", env!("CARGO_BIN_EXE_mower_flash"))])
        .arg("--golden")
        .arg(repo.join("deploy/test/golden/firmware_sync"))
        // the flash itself, a retried and an abandoned write, power lost
        // mid-flash and the recovery, a manifest Python chokes on, a short ack
        .args(["--scenarios", "S02,S05,S12,S18b,S20c,S22"])
        .arg("--work")
        .arg(&work)
        .env("PYTHONDONTWRITEBYTECODE", "1")
        .output()
        .expect("python3 runs");
    let text = format!("{}{}", String::from_utf8_lossy(&out.stdout), String::from_utf8_lossy(&out.stderr));
    let _ = std::fs::remove_dir_all(&work);
    assert!(out.status.success(), "{text}");
    assert!(text.contains("6/6 runs as expected"), "{text}");
}
