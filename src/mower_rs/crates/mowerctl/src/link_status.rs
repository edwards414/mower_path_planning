//! `mowerctl link-status [STATE_DIR] [--once]`: `deploy/host/mower-link-status.py`
//! on `mower_link`, its command line as it stands: the first argument not
//! starting with `--` is the state dir, every other one is ignored. Runs in the
//! place of the Python tool, never beside it: `/dev/lte_at` takes one sampler.

use mower_link::{now_unix, once_text, period_from_env, write, Sources, DEFAULT_STATE_DIR};

use crate::Fail;

const HELP: &str = "\
usage: mowerctl link-status [STATE_DIR] [--once]

Every MOWER_LINK_PERIOD_S (5) seconds, sample the modem on MOWER_LTE_AT
(/dev/lte_at), /proc/net/wireless and `ip -j -4 addr` into
STATE_DIR/link_status.json (default /home/cat/.mower). --once does it one time
and prints the document. Stop the host's mower-link-status.service first: the AT
port takes one sampler.
";

pub fn main(args: &[String]) -> Result<(), Fail> {
    if args.iter().any(|a| a == "--help") {
        print!("{HELP}");
        return Ok(());
    }
    // read before anything else, as the Python module does at its import
    let period = period_from_env().map_err(Fail::failed)?;
    let once = args.iter().any(|a| a == "--once");
    let state_dir = args.iter().find(|a| !a.starts_with("--")).map_or(DEFAULT_STATE_DIR, String::as_str);
    let src = Sources::from_env();
    if once {
        let doc = write(state_dir, &src, now_unix()).map_err(|e| Fail::failed(e.to_string()))?;
        println!("{}", once_text(&doc));
        return Ok(());
    }
    loop {
        // keep the loop alive; the container just sees a stale file
        if let Err(e) = write(state_dir, &src, now_unix()) {
            eprintln!("link-status: {e}");
        }
        let nap = std::time::Duration::try_from_secs_f64(period)
            .map_err(|_| Fail::failed(format!("MOWER_LINK_PERIOD_S: no such sleep: {period}")))?;
        std::thread::sleep(nap);
    }
}
