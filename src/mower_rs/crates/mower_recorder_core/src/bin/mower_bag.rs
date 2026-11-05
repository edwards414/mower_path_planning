//! `mower_bag`: mower-bag's `upload` and `verify` without Python (E5-7,
//! design D9), for mower-data-collection.sh on the robot; the logic is
//! [`mower_recorder_core::cli`]. In the image as /usr/local/bin/mower_bag.
//!
//! `MOWER_BAG_R2_ENDPOINT_URL` (tests only, as bag_store's `r2_endpoint_url`)
//! replaces the endpoint the account id gives, e.g. a local S3.

use std::path::PathBuf;
use std::process::ExitCode;

use mower_recorder_core::cli;
use mower_recorder_core::s3::{Client, Credentials, S3};
use mower_recorder_core::store::R2Config;

fn main() -> ExitCode {
    let argv: Vec<String> = std::env::args_os().skip(1).map(|a| a.to_string_lossy().into_owned()).collect();
    let env = |k: &str| std::env::var(k).ok();
    let cwd = std::env::current_dir().unwrap_or_else(|_| PathBuf::from("/"));
    let endpoint = std::env::var("MOWER_BAG_R2_ENDPOINT_URL").unwrap_or_default();
    let connect = |c: &R2Config| -> Box<dyn S3> {
        let cred = Credentials { access_key: c.access_key_id.clone(), secret_key: c.secret_access_key.clone() };
        Box::new(if endpoint.is_empty() { Client::r2(&c.account_id, cred) } else { Client::new(&endpoint, cred) })
    };
    let code = cli::run(&argv, &env, &cwd, &connect, &mut std::io::stdout().lock(), &mut std::io::stderr().lock());
    ExitCode::from(code as u8)
}
