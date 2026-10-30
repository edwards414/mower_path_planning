//! G5's Rust side (e2-verify `run.sh upload B`): one uploader step against
//! an S3 endpoint, as `upload_py.py` runs the Python's, so the two can be
//! compared request by request and file by file.
//!
//!     upload_check upload <run_dir> <run_id>   upload_run, prints the result as results.json holds it
//!     upload_check verify <run_id>             verify_remote
//!     upload_check delete <run_id>             delete_run_objects
//!
//! Environment: S3_ENDPOINT (e.g. http://127.0.0.1:9000), the keys of
//! upload_py.py, bucket `mower-bags`, robot `MW-E5FIX`, zone UTC.

use std::path::Path;

use mower_pyfmt::json::{self, JsonOptions};
use mower_pyfmt::Py;
use mower_recorder_core::manifest;
use mower_recorder_core::s3::{Client, Credentials};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let endpoint = std::env::var("S3_ENDPOINT").unwrap_or_else(|_| "http://127.0.0.1:9000".into());
    let client = Client::new(
        &endpoint,
        Credentials { access_key: "AKIDE5TEST0000000000".into(), secret_key: "e5-test-secret-key/0000000000000000000000".into() },
    );
    let bucket = "mower-bags";
    let result: Py = match args.get(1).map(String::as_str) {
        Some("upload") => {
            let (dir, run_id) = (Path::new(&args[2]), &args[3]);
            let base = manifest::run_key("bags", "MW-E5FIX", run_id);
            match manifest::upload_run(&client, bucket, &base, dir, run_id, "MW-E5FIX", None, &mut |l| eprintln!("{l}")) {
                Ok(o) => Py::dict([(
                    "ok",
                    Py::dict([
                        ("files", Py::Int(o.files as i64)),
                        ("manifest_key", Py::Str(o.manifest_key)),
                        ("skipped", Py::Int(o.skipped as i64)),
                        ("uploaded", Py::Int(o.uploaded as i64)),
                    ]),
                )]),
                Err(e) => Py::dict([("error", Py::Str(e.upload_error()))]),
            }
        }
        Some("verify") => {
            let base = manifest::run_key("bags", "MW-E5FIX", &args[2]);
            match manifest::verify_remote(&client, bucket, &base) {
                Ok((ok, problems, _)) => Py::dict([("ok", Py::List(vec![Py::Bool(ok), Py::List(problems.into_iter().map(Py::Str).collect())]))]),
                Err(e) => Py::dict([("error", Py::Str(e.upload_error()))]),
            }
        }
        Some("delete") => {
            let base = manifest::run_key("bags", "MW-E5FIX", &args[2]);
            match manifest::delete_run_objects(&client, bucket, &base) {
                Ok(n) => Py::dict([("ok", Py::Int(n as i64))]),
                Err(e) => Py::dict([("error", Py::Str(format!("ClientError: {e}")))]),
            }
        }
        _ => {
            eprintln!("usage: upload_check upload <run_dir> <run_id> | verify <run_id> | delete <run_id>");
            std::process::exit(2);
        }
    };
    println!("{}", json::dumps(&result, &JsonOptions { ensure_ascii: true, indent: Some(1), sort_keys: true }).unwrap());
}
