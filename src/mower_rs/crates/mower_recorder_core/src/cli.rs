//! `mower_bag` (src/bin/mower_bag.rs): the `upload` and `verify` commands of
//! mower-bag (src/mower_recorder/mower_recorder/cli.py), the two that
//! mower-data-collection.sh runs on the robot (design D9, E5-7). Their
//! arguments, credentials (the environment first, then the first `.env` file
//! found fills what it left empty), output lines and exit codes are the
//! Python's; `ls`, `get` and `rm` stay in the Python mower-bag, an off-robot
//! tool. The upload is [`manifest::upload_run`], bag_store's.
//!
//! Exit codes: 0 done, 1 a run failed or was skipped, a check failed, no
//! credentials, or R2 could not be asked; 2 bad arguments (argparse's).
//! Where the Python died with a traceback (an unreadable `--root`, an R2
//! error in `verify`) this prints one line on stderr and exits 1 as it did.

use std::io::Write;
use std::path::{Component, Path, PathBuf};

use mower_pyfmt::Py;

use crate::manifest::{self, UploadError, UPLOADED_MARKER};
use crate::s3::S3;
use crate::store::{load_cli_env, R2Config};
use crate::util::expand_user;

/// What `-h` prints.
pub const HELP: &str = "\
usage: mower_bag [-h] [--env-file ENV_FILE] {upload,verify} ...

mower-bag's upload and verify (cli.py), as mower-data-collection.sh runs
them on the robot; ls, get and rm are the Python mower-bag's (off the robot).

  mower_bag upload <run_dir>... | --all [--root DIR] [--robot-id ID] [-v]
                                    manifest last, resumable
  mower_bag verify <run_id>         R2 manifest vs objects (sizes)

R2 credentials: R2_ACCOUNT_ID R2_BUCKET R2_ACCESS_KEY_ID R2_SECRET_ACCESS_KEY
R2_PREFIX from the environment, else --env-file (or the first of ./.env,
~/.config/mower/r2.env, ~/mower_ws/.env). --root defaults to MOWER_BAG_ROOT,
else ~/.mower/bags; --robot-id (when run_metadata.yaml has none) to
MOWER_ROBOT_ID, else mower.
";

const USAGE: &str = "usage: mower_bag [-h] [--env-file ENV_FILE] {upload,verify} ...";

/// The four keys a client needs (cli `_client`).
const NEEDED: [&str; 4] = ["R2_ACCOUNT_ID", "R2_BUCKET", "R2_ACCESS_KEY_ID", "R2_SECRET_ACCESS_KEY"];

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Command {
    Upload { run_dirs: Vec<String>, all: bool, root: String, robot_id: String, verbose: bool },
    Verify { run_id: String },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Args {
    pub env_file: String,
    pub command: Command,
}

/// An option's value: `--name VALUE` or `--name=VALUE`.
fn value(arg: &str, name: &str, rest: &mut std::slice::Iter<'_, String>) -> Option<Result<String, String>> {
    if arg == name {
        return Some(rest.next().cloned().ok_or_else(|| format!("argument {name}: expected one argument")));
    }
    arg.strip_prefix(name).and_then(|v| v.strip_prefix('=')).map(|v| Ok(v.to_string()))
}

/// cli.py's argparse for the two commands: `Ok(None)` is `-h`, `Err` the
/// error text (exit 2).
pub fn parse(argv: &[String], env: &dyn Fn(&str) -> Option<String>) -> Result<Option<Args>, String> {
    let mut it = argv.iter();
    let mut env_file = String::new();
    let cmd = loop {
        let Some(a) = it.next() else { return Err("the following arguments are required: cmd".into()) };
        if a == "-h" || a == "--help" {
            return Ok(None);
        }
        if let Some(v) = value(a, "--env-file", &mut it) {
            env_file = v?;
            continue;
        }
        if a.starts_with('-') && a != "-" {
            return Err(format!("unrecognized arguments: {a}"));
        }
        break a.as_str();
    };
    let command = match cmd {
        "upload" => {
            let mut run_dirs = Vec::new();
            let mut all = false;
            // os.environ.get('MOWER_BAG_ROOT', '~/.mower/bags')
            let mut root = env("MOWER_BAG_ROOT").unwrap_or_else(|| "~/.mower/bags".into());
            // os.environ.get('MOWER_ROBOT_ID') or 'mower'
            let mut robot_id = env("MOWER_ROBOT_ID").filter(|v| !v.is_empty()).unwrap_or_else(|| "mower".into());
            let mut verbose = false;
            let mut options = true;
            while let Some(a) = it.next() {
                if !options || !a.starts_with('-') || a == "-" {
                    run_dirs.push(a.clone());
                } else if a == "--" {
                    options = false;
                } else if a == "-h" || a == "--help" {
                    return Ok(None);
                } else if a == "--all" {
                    all = true;
                } else if a == "-v" || a == "--verbose" {
                    verbose = true;
                } else if let Some(v) = value(a, "--root", &mut it) {
                    root = v?;
                } else if let Some(v) = value(a, "--robot-id", &mut it) {
                    robot_id = v?;
                } else {
                    return Err(format!("unrecognized arguments: {a}"));
                }
            }
            Command::Upload { run_dirs, all, root, robot_id, verbose }
        }
        "verify" => {
            let mut ids = Vec::new();
            let mut options = true;
            for a in it {
                if !options || !a.starts_with('-') || a == "-" {
                    ids.push(a.clone());
                } else if a == "--" {
                    options = false;
                } else if a == "-h" || a == "--help" {
                    return Ok(None);
                } else {
                    return Err(format!("unrecognized arguments: {a}"));
                }
            }
            match ids.len() {
                0 => return Err("the following arguments are required: run_id".into()),
                1 => Command::Verify { run_id: ids.remove(0) },
                _ => return Err(format!("unrecognized arguments: {}", ids[1..].join(" "))),
            }
        }
        "ls" | "get" | "rm" => {
            return Err(format!("{cmd} is the Python mower-bag's (ros2 run mower_recorder mower-bag {cmd}, off the robot)"))
        }
        other => {
            return Err(format!("argument cmd: invalid choice: {} (choose from 'upload', 'verify')", py_str_repr(other)))
        }
    };
    Ok(Some(Args { env_file, command }))
}

/// Python's `repr(str)` for the plain texts this prints (quotes and
/// backslashes; run ids and keys hold nothing else).
fn py_str_repr(s: &str) -> String {
    let q = if s.contains('\'') && !s.contains('"') { '"' } else { '\'' };
    let mut out = String::with_capacity(s.len() + 2);
    out.push(q);
    for c in s.chars() {
        match c {
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if c == q => {
                out.push('\\');
                out.push(c);
            }
            c => out.push(c),
        }
    }
    out.push(q);
    out
}

/// `os.path.abspath` (lexical: `.` and `..` folded, no symlinks resolved).
fn abspath(cwd: &Path, p: &str) -> PathBuf {
    let joined = cwd.join(p);
    let mut out = PathBuf::from("/");
    for c in joined.components() {
        match c {
            Component::ParentDir => {
                out.pop();
            }
            Component::Normal(n) => out.push(n),
            Component::RootDir | Component::CurDir | Component::Prefix(_) => {}
        }
    }
    out
}

/// `os.path.basename(run_dir.rstrip('/'))`
fn basename(p: &Path) -> String {
    p.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default()
}

/// cli `_pending_runs`: the run directories under `root` without
/// `.uploaded`, dot names left out, sorted by name.
fn pending_runs(root: &Path) -> std::io::Result<Vec<PathBuf>> {
    let mut names: Vec<String> =
        std::fs::read_dir(root)?.flatten().map(|e| e.file_name().to_string_lossy().into_owned()).collect();
    names.sort();
    Ok(names
        .into_iter()
        .filter(|n| !n.starts_with('.'))
        .map(|n| root.join(n))
        .filter(|d| d.is_dir() && !d.join(UPLOADED_MARKER).is_file())
        .collect())
}

/// The credential files cli `_load_env` looks in, in order.
fn candidates(env_file: &str, cwd: &Path) -> Vec<PathBuf> {
    if env_file.is_empty() {
        vec![
            cwd.join(".env"),
            PathBuf::from(expand_user("~/.config/mower/r2.env")),
            PathBuf::from(expand_user("~/mower_ws/.env")),
        ]
    } else {
        // as given: cli.py does not expand a `~` in it
        vec![cwd.join(env_file)]
    }
}

/// `main(argv)`: the exit code. `connect` makes the client for the settings
/// (the R2 endpoint in the binary, an in-memory S3 in the tests).
pub fn run(
    argv: &[String],
    env: &dyn Fn(&str) -> Option<String>,
    cwd: &Path,
    connect: &dyn Fn(&R2Config) -> Box<dyn S3>,
    out: &mut dyn Write,
    err: &mut dyn Write,
) -> i32 {
    let args = match parse(argv, env) {
        Ok(Some(a)) => a,
        Ok(None) => {
            let _ = out.write_all(HELP.as_bytes());
            return 0;
        }
        Err(e) => {
            let _ = writeln!(err, "{USAGE}\nmower_bag: error: {e}");
            return 2;
        }
    };
    let cfg = load_cli_env(env, &candidates(&args.env_file, cwd));
    let value = |k: &str| match k {
        "R2_ACCOUNT_ID" => &cfg.account_id,
        "R2_BUCKET" => &cfg.bucket,
        "R2_ACCESS_KEY_ID" => &cfg.access_key_id,
        _ => &cfg.secret_access_key,
    };
    let missing: Vec<&str> = NEEDED.iter().copied().filter(|k| value(k).is_empty()).collect();
    if !missing.is_empty() {
        let _ = writeln!(err, "missing R2 creds: {}  (set env or .env)", missing.join(", "));
        return 1;
    }
    let client = connect(&cfg);
    match args.command {
        Command::Upload { run_dirs, all, root, robot_id, verbose } => {
            upload(client.as_ref(), &cfg, &run_dirs, all, &root, &robot_id, verbose, cwd, out, err)
        }
        Command::Verify { run_id } => verify(client.as_ref(), &cfg, &run_id, out, err),
    }
}

/// cli `cmd_upload`: bag_store's upload, one run after the other.
#[allow(clippy::too_many_arguments)]
fn upload(
    client: &dyn S3,
    cfg: &R2Config,
    run_dirs: &[String],
    all: bool,
    root: &str,
    robot_id: &str,
    verbose: bool,
    cwd: &Path,
    out: &mut dyn Write,
    err: &mut dyn Write,
) -> i32 {
    let root = cwd.join(expand_user(root));
    let mut dirs: Vec<PathBuf> = run_dirs.iter().map(|d| abspath(cwd, &expand_user(d))).collect();
    if all {
        match pending_runs(&root) {
            Ok(found) => dirs.extend(found),
            Err(e) => {
                let _ = writeln!(err, "mower_bag: {}: {e}", root.display());
                return 1;
            }
        }
    }
    if dirs.is_empty() {
        let _ = writeln!(out, "(nothing to upload)");
        return 0;
    }
    let mut failed = 0;
    for run_dir in &dirs {
        let run_id = basename(run_dir);
        // meta.get('robot_id') or args.robot_id; a run_metadata.yaml that
        // does not parse fails the upload below, which reads it again
        let robot = match manifest::read_run_metadata(run_dir).ok().as_ref().and_then(|m| m.get("robot_id")) {
            Some(Py::Str(s)) if !s.is_empty() => s.clone(),
            _ => robot_id.to_string(),
        };
        let base = manifest::run_key(&cfg.prefix, &robot, &run_id);
        let res = manifest::upload_run(client, &cfg.bucket, &base, run_dir, &run_id, &robot, None, &mut |l| {
            if verbose {
                let _ = writeln!(out, "{l}");
            }
        });
        match res {
            Ok(o) => {
                let _ = writeln!(
                    out,
                    "OK   {run_id} -> {base}/ ({} files, {} uploaded, {} already there)",
                    o.files, o.uploaded, o.skipped
                );
            }
            Err(UploadError::NotReady(m)) => {
                let _ = writeln!(out, "SKIP {run_id}: {m}");
                failed += 1;
            }
            Err(e) => {
                let _ = writeln!(out, "FAIL {run_id}: {}（再執行一次會接續上傳）", e.upload_error());
                failed += 1;
            }
        }
    }
    i32::from(failed > 0)
}

/// cli `cmd_verify`: the run by its id (or robot/run_id) among the bucket's
/// runs, then its manifest against the objects.
fn verify(client: &dyn S3, cfg: &R2Config, run_id: &str, out: &mut dyn Write, err: &mut dyn Write) -> i32 {
    let prefix = cfg.prefix.trim_end_matches('/');
    let base = format!("{prefix}/");
    let keys = match manifest::iter_keys(client, &cfg.bucket, &base) {
        Ok(k) => k,
        Err(e) => {
            let _ = writeln!(err, "mower_bag: {}", UploadError::Client(e).upload_error());
            return 1;
        }
    };
    // cli `_runs`: robot/run_id of every key with a file under it, in the
    // listing's order
    let mut runs: Vec<(String, String)> = Vec::new();
    for key in &keys {
        let parts: Vec<&str> = key.get(base.len()..).unwrap_or("").split('/').collect();
        if parts.len() < 3 {
            continue;
        }
        let run_key = format!("{}/{}", parts[0], parts[1]);
        if !runs.iter().any(|(k, _)| *k == run_key) {
            runs.push((run_key, parts[1].to_string()));
        }
    }
    // cli `_resolve`
    let hits: Vec<&str> = runs.iter().filter(|(k, id)| id == run_id || k == run_id).map(|(k, _)| k.as_str()).collect();
    let run_key = match hits.as_slice() {
        [one] => *one,
        [] => {
            let _ = writeln!(err, "run not found: {run_id}");
            return 1;
        }
        many => {
            let shown: Vec<String> = many.iter().map(|h| py_str_repr(h)).collect();
            let _ = writeln!(err, "ambiguous, use robot/run_id: [{}]", shown.join(", "));
            return 1;
        }
    };
    let base = format!("{prefix}/{run_key}");
    match manifest::verify_remote(client, &cfg.bucket, &base) {
        Ok((true, _, manifest)) => {
            let n = match manifest.as_ref().and_then(|m| m.get("files")) {
                Some(Py::List(files)) => files.len(),
                _ => 0,
            };
            let _ = writeln!(out, "OK   {base}: _manifest.json 列出 {n} 個檔案，大小都相符");
            0
        }
        Ok((false, problems, _)) => {
            for p in problems {
                let _ = writeln!(out, "FAIL {base}: {p}");
            }
            1
        }
        Err(e) => {
            let _ = writeln!(err, "mower_bag: {}", e.upload_error());
            1
        }
    }
}
