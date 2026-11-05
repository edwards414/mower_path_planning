//! `mower_bag` (cli.rs) against mower-bag's `upload` and `verify`
//! (src/mower_recorder/mower_recorder/cli.py): test_cli.py's cases on an
//! in-memory S3 with F1's run, and the lines, streams and exit codes
//! cli.py's code gives for each outcome; the argument forms
//! mower-data-collection.sh passes; the binary itself where no network is
//! needed.

use std::cell::RefCell;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::rc::Rc;

use mower_recorder_core::cli::{self, Args};
use mower_recorder_core::s3::{ListPage, S3Error, S3};
use mower_recorder_core::store::R2Config;

const R1: &str = "MW-E5FIX_20261030T081851";
const BUCKET: &str = "mower-cli-test";

fn golden() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../../deploy/test/golden/recorder")
}

#[derive(Default)]
struct Store {
    objects: BTreeMap<String, Vec<u8>>,
    /// the n-th upload_file fails (1-based), as F4's injected 500
    fail_on_upload: Option<usize>,
    uploads: usize,
}

/// An in-memory S3 that the test keeps a handle on.
#[derive(Clone, Default)]
struct FakeS3(Rc<RefCell<Store>>);

impl S3 for FakeS3 {
    fn upload_file(&self, path: &Path, _b: &str, key: &str, _m: &[(String, String)]) -> Result<(), S3Error> {
        let mut s = self.0.borrow_mut();
        s.uploads += 1;
        if Some(s.uploads) == s.fail_on_upload {
            return Err(S3Error {
                operation: "PutObject",
                status: Some(500),
                code: "InternalError".into(),
                message: "injected".into(),
                retries: 4,
            });
        }
        s.objects.insert(key.into(), std::fs::read(path).unwrap());
        Ok(())
    }
    fn put_object(&self, _b: &str, key: &str, body: &[u8], _ct: Option<&str>) -> Result<(), S3Error> {
        self.0.borrow_mut().objects.insert(key.into(), body.to_vec());
        Ok(())
    }
    fn head_object(&self, _b: &str, key: &str) -> Result<Option<u64>, S3Error> {
        Ok(self.0.borrow().objects.get(key).map(|o| o.len() as u64))
    }
    fn get_object(&self, _b: &str, key: &str) -> Result<Option<Vec<u8>>, S3Error> {
        Ok(self.0.borrow().objects.get(key).cloned())
    }
    fn list_objects_v2(&self, _b: &str, prefix: &str, token: Option<&str>) -> Result<ListPage, S3Error> {
        // pages of 3, to walk the continuation
        let keys: Vec<String> = self.0.borrow().objects.keys().filter(|k| k.starts_with(prefix)).cloned().collect();
        let start: usize = token.map_or(0, |t| t.parse().unwrap());
        let page: Vec<String> = keys.iter().skip(start).take(3).cloned().collect();
        let more = start + 3 < keys.len();
        Ok(ListPage { keys: page, is_truncated: more, next_token: more.then(|| (start + 3).to_string()) })
    }
    fn delete_object(&self, _b: &str, key: &str) -> Result<(), S3Error> {
        self.0.borrow_mut().objects.remove(key);
        Ok(())
    }
}

fn tmpdir(tag: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("mower_recorder_core_cli_{tag}_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).unwrap();
    d
}

/// F1's run under `root` as `name`, its MCAPs decompressed.
fn make_run(root: &Path, name: &str) -> PathBuf {
    fn copy(src: &Path, dst: &Path) {
        std::fs::create_dir_all(dst).unwrap();
        for e in std::fs::read_dir(src).unwrap().flatten() {
            let p = e.path();
            let n = e.file_name().to_string_lossy().into_owned();
            if p.is_dir() {
                copy(&p, &dst.join(&n));
            } else if let Some(stem) = n.strip_suffix(".zst") {
                std::fs::write(dst.join(stem), zstd::decode_all(std::fs::File::open(&p).unwrap()).unwrap()).unwrap();
            } else {
                std::fs::copy(&p, dst.join(&n)).unwrap();
            }
        }
    }
    let d = root.join(name);
    copy(&golden().join("F1").join(R1), &d);
    d
}

/// The files the uploader sends for a run (dot files and the manifest left out).
fn file_count(run: &Path) -> usize {
    mower_recorder_core::manifest::list_run_files(run).len()
}

struct Out {
    code: i32,
    out: String,
    err: String,
}

/// cli.py's `main(argv)` with `env` as the environment and `s3` as the client.
fn run(argv: &[&str], env: &[(&str, &str)], cwd: &Path, s3: &FakeS3) -> Out {
    let argv: Vec<String> = argv.iter().map(|a| a.to_string()).collect();
    let env: BTreeMap<String, String> = env.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect();
    let lookup = |k: &str| env.get(k).cloned();
    let made = s3.clone();
    let connect = move |c: &R2Config| -> Box<dyn S3> {
        assert_eq!(c.bucket, BUCKET);
        Box::new(made.clone())
    };
    let (mut out, mut err) = (Vec::new(), Vec::new());
    let code = cli::run(&argv, &lookup, cwd, &connect, &mut out, &mut err);
    Out { code, out: String::from_utf8(out).unwrap(), err: String::from_utf8(err).unwrap() }
}

const CREDS: [(&str, &str); 4] =
    [("R2_ACCOUNT_ID", "acct"), ("R2_BUCKET", BUCKET), ("R2_ACCESS_KEY_ID", "AKID"), ("R2_SECRET_ACCESS_KEY", "secret")];

fn creds_with(extra: &[(&'static str, &'static str)]) -> Vec<(&'static str, &'static str)> {
    CREDS.iter().copied().chain(extra.iter().copied()).collect()
}

/// test_cli.py `test_upload_all_then_verify_and_ls` (ls is the Python's):
/// upload --all, nothing pending the second time, verify OK, and FAIL once
/// the manifest is gone.
#[test]
fn upload_all_then_verify() {
    let tmp = tmpdir("all");
    let root = tmp.join("bags");
    let run_dir = make_run(&root, R1);
    let n = file_count(&run_dir);
    let s3 = FakeS3::default();
    let env = creds_with(&[]);
    let root_s = root.to_str().unwrap();

    let r = run(&["upload", "--all", "--root", root_s], &env, &tmp, &s3);
    assert_eq!((r.code, r.err.as_str()), (0, ""), "{}", r.out);
    assert_eq!(r.out, format!("OK   {R1} -> bags/MW-E5FIX/{R1}/ ({n} files, {n} uploaded, 0 already there)\n"));
    assert!(run_dir.join(".uploaded").is_file());
    assert_eq!(s3.0.borrow().objects.len(), n + 1, "every file and the manifest");

    let r = run(&["upload", "--all", "--root", root_s], &env, &tmp, &s3);
    assert_eq!((r.code, r.out.as_str()), (0, "(nothing to upload)\n"));

    let r = run(&["verify", R1], &env, &tmp, &s3);
    assert_eq!((r.code, r.err.as_str()), (0, ""));
    assert_eq!(r.out, format!("OK   bags/MW-E5FIX/{R1}: _manifest.json 列出 {n} 個檔案，大小都相符\n"));
    // robot/run_id resolves too
    let r = run(&["verify", &format!("MW-E5FIX/{R1}")], &env, &tmp, &s3);
    assert_eq!(r.code, 0, "{}", r.err);

    s3.0.borrow_mut().objects.remove(&format!("bags/MW-E5FIX/{R1}/_manifest.json"));
    let r = run(&["verify", R1], &env, &tmp, &s3);
    assert_eq!(r.code, 1);
    assert_eq!(r.out, format!("FAIL bags/MW-E5FIX/{R1}: 沒有 _manifest.json（上傳未完成）\n"));

    // an object that went missing is named
    let r = run(&["upload", run_dir.to_str().unwrap()], &env, &tmp, &s3);
    assert_eq!(r.code, 0, "{}", r.out);
    s3.0.borrow_mut().objects.remove(&format!("bags/MW-E5FIX/{R1}/bag/metadata.yaml"));
    let r = run(&["verify", R1], &env, &tmp, &s3);
    assert_eq!(r.code, 1);
    assert_eq!(r.out, format!("FAIL bags/MW-E5FIX/{R1}: 缺少 bags/MW-E5FIX/{R1}/bag/metadata.yaml\n"));
    let _ = std::fs::remove_dir_all(&tmp);
}

/// test_cli.py `test_upload_skips_unfinished_run`: SKIP, exit 1, nothing sent.
#[test]
fn an_unfinished_run_is_skipped() {
    let tmp = tmpdir("skip");
    let run_dir = make_run(&tmp.join("bags"), R1);
    std::fs::remove_file(run_dir.join("bag/metadata.yaml")).unwrap();
    let s3 = FakeS3::default();
    let r = run(&["upload", run_dir.to_str().unwrap()], &creds_with(&[]), &tmp, &s3);
    assert_eq!(r.code, 1);
    assert_eq!(
        r.out,
        format!("SKIP {R1}: bag/ 沒有 metadata.yaml（錄製未正常結束？先執行 `ros2 bag reindex bag -s mcap`）\n")
    );
    assert!(s3.0.borrow().objects.is_empty());
    let _ = std::fs::remove_dir_all(&tmp);
}

/// A failed request: FAIL with the exception's name and text and the hint,
/// exit 1; the next run resumes where it stopped.
#[test]
fn a_failed_upload_resumes_on_the_next_run() {
    let tmp = tmpdir("fail");
    let run_dir = make_run(&tmp.join("bags"), R1);
    let n = file_count(&run_dir);
    let s3 = FakeS3::default();
    s3.0.borrow_mut().fail_on_upload = Some(3);
    let rel = mower_recorder_core::manifest::list_run_files(&run_dir)[2].clone();
    let r = run(&["upload", "--all", "--root", "bags"], &creds_with(&[]), &tmp, &s3);
    assert_eq!(r.code, 1);
    assert_eq!(
        r.out,
        format!(
            "FAIL {R1}: S3UploadFailedError: Failed to upload {} to {BUCKET}/bags/MW-E5FIX/{R1}/{rel}: \
             An error occurred (InternalError) when calling the PutObject operation (reached max retries: 4): \
             injected（再執行一次會接續上傳）\n",
            run_dir.join(&rel).display()
        )
    );
    let r = run(&["upload", "--all", "--root", "bags"], &creds_with(&[]), &tmp, &s3);
    assert_eq!(r.code, 0, "{}", r.out);
    assert_eq!(r.out, format!("OK   {R1} -> bags/MW-E5FIX/{R1}/ ({n} files, {} uploaded, 2 already there)\n", n - 2));
    let _ = std::fs::remove_dir_all(&tmp);
}

/// The robot id from run_metadata.yaml, else --robot-id, MOWER_ROBOT_ID or
/// `mower`; R2_PREFIX; relative run dirs from the working directory; `-v`
/// prints the uploader's lines first.
#[test]
fn keys_robot_ids_and_verbose() {
    let tmp = tmpdir("keys");
    let root = tmp.join("bags");
    let run_dir = make_run(&root, "R-A");
    let meta = run_dir.join("run_metadata.yaml");
    let text = std::fs::read_to_string(&meta).unwrap();
    let text: String = text.lines().filter(|l| !l.starts_with("robot_id:")).map(|l| format!("{l}\n")).collect();
    std::fs::write(&meta, text).unwrap();
    let s3 = FakeS3::default();
    let r = run(&["upload", "bags/R-A"], &creds_with(&[("R2_PREFIX", "/runs/")]), &tmp, &s3);
    assert!(r.out.starts_with("OK   R-A -> runs/mower/R-A/ ("), "{}", r.out);
    assert!(s3.0.borrow().objects.contains_key("runs/mower/R-A/_manifest.json"));
    std::fs::remove_dir_all(&run_dir).unwrap();

    let run_dir = make_run(&root, "R-B");
    let text = std::fs::read_to_string(run_dir.join("run_metadata.yaml")).unwrap();
    let text: String = text.lines().filter(|l| !l.starts_with("robot_id:")).map(|l| format!("{l}\n")).collect();
    std::fs::write(run_dir.join("run_metadata.yaml"), text).unwrap();
    let r = run(&["upload", "--robot-id", "MW-X", "bags/R-B"], &creds_with(&[("MOWER_ROBOT_ID", "MW-ENV")]), &tmp, &s3);
    assert!(r.out.starts_with("OK   R-B -> bags/MW-X/R-B/ ("), "{}", r.out);
    std::fs::remove_file(run_dir.join(".uploaded")).unwrap();
    std::fs::remove_file(run_dir.join(".upload_state.json")).unwrap();
    let r = run(&["upload", "-v", "--root=bags", "--all"], &creds_with(&[("MOWER_ROBOT_ID", "MW-ENV")]), &tmp, &s3);
    let lines: Vec<&str> = r.out.lines().collect();
    assert!(lines[0].starts_with("uploaded bags/MW-ENV/R-B/"), "{}", r.out);
    assert!(lines[lines.len() - 2].starts_with("manifest -> bags/MW-ENV/R-B/_manifest.json ("), "{}", r.out);
    assert!(lines[lines.len() - 1].starts_with("OK   R-B -> bags/MW-ENV/R-B/ ("), "{}", r.out);
    let _ = std::fs::remove_dir_all(&tmp);
}

/// `_resolve`: an unknown run and a run id two robots have, on stderr.
#[test]
fn verify_resolves_the_run() {
    let tmp = tmpdir("resolve");
    let s3 = FakeS3::default();
    for k in ["bags/MW-A/R1/_manifest.json", "bags/MW-B/R1/_manifest.json", "bags/MW-B/loose", "other/MW-C/R2/x"] {
        s3.0.borrow_mut().objects.insert(k.into(), b"{\"files\": []}".to_vec());
    }
    let env = creds_with(&[]);
    let r = run(&["verify", "R2"], &env, &tmp, &s3);
    assert_eq!((r.code, r.out.as_str(), r.err.as_str()), (1, "", "run not found: R2\n"));
    let r = run(&["verify", "R1"], &env, &tmp, &s3);
    assert_eq!((r.code, r.out.as_str(), r.err.as_str()), (1, "", "ambiguous, use robot/run_id: ['MW-A/R1', 'MW-B/R1']\n"));
    let r = run(&["verify", "MW-B/R1"], &env, &tmp, &s3);
    assert_eq!((r.code, r.out.as_str()), (0, "OK   bags/MW-B/R1: _manifest.json 列出 0 個檔案，大小都相符\n"));
    let _ = std::fs::remove_dir_all(&tmp);
}

/// `_load_env` and `_client`: the environment wins, the first file found
/// fills what it left empty (`--env-file`, else ./.env), and the four keys
/// a client needs are checked before any command runs.
#[test]
fn credentials_as_cli_py_loads_them() {
    let tmp = tmpdir("creds");
    std::fs::create_dir_all(tmp.join("bags")).unwrap();
    let s3 = FakeS3::default();
    let r = run(&["upload", "--all", "--root", "bags"], &[("R2_BUCKET", BUCKET)], &tmp, &s3);
    assert_eq!((r.code, r.out.as_str()), (1, ""));
    assert_eq!(r.err, "missing R2 creds: R2_ACCOUNT_ID, R2_ACCESS_KEY_ID, R2_SECRET_ACCESS_KEY  (set env or .env)\n");

    std::fs::write(
        tmp.join("r2.env"),
        "# R2\nR2_ACCOUNT_ID=acct\nR2_BUCKET=not-this-one\nR2_ACCESS_KEY_ID=\"AKID\"\nR2_SECRET_ACCESS_KEY='s'\n",
    )
    .unwrap();
    // connect asserts the bucket is the environment's
    let r = run(&["--env-file", "r2.env", "upload", "--all", "--root", "bags"], &[("R2_BUCKET", BUCKET)], &tmp, &s3);
    assert_eq!((r.code, r.out.as_str(), r.err.as_str()), (0, "(nothing to upload)\n", ""));
    let r = run(&["--env-file=r2.env", "upload", "--all", "--root", "bags"], &[("R2_BUCKET", BUCKET)], &tmp, &s3);
    assert_eq!(r.code, 0, "{}", r.err);
    // without --env-file: ./.env
    std::fs::rename(tmp.join("r2.env"), tmp.join(".env")).unwrap();
    let r = run(&["upload", "--all", "--root", "bags"], &[("R2_BUCKET", BUCKET), ("HOME", "/nonexistent")], &tmp, &s3);
    assert_eq!(r.code, 0, "{}", r.err);
    // an --env-file that does not exist is not replaced by ./.env
    let r = run(&["--env-file", "none.env", "upload", "--all", "--root", "bags"], &[("R2_BUCKET", BUCKET)], &tmp, &s3);
    assert_eq!(r.code, 1);
    let _ = std::fs::remove_dir_all(&tmp);
}

/// The forms mower-data-collection.sh passes, and argparse's refusals (exit 2).
#[test]
fn arguments() {
    let env = |k: &str| (k == "MOWER_BAG_ROOT").then(|| "/from/env".to_string());
    let parse = |a: &[&str]| cli::parse(&a.iter().map(|s| s.to_string()).collect::<Vec<_>>(), &env);
    assert_eq!(
        parse(&["--env-file", "/home/mower/.mower/r2.env", "upload", "--all", "--root", "/home/mower/.mower/bags"]),
        Ok(Some(Args {
            env_file: "/home/mower/.mower/r2.env".into(),
            command: cli::Command::Upload {
                run_dirs: vec![],
                all: true,
                root: "/home/mower/.mower/bags".into(),
                robot_id: "mower".into(),
                verbose: false
            },
        }))
    );
    assert_eq!(
        parse(&["--env-file", "/home/mower/.mower/r2.env", "verify", "MW-1_20261104T101010"]),
        Ok(Some(Args {
            env_file: "/home/mower/.mower/r2.env".into(),
            command: cli::Command::Verify { run_id: "MW-1_20261104T101010".into() },
        }))
    );
    match parse(&["upload", "a", "-v", "b"]) {
        Ok(Some(Args { command: cli::Command::Upload { run_dirs, root, verbose, .. }, .. })) => {
            assert_eq!((run_dirs, root.as_str(), verbose), (vec!["a".to_string(), "b".to_string()], "/from/env", true))
        }
        other => panic!("{other:?}"),
    }
    assert_eq!(parse(&["-h"]), Ok(None));
    assert_eq!(parse(&["upload", "--help"]), Ok(None));
    for bad in [&[][..], &["verify"], &["verify", "a", "b"], &["upload", "--root"], &["upload", "--frob"], &["frob"], &["upload", "--env-file", "x"]] {
        assert!(parse(bad).is_err(), "{bad:?}");
    }
    let s3 = FakeS3::default();
    let tmp = tmpdir("args");
    let r = run(&["ls"], &creds_with(&[]), &tmp, &s3);
    assert_eq!(r.code, 2);
    assert!(r.err.ends_with("mower_bag: error: ls is the Python mower-bag's (ros2 run mower_recorder mower-bag ls, off the robot)\n"), "{}", r.err);
    let r = run(&["verify"], &creds_with(&[]), &tmp, &s3);
    assert_eq!(r.code, 2);
    assert!(r.err.starts_with("usage: mower_bag "), "{}", r.err);
    let _ = std::fs::remove_dir_all(&tmp);
}

/// The binary: help, a bad argument, no credentials, and nothing to upload
/// (none of them reaches the network).
#[test]
fn the_binary() {
    let tmp = tmpdir("bin");
    std::fs::create_dir_all(tmp.join("bags")).unwrap();
    let bin = env!("CARGO_BIN_EXE_mower_bag");
    let go = |args: &[&str], creds: bool| {
        let mut c = Command::new(bin);
        c.args(args).current_dir(&tmp).env_clear().env("HOME", &tmp);
        if creds {
            c.envs(CREDS);
        }
        c.output().unwrap()
    };
    let o = go(&["-h"], false);
    assert_eq!(o.status.code(), Some(0));
    assert!(String::from_utf8_lossy(&o.stdout).starts_with("usage: mower_bag "));
    let o = go(&["frob"], true);
    assert_eq!(o.status.code(), Some(2));
    let o = go(&["upload", "--all", "--root", "bags"], false);
    assert_eq!(o.status.code(), Some(1));
    assert_eq!(
        String::from_utf8_lossy(&o.stderr),
        "missing R2 creds: R2_ACCOUNT_ID, R2_BUCKET, R2_ACCESS_KEY_ID, R2_SECRET_ACCESS_KEY  (set env or .env)\n"
    );
    let o = go(&["upload", "--all", "--root", "bags"], true);
    assert_eq!((o.status.code(), String::from_utf8_lossy(&o.stdout).as_ref()), (Some(0), "(nothing to upload)\n"));
    let o = go(&["upload", "--all", "--root", "missing"], true);
    assert_eq!(o.status.code(), Some(1));
    let _ = std::fs::remove_dir_all(&tmp);
}
