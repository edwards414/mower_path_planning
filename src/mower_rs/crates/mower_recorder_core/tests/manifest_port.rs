//! The uploader (manifest.rs) against what the Python uploader did and
//! wrote: test_run_manifest.py's cases on an in-memory S3, and the F4
//! capture (deploy/test/golden/recorder/F4, e2-verify `run.sh upload A`):
//! the same runs rebuilt as upload_run.sh builds them (F1's and F3's run dirs,
//! a 20 MiB video segment, fixed mtimes), the same scenarios, and every file
//! the uploader leaves in the run (`.upload_state.json`, `.uploaded`,
//! `_manifest.json`) byte for byte the Python's.

use std::cell::RefCell;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use mower_recorder_core::manifest::{self, UploadError};
use mower_recorder_core::s3::{ListPage, S3Error, S3};
use serde_json::Value;

fn golden() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../../deploy/test/golden/recorder")
}

const R1: &str = "MW-E5FIX_20261030T081851";
const R3: &str = "MW-E5FIX_20261030T080906";
const BUCKET: &str = "mower-bags";

/// An S3 that keeps objects in memory and logs every call; optionally the
/// n-th upload_file fails as F4's injected 500 did.
#[derive(Default)]
struct FakeS3 {
    objects: RefCell<BTreeMap<String, (Vec<u8>, Vec<(String, String)>)>>,
    calls: RefCell<Vec<(&'static str, String)>>,
    fail_on_upload: Option<usize>,
    uploads: RefCell<usize>,
}

impl S3 for FakeS3 {
    fn upload_file(&self, path: &Path, _b: &str, key: &str, metadata: &[(String, String)]) -> Result<(), S3Error> {
        *self.uploads.borrow_mut() += 1;
        if Some(*self.uploads.borrow()) == self.fail_on_upload {
            self.calls.borrow_mut().push(("upload_file_failed", key.into()));
            return Err(S3Error {
                operation: "PutObject",
                status: Some(500),
                code: "InternalError".into(),
                message: "injected by s3_proxy".into(),
                retries: 4,
            });
        }
        self.objects.borrow_mut().insert(key.into(), (std::fs::read(path).unwrap(), metadata.to_vec()));
        self.calls.borrow_mut().push(("upload_file", key.into()));
        Ok(())
    }
    fn put_object(&self, _b: &str, key: &str, body: &[u8], ct: Option<&str>) -> Result<(), S3Error> {
        let meta = ct.map(|c| vec![("content-type".to_string(), c.to_string())]).unwrap_or_default();
        self.objects.borrow_mut().insert(key.into(), (body.to_vec(), meta));
        self.calls.borrow_mut().push(("put_object", key.into()));
        Ok(())
    }
    fn head_object(&self, _b: &str, key: &str) -> Result<Option<u64>, S3Error> {
        self.calls.borrow_mut().push(("head_object", key.into()));
        Ok(self.objects.borrow().get(key).map(|o| o.0.len() as u64))
    }
    fn get_object(&self, _b: &str, key: &str) -> Result<Option<Vec<u8>>, S3Error> {
        Ok(self.objects.borrow().get(key).map(|o| o.0.clone()))
    }
    fn list_objects_v2(&self, _b: &str, prefix: &str, token: Option<&str>) -> Result<ListPage, S3Error> {
        // pages of 3, to walk the continuation
        let keys: Vec<String> = self.objects.borrow().keys().filter(|k| k.starts_with(prefix)).cloned().collect();
        let start: usize = token.map_or(0, |t| t.parse().unwrap());
        let page: Vec<String> = keys.iter().skip(start).take(3).cloned().collect();
        let more = start + 3 < keys.len();
        Ok(ListPage { keys: page, is_truncated: more, next_token: more.then(|| (start + 3).to_string()) })
    }
    fn delete_object(&self, _b: &str, key: &str) -> Result<(), S3Error> {
        self.objects.borrow_mut().remove(key);
        self.calls.borrow_mut().push(("delete_object", key.into()));
        Ok(())
    }
}

/// upload_run.sh's run: the fixture run decompressed, F1's with a 20 MiB
/// video segment, every file's mtime fixed in `sorted(os.walk(d))` order.
fn build_run(tmp: &Path, name: &str, src: &str, run_id: &str) -> PathBuf {
    let d = tmp.join(name).join(run_id);
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
    copy(&golden().join(src).join(run_id), &d);
    if src == "F1" {
        std::fs::create_dir_all(d.join("video")).unwrap();
        std::fs::write(d.join("video/front_2026-10-30_08-18-51-000000.mp4"), segment()).unwrap();
    }
    // sorted(os.walk(d)): directories by their path string, files sorted
    fn dirs(p: &Path, out: &mut Vec<PathBuf>) {
        out.push(p.to_path_buf());
        for e in std::fs::read_dir(p).unwrap().flatten() {
            if e.path().is_dir() {
                dirs(&e.path(), out);
            }
        }
    }
    let mut all = Vec::new();
    dirs(&d, &mut all);
    all.sort_by(|a, b| a.to_string_lossy().cmp(&b.to_string_lossy()));
    let mut t: i64 = 1_793_348_331_000_000_000;
    for dir in all {
        let mut files: Vec<String> = std::fs::read_dir(&dir)
            .unwrap()
            .flatten()
            .filter(|e| e.path().is_file())
            .map(|e| e.file_name().to_string_lossy().into_owned())
            .collect();
        files.sort();
        for f in files {
            t += 1000;
            set_mtime(&dir.join(f), t);
        }
    }
    d
}

/// upload_run.sh's 20 MiB segment (sha256 of each index), made once.
fn segment() -> &'static [u8] {
    static DATA: std::sync::OnceLock<Vec<u8>> = std::sync::OnceLock::new();
    DATA.get_or_init(|| {
        use sha2::Digest;
        let mut data = Vec::with_capacity(20 << 20);
        for i in 0u32..((20 << 20) / 32) {
            data.extend_from_slice(&sha2::Sha256::digest(i.to_le_bytes()));
        }
        data
    })
}

fn set_mtime(p: &Path, ns: i64) {
    let t = std::time::UNIX_EPOCH + std::time::Duration::from_nanos(ns as u64);
    std::fs::File::options().write(true).open(p).unwrap().set_modified(t).unwrap();
}

fn tmpdir(tag: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("mower_recorder_core_{tag}_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).unwrap();
    d
}

fn base(run_id: &str) -> String {
    manifest::run_key("bags", "MW-E5FIX", run_id)
}

fn upload(s3: &FakeS3, run: &Path, run_id: &str) -> Result<manifest::Outcome, UploadError> {
    manifest::upload_run(s3, BUCKET, &base(run_id), run, run_id, "MW-E5FIX", Some(0), &mut |_| {})
}

/// The run's bookkeeping files against F4's for scenario `s`.
fn same_as_python(run: &Path, s: &str) {
    let g = golden().join("F4").join(s);
    for (ours, theirs) in [(".upload_state.json", "upload_state.json"), (".uploaded", "uploaded"), ("_manifest.json", "_manifest.json")] {
        let want = std::fs::read(g.join(theirs));
        let got = std::fs::read(run.join(ours));
        match (want, got) {
            (Ok(w), Ok(g)) => assert!(w == g, "{s} {ours}:\n--- python\n{}\n--- rust\n{}", String::from_utf8_lossy(&w), String::from_utf8_lossy(&g)),
            (Err(_), Err(_)) => {}
            (w, g) => panic!("{s} {ours}: python has it {}, rust {}", w.is_ok(), g.is_ok()),
        }
    }
}

fn results() -> Value {
    serde_json::from_str(&std::fs::read_to_string(golden().join("F4/results.json")).unwrap()).unwrap()
}

fn outcome_is(o: &manifest::Outcome, v: &Value) {
    let ok = &v["ok"];
    assert_eq!((o.uploaded as u64, o.skipped as u64, o.files as u64), (ok["uploaded"].as_u64().unwrap(), ok["skipped"].as_u64().unwrap(), ok["files"].as_u64().unwrap()));
    assert_eq!(o.manifest_key, ok["manifest_key"].as_str().unwrap());
}

#[test]
fn f4_every_scenario_leaves_the_python_s_files() {
    let tmp = tmpdir("f4");
    let res = results();
    // S1: the first upload; S2: again, nothing to do
    let r1 = build_run(&tmp, "R1", "F1", R1);
    let s3 = FakeS3::default();
    outcome_is(&upload(&s3, &r1, R1).unwrap(), &res["S1"]);
    same_as_python(&r1, "S1");
    // the objects: keys, sizes, bodies and the sha256 metadata
    let objects: Value = serde_json::from_str(&std::fs::read_to_string(golden().join("F4/S1/objects.json")).unwrap()).unwrap();
    let ours = s3.objects.borrow();
    assert_eq!(objects.as_array().unwrap().len(), ours.len());
    for o in objects.as_array().unwrap() {
        let (body, meta) = &ours[o["key"].as_str().unwrap()];
        assert_eq!(body.len() as u64, o["size"].as_u64().unwrap());
        assert_eq!(mower_recorder_core::util::sha256_hex(body), o["sha256"].as_str().unwrap());
        if let Some(sha) = o["metadata"]["sha256"].as_str() {
            assert_eq!(meta, &vec![("sha256".to_string(), sha.to_string())]);
        }
    }
    drop(ours);
    let writes: Vec<_> = s3.calls.borrow().iter().filter(|c| c.0 != "head_object").cloned().collect();
    assert_eq!(writes.last().unwrap(), &("put_object", format!("{}/_manifest.json", base(R1))));
    outcome_is(&upload(&s3, &r1, R1).unwrap(), &res["S2"]);
    same_as_python(&r1, "S2");
    // S3, S4: verify, then delete (manifest first)
    let (ok, problems, _) = manifest::verify_remote(&s3, BUCKET, &base(R1)).unwrap();
    assert!(ok && problems.is_empty());
    s3.calls.borrow_mut().clear();
    assert_eq!(manifest::delete_run_objects(&s3, BUCKET, &base(R1)).unwrap() as u64, res["S4"]["ok"].as_u64().unwrap());
    let dels: Vec<String> = s3.calls.borrow().iter().filter(|c| c.0 == "delete_object").map(|c| c.1.clone()).collect();
    assert_eq!(dels[0], format!("{}/_manifest.json", base(R1)));
    assert!(s3.objects.borrow().is_empty());
    // S5: the data_collection run
    let r3 = build_run(&tmp, "R3", "F3", R3);
    outcome_is(&upload(&s3, &r3, R3).unwrap(), &res["S5"]);
    same_as_python(&r3, "S5");
    // S6: the 3rd upload fails every retry, then the resume
    let r1b = build_run(&tmp, "R1b", "F1", R1);
    let failing = FakeS3 { fail_on_upload: Some(3), ..Default::default() };
    let err = upload(&failing, &r1b, R1).unwrap_err();
    let want = res["S6a"]["error"].as_str().unwrap().replace("/work/runs/R1b/", &format!("{}/", tmp.join("R1b").display()));
    assert_eq!(err.upload_error(), want);
    same_as_python(&r1b, "S6a");
    let resume = FakeS3 { objects: RefCell::new(failing.objects.borrow().clone()), ..Default::default() };
    outcome_is(&upload(&resume, &r1b, R1).unwrap(), &res["S6b"]);
    same_as_python(&r1b, "S6b");
    // S7: an object gone behind the uploader's back: the HEAD pass re-uploads it
    resume.objects.borrow_mut().remove(&format!("{}/run_metadata.yaml", base(R1)));
    outcome_is(&upload(&resume, &r1b, R1).unwrap(), &res["S7"]);
    same_as_python(&r1b, "S7");
    // S8: a file changed after the upload is hashed again and re-uploaded
    let p = r1b.join("params/map_manage.yaml");
    let mut text = std::fs::read_to_string(&p).unwrap();
    text.push_str("# changed\n");
    std::fs::write(&p, text).unwrap();
    set_mtime(&p, 1_793_349_000_000_000_000);
    outcome_is(&upload(&resume, &r1b, R1).unwrap(), &res["S8"]);
    same_as_python(&r1b, "S8");
    std::fs::remove_dir_all(&tmp).unwrap();
}

#[test]
fn an_unfinished_bag_blocks_before_any_request() {
    let tmp = tmpdir("unfinished");
    let r = build_run(&tmp, "R", "F3", R3);
    std::fs::remove_file(r.join("camera/metadata.yaml")).unwrap();
    let s3 = FakeS3::default();
    match upload(&s3, &r, R3) {
        Err(UploadError::NotReady(m)) => {
            assert_eq!(m, "camera/ 沒有 metadata.yaml（錄製未正常結束？先執行 `ros2 bag reindex camera -s mcap`）")
        }
        other => panic!("{other:?}"),
    }
    assert!(s3.calls.borrow().is_empty());
    // a file the metadata names but the disk lacks
    std::fs::copy(golden().join("F3").join(R3).join("camera/metadata.yaml"), r.join("camera/metadata.yaml")).unwrap();
    std::fs::remove_file(r.join("camera/camera_0.mcap")).unwrap();
    std::fs::write(r.join("camera/other.mcap"), b"x").unwrap();
    let m = manifest::check_ready(&r, &manifest::list_run_files(&r));
    assert_eq!(m, ["camera/camera_0.mcap 列在 metadata.yaml 但檔案不存在"]);
    std::fs::remove_dir_all(&tmp).unwrap();
}

#[test]
fn a_legacy_run_without_camera_and_the_manifest_s_fields() {
    let tmp = tmpdir("legacy");
    let r = build_run(&tmp, "R", "F1", R1);
    std::fs::write(r.join("run_metadata.yaml"), "run_id: x\nbuild:\n  git_sha: abc123\nprofile: data_collection\n").unwrap();
    let s3 = FakeS3::default();
    manifest::upload_run(&s3, BUCKET, &base(R1), &r, R1, "MW-E5FIX", Some(8 * 3600), &mut |_| {}).unwrap();
    let man = mower_pyfmt::json::loads(&std::fs::read_to_string(r.join("_manifest.json")).unwrap()).unwrap();
    let keys: Vec<&str> = match &man { mower_pyfmt::Py::Dict(d) => d.iter().map(|(k, _)| k.as_str()).collect(), _ => vec![] };
    assert_eq!(keys, ["schema", "robot_id", "run_id", "profile", "started_at", "ended_at", "ros_distro", "git_sha", "files", "topics", "camera", "camera_height_m"]);
    assert_eq!(man.get("git_sha"), Some(&mower_pyfmt::Py::str("abc123")));
    assert_eq!(man.get("profile"), Some(&mower_pyfmt::Py::str("data_collection")));
    assert_eq!(man.get("camera"), Some(&mower_pyfmt::Py::None));
    assert!(matches!(man.get("started_at"), Some(mower_pyfmt::Py::Str(s)) if s.ends_with("+08:00")));
    // verify finds a missing object
    s3.objects.borrow_mut().remove(&format!("{}/bag/bag_0.mcap", base(R1)));
    let (ok, problems, _) = manifest::verify_remote(&s3, BUCKET, &base(R1)).unwrap();
    assert!(!ok);
    assert_eq!(problems, [format!("缺少 {}/bag/bag_0.mcap", base(R1))]);
    let (ok, problems, m) = manifest::verify_remote(&FakeS3::default(), BUCKET, &base(R1)).unwrap();
    assert!(!ok && m.is_none());
    assert_eq!(problems, ["沒有 _manifest.json（上傳未完成）"]);
    std::fs::remove_dir_all(&tmp).unwrap();
}

#[test]
fn run_key_as_python() {
    assert_eq!(manifest::run_key("bags", "mower-03", "r1"), "bags/mower-03/r1");
    assert_eq!(manifest::run_key("/x/y/", "m", "r"), "x/y/m/r");
    assert_eq!(manifest::run_key("", "m", "r"), "bags/m/r");
}
