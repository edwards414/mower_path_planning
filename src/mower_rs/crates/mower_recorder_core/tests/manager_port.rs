//! `manager` against fakes, held to what recorder_manager did in the E5-0
//! runs (deploy/test/golden/recorder): F1 (the default profile, MediaMTX, a
//! snapshot) and F3 (data_collection) are driven through the same sequence
//! of calls the Python node got (its constructor, every 2 s tick, start,
//! snapshot, stop), with a clock that answers what `datetime.now()`
//! answered, and every output is compared with the recorded one: the
//! replies, each `/mower_recorder/status` (byte for byte, but `bag_bytes`,
//! which is the size of other bags) and rear light, the MediaMTX requests
//! (through `MediaMtx`, against a stub like e2-verify's), `run_metadata.yaml`,
//! `calib/`, `params/*.yaml` and the run directory's layout; then the
//! order of the steps, the disk guard, the refusals and the video retry.

mod bagfile;

use std::collections::{BTreeMap, VecDeque};
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU16, Ordering};
use std::sync::{Arc, Mutex};

use bagfile::*;
use mower_mcap::{Qos, TopicMetadata};
use mower_pyfmt::{json, Py};
use mower_recorder_core::manager::{Clock, Config, DcParams, Disk, Hooks, Manager, MediaMtx, Outputs, ParamSource, Sources};
use mower_recorder_core::params::ParamValue;
use mower_recorder_core::profile::{self, GroupSpec};
use mower_recorder_core::recording::{Feed, Level, QosOverrides};
use mower_recorder_core::run::Fix;
use mower_recorder_core::util::{civil, LocalTime};

const R1: &str = "MW-E5FIX_20261030T081851";
const R3: &str = "MW-E5FIX_20261030T080906";
/// where the fixtures' runs were recorded
const ROBOT_ROOT: &str = "/home/mower/.mower/bags";

// ---- the fakes ----------------------------------------------------------------------------

#[derive(Default)]
struct Trace(Mutex<Vec<String>>);

impl Trace {
    fn push(&self, s: impl Into<String>) {
        self.0.lock().unwrap().push(s.into());
    }
    fn take(&self) -> Vec<String> {
        std::mem::take(&mut *self.0.lock().unwrap())
    }
}

/// How many `metadata.yaml` files (finalized bags) are under `dir`.
fn finalized(dir: &Path) -> usize {
    tree(dir).iter().filter(|p| p.ends_with("/metadata.yaml") && !p.starts_with("params")).count()
}

struct Out {
    trace: Arc<Trace>,
    root: PathBuf,
    status: Mutex<Vec<String>>,
    rear: Mutex<Vec<String>>,
    logs: Mutex<Vec<(Level, String)>>,
}

impl Outputs for Out {
    fn status(&self, j: &str) {
        let what = if j.contains("\"recording\": true") { "recording".to_string() } else { format!("idle (finalized {})", finalized(&self.root)) };
        self.trace.push(format!("status {what}"));
        self.status.lock().unwrap().push(j.to_string());
    }
    fn rear_light(&self, j: &str) {
        self.rear.lock().unwrap().push(j.to_string());
    }
    fn log(&self, level: Level, msg: &str) {
        self.logs.lock().unwrap().push((level, msg.to_string()));
    }
}

impl Out {
    fn logs(&self, level: Level) -> Vec<String> {
        self.logs.lock().unwrap().iter().filter(|(l, _)| *l == level).map(|(_, m)| m.clone()).collect()
    }
}

/// `datetime.now()`: the times the Python got, in order.
struct QueueClock(Arc<Trace>, Arc<Mutex<VecDeque<LocalTime>>>);

impl Clock for QueueClock {
    fn now(&self) -> LocalTime {
        self.0.push("clock");
        self.1.lock().unwrap().pop_front().expect("the clock was asked more often than the Python asked it")
    }
}

/// A clock that moves one second per call.
struct StepClock(Mutex<i64>);

impl Clock for StepClock {
    fn now(&self) -> LocalTime {
        let mut t = self.0.lock().unwrap();
        *t += 1;
        civil(*t, 0)
    }
}

struct FakeDisk(Arc<Trace>, PathBuf, Arc<AtomicBool>);

impl Disk for FakeDisk {
    fn low(&self, path: &Path, _min: i64) -> bool {
        self.0.push(format!("disk {}", if path == self.1 { "root" } else { "run" }));
        self.2.load(Ordering::SeqCst)
    }
}

struct FakeParams(Arc<Trace>, BTreeMap<String, Vec<(String, ParamValue)>>);

impl ParamSource for FakeParams {
    fn parameters(&self, node: &str) -> Result<Vec<(String, ParamValue)>, String> {
        self.0.push(format!("params {node}"));
        self.1.get(node).cloned().ok_or_else(|| "Node not found".to_string())
    }
}

type Script = Box<dyn FnMut(&GroupSpec, &Feed) + Send>;

struct FakeSources {
    trace: Arc<Trace>,
    root: PathBuf,
    script: Script,
}

impl Sources for FakeSources {
    fn start(&mut self, spec: &GroupSpec, overrides: &QosOverrides, feed: Feed) -> Result<(), String> {
        assert_eq!(feed.group(), spec.name);
        self.trace.push(format!("sources.start {} ({} names, {} overrides)", spec.name, spec.recorded.len(), overrides.topics().count()));
        (self.script)(spec, &feed);
        Ok(())
    }
    fn stop(&mut self, group: &str) {
        self.trace.push(format!("sources.stop {group} (finalized {})", finalized(&self.root)));
    }
}

/// Feeds a fixture bag's topics and its first `n` messages.
fn replay_into(feed: &Feed, bag: &Path, n: usize, decompress: bool) {
    let m = read(&originals(bag)[0]);
    for ch in m.channels.values() {
        let t = TopicMetadata {
            name: ch.topic.clone(),
            type_name: m.type_of(ch).to_string(),
            serialization_format: ch.encoding.clone(),
            offered_qos_profiles: Qos::parse_list(ch.meta("offered_qos_profiles")).unwrap(),
            type_description_hash: ch.meta("topic_type_hash").to_string(),
        };
        assert!(feed.topic_appeared(t));
    }
    for msg in m.messages.iter().take(n) {
        let data = if decompress { zstd::decode_all(&msg.data[..]).unwrap() } else { msg.data.clone() };
        assert!(feed.message(m.topic(msg), msg.log_time as i64, msg.publish_time as i64, data));
    }
}

// ---- a MediaMTX stub (e2-verify's mediamtx_stub.py) ----------------------------------------

#[derive(Debug, Clone, PartialEq)]
struct Req {
    method: String,
    path: String,
    content_type: Option<String>,
    body: String,
}

struct Stub {
    url: String,
    log: Arc<Mutex<Vec<Req>>>,
    fail_patch: Arc<AtomicBool>,
    get_status: Arc<AtomicU16>,
}

fn stub(trace: Arc<Trace>) -> Stub {
    let l = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let url = format!("http://{}", l.local_addr().unwrap());
    let log = Arc::new(Mutex::new(Vec::new()));
    let fail_patch = Arc::new(AtomicBool::new(false));
    let get_status = Arc::new(AtomicU16::new(200));
    let (lg, fp, gs) = (log.clone(), fail_patch.clone(), get_status.clone());
    std::thread::spawn(move || {
        for s in l.incoming() {
            let Ok(mut s) = s else { continue };
            let mut got = Vec::new();
            let mut buf = [0u8; 65536];
            let (head, body) = loop {
                let n = s.read(&mut buf).unwrap_or(0);
                got.extend_from_slice(&buf[..n]);
                if let Some(i) = got.windows(4).position(|w| w == b"\r\n\r\n") {
                    let head = String::from_utf8_lossy(&got[..i]).into_owned();
                    let len = head.lines().find_map(|l| l.to_lowercase().strip_prefix("content-length:").map(|v| v.trim().parse::<usize>().unwrap())).unwrap_or(0);
                    if got.len() >= i + 4 + len || n == 0 {
                        break (head, String::from_utf8_lossy(&got[i + 4..(i + 4 + len).min(got.len())]).into_owned());
                    }
                }
                if n == 0 {
                    break (String::from_utf8_lossy(&got).into_owned(), String::new());
                }
            };
            let mut first = head.lines().next().unwrap_or("").split(' ');
            let (method, path) = (first.next().unwrap_or("").to_string(), first.next().unwrap_or("").to_string());
            let content_type = head.lines().find_map(|l| {
                let (k, v) = l.split_once(':')?;
                k.eq_ignore_ascii_case("content-type").then(|| v.trim().to_string())
            });
            // what the run's directory held when the camera was turned on
            let mut note = String::new();
            if body.contains("\"record\": true") {
                let v = json::loads(&body).unwrap();
                let Some(Py::Str(rp)) = v.get("recordPath") else { panic!() };
                let run = rp.split("/video/").next().unwrap();
                note = format!(" (metadata {})", if Path::new(run).join("run_metadata.yaml").is_file() { "written" } else { "missing" });
            }
            trace.push(format!("video {method}{}{note}", if body.contains("true") { " on" } else if method == "PATCH" { " off" } else { "" }));
            lg.lock().unwrap().push(Req { method: method.clone(), path: path.clone(), content_type, body });
            let (code, reply) = if method == "GET" {
                match gs.load(Ordering::SeqCst) {
                    200 => (200, "{\"name\": \"front\", \"ready\": true}"),
                    c => (c, ""),
                }
            } else if fp.load(Ordering::SeqCst) {
                (500, " {\"error\": \"boom\"}\n")
            } else {
                (200, "")
            };
            let _ = write!(s, "HTTP/1.1 {code} X\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{reply}", reply.len());
        }
    });
    Stub { url, log, fail_patch, get_status }
}

// ---- the fixtures ----------------------------------------------------------------------------

fn listen(f: &str, topic: &str) -> Vec<String> {
    let raw = String::from_utf8(unzst(&golden().join(f).join("listen.jsonl.zst"))).unwrap();
    raw.lines()
        .map(|l| serde_json::from_str::<serde_json::Value>(l).unwrap())
        .filter(|v| v["topic"] == topic)
        .map(|v| v["data"].as_str().unwrap().to_string())
        .collect()
}

/// The `Trigger` replies `ros2 service call` printed.
fn replies(f: &str) -> Vec<(bool, String)> {
    let text = std::fs::read_to_string(golden().join(f).join("replies.txt")).unwrap();
    text.lines()
        .filter_map(|l| l.strip_prefix("std_srvs.srv.Trigger_Response(success="))
        .map(|r| {
            let (ok, rest) = r.split_once(", message='").unwrap();
            (ok == "True", rest.strip_suffix("')").unwrap().to_string())
        })
        .collect()
}

/// `datetime.fromisoformat` of a naive isoformat string.
fn parse_iso(s: &str) -> LocalTime {
    let (date, time) = s.split_once('T').unwrap();
    let d: Vec<i64> = date.split('-').map(|x| x.parse().unwrap()).collect();
    let (hms, us) = time.split_once('.').unwrap_or((time, "0"));
    let t: Vec<u32> = hms.split(':').map(|x| x.parse().unwrap()).collect();
    LocalTime { year: d[0] as i32, month: d[1] as u32, day: d[2] as u32, hour: t[0], minute: t[1], second: t[2], microsecond: format!("{us:0<6}").parse().unwrap(), utc_offset: 0 }
}

/// `t + timedelta(seconds=s)` (UTC, as in the fixtures' containers).
fn plus(t: &LocalTime, s: f64) -> LocalTime {
    let (y, m) = (t.year as i64 - if t.month <= 2 { 1 } else { 0 }, t.month as i64);
    let era = y.div_euclid(400);
    let yoe = y.rem_euclid(400);
    let doy = (153 * (if m > 2 { m - 3 } else { m + 9 }) + 2) / 5 + t.day as i64 - 1;
    let days = era * 146_097 + yoe * 365 + yoe / 4 - yoe / 100 + doy - 719_468;
    let us = ((days * 86_400 + t.hour as i64 * 3600 + t.minute as i64 * 60 + t.second as i64) * 1_000_000) + t.microsecond as i64 + (s * 1e6).round() as i64;
    LocalTime { microsecond: us.rem_euclid(1_000_000) as u32, ..civil(us.div_euclid(1_000_000), 0) }
}

fn get_str(v: &Py, k: &str) -> String {
    match v.get(k) {
        Some(Py::Str(s)) => s.clone(),
        other => panic!("{k}: {other:?}"),
    }
}

fn get_f64(v: &Py, k: &str) -> f64 {
    match v.get(k) {
        Some(Py::Float(f)) => *f,
        other => panic!("{k}: {other:?}"),
    }
}

/// The clock's answers for a run: the run id, `_start_time`, the metadata's
/// `start_time`, then one per status while recording (`elapsed_s`).
fn clock_for(statuses: &[String], metadata_start: &str) -> (VecDeque<LocalTime>, usize, usize) {
    let s0 = statuses.iter().position(|j| j.contains("\"recording\": true")).unwrap();
    let e0 = s0 + statuses[s0..].iter().position(|j| j.contains("\"recording\": false")).unwrap();
    let start = parse_iso(&get_str(&json::loads(&statuses[s0]).unwrap(), "start_time"));
    let mut q = VecDeque::from([start, start, parse_iso(metadata_start)]);
    for j in &statuses[s0..e0] {
        q.push_back(plus(&start, get_f64(&json::loads(j).unwrap(), "elapsed_s")));
    }
    (q, s0, e0)
}

/// Our statuses against the recorded ones, `bag_bytes` taken from theirs.
fn assert_statuses(ours: &[String], theirs: &[String]) {
    assert_eq!(ours.len(), theirs.len());
    for (i, (o, t)) in ours.iter().zip(theirs).enumerate() {
        let mut v = json::loads(o).unwrap();
        let tv = json::loads(t).unwrap();
        profile::dict_set(&mut v, "bag_bytes", tv.get("bag_bytes").unwrap().clone());
        assert_eq!(json::dumps(&v, &json::JsonOptions::unicode()).unwrap(), *t, "status #{i}: ours {o}");
    }
}

fn flatten(prefix: &str, v: &Py, out: &mut Vec<(String, ParamValue)>) {
    use ParamValue as P;
    let name = |k: &str| if prefix.is_empty() { k.to_string() } else { format!("{prefix}.{k}") };
    if let Py::Dict(d) = v {
        for (k, x) in d {
            match x {
                Py::Dict(_) => flatten(&name(k), x, out),
                Py::Bool(b) => out.push((name(k), P::Bool(*b))),
                Py::Int(i) => out.push((name(k), P::Integer(*i))),
                Py::Float(f) => out.push((name(k), P::Double(*f))),
                Py::Str(s) => out.push((name(k), P::String(s.clone()))),
                Py::None => out.push((name(k), P::NotSet)),
                Py::List(l) => out.push((
                    name(k),
                    match l.first() {
                        Some(Py::Bool(_)) => P::BoolArray(l.iter().map(|x| matches!(x, Py::Bool(true))).collect()),
                        Some(Py::Int(_)) => P::IntegerArray(l.iter().map(|x| if let Py::Int(i) = x { *i } else { panic!() }).collect()),
                        Some(Py::Str(_)) => P::StringArray(l.iter().map(|x| if let Py::Str(s) = x { s.clone() } else { panic!() }).collect()),
                        _ => P::DoubleArray(l.iter().map(|x| if let Py::Float(f) = x { *f } else { panic!() }).collect()),
                    },
                )),
                other => panic!("{other:?}"),
            }
        }
    }
}

/// The nodes' parameters as the fixture's dumps show them, in reverse
/// order (the dump sorts them).
fn params_of(run_dir: &Path) -> BTreeMap<String, Vec<(String, ParamValue)>> {
    let mut out = BTreeMap::new();
    for e in std::fs::read_dir(run_dir.join("params")).unwrap().flatten() {
        let doc = mower_pyfmt::yaml::safe_load(&std::fs::read_to_string(e.path()).unwrap()).unwrap();
        let Py::Dict(top) = &doc else { panic!() };
        let (node, body) = &top[0];
        let mut ps = Vec::new();
        flatten("", body.get("ros__parameters").unwrap(), &mut ps);
        ps.reverse();
        out.insert(node.clone(), ps);
    }
    out
}

/// The fixture run's files (`.zst` dropped) against ours.
fn assert_same_files(golden_run: &Path, ours: &Path, extra: &[&str]) {
    let mut want: Vec<String> = tree(golden_run).into_iter().map(|p| p.strip_suffix(".zst").map(String::from).unwrap_or(p)).collect();
    want.extend(extra.iter().map(|s| s.to_string()));
    want.sort();
    assert_eq!(tree(ours), want);
}

fn same_file(a: &Path, b: &Path) {
    let (x, y) = (std::fs::read(a).unwrap(), std::fs::read(b).unwrap());
    assert!(x == y, "{} differs from {}:\n{}", b.display(), a.display(), String::from_utf8_lossy(&y));
}

struct Rig {
    trace: Arc<Trace>,
    out: Arc<Out>,
    root: PathBuf,
}

fn rig(tmp: &Path) -> Rig {
    let root = tmp.join("bags");
    std::fs::create_dir_all(&root).unwrap();
    let trace = Arc::new(Trace::default());
    let out = Arc::new(Out { trace: trace.clone(), root: root.clone(), status: Mutex::default(), rear: Mutex::default(), logs: Mutex::default() });
    Rig { trace, out, root }
}

// ---- F1: the default profile ------------------------------------------------------------------

#[test]
fn f1_a_default_run_as_the_python_recorder_made_it() {
    let tmp = tmp("mgr_f1");
    let Rig { trace, out, root } = rig(&tmp);
    let g = golden().join("F1").join(R1);
    let theirs = listen("F1", "/mower_recorder/status");
    let meta_text = std::fs::read_to_string(g.join("run_metadata.yaml")).unwrap();
    let meta = mower_pyfmt::yaml::safe_load(&meta_text).unwrap();
    let (times, s0, e0) = clock_for(&theirs, &get_str(&meta, "start_time"));
    let stub = stub(trace.clone());
    let low = Arc::new(AtomicBool::new(false));

    let mut cfg = Config::new();
    cfg.robot_id = "MW-E5FIX".into();
    cfg.output_root = root.clone();
    cfg.config_file = config_dir().join("record_topics.yaml").to_string_lossy().into_owned();
    cfg.qos_overrides_path = config_dir().join("qos_overrides.yaml").to_string_lossy().into_owned();
    cfg.params_dump_nodes = vec!["/boustrophedon_coverage".into(), "/map_manage".into()];
    cfg.library = "libmcap 1.3.1".into();
    let f1 = golden().join("F1").join(R1);
    let script: Script = Box::new(move |spec, feed| match spec.name {
        "full" => replay_into(feed, &f1.join("bag"), 300, true),
        "snapshot" => replay_into(feed, &f1.join("snapshots"), usize::MAX, false),
        other => panic!("{other}"),
    });
    let hooks = Hooks {
        clock: Box::new(QueueClock(trace.clone(), Arc::new(Mutex::new(times)))),
        disk: Box::new(FakeDisk(trace.clone(), root.clone(), low.clone())),
        video: Some(Box::new(MediaMtx { api_url: stub.url.clone(), path: "front".into() })),
        params: Arc::new(FakeParams(trace.clone(), params_of(&g))),
        sources: Box::new(FakeSources { trace: trace.clone(), root: root.clone(), script }),
        out: out.clone(),
    };

    // the node starts: MediaMTX off, the first status
    let mut m = Manager::new(cfg, hooks);
    assert_eq!(trace.take(), ["video PATCH off", "status idle (finalized 0)"]);
    for _ in 1..s0 {
        m.tick();
    }
    let gps = meta.get("gps_start").unwrap();
    m.on_fix(Fix { latitude: get_f64(gps, "lat"), longitude: get_f64(gps, "lon"), altitude: get_f64(gps, "alt") });
    trace.take();

    // start, in the Python's order
    let mut got = vec![m.start()];
    m.join_param_dumps();
    assert_eq!(
        trace.take(),
        [
            "disk root",
            "clock",
            "clock",
            "clock",
            "video PATCH on (metadata written)",
            "video GET",
            "sources.start full (30 names, 9 overrides)",
            "sources.start snapshot (3 names, 9 overrides)",
            "disk run",
            "clock",
            "status recording",
            "params /boustrophedon_coverage",
            "params /map_manage",
        ]
    );
    assert_eq!(m.run_dir(), Some(root.join(R1).as_path()));
    // the ticks; the snapshot at +40 s
    let snap_at = (s0 + 1..e0).find(|&i| get_f64(&json::loads(&theirs[i]).unwrap(), "elapsed_s") >= 40.0).unwrap();
    for i in s0 + 1..e0 {
        if i == snap_at {
            got.push(m.snapshot());
        }
        m.tick();
    }
    trace.take();
    // stop: video off, every group's sources stopped before any is finalized
    got.push(m.stop());
    assert_eq!(trace.take(), ["video PATCH off", "sources.stop full (finalized 0)", "sources.stop snapshot (finalized 0)", "status idle (finalized 2)"]);
    for _ in e0 + 1..theirs.len() {
        m.tick();
    }

    assert_eq!(got, replies("F1"));
    assert_statuses(&out.status.lock().unwrap(), &theirs);
    assert_eq!(*out.rear.lock().unwrap(), listen("F1", "/mower_base/rear_light"));
    // MediaMTX: the same requests, the bodies byte for byte (the run under
    // our root instead of the robot's)
    let want: Vec<Req> = String::from_utf8(unzst(&golden().join("F1/mediamtx.jsonl.zst")))
        .unwrap()
        .lines()
        .map(|l| {
            let v: serde_json::Value = serde_json::from_str(l).unwrap();
            Req { method: v["method"].as_str().unwrap().into(), path: v["path"].as_str().unwrap().into(), content_type: v["content_type"].as_str().map(String::from), body: v["body"].as_str().unwrap().into() }
        })
        .collect();
    let ours: Vec<Req> = stub.log.lock().unwrap().iter().map(|r| Req { body: r.body.replace(&root.to_string_lossy().into_owned(), ROBOT_ROOT), ..r.clone() }).collect();
    assert_eq!(ours, want);
    // the run's files
    let run = root.join(R1);
    same_file(&g.join("run_metadata.yaml"), &run.join("run_metadata.yaml"));
    for n in ["boustrophedon_coverage", "map_manage"] {
        same_file(&g.join(format!("params/{n}.yaml")), &run.join(format!("params/{n}.yaml")));
    }
    assert_same_files(&g, &run, &["video/"]);
    // the snapshot wrote the buffer and started snapshots_1
    let snap = read(&std::fs::read(run.join("snapshots/snapshots_0.mcap")).unwrap());
    assert_eq!(snap.messages.len(), read(&originals(&g.join("snapshots"))[0]).messages.len());
    assert!(out.logs(Level::Warn).is_empty() && out.logs(Level::Error).is_empty(), "{:?}", out.logs.lock().unwrap());
    assert_eq!(
        out.logs(Level::Info),
        [
            "video off: front".to_string(),
            format!("metadata -> {}", run.join("run_metadata.yaml").display()),
            "video on: front".to_string(),
            format!("full record -> {} (32 topics)", run.join("bag").display()),
            format!("snapshot record -> {} (3 topics)", run.join("snapshots").display()),
            "video off: front".to_string(),
        ]
    );
    std::fs::remove_dir_all(&tmp).unwrap();
}

// ---- F3: data_collection ------------------------------------------------------------------------

#[test]
fn f3_a_data_collection_run_as_the_python_recorder_made_it() {
    let tmp = tmp("mgr_f3");
    let Rig { trace, out, root } = rig(&tmp);
    let g = golden().join("F3").join(R3);
    let theirs = listen("F3", "/mower_recorder/status");
    let meta_text = std::fs::read_to_string(g.join("run_metadata.yaml")).unwrap();
    let meta = mower_pyfmt::yaml::safe_load(&meta_text).unwrap();
    let (times, s0, e0) = clock_for(&theirs, &get_str(&meta, "start_time"));

    // what data_collection.launch.py passed: the camera launch's JSON, the
    // calibration files (the run's own copies, under their original names)
    let src = tmp.join("calib_src");
    std::fs::create_dir_all(&src).unwrap();
    std::fs::copy(g.join("calib/camera_front.yaml"), src.join("camera_front.yaml")).unwrap();
    std::fs::copy(g.join("calib/extrinsics.yaml"), src.join("extrinsics.yaml")).unwrap();
    let ext = mower_pyfmt::yaml::safe_load(&std::fs::read_to_string(g.join("calib/extrinsics.yaml")).unwrap()).unwrap();
    let cam = meta.get("camera").unwrap();
    let camera_params = Py::dict(
        ["device", "width", "height", "fps_native", "fps_recorded", "format", "autofocus", "driver", "pipeline"].map(|k| (k, cam.get(k).unwrap().clone())),
    );
    let derived = Py::dict([
        ("summary", Py::dict([("source", meta.get("extrinsics_source").unwrap().clone()), ("camera_height_m", meta.get("camera_height_m").unwrap().clone()), ("camera_pitch_deg", meta.get("camera_pitch_deg").unwrap().clone())])),
        ("block", ext.get("derived").unwrap().clone()),
    ]);
    let dumps = |v: &Py| json::dumps(v, &json::JsonOptions::default()).unwrap();
    let mut cfg = Config::new();
    cfg.robot_id = "MW-E5FIX".into();
    cfg.output_root = root.clone();
    cfg.config_file = config_dir().join("data_collection.yaml").to_string_lossy().into_owned();
    cfg.qos_overrides_path = config_dir().join("qos_overrides.yaml").to_string_lossy().into_owned();
    cfg.params_dump_nodes = vec!["/ekf_filter_node_odom".into(), "/ekf_filter_node_map".into(), "/navsat_transform".into()];
    cfg.data_collection = DcParams {
        session_file: golden().join("F3/session.yaml").to_string_lossy().into_owned(),
        run_info: String::new(),
        camera_params: dumps(&camera_params),
        intrinsics_file: src.join("camera_front.yaml").to_string_lossy().into_owned(),
        intrinsics_placeholder: false,
        extrinsics_file: src.join("extrinsics.yaml").to_string_lossy().into_owned(),
        extrinsics_derived: dumps(&derived),
    };
    let build = meta.get("build").unwrap().clone();
    cfg.env = Arc::new(move |k| match k {
        "MOWER_VERSION" => Some(get_str(&build, "version")),
        "MOWER_GIT_SHA" => Some(get_str(&build, "git_sha")),
        "MOWER_IMAGE" => Some(get_str(&build, "image")),
        _ => None,
    });
    let g3 = g.clone();
    let script: Script = Box::new(move |spec, feed| match spec.name {
        "full" => replay_into(feed, &g3.join("bag"), 300, false),
        "camera" => replay_into(feed, &g3.join("camera"), 50, false),
        other => panic!("{other}"),
    });
    let hooks = Hooks {
        clock: Box::new(QueueClock(trace.clone(), Arc::new(Mutex::new(times)))),
        disk: Box::new(FakeDisk(trace.clone(), root.clone(), Arc::new(AtomicBool::new(false)))),
        video: None,
        params: Arc::new(FakeParams(trace.clone(), params_of(&g))),
        sources: Box::new(FakeSources { trace: trace.clone(), root: root.clone(), script }),
        out: out.clone(),
    };
    let mut m = Manager::new(cfg, hooks);
    for _ in 1..s0 {
        m.tick();
    }
    let gps = meta.get("gps_start").unwrap();
    m.on_fix(Fix { latitude: get_f64(gps, "lat"), longitude: get_f64(gps, "lon"), altitude: get_f64(gps, "alt") });
    let mut got = vec![m.start()];
    m.join_param_dumps();
    for _ in s0 + 1..e0 {
        m.tick();
    }
    trace.take();
    got.push(m.stop());
    assert_eq!(trace.take(), ["sources.stop full (finalized 0)", "sources.stop camera (finalized 0)", "status idle (finalized 2)"]);
    for _ in e0 + 1..theirs.len() {
        m.tick();
    }

    assert_eq!(got, replies("F3"));
    assert_statuses(&out.status.lock().unwrap(), &theirs);
    // the listener lost one rear light, the one sent with the stop's status
    // (777.476 s: a status and no rear light; every other status has one)
    let mut ours = out.rear.lock().unwrap().clone();
    assert_eq!(ours.remove(e0), r#"{"effect": "off", "source": "bag"}"#);
    assert_eq!(ours, listen("F3", "/mower_base/rear_light"));
    let run = root.join(R3);
    for f in ["run_metadata.yaml", "calib/camera_front.yaml", "calib/extrinsics.yaml", "params/ekf_filter_node_odom.yaml", "params/ekf_filter_node_map.yaml", "params/navsat_transform.yaml"] {
        same_file(&g.join(f), &run.join(f));
    }
    assert_same_files(&g, &run, &[]);
    assert!(out.logs(Level::Warn).is_empty() && out.logs(Level::Error).is_empty(), "{:?}", out.logs.lock().unwrap());
    std::fs::remove_dir_all(&tmp).unwrap();
}

// ---- the guards, the refusals, the video retry ----------------------------------------------

struct Synthetic {
    m: Manager,
    out: Arc<Out>,
    stub: Stub,
    low: Arc<AtomicBool>,
    root: PathBuf,
    trace: Arc<Trace>,
}

fn synthetic(tmp: &Path, config_file: &str) -> Synthetic {
    let Rig { trace, out, root } = rig(tmp);
    let stub = stub(trace.clone());
    let low = Arc::new(AtomicBool::new(false));
    let mut cfg = Config::new();
    cfg.robot_id = "MW-T".into();
    cfg.output_root = root.clone();
    cfg.config_file = config_file.to_string();
    cfg.qos_overrides_path = config_dir().join("qos_overrides.yaml").to_string_lossy().into_owned();
    cfg.params_dump_nodes = vec!["".into(), "/gone".into()];
    let hooks = Hooks {
        clock: Box::new(StepClock(Mutex::new(1_793_348_331))),
        disk: Box::new(FakeDisk(trace.clone(), root.clone(), low.clone())),
        video: Some(Box::new(MediaMtx { api_url: stub.url.clone(), path: "front".into() })),
        params: Arc::new(FakeParams(trace.clone(), BTreeMap::new())),
        sources: Box::new(FakeSources {
            trace: trace.clone(),
            root: root.clone(),
            script: Box::new(|spec, feed| {
                if spec.name == "snapshot" {
                    let t = mower_recorder_core::recording::topic_metadata("/plan", "nav_msgs/msg/Path", &[Qos::new("keep_last", 1, "reliable", "volatile")], "");
                    assert!(feed.topic_appeared(t));
                    assert!(feed.message("/plan", 1, 1, vec![0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]));
                }
            }),
        }),
        out: out.clone(),
    };
    let m = Manager::new(cfg, hooks);
    Synthetic { m, out, stub, low, root, trace }
}

fn idle(stop_reason: &str) -> String {
    format!(r#"{{"recording": false, "run_id": null, "start_time": null, "elapsed_s": 0.0, "bag_bytes": 0, "num_topics": 32, "profile": "default", "video": false, "stop_reason": {stop_reason}}}"#)
}

fn last(v: &Mutex<Vec<String>>) -> String {
    v.lock().unwrap().last().cloned().unwrap()
}

#[test]
fn the_disk_guard_the_refusals_and_the_fault() {
    let tmp = tmp("mgr_guard");
    let topics = config_dir().join("record_topics.yaml").to_string_lossy().into_owned();
    let Synthetic { mut m, out, stub, low, root, trace } = synthetic(&tmp, &topics);
    assert_eq!(last(&out.status), idle("null"));
    assert_eq!(m.snapshot(), (false, "snapshot 觸發失敗".into()));
    assert_eq!(out.logs(Level::Warn), ["no snapshot recorder running"]);
    assert_eq!(m.stop(), (false, "目前沒有在錄製".into()));
    low.store(true, Ordering::SeqCst);
    assert_eq!(m.start(), (false, "磁碟剩不到 2048 MB，不能開始錄製".into()));
    low.store(false, Ordering::SeqCst);

    let (ok, msg) = m.start();
    let id = m.run_id().unwrap().to_string();
    assert_eq!((ok, msg), (true, format!("開始錄製 run={id}，含前鏡頭影像")));
    assert_eq!(id, "MW-T_20261030T081852");
    m.join_param_dumps();
    assert_eq!(m.start(), (false, "已在錄製中".into()));
    m.on_fault(false);
    m.on_fault(true);
    assert!(root.join(&id).join("snapshots/snapshots_1.mcap").is_file(), "the fault flushed the snapshot buffer");
    assert_eq!(out.logs(Level::Warn)[1..], ["param dump /gone failed: Node not found".to_string(), "fault received -> flushing snapshot buffer".to_string()]);
    m.tick();
    assert!(last(&out.status).starts_with(&format!(r#"{{"recording": true, "run_id": "{id}", "start_time": "2026-10-30T08:18:53", "elapsed_s": "#)), "{}", last(&out.status));
    assert_eq!(last(&out.rear), r#"{"effect": "recording", "source": "bag"}"#);

    // the disk gets low while recording: the run stops itself
    low.store(true, Ordering::SeqCst);
    trace.take();
    m.tick();
    assert_eq!(trace.take(), ["disk run", "video PATCH off", "sources.stop full (finalized 0)", "sources.stop snapshot (finalized 0)", "status idle (finalized 2)"]);
    assert_eq!(out.logs(Level::Error), [format!("under 2048 MB free: stopping run {id}")]);
    assert_eq!(last(&out.status), idle(r#""disk_low""#));
    assert_eq!(last(&out.rear), r#"{"effect": "off", "source": "bag"}"#);
    m.tick();
    assert_eq!(last(&out.status), idle(r#""disk_low""#), "the reason stays until the next start");
    low.store(false, Ordering::SeqCst);
    assert!(m.start().0);
    assert!(last(&out.status).ends_with(r#""video": true, "stop_reason": null}"#));
    let (ok, msg) = m.stop();
    assert_eq!((ok, msg), (true, format!("已停止並封存: full, snapshot (run={})", m.run_id().unwrap())));
    assert_eq!(stub.log.lock().unwrap().iter().filter(|r| r.method == "PATCH").count(), 5);

    // a profile without topics
    let tmp2 = tmp.join("none");
    let s = synthetic(&tmp2, "");
    let mut m2 = s.m;
    assert_eq!(s.out.logs(Level::Warn), ["config_file not found: ''"]);
    assert_eq!(m2.start(), (false, "record_topics.yaml 沒有指定 topics".into()));
    m2.autostart();
    assert_eq!(s.out.logs(Level::Error), ["autostart: record_topics.yaml 沒有指定 topics"]);
    std::fs::remove_dir_all(&tmp).unwrap();
}

#[test]
fn the_video_notes_and_the_off_retry() {
    let tmp = tmp("mgr_video");
    let topics = config_dir().join("record_topics.yaml").to_string_lossy().into_owned();
    let Synthetic { mut m, out, stub, .. } = synthetic(&tmp, &topics);
    let patches = || stub.log.lock().unwrap().iter().filter(|r| r.method == "PATCH").count();
    // the camera has no stream
    stub.get_status.store(404, Ordering::SeqCst);
    let (_, msg) = m.start();
    assert_eq!(msg, format!("開始錄製 run={}（前鏡頭目前沒有畫面，有畫面時才會錄進去）", m.run_id().unwrap()));
    m.stop();
    // MediaMTX refuses
    stub.get_status.store(200, Ordering::SeqCst);
    stub.fail_patch.store(true, Ordering::SeqCst);
    let (_, msg) = m.start();
    assert_eq!(msg, format!("開始錄製 run={}（前鏡頭影像沒有錄：HTTP 500 {{\"error\": \"boom\"}}）", m.run_id().unwrap()));
    assert!(last(&out.status).ends_with(r#""video": false, "stop_reason": null}"#));
    let n = patches();
    m.stop();
    assert_eq!(patches(), n, "nothing to turn off");
    // the off fails: retried every tick, said once
    stub.fail_patch.store(false, Ordering::SeqCst);
    m.start();
    stub.fail_patch.store(true, Ordering::SeqCst);
    let n = patches();
    m.stop();
    m.tick();
    m.tick();
    assert_eq!(patches(), n + 4, "the stop, its status and two ticks");
    stub.fail_patch.store(false, Ordering::SeqCst);
    m.tick();
    m.tick();
    assert_eq!(patches(), n + 5);
    let warns = out.logs(Level::Warn);
    assert_eq!(warns.iter().filter(|w| w.starts_with("video off failed (front): HTTP 500")).count(), 1, "{warns:?}");
    assert_eq!(warns.iter().filter(|w| w.starts_with("video on failed (front): HTTP 500")).count(), 1, "{warns:?}");
    assert_eq!(out.logs(Level::Info).last().unwrap(), "video off: front");
    std::fs::remove_dir_all(&tmp).unwrap();
}

/// R5: a manager dropped mid-run (the module panics or goes away) leaves
/// finished bags.
#[test]
fn a_dropped_manager_finalizes_its_bags() {
    let tmp = tmp("mgr_drop");
    let topics = config_dir().join("record_topics.yaml").to_string_lossy().into_owned();
    let s = synthetic(&tmp, &topics);
    let mut m = s.m;
    assert!(m.start().0);
    let run = m.run_dir().unwrap().to_path_buf();
    assert_eq!(finalized(&run), 0);
    let r = std::thread::spawn(move || {
        let _m = m;
        panic!("a bug in the module");
    })
    .join();
    assert!(r.is_err());
    assert_eq!(finalized(&run), 2);
    let snap = std::fs::read_to_string(run.join("snapshots/metadata.yaml")).unwrap();
    assert!(snap.contains("message_count: 1"), "{snap}");
    std::fs::remove_dir_all(&tmp).unwrap();
}
