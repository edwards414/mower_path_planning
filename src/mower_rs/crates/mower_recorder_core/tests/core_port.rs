//! The core against the Python recorder: test_record_profile.py's command
//! lines, test_video_record.py's MediaMTX calls, and goldens from the E5-0
//! runs (deploy/test/golden/recorder): `run_metadata.yaml`, `params/*.yaml`,
//! and the recorder's own messages (`listen.jsonl`: status, bags, rear
//! light, graph snapshots and events) rebuilt from their parts, byte for byte.

use std::collections::BTreeSet;
use std::io::{Read, Write};
use std::path::{Path, PathBuf};

use mower_pyfmt::{json, Py};
use mower_recorder_core::{graph, mediamtx, params, profile, run, store, util::LocalTime};

fn golden() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../../deploy/test/golden/recorder")
}

fn config_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../mower_recorder/config")
}

const R1: &str = "MW-E5FIX_20261030T081851";
const R3: &str = "MW-E5FIX_20261030T080906";

/// Every message of a run's listen.jsonl: (topic, data).
fn listen(f: &str) -> Vec<(String, String)> {
    let raw = zstd::decode_all(std::fs::File::open(golden().join(f).join("listen.jsonl.zst")).unwrap()).unwrap();
    String::from_utf8(raw)
        .unwrap()
        .lines()
        .map(|l| {
            let v: serde_json::Value = serde_json::from_str(l).unwrap();
            (v["topic"].as_str().unwrap().to_string(), v["data"].as_str().unwrap().to_string())
        })
        .collect()
}

/// `datetime.fromisoformat` of a naive isoformat string.
fn parse_iso(s: &str) -> LocalTime {
    let (date, time) = s.split_once('T').unwrap();
    let d: Vec<i64> = date.split('-').map(|x| x.parse().unwrap()).collect();
    let (hms, us) = time.split_once('.').unwrap_or((time, "0"));
    let t: Vec<u32> = hms.split(':').map(|x| x.parse().unwrap()).collect();
    LocalTime {
        year: d[0] as i32,
        month: d[1] as u32,
        day: d[2] as u32,
        hour: t[0],
        minute: t[1],
        second: t[2],
        microsecond: format!("{us:0<6}").parse().unwrap(),
        utc_offset: 0,
    }
}

// ---- profiles (test_record_profile.py) -----------------------------------------------

/// test_record_profile's `_legacy_commands`: the command lines before the
/// profile refactor.
fn legacy_commands(cfg: &Py, run_dir: &str, qos: &str) -> Vec<Vec<String>> {
    let i = |k: &str| profile::py_int(cfg.get(k)).unwrap();
    let mut cmd: Vec<String> = ["ros2", "bag", "record", "-s", "mcap", "-o"].map(String::from).to_vec();
    cmd.extend([format!("{run_dir}/bag"), "--node-name".into(), "mower_full_recorder".into()]);
    if profile::truthy(cfg.get("compression")) {
        cmd.extend(["--compression-mode", "message", "--compression-format", "zstd"].map(String::from));
    }
    cmd.extend(["--max-bag-size".into(), (i("max_bag_size_mb") * 1024 * 1024).to_string()]);
    cmd.extend(["--max-bag-duration".into(), i("max_bag_duration_s").to_string()]);
    cmd.extend(["--qos-profile-overrides-path".into(), qos.into()]);
    cmd.extend(profile::str_list(cfg.get("topics")));
    let mut snap: Vec<String> = ["ros2", "bag", "record", "-s", "mcap", "-o"].map(String::from).to_vec();
    snap.extend([format!("{run_dir}/snapshots"), "--node-name".into(), "mower_snapshot_recorder".into(), "--snapshot-mode".into()]);
    snap.extend(["--max-cache-size".into(), (i("snapshot_max_cache_mb") * 1024 * 1024).to_string()]);
    snap.extend(["--qos-profile-overrides-path".into(), qos.into()]);
    snap.extend(profile::str_list(cfg.get("snapshot_topics")));
    vec![cmd, snap]
}

#[test]
fn the_always_on_profile_is_unchanged() {
    let qos = config_dir().join("qos_overrides.yaml");
    let mut log = Vec::new();
    let cfg = profile::load_config(Some(&config_dir().join("record_topics.yaml")), &mut log);
    assert!(log.is_empty(), "{log:?}");
    let specs = profile::recorder_specs(&cfg, Path::new("/runs/r1"), Some(&qos), "test").unwrap();
    let cmds: Vec<Vec<String>> = specs.iter().map(|s| s.cmd.clone()).collect();
    assert_eq!(cmds, legacy_commands(&cfg, "/runs/r1", &qos.to_string_lossy()));
    assert_eq!(specs.iter().map(|s| s.name).collect::<Vec<_>>(), ["full", "snapshot"]);
    assert!(cfg.get("profile").is_none());
    // what the Rust writer does with it: per-message zstd, the splits,
    // the 256 MiB snapshot ring; the two hidden names not recorded
    let full = specs[0].bag.as_ref().unwrap();
    assert!(full.message_compression);
    assert_eq!((full.max_bagfile_size, full.max_bagfile_duration), (512 << 20, 3600));
    assert_eq!(specs[0].topics.len(), 32);
    assert_eq!(specs[0].recorded.len(), 30);
    assert!(!specs[0].recorded.iter().any(|t| t.contains("/_action/")));
    assert_eq!(specs[1].bag.as_ref().unwrap().snapshot_cache_size, Some(256 << 20));
    assert_eq!(run::num_topics(&cfg), 32);
}

#[test]
fn the_data_collection_profile_commands() {
    let qos = config_dir().join("qos_overrides.yaml");
    let cfg = profile::load_config(Some(&config_dir().join("data_collection.yaml")), &mut Vec::new());
    let specs = profile::recorder_specs(&cfg, Path::new("/runs/r2"), Some(&qos), "test").unwrap();
    assert_eq!(specs.iter().map(|s| s.name).collect::<Vec<_>>(), ["full", "camera"]);
    let at = |c: &Vec<String>, f: &str| c[c.iter().position(|x| x == f).unwrap() + 1].clone();
    let tele = &specs[0].cmd;
    assert_eq!(at(tele, "--storage-preset-profile"), "zstd_fast");
    assert!(!tele.contains(&"--compression-mode".to_string()));
    assert_eq!(at(tele, "-o"), "/runs/r2/bag");
    assert_eq!(at(tele, "--max-bag-size"), (512u64 << 20).to_string());
    for t in ["/gps/status", "/odometry/local", "/odometry/global", "/odometry/gps", "/tf", "/tf_static", "/imu/data", "/odom", "/fix", "/cmd_vel", "/rosout"] {
        assert!(tele.contains(&t.to_string()), "{t}");
    }
    let cam = &specs[1].cmd;
    assert_eq!(at(cam, "-o"), "/runs/r2/camera");
    assert_eq!(at(cam, "--node-name"), "mower_camera_recorder");
    assert!(!cam.contains(&"--storage-preset-profile".to_string()) && !cam.contains(&"--compression-mode".to_string()));
    assert_eq!(at(cam, "--max-bag-size"), "268435456");
    assert!(!cam.contains(&"--max-bag-duration".to_string()));
    assert_eq!(cam[cam.len() - 3..], ["/camera/front/image_raw/compressed", "/camera/front/camera_info", "/tf_static"]);
    assert_eq!(at(cam, "--qos-profile-overrides-path"), qos.to_string_lossy());
    assert_eq!(specs[0].bag.as_ref().unwrap().preset, mower_mcap::Preset::ZstdFast);
    let c = specs[1].bag.as_ref().unwrap();
    assert_eq!((c.preset, c.message_compression, c.max_bagfile_size, c.max_bagfile_duration), (mower_mcap::Preset::None, false, 256 << 20, 0));
    assert_eq!(run::num_topics(&cfg), 19);
}

#[test]
fn a_storage_preset_wins_over_message_compression_and_parse_mapping() {
    let mut cfg = profile::load_config(None, &mut Vec::new());
    profile::dict_update(&mut cfg, &Py::dict([("topics", Py::List(vec![Py::str("/a")])), ("compression", Py::Bool(true)), ("storage_preset_profile", Py::str("zstd_fast"))]));
    let cmd = &profile::recorder_specs(&cfg, Path::new("/r"), None, "t").unwrap()[0].cmd;
    assert!(!cmd.contains(&"--compression-mode".to_string()) && cmd.contains(&"--storage-preset-profile".to_string()));
    assert_eq!(profile::parse_mapping("", "v").unwrap(), Py::Dict(vec![]));
    assert_eq!(profile::parse_mapping("{weather: rain, n: 2}", "v").unwrap(), Py::dict([("weather", Py::str("rain")), ("n", Py::Int(2))]));
    assert_eq!(profile::parse_mapping("[1, 2]", "run_info").unwrap_err(), "run_info must be a mapping, got list");
    assert_eq!(profile::json_param("[1]", "camera_params").unwrap_err(), "camera_params must be a JSON object");
    let mut log = Vec::new();
    profile::load_config(Some(Path::new("/nonexistent/x.yaml")), &mut log);
    assert_eq!(log, [profile::ConfigLog::Warn("config_file not found: '/nonexistent/x.yaml'".into())]);
}

// ---- the run (recorder_manager) ---------------------------------------------------------

#[test]
fn run_metadata_is_rebuilt_byte_for_byte() {
    let text = std::fs::read_to_string(golden().join("F1").join(R1).join("run_metadata.yaml")).unwrap();
    let meta = mower_pyfmt::yaml::safe_load(&text).unwrap();
    let s = |k: &str| match meta.get(k) { Some(Py::Str(s)) => s.clone(), other => panic!("{k}: {other:?}") };
    let gps = meta.get("gps_start").and_then(|g| match (g.get("lat"), g.get("lon"), g.get("alt")) {
        (Some(Py::Float(a)), Some(Py::Float(b)), Some(Py::Float(c))) => Some(run::Fix { latitude: *a, longitude: *b, altitude: *c }),
        _ => None,
    });
    let cfg = profile::load_config(Some(&config_dir().join("record_topics.yaml")), &mut Vec::new());
    let ours = run::base_metadata(&s("run_id"), &s("robot_id"), &parse_iso(&s("start_time")), &s("ros_distro"), run::unknown_git(), gps, &cfg);
    assert_eq!(run::metadata_yaml(&ours), text);
    assert_eq!(run::run_id("MW-E5FIX", &parse_iso("2026-10-30T08:18:51.000002")), R1);
}

#[test]
fn status_and_rear_light_as_the_recorder_sent_them() {
    let cfg = profile::load_config(Some(&config_dir().join("record_topics.yaml")), &mut Vec::new());
    let mut n = 0;
    for (topic, data) in listen("F1").into_iter().chain(listen("F2")) {
        match topic.as_str() {
            "/mower_recorder/status" => {
                let v = json::loads(&data).unwrap();
                let st = run::Status {
                    recording: v.get("recording") == Some(&Py::Bool(true)),
                    run_id: match v.get("run_id") { Some(Py::Str(s)) => Some(s.clone()), _ => None },
                    start_time: match v.get("start_time") { Some(Py::Str(s)) => Some(parse_iso(s)), _ => None },
                    elapsed_s: match v.get("elapsed_s") { Some(Py::Float(f)) => *f, _ => panic!() },
                    bag_bytes: match v.get("bag_bytes") { Some(Py::Int(i)) => *i as u64, _ => panic!() },
                    num_topics: run::num_topics(&cfg),
                    profile: run::profile_name(&cfg),
                    video: v.get("video") == Some(&Py::Bool(true)),
                    stop_reason: None,
                };
                assert_eq!(st.to_json(), data);
                n += 1;
            }
            "/mower_base/rear_light" => {
                assert!(data == run::rear_light_json(true) || data == run::rear_light_json(false), "{data}");
                n += 1;
            }
            _ => {}
        }
    }
    assert!(n > 50, "{n}");
    assert_eq!(run::started_message(R1, &run::VideoNote::Recording), format!("開始錄製 run={R1}，含前鏡頭影像"));
    assert_eq!(run::stop_message(&["full", "snapshot"], R1), (true, format!("已停止並封存: full, snapshot (run={R1})")));
    assert_eq!(run::params_file_name("/boustrophedon_coverage"), "boustrophedon_coverage.yaml");
    assert_eq!(run::params_file_name("/a/b"), "a__b.yaml");
    // the replies the fixtures logged
    let replies = std::fs::read_to_string(golden().join("F1/replies.txt")).unwrap();
    assert!(replies.contains(&run::started_message(R1, &run::VideoNote::Recording)));
    assert!(replies.contains(run::snapshot_message(true)));
    assert!(replies.contains(&run::stop_message(&["full", "snapshot"], R1).1));
}

#[test]
fn the_graph_messages_as_the_recorder_sent_them() {
    let (mut snaps, mut events) = (0, 0);
    for (topic, data) in listen("F3") {
        let v = json::loads(&data).unwrap();
        let strs = |p: &Py| match p { Py::List(l) => l.iter().map(|x| match x { Py::Str(s) => s.clone(), _ => panic!() }).collect::<Vec<_>>(), _ => panic!() };
        match topic.as_str() {
            "/graph_snapshot" => {
                let stamp = v.get("stamp").unwrap();
                let int = |p: Option<&Py>| match p { Some(Py::Int(i)) => *i, _ => panic!() };
                let nodes: BTreeSet<String> = strs(v.get("nodes").unwrap()).into_iter().collect();
                let topics: Vec<graph::Topic> = match v.get("topics").unwrap() {
                    Py::List(l) => l.iter().map(|t| graph::Topic {
                        name: match t.get("name") { Some(Py::Str(s)) => s.clone(), _ => panic!() },
                        types: strs(t.get("types").unwrap()),
                        pubs: int(t.get("pubs")),
                        subs: int(t.get("subs")),
                    }).collect(),
                    _ => panic!(),
                };
                let services: Vec<graph::Service> = match v.get("services").unwrap() {
                    Py::List(l) => l.iter().map(|s| graph::Service { name: match s.get("name") { Some(Py::Str(x)) => x.clone(), _ => panic!() }, types: strs(s.get("types").unwrap()) }).collect(),
                    _ => panic!(),
                };
                assert_eq!(graph::snapshot_json(int(stamp.get("sec")) as i32, int(stamp.get("nanosec")) as u32, &nodes, &topics, &services), data);
                snaps += 1;
            }
            "/graph_events" => {
                let s = |k: &str| match v.get(k) { Some(Py::Str(s)) => s.clone(), _ => panic!() };
                assert_eq!(graph::event_json(&s("event"), &s("target")), data);
                events += 1;
            }
            _ => {}
        }
    }
    assert!(snaps > 5 && events > 0, "{snaps} {events}");
    // the diff and the watch rule
    let mut w = graph::Watch::new(&["map_manage".into(), "".into()]);
    let set = |v: &[&str]| v.iter().map(|s| s.to_string()).collect::<BTreeSet<_>>();
    assert!(w.tick(&set(&["/a", "/map_manage"])).is_empty());
    assert_eq!(w.tick(&set(&["/b"])), [graph::Event::Disappeared("/a".into()), graph::Event::Disappeared("/map_manage".into()), graph::Event::Appeared("/b".into())]);
    let mut w = graph::Watch::new(&["/x".into()]);
    w.tick(&set(&["/x"]));
    assert_eq!(w.tick(&set(&["/y"])), [graph::Event::Disappeared("/x".into()), graph::Event::Fault("/x".into()), graph::Event::Appeared("/y".into())]);
    assert_eq!(graph::full_name("n", "/"), "/n");
    assert_eq!(graph::full_name("n", "/ns/"), "/ns/n");
}

// ---- parameter dumps ------------------------------------------------------------------------

fn flatten(prefix: &str, v: &Py, out: &mut Vec<(String, params::ParamValue)>) {
    use params::ParamValue as P;
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
                Py::List(l) => out.push((name(k), match l.first() {
                    Some(Py::Bool(_)) => P::BoolArray(l.iter().map(|x| matches!(x, Py::Bool(true))).collect()),
                    Some(Py::Int(_)) => P::IntegerArray(l.iter().map(|x| if let Py::Int(i) = x { *i } else { panic!() }).collect()),
                    Some(Py::Str(_)) => P::StringArray(l.iter().map(|x| if let Py::Str(s) = x { s.clone() } else { panic!() }).collect()),
                    _ => P::DoubleArray(l.iter().map(|x| if let Py::Float(f) = x { *f } else { panic!() }).collect()),
                })),
                other => panic!("{other:?}"),
            }
        }
    }
}

#[test]
fn parameter_dumps_as_ros2_param_dump_printed_them() {
    let mut n = 0;
    for (f, run_id) in [("F1", R1), ("F2", "MW-E5FIX_20261030T080903"), ("F3", R3)] {
        for e in std::fs::read_dir(golden().join(f).join(run_id).join("params")).unwrap().flatten() {
            let text = std::fs::read_to_string(e.path()).unwrap();
            let doc = mower_pyfmt::yaml::safe_load(&text).unwrap();
            let Py::Dict(top) = &doc else { panic!() };
            let (node, body) = &top[0];
            let mut ps = Vec::new();
            flatten("", body.get("ros__parameters").unwrap(), &mut ps);
            assert_eq!(params::dump(node, &ps).unwrap(), text, "{}", e.path().display());
            assert_eq!(run::params_file_name(node), e.file_name().to_string_lossy());
            n += 1;
        }
    }
    assert_eq!(n, 7);
}

// ---- the bag store ----------------------------------------------------------------------------

#[test]
fn the_bag_list_as_bag_store_sent_it() {
    let tmp = std::env::temp_dir().join(format!("mower_recorder_core_bags_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&tmp);
    std::fs::create_dir_all(tmp.join(R1)).unwrap();
    std::fs::copy(golden().join("F1").join(R1).join("run_metadata.yaml"), tmp.join(R1).join("run_metadata.yaml")).unwrap();
    let mut checked = 0;
    for (topic, data) in listen("F1") {
        if topic != "/mower_recorder/bags" {
            continue;
        }
        let real = json::loads(&data).unwrap();
        let Some(Py::List(bags)) = real.get("bags") else { panic!() };
        if bags.is_empty() {
            continue;
        }
        let rec = matches!(bags[0].get("recording"), Some(Py::Bool(true)));
        let st = store::ListState { robot_id: "MW-E5FIX".into(), recording: rec.then(|| R1.to_string()), ..Default::default() };
        let ours = store::bags_json(&tmp, &st, false, false).unwrap();
        // the size is the run dir's at that moment: take the real one
        let mut v = json::loads(&ours).unwrap();
        if let (Py::Dict(top), Some(Py::Int(size))) = (&mut v, bags[0].get("size_bytes")) {
            if let Some((_, Py::List(l))) = top.iter_mut().find(|(k, _)| k == "bags") {
                profile::dict_set(&mut l[0], "size_bytes", Py::Int(*size));
            }
        }
        assert_eq!(json::dumps(&v, &json::JsonOptions::unicode()).unwrap(), data);
        checked += 1;
    }
    assert!(checked > 3, "{checked}");
    std::fs::remove_dir_all(&tmp).unwrap();
}

#[test]
fn commands_and_their_replies() {
    let tmp = std::env::temp_dir().join(format!("mower_recorder_core_cmd_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&tmp);
    std::fs::create_dir_all(tmp.join("r1")).unwrap();
    std::fs::write(tmp.join("r1/run_metadata.yaml"), "run_id: r1\nstart_time: '2026-10-27T10:15:00.123456'\n").unwrap();
    assert_eq!(store::parse_command("{bad"), Some(Err("bad command json: Expecting property name enclosed in double quotes: line 1 column 2 (char 1)".into())));
    let (c, req) = store::parse_command(r#"{"action": "rename", "run_id": "r1", "new_name": " 東區 ", "req_id": "app_1_2"}"#).unwrap().unwrap();
    assert_eq!(c, store::Command::Rename { run_id: "r1".into(), new_name: " 東區 ".into() });
    assert_eq!(store::result_json(&req, true, "已更名為「東區」"), r#"{"req_id": "app_1_2", "ok": true, "message": "已更名為「東區」"}"#);
    assert_eq!(store::rename(&tmp, "r1", " 東區 "), (true, "已更名為「東區」".into()));
    assert_eq!(std::fs::read_to_string(tmp.join("r1/run_metadata.yaml")).unwrap(), "run_id: r1\nstart_time: '2026-10-27T10:15:00.123456'\ndisplay_name: 東區\n");
    assert_eq!(store::rename(&tmp, "r1", "  "), (false, "新名稱不可空白".into()));
    assert_eq!(store::rename(&tmp, "../x", "a"), (false, "找不到 run: ../x".into()));
    // an unparsable file becomes {display_name}
    std::fs::write(tmp.join("r1/run_metadata.yaml"), "a: [\n").unwrap();
    store::rename(&tmp, "r1", "x");
    assert_eq!(std::fs::read_to_string(tmp.join("r1/run_metadata.yaml")).unwrap(), "display_name: x\n");
    assert_eq!(store::delete_local(&tmp, "r1", Some("r1"), "bags", "m").message, "這個 run 正在錄製中,無法刪除");
    std::fs::write(tmp.join("r1/.uploaded"), "{}").unwrap();
    let d = store::delete_local(&tmp, "r1", None, "bags/", "MW-E5FIX");
    assert_eq!((d.ok, d.message.as_str(), d.r2_base_key.as_deref()), (true, "已刪除 r1", Some("bags/MW-E5FIX/r1")));
    assert!(!tmp.join("r1").exists());
    assert_eq!(store::parse_command(r#"{"action": "nope", "req_id": 3}"#).unwrap().unwrap(), (store::Command::Unknown("nope".into()), Py::Int(3)));
    std::fs::remove_dir_all(&tmp).unwrap();
}

#[test]
fn the_two_credential_precedences() {
    let tmp = std::env::temp_dir().join(format!("mower_recorder_core_env_{}", std::process::id()));
    std::fs::create_dir_all(&tmp).unwrap();
    let f = tmp.join("r2.env");
    std::fs::write(&f, "# c\nR2_BUCKET=\"file-bucket\"\nr2_account_id = 'acct'\nR2_PREFIX=runs\n").unwrap();
    let env = |k: &str| match k { "R2_BUCKET" => Some("env-bucket".to_string()), "R2_ACCESS_KEY_ID" => Some("key".to_string()), _ => None };
    // bag_store: the file overrides the env
    let c = store::load_r2_config(&env, Some(&f)).unwrap();
    assert_eq!((c.bucket.as_str(), c.account_id.as_str(), c.prefix.as_str(), c.access_key_id.as_str()), ("file-bucket", "acct", "runs", "key"));
    // mower-bag: the env wins, the file fills what it left empty, with the
    // exact upper-case key only
    let c = store::load_cli_env(&env, &[tmp.join("none"), f.clone()]);
    assert_eq!((c.bucket.as_str(), c.account_id.as_str(), c.prefix.as_str()), ("env-bucket", "", "runs"));
    assert_eq!(store::load_cli_env(&env, &[]).prefix, "bags");
    std::fs::remove_dir_all(&tmp).unwrap();
}

// ---- MediaMTX (test_video_record.py) --------------------------------------------------------------

/// A one-request HTTP server: answers `status` with `body`, returns what it got.
fn serve_once(status: u16, body: &'static str) -> (String, std::thread::JoinHandle<String>) {
    let l = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let url = format!("http://{}", l.local_addr().unwrap());
    let h = std::thread::spawn(move || {
        let (mut s, _) = l.accept().unwrap();
        let mut buf = vec![0u8; 65536];
        let mut got = Vec::new();
        loop {
            let n = s.read(&mut buf).unwrap();
            got.extend_from_slice(&buf[..n]);
            let text = String::from_utf8_lossy(&got).into_owned();
            if let Some(i) = text.find("\r\n\r\n") {
                let len = text.lines().find_map(|l| l.to_lowercase().strip_prefix("content-length: ").map(|v| v.trim().parse::<usize>().unwrap())).unwrap_or(0);
                if got.len() >= i + 4 + len {
                    break;
                }
            }
            if n == 0 {
                break;
            }
        }
        write!(s, "HTTP/1.1 {status} X\r\nContent-Length: {}\r\nContent-Type: application/json\r\n\r\n{body}", body.len()).unwrap();
        String::from_utf8_lossy(&got).into_owned()
    });
    (url, h)
}

#[test]
fn mediamtx_calls_as_video_record_makes_them() {
    let run = format!("/home/mower/.mower/bags/{R1}");
    assert_eq!(mediamtx::record_patch(&run).get("recordPath"), Some(&Py::Str(format!("{run}/video/%path_%Y-%m-%d_%H-%M-%S-%f"))));
    // the bodies the Python sent in F1, byte for byte
    let log = String::from_utf8(zstd::decode_all(std::fs::File::open(golden().join("F1/mediamtx.jsonl.zst")).unwrap()).unwrap()).unwrap();
    let bodies: Vec<String> = log.lines().map(|l| serde_json::from_str::<serde_json::Value>(l).unwrap()["body"].as_str().unwrap().to_string()).collect();
    let ours = [mediamtx::body_bytes(&mediamtx::record_patch(&run)), mediamtx::body_bytes(&mediamtx::stop_patch())].map(|b| String::from_utf8(b).unwrap());
    for b in bodies.iter().filter(|b| !b.is_empty()) {
        assert!(ours.contains(b), "{b}");
    }
    let (url, h) = serve_once(200, "");
    assert_eq!(mediamtx::patch_path(&url, "front", &mediamtx::stop_patch()), (true, "HTTP 200".into()));
    let req = h.join().unwrap();
    assert!(req.starts_with("PATCH /v3/config/paths/patch/front HTTP/1.1\r\n"), "{req}");
    assert!(req.to_lowercase().contains("content-type: application/json") && req.ends_with("{\"record\": false}"));
    let (url, h) = serve_once(400, " {\"error\": \"bad\"} ");
    assert_eq!(mediamtx::patch_path(&url, "front", &mediamtx::stop_patch()), (false, "HTTP 400 {\"error\": \"bad\"}".into()));
    h.join().unwrap();
    let (url, h) = serve_once(200, "{\"name\": \"front\", \"ready\": true}");
    assert_eq!(mediamtx::path_ready(&url, "front"), Some(true));
    assert!(h.join().unwrap().starts_with("GET /v3/paths/get/front "));
    let (url, h) = serve_once(404, "");
    assert_eq!(mediamtx::path_ready(&url, "front"), Some(false));
    h.join().unwrap();
    assert_eq!(mediamtx::path_ready("http://127.0.0.1:1", "front"), None);
}

#[test]
fn the_disk_guard() {
    assert!(!profile::disk_low(Path::new("/"), 0));
    assert!(profile::disk_low(Path::new("/nonexistent/deeper/run"), i64::MAX / (1 << 21)));
    assert!(!profile::disk_low(Path::new("/tmp"), 1));
}
