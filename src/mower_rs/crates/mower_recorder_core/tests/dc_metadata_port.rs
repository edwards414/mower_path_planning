//! The data_collection tests of `test/test_record_profile.py`, one for one:
//! `test_build_run_metadata_spec_fields` and `test_copy_calibration`. The
//! session template is what PyYAML loads from
//! `config/data_collection_session.yaml`; the loader handed to
//! copy_calibration stands for `yaml.safe_load` of the two texts written.

use mower_pyfmt::yaml::{safe_dump, YamlOptions};
use mower_pyfmt::Py;
use mower_recorder_core::dc_metadata::{self as rp, MetadataInputs};

fn session_template() -> Py {
    Py::dict([
        ("camera", Py::dict([("model", Py::str("<型號>")), ("serial", Py::str("<序號>")), ("exposure", Py::str("auto"))])),
        ("grass_height_cm", Py::Int(8)),
        ("weather", Py::str("sunny")),
        ("blade", Py::str("off")),
        ("heading_calibration", Py::str("pending")),
        ("calibration_date", Py::str("")),
        ("notes", Py::str("")),
    ])
}

fn keys(v: &Py) -> Vec<&str> {
    match v {
        Py::Dict(items) => items.iter().map(|(k, _)| k.as_str()).collect(),
        _ => vec![],
    }
}

fn tmp(name: &str) -> std::path::PathBuf {
    let d = std::env::temp_dir().join(format!("mower_recorder_core_{name}_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).unwrap();
    d
}

#[test]
fn test_build_run_metadata_spec_fields() {
    let tmp_path = tmp("spec");
    let mut session = session_template();
    // what an unquoted `blade: off` becomes (P-099)
    if let Py::Dict(items) = &mut session {
        items.iter_mut().find(|(k, _)| k == "blade").unwrap().1 = Py::Bool(false);
    }
    let base = Py::dict([("run_id", Py::str("r")), ("robot_id", Py::str("mower-03")), ("git", Py::dict([("sha", Py::str("unknown"))]))]);
    let calib = rp::copy_calibration(&tmp_path, "", "", None, false, &|_| unreachable!()).unwrap();
    let run_info = Py::dict([("weather", Py::str("cloudy")), ("grass_height_cm", Py::Int(6)), ("site", Py::str("yard2"))]);
    let camera = Py::dict([
        ("device", Py::str("/dev/video0")), ("width", Py::Int(1280)), ("height", Py::Int(720)), ("fps_native", Py::Int(25)),
        ("fps_recorded", Py::Int(10)), ("format", Py::str("jpeg")), ("autofocus", Py::str("off")),
    ]);
    let ext = Py::dict([("source", Py::str("measured")), ("camera_height_m", Py::Float(0.62)), ("camera_pitch_deg", Py::Float(20.0))]);
    let topics = ["/camera/front/image_raw/compressed".to_string()];
    let inputs = MetadataInputs {
        session: Some(&session),
        run_info: Some(&run_info),
        camera_params: Some(&camera),
        extrinsics: Some(&ext),
        calibration: Some(&calib),
        camera_topics: Some(&topics),
    };
    let env = |k: &str| match k {
        "MOWER_GIT_SHA" => Some("abc".to_string()),
        "MOWER_VERSION" => Some("v1".to_string()),
        _ => None,
    };
    let meta = rp::build_run_metadata(&base, "data_collection", &inputs, &env).unwrap();
    for key in ["profile", "camera", "camera_height_m", "camera_pitch_deg", "extrinsics_source", "calibration",
        "grass_height_cm", "weather", "blade", "heading_calibration", "notes"] {
        assert!(meta.get(key).is_some(), "{key}");
    }
    let cam = meta.get("camera").unwrap();
    assert_eq!(keys(cam)[..10], ["model", "serial", "device", "width", "height", "fps_native", "fps_recorded", "format",
        "autofocus", "exposure"]);
    assert_eq!(meta.get("weather"), Some(&Py::str("cloudy")));
    assert_eq!(meta.get("grass_height_cm"), Some(&Py::Int(6)));
    assert_eq!(meta.get("blade"), Some(&Py::str("off")));
    assert_eq!(cam.get("exposure"), Some(&Py::str("auto")));
    assert_eq!(meta.get("build").unwrap().get("git_sha"), Some(&Py::str("abc")));
    assert_eq!(meta.get("session_extra"), Some(&Py::dict([("site", Py::str("yard2"))])));
    let text = safe_dump(&meta, &YamlOptions { allow_unicode: true, sort_keys: false });
    // stays a string for PyYAML
    assert!(text.contains("blade: 'off'"), "{text}");
    std::fs::remove_dir_all(&tmp_path).unwrap();
}

#[test]
fn test_copy_calibration() {
    let tmp_path = tmp("copy");
    let src = tmp_path.join("src");
    std::fs::create_dir_all(&src).unwrap();
    std::fs::write(src.join("camera_front.yaml"), "image_width: 1280\n").unwrap();
    std::fs::write(src.join("extrinsics.yaml"), "source: measured\ndate: 2026-09-25\n").unwrap();
    let run = tmp_path.join("run");
    std::fs::create_dir_all(&run).unwrap();
    // yaml.safe_load of the extrinsics text; its unquoted date is a
    // datetime.date in PyYAML, and str() of it is what the info keeps
    let load = |text: &str| {
        assert_eq!(text, "source: measured\ndate: 2026-09-25\n");
        Ok(Py::dict([("source", Py::str("measured")), ("date", Py::str("2026-09-25"))]))
    };
    let derived = Py::dict([("camera_height_m", Py::Float(0.62))]);
    let info = rp::copy_calibration(&run, &src.join("camera_front.yaml").to_string_lossy(),
        &src.join("extrinsics.yaml").to_string_lossy(), Some(&derived), false, &load).unwrap();
    assert_eq!(info, Py::dict([
        ("intrinsics_file", Py::str("calib/camera_front.yaml")),
        ("extrinsics_file", Py::str("calib/extrinsics.yaml")),
        ("intrinsics_source", Py::str("calibrated")),
        ("date", Py::str("2026-09-25")),
    ]));
    let written = std::fs::read_to_string(run.join("calib/extrinsics.yaml")).unwrap();
    assert!(written.ends_with("derived:\n  camera_height_m: 0.62\n"), "{written}");
    assert_eq!(std::fs::read(run.join("calib/camera_front.yaml")).unwrap(), b"image_width: 1280\n");
    let missing = rp::copy_calibration(&run, "/nope.yaml", "", None, false, &|_| unreachable!()).unwrap();
    let Some(Py::List(problems)) = missing.get("problems") else { panic!("{missing:?}") };
    assert_eq!(problems.len(), 2);
    assert_eq!(problems[0], Py::str("intrinsics not found: '/nope.yaml'"));
    std::fs::remove_dir_all(&tmp_path).unwrap();
}

#[test]
fn deep_merge_and_onoff() {
    let base = Py::dict([("a", Py::dict([("x", Py::Int(1)), ("y", Py::Int(2))])), ("b", Py::Int(1))]);
    let extra = Py::dict([("a", Py::dict([("y", Py::Int(3)), ("z", Py::Int(4))])), ("c", Py::Bool(false))]);
    assert_eq!(rp::deep_merge(&base, &extra), Py::dict([
        ("a", Py::dict([("x", Py::Int(1)), ("y", Py::Int(3)), ("z", Py::Int(4))])),
        ("b", Py::Int(1)),
        ("c", Py::Bool(false)),
    ]));
    assert_eq!(rp::onoff(&Py::Bool(false)), Py::str("off"));
    assert_eq!(rp::onoff(&Py::Bool(true)), Py::str("on"));
    assert_eq!(rp::onoff(&Py::None), Py::None);
    assert_eq!(rp::onoff(&Py::Int(0)), Py::str("0"));
}
