//! (E5-5) data_collection's request file and what its launch worked out
//! (camera_launch.py `resolve_camera`, data_collection.launch.py `_setup`),
//! held to the Python: the pipeline builder's cases, the placeholder
//! intrinsics as PyYAML wrote them, and F3's run, whose camera block and
//! derived extrinsics (E5-0's golden) `resolve` must give again from the same
//! calibration, session and URDF.

use std::path::{Path, PathBuf};

use mower_pyfmt::Py;
use mower_recorder_core::dc_request::{self, build_pipeline, placeholder_intrinsics, resolve, CameraArgs, Request};

fn repo() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../..").canonicalize().unwrap()
}

fn golden_f3() -> PathBuf {
    repo().join("deploy/test/golden/recorder/F3/MW-E5FIX_20261030T080906")
}

fn share_config() -> PathBuf {
    repo().join("src/mower_recorder/config")
}

fn tmp(name: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("dc_request_{name}_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).unwrap();
    d
}

/// tests/vectors/extrinsics.json's `xacro.process_file(real_robot.xacro).toxml()`
fn real_robot_urdf() -> String {
    let v: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/vectors/extrinsics.json")).unwrap())
            .unwrap();
    v["texts"]["real_robot_urdf"].as_str().unwrap().to_string()
}

#[test]
fn the_request_file_is_strict() {
    let r = Request::parse(
        r#"{"profile": "data_collection", "camera": {"camera_device": "/dev/video2", "camera_size": "640x480",
            "camera_fps": 30, "record_fps": "5", "calib_dir": "/home/mower/.mower/calib"},
            "session_file": "/s.yaml", "run_info": "{weather: cloudy}", "upload_policy": "manual"}"#,
    )
    .unwrap();
    assert_eq!(r.camera.camera_device, "/dev/video2");
    assert_eq!(r.camera.camera_fps, "30", "a number is the text the launch would have had");
    assert_eq!(r.camera.v4l2_controls, dc_request::DEFAULT_V4L2_CONTROLS, "the launch's default");
    assert_eq!((r.session_file.as_str(), r.run_info.as_str(), r.upload_policy.as_str()), ("/s.yaml", "{weather: cloudy}", "manual"));
    let minimal = Request::parse(r#"{"profile": "data_collection"}"#).unwrap();
    assert_eq!(minimal.camera, CameraArgs::default());
    assert_eq!(minimal.session_file, "~/.mower/data_collection_session.yaml");
    for bad in [
        r#"{"profile": "default"}"#,
        r#"{"profile": "data_collection", "camera": {"camera_devise": "/dev/video0"}}"#,
        r#"{"profile": "data_collection", "upload": "manual"}"#,
        r#"{"profile": "data_collection", "upload_policy": "sometimes"}"#,
        r#"["data_collection"]"#,
        "not json",
    ] {
        assert!(Request::parse(bad).is_err(), "{bad}");
    }
}

/// camera_launch.py `build_pipeline`
#[test]
fn the_pipeline_is_camera_launch_s() {
    let c = dc_request::DEFAULT_V4L2_CONTROLS;
    assert_eq!(
        build_pipeline("/dev/video0", 1280, 720, 25, 10.0, c),
        format!(
            "v4l2src device=/dev/video0 extra-controls=\"{c}\" ! image/jpeg,width=1280,height=720,framerate=25/1 ! \
             videorate drop-only=true max-rate=10"
        )
    );
    assert_eq!(build_pipeline("/dev/video1", 640, 480, 30, 30.0, ""), "v4l2src device=/dev/video1 ! image/jpeg,width=640,height=480,framerate=30/1");
    // int(round(12.5)) is 12: half to even
    assert!(build_pipeline("/dev/video0", 640, 480, 30, 12.5, "").ends_with("max-rate=12"));
    assert!(build_pipeline("/dev/video0", 640, 480, 30, 13.5, "").ends_with("max-rate=14"));
    assert!(!build_pipeline("/dev/video0", 640, 480, 30, 0.0, "").contains("videorate"), "record_fps 0: no thinning");
}

/// `placeholder_intrinsics(80, 60)` as PyYAML 6.0.3 wrote it
#[test]
fn the_placeholder_intrinsics_are_the_launch_s() {
    let (path, text) = placeholder_intrinsics(80, 60);
    assert_eq!(path, std::env::temp_dir().join("mower_camera_front_uncalibrated_80x60.yaml"));
    let zeros = |n: usize| "  - 0.0\n".repeat(n);
    let expected = format!(
        "# 自動產生：尚未標定（K 全 0）。完成棋盤格標定後放到 ~/.mower/calib/camera_front.yaml\nimage_width: 80\n\
         image_height: 60\ncamera_name: camera_front\ncamera_matrix:\n  rows: 3\n  cols: 3\n  data:\n{}\
         distortion_model: plumb_bob\ndistortion_coefficients:\n  rows: 1\n  cols: 5\n  data:\n{}\
         rectification_matrix:\n  rows: 3\n  cols: 3\n  data:\n  - 1.0\n  - 0.0\n  - 0.0\n  - 0.0\n  - 1.0\n  - 0.0\n  \
         - 0.0\n  - 0.0\n  - 1.0\nprojection_matrix:\n  rows: 3\n  cols: 4\n  data:\n{}",
        zeros(9),
        zeros(5),
        zeros(12)
    );
    assert_eq!(text, expected);
}

/// F3: what data_collection.launch.py passed recorder_manager for E5-0's run
/// (e2-verify's record_run.sh: the fake camera's pipeline, an 80x60
/// calibration, the measured extrinsics, the session file), back from the
/// request and the URDF: the run's camera block and derived extrinsics.
#[test]
fn f3_s_camera_parameters_come_back_from_the_request() {
    let g = golden_f3();
    let d = tmp("f3");
    let calib = d.join("calib");
    std::fs::create_dir_all(&calib).unwrap();
    std::fs::copy(g.join("calib/camera_front.yaml"), calib.join("camera_front.yaml")).unwrap();
    std::fs::copy(g.join("calib/extrinsics.yaml"), calib.join("extrinsics.yaml")).unwrap();
    let meta = mower_pyfmt::yaml::safe_load(&std::fs::read_to_string(g.join("run_metadata.yaml")).unwrap()).unwrap();
    let cam = meta.get("camera").unwrap();
    let Some(Py::Str(pipeline)) = cam.get("pipeline") else { panic!() };
    let session = repo().join("deploy/test/golden/recorder/F3/session.yaml");
    let req = Request {
        profile: "data_collection".into(),
        camera: CameraArgs {
            camera_size: "80x60".into(),
            gscam_pipeline: pipeline.clone(),
            calib_dir: calib.to_string_lossy().into_owned(),
            ..CameraArgs::default()
        },
        session_file: session.to_string_lossy().into_owned(),
        run_info: String::new(),
        upload_policy: "manual".into(),
    };
    let urdf = real_robot_urdf();
    let r = resolve(&req, &share_config(), Ok(&urdf)).unwrap();
    // the fixture's extrinsics say `source: placeholder` (record_run.sh's
    // replace hit the comment): camera_extrinsics.validate's warning
    assert_eq!(r.notes, ["source=placeholder：外參是佔位值，必須實測後覆寫"]);
    assert_eq!(r.placeholder, None);
    assert!(!r.dc.intrinsics_placeholder);
    assert_eq!(r.dc.intrinsics_file, calib.join("camera_front.yaml").to_string_lossy());
    assert_eq!(r.dc.extrinsics_file, calib.join("extrinsics.yaml").to_string_lossy());
    assert_eq!(r.dc.session_file, session.to_string_lossy());
    let params = mower_pyfmt::json::loads(&r.dc.camera_params).unwrap();
    let golden = Py::dict(
        ["device", "width", "height", "fps_native", "fps_recorded", "format", "autofocus", "driver", "pipeline"].map(|k| (k, cam.get(k).unwrap().clone())),
    );
    assert_eq!(params, golden, "the run's camera block, in the launch's key order");
    let derived = mower_pyfmt::json::loads(&r.dc.extrinsics_derived).unwrap();
    let ext = mower_pyfmt::yaml::safe_load(&std::fs::read_to_string(g.join("calib/extrinsics.yaml")).unwrap()).unwrap();
    assert_eq!(derived.get("block").unwrap(), ext.get("derived").unwrap(), "calib/extrinsics.yaml's derived block");
    let summary = derived.get("summary").unwrap();
    assert_eq!(summary.get("source"), meta.get("extrinsics_source"));
    assert_eq!(summary.get("camera_height_m"), meta.get("camera_height_m"));
    assert_eq!(summary.get("camera_pitch_deg"), meta.get("camera_pitch_deg"));
    std::fs::remove_dir_all(&d).unwrap();
}

/// No calibration, no session, no URDF: the placeholder intrinsics, the
/// package's extrinsics and session template and the built-in geometry,
/// each said, as the launch said them.
#[test]
fn without_a_calibration_the_placeholders_and_the_notes() {
    let d = tmp("bare");
    let req = Request::parse(&format!(
        r#"{{"profile": "data_collection", "camera": {{"calib_dir": "{}", "camera_size": "640x480"}},
            "session_file": "{}"}}"#,
        d.join("calib").display(),
        d.join("nope.yaml").display()
    ))
    .unwrap();
    let r = resolve(&req, &share_config(), Err("no /robot_description".into())).unwrap();
    assert!(r.dc.intrinsics_placeholder);
    let (path, text) = r.placeholder.clone().unwrap();
    assert_eq!(r.dc.intrinsics_file, path.to_string_lossy());
    assert!(text.contains("image_width: 640\nimage_height: 480\n"));
    assert_eq!(r.dc.extrinsics_file, share_config().join("camera_front_extrinsics.yaml").to_string_lossy());
    assert_eq!(r.dc.session_file, share_config().join("data_collection_session.yaml").to_string_lossy());
    assert!(r.notes[0].starts_with("找不到內參 "), "{:?}", r.notes);
    assert!(r.notes[1].starts_with("找不到外參 "), "{:?}", r.notes);
    assert_eq!(r.notes[2], "URDF 讀取失敗，改用內建幾何：no /robot_description");
    assert!(r.notes.last().unwrap().starts_with("找不到 "), "{:?}", r.notes);
    let params = mower_pyfmt::json::loads(&r.dc.camera_params).unwrap();
    assert_eq!(params.get("autofocus"), Some(&Py::str("off")));
    assert_eq!(params.get("fps_recorded"), Some(&Py::Int(10)));
    let derived = mower_pyfmt::json::loads(&r.dc.extrinsics_derived).unwrap();
    assert_eq!(derived.get("block").unwrap().get("reference_source"), Some(&Py::str("builtin_default")));
    // a size that is not WxH fails, as the launch did
    let mut bad = req.clone();
    bad.camera.camera_size = "640".into();
    assert!(resolve(&bad, &share_config(), Err(String::new())).is_err());
    std::fs::remove_dir_all(&d).unwrap();
}
