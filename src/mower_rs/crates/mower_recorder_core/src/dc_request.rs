//! (E5-5) data_collection in the module: the request file and what
//! data_collection.launch.py and camera_launch.py worked out before the
//! Python recorder started (E5 design 4.7, D7).
//!
//! - **The request file** `~/.mower/recorder_profile.json`, written by
//!   `mower-data-collection.sh start` and deleted by `stop`: the profile, the
//!   camera's launch arguments (camera_launch.py's `CAMERA_ARGS`, by their
//!   names), `session_file`, `run_info`, `upload_policy` ([`Request`]).
//!   recorder_manager applies it while idle; bag_store takes its
//!   `upload_policy`.
//! - **[`resolve`]**: camera_launch.py's `resolve_camera` and
//!   data_collection.launch.py's `_setup`, for recorder_manager: the
//!   pipeline (`build_pipeline`), the intrinsics (the calibration, else the
//!   uncalibrated placeholder written to the temp dir, as the launch did),
//!   the extrinsics (the calibration's, else the package's placeholder), the
//!   URDF reference (robot_state_publisher's `/robot_description`, else the
//!   built-in geometry), `camera_extrinsics.compute`, and from them the
//!   parameters recorder_manager got: `camera_params` and
//!   `extrinsics_derived` as the launch's `json.dumps`, `intrinsics_*`,
//!   `extrinsics_file`, `session_file` (the package's template when the
//!   robot's is missing), `run_info`. The camera itself stays gscam's in the
//!   camera-only compose (front_camera.launch.py), which works out the same
//!   transforms for the static TF from the same files.
//! - (E5-6) `camera.driver`: `gscam` (E5-5's camera-only compose) or
//!   `gstreamer` (the `bag` module's `camera_front` instance, which takes
//!   the pipeline, the intrinsics and [`Resolved::extrinsics`]'s two
//!   transforms from here). A file without it is E5-5's: gscam.

use std::path::{Path, PathBuf};

use mower_pyfmt::json::JsonOptions;
use mower_pyfmt::yaml::YamlOptions;
use mower_pyfmt::Py;

use crate::extrinsics::{self, Reference};
use crate::manager::DcParams;
use crate::util::expand_user;

/// The file's name in the state directory (`~/.mower`).
pub const FILE_NAME: &str = "recorder_profile.json";

/// The profile the request asks for (the only one so far).
pub const DATA_COLLECTION: &str = "data_collection";

/// data_collection.launch.py's `PARAMS_DUMP_NODES`: the EKF modes and
/// navsat's yaw offset, needed offline (P-086).
pub const PARAMS_DUMP_NODES: [&str; 3] = ["/ekf_filter_node_odom", "/ekf_filter_node_map", "/navsat_transform"];

/// camera_launch.py's `DEFAULT_V4L2_CONTROLS`.
pub const DEFAULT_V4L2_CONTROLS: &str =
    "c,focus_automatic_continuous=0,focus_auto=0,exposure_dynamic_framerate=0,exposure_auto_priority=0";

/// `CAMERA_NAME`
pub const CAMERA_NAME: &str = "camera_front";

/// `camera.driver`'s values.
pub const DRIVER_GSCAM: &str = "gscam";
pub const DRIVER_GSTREAMER: &str = "gstreamer";

/// The camera's launch arguments, with camera_launch.py's defaults.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CameraArgs {
    pub camera_device: String,
    pub camera_size: String,
    pub camera_fps: String,
    pub record_fps: String,
    pub v4l2_controls: String,
    pub gscam_pipeline: String,
    pub calib_dir: String,
    pub intrinsics_file: String,
    pub extrinsics_file: String,
    /// E5-6: who runs the camera, [`DRIVER_GSCAM`] or [`DRIVER_GSTREAMER`]
    pub driver: String,
}

impl Default for CameraArgs {
    fn default() -> Self {
        CameraArgs {
            camera_device: "/dev/video0".into(),
            camera_size: "1280x720".into(),
            camera_fps: "25".into(),
            record_fps: "10".into(),
            v4l2_controls: DEFAULT_V4L2_CONTROLS.into(),
            gscam_pipeline: String::new(),
            calib_dir: "~/.mower/calib".into(),
            intrinsics_file: String::new(),
            extrinsics_file: String::new(),
            driver: DRIVER_GSCAM.into(),
        }
    }
}

/// What the request file asks for.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Request {
    pub profile: String,
    pub camera: CameraArgs,
    /// data_collection.launch.py's `session_file` (its default)
    pub session_file: String,
    pub run_info: String,
    /// `wifi_or_dock`, `always` or `manual`; empty: bag_store's own
    pub upload_policy: String,
}

/// A value of the file as the launch's `LaunchConfiguration(...).perform()`
/// gave it, stripped: a string, or a number written as one.
fn text(v: &Py) -> Option<String> {
    match v {
        Py::Str(s) => Some(s.trim().to_string()),
        Py::Int(i) => Some(i.to_string()),
        Py::Float(f) => Some(mower_pyfmt::repr::float(*f)),
        Py::Bool(b) => Some(if *b { "true" } else { "false" }.into()),
        _ => None,
    }
}

impl Request {
    /// The file's JSON: an object with `profile` (only `data_collection`),
    /// and optionally `camera` (an object of the camera arguments),
    /// `session_file`, `run_info`, `upload_policy`. A key it does not know
    /// is an error, so that a typo does not pass for a default.
    pub fn parse(json: &str) -> Result<Request, String> {
        let doc = mower_pyfmt::json::loads(json)?;
        let Py::Dict(items) = &doc else { return Err("not a JSON object".into()) };
        let mut r = Request {
            profile: String::new(),
            camera: CameraArgs::default(),
            session_file: "~/.mower/data_collection_session.yaml".into(),
            run_info: String::new(),
            upload_policy: String::new(),
        };
        for (k, v) in items {
            let s = || text(v).ok_or_else(|| format!("{k}: not a string"));
            match k.as_str() {
                "profile" => r.profile = s()?,
                "session_file" => r.session_file = s()?,
                "run_info" => r.run_info = s()?,
                "upload_policy" => r.upload_policy = s()?,
                "camera" => {
                    let Py::Dict(cam) = v else { return Err("camera: not an object".into()) };
                    for (ck, cv) in cam {
                        let val = text(cv).ok_or_else(|| format!("camera.{ck}: not a string"))?;
                        let c = &mut r.camera;
                        match ck.as_str() {
                            "camera_device" => c.camera_device = val,
                            "camera_size" => c.camera_size = val,
                            "camera_fps" => c.camera_fps = val,
                            "record_fps" => c.record_fps = val,
                            "v4l2_controls" => c.v4l2_controls = val,
                            "gscam_pipeline" => c.gscam_pipeline = val,
                            "calib_dir" => c.calib_dir = val,
                            "intrinsics_file" => c.intrinsics_file = val,
                            "extrinsics_file" => c.extrinsics_file = val,
                            "driver" => c.driver = val,
                            other => return Err(format!("camera.{other}: unknown")),
                        }
                    }
                }
                other => return Err(format!("{other}: unknown")),
            }
        }
        if r.profile != DATA_COLLECTION {
            return Err(format!("profile {:?}: only {DATA_COLLECTION:?}", r.profile));
        }
        if r.camera.driver != DRIVER_GSCAM && r.camera.driver != DRIVER_GSTREAMER {
            return Err(format!("camera.driver {:?}: {DRIVER_GSCAM} or {DRIVER_GSTREAMER}", r.camera.driver));
        }
        if !matches!(r.upload_policy.as_str(), "" | "wifi_or_dock" | "always" | "manual") {
            return Err(format!("upload_policy {:?}: wifi_or_dock, always or manual", r.upload_policy));
        }
        Ok(r)
    }
}

/// `build_pipeline(device, width, height, fps, record_fps, controls)`
pub fn build_pipeline(device: &str, width: i64, height: i64, fps: i64, record_fps: f64, controls: &str) -> String {
    let mut src = format!("v4l2src device={device}");
    if !controls.is_empty() {
        src += &format!(" extra-controls=\"{controls}\"");
    }
    let mut parts = vec![src, format!("image/jpeg,width={width},height={height},framerate={fps}/1")];
    if record_fps != 0.0 && record_fps < fps as f64 {
        // Python's int(round(x)): half to even
        parts.push(format!("videorate drop-only=true max-rate={}", record_fps.round_ties_even() as i64));
    }
    parts.join(" ! ")
}

/// `placeholder_intrinsics`'s file: where it goes and what it says.
pub fn placeholder_intrinsics(width: i64, height: i64) -> (PathBuf, String) {
    let zeros = |n: usize| Py::List(vec![Py::Float(0.0); n]);
    let matrix = |rows: i64, cols: i64, data: Py| Py::dict([("rows", Py::Int(rows)), ("cols", Py::Int(cols)), ("data", data)]);
    let identity = Py::List([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0].into_iter().map(Py::Float).collect());
    let doc = Py::dict([
        ("image_width", Py::Int(width)),
        ("image_height", Py::Int(height)),
        ("camera_name", Py::str(CAMERA_NAME)),
        ("camera_matrix", matrix(3, 3, zeros(9))),
        ("distortion_model", Py::str("plumb_bob")),
        ("distortion_coefficients", matrix(1, 5, zeros(5))),
        ("rectification_matrix", matrix(3, 3, identity)),
        ("projection_matrix", matrix(3, 4, zeros(12))),
    ]);
    let text = format!(
        "# 自動產生：尚未標定（K 全 0）。完成棋盤格標定後放到 ~/.mower/calib/camera_front.yaml\n{}",
        mower_pyfmt::yaml::safe_dump(&doc, &YamlOptions { allow_unicode: false, sort_keys: false })
    );
    let path = std::env::temp_dir().join(format!("mower_camera_front_uncalibrated_{width}x{height}.yaml"));
    (path, text)
}

/// What [`resolve`] made.
#[derive(Debug, Clone, PartialEq)]
pub struct Resolved {
    /// recorder_manager's data_collection parameters
    pub dc: DcParams,
    pub pipeline: String,
    /// The uncalibrated placeholder to write before a run copies it into
    /// `calib/` ([`placeholder_intrinsics`]), when there is no calibration.
    pub placeholder: Option<(PathBuf, String)>,
    /// The launch's warnings (LogInfo `WARNING:` lines), in its order.
    pub notes: Vec<String>,
    /// The intrinsics file the camera publishes (the calibration's, or the
    /// placeholder's path)
    pub intrinsics: PathBuf,
    /// `camera_extrinsics.compute`'s result: the static TF's two transforms
    /// (`base_link_to_camera_link`, `camera_link_to_optical`)
    pub extrinsics: extrinsics::Computed,
}

/// Python's `float(text)` (`"25"`, `" 10.0 "`, `"1e1"`).
fn py_float(s: &str) -> Result<f64, String> {
    extrinsics::parse_py_float(s.trim()).ok_or_else(|| format!("could not convert string to float: {s:?}"))
}

/// `resolve_camera(context)` and `_setup`'s session fallback, for a request.
/// `share_config` is mower_recorder's `config/` (the extrinsics placeholder
/// and the session template); `urdf` robot_state_publisher's
/// `/robot_description` if it came (`Err` says why not). An `Err` is where
/// the launch would have failed (a size that is not `WxH`, extrinsics that
/// do not validate).
pub fn resolve(req: &Request, share_config: &Path, urdf: Result<&str, String>) -> Result<Resolved, String> {
    let cam = &req.camera;
    let mut notes = Vec::new();
    let size: Vec<&str> = cam.camera_size.split(['x', 'X']).collect();
    let [w, h] = size.as_slice() else { return Err(format!("camera_size {:?}: not WIDTHxHEIGHT", cam.camera_size)) };
    let parse_int = |s: &str| s.trim().parse::<i64>().map_err(|_| format!("camera_size {:?}: not WIDTHxHEIGHT", cam.camera_size));
    let (width, height) = (parse_int(w)?, parse_int(h)?);
    let fps = py_float(&cam.camera_fps)?.trunc() as i64;
    let record_fps = py_float(&cam.record_fps)?;
    let controls = cam.v4l2_controls.clone();
    let calib_dir = PathBuf::from(expand_user(&cam.calib_dir));

    let mut intrinsics = if cam.intrinsics_file.is_empty() {
        calib_dir.join("camera_front.yaml")
    } else {
        PathBuf::from(expand_user(&cam.intrinsics_file))
    };
    let intrinsics_placeholder = !intrinsics.is_file();
    let mut placeholder = None;
    if intrinsics_placeholder {
        notes.push(format!(
            "找不到內參 {}：改用未標定佔位檔（check_run 會 FAIL，錄棋盤格時請加 --allow-uncalibrated）",
            intrinsics.display()
        ));
        let (path, text) = placeholder_intrinsics(width, height);
        intrinsics = path.clone();
        placeholder = Some((path, text));
    }

    let mut ext_path = if cam.extrinsics_file.is_empty() {
        calib_dir.join("extrinsics.yaml")
    } else {
        PathBuf::from(expand_user(&cam.extrinsics_file))
    };
    if !ext_path.is_file() {
        notes.push(format!("找不到外參 {}：改用套件內的佔位值", ext_path.display()));
        ext_path = share_config.join("camera_front_extrinsics.yaml");
    }
    let ext_text = std::fs::read_to_string(&ext_path).map_err(|e| format!("{}: {e}", ext_path.display()))?;
    let ext_doc = match mower_pyfmt::yaml::safe_load(&ext_text).map_err(|e| format!("{}: {e}", ext_path.display()))? {
        Py::None => Py::Dict(Vec::new()),
        d => d,
    };
    let reference = match urdf.and_then(|u| extrinsics::urdf_reference(u)) {
        Ok(r) => r,
        Err(e) => {
            notes.push(format!("URDF 讀取失敗，改用內建幾何：{e}"));
            extrinsics::default_reference()
        }
    };
    let result = extrinsics::compute(&ext_doc, Some(&reference)).map_err(|e| format!("{}: {e}", ext_path.display()))?;
    notes.extend(result.warnings.iter().cloned());

    let pipeline = if cam.gscam_pipeline.is_empty() {
        build_pipeline(&cam.camera_device, width, height, fps, record_fps, &controls)
    } else {
        cam.gscam_pipeline.clone()
    };
    let autofocus = if controls.contains("focus_automatic_continuous=0") || controls.contains("focus_auto=0") {
        Py::str("off")
    } else {
        Py::None
    };
    let fps_recorded = if record_fps.fract() == 0.0 && record_fps.is_finite() {
        Py::Int(record_fps as i64)
    } else {
        Py::Float(record_fps)
    };
    let camera_params = Py::dict([
        ("device", Py::str(&cam.camera_device)),
        ("width", Py::Int(width)),
        ("height", Py::Int(height)),
        ("fps_native", Py::Int(fps)),
        ("fps_recorded", fps_recorded),
        ("format", Py::str("jpeg")),
        ("autofocus", autofocus),
        // what runs the camera: gscam (the Python launch's, and E5-5's
        // camera-only compose) or the module's GStreamer instance (E5-6)
        ("driver", Py::str(&cam.driver)),
        ("pipeline", Py::str(&pipeline)),
    ]);
    let derived = Py::dict([
        (
            "summary",
            Py::dict([
                ("source", Py::str(&result.source)),
                ("camera_height_m", Py::Float(result.camera_height_m)),
                ("camera_pitch_deg", Py::Float(result.camera_pitch_deg)),
            ]),
        ),
        ("block", extrinsics::derived_block(&result)),
    ]);
    let dumps = |v: &Py| mower_pyfmt::json::dumps(v, &JsonOptions::default()).map_err(|e| e.0);

    let mut session = PathBuf::from(expand_user(&req.session_file));
    if !session.is_file() {
        notes.push(format!(
            "找不到 {}，run_metadata 使用範本 config/data_collection_session.yaml（草高、天氣等要手動補）",
            session.display()
        ));
        session = share_config.join("data_collection_session.yaml");
    }
    Ok(Resolved {
        dc: DcParams {
            session_file: session.to_string_lossy().into_owned(),
            run_info: req.run_info.clone(),
            camera_params: dumps(&camera_params)?,
            intrinsics_file: intrinsics.to_string_lossy().into_owned(),
            intrinsics_placeholder,
            extrinsics_file: ext_path.to_string_lossy().into_owned(),
            extrinsics_derived: dumps(&derived)?,
        },
        intrinsics: intrinsics.clone(),
        pipeline,
        placeholder,
        notes,
        extrinsics: result,
    })
}

/// The robot's reference geometry, for a caller without the URDF: what
/// [`resolve`] falls back to.
pub fn default_reference() -> Reference {
    extrinsics::default_reference()
}
