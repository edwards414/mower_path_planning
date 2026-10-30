//! The data_collection profile's run metadata (spec 1.6 of the GrassVision
//! pipeline) and the calibration copied into `<run>/calib`: a port of
//! `onoff`, `_deep_merge`, `build_run_metadata` and `copy_calibration` of
//! mower_recorder's `record_profile.py` (lines 196-317).
//!
//! Values are `mower_pyfmt::Py` dicts with Python's dict semantics: setting
//! a key that is there keeps its place, a new key goes last. The YAML
//! loader is the caller's (`load`), so this module has no parser of its own.

use std::path::Path;

use mower_pyfmt::{yaml, Py};

use crate::extrinsics::{py_str, str_repr, truthy};

pub const INTRINSICS_NAME: &str = "camera_front.yaml";
pub const EXTRINSICS_NAME: &str = "extrinsics.yaml";

/// The operator fields of spec 1.6, written as null when missing
/// (record_profile.py:65-70).
pub const SESSION_FIELDS: [&str; 5] = ["grass_height_cm", "weather", "blade", "heading_calibration", "notes"];
pub const ONOFF_FIELDS: [&str; 1] = ["blade"];
pub const CAMERA_ONOFF_FIELDS: [&str; 1] = ["autofocus"];

/// The camera block's leading keys, in order (record_profile.py:241-242).
const CAMERA_KEYS: [&str; 10] =
    ["model", "serial", "device", "width", "height", "fps_native", "fps_recorded", "format", "autofocus", "exposure"];

// ---- dict operations with Python's semantics --------------------------------------------

fn items(v: &Py) -> &[(String, Py)] {
    match v {
        Py::Dict(items) => items,
        _ => &[],
    }
}

/// `d[k] = v`.
fn set(d: &mut Vec<(String, Py)>, k: &str, v: Py) {
    match d.iter_mut().find(|(key, _)| key == k) {
        Some(slot) => slot.1 = v,
        None => d.push((k.to_string(), v)),
    }
}

/// `d.pop(k, None)`.
fn pop(d: &mut Vec<(String, Py)>, k: &str) -> Py {
    match d.iter().position(|(key, _)| key == k) {
        Some(i) => d.remove(i).1,
        None => Py::None,
    }
}

fn get<'a>(d: &'a [(String, Py)], k: &str) -> Option<&'a Py> {
    d.iter().find(|(key, _)| key == k).map(|(_, v)| v)
}

/// `x or {}` for an optional dict argument.
fn or_empty(v: Option<&Py>) -> Result<Vec<(String, Py)>, String> {
    match v {
        None => Ok(Vec::new()),
        Some(v) if !truthy(v) => Ok(Vec::new()),
        Some(Py::Dict(items)) => Ok(items.clone()),
        Some(other) => Err(format!("a mapping is needed, not {}", crate::extrinsics::type_name(other))),
    }
}

/// `onoff(value)`: YAML 1.1 reads an unquoted off/on as a bool (P-099); the
/// metadata stores the words.
pub fn onoff(value: &Py) -> Py {
    match value {
        Py::Bool(b) => Py::str(if *b { "on" } else { "off" }),
        Py::None => Py::None,
        other => Py::Str(py_str(other)),
    }
}

/// `_deep_merge(base, extra)`: `extra` over `base`, dicts merged key by key.
pub fn deep_merge(base: &Py, extra: &Py) -> Py {
    let mut out: Vec<(String, Py)> = items(base).to_vec();
    for (k, v) in items(extra) {
        let merged = match (v, get(&out, k)) {
            (Py::Dict(_), Some(old @ Py::Dict(_))) => deep_merge(old, v),
            _ => v.clone(),
        };
        set(&mut out, k, merged);
    }
    Py::Dict(out)
}

/// The keyword arguments of `build_run_metadata` after `base_meta` and
/// `profile`; `None` is the Python's default.
#[derive(Debug, Clone, Copy, Default)]
pub struct MetadataInputs<'a> {
    /// the operator YAML (grass height, weather, camera model ...)
    pub session: Option<&'a Py>,
    /// the per-run overrides from the launch (the same keys; they win)
    pub run_info: Option<&'a Py>,
    /// what the camera launch used (device, size, fps)
    pub camera_params: Option<&'a Py>,
    /// the extrinsics summary (`source`, `camera_height_m`, `camera_pitch_deg`)
    pub extrinsics: Option<&'a Py>,
    /// what [`copy_calibration`] returned
    pub calibration: Option<&'a Py>,
    pub camera_topics: Option<&'a [String]>,
}

/// `build_run_metadata(base_meta, profile, ...)`: the run metadata of a
/// profile run. `env` is `os.environ.get` (`MOWER_VERSION`,
/// `MOWER_GIT_SHA`, `MOWER_IMAGE`). The error is what the Python would
/// raise on inputs that are not mappings.
pub fn build_run_metadata(
    base_meta: &Py,
    profile: &str,
    inputs: &MetadataInputs,
    env: &dyn Fn(&str) -> Option<String>,
) -> Result<Py, String> {
    let user = deep_merge(&Py::Dict(or_empty(inputs.session)?), &Py::Dict(or_empty(inputs.run_info)?));
    let user = items(&user).to_vec();
    let mut meta: Vec<(String, Py)> = items(base_meta).to_vec();
    set(&mut meta, "profile", Py::str(profile));

    // cam.update(user.get('camera') or {}); camera_params fill what is missing
    let mut cam: Vec<(String, Py)> = or_empty(get(&user, "camera"))?;
    for (k, v) in or_empty(inputs.camera_params)? {
        if get(&cam, &k).is_none() {
            cam.push((k, v));
        }
    }
    for k in CAMERA_ONOFF_FIELDS {
        if let Some(v) = get(&cam, k).cloned() {
            set(&mut cam, k, onoff(&v));
        }
    }
    let mut ordered: Vec<(String, Py)> = CAMERA_KEYS.iter().map(|k| (k.to_string(), pop(&mut cam, k))).collect();
    for (k, v) in cam {
        set(&mut ordered, &k, v);
    }
    set(&mut meta, "camera", Py::Dict(ordered));

    let ext = or_empty(inputs.extrinsics)?;
    let pick = |user_key: &str, ext_key: &str| -> Py {
        match get(&user, user_key) {
            Some(v) => v.clone(),
            None => get(&ext, ext_key).cloned().unwrap_or(Py::None),
        }
    };
    set(&mut meta, "camera_height_m", pick("camera_height_m", "camera_height_m"));
    set(&mut meta, "camera_pitch_deg", pick("camera_pitch_deg", "camera_pitch_deg"));
    set(&mut meta, "extrinsics_source", pick("extrinsics_source", "source"));

    // date = user.get('calibration_date') or calib.pop('date', None): the pop
    // happens only when the user value is falsy, and a date left in calib
    // then replaces the str() below through **calib
    let mut calib = or_empty(inputs.calibration)?;
    let user_date = get(&user, "calibration_date").cloned().unwrap_or(Py::None);
    let date = if truthy(&user_date) { user_date } else { pop(&mut calib, "date") };
    let mut calibration = vec![
        ("intrinsics_file".to_string(), pop(&mut calib, "intrinsics_file")),
        ("extrinsics_file".to_string(), pop(&mut calib, "extrinsics_file")),
        ("date".to_string(), if truthy(&date) { Py::Str(py_str(&date)) } else { Py::None }),
    ];
    for (k, v) in calib {
        set(&mut calibration, &k, v);
    }
    set(&mut meta, "calibration", Py::Dict(calibration));

    for k in SESSION_FIELDS {
        let v = get(&user, k).cloned().unwrap_or(Py::None);
        set(&mut meta, k, if ONOFF_FIELDS.contains(&k) { onoff(&v) } else { v });
    }
    if matches!(get(&meta, "notes"), None | Some(Py::None)) {
        set(&mut meta, "notes", Py::str(""));
    }
    if let Some(topics) = inputs.camera_topics.filter(|t| !t.is_empty()) {
        set(&mut meta, "camera_topics", Py::List(topics.iter().map(|t| Py::str(t)).collect()));
    }
    let e = |k: &str| Py::Str(env(k).unwrap_or_default());
    set(&mut meta, "build", Py::dict([("version", e("MOWER_VERSION")), ("git_sha", e("MOWER_GIT_SHA")), ("image", e("MOWER_IMAGE"))]));
    let extra: Vec<(String, Py)> = user
        .iter()
        .filter(|(k, _)| get(&meta, k).is_none() && k != "camera" && k != "calibration_date")
        .cloned()
        .collect();
    if !extra.is_empty() {
        set(&mut meta, "session_extra", Py::Dict(extra));
    }
    Ok(Py::Dict(meta))
}

/// `copy_calibration(run_dir, intrinsics_path, extrinsics_path, derived,
/// intrinsics_placeholder)`: copies the intrinsics into
/// `<run>/calib/camera_front.yaml` byte for byte, and writes the extrinsics
/// as `<run>/calib/extrinsics.yaml` with a comment line and the `derived`
/// block, as `yaml.safe_dump(ext, allow_unicode=True, sort_keys=False)`.
/// `load` is `yaml.safe_load` of the file's text. Missing files are
/// reported in `problems`, not errors. Returns the Python's info dict.
pub fn copy_calibration(
    run_dir: &Path,
    intrinsics_path: &str,
    extrinsics_path: &str,
    derived: Option<&Py>,
    intrinsics_placeholder: bool,
    load: &dyn Fn(&str) -> Result<Py, String>,
) -> Result<Py, String> {
    let calib_dir = run_dir.join("calib");
    std::fs::create_dir_all(&calib_dir).map_err(|e| e.to_string())?;
    let mut info: Vec<(String, Py)> = vec![
        ("intrinsics_file".into(), Py::None),
        ("extrinsics_file".into(), Py::None),
        ("intrinsics_source".into(), Py::None),
        ("problems".into(), Py::List(Vec::new())),
    ];
    let mut problems = Vec::new();
    if !intrinsics_path.is_empty() && Path::new(intrinsics_path).is_file() {
        std::fs::copy(intrinsics_path, calib_dir.join(INTRINSICS_NAME)).map_err(|e| e.to_string())?;
        set(&mut info, "intrinsics_file", Py::Str(format!("calib/{INTRINSICS_NAME}")));
        set(&mut info, "intrinsics_source", Py::str(if intrinsics_placeholder { "placeholder" } else { "calibrated" }));
    } else {
        problems.push(Py::Str(format!("intrinsics not found: {}", str_repr(intrinsics_path))));
    }
    if !extrinsics_path.is_empty() && Path::new(extrinsics_path).is_file() {
        let text = std::fs::read_to_string(extrinsics_path).map_err(|e| e.to_string())?;
        let loaded = load(&text)?;
        let mut ext = if truthy(&loaded) { loaded } else { Py::Dict(Vec::new()) };
        if let Some(d) = derived.filter(|d| truthy(d)) {
            match &mut ext {
                Py::Dict(items) => set(items, "derived", d.clone()),
                other => return Err(format!("'{}' object does not support item assignment", crate::extrinsics::type_name(other))),
            }
        }
        let base = Path::new(extrinsics_path).file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
        let mut out = format!("# 由 mower_recorder 在開錄時從 {base} 複製，並加上 derived 區塊\n");
        out.push_str(&yaml::safe_dump(&ext, &yaml::YamlOptions { allow_unicode: true, sort_keys: false }));
        std::fs::write(calib_dir.join(EXTRINSICS_NAME), out).map_err(|e| e.to_string())?;
        set(&mut info, "extrinsics_file", Py::Str(format!("calib/{EXTRINSICS_NAME}")));
        if let Py::Dict(items) = &ext {
            if let Some(d) = get(items, "date").filter(|d| truthy(d)) {
                info.push(("date".into(), Py::Str(py_str(d))));
            }
        }
    } else {
        problems.push(Py::Str(format!("extrinsics not found: {}", str_repr(extrinsics_path))));
    }
    if problems.is_empty() {
        pop(&mut info, "problems");
    } else {
        set(&mut info, "problems", Py::List(problems));
    }
    Ok(Py::Dict(info))
}
