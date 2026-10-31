//! The calibration file as camera_info_manager loads it for gscam
//! (`camera_info_url: file://...`): camera_calibration_parsers'
//! `readCalibrationYml` (image_common, Jazzy):
//!
//! - `camera_name` (else `unknown`), `image_width`, `image_height`;
//! - `camera_matrix` (K, 3x3), `rectification_matrix` (R, 3x3),
//!   `projection_matrix` (P, 3x4): `rows` x `cols` values of `data`;
//! - `distortion_model` (else `plumb_bob`), `distortion_coefficients`:
//!   `rows * cols` values (D's length);
//! - `binning_x`, `binning_y` and `roi` when present, zero otherwise.
//!
//! A file it cannot read leaves camera_info_manager's default: every field
//! zero or empty (gscam then said "Using an uncalibrated config"); the
//! caller logs why.

use mower_pyfmt::Py;

/// `sensor_msgs/CameraInfo` without its header.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CameraInfo {
    pub height: u32,
    pub width: u32,
    pub distortion_model: String,
    pub d: Vec<f64>,
    pub k: [f64; 9],
    pub r: [f64; 9],
    pub p: [f64; 12],
    pub binning_x: u32,
    pub binning_y: u32,
    /// (x_offset, y_offset, height, width, do_rectify)
    pub roi: (u32, u32, u32, u32, bool),
}

fn number(v: &Py, what: &str) -> Result<f64, String> {
    match v {
        Py::Float(f) => Ok(*f),
        Py::Int(i) => Ok(*i as f64),
        other => Err(format!("{what}: not a number ({})", other.type_name())),
    }
}

fn uint(v: Option<&Py>, what: &str) -> Result<u32, String> {
    match v {
        Some(Py::Int(i)) if *i >= 0 && *i <= u32::MAX as i64 => Ok(*i as u32),
        Some(other) => Err(format!("{what}: not an unsigned integer ({})", other.type_name())),
        None => Err(format!("{what}: missing")),
    }
}

/// `rows` x `cols` values of a matrix node (yaml-cpp's `as<double>` of each).
fn matrix<const N: usize>(doc: &Py, key: &str, rows: i64, cols: i64) -> Result<[f64; N], String> {
    let node = doc.get(key).ok_or_else(|| format!("{key}: missing"))?;
    let (r, c) = (node.get("rows"), node.get("cols"));
    if r != Some(&Py::Int(rows)) || c != Some(&Py::Int(cols)) {
        return Err(format!("{key}: not {rows}x{cols}"));
    }
    let Some(Py::List(data)) = node.get("data") else { return Err(format!("{key}.data: not a list")) };
    let mut out = [0.0; N];
    for (i, slot) in out.iter_mut().enumerate() {
        *slot = number(data.get(i).ok_or_else(|| format!("{key}.data: short"))?, key)?;
    }
    Ok(out)
}

/// The calibration in `text` (the file's YAML) and its camera name.
pub fn parse(text: &str) -> Result<(String, CameraInfo), String> {
    let doc = mower_pyfmt::yaml::safe_load(text).map_err(|e| e.to_string())?;
    if !matches!(doc, Py::Dict(_)) {
        return Err("not a mapping".into());
    }
    // yaml-cpp's `as<std::string>` of a scalar; only a name that is a
    // string is kept (it only labels the manager)
    let name = match doc.get("camera_name") {
        Some(Py::Str(s)) => s.clone(),
        _ => "unknown".into(),
    };
    let mut info = CameraInfo {
        width: uint(doc.get("image_width"), "image_width")?,
        height: uint(doc.get("image_height"), "image_height")?,
        k: matrix(&doc, "camera_matrix", 3, 3)?,
        r: matrix(&doc, "rectification_matrix", 3, 3)?,
        p: matrix(&doc, "projection_matrix", 3, 4)?,
        distortion_model: match doc.get("distortion_model") {
            Some(Py::Str(s)) => s.clone(),
            Some(_) => return Err("distortion_model: not a string".into()),
            None => "plumb_bob".into(),
        },
        ..CameraInfo::default()
    };
    let dnode = doc.get("distortion_coefficients").ok_or("distortion_coefficients: missing")?;
    let rows = uint(dnode.get("rows"), "distortion_coefficients.rows")? as usize;
    let cols = uint(dnode.get("cols"), "distortion_coefficients.cols")? as usize;
    let Some(Py::List(data)) = dnode.get("data") else { return Err("distortion_coefficients.data: not a list".into()) };
    for i in 0..rows * cols {
        info.d.push(number(data.get(i).ok_or("distortion_coefficients.data: short")?, "distortion_coefficients")?);
    }
    if doc.get("binning_x").is_some() {
        info.binning_x = uint(doc.get("binning_x"), "binning_x")?;
    }
    if doc.get("binning_y").is_some() {
        info.binning_y = uint(doc.get("binning_y"), "binning_y")?;
    }
    if let Some(roi) = doc.get("roi") {
        info.roi = (
            uint(roi.get("x_offset"), "roi.x_offset")?,
            uint(roi.get("y_offset"), "roi.y_offset")?,
            uint(roi.get("height"), "roi.height")?,
            uint(roi.get("width"), "roi.width")?,
            matches!(roi.get("do_rectify"), Some(Py::Bool(true))),
        );
    }
    Ok((name, info))
}

/// [`parse`] of a file (`camera_info_url`'s path).
pub fn load(path: &std::path::Path) -> Result<(String, CameraInfo), String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    parse(&text).map_err(|e| format!("{}: {e}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// e2-verify's F3 calibration (record_run.sh), as PyYAML wrote it.
    const F3: &str = "image_width: 80\nimage_height: 60\ncamera_name: camera_front\ncamera_matrix:\n  rows: 3\n  cols: 3\n  \
        data:\n  - 65.75\n  - 0.0\n  - 40.125\n  - 0.0\n  - 65.875\n  - 29.75\n  - 0.0\n  - 0.0\n  - 1.0\n\
        distortion_model: plumb_bob\ndistortion_coefficients:\n  rows: 1\n  cols: 5\n  data:\n  - -0.31\n  - 0.11\n  \
        - 0.0012\n  - -0.0004\n  - 0.0\nrectification_matrix:\n  rows: 3\n  cols: 3\n  data:\n  - 1.0\n  - 0.0\n  - 0.0\n  \
        - 0.0\n  - 1.0\n  - 0.0\n  - 0.0\n  - 0.0\n  - 1.0\nprojection_matrix:\n  rows: 3\n  cols: 4\n  data:\n  - 55.1\n  \
        - 0.0\n  - 39.75\n  - 0.0\n  - 0.0\n  - 59.2\n  - 30.05\n  - 0.0\n  - 0.0\n  - 0.0\n  - 1.0\n  - 0.0\n";

    #[test]
    fn a_calibration_file_as_camera_info_manager_reads_it() {
        let (name, info) = parse(F3).unwrap();
        assert_eq!(name, "camera_front");
        assert_eq!((info.width, info.height), (80, 60));
        assert_eq!(info.k, [65.75, 0.0, 40.125, 0.0, 65.875, 29.75, 0.0, 0.0, 1.0]);
        assert_eq!(info.d, vec![-0.31, 0.11, 0.0012, -0.0004, 0.0]);
        assert_eq!(info.p[0], 55.1);
        assert_eq!(info.distortion_model, "plumb_bob");
        assert_eq!((info.binning_x, info.binning_y, info.roi), (0, 0, (0, 0, 0, 0, false)));
        // no distortion model: plumb_bob; no camera name: unknown
        let bare = F3.replace("distortion_model: plumb_bob\n", "").replace("camera_name: camera_front\n", "");
        let (name, info) = parse(&bare).unwrap();
        assert_eq!((name.as_str(), info.distortion_model.as_str()), ("unknown", "plumb_bob"));
        // binning and ROI when given
        let more = format!("{F3}binning_x: 2\nbinning_y: 2\nroi:\n  x_offset: 1\n  y_offset: 2\n  height: 3\n  width: 4\n  do_rectify: true\n");
        let (_, info) = parse(&more).unwrap();
        assert_eq!((info.binning_x, info.roi), (2, (1, 2, 3, 4, true)));
        assert!(parse("image_width: 80\n").is_err());
        assert!(parse(&F3.replace("rows: 3\n  cols: 4", "rows: 4\n  cols: 3")).is_err());
    }
}
