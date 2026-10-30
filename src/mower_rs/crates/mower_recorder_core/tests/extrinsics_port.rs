//! `test/test_camera_extrinsics.py`, one for one: the optical-frame axes,
//! the base_link chain, the URDF reference. The placeholder YAML and
//! robot.urdf.xacro come from the vectors (what PyYAML loaded, the file).

mod common;

use common::*;
use mower_pyfmt::Py;
use mower_recorder_core::extrinsics as ce;

fn placeholder() -> Py {
    decode(&vectors()["values"]["placeholder"])
}

fn set(d: &mut Py, path: &[&str], v: Py) {
    let Py::Dict(items) = d else { panic!("not a dict") };
    let slot = items.iter_mut().find(|(k, _)| k == path[0]);
    if path.len() == 1 {
        match slot {
            Some(s) => s.1 = v,
            None => items.push((path[0].into(), v)),
        }
    } else {
        set(&mut slot.unwrap().1, &path[1..], v);
    }
}

fn remove(d: &mut Py, path: &[&str]) {
    let Py::Dict(items) = d else { panic!("not a dict") };
    if path.len() == 1 {
        items.retain(|(k, _)| k != path[0]);
    } else {
        remove(&mut items.iter_mut().find(|(k, _)| k == path[0]).unwrap().1, &path[1..]);
    }
}

fn close(a: &[f64], b: &[f64], tol: f64) -> bool {
    a.iter().zip(b).all(|(x, y)| (x - y).abs() < tol)
}

#[test]
fn test_shipped_placeholder_is_valid_but_flagged() {
    let (errors, warnings) = ce::validate(&placeholder());
    assert!(errors.is_empty());
    assert!(warnings.iter().any(|w| w.contains("placeholder")));
}

#[test]
fn test_optical_axes_for_downward_pitch() {
    let mut ext = placeholder();
    set(&mut ext, &["camera", "pitch_deg"], Py::Float(20.0));
    set(&mut ext, &["camera", "roll_deg"], Py::Float(0.0));
    set(&mut ext, &["camera", "yaw_deg"], Py::Float(0.0));
    let res = ce::compute(&ext, None).unwrap();
    let t = res.base_footprint_to_optical;
    let th = ce::radians(20.0);
    assert!(close(&t.rotate([0.0, 0.0, 1.0]), &[th.cos(), 0.0, -th.sin()], 1e-6));
    assert!(close(&t.rotate([1.0, 0.0, 0.0]), &[0.0, -1.0, 0.0], 1e-6));
    assert!(close(&t.rotate([0.0, 1.0, 0.0]), &[-th.sin(), 0.0, -th.cos()], 1e-6));
    assert_eq!(res.camera_height_m, 0.62);
    assert_eq!(res.camera_pitch_deg, 20.0);
}

#[test]
fn test_height_uses_axle_reference() {
    let r = ce::default_reference();
    let res = ce::compute(&placeholder(), Some(&r)).unwrap();
    let axle = r.axle_center_in_base_footprint;
    let [x, y, z] = res.base_footprint_to_camera_link.translation;
    assert!((x - (axle[0] + 0.35)).abs() < 1e-9);
    assert!((y - axle[1]).abs() < 1e-9);
    // z in base_footprint = height above ground - base_footprint height
    assert!((z - (0.62 - (0.09 - axle[2]))).abs() < 1e-9);
    assert!((res.base_footprint_height_m - (0.09 - axle[2])).abs() < 1e-9);
}

#[test]
fn test_base_link_chain_composes_back() {
    let mut ext = placeholder();
    set(&mut ext, &["camera", "roll_deg"], Py::Float(2.0));
    set(&mut ext, &["camera", "yaw_deg"], Py::Float(-3.0));
    let r = ce::default_reference();
    let res = ce::compute(&ext, Some(&r)).unwrap();
    let chained = r.base_link_in_base_footprint.compose(&res.base_link_to_camera_link);
    let direct = res.base_footprint_to_camera_link;
    assert!(close(&chained.translation, &direct.translation, 1e-6));
    for i in 0..3 {
        assert!(close(&chained.rotation[i], &direct.rotation[i], 1e-6));
    }
}

#[test]
#[allow(clippy::approx_constant)] // the URDF's 1.5708, as the Python test
fn test_real_urdf_reference() {
    let r = ce::urdf_reference(vectors()["texts"]["robot_urdf_xacro"].as_str().unwrap()).unwrap();
    assert!(close(&r.base_link_in_base_footprint.rpy(), &[0.0, 0.0, 1.5708], 1e-6));
    let axle = r.axle_center_in_base_footprint;
    // drive axle ~0.166 m behind base_footprint, ~3 mm below it (P-098)
    assert!((axle[0] - -0.165534).abs() < 1e-5);
    assert!((axle[1] - 0.01453).abs() < 1e-5);
    assert!((axle[2] - -0.00295275).abs() < 1e-6);
    assert!(close(&axle, &ce::default_reference().axle_center_in_base_footprint, 1e-5));
}

#[test]
fn test_explicit_axle_overrides_urdf() {
    let mut ext = placeholder();
    set(&mut ext, &["reference", "axle_center_in_base_footprint_m"], Py::List(vec![Py::Float(0.0); 3]));
    let res = ce::compute(&ext, None).unwrap();
    assert!((res.base_footprint_to_camera_link.translation[0] - 0.35).abs() < 1e-9);
}

#[test]
fn test_rpy_quaternion_round_trip() {
    for rpy in [[0.1, -0.4, 2.0], [-1.2, 0.3, -2.9], [0.0, 1.2, 0.5]] {
        let m = ce::rpy_to_matrix(rpy[0], rpy[1], rpy[2]);
        assert!(close(&ce::matrix_to_rpy(&m), &rpy, 1e-9));
        let m2 = ce::quaternion_to_matrix(ce::matrix_to_quaternion(&m));
        for i in 0..3 {
            assert!(close(&m[i], &m2[i], 1e-9));
        }
    }
}

#[test]
fn test_validation_errors() {
    let mut ext = placeholder();
    set(&mut ext, &["camera", "pitch_deg"], Py::Int(95));
    set(&mut ext, &["source"], Py::str("guess"));
    remove(&mut ext, &["camera", "height_m"]);
    let (errors, _) = ce::validate(&ext);
    assert!(errors.len() >= 2);
    assert!(matches!(ce::compute(&ext, None), Err(ce::ComputeError::Invalid(_))));
}

#[test]
fn test_static_tf_arguments_and_derived_block() {
    let res = ce::compute(&placeholder(), None).unwrap();
    let args = ce::static_tf_arguments(&res.camera_link_to_optical, ce::CAMERA_LINK, ce::OPTICAL_FRAME);
    assert_eq!(args[args.len() - 4..], ["--frame-id", ce::CAMERA_LINK, "--child-frame-id", ce::OPTICAL_FRAME]);
    let q: Vec<f64> = [7, 9, 11, 13].iter().map(|&i| args[i].parse().unwrap()).collect();
    assert!(close(&q, &[-0.5, 0.5, -0.5, 0.5], 1e-9));
    let block = ce::derived_block(&res);
    let opt = block.get("base_footprint_to_camera_front_optical_frame").unwrap();
    assert_eq!(opt.get("parent"), Some(&Py::str("base_footprint")));
    let Some(Py::List(published)) = block.get("published_tf") else { panic!() };
    let parents: Vec<&Py> = published.iter().map(|t| t.get("parent").unwrap()).collect();
    assert_eq!(parents, [&Py::str("base_link"), &Py::str("camera_front_link")]);
    // plain types only
    mower_pyfmt::yaml::safe_dump(&block, &Default::default());
}

#[test]
fn python_float_strings() {
    for (s, want) in [("1_000.5", Some(1000.5)), (" 0.25 ", Some(0.25)), ("-inf", Some(f64::NEG_INFINITY)), ("1__0", None),
        ("_1", None), ("--1", None), ("1e3", Some(1000.0)), (".5", Some(0.5)), ("x", None), ("", None)] {
        let got = ce::parse_py_float(s);
        assert_eq!(got.map(f64::to_bits), want.map(f64::to_bits), "{s:?}");
    }
}
