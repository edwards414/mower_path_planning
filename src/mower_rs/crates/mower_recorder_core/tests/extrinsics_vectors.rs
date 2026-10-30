//! Replays `tests/vectors/extrinsics.json`, which the real Python wrote in
//! the robot image (`tests/oracle/ext_gen.py`): every float bit for bit,
//! every text and YAML byte for byte.

mod common;

use common::*;
use mower_pyfmt::yaml::{safe_dump, YamlOptions};
use mower_pyfmt::Py;
use mower_recorder_core::dc_metadata::{self, MetadataInputs};
use mower_recorder_core::extrinsics::{self as ce, Computed, ComputeError, Reference, Transform};
use serde_json::Value;

fn dump(v: &Py) -> String {
    safe_dump(v, &YamlOptions { allow_unicode: true, sort_keys: false })
}

fn check_tr(what: &str, t: &Transform, want: &Value) {
    let r: Vec<f64> = t.rotation.iter().flatten().copied().collect();
    let wr = fs(&want["r"]);
    assert!(r.iter().zip(&wr).all(|(a, b)| bits_eq(*a, *b)), "{what} rotation\n ours {r:?}\n py   {wr:?}");
    let wt = fs(&want["t"]);
    assert!(t.translation.iter().zip(&wt).all(|(a, b)| bits_eq(*a, *b)), "{what} translation {:?} vs {wt:?}", t.translation);
    let wrpy = fs(&want["rpy"]);
    assert!(t.rpy().iter().zip(&wrpy).all(|(a, b)| bits_eq(*a, *b)), "{what} rpy {:?} vs {wrpy:?}", t.rpy());
    let wq = fs(&want["q"]);
    assert!(t.quaternion().iter().zip(&wq).all(|(a, b)| bits_eq(*a, *b)), "{what} quaternion {:?} vs {wq:?}", t.quaternion());
}

fn tr_of(want: &Value) -> Transform {
    let r = fs(&want["r"]);
    let t = fs(&want["t"]);
    Transform::new([[r[0], r[1], r[2]], [r[3], r[4], r[5]], [r[6], r[7], r[8]]], [t[0], t[1], t[2]])
}

fn check_ref(what: &str, r: &Reference, want: &Value) {
    check_tr(what, &r.base_link_in_base_footprint, &want["base_link_in_base_footprint"]);
    let a = fs(&want["axle"]);
    assert!(r.axle_center_in_base_footprint.iter().zip(&a).all(|(x, y)| bits_eq(*x, *y)), "{what} axle");
    assert_eq!(r.source, want["source"].as_str().unwrap(), "{what}");
}

#[test]
fn references() {
    let v = vectors();
    check_ref("default", &ce::default_reference(), &v["reference"]["default"]);
    let real = ce::urdf_reference(v["texts"]["real_robot_urdf"].as_str().unwrap()).unwrap();
    check_ref("urdf_real", &real, &v["reference"]["urdf_real"]);
    let xacro = ce::urdf_reference(v["texts"]["robot_urdf_xacro"].as_str().unwrap()).unwrap();
    check_ref("urdf_xacro", &xacro, &v["reference"]["urdf_xacro"]);
    for (i, c) in v["urdf_cases"].as_array().unwrap().iter().enumerate() {
        let got = ce::urdf_reference(c["xml"].as_str().unwrap());
        match c["error"].as_str() {
            Some(e) => {
                let err = got.expect_err(&format!("urdf case {i} must fail: {e}"));
                if let Some(m) = e.strip_prefix("ValueError: ") {
                    assert_eq!(err, m, "urdf case {i}");
                }
            }
            None => check_ref(&format!("urdf case {i}"), &got.unwrap(), &c["ref"]),
        }
    }
}

#[test]
fn matrix_helpers() {
    let v = vectors();
    for (i, c) in v["matrix"].as_array().unwrap().iter().enumerate() {
        let rpy = fs(&c["rpy"]);
        let m = ce::rpy_to_matrix(rpy[0], rpy[1], rpy[2]);
        let flat: Vec<f64> = m.iter().flatten().copied().collect();
        assert!(flat.iter().zip(fs(&c["m"])).all(|(a, b)| bits_eq(*a, b)), "#{i} rpy_to_matrix");
        assert!(ce::matrix_to_rpy(&m).iter().zip(fs(&c["rpy_back"])).all(|(a, b)| bits_eq(*a, b)), "#{i} matrix_to_rpy");
        let q = ce::matrix_to_quaternion(&m);
        assert!(q.iter().zip(fs(&c["q"])).all(|(a, b)| bits_eq(*a, b)), "#{i} matrix_to_quaternion");
        let mq: Vec<f64> = ce::quaternion_to_matrix(q).iter().flatten().copied().collect();
        assert!(mq.iter().zip(fs(&c["m_from_q"])).all(|(a, b)| bits_eq(*a, b)), "#{i} quaternion_to_matrix");
        let (t1, t2) = (tr_of(&c["t1"]), tr_of(&c["t2"]));
        check_tr(&format!("#{i} t1"), &t1, &c["t1"]);
        check_tr(&format!("#{i} t1 @ t2"), &t1.compose(&t2), &c["compose"]);
        check_tr(&format!("#{i} inverse"), &t1.inverse(), &c["inverse"]);
        assert!(t1.apply([0.3, -0.2, 0.1]).iter().zip(fs(&c["apply"])).all(|(a, b)| bits_eq(*a, b)), "#{i} apply");
        assert!(t1.rotate([0.0, 0.0, 1.0]).iter().zip(fs(&c["rotate"])).all(|(a, b)| bits_eq(*a, b)), "#{i} rotate");
    }
}

#[test]
fn round_is_pythons() {
    let v = vectors();
    let rounds = v["rounds"].as_array().unwrap();
    for r in rounds {
        let (x, n, want) = (f(&r[0]), r[1].as_u64().unwrap() as usize, f(&r[2]));
        let got = ce::py_round(x, n);
        assert!(bits_eq(got, want), "round({}, {n}) = {} in Python, {} here", r[0], r[2], mower_pyfmt::repr::float(got));
    }
    assert!(rounds.len() > 2000);
}

fn check_result(id: &str, res: &Computed, want: &Value) {
    assert_eq!(res.source, want["source"].as_str().unwrap(), "{id}");
    assert_eq!(res.warnings, strings(&want["warnings"]), "{id}");
    assert_eq!(res.reference_source, want["reference_source"].as_str().unwrap(), "{id}");
    assert!(res.axle_center_in_base_footprint.iter().zip(fs(&want["axle"])).all(|(a, b)| bits_eq(*a, b)), "{id} axle");
    for (k, x) in [
        ("base_footprint_height_m", res.base_footprint_height_m),
        ("camera_height_m", res.camera_height_m),
        ("camera_pitch_deg", res.camera_pitch_deg),
    ] {
        assert!(bits_eq(x, f(&want[k])), "{id} {k}: {x:?} vs {}", want[k]);
    }
    check_tr(&format!("{id} bf->cam"), &res.base_footprint_to_camera_link, &want["base_footprint_to_camera_link"]);
    check_tr(&format!("{id} bl->cam"), &res.base_link_to_camera_link, &want["base_link_to_camera_link"]);
    check_tr(&format!("{id} cam->opt"), &res.camera_link_to_optical, &want["camera_link_to_optical"]);
    check_tr(&format!("{id} bf->opt"), &res.base_footprint_to_optical, &want["base_footprint_to_optical"]);
}

#[test]
fn validate_and_compute_are_pythons() {
    let v = vectors();
    let real = ce::urdf_reference(v["texts"]["real_robot_urdf"].as_str().unwrap()).unwrap();
    let (mut computed, mut failed) = (0, 0);
    for c in v["compute"].as_array().unwrap() {
        let id = c["id"].as_str().unwrap();
        let ext = decode(&c["ext"]);
        let (errors, warnings) = ce::validate(&ext);
        assert_eq!(errors, strings(&c["validate"][0]), "{id} errors");
        assert_eq!(warnings, strings(&c["validate"][1]), "{id} warnings");
        let reference = if c["ref"] == "urdf_real" { Some(&real) } else { None };
        let got = ce::compute(&ext, reference);
        if c["error"].is_null() {
            let res = got.unwrap_or_else(|e| panic!("{id}: {e}"));
            check_result(id, &res, &c["result"]);
            let block = ce::derived_block(&res);
            assert_eq!(dump(&block), c["derived_yaml"].as_str().unwrap(), "{id} derived_block");
            for (k, t, p, ch) in [
                ("base_link_to_camera_link", &res.base_link_to_camera_link, ce::BASE_LINK, ce::CAMERA_LINK),
                ("camera_link_to_optical", &res.camera_link_to_optical, ce::CAMERA_LINK, ce::OPTICAL_FRAME),
                ("base_footprint_to_optical", &res.base_footprint_to_optical, ce::BASE_FOOTPRINT, ce::OPTICAL_FRAME),
            ] {
                assert_eq!(ce::static_tf_arguments(t, p, ch), strings(&c["static_tf"][k]), "{id} static_tf {k}");
            }
            computed += 1;
        } else {
            let (kind, msg) = (c["error"][0].as_str().unwrap(), c["error"][1].as_str().unwrap());
            match got.expect_err(id) {
                ComputeError::Invalid(m) => assert_eq!((kind, m.as_str()), ("ValueError", msg), "{id}"),
                ComputeError::Conversion { kind: k, message } => assert_eq!((k, message.as_str()), (kind, msg), "{id}"),
            }
            failed += 1;
        }
    }
    assert!(computed > 50 && failed > 20, "{computed} computed, {failed} failed");
}

#[test]
fn copy_calibration_is_pythons() {
    let v = vectors();
    let tmp = std::env::temp_dir().join(format!("mower_recorder_core_cc_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&tmp);
    for (n, c) in v["copy_calibration"].as_array().unwrap().iter().enumerate() {
        let id = c["id"].as_str().unwrap();
        let dir = tmp.join(n.to_string());
        std::fs::create_dir_all(dir.join("src")).unwrap();
        let run = dir.join("run");
        std::fs::create_dir_all(&run).unwrap();
        let ipath = match c["intrinsics"].as_str() {
            Some(text) => {
                let p = dir.join("src").join("cam_intrinsics.yaml");
                std::fs::write(&p, text).unwrap();
                p.to_string_lossy().into_owned()
            }
            None => String::new(),
        };
        let epath = match c["extrinsics_text"].as_str() {
            Some(text) => {
                let p = dir.join("src").join(c["extrinsics_name"].as_str().unwrap());
                std::fs::write(&p, text).unwrap();
                p.to_string_lossy().into_owned()
            }
            None => c["missing_extrinsics_path"].as_str().unwrap().to_string(),
        };
        let loaded = decode(&c["extrinsics_loaded"]);
        let want_text = c["extrinsics_text"].as_str().map(str::to_string);
        let load = |text: &str| -> Result<Py, String> {
            assert_eq!(Some(text), want_text.as_deref(), "{id}: the loader got another text");
            Ok(loaded.clone())
        };
        let derived = decode(&c["derived"]);
        let info = dc_metadata::copy_calibration(&run, &ipath, &epath, Some(&derived), c["placeholder"].as_bool().unwrap(), &load)
            .unwrap_or_else(|e| panic!("{id}: {e}"));
        assert!(same(&info, &decode(&c["info"])), "{id} info\n ours {info:?}\n py   {:?}", decode(&c["info"]));
        for name in ["camera_front.yaml", "extrinsics.yaml"] {
            let got = std::fs::read_to_string(run.join("calib").join(name)).ok();
            assert_eq!(got.as_deref(), c["files"][name].as_str(), "{id} {name}");
        }
    }
    std::fs::remove_dir_all(&tmp).unwrap();
}

#[test]
fn build_run_metadata_is_pythons() {
    let v = vectors();
    let opt = |c: &Value, k: &str| -> Option<Py> { c.get(k).map(decode).filter(|p| *p != Py::None) };
    for c in v["metadata"].as_array().unwrap() {
        let id = c["id"].as_str().unwrap();
        let (session, run_info, camera, ext, calib) =
            (opt(c, "session"), opt(c, "run_info"), opt(c, "camera_params"), opt(c, "extrinsics"), opt(c, "calibration"));
        let topics: Option<Vec<String>> = opt(c, "camera_topics").map(|p| match p {
            Py::List(items) => items.into_iter().map(|t| if let Py::Str(s) = t { s } else { panic!() }).collect(),
            _ => panic!(),
        });
        let inputs = MetadataInputs {
            session: session.as_ref(),
            run_info: run_info.as_ref(),
            camera_params: camera.as_ref(),
            extrinsics: ext.as_ref(),
            calibration: calib.as_ref(),
            camera_topics: topics.as_deref(),
        };
        let env = c["env"].clone();
        let meta = dc_metadata::build_run_metadata(&decode(&c["base"]), c["profile"].as_str().unwrap(), &inputs, &|k| {
            env.get(k).and_then(|v| v.as_str()).map(str::to_string)
        })
        .unwrap_or_else(|e| panic!("{id}: {e}"));
        assert_eq!(dump(&meta), c["yaml"].as_str().unwrap(), "{id}");
    }
}

#[test]
fn the_f3_run_s_calib_and_metadata() {
    let v = vectors();
    let f3 = &v["f3"];
    let ext = decode(&f3["ext"]);
    let real = ce::urdf_reference(v["texts"]["real_robot_urdf"].as_str().unwrap()).unwrap();
    let res = ce::compute(&ext, Some(&real)).unwrap();
    let derived = decode(&f3["derived"]);
    assert!(same(&ce::derived_block(&res), derived.get("block").unwrap()), "the F3 derived block");
    let tmp = std::env::temp_dir().join(format!("mower_recorder_core_f3_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&tmp);
    std::fs::create_dir_all(&tmp).unwrap();
    let src = tmp.join("extrinsics.yaml");
    std::fs::write(&src, "(the loader below stands for yaml.safe_load)").unwrap();
    let block = derived.get("block").unwrap().clone();
    let info = dc_metadata::copy_calibration(&tmp.join("run"), "", &src.to_string_lossy(), Some(&block), false, &|_| Ok(ext.clone())).unwrap();
    let written = std::fs::read_to_string(tmp.join("run/calib/extrinsics.yaml")).unwrap();
    assert_eq!(written, f3["calib_extrinsics_yaml"].as_str().unwrap(), "the F3 run's calib/extrinsics.yaml");
    assert_eq!(info.get("date"), Some(&Py::str("2026-10-30")));
    let meta = v["metadata"].as_array().unwrap().iter().find(|c| c["id"] == "f3_run").unwrap();
    assert_eq!(meta["yaml"], f3["run_metadata_yaml"], "the vectors' F3 case is the run's own file");
    std::fs::remove_dir_all(&tmp).unwrap();
}

#[test]
fn onoff_is_pythons() {
    let v = vectors();
    for c in v["onoff"].as_array().unwrap() {
        assert!(same(&dc_metadata::onoff(&decode(&c[0])), &decode(&c[1])), "{c}");
    }
}
