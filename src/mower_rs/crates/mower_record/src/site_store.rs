//! Named zone-set "sites": WGS84-anchored snapshots of recorded objects
//! (port of `utils/site_store.py`; same files, same JSON layout, same local
//! equirectangular projection as the app's GeoAnchor).
//!
//! Files: `<sites_dir>/<name>.json` (UTF-8, indent 2, non-ASCII kept) and the
//! manifest `<sites_dir>/.active_site` = `{"version": 1, "active": name}`.
//! Every write goes through a temp file + rename with fsync of the file and
//! the directory, so power loss cannot truncate the only copy.

use std::fs;
use std::path::{Path, PathBuf};

use serde_json::{json, Map, Value};

pub const M_PER_DEG_LAT: f64 = 111_320.0;
const ACTIVE_SITE_FILE: &str = ".active_site";

#[derive(Debug, Clone, PartialEq)]
pub struct Datum {
    pub lat: f64,
    pub lon: f64,
    pub bearing_rad: f64,
    pub source: String,
}

impl Datum {
    pub fn to_json(&self) -> Value {
        json!({"lat": self.lat, "lon": self.lon, "bearing_rad": self.bearing_rad, "source": self.source})
    }

    /// Parse the `/adapter/map_datum` document; `Err` for anything invalid.
    pub fn from_map_datum(doc: &Value) -> Result<Datum, String> {
        let num = |k: &str| doc.get(k).and_then(|v| v.as_f64()).ok_or_else(|| format!("missing {k}"));
        let lat = num("origin_lat")?;
        let lon = num("origin_lon")?;
        let bearing_rad = doc.get("bearing_rad").and_then(|v| v.as_f64()).unwrap_or(0.0);
        if !(lat.is_finite() && lon.is_finite() && bearing_rad.is_finite())
            || !(-90.0..=90.0).contains(&lat)
            || !(-180.0..=180.0).contains(&lon)
        {
            return Err("datum contains invalid coordinates".into());
        }
        Ok(Datum { lat, lon, bearing_rad, source: doc.get("source").and_then(|v| v.as_str()).unwrap_or("").to_string() })
    }
}

/// Wrap a longitude difference into [-180, 180).
fn wrap_deg(deg: f64) -> f64 {
    (deg + 180.0).rem_euclid(360.0) - 180.0
}

/// Map-frame metres -> (lat, lon) using the given datum.
pub fn ll_from_xy(x: f64, y: f64, datum: &Datum) -> (f64, f64) {
    let b = datum.bearing_rad;
    let east = x * b.sin() - y * b.cos();
    let north = x * b.cos() + y * b.sin();
    let m_per_deg_lon = M_PER_DEG_LAT * datum.lat.to_radians().cos();
    let lat = datum.lat + north / M_PER_DEG_LAT;
    let lon = datum.lon + if m_per_deg_lon.abs() < 1e-9 { 0.0 } else { east / m_per_deg_lon };
    (lat, wrap_deg(lon))
}

/// (lat, lon) -> map-frame metres using the given datum.
pub fn xy_from_ll(lat: f64, lon: f64, datum: &Datum) -> (f64, f64) {
    let b = datum.bearing_rad;
    let m_per_deg_lon = M_PER_DEG_LAT * datum.lat.to_radians().cos();
    let east = wrap_deg(lon - datum.lon) * m_per_deg_lon;
    let north = (lat - datum.lat) * M_PER_DEG_LAT;
    (east * b.sin() + north * b.cos(), -east * b.cos() + north * b.sin())
}

/// 站名可用中文/emoji，只擋路徑字元與空名。
pub fn valid_name(name: &str) -> Option<String> {
    let name = name.trim();
    if name.is_empty() || name == "." || name == ".." || name.chars().any(|c| c == '\\' || c == '/' || (c as u32) < 0x20) {
        return None;
    }
    Some(name.to_string())
}

pub fn site_path(sites_dir: &str, name: &str) -> PathBuf {
    Path::new(sites_dir).join(format!("{name}.json"))
}

pub fn active_site_path(sites_dir: &str) -> PathBuf {
    Path::new(sites_dir).join(ACTIVE_SITE_FILE)
}

fn fsync_dir(dir: &Path) -> std::io::Result<()> {
    fs::File::open(dir)?.sync_all()
}

/// Write and atomically replace one durable JSON file (indent 2, UTF-8).
pub fn write_json_atomic(path: &Path, payload: &Value) -> std::io::Result<()> {
    let dir = path.parent().filter(|p| !p.as_os_str().is_empty()).map(Path::to_path_buf).unwrap_or_else(|| PathBuf::from("."));
    fs::create_dir_all(&dir)?;
    let tmp = PathBuf::from(format!("{}.tmp", path.to_string_lossy()));
    {
        let mut f = fs::File::create(&tmp)?;
        use std::io::Write;
        f.write_all(serde_json::to_string_pretty(payload).expect("json").as_bytes())?;
        f.flush()?;
        f.sync_all()?;
    }
    fs::rename(&tmp, path)?;
    fsync_dir(&dir)
}

fn read_json(path: &Path) -> Result<Value, String> {
    let text = fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    serde_json::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))
}

/// XY object as path_record keeps it in memory: `{id, ns, points: [[x, y]],
/// color?, scale?}`.
#[derive(Debug, Clone, PartialEq)]
pub struct XyObject {
    pub id: i32,
    pub ns: String,
    pub points: Vec<(f64, f64)>,
    pub color: Option<(f64, f64, f64, f64)>,
    pub scale: Option<f64>,
}

fn now_iso() -> String {
    // datetime.now(timezone.utc).isoformat(timespec='seconds')
    let secs = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs() as i64).unwrap_or(0);
    let days = secs.div_euclid(86_400);
    let sod = secs.rem_euclid(86_400);
    // civil from days (Howard Hinnant)
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = if m <= 2 { y + 1 } else { y };
    format!("{y:04}-{m:02}-{d:02}T{:02}:{:02}:{:02}+00:00", sod / 3600, (sod % 3600) / 60, sod % 60)
}

fn color_json(c: (f64, f64, f64, f64)) -> Value {
    json!({"r": c.0, "g": c.1, "b": c.2, "a": c.3})
}

/// Assemble a site document from XY objects, converting vertices to LL.
pub fn build_site(name: &str, datum: &Datum, zones: &[XyObject], risks: &[XyObject], channels: &[XyObject], created_at: Option<&str>) -> Value {
    let now = now_iso();
    let to_ll = |objs: &[XyObject], keep_extras: bool| -> Vec<Value> {
        objs.iter()
            .map(|o| {
                let mut entry = Map::new();
                entry.insert("id".into(), json!(o.id));
                entry.insert("ns".into(), json!(o.ns));
                entry.insert(
                    "points_ll".into(),
                    Value::Array(
                        o.points
                            .iter()
                            .map(|(x, y)| {
                                let (lat, lon) = ll_from_xy(*x, *y, datum);
                                json!([lat, lon])
                            })
                            .collect(),
                    ),
                );
                if keep_extras {
                    if let Some(c) = o.color {
                        entry.insert("color".into(), color_json(c));
                    }
                    if let Some(s) = o.scale {
                        entry.insert("scale".into(), json!(s));
                    }
                }
                Value::Object(entry)
            })
            .collect()
    };
    let area: f64 = zones.iter().map(|z| crate::geometry::polygon_area_m2(&z.points)).sum();
    json!({
        "version": 1,
        "name": name,
        "created_at": created_at.map(str::to_string).unwrap_or_else(|| now.clone()),
        "updated_at": now,
        "datum": datum.to_json(),
        "zones": to_ll(zones, false),
        "risk_zones": to_ll(risks, false),
        "channels": to_ll(channels, true),
        "meta": {
            "zone_count": zones.len(),
            "risk_count": risks.len(),
            "channel_count": channels.len(),
            "area_m2": (area * 10.0).round_ties_even() / 10.0,
            "datum_source": datum.source,
        },
    })
}

fn object_from_site(o: &Value, datum: &Datum, keep_extras: bool) -> Result<XyObject, String> {
    let id = o.get("id").and_then(|v| v.as_i64()).ok_or("object without id")? as i32;
    let ns = o.get("ns").and_then(|v| v.as_str()).ok_or("object without ns")?.to_string();
    let points = o
        .get("points_ll")
        .and_then(|v| v.as_array())
        .ok_or("object without points_ll")?
        .iter()
        .map(|ll| {
            let arr = ll.as_array().filter(|a| a.len() >= 2).ok_or("bad vertex")?;
            let (lat, lon) = (arr[0].as_f64().ok_or("bad lat")?, arr[1].as_f64().ok_or("bad lon")?);
            Ok(xy_from_ll(lat, lon, datum))
        })
        .collect::<Result<Vec<_>, String>>()?;
    let mut obj = XyObject { id, ns, points, color: None, scale: None };
    if keep_extras {
        if let Some(c) = o.get("color").and_then(|v| v.as_object()) {
            let f = |k: &str, d: f64| c.get(k).and_then(|v| v.as_f64()).unwrap_or(d);
            obj.color = Some((f("r", 0.0), f("g", 1.0), f("b", 0.0), f("a", 0.8)));
        }
        obj.scale = o.get("scale").and_then(|v| v.as_f64());
    }
    Ok(obj)
}

/// Re-project a site's LL vertices into the current map frame.
pub fn site_to_xy(site: &Value, datum: &Datum) -> Result<(Vec<XyObject>, Vec<XyObject>, Vec<XyObject>), String> {
    let list = |key: &str, extras: bool| -> Result<Vec<XyObject>, String> {
        site.get(key)
            .and_then(|v| v.as_array())
            .map(|a| a.iter().map(|o| object_from_site(o, datum, extras)).collect())
            .unwrap_or(Ok(Vec::new()))
    };
    Ok((list("zones", false)?, list("risk_zones", false)?, list("channels", true)?))
}

pub fn write_site(sites_dir: &str, site: &Value) -> Result<(), String> {
    let name = site.get("name").and_then(|v| v.as_str()).ok_or("site without name")?;
    write_json_atomic(&site_path(sites_dir, name), site).map_err(|e| e.to_string())
}

pub fn read_site(sites_dir: &str, name: &str) -> Result<Value, String> {
    read_json(&site_path(sites_dir, name))
}

pub fn delete_site(sites_dir: &str, name: &str) -> Result<(), String> {
    fs::remove_file(site_path(sites_dir, name)).map_err(|e| e.to_string())?;
    fsync_dir(Path::new(sites_dir)).map_err(|e| e.to_string())
}

/// Persist the active named site after validating its file-safe name.
pub fn write_active_site(sites_dir: &str, name: &str) -> Result<(), String> {
    match valid_name(name) {
        Some(n) if n == name => {}
        _ => return Err(format!("invalid active site name: {name:?}")),
    }
    write_json_atomic(&active_site_path(sites_dir), &json!({"version": 1, "active": name})).map_err(|e| e.to_string())
}

/// The active named-site manifest, `Ok(None)` when it does not exist.
pub fn read_active_site(sites_dir: &str) -> Result<Option<String>, String> {
    let path = active_site_path(sites_dir);
    if !path.exists() {
        return Ok(None);
    }
    let payload = read_json(&path)?;
    if payload.get("version").and_then(|v| v.as_i64()) != Some(1) {
        return Err("active-site manifest has an unsupported format".into());
    }
    let name = payload.get("active").and_then(|v| v.as_str()).unwrap_or("");
    match valid_name(name) {
        Some(n) if n == name => Ok(Some(n)),
        _ => Err("active-site manifest contains an invalid name".into()),
    }
}

/// Durably clear the active named-site association.
pub fn clear_active_site(sites_dir: &str) -> Result<(), String> {
    match fs::remove_file(active_site_path(sites_dir)) {
        Ok(()) => fsync_dir(Path::new(sites_dir)).map_err(|e| e.to_string()),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(()),
        Err(e) => Err(e.to_string()),
    }
}

/// Listing payload for /site_list and SiteOp.sites_json (corrupt files are
/// skipped rather than failing the whole listing).
pub fn list_sites(sites_dir: &str, active: Option<&str>) -> Value {
    let mut sites = Vec::new();
    if let Ok(entries) = fs::read_dir(sites_dir) {
        let mut names: Vec<String> = entries
            .filter_map(|e| e.ok())
            .map(|e| e.file_name().to_string_lossy().into_owned())
            .filter(|n| n.ends_with(".json"))
            .collect();
        names.sort();
        for fname in names {
            let Ok(site) = read_json(&Path::new(sites_dir).join(&fname)) else { continue };
            let meta = site.get("meta").cloned().unwrap_or(Value::Null);
            let get = |k: &str, d: Value| meta.get(k).cloned().unwrap_or(d);
            sites.push(json!({
                "name": site.get("name").cloned().unwrap_or_else(|| json!(fname.trim_end_matches(".json"))),
                "created_at": site.get("created_at").cloned().unwrap_or_else(|| json!("")),
                "updated_at": site.get("updated_at").cloned().unwrap_or_else(|| json!("")),
                "zone_count": get("zone_count", json!(0)),
                "risk_count": get("risk_count", json!(0)),
                "channel_count": get("channel_count", json!(0)),
                "area_m2": get("area_m2", json!(0.0)),
                "datum_source": get("datum_source", json!("")),
            }));
        }
    }
    json!({"active": active, "sites": sites})
}

#[cfg(test)]
mod tests {
    use super::*;

    fn datum_a() -> Datum {
        Datum { lat: 23.6939508, lon: 120.5376539, bearing_rad: 0.0, source: "navsat".into() }
    }
    fn datum_b() -> Datum {
        Datum { lat: 23.6942, lon: 120.5379, bearing_rad: 30f64.to_radians(), source: "navsat".into() }
    }

    #[test]
    fn ll_xy_round_trip() {
        for datum in [datum_a(), datum_b()] {
            for xy in [(0.0, 0.0), (10.0, 0.0), (0.0, 10.0), (-7.3, 42.1), (123.4, -56.7)] {
                let (lat, lon) = ll_from_xy(xy.0, xy.1, &datum);
                let (x, y) = xy_from_ll(lat, lon, &datum);
                assert!((x - xy.0).abs() < 1e-6 && (y - xy.1).abs() < 1e-6, "{xy:?}");
            }
        }
    }

    #[test]
    fn bearing_convention_matches_app_geo_anchor() {
        let mut d = datum_a();
        d.bearing_rad = 90f64.to_radians();
        let (lat, lon) = ll_from_xy(10.0, 0.0, &d);
        assert!((lat - d.lat).abs() < 1e-9 && lon > d.lon);
        let (lat, lon) = ll_from_xy(10.0, 0.0, &datum_a());
        assert!((lon - datum_a().lon).abs() < 1e-9);
        assert!(((lat - datum_a().lat) * M_PER_DEG_LAT - 10.0).abs() < 1e-6);
    }

    #[test]
    fn cross_session_reprojection_is_earth_fixed() {
        let zone = XyObject { id: 1, ns: "zones".into(), points: vec![(0.0, 0.0), (8.0, 0.5), (7.5, 6.0), (-0.5, 5.5), (0.0, 0.0)], color: None, scale: None };
        let site = build_site("field", &datum_a(), &[zone.clone()], &[], &[], None);
        let (zones_b, _, _) = site_to_xy(&site, &datum_b()).unwrap();
        for ((xa, ya), (xb, yb)) in zone.points.iter().zip(&zones_b[0].points) {
            let a = ll_from_xy(*xa, *ya, &datum_a());
            let b = ll_from_xy(*xb, *yb, &datum_b());
            assert!((a.0 - b.0).abs() < 1e-9 && (a.1 - b.1).abs() < 1e-9);
        }
        assert!((zones_b[0].points[1].0 - 8.0).abs() > 0.1 || (zones_b[0].points[1].1 - 0.5).abs() > 0.1);
        assert_eq!(site["meta"]["zone_count"], json!(1));
        assert_eq!(site["meta"]["area_m2"], json!(44.2));
        assert!(site["created_at"].as_str().unwrap().ends_with("+00:00"));
    }

    #[test]
    fn antimeridian_wrap() {
        let west = Datum { lat: -16.8, lon: 179.9999, bearing_rad: 0.0, source: "navsat".into() };
        let east = Datum { lat: -16.8, lon: -179.9999, bearing_rad: 0.0, source: "navsat".into() };
        let (lat, lon) = ll_from_xy(5.0, 5.0, &west);
        assert!((-180.0..180.0).contains(&lon));
        let (x, y) = xy_from_ll(lat, lon, &east);
        assert!(x.abs() < 100.0 && y.abs() < 100.0);
        let (lat2, _) = ll_from_xy(x, y, &east);
        assert!((lat2 - lat).abs() < 1e-9);
    }

    #[test]
    fn names_files_and_manifest() {
        for bad in ["", "  ", ".", "..", "a/b", "a\\b", "x\u{1}y"] {
            assert!(valid_name(bad).is_none(), "{bad:?}");
        }
        assert_eq!(valid_name(" 後院 🌱 ").as_deref(), Some("後院 🌱"));
        let dir = std::env::temp_dir().join(format!("mower_sites_{}", std::process::id()));
        let dir_s = dir.to_string_lossy().into_owned();
        let _ = fs::remove_dir_all(&dir);
        assert_eq!(list_sites(&dir_s, None), json!({"active": null, "sites": []}));
        let site = build_site("後院", &datum_a(), &[XyObject { id: 1, ns: "zones".into(), points: vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)], color: None, scale: None }], &[], &[XyObject { id: 2, ns: "chennal_path".into(), points: vec![(0.0, 0.0), (2.0, 2.0)], color: Some((0.0, 1.0, 0.0, 0.8)), scale: Some(0.1) }], Some("2026-01-01T00:00:00+00:00"));
        write_site(&dir_s, &site).unwrap();
        assert!(fs::read_to_string(site_path(&dir_s, "後院")).unwrap().contains("後院"));
        assert!(!dir.join("後院.json.tmp").exists());
        let back = read_site(&dir_s, "後院").unwrap();
        assert_eq!(back["created_at"], json!("2026-01-01T00:00:00+00:00"));
        assert_eq!(back["channels"][0]["scale"], json!(0.1));
        assert_eq!(read_active_site(&dir_s).unwrap(), None);
        write_active_site(&dir_s, "後院").unwrap();
        assert_eq!(read_active_site(&dir_s).unwrap().as_deref(), Some("後院"));
        assert!(write_active_site(&dir_s, "a/b").is_err());
        let listing = list_sites(&dir_s, Some("後院"));
        assert_eq!(listing["sites"][0]["name"], json!("後院"));
        assert_eq!(listing["sites"][0]["channel_count"], json!(1));
        fs::write(dir.join("broken.json"), "{not json").unwrap();
        assert_eq!(list_sites(&dir_s, None)["sites"].as_array().unwrap().len(), 1);
        fs::write(active_site_path(&dir_s), r#"{"version": 2, "active": "x"}"#).unwrap();
        assert!(read_active_site(&dir_s).is_err());
        clear_active_site(&dir_s).unwrap();
        clear_active_site(&dir_s).unwrap();
        assert_eq!(read_active_site(&dir_s).unwrap(), None);
        delete_site(&dir_s, "後院").unwrap();
        assert!(read_site(&dir_s, "後院").is_err());
        let _ = fs::remove_dir_all(&dir);
    }
}
