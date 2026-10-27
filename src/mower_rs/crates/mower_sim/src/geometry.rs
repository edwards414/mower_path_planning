//! The robot's geometry as the production stack knows it, read from the
//! files the stack reads, not written down a second time:
//!
//! - where the IMU and the GNSS antenna sit: the URDF that
//!   robot_state_publisher turns into /tf_static
//!   (mower_description/mower_robot/robot.urdf.xacro, whose joints are plain
//!   numbers; an expanded URDF works as well);
//! - the wheels mower_base turns encoder counts into /odom with: its
//!   `mower_rsd.yaml` section over its built-in production defaults;
//! - the local tangent plane the truth world lives in (WGS84 radii at the
//!   start point).

pub type Mat3 = [[f64; 3]; 3];
pub type Vec3 = [f64; 3];

pub const IDENTITY: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// URDF `rpy`: fixed-axis roll, pitch, yaw, i.e. Rz(yaw) Ry(pitch) Rx(roll).
pub fn rpy_matrix(rpy: Vec3) -> Mat3 {
    let (sr, cr) = rpy[0].sin_cos();
    let (sp, cp) = rpy[1].sin_cos();
    let (sy, cy) = rpy[2].sin_cos();
    [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
}

pub fn mul(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut out = [[0.0; 3]; 3];
    for (i, row) in out.iter_mut().enumerate() {
        for (j, cell) in row.iter_mut().enumerate() {
            *cell = (0..3).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    out
}

pub fn mul_vec(a: &Mat3, v: Vec3) -> Vec3 {
    [0, 1, 2].map(|i| a[i][0] * v[0] + a[i][1] * v[1] + a[i][2] * v[2])
}

pub fn transpose(a: &Mat3) -> Mat3 {
    [0, 1, 2].map(|i| [a[0][i], a[1][i], a[2][i]])
}

#[derive(Debug, Clone, PartialEq)]
pub struct Joint {
    pub name: String,
    pub parent: String,
    pub child: String,
    pub xyz: Vec3,
    pub rpy: Vec3,
}

/// `name="value"` of one tag's text.
fn attr<'a>(tag: &'a str, name: &str) -> Option<&'a str> {
    let key = format!("{name}=\"");
    let mut from = 0;
    while let Some(i) = tag[from..].find(&key) {
        let at = from + i;
        // a whole attribute name, not the tail of another one
        if at == 0 || tag.as_bytes()[at - 1].is_ascii_whitespace() {
            let start = at + key.len();
            return tag[start..].find('"').map(|end| &tag[start..start + end]);
        }
        from = at + key.len();
    }
    None
}

/// The first `<name ...>` tag in `text`, without its brackets.
fn tag<'a>(text: &'a str, name: &str) -> Option<&'a str> {
    let open = format!("<{name}");
    let mut from = 0;
    while let Some(i) = text[from..].find(&open) {
        let at = from + i + open.len();
        if text[at..].starts_with(|c: char| c.is_ascii_whitespace() || c == '/' || c == '>') {
            return text[at..].find('>').map(|end| &text[at..at + end]);
        }
        from = at;
    }
    None
}

fn triple(text: Option<&str>, what: &str) -> Result<Vec3, String> {
    let Some(text) = text else { return Ok([0.0; 3]) };
    let v: Vec<f64> = text
        .split_whitespace()
        .map(|t| t.parse::<f64>())
        .collect::<Result<_, _>>()
        .map_err(|_| format!("{what} {text:?} is not three plain numbers (a xacro expression? give mower_sim the expanded URDF)"))?;
    if v.len() != 3 {
        return Err(format!("{what} {text:?} is not three numbers"));
    }
    Ok([v[0], v[1], v[2]])
}

/// Every `<joint>` of a URDF: name, parent and child links, origin.
pub fn joints(urdf: &str) -> Result<Vec<Joint>, String> {
    // comments first: a commented-out joint is not one
    let mut text = String::with_capacity(urdf.len());
    let mut rest = urdf;
    while let Some(i) = rest.find("<!--") {
        text.push_str(&rest[..i]);
        rest = rest[i..].find("-->").map(|j| &rest[i + j + 3..]).unwrap_or("");
    }
    text.push_str(rest);

    let mut out = Vec::new();
    let mut rest = text.as_str();
    while let Some(i) = rest.find("<joint") {
        let after = &rest[i + "<joint".len()..];
        if !after.starts_with(|c: char| c.is_ascii_whitespace()) {
            rest = after;
            continue;
        }
        let end = after.find("</joint>").ok_or("a <joint> without </joint>")?;
        let body = &after[..end];
        let open = &body[..body.find('>').unwrap_or(body.len())];
        let name = attr(open, "name").unwrap_or("").to_string();
        let link = |t: &str| tag(body, t).and_then(|t| attr(t, "link")).map(str::to_string);
        let (Some(parent), Some(child)) = (link("parent"), link("child")) else {
            return Err(format!("joint {name:?} has no parent or child link"));
        };
        let origin = tag(body, "origin");
        let xyz = triple(origin.and_then(|o| attr(o, "xyz")), &format!("joint {name:?} origin xyz"))?;
        let rpy = triple(origin.and_then(|o| attr(o, "rpy")), &format!("joint {name:?} origin rpy"))?;
        out.push(Joint { name, parent, child, xyz, rpy });
        rest = &after[end..];
    }
    Ok(out)
}

/// The pose of link `to` in link `from` along the joint tree.
pub fn chain(joints: &[Joint], from: &str, to: &str) -> Result<(Mat3, Vec3), String> {
    let mut steps = Vec::new();
    let mut link = to.to_string();
    while link != from {
        let j = joints.iter().find(|j| j.child == link).ok_or_else(|| format!("no joint leads to {link} (from {from})"))?;
        steps.push(j);
        link = j.parent.clone();
        if steps.len() > joints.len() {
            return Err(format!("the joints around {to} form a loop"));
        }
    }
    let (mut r, mut t) = (IDENTITY, [0.0; 3]);
    for j in steps.iter().rev() {
        let o = mul_vec(&r, j.xyz);
        t = [t[0] + o[0], t[1] + o[1], t[2] + o[2]];
        r = mul(&r, &rpy_matrix(j.rpy));
    }
    Ok((r, t))
}

/// Where the sensors sit on the robot (in base_footprint, the frame the
/// EKFs estimate).
#[derive(Debug, Clone, PartialEq)]
pub struct Mount {
    /// imu_link's orientation in base_footprint (`M`).
    pub imu: Mat3,
    /// Its yaw: how far the IMU's heading is turned from the robot's.
    pub imu_yaw: f64,
    /// Mounted upside down (`M`'s z axis points down).
    pub imu_flipped: bool,
    /// gps_link in base_footprint, metres.
    pub antenna: Vec3,
}

impl Mount {
    pub fn from_urdf(urdf: &str) -> Result<Mount, String> {
        let joints = joints(urdf)?;
        let (imu, _) = chain(&joints, "base_footprint", "imu_link")?;
        let (_, antenna) = chain(&joints, "base_footprint", "gps_link")?;
        // The truth world is flat and the IMU model a level one: upright or
        // upside down, turned about the vertical only.
        if (imu[2][2].abs() - 1.0).abs() > 1e-3 {
            return Err(format!("imu_link is tilted in base_footprint (z axis {:?}): mower_sim models a level IMU", [imu[0][2], imu[1][2], imu[2][2]]));
        }
        Ok(Mount { imu_yaw: imu[1][0].atan2(imu[0][0]), imu_flipped: imu[2][2] < 0.0, imu, antenna })
    }
}

/// The wheels as mower_base knows them, which makes /odom the truth when
/// nothing slips.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Wheels {
    /// effective (wheel_separation x wheel_separation_multiplier), m
    pub separation: f64,
    pub left_radius: f64,
    pub right_radius: f64,
    /// 1000 permille
    pub max_rpm: f64,
    pub counts_per_rev: f64,
}

/// mower_base's built-in production values (mower_base/src/lib.rs:
/// production_diff_drive() and run()); tests::defaults_are_mower_bases keeps
/// them equal.
pub const MOWER_BASE_DEFAULTS: [(&str, f64); 7] = [
    ("wheel_separation", 0.35),
    ("wheel_radius", 0.09),
    ("wheel_separation_multiplier", 1.0),
    ("left_wheel_radius_multiplier", 1.0),
    ("right_wheel_radius_multiplier", 1.0),
    ("max_rpm", 58.0),
    ("counts_per_rev", 8896.0),
];

impl Wheels {
    /// mower_base's defaults with the values a `mower_rsd.yaml` sets in its
    /// `mower_base: ros__parameters:` section.
    pub fn from_params(yaml: Option<&str>) -> Result<Wheels, String> {
        let doc: serde_json::Value = match yaml {
            Some(text) => serde_yaml::from_str(text).map_err(|e| format!("not a ROS parameter file: {e}"))?,
            None => serde_json::Value::Null,
        };
        let section = &doc["mower_base"]["ros__parameters"];
        let get = |name: &str| -> Result<f64, String> {
            let default = MOWER_BASE_DEFAULTS.iter().find(|(n, _)| *n == name).map(|(_, v)| *v).expect("a known name");
            match &section[name] {
                serde_json::Value::Null => Ok(default),
                v => v.as_f64().filter(|x| x.is_finite() && *x > 0.0).ok_or_else(|| format!("mower_base.{name}: {v} is not a positive number")),
            }
        };
        let radius = get("wheel_radius")?;
        Ok(Wheels {
            separation: get("wheel_separation")? * get("wheel_separation_multiplier")?,
            left_radius: radius * get("left_wheel_radius_multiplier")?,
            right_radius: radius * get("right_wheel_radius_multiplier")?,
            max_rpm: get("max_rpm")?,
            counts_per_rev: get("counts_per_rev")?,
        })
    }
}

/// East / north / up metres around a start point, and back to latitude,
/// longitude and height on WGS84.
#[derive(Debug, Clone, Copy)]
pub struct Origin {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub height_m: f64,
    /// metres per radian of latitude / longitude at the start point
    m_lat: f64,
    m_lon: f64,
}

impl Origin {
    pub fn new(lat_deg: f64, lon_deg: f64, height_m: f64) -> Origin {
        let (a, f) = (6_378_137.0, 1.0 / 298.257_223_563);
        let e2 = f * (2.0 - f);
        let s = lat_deg.to_radians().sin();
        let w = (1.0 - e2 * s * s).sqrt();
        let m = a * (1.0 - e2) / (w * w * w);
        let n = a / w;
        Origin { lat_deg, lon_deg, height_m, m_lat: m + height_m, m_lon: (n + height_m) * lat_deg.to_radians().cos() }
    }

    pub fn llh(&self, e: f64, n: f64, u: f64) -> (f64, f64, f64) {
        (self.lat_deg + (n / self.m_lat).to_degrees(), self.lon_deg + (e / self.m_lon).to_degrees(), self.height_m + u)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const REPO_URDF: &str = include_str!("../../../../mower_description/mower_robot/robot.urdf.xacro");

    #[test]
    fn the_repos_urdf_gives_the_mount_fake_sensors_assumed() {
        // fake_sensors.py --follow-odom: imu_link upside down (roll 180 deg)
        // under a base_link turned +90 deg from base_footprint, so base yaw =
        // IMU yaw + 90 deg and base yaw rate = -IMU z rate; no horizontal
        // antenna offset
        let m = Mount::from_urdf(REPO_URDF).unwrap();
        assert!((m.imu_yaw.to_degrees() - 90.0).abs() < 0.01, "{m:?}");
        assert!(m.imu_flipped);
        assert!(m.antenna[0].abs() < 1e-9 && m.antenna[1].abs() < 1e-9, "{m:?}");
        assert!((m.antenna[2] - 0.12).abs() < 1e-9);
    }

    #[test]
    fn joints_parse_and_chains_compose() {
        let urdf = r#"<robot>
          <!-- <joint name="old" type="fixed"><parent link="a"/><child link="imu_link"/></joint> -->
          <joint name="j1" type="fixed"><origin xyz="1 0 0" rpy="0 0 1.5707963267948966"/>
            <parent link="base_footprint"/><child link="base_link"/></joint>
          <joint name="j2" type="fixed"><origin xyz="0 2 0" rpy="0 0 0"/><parent link="base_link"/><child link="gps_link"/></joint>
          <jointx/>
        </robot>"#;
        let j = joints(urdf).unwrap();
        assert_eq!(j.len(), 2, "{j:?}");
        assert_eq!((j[0].name.as_str(), j[0].parent.as_str(), j[0].child.as_str()), ("j1", "base_footprint", "base_link"));
        let (_, t) = chain(&j, "base_footprint", "gps_link").unwrap();
        // 2 m along base_link's y, which is -x of base_footprint
        assert!((t[0] + 1.0).abs() < 1e-12 && t[1].abs() < 1e-12, "{t:?}");
        assert!(chain(&j, "base_footprint", "imu_link").unwrap_err().contains("no joint leads to imu_link"));
        let bad = r#"<joint name="x"><origin xyz="${a} 0 0"/><parent link="p"/><child link="c"/></joint>"#;
        assert!(joints(bad).unwrap_err().contains("xacro"));
    }

    #[test]
    fn rotations() {
        let r = rpy_matrix([std::f64::consts::PI, 0.0, 0.0]);
        let v = mul_vec(&r, [0.0, 0.0, 1.0]);
        assert!((v[2] + 1.0).abs() < 1e-12);
        let y = rpy_matrix([0.0, 0.0, 0.3]);
        let back = mul(&transpose(&y), &y);
        for (i, row) in back.iter().enumerate() {
            for (k, x) in row.iter().enumerate() {
                assert!((x - IDENTITY[i][k]).abs() < 1e-12);
            }
        }
    }

    #[test]
    fn defaults_are_mower_bases() {
        // the literals mower_base falls back to, from its source
        let src = include_str!("../../mower_base/src/lib.rs");
        let prod = &src[src.find("pub fn production_diff_drive()").unwrap()..];
        let prod = &prod[..prod.find("\n}").unwrap()];
        for (name, v) in MOWER_BASE_DEFAULTS {
            let found = match name {
                "max_rpm" | "counts_per_rev" => src.contains(&format!("params::f64(&node, \"{name}\", {v:.1})")),
                _ => prod.contains(&format!("{name}: {v},")) || prod.contains(&format!("{name}: {v:.1},")),
            };
            assert!(found, "mower_base's default for {name} is no longer {v}");
        }
        let w = Wheels::from_params(None).unwrap();
        assert_eq!(w, Wheels { separation: 0.35, left_radius: 0.09, right_radius: 0.09, max_rpm: 58.0, counts_per_rev: 8896.0 });
    }

    #[test]
    fn parameters_override_the_defaults() {
        let yaml = "mower_base:\n  ros__parameters:\n    device: /dev/stmcom\n    wheel_separation: 0.4\n    wheel_separation_multiplier: 1.05\n    right_wheel_radius_multiplier: 0.98\nrobot_status:\n  ros__parameters: {}\n";
        let w = Wheels::from_params(Some(yaml)).unwrap();
        assert!((w.separation - 0.42).abs() < 1e-12);
        assert!((w.right_radius - 0.0882).abs() < 1e-12 && w.left_radius == 0.09);
        // the production file sets none of them
        let prod = include_str!("../../../../mower_bringup/config/mower_rsd.yaml");
        assert_eq!(Wheels::from_params(Some(prod)).unwrap(), Wheels::from_params(None).unwrap());
        assert!(Wheels::from_params(Some("mower_base:\n  ros__parameters:\n    wheel_radius: -1\n")).is_err());
    }

    #[test]
    fn the_tangent_plane_matches_wgs84_at_the_start() {
        let o = Origin::new(25.033, 121.5654, 45.0);
        let (lat, lon, h) = o.llh(0.0, 0.0, 0.0);
        assert_eq!((lat, lon, h), (25.033, 121.5654, 45.0));
        // one metre north / east there, 45 m up: 1 / 110 774.16 deg and
        // 1 / 100 923.82 deg (WGS84 meridian and prime-vertical radii); the
        // spherical 111 320 m/deg of fake_sensors.py is 0.5 % off
        let (lat, _, _) = o.llh(0.0, 1.0, 0.0);
        assert!(((lat - 25.033) * 110_774.16 - 1.0).abs() < 1e-5, "{}", (lat - 25.033) * 110_774.16);
        let (_, lon, _) = o.llh(1.0, 0.0, 0.0);
        assert!(((lon - 121.5654) * 100_923.82 - 1.0).abs() < 1e-5, "{}", (lon - 121.5654) * 100_923.82);
    }
}
