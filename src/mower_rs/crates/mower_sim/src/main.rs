//! mower_sim: the robot's hardware for a development container, so that the
//! production stack runs against it unchanged (E8 design §4; docs:
//! README.md next to this crate, deploy/sim/README.md).
//!
//! One truth world (world.rs: the wheels, the body, the pack) drives three
//! devices, each on a pty linked where its udev name would be:
//!
//! | device | link | protocol |
//! |---|---|---|
//! | STM32 base board | /dev/stmcom | the UART frames of mower_base_core (stm32.rs) |
//! | WIT IMU | /dev/imu_usb | 0x51..0x54, 10 cycles/s (imu.rs) |
//! | u-blox ZED-F9P | /dev/gps_rtk | UBX NAV-PVT/HPPOSLLH/RELPOSNED/SAT/EOE, CFG acknowledged (gnss.rs) |
//!
//! The geometry is the stack's own (geometry.rs): the sensors' mount from the
//! URDF, the wheels from mower_rsd.yaml over mower_base's defaults. Faults
//! come from a control directory (control.rs). It runs in real time on the
//! system clock (robot.launch.py wants use_sim_time:=false).
//!
//! **Never on a robot.** It only ever creates ptys and links, and refuses to
//! start when a device path exists as anything but a link to a pty: on the
//! robot those are the real devices.

mod control;
mod geometry;
mod gnss;
mod imu;
mod pty;
mod rng;
mod stm32;
mod world;

// The production parsers the tests feed the devices' output to, compiled
// from their own source: mower_imu and mower_gps link r2r, these two files
// do not.
#[cfg(test)]
#[path = "../../mower_imu/src/wit.rs"]
#[allow(dead_code)]
mod wit_parser;
#[cfg(test)]
#[path = "../../mower_gps/src/epoch.rs"]
#[allow(dead_code)]
mod gps_epoch;

use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use crate::control::Control;
use crate::geometry::{Mount, Origin, Wheels};
use crate::gnss::{Gnss, Mode};
use crate::pty::Pty;
use crate::rng::Rng;
use crate::stm32::{FirmwareId, Stm32};
use crate::world::{Battery, World};

const USAGE: &str = "\
mower_sim: a simulated STM32 base board, WIT IMU and u-blox F9P on ptys,
driven by one truth world, for the production stack in a development
container. Never on a robot: it refuses device paths that are real devices.

usage: mower_sim [options]
  --stm32-link PATH --imu-link PATH --gps-link PATH
                      the links to make (default /dev/stmcom, /dev/imu_usb,
                      /dev/gps_rtk, the udev names the drivers open)
  --owner UID[:GID]   give the ptys to the stack's user (when run as root
                      to write /dev)
  --lat DEG --lon DEG --height M --heading DEG
                      the start: WGS84 position of the antenna and the
                      robot's ENU heading, counter-clockwise from east
                      (default 25.0330 121.5654 45.123 120, fake_sensors.py's)
  --urdf FILE         the robot description the sensor mount is read from
                      (default: mower_description's robot.urdf.xacro on
                      $AMENT_PREFIX_PATH)
  --params FILE       mower_rsd.yaml, for mower_base's wheel parameters
                      (default: mower_bringup's on $AMENT_PREFIX_PATH, else
                      mower_base's built-in values)
  --firmware-manifest FILE
                      what the board reports as its firmware (default
                      $MOWER_FIRMWARE_MANIFEST or
                      /opt/mower/firmware/mower_robot_firmware.json, else
                      fake_base.py's 0.6.0+abcd1234)
  --gps-mode MODE     fixed (RTK, default), float, 3d or none
  --gps-noise M       horizontal 1-sigma of the position (default by mode:
                      0.005 / 0.12 / 0.9 m)
  --slip F            fraction of the wheels' travel lost on the ground (0)
  --battery-v V       the pack's resting voltage at the start (24.3, ~85 %)
  --capacity-ah AH    the pack's capacity (10)
  --flash-stall-s S   how long a PID save stalls the board (1)
  --seed N            the noise's seed (1)
  --control-dir DIR   fault files: base_mute, base_deaf, sensors_mute and
                      gps_mode (fixed|float|3d|none), read every 50 ms
  --truth-log FILE    the truth at 10 Hz, one JSON line each
  --duration S        stop after S seconds (default: until SIGINT / SIGTERM)
";

static STOP: AtomicBool = AtomicBool::new(false);

extern "C" fn on_signal(_: libc::c_int) {
    STOP.store(true, Ordering::Relaxed);
}

#[derive(Debug, Clone)]
struct Options {
    stm32: PathBuf,
    imu: PathBuf,
    gps: PathBuf,
    owner: Option<(u32, u32)>,
    lat: f64,
    lon: f64,
    height: f64,
    heading_deg: f64,
    urdf: Option<PathBuf>,
    params: Option<PathBuf>,
    manifest: Option<PathBuf>,
    gps_mode: Mode,
    gps_noise: Option<f64>,
    slip: f64,
    battery_v: f64,
    capacity_ah: f64,
    flash_stall_s: f64,
    seed: u64,
    control_dir: Option<PathBuf>,
    truth_log: Option<PathBuf>,
    duration: Option<f64>,
}

fn parse(argv: &[String]) -> Result<Options, String> {
    let mut o = Options {
        stm32: "/dev/stmcom".into(),
        imu: "/dev/imu_usb".into(),
        gps: "/dev/gps_rtk".into(),
        owner: None,
        lat: 25.0330,
        lon: 121.5654,
        height: 45.123,
        heading_deg: 120.0,
        urdf: None,
        params: None,
        manifest: None,
        gps_mode: Mode::Fixed,
        gps_noise: None,
        slip: 0.0,
        battery_v: 24.3,
        capacity_ah: 10.0,
        flash_stall_s: 1.0,
        seed: 1,
        control_dir: None,
        truth_log: None,
        duration: None,
    };
    let mut it = argv.iter();
    while let Some(a) = it.next() {
        let mut value = || it.next().cloned().ok_or_else(|| format!("{a} needs a value"));
        let num = |v: String| v.parse::<f64>().ok().filter(|x| x.is_finite()).ok_or_else(|| format!("{a}: {v:?} is not a number"));
        match a.as_str() {
            "--stm32-link" => o.stm32 = value()?.into(),
            "--imu-link" => o.imu = value()?.into(),
            "--gps-link" => o.gps = value()?.into(),
            "--owner" => {
                let v = value()?;
                let (u, g) = v.split_once(':').unwrap_or((&v, &v));
                o.owner = Some((u.parse().map_err(|_| format!("--owner {v:?}"))?, g.parse().map_err(|_| format!("--owner {v:?}"))?));
            }
            "--lat" => o.lat = num(value()?)?,
            "--lon" => o.lon = num(value()?)?,
            "--height" => o.height = num(value()?)?,
            "--heading" => o.heading_deg = num(value()?)?,
            "--urdf" => o.urdf = Some(value()?.into()),
            "--params" => o.params = Some(value()?.into()),
            "--firmware-manifest" => o.manifest = Some(value()?.into()),
            "--gps-mode" => {
                let v = value()?;
                o.gps_mode = Mode::parse(&v).ok_or_else(|| format!("--gps-mode {v:?}: fixed, float, 3d or none"))?;
            }
            "--gps-noise" => o.gps_noise = Some(num(value()?)?.max(0.0)),
            "--slip" => o.slip = num(value()?)?.clamp(0.0, 0.95),
            "--battery-v" => o.battery_v = num(value()?)?,
            "--capacity-ah" => o.capacity_ah = num(value()?)?.max(0.1),
            "--flash-stall-s" => o.flash_stall_s = num(value()?)?.max(0.0),
            "--seed" => o.seed = value()?.parse().map_err(|_| "--seed wants an integer".to_string())?,
            "--control-dir" => o.control_dir = Some(value()?.into()),
            "--truth-log" => o.truth_log = Some(value()?.into()),
            "--duration" => o.duration = Some(num(value()?)?),
            "-h" | "--help" => return Err(String::new()),
            other => return Err(format!("unknown argument {other:?}")),
        }
    }
    for (a, b) in [(&o.stm32, &o.imu), (&o.stm32, &o.gps), (&o.imu, &o.gps)] {
        if a == b {
            return Err(format!("two devices on {}", a.display()));
        }
    }
    Ok(o)
}

/// `relative` under the first prefix of $AMENT_PREFIX_PATH that has it.
fn on_ament_path(relative: &str) -> Option<PathBuf> {
    std::env::var("AMENT_PREFIX_PATH")
        .ok()?
        .split(':')
        .map(|p| Path::new(p).join(relative))
        .find(|p| p.is_file())
}

fn read(path: &Path) -> Result<String, String> {
    std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))
}

fn unix_now() -> f64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0)
}

fn main() -> ExitCode {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let o = match parse(&argv) {
        Ok(o) => o,
        Err(e) if e.is_empty() => {
            print!("{USAGE}");
            return ExitCode::SUCCESS;
        }
        Err(e) => {
            eprintln!("mower_sim: {e}\n{USAGE}");
            return ExitCode::from(2);
        }
    };
    match run(o) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("mower_sim: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run(o: Options) -> Result<(), String> {
    // Refuse before anything is made: a device path that is a real device
    // means this is not a development container.
    for p in [&o.stm32, &o.imu, &o.gps] {
        pty::check_link(p)?;
    }
    let urdf_path = o
        .urdf
        .clone()
        .or_else(|| on_ament_path("share/mower_description/mower_robot/robot.urdf.xacro"))
        .ok_or("no robot description: pass --urdf (mower_description/mower_robot/robot.urdf.xacro)")?;
    let mount = Mount::from_urdf(&read(&urdf_path)?).map_err(|e| format!("{}: {e}", urdf_path.display()))?;
    let params_path = o.params.clone().or_else(|| on_ament_path("share/mower_bringup/config/mower_rsd.yaml"));
    let wheels = match &params_path {
        Some(p) => Wheels::from_params(Some(&read(p)?)).map_err(|e| format!("{}: {e}", p.display()))?,
        None => Wheels::from_params(None)?,
    };
    let manifest_path = o
        .manifest
        .clone()
        .or_else(|| std::env::var("MOWER_FIRMWARE_MANIFEST").ok().filter(|s| !s.is_empty()).map(PathBuf::from))
        .unwrap_or_else(|| "/opt/mower/firmware/mower_robot_firmware.json".into());
    let firmware = match std::fs::read_to_string(&manifest_path) {
        Ok(text) => FirmwareId::from_manifest(&text).map_err(|e| format!("{}: {e}", manifest_path.display()))?,
        Err(_) if o.manifest.is_none() => FirmwareId::FAKE,
        Err(e) => return Err(format!("{}: {e}", manifest_path.display())),
    };

    let origin = Origin::new(o.lat, o.lon, o.height);
    let mut world = World::new(wheels, o.heading_deg.to_radians(), Battery::at_rest(o.battery_v, o.capacity_ah));
    world.slip = o.slip;
    let mut board = Stm32::new(firmware, o.flash_stall_s);
    let mut gnss = Gnss::new(o.gps_mode, o.gps_noise);
    let mut rng = Rng::new(o.seed);
    let mut control = Control::new(o.control_dir.clone());

    let stm = Pty::open(&o.stm32, o.owner)?;
    let imu_pty = Pty::open(&o.imu, o.owner)?;
    let gps_pty = Pty::open(&o.gps, o.owner)?;
    // SAFETY: the handler only stores to an atomic.
    unsafe {
        libc::signal(libc::SIGINT, on_signal as *const () as libc::sighandler_t);
        libc::signal(libc::SIGTERM, on_signal as *const () as libc::sighandler_t);
    }
    let mut truth = match &o.truth_log {
        Some(p) => Some(std::io::BufWriter::new(std::fs::File::create(p).map_err(|e| format!("{}: {e}", p.display()))?)),
        None => None,
    };
    println!(
        "mower_sim: STM32 {} -> {}, IMU {} -> {}, F9P {} -> {}",
        stm.link.display(),
        stm.slave_name,
        imu_pty.link.display(),
        imu_pty.slave_name,
        gps_pty.link.display(),
        gps_pty.slave_name
    );
    println!(
        "mower_sim: start {:.7} {:.7} {:.3} m heading {:.1} deg (ENU); wheels {:.3} m apart, r {:.3}/{:.3} m, {} rpm, {} counts/rev ({}); \
         IMU yaw {:+.1} deg{} ({}); antenna ({:.3}, {:.3}, {:.3}) m; firmware {}.{}.{}+{:08x}; GPS {}",
        o.lat,
        o.lon,
        o.height,
        o.heading_deg,
        wheels.separation,
        wheels.left_radius,
        wheels.right_radius,
        wheels.max_rpm,
        wheels.counts_per_rev,
        params_path.as_ref().map(|p| p.display().to_string()).unwrap_or_else(|| "mower_base's defaults".into()),
        mount.imu_yaw.to_degrees(),
        if mount.imu_flipped { ", upside down" } else { "" },
        urdf_path.display(),
        mount.antenna[0],
        mount.antenna[1],
        mount.antenna[2],
        firmware.major,
        firmware.minor,
        firmware.patch,
        firmware.git_sha32,
        o.gps_mode.name()
    );
    let _ = std::io::stdout().flush();

    const DT: f64 = 0.005;
    let start = Instant::now();
    let mut sim_t = 0.0;
    let (mut next_control, mut next_truth, mut next_status) = (0.0, 0.0, 10.0);
    let (mut next_imu, mut imu_cycles) = (0.0, 0u64);
    let mut buf = [0u8; 4096];
    let mut out = Vec::with_capacity(4096);
    while !STOP.load(Ordering::Relaxed) {
        let now = start.elapsed().as_secs_f64();
        if o.duration.is_some_and(|d| now >= d) {
            break;
        }
        if now >= next_control {
            for change in control.poll() {
                println!("mower_sim: {change}");
            }
            gnss.mode = control.gps_mode.unwrap_or(o.gps_mode);
            next_control = now + 0.05;
        }
        let base_faults = control.base;
        let sensors_mute = control.sensors_mute;

        // what the drivers wrote
        out.clear();
        loop {
            let n = stm.read(&mut buf);
            if n == 0 {
                break;
            }
            board.on_bytes(&buf[..n], now, base_faults, &mut world, &mut out);
        }
        loop {
            let n = gps_pty.read(&mut buf);
            if n == 0 {
                break;
            }
            if !sensors_mute {
                let mut acks = Vec::new();
                gnss.on_bytes(&buf[..n], &mut acks);
                gps_pty.write(&acks);
            }
        }
        while imu_pty.read(&mut buf) > 0 {} // nothing is ever sent to the IMU

        // the world up to now, in fixed steps
        while sim_t + DT <= now {
            world.step(DT);
            sim_t += DT;
        }

        // the devices' own clocks
        board.tick(now, base_faults, &mut world, &mut out);
        stm.write(&out);
        if now >= next_imu {
            if !sensors_mute {
                imu_pty.write(&imu::cycle(&world, &mount));
            }
            imu_cycles += 1;
            next_imu += 0.1;
            if next_imu < now {
                next_imu = now + 0.1;
            }
        }
        let mut epoch = Vec::new();
        gnss.tick(now, unix_now(), &world, &mount, &origin, &mut rng, &mut epoch);
        if !sensors_mute {
            gps_pty.write(&epoch);
        }

        if now >= next_truth {
            if let Some(w) = truth.as_mut() {
                let (lat, lon, _) = origin.llh(world.x, world.y, 0.0);
                let line = serde_json::json!({
                    "t": unix_now(), "x": world.x, "y": world.y, "yaw_deg": world.yaw.to_degrees(),
                    "v": world.v, "w": world.w, "lat": lat, "lon": lon,
                    "left_rpm": world.left.rpm, "right_rpm": world.right.rpm, "blade": world.blade,
                    "battery_v": world.battery.voltage(), "odometer_m": world.odometer,
                });
                let _ = writeln!(w, "{line}");
                let _ = w.flush();
            }
            next_truth = now + 0.1;
        }
        if now >= next_status {
            println!(
                "mower_sim: t {:.0} s  x {:.2} y {:.2} m heading {:.1} deg  v {:.2} m/s  {:.1} m driven  pack {:.2} V  \
                 STM32 {} frames in, {} out  IMU {} cycles  GPS {} epochs, {} CFG acks, {} ms",
                now,
                world.x,
                world.y,
                world.yaw.to_degrees(),
                world.v,
                world.odometer,
                world.battery.voltage(),
                board.frames_rx,
                board.frames_tx,
                imu_cycles,
                gnss.epochs,
                gnss.acks,
                gnss.period_ms()
            );
            let _ = std::io::stdout().flush();
            next_status = now + 10.0;
        }

        // sleep until a driver writes or the next deadline (5 ms at most)
        let mut fds = [stm.fd(), gps_pty.fd(), imu_pty.fd()].map(|fd| libc::pollfd { fd, events: libc::POLLIN, revents: 0 });
        let wait = Duration::from_millis(2);
        // SAFETY: poll on three descriptors we own.
        unsafe { libc::poll(fds.as_mut_ptr(), fds.len() as libc::nfds_t, wait.as_millis() as libc::c_int) };
    }
    println!("mower_sim: stopping after {:.1} s, {:.1} m driven", start.elapsed().as_secs_f64(), world.odometer);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v(list: &[&str]) -> Vec<String> {
        list.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn options() {
        let o = parse(&v(&["--stm32-link", "/tmp/a", "--gps-mode", "float", "--owner", "1000:1001", "--heading", "-30", "--slip", "2"])).unwrap();
        assert_eq!(o.stm32, PathBuf::from("/tmp/a"));
        assert_eq!(o.gps_mode, Mode::Float);
        assert_eq!(o.owner, Some((1000, 1001)));
        assert_eq!((o.heading_deg, o.slip), (-30.0, 0.95));
        assert_eq!(parse(&v(&["--owner", "1000"])).unwrap().owner, Some((1000, 1000)));
        assert!(parse(&v(&["--gps-mode", "/dev/gps_rtk"])).unwrap_err().contains("fixed, float, 3d or none"));
        assert!(parse(&v(&["--imu-link", "/dev/stmcom"])).unwrap_err().contains("two devices"));
        assert!(parse(&v(&["--lat"])).is_err());
        assert!(parse(&v(&["--lat", "north"])).is_err());
        assert_eq!(parse(&v(&["--help"])).unwrap_err(), "");
        let d = parse(&[]).unwrap();
        assert_eq!((d.stm32.to_str(), d.imu.to_str(), d.gps.to_str()), (Some("/dev/stmcom"), Some("/dev/imu_usb"), Some("/dev/gps_rtk")));
        assert_eq!((d.lat, d.lon, d.heading_deg), (25.0330, 121.5654, 120.0));
    }

    #[test]
    fn nothing_of_the_robot_starts_or_ships_it() {
        // the robot's compose, image entry point, unit and launch files
        // (src/mower_sim, the older Gazebo package, is another thing)
        for (name, text) in [
            ("deploy/docker-compose.yaml", include_str!("../../../../../deploy/docker-compose.yaml")),
            ("deploy/host/mower.service", include_str!("../../../../../deploy/host/mower.service")),
            ("utils/docker-entrypoint.sh", include_str!("../../../../../utils/docker-entrypoint.sh")),
            ("Dockerfile", include_str!("../../../../../Dockerfile")),
            ("robot.launch.py", include_str!("../../../../mower_bringup/launch/robot.launch.py")),
            ("mower.launch.py", include_str!("../../../../mower_bringup/launch/mower.launch.py")),
            ("mission.launch.py", include_str!("../../../../mower_mission/launch/mission.launch.py")),
        ] {
            assert!(!text.contains("mower_sim"), "{name} mentions mower_sim");
        }
        // and the image does not carry it: colcon installs MOWER_RS_BINARIES
        let cmake = include_str!("../../../CMakeLists.txt");
        let installed = cmake.split("set(MOWER_RS_BINARIES").nth(1).and_then(|s| s.split(')').next()).expect("the install list");
        assert!(!installed.split_whitespace().any(|b| b == "mower_sim"), "mower_sim is in MOWER_RS_BINARIES");
    }

    #[test]
    fn it_refuses_a_device_path_that_is_a_real_device() {
        let dir = std::env::temp_dir().join(format!("mower_sim_main_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        std::os::unix::fs::symlink("/dev/ttyS3", dir.join("stmcom")).unwrap();
        let mut o = parse(&[]).unwrap();
        o.stm32 = dir.join("stmcom");
        o.imu = dir.join("imu_usb");
        o.gps = dir.join("gps_rtk");
        let e = run(o).unwrap_err();
        assert!(e.contains("ttyS3") && e.contains("is this the robot"), "{e}");
        assert!(!dir.join("imu_usb").exists(), "nothing made");
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
