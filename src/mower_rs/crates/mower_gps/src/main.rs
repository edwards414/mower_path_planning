//! mower_gps: u-blox ZED-F9P driver over USB (CDC-ACM), node `gps` in
//! mower.launch.py, publishing `sensor_msgs/NavSatFix` on the canonical fix
//! topic and a small JSON status on `gps/status`.
//!
//! Why not the ROS `ublox_gps` node: after a USB disconnect it keeps the
//! hung-up port open and reposts the failed read forever (100 % of a core,
//! log flood, and the re-enumerated receiver comes back as a new tty minor
//! that the container's static /dev/gps_rtk no longer points at). Like
//! `mower_imu`, this process ends on any serial failure or when NAV-PVT
//! stops arriving, so launch respawns it onto the fresh device and the
//! navigation health gate fails closed in between.
//!
//! Receiver configuration is applied to RAM at every start (nothing is
//! saved): NAV-PVT, NAV-HPPOSLLH and NAV-EOE on this port at `rate_hz`.
//! Whatever else the receiver streams (RAWX, MON-*, NMEA) is skipped by the
//! parser. Correction input (RTCM over UART1 from a base / NTRIP) is not
//! this driver's business.

use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use mower_rs_common::{json_number, params};
use mower_ubx::{class, id, Ack, CarrierSolution, Fix, Frame, NavHpPosLlh, NavPvt, Parser};
use r2r::builtin_interfaces::msg::Time;
use r2r::sensor_msgs::msg::NavSatFix;
use r2r::std_msgs::msg::String as StringMsg;
use r2r::QosProfile;

/// NavSatStatus.service: GPS | GLONASS | BeiDou (COMPASS) | Galileo, the
/// ZED-F9P default constellation set.
const SERVICE_ALL: u16 = 1 | 2 | 4 | 8;
const COVARIANCE_TYPE_DIAGONAL_KNOWN: u8 = 2;

const ACK_TIMEOUT: Duration = Duration::from_millis(1000);
const CONFIG_ATTEMPTS: usize = 3;
const STATUS_PERIOD: Duration = Duration::from_secs(1);

fn stamp_now() -> Time {
    let ns = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as i64)
        .unwrap_or(0);
    Time { sec: (ns / 1_000_000_000) as i32, nanosec: (ns % 1_000_000_000) as u32 }
}

struct Settings {
    device: String,
    frame_id: String,
    rate_hz: f64,
    configure: bool,
    startup_timeout: Duration,
    stale_timeout: Duration,
}

/// What the receiver acknowledged, which decides how epochs are assembled.
#[derive(Default, Clone, Copy)]
struct Outputs {
    pvt: bool,
    hpposllh: bool,
    eoe: bool,
}

#[derive(Default)]
struct Epoch {
    pvt: Option<NavPvt>,
    hp: Option<NavHpPosLlh>,
}

struct Driver {
    fix_pub: r2r::Publisher<NavSatFix>,
    status_pub: r2r::Publisher<StringMsg>,
    logger: String,
    stop: Arc<AtomicBool>,
    settings: Settings,
}

struct Port {
    inner: Box<dyn serialport::SerialPort>,
    parser: Parser,
    buf: [u8; 2048],
}

impl Port {
    /// Read whatever is available (bounded by the port timeout) and return
    /// the complete frames in it. `Ok(empty)` on a timeout; an error, or
    /// EOF (the hung-up tty of an unplugged receiver), ends the driver.
    fn read_frames(&mut self) -> Result<Vec<Frame>, String> {
        match self.inner.read(&mut self.buf) {
            Ok(0) => Err("GPS serial port returned EOF (receiver unplugged?)".to_string()),
            Ok(n) => Ok(self.parser.push_all(&self.buf[..n])),
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut || e.kind() == std::io::ErrorKind::Interrupted => Ok(Vec::new()),
            Err(e) => Err(format!("GPS serial read failed: {e}")),
        }
    }

    fn write(&mut self, frame: &[u8]) -> Result<(), String> {
        self.inner.write_all(frame).and_then(|_| self.inner.flush()).map_err(|e| format!("GPS serial write failed: {e}"))
    }

    /// Send one CFG message and wait for its ACK/NAK. Frames received while
    /// waiting are dropped (the stream restarts cleanly once configured).
    fn configure_one(&mut self, frame: &[u8], cfg_class: u8, cfg_id: u8) -> Result<Option<bool>, String> {
        for _ in 0..CONFIG_ATTEMPTS {
            self.write(frame)?;
            let deadline = Instant::now() + ACK_TIMEOUT;
            while Instant::now() < deadline {
                for f in self.read_frames()? {
                    if let Some(ack) = Ack::parse(&f) {
                        if ack.class == cfg_class && ack.id == cfg_id {
                            return Ok(Some(ack.ok));
                        }
                    }
                }
            }
        }
        Ok(None)
    }
}

impl Driver {
    fn run(&self) -> Result<(), String> {
        let s = &self.settings;
        let inner = serialport::new(&s.device, 38_400) // baud is nominal on CDC-ACM
            .timeout(Duration::from_millis(200))
            .open()
            .map_err(|e| format!("GPS serial open failed ({}): {e}", s.device))?;
        let mut port = Port { inner, parser: Parser::default(), buf: [0u8; 2048] };
        let _ = port.inner.clear(serialport::ClearBuffer::All);
        r2r::log_info!(&self.logger, "u-blox receiver on {} opened", s.device);

        let outputs = if s.configure { self.configure(&mut port)? } else { Outputs { pvt: true, hpposllh: true, eoe: true } };

        let mut epoch = Epoch::default();
        let mut last_pvt_at: Option<Instant> = None;
        let started = Instant::now();
        let mut last_status_at: Option<Instant> = None;
        let mut last_state: Option<(i8, CarrierSolution, u8)> = None;
        let mut checksum_failures_logged = 0u64;

        while !self.stop.load(Ordering::Relaxed) {
            for frame in port.read_frames()? {
                if frame.class != class::NAV {
                    continue;
                }
                match frame.id {
                    id::PVT => {
                        let Some(pvt) = NavPvt::parse(&frame.payload) else { continue };
                        last_pvt_at = Some(Instant::now());
                        epoch.pvt = Some(pvt);
                        if !outputs.eoe {
                            self.publish_epoch(&mut epoch, outputs, &mut last_status_at, &mut last_state);
                        }
                    }
                    id::HPPOSLLH => {
                        if let Some(hp) = NavHpPosLlh::parse(&frame.payload) {
                            epoch.hp = Some(hp);
                        }
                    }
                    id::EOE => {
                        if let (true, Some(itow)) = (outputs.eoe, mower_ubx::nav_eoe_itow(&frame.payload)) {
                            if epoch.pvt.map(|p| p.itow_ms) == Some(itow) {
                                self.publish_epoch(&mut epoch, outputs, &mut last_status_at, &mut last_state);
                            } else {
                                // PVT of this epoch was lost; drop the partial epoch
                                epoch = Epoch::default();
                            }
                        }
                    }
                    _ => {}
                }
            }

            let now = Instant::now();
            match last_pvt_at {
                Some(t) if now.duration_since(t) > s.stale_timeout => {
                    return Err(format!("no NAV-PVT from the receiver for {:.1} s", s.stale_timeout.as_secs_f64()));
                }
                None if now.duration_since(started) > s.startup_timeout => {
                    return Err(format!("no NAV-PVT from the receiver within {:.0} s of start", s.startup_timeout.as_secs_f64()));
                }
                _ => {}
            }
            if port.parser.checksum_failures > checksum_failures_logged {
                checksum_failures_logged = port.parser.checksum_failures;
                r2r::log_warn!(&self.logger, "UBX frame checksum failure ({} so far)", port.parser.checksum_failures);
            }
        }
        Ok(())
    }

    fn configure(&self, port: &mut Port) -> Result<Outputs, String> {
        let s = &self.settings;
        let meas_ms = (1000.0 / s.rate_hz).round().clamp(25.0, 65_535.0) as u16;
        let rate_ok = port.configure_one(&mower_ubx::cfg::rate(meas_ms, 1), class::CFG, id::CFG_RATE)?;
        let mut outputs = Outputs::default();
        let mut enable = |msg_id: u8, name: &str, slot: &mut bool| -> Result<(), String> {
            let ack = port.configure_one(&mower_ubx::cfg::msg_rate(class::NAV, msg_id, 1), class::CFG, id::CFG_MSG)?;
            *slot = ack == Some(true);
            match ack {
                Some(true) => {}
                Some(false) => r2r::log_warn!(&self.logger, "receiver rejected enabling NAV-{name} (NAK)"),
                None => r2r::log_warn!(&self.logger, "no answer to enabling NAV-{name}"),
            }
            Ok(())
        };
        enable(id::PVT, "PVT", &mut outputs.pvt)?;
        enable(id::HPPOSLLH, "HPPOSLLH", &mut outputs.hpposllh)?;
        enable(id::EOE, "EOE", &mut outputs.eoe)?;
        match rate_ok {
            Some(true) => r2r::log_info!(&self.logger, "receiver configured: {} ms epochs, NAV-PVT{}{}", meas_ms,
                if outputs.hpposllh { " + HPPOSLLH" } else { "" }, if outputs.eoe { " + EOE" } else { " (no EOE: fix published on PVT)" }),
            Some(false) => r2r::log_warn!(&self.logger, "receiver rejected the {} ms measurement rate (NAK); keeping its own", meas_ms),
            None => r2r::log_warn!(&self.logger, "no answer to the measurement rate; the receiver keeps its own"),
        }
        // Start the watchdog from a clean stream.
        port.parser.reset();
        Ok(outputs)
    }

    fn publish_epoch(
        &self,
        epoch: &mut Epoch,
        outputs: Outputs,
        last_status_at: &mut Option<Instant>,
        last_state: &mut Option<(i8, CarrierSolution, u8)>,
    ) {
        let Some(pvt) = epoch.pvt.take() else { return };
        let hp = if outputs.hpposllh { epoch.hp.take() } else { None };
        epoch.hp = None;
        let fix = Fix::from_epoch(&pvt, hp.as_ref());

        let mut msg = NavSatFix::default();
        msg.header.stamp = stamp_now();
        msg.header.frame_id = self.settings.frame_id.clone();
        msg.status.status = fix.status;
        msg.status.service = SERVICE_ALL;
        msg.latitude = fix.latitude_deg;
        msg.longitude = fix.longitude_deg;
        msg.altitude = fix.altitude_m;
        msg.position_covariance = vec![fix.h_var_m2, 0.0, 0.0, 0.0, fix.h_var_m2, 0.0, 0.0, 0.0, fix.v_var_m2];
        msg.position_covariance_type = COVARIANCE_TYPE_DIAGONAL_KNOWN;
        if let Err(e) = self.fix_pub.publish(&msg) {
            r2r::log_error!(&self.logger, "publish fix failed: {:?}", e);
        }

        let state = (fix.status, pvt.carrier_solution(), pvt.fix_type);
        let now = Instant::now();
        let changed = *last_state != Some(state);
        if changed {
            r2r::log_info!(
                &self.logger,
                "fix: type {} status {} carrier {} sats {} hAcc {:.3} m{}",
                pvt.fix_type, fix.status, pvt.carrier_solution().as_str(), pvt.num_sv, fix.h_var_m2.sqrt(),
                if fix.high_precision { " (hp)" } else { "" }
            );
            *last_state = Some(state);
        }
        if changed || last_status_at.map(|t| now.duration_since(t) >= STATUS_PERIOD).unwrap_or(true) {
            *last_status_at = Some(now);
            let status = serde_json::json!({
                "fix_type": pvt.fix_type,
                "fix_ok": pvt.fix_ok(),
                "status": fix.status,
                "carrier_solution": pvt.carrier_solution().as_str(),
                "diff_soln": pvt.diff_soln(),
                "num_sv": pvt.num_sv,
                "h_acc_m": json_number(fix.h_var_m2.sqrt()),
                "v_acc_m": json_number(fix.v_var_m2.sqrt()),
                "pdop": json_number(pvt.pdop()),
                "ground_speed_m_s": json_number(pvt.ground_speed_mm_s as f64 * 1e-3),
                "high_precision": fix.high_precision,
                "utc": pvt.utc(),
                "rate_hz": json_number(self.settings.rate_hz),
            });
            if let Err(e) = self.status_pub.publish(&StringMsg { data: status.to_string() }) {
                r2r::log_error!(&self.logger, "publish gps status failed: {:?}", e);
            }
        }
    }
}

fn positive_secs(node: &r2r::Node, name: &str, default: f64) -> Result<Duration, String> {
    let value = params::f64(node, name, default);
    if !value.is_finite() || value <= 0.0 {
        return Err(format!("{name} must be finite and > 0"));
    }
    Ok(Duration::from_secs_f64(value))
}

#[tokio::main(flavor = "multi_thread", worker_threads = 2)]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "gps_driver_node", "")?;
    let logger = node.logger().to_string();

    let rate_hz = params::f64(&node, "rate_hz", 4.0);
    if !rate_hz.is_finite() || !(0.02..=40.0).contains(&rate_hz) {
        return Err("rate_hz must be within 0.02..=40".into());
    }
    let settings = Settings {
        device: params::string(&node, "device", "/dev/gps_rtk"),
        frame_id: params::string(&node, "frame_id", "gps_link"),
        rate_hz,
        configure: params::bool(&node, "configure_receiver", true),
        startup_timeout: positive_secs(&node, "startup_timeout_s", 20.0)?,
        stale_timeout: positive_secs(&node, "stale_timeout_s", 5.0)?,
    };
    let fix_topic = params::string(&node, "fix_topic", "/fix");
    let status_topic = params::string(&node, "status_topic", "gps/status");
    let fix_pub = node.create_publisher::<NavSatFix>(&fix_topic, QosProfile::default())?;
    let status_pub = node.create_publisher::<StringMsg>(&status_topic, QosProfile::default())?;

    let stop = Arc::new(AtomicBool::new(false));
    let driver = Driver { fix_pub, status_pub, logger: logger.clone(), stop: stop.clone(), settings };
    let serial_thread = std::thread::Builder::new().name("ublox-serial".into()).spawn(move || driver.run())?;

    // Publish-only node: nothing to spin, keep the process alive until a
    // signal or until the serial thread ends (which must end the process,
    // see the module comment).
    let mut sigterm = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())?;
    let mut exit_code = 0;
    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => break,
            _ = sigterm.recv() => break,
            _ = tokio::time::sleep(Duration::from_millis(200)) => {
                if serial_thread.is_finished() {
                    exit_code = 1;
                    break;
                }
            }
        }
    }
    stop.store(true, Ordering::Relaxed);
    match serial_thread.join() {
        Ok(Err(message)) => {
            r2r::log_fatal!(&logger, "{}", message);
            exit_code = 1;
        }
        Ok(Ok(())) => {}
        Err(_) => {
            r2r::log_fatal!(&logger, "GPS serial thread panicked");
            exit_code = 1;
        }
    }
    drop(node);
    if exit_code != 0 {
        std::process::exit(exit_code);
    }
    Ok(())
}
