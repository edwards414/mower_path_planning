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
//! saved): UART1 output off (`disable_uart1_output`), then NAV-PVT,
//! NAV-HPPOSLLH and NAV-EOE on this port at `rate_hz`. Whatever else the
//! receiver streams (RAWX, MON-*, NMEA) is skipped by the parser. Correction
//! input (RTCM over UART1 from a base / NTRIP) is not this driver's business.
//! How the messages become one fix per epoch, also when the receiver drops
//! some of them, is in `epoch.rs`.

use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

use mower_rs_common::{json_number, params, ModuleCtx, ModuleResult};
use mower_ubx::{class, id, Ack, CarrierSolution, Fix, Frame, NavHpPosLlh, NavPvt, Parser};
use r2r::builtin_interfaces::msg::Time;
use r2r::sensor_msgs::msg::NavSatFix;
use r2r::std_msgs::msg::String as StringMsg;
use r2r::QosProfile;

mod epoch;
use epoch::{Assembler, Epoch};

/// NavSatStatus.service: GPS | GLONASS | BeiDou (COMPASS) | Galileo, the
/// ZED-F9P default constellation set.
const SERVICE_ALL: u16 = 1 | 2 | 4 | 8;
const COVARIANCE_TYPE_DIAGONAL_KNOWN: u8 = 2;

const ACK_TIMEOUT: Duration = Duration::from_millis(1000);
const CONFIG_ATTEMPTS: usize = 3;
const STATUS_PERIOD: Duration = Duration::from_secs(1);
const LATE_EPOCH_LOG_PERIOD: Duration = Duration::from_secs(10);

/// UBX-CFG-VALSET (generation 9 configuration interface).
const CFG_VALSET: u8 = 0x8A;
/// CFG-UART1OUTPROT-UBX / -NMEA / -RTCM3X.
const UART1_OUTPUT_KEYS: [u32; 3] = [0x1074_0001, 0x1074_0002, 0x1074_0004];

fn stamp(t: SystemTime) -> Time {
    let ns = t.duration_since(std::time::UNIX_EPOCH).map(|d| d.as_nanos() as i64).unwrap_or(0);
    Time { sec: (ns / 1_000_000_000) as i32, nanosec: (ns % 1_000_000_000) as u32 }
}

/// CFG-VALSET, RAM layer only: every UART1 output protocol off.
fn uart1_output_off() -> Vec<u8> {
    let mut payload = vec![0x00, 0x01, 0x00, 0x00]; // version 0, layers: RAM
    for key in UART1_OUTPUT_KEYS {
        payload.extend_from_slice(&key.to_le_bytes());
        payload.push(0);
    }
    mower_ubx::encode(class::CFG, CFG_VALSET, &payload)
}

struct Settings {
    device: String,
    frame_id: String,
    rate_hz: f64,
    configure: bool,
    uart1_output_off: bool,
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

        let mut epochs = Assembler::new(outputs.hpposllh, outputs.eoe);
        let mut last_pvt_at: Option<Instant> = None;
        let started = Instant::now();
        let mut last_status_at: Option<Instant> = None;
        let mut last_state: Option<(i8, CarrierSolution, u8)> = None;
        let mut checksum_failures_logged = 0u64;
        let mut late_epochs_logged = 0u64;
        let mut late_epochs_logged_at: Option<Instant> = None;

        while !self.stop.load(Ordering::Relaxed) {
            for frame in port.read_frames()? {
                if frame.class != class::NAV {
                    continue;
                }
                match frame.id {
                    id::PVT => {
                        let Some(pvt) = NavPvt::parse(&frame.payload) else { continue };
                        last_pvt_at = Some(Instant::now());
                        for epoch in epochs.on_pvt(pvt, SystemTime::now()) {
                            self.publish_epoch(epoch, &mut last_status_at, &mut last_state);
                        }
                    }
                    id::HPPOSLLH => {
                        if let Some(epoch) = NavHpPosLlh::parse(&frame.payload).and_then(|hp| epochs.on_hp(hp)) {
                            self.publish_epoch(epoch, &mut last_status_at, &mut last_state);
                        }
                    }
                    id::EOE => {
                        if let Some(epoch) = mower_ubx::nav_eoe_itow(&frame.payload).and_then(|itow| epochs.on_eoe(itow)) {
                            self.publish_epoch(epoch, &mut last_status_at, &mut last_state);
                        }
                    }
                    _ => {}
                }
            }

            let now = Instant::now();
            if epochs.late > late_epochs_logged
                && late_epochs_logged_at.map(|t| now.duration_since(t) >= LATE_EPOCH_LOG_PERIOD).unwrap_or(true)
            {
                late_epochs_logged = epochs.late;
                late_epochs_logged_at = Some(now);
                r2r::log_warn!(
                    &self.logger,
                    "{} epochs without their NAV-HPPOSLLH and NAV-EOE so far, published one epoch late \
                     (receiver output buffer full? check MON-COMMS)",
                    epochs.late
                );
            }
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
        if s.uart1_output_off {
            // All ports share the receiver's output buffer. XR-02's rover had
            // UBX + NMEA + RTCM3 output on UART1 at 38400 stored in flash;
            // that port sat at 99 % with 10 kB pending, the receiver logged
            // "txbuf alloc" and dropped USB messages, NAV-EOE first
            // (2026-10-12, 2026-10-24; its flash was fixed by hand, this
            // covers a replaced or reset receiver). First, so the ACKs below
            // get through. Output only: the RTCM input on UART1 stays.
            match port.configure_one(&uart1_output_off(), class::CFG, CFG_VALSET)? {
                Some(true) => r2r::log_info!(&self.logger, "UART1 output off (RAM)"),
                Some(false) => r2r::log_warn!(&self.logger, "receiver rejected turning UART1 output off (NAK)"),
                None => r2r::log_warn!(&self.logger, "no answer to turning UART1 output off"),
            }
        }
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
        epoch: Epoch,
        last_status_at: &mut Option<Instant>,
        last_state: &mut Option<(i8, CarrierSolution, u8)>,
    ) {
        let Epoch { pvt, hp, received_at } = epoch;
        let fix = Fix::from_epoch(&pvt, hp.as_ref());

        let mut msg = NavSatFix::default();
        msg.header.stamp = stamp(received_at);
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

/// Publish-only driver node. Runs as the `mower_gps` binary or as a module of
/// `mower_rsd`; either way a dead serial port ends the *process* (the Err
/// returned here becomes an orderly shutdown and exit 1) so launch respawns
/// it and the navigation health gate closes in between.
pub async fn run(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    let mut node = r2r::Node::create(ctx, &m.node_name, &m.namespace)?;
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
        uart1_output_off: params::bool(&node, "disable_uart1_output", true),
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
    let mut failure: Option<String> = None;
    loop {
        tokio::select! {
            _ = m.shutdown.wait() => break,
            _ = tokio::time::sleep(Duration::from_millis(200)) => {
                if serial_thread.is_finished() {
                    failure = Some("GPS serial thread stopped".to_string());
                    break;
                }
            }
        }
    }
    stop.store(true, Ordering::Relaxed);
    match serial_thread.join() {
        Ok(Err(message)) => {
            r2r::log_fatal!(&logger, "{}", message);
            failure = Some(message);
        }
        Ok(Ok(())) => {}
        Err(_) => {
            r2r::log_fatal!(&logger, "GPS serial thread panicked");
            failure = Some("GPS serial thread panicked".to_string());
        }
    }
    // Stop the node before returning: this one never spins, but in mower_rsd
    // the shared context's other nodes do, and rmw's destructors abort the
    // process if a thread is still inside rcl_wait.
    drop(node);
    match failure {
        Some(message) => Err(message.into()),
        None => Ok(()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn uart1_output_off_is_the_frame_the_rover_acknowledged() {
        // sent to XR-02's F9P by hand on 2026-10-24, answered with ACK-ACK
        assert_eq!(
            uart1_output_off(),
            [
                0xB5, 0x62, 0x06, 0x8A, 0x13, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x74, 0x10, 0x00, 0x02, 0x00,
                0x74, 0x10, 0x00, 0x04, 0x00, 0x74, 0x10, 0x00, 0x37, 0x6E,
            ]
        );
    }
}
