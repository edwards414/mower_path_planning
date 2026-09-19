//! mower_imu: WIT serial IMU driver (port of `wit_ros2_imu`), node name
//! `imu`, publishing `imu/data_raw` (remapped to `/imu/data` by
//! mower.launch.py) at the sensor's 10 Hz orientation rate.
//!
//! Safety behaviour is kept from the Python driver: after a host scheduling
//! gap or an oversized backlog the serial input is discarded rather than
//! re-stamped as current data, an orientation frame is only published when
//! the acceleration and gyro frames of the same cycle are fresh, and a
//! serial failure ends the process so launch supervision and the navigation
//! freshness gate both fail closed. rclpy spent ~6 % of a core on the 10 Hz
//! publish path plus pyserial polling; this driver needs well under 1 %.

mod wit;

use std::io::Read;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use mower_rs_common::params;
use r2r::builtin_interfaces::msg::Time;
use r2r::sensor_msgs::msg::Imu;
use r2r::QosProfile;

use crate::wit::{Frame, Parser};

/// Acceleration / gyro frames older than this do not qualify an orientation
/// frame; also the host scheduling gap after which the backlog is dropped.
const COMPONENT_TIMEOUT: Duration = Duration::from_millis(200);
/// At 9600 baud, more than two complete 0x51..0x54 cycles means the bytes
/// are a backlog.
const MAX_BACKLOG_BYTES: u32 = 88;
/// The sensor streams ~40 frames/s; nothing for this long means the port is
/// open but dead. The CH341 does that after a USB endpoint stall (kernel
/// "urb stopped: -32"): the fd stays valid, reads just never return data
/// until the port is closed and reopened, so the process must end for
/// launch to respawn it (2026-09-20: 2 of 7 stalls that day were of that
/// kind and left the previous driver silent for 6 min).
const FRAME_TIMEOUT: Duration = Duration::from_secs(3);

fn stamp_now() -> Time {
    let ns = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as i64)
        .unwrap_or(0);
    Time { sec: (ns / 1_000_000_000) as i32, nanosec: (ns % 1_000_000_000) as u32 }
}

fn covariance(stddev: f64) -> Vec<f64> {
    let v = stddev * stddev;
    vec![v, 0.0, 0.0, 0.0, v, 0.0, 0.0, 0.0, v]
}

fn stddev_param(node: &r2r::Node, name: &str, default: f64) -> Result<f64, String> {
    let value = params::f64(node, name, default);
    if !value.is_finite() || value <= 0.0 {
        return Err(format!("{name} must be finite and > 0"));
    }
    Ok(value)
}

struct Driver {
    publisher: r2r::Publisher<Imu>,
    template: Imu,
    logger: String,
    stop: Arc<AtomicBool>,
}

impl Driver {
    /// Blocking serial loop; returns the fatal error that ended it, if any.
    fn run(&self, port_name: &str) -> Result<(), String> {
        let mut port = serialport::new(port_name, 9600)
            .timeout(Duration::from_millis(500))
            .open()
            .map_err(|e| format!("IMU serial open failed: {e}"))?;
        r2r::log_info!(&self.logger, "Serial port {} opened successfully...", port_name);

        let mut parser = Parser::default();
        let mut last_acceleration_at: Option<Instant> = None;
        let mut last_angular_velocity_at: Option<Instant> = None;
        let mut last_poll_at = Instant::now();
        let mut last_frame_at = Instant::now();
        let mut buf = [0u8; 256];
        let mut checksum_failures_logged = 0u64;

        while !self.stop.load(Ordering::Relaxed) {
            let poll_at = Instant::now();
            if poll_at.duration_since(last_poll_at) > COMPONENT_TIMEOUT {
                let _ = port.clear(serialport::ClearBuffer::Input);
                parser.reset();
                last_acceleration_at = None;
                last_angular_velocity_at = None;
                last_poll_at = poll_at;
                r2r::log_warn!(&self.logger, "Discarded IMU serial backlog after a host timing gap");
                continue;
            }
            last_poll_at = poll_at;
            if poll_at.duration_since(last_frame_at) > FRAME_TIMEOUT {
                return Err(format!(
                    "no complete IMU frames for {:.0} s with the port open (USB stall?); exiting for respawn",
                    FRAME_TIMEOUT.as_secs_f64()
                ));
            }
            let available = port.bytes_to_read().map_err(|e| format!("IMU serial read/parser failed: {e}"))?;
            if available == 0 {
                // Do not consume a full CPU core when the sensor is quiet.
                std::thread::sleep(Duration::from_millis(5));
                continue;
            }
            if available > MAX_BACKLOG_BYTES {
                let _ = port.clear(serialport::ClearBuffer::Input);
                parser.reset();
                last_acceleration_at = None;
                last_angular_velocity_at = None;
                r2r::log_warn!(&self.logger, "Discarded oversized IMU serial backlog");
                continue;
            }
            let want = (available as usize).min(buf.len());
            let n = port
                .read(&mut buf[..want])
                .map_err(|e| format!("IMU serial read/parser failed: {e}"))?;
            let now = Instant::now();
            for byte in &buf[..n] {
                let frame = parser.push(*byte);
                if frame.is_some() {
                    last_frame_at = now;
                }
                match frame {
                    Some(Frame::Acceleration) => last_acceleration_at = Some(now),
                    Some(Frame::AngularVelocity) => last_angular_velocity_at = Some(now),
                    Some(Frame::Angle) => {
                        let fresh = |t: Option<Instant>| t.map(|t| now.duration_since(t) <= COMPONENT_TIMEOUT).unwrap_or(false);
                        if fresh(last_acceleration_at) && fresh(last_angular_velocity_at) {
                            self.publish(&parser);
                        } else {
                            r2r::log_warn!(
                                &self.logger,
                                "Dropping IMU orientation frame because acceleration/gyro components are stale"
                            );
                        }
                    }
                    Some(Frame::Magnetometer) | None => {}
                }
            }
            if parser.checksum_failures > checksum_failures_logged {
                checksum_failures_logged = parser.checksum_failures;
                r2r::log_warn!(&self.logger, "IMU frame checksum failure ({} so far)", parser.checksum_failures);
            }
        }
        Ok(())
    }

    fn publish(&self, parser: &Parser) {
        let mut msg = self.template.clone();
        msg.header.stamp = stamp_now();
        msg.linear_acceleration.x = parser.acceleration[0];
        msg.linear_acceleration.y = parser.acceleration[1];
        msg.linear_acceleration.z = parser.acceleration[2];
        msg.angular_velocity.x = parser.angular_velocity[0];
        msg.angular_velocity.y = parser.angular_velocity[1];
        msg.angular_velocity.z = parser.angular_velocity[2];
        let q = wit::quaternion_from_euler(
            parser.angle_degree[0].to_radians(),
            parser.angle_degree[1].to_radians(),
            parser.angle_degree[2].to_radians(),
        );
        msg.orientation.x = q[0];
        msg.orientation.y = q[1];
        msg.orientation.z = q[2];
        msg.orientation.w = q[3];
        if let Err(e) = self.publisher.publish(&msg) {
            r2r::log_error!(&self.logger, "publish imu failed: {:?}", e);
        }
    }
}

#[tokio::main(flavor = "multi_thread", worker_threads = 2)]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "imu_driver_node", "")?;
    let logger = node.logger().to_string();
    let port_name = params::string(&node, "port", "/dev/imu_usb");
    let mut template = Imu::default();
    template.header.frame_id = "imu_link".to_string();
    template.orientation_covariance = covariance(stddev_param(&node, "orientation_stddev_rad", 0.35)?);
    template.angular_velocity_covariance = covariance(stddev_param(&node, "angular_velocity_stddev_rad_s", 0.10)?);
    template.linear_acceleration_covariance = covariance(stddev_param(&node, "linear_acceleration_stddev_m_s2", 0.50)?);
    let publisher = node.create_publisher::<Imu>("imu/data_raw", QosProfile::default())?;

    let stop = Arc::new(AtomicBool::new(false));
    let driver = Driver { publisher, template, logger: logger.clone(), stop: stop.clone() };
    let serial_thread = {
        let port_name = port_name.clone();
        std::thread::Builder::new()
            .name("wit-imu-serial".into())
            .spawn(move || driver.run(&port_name))?
    };

    // No subscriptions, services or timers: publishing needs no spinning, and
    // r2r's spin_once returns at once on an empty wait set (a busy loop), so
    // the node is simply kept alive until shutdown.
    let mut sigterm = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())?;
    let mut exit_code = 0;
    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => break,
            _ = sigterm.recv() => break,
            _ = tokio::time::sleep(Duration::from_millis(200)) => {
                if serial_thread.is_finished() {
                    // Exiting only the worker thread would leave a healthy-looking
                    // ROS process publishing no IMU: stop the whole process.
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
            r2r::log_fatal!(&logger, "IMU serial thread panicked");
            exit_code = 1;
        }
    }
    drop(node);
    if exit_code != 0 {
        std::process::exit(exit_code);
    }
    Ok(())
}
