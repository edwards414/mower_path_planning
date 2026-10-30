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
//!
//! The topic is on the in-process bus (step E2 of docs/ROS_FREE_PLAN.md),
//! under the name rcl gives `imu/data_raw` on the node (`/imu/data` with
//! mower_rsd's `-r imu:imu/data_raw:=imu/data`). The r2r node stays for the
//! parameters, the log and the node's place in the graph.

mod wit;

use std::io::Read;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use mower_bus::Qos;
use mower_rs_common::{params, ModuleCtx, ModuleResult};
use r2r::builtin_interfaces::msg::Time;
use r2r::sensor_msgs::msg::Imu;

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
    publisher: mower_bus::Publisher<Imu>,
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

/// Publish-only driver node. Runs as the `mower_imu` binary or as a module of
/// `mower_rsd`; either way a dead serial port ends the *process* (the Err
/// returned here becomes an orderly shutdown and exit 1) so launch respawns
/// it and the navigation health gate closes in between.
pub async fn run(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    let node = r2r::Node::create(ctx, &m.node_name, &m.namespace)?;
    run_node(node, m).await
}

/// Everything after `Node::create`, so a test can hand in a node whose
/// parameters it has set itself.
async fn run_node(node: r2r::Node, m: ModuleCtx) -> ModuleResult {
    let logger = node.logger().to_string();
    let port_name = params::string(&node, "port", "/dev/imu_usb");
    let mut template = Imu::default();
    template.header.frame_id = "imu_link".to_string();
    template.orientation_covariance = covariance(stddev_param(&node, "orientation_stddev_rad", 0.35)?);
    template.angular_velocity_covariance = covariance(stddev_param(&node, "angular_velocity_stddev_rad_s", 0.10)?);
    template.linear_acceleration_covariance = covariance(stddev_param(&node, "linear_acceleration_stddev_m_s2", 0.50)?);
    let publisher = m.bus.publisher::<Imu>(&m.resolve_topic("imu/data_raw")?, Qos::default())?;

    let stop = Arc::new(AtomicBool::new(false));
    let driver = Driver { publisher, template, logger: logger.clone(), stop: stop.clone() };
    let serial_thread = {
        let port_name = port_name.clone();
        std::thread::Builder::new()
            .name("wit-imu-serial".into())
            .spawn(move || driver.run(&port_name))?
    };

    // No subscriptions, services or timers on the node: it is kept alive
    // until shutdown for its parameters, the log and the graph, and never
    // spun (r2r's spin_once returns at once on an empty wait set, a busy
    // loop). The bus needs no spinning.
    let mut failure: Option<String> = None;
    loop {
        tokio::select! {
            _ = m.shutdown.wait() => break,
            // Closed under the node (its instance ended): nothing published
            // arrives any more, so stop the serial thread as for a shutdown.
            _ = m.bus.closed() => break,
            _ = m.clock.sleep(Duration::from_millis(200)) => {
                if serial_thread.is_finished() {
                    // Exiting only the worker thread would leave a healthy-looking
                    // ROS process publishing no IMU: stop the whole process.
                    failure = Some("IMU serial thread stopped".to_string());
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
            r2r::log_fatal!(&logger, "IMU serial thread panicked");
            failure = Some("IMU serial thread panicked".to_string());
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
    use std::io::Write;
    use std::sync::OnceLock;

    use mower_rs_common::{Remaps, Shutdown};
    use serialport::SerialPort;

    use super::*;

    /// The process's r2r context, on ROS domain 88: the module still creates
    /// its node (parameters, log), and other containers on a dev host speak
    /// DDS. Set before the process's one context exists.
    fn context() -> Option<r2r::Context> {
        static CONTEXT: OnceLock<Option<r2r::Context>> = OnceLock::new();
        CONTEXT
            .get_or_init(|| {
                std::env::set_var("ROS_DOMAIN_ID", "88");
                r2r::Context::create().ok()
            })
            .clone()
    }

    /// The module as mower_rsd runs it: its scope of `bus`, and the
    /// production `-r` rules (robot.launch.py's, with another node's `imu`
    /// rule that must not apply here).
    fn module(bus: &mower_bus::Bus, shutdown: &Shutdown) -> ModuleCtx {
        let argv: Vec<String> = "--modules imu,localize --ros-args -r imu:imu/data_raw:=imu/data \
             -r navsat_transform:imu:=imu/data"
            .split_whitespace()
            .map(str::to_string)
            .collect();
        ModuleCtx::new("imu", shutdown.clone())
            .with_bus(bus.scope("imu"))
            .with_remaps(Arc::new(Remaps::from_args(&argv).unwrap()))
    }

    fn node(ctx: r2r::Context, port: &str) -> r2r::Node {
        // A namespace of the test's own: nothing on the graph meets it.
        let node = r2r::Node::create(ctx, "imu", "/mower_imu_test").unwrap();
        let port = r2r::ParameterValue::String(port.to_string());
        node.params.lock().unwrap().insert("port".to_string(), r2r::Parameter::new(port));
        node
    }

    fn frame(kind: u8, regs: [i16; 4]) -> Vec<u8> {
        let mut f = vec![0x55, kind];
        for r in regs {
            f.extend_from_slice(&r.to_le_bytes());
        }
        let sum = f.iter().map(|b| *b as u32).sum::<u32>() & 0xff;
        f.push(sum as u8);
        f
    }

    /// The sensor's frames on a pseudo terminal come out on the bus as
    /// `/imu/data`, from the `imu` scope, with the frame id and covariances
    /// the driver always set.
    #[test]
    fn serial_frames_go_out_on_the_bus_as_imu_data() {
        let Some(ctx) = context() else { return };
        let runtime = tokio::runtime::Builder::new_multi_thread().worker_threads(2).enable_all().build().unwrap();
        runtime.block_on(async {
            let (mut sensor, port) = serialport::TTYPort::pair().expect("a pseudo terminal");
            let port_name = port.name().expect("the terminal's name");
            // The driver opens the port by name, as it opens /dev/imu_usb.
            drop(port);
            let bus = mower_bus::Bus::new();
            let shutdown = Shutdown::new();
            let reader = bus.scope("reader");
            let mut imu = reader.subscribe::<Imu>("/imu/data", Qos::sensor_data()).unwrap();
            let driver = tokio::spawn(run_node(node(ctx, &port_name), module(&bus, &shutdown)));

            // One cycle at a time, every 100 ms like the sensor, until the
            // driver has opened the port and one comes out. Until it has, the
            // terminal has no other end and refuses the write.
            let cycle =
                [frame(0x51, [2048, 0, 0, 0]), frame(0x52, [0, 0, 16384, 0]), frame(0x53, [0, 0, 16384, 0])].concat();
            let mut got = None;
            for _ in 0..200 {
                let _ = sensor.write_all(&cycle);
                if let Ok(msg) = tokio::time::timeout(Duration::from_millis(100), imu.recv()).await {
                    got = msg;
                    break;
                }
            }
            let msg = got.expect("no /imu/data within 20 s");
            assert_eq!(msg.header.frame_id, "imu_link");
            assert!((msg.linear_acceleration.x - 9.8).abs() < 1e-9, "{:?}", msg.linear_acceleration);
            assert!((msg.angular_velocity.z - 1000f64.to_radians()).abs() < 1e-9, "{:?}", msg.angular_velocity);
            // yaw 90 degrees
            assert!((msg.orientation.z - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-12, "{:?}", msg.orientation);
            assert!((msg.orientation.w - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-12, "{:?}", msg.orientation);
            assert_eq!(msg.orientation_covariance, covariance(0.35));
            assert_eq!(msg.angular_velocity_covariance, covariance(0.10));
            assert_eq!(msg.linear_acceleration_covariance, covariance(0.50));
            let age = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .saturating_sub(Duration::new(msg.header.stamp.sec as u64, msg.header.stamp.nanosec));
            assert!(age < Duration::from_secs(5), "stamped {age:?} ago");
            let topic = bus.topics().into_iter().find(|t| t.name == "/imu/data").unwrap();
            assert_eq!(topic.publishers, ["imu"]);
            assert_eq!(topic.ros_type.as_deref(), Some("sensor_msgs/msg/Imu"));
            // Nothing under the name before the rule.
            assert!(bus.topics().iter().all(|t| t.name != "/imu/data_raw"));

            shutdown.trigger();
            tokio::time::timeout(Duration::from_secs(5), driver)
                .await
                .expect("the driver did not stop")
                .unwrap()
                .expect("a clean stop");
        });
    }

    /// A port that cannot be opened still ends the module with an error, so
    /// that it is restarted and the navigation gate closes meanwhile.
    #[test]
    fn a_port_that_does_not_open_fails_the_module() {
        let Some(ctx) = context() else { return };
        let runtime = tokio::runtime::Builder::new_multi_thread().worker_threads(2).enable_all().build().unwrap();
        runtime.block_on(async {
            let bus = mower_bus::Bus::new();
            let shutdown = Shutdown::new();
            let driver = run_node(node(ctx, "/dev/mower_imu_test_no_such_port"), module(&bus, &shutdown));
            let err = tokio::time::timeout(Duration::from_secs(5), driver)
                .await
                .expect("still running with no port")
                .expect_err("must fail");
            assert!(err.to_string().contains("IMU serial open failed"), "{err}");
        });
    }
}

/// The serial loop's fail-closed rules, on a pseudo terminal (E9-07: what
/// test_controller_configuration.py greps from both drivers). The driver
/// opens the terminal by name as it opens /dev/imu_usb.
#[cfg(test)]
mod serial_rules {
    use std::io::Write;
    use std::sync::OnceLock;

    use mower_rs_common::Shutdown;
    use serialport::{SerialPort, TTYPort};

    use super::*;

    /// On ROS domain 88, as the other tests of this crate: the module still
    /// creates its node (parameters, log).
    fn context() -> Option<r2r::Context> {
        static CONTEXT: OnceLock<Option<r2r::Context>> = OnceLock::new();
        CONTEXT
            .get_or_init(|| {
                std::env::set_var("ROS_DOMAIN_ID", "88");
                r2r::Context::create().ok()
            })
            .clone()
    }

    fn frame(kind: u8, regs: [i16; 4]) -> Vec<u8> {
        let mut f = vec![0x55, kind];
        for r in regs {
            f.extend_from_slice(&r.to_le_bytes());
        }
        let sum = f.iter().map(|b| *b as u32).sum::<u32>() & 0xff;
        f.push(sum as u8);
        f
    }

    fn acceleration() -> Vec<u8> {
        frame(0x51, [2048, 0, 0, 0])
    }

    fn angular_velocity() -> Vec<u8> {
        frame(0x52, [0, 0, 16384, 0])
    }

    fn angle() -> Vec<u8> {
        frame(0x53, [0, 0, 16384, 0])
    }

    fn cycle() -> Vec<u8> {
        [acceleration(), angular_velocity(), angle()].concat()
    }

    struct Driver {
        sensor: TTYPort,
        imu: mower_bus::Subscriber<Imu>,
        shutdown: Shutdown,
        task: tokio::task::JoinHandle<ModuleResult>,
        _bus: mower_bus::Bus,
    }

    impl Driver {
        /// The driver on a fresh terminal, once a first cycle has come out.
        async fn start(ctx: r2r::Context, test: &str) -> Driver {
            let (mut sensor, port) = TTYPort::pair().expect("a pseudo terminal");
            let port_name = port.name().expect("the terminal's name");
            drop(port);
            let node = r2r::Node::create(ctx, "imu", &format!("/mower_imu_{test}")).unwrap();
            node.params.lock().unwrap().insert("port".to_string(), r2r::Parameter::new(r2r::ParameterValue::String(port_name)));
            let bus = mower_bus::Bus::new();
            let shutdown = Shutdown::new();
            let mut imu = bus.scope("reader").subscribe::<Imu>("/imu/data_raw", Qos::default().keep_last(100)).unwrap();
            let m = ModuleCtx::new("imu", shutdown.clone()).with_bus(bus.scope("imu"));
            let task = tokio::spawn(run_node(node, m));
            // Until the driver has opened the terminal, it has no other end
            // and refuses the write.
            for _ in 0..200 {
                let _ = sensor.write_all(&cycle());
                if let Ok(Some(_)) = tokio::time::timeout(Duration::from_millis(100), imu.recv()).await {
                    let mut driver = Driver { sensor, imu, shutdown, task, _bus: bus };
                    driver.count(Duration::from_millis(300)).await;
                    return driver;
                }
            }
            panic!("no IMU message within 20 s");
        }

        fn send(&mut self, bytes: &[u8]) {
            self.sensor.write_all(bytes).expect("the driver's end is open");
        }

        /// The messages that come out within `window`.
        async fn count(&mut self, window: Duration) -> usize {
            let deadline = tokio::time::Instant::now() + window;
            let mut n = 0;
            while let Ok(Some(_)) = tokio::time::timeout_at(deadline, self.imu.recv()).await {
                n += 1;
            }
            n
        }

        async fn stop(self) {
            self.shutdown.trigger();
            tokio::time::timeout(Duration::from_secs(5), self.task).await.expect("the driver stops").unwrap().expect("a clean stop");
        }
    }

    fn run(test: impl std::future::Future<Output = ()>) {
        let runtime = tokio::runtime::Builder::new_multi_thread().worker_threads(2).enable_all().build().unwrap();
        runtime.block_on(test);
    }

    /// Up to 88 bytes waiting (two cycles at 9600 baud) are frames; one byte
    /// more is a backlog, dropped unread, and the driver goes on after it.
    #[test]
    fn a_backlog_over_88_bytes_is_dropped_unread() {
        let Some(ctx) = context() else { return };
        run(async {
            let mut driver = Driver::start(ctx, "backlog").await;
            let eight_frames = [cycle(), cycle(), acceleration(), angular_velocity()].concat();
            assert_eq!(eight_frames.len(), 88);
            driver.send(&eight_frames);
            assert_eq!(driver.count(Duration::from_millis(500)).await, 2, "88 bytes are read");
            driver.send(&[&[0u8][..], &eight_frames].concat());
            assert_eq!(driver.count(Duration::from_millis(500)).await, 0, "89 bytes are a backlog");
            driver.send(&cycle());
            assert_eq!(driver.count(Duration::from_millis(500)).await, 1, "and then a cycle is read again");
            driver.stop().await;
        });
    }

    /// An orientation goes out only with acceleration and gyro from the last
    /// 200 ms: never a sample put together from stale parts.
    #[test]
    fn an_orientation_needs_acceleration_and_gyro_from_the_last_200_ms() {
        let Some(ctx) = context() else { return };
        run(async {
            let mut driver = Driver::start(ctx, "fresh").await;
            driver.send(&[acceleration(), angular_velocity()].concat());
            tokio::time::sleep(Duration::from_millis(50)).await;
            driver.send(&angle());
            assert_eq!(driver.count(Duration::from_millis(300)).await, 1, "parts 50 ms old");
            driver.send(&[acceleration(), angular_velocity()].concat());
            tokio::time::sleep(Duration::from_millis(400)).await;
            driver.send(&angle());
            assert_eq!(driver.count(Duration::from_millis(300)).await, 0, "parts 400 ms old");
            driver.send(&cycle());
            assert_eq!(driver.count(Duration::from_millis(300)).await, 1);
            driver.stop().await;
        });
    }

    /// A port that dies under the driver (the sensor unplugged) ends the
    /// module with an error, without a shutdown, so that it is restarted and
    /// the navigation gate closes meanwhile.
    #[test]
    fn a_port_that_dies_fails_the_module() {
        let Some(ctx) = context() else { return };
        run(async {
            let Driver { sensor, task, shutdown, _bus, imu } = Driver::start(ctx, "dies").await;
            drop(sensor);
            let err = tokio::time::timeout(Duration::from_secs(10), task)
                .await
                .expect("the module ends on its own")
                .unwrap()
                .expect_err("with an error");
            assert!(err.to_string().contains("IMU"), "{err}");
            drop((shutdown, imu));
        });
    }
}
