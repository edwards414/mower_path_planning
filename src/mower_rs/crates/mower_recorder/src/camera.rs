//! `camera_front` (E5-6, E5 design 4.7 and D6): the data-collection front
//! camera in the module, in place of the camera-only compose's gscam and
//! its two static_transform_publishers (front_camera.launch.py).
//!
//! - **When:** while mower-data-collection.sh's request file
//!   (`mower_recorder_core::dc_request`, `--request-file` in the instance's
//!   arguments, else `$MOWER_STATE_DIR/recorder_profile.json`, looked at
//!   every 2 s) asks for `camera.driver: gstreamer`. Otherwise nothing of
//!   it is on the graph, and `/dev/video0` stays free for
//!   mower-camera.service. A changed request starts the camera again.
//! - **What it works out** as recorder_manager does
//!   (`dc_request::resolve`, the URDF from `/robot_description`): the
//!   pipeline (`build_pipeline`, or the request's `gscam_pipeline`), the
//!   calibration file (or the placeholder it writes), the two transforms.
//! - **Its nodes and endpoints, as the launch's:** `camera_front` publishes
//!   `/camera/front/image_raw/compressed` (`CompressedImage`, `format:
//!   jpeg`) and `/camera/front/camera_info`, each `rclcpp::QoS(1)` (keep last
//!   1, reliable, volatile), the image first; `camera_front_link_tf` and
//!   `camera_front_optical_tf` each publish their transform once on
//!   `/tf_static` (keep last 1, reliable, transient local; stamped when
//!   published; the values as static_transform_publisher parses
//!   `static_tf_arguments`'s `repr(round(v, 9))` text).
//! - **The frames:** `mower_camera::stream` (gscam's stream, feature
//!   `camera`), on the thread `camera-front`: each sample's JPEG, stamped
//!   with gscam's formula, `frame_id` `camera_front_optical_frame`; the
//!   CameraInfo from the calibration file as camera_info_manager loaded it
//!   (`mower_camera::info`), stamped the same. The end of a stream opens it
//!   again (`reopen_on_eof`), an error 3 s later (launch's respawn delay).
//! - **Without the feature** (a build for tests), a request for it is
//!   refused in the log, once.

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use mower_recorder_core::dc_request::{self, Request, DRIVER_GSTREAMER};
use mower_recorder_core::extrinsics::{self, Transform};
use mower_rs_common::{ModuleCtx, ModuleResult};
use r2r::QosProfile;

/// The instance's node, gscam's (`CAMERA_NAME`).
pub const NODE: &str = dc_request::CAMERA_NAME;

/// The two static TF nodes of front_camera.launch.py.
pub const TF_NODES: [&str; 2] = ["camera_front_link_tf", "camera_front_optical_tf"];

pub const IMAGE_TOPIC: &str = "/camera/front/image_raw/compressed";
pub const INFO_TOPIC: &str = "/camera/front/camera_info";

/// How often the request file is looked at.
pub const POLL: Duration = Duration::from_secs(2);

/// After a stream's error (launch's `respawn_delay`).
pub const RESTART_AFTER: Duration = Duration::from_secs(3);

/// The request file: `--request-file` in the instance's arguments, else the
/// state dir's.
fn request_file(m: &ModuleCtx) -> PathBuf {
    let mut it = m.args.iter();
    while let Some(a) = it.next() {
        if a == "--request-file" {
            if let Some(p) = it.next() {
                return PathBuf::from(crate::expanduser(p));
            }
        }
    }
    let dir = std::env::var("MOWER_STATE_DIR").ok().filter(|d| !d.is_empty()).unwrap_or_else(|| crate::expanduser("~/.mower"));
    PathBuf::from(dir).join(dc_request::FILE_NAME)
}

/// The request in force for this instance: one whose driver is gstreamer.
fn wanted(file: &PathBuf) -> Option<(String, Request)> {
    let text = std::fs::read_to_string(file).ok()?;
    let req = Request::parse(&text).ok()?;
    (req.camera.driver == DRIVER_GSTREAMER).then_some((text, req))
}

/// A running camera: its nodes (kept for as long as it runs) and its thread.
struct Running {
    _nodes: Vec<r2r::Node>,
    stop: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl Running {
    fn stop(mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
        // the nodes go last: the thread's publishers are theirs
    }
}

pub async fn run(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    let file = request_file(&m);
    let logger = NODE.to_string();
    let mut tick = m.bus.wall_timer(&m.clock, POLL)?;
    let mut running: Option<Running> = None;
    let mut applied: Option<String> = None;
    let mut refused: Option<String> = None;
    loop {
        tokio::select! {
            () = m.shutdown.wait() => break,
            t = tick.tick() => if t.is_err() { break },
        }
        let now = wanted(&file);
        let text = now.as_ref().map(|(t, _)| t.clone());
        if text == applied || (text.is_some() && text == refused) {
            continue;
        }
        if let Some(r) = running.take() {
            tokio::task::spawn_blocking(move || r.stop()).await.ok();
            r2r::log_info!(&logger, "camera stopped");
        }
        applied = None;
        if let Some((text, req)) = now {
            let ctx = ctx.clone();
            let log = logger.clone();
            match tokio::task::spawn_blocking(move || start(&ctx, &req, &log)).await.expect("the camera's start panicked") {
                Ok(r) => {
                    running = Some(r);
                    applied = Some(text);
                    refused = None;
                }
                Err(e) => {
                    r2r::log_error!(&logger, "{}: {e}; no camera", file.display());
                    refused = Some(text);
                }
            }
        }
    }
    if let Some(r) = running.take() {
        tokio::task::spawn_blocking(move || r.stop()).await.ok();
    }
    Ok(())
}

/// A transform as static_transform_publisher has it from
/// `static_tf_arguments` (`repr(round(v, 9))`, read back by `std::stod`):
/// translation and quaternion (x, y, z, w).
pub fn published(t: &Transform) -> Result<([f64; 3], [f64; 4]), String> {
    let args = extrinsics::static_tf_arguments(t, "", "");
    let v = |flag: &str| -> Result<f64, String> {
        let i = args.iter().position(|a| a == flag).ok_or(format!("{flag} missing"))?;
        args[i + 1].parse::<f64>().map_err(|e| format!("{flag} {}: {e}", args[i + 1]))
    };
    Ok(([v("--x")?, v("--y")?, v("--z")?], [v("--qx")?, v("--qy")?, v("--qz")?, v("--qw")?]))
}

/// `tf2_ros::StaticTransformBroadcasterNode`'s one message.
fn tf_message(parent: &str, child: &str, t: &Transform) -> Result<r2r::tf2_msgs::msg::TFMessage, String> {
    use r2r::geometry_msgs::msg::{Quaternion, Transform as Tf, TransformStamped, Vector3};
    let ([x, y, z], [qx, qy, qz, qw]) = published(t)?;
    let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).map_err(|e| e.to_string())?;
    let stamp = r2r::Clock::to_builtin_time(&clock.get_now().map_err(|e| e.to_string())?);
    Ok(r2r::tf2_msgs::msg::TFMessage {
        transforms: vec![TransformStamped {
            header: r2r::std_msgs::msg::Header { stamp, frame_id: parent.to_string() },
            child_frame_id: child.to_string(),
            transform: Tf { translation: Vector3 { x, y, z }, rotation: Quaternion { x: qx, y: qy, z: qz, w: qw } },
        }],
    })
}

/// The camera_info message of a calibration (without its header).
fn camera_info(info: &mower_camera::info::CameraInfo) -> r2r::sensor_msgs::msg::CameraInfo {
    let mut m = r2r::sensor_msgs::msg::CameraInfo::default();
    m.height = info.height;
    m.width = info.width;
    m.distortion_model = info.distortion_model.clone();
    m.d = info.d.clone();
    m.k = info.k.to_vec();
    m.r = info.r.to_vec();
    m.p = info.p.to_vec();
    m.binning_x = info.binning_x;
    m.binning_y = info.binning_y;
    let (x_offset, y_offset, height, width, do_rectify) = info.roi;
    m.roi = r2r::sensor_msgs::msg::RegionOfInterest { x_offset, y_offset, height, width, do_rectify };
    m
}

/// The camera for `req`: its parameters, nodes, static TF, and the stream.
fn start(ctx: &r2r::Context, req: &Request, logger: &str) -> Result<Running, String> {
    if !mower_camera::CAPTURES {
        return Err("this build has no camera (mower_rsd's feature `camera`)".into());
    }
    let share = crate::share_config().ok_or("mower_recorder's share/config is not on AMENT_PREFIX_PATH")?;
    let mut camera = r2r::Node::create(ctx.clone(), NODE, "").map_err(|e| e.to_string())?;
    let urdf = crate::robot_description(&mut camera);
    let r = dc_request::resolve(req, &share, urdf.as_deref().map_err(|e| e.clone()))?;
    for note in &r.notes {
        r2r::log_warn!(logger, "[data_collection] WARNING: {note}");
    }
    if let Some((path, body)) = &r.placeholder {
        std::fs::write(path, body).map_err(|e| format!("{}: {e}", path.display()))?;
    }
    let info = match mower_camera::info::load(&r.intrinsics) {
        Ok((_, info)) => {
            r2r::log_info!(logger, "Loaded camera calibration from file://{}", r.intrinsics.display());
            info
        }
        Err(e) => {
            r2r::log_warn!(logger, "Camera info at: file://{} not found ({e}). Using an uncalibrated config.", r.intrinsics.display());
            Default::default()
        }
    };
    // the static TF, each on its node as static_transform_publisher did
    let mut nodes = Vec::new();
    let latched = QosProfile::default().keep_last(1).reliable().transient_local();
    for (name, parent, child, t) in [
        (TF_NODES[0], extrinsics::BASE_LINK, extrinsics::CAMERA_LINK, &r.extrinsics.base_link_to_camera_link),
        (TF_NODES[1], extrinsics::CAMERA_LINK, extrinsics::OPTICAL_FRAME, &r.extrinsics.camera_link_to_optical),
    ] {
        let mut node = r2r::Node::create(ctx.clone(), name, "").map_err(|e| e.to_string())?;
        let publisher =
            node.create_publisher::<r2r::tf2_msgs::msg::TFMessage>("/tf_static", latched.clone()).map_err(|e| e.to_string())?;
        // the writer, the node's, keeps its sample for late joiners while the
        // node lives (the handle may go)
        publisher.publish(&tf_message(parent, child, t)?).map_err(|e| e.to_string())?;
        nodes.push(node);
    }
    let qos = QosProfile::default().keep_last(1).reliable().volatile();
    let image = camera
        .create_publisher::<r2r::sensor_msgs::msg::CompressedImage>(IMAGE_TOPIC, qos.clone())
        .map_err(|e| e.to_string())?;
    let cinfo = camera.create_publisher::<r2r::sensor_msgs::msg::CameraInfo>(INFO_TOPIC, qos).map_err(|e| e.to_string())?;
    let template = camera_info(&info);
    let stop = Arc::new(AtomicBool::new(false));
    let thread = spawn(r.pipeline.clone(), stop.clone(), logger.to_string(), image, cinfo, template)?;
    r2r::log_info!(logger, "Using gstreamer config from the request: \"{}\"", r.pipeline);
    nodes.push(camera);
    Ok(Running { _nodes: nodes, stop, thread: Some(thread) })
}

#[cfg(feature = "camera")]
fn spawn(
    pipeline: String, stop: Arc<AtomicBool>, logger: String, image: r2r::Publisher<r2r::sensor_msgs::msg::CompressedImage>,
    cinfo: r2r::Publisher<r2r::sensor_msgs::msg::CameraInfo>, template: r2r::sensor_msgs::msg::CameraInfo,
) -> Result<std::thread::JoinHandle<()>, String> {
    use mower_camera::stream::{run, Event, Options};
    let opts = Options { pipeline, reopen_on_eof: true, restart_after: RESTART_AFTER };
    std::thread::Builder::new()
        .name("camera-front".into())
        .spawn(move || {
            run(&opts, &stop, |ev| match ev {
                Event::Started { offset } => {
                    r2r::log_info!(&logger, "Time offset: {:.6}", offset as f64 / 1e9);
                    r2r::log_info!(&logger, "Started stream.");
                }
                Event::Frame { jpeg, stamp_ns } => {
                    let (sec, nanosec) = mower_camera::stamp::to_parts(stamp_ns);
                    let header = r2r::std_msgs::msg::Header {
                        stamp: r2r::builtin_interfaces::msg::Time { sec, nanosec },
                        frame_id: extrinsics::OPTICAL_FRAME.to_string(),
                    };
                    let img = r2r::sensor_msgs::msg::CompressedImage {
                        header: header.clone(),
                        format: "jpeg".into(),
                        data: jpeg.to_vec(),
                    };
                    let mut info = template.clone();
                    info.header = header;
                    let _ = image.publish(&img);
                    let _ = cinfo.publish(&info);
                }
                Event::Ended { why, again } => {
                    if again {
                        r2r::log_warn!(&logger, "{why} Reopening stream...");
                    } else {
                        r2r::log_error!(&logger, "{why}");
                    }
                }
            })
        })
        .map_err(|e| e.to_string())
}

#[cfg(not(feature = "camera"))]
fn spawn(
    _pipeline: String, _stop: Arc<AtomicBool>, _logger: String,
    _image: r2r::Publisher<r2r::sensor_msgs::msg::CompressedImage>, _cinfo: r2r::Publisher<r2r::sensor_msgs::msg::CameraInfo>,
    _template: r2r::sensor_msgs::msg::CameraInfo,
) -> Result<std::thread::JoinHandle<()>, String> {
    Err("this build has no camera (mower_rsd's feature `camera`)".into())
}
