//! `navsat_transform`: the ROS shell around
//! `mower_localize_core::navsat::NavSatTransformCore`.
//!
//! The datum, the UTM series, the antenna-offset removal and both published
//! messages are the core's, checked against the real `navsat_transform` to
//! 1.6e-9 m. This file is `navsat_transform_node.cpp`'s wiring: three
//! subscriptions, two publishers, the `utm` static transform, the four
//! services and the `frequency` timer (`transformCallback`).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use futures::stream::StreamExt;
use mower_localize_core::config::NavSatSettings;
use mower_localize_core::navsat::NavSatTransformCore;
use mower_localize_core::tf::{Quaternion, Vector3};
use mower_rs_common::{ModuleCtx, ModuleResult};
use r2r::geographic_msgs::msg::GeoPoint;
use r2r::geometry_msgs::msg::Point;
use r2r::nav_msgs::msg::Odometry as ROdometry;
use r2r::robot_localization::srv::{FromLL, FromLLArray, SetDatum, ToLL};
use r2r::sensor_msgs::msg::{Imu as RImu, NavSatFix as RNavSatFix};
use r2r::tf2_msgs::msg::TFMessage;
use r2r::QosProfile;
use tokio::sync::Mutex;

use crate::conv;
use crate::paramsrv;
use crate::tfbus;

/// `navsat_transform`'s Cartesian frame when `use_local_cartesian` is false.
const CARTESIAN_FRAME: &str = "utm";

/// Upstream's topic names; the launch file remaps them with node-scoped `-r`
/// rules exactly as it remaps the C++ node (`gps/fix` -> `/fix`, `imu` ->
/// `imu/data`, `odometry/filtered` -> `odometry/global`).
const ODOM_TOPIC: &str = "odometry/filtered";
const FIX_TOPIC: &str = "gps/fix";
const IMU_TOPIC: &str = "imu";
const GPS_ODOM_TOPIC: &str = "odometry/gps";
const FILTERED_GPS_TOPIC: &str = "gps/filtered";

struct Nav {
    core: NavSatTransformCore,
    tf_revision: Option<u64>,
}

impl Nav {
    /// The three lookups the core makes: `base_link <- imu frame`,
    /// `base_link <- gps frame` and `world <- base_link`. The first two are
    /// static; the third is the two EKF broadcasts composed.
    fn sync_transforms(&mut self) {
        let base = self.core.base_link_frame_id.clone();
        let world = self.core.world_frame_id.clone();
        if let Some((rev, tree)) =
            tfbus::tree_if_changed(self.tf_revision, &base, &[(&world, &base)])
        {
            self.core.transforms = tree;
            self.tf_revision = Some(rev);
        }
    }

    /// Force a rebuild: the frame names themselves changed (first odometry
    /// message), so the cached revision no longer describes the right pairs.
    fn invalidate(&mut self) {
        self.tf_revision = None;
    }
}

/// `rclcpp::SensorDataQoS(rclcpp::KeepLast(1))`, upstream's `custom_qos` for
/// all three inputs. The depth of one is part of the start-up behaviour: while
/// the node waits out `delay` nothing is taken, so the reader holds only the
/// latest odometry, fix and IMU sample, and those are what the datum is built
/// from.
fn input_qos() -> QosProfile {
    QosProfile::default().keep_last(1).best_effort().volatile()
}

fn output_qos() -> QosProfile {
    QosProfile::default().keep_last(10).reliable().volatile()
}

fn tf_static_qos() -> QosProfile {
    QosProfile::default().keep_last(100).reliable().transient_local()
}

pub async fn run(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    let node = r2r::Node::create(ctx, &m.node_name, &m.namespace)?;
    run_node(node, m).await
}

/// Everything after `Node::create`, so a test can hand in a node whose
/// parameters it has set itself.
async fn run_node(mut node: r2r::Node, m: ModuleCtx) -> ModuleResult {
    let logger = node.logger().to_string();
    // The node's section of dual_ekf_navsat_params.yaml, resolved as the
    // `NavSatTransform` constructor reads it.
    let resolved = NavSatSettings::from_params(&m.node_name, &paramsrv::overrides(&node))?;
    for w in &resolved.warnings {
        r2r::log_warn!(&logger, "{w}");
    }
    let cfg = resolved.settings.config.clone();
    let frequency = resolved.settings.frequency;
    let delay = resolved.settings.delay;

    let nav = Arc::new(Mutex::new(Nav {
        core: NavSatTransformCore::new(cfg.clone()),
        tf_revision: None,
    }));

    // ---- static sensor offsets (base_link <- gps_link / imu_link) ----------
    {
        let mut stream = node.subscribe::<TFMessage>("/tf_static", tf_static_qos())?;
        tokio::spawn(async move {
            while let Some(msg) = stream.next().await {
                for t in &msg.transforms {
                    tfbus::set(
                        &t.header.frame_id,
                        &t.child_frame_id,
                        conv::transform_in(&t.transform),
                    );
                }
            }
        });
    }

    // ---- inputs -------------------------------------------------------------
    {
        let mut stream = node.subscribe::<ROdometry>(ODOM_TOPIC, input_qos())?;
        let nav = nav.clone();
        tokio::spawn(async move {
            while let Some(msg) = stream.next().await {
                let converted = conv::odometry_in(&msg);
                let mut n = nav.lock().await;
                let frames_changed = n.core.world_frame_id != converted.header.frame_id
                    || n.core.base_link_frame_id != converted.child_frame_id;
                n.core.odom_callback(&converted);
                if frames_changed {
                    n.invalidate();
                }
                n.sync_transforms();
            }
        });
    }
    {
        // `gps/fix` is best-effort sensor data, like the driver publishes it.
        let mut stream = node.subscribe::<RNavSatFix>(FIX_TOPIC, input_qos())?;
        let nav = nav.clone();
        tokio::spawn(async move {
            while let Some(msg) = stream.next().await {
                let converted = conv::fix_in(&msg);
                let mut n = nav.lock().await;
                n.sync_transforms();
                n.core.gps_fix_callback(&converted);
            }
        });
    }
    // Upstream drops the IMU subscription once the transform is good (and the
    // yaml has use_odometry_yaw false, no manual datum), so the flag below
    // reproduces that: the yaw is only needed to build the datum.
    let imu_done = Arc::new(AtomicBool::new(false));
    {
        let mut stream = node.subscribe::<RImu>(IMU_TOPIC, input_qos())?;
        let nav = nav.clone();
        let imu_done = imu_done.clone();
        tokio::spawn(async move {
            while let Some(msg) = stream.next().await {
                if imu_done.load(Ordering::Relaxed) {
                    continue;
                }
                let converted = conv::imu_in(&msg);
                let mut n = nav.lock().await;
                n.sync_transforms();
                n.core.imu_callback(&converted);
            }
        });
    }

    // ---- outputs ------------------------------------------------------------
    let gps_odom_pub = node.create_publisher::<ROdometry>(GPS_ODOM_TOPIC, output_qos())?;
    let filtered_gps_pub = if cfg.publish_filtered_gps {
        Some(node.create_publisher::<RNavSatFix>(FILTERED_GPS_TOPIC, output_qos())?)
    } else {
        None
    };
    // tf2_ros::StaticTransformBroadcaster: QoS(1), transient local.
    let cartesian_pub = node.create_publisher::<TFMessage>(
        "/tf_static",
        QosProfile::default().keep_last(1).reliable().transient_local(),
    )?;

    // ---- services -----------------------------------------------------------
    {
        let mut stream =
            node.create_service::<ToLL::Service>("/toLL", QosProfile::services_default())?;
        let nav = nav.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let p = &req.message.map_point;
                // `toLLCallback` returns false before the datum exists and
                // rclcpp still sends the default response, so the answer is
                // (0, 0, 0) until then -- which is what both adapters poll for
                // ("no navsat datum yet"). Projecting through the identity
                // transform instead gives e.g. (0.0, 118.51) once a fix has
                // set the UTM zone, and the adapters would lock that.
                let answer = nav.lock().await.core.to_ll(&Vector3::new(p.x, p.y, p.z));
                let ll_point = match answer {
                    Some((latitude, longitude, altitude)) => {
                        GeoPoint { latitude, longitude, altitude }
                    }
                    None => GeoPoint::default(),
                };
                let _ = req.respond(ToLL::Response { ll_point });
            }
        });
    }
    {
        let mut stream =
            node.create_service::<FromLL::Service>("/fromLL", QosProfile::services_default())?;
        let nav = nav.clone();
        let logger = logger.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let ll = &req.message.ll_point;
                let map_point = from_ll(&nav, ll, &logger).await;
                let _ = req.respond(FromLL::Response { map_point });
            }
        });
    }
    {
        let mut stream = node
            .create_service::<FromLLArray::Service>("/fromLLArray", QosProfile::services_default())?;
        let nav = nav.clone();
        let logger = logger.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                // `fromLLArrayCallback` converts all points or none: on the
                // first failure it returns false and the default response,
                // an empty array, goes out.
                let map_points = {
                    let n = nav.lock().await;
                    req.message
                        .ll_points
                        .iter()
                        .map(|ll| n.core.from_ll(ll.latitude, ll.longitude, ll.altitude))
                        .collect::<Option<Vec<_>>>()
                };
                let map_points = match map_points {
                    Some(points) => {
                        points.into_iter().map(|p| Point { x: p.x, y: p.y, z: p.z }).collect()
                    }
                    None => {
                        r2r::log_error!(
                            &logger,
                            "fromLLArray: no datum yet or a point outside UTM; answering an empty array"
                        );
                        Vec::new()
                    }
                };
                let _ = req.respond(FromLLArray::Response { map_points });
            }
        });
    }
    {
        let mut stream =
            node.create_service::<SetDatum::Service>("/datum", QosProfile::services_default())?;
        let nav = nav.clone();
        let logger = logger.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let pose = &req.message.geo_pose;
                let q = &pose.orientation;
                nav.lock().await.core.set_datum(
                    pose.position.latitude,
                    pose.position.longitude,
                    pose.position.altitude,
                    Quaternion::new(q.x, q.y, q.z, q.w),
                );
                r2r::log_info!(
                    &logger,
                    "Datum set to {} {} {}",
                    pose.position.latitude,
                    pose.position.longitude,
                    pose.position.altitude
                );
                let _ = req.respond(SetDatum::Response {});
            }
        });
    }

    let mut advertised: Vec<paramsrv::Param> =
        resolved.parameters.iter().map(|(name, v)| paramsrv::from_core(name, v)).collect();
    // rclcpp declares it on every node; the resolver refused anything but false.
    if advertised.iter().all(|p| p.name != "use_sim_time") {
        advertised.push(paramsrv::boolean("use_sim_time", false));
    }
    paramsrv::advertise(&mut node, &m.node_name, advertised)?;

    r2r::log_info!(
        &logger,
        "navsat_transform (mower_rs): {frequency} Hz, {FIX_TOPIC} + {IMU_TOPIC} + {ODOM_TOPIC} -> {GPS_ODOM_TOPIC}"
    );

    // `delay`, as the upstream constructor does it: everything above exists,
    // but the node is not spun until the delay has passed, so no callback and
    // no service runs meanwhile, and the depth-1 readers keep only the latest
    // odometry, fix and IMU sample. The datum is therefore built from inputs
    // at least `delay` old -- by then the map EKF's yaw has had time to
    // converge on the IMU heading when it was initialised from /odom (yaw 0).
    // Locking on the first tick instead rotated the map<->UTM datum by most of
    // the robot's initial heading.
    if delay > 0.0 {
        r2r::log_info!(&logger, "Delaying for {delay} seconds before starting...");
        tokio::select! {
            // Nothing is spinning yet, so returning drops the node cleanly.
            _ = m.shutdown.wait() => return Ok(()),
            _ = tokio::time::sleep(Duration::from_secs_f64(delay)) => {}
        }
        r2r::log_info!(&logger, "Delay elapsed. Continuing.");
    }

    let running = Arc::new(AtomicBool::new(true));
    let spin = {
        let running = running.clone();
        tokio::task::spawn_blocking(move || {
            while running.load(Ordering::Relaxed) {
                node.spin_once(Duration::from_millis(50));
            }
            drop(node);
        })
    };

    // `transformCallback`: while the datum is missing, try to build it; once
    // it exists, publish. Never both in the same tick, as upstream's if/else.
    // Upstream creates the wall timer after the delay, so its first tick is
    // one period later: the first spin takes the waiting samples before the
    // datum is attempted. A tokio interval would tick at once.
    let period = Duration::from_secs_f64(1.0 / frequency);
    let mut tick = tokio::time::interval_at(tokio::time::Instant::now() + period, period);
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    loop {
        tokio::select! {
            _ = m.shutdown.wait() => break,
            _ = tick.tick() => {}
        }
        let mut n = nav.lock().await;
        n.sync_transforms();
        if !n.core.transform_good {
            n.core.compute_transform();
            if n.core.transform_good {
                r2r::log_info!(
                    &logger,
                    "Datum locked: UTM zone {}{}",
                    n.core.utm_zone,
                    if n.core.northp { "N" } else { "S" }
                );
                if cfg.broadcast_cartesian_transform {
                    let mut t = n.core.cartesian_world_transform;
                    if cfg.zero_altitude {
                        t.origin.z = 0.0;
                    }
                    let (parent, child, t) = if cfg.broadcast_cartesian_transform_as_parent_frame {
                        (CARTESIAN_FRAME, n.core.world_frame_id.as_str(), t.inverse())
                    } else {
                        (n.core.world_frame_id.as_str(), CARTESIAN_FRAME, t)
                    };
                    let stamped = conv::transform_stamped(conv::now_ns(), parent, child, &t);
                    tfbus::set(parent, child, t);
                    let _ = cartesian_pub.publish(&TFMessage { transforms: vec![stamped] });
                }
                if !cfg.use_odometry_yaw {
                    // Upstream resets the IMU subscription here.
                    imu_done.store(true, Ordering::Relaxed);
                }
            }
            continue;
        }
        let gps_odom = n.core.prepare_gps_odometry();
        let filtered = if cfg.publish_filtered_gps {
            n.core.prepare_filtered_gps()
        } else {
            None
        };
        drop(n);
        if let Some(odom) = gps_odom {
            if let Err(e) = gps_odom_pub.publish(&conv::odometry_out(&odom)) {
                r2r::log_warn!(&logger, "{GPS_ODOM_TOPIC} publish failed: {e:?}");
            }
        }
        if let (Some(pubr), Some(fix)) = (filtered_gps_pub.as_ref(), filtered) {
            if let Err(e) = pubr.publish(&conv::fix_out(&fix)) {
                r2r::log_warn!(&logger, "{FILTERED_GPS_TOPIC} publish failed: {e:?}");
            }
        }
    }

    running.store(false, Ordering::Relaxed);
    let _ = spin.await;
    Ok(())
}

/// `NavSatTransform::fromLLCallback`. Before the datum exists upstream's
/// `fromLL` throws, the callback returns false and rclcpp sends the default
/// response, the map origin; this answers the same and also logs it.
async fn from_ll(nav: &Arc<Mutex<Nav>>, ll: &GeoPoint, logger: &str) -> Point {
    match nav
        .lock()
        .await
        .core
        .from_ll(ll.latitude, ll.longitude, ll.altitude)
    {
        Some(p) => Point { x: p.x, y: p.y, z: p.z },
        None => {
            r2r::log_error!(
                logger,
                "fromLL: no datum yet or a point outside UTM; answering the map origin"
            );
            Point { x: 0.0, y: 0.0, z: 0.0 }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn yaw_quat(yaw: f64) -> r2r::geometry_msgs::msg::Quaternion {
        r2r::geometry_msgs::msg::Quaternion {
            x: 0.0,
            y: 0.0,
            z: (yaw / 2.0).sin(),
            w: (yaw / 2.0).cos(),
        }
    }

    /// `delay` as upstream applies it: no datum before the delay has passed,
    /// and then the one built from the *latest* odometry, IMU and fix, not
    /// from the first ones -- the samples that arrived meanwhile were never
    /// taken. Needs a ROS environment (it talks DDS within one context); it
    /// returns early without one, like the dispatch test in lib.rs.
    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn the_datum_waits_for_the_delay_and_uses_the_latest_inputs() {
        let ctx = match r2r::Context::create() {
            Ok(c) => c,
            Err(_) => return,
        };
        // A namespace of its own, so the relative input topics cannot meet
        // anything else on the graph.
        const NS: &str = "/navsat_delay_test";
        const DELAY: f64 = 1.5;
        const SWITCH: f64 = 0.6;

        let node = r2r::Node::create(ctx.clone(), crate::NODE_NAVSAT, NS).unwrap();
        for (key, value) in [
            ("delay", r2r::ParameterValue::Double(DELAY)),
            ("frequency", r2r::ParameterValue::Double(30.0)),
            ("broadcast_cartesian_transform", r2r::ParameterValue::Bool(true)),
        ] {
            node.params.lock().unwrap().insert(key.to_string(), r2r::Parameter::new(value));
        }
        let shutdown = mower_rs_common::Shutdown::new();
        let mut m = ModuleCtx::new(crate::NODE_NAVSAT, shutdown.clone());
        m.namespace = NS.to_string();

        let mut feeder = r2r::Node::create(ctx, "navsat_delay_feeder", NS).unwrap();
        let qos = || QosProfile::default().keep_last(10).reliable().volatile();
        let odom_pub = feeder.create_publisher::<ROdometry>(ODOM_TOPIC, qos()).unwrap();
        let imu_pub = feeder.create_publisher::<RImu>(IMU_TOPIC, qos()).unwrap();
        let fix_pub = feeder.create_publisher::<RNavSatFix>(FIX_TOPIC, qos()).unwrap();
        let mut tf_static = feeder.subscribe::<TFMessage>("/tf_static", tf_static_qos()).unwrap();

        let started = tokio::time::Instant::now();
        let navsat = tokio::spawn(run_node(node, m));

        let feeding = Arc::new(AtomicBool::new(true));
        let spin = {
            let feeding = feeding.clone();
            tokio::task::spawn_blocking(move || {
                while feeding.load(Ordering::Relaxed) {
                    feeder.spin_once(Duration::from_millis(10));
                }
            })
        };
        // Two phases, both inside the delay. Phase 1 is what a node that
        // locks on its first tick would use; phase 2 keeps being published,
        // so it is the latest of each input when the delay is over.
        let feed = {
            let feeding = feeding.clone();
            tokio::spawn(async move {
                let mut k = 0i64;
                while feeding.load(Ordering::Relaxed) {
                    let phase2 = started.elapsed().as_secs_f64() >= SWITCH;
                    let (odom_yaw, imu_yaw, lat, lon) = if phase2 {
                        (0.1, 1.2, 23.6950, 120.5387)
                    } else {
                        (0.0, 0.3, 23.6940, 120.5377)
                    };
                    let stamp = r2r::builtin_interfaces::msg::Time { sec: 1000 + (k / 20) as i32, nanosec: ((k % 20) * 50_000_000) as u32 };
                    let mut odom = ROdometry::default();
                    odom.header.stamp = stamp.clone();
                    odom.header.frame_id = "map".to_string();
                    odom.child_frame_id = "base_footprint".to_string();
                    odom.pose.pose.orientation = yaw_quat(odom_yaw);
                    let _ = odom_pub.publish(&odom);
                    let mut imu = RImu::default();
                    imu.header.stamp = stamp.clone();
                    imu.header.frame_id = "base_footprint".to_string();
                    imu.orientation = yaw_quat(imu_yaw);
                    let _ = imu_pub.publish(&imu);
                    let mut fix = RNavSatFix::default();
                    fix.header.stamp = stamp;
                    fix.header.frame_id = "base_footprint".to_string();
                    fix.status.status = 0; // STATUS_FIX
                    fix.latitude = lat;
                    fix.longitude = lon;
                    fix.altitude = 50.0;
                    fix.position_covariance[0] = 0.04;
                    fix.position_covariance[4] = 0.04;
                    fix.position_covariance[8] = 0.09;
                    let _ = fix_pub.publish(&fix);
                    k += 1;
                    tokio::time::sleep(Duration::from_millis(50)).await;
                }
            })
        };

        // The datum announces itself as map -> utm on /tf_static.
        let locked = tokio::time::timeout(Duration::from_secs_f64(DELAY + 5.0), async {
            while let Some(msg) = tf_static.next().await {
                if let Some(t) = msg.transforms.iter().find(|t| t.child_frame_id == CARTESIAN_FRAME) {
                    return Some((started.elapsed().as_secs_f64(), t.clone()));
                }
            }
            None
        })
        .await;

        feeding.store(false, Ordering::Relaxed);
        shutdown.trigger();
        let _ = feed.await;
        let _ = spin.await;
        navsat.await.expect("navsat task").expect("navsat result");

        let (at, t) = locked.expect("no datum within the delay + 5 s").expect("tf_static closed");
        assert!(at >= DELAY, "datum locked after {at:.3} s, before the {DELAY} s delay");
        assert_eq!(t.header.frame_id, "map");
        let q = &t.transform.rotation;
        let yaw = (2.0 * (q.w * q.z + q.x * q.y)).atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        // map->utm yaw = odometry yaw - (IMU yaw + meridian convergence); the
        // convergence at 120.54 E in zone 51 is -0.99 deg (-0.0173 rad).
        let convergence = -0.0173;
        let latest = 0.1 - (1.2 + convergence);
        let first = 0.0 - (0.3 + convergence);
        assert!(
            (yaw - latest).abs() < 0.01,
            "datum yaw {yaw:.4} rad: expected the latest inputs' {latest:.4}, the first inputs give {first:.4}"
        );
    }

    #[test]
    fn nothing_is_published_before_a_datum_exists() {
        let mut core = NavSatTransformCore::new(Default::default());
        assert!(!core.transform_good);
        assert!(core.prepare_gps_odometry().is_none());
        assert!(core.prepare_filtered_gps().is_none());
        assert!(core.from_ll(23.0, 120.0, 0.0).is_none());
    }
}
