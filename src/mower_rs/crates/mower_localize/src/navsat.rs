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
use mower_localize_core::config;
use mower_localize_core::navsat::NavSatTransformCore;
use mower_localize_core::tf::{Quaternion, Vector3};
use mower_rs_common::{params, ModuleCtx, ModuleResult};
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

fn input_qos(depth: usize) -> QosProfile {
    QosProfile::default().keep_last(depth).best_effort().volatile()
}

fn output_qos() -> QosProfile {
    QosProfile::default().keep_last(10).reliable().volatile()
}

fn tf_static_qos() -> QosProfile {
    QosProfile::default().keep_last(100).reliable().transient_local()
}

pub async fn run(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    let mut node = r2r::Node::create(ctx, &m.node_name, &m.namespace)?;
    let logger = node.logger().to_string();
    let cfg = config::navsat_config();

    // Resolved topic names (see the note in ekf.rs: upstream gets these from
    // launch `remappings=`, this module from parameters, defaulting to the
    // production wiring).
    let fix_topic = params::string(&node, "gps_fix_topic", "fix");
    let imu_topic = params::string(&node, "imu_topic", "imu/data");
    let odom_topic = params::string(&node, "odometry_topic", "odometry/global");
    let gps_odom_topic = params::string(&node, "gps_odometry_topic", "odometry/gps");
    let filtered_gps_topic = params::string(&node, "filtered_gps_topic", "gps/filtered");
    let frequency = params::f64(&node, "frequency", 30.0).max(1.0);
    let delay = params::f64(&node, "delay", 3.0);

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
        let mut stream = node.subscribe::<ROdometry>(&odom_topic, input_qos(10))?;
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
        let mut stream = node.subscribe::<RNavSatFix>(&fix_topic, input_qos(10))?;
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
        let mut stream = node.subscribe::<RImu>(&imu_topic, input_qos(10))?;
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
    let gps_odom_pub = node.create_publisher::<ROdometry>(&gps_odom_topic, output_qos())?;
    let filtered_gps_pub = if cfg.publish_filtered_gps {
        Some(node.create_publisher::<RNavSatFix>(&filtered_gps_topic, output_qos())?)
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

    paramsrv::advertise(
        &mut node,
        &m.node_name,
        vec![
            paramsrv::double("frequency", frequency),
            paramsrv::double("delay", delay),
            paramsrv::double(
                "magnetic_declination_radians",
                cfg.magnetic_declination_radians,
            ),
            paramsrv::double("yaw_offset", cfg.yaw_offset),
            paramsrv::boolean("zero_altitude", cfg.zero_altitude),
            paramsrv::boolean("broadcast_cartesian_transform", cfg.broadcast_cartesian_transform),
            paramsrv::boolean(
                "broadcast_cartesian_transform_as_parent_frame",
                cfg.broadcast_cartesian_transform_as_parent_frame,
            ),
            paramsrv::boolean("publish_filtered_gps", cfg.publish_filtered_gps),
            paramsrv::boolean("use_odometry_yaw", cfg.use_odometry_yaw),
            paramsrv::boolean("wait_for_datum", cfg.wait_for_datum),
            paramsrv::boolean("use_local_cartesian", false),
            paramsrv::boolean("use_sim_time", false),
            paramsrv::string("gps_fix_topic", &fix_topic),
            paramsrv::string("imu_topic", &imu_topic),
            paramsrv::string("odometry_topic", &odom_topic),
            paramsrv::string("gps_odometry_topic", &gps_odom_topic),
            paramsrv::string("filtered_gps_topic", &filtered_gps_topic),
        ],
    )?;

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

    r2r::log_info!(
        &logger,
        "navsat_transform (mower_rs): {frequency} Hz, fix {fix_topic}, imu {imu_topic}, odom {odom_topic} -> {gps_odom_topic}"
    );

    // `transformCallback`: while the datum is missing, try to build it; once
    // it exists, publish. Never both in the same tick, as upstream's if/else.
    let mut tick = tokio::time::interval(Duration::from_secs_f64(1.0 / frequency));
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
                r2r::log_warn!(&logger, "{gps_odom_topic} publish failed: {e:?}");
            }
        }
        if let (Some(pubr), Some(fix)) = (filtered_gps_pub.as_ref(), filtered) {
            if let Err(e) = pubr.publish(&conv::fix_out(&fix)) {
                r2r::log_warn!(&logger, "{filtered_gps_topic} publish failed: {e:?}");
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

    #[test]
    fn the_yaml_settings_reach_the_core() {
        let cfg = config::navsat_config();
        assert!(cfg.zero_altitude);
        assert!(cfg.publish_filtered_gps);
        assert!(!cfg.use_odometry_yaw);
        assert!(!cfg.wait_for_datum);
        assert!(cfg.broadcast_cartesian_transform);
        assert!(!cfg.broadcast_cartesian_transform_as_parent_frame);
        assert_eq!(cfg.magnetic_declination_radians, 0.0);
        assert_eq!(cfg.yaw_offset, 0.0);
    }

    #[test]
    fn nothing_is_published_before_a_datum_exists() {
        let mut core = NavSatTransformCore::new(config::navsat_config());
        assert!(!core.transform_good);
        assert!(core.prepare_gps_odometry().is_none());
        assert!(core.prepare_filtered_gps().is_none());
        assert!(core.from_ll(23.0, 120.0, 0.0).is_none());
    }
}
