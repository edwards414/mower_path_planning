//! `navsat_transform`: the ROS shell around
//! `mower_localize_core::navsat::NavSatTransformCore`.
//!
//! The datum, the UTM series, the antenna-offset removal and both published
//! messages are the core's, checked against the real `navsat_transform` to
//! 1.6e-9 m. This file is `navsat_transform_node.cpp`'s wiring: three
//! subscriptions, two publishers, the `utm` static transform, the four
//! services and the `frequency` timer (`transformCallback`).
//!
//! The topics, the four services and the timer are on the in-process bus
//! (step E2 of docs/ROS_FREE_PLAN.md), under the names rcl gives them on the
//! node. The r2r node stays for the parameters, their services, the log, the
//! node's place in the graph, and the `utm` frame: that one is published on
//! `/tf_static` over DDS until robot_state_publisher's frames join it on the
//! bus in E4 (the bridge brings `/tf_static` onto the bus, so it cannot also
//! take it off; mower_bus_ros README, "The /tf_static question").

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use futures::future::{BoxFuture, FutureExt};
use futures::stream::StreamExt;
use mower_bus::Qos;
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
fn input_qos() -> Qos {
    Qos::default().keep_last(1).best_effort().volatile()
}

fn output_qos() -> Qos {
    Qos::default().keep_last(10).reliable().volatile()
}

fn tf_static_qos() -> Qos {
    Qos::default().keep_last(100).reliable().transient_local()
}

pub async fn run(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    let node = r2r::Node::create(ctx, &m.node_name, &m.namespace)?;
    run_node(node, m).await
}

/// Everything after `Node::create`, so a test can hand in a node whose
/// parameters it has set itself.
pub(crate) async fn run_node(mut node: r2r::Node, m: ModuleCtx) -> ModuleResult {
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

    // What takes from the readers and serves the services. Bus readers and
    // services queue from the moment they exist, but nothing is taken from
    // them until these run, which is after `delay` (below): what an r2r node
    // that is not spun yet does, and what the depth-1 readers rely on.
    let mut deferred: Vec<BoxFuture<'static, ()>> = Vec::new();

    // ---- static sensor offsets (base_link <- gps_link / imu_link) ----------
    {
        let mut stream = m.bus.subscribe::<TFMessage>(&m.resolve_topic("/tf_static")?, tf_static_qos())?;
        deferred.push(async move {
            while let Some(msg) = stream.next().await {
                for t in &msg.transforms {
                    tfbus::set(
                        &t.header.frame_id,
                        &t.child_frame_id,
                        conv::transform_in(&t.transform),
                    );
                }
            }
        }.boxed());
    }

    // ---- inputs -------------------------------------------------------------
    {
        let mut stream = m.bus.subscribe::<ROdometry>(&m.resolve_topic(ODOM_TOPIC)?, input_qos())?;
        let nav = nav.clone();
        deferred.push(async move {
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
        }.boxed());
    }
    {
        // `gps/fix` is best-effort sensor data, like the driver publishes it.
        let mut stream = m.bus.subscribe::<RNavSatFix>(&m.resolve_topic(FIX_TOPIC)?, input_qos())?;
        let nav = nav.clone();
        deferred.push(async move {
            while let Some(msg) = stream.next().await {
                let converted = conv::fix_in(&msg);
                let mut n = nav.lock().await;
                n.sync_transforms();
                n.core.gps_fix_callback(&converted);
            }
        }.boxed());
    }
    // Upstream drops the IMU subscription once the transform is good (and the
    // yaml has use_odometry_yaw false, no manual datum), so the flag below
    // reproduces that: the yaw is only needed to build the datum.
    let imu_done = Arc::new(AtomicBool::new(false));
    {
        let mut stream = m.bus.subscribe::<RImu>(&m.resolve_topic(IMU_TOPIC)?, input_qos())?;
        let nav = nav.clone();
        let imu_done = imu_done.clone();
        deferred.push(async move {
            while let Some(msg) = stream.next().await {
                if imu_done.load(Ordering::Relaxed) {
                    continue;
                }
                let converted = conv::imu_in(&msg);
                let mut n = nav.lock().await;
                n.sync_transforms();
                n.core.imu_callback(&converted);
            }
        }.boxed());
    }

    // ---- outputs ------------------------------------------------------------
    let gps_odom_pub = m.bus.publisher::<ROdometry>(&m.resolve_topic(GPS_ODOM_TOPIC)?, output_qos())?;
    let filtered_gps_pub = if cfg.publish_filtered_gps {
        Some(m.bus.publisher::<RNavSatFix>(&m.resolve_topic(FILTERED_GPS_TOPIC)?, output_qos())?)
    } else {
        None
    };
    // tf2_ros::StaticTransformBroadcaster: QoS(1), transient local. On DDS
    // until E4 (module comment).
    let cartesian_pub = node.create_publisher::<TFMessage>(
        "/tf_static",
        QosProfile::default().keep_last(1).reliable().transient_local(),
    )?;

    // ---- services -----------------------------------------------------------
    {
        let mut stream = m.bus.service::<ToLL::Request, ToLL::Response>(&m.resolve_service("/toLL")?)?;
        let nav = nav.clone();
        deferred.push(async move {
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
        }.boxed());
    }
    {
        let mut stream = m.bus.service::<FromLL::Request, FromLL::Response>(&m.resolve_service("/fromLL")?)?;
        let nav = nav.clone();
        let logger = logger.clone();
        deferred.push(async move {
            while let Some(req) = stream.next().await {
                let ll = &req.message.ll_point;
                let map_point = from_ll(&nav, ll, &logger).await;
                let _ = req.respond(FromLL::Response { map_point });
            }
        }.boxed());
    }
    {
        let mut stream =
            m.bus.service::<FromLLArray::Request, FromLLArray::Response>(&m.resolve_service("/fromLLArray")?)?;
        let nav = nav.clone();
        let logger = logger.clone();
        deferred.push(async move {
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
        }.boxed());
    }
    {
        let mut stream = m.bus.service::<SetDatum::Request, SetDatum::Response>(&m.resolve_service("/datum")?)?;
        let nav = nav.clone();
        let logger = logger.clone();
        deferred.push(async move {
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
        }.boxed());
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
    // but the node is not spun and nothing is taken from the bus until the
    // delay has passed, so no callback and no service runs meanwhile, and the
    // depth-1 readers keep only the latest odometry, fix and IMU sample. The
    // datum is therefore built from inputs at least `delay` old -- by then the
    // map EKF's yaw has had time to converge on the IMU heading when it was
    // initialised from /odom (yaw 0). Locking on the first tick instead
    // rotated the map<->UTM datum by most of the robot's initial heading.
    if delay > 0.0 {
        r2r::log_info!(&logger, "Delaying for {delay} seconds before starting...");
        tokio::select! {
            // Nothing is spinning or running yet, so returning drops the node
            // and the bus endpoints cleanly.
            _ = m.shutdown.wait() => return Ok(()),
            _ = m.clock.sleep(Duration::from_secs_f64(delay)) => {}
        }
        r2r::log_info!(&logger, "Delay elapsed. Continuing.");
    }
    for task in deferred {
        tokio::spawn(task);
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
    // one period later: the tasks started above take the waiting samples
    // before the datum is attempted. A tokio interval would tick at once. The
    // order those three samples are taken in is not fixed upstream either
    // (rclcpp's executor keys its entities by handle address in an
    // unordered_map): an IMU sample handled before the odometry is dropped,
    // as `imuCallback` drops it, and the datum waits for the next one, up to
    // 0.1 s at 10 Hz.
    let mut tick = m.bus.wall_timer(&m.clock, Duration::from_secs_f64(1.0 / frequency))?;
    loop {
        tokio::select! {
            _ = m.shutdown.wait() => break,
            ticked = tick.tick() => {
                // Closed under the node (its instance ended): nothing
                // published arrives any more.
                if ticked.is_err() {
                    break;
                }
            }
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
                    let stamped = conv::transform_stamped(conv::now_ns(&m.clock), parent, child, &t);
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
    use mower_bus::{Bus, Clock, ManualClock, Mono, ResponseFuture, Scope, Time};

    use super::*;
    use crate::testing;

    fn yaw_quat(yaw: f64) -> r2r::geometry_msgs::msg::Quaternion {
        r2r::geometry_msgs::msg::Quaternion {
            x: 0.0,
            y: 0.0,
            z: (yaw / 2.0).sin(),
            w: (yaw / 2.0).cos(),
        }
    }

    /// For anything that waits on the module's tasks. A pass never waits
    /// this long.
    const WAIT: Duration = Duration::from_secs(20);

    /// `navsat_transform` on a bus of its own, under the production rules,
    /// on a manual clock: its inputs come from the `feeder` scope, as from
    /// the EKF, the GPS driver and the IMU driver in mower_rsd.
    struct Navsat {
        bus: Bus,
        feeder: Scope,
        manual: ManualClock,
        shutdown: mower_rs_common::Shutdown,
        task: tokio::task::JoinHandle<ModuleResult>,
    }

    impl Navsat {
        fn start(ctx: &r2r::Context, ns: &str, params: &[(&str, r2r::ParameterValue)]) -> Navsat {
            let bus = Bus::new();
            let (clock, manual) = Clock::manual(Time::from_ns(1_790_000_000_000_000_000));
            let m = testing::module(crate::NODE_NAVSAT, &bus.scope(crate::NODE_NAVSAT), &clock);
            let shutdown = m.shutdown.clone();
            let node = testing::node(ctx, crate::NODE_NAVSAT, ns, params);
            let task = tokio::spawn(run_node(node, m));
            Navsat { feeder: bus.scope("feeder"), bus, manual, shutdown, task }
        }

        /// Wait until the node has made its endpoints and armed its first
        /// deadline (the delay, or the timer), and return that deadline.
        async fn armed(&self) -> Mono {
            let deadline = tokio::time::Instant::now() + WAIT;
            loop {
                if let Some(next) = self.manual.next_deadline() {
                    return next;
                }
                assert!(tokio::time::Instant::now() < deadline, "navsat never armed a deadline");
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        }

        async fn stop(self) {
            self.shutdown.trigger();
            tokio::time::timeout(WAIT, self.task)
                .await
                .expect("navsat did not stop")
                .expect("navsat task")
                .expect("navsat result");
        }
    }

    /// Odometry, IMU and fix as the map EKF and the two drivers publish
    /// them, in the frames `world` and `base` of the test's own: the
    /// transform cache is process-wide.
    struct Inputs {
        odom: mower_bus::Publisher<ROdometry>,
        imu: mower_bus::Publisher<RImu>,
        fix: mower_bus::Publisher<RNavSatFix>,
        world: &'static str,
        base: &'static str,
        k: i64,
    }

    impl Inputs {
        fn new(feeder: &Scope, world: &'static str, base: &'static str) -> Inputs {
            let qos = Qos::default();
            Inputs {
                odom: feeder.publisher("/odometry/global", qos).unwrap(),
                imu: feeder.publisher("/imu/data", qos).unwrap(),
                fix: feeder.publisher("/fix", qos).unwrap(),
                world,
                base,
                k: 0,
            }
        }

        fn publish(&mut self, odom_yaw: f64, imu_yaw: f64, lat: f64, lon: f64) {
            let k = self.k;
            self.k += 1;
            // 20 samples a second from t = 1000 s
            let stamp = conv::from_ns(1_000_000_000_000 + k * 50_000_000);
            let mut odom = ROdometry::default();
            odom.header.stamp = stamp.clone();
            odom.header.frame_id = self.world.to_string();
            odom.child_frame_id = self.base.to_string();
            odom.pose.pose.orientation = yaw_quat(odom_yaw);
            self.odom.publish(&odom).unwrap();
            let mut imu = RImu::default();
            imu.header.stamp = stamp.clone();
            imu.header.frame_id = self.base.to_string();
            imu.orientation = yaw_quat(imu_yaw);
            self.imu.publish(&imu).unwrap();
            let mut fix = RNavSatFix::default();
            fix.header.stamp = stamp;
            fix.header.frame_id = self.base.to_string();
            fix.status.status = 0; // STATUS_FIX
            fix.latitude = lat;
            fix.longitude = lon;
            fix.altitude = 50.0;
            fix.position_covariance = vec![0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.09];
            self.fix.publish(&fix).unwrap();
        }
    }

    fn yaw_of(q: &Quaternion) -> f64 {
        (2.0 * (q.w * q.z + q.x * q.y)).atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    }

    async fn answer<R: mower_bus::Message>(call: ResponseFuture<R>) -> R {
        tokio::time::timeout(WAIT, call).await.expect("no answer").expect("an answer")
    }

    /// `delay` as upstream applies it: no datum before the delay has passed,
    /// and then the one built from the *latest* odometry, IMU and fix, not
    /// from the first ones -- the samples that arrived meanwhile were never
    /// taken. No service is answered meanwhile either. On the bus with a
    /// manual clock, so "before" and "after" are exact.
    #[test]
    fn the_datum_waits_for_the_delay_and_uses_the_latest_inputs() {
        testing::run(async {
            let Some(ctx) = testing::context() else { return };
            const DELAY: Duration = Duration::from_millis(1500);
            const SWITCH: Duration = Duration::from_millis(600);
            const WORLD: &str = "map_navsat_delay_test";
            let navsat = Navsat::start(&ctx, "/navsat_delay_test", &[
                ("delay", r2r::ParameterValue::Double(DELAY.as_secs_f64())),
                ("frequency", r2r::ParameterValue::Double(30.0)),
                ("broadcast_cartesian_transform", r2r::ParameterValue::Bool(true)),
            ]);
            assert_eq!(navsat.armed().await, Mono::from_ns(0) + DELAY, "the delay, from the start");
            let mut inputs = Inputs::new(&navsat.feeder, WORLD, "base_navsat_delay_test");
            let mut gps_odom = navsat.feeder.subscribe::<ROdometry>("/odometry/gps", Qos::default()).unwrap();
            let mut filtered = navsat.feeder.subscribe::<RNavSatFix>("/gps/filtered", Qos::default()).unwrap();
            let to_ll = navsat.feeder.client::<ToLL::Request, ToLL::Response>("/toLL").unwrap();
            let mut asked = Box::pin(to_ll.request(&ToLL::Request::default()).unwrap());

            // Two phases, both inside the delay. Phase 1 is what a node that
            // locks on its first tick would use; phase 2 is the latest of each
            // input when the delay is over.
            for _ in 0..12 {
                inputs.publish(0.0, 0.3, 23.6940, 120.5377);
            }
            navsat.manual.advance(SWITCH);
            for _ in 0..18 {
                inputs.publish(0.1, 1.2, 23.6950, 120.5387);
            }
            navsat.manual.advance(DELAY - SWITCH - Duration::from_millis(1));
            tokio::time::sleep(Duration::from_millis(100)).await;
            assert!(tfbus::last_set(WORLD, CARTESIAN_FRAME).is_none(), "a datum before the delay");
            assert!(futures::poll!(asked.as_mut()).is_pending(), "/toLL answered during the delay");
            assert!(gps_odom.try_recv().is_err());

            // The delay ends: the waiting samples are taken, then the timer
            // ticks one period later. An IMU sample taken before the odometry
            // is dropped (upstream's imuCallback), so the inputs keep coming.
            navsat.manual.advance(Duration::from_millis(1));
            tokio::time::timeout(WAIT, asked.as_mut()).await.expect("/toLL after the delay").unwrap();
            let mut ticks = 0;
            while tfbus::last_set(WORLD, CARTESIAN_FRAME).is_none() {
                assert!(ticks < 300, "no datum within 300 ticks of the delay");
                inputs.publish(0.1, 1.2, 23.6950, 120.5387);
                tokio::time::sleep(Duration::from_millis(5)).await;
                navsat.manual.advance(Duration::from_secs_f64(1.0 / 30.0));
                tokio::time::sleep(Duration::from_millis(5)).await;
                ticks += 1;
            }
            let t = tfbus::lookup(WORLD, CARTESIAN_FRAME).expect("map -> utm in the transform cache");
            let yaw = yaw_of(&t.rotation());
            // map->utm yaw = odometry yaw - (IMU yaw + meridian convergence); the
            // convergence at 120.54 E in zone 51 is -0.99 deg (-0.0173 rad).
            let convergence = -0.0173;
            let latest = 0.1 - (1.2 + convergence);
            let first = 0.0 - (0.3 + convergence);
            assert!(
                (yaw - latest).abs() < 0.01,
                "datum yaw {yaw:.4} rad: expected the latest inputs' {latest:.4}, the first inputs give {first:.4}"
            );

            // From the next tick on, the fixes go out in the map frame.
            inputs.publish(0.1, 1.2, 23.6950, 120.5387);
            let mut out = None;
            for _ in 0..300 {
                tokio::time::sleep(Duration::from_millis(5)).await;
                navsat.manual.advance(Duration::from_secs_f64(1.0 / 30.0));
                tokio::time::sleep(Duration::from_millis(5)).await;
                if let Ok(msg) = gps_odom.try_recv() {
                    out = Some(msg);
                    break;
                }
            }
            let out = out.expect("no /odometry/gps after the datum");
            // upstream's prepareGpsOdometry sets no child frame
            assert_eq!((out.header.frame_id.as_str(), out.child_frame_id.as_str()), (WORLD, ""));
            let fix = tokio::time::timeout(WAIT, filtered.recv()).await.expect("no /gps/filtered").unwrap();
            assert_eq!(fix.header.frame_id, "base_navsat_delay_test");
            assert!((fix.latitude - 23.6950).abs() < 1e-3, "{fix:?}");
            navsat.stop().await;
        });
    }

    /// A shutdown during the delay ends the node at once, instead of after the
    /// delay (mower_rsd restarts a module in place, and a stop must not hang
    /// for `delay` seconds).
    #[test]
    fn a_shutdown_during_the_delay_ends_the_node_at_once() {
        testing::run(async {
            let Some(ctx) = testing::context() else { return };
            let navsat = Navsat::start(&ctx, "/navsat_delay_shutdown_test", &[
                ("delay", r2r::ParameterValue::Double(30.0)),
                ("frequency", r2r::ParameterValue::Double(30.0)),
            ]);
            navsat.armed().await;
            tokio::time::sleep(Duration::from_millis(100)).await;
            assert!(!navsat.task.is_finished(), "returned before the shutdown");
            let asked = tokio::time::Instant::now();
            navsat.stop().await;
            assert!(asked.elapsed() < Duration::from_secs(1));
        });
    }

    /// The four services on the bus, under their absolute names: before the
    /// datum they answer what upstream's default responses are, and a
    /// manual datum (`/datum`, as datum_check.py --fix sends it) is what
    /// `/toLL` and `/fromLL` then project through.
    #[test]
    fn the_services_answer_on_the_bus_before_and_after_a_manual_datum() {
        testing::run(async {
            let Some(ctx) = testing::context() else { return };
            let navsat = Navsat::start(&ctx, "/navsat_services_test", &[
                ("delay", r2r::ParameterValue::Double(0.0)),
                ("frequency", r2r::ParameterValue::Double(30.0)),
                // the cache is process-wide: no `utm` edge from this test
                ("broadcast_cartesian_transform", r2r::ParameterValue::Bool(false)),
            ]);
            navsat.armed().await;
            let scope = &navsat.feeder;
            let to_ll = scope.client::<ToLL::Request, ToLL::Response>("/toLL").unwrap();
            let from_ll = scope.client::<FromLL::Request, FromLL::Response>("/fromLL").unwrap();
            let from_ll_array = scope.client::<FromLLArray::Request, FromLLArray::Response>("/fromLLArray").unwrap();
            let datum = scope.client::<SetDatum::Request, SetDatum::Response>("/datum").unwrap();
            let here = GeoPoint { latitude: 25.0330, longitude: 121.5654, altitude: 10.0 };

            let ll = answer(to_ll.request(&ToLL::Request::default()).unwrap()).await.ll_point;
            assert_eq!((ll.latitude, ll.longitude, ll.altitude), (0.0, 0.0, 0.0));
            let p = answer(from_ll.request(&FromLL::Request { ll_point: here.clone() }).unwrap()).await.map_point;
            assert_eq!((p.x, p.y, p.z), (0.0, 0.0, 0.0));
            let points = from_ll_array.request(&FromLLArray::Request { ll_points: vec![here.clone()] }).unwrap();
            assert!(answer(points).await.map_points.is_empty());

            let mut set = SetDatum::Request::default();
            set.geo_pose.position = here.clone();
            set.geo_pose.orientation.w = 1.0;
            answer(datum.request(&set).unwrap()).await;
            // The next tick computes the transform from the manual datum, as
            // soon as the map EKF's odometry has been seen (computeTransform).
            let mut odom = ROdometry::default();
            odom.header.frame_id = "map_navsat_services_test".to_string();
            odom.child_frame_id = "base_navsat_services_test".to_string();
            odom.pose.pose.orientation.w = 1.0;
            scope.publisher::<ROdometry>("/odometry/global", Qos::default()).unwrap().publish(&odom).unwrap();
            let mut origin = (0.0, 0.0);
            for _ in 0..300 {
                navsat.manual.advance(Duration::from_secs_f64(1.0 / 30.0));
                tokio::time::sleep(Duration::from_millis(5)).await;
                let ll = answer(to_ll.request(&ToLL::Request::default()).unwrap()).await.ll_point;
                origin = (ll.latitude, ll.longitude);
                if origin != (0.0, 0.0) {
                    break;
                }
            }
            assert!((origin.0 - here.latitude).abs() < 1e-7 && (origin.1 - here.longitude).abs() < 1e-7, "{origin:?}");
            let p = answer(from_ll.request(&FromLL::Request { ll_point: here.clone() }).unwrap()).await.map_point;
            assert!(p.x.abs() < 1e-3 && p.y.abs() < 1e-3, "{p:?}");
            let north = GeoPoint { latitude: here.latitude + 0.001, ..here.clone() };
            let points = from_ll_array.request(&FromLLArray::Request { ll_points: vec![here, north] }).unwrap();
            let points = answer(points).await.map_points;
            assert_eq!(points.len(), 2);
            // 0.001 deg of latitude is about 111 m, along map +y (no yaw offset)
            assert!((points[1].y - 110.8).abs() < 1.0 && points[1].x.abs() < 1.0, "{points:?}");

            let services: Vec<_> = navsat.bus.services().into_iter().filter(|s| s.server.is_some()).collect();
            let served: Vec<(&str, &str)> =
                services.iter().map(|s| (s.name.as_str(), s.server.as_deref().unwrap())).collect();
            assert_eq!(
                served,
                [
                    ("/datum", crate::NODE_NAVSAT),
                    ("/fromLL", crate::NODE_NAVSAT),
                    ("/fromLLArray", crate::NODE_NAVSAT),
                    ("/toLL", crate::NODE_NAVSAT),
                ]
            );
            assert_eq!(services[0].ros_type.as_deref(), Some("robot_localization/srv/SetDatum"));
            navsat.stop().await;
        });
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
