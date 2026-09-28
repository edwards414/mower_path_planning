//! `ekf_filter_node_odom` and `ekf_filter_node_map`: the ROS shell around
//! `mower_localize_core::prepare::RosFilterCore`.
//!
//! Everything numeric is the core's, which is checked against the real
//! `librl_lib.so` to 1e-15 (see `mower_localize_core/README.md`). This file
//! only does what `ros_filter_node.cpp` does around it: subscribe, convert,
//! enqueue, and run `periodicUpdate` on the `frequency` timer.

use std::sync::Arc;
use std::time::Duration;

use futures::stream::StreamExt;
use mower_localize_core::config::{self, EkfConfig};
use mower_localize_core::prepare::{RosFilterCore, SensorConfig};
use mower_rs_common::{params, ModuleCtx, ModuleResult};
use r2r::nav_msgs::msg::Odometry as ROdometry;
use r2r::sensor_msgs::msg::Imu as RImu;
use r2r::tf2_msgs::msg::TFMessage;
use r2r::QosProfile;
use tokio::sync::Mutex;

use crate::conv;
use crate::paramsrv;
use crate::tfbus;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Kind {
    /// `world_frame: odom` -- wheel odometry + IMU, broadcasts `odom -> base`.
    Odom,
    /// `world_frame: map` -- adds `/odometry/gps`, broadcasts `map -> odom`.
    Map,
}

/// What `robot_localization` subscribes to and publishes, after the launch
/// file's `remappings=`. Upstream reads the *raw* names from the yaml
/// (`odom0: odom`, `imu0: imu`) and lets rcl remap them; this module takes the
/// resolved names as parameters instead, so one params file describes the
/// wiring without a node-scoped `-r` rule per topic. The defaults below *are*
/// the production wiring.
struct Topics {
    odom0: String,
    odom1: Option<String>,
    imu0: String,
    out: String,
}

fn topics(node: &r2r::Node, kind: Kind) -> Topics {
    Topics {
        odom0: params::string(node, "odom0", "odom"),
        odom1: match kind {
            Kind::Map => Some(params::string(node, "odom1", "odometry/gps")),
            Kind::Odom => None,
        },
        imu0: params::string(node, "imu0", "imu/data"),
        out: params::string(
            node,
            "odometry_topic",
            match kind {
                Kind::Odom => "odometry/local",
                Kind::Map => "odometry/global",
            },
        ),
    }
}

/// Inputs: `rclcpp::SensorDataQoS().keep_last(queue_size)` -- best effort, as
/// `ros2 topic info -v` reports for the C++ nodes.
fn input_qos(queue_size: usize) -> QosProfile {
    QosProfile::default().keep_last(queue_size).best_effort().volatile()
}

/// Outputs: plain `rclcpp::QoS(10)` -- reliable, volatile.
fn output_qos() -> QosProfile {
    QosProfile::default().keep_last(10).reliable().volatile()
}

/// `tf2_ros::TransformBroadcaster` / `TransformListener`: `QoS(100)`.
fn tf_qos() -> QosProfile {
    QosProfile::default().keep_last(100).reliable().volatile()
}

fn tf_static_qos() -> QosProfile {
    QosProfile::default().keep_last(100).reliable().transient_local()
}

/// `periodicUpdate`'s corrected-data rule. A tick whose filtered state carries
/// a stamp no newer than the last one published -- nothing new was fused and
/// the queue was empty for less than `sensor_timeout`, or the only new
/// measurement was out of order, e.g. a late `/odometry/gps` -- publishes
/// neither the transform nor the odometry, unless
/// `permit_corrected_publication` is set (upstream's default is false and the
/// yaml does not set it). The stamp is remembered either way.
struct PublishGate {
    permit_corrected_publication: bool,
    /// `last_published_stamp_`, `rclcpp::Time(0)` at start.
    last_published_stamp_ns: i64,
}

impl PublishGate {
    fn new(permit_corrected_publication: bool) -> Self {
        Self { permit_corrected_publication, last_published_stamp_ns: 0 }
    }

    /// Whether a state stamped `stamp_ns` goes out.
    fn admit(&mut self, stamp_ns: i64) -> bool {
        let corrected =
            !self.permit_corrected_publication && self.last_published_stamp_ns >= stamp_ns;
        self.last_published_stamp_ns = stamp_ns;
        !corrected
    }
}

/// The filter plus the transform revision its `TransformTree` was built from.
struct Filter {
    core: RosFilterCore,
    tf_revision: Option<u64>,
}

impl Filter {
    /// Refresh the core's `TransformTree` from the shared cache, but only when
    /// something actually moved.
    fn sync_transforms(&mut self) {
        let base = self.core.base_link_frame_id.clone();
        if let Some((rev, tree)) = tfbus::tree_if_changed(self.tf_revision, &base, &[]) {
            self.core.transforms = tree;
            self.tf_revision = Some(rev);
        }
    }
}

fn parameters(config: &EkfConfig, kind: Kind, t: &Topics) -> Vec<paramsrv::Param> {
    let mut noise = Vec::with_capacity(225);
    for row in config.process_noise_covariance.iter() {
        noise.extend_from_slice(row);
    }
    let mut out = vec![
        paramsrv::double("frequency", config.frequency),
        paramsrv::double("sensor_timeout", 1.0 / config.frequency),
        paramsrv::boolean("two_d_mode", config.two_d_mode),
        paramsrv::boolean("publish_tf", config.publish_tf),
        // Both false in dual_ekf_navsat_params.yaml, and not implemented here.
        paramsrv::boolean("print_diagnostics", false),
        paramsrv::boolean("debug", false),
        paramsrv::boolean("publish_acceleration", false),
        paramsrv::boolean("use_control", config.use_control),
        paramsrv::boolean("use_sim_time", false),
        paramsrv::string("map_frame", config.map_frame),
        paramsrv::string("odom_frame", config.odom_frame),
        paramsrv::string("base_link_frame", config.base_link_frame),
        paramsrv::string("world_frame", config.world_frame),
        paramsrv::string("odom0", &t.odom0),
        paramsrv::string("imu0", &t.imu0),
        paramsrv::string("odometry_topic", &t.out),
        paramsrv::integer("odom0_queue_size", 10),
        paramsrv::integer("imu0_queue_size", 10),
        paramsrv::double_array("process_noise_covariance", noise),
    ];
    if kind == Kind::Map {
        out.push(paramsrv::string("odom1", t.odom1.as_deref().unwrap_or("")));
        out.push(paramsrv::integer("odom1_queue_size", 10));
    }
    out
}

pub async fn run(ctx: r2r::Context, m: ModuleCtx, kind: Kind) -> ModuleResult {
    let mut node = r2r::Node::create(ctx, &m.node_name, &m.namespace)?;
    let logger = node.logger().to_string();
    let config = match kind {
        Kind::Odom => config::ekf_odom_config(),
        Kind::Map => config::ekf_map_config(),
    };
    let t = topics(&node, kind);
    let permit_corrected_publication = params::bool(&node, "permit_corrected_publication", false);

    let filter = Arc::new(Mutex::new(Filter {
        core: config::build_filter(&config),
        tf_revision: None,
    }));

    let odom0_config = match kind {
        Kind::Odom => config::odom_ekf_odom0(),
        Kind::Map => config::map_ekf_odom0(),
    };
    let imu0_config = match kind {
        Kind::Odom => config::odom_ekf_imu0(),
        Kind::Map => config::map_ekf_imu0(),
    };

    // ---- static sensor offsets ---------------------------------------------
    // The only lookups the EKF makes are base_link <- sensor frame, and they
    // are rigid links published once, transient-local, by
    // robot_state_publisher.
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

    // ---- measurements: convert and enqueue, nothing else --------------------
    spawn_odom(&mut node, &t.odom0, odom0_config, filter.clone(), &logger)?;
    if let Some(topic) = &t.odom1 {
        spawn_odom(&mut node, topic, config::map_ekf_odom1(), filter.clone(), &logger)?;
    }
    {
        let mut stream = node.subscribe::<RImu>(&t.imu0, input_qos(10))?;
        let filter = filter.clone();
        let cfg = imu0_config;
        tokio::spawn(async move {
            while let Some(msg) = stream.next().await {
                let converted = conv::imu_in(&msg);
                let mut f = filter.lock().await;
                f.sync_transforms();
                f.core.imu_callback(&converted, &cfg);
            }
        });
    }

    // ---- outputs ------------------------------------------------------------
    let odom_pub = node.create_publisher::<ROdometry>(&t.out, output_qos())?;
    let tf_pub = node.create_publisher::<TFMessage>("/tf", tf_qos())?;

    let mut advertised = parameters(&config, kind, &t);
    advertised.push(paramsrv::boolean("permit_corrected_publication", permit_corrected_publication));
    paramsrv::advertise(&mut node, &m.node_name, advertised)?;

    // ---- spin ---------------------------------------------------------------
    let running = Arc::new(std::sync::atomic::AtomicBool::new(true));
    let spin = {
        let running = running.clone();
        tokio::task::spawn_blocking(move || {
            while running.load(std::sync::atomic::Ordering::Relaxed) {
                node.spin_once(Duration::from_millis(50));
            }
            drop(node);
        })
    };

    r2r::log_info!(
        &logger,
        "{} (mower_rs): {} Hz, world_frame {}, odom0 {}, imu0 {}, publishing {}",
        m.node_name,
        config.frequency,
        config.world_frame,
        t.odom0,
        t.imu0,
        t.out
    );

    // `periodicUpdate` on the yaml's frequency. Skip, not Burst: after a
    // scheduling hiccup the filter must resume at the current time, not run a
    // backlog of predicts against stale timestamps (the mower_nav lesson in
    // src/mower_rs/README.md).
    let mut tick = tokio::time::interval(Duration::from_secs_f64(1.0 / config.frequency));
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let mut gate = PublishGate::new(permit_corrected_publication);
    loop {
        tokio::select! {
            _ = m.shutdown.wait() => break,
            _ = tick.tick() => {}
        }
        let now = conv::now_ns();
        let mut f = filter.lock().await;
        f.sync_transforms();
        f.core.integrate_measurements(now);
        f.core.differentiate_measurements(now);
        let Some(odom) = f.core.filtered_odometry() else { continue };
        let map_to_odom = if config.world_frame == config.map_frame {
            f.core.map_to_odom()
        } else {
            None
        };
        drop(f);
        if !gate.admit(odom.header.stamp_ns) {
            // Not even the in-process tfbus: the C++ consumers would not
            // have seen this value on /tf either.
            continue;
        }

        if config.publish_tf {
            // `periodicUpdate` stamps the transform with the filtered
            // position's stamp (tf_time_offset is 0) and broadcasts
            // world_frame -> base_link, or map -> odom on the map instance.
            let stamped = if config.world_frame == config.odom_frame {
                Some(conv::transform_stamped(
                    odom.header.stamp_ns,
                    config.odom_frame,
                    &odom.child_frame_id,
                    &mower_localize_core::tf::Transform::from_parts(
                        &odom.pose.orientation,
                        odom.pose.position,
                    ),
                ))
            } else {
                // Without odom -> base_link there is nothing to compose, and
                // upstream's lookup throws and skips the broadcast too.
                map_to_odom.map(|t| {
                    conv::transform_stamped(
                        odom.header.stamp_ns,
                        config.map_frame,
                        config.odom_frame,
                        &t,
                    )
                })
            };
            if let Some(stamped) = stamped {
                tfbus::set(
                    &stamped.header.frame_id,
                    &stamped.child_frame_id,
                    conv::transform_in(&stamped.transform),
                );
                if let Err(e) = tf_pub.publish(&TFMessage { transforms: vec![stamped] }) {
                    r2r::log_warn!(&logger, "tf publish failed: {e:?}");
                }
            }
        }
        if let Err(e) = odom_pub.publish(&conv::odometry_out(&odom)) {
            r2r::log_warn!(&logger, "{} publish failed: {e:?}", t.out);
        }
    }

    running.store(false, std::sync::atomic::Ordering::Relaxed);
    let _ = spin.await;
    Ok(())
}

fn spawn_odom(
    node: &mut r2r::Node, topic: &str, cfg: SensorConfig, filter: Arc<Mutex<Filter>>,
    _logger: &str,
) -> Result<(), r2r::Error> {
    let mut stream = node.subscribe::<ROdometry>(topic, input_qos(10))?;
    tokio::spawn(async move {
        while let Some(msg) = stream.next().await {
            let converted = conv::odometry_in(&msg);
            let mut f = filter.lock().await;
            f.sync_transforms();
            f.core.odometry_callback(&converted, &cfg);
        }
    });
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_two_instances_keep_the_yaml_s_frames_and_rate() {
        let odom = config::ekf_odom_config();
        let map = config::ekf_map_config();
        assert_eq!(odom.frequency, 20.0);
        assert_eq!(map.frequency, 20.0);
        assert_eq!(odom.world_frame, "odom");
        assert_eq!(map.world_frame, "map");
        assert_eq!(odom.base_link_frame, "base_footprint");
        assert!(odom.publish_tf && map.publish_tf);
        assert!(odom.two_d_mode && map.two_d_mode);
    }

    #[test]
    fn a_state_whose_stamp_did_not_advance_is_not_published_again() {
        let mut gate = PublishGate::new(false);
        let admitted: Vec<bool> = [10, 20, 20, 30, 25, 40].iter().map(|&s| gate.admit(s)).collect();
        assert_eq!(admitted, [true, true, false, true, false, true]);
        // permit_corrected_publication: true publishes every tick.
        let mut permissive = PublishGate::new(true);
        assert!([10, 20, 20, 30, 25].iter().all(|&s| permissive.admit(s)));
    }

    #[test]
    fn two_ticks_without_a_new_measurement_publish_once() {
        // The real filter: one /odom fused, then two ticks less than
        // sensor_timeout (50 ms at 20 Hz) after it. Both report the same
        // stamp, and only the first goes out.
        use mower_localize_core::msgs::{Header, Odometry};
        let cfg = config::ekf_odom_config();
        let mut core = config::build_filter(&cfg);
        let t0 = 1_000_000_000_000i64;
        let mut odom = Odometry {
            header: Header { frame_id: "odom".into(), stamp_ns: t0 },
            child_frame_id: "base_footprint".into(),
            ..Default::default()
        };
        odom.twist.linear.x = 0.3;
        for k in 0..6 {
            odom.twist.covariance[k][k] = 0.01;
        }
        core.odometry_callback(&odom, &config::odom_ekf_odom0());
        let mut gate = PublishGate::new(false);
        let mut published = 0;
        for now in [t0 + 10_000_000, t0 + 30_000_000] {
            core.integrate_measurements(now);
            let stamp = core.filtered_odometry().expect("initialised").header.stamp_ns;
            assert_eq!(stamp, t0);
            if gate.admit(stamp) {
                published += 1;
            }
        }
        assert_eq!(published, 1);
    }

    #[test]
    fn the_parameter_set_reports_what_the_filter_is_really_running() {
        let cfg = config::ekf_map_config();
        let t = Topics {
            odom0: "odom".into(),
            odom1: Some("odometry/gps".into()),
            imu0: "imu/data".into(),
            out: "odometry/global".into(),
        };
        let params = parameters(&cfg, Kind::Map, &t);
        let find = |n: &str| params.iter().find(|p| p.name == n).expect(n);
        assert_eq!(find("frequency").value.double_value, 20.0);
        assert_eq!(find("world_frame").value.string_value, "map");
        assert_eq!(find("odom1").value.string_value, "odometry/gps");
        assert_eq!(find("process_noise_covariance").value.double_array_value.len(), 225);
        assert!(!find("print_diagnostics").value.bool_value);
        // The odom instance has no second odometry input.
        let t_odom = Topics {
            odom0: "odom".into(),
            odom1: None,
            imu0: "imu/data".into(),
            out: "odometry/local".into(),
        };
        let odom_params = parameters(&config::ekf_odom_config(), Kind::Odom, &t_odom);
        assert!(odom_params.iter().all(|p| p.name != "odom1"));
    }
}
