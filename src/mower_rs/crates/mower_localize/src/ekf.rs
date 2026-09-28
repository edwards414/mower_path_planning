//! `ekf_filter_node_odom` and `ekf_filter_node_map`: the ROS shell around
//! `mower_localize_core::prepare::RosFilterCore`.
//!
//! Everything numeric is the core's, which is checked against the real
//! `librl_lib.so` to 1e-15 (see `mower_localize_core/README.md`). This file
//! only does what `ros_filter_node.cpp` does around it: subscribe, convert,
//! enqueue, and run `periodicUpdate` on the `frequency` timer -- plus the
//! 5 Hz slow copy of the map instance's output that `mission.launch.py`'s
//! `global_odom_throttle` makes when the C++ filter runs.

use std::sync::Arc;
use std::time::Duration;

use futures::stream::StreamExt;
use mower_localize_core::config::{EkfSettings, ParamValue, Params, SensorInput};
use mower_localize_core::prepare::RosFilterCore;
use mower_rs_common::throttle::SlowCopy;
use mower_rs_common::{params, ModuleCtx, ModuleResult};
use r2r::nav_msgs::msg::Odometry as ROdometry;
use r2r::sensor_msgs::msg::Imu as RImu;
use r2r::tf2_msgs::msg::TFMessage;
use r2r::QosProfile;
use tokio::sync::Mutex;

use crate::conv;
use crate::paramsrv;
use crate::tfbus;

/// What `ekf_node` publishes before the launch file's `remappings=`
/// (`odometry/local` / `odometry/global`). The inputs are the yaml's own
/// `odomN` / `imuN` values (`odom`, `imu`), and the launch file remaps them
/// with node-scoped `-r` rules exactly as it remaps the C++ nodes.
const OUTPUT_TOPIC: &str = "odometry/filtered";

/// The two keys this port reads that robot_localization has none of: the
/// slow copy of the output that `mission.launch.py`'s `global_odom_throttle`
/// (`topic_tools throttle messages /odometry/global 5.0
/// /odometry/global_slow`) makes when the C++ map filter runs, and does not
/// start when this module runs (`rust_localize`). `flutter_adapter` and
/// `path_record_node` read it as `robot_pose_source_topic`. They come from
/// `mower_rsd.yaml` (or the defaults), are taken out before the resolver sees
/// the section -- which would call them unknown and ignored -- and are
/// reported on the parameter services with the rest.
const SLOW_TOPIC_KEY: &str = "odometry_slow_topic";
const SLOW_RATE_KEY: &str = "odometry_slow_rate_hz";

/// Where the slow copy goes and how often.
struct SlowCopySettings {
    /// Empty turns it off.
    topic: String,
    rate_hz: f64,
}

/// The throttle's output for the node whose `odometry/filtered` the launch
/// files remap to `/odometry/global`; nothing throttles `/odometry/local`.
fn default_slow_topic(node_name: &str) -> &'static str {
    match node_name {
        crate::NODE_EKF_MAP => "odometry/global_slow",
        _ => "",
    }
}

fn slow_copy_settings(node: &r2r::Node, node_name: &str) -> SlowCopySettings {
    SlowCopySettings {
        topic: params::string(node, "odometry_slow_topic", default_slow_topic(node_name)),
        rate_hz: params::f64(node, "odometry_slow_rate_hz", 5.0),
    }
}

/// The node's parameter section without the slow-copy keys: what the
/// resolver gets, i.e. what `ekf_node` would have read.
fn without_slow_copy_keys(mut section: Params) -> Params {
    section.remove(SLOW_TOPIC_KEY);
    section.remove(SLOW_RATE_KEY);
    section
}

/// What the parameter services report: every key the resolver declared with
/// its value in effect, the slow copy's two, and `use_sim_time`.
fn advertised_parameters(
    resolved: &[(String, ParamValue)],
    slow: &SlowCopySettings,
) -> Vec<paramsrv::Param> {
    let mut advertised: Vec<paramsrv::Param> =
        resolved.iter().map(|(name, v)| paramsrv::from_core(name, v)).collect();
    advertised.push(paramsrv::from_core(SLOW_TOPIC_KEY, &ParamValue::String(slow.topic.clone())));
    advertised.push(paramsrv::from_core(SLOW_RATE_KEY, &ParamValue::Double(slow.rate_hz)));
    // rclcpp declares it on every node; the resolver refused anything but false.
    if advertised.iter().all(|p| p.name != "use_sim_time") {
        advertised.push(paramsrv::boolean("use_sim_time", false));
    }
    advertised
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

pub async fn run(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    let mut node = r2r::Node::create(ctx, &m.node_name, &m.namespace)?;
    let logger = node.logger().to_string();
    // Everything the filter does comes from the node's section of
    // dual_ekf_navsat_params.yaml, resolved as `loadParams` would; a value
    // this port does not implement stops it here instead of being dropped.
    let resolved = EkfSettings::from_params(
        &m.node_name,
        &without_slow_copy_keys(paramsrv::overrides(&node)),
    )?;
    for w in &resolved.warnings {
        r2r::log_warn!(&logger, "{w}");
    }
    let config = resolved.settings;

    let filter = Arc::new(Mutex::new(Filter {
        core: config.build_filter(),
        tf_revision: None,
    }));

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
    for input in &config.odoms {
        spawn_odom(&mut node, input, filter.clone())?;
    }
    for input in &config.imus {
        spawn_imu(&mut node, input, filter.clone())?;
    }

    // ---- outputs ------------------------------------------------------------
    let odom_pub = node.create_publisher::<ROdometry>(OUTPUT_TOPIC, output_qos())?;
    // The throttle's own QoS, derived from this publisher's: keep last 10,
    // reliable, volatile (throttle::output_qos). ekf_node's QoS(10) is explicit,
    // so it reads the same under every RMW. A bad topic or rate costs the copy,
    // logged, never the filter.
    let slow = slow_copy_settings(&node, &m.node_name);
    let mut odom_slow =
        SlowCopy::<ROdometry>::create(&mut node, &slow.topic, slow.rate_hz, &output_qos());
    let tf_pub = node.create_publisher::<TFMessage>("/tf", tf_qos())?;

    paramsrv::advertise(
        &mut node,
        &m.node_name,
        advertised_parameters(&resolved.parameters, &slow),
    )?;

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

    let inputs: Vec<String> = config
        .odoms
        .iter()
        .chain(&config.imus)
        .map(|i| format!("{} {}", i.name, i.topic))
        .collect();
    r2r::log_info!(
        &logger,
        "{} (mower_rs): {} Hz, world_frame {}, {}, publishing {}{}",
        m.node_name,
        config.frequency,
        config.world_frame,
        inputs.join(", "),
        OUTPUT_TOPIC,
        if odom_slow.is_some() {
            format!(" and {} at {} Hz", slow.topic, slow.rate_hz)
        } else {
            String::new()
        }
    );

    // `periodicUpdate` on the yaml's frequency. Skip, not Burst: after a
    // scheduling hiccup the filter must resume at the current time, not run a
    // backlog of predicts against stale timestamps (the mower_nav lesson in
    // src/mower_rs/README.md).
    let mut tick = tokio::time::interval(Duration::from_secs_f64(1.0 / config.frequency));
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let mut gate = PublishGate::new(config.permit_corrected_publication);
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
                    &config.odom_frame,
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
                        &config.map_frame,
                        &config.odom_frame,
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
        // Only what passed the gate above reaches here, so the copy never
        // sees a corrected (not newer) state either: the throttle only ever
        // saw what reached the output topic, and the copy is this same
        // message, stamp and all.
        let out = conv::odometry_out(&odom);
        match odom_pub.publish(&out) {
            Ok(()) => {
                if let Some(slow) = odom_slow.as_mut() {
                    slow.offer(&out);
                }
            }
            Err(e) => r2r::log_warn!(&logger, "{OUTPUT_TOPIC} publish failed: {e:?}"),
        }
    }

    running.store(false, std::sync::atomic::Ordering::Relaxed);
    let _ = spin.await;
    Ok(())
}

fn spawn_odom(
    node: &mut r2r::Node, input: &SensorInput, filter: Arc<Mutex<Filter>>,
) -> Result<(), r2r::Error> {
    let mut stream = node.subscribe::<ROdometry>(&input.topic, input_qos(input.queue_size))?;
    let cfg = input.config.clone();
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

fn spawn_imu(
    node: &mut r2r::Node, input: &SensorInput, filter: Arc<Mutex<Filter>>,
) -> Result<(), r2r::Error> {
    let mut stream = node.subscribe::<RImu>(&input.topic, input_qos(input.queue_size))?;
    let cfg = input.config.clone();
    tokio::spawn(async move {
        while let Some(msg) = stream.next().await {
            let converted = conv::imu_in(&msg);
            let mut f = filter.lock().await;
            f.sync_transforms();
            f.core.imu_callback(&converted, &cfg);
        }
    });
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use mower_localize_core::config::{ParamValue as V, Params};

    /// The map instance's section of dual_ekf_navsat_params.yaml, abridged.
    fn map_section() -> Params {
        let t = true;
        let f = false;
        [
            ("frequency", V::Double(20.0)),
            ("two_d_mode", V::Bool(true)),
            ("world_frame", V::String("map".into())),
            ("base_link_frame", V::String("base_footprint".into())),
            ("odom0", V::String("odom".into())),
            ("odom0_config", V::BoolArray(vec![f, f, f, f, f, f, t, t, t, f, f, t, f, f, f])),
            ("odom1", V::String("odometry/gps".into())),
            ("odom1_config", V::BoolArray(vec![t, t, f, f, f, f, f, f, f, f, f, f, f, f, f])),
            ("imu0", V::String("imu".into())),
            ("imu0_config", V::BoolArray(vec![f, f, f, f, f, t, f, f, f, f, f, f, f, f, f])),
            ("imu0_remove_gravitational_acceleration", V::Bool(true)),
        ]
        .into_iter()
        .map(|(k, v)| (k.to_string(), v))
        .collect()
    }

    #[test]
    fn the_parameter_services_report_what_ekf_node_would() {
        let r = EkfSettings::from_params("ekf_filter_node_map", &map_section()).unwrap();
        let params: Vec<paramsrv::Param> =
            r.parameters.iter().map(|(n, v)| paramsrv::from_core(n, v)).collect();
        let find = |n: &str| &params.iter().find(|p| p.name == n).unwrap_or_else(|| panic!("{n}")).value;
        // The yaml's values, before remapping, as the C++ node answers them.
        assert_eq!(find("odom0").string_value, "odom");
        assert_eq!(find("imu0").string_value, "imu");
        assert_eq!(find("odom1").string_value, "odometry/gps");
        assert_eq!(find("odom1_config").bool_array_value[..2], [true, true]);
        assert!(find("imu0_remove_gravitational_acceleration").bool_value);
        assert_eq!(find("frequency").double_value, 20.0);
        assert_eq!(find("world_frame").string_value, "map");
        // Declared defaults are listed too, as rclcpp lists them.
        assert!(!find("permit_corrected_publication").bool_value);
        assert_eq!(find("transform_time_offset").double_value, 0.0);
        assert_eq!(find("gravitational_acceleration").double_value, 9.80665);
        assert_eq!(find("sensor_timeout").double_value, 1.0 / 20.0);
        assert_eq!(find("odom1_queue_size").integer_value, 10);
        // Declared unconditionally by loadParams, absent from the yaml.
        assert_eq!(find("history_length").type_, paramsrv::PARAMETER_DOUBLE);
        assert_eq!(find("history_length").double_value, 0.0);
        assert!(!find("reset_on_time_jump").bool_value);
        assert!(!find("stamped_control").bool_value);
        assert_eq!(find("control_timeout").double_value, 0.0);
        // Only with debug: true, as upstream.
        assert!(params.iter().all(|p| p.name != "debug_out_file"));
        // Nothing mower_localize invented.
        assert!(params.iter().all(|p| p.name != "odometry_topic"));
        assert_eq!(r.settings.odoms.len(), 2);
        assert_eq!(r.settings.imus.len(), 1);
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
        let settings = EkfSettings::from_params("ekf_filter_node_map", &map_section()).unwrap().settings;
        let mut core = settings.build_filter();
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
        core.odometry_callback(&odom, &settings.odoms[0].config);
        let mut gate = PublishGate::new(settings.permit_corrected_publication);
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

    /// The slow-copy keys are reported with the rest and never reach the
    /// resolver, which would warn that they are not robot_localization's.
    #[test]
    fn the_parameter_set_reports_the_slow_copy_the_filter_is_really_running() {
        let mut section = map_section();
        section.insert(SLOW_TOPIC_KEY.into(), V::String("/odometry/global_slow".into()));
        section.insert(SLOW_RATE_KEY.into(), V::Double(5.0));
        let r = EkfSettings::from_params("ekf_filter_node_map", &without_slow_copy_keys(section))
            .unwrap();
        assert!(r.warnings.iter().all(|w| !w.contains("odometry_slow")), "{:?}", r.warnings);
        assert!(r.parameters.iter().all(|(k, _)| !k.starts_with("odometry_slow")));
        let slow = SlowCopySettings { topic: "/odometry/global_slow".into(), rate_hz: 5.0 };
        let params = advertised_parameters(&r.parameters, &slow);
        let find = |n: &str| &params.iter().find(|p| p.name == n).unwrap_or_else(|| panic!("{n}")).value;
        assert_eq!(find("odometry_slow_topic").string_value, "/odometry/global_slow");
        assert_eq!(find("odometry_slow_topic").type_, paramsrv::PARAMETER_STRING);
        assert_eq!(find("odometry_slow_rate_hz").double_value, 5.0);
        assert_eq!(find("odometry_slow_rate_hz").type_, paramsrv::PARAMETER_DOUBLE);
        assert_eq!(find("frequency").double_value, 20.0);
        assert_eq!(find("odom1").string_value, "odometry/gps");
        assert!(!find("use_sim_time").bool_value);
        assert_eq!(params.iter().filter(|p| p.name == "use_sim_time").count(), 1);
        // Only the map instance has a copy by default: nothing throttles
        // /odometry/local.
        assert_eq!(default_slow_topic(crate::NODE_EKF_MAP), "odometry/global_slow");
        assert_eq!(default_slow_topic(crate::NODE_EKF_ODOM), "");
    }
}
