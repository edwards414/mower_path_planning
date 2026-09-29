//! The EKF and navsat_transform settings, resolved from a node's ROS
//! parameters exactly the way robot_localization 3.8.3 declares them.
//!
//! There is one source for the numbers: the node's parameter section in
//! `mower_nav2/config/dual_ekf_navsat_params.yaml`, which the launch files pass
//! to the C++ nodes and to `mower_localize` alike. Nothing here repeats a
//! value from that file; what is written below are upstream's declared
//! defaults (`loadParams` in `ros_filter.cpp`, the `NavSatTransform`
//! constructor), used when a key is absent, just as the C++ node would.
//!
//! Every key is sorted into one of three kinds, so a value can never be
//! silently dropped:
//!
//! * **applied** -- read and used, with rclcpp's strict typing (a `20` where a
//!   double is declared is an error, as `declare_parameter` throws);
//! * **refused** -- a feature neither this port nor its ROS shell implements
//!   (`smooth_lagged_data`, `use_control`, `poseN`, ...). Its upstream default
//!   is accepted, anything else is an error, because running a different
//!   filter than the C++ node would is worse than not starting;
//! * **inert** -- keys that change nothing this port produces
//!   (`print_diagnostics`, `debug`, `reset_on_time_jump`, the control limits
//!   while `use_control` is false, ...). A non-default value is a warning.
//!
//! A key robot_localization does not declare is ignored by the C++ node, and
//! is a warning here. [`Resolved::parameters`] is what the node reports on its
//! parameter services: every declared key with the value in effect.

use std::collections::{BTreeMap, BTreeSet};

use crate::filter_common::{Mat15, STATE_SIZE, ZERO_MAT15};
use crate::navsat::NavSatConfig;
use crate::prepare::{RosFilterCore, SensorConfig};

/// One ROS parameter value, as rcl parses it from a params file.
#[derive(Clone, Debug, PartialEq)]
pub enum ParamValue {
    Bool(bool),
    Integer(i64),
    Double(f64),
    String(String),
    BoolArray(Vec<bool>),
    IntegerArray(Vec<i64>),
    DoubleArray(Vec<f64>),
    StringArray(Vec<String>),
}

impl ParamValue {
    fn type_name(&self) -> &'static str {
        match self {
            ParamValue::Bool(_) => "bool",
            ParamValue::Integer(_) => "integer",
            ParamValue::Double(_) => "double",
            ParamValue::String(_) => "string",
            ParamValue::BoolArray(_) => "bool array",
            ParamValue::IntegerArray(_) => "integer array",
            ParamValue::DoubleArray(_) => "double array",
            ParamValue::StringArray(_) => "string array",
        }
    }
}

/// A node's parameter overrides, by name.
pub type Params = BTreeMap<String, ParamValue>;

/// Settings plus what the node should say about them.
#[derive(Clone, Debug)]
pub struct Resolved<T> {
    pub settings: T,
    /// Non-fatal findings, one line each, for the node to log at start-up.
    pub warnings: Vec<String>,
    /// Every declared parameter with its value in effect, in declaration
    /// order: what `get_parameters` / `list_parameters` answer.
    pub parameters: Vec<(String, ParamValue)>,
}

/// Reads parameters the way `declare_parameter` does, and remembers what it
/// read.
struct Reader<'a> {
    params: &'a Params,
    node: &'a str,
    used: BTreeSet<String>,
    declared: Vec<(String, ParamValue)>,
    warnings: Vec<String>,
}

macro_rules! typed {
    ($name:ident, $variant:ident, $t:ty) => {
        fn $name(&mut self, key: &str, default: $t) -> Result<$t, String> {
            let v = match self.raw(key) {
                None => default,
                Some(ParamValue::$variant(v)) => v.clone(),
                Some(other) => return Err(self.wrong_type(key, stringify!($variant), other)),
            };
            self.declared.push((key.to_string(), ParamValue::$variant(v.clone())));
            Ok(v)
        }
    };
}

impl<'a> Reader<'a> {
    fn new(node: &'a str, params: &'a Params) -> Self {
        Self { params, node, used: BTreeSet::new(), declared: Vec::new(), warnings: Vec::new() }
    }

    fn raw(&mut self, key: &str) -> Option<&'a ParamValue> {
        self.used.insert(key.to_string());
        self.params.get(key)
    }

    fn wrong_type(&self, key: &str, expected: &str, got: &ParamValue) -> String {
        format!(
            "{}: parameter {key} must be {}, the params file gives {} ({got:?}); \
             robot_localization's declare_parameter would throw too",
            self.node,
            expected.to_lowercase(),
            got.type_name()
        )
    }

    typed!(boolean, Bool, bool);
    typed!(integer, Integer, i64);
    typed!(double, Double, f64);
    typed!(string, String, String);
    typed!(bool_array, BoolArray, Vec<bool>);

    /// `declare_parameter(name, rclcpp::PARAMETER_<type>)`: no default, and
    /// absent means "not set".
    fn opt_string(&mut self, key: &str) -> Result<Option<String>, String> {
        match self.raw(key) {
            None => Ok(None),
            Some(ParamValue::String(v)) => {
                self.declared.push((key.to_string(), ParamValue::String(v.clone())));
                Ok(Some(v.clone()))
            }
            Some(other) => Err(self.wrong_type(key, "String", other)),
        }
    }

    fn opt_double_array(&mut self, key: &str) -> Result<Option<Vec<f64>>, String> {
        match self.raw(key) {
            None => Ok(None),
            Some(ParamValue::DoubleArray(v)) => {
                self.declared.push((key.to_string(), ParamValue::DoubleArray(v.clone())));
                Ok(Some(v.clone()))
            }
            Some(other) => Err(self.wrong_type(key, "DoubleArray", other)),
        }
    }

    fn refuse_bool(&mut self, key: &str, default: bool, why: &str) -> Result<(), String> {
        if self.boolean(key, default)? != default {
            return Err(self.refusal(key, why));
        }
        Ok(())
    }

    fn refuse_double(&mut self, key: &str, default: f64, why: &str) -> Result<(), String> {
        if self.double(key, default)? != default {
            return Err(self.refusal(key, why));
        }
        Ok(())
    }

    fn refusal(&self, key: &str, why: &str) -> String {
        format!(
            "{}: {key} is set to a non-default value, but mower_localize does not implement it \
             ({why}); run the C++ nodes (rust_localize:=false) for this configuration",
            self.node
        )
    }

    fn inert_bool(&mut self, key: &str, default: bool, effect: &str) -> Result<(), String> {
        if self.boolean(key, default)? != default {
            self.warnings.push(format!("{}: {key}: {} has no effect here ({effect})", self.node, !default));
        }
        Ok(())
    }

    fn inert_double(&mut self, key: &str, default: f64, effect: &str) -> Result<(), String> {
        let v = self.double(key, default)?;
        if v != default {
            self.warnings.push(format!("{}: {key}: {v} has no effect here ({effect})", self.node));
        }
        Ok(())
    }

    fn untested(&mut self, key: &str) {
        self.warnings.push(format!(
            "{}: {key} is applied, but that path is not covered by the oracle vectors",
            self.node
        ));
    }

    /// Everything given but never read: robot_localization does not declare
    /// it, so the C++ node ignores it too.
    fn finish<T>(mut self, settings: T) -> Resolved<T> {
        for key in self.params.keys() {
            if !self.used.contains(key) {
                self.warnings.push(format!(
                    "{}: parameter {key} is not a robot_localization parameter; ignored",
                    self.node
                ));
            }
        }
        Resolved { settings, warnings: self.warnings, parameters: self.declared }
    }
}

/// `process_noise_covariance` / `initial_estimate_covariance`: 15 values are a
/// diagonal, 225 a full matrix read column-major, as upstream's
/// `Eigen::MatrixXd::Map` reads it (element `i` is row `i % 15`, column
/// `i / 15`; the transpose of the row-major reading for an asymmetric matrix).
/// Any other length is refused. Upstream means to throw there too, but its
/// check tests the destination's size (always 225) instead of the list's, so
/// it maps 225 values out of a shorter list.
fn covariance(node: &str, key: &str, flat: &[f64]) -> Result<Mat15, String> {
    let mut m = ZERO_MAT15;
    if flat.len() == STATE_SIZE {
        for (i, v) in flat.iter().enumerate() {
            m[i][i] = *v;
        }
    } else if flat.len() == STATE_SIZE * STATE_SIZE {
        for (i, v) in flat.iter().enumerate() {
            m[i % STATE_SIZE][i / STATE_SIZE] = *v;
        }
    } else {
        return Err(format!(
            "{node}: invalid {key}: expected {STATE_SIZE} or {} values, got {}",
            STATE_SIZE * STATE_SIZE,
            flat.len()
        ));
    }
    Ok(m)
}

/// One `odomN` / `imuN` input.
#[derive(Clone, Debug)]
pub struct SensorInput {
    /// The parameter name, `odom0`: also the key the filter tracks it by.
    pub name: String,
    /// The topic as the yaml gives it (`odom`, `imu`), before remapping.
    pub topic: String,
    pub queue_size: usize,
    pub config: SensorConfig,
}

/// `ekf_filter_node_odom` / `ekf_filter_node_map`.
#[derive(Clone, Debug)]
pub struct EkfSettings {
    pub frequency: f64,
    pub sensor_timeout: f64,
    pub two_d_mode: bool,
    pub publish_tf: bool,
    pub map_frame: String,
    pub odom_frame: String,
    pub base_link_frame: String,
    pub world_frame: String,
    pub permit_corrected_publication: bool,
    pub predict_to_current_time: bool,
    pub gravitational_acceleration: f64,
    pub dynamic_process_noise_covariance: bool,
    /// `None`: `FilterBase`'s built-in default.
    pub process_noise_covariance: Option<Mat15>,
    /// `None`: `FilterBase`'s built-in default (1e-9 on the diagonal).
    pub initial_estimate_covariance: Option<Mat15>,
    pub odoms: Vec<SensorInput>,
    pub imus: Vec<SensorInput>,
}

impl EkfSettings {
    /// `RosFilter::loadParams`.
    pub fn from_params(node: &str, params: &Params) -> Result<Resolved<Self>, String> {
        let mut r = Reader::new(node, params);

        r.inert_bool("print_diagnostics", false, "no /diagnostics publisher")?;
        let gravitational_acceleration = r.double("gravitational_acceleration", 9.80665)?;
        let debug = r.boolean("debug", false)?;
        if debug {
            r.warnings.push(format!("{node}: debug: true has no effect here (no debug file)"));
            // Declared only with debug on, as upstream.
            r.string("debug_out_file", "robot_localization_debug.txt".to_string())?;
        }
        let map_frame = r.string("map_frame", "map".to_string())?;
        let odom_frame = r.string("odom_frame", "odom".to_string())?;
        let base_link_frame = r.string("base_link_frame", "base_link".to_string())?;
        let base_link_frame_output = r.string("base_link_frame_output", base_link_frame.clone())?;
        if base_link_frame_output != base_link_frame {
            return Err(r.refusal("base_link_frame_output", "the output frame is always base_link_frame"));
        }
        let world_frame = r.string("world_frame", odom_frame.clone())?;
        if map_frame == odom_frame || odom_frame == base_link_frame || map_frame == base_link_frame {
            r.warnings.push(format!(
                "{node}: invalid frame configuration: map_frame, odom_frame and base_link_frame must be unique"
            ));
        }
        if world_frame != map_frame && world_frame != odom_frame {
            return Err(format!(
                "{node}: world_frame {world_frame} is neither map_frame {map_frame} nor odom_frame {odom_frame}"
            ));
        }
        if let Some(prefix) = r.opt_string("tf_prefix")? {
            if !prefix.is_empty() {
                return Err(r.refusal("tf_prefix", "frame prefixes"));
            }
        }
        let publish_tf = r.boolean("publish_tf", true)?;
        r.refuse_bool("publish_acceleration", false, "/accel/filtered")?;
        let permit_corrected_publication = r.boolean("permit_corrected_publication", false)?;
        r.refuse_double("transform_time_offset", 0.0, "a transform stamp offset")?;
        if r.double("transform_timeout", 0.0)? != 0.0 {
            r.warnings.push(format!(
                "{node}: transform_timeout has no effect here: the only lookups are rigid sensor offsets, answered at once"
            ));
        }
        let frequency = r.double("frequency", 30.0)?;
        if !(frequency > 0.0) {
            return Err(format!("{node}: frequency must be positive, got {frequency}"));
        }
        let predict_to_current_time = r.boolean("predict_to_current_time", false)?;
        let sensor_timeout = r.double("sensor_timeout", 1.0 / frequency)?;
        let two_d_mode = r.boolean("two_d_mode", false)?;
        r.refuse_bool("smooth_lagged_data", false, "the lagged-data history")?;
        // These four are declared unconditionally upstream, so they are
        // listed with their defaults even when the yaml leaves them out.
        r.inert_double("history_length", 0.0, "only used with smooth_lagged_data")?;
        r.inert_bool("reset_on_time_jump", false, "its body is commented out upstream")?;
        r.refuse_bool("use_control", false, "the control input subscriptions")?;
        r.inert_bool("stamped_control", false, "only used with use_control")?;
        r.inert_double("control_timeout", 0.0, "only used with use_control")?;
        let dynamic_process_noise_covariance = r.boolean("dynamic_process_noise_covariance", false)?;
        if dynamic_process_noise_covariance {
            r.untested("dynamic_process_noise_covariance");
        }
        if r.opt_double_array("initial_state")?.is_some() {
            return Err(r.refusal("initial_state", "a configured initial state"));
        }
        r.refuse_bool("disabled_at_startup", false, "the enable service")?;

        let mut odoms = Vec::new();
        for i in 0.. {
            let name = format!("odom{i}");
            let Some(topic) = r.opt_string(&name)? else { break };
            odoms.push(sensor(&mut r, &name, topic, false)?);
        }
        for kind in ["pose", "twist"] {
            if r.opt_string(&format!("{kind}0"))?.is_some() {
                return Err(r.refusal(&format!("{kind}0"), "only odomN and imuN inputs are wired"));
            }
        }
        let mut imus = Vec::new();
        for i in 0.. {
            let name = format!("imu{i}");
            let Some(topic) = r.opt_string(&name)? else { break };
            imus.push(sensor(&mut r, &name, topic, true)?);
        }
        if odoms.is_empty() && imus.is_empty() {
            return Err(format!(
                "{node}: no odomN or imuN input configured -- was dual_ekf_navsat_params.yaml passed \
                 (--params-file) with a section for this node?"
            ));
        }

        let process_noise_covariance = match r.opt_double_array("process_noise_covariance")? {
            Some(flat) => Some(covariance(node, "process_noise_covariance", &flat)?),
            None => None,
        };
        let initial_estimate_covariance = match r.opt_double_array("initial_estimate_covariance")? {
            Some(flat) => {
                r.untested("initial_estimate_covariance");
                Some(covariance(node, "initial_estimate_covariance", &flat)?)
            }
            None => None,
        };
        // rclcpp declares it on every node; the port runs on the wall clock.
        r.refuse_bool("use_sim_time", false, "simulated time")?;

        let settings = EkfSettings {
            frequency,
            sensor_timeout,
            two_d_mode,
            publish_tf,
            map_frame,
            odom_frame,
            base_link_frame,
            world_frame,
            permit_corrected_publication,
            predict_to_current_time,
            gravitational_acceleration,
            dynamic_process_noise_covariance,
            process_noise_covariance,
            initial_estimate_covariance,
            odoms,
            imus,
        };
        Ok(r.finish(settings))
    }

    /// The filter, configured as `loadParams` configures it.
    pub fn build_filter(&self) -> RosFilterCore {
        let mut core = RosFilterCore::new(&self.world_frame, &self.base_link_frame);
        core.two_d_mode = self.two_d_mode;
        core.map_frame_id = self.map_frame.clone();
        core.odom_frame_id = self.odom_frame.clone();
        core.gravitational_acceleration = self.gravitational_acceleration;
        core.predict_to_current_time = self.predict_to_current_time;
        core.filter.use_control = false;
        core.filter.use_dynamic_process_noise_covariance = self.dynamic_process_noise_covariance;
        if let Some(q) = self.process_noise_covariance {
            core.filter.set_process_noise_covariance(q);
        }
        if let Some(p) = self.initial_estimate_covariance {
            core.filter.estimate_error_covariance = p;
        }
        // rclcpp::Duration::from_seconds: seconds * 1e9, truncated.
        core.filter.sensor_timeout_ns = (self.sensor_timeout * 1e9) as i64;
        core
    }
}

/// The per-sensor block of `loadParams` for `odomN` (`imu` false) and `imuN`.
fn sensor(r: &mut Reader, name: &str, topic: String, imu: bool) -> Result<SensorInput, String> {
    let key = |suffix: &str| format!("{name}_{suffix}");
    let differential = r.boolean(&key("differential"), false)?;
    let mut relative = r.boolean(&key("relative"), false)?;
    if relative && differential {
        r.warnings.push(format!(
            "{}: both {name}_differential and {name}_relative are true; using differential mode, as upstream",
            r.node
        ));
        relative = false;
    }
    if differential {
        r.untested(&key("differential"));
    }
    if relative {
        r.untested(&key("relative"));
    }
    let mut pose_use_child_frame = false;
    if !imu {
        pose_use_child_frame = r.boolean(&key("pose_use_child_frame"), false)?;
        if pose_use_child_frame {
            r.untested(&key("pose_use_child_frame"));
        }
    }
    // The port keeps one Mahalanobis threshold per sensor; upstream has one
    // per portion. Only the default (none) is accepted.
    let thresholds: &[&str] = if imu {
        &["pose_rejection_threshold", "twist_rejection_threshold", "linear_acceleration_rejection_threshold"]
    } else {
        &["pose_rejection_threshold", "twist_rejection_threshold"]
    };
    for t in thresholds {
        r.refuse_double(&key(t), f64::MAX, "per-portion rejection thresholds")?;
    }
    let mut remove_gravitational_acceleration = false;
    if imu {
        remove_gravitational_acceleration = r.boolean(&key("remove_gravitational_acceleration"), false)?;
    }
    let queue_size = r.integer(&key("queue_size"), 10)?;
    if queue_size < 1 {
        return Err(format!("{}: {name}_queue_size must be at least 1, got {queue_size}", r.node));
    }
    let update = r.bool_array(&key("config"), vec![false; STATE_SIZE])?;
    let update_vector: [bool; STATE_SIZE] = update.as_slice().try_into().map_err(|_| {
        format!("{}: {name}_config must have {STATE_SIZE} entries, got {}", r.node, update.len())
    })?;

    let mut config = SensorConfig::new(name, update_vector);
    config.differential = differential;
    config.relative = relative;
    config.pose_use_child_frame = pose_use_child_frame;
    config.remove_gravitational_acceleration = remove_gravitational_acceleration;
    Ok(SensorInput { name: name.to_string(), topic, queue_size: queue_size as usize, config })
}

/// `navsat_transform`.
#[derive(Clone, Debug)]
pub struct NavSatSettings {
    pub config: NavSatConfig,
    pub frequency: f64,
    /// Seconds the node waits, unspun, before its timer starts.
    pub delay: f64,
}

impl NavSatSettings {
    /// The `NavSatTransform` constructor's parameter block.
    pub fn from_params(node: &str, params: &Params) -> Result<Resolved<Self>, String> {
        if params.keys().all(|k| k == "use_sim_time") {
            return Err(format!(
                "{node}: no parameters -- was dual_ekf_navsat_params.yaml passed (--params-file) \
                 with a section for this node?"
            ));
        }
        let mut r = Reader::new(node, params);
        let magnetic_declination_radians = r.double("magnetic_declination_radians", 0.0)?;
        let yaw_offset = r.double("yaw_offset", 0.0)?;
        let zero_altitude = r.boolean("zero_altitude", false)?;
        let publish_filtered_gps = r.boolean("publish_filtered_gps", true)?;
        r.refuse_bool("use_odometry_yaw", false, "the heading from odometry instead of the IMU")?;
        r.refuse_bool("wait_for_datum", false, "a manual datum from parameters")?;
        r.refuse_bool("use_local_cartesian", false, "the local Cartesian frame; the port works in UTM")?;
        let frequency = r.double("frequency", 10.0)?;
        if !(frequency > 0.0) {
            return Err(format!("{node}: frequency must be positive, got {frequency}"));
        }
        let delay = r.double("delay", 0.0)?;
        if r.double("transform_timeout", 0.0)? != 0.0 {
            r.warnings.push(format!(
                "{node}: transform_timeout has no effect here: the only lookups are rigid sensor offsets, answered at once"
            ));
        }
        // The deprecated spellings win when set, as upstream (including the
        // trailing underscore of the second one, which is upstream's).
        let mut broadcast_cartesian_transform = r.boolean("broadcast_utm_transform", false)?;
        if broadcast_cartesian_transform {
            r.warnings.push(format!("{node}: broadcast_utm_transform is deprecated upstream; use broadcast_cartesian_transform"));
        } else {
            broadcast_cartesian_transform = r.boolean("broadcast_cartesian_transform", false)?;
        }
        let mut as_parent = r.boolean("broadcast_utm_transform_as_parent_frame_", false)?;
        if as_parent {
            r.warnings.push(format!(
                "{node}: broadcast_utm_transform_as_parent_frame_ is deprecated upstream; use broadcast_cartesian_transform_as_parent_frame"
            ));
        } else {
            as_parent = r.boolean("broadcast_cartesian_transform_as_parent_frame", false)?;
        }
        r.refuse_bool("use_sim_time", false, "simulated time")?;
        for key in params.keys() {
            if key.starts_with("qos_overrides.") {
                r.used.insert(key.clone());
                return Err(r.refusal(key, "QoS overrides"));
            }
        }

        let settings = NavSatSettings {
            config: NavSatConfig {
                magnetic_declination_radians,
                yaw_offset,
                zero_altitude,
                publish_filtered_gps,
                use_odometry_yaw: false,
                wait_for_datum: false,
                broadcast_cartesian_transform,
                broadcast_cartesian_transform_as_parent_frame: as_parent,
            },
            frequency,
            delay,
        };
        Ok(r.finish(settings))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params(entries: &[(&str, ParamValue)]) -> Params {
        entries.iter().map(|(k, v)| (k.to_string(), v.clone())).collect()
    }

    fn odom_only() -> Vec<(&'static str, ParamValue)> {
        vec![("odom0", ParamValue::String("odom".into()))]
    }

    #[test]
    fn absent_keys_take_robot_localization_s_declared_defaults() {
        let r = EkfSettings::from_params("ekf", &params(&odom_only())).unwrap();
        let s = &r.settings;
        assert_eq!(s.frequency, 30.0);
        assert_eq!(s.sensor_timeout, 1.0 / 30.0);
        assert!(!s.two_d_mode && s.publish_tf && !s.permit_corrected_publication);
        assert_eq!((s.map_frame.as_str(), s.odom_frame.as_str()), ("map", "odom"));
        assert_eq!(s.base_link_frame, "base_link");
        assert_eq!(s.world_frame, "odom");
        assert!(s.process_noise_covariance.is_none());
        assert_eq!(s.odoms.len(), 1);
        assert_eq!(s.odoms[0].queue_size, 10);
        assert_eq!(s.odoms[0].config.update_vector, [false; STATE_SIZE]);
        assert!(r.warnings.is_empty(), "{:?}", r.warnings);
        // loadParams declares these whatever the yaml says, so the parameter
        // services list them at their defaults.
        let declared = |k: &str| r.parameters.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone());
        assert_eq!(declared("history_length"), Some(ParamValue::Double(0.0)));
        assert_eq!(declared("reset_on_time_jump"), Some(ParamValue::Bool(false)));
        assert_eq!(declared("stamped_control"), Some(ParamValue::Bool(false)));
        assert_eq!(declared("control_timeout"), Some(ParamValue::Double(0.0)));
        assert_eq!(declared("debug_out_file"), None);
    }

    #[test]
    fn inert_keys_are_typed_and_a_non_default_value_is_a_warning() {
        let mut p = odom_only();
        p.push(("history_length", ParamValue::Double(2.0)));
        p.push(("stamped_control", ParamValue::Bool(true)));
        p.push(("debug", ParamValue::Bool(true)));
        let r = EkfSettings::from_params("ekf", &params(&p)).unwrap();
        assert_eq!(r.warnings.len(), 3, "{:?}", r.warnings);
        assert!(r.parameters.iter().any(|(k, v)| k == "history_length" && *v == ParamValue::Double(2.0)));
        assert!(r.parameters.iter().any(|(k, _)| k == "debug_out_file"));
        let mut p = odom_only();
        p.push(("control_timeout", ParamValue::Integer(1)));
        let err = EkfSettings::from_params("ekf", &params(&p)).unwrap_err();
        assert!(err.contains("control_timeout"), "{err}");
    }

    #[test]
    fn a_wrongly_typed_value_is_an_error_like_declare_parameter() {
        let mut p = odom_only();
        p.push(("frequency", ParamValue::Integer(20)));
        let err = EkfSettings::from_params("ekf", &params(&p)).unwrap_err();
        assert!(err.contains("frequency") && err.contains("integer"), "{err}");
    }

    #[test]
    fn an_unimplemented_feature_refuses_to_start_instead_of_being_dropped() {
        for (key, value) in [
            ("smooth_lagged_data", ParamValue::Bool(true)),
            ("use_control", ParamValue::Bool(true)),
            ("publish_acceleration", ParamValue::Bool(true)),
            ("transform_time_offset", ParamValue::Double(0.1)),
            ("odom0_twist_rejection_threshold", ParamValue::Double(5.0)),
            ("pose0", ParamValue::String("pose".into())),
            ("use_sim_time", ParamValue::Bool(true)),
        ] {
            let mut p = odom_only();
            p.push((key, value));
            let err = EkfSettings::from_params("ekf", &params(&p)).unwrap_err();
            assert!(err.contains(key), "{key}: {err}");
        }
        // Their defaults are fine.
        let mut p = odom_only();
        p.push(("smooth_lagged_data", ParamValue::Bool(false)));
        p.push(("use_sim_time", ParamValue::Bool(false)));
        assert!(EkfSettings::from_params("ekf", &params(&p)).is_ok());
    }

    #[test]
    fn an_unknown_key_is_reported_not_silently_dropped() {
        let mut p = odom_only();
        p.push(("frequncy", ParamValue::Double(20.0)));
        let r = EkfSettings::from_params("ekf", &params(&p)).unwrap();
        assert_eq!(r.warnings.len(), 1, "{:?}", r.warnings);
        assert!(r.warnings[0].contains("frequncy"));
        assert!(r.parameters.iter().all(|(k, _)| k != "frequncy"));
    }

    #[test]
    fn a_node_without_inputs_says_its_params_file_is_missing() {
        let err = EkfSettings::from_params("ekf", &Params::new()).unwrap_err();
        assert!(err.contains("dual_ekf_navsat_params.yaml"), "{err}");
        let err = NavSatSettings::from_params("navsat", &Params::new()).unwrap_err();
        assert!(err.contains("dual_ekf_navsat_params.yaml"), "{err}");
    }

    #[test]
    fn covariances_take_a_diagonal_or_a_full_matrix() {
        let diag: Vec<f64> = (0..STATE_SIZE).map(|i| i as f64 + 1.0).collect();
        let m = covariance("n", "k", &diag).unwrap();
        assert_eq!(m[3][3], 4.0);
        assert_eq!(m[3][4], 0.0);
        // An asymmetric matrix shows the storage order: Eigen's Map is
        // column-major, so the 16th value is row 0, column 1 (and the 3rd is
        // row 2, column 0), not row 1, column 0.
        let mut full = vec![0.0; STATE_SIZE * STATE_SIZE];
        full[STATE_SIZE] = 7.0;
        full[2] = 5.0;
        let m = covariance("n", "k", &full).unwrap();
        assert_eq!((m[0][1], m[1][0]), (7.0, 0.0));
        assert_eq!((m[2][0], m[0][2]), (5.0, 0.0));
        assert!(covariance("n", "k", &[1.0; 3]).is_err());
        assert!(covariance("n", "k", &[1.0; STATE_SIZE * STATE_SIZE - 1]).is_err());
    }

    #[test]
    fn navsat_defaults_and_deprecated_spellings() {
        let r = NavSatSettings::from_params("navsat", &params(&[("frequency", ParamValue::Double(30.0))])).unwrap();
        assert_eq!(r.settings.delay, 0.0);
        assert!(!r.settings.config.broadcast_cartesian_transform);
        assert!(r.settings.config.publish_filtered_gps);
        let r = NavSatSettings::from_params(
            "navsat",
            &params(&[("broadcast_utm_transform", ParamValue::Bool(true))]),
        )
        .unwrap();
        assert!(r.settings.config.broadcast_cartesian_transform);
        assert_eq!(r.warnings.len(), 1);
        let err = NavSatSettings::from_params("navsat", &params(&[("wait_for_datum", ParamValue::Bool(true))]))
            .unwrap_err();
        assert!(err.contains("wait_for_datum"), "{err}");
    }
}
