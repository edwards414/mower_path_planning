//! The six per-node parameter services, by hand.
//!
//! r2r does not create them, so every mower_rs node that needs them builds
//! them itself (`mower_map` does the same for `/map_manage/*`). The C++ nodes
//! declare their parameters with rclcpp, which answers `get`, `list` and
//! `describe` from the declarations and lets `set` succeed even for the many
//! values the node only ever reads at start-up.
//!
//! This module answers `get` / `get_parameter_types` / `list` / `describe`
//! from the values the node is actually running with -- the declared set that
//! `mower_localize_core::config` resolved from the params file, defaults
//! included, as rclcpp would list it -- and **refuses every `set`** with a
//! reason instead of accepting a change that would not take effect. Nothing on this robot calls these services (`mower_adapter` only
//! calls `/map_manage/*` and `/boustrophedon_coverage/*`); they exist so
//! `ros2 param get <node> frequency` still answers with the truth.

use r2r::rcl_interfaces::msg::{
    ListParametersResult, Parameter, ParameterDescriptor, ParameterValue, SetParametersResult,
};
use r2r::rcl_interfaces::srv::{
    DescribeParameters, GetParameterTypes, GetParameters, ListParameters, SetParameters,
    SetParametersAtomically,
};
use r2r::QosProfile;

use futures::stream::StreamExt;
use mower_localize_core::config::{ParamValue as CoreValue, Params as CoreParams};

// rcl_interfaces/msg/ParameterType
pub const PARAMETER_NOT_SET: u8 = 0;
pub const PARAMETER_BOOL: u8 = 1;
pub const PARAMETER_INTEGER: u8 = 2;
pub const PARAMETER_DOUBLE: u8 = 3;
pub const PARAMETER_STRING: u8 = 4;
pub const PARAMETER_BOOL_ARRAY: u8 = 6;
pub const PARAMETER_INTEGER_ARRAY: u8 = 7;
pub const PARAMETER_DOUBLE_ARRAY: u8 = 8;
pub const PARAMETER_STRING_ARRAY: u8 = 9;

/// One exposed parameter and its current value.
pub struct Param {
    pub name: String,
    pub value: ParameterValue,
}

pub fn boolean(name: &str, v: bool) -> Param {
    Param {
        name: name.to_string(),
        value: ParameterValue { type_: PARAMETER_BOOL, bool_value: v, ..Default::default() },
    }
}

/// A resolved value, as the parameter services report it.
pub fn from_core(name: &str, v: &CoreValue) -> Param {
    let value = match v {
        CoreValue::Bool(b) => ParameterValue { type_: PARAMETER_BOOL, bool_value: *b, ..Default::default() },
        CoreValue::Integer(i) => {
            ParameterValue { type_: PARAMETER_INTEGER, integer_value: *i, ..Default::default() }
        }
        CoreValue::Double(d) => ParameterValue { type_: PARAMETER_DOUBLE, double_value: *d, ..Default::default() },
        CoreValue::String(s) => {
            ParameterValue { type_: PARAMETER_STRING, string_value: s.clone(), ..Default::default() }
        }
        CoreValue::BoolArray(a) => {
            ParameterValue { type_: PARAMETER_BOOL_ARRAY, bool_array_value: a.clone(), ..Default::default() }
        }
        CoreValue::IntegerArray(a) => ParameterValue {
            type_: PARAMETER_INTEGER_ARRAY,
            integer_array_value: a.clone(),
            ..Default::default()
        },
        CoreValue::DoubleArray(a) => ParameterValue {
            type_: PARAMETER_DOUBLE_ARRAY,
            double_array_value: a.clone(),
            ..Default::default()
        },
        CoreValue::StringArray(a) => ParameterValue {
            type_: PARAMETER_STRING_ARRAY,
            string_array_value: a.clone(),
            ..Default::default()
        },
    };
    Param { name: name.to_string(), value }
}

/// The node's parameter overrides (its section of every `--params-file`, and
/// `-p` rules), in the core's terms.
pub fn overrides(node: &r2r::Node) -> CoreParams {
    let params = node.params.lock().unwrap();
    params
        .iter()
        .filter_map(|(name, p)| {
            let v = match &p.value {
                r2r::ParameterValue::NotSet => return None,
                r2r::ParameterValue::Bool(b) => CoreValue::Bool(*b),
                r2r::ParameterValue::Integer(i) => CoreValue::Integer(*i),
                r2r::ParameterValue::Double(d) => CoreValue::Double(*d),
                r2r::ParameterValue::String(s) => CoreValue::String(s.clone()),
                r2r::ParameterValue::BoolArray(a) => CoreValue::BoolArray(a.clone()),
                r2r::ParameterValue::ByteArray(a) => {
                    CoreValue::IntegerArray(a.iter().map(|b| *b as i64).collect())
                }
                r2r::ParameterValue::IntegerArray(a) => CoreValue::IntegerArray(a.clone()),
                r2r::ParameterValue::DoubleArray(a) => CoreValue::DoubleArray(a.clone()),
                r2r::ParameterValue::StringArray(a) => CoreValue::StringArray(a.clone()),
            };
            Some((name.clone(), v))
        })
        .collect()
}

fn not_set() -> ParameterValue {
    ParameterValue { type_: PARAMETER_NOT_SET, ..Default::default() }
}

fn refusal(p: &Parameter) -> SetParametersResult {
    SetParametersResult {
        successful: false,
        reason: format!(
            "{} is read-only in mower_localize: the settings are resolved from \
             dual_ekf_navsat_params.yaml at start-up, change the file and restart",
            p.name
        ),
    }
}

/// Advertise `<node>/{get,set,list,describe,...}_parameters`.
///
/// `params` is the node's whole exposed set, so `list_parameters` can answer
/// without a lock; the values never change (a `set` is always refused), which
/// is why they are moved in here rather than shared.
pub fn advertise(node: &mut r2r::Node, node_name: &str, params: Vec<Param>) -> Result<(), r2r::Error> {
    let params = std::sync::Arc::new(params);
    let value_of = {
        let params = params.clone();
        move |name: &str| -> ParameterValue {
            params
                .iter()
                .find(|p| p.name == name)
                .map(|p| p.value.clone())
                .unwrap_or_else(not_set)
        }
    };

    {
        let mut stream = node.create_service::<GetParameters::Service>(
            &format!("/{node_name}/get_parameters"),
            QosProfile::services_default(),
        )?;
        let value_of = value_of.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let values = req.message.names.iter().map(|n| value_of(n)).collect();
                let _ = req.respond(GetParameters::Response { values });
            }
        });
    }
    {
        let mut stream = node.create_service::<GetParameterTypes::Service>(
            &format!("/{node_name}/get_parameter_types"),
            QosProfile::services_default(),
        )?;
        let value_of = value_of.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let types = req.message.names.iter().map(|n| value_of(n).type_).collect();
                let _ = req.respond(GetParameterTypes::Response { types });
            }
        });
    }
    {
        let mut stream = node.create_service::<ListParameters::Service>(
            &format!("/{node_name}/list_parameters"),
            QosProfile::services_default(),
        )?;
        let all = params.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let prefixes = &req.message.prefixes;
                let names = all
                    .iter()
                    .map(|p| p.name.clone())
                    .filter(|n| {
                        prefixes.is_empty() || prefixes.iter().any(|p| n.starts_with(p.as_str()))
                    })
                    .collect();
                let _ = req.respond(ListParameters::Response {
                    result: ListParametersResult { names, prefixes: Vec::new() },
                });
            }
        });
    }
    {
        let mut stream = node.create_service::<DescribeParameters::Service>(
            &format!("/{node_name}/describe_parameters"),
            QosProfile::services_default(),
        )?;
        let value_of = value_of.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let descriptors = req
                    .message
                    .names
                    .iter()
                    .map(|n| ParameterDescriptor {
                        name: n.clone(),
                        type_: value_of(n).type_,
                        read_only: true,
                        ..Default::default()
                    })
                    .collect();
                let _ = req.respond(DescribeParameters::Response { descriptors });
            }
        });
    }
    {
        let mut stream = node.create_service::<SetParameters::Service>(
            &format!("/{node_name}/set_parameters"),
            QosProfile::services_default(),
        )?;
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let results = req.message.parameters.iter().map(refusal).collect();
                let _ = req.respond(SetParameters::Response { results });
            }
        });
    }
    {
        let mut stream = node.create_service::<SetParametersAtomically::Service>(
            &format!("/{node_name}/set_parameters_atomically"),
            QosProfile::services_default(),
        )?;
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let result = req
                    .message
                    .parameters
                    .first()
                    .map(refusal)
                    .unwrap_or(SetParametersResult { successful: true, reason: String::new() });
                let _ = req.respond(SetParametersAtomically::Response { result });
            }
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_set_is_always_refused_and_says_why() {
        let p = Parameter { name: "frequency".into(), value: not_set() };
        let r = refusal(&p);
        assert!(!r.successful);
        assert!(r.reason.contains("frequency"), "{}", r.reason);
        assert!(r.reason.contains("read-only"), "{}", r.reason);
    }

    #[test]
    fn the_constructors_set_the_rcl_type_tag() {
        assert_eq!(boolean("use_sim_time", false).value.type_, PARAMETER_BOOL);
        assert_eq!(
            from_core("process_noise_covariance", &CoreValue::DoubleArray(vec![1.0])).value.type_,
            PARAMETER_DOUBLE_ARRAY
        );
        assert_eq!(from_core("frequency", &CoreValue::Double(20.0)).value.type_, PARAMETER_DOUBLE);
        assert_eq!(from_core("world_frame", &CoreValue::String("odom".into())).value.type_, PARAMETER_STRING);
        assert_eq!(from_core("odom0_queue_size", &CoreValue::Integer(10)).value.type_, PARAMETER_INTEGER);
    }

    #[test]
    fn resolved_values_keep_their_rcl_type() {
        let p = from_core("odom0_config", &CoreValue::BoolArray(vec![true, false]));
        assert_eq!(p.value.type_, PARAMETER_BOOL_ARRAY);
        assert_eq!(p.value.bool_array_value, [true, false]);
        assert_eq!(from_core("odom0_queue_size", &CoreValue::Integer(10)).value.integer_value, 10);
        assert_eq!(from_core("odom0", &CoreValue::String("odom".into())).value.string_value, "odom");
        assert_eq!(from_core("frequency", &CoreValue::Double(20.0)).value.type_, PARAMETER_DOUBLE);
    }
}
