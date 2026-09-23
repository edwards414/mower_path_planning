//! The six per-node parameter services, by hand.
//!
//! r2r does not create them, so every mower_rs node that needs them builds
//! them itself (`mower_map` does the same for `/map_manage/*`). The C++ nodes
//! declare their parameters with rclcpp, which answers `get`, `list` and
//! `describe` from the declarations and lets `set` succeed even for the many
//! values the node only ever reads at start-up.
//!
//! This module answers `get` / `get_parameter_types` / `list` / `describe`
//! from the values the node is actually running with, and **refuses every
//! `set`** with a reason instead of accepting a change that would not take
//! effect. Nothing on this robot calls these services (`mower_adapter` only
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

// rcl_interfaces/msg/ParameterType
pub const PARAMETER_NOT_SET: u8 = 0;
pub const PARAMETER_BOOL: u8 = 1;
pub const PARAMETER_INTEGER: u8 = 2;
pub const PARAMETER_DOUBLE: u8 = 3;
pub const PARAMETER_STRING: u8 = 4;
pub const PARAMETER_DOUBLE_ARRAY: u8 = 8;

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

pub fn double(name: &str, v: f64) -> Param {
    Param {
        name: name.to_string(),
        value: ParameterValue { type_: PARAMETER_DOUBLE, double_value: v, ..Default::default() },
    }
}

pub fn string(name: &str, v: &str) -> Param {
    Param {
        name: name.to_string(),
        value: ParameterValue {
            type_: PARAMETER_STRING,
            string_value: v.to_string(),
            ..Default::default()
        },
    }
}

pub fn integer(name: &str, v: i64) -> Param {
    Param {
        name: name.to_string(),
        value: ParameterValue { type_: PARAMETER_INTEGER, integer_value: v, ..Default::default() },
    }
}

pub fn double_array(name: &str, v: Vec<f64>) -> Param {
    Param {
        name: name.to_string(),
        value: ParameterValue {
            type_: PARAMETER_DOUBLE_ARRAY,
            double_array_value: v,
            ..Default::default()
        },
    }
}

fn not_set() -> ParameterValue {
    ParameterValue { type_: PARAMETER_NOT_SET, ..Default::default() }
}

fn refusal(p: &Parameter) -> SetParametersResult {
    SetParametersResult {
        successful: false,
        reason: format!(
            "{} is read-only in mower_localize: the filter is built from \
             mower_localize_core::config at start-up, change the launch \
             parameters and restart",
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
        assert_eq!(boolean("two_d_mode", true).value.type_, PARAMETER_BOOL);
        assert_eq!(double("frequency", 20.0).value.type_, PARAMETER_DOUBLE);
        assert_eq!(string("world_frame", "odom").value.type_, PARAMETER_STRING);
        assert_eq!(integer("odom0_queue_size", 10).value.type_, PARAMETER_INTEGER);
        assert_eq!(double_array("process_noise_covariance", vec![1.0]).value.type_, PARAMETER_DOUBLE_ARRAY);
    }
}
