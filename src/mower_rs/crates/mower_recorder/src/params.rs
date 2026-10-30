//! What `mower_rs_common::params` lacks: string arrays, as rclpy's
//! `declare_parameter(name, [''])` reads them.

use r2r::{Node, ParameterValue};

/// A string-array parameter, `default` when it was not passed. An empty
/// array passed as such (`[]`, which rcl reads as no type) counts as unset.
pub fn strings(node: &Node, name: &str, default: &[&str]) -> Vec<String> {
    let value = node.params.lock().unwrap_or_else(|e| e.into_inner()).get(name).map(|p| p.value.clone());
    match value {
        Some(ParameterValue::StringArray(v)) => v,
        None | Some(ParameterValue::NotSet) => default.iter().map(|s| s.to_string()).collect(),
        Some(other) => {
            r2r::log_warn!(node.logger(), "parameter {name}: expected a string array, got {other:?}; using default");
            default.iter().map(|s| s.to_string()).collect()
        }
    }
}
