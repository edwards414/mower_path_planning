//! The bus/DDS bridge configuration of step E2 (docs/ROS_FREE_PLAN.md, docs/
//! BUS_TOPICS.md section 6, step 0). README.md has the rules and why.
//!
//! - [`manifest`]: every endpoint of every mower_rsd module instance, with its
//!   production name, r2r type and QoS, and the outside nodes modules depend
//!   on.
//! - [`on_bus`]: one flag per module, whether its IO is on the bus. Moving a
//!   module is flipping it.
//! - [`plan`]: the policy, a pure function from the instances of a process
//!   and their flags to the bridge's declarations.
//! - [`coverage`]: the check that every endpoint a module made on the bus is
//!   in the manifest, so that none goes silently unbridged.
//! - [`run`] and [`module_main`]: the bridge as mower_rsd's module, and next
//!   to a moved module in its own binary.

pub mod coverage;
pub mod manifest;
mod on_bus;
mod policy;
mod run;
pub mod types;

pub use manifest::{instance, module, production_remaps, MODULES, OUTSIDE, PRODUCTION_REMAPS};
pub use on_bus::{instance_on_bus, on_bus, on_bus_with, INSTANCES, ON_BUS, RUNTIME};
pub use policy::{plan, Declaration, Plan, PlanError, Running};
pub use run::{
    module_main, module_main_by_flag, rsd_args, rsd_plan, rsd_plan_with, run, run_standalone, NODE,
    STANDALONE_NAMESPACE,
};

#[cfg(test)]
mod tests;
