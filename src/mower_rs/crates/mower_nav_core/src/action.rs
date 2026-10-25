//! What the ported action servers (`controller_server`, `planner_server`,
//! `navigator`) share: goal ids and terminal states.

/// An accepted goal, numbered by its server in acceptance order from 1.
pub type GoalId = u64;

/// How a goal ended (`rclcpp_action` terminal states).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GoalStatus {
    Succeeded,
    Aborted,
    Canceled,
}
