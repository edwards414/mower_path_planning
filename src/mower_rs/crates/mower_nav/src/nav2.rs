//! Correlated Nav2 goal tracking: the cancelable goal handle and the
//! observed terminal result of the active dispatch generation
//! (the `_nav2_current_goal_handle` / `_nav2_current_result_future` pair).

use std::future::Future;
use std::pin::Pin;
use std::sync::{Arc, Mutex};

use crate::core_backend::{GoalOutcome, GoalStatus as CoreStatus};
use crate::state::TaskResult;

pub type BoxFuture<T> = Pin<Box<dyn Future<Output = T> + Send>>;

/// A Nav2 goal we can cancel, independent of its action type.
pub trait CancelableGoal: Send + Sync {
    fn cancel(&self) -> BoxFuture<Result<(), String>>;
}

/// Observed terminal outcome: `None` until the result arrives, `Err` when
/// the result channel ended without a proven terminal status.
#[derive(Debug, Clone)]
pub struct Terminal {
    pub result: TaskResult,
    pub error_code: u16,
    pub error_msg: String,
}

#[derive(Default)]
pub struct TerminalSlot {
    value: Mutex<Option<Result<Terminal, ()>>>,
}

impl TerminalSlot {
    pub fn set(&self, v: Result<Terminal, ()>) {
        *self.value.lock().unwrap() = Some(v);
    }

    pub fn get(&self) -> Option<Result<Terminal, ()>> {
        self.value.lock().unwrap().clone()
    }
}

#[derive(Default)]
pub struct Nav2Tracker {
    pub goal: Option<Arc<dyn CancelableGoal>>,
    pub terminal: Option<Arc<TerminalSlot>>,
    pub last_feedback: Option<String>,
    /// Path length left, from the active FollowPath feedback.
    pub last_feedback_distance: Option<f64>,
}

impl Nav2Tracker {
    pub fn clear(&mut self) {
        self.goal = None;
        self.terminal = None;
        self.last_feedback = None;
        self.last_feedback_distance = None;
    }
}

/// rclpy `GoalStatus` -> `_task_result_from_future` semantics.
pub fn terminal_from_status(status: r2r::GoalStatus, error_code: u16, error_msg: String) -> Result<Terminal, ()> {
    let result = match status {
        r2r::GoalStatus::Succeeded => TaskResult::Succeeded,
        r2r::GoalStatus::Canceled => TaskResult::Canceled,
        r2r::GoalStatus::Aborted => TaskResult::Failed,
        // UNKNOWN and non-terminal statuses are not terminal evidence
        _ => return Err(()),
    };
    Ok(Terminal { result, error_code, error_msg })
}

/// The in-process runtime's outcome as the same evidence (it only reports
/// terminal statuses).
pub fn terminal_from_core(outcome: GoalOutcome) -> Result<(r2r::GoalStatus, Terminal), ()> {
    let status = match outcome.status {
        CoreStatus::Succeeded => r2r::GoalStatus::Succeeded,
        CoreStatus::Canceled => r2r::GoalStatus::Canceled,
        CoreStatus::Aborted => r2r::GoalStatus::Aborted,
    };
    terminal_from_status(status, outcome.error_code, outcome.error_msg).map(|t| (status, t))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn core_outcomes_map_like_nav2_results() {
        let outcome = |status, error_code, error_msg: &str| GoalOutcome { status, error_code, error_msg: error_msg.into() };
        let cases = [
            (outcome(CoreStatus::Succeeded, 0, ""), r2r::GoalStatus::Succeeded, TaskResult::Succeeded),
            (outcome(CoreStatus::Canceled, 0, ""), r2r::GoalStatus::Canceled, TaskResult::Canceled),
            // preempted: the empty result
            (outcome(CoreStatus::Aborted, 0, ""), r2r::GoalStatus::Aborted, TaskResult::Failed),
            (outcome(CoreStatus::Aborted, 105, "Failed to make progress"), r2r::GoalStatus::Aborted, TaskResult::Failed),
        ];
        for (outcome, status, result) in cases {
            let (code, msg) = (outcome.error_code, outcome.error_msg.clone());
            let (s, t) = terminal_from_core(outcome).unwrap();
            assert_eq!(s, status);
            assert_eq!((t.result, t.error_code, t.error_msg), (result, code, msg));
        }
    }
}

/// Only a terminal status is terminal evidence: _task_result_from_future's
/// rule (E9-07, checked only on the Python node by
/// test_qos_configuration.py).
#[cfg(test)]
mod terminal_evidence {
    use super::*;

    #[test]
    fn only_succeeded_canceled_and_aborted_end_a_task() {
        for status in [r2r::GoalStatus::Unknown, r2r::GoalStatus::Accepted, r2r::GoalStatus::Executing, r2r::GoalStatus::Canceling] {
            assert!(terminal_from_status(status, 0, String::new()).is_err(), "{status:?}");
        }
        for (status, result) in [
            (r2r::GoalStatus::Succeeded, TaskResult::Succeeded),
            (r2r::GoalStatus::Canceled, TaskResult::Canceled),
            (r2r::GoalStatus::Aborted, TaskResult::Failed),
        ] {
            let terminal = terminal_from_status(status, 105, "Failed to make progress".into()).expect("terminal");
            assert_eq!((terminal.result, terminal.error_code, terminal.error_msg.as_str()), (result, 105, "Failed to make progress"));
        }
    }
}
