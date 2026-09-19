//! Correlated Nav2 goal tracking: the cancelable goal handle and the
//! observed terminal result of the active dispatch generation
//! (the `_nav2_current_goal_handle` / `_nav2_current_result_future` pair).

use std::future::Future;
use std::pin::Pin;
use std::sync::{Arc, Mutex};

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
}

impl Nav2Tracker {
    pub fn clear(&mut self) {
        self.goal = None;
        self.terminal = None;
        self.last_feedback = None;
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
