//! Fail-closed cross-node guard for mutations during navigation (port of
//! `navigation_guard.py`): the nav server's transient-local activity flag
//! plus the central `/mission_operation_lock` lease.

use std::time::Duration;

use r2r::mower_interface::srv::MissionOperationLock;

pub const RELEASE_UNCONFIRMED: &str = "operation may have completed, but mutation-lock release is unconfirmed; navigation remains blocked. Inspect the robot, then restart this node and nav_action_server";

pub struct Guard {
    pub owner: String,
    /// Unknown is treated as active: mutation must not become available
    /// merely because the nav action server is absent / restarting.
    pub nav_active: bool,
    pub nav_seen: bool,
    local_locked: bool,
    lease_held: bool,
    client: r2r::Client<MissionOperationLock::Service>,
    logger: String,
}

impl Guard {
    pub fn new(owner: String, client: r2r::Client<MissionOperationLock::Service>, logger: String) -> Self {
        Guard { owner, nav_active: true, nav_seen: false, local_locked: false, lease_held: false, client, logger }
    }

    /// `_mutation_block_reason() is not None`.
    pub fn blocked(&self) -> bool {
        self.block_reason().is_some()
    }

    /// `_reject_mutation(response, operation)`: the message to answer with
    /// (already logged) when a non-leased operation is unsafe right now.
    pub fn reject_reason(&self, operation: &str) -> Option<String> {
        let why = self.block_reason()?;
        let msg = format!("Cannot {operation}: {why}");
        r2r::log_warn!(&self.logger, "{}", msg);
        Some(msg)
    }

    /// `self._local_mutation_lock.locked()`: a guarded operation is running.
    pub fn busy(&self) -> bool {
        self.local_locked
    }

    fn block_reason(&self) -> Option<&'static str> {
        if !self.nav_seen {
            Some("navigation state is not available")
        } else if self.nav_active {
            Some("navigation is active")
        } else {
            None
        }
    }

    async fn call(&self, acquire: bool, operation: &str, timeout_s: f64) -> Option<MissionOperationLock::Response> {
        let req = MissionOperationLock::Request { owner: self.owner.clone(), operation: operation.to_string(), acquire };
        let fut = self.client.request(&req).ok()?;
        tokio::time::timeout(Duration::from_secs_f64(timeout_s), fut).await.ok()?.ok()
    }

    /// Acquire the nav server's atomic mutation lease, or explain why not.
    pub async fn acquire(&mut self, operation: &str) -> Result<(), String> {
        if let Some(why) = self.block_reason() {
            let msg = format!("Cannot {operation}: {why}");
            r2r::log_warn!(&self.logger, "{}", msg);
            return Err(msg);
        }
        if self.local_locked {
            return Err("Another mutation is already active in this node".into());
        }
        self.local_locked = true;
        let available = match r2r::Node::is_available(&self.client) {
            Ok(f) => tokio::time::timeout(Duration::from_secs(1), f).await.is_ok(),
            Err(_) => false,
        };
        if !available {
            self.local_locked = false;
            return Err("Mission operation guard is unavailable".into());
        }
        match self.call(true, operation, 3.0).await {
            None => {
                // The server may have acquired the lease even though its
                // response was lost. Keep the local lock latched too.
                let msg = "Mission operation guard response timed out; inspect robot state, then restart both this node and nav_action_server".to_string();
                r2r::log_error!(&self.logger, "{}", msg);
                Err(msg)
            }
            Some(r) if !r.success => {
                self.local_locked = false;
                Err(r.message)
            }
            Some(_) => {
                self.lease_held = true;
                Ok(())
            }
        }
    }

    /// Release a held mutation lease; stay fail-closed if uncertain.
    pub async fn release(&mut self) -> bool {
        if !self.lease_held {
            return false;
        }
        match self.call(false, "", 3.0).await {
            Some(r) if r.success => {
                self.lease_held = false;
                self.local_locked = false;
                true
            }
            other => {
                let mut message = "Mission mutation lease release is unconfirmed; navigation remains blocked".to_string();
                if let Some(r) = other {
                    if !r.message.is_empty() {
                        message.push_str(&format!(": {}", r.message));
                    }
                }
                r2r::log_error!(&self.logger, "{}", message);
                false
            }
        }
    }
}
