//! Fail-closed cross-node guard for mutations during navigation (port of
//! `navigation_guard.py`): the nav server's transient-local activity flag
//! plus the central `/mission_operation_lock` lease.

use std::time::Duration;

use r2r::mower_interface::srv::MissionOperationLock;

pub const RELEASE_UNCONFIRMED: &str = "operation may have completed, but mutation-lock release is unconfirmed; navigation remains blocked. Inspect the robot, then restart this node and nav_action_server";

/// The lease service's client: r2r, or the in-process bus for a module whose
/// IO step E2 has moved (map, record). The calls and their bounds are the
/// same; on the bus they run on the module's clock.
enum LockClient {
    Ros(r2r::Client<MissionOperationLock::Service>),
    Bus {
        client: mower_bus::Client<MissionOperationLock::Request, MissionOperationLock::Response>,
        clock: mower_bus::Clock,
    },
}

pub struct Guard {
    pub owner: String,
    /// Unknown is treated as active: mutation must not become available
    /// merely because the nav action server is absent / restarting.
    pub nav_active: bool,
    pub nav_seen: bool,
    local_locked: bool,
    lease_held: bool,
    client: LockClient,
    logger: String,
}

impl Guard {
    pub fn new(owner: String, client: r2r::Client<MissionOperationLock::Service>, logger: String) -> Self {
        Guard::with_client(owner, LockClient::Ros(client), logger)
    }

    /// [`Guard::new`] with the lease service reached over the bus, timed on
    /// `clock`.
    pub fn on_bus(
        owner: String,
        client: mower_bus::Client<MissionOperationLock::Request, MissionOperationLock::Response>,
        clock: mower_bus::Clock,
        logger: String,
    ) -> Self {
        Guard::with_client(owner, LockClient::Bus { client, clock }, logger)
    }

    fn with_client(owner: String, client: LockClient, logger: String) -> Self {
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
        let timeout = Duration::from_secs_f64(timeout_s);
        match &self.client {
            LockClient::Ros(client) => {
                let fut = client.request(&req).ok()?;
                tokio::time::timeout(timeout, fut).await.ok()?.ok()
            }
            // The bus fails a call it cannot deliver or that goes unanswered
            // (ServiceUnavailable, NoResponse) instead of leaving it to the
            // timeout: either way it is a call without an answer.
            LockClient::Bus { client, clock } => {
                let fut = client.request(&req).ok()?;
                clock.timeout(timeout, fut).await.ok()?.ok()
            }
        }
    }

    /// Whether the lease service shows up within 1 s.
    async fn available(&self) -> bool {
        let wait = Duration::from_secs(1);
        match &self.client {
            LockClient::Ros(client) => match r2r::Node::is_available(client) {
                Ok(f) => tokio::time::timeout(wait, f).await.is_ok(),
                Err(_) => false,
            },
            LockClient::Bus { client, clock } => clock.timeout(wait, client.wait_for_service()).await.is_ok(),
        }
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
        if !self.available().await {
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
