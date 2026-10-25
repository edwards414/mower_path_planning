//! Service forwarders: `service_to_ros` (DDS clients call a bus server) and
//! `service_from_ros` (bus clients call a DDS server). Contract: README.md,
//! "Services".
//!
//! Calls are forwarded concurrently, each in its own future of the
//! forwarder's task: one slow call must not hold up the others (the bus and
//! DDS servers still decide how they serve them).

use std::future::Future;
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::time::{Duration, Instant};

use futures::future::{BoxFuture, FutureExt, Shared};
use futures::stream::{FuturesUnordered, StreamExt};
use mower_bus::{BusError, Client, Message, Request, Scope};
use r2r::{QosProfile, ServiceRequest};

use crate::log::{Failure, Log};
use crate::stats::Counters;
use crate::{BridgeError, BridgeService, Forwarder};

/// How long a bus call waits for the DDS server to be matched before it is
/// dropped. r2r answers an availability probe at the node's next spin, at
/// most `node::SPIN_TIMEOUT` (20 ms) away; the rest is margin for a busy
/// robot.
const PROBE_WINDOW: Duration = Duration::from_millis(500);

/// How long a confirmed DDS server counts as available without a new probe,
/// so that a busy service does not wait for a spin on every call.
const FRESH: Duration = Duration::from_secs(1);

/// How often a pending DDS call checks that its server is still there.
const WATCH_PERIOD: Duration = Duration::from_secs(1);

/// What the calls of one forwarder share.
struct Env {
    counters: Arc<Counters>,
    log: Log,
}

/// `service_to_ros`: a DDS server whose requests become bus calls. A call
/// that finds no bus server waits up to `wait` for one (`ServiceTimeouts`).
pub(crate) fn service_to_ros<S: BridgeService>(
    scope: &Scope, node: &mut r2r::Node, name: &str, counters: Arc<Counters>, log: Log, wait: Duration,
) -> Result<Forwarder, BridgeError> {
    let client = scope.client::<S::Request, S::Response>(name)?;
    let mut requests = node.create_service::<S>(name, QosProfile::services_default())?;
    let env = Arc::new(Env { counters, log });
    Ok(Box::pin(async move {
        let mut calls = FuturesUnordered::new();
        loop {
            tokio::select! {
                request = requests.next() => match request {
                    Some(request) => calls.push(answer::<S>(client.clone(), request, env.clone(), wait)),
                    None => return,
                },
                Some(()) = calls.next() => {}
            }
        }
    }))
}

/// One DDS request: call the bus server and answer with what it says.
///
/// A call the bus refuses at once, because no server is live when it is
/// made, waits up to `wait` for one and is made once more. Nothing has run
/// for it yet, so that is safe. The DDS service stays advertised meanwhile
/// (r2r cannot take it back), so a DDS client cannot learn that the server
/// is gone; but mower_rsd starts a failed module again after 2 s, and a call
/// made during that restart is answered by the restarted module.
///
/// A call a server has taken is never made again, however it ends: its
/// handler may have run, and dispatch, start-coverage and the lock services
/// are not idempotent. A direct r2r server never runs a request twice. So a
/// server that closes holding it (`ServiceUnavailable` from the response) or
/// drops it (`NoResponse`) loses it, as a crashed r2r server would.
///
/// ROS has no error reply, so the request is then dropped unanswered, as it
/// is on any other bus error or when the wait runs out: the ROS client sees
/// what it sees of a crashed server, and times out.
async fn answer<S: BridgeService>(
    client: Client<S::Request, S::Response>, request: ServiceRequest<S>, env: Arc<Env>, wait: Duration,
) {
    let mut waited = "";
    let response = match client.request(&request.message) {
        Ok(response) => response.await,
        Err(BusError::ServiceUnavailable { .. }) => match tokio::time::timeout(wait, client.wait_for_service()).await {
            Ok(Ok(())) => bus_call(&client, &request.message).await,
            // The bridge's scope closed.
            Ok(Err(e)) => Err(e),
            Err(_) => {
                waited = ", and none came in time";
                Err(BusError::ServiceUnavailable { name: client.name().to_string() })
            }
        },
        Err(e) => Err(e),
    };
    match response {
        Ok(response) => match request.respond(response) {
            Ok(()) => {
                env.counters.forwarded();
                env.log.ok();
            }
            Err(e) => {
                env.counters.unanswered();
                env.log.error(Failure::Dds, || format!("sending the response failed: {e}"));
            }
        },
        Err(e) => {
            env.counters.unanswered();
            env.log.warn(Failure::of(&e), || format!("the bus call failed ({e}{waited}), so the ROS client gets no answer"));
        }
    }
}

/// A bus call, its two levels of error in one.
async fn bus_call<Req: Message, Resp: Message>(client: &Client<Req, Resp>, request: &Req) -> Result<Resp, BusError> {
    match client.request(request) {
        Ok(response) => response.await,
        Err(e) => Err(e),
    }
}

/// `service_from_ros`: a bus server whose requests become DDS calls, each
/// given up at `deadline` (`ServiceTimeouts`).
pub(crate) fn service_from_ros<S: BridgeService>(
    scope: &Scope, node: &mut r2r::Node, name: &str, counters: Arc<Counters>, log: Log, deadline: Duration,
) -> Result<Forwarder, BridgeError> {
    let mut service = scope.service::<S::Request, S::Response>(name)?;
    let client = node.create_client::<S>(name, QosProfile::services_default())?;
    let server = Arc::new(DdsServer { client, probe: Mutex::new(Probe::default()) });
    let env = Arc::new(Env { counters, log });
    Ok(Box::pin(async move {
        let mut calls = FuturesUnordered::new();
        loop {
            tokio::select! {
                request = service.next() => match request {
                    Some(request) => calls.push(call(server.clone(), request, env.clone(), deadline)),
                    // The bridge's scope closed.
                    None => return,
                },
                Some(()) = calls.next() => {}
            }
        }
    }))
}

/// One bus request: call the DDS server, if there is one, and answer with
/// what it says. Dropping `request` unanswered gives the bus caller
/// `NoResponse`, which is what it gets while no DDS server is matched, when
/// none is any more during the call, and when the call passes `deadline`.
///
/// The deadline is what ends a call whose server is replaced while it holds
/// it (a crash and a respawn, or a second instance that comes up first): a
/// server is matched at every check then, only not the one that has the
/// call, and r2r's response future never completes.
async fn call<S: BridgeService>(
    server: Arc<DdsServer<S>>, request: Request<S::Request, S::Response>, env: Arc<Env>, deadline: Duration,
) {
    if !server.available().await {
        env.counters.unanswered();
        env.log.warn(Failure::DdsUnavailable, || "no DDS server is matched, so the bus caller gets NoResponse".into());
        return;
    }
    let response = match server.client.request(&request.message) {
        Ok(response) => response,
        Err(e) => {
            env.counters.unanswered();
            return env.log.error(Failure::Dds, || format!("sending the DDS request failed: {e}"));
        }
    };
    let Ok(response) = tokio::time::timeout(deadline, server.watch(response)).await else {
        env.counters.unanswered();
        return env.log.warn(Failure::DdsTimeout, || {
            format!("the DDS server did not answer within {deadline:?}, so the bus caller gets NoResponse")
        });
    };
    match response {
        Some(Ok(response)) => {
            // `Closed` only: the bridge's scope closed meanwhile, and nobody
            // waits for the answer.
            let _ = request.respond(response);
            env.counters.forwarded();
            env.log.ok();
        }
        Some(Err(e)) => {
            env.counters.unanswered();
            env.log.error(Failure::Dds, || format!("the DDS call failed: {e}"));
        }
        None => {
            env.counters.unanswered();
            env.log.warn(Failure::DdsLost, || "the DDS server went away during a call, so the bus caller gets NoResponse".into());
        }
    }
}

/// The DDS side of `service_from_ros`: the client, and what is known about
/// its server.
struct DdsServer<S: BridgeService> {
    client: r2r::Client<S>,
    probe: Mutex<Probe>,
}

/// r2r tells whether a server is matched only through `Node::is_available`:
/// a future that the node's next spin completes if the server is matched,
/// and that stays registered in the client until one is. So one probe at a
/// time is kept, and reused while it is still pending: a pending probe is
/// answered at the next spin like a new one, and new ones would pile up in
/// the client for as long as the server is missing.
#[derive(Default)]
struct Probe {
    /// When a probe last saw the server matched.
    seen: Option<Instant>,
    pending: Option<Shared<BoxFuture<'static, bool>>>,
}

impl<S: BridgeService> DdsServer<S> {
    /// Whether the server is matched now, as far as a probe can tell within
    /// `PROBE_WINDOW`.
    async fn available(&self) -> bool {
        let probe = {
            let mut probe = lock(&self.probe);
            if probe.seen.is_some_and(|seen| seen.elapsed() < FRESH) {
                return true;
            }
            probe.current(&self.client)
        };
        let Some(probe) = probe else {
            // The client is gone with the node: the bridge is stopping.
            return false;
        };
        let available = matches!(tokio::time::timeout(PROBE_WINDOW, probe).await, Ok(true));
        if available {
            lock(&self.probe).seen = Some(Instant::now());
        }
        available
    }

    /// Wait for `response`, or `None` once the server is no longer matched.
    /// r2r's response future never completes when the server dies during
    /// the call; the bus caller would wait forever, and the call would stay
    /// in this forwarder for good.
    async fn watch<F: Future>(&self, response: F) -> Option<F::Output> {
        let mut response = std::pin::pin!(response);
        loop {
            match tokio::time::timeout(WATCH_PERIOD, response.as_mut()).await {
                Ok(output) => return Some(output),
                Err(_) if self.available().await => {}
                Err(_) => return None,
            }
        }
    }
}

impl Probe {
    /// The pending probe, or a new one when the last has completed: its
    /// answer is from some earlier spin, and r2r has let go of it.
    fn current<S: BridgeService>(&mut self, client: &r2r::Client<S>) -> Option<Shared<BoxFuture<'static, bool>>> {
        if let Some(pending) = &self.pending {
            if pending.clone().now_or_never().is_none() {
                return Some(pending.clone());
            }
        }
        let registered = r2r::Node::is_available(client).ok()?;
        let probe = async move { registered.await.is_ok() }.boxed().shared();
        self.pending = Some(probe.clone());
        Some(probe)
    }
}

fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    // No await and nothing that panics under this lock.
    mutex.lock().unwrap_or_else(PoisonError::into_inner)
}
