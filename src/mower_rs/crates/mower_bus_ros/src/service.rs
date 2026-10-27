//! Service forwarders: `service_to_ros` (DDS clients call a bus server) and
//! `service_from_ros` (bus clients call a DDS server). Contract: README.md,
//! "Services" and "Graph presence".
//!
//! Calls are forwarded concurrently, each in its own future of the
//! forwarder's task: one slow call must not hold up the others (the bus and
//! DDS servers still decide how they serve them).
//!
//! Each side's server exists only while the other side has one: the DDS
//! server of `service_to_ros` follows the bus server, the bus server of
//! `service_from_ros` the DDS server.

use std::future::Future;
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::time::{Duration, Instant};

use futures::future::{BoxFuture, FutureExt, Shared};
use futures::stream::{FuturesUnordered, StreamExt};
use mower_bus::{BusError, Client, Message, Request, Scope, Service};
use r2r::{QosProfile, ServiceRequest};

use crate::convert::convertible;
use crate::log::{Failure, Log};
use crate::node::{NodeHandle, DWELL, PRESENCE_PERIOD};
use crate::stats::Counters;
use crate::{topic, BridgeError, BridgeService, Forwarder};

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

/// `service_to_ros`: a DDS server whose requests become bus calls, which
/// exists only while the bus has a server. A call in hand that finds no bus
/// server waits up to `wait` for one (`ServiceTimeouts`), while the DDS
/// server still exists.
pub(crate) fn service_to_ros<S: BridgeService>(
    scope: &Scope, name: &str, env: topic::Env, wait: Duration,
) -> Result<Forwarder, BridgeError> {
    // The name and the type are checked now; the DDS server comes with a
    // bus server, once the bridge runs.
    let client = scope.client::<S::Request, S::Response>(name)?;
    let topic::Env { node, counters, log } = env;
    let forwarder = ServiceToRos::<S> { name: name.to_string(), client, node, env: Arc::new(Env { counters, log }), wait };
    Ok(Box::pin(forwarder.run()))
}

struct ServiceToRos<S: BridgeService> {
    name: String,
    client: Client<S::Request, S::Response>,
    node: NodeHandle,
    env: Arc<Env>,
    wait: Duration,
}

impl<S: BridgeService> ServiceToRos<S> {
    /// The DDS server while the bus has a server, made and destroyed on the
    /// spin thread, and its requests forwarded meanwhile. A DDS client then
    /// sees what it saw of the module's own r2r server: none while the
    /// module is down (`is_available` waits, as ws_bridge's 5 s check does),
    /// one as soon as it is up.
    async fn run(self) {
        let mut dds: Option<r2r::Service<S>> = None;
        let mut calls = FuturesUnordered::new();
        // When the DDS server last came or went.
        let mut changed: Option<tokio::time::Instant> = None;
        loop {
            tokio::select! {
                followed = self.presence_changed(dds.is_some(), changed) => {
                    if followed.is_err() {
                        // The bridge's scope closed.
                        return;
                    }
                    match (self.client.is_available(), dds.take()) {
                        (true, None) => {
                            let name = self.name.clone();
                            match self.node.call(move |node| node.create_service::<S>(&name, QosProfile::services_default())).await {
                                Some(Ok(service)) => {
                                    dds = Some(service);
                                    self.env.counters.set_dds_server(true);
                                }
                                Some(Err(e)) => self.env.log.error(Failure::Dds, || format!("creating the DDS server failed: {e}")),
                                // The bridge is stopping.
                                None => return,
                            }
                            self.env.counters.resynced();
                            changed = Some(tokio::time::Instant::now());
                        }
                        (false, Some(service)) => {
                            if self.withdraw(service, std::mem::take(&mut calls)).await.is_none() {
                                // The bridge is stopping.
                                return;
                            }
                            self.env.counters.resynced();
                            changed = Some(tokio::time::Instant::now());
                        }
                        // It came back while the dwell ran.
                        (_, same) => dds = same,
                    }
                }
                request = next_dds_request(&mut dds) => match request {
                    Some(request) => calls.push(answer::<S>(self.client.clone(), request, self.env.clone(), self.wait)),
                    // Only with the node: the bridge is stopping.
                    None => return,
                },
                Some(()) = calls.next() => {}
            }
        }
    }

    /// Once the bus is not as the DDS server is (no bus server while there
    /// is a DDS server, `served`; one while there is none), and `DWELL`
    /// after the last change. `Closed`: the bridge's scope closed.
    async fn presence_changed(&self, served: bool, changed: Option<tokio::time::Instant>) -> Result<(), BusError> {
        if served {
            self.client.wait_for_no_service().await?;
        } else {
            self.client.wait_for_service().await?;
        }
        if let Some(changed) = changed {
            tokio::time::sleep_until(changed + DWELL).await;
        }
        Ok(())
    }

    /// The bus server is gone: so is the DDS server, as the module's own r2r
    /// server went with its node. What the server answered before it went
    /// still goes out first. The other calls in hand, and the requests r2r
    /// has queued, can be answered by nothing any more: none was taken by a
    /// live server, and one made to the next server would run for a client
    /// that gets no answer. They are lost, as with a crashed r2r server.
    /// `None`: the bridge is stopping.
    async fn withdraw(
        &self, mut service: r2r::Service<S>, mut calls: FuturesUnordered<impl Future<Output = ()>>,
    ) -> Option<()> {
        while let Some(Some(())) = calls.next().now_or_never() {}
        let mut lost = calls.len();
        drop(calls);
        while let Some(Some(_)) = service.next().now_or_never() {
            lost += 1;
        }
        for _ in 0..lost {
            self.env.counters.unanswered();
        }
        if lost > 0 {
            self.env.log.warn(Failure::BusUnavailable, || {
                format!("the bus server went away, so the {lost} DDS calls in hand get no answer")
            });
        }
        self.env.counters.set_dds_server(false);
        // Awaited, so that the next DDS server never lives next to this one.
        self.node.call(move |node| node.destroy_service(service)).await
    }
}

/// The next request of the DDS server; while there is none, nothing.
async fn next_dds_request<S: BridgeService>(service: &mut Option<r2r::Service<S>>) -> Option<ServiceRequest<S>> {
    match service {
        Some(service) => service.next().await,
        None => std::future::pending().await,
    }
}

/// One DDS request: call the bus server and answer with what it says.
///
/// A call the bus refuses at once, because no server is live when it is
/// made, waits up to `wait` for one and is made once more. Nothing has run
/// for it yet, so that is safe. Only a call in hand meets that: one that came
/// as the bus server went, before the DDS server did. The forwarder drops it
/// once the DDS server goes, as nothing could answer it then; a server that
/// comes back within `DWELL` keeps the DDS server and answers it.
///
/// A call a server has taken is never made again, however it ends: its
/// handler may have run, and dispatch, start-coverage and the lock services
/// are not idempotent. A direct r2r server never runs a request twice. So a
/// server that closes holding it (`ServiceUnavailable` from the response) or
/// drops it (`NoResponse`) loses it, as a crashed r2r server would.
///
/// ROS has no error reply, so the request is then dropped unanswered, as it
/// is on any other bus error, when the wait runs out, and when r2r cannot
/// convert the response: the ROS client sees what it sees of a crashed
/// server, and times out.
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
    let response = match response {
        Ok(response) => response,
        Err(e) => {
            env.counters.unanswered();
            return env.log.warn(Failure::of(&e), || format!("the bus call failed ({e}{waited}), so the ROS client gets no answer"));
        }
    };
    // r2r converts the response under its service's lock (convert.rs).
    if let Err(why) = convertible(&response) {
        env.counters.invalid();
        env.counters.unanswered();
        return env.log.error(Failure::Invalid, || {
            format!("r2r cannot convert the bus server's response ({why}), so the ROS client gets no answer")
        });
    }
    match request.respond(response) {
        Ok(()) => {
            env.counters.forwarded();
            env.log.ok();
        }
        Err(e) => {
            env.counters.unanswered();
            env.log.error(Failure::Dds, || format!("sending the response failed: {e}"));
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
///
/// The bus server exists only while the graph shows a DDS server, looked at
/// every `PRESENCE_PERIOD` on the spin thread (`node`), so that a bus caller
/// finds what an r2r caller found: no server while there is none
/// (`ServiceUnavailable` at the call, `wait_for_service` waits), one as soon as
/// there is. What the graph shows is never the bridge's own: one direction per
/// name. A server that goes takes the bus server with it, and the calls it
/// still holds end with `ServiceUnavailable`.
///
/// The graph can show a server before the bridge's client is matched with
/// it, and calls until then would end in `NoResponse`: mower_rs_common's
/// guard takes that for a lock that may have been granted, and stays
/// latched. So the bus server comes only once a probe finds the server
/// matched too, as an r2r caller's `is_available` did.
pub(crate) fn service_from_ros<S: BridgeService>(
    scope: &Scope, node: &mut r2r::Node, handle: NodeHandle, name: &str, counters: Arc<Counters>, log: Log,
    deadline: Duration,
) -> Result<Forwarder, BridgeError> {
    // The name, the type and a bus server already there are checked now;
    // the bus server itself comes with a DDS server.
    drop(scope.service::<S::Request, S::Response>(name)?);
    let client = node.create_client::<S>(name, QosProfile::services_default())?;
    let server = Arc::new(DdsServer::new(client));
    let env = Arc::new(Env { counters, log });
    let (scope, name) = (scope.clone(), name.to_string());
    // A server's writer: clients write `rq<name>Request` and read this.
    let replies = format!("rr{name}Reply");
    Ok(Box::pin(async move {
        let mut service: Option<Serving<S::Request, S::Response>> = None;
        let mut calls = FuturesUnordered::new();
        let mut look = tokio::time::interval(PRESENCE_PERIOD);
        look.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
        loop {
            tokio::select! {
                _ = look.tick() => {
                    let replies = replies.clone();
                    let shown = handle.call(move |node| node.get_publishers_info_by_topic(&replies, true)).await;
                    env.counters.resynced();
                    match shown {
                        // The bridge is stopping.
                        None => return,
                        Some(Ok(servers)) if servers.is_empty() => {
                            // Dropping it closes its scope, which ends the calls
                            // it holds.
                            if service.take().is_some() {
                                env.counters.set_bus_server(false);
                            }
                            // A server that comes next is probed afresh.
                            server.forget();
                        }
                        Some(Ok(_)) if service.is_some() => {}
                        // Not matched yet (within `PROBE_WINDOW`): the next look
                        // probes again.
                        Some(Ok(_)) if !server.available().await => {}
                        Some(Ok(_)) => match Serving::new(&scope, &name) {
                            Ok(serving) => {
                                service = Some(serving);
                                env.counters.set_bus_server(true);
                            }
                            // The bridge's scope closed.
                            Err(BusError::Closed { .. }) => return,
                            Err(e) => env.log.error(Failure::BusOther, || format!("serving on the bus failed: {e}")),
                        },
                        Some(Err(e)) => env.log.error(Failure::Dds, || format!("looking at the graph failed: {e}")),
                    }
                }
                request = next_request(&mut service) => match request {
                    Some(request) => calls.push(call(server.clone(), request, env.clone(), deadline)),
                    // The bridge's scope closed.
                    None => return,
                },
                Some(()) = calls.next() => {}
            }
        }
    }))
}

/// The next request of `service`; while there is no service, nothing.
async fn next_request<Req: Message, Resp: Message>(
    service: &mut Option<Serving<Req, Resp>>,
) -> Option<Request<Req, Resp>> {
    match service {
        Some(serving) => serving.service.next().await,
        None => std::future::pending().await,
    }
}

/// A `service_from_ros` bus server, in a child scope of its own that closes
/// when the server goes, as `to_ros`'s readers do: the bridge's scope keeps
/// no close hook of it, however often the DDS server comes and goes (mower_bus
/// README, "Hooks of dropped endpoints"). The close ends the calls it holds
/// with `ServiceUnavailable`, as dropping it did.
struct Serving<Req: Message, Resp: Message> {
    service: Service<Req, Resp>,
    own_scope: Scope,
}

impl<Req: Message, Resp: Message> Serving<Req, Resp> {
    fn new(scope: &Scope, name: &str) -> Result<Self, BusError> {
        let own_scope = scope.child(scope.name());
        match own_scope.service::<Req, Resp>(name) {
            Ok(service) => Ok(Serving { service, own_scope }),
            Err(e) => {
                own_scope.close();
                Err(e)
            }
        }
    }
}

impl<Req: Message, Resp: Message> Drop for Serving<Req, Resp> {
    fn drop(&mut self) {
        self.own_scope.close();
    }
}

/// One bus request: call the DDS server, if there is one, and answer with
/// what it says. Dropping `request` unanswered gives the bus caller
/// `NoResponse`, which is what it gets while no DDS server is matched, when
/// none is any more during the call, when the call passes `deadline`, and
/// when r2r cannot convert the request.
///
/// The deadline is what ends a call whose server is replaced while it holds
/// it (a crash and a respawn, or a second instance that comes up first): a
/// server is matched at every check then, only not the one that has the
/// call, and r2r's response future never completes.
async fn call<S: BridgeService>(
    server: Arc<DdsServer<r2r::Client<S>>>, request: Request<S::Request, S::Response>, env: Arc<Env>, deadline: Duration,
) {
    // r2r converts the request under its client's lock (convert.rs).
    if let Err(why) = convertible(&request.message) {
        env.counters.invalid();
        env.counters.unanswered();
        return env.log.error(Failure::Invalid, || {
            format!("r2r cannot convert the bus request ({why}), so the bus caller gets NoResponse")
        });
    }
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

/// The DDS side of `service_from_ros` and `action_from_ros`: the client, and
/// what is known about its server.
pub(crate) struct DdsServer<C> {
    pub(crate) client: C,
    probe: Mutex<Probe>,
}

/// A DDS client whose server r2r can probe for: a service client, or an
/// action client (action.rs).
pub(crate) trait Probed {
    /// r2r's `Node::is_available` for the client: true once the node's
    /// next spin finds the server matched; `None` once the client is gone
    /// with the node.
    fn probe(&self) -> Option<BoxFuture<'static, bool>>;
}

impl<S: BridgeService> Probed for r2r::Client<S> {
    fn probe(&self) -> Option<BoxFuture<'static, bool>> {
        let registered = r2r::Node::is_available(self).ok()?;
        Some(async move { registered.await.is_ok() }.boxed())
    }
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

impl<C: Probed> DdsServer<C> {
    pub(crate) fn new(client: C) -> Self {
        DdsServer { client, probe: Mutex::new(Probe::default()) }
    }

    /// Whether the server is matched now, as far as a probe can tell within
    /// `PROBE_WINDOW`.
    pub(crate) async fn available(&self) -> bool {
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

    /// The graph shows no server: what a probe confirmed was about one that
    /// is gone, and must not count for the next (`FRESH`).
    pub(crate) fn forget(&self) {
        lock(&self.probe).seen = None;
    }

    /// Wait for `response`, or `None` once the server is no longer matched.
    /// r2r's response future never completes when the server dies during
    /// the call; the bus caller would wait forever, and the call would stay
    /// in this forwarder for good.
    pub(crate) async fn watch<F: Future>(&self, response: F) -> Option<F::Output> {
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
    fn current(&mut self, client: &impl Probed) -> Option<Shared<BoxFuture<'static, bool>>> {
        if let Some(pending) = &self.pending {
            if pending.clone().now_or_never().is_none() {
                return Some(pending.clone());
            }
        }
        let probe = client.probe()?.shared();
        self.pending = Some(probe.clone());
        Some(probe)
    }
}

fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    // No await and nothing that panics under this lock.
    mutex.lock().unwrap_or_else(PoisonError::into_inner)
}
