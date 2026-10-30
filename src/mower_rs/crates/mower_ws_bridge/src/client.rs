//! One WebSocket client: parses rosbridge v2 ops and drives the hub.
//!
//! Supported ops are the ones the app, the studio and the relay use:
//! subscribe (type, throttle_rate), unsubscribe, advertise, unadvertise,
//! publish, call_service (with `/rosapi/topics` and `/rosapi/get_time`
//! answered natively) and status replies on errors. Fragments, compression
//! and actions are not implemented.
//!
//! A connection's frames are handled one at a time, in the order they came,
//! except that a call to a server runs in a task of its own (E3.3, as Python
//! rosbridge's `call_services_in_new_thread`): a slow call holds up neither
//! the frames behind it (the joystick, subscribes) nor the client's other
//! calls, and its reply carries its request's `id`. What a call is checked
//! for (a service name, the allow-list) and `/rosapi/*` are still answered
//! in order. At most `MAX_CALLS_IN_FLIGHT` calls of a connection run at once.

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use serde_json::{json, Value};
use tokio::sync::{mpsc, Semaphore};

use crate::config::Policy;
use crate::entry::TopicEntry;
use crate::hub::Hub;

/// How many calls of one connection may wait for their servers at once; one
/// more is answered at once with `<s>: too many calls in flight …`. The app
/// awaits each call of a flow, but its 1 Hz `/check_nav_status` poll gives
/// up after 4 s and polls again while the bridge holds a call for up to
/// 30 s, so a hung server collects about seven of them: 16 leaves room for
/// the rest.
pub const MAX_CALLS_IN_FLIGHT: usize = 16;

/// The answer to a `call_service`: `values` and `result: true`, or the
/// error text as `values` and `result: false`, with the request's `id` if it
/// had one. The text of a `serde_json::Value`, as every frame.
pub fn service_response_frame(service: &str, response: Result<Value, String>, id: Option<Value>) -> String {
    let mut reply = match response {
        Ok(values) => json!({"op": "service_response", "service": service, "values": values, "result": true}),
        Err(e) => json!({"op": "service_response", "service": service, "values": e, "result": false}),
    };
    if let Some(id) = id {
        reply["id"] = id;
    }
    reply.to_string()
}

/// Send the answer to a call, logging an error.
fn answer(handle: &ClientHandle, logger: &str, peer: &str, service: &str, response: Result<Value, String>, id: Option<Value>) {
    if let Err(e) = &response {
        r2r::log_warn!(logger, "{}: call_service {}: {}", peer, service, e);
    }
    handle.send(service_response_frame(service, response, id).into());
}

/// Outgoing side of a client, shared with topic fan-out.
#[derive(Clone)]
pub struct ClientHandle {
    pub id: u64,
    tx: mpsc::Sender<Arc<str>>,
    closed: Arc<AtomicBool>,
}

impl ClientHandle {
    pub fn send(&self, payload: Arc<str>) {
        // A slow consumer loses messages rather than stalling the graph.
        if self.tx.try_send(payload).is_err() && self.tx.is_closed() {
            self.closed.store(true, Ordering::Relaxed);
        }
    }

    pub fn is_closed(&self) -> bool {
        self.closed.load(Ordering::Relaxed) || self.tx.is_closed()
    }

    fn send_json(&self, v: &Value) {
        self.send(v.to_string().into());
    }

    #[cfg(test)]
    pub(crate) fn for_test(id: u64, tx: mpsc::Sender<Arc<str>>) -> Self {
        ClientHandle { id, tx, closed: Arc::new(AtomicBool::new(false)) }
    }
}

pub struct Client {
    pub handle: ClientHandle,
    hub: Hub,
    policy: Arc<Policy>,
    logger: String,
    peer: String,
    subscriptions: HashMap<String, Arc<TopicEntry>>,
    service_timeout: Duration,
    /// A place for each call of this connection that waits for its server.
    calls: Arc<Semaphore>,
}

impl Client {
    pub fn new(id: u64, tx: mpsc::Sender<Arc<str>>, hub: Hub, policy: Arc<Policy>, logger: String, peer: String, service_timeout: Duration) -> Self {
        Self {
            handle: ClientHandle { id, tx, closed: Arc::new(AtomicBool::new(false)) },
            hub,
            policy,
            logger,
            peer,
            subscriptions: HashMap::new(),
            service_timeout,
            calls: Arc::new(Semaphore::new(MAX_CALLS_IN_FLIGHT)),
        }
    }

    fn status(&self, level: &str, msg: String, id: Option<&Value>) {
        let mut doc = json!({"op": "status", "level": level, "msg": msg});
        if let Some(id) = id {
            doc["id"] = id.clone();
        }
        if level == "error" {
            r2r::log_warn!(&self.logger, "{}: {}", self.peer, doc["msg"].as_str().unwrap_or(""));
        }
        self.handle.send_json(&doc);
    }

    /// Handle one text frame from the client.
    pub async fn on_text(&mut self, text: &str) {
        let doc: Value = match serde_json::from_str(text) {
            Ok(v) => v,
            Err(e) => {
                self.status("error", format!("invalid JSON: {e}"), None);
                return;
            }
        };
        let id = doc.get("id").cloned();
        let op = doc.get("op").and_then(|v| v.as_str()).unwrap_or("");
        let str_field = |k: &str| doc.get(k).and_then(|v| v.as_str()).map(str::to_string);
        match op {
            "subscribe" => {
                let Some(topic) = str_field("topic") else {
                    return self.status("error", "subscribe: missing topic".into(), id.as_ref());
                };
                if !self.policy.may_subscribe(&topic) {
                    return self.status("error", format!("subscribe: {topic} is not allowed"), id.as_ref());
                }
                let throttle = doc.get("throttle_rate").and_then(|v| v.as_u64()).unwrap_or(0);
                match self.hub.subscribe(&topic, str_field("type"), &self.handle, throttle).await {
                    Ok(entry) => {
                        self.subscriptions.insert(topic, entry);
                    }
                    Err(e) => self.status("error", format!("subscribe: {e}"), id.as_ref()),
                }
            }
            "unsubscribe" => {
                if let Some(topic) = str_field("topic") {
                    if let Some(entry) = self.subscriptions.remove(&topic) {
                        entry.remove_client(self.handle.id);
                    }
                }
            }
            "advertise" => {
                let Some(topic) = str_field("topic") else {
                    return self.status("error", "advertise: missing topic".into(), id.as_ref());
                };
                if !self.policy.may_publish(&topic) {
                    return self.status("error", format!("advertise: {topic} is not allowed"), id.as_ref());
                }
                // The publisher is created lazily by the first publish; the
                // type is validated then. Nothing else to do here.
                let _ = str_field("type");
            }
            "unadvertise" => {}
            "publish" => {
                let Some(topic) = str_field("topic") else {
                    return self.status("error", "publish: missing topic".into(), id.as_ref());
                };
                if !self.policy.may_publish(&topic) {
                    return self.status("error", format!("publish: {topic} is not allowed"), id.as_ref());
                }
                let msg = doc.get("msg").cloned().unwrap_or_else(|| json!({}));
                if let Err(e) = self.hub.publish(&topic, str_field("type"), msg).await {
                    self.status("error", format!("publish: {e}"), id.as_ref());
                }
            }
            "call_service" => {
                let Some(service) = str_field("service") else {
                    return self.status("error", "call_service: missing service".into(), id.as_ref());
                };
                let args = doc.get("args").cloned().unwrap_or_else(|| json!({}));
                let response = if service == "/rosapi/topics" {
                    Ok(self.rosapi_topics().await)
                } else if service == "/rosapi/get_time" {
                    // the studio's latency ping
                    let now = self.hub.now();
                    Ok(json!({"time": {"sec": now.as_secs() as i64, "nanosec": now.subsec_nanos()}}))
                } else {
                    match self.policy.service_type(&service) {
                        None => Err(format!("service {service} is not allowed")),
                        Some(t) => return self.spawn_call(service, t.to_string(), args, id),
                    }
                };
                answer(&self.handle, &self.logger, &self.peer, &service, response, id);
            }
            "set_level" | "auth" | "fragment" => {}
            other => self.status("error", format!("unsupported op {other:?}"), id.as_ref()),
        }
    }

    /// Call `service` in a task of its own, bounded by the service timeout,
    /// and answer with the request's `id` whenever its server does; refused
    /// at once while `MAX_CALLS_IN_FLIGHT` calls of this connection wait.
    /// A call runs on after its connection has gone, as it did when the
    /// connection waited for each: a cancel sent just before the link went
    /// still reaches its server. Its answer then goes nowhere.
    fn spawn_call(&self, service: String, service_type: String, args: Value, id: Option<Value>) {
        let Ok(place) = self.calls.clone().try_acquire_owned() else {
            let refused = format!("{service}: too many calls in flight on this connection (at most {MAX_CALLS_IN_FLIGHT})");
            return answer(&self.handle, &self.logger, &self.peer, &service, Err(refused), id);
        };
        let (hub, handle, logger, peer, timeout) =
            (self.hub.clone(), self.handle.clone(), self.logger.clone(), self.peer.clone(), self.service_timeout);
        tokio::spawn(async move {
            let response = hub.call_service(&service, &service_type, args, timeout).await;
            // free before the answer is out, so that a client that sends its
            // next call on this answer finds the place
            drop(place);
            answer(&handle, &logger, &peer, &service, response, id);
        });
    }

    /// `rosapi/srv/Topics` limited to the subscribe allow-list.
    async fn rosapi_topics(&self) -> Value {
        let all = self.hub.topics_and_types().await;
        let mut names: Vec<&String> = all.keys().filter(|t| self.policy.may_subscribe(t)).collect();
        names.sort();
        let types: Vec<String> = names.iter().map(|n| all[*n].first().cloned().unwrap_or_default()).collect();
        json!({"topics": names, "types": types})
    }

    pub fn close(&mut self) {
        self.handle.closed.store(true, Ordering::Relaxed);
        for (_, entry) in self.subscriptions.drain() {
            entry.remove_client(self.handle.id);
        }
    }
}
