//! One WebSocket client: parses rosbridge v2 ops and drives the hub.
//!
//! Supported ops are the ones the app, the studio and the relay use:
//! subscribe (type, throttle_rate), unsubscribe, advertise, unadvertise,
//! publish, call_service (with `/rosapi/topics` answered natively) and
//! status replies on errors. Fragments, compression and actions are not
//! implemented.

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use serde_json::{json, Value};
use tokio::sync::mpsc;

use crate::config::Policy;
use crate::hub::{Hub, TopicEntry};

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
}

pub struct Client {
    pub handle: ClientHandle,
    hub: Hub,
    policy: Arc<Policy>,
    logger: String,
    peer: String,
    subscriptions: HashMap<String, Arc<TopicEntry>>,
    service_timeout: Duration,
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
                match self.hub.subscribe(&topic, str_field("type")).await {
                    Ok(entry) => {
                        entry.add_client(self.handle.clone(), throttle);
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
                } else {
                    match self.policy.service_type(&service) {
                        None => Err(format!("service {service} is not allowed")),
                        Some(t) => self.hub.call_service(&service, t, args, self.service_timeout).await,
                    }
                };
                let mut reply = match response {
                    Ok(values) => json!({"op": "service_response", "service": service, "values": values, "result": true}),
                    Err(e) => {
                        r2r::log_warn!(&self.logger, "{}: call_service {}: {}", self.peer, service, e);
                        json!({"op": "service_response", "service": service, "values": e, "result": false})
                    }
                };
                if let Some(id) = id {
                    reply["id"] = id;
                }
                self.handle.send_json(&reply);
            }
            "set_level" | "auth" | "fragment" => {}
            other => self.status("error", format!("unsupported op {other:?}"), id.as_ref()),
        }
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
