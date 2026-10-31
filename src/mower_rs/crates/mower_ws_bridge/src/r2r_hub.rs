//! The r2r backend: the bridge's ROS side as one r2r node. r2r's `Node` is
//! single-owner and not `Sync`, so it lives on one thread that spins and
//! services commands from the WebSocket clients through a channel.
//! Subscriptions are shared: one ROS subscription per topic, its JSON
//! fanned out to every client that asked for it (rosbridge serialises per
//! client; here a message is serialised once). Publishers and service
//! clients are created once per topic / service and reused.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use futures::StreamExt;
use r2r::QosProfile;
use serde_json::Value;
use tokio::sync::{mpsc, oneshot};

use crate::entry::TopicEntry;

/// How long a call waits for its service to be matched before failing.
const AVAILABILITY_TIMEOUT: Duration = Duration::from_secs(5);

/// Requests that need the node thread.
pub enum Cmd {
    /// One shared subscription for `topic` (creating it if needed).
    Subscribe { topic: String, type_hint: Option<String>, reply: oneshot::Sender<Result<Arc<TopicEntry>, String>> },
    Publish { topic: String, type_hint: Option<String>, msg: Value, reply: oneshot::Sender<Result<(), String>> },
    CallService { service: String, service_type: String, args: Value, reply: oneshot::Sender<Result<Value, String>> },
    TopicsAndTypes { reply: oneshot::Sender<HashMap<String, Vec<String>>> },
}

#[derive(Clone)]
pub struct R2rHub {
    tx: mpsc::UnboundedSender<Cmd>,
    /// Publishing on the node's private wake topic interrupts `spin_once`
    /// so a queued command is handled at once instead of after the spin
    /// timeout (r2r exposes no guard condition).
    wake: Arc<Mutex<r2r::Publisher<r2r::std_msgs::msg::Empty>>>,
    running: Arc<std::sync::atomic::AtomicBool>,
    thread: Arc<Mutex<Option<std::thread::JoinHandle<()>>>>,
}

struct NodeState {
    node: r2r::Node,
    logger: String,
    topics: HashMap<String, Arc<TopicEntry>>,
    publishers: HashMap<String, (String, r2r::PublisherUntyped)>,
    clients: HashMap<String, Arc<r2r::ClientUntyped>>,
}

impl R2rHub {
    /// Start the node thread; returns the handle and the logger name.
    pub fn start(mut node: r2r::Node) -> Result<R2rHub, String> {
        let (tx, mut rx) = mpsc::unbounded_channel::<Cmd>();
        let logger = node.logger().to_string();
        let runtime = tokio::runtime::Handle::current();
        // The wake topic also keeps the wait set non-empty: r2r's spin_once
        // returns immediately on an empty wait set (a busy loop).
        let wake_topic = format!("{}/wake", node.fully_qualified_name().map_err(|e| format!("{e:?}"))?);
        let wake = node
            .create_publisher::<r2r::std_msgs::msg::Empty>(&wake_topic, QosProfile::default().keep_last(1))
            .map_err(|e| format!("wake publisher: {e:?}"))?;
        let mut wake_sub = node
            .subscribe::<r2r::std_msgs::msg::Empty>(&wake_topic, QosProfile::default().keep_last(1))
            .map_err(|e| format!("wake subscription: {e:?}"))?;
        runtime.spawn(async move { while wake_sub.next().await.is_some() {} });
        let running = Arc::new(std::sync::atomic::AtomicBool::new(true));
        let thread = {
            let running = running.clone();
            std::thread::Builder::new()
                .name("ros-node".into())
                .spawn(move || {
                    let mut st = NodeState { node, logger, topics: HashMap::new(), publishers: HashMap::new(), clients: HashMap::new() };
                    while running.load(std::sync::atomic::Ordering::Relaxed) {
                        let t = Instant::now();
                        st.node.spin_once(Duration::from_millis(100));
                        if t.elapsed() < Duration::from_micros(200) {
                            // defensive: never let an empty wait set spin hot
                            std::thread::sleep(Duration::from_millis(2));
                        }
                        loop {
                            match rx.try_recv() {
                                Ok(cmd) => st.handle(cmd, &runtime),
                                Err(mpsc::error::TryRecvError::Empty) => break,
                                Err(mpsc::error::TryRecvError::Disconnected) => return,
                            }
                        }
                    }
                    // the node (and with it every subscription, publisher and
                    // client) is dropped here, on the thread that used it
                })
                .expect("ros thread")
        };
        Ok(R2rHub { tx, wake: Arc::new(Mutex::new(wake)), running, thread: Arc::new(Mutex::new(Some(thread))) })
    }

    /// Stop the node thread and wait for it, so rcl is torn down before the
    /// process exits (otherwise rmw's own destructors race the spinning
    /// thread and glibc aborts on a locked mutex).
    pub fn shutdown(&self) {
        self.running.store(false, std::sync::atomic::Ordering::Relaxed);
        let _ = self.wake.lock().unwrap().publish(&r2r::std_msgs::msg::Empty {});
        if let Some(t) = self.thread.lock().unwrap().take() {
            let _ = t.join();
        }
    }

    fn send(&self, cmd: Cmd) -> Result<(), String> {
        self.tx.send(cmd).map_err(|_| "ros thread gone".to_string())?;
        let _ = self.wake.lock().unwrap().publish(&r2r::std_msgs::msg::Empty {});
        Ok(())
    }

    pub async fn subscribe(&self, topic: &str, type_hint: Option<String>) -> Result<Arc<TopicEntry>, String> {
        let (reply, rx) = oneshot::channel();
        self.send(Cmd::Subscribe { topic: topic.into(), type_hint, reply })?;
        rx.await.map_err(|_| "ros thread gone".to_string())?
    }

    pub async fn publish(&self, topic: &str, type_hint: Option<String>, msg: Value) -> Result<(), String> {
        let (reply, rx) = oneshot::channel();
        self.send(Cmd::Publish { topic: topic.into(), type_hint, msg, reply })?;
        rx.await.map_err(|_| "ros thread gone".to_string())?
    }

    /// Call a service; the future resolves off the node thread.
    pub async fn call_service(&self, service: &str, service_type: &str, args: Value, timeout: Duration) -> Result<Value, String> {
        let (reply, rx) = oneshot::channel();
        self.send(Cmd::CallService { service: service.into(), service_type: service_type.into(), args, reply })?;
        match tokio::time::timeout(timeout, rx).await {
            Ok(Ok(r)) => r,
            Ok(Err(_)) => Err("ros thread gone".into()),
            Err(_) => Err(format!("service {service} did not answer within {} s", timeout.as_secs())),
        }
    }

    pub async fn topics_and_types(&self) -> HashMap<String, Vec<String>> {
        let (reply, rx) = oneshot::channel();
        if self.send(Cmd::TopicsAndTypes { reply }).is_err() {
            return HashMap::new();
        }
        rx.await.unwrap_or_default()
    }
}

/// Subscription QoS that matches the discovered publishers, as rosbridge
/// does: reliable only if every publisher is, transient-local only if every
/// publisher is (a durable subscriber never matches a volatile publisher).
fn matching_qos(node: &r2r::Node, topic: &str) -> (QosProfile, bool) {
    let infos = node.get_publishers_info_by_topic(topic, false).unwrap_or_default();
    let mut qos = QosProfile::default().keep_last(10);
    let mut latched = false;
    if !infos.is_empty() {
        let all_reliable = infos.iter().all(|i| i.qos_profile.reliability == r2r::qos::ReliabilityPolicy::Reliable);
        let all_latched = infos.iter().all(|i| i.qos_profile.durability == r2r::qos::DurabilityPolicy::TransientLocal);
        if !all_reliable {
            qos = qos.best_effort();
        }
        if all_latched {
            qos = qos.transient_local();
            latched = true;
        }
    }
    (qos, latched)
}

impl NodeState {
    fn topic_type(&self, topic: &str, hint: Option<String>) -> Result<String, String> {
        if let Some(t) = hint.filter(|t| !t.is_empty()) {
            return Ok(t);
        }
        let types = self.node.get_topic_names_and_types().map_err(|e| format!("graph query failed: {e:?}"))?;
        types
            .get(topic)
            .and_then(|v| v.first().cloned())
            .ok_or_else(|| format!("cannot infer type of {topic}: not advertised yet"))
    }

    fn handle(&mut self, cmd: Cmd, runtime: &tokio::runtime::Handle) {
        match cmd {
            Cmd::Subscribe { topic, type_hint, reply } => {
                let _ = reply.send(self.subscribe(&topic, type_hint, runtime));
            }
            Cmd::Publish { topic, type_hint, msg, reply } => {
                let _ = reply.send(self.publish(&topic, type_hint, msg));
            }
            Cmd::CallService { service, service_type, args, reply } => {
                if !self.clients.contains_key(&service) {
                    match self.node.create_client_untyped(&service, &service_type, QosProfile::services_default()) {
                        Ok(c) => {
                            self.clients.insert(service.clone(), Arc::new(c));
                        }
                        Err(e) => {
                            let _ = reply.send(Err(format!("cannot create client for {service} ({service_type}): {e:?}")));
                            return;
                        }
                    }
                }
                let client = self.clients.get(&service).expect("just inserted").clone();
                let available = match r2r::Node::is_available(client.as_ref()) {
                    Ok(f) => f,
                    Err(e) => {
                        let _ = reply.send(Err(format!("{service}: {e:?}")));
                        return;
                    }
                };
                // Resolve on the runtime so the node thread keeps spinning. The
                // request is only sent once the server is matched: a request
                // sent by a freshly created client before discovery completes
                // is silently dropped by DDS and would never be answered.
                // Several calls of one client wait at once (E3.3); r2r matches
                // each answer to its request by sequence number.
                runtime.spawn(async move {
                    let mut reply = reply;
                    if tokio::time::timeout(AVAILABILITY_TIMEOUT, available).await.map(|r| r.is_err()).unwrap_or(true) {
                        let _ = reply.send(Err(format!("{service}: service unavailable")));
                        return;
                    }
                    let request = match client.request(args) {
                        Ok(f) => f,
                        Err(e) => {
                            let _ = reply.send(Err(format!("{service}: request failed: {e:?}")));
                            return;
                        }
                    };
                    // A server that never answers no longer keeps this task
                    // for good: it ends with the caller's wait (the service
                    // timeout), as every other part of the call does.
                    let response = tokio::select! {
                        response = request => Some(response),
                        () = reply.closed() => None,
                    };
                    let Some(response) = response else { return };
                    let result = match response {
                        Ok(Ok(v)) => Ok(v),
                        Ok(Err(e)) => Err(format!("{service}: {e:?}")),
                        Err(e) => Err(format!("{service}: {e:?}")),
                    };
                    let _ = reply.send(result);
                });
            }
            Cmd::TopicsAndTypes { reply } => {
                let _ = reply.send(self.node.get_topic_names_and_types().unwrap_or_default());
            }
        }
    }

    fn subscribe(&mut self, topic: &str, hint: Option<String>, runtime: &tokio::runtime::Handle) -> Result<Arc<TopicEntry>, String> {
        if let Some(entry) = self.topics.get(topic) {
            return Ok(entry.clone());
        }
        let topic_type = self.topic_type(topic, hint)?;
        let (qos, latched) = matching_qos(&self.node, topic);
        let mut stream = self
            .node
            .subscribe_untyped(topic, &topic_type, qos)
            .map_err(|e| format!("subscribe {topic} ({topic_type}) failed: {e:?}"))?;
        let entry = Arc::new(TopicEntry::new(topic, &topic_type, latched));
        self.topics.insert(topic.to_string(), entry.clone());
        r2r::log_info!(&self.logger, "subscribed {} [{}]{}", topic, topic_type, if latched { " latched" } else { "" });
        let fan = entry.clone();
        let logger = self.logger.clone();
        runtime.spawn(async move {
            while let Some(item) = stream.next().await {
                match item {
                    Ok(msg) => fan.fan_out(&msg),
                    Err(e) => r2r::log_warn!(&logger, "{}: bad message: {:?}", fan.topic, e),
                }
            }
        });
        Ok(entry)
    }

    fn publish(&mut self, topic: &str, hint: Option<String>, msg: Value) -> Result<(), String> {
        if !self.publishers.contains_key(topic) {
            let topic_type = self.topic_type(topic, hint)?;
            let publisher = self
                .node
                .create_publisher_untyped(topic, &topic_type, QosProfile::default().keep_last(10))
                .map_err(|e| format!("advertise {topic} ({topic_type}) failed: {e:?}"))?;
            r2r::log_info!(&self.logger, "advertised {} [{}]", topic, topic_type);
            self.publishers.insert(topic.to_string(), (topic_type, publisher));
        }
        let (_, publisher) = self.publishers.get(topic).expect("just inserted");
        publisher.publish(msg).map_err(|e| format!("publish {topic} failed: {e:?}"))
    }
}
