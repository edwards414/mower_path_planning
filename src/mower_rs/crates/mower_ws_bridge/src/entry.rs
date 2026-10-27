//! One topic as the clients see it, whichever backend feeds it: the clients
//! subscribed to it, each with its own throttle, and the last message for a
//! latched topic's late joiners. A message is turned into its frame once and
//! fanned out to every client (rosbridge serialised per client).

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde_json::Value;

use crate::client::ClientHandle;

/// The frame of one message on `topic`: the text of a `serde_json::Value`,
/// so keys are sorted and an `f32` field prints as the `f64` it widens to,
/// as the app has always had it.
pub fn publish_frame(topic: &str, msg: &Value) -> Arc<str> {
    serde_json::json!({"op": "publish", "topic": topic, "msg": msg}).to_string().into()
}

/// Per-client subscription bookkeeping inside a topic entry.
struct ClientSub {
    handle: ClientHandle,
    throttle: Duration,
    last_sent: Option<Instant>,
    /// The newest message that arrived inside the throttle window. It goes
    /// out when the window closes: a throttle may thin a burst but must not
    /// end it on a stale message (the coverage planner publishes a
    /// DELETEALL and the new path ~5 ms apart; dropping the second left the
    /// app with an empty path layer).
    pending: Option<Arc<str>>,
    flush_scheduled: bool,
}

pub struct TopicEntry {
    pub topic: String,
    pub topic_type: String,
    pub latched: bool,
    inner: Mutex<TopicInner>,
}

#[derive(Default)]
struct TopicInner {
    subs: HashMap<u64, ClientSub>,
    last_payload: Option<Arc<str>>,
}

impl TopicEntry {
    pub fn new(topic: &str, topic_type: &str, latched: bool) -> Self {
        TopicEntry {
            topic: topic.to_string(),
            topic_type: topic_type.to_string(),
            latched,
            inner: Mutex::new(TopicInner::default()),
        }
    }

    /// Attach a client; a latched topic replays its last message at once.
    pub fn add_client(&self, handle: ClientHandle, throttle_ms: u64) {
        let mut inner = self.inner.lock().unwrap();
        let replay = if self.latched { inner.last_payload.clone() } else { None };
        inner.subs.insert(
            handle.id,
            ClientSub { handle: handle.clone(), throttle: Duration::from_millis(throttle_ms), last_sent: None, pending: None, flush_scheduled: false },
        );
        if let Some(payload) = replay {
            handle.send(payload);
        }
    }

    pub fn remove_client(&self, client_id: u64) {
        self.inner.lock().unwrap().subs.remove(&client_id);
    }

    pub fn fan_out(self: &Arc<Self>, msg: &Value) {
        let payload = publish_frame(&self.topic, msg);
        self.fan_out_payload(payload);
    }

    fn fan_out_payload(self: &Arc<Self>, payload: Arc<str>) {
        let now = Instant::now();
        let mut inner = self.inner.lock().unwrap();
        inner.last_payload = Some(payload.clone());
        inner.subs.retain(|_, sub| !sub.handle.is_closed());
        for (&id, sub) in inner.subs.iter_mut() {
            if let Some(last) = sub.last_sent {
                let since = now.duration_since(last);
                if !sub.throttle.is_zero() && since < sub.throttle {
                    sub.pending = Some(payload.clone());
                    if !sub.flush_scheduled {
                        sub.flush_scheduled = true;
                        let entry = Arc::clone(self);
                        let wait = sub.throttle - since;
                        tokio::spawn(async move {
                            tokio::time::sleep(wait).await;
                            entry.flush(id);
                        });
                    }
                    continue;
                }
            }
            sub.last_sent = Some(now);
            sub.pending = None;
            sub.handle.send(payload.clone());
        }
    }

    /// End of a client's throttle window: send what arrived inside it.
    fn flush(&self, client_id: u64) {
        let mut inner = self.inner.lock().unwrap();
        if let Some(sub) = inner.subs.get_mut(&client_id) {
            sub.flush_scheduled = false;
            if let Some(payload) = sub.pending.take() {
                sub.last_sent = Some(Instant::now());
                sub.handle.send(payload);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tokio::sync::mpsc;

    fn entry() -> Arc<TopicEntry> {
        Arc::new(TopicEntry::new("/t", "std_msgs/msg/String", true))
    }

    fn drain(rx: &mut mpsc::Receiver<Arc<str>>) -> Vec<String> {
        let mut out = vec![];
        while let Ok(p) = rx.try_recv() {
            out.push(p.to_string());
        }
        out
    }

    #[tokio::test(flavor = "current_thread", start_paused = true)]
    async fn a_throttled_burst_ends_on_its_newest_message() {
        let e = entry();
        let (tx, mut rx) = mpsc::channel(16);
        e.add_client(ClientHandle::for_test(1, tx), 100);
        e.fan_out_payload("clear".into());
        e.fan_out_payload("path".into());
        assert_eq!(drain(&mut rx), vec!["clear"], "the second message waits for the window");
        tokio::time::sleep(Duration::from_millis(120)).await;
        assert_eq!(drain(&mut rx), vec!["path"], "and goes out when it closes");
        tokio::time::sleep(Duration::from_millis(500)).await;
        assert!(drain(&mut rx).is_empty(), "nothing is sent twice");
    }

    #[tokio::test(flavor = "current_thread", start_paused = true)]
    async fn a_burst_inside_one_window_sends_only_the_newest() {
        let e = entry();
        let (tx, mut rx) = mpsc::channel(16);
        e.add_client(ClientHandle::for_test(1, tx), 100);
        for p in ["a", "b", "c", "d"] {
            e.fan_out_payload(p.into());
        }
        tokio::time::sleep(Duration::from_millis(150)).await;
        assert_eq!(drain(&mut rx), vec!["a", "d"]);
    }

    #[tokio::test(flavor = "current_thread", start_paused = true)]
    async fn unthrottled_clients_get_everything_at_once() {
        let e = entry();
        let (tx, mut rx) = mpsc::channel(16);
        e.add_client(ClientHandle::for_test(1, tx), 0);
        e.fan_out_payload("a".into());
        e.fan_out_payload("b".into());
        assert_eq!(drain(&mut rx), vec!["a", "b"]);
    }
}
