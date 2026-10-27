//! One topic as the clients see it, whichever backend feeds it: the clients
//! subscribed to it, each with its own throttle, and the last message for a
//! latched topic's late joiners. A message is turned into its frame once and
//! fanned out to every client (rosbridge serialised per client).
//!
//! The r2r backend fixes `latched` at the first subscribe and frames every
//! message (`fan_out`). The bus backend updates `latched` on each subscribe,
//! frames a message only when a client wants it (`offer`), clears the cache
//! when the topic's last writer goes, and watches the client count to drop
//! the entry after its last client (bus_hub.rs).

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use mower_bus::{Clock, Mono};
use serde_json::Value;
use tokio::sync::Notify;

use crate::client::ClientHandle;

/// The frame of one message on `topic`: the text of a `serde_json::Value`,
/// so keys are sorted and an `f32` field prints as the `f64` it widens to,
/// as the app has always had it.
pub fn publish_frame(topic: &str, msg: &Value) -> Arc<str> {
    serde_json::json!({"op": "publish", "topic": topic, "msg": msg}).to_string().into()
}

/// A message's frame, made only when somebody wants it.
pub type Frame = Box<dyn FnOnce() -> Option<Arc<str>> + Send>;

/// Per-client subscription bookkeeping inside a topic entry.
struct ClientSub {
    handle: ClientHandle,
    throttle: Duration,
    last_sent: Option<Mono>,
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
    /// Whether a client that comes later gets the last message at once.
    latched: AtomicBool,
    /// What the throttle measures and waits on.
    clock: Clock,
    /// Woken whenever the client count changes, if somebody watches it.
    clients_changed: Option<Arc<Notify>>,
    inner: Mutex<TopicInner>,
}

#[derive(Default)]
struct TopicInner {
    subs: HashMap<u64, ClientSub>,
    last_payload: Option<Arc<str>>,
    /// The last message, offered while the entry had no client, not framed
    /// yet: a late joiner of a latched entry gets it framed (bus backend).
    held: Option<Frame>,
}

impl TopicEntry {
    /// The r2r backend's entry: `latched` as the writers were at the first
    /// subscribe, the throttle on the system clock.
    pub fn new(topic: &str, topic_type: &str, latched: bool) -> Self {
        Self::with_clock(topic, topic_type, latched, Clock::system(), None)
    }

    /// An entry whose throttle runs on `clock`, telling `clients_changed`
    /// whenever a client comes or goes.
    pub fn with_clock(topic: &str, topic_type: &str, latched: bool, clock: Clock, clients_changed: Option<Arc<Notify>>) -> Self {
        TopicEntry {
            topic: topic.to_string(),
            topic_type: topic_type.to_string(),
            latched: AtomicBool::new(latched),
            clock,
            clients_changed,
            inner: Mutex::new(TopicInner::default()),
        }
    }

    pub fn set_latched(&self, latched: bool) {
        self.latched.store(latched, Ordering::Relaxed);
    }

    pub fn is_latched(&self) -> bool {
        self.latched.load(Ordering::Relaxed)
    }

    fn changed(&self) {
        if let Some(notify) = &self.clients_changed {
            notify.notify_one();
        }
    }

    pub fn client_count(&self) -> usize {
        self.inner.lock().unwrap().subs.len()
    }

    /// Attach a client; a latched topic replays its last message at once.
    pub fn add_client(&self, handle: ClientHandle, throttle_ms: u64) {
        let mut inner = self.inner.lock().unwrap();
        let latched = self.is_latched();
        if latched && inner.last_payload.is_none() {
            if let Some(frame) = inner.held.take() {
                inner.last_payload = frame();
            }
        }
        let replay = if latched { inner.last_payload.clone() } else { None };
        inner.subs.insert(
            handle.id,
            ClientSub { handle: handle.clone(), throttle: Duration::from_millis(throttle_ms), last_sent: None, pending: None, flush_scheduled: false },
        );
        drop(inner);
        self.changed();
        if let Some(payload) = replay {
            handle.send(payload);
        }
    }

    pub fn remove_client(&self, client_id: u64) {
        let removed = self.inner.lock().unwrap().subs.remove(&client_id);
        if removed.is_some() {
            self.changed();
        }
    }

    /// Frame `msg` and send it to every client (r2r backend).
    pub fn fan_out(self: &Arc<Self>, msg: &Value) {
        let payload = publish_frame(&self.topic, msg);
        self.fan_out_payload(payload);
    }

    /// A message for the clients (bus backend), framed only if one wants it:
    /// at once while the entry has clients; with none, kept as it is, and
    /// framed for a late joiner of a latched entry (`add_client`).
    pub fn offer(self: &Arc<Self>, frame: Frame) {
        let mut inner = self.inner.lock().unwrap();
        let before = inner.subs.len();
        inner.subs.retain(|_, sub| !sub.handle.is_closed());
        let pruned = inner.subs.len() != before;
        let wanted = if inner.subs.is_empty() {
            inner.last_payload = None;
            inner.held = Some(frame);
            None
        } else {
            inner.held = None;
            Some(frame)
        };
        drop(inner);
        if pruned {
            self.changed();
        }
        // Framed with no lock held: a map layer takes a few milliseconds.
        if let Some(payload) = wanted.and_then(|frame| frame()) {
            self.fan_out_payload(payload);
        }
    }

    /// Whether a later client could get a message replayed.
    #[cfg(test)]
    pub fn has_cache(&self) -> bool {
        let inner = self.inner.lock().unwrap();
        inner.last_payload.is_some() || inner.held.is_some()
    }

    /// Forget the last message (bus backend: the topic's last writer went).
    pub fn clear_cache(&self) {
        let mut inner = self.inner.lock().unwrap();
        inner.last_payload = None;
        inner.held = None;
    }

    fn fan_out_payload(self: &Arc<Self>, payload: Arc<str>) {
        let now = self.clock.mono();
        let mut inner = self.inner.lock().unwrap();
        inner.last_payload = Some(payload.clone());
        let before = inner.subs.len();
        inner.subs.retain(|_, sub| !sub.handle.is_closed());
        let pruned = inner.subs.len() != before;
        for (&id, sub) in inner.subs.iter_mut() {
            if let Some(last) = sub.last_sent {
                let since = now.saturating_duration_since(last);
                if !sub.throttle.is_zero() && since < sub.throttle {
                    sub.pending = Some(payload.clone());
                    if !sub.flush_scheduled {
                        sub.flush_scheduled = true;
                        let entry = Arc::clone(self);
                        let window_end = last + sub.throttle;
                        tokio::spawn(async move {
                            entry.clock.sleep_until(window_end).await;
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
        drop(inner);
        if pruned {
            self.changed();
        }
    }

    /// End of a client's throttle window: send what arrived inside it.
    fn flush(&self, client_id: u64) {
        let mut inner = self.inner.lock().unwrap();
        if let Some(sub) = inner.subs.get_mut(&client_id) {
            sub.flush_scheduled = false;
            if let Some(payload) = sub.pending.take() {
                sub.last_sent = Some(self.clock.mono());
                sub.handle.send(payload);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use futures::FutureExt;
    use mower_bus::{ManualClock, Time};
    use tokio::sync::mpsc;

    fn entry() -> (Arc<TopicEntry>, ManualClock) {
        let (clock, manual) = Clock::manual(Time::from_ns(1_793_000_000_000_000_000));
        (Arc::new(TopicEntry::with_clock("/t", "std_msgs/msg/String", true, clock, None)), manual)
    }

    fn drain(rx: &mut mpsc::Receiver<Arc<str>>) -> Vec<String> {
        let mut out = vec![];
        while let Ok(p) = rx.try_recv() {
            out.push(p.to_string());
        }
        out
    }

    /// Let the spawned flush tasks run.
    async fn settle() {
        for _ in 0..10 {
            tokio::task::yield_now().await;
        }
    }

    #[tokio::test(flavor = "current_thread")]
    async fn a_throttled_burst_ends_on_its_newest_message() {
        let (e, manual) = entry();
        let (tx, mut rx) = mpsc::channel(16);
        e.add_client(ClientHandle::for_test(1, tx), 100);
        e.fan_out_payload("clear".into());
        e.fan_out_payload("path".into());
        assert_eq!(drain(&mut rx), vec!["clear"], "the second message waits for the window");
        settle().await;
        manual.advance(Duration::from_millis(99));
        settle().await;
        assert!(drain(&mut rx).is_empty(), "the window is still open");
        manual.advance(Duration::from_millis(21));
        settle().await;
        assert_eq!(drain(&mut rx), vec!["path"], "and goes out when it closes");
        manual.advance(Duration::from_millis(500));
        settle().await;
        assert!(drain(&mut rx).is_empty(), "nothing is sent twice");
    }

    #[tokio::test(flavor = "current_thread")]
    async fn a_burst_inside_one_window_sends_only_the_newest() {
        let (e, manual) = entry();
        let (tx, mut rx) = mpsc::channel(16);
        e.add_client(ClientHandle::for_test(1, tx), 100);
        for p in ["a", "b", "c", "d"] {
            e.fan_out_payload(p.into());
        }
        settle().await;
        manual.advance(Duration::from_millis(150));
        settle().await;
        assert_eq!(drain(&mut rx), vec!["a", "d"]);
    }

    #[tokio::test(flavor = "current_thread")]
    async fn unthrottled_clients_get_everything_at_once() {
        let (e, _manual) = entry();
        let (tx, mut rx) = mpsc::channel(16);
        e.add_client(ClientHandle::for_test(1, tx), 0);
        e.fan_out_payload("a".into());
        e.fan_out_payload("b".into());
        assert_eq!(drain(&mut rx), vec!["a", "b"]);
    }

    /// A message that comes after the window has closed goes out at once and
    /// opens the next window; a re-subscribe starts the client afresh.
    #[tokio::test(flavor = "current_thread")]
    async fn a_window_opens_at_each_message_sent_and_a_resubscribe_resets_it() {
        let (e, manual) = entry();
        let (tx, mut rx) = mpsc::channel(16);
        let handle = ClientHandle::for_test(1, tx);
        e.add_client(handle.clone(), 100);
        e.fan_out_payload("a".into());
        manual.advance(Duration::from_millis(100));
        e.fan_out_payload("b".into());
        assert_eq!(drain(&mut rx), vec!["a", "b"], "a full window later: at once");
        manual.advance(Duration::from_millis(30));
        e.fan_out_payload("c".into());
        assert!(drain(&mut rx).is_empty());
        e.add_client(handle, 100);
        assert_eq!(drain(&mut rx), vec!["c"], "the latched replay of the re-subscribe");
        e.fan_out_payload("d".into());
        assert_eq!(drain(&mut rx), vec!["d"], "a fresh window");
    }

    /// The bus backend: a message nobody wants is not framed, and a late
    /// joiner of a latched entry gets it framed; a volatile one does not.
    #[test]
    fn a_message_offered_to_nobody_is_framed_only_for_a_latched_late_joiner() {
        let (e, _manual) = entry();
        let framed = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let frame = |text: &'static str| -> Frame {
            let framed = framed.clone();
            Box::new(move || {
                framed.fetch_add(1, Ordering::SeqCst);
                Some(text.into())
            })
        };
        e.offer(frame("one"));
        e.offer(frame("two"));
        assert_eq!(framed.load(Ordering::SeqCst), 0, "nobody to send it to");
        e.set_latched(false);
        let (tx, mut volatile) = mpsc::channel(16);
        e.add_client(ClientHandle::for_test(1, tx), 0);
        assert!(drain(&mut volatile).is_empty(), "no replay for a volatile topic");
        e.remove_client(1);
        e.set_latched(true);
        let (tx, mut late) = mpsc::channel(16);
        e.add_client(ClientHandle::for_test(2, tx), 0);
        assert_eq!(drain(&mut late), vec!["two"], "the newest one, framed now");
        assert_eq!(framed.load(Ordering::SeqCst), 1);
        // with a client, framed at once; once cleared, nothing to replay
        e.offer(frame("three"));
        assert_eq!(drain(&mut late), vec!["three"]);
        e.clear_cache();
        let (tx, mut later) = mpsc::channel(16);
        e.add_client(ClientHandle::for_test(3, tx), 0);
        assert!(drain(&mut later).is_empty(), "the cache went with the writers");
    }

    #[tokio::test(flavor = "current_thread")]
    async fn the_client_count_is_told_when_clients_come_and_go() {
        let (clock, _manual) = Clock::manual(Time::from_ns(0));
        let notify = Arc::new(Notify::new());
        let e = Arc::new(TopicEntry::with_clock("/t", "std_msgs/msg/String", false, clock, Some(notify.clone())));
        let told = || notify.notified().now_or_never().is_some();
        let (tx, rx) = mpsc::channel(16);
        e.add_client(ClientHandle::for_test(1, tx), 0);
        assert!(told() && e.client_count() == 1);
        e.remove_client(1);
        assert!(told() && e.client_count() == 0);
        e.remove_client(1);
        assert!(!told(), "nothing changed");
        // a client whose connection went is pruned at the next message
        let (tx2, _rx2) = mpsc::channel(16);
        e.add_client(ClientHandle::for_test(2, tx2), 0);
        let _ = told();
        drop(rx);
        let (tx3, rx3) = mpsc::channel(16);
        e.add_client(ClientHandle::for_test(3, tx3), 0);
        let _ = told();
        drop(rx3);
        e.offer(Box::new(|| Some("x".into())));
        assert!(told() && e.client_count() == 1, "the closed client went");
    }
}
