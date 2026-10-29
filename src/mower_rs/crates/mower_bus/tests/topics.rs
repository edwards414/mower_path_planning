//! Topics through the public API (README.md, "Topics" and "Erased access"):
//! delivery and queues, late joiners, types, the erased endpoints, scope
//! close, introspection, and use from std threads, tokio and other
//! executors.

use std::any::type_name;
use std::collections::BTreeMap;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::pin::Pin;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;
use std::task::{Context, Poll, Wake, Waker};
use std::thread;
use std::time::{Duration, Instant};

use futures::{FutureExt, Stream, StreamExt};
use mower_bus::{Bus, BusError, ErasedSubscriber, JsonPublisher, Publisher, PublisherChanges, Qos, RecvError, Subscriber};
use serde::{Deserialize, Serialize};
use serde_json::json;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Pose {
    x: f64,
    frame_id: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Twist {
    linear: f64,
    angular: f64,
}

/// The `n`-th message of publisher `from`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
struct Seq {
    from: u32,
    n: u64,
}

/// Laid out like an r2r message (nested, a header-ish string, arrays, an
/// optional field) to compare the erased JSON with serde's.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Path {
    frame_id: String,
    poses: Vec<Pose>,
    covariance: [f64; 3],
    goal: Option<Twist>,
}

/// Named like r2r's generated types, so that introspection maps it to a ROS
/// type name.
mod nav_msgs {
    pub mod msg {
        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct Odometry {
            pub x: f64,
        }
    }
}
use nav_msgs::msg::Odometry;

fn pose(x: f64) -> Pose {
    Pose { x, frame_id: "map".into() }
}

/// Everything queued for `sub`, without waiting.
fn queued<T: mower_bus::Message>(sub: &mut Subscriber<T>) -> Vec<Arc<T>> {
    std::iter::from_fn(|| sub.try_recv().ok()).collect()
}

fn xs(sub: &mut Subscriber<Pose>) -> Vec<f64> {
    queued(sub).iter().map(|p| p.x).collect()
}

const WAIT: Duration = Duration::from_secs(10);

#[test]
fn a_subscriber_gets_every_message_published_after_it_in_order() {
    let bus = Bus::new();
    let loc = bus.scope("localize");
    let nav = bus.scope("nav");
    let odom = loc.publisher::<Pose>("/odom", Qos::default()).unwrap();
    odom.publish(&pose(0.0)).unwrap(); // nobody listens yet: volatile, gone
    let mut a = nav.subscribe::<Pose>("/odom", Qos::default()).unwrap();
    let mut b = nav.subscribe::<Pose>("/odom", Qos::default()).unwrap();
    assert_eq!(a.try_recv().unwrap_err(), RecvError::Empty);

    odom.publish(&pose(1.0)).unwrap();
    odom.publish_owned(pose(2.0)).unwrap();
    let shared = Arc::new(pose(3.0));
    odom.publish_arc(shared.clone()).unwrap();

    let got_a = queued(&mut a);
    let got_b = queued(&mut b);
    assert_eq!(got_a.iter().map(|p| p.x).collect::<Vec<_>>(), [1.0, 2.0, 3.0]);
    // one Arc per message, shared by every reader; publish_arc passes it on as is
    for (x, y) in got_a.iter().zip(&got_b) {
        assert!(Arc::ptr_eq(x, y));
    }
    assert!(Arc::ptr_eq(&got_a[2], &shared));
    assert_eq!((a.dropped(), b.dropped()), (0, 0));
    assert_eq!((a.name(), odom.name()), ("/odom", "/odom"));
    assert_eq!((a.qos(), odom.qos()), (Qos::default(), Qos::default()));
}

#[test]
fn a_full_queue_drops_its_oldest_message_and_counts_it() {
    let bus = Bus::new();
    let s = bus.scope("m");
    let p = s.publisher::<Pose>("/p", Qos::default()).unwrap();
    let mut three = s.subscribe::<Pose>("/p", Qos::default().keep_last(3)).unwrap();
    let mut zero = s.subscribe::<Pose>("/p", Qos::default().keep_last(0)).unwrap();
    let mut deep = s.subscribe::<Pose>("/p", Qos::default().keep_last(100)).unwrap();
    for x in 1..=5 {
        p.publish(&pose(x as f64)).unwrap();
    }
    assert_eq!(xs(&mut three), [3.0, 4.0, 5.0]);
    assert_eq!(three.dropped(), 2);
    assert_eq!(xs(&mut zero), [5.0], "a depth of 0 counts as 1");
    assert_eq!(zero.dropped(), 4);
    // every reader has its own queue
    assert_eq!(xs(&mut deep), [1.0, 2.0, 3.0, 4.0, 5.0]);
    assert_eq!(deep.dropped(), 0);

    // latest() skips to the newest without counting the skipped as dropped
    for x in 6..=8 {
        p.publish(&pose(x as f64)).unwrap();
    }
    assert_eq!(deep.latest().map(|m| m.x), Some(8.0));
    assert_eq!(deep.try_recv().unwrap_err(), RecvError::Empty);
    assert_eq!(deep.latest(), None);
    assert_eq!(deep.dropped(), 0);
}

#[test]
fn a_late_transient_local_reader_gets_each_live_writers_history_in_publish_order() {
    let bus = Bus::new();
    let (a, b, c) = (bus.scope("a"), bus.scope("b"), bus.scope("c"));
    let readers = bus.scope("late");
    let pa = a.publisher::<Seq>("/info", Qos::latched().keep_last(2)).unwrap();
    let pb = b.publisher::<Seq>("/info", Qos::latched()).unwrap();
    let pc = c.publisher::<Seq>("/info", Qos::default()).unwrap(); // volatile: keeps nothing
    let seq = |from, n| Seq { from, n };
    for (p, m) in [(&pa, seq(0, 1)), (&pb, seq(1, 1)), (&pa, seq(0, 2))] {
        p.publish(&m).unwrap();
    }
    pc.publish(&seq(2, 1)).unwrap();
    pa.publish(&seq(0, 3)).unwrap();
    pb.publish(&seq(1, 2)).unwrap();
    // a keeps its last 2, b its last 1, c nothing
    let info = &bus.topics()[0];
    assert_eq!(info.retained, 3);

    let mut all = readers.subscribe::<Seq>("/info", Qos::latched().keep_last(10)).unwrap();
    let got: Vec<Seq> = queued(&mut all).iter().map(|m| **m).collect();
    assert_eq!(got, [seq(0, 2), seq(0, 3), seq(1, 2)], "oldest first, in publish order across writers");
    assert_eq!(all.dropped(), 0);

    // limited by the reader's own depth: the newest ones
    let mut two = readers.subscribe::<Seq>("/info", Qos::latched().keep_last(2)).unwrap();
    assert_eq!(queued(&mut two).iter().map(|m| **m).collect::<Vec<_>>(), [seq(0, 3), seq(1, 2)]);
    assert_eq!(two.dropped(), 0, "history that does not fit was never queued");
    let mut erased = readers.subscribe_erased("/info", Qos::latched()).unwrap();
    assert_eq!(erased.try_recv().unwrap().downcast::<Seq>().as_deref(), Some(&seq(1, 2)));

    // a volatile reader gets no history, transient-local writers or not
    let mut volatile = readers.subscribe::<Seq>("/info", Qos::default()).unwrap();
    assert_eq!(volatile.try_recv().unwrap_err(), RecvError::Empty);

    // after the history come new messages, for everybody
    pc.publish(&seq(2, 2)).unwrap();
    assert_eq!(queued(&mut all).iter().map(|m| **m).collect::<Vec<_>>(), [seq(2, 2)]);
    assert_eq!(queued(&mut volatile).iter().map(|m| **m).collect::<Vec<_>>(), [seq(2, 2)]);
}

#[test]
fn a_writers_history_goes_when_its_last_clone_drops() {
    let bus = Bus::new();
    let s = bus.scope("map");
    let late = bus.scope("late");
    let p = s.publisher::<Pose>("/map", Qos::latched()).unwrap();
    let q = p.clone();
    q.publish(&pose(1.0)).unwrap();
    p.publish(&pose(2.0)).unwrap(); // clones are one writer: one history of depth 1
    assert_eq!(bus.topics()[0].publishers, ["map"]);
    assert_eq!(bus.topics()[0].retained, 1);

    drop(p);
    let mut first = late.subscribe::<Pose>("/map", Qos::latched()).unwrap();
    assert_eq!(xs(&mut first), [2.0], "one clone left: the history stays");

    drop(q);
    assert_eq!(bus.topics()[0].retained, 0);
    assert!(bus.topics()[0].publishers.is_empty());
    let mut second = late.subscribe::<Pose>("/map", Qos::latched()).unwrap();
    assert_eq!(second.try_recv().unwrap_err(), RecvError::Empty);
}

#[test]
fn closing_a_writers_scope_drops_its_history_and_fails_its_publishes() {
    let bus = Bus::new();
    let m = bus.scope("map_manage");
    let other = bus.scope("app");
    let p = m.publisher::<Pose>("/map", Qos::latched()).unwrap();
    let clone = p.clone();
    let json = m.publisher_json("/map", Qos::latched()).unwrap();
    let mut reader = other.subscribe::<Pose>("/map", Qos::default()).unwrap();
    p.publish(&pose(1.0)).unwrap();
    json.publish(&json!({"x": 2.0, "frame_id": "map"})).unwrap();
    assert_eq!(bus.topics()[0].retained, 2);

    m.close();
    let closed = Err(BusError::Closed { scope: "map_manage".into() });
    assert_eq!(p.publish(&pose(3.0)), closed);
    assert_eq!(clone.publish_owned(pose(3.0)), closed);
    assert_eq!(p.publish_arc(Arc::new(pose(3.0))), closed);
    assert_eq!(json.publish(&json!({"x": 3.0, "frame_id": "map"})), closed);
    let info = &bus.topics()[0];
    assert_eq!((info.retained, info.published), (0, 2));
    assert!(info.publishers.is_empty());
    // other scopes' endpoints carry on: the reader keeps what it got, and
    // a late joiner finds no history
    assert_eq!(xs(&mut reader), [1.0, 2.0]);
    assert!(!reader.is_closed());
    let mut late = other.subscribe::<Pose>("/map", Qos::latched()).unwrap();
    assert_eq!(late.try_recv().unwrap_err(), RecvError::Empty);
    // a restarted module gets a fresh scope with the same name
    let restarted = bus.scope("map_manage");
    let p2 = restarted.publisher::<Pose>("/map", Qos::latched()).unwrap();
    p2.publish(&pose(4.0)).unwrap();
    assert_eq!(xs(&mut late), [4.0]);
    assert_eq!(p.publish(&pose(5.0)), closed, "the old instance stays closed");
}

#[tokio::test]
async fn closing_a_scope_ends_its_readers_at_once_and_discards_their_queues() {
    let bus = Bus::new();
    let m = bus.scope("nav");
    let other = bus.scope("localize");
    let p = other.publisher::<Pose>("/odom", Qos::default()).unwrap();
    let mut sub = m.subscribe::<Pose>("/odom", Qos::default()).unwrap();
    let mut erased = m.subscribe_erased("/odom", Qos::default()).unwrap();
    let mut survivor = other.subscribe::<Pose>("/odom", Qos::default()).unwrap();
    let mut waiting = m.subscribe::<Pose>("/goal", Qos::default()).unwrap();
    let parked = tokio::spawn(async move { waiting.next().await });
    p.publish(&pose(1.0)).unwrap();
    p.publish(&pose(2.0)).unwrap();
    tokio::time::sleep(Duration::from_millis(20)).await; // let the task park
    assert_eq!(p.subscription_count(), 3);

    m.close();
    assert_eq!(tokio::time::timeout(WAIT, parked).await.unwrap().unwrap(), None, "a parked task wakes up to the end");
    assert!(sub.is_closed());
    assert_eq!(sub.try_recv().unwrap_err(), RecvError::Closed, "queued messages are discarded");
    assert_eq!(sub.next().await, None);
    assert_eq!(sub.recv().await, None);
    let t = Instant::now();
    assert_eq!(sub.recv_timeout(WAIT).unwrap_err(), RecvError::Closed);
    assert!(t.elapsed() < Duration::from_secs(1));
    assert_eq!(sub.latest(), None);
    assert_eq!(erased.try_recv().unwrap_err(), RecvError::Closed);
    assert!(erased.next().await.is_none());
    // closed readers no longer count, and later messages do not reach them
    assert_eq!(p.subscription_count(), 1);
    p.publish(&pose(3.0)).unwrap();
    assert_eq!(sub.try_recv().unwrap_err(), RecvError::Closed);
    assert_eq!(xs(&mut survivor), [1.0, 2.0, 3.0]);
    assert!(!survivor.is_closed());
}

#[test]
fn a_thread_blocked_in_recv_timeout_wakes_when_its_scope_closes() {
    let bus = Bus::new();
    let m = bus.scope("imu");
    let mut sub = m.subscribe::<Pose>("/imu", Qos::default()).unwrap();
    let blocked = thread::spawn(move || {
        let t = Instant::now();
        (sub.recv_timeout(WAIT), t.elapsed())
    });
    thread::sleep(Duration::from_millis(50));
    m.close();
    let (result, waited) = blocked.join().unwrap();
    assert_eq!(result.unwrap_err(), RecvError::Closed);
    assert!(waited < Duration::from_secs(5), "woken by the close, not the timeout: {waited:?}");
}

#[test]
fn a_closed_scope_creates_no_endpoints() {
    let bus = Bus::new();
    let m = bus.scope("gone");
    m.close();
    let closed = BusError::Closed { scope: "gone".into() };
    assert_eq!(m.publisher::<Pose>("/a", Qos::default()).unwrap_err(), closed);
    assert_eq!(m.subscribe::<Pose>("/b", Qos::default()).unwrap_err(), closed);
    assert_eq!(m.subscribe_erased("/c", Qos::default()).unwrap_err(), closed);
    assert_eq!(m.publisher_json("/d", Qos::default()).unwrap_err(), closed);
    assert!(bus.topics().is_empty(), "and leaves no channel behind: {:?}", bus.topics());
}

#[test]
fn endpoints_need_absolute_names() {
    let bus = Bus::new();
    let s = bus.scope("m");
    let invalid = |e: BusError| matches!(e, BusError::InvalidName { .. });
    assert!(invalid(s.publisher::<Pose>("odom", Qos::default()).unwrap_err()));
    assert!(invalid(s.subscribe::<Pose>("~/odom", Qos::default()).unwrap_err()));
    assert!(invalid(s.subscribe_erased("/odom/", Qos::default()).unwrap_err()));
    assert!(invalid(s.publisher_json("", Qos::default()).unwrap_err()));
    assert!(bus.topics().is_empty());
}

#[test]
fn the_first_typed_endpoint_fixes_the_type_for_the_life_of_the_bus() {
    let bus = Bus::new();
    let s = bus.scope("m");
    let (pose_t, twist_t) = (type_name::<Pose>(), type_name::<Twist>());
    let mismatch = |name: &str, existing, requested| BusError::TypeMismatch { name: name.into(), existing, requested };

    // publisher first, then a subscriber of another type
    let p = s.publisher::<Pose>("/a", Qos::default()).unwrap();
    assert_eq!(s.subscribe::<Twist>("/a", Qos::default()).unwrap_err(), mismatch("/a", pose_t, twist_t));
    assert_eq!(s.publisher::<Twist>("/a", Qos::default()).unwrap_err(), mismatch("/a", pose_t, twist_t));
    // subscriber first, then a publisher of another type
    let sub = s.subscribe::<Twist>("/b", Qos::default()).unwrap();
    assert_eq!(s.publisher::<Pose>("/b", Qos::default()).unwrap_err(), mismatch("/b", twist_t, pose_t));
    // the same type is fine, and the failed attempts changed nothing
    let mut same = s.subscribe::<Pose>("/a", Qos::default()).unwrap();
    p.publish(&pose(1.0)).unwrap();
    assert_eq!(xs(&mut same), [1.0]);

    // still fixed once every endpoint is gone
    drop((p, sub, same));
    assert_eq!(s.subscribe::<Twist>("/a", Qos::default()).unwrap_err(), mismatch("/a", pose_t, twist_t));
    assert_eq!(s.publisher::<Pose>("/b", Qos::default()).unwrap_err(), mismatch("/b", twist_t, pose_t));

    // erased endpoints fix nothing
    let erased = s.subscribe_erased("/c", Qos::default()).unwrap();
    let json = s.publisher_json("/c", Qos::default()).unwrap();
    assert_eq!(erased.type_name(), None);
    let _twist = s.publisher::<Twist>("/c", Qos::default()).unwrap();
    assert_eq!(erased.type_name(), Some(twist_t));
    assert_eq!(s.subscribe::<Pose>("/c", Qos::default()).unwrap_err(), mismatch("/c", twist_t, pose_t));
    drop(json);
}

#[tokio::test]
async fn an_erased_reader_created_before_the_type_is_known_attaches_when_it_is() {
    let bus = Bus::new();
    let bridge = bus.scope("ws_bridge");
    let loc = bus.scope("localize");
    let mut early = bridge.subscribe_erased("/odom", Qos::default()).unwrap();
    let mut parked = bridge.subscribe_erased("/odom", Qos::latched()).unwrap();
    assert_eq!((early.name(), early.type_name()), ("/odom", None));
    assert_eq!(early.try_recv().unwrap_err(), RecvError::Empty);
    assert_eq!(bus.topics()[0].type_name, None);
    let task = tokio::spawn(async move {
        let msg = parked.recv().await.unwrap();
        (msg.type_name(), msg.to_json().unwrap())
    });
    tokio::time::sleep(Duration::from_millis(20)).await;

    let p = loc.publisher::<Pose>("/odom", Qos::default()).unwrap();
    assert_eq!(early.type_name(), Some(type_name::<Pose>()));
    p.publish(&pose(1.5)).unwrap();
    let msg = early.try_recv().unwrap();
    assert_eq!(msg.type_name(), type_name::<Pose>());
    assert_eq!(msg.to_json().unwrap(), json!({"x": 1.5, "frame_id": "map"}));
    let (type_seen, json_seen) = tokio::time::timeout(WAIT, task).await.unwrap().unwrap();
    assert_eq!(type_seen, type_name::<Pose>());
    assert_eq!(json_seen, json!({"x": 1.5, "frame_id": "map"}));
}

#[test]
fn a_json_publisher_needs_the_type_then_decodes_with_its_codec() {
    let bus = Bus::new();
    let bridge = bus.scope("ws_bridge");
    let base = bus.scope("base");
    // creating it works before the type is known; publishing does not
    let teleop: JsonPublisher = bridge.publisher_json("/cmd_vel", Qos::default()).unwrap();
    let good = json!({"linear": 0.2, "angular": -0.5});
    assert_eq!(teleop.publish(&good), Err(BusError::UnknownName { name: "/cmd_vel".into() }));

    let mut sub = base.subscribe::<Twist>("/cmd_vel", Qos::default()).unwrap();
    let mut tap: ErasedSubscriber = bridge.subscribe_erased("/cmd_vel", Qos::default()).unwrap();
    for bad in [json!({"linear": "fast", "angular": 0.0}), json!({"linear": 0.2}), json!(null)] {
        match teleop.publish(&bad) {
            Err(BusError::Json { name, error }) => assert!(name == "/cmd_vel" && !error.is_empty(), "{name}: {error}"),
            other => panic!("{bad} gave {other:?}"),
        }
    }
    teleop.clone().publish(&good).unwrap();
    assert_eq!(*sub.try_recv().unwrap(), Twist { linear: 0.2, angular: -0.5 });
    assert_eq!(tap.try_recv().unwrap().to_json().unwrap(), good, "the JSON layout is serde's both ways");
    assert_eq!(sub.try_recv().unwrap_err(), RecvError::Empty, "failed publishes deliver nothing");

    let info = &bus.topics()[0];
    assert_eq!(info.published, 1, "and count for nothing");
    assert_eq!(info.publishers, ["ws_bridge"], "a JsonPublisher and its clones are one writer");
    assert_eq!(teleop.name(), "/cmd_vel");
}

#[test]
fn erased_messages_serialize_like_serde_and_downcast_without_a_copy() {
    let bus = Bus::new();
    let s = bus.scope("coverage");
    let tap = bus.scope("recorder");
    let p: Publisher<Path> = s.publisher("/plan", Qos::default()).unwrap();
    let mut erased = tap.subscribe_erased("/plan", Qos::default()).unwrap();
    let mut typed = tap.subscribe::<Path>("/plan", Qos::default()).unwrap();
    let path = Arc::new(Path {
        frame_id: "map".into(),
        poses: vec![pose(1.0), Pose { x: -2.25, frame_id: "odom".into() }],
        covariance: [0.1, 0.0, 1e-9],
        goal: None,
    });
    p.publish_arc(path.clone()).unwrap();

    let msg = erased.try_recv().unwrap();
    assert_eq!(msg.type_name(), type_name::<Path>());
    assert_eq!(msg.to_json().unwrap(), serde_json::to_value(&*path).unwrap());
    let back: Arc<Path> = msg.downcast::<Path>().unwrap();
    assert!(Arc::ptr_eq(&back, &path), "the published Arc itself");
    assert!(Arc::ptr_eq(&typed.try_recv().unwrap(), &path), "typed and erased readers share it");
    assert!(msg.downcast::<Pose>().is_none());
}

#[test]
fn subscription_count_counts_live_typed_and_erased_readers() {
    let bus = Bus::new();
    let loc = bus.scope("localize");
    let bridge = bus.scope("ws_bridge");
    let p = loc.publisher::<Pose>("/odom", Qos::default()).unwrap();
    assert_eq!(p.subscription_count(), 0);
    let a = loc.subscribe::<Pose>("/odom", Qos::default()).unwrap();
    let b = bridge.subscribe::<Pose>("/odom", Qos::sensor_data()).unwrap();
    let e = bridge.subscribe_erased("/odom", Qos::default()).unwrap();
    let _elsewhere = loc.subscribe::<Pose>("/other", Qos::default()).unwrap();
    assert_eq!(p.subscription_count(), 3);
    assert_eq!(p.clone().subscription_count(), 3);
    drop(a);
    assert_eq!(p.subscription_count(), 2);
    bridge.close();
    assert_eq!(p.subscription_count(), 0);
    drop((b, e)); // dropping closed readers is fine
    assert_eq!(p.subscription_count(), 0);
}

#[test]
fn bus_topics_reports_types_endpoints_history_and_traffic() {
    let bus = Bus::new();
    let loc = bus.scope("localize");
    let nav = bus.scope("nav");
    let bridge = bus.scope("ws_bridge");
    let odom = loc.publisher::<Odometry>("/odometry/global", Qos::latched().keep_last(2)).unwrap();
    let _again = odom.clone();
    let _json = bridge.publisher_json("/odometry/global", Qos::default()).unwrap();
    let _sub = nav.subscribe::<Odometry>("/odometry/global", Qos::default()).unwrap();
    let _tap = bridge.subscribe_erased("/odometry/global", Qos::default()).unwrap();
    let _untyped = bridge.subscribe_erased("/app/only", Qos::default()).unwrap();
    let _local = nav.publisher::<Pose>("/nav/pose", Qos::default()).unwrap();
    let before = Instant::now();
    for x in 0..3 {
        odom.publish(&Odometry { x: x as f64 }).unwrap();
    }
    let after = Instant::now();

    let topics = bus.topics();
    let names: Vec<&str> = topics.iter().map(|t| t.name.as_str()).collect();
    assert_eq!(names, ["/app/only", "/nav/pose", "/odometry/global"], "sorted by name");
    let by_name: BTreeMap<&str, _> = topics.iter().map(|t| (t.name.as_str(), t)).collect();

    let o = by_name["/odometry/global"];
    assert_eq!(o.type_name, Some(type_name::<Odometry>()));
    assert_eq!(o.ros_type.as_deref(), Some("nav_msgs/msg/Odometry"));
    assert_eq!(o.publishers, ["localize", "ws_bridge"]);
    assert_eq!(o.publisher_qos, [Qos::latched().keep_last(2), Qos::default()], "one per writer, clones included");
    assert_eq!(o.subscribers, ["nav", "ws_bridge"]);
    assert_eq!(o.subscriber_qos, [Qos::default(), Qos::default()]);
    assert_eq!((o.retained, o.published), (2, 3));
    let last = o.last_published.unwrap();
    assert!(before <= last && last <= after);

    let untyped = by_name["/app/only"];
    assert_eq!((untyped.type_name, untyped.ros_type.as_deref()), (None, None));
    assert!(untyped.publishers.is_empty() && untyped.publisher_qos.is_empty());
    assert_eq!(untyped.subscribers, ["ws_bridge"]);
    assert_eq!((untyped.published, untyped.last_published), (0, None));

    let local = by_name["/nav/pose"];
    assert_eq!((local.type_name, local.ros_type.as_deref()), (Some(type_name::<Pose>()), None), "not an r2r type");

    // One name alone, as ws_bridge looks at it on each subscribe.
    for t in &topics {
        assert_eq!(bus.topic(&t.name).as_ref(), Some(t), "{}", t.name);
    }
    assert_eq!(bus.topic("/never/seen"), None);

    // the type outlives the endpoints
    drop((odom, _again, _json, _sub, _tap));
    let o = bus.topic("/odometry/global").unwrap();
    assert_eq!(o.type_name, Some(type_name::<Odometry>()));
    assert!(o.publishers.is_empty() && o.subscribers.is_empty());
    assert!(o.publisher_qos.is_empty() && o.subscriber_qos.is_empty());
    assert_eq!((o.retained, o.published), (0, 3));
}

#[test]
fn publisher_count_counts_each_live_writer_once() {
    let bus = Bus::new();
    let base = bus.scope("base");
    let tap = bus.scope("recorder");
    let cmd = base.subscribe::<Twist>("/cmd_vel", Qos::default()).unwrap();
    let erased = tap.subscribe_erased("/cmd_vel", Qos::default()).unwrap();
    let count = || {
        let n = cmd.publisher_count();
        assert_eq!(erased.publisher_count(), n, "typed and erased readers agree");
        n
    };
    let teleop = bus.scope("teleop");
    let _reader = teleop.subscribe::<Twist>("/cmd_vel", Qos::default()).unwrap();
    let _elsewhere = teleop.publisher::<Twist>("/nav_cmd_vel", Qos::default()).unwrap();
    assert_eq!(count(), 0, "readers and other topics' writers do not count");

    let p = teleop.publisher::<Twist>("/cmd_vel", Qos::default()).unwrap();
    let clone = p.clone();
    assert_eq!(count(), 1, "a publisher and its clones are one writer");
    let app = bus.scope("ws_bridge");
    let json = app.publisher_json("/cmd_vel", Qos::default()).unwrap();
    let _json_clone = json.clone();
    assert_eq!(count(), 2, "a JsonPublisher is a writer");
    let nav = bus.scope("nav");
    let q = nav.publisher::<Twist>("/cmd_vel", Qos::default()).unwrap();
    assert_eq!(count(), 3);

    drop(p);
    assert_eq!(count(), 3, "a clone is left");
    drop(clone);
    assert_eq!(count(), 2, "the last clone takes the writer with it");
    app.close();
    assert_eq!(count(), 1, "a closed scope's writers stop counting, whoever still holds them");
    drop(q);
    assert_eq!(count(), 0);
}

#[tokio::test]
async fn publisher_changes_report_writers_coming_and_going_until_the_readers_scope_closes() {
    let bus = Bus::new();
    let bridge = bus.scope("ros_bridge");
    let sub = bridge.subscribe::<Pose>("/map", Qos::latched()).unwrap();
    let mut changes = sub.publisher_changes();
    assert_eq!(sub.publisher_count(), 0);
    assert_eq!(changes.next().now_or_never(), None, "nothing has changed yet");

    let map = bus.scope("map_manage");
    let p = map.publisher::<Pose>("/map", Qos::latched()).unwrap();
    assert_eq!(changes.next().await, Some(1));
    assert_eq!(changes.next().now_or_never(), None, "one item per change");

    // several changes between two looks give one item; a clone is no change
    let app = bus.scope("ws_bridge");
    let json = app.publisher_json("/map", Qos::default()).unwrap();
    let _clone = p.clone();
    drop(json);
    assert_eq!(changes.next().await, Some(1));
    assert_eq!(changes.next().now_or_never(), None);

    // a module restart: the writer goes with its scope and a new one comes;
    // the count ends where it was, the item still comes
    map.close();
    let restarted = bus.scope("map_manage");
    let _p2 = restarted.publisher::<Pose>("/map", Qos::latched()).unwrap();
    assert_eq!(changes.next().await, Some(1));

    // a task parked on a stream is woken by the change
    let mut parked = sub.publisher_changes();
    let task = tokio::spawn(async move { parked.next().await });
    tokio::time::sleep(Duration::from_millis(20)).await;
    restarted.close();
    assert_eq!(tokio::time::timeout(WAIT, task).await.unwrap().unwrap(), Some(0));

    // the reader's scope closing ends its streams, parked or not, also
    // after the subscriber itself is gone
    let mut parked = sub.publisher_changes();
    let task = tokio::spawn(async move { parked.next().await });
    drop(sub);
    tokio::time::sleep(Duration::from_millis(20)).await;
    bridge.close();
    assert_eq!(tokio::time::timeout(WAIT, task).await.unwrap().unwrap(), None);
    assert_eq!(changes.next().await, None, "the end wins over a change not yet reported");
    assert_eq!(changes.next().await, None);
}

/// The mirror for readers, which the ROS bridge's `from_ros` follows with its
/// DDS reader: a writer's `subscription_changes()` reports readers, typed and
/// erased, coming and going, and ends when the writer's scope closes, also
/// once the publisher itself is gone (the bridge keeps the stream of a
/// publisher it dropped at once).
#[tokio::test]
async fn subscription_changes_report_readers_coming_and_going_until_the_writers_scope_closes() {
    let bus = Bus::new();
    let bridge = bus.scope("ros_bridge");
    let p = bridge.publisher::<Pose>("/map", Qos::latched()).unwrap();
    let mut changes = p.subscription_changes();
    assert_eq!(p.subscription_count(), 0);
    assert_eq!(changes.next().now_or_never(), None, "nothing has changed yet");

    let nav = bus.scope("nav");
    let sub = nav.subscribe::<Pose>("/map", Qos::latched()).unwrap();
    assert_eq!(changes.next().await, Some(1));
    assert_eq!(changes.next().now_or_never(), None, "one item per change");

    // several changes between two looks give one item; a writer is no change
    let app = bus.scope("ws_bridge");
    let erased = app.subscribe_erased("/map", Qos::default()).unwrap();
    let _writer = app.publisher_json("/map", Qos::default()).unwrap();
    assert_eq!(changes.next().await, Some(2));
    drop(erased);
    let _other = app.subscribe::<Pose>("/map", Qos::default()).unwrap();
    assert_eq!(changes.next().await, Some(2));
    assert_eq!(changes.next().now_or_never(), None);

    // a module restart: the reader goes with its scope and a new one comes;
    // the count ends where it was, the item still comes
    nav.close();
    drop(sub);
    let restarted = bus.scope("nav");
    let _sub = restarted.subscribe::<Pose>("/map", Qos::latched()).unwrap();
    assert_eq!(changes.next().await, Some(2));

    // a task parked on a stream is woken by the change
    let mut parked = p.subscription_changes();
    let task = tokio::spawn(async move { parked.next().await });
    tokio::time::sleep(Duration::from_millis(20)).await;
    restarted.close();
    assert_eq!(tokio::time::timeout(WAIT, task).await.unwrap().unwrap(), Some(1));

    // the writer's scope closing ends its streams, parked or not, also after
    // the publisher itself is gone
    let mut parked = p.subscription_changes();
    let task = tokio::spawn(async move { parked.next().await });
    drop(p);
    tokio::time::sleep(Duration::from_millis(20)).await;
    bridge.close();
    assert_eq!(tokio::time::timeout(WAIT, task).await.unwrap().unwrap(), None);
    assert_eq!(changes.next().await, None, "the end wins over a change not yet reported");
    assert_eq!(changes.next().await, None);
}

/// The ROS bridge's loop: forward a latched topic's messages and notice its
/// last writer going, in one `select!` over the reader and its changes.
#[tokio::test]
async fn a_bridge_loop_selects_over_messages_and_writer_changes() {
    let bus = Bus::new();
    let bridge = bus.scope("ros_bridge");
    let mut sub = bridge.subscribe_erased("/robot/info", Qos::latched()).unwrap();
    let mut writers = sub.publisher_changes();
    let (tx, mut events) = tokio::sync::mpsc::unbounded_channel();
    let task = tokio::spawn(async move {
        loop {
            tokio::select! {
                Some(msg) = sub.next() => tx.send(format!("message {}", msg.to_json().unwrap()["x"])).unwrap(),
                Some(count) = writers.next() => tx.send(format!("writers {count}")).unwrap(),
                else => break,
            }
        }
    });

    let robot = bus.scope("robot_status");
    let p = robot.publisher::<Pose>("/robot/info", Qos::latched()).unwrap();
    assert_eq!(events.recv().await.unwrap(), "writers 1");
    p.publish(&pose(1.5)).unwrap();
    assert_eq!(events.recv().await.unwrap(), "message 1.5");
    robot.close();
    assert_eq!(events.recv().await.unwrap(), "writers 0");
    bridge.close();
    tokio::time::timeout(WAIT, task).await.unwrap().unwrap();
    assert_eq!(events.recv().await, None, "both streams ended with the scope");
}

static GRIDS_LIVE: AtomicUsize = AtomicUsize::new(0);

/// A message that counts its live copies (an OccupancyGrid, as far as
/// memory goes).
#[derive(Serialize, Deserialize)]
struct Grid(u32);

impl Grid {
    fn new(n: u32) -> Self {
        GRIDS_LIVE.fetch_add(1, Ordering::SeqCst);
        Grid(n)
    }
}

impl Clone for Grid {
    fn clone(&self) -> Self {
        Grid::new(self.0)
    }
}

impl Drop for Grid {
    fn drop(&mut self) {
        GRIDS_LIVE.fetch_sub(1, Ordering::SeqCst);
    }
}

/// Review 2026-10-26: a `publisher_changes()` stream "can outlive the
/// subscriber" and held its inbox, queue included, so dropping the
/// subscriber freed nothing it had queued while the stream lived (the ROS
/// bridge keeps such streams for latched topics: 160 kB grids).
#[test]
fn dropping_a_subscriber_frees_its_queue_while_its_publisher_watch_lives() {
    let bus = Bus::new();
    let s = bus.scope("bridge");
    let map = bus.scope("map");
    let p = map.publisher::<Grid>("/map_grid", Qos::default()).unwrap();
    let sub = s.subscribe::<Grid>("/map_grid", Qos::default()).unwrap();
    for n in 0..3 {
        p.publish_owned(Grid::new(n)).unwrap();
    }
    assert_eq!(GRIDS_LIVE.load(Ordering::SeqCst), 3);
    let mut changes = sub.publisher_changes();
    drop(sub);
    assert_eq!(GRIDS_LIVE.load(Ordering::SeqCst), 0, "the dropped subscriber's queued messages are still alive");

    // the same for an erased reader
    let erased = s.subscribe_erased("/map_grid", Qos::default()).unwrap();
    p.publish_owned(Grid::new(3)).unwrap();
    let erased_changes = erased.publisher_changes();
    drop(erased);
    assert_eq!(GRIDS_LIVE.load(Ordering::SeqCst), 0);
    drop(erased_changes);

    // and the stream itself works on until the reader's scope closes
    map.close();
    assert_eq!(changes.next().now_or_never(), Some(Some(0)));
    s.close();
    assert_eq!(changes.next().now_or_never(), Some(None));
}

/// The ROS bridge keeps the stream in a struct next to the reader, so the
/// type has a name; code that took it as an `impl Stream` keeps compiling.
#[test]
fn publisher_changes_has_a_name_and_still_works_as_an_impl_stream() {
    struct LatchedTopic {
        reader: ErasedSubscriber,
        writers: PublisherChanges,
    }
    fn first_change(stream: impl Stream<Item = usize> + Send + Sync + Unpin + 'static) -> Option<usize> {
        futures::executor::block_on(stream.take(1).collect::<Vec<_>>()).pop()
    }

    let bus = Bus::new();
    let bridge = bus.scope("ros_bridge");
    let sub = bridge.subscribe::<Pose>("/map", Qos::latched()).unwrap();
    let typed: PublisherChanges = sub.publisher_changes();
    let reader = bridge.subscribe_erased("/map", Qos::latched()).unwrap();
    let writers = reader.publisher_changes();
    let mut topic = LatchedTopic { reader, writers };

    let _p = bus.scope("map_manage").publisher::<Pose>("/map", Qos::latched()).unwrap();
    assert_eq!(first_change(typed), Some(1));
    assert_eq!(futures::executor::block_on(topic.writers.next()), Some(1));
    assert_eq!(topic.reader.publisher_count(), 1);
    assert!(format!("{:?}", topic.writers).contains("/map"));
}

// ---- user code that panics while a reader closes or a publish wakes ----

/// A waker that panics when woken.
struct Exploding;

impl Wake for Exploding {
    fn wake(self: Arc<Self>) {
        panic!("waker exploded");
    }
}

/// A waker that counts its wake-ups.
#[derive(Default)]
struct Count(AtomicUsize);

impl Wake for Count {
    fn wake(self: Arc<Self>) {
        self.0.fetch_add(1, Ordering::SeqCst);
    }
}

fn counter() -> (Arc<Count>, Waker) {
    let count = Arc::new(Count::default());
    (count.clone(), Waker::from(count))
}

fn exploding() -> Waker {
    Waker::from(Arc::new(Exploding))
}

/// Poll `stream` once with `waker`: it must be waiting.
fn park<S: futures::Stream + Unpin>(stream: &mut S, waker: &Waker) {
    assert!(Pin::new(stream).poll_next(&mut Context::from_waker(waker)).is_pending());
}

/// Review 2026-10-26, second pass: a reader's close hook was
/// `inbox.close(); channel.close_reader(&inbox)`, and a panic in the first
/// half (its task's waker, or drop glue of what it discarded) skipped the
/// second: the closed reader stayed registered on its channel, and a task
/// parked on its `publisher_changes()` was never woken.
#[test]
fn a_panic_inside_a_readers_close_still_unregisters_it_and_wakes_its_watch() {
    let bus = Bus::new();
    let m = bus.scope("ros_bridge");
    let p = bus.scope("map_manage").publisher::<u32>("/map_grid", Qos::default()).unwrap();
    let mut sub = m.subscribe::<u32>("/map_grid", Qos::default()).unwrap();
    park(&mut sub, &exploding());
    let mut changes = sub.publisher_changes();
    let (counted, watcher) = counter();
    park(&mut changes, &watcher);

    let panicked = catch_unwind(AssertUnwindSafe(|| m.close())).is_err();
    assert!(panicked);
    let (registered, woken) = (p.subscription_count(), counted.0.load(Ordering::SeqCst));
    assert_eq!(
        (registered, woken),
        (0, 1),
        "after the panic: closed reader still registered on its channel ({registered}), parked publisher_changes() woken {woken} times"
    );
    assert_eq!(Pin::new(&mut changes).poll_next(&mut Context::from_waker(&watcher)), Poll::Ready(None));
}

/// A message whose drop glue panics.
#[derive(Clone, Serialize, Deserialize)]
struct Bomb;

impl Drop for Bomb {
    fn drop(&mut self) {
        if !thread::panicking() {
            panic!("bomb");
        }
    }
}

#[test]
fn drop_glue_that_panics_in_a_closing_readers_queue_skips_nothing_else() {
    let bus = Bus::new();
    let m = bus.scope("recorder");
    let bombs = bus.scope("map_manage").publisher::<Bomb>("/bombs", Qos::default()).unwrap();
    let sub = m.subscribe::<Bomb>("/bombs", Qos::default()).unwrap();
    let mut changes = sub.publisher_changes();
    let (watched, watcher) = counter();
    park(&mut changes, &watcher);
    bombs.publish_owned(Bomb).unwrap(); // queued, unread: the close frees it

    assert!(catch_unwind(AssertUnwindSafe(|| m.close())).is_err(), "the bomb's panic goes on");
    assert_eq!(bombs.subscription_count(), 0, "the reader is unregistered all the same");
    assert_eq!(watched.0.load(Ordering::SeqCst), 1, "and its watch woken");
    assert!(sub.is_closed());
}

#[test]
fn a_watch_whose_waker_panics_keeps_no_other_waiter_of_a_closing_reader_asleep() {
    let bus = Bus::new();
    let m = bus.scope("ros_bridge");
    let _writer = bus.scope("map_manage").publisher::<u32>("/map", Qos::latched()).unwrap();
    let mut sub = m.subscribe::<u32>("/map", Qos::latched()).unwrap();
    let mut first = sub.publisher_changes();
    let mut second = sub.publisher_changes();
    park(&mut first, &exploding());
    let (watched, watcher) = counter();
    park(&mut second, &watcher);
    let (woken, waker) = counter();
    park(&mut sub, &waker);

    assert!(catch_unwind(AssertUnwindSafe(|| m.close())).is_err());
    assert_eq!(watched.0.load(Ordering::SeqCst), 1, "the other watch woken");
    assert_eq!(woken.0.load(Ordering::SeqCst), 1, "the reader's own task woken");
    assert_eq!(Pin::new(&mut second).poll_next(&mut Context::from_waker(&watcher)), Poll::Ready(None));
}

#[test]
fn a_watch_whose_waker_panics_keeps_no_other_watch_asleep_when_a_writer_closes() {
    let bus = Bus::new();
    let bridge = bus.scope("ros_bridge");
    let map = bus.scope("map_manage");
    let p = map.publisher::<Pose>("/map", Qos::latched()).unwrap();
    let retained = Arc::new(pose(1.0));
    p.publish_arc(retained.clone()).unwrap();
    let (a, b) = (bridge.subscribe::<Pose>("/map", Qos::default()).unwrap(), bridge.subscribe::<Pose>("/map", Qos::default()).unwrap());
    let (mut first, mut second) = (a.publisher_changes(), b.publisher_changes());
    park(&mut first, &exploding());
    let (watched, watcher) = counter();
    park(&mut second, &watcher);

    assert!(catch_unwind(AssertUnwindSafe(|| map.close())).is_err());
    assert_eq!(watched.0.load(Ordering::SeqCst), 1, "the other watch woken");
    assert_eq!(Arc::strong_count(&retained), 1, "the writer's history freed");
    assert_eq!(Pin::new(&mut second).poll_next(&mut Context::from_waker(&watcher)), Poll::Ready(Some(0)));
}

#[test]
fn a_reader_whose_waker_panics_keeps_no_other_reader_asleep_on_a_publish() {
    let bus = Bus::new();
    let m = bus.scope("nav");
    let p = bus.scope("localize").publisher::<Pose>("/odom", Qos::default()).unwrap();
    let mut broken = m.subscribe::<Pose>("/odom", Qos::default()).unwrap();
    let mut fine = m.subscribe::<Pose>("/odom", Qos::default()).unwrap();
    park(&mut broken, &exploding());
    let (woken, waker) = counter();
    park(&mut fine, &waker);

    assert!(catch_unwind(AssertUnwindSafe(|| p.publish(&pose(1.0)))).is_err(), "the panic goes on to the publisher");
    assert_eq!(woken.0.load(Ordering::SeqCst), 1, "the other reader woken");
    assert_eq!((xs(&mut broken), xs(&mut fine)), (vec![1.0], vec![1.0]), "delivered to both");
    p.publish(&pose(2.0)).unwrap(); // nobody parked now: an ordinary publish
}

#[test]
fn publisher_changes_need_no_runtime() {
    let bus = Bus::new();
    let base = bus.scope("base");
    let sub = base.subscribe::<Twist>("/wheel_override", Qos::default()).unwrap();
    let mut changes = sub.publisher_changes();
    let teleop = bus.scope("teleop");
    let creator = thread::spawn(move || {
        thread::sleep(Duration::from_millis(20));
        teleop.publisher::<Twist>("/wheel_override", Qos::default()).unwrap()
    });
    assert_eq!(futures::executor::block_on(changes.next()), Some(1));
    drop(creator.join().unwrap());
    assert_eq!(futures::executor::block_on(changes.next()), Some(0));
}

#[test]
fn recv_timeout_waits_for_the_timeout_and_returns_as_soon_as_a_message_arrives() {
    let bus = Bus::new();
    let s = bus.scope("imu");
    let p = s.publisher::<Pose>("/imu", Qos::default()).unwrap();
    let mut sub = s.subscribe::<Pose>("/imu", Qos::default()).unwrap();

    let t = Instant::now();
    assert_eq!(sub.recv_timeout(Duration::from_millis(50)).unwrap_err(), RecvError::Timeout);
    let waited = t.elapsed();
    assert!(waited >= Duration::from_millis(50) && waited < Duration::from_secs(2), "{waited:?}");

    // a zero timeout is a poll
    assert_eq!(sub.recv_timeout(Duration::ZERO).unwrap_err(), RecvError::Timeout);
    p.publish(&pose(1.0)).unwrap();
    assert_eq!(sub.recv_timeout(Duration::ZERO).unwrap().x, 1.0);

    // a message from another thread ends the wait early
    let later = p.clone();
    let publisher = thread::spawn(move || {
        thread::sleep(Duration::from_millis(30));
        later.publish(&pose(2.0)).unwrap();
    });
    let t = Instant::now();
    assert_eq!(sub.recv_timeout(WAIT).unwrap().x, 2.0);
    let waited = t.elapsed();
    assert!(waited >= Duration::from_millis(25) && waited < Duration::from_secs(5), "{waited:?}");
    publisher.join().unwrap();

    // a timeout too long to represent waits as long as it takes
    let later = p.clone();
    let publisher = thread::spawn(move || {
        thread::sleep(Duration::from_millis(10));
        later.publish(&pose(3.0)).unwrap();
    });
    assert_eq!(sub.recv_timeout(Duration::MAX).unwrap().x, 3.0);
    publisher.join().unwrap();
}

#[tokio::test]
async fn subscribers_are_streams_for_stream_ext_and_tokio_select() {
    let bus = Bus::new();
    let s = bus.scope("agent");
    let pa = s.publisher::<Seq>("/a", Qos::default()).unwrap();
    let pb = s.publisher::<Seq>("/b", Qos::default()).unwrap();
    let mut a = s.subscribe::<Seq>("/a", Qos::default().keep_last(100)).unwrap();
    let mut b = s.subscribe::<Seq>("/b", Qos::default()).unwrap();

    // StreamExt combinators, borrowing the subscriber
    for n in 0..6 {
        pa.publish(&Seq { from: 0, n }).unwrap();
    }
    let evens: Vec<u64> =
        a.by_ref().map(|m| m.n).filter(|n| futures::future::ready(n % 2 == 0)).take(3).collect().await;
    assert_eq!(evens, [0, 2, 4]);
    assert_eq!(a.next().await.map(|m| m.n), Some(5), "the rest is still queued");

    // select! over two topics and a timer: the branches that lose are
    // cancelled, and nothing is lost with them
    pa.publish(&Seq { from: 0, n: 10 }).unwrap();
    pb.publish(&Seq { from: 1, n: 20 }).unwrap();
    pa.publish(&Seq { from: 0, n: 11 }).unwrap();
    let mut seen = Vec::new();
    loop {
        tokio::select! {
            Some(m) = a.recv() => seen.push(m.n),
            Some(m) = b.next() => seen.push(m.n),
            _ = tokio::time::sleep(Duration::from_millis(50)) => break,
        }
    }
    seen.sort_unstable();
    assert_eq!(seen, [10, 11, 20]);

    // a publish from another thread wakes a select! that waits
    let from_thread = pb.clone();
    let publisher = thread::spawn(move || {
        thread::sleep(Duration::from_millis(20));
        from_thread.publish(&Seq { from: 1, n: 21 }).unwrap();
    });
    tokio::select! {
        Some(m) = b.recv() => assert_eq!(m.n, 21),
        _ = tokio::time::sleep(WAIT) => panic!("no wake-up"),
    }
    publisher.join().unwrap();
}

#[test]
fn publishing_needs_no_runtime_and_any_executor_can_read() {
    let bus = Bus::new();
    let s = bus.scope("gps");
    let p = s.publisher::<Seq>("/fix", Qos::default()).unwrap();
    let mut sub = s.subscribe::<Seq>("/fix", Qos::default()).unwrap();
    let mut erased = s.subscribe_erased("/fix", Qos::default()).unwrap();
    let publisher = thread::spawn(move || {
        for n in 0..5 {
            p.publish(&Seq { from: 0, n }).unwrap();
            thread::sleep(Duration::from_millis(2));
        }
    });
    let got: Vec<u64> = futures::executor::block_on(sub.by_ref().take(5).map(|m| m.n).collect());
    assert_eq!(got, [0, 1, 2, 3, 4]);
    let got: Vec<u64> = futures::executor::block_on(async {
        let mut out = Vec::new();
        while out.len() < 5 {
            out.push(erased.recv().await.unwrap().downcast::<Seq>().unwrap().n);
        }
        out
    });
    assert_eq!(got, [0, 1, 2, 3, 4]);
    publisher.join().unwrap();
}

/// Several publisher threads, async readers on a multi-thread runtime and
/// blocking readers on std threads, with a depth that holds everything:
/// nothing is lost, each publisher's order holds, and every reader sees the
/// same interleaving (the channel's lock orders publishes for everyone).
#[test]
fn many_publisher_threads_and_mixed_readers_lose_nothing_with_a_large_depth() {
    const PUBLISHERS: u32 = 4;
    const EACH: u64 = 5_000;
    const TOTAL: usize = PUBLISHERS as usize * EACH as usize;
    let bus = Bus::new();
    let s = bus.scope("stress");
    let qos = Qos::default().keep_last(TOTAL);
    let rt = tokio::runtime::Builder::new_multi_thread().worker_threads(3).enable_time().build().unwrap();

    let typed_async: Vec<_> = (0..2)
        .map(|_| {
            let mut sub = s.subscribe::<Seq>("/stress", qos).unwrap();
            rt.spawn(async move {
                let mut got = Vec::with_capacity(TOTAL);
                while got.len() < TOTAL {
                    got.push(*sub.recv().await.unwrap());
                }
                (got, sub.dropped())
            })
        })
        .collect();
    let erased_async = {
        let mut sub = s.subscribe_erased("/stress", qos).unwrap();
        rt.spawn(async move {
            let mut got = Vec::with_capacity(TOTAL);
            while let Some(msg) = sub.next().await {
                got.push(*msg.downcast::<Seq>().unwrap());
                if got.len() == TOTAL {
                    break;
                }
            }
            (got, sub.dropped())
        })
    };
    let blocking: Vec<_> = (0..2)
        .map(|_| {
            let mut sub = s.subscribe::<Seq>("/stress", qos).unwrap();
            thread::spawn(move || {
                let mut got = Vec::with_capacity(TOTAL);
                while got.len() < TOTAL {
                    got.push(*sub.recv_timeout(WAIT).expect("a message within the wait"));
                }
                (got, sub.dropped())
            })
        })
        .collect();

    let publishers: Vec<_> = (0..PUBLISHERS)
        .map(|from| {
            let p = s.publisher::<Seq>("/stress", Qos::default()).unwrap();
            thread::spawn(move || {
                for n in 0..EACH {
                    p.publish(&Seq { from, n }).unwrap();
                    if n % 512 == 0 {
                        thread::yield_now();
                    }
                }
            })
        })
        .collect();
    for p in publishers {
        p.join().unwrap();
    }

    let mut results = Vec::new();
    for task in typed_async.into_iter().chain([erased_async]) {
        let finished = rt.block_on(async { tokio::time::timeout(WAIT, task).await });
        results.push(finished.expect("the async reader finishes").unwrap());
    }
    for thread in blocking {
        results.push(thread.join().unwrap());
    }
    assert_eq!(results.len(), 5);
    let reference = results[0].0.clone();
    for (got, dropped) in &results {
        assert_eq!((got.len(), *dropped), (TOTAL, 0));
        let mut next = vec![0u64; PUBLISHERS as usize];
        for m in got {
            assert_eq!(m.n, next[m.from as usize], "publisher {}'s order", m.from);
            next[m.from as usize] += 1;
        }
        assert_eq!(*got, reference, "every reader sees the same interleaving");
    }
    assert_eq!(bus.topics()[0].published, TOTAL as u64);
}

#[test]
fn endpoints_are_send_sync_and_unpin() {
    fn check<T: Send + Sync + Unpin>() {}
    check::<Publisher<Pose>>();
    check::<Subscriber<Pose>>();
    check::<ErasedSubscriber>();
    check::<JsonPublisher>();
    check::<mower_bus::ErasedMsg>();
    check::<PublisherChanges>();
}

/// Review 2026-10-26 (a stress test that passed: evidence for "nothing
/// found"). Readers of every kind race publishes from two writers, drops and
/// scope closes: every reader's stream ends, the watches end after it, and
/// no reader sees a message twice in a row.
#[test]
fn readers_racing_publishes_drops_and_scope_closes_always_end_and_keep_order() {
    let rt = tokio::runtime::Builder::new_multi_thread().worker_threads(4).enable_time().build().unwrap();
    let bus = Bus::new();
    for round in 0..300u64 {
        let readers_scope = bus.scope("readers");
        let writer_scope = bus.scope("writer");
        let p = writer_scope.publisher::<u64>("/t", Qos::default().keep_last(4)).unwrap();
        let latched = writer_scope.publisher::<u64>("/t", Qos::latched().keep_last(3)).unwrap();
        let mut tasks = Vec::new();
        for r in 0..4 {
            let qos = if r % 2 == 0 { Qos::default().keep_last(1 + r) } else { Qos::latched().keep_last(1 + r) };
            let mut sub = readers_scope.subscribe::<u64>("/t", qos).unwrap();
            let mut changes = sub.publisher_changes();
            tasks.push(rt.spawn(async move {
                let mut last = None;
                loop {
                    tokio::select! {
                        m = sub.next() => match m {
                            Some(v) => {
                                // per writer, values only go up; across the two writers they interleave
                                let v = *v;
                                if let Some(prev) = last { assert!(v != prev, "repeated {v}"); }
                                last = Some(v);
                            }
                            None => break,
                        },
                        Some(_) = changes.next() => {}
                    }
                }
                // after the stream ends, the watch ends too
                assert_eq!(tokio::time::timeout(WAIT, changes.next()).await.expect("watch hung"), None);
            }));
        }
        let blocking = {
            let mut sub = readers_scope.subscribe::<u64>("/t", Qos::default()).unwrap();
            thread::spawn(move || loop {
                match sub.recv_timeout(WAIT) {
                    Ok(_) => {}
                    Err(RecvError::Closed) => break,
                    Err(e) => panic!("blocking reader: {e}"),
                }
            })
        };
        let publisher = {
            let (p, latched) = (p.clone(), latched.clone());
            thread::spawn(move || {
                for n in 0..200u64 {
                    let v = round * 1_000_000 + n * 2;
                    let _ = p.publish(&v);
                    let _ = latched.publish(&(v + 1));
                }
            })
        };
        if round % 2 == 0 {
            readers_scope.close();
            publisher.join().unwrap();
        } else {
            publisher.join().unwrap();
            drop((p, latched));
            readers_scope.close();
        }
        writer_scope.close();
        rt.block_on(async {
            for t in tasks {
                tokio::time::timeout(WAIT, t).await.expect("an async reader never ended").unwrap();
            }
        });
        blocking.join().unwrap();
    }
}
