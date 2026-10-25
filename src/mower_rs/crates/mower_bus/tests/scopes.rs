//! Scopes through the public API (README.md, "Model"): what closing one
//! ends and when it has, concurrent and panicking closes, `closed()`, child
//! scopes, and the wall timers tied to a scope.

use std::future::Future;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::pin::Pin;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::task::{Context, Poll, Wake, Waker};
use std::thread;
use std::time::{Duration, Instant};

use futures::StreamExt;
use mower_bus::{Bus, BusError, Clock, Mono, Qos, RecvError, Scope, Subscriber, Time, Timer};
use serde::{Deserialize, Serialize};

const WAIT: Duration = Duration::from_secs(10);

fn closed(scope: &str) -> BusError {
    BusError::Closed { scope: scope.into() }
}

fn ms(n: u64) -> Duration {
    Duration::from_millis(n)
}

fn at_ms(n: u64) -> Mono {
    Mono::from_ns(n * 1_000_000)
}

fn poll_once<F: Future + Unpin>(f: &mut F) -> Poll<F::Output> {
    Pin::new(f).poll(&mut Context::from_waker(futures::task::noop_waker_ref()))
}

fn poll_with<F: Future + Unpin>(f: &mut F, waker: &Waker) -> Poll<F::Output> {
    Pin::new(f).poll(&mut Context::from_waker(waker))
}

/// A waker that counts its wake-ups; its clones share the count.
#[derive(Default)]
struct Count(AtomicUsize);

impl Wake for Count {
    fn wake(self: Arc<Self>) {
        self.wake_by_ref();
    }

    fn wake_by_ref(self: &Arc<Self>) {
        self.0.fetch_add(1, Ordering::SeqCst);
    }
}

impl Count {
    fn new() -> Arc<Self> {
        Arc::default()
    }

    fn waker(self: &Arc<Self>) -> Waker {
        Waker::from(self.clone())
    }

    fn get(&self) -> usize {
        self.0.load(Ordering::SeqCst)
    }
}

/// Spin until another thread gets `cond` true, failing after `WAIT`.
fn wait_until(what: &str, cond: impl Fn() -> bool) {
    let start = Instant::now();
    while !cond() {
        assert!(start.elapsed() < WAIT, "timed out waiting until {what}");
        thread::yield_now();
    }
}

// ---- holding a close inside one of its hooks ----

/// Where a close stops for as long as a test needs: freeing a `Gated`
/// message (drop glue, so user code) waits until the gate opens.
#[derive(Default)]
struct Gate {
    /// (a drop waits in it, it is open)
    state: Mutex<(bool, bool)>,
    changed: Condvar,
}

impl Gate {
    fn wait_in(&self) {
        let mut state = self.state.lock().unwrap();
        state.0 = true;
        self.changed.notify_all();
        while !state.1 {
            state = self.changed.wait(state).unwrap();
        }
    }

    fn wait_until_entered(&self) {
        let state = self.state.lock().unwrap();
        let (state, timeout) = self.changed.wait_timeout_while(state, WAIT, |s| !s.0).unwrap();
        assert!(state.0 && !timeout.timed_out(), "the close never reached the gate");
    }

    fn open(&self) {
        self.state.lock().unwrap().1 = true;
        self.changed.notify_all();
    }
}

#[derive(Clone, Default, Serialize, Deserialize)]
struct Gated {
    #[serde(skip)]
    gate: Option<Arc<Gate>>,
}

impl Drop for Gated {
    fn drop(&mut self) {
        if let Some(gate) = self.gate.take() {
            gate.wait_in();
        }
    }
}

/// Give `scope` a reader that holds the only reference to a `Gated`
/// message, so that its close stops in that reader's hook until the
/// returned gate opens. The reader must be kept.
fn hold_the_close_of(scope: &Scope) -> (Arc<Gate>, Subscriber<Gated>) {
    let gate = Arc::new(Gate::default());
    let reader = scope.subscribe::<Gated>("/gated", Qos::default()).unwrap();
    let feeder = scope.bus().scope("feeder");
    let p = feeder.publisher::<Gated>("/gated", Qos::default()).unwrap();
    p.publish_owned(Gated { gate: Some(gate.clone()) }).unwrap();
    (gate, reader)
}

/// Close `scope` on a thread of its own and return once that close is
/// stopped in its gated hook, i.e. while the scope is closing.
fn start_closing(scope: &Scope, gate: &Gate) -> thread::JoinHandle<()> {
    let closer = {
        let scope = scope.clone();
        thread::spawn(move || scope.close())
    };
    gate.wait_until_entered();
    closer
}

// ---- what a close ends, and when ----

#[test]
fn closing_a_scope_ends_its_endpoints_timers_and_children_before_it_returns() {
    let bus = Bus::new();
    let (clock, mc) = Clock::manual(Time::default());
    let m = bus.scope("nav");
    let other = bus.scope("other");
    let out = m.publisher::<u32>("/out", Qos::default()).unwrap();
    let mut sub = m.subscribe::<u32>("/in", Qos::default()).unwrap();
    let _srv = m.service::<u32, u32>("/srv").unwrap();
    let mut timer = m.wall_timer(&clock, ms(50)).unwrap();
    let child = m.child("nav_planner");
    let child_out = child.publisher::<u32>("/plan", Qos::default()).unwrap();
    let feed = other.publisher::<u32>("/in", Qos::default()).unwrap();
    feed.publish(&1).unwrap();

    m.close();
    assert!(m.is_closed() && child.is_closed());
    assert_eq!(out.publish(&1), Err(closed("nav")));
    assert_eq!(sub.try_recv(), Err(RecvError::Closed));
    assert_eq!(child_out.publish(&1), Err(closed("nav_planner")));
    mc.advance(ms(50));
    assert_eq!(poll_once(&mut Box::pin(timer.tick())), Poll::Ready(Err(closed("nav"))));
    assert!(bus.scope("nav").service::<u32, u32>("/srv").is_ok(), "the restart can serve the name again");
    assert!(poll_once(&mut Box::pin(m.closed())).is_ready());
    assert!(poll_once(&mut Box::pin(child.closed())).is_ready());
}

/// Review 2026-10-26: `close()` set the flag, then ran the hooks, and a
/// second `close()` (or `is_closed()`) that came in between returned at
/// once although the scope's later endpoints were still live.
#[test]
fn after_close_returns_the_scopes_endpoints_are_closed_even_with_a_concurrent_close() {
    /// A message whose drop takes a while when `slow`: what freeing a big
    /// message (or any drop glue) costs inside a close hook.
    #[derive(Clone, Serialize, Deserialize)]
    struct SlowDrop {
        slow: bool,
    }

    impl Drop for SlowDrop {
        fn drop(&mut self) {
            if self.slow {
                thread::sleep(Duration::from_millis(300));
            }
        }
    }

    let bus = Bus::new();
    let m = bus.scope("m");
    let other = bus.scope("other");
    // hook 1: a reader whose queued message is slow to free
    let _sub = m.subscribe::<SlowDrop>("/slow", Qos::default()).unwrap();
    // hooks 2 and 3: a writer and a server of the same scope
    let out = m.publisher::<u32>("/out", Qos::default()).unwrap();
    let _srv = m.service::<u32, u32>("/srv").unwrap();
    let feed = other.publisher::<SlowDrop>("/slow", Qos::default()).unwrap();
    let mut watch = other.subscribe::<u32>("/out", Qos::default()).unwrap();
    feed.publish_owned(SlowDrop { slow: true }).unwrap();

    let first = {
        let m = m.clone();
        thread::spawn(move || m.close())
    };
    wait_until("the first close has started", || m.is_closed());
    m.close(); // the first close is still in hook 1

    let publish = out.publish(&7);
    let delivered = watch.try_recv().is_ok();
    let restarted = bus.scope("m").service::<u32, u32>("/srv").map(drop);
    first.join().unwrap();
    assert_eq!(publish, Err(closed("m")), "close() returned, yet the scope's publisher published");
    assert!(!delivered, "a message from a closed scope was delivered");
    assert_eq!(restarted, Ok(()), "close() returned, yet the scope still holds its service name");
}

#[test]
fn a_second_close_waits_for_the_first_to_finish() {
    let bus = Bus::new();
    let m = bus.scope("m");
    let (gate, _reader) = hold_the_close_of(&m);
    let first = start_closing(&m, &gate);
    let returned = Arc::new(AtomicUsize::new(0));
    let second = {
        let (m, returned) = (m.clone(), returned.clone());
        thread::spawn(move || {
            m.close();
            returned.fetch_add(1, Ordering::SeqCst);
        })
    };
    thread::sleep(ms(30));
    assert_eq!(returned.load(Ordering::SeqCst), 0, "the second close returned while the first was in a hook");
    gate.open();
    second.join().unwrap();
    first.join().unwrap();
    assert_eq!(returned.load(Ordering::SeqCst), 1);
    m.close(); // and later ones return at once
}

#[test]
fn nothing_can_be_made_on_a_closing_scope() {
    let bus = Bus::new();
    let (clock, _mc) = Clock::manual(Time::default());
    let m = bus.scope("m");
    let (gate, _reader) = hold_the_close_of(&m);
    let closer = start_closing(&m, &gate);

    assert!(m.is_closed(), "closing counts as closed");
    assert_eq!(m.publisher::<u32>("/a", Qos::default()).unwrap_err(), closed("m"));
    assert_eq!(m.subscribe::<u32>("/b", Qos::default()).unwrap_err(), closed("m"));
    assert_eq!(m.subscribe_erased("/c", Qos::default()).unwrap_err(), closed("m"));
    assert_eq!(m.publisher_json("/d", Qos::default()).unwrap_err(), closed("m"));
    assert_eq!(m.service::<u32, u32>("/e").unwrap_err(), closed("m"));
    assert_eq!(m.client::<u32, u32>("/f").unwrap_err(), closed("m"));
    assert_eq!(m.wall_timer(&clock, ms(10)).unwrap_err(), closed("m"));
    assert!(m.child("late").is_closed(), "a child of a closing scope is created closed");
    let mut done = Box::pin(m.closed());
    assert!(poll_once(&mut done).is_pending(), "closed() waits for the close to finish");

    gate.open();
    closer.join().unwrap();
    assert!(poll_once(&mut done).is_ready());
}

// ---- panics inside a close ----

/// A message whose drop glue panics when `armed`.
#[derive(Clone, Serialize, Deserialize)]
struct PanicOnDrop {
    armed: bool,
}

impl Drop for PanicOnDrop {
    fn drop(&mut self) {
        if self.armed && !thread::panicking() {
            panic!("drop glue panicked");
        }
    }
}

/// Review 2026-10-26: the first hook frees a queued message whose drop
/// panics. The panic left `close()` with the flag set and the hook list
/// emptied, so the later hooks never ran: the scope reported closed while
/// its publisher kept publishing and its service name stayed taken.
#[test]
fn a_panic_in_one_close_hook_still_closes_the_rest_of_the_scope() {
    let bus = Bus::new();
    let m = bus.scope("m");
    let other = bus.scope("other");
    let _sub = m.subscribe::<PanicOnDrop>("/p", Qos::default()).unwrap();
    let out = m.publisher::<u32>("/out", Qos::default()).unwrap();
    let _srv = m.service::<u32, u32>("/srv").unwrap();
    other.publisher::<PanicOnDrop>("/p", Qos::default()).unwrap().publish_owned(PanicOnDrop { armed: true }).unwrap();

    let panicked = catch_unwind(AssertUnwindSafe(|| m.close())).is_err();
    assert!(panicked, "the drop glue panicked inside close");
    assert!(m.is_closed());
    assert_eq!(out.publish(&1), Err(closed("m")), "the scope says closed, its publisher still works");
    assert_eq!(
        bus.scope("m").service::<u32, u32>("/srv").map(drop),
        Ok(()),
        "the scope says closed, its service name is still taken"
    );
}

/// A message whose drop glue panics with its number.
#[derive(Clone, Serialize, Deserialize)]
struct Bomb(u32);

impl Drop for Bomb {
    fn drop(&mut self) {
        if !thread::panicking() {
            panic!("bomb {}", self.0);
        }
    }
}

#[test]
fn a_close_that_panics_finishes_and_wakes_its_waiters_then_re_raises_the_first_panic() {
    let bus = Bus::new();
    let (clock, _mc) = Clock::manual(Time::default());
    let m = bus.scope("m");
    let feeder = bus.scope("feeder");
    let (_one, _two) = (m.subscribe::<Bomb>("/one", Qos::default()).unwrap(), m.subscribe::<Bomb>("/two", Qos::default()).unwrap());
    let child = m.child("child");
    let child_out = child.publisher::<u32>("/child", Qos::default()).unwrap();
    feeder.publisher::<Bomb>("/one", Qos::default()).unwrap().publish_owned(Bomb(1)).unwrap();
    feeder.publisher::<Bomb>("/two", Qos::default()).unwrap().publish_owned(Bomb(2)).unwrap();
    let mut timer = m.wall_timer(&clock, ms(100)).unwrap();
    let ticking = Count::new();
    let mut tick = Box::pin(timer.tick());
    assert!(poll_with(&mut tick, &ticking.waker()).is_pending());
    let waiter = {
        let m = m.clone();
        thread::spawn(move || futures::executor::block_on(m.closed()))
    };
    thread::sleep(ms(20)); // let it wait

    let panic = catch_unwind(AssertUnwindSafe(|| m.close())).expect_err("both bombs went off inside close");
    assert_eq!(panic.downcast_ref::<String>().map(String::as_str), Some("bomb 1"), "the first panic is the one re-raised");
    waiter.join().expect("closed() resolved");
    assert_eq!(ticking.get(), 1, "the waiting tick was woken");
    assert_eq!(poll_once(&mut tick), Poll::Ready(Err(closed("m"))));
    assert_eq!(child_out.publish(&1), Err(closed("child")), "the panics skipped nothing");
    m.close(); // closed now: returns, and does not panic again
}

// ---- closed() ----

#[test]
fn closed_resolves_at_once_on_a_closed_scope_and_when_another_thread_closes_it_without_a_runtime() {
    let bus = Bus::new();
    let gone = bus.scope("gone");
    gone.close();
    assert!(poll_once(&mut Box::pin(gone.closed())).is_ready());

    let m = bus.scope("imu");
    let waiter = {
        let m = m.clone();
        thread::spawn(move || {
            futures::executor::block_on(m.closed());
            m.is_closed()
        })
    };
    thread::sleep(ms(20));
    assert!(!waiter.is_finished());
    m.close();
    assert!(waiter.join().unwrap());
}

#[test]
fn closed_futures_given_up_or_polled_again_leave_no_waker_behind() {
    let bus = Bus::new();
    let m = bus.scope("ws_bridge");
    // the losing branch of a select!, a thousand turns of a loop
    let given_up = Count::new();
    for _ in 0..1000 {
        let mut closed = Box::pin(m.closed());
        assert!(poll_with(&mut closed, &given_up.waker()).is_pending());
    }
    // one future polled again and again, each time by another task
    let polled = Count::new();
    let mut closed = Box::pin(m.closed());
    for _ in 0..1000 {
        let waker = Waker::from(Arc::new(Count::default()));
        assert!(poll_with(&mut closed, &waker).is_pending());
    }
    assert!(poll_with(&mut closed, &polled.waker()).is_pending());

    m.close();
    assert_eq!(given_up.get(), 0, "a dropped closed() was woken");
    assert_eq!(polled.get(), 1, "only the last waker is kept");
    assert!(poll_once(&mut closed).is_ready());
}

#[tokio::test]
async fn a_select_loop_on_closed_ends_when_the_scope_closes() {
    let bus = Bus::new();
    let session = bus.scope("session");
    let feeder = bus.scope("app");
    let p = feeder.publisher::<u32>("/joy", Qos::default()).unwrap();
    let mut joy = feeder.subscribe::<u32>("/joy", Qos::default()).unwrap();
    let task = {
        let session = session.clone();
        tokio::spawn(async move {
            let mut seen = 0;
            loop {
                tokio::select! {
                    () = session.closed() => break seen,
                    Some(_) = joy.next() => seen += 1,
                }
            }
        })
    };
    for n in 0..3 {
        p.publish(&n).unwrap();
        tokio::task::yield_now().await;
    }
    session.close();
    let seen = tokio::time::timeout(WAIT, task).await.expect("the loop ended").unwrap();
    assert!(seen <= 3);
}

// ---- child scopes ----

/// Review 2026-10-26: `mower_localize::run_all` made its three scopes with
/// `m.bus.bus().scope(name)`, so closing the module's scope left them open.
#[test]
fn scopes_made_from_a_modules_scope_close_with_it() {
    let bus = Bus::new();
    let module = bus.scope("localize");
    let role = module.child("ekf_filter_node_odom");
    let odom = role.publisher::<u32>("/odometry/filtered", Qos::default()).unwrap();
    module.close();
    assert_eq!(odom.publish(&1), Err(closed("ekf_filter_node_odom")), "the role's publisher outlived the module's scope");
}

#[test]
fn a_child_closes_with_its_parent_and_not_the_other_way_round() {
    let bus = Bus::new();
    let bridge = bus.scope("ws_bridge");
    let own = bridge.publisher::<u32>("/bridge", Qos::default()).unwrap();
    let a = bridge.child("session_a");
    let b = bridge.child("session_b");
    let deeper = b.child("session_b_upload");
    let (pa, pd) = (a.publisher::<u32>("/a", Qos::default()).unwrap(), deeper.publisher::<u32>("/d", Qos::default()).unwrap());

    a.close(); // a session ends on its own
    assert_eq!(pa.publish(&1), Err(closed("session_a")));
    assert!(!bridge.is_closed() && !b.is_closed());
    own.publish(&1).unwrap();

    bridge.close();
    assert!(b.is_closed() && deeper.is_closed(), "children and theirs close with the parent");
    assert_eq!(pd.publish(&1), Err(closed("session_b_upload")));
    assert_eq!(own.publish(&1), Err(closed("ws_bridge")));
    assert!(bridge.child("late").is_closed(), "a child of a closed scope is created closed");
}

// ---- wall timers ----

#[allow(dead_code)]
fn a_pending_tick_is_send(timer: &mut Timer) -> impl Future<Output = Result<Mono, BusError>> + Send + '_ {
    timer.tick()
}

#[test]
fn timers_are_send_and_unpin() {
    fn send_unpin<T: Send + Unpin>() {}
    send_unpin::<Timer>();
}

#[test]
fn a_wall_timer_ticks_like_an_rcl_wall_timer() {
    let bus = Bus::new();
    let (clock, mc) = Clock::manual(Time::default());
    let m = bus.scope("robot_status");
    mc.advance(ms(7));
    let mut timer = m.wall_timer(&clock, ms(100)).unwrap();
    assert_eq!(timer.period(), ms(100));
    let tick = |timer: &mut Timer| {
        let mut tick = Box::pin(timer.tick());
        loop {
            if let Poll::Ready(at) = poll_once(&mut tick) {
                return at;
            }
            mc.advance_to_next().expect("a tick that is not due is pending");
        }
    };
    assert_eq!(tick(&mut timer), Ok(at_ms(107)), "the first tick one period after creation");
    mc.advance(ms(250));
    assert_eq!(tick(&mut timer), Ok(at_ms(207)), "after an overrun it fires once, late");
    assert_eq!(clock.mono(), at_ms(357));
    assert_eq!(tick(&mut timer), Ok(at_ms(407)), "then keeps the original phase");
}

#[test]
fn a_wall_timer_is_refused_on_a_closed_scope() {
    let bus = Bus::new();
    let (clock, mc) = Clock::manual(Time::default());
    let gone = bus.scope("gone");
    gone.close();
    assert_eq!(gone.wall_timer(&clock, ms(10)).unwrap_err(), closed("gone"));
    assert_eq!(mc.next_deadline(), None, "and leaves no deadline behind");
}

/// Review 2026-10-26: r2r's `while timer.tick().await.is_ok() { .. }`
/// (velocity_command_guard, robot_status, mower_adapter) ends when the node
/// drops. Ported to a plain interval it never ended: the old instance's
/// watchdog kept ticking after the supervisor closed its scope.
#[test]
fn a_ported_wall_timer_loop_ends_when_its_scope_closes() {
    let bus = Bus::new();
    let (clock, mc) = Clock::manual(Time::default());
    let guard = bus.scope("velocity_command_guard");
    let publisher = guard.publisher::<u32>("/cmd_vel_out", Qos::default().keep_last(1)).unwrap();
    let ticks = Arc::new(AtomicUsize::new(0));
    let period = ms(50);
    let mut watchdog = guard.wall_timer(&clock, period).unwrap();
    let task = {
        let ticks = ticks.clone();
        async move {
            // the loop as velocity_command_guard has it
            while watchdog.tick().await.is_ok() {
                ticks.fetch_add(1, Ordering::SeqCst);
                let _ = publisher.publish(&0);
            }
        }
    };
    let mut task = Box::pin(task);
    assert!(poll_once(&mut task).is_pending());
    mc.advance(period);
    assert!(poll_once(&mut task).is_pending());
    assert_eq!(ticks.load(Ordering::SeqCst), 1);

    guard.close(); // what the supervisor does when the instance ends
    let mut finished = false;
    for _ in 0..20 {
        mc.advance(period);
        if poll_once(&mut task).is_ready() {
            finished = true;
            break;
        }
    }
    assert!(
        finished,
        "the closed instance's timer loop ran {} more times and is still running",
        ticks.load(Ordering::SeqCst) - 1
    );
    assert_eq!(ticks.load(Ordering::SeqCst), 1, "no tick after the close");
}

#[test]
fn a_tick_waiting_when_the_scope_closes_is_woken_and_fails_with_closed() {
    let bus = Bus::new();
    let (clock, mc) = Clock::manual(Time::default());
    let m = bus.scope("mower_adapter");
    let mut timer = m.wall_timer(&clock, ms(100)).unwrap();
    let woken = Count::new();
    let mut tick = Box::pin(timer.tick());
    assert!(poll_with(&mut tick, &woken.waker()).is_pending());
    mc.advance(ms(30));
    assert_eq!(woken.get(), 0);

    m.close();
    assert_eq!(woken.get(), 1, "the close wakes the waiting tick");
    assert_eq!(poll_once(&mut tick), Poll::Ready(Err(closed("mower_adapter"))));
    drop(tick);
    assert_eq!(poll_once(&mut Box::pin(timer.tick())), Poll::Ready(Err(closed("mower_adapter"))), "and every later one");
}

#[test]
fn when_the_close_and_a_tick_come_together_closed_wins() {
    let bus = Bus::new();
    let (clock, mc) = Clock::manual(Time::default());

    // both have happened by the time the tick is polled
    let m = bus.scope("a");
    let mut timer = m.wall_timer(&clock, ms(10)).unwrap();
    let mut tick = Box::pin(timer.tick());
    assert!(poll_once(&mut tick).is_pending());
    mc.advance(ms(10));
    m.close();
    assert_eq!(poll_once(&mut tick), Poll::Ready(Err(closed("a"))));

    // the tick comes due while the close is still running its hooks
    let m = bus.scope("b");
    let (gate, _reader) = hold_the_close_of(&m);
    let mut timer = m.wall_timer(&clock, ms(10)).unwrap();
    mc.advance(ms(10));
    let closer = start_closing(&m, &gate);
    assert_eq!(poll_once(&mut Box::pin(timer.tick())), Poll::Ready(Err(closed("b"))));
    gate.open();
    closer.join().unwrap();
}

#[tokio::test]
async fn a_system_clock_tick_waiting_in_a_task_ends_with_the_scope() {
    let bus = Bus::new();
    let m = bus.scope("robot_status");
    let mut heartbeat = m.wall_timer(&Clock::system(), Duration::from_secs(3600)).unwrap();
    let task = tokio::spawn(async move { heartbeat.tick().await });
    tokio::time::sleep(ms(20)).await; // let it wait
    m.close();
    let result = tokio::time::timeout(WAIT, task).await.expect("woken by the close").unwrap();
    assert_eq!(result, Err(closed("robot_status")));
}
