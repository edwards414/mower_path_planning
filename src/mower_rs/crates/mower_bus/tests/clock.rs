//! The clock (README.md, "Clock"): `Time` and `Mono` arithmetic, the system
//! clock against std and tokio, the manual clock's wake-ups, `Interval`
//! against tokio's own interval under paused time as the oracle, and
//! `Timeout` on both clocks.

use std::future::Future;
use std::pin::{pin, Pin};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::task::{Context, Poll, Wake, Waker};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use mower_bus::{Clock, Elapsed, Interval, ManualClock, MissedTick, Mono, Sleep, Time, Timeout};

const SEC: i64 = 1_000_000_000;

fn ms(n: u64) -> Duration {
    Duration::from_millis(n)
}

fn at_ms(n: u64) -> Mono {
    Mono::from_ns(n * 1_000_000)
}

/// Poll once with a waker that does nothing.
fn poll_once<F: Future + Unpin>(f: &mut F) -> Poll<F::Output> {
    Pin::new(f).poll(&mut Context::from_waker(futures::task::noop_waker_ref()))
}

/// Poll once with `waker`.
fn poll_with<F: Future + Unpin>(f: &mut F, waker: &Waker) -> Poll<F::Output> {
    Pin::new(f).poll(&mut Context::from_waker(waker))
}

/// A waker that writes `id` into `log` when woken.
struct Recorder {
    id: &'static str,
    log: Arc<Mutex<Vec<&'static str>>>,
}

impl Wake for Recorder {
    fn wake(self: Arc<Self>) {
        self.log.lock().unwrap().push(self.id);
    }
}

fn recorder(id: &'static str, log: &Arc<Mutex<Vec<&'static str>>>) -> Waker {
    Waker::from(Arc::new(Recorder { id, log: log.clone() }))
}

/// One tick on a manual clock. When it is not due, time jumps to its
/// deadline, as tokio's paused time auto-advances to its next timer.
fn tick_now(interval: &mut Interval, mc: &ManualClock) -> Mono {
    let mut tick = pin!(interval.tick());
    loop {
        if let Poll::Ready(at) = poll_once(&mut tick) {
            return at;
        }
        mc.advance_to_next().expect("a tick that is not due has a pending deadline");
    }
}

/// Spin until another thread gets `cond` true, failing after 10 s.
fn wait_until(what: &str, cond: impl Fn() -> bool) {
    let start = Instant::now();
    while !cond() {
        assert!(start.elapsed() < Duration::from_secs(10), "timed out waiting until {what}");
        std::thread::yield_now();
    }
}

// Static assertions (they fail to compile, not to run): sleeps, intervals,
// timeouts and a pending tick can be held across `.await` in spawned tasks,
// and a `Sleep` (or a timeout of an `Unpin` future) can be polled in place,
// as tokio's `Pin<Box<Sleep>>` could.
const _: () = {
    const fn send_unpin<T: Send + Unpin>() {}
    const fn send_sync<T: Send + Sync>() {}
    send_unpin::<Sleep>();
    send_unpin::<Interval>();
    send_unpin::<Timeout<Sleep>>();
    send_sync::<Clock>();
    send_sync::<ManualClock>();
};

#[allow(dead_code)]
fn a_pending_tick_is_send(interval: &mut Interval) -> impl Future<Output = Mono> + Send + '_ {
    interval.tick()
}

#[allow(dead_code)]
fn a_timeout_of_a_send_future_that_is_not_unpin_is_send(clock: &Clock) -> impl Future<Output = Result<u8, Elapsed>> + Send {
    let sleep = clock.sleep(ms(1));
    clock.timeout(ms(1), async move {
        sleep.await;
        1
    })
}

// ---- Time and Mono ----

#[test]
fn time_parts_have_floor_seconds_and_nanoseconds_below_one_second_also_before_the_epoch() {
    let cases = [
        (0, (0, 0)),
        (1, (0, 1)),
        (-1, (-1, 999_999_999)),
        (-SEC, (-1, 0)),
        (1_500_000_000, (1, 500_000_000)),
        (-1_500_000_000, (-2, 500_000_000)),
        (1_792_900_000_123_456_789, (1_792_900_000, 123_456_789)),
        (-1_792_900_000_123_456_789, (-1_792_900_001, 876_543_211)),
        (i32::MAX as i64 * SEC + 999_999_999, (i32::MAX, 999_999_999)),
        (i32::MIN as i64 * SEC, (i32::MIN, 0)),
    ];
    for (ns, parts) in cases {
        let t = Time::from_ns(ns);
        assert_eq!(t.to_parts(), parts, "{ns}");
        assert_eq!(Time::from_parts(parts.0, parts.1), t, "{ns}");
    }
}

#[test]
fn nanoseconds_of_a_second_or_more_carry_into_the_seconds() {
    assert_eq!(Time::from_parts(1, 1_500_000_000), Time::from_parts(2, 500_000_000));
    assert_eq!(Time::from_parts(-1, 1_000_000_000), Time::from_ns(0));
    assert_eq!(Time::from_parts(-3, 2_500_000_000).to_parts(), (-1, 500_000_000));
    assert_eq!(Time::from_parts(i32::MIN, u32::MAX).to_parts(), (i32::MIN + 4, 294_967_295));
    assert_eq!(Time::from_parts(i32::MAX, u32::MAX).ns(), i32::MAX as i64 * SEC + u32::MAX as i64);
}

#[test]
fn parts_saturate_outside_the_i32_seconds_of_builtin_interfaces() {
    assert_eq!(Time::from_ns((i32::MAX as i64 + 1) * SEC).to_parts(), (i32::MAX, 999_999_999));
    assert_eq!(Time::from_ns(i64::MAX).to_parts(), (i32::MAX, 999_999_999));
    assert_eq!(Time::from_ns(i32::MIN as i64 * SEC - 1).to_parts(), (i32::MIN, 0));
    assert_eq!(Time::from_ns(i64::MIN).to_parts(), (i32::MIN, 0));
}

#[test]
fn time_round_trips_through_system_time_on_both_sides_of_the_epoch() {
    let now = SystemTime::now();
    assert_eq!(Time::from_system(now).to_system(), now);
    for ns in [0, 1, -1, 1_792_900_000_123_456_789, -1_500_000_000, i64::MAX, i64::MIN] {
        let t = Time::from_ns(ns);
        assert_eq!(Time::from_system(t.to_system()), t, "{ns}");
    }
    assert_eq!(Time::from_system(UNIX_EPOCH + Duration::new(1, 5)), Time::from_ns(SEC + 5));
    assert_eq!(Time::from_system(UNIX_EPOCH - ms(1500)), Time::from_ns(-1_500_000_000));
    // beyond what i64 nanoseconds hold (about 292 years each way)
    let centuries = Duration::from_secs(400 * 366 * 86_400);
    assert_eq!(Time::from_system(UNIX_EPOCH + centuries), Time::from_ns(i64::MAX));
    assert_eq!(Time::from_system(UNIX_EPOCH - centuries), Time::from_ns(i64::MIN));
}

#[test]
fn time_arithmetic_saturates_at_the_i64_limits() {
    let t = Time::from_ns(10);
    assert_eq!(t + Duration::from_nanos(5), Time::from_ns(15));
    assert_eq!(t - Duration::from_nanos(15), Time::from_ns(-5));
    assert_eq!(Time::from_ns(i64::MAX - 1) + Duration::from_nanos(5), Time::from_ns(i64::MAX));
    assert_eq!(Time::from_ns(i64::MIN + 1) - Duration::from_nanos(5), Time::from_ns(i64::MIN));
    assert_eq!(t + Duration::MAX, Time::from_ns(i64::MAX));
    assert_eq!(t - Duration::MAX, Time::from_ns(i64::MIN));

    assert_eq!(Time::from_ns(15).duration_since(t), Some(Duration::from_nanos(5)));
    assert_eq!(t.duration_since(t), Some(Duration::ZERO));
    assert_eq!(t.duration_since(Time::from_ns(11)), None);
    assert_eq!(Time::from_ns(500_000_000).duration_since(Time::from_ns(-1_500_000_000)), Some(Duration::from_secs(2)));
    assert_eq!(Time::from_ns(i64::MAX).duration_since(Time::from_ns(i64::MIN)), Some(Duration::from_nanos(u64::MAX)));
}

#[test]
fn as_secs_f64_is_rclcpp_seconds() {
    for ns in [0, 1, -1, 1_792_900_000_123_456_789, -1_500_000_000] {
        assert_eq!(Time::from_ns(ns).as_secs_f64(), ns as f64 / 1e9, "{ns}");
    }
    assert_eq!(Time::from_parts(12, 500_000_000).as_secs_f64(), 12.5);
}

#[test]
fn mono_arithmetic_saturates_and_durations_between_readings_are_checked() {
    let m = Mono::from_ns(100);
    assert_eq!(m + Duration::from_nanos(5), Mono::from_ns(105));
    assert_eq!(m - Duration::from_nanos(30), Mono::from_ns(70));
    assert_eq!(m - Duration::from_nanos(101), Mono::from_ns(0));
    assert_eq!(m + Duration::MAX, Mono::from_ns(u64::MAX));
    assert_eq!(Mono::from_ns(u64::MAX - 1) + Duration::from_nanos(2), Mono::from_ns(u64::MAX));

    assert_eq!(Mono::from_ns(150).checked_duration_since(m), Some(Duration::from_nanos(50)));
    assert_eq!(m.checked_duration_since(m), Some(Duration::ZERO));
    assert_eq!(m.checked_duration_since(Mono::from_ns(150)), None);
    assert_eq!(Mono::from_ns(150).saturating_duration_since(m), Duration::from_nanos(50));
    assert_eq!(m.saturating_duration_since(Mono::from_ns(150)), Duration::ZERO);
    assert!(Mono::from_ns(1) < Mono::from_ns(2));
    assert_eq!(Mono::default(), Mono::from_ns(0));
}

// ---- the system clock ----

#[test]
fn system_mono_never_goes_back_and_compares_across_handles_and_threads() {
    let (a, b) = (Clock::system(), Clock::system());
    assert!(!a.is_manual());
    let mut last = a.mono();
    for i in 0..10_000 {
        let m = if i % 2 == 0 { a.mono() } else { b.mono() };
        assert!(m >= last, "{m:?} after {last:?}");
        last = m;
    }
    let elsewhere = std::thread::spawn(|| Clock::system().mono()).join().unwrap();
    assert!(elsewhere >= last);
    assert!(b.mono() >= elsewhere);
}

#[test]
fn system_wall_time_is_system_time() {
    let before = Time::from_system(SystemTime::now());
    let now = Clock::system().now();
    let after = Time::from_system(SystemTime::now());
    assert!(before <= now && now <= after, "{before:?} {now:?} {after:?}");
}

#[tokio::test]
async fn system_async_sleeps_last_at_least_their_duration() {
    let clock = Clock::system();
    let (start, real_start) = (clock.mono(), Instant::now());
    clock.sleep(ms(30)).await;
    assert!(clock.mono().saturating_duration_since(start) >= ms(30));
    assert!(real_start.elapsed() >= ms(30));

    let deadline = clock.mono() + ms(20);
    clock.sleep_until(deadline).await;
    assert!(clock.mono() >= deadline);
}

#[tokio::test]
async fn a_system_sleep_of_duration_max_waits_without_panicking() {
    let clock = Clock::system();
    assert!(tokio::time::timeout(ms(20), clock.sleep(Duration::MAX)).await.is_err());
}

#[test]
fn system_sleeps_and_ticks_already_due_are_ready_at_once_without_a_runtime() {
    let clock = Clock::system();
    assert!(poll_once(&mut clock.sleep(Duration::ZERO)).is_ready());
    assert!(poll_once(&mut clock.sleep_until(Mono::from_ns(0))).is_ready());
    let mut interval = clock.interval(ms(10));
    assert!(poll_once(&mut pin!(interval.tick())).is_ready());
}

#[test]
fn system_blocking_sleeps_last_at_least_their_duration() {
    let clock = Clock::system();
    let (start, real_start) = (clock.mono(), Instant::now());
    clock.sleep_blocking(ms(20));
    assert!(clock.mono().saturating_duration_since(start) >= ms(20));
    assert!(real_start.elapsed() >= ms(20));
    let deadline = clock.mono() + ms(15);
    clock.sleep_until_blocking(deadline);
    assert!(clock.mono() >= deadline);
}

#[tokio::test]
async fn a_system_interval_ticks_at_once_then_every_period() {
    let clock = Clock::system();
    let created = clock.mono();
    let mut interval = clock.interval(ms(20));
    let first = interval.tick().await;
    assert!(created <= first && first <= clock.mono(), "the first deadline is the time of creation");
    // Burst keeps every deadline on the grid, however late the test runs.
    assert_eq!(interval.tick().await, first + ms(20));
    assert!(clock.mono() >= first + ms(20));
    assert_eq!(interval.tick().await, first + ms(40));
}

#[test]
fn a_system_interval_ticks_on_a_std_thread_without_a_runtime() {
    let clock = Clock::system();
    let mut interval = clock.interval(ms(10));
    let first = interval.tick_blocking();
    assert_eq!(interval.tick_blocking(), first + ms(10));
    assert_eq!(interval.tick_blocking(), first + ms(20));
    assert!(clock.mono() >= first + ms(20));
}

// ---- the manual clock ----

#[test]
fn a_manual_clock_starts_at_mono_zero_and_moves_only_when_told() {
    let start = Time::from_parts(1_792_900_000, 0);
    let (clock, mc) = Clock::manual(start);
    assert!(clock.is_manual());
    assert_eq!((clock.mono(), clock.now()), (Mono::from_ns(0), start));
    std::thread::sleep(ms(2));
    assert_eq!((clock.mono(), clock.now()), (Mono::from_ns(0), start), "real time does not move it");

    mc.advance(ms(1500));
    assert_eq!((clock.mono(), clock.now()), (at_ms(1500), start + ms(1500)));
    mc.advance_to(at_ms(1000));
    assert_eq!((clock.mono(), clock.now()), (at_ms(1500), start + ms(1500)), "never back");
    mc.advance_to(at_ms(2000));
    assert_eq!((clock.mono(), clock.now()), (at_ms(2000), start + ms(2000)));

    let same = mc.clone().clock();
    mc.advance(ms(1));
    assert_eq!(same.mono(), at_ms(2001), "every handle is the same clock");
}

#[test]
fn manual_wake_ups_go_in_deadline_order_then_registration_order() {
    let (clock, mc) = Clock::manual(Time::default());
    let log = Arc::new(Mutex::new(Vec::new()));
    let mut sleeps =
        [("30", clock.sleep(ms(30))), ("10a", clock.sleep_until(at_ms(10))), ("20", clock.sleep(ms(20))), ("10b", clock.sleep(ms(10)))];
    for (id, sleep) in &mut sleeps {
        assert!(poll_with(sleep, &recorder(id, &log)).is_pending());
    }
    let mut interval = clock.interval_at(at_ms(15), ms(100));
    assert!(poll_with(&mut pin!(interval.tick()), &recorder("tick", &log)).is_pending());

    mc.advance(ms(25));
    assert_eq!(*log.lock().unwrap(), ["10a", "10b", "tick", "20"]);
    for (id, sleep) in &mut sleeps {
        assert_eq!(poll_once(sleep).is_ready(), *id != "30", "{id}");
    }
    assert_eq!(poll_once(&mut pin!(interval.tick())), Poll::Ready(at_ms(15)));
    assert_eq!(mc.next_deadline(), Some(at_ms(30)));
}

#[test]
fn woken_tasks_see_the_new_time_without_a_runtime_timer() {
    // A runtime without time enabled: the manual clock needs no tokio timer.
    let rt = tokio::runtime::Builder::new_current_thread().build().unwrap();
    let (clock, mc) = Clock::manual(Time::default());
    let seen = Arc::new(Mutex::new(Vec::new()));
    let ready = Arc::new(AtomicUsize::new(0));
    rt.block_on(async {
        let mut tasks = Vec::new();
        for (id, deadline) in [(3, 30), (1, 10), (2, 20)] {
            let (clock, seen, ready) = (clock.clone(), seen.clone(), ready.clone());
            tasks.push(tokio::spawn(async move {
                let sleep = clock.sleep_until(at_ms(deadline));
                ready.fetch_add(1, Ordering::SeqCst);
                sleep.await;
                seen.lock().unwrap().push((id, clock.mono()));
            }));
        }
        while ready.load(Ordering::SeqCst) < 3 {
            tokio::task::yield_now().await;
        }
        mc.advance(ms(25));
        while seen.lock().unwrap().len() < 2 {
            tokio::task::yield_now().await;
        }
        assert_eq!(*seen.lock().unwrap(), [(1, at_ms(25)), (2, at_ms(25))], "in deadline order, at the new time");
        assert_eq!(mc.advance_to_next(), Some(at_ms(30)));
        for task in tasks {
            task.await.unwrap();
        }
    });
    assert_eq!(seen.lock().unwrap()[2], (3, at_ms(30)));
}

#[test]
fn an_async_sleeper_on_another_thread_is_woken_by_advance() {
    let (clock, mc) = Clock::manual(Time::default());
    let sleeper = std::thread::spawn({
        let clock = clock.clone();
        move || {
            futures::executor::block_on(clock.sleep(ms(50)));
            clock.mono()
        }
    });
    wait_until("the sleep is registered", || mc.next_deadline() == Some(at_ms(50)));
    mc.advance(ms(80));
    assert_eq!(sleeper.join().unwrap(), at_ms(80));
}

#[test]
fn blocking_sleepers_on_std_threads_wake_when_the_manual_clock_moves() {
    let (clock, mc) = Clock::manual(Time::default());
    let first = std::thread::spawn({
        let clock = clock.clone();
        move || {
            clock.sleep_blocking(ms(10));
            clock.mono()
        }
    });
    let second = std::thread::spawn({
        let clock = clock.clone();
        move || {
            clock.sleep_until_blocking(at_ms(20));
            clock.mono()
        }
    });
    wait_until("the first thread sleeps", || mc.next_deadline() == Some(at_ms(10)));
    mc.advance(ms(10));
    assert_eq!(first.join().unwrap(), at_ms(10));
    wait_until("the second thread sleeps", || mc.next_deadline() == Some(at_ms(20)));
    assert!(!second.is_finished());
    assert_eq!(mc.advance_to_next(), Some(at_ms(20)));
    assert_eq!(second.join().unwrap(), at_ms(20));
    assert_eq!(mc.next_deadline(), None);

    let mut interval = clock.interval(ms(5));
    let ticker = std::thread::spawn(move || [interval.tick_blocking(), interval.tick_blocking(), interval.tick_blocking()]);
    for deadline in [25, 30] {
        wait_until("the next tick is pending", || mc.next_deadline() == Some(at_ms(deadline)));
        mc.advance_to_next();
    }
    assert_eq!(ticker.join().unwrap(), [at_ms(20), at_ms(25), at_ms(30)]);
}

#[test]
fn advance_to_next_jumps_to_the_earliest_pending_deadline() {
    let start = Time::from_ns(SEC);
    let (clock, mc) = Clock::manual(start);
    assert_eq!((mc.next_deadline(), mc.advance_to_next()), (None, None));
    assert_eq!(clock.mono(), Mono::from_ns(0), "nothing pending: no move");

    let mut late = clock.sleep(ms(50));
    let mut early = clock.sleep(ms(20));
    let mut interval = clock.interval_at(at_ms(35), ms(100));
    assert_eq!(mc.next_deadline(), Some(at_ms(20)), "pending from creation, before any poll");

    assert_eq!(mc.advance_to_next(), Some(at_ms(20)));
    assert_eq!((clock.mono(), clock.now()), (at_ms(20), start + ms(20)));
    assert!(poll_once(&mut early).is_ready() && poll_once(&mut late).is_pending());
    assert_eq!(mc.advance_to_next(), Some(at_ms(35)));
    assert_eq!(poll_once(&mut pin!(interval.tick())), Poll::Ready(at_ms(35)));
    assert_eq!(mc.advance_to_next(), Some(at_ms(50)));
    assert!(poll_once(&mut late).is_ready());
    assert_eq!(mc.next_deadline(), Some(at_ms(135)), "an interval's next tick is pending while its task is busy");
    assert_eq!(mc.advance_to_next(), Some(at_ms(135)));
    assert_eq!(mc.advance_to_next(), None, "a due tick is no longer pending");
    assert_eq!(poll_once(&mut pin!(interval.tick())), Poll::Ready(at_ms(135)));
    assert_eq!(mc.next_deadline(), Some(at_ms(235)));
}

/// Review 2026-10-26: clock.rs said "`mono + Duration::MAX` is a deadline
/// that never comes", yet on a manual clock `advance_to_next()` jumped
/// straight to it: a `timeout(Duration::MAX, ..)` ("no timeout") elapsed,
/// and the clock was left at `Mono(u64::MAX)`, where every `mono() + d`
/// saturates to `mono()`, so every later sleep was due at once.
#[test]
fn a_deadline_of_duration_max_never_comes_on_a_manual_clock() {
    let (clock, mc) = Clock::manual(Time::default());
    let mut no_timeout = clock.timeout(Duration::MAX, futures::future::pending::<()>());
    assert!(poll_once(&mut no_timeout).is_pending());

    let jumped = mc.advance_to_next();
    let elapsed = poll_once(&mut no_timeout).is_ready();
    let later_sleep_due_at_once = poll_once(&mut clock.sleep(Duration::from_secs(1))).is_ready();
    assert_eq!(jumped, None, "advance_to_next jumped to the end of time ({:?})", clock.mono());
    assert!(!elapsed, "a timeout of Duration::MAX elapsed");
    assert!(!later_sleep_due_at_once, "a 1 s sleep made afterwards was due at once");
    assert_ne!(clock.mono(), Mono::from_ns(u64::MAX));
}

#[test]
fn a_deadline_of_mono_max_comes_only_when_the_clock_is_moved_there() {
    let never = Mono::from_ns(u64::MAX);
    let (clock, mc) = Clock::manual(Time::default());
    let mut forever = clock.sleep(Duration::MAX);
    let mut also = clock.sleep_until(never);
    let mut soon = clock.sleep(ms(10));
    let blocked = std::thread::spawn({
        let clock = clock.clone();
        move || {
            clock.sleep_blocking(Duration::MAX);
            clock.mono()
        }
    });
    assert!(poll_once(&mut forever).is_pending() && poll_once(&mut also).is_pending());
    assert_eq!(mc.next_deadline(), Some(at_ms(10)), "the deadlines that can come are still reported");
    assert_eq!(mc.advance_to_next(), Some(at_ms(10)));
    assert!(poll_once(&mut soon).is_ready());
    assert_eq!((mc.next_deadline(), mc.advance_to_next()), (None, None), "never is not pending");

    mc.advance(Duration::from_secs(100 * 365 * 86_400));
    assert!(poll_once(&mut forever).is_pending(), "a century is not the end of time");
    assert!(!blocked.is_finished());
    mc.advance(Duration::MAX); // saturates at the end of time: an explicit move there
    assert_eq!(clock.mono(), never);
    assert!(poll_once(&mut forever).is_ready() && poll_once(&mut also).is_ready());
    assert_eq!(blocked.join().unwrap(), never);
}

#[test]
fn set_wall_steps_wall_time_only_and_wakes_nothing() {
    let start = Time::from_parts(1_792_900_000, 0);
    let (clock, mc) = Clock::manual(start);
    let log = Arc::new(Mutex::new(Vec::new()));
    let waker = recorder("sleep", &log);
    let mut sleep = clock.sleep(ms(10));
    assert!(poll_with(&mut sleep, &waker).is_pending());

    mc.set_wall(start + Duration::from_secs(3600));
    mc.set_wall(start - Duration::from_secs(7200));
    assert_eq!((clock.now(), clock.mono()), (start - Duration::from_secs(7200), Mono::from_ns(0)));
    assert!(log.lock().unwrap().is_empty());
    assert!(poll_with(&mut sleep, &waker).is_pending());
    assert_eq!(mc.next_deadline(), Some(at_ms(10)));

    mc.advance(ms(10));
    assert_eq!(clock.now(), start - Duration::from_secs(7200) + ms(10), "wall moves on from the step");
    assert_eq!(*log.lock().unwrap(), ["sleep"]);
}

#[test]
fn manual_sleeps_already_due_are_ready_at_once_and_never_pending() {
    let (clock, mc) = Clock::manual(Time::default());
    mc.advance(ms(10));
    let mut now = clock.sleep(Duration::ZERO);
    let mut past = clock.sleep_until(at_ms(3));
    assert_eq!(mc.next_deadline(), None);
    assert!(poll_once(&mut now).is_ready() && poll_once(&mut past).is_ready());
}

#[test]
fn a_dropped_sleep_or_interval_leaves_no_deadline_behind() {
    let (clock, mc) = Clock::manual(Time::default());
    let unpolled = clock.sleep(ms(10));
    assert_eq!(mc.next_deadline(), Some(at_ms(10)));
    drop(unpolled);
    assert_eq!(mc.next_deadline(), None);

    let mut polled = clock.sleep(ms(20));
    assert!(poll_once(&mut polled).is_pending());
    drop(polled);
    assert_eq!(mc.next_deadline(), None);

    // the losing branch of a select
    let raced = futures::executor::block_on(futures::future::select(clock.sleep(ms(5)), futures::future::ready(())));
    assert!(matches!(raced, futures::future::Either::Right(_)));
    drop(raced);
    assert_eq!(mc.next_deadline(), None);

    let ahead = clock.interval_at(at_ms(3), ms(7));
    assert_eq!(mc.next_deadline(), Some(at_ms(3)));
    drop(ahead);
    let mut ticked = clock.interval(ms(7));
    assert_eq!(poll_once(&mut pin!(ticked.tick())), Poll::Ready(Mono::from_ns(0)));
    assert_eq!(mc.next_deadline(), Some(at_ms(7)));
    drop(ticked);
    assert_eq!(mc.next_deadline(), None);
    assert_eq!(mc.advance_to_next(), None);
}

// ---- Interval ----

/// One step of an interval schedule, run against both clocks.
#[derive(Clone, Copy, Debug)]
enum Step {
    /// Take a tick; one that is not due waits (time jumps to it).
    Tick,
    /// The task is busy this many ms without ticking: an overrun when it is
    /// longer than the period.
    Busy(u64),
    /// `Interval::reset`.
    Reset,
}

use Step::{Busy, Reset, Tick};

struct Schedule {
    what: &'static str,
    period: u64,
    /// ms that pass before the interval is created.
    after: u64,
    /// `None`: `interval(period)`; `Some(ms)`: `interval_at(now + ms)`,
    /// in the past when negative.
    first: Option<i64>,
    steps: &'static [Step],
}

/// Whole milliseconds only: tokio's timer wheel rounds deadlines up to the
/// next ms, so with whole ms its paused clock lands on them exactly.
const SCHEDULES: &[Schedule] = &[
    Schedule { what: "on time", period: 10, after: 0, first: None, steps: &[Tick, Tick, Tick, Tick] },
    Schedule {
        what: "busy within the period, once exactly up to the deadline",
        period: 10,
        after: 0,
        first: None,
        steps: &[Tick, Busy(4), Tick, Busy(10), Tick, Busy(9), Tick, Tick],
    },
    Schedule {
        what: "overrun of several periods",
        period: 10,
        after: 0,
        first: None,
        steps: &[Tick, Busy(35), Tick, Tick, Tick, Tick, Tick],
    },
    Schedule {
        what: "overrun of exactly three periods",
        period: 10,
        after: 0,
        first: None,
        steps: &[Tick, Busy(30), Tick, Tick, Tick, Tick, Tick],
    },
    Schedule {
        what: "late by 5 ms (on time) and by 6 ms (missed)",
        period: 10,
        after: 0,
        first: None,
        steps: &[Tick, Busy(15), Tick, Tick, Busy(16), Tick, Tick, Tick],
    },
    Schedule {
        what: "a period shorter than the 5 ms tolerance",
        period: 2,
        after: 0,
        first: None,
        steps: &[Tick, Busy(4), Tick, Tick, Tick, Busy(9), Tick, Tick, Tick, Tick, Tick, Tick],
    },
    Schedule {
        what: "a long overrun ending mid-period",
        period: 10,
        after: 0,
        first: None,
        steps: &[Tick, Busy(57), Tick, Tick, Tick, Tick, Tick, Tick, Tick, Tick],
    },
    Schedule {
        what: "repeated overruns",
        period: 10,
        after: 0,
        first: None,
        steps: &[Tick, Busy(12), Tick, Busy(23), Tick, Busy(7), Tick, Tick, Busy(31), Tick, Tick],
    },
    Schedule {
        what: "reset after an overrun and mid-period",
        period: 10,
        after: 0,
        first: None,
        steps: &[Tick, Busy(25), Reset, Tick, Tick, Busy(3), Reset, Tick, Tick],
    },
    Schedule { what: "reset before the first tick", period: 10, after: 0, first: None, steps: &[Reset, Tick, Tick] },
    Schedule {
        what: "an rcl wall timer: first tick one period ahead",
        period: 100,
        after: 7,
        first: Some(100),
        steps: &[Tick, Busy(250), Tick, Tick, Busy(100), Tick, Tick],
    },
    Schedule { what: "interval_at shortly ahead", period: 50, after: 13, first: Some(7), steps: &[Tick, Tick, Busy(120), Tick, Tick] },
    Schedule { what: "interval_at in the past", period: 10, after: 40, first: Some(-25), steps: &[Tick, Tick, Tick, Tick, Tick] },
    Schedule { what: "created after time has passed", period: 20, after: 33, first: None, steps: &[Tick, Busy(45), Tick, Tick, Tick] },
];

/// Per tick: the deadline it returned and the time it returned at, both
/// since the clock started.
type Ticks = Vec<(Duration, Duration)>;

fn ticks_on_a_manual_clock(s: &Schedule, behavior: MissedTick) -> Ticks {
    let (clock, mc) = Clock::manual(Time::default());
    mc.advance(ms(s.after));
    let mut interval = match s.first {
        None => clock.interval(ms(s.period)),
        Some(ahead) if ahead >= 0 => clock.interval_at(clock.mono() + ms(ahead as u64), ms(s.period)),
        Some(behind) => clock.interval_at(clock.mono() - ms(behind.unsigned_abs()), ms(s.period)),
    };
    interval.set_missed_tick_behavior(behavior);
    let since_start = |m: Mono| Duration::from_nanos(m.ns());
    let mut ticks = Vec::new();
    for &step in s.steps {
        match step {
            Tick => {
                let at = tick_now(&mut interval, &mc);
                ticks.push((since_start(at), since_start(clock.mono())));
            }
            Busy(n) => mc.advance(ms(n)),
            Reset => interval.reset(),
        }
    }
    ticks
}

fn ticks_on_tokio(s: &Schedule, behavior: MissedTick) -> Ticks {
    use tokio::time::{advance, Instant, MissedTickBehavior};
    let rt = tokio::runtime::Builder::new_current_thread().enable_time().start_paused(true).build().unwrap();
    rt.block_on(async {
        let start = Instant::now();
        advance(ms(s.after)).await;
        let now = Instant::now();
        let mut interval = match s.first {
            None => tokio::time::interval(ms(s.period)),
            Some(ahead) if ahead >= 0 => tokio::time::interval_at(now + ms(ahead as u64), ms(s.period)),
            Some(behind) => tokio::time::interval_at(now - ms(behind.unsigned_abs()), ms(s.period)),
        };
        interval.set_missed_tick_behavior(match behavior {
            MissedTick::Burst => MissedTickBehavior::Burst,
            MissedTick::Delay => MissedTickBehavior::Delay,
            MissedTick::Skip => MissedTickBehavior::Skip,
        });
        let mut ticks = Vec::new();
        for &step in s.steps {
            match step {
                Tick => {
                    let at = interval.tick().await;
                    ticks.push((at - start, Instant::now() - start));
                }
                Busy(n) => advance(ms(n)).await,
                Reset => interval.reset(),
            }
        }
        ticks
    })
}

#[test]
fn intervals_tick_like_tokio_for_every_missed_tick_behavior() {
    for s in SCHEDULES {
        for behavior in [MissedTick::Burst, MissedTick::Delay, MissedTick::Skip] {
            let ours = ticks_on_a_manual_clock(s, behavior);
            assert_eq!(ours.len(), s.steps.iter().filter(|s| matches!(s, Tick)).count());
            assert_eq!(ours, ticks_on_tokio(s, behavior), "{} with {behavior:?}", s.what);
        }
    }
}

#[test]
fn after_an_overrun_ticks_burst_delay_or_skip() {
    let overrun = SCHEDULES.iter().find(|s| s.what == "overrun of several periods").unwrap();
    let in_ms = |ticks: Ticks| ticks.iter().map(|(at, now)| (at.as_millis(), now.as_millis())).collect::<Vec<_>>();
    // the tick due at 10 fires at 35 in each case
    assert_eq!(
        in_ms(ticks_on_a_manual_clock(overrun, MissedTick::Burst)),
        [(0, 0), (10, 35), (20, 35), (30, 35), (40, 40), (50, 50)],
        "the missed ticks back to back"
    );
    assert_eq!(
        in_ms(ticks_on_a_manual_clock(overrun, MissedTick::Delay)),
        [(0, 0), (10, 35), (45, 45), (55, 55), (65, 65), (75, 75)],
        "every period from the late tick"
    );
    assert_eq!(
        in_ms(ticks_on_a_manual_clock(overrun, MissedTick::Skip)),
        [(0, 0), (10, 35), (40, 40), (50, 50), (60, 60), (70, 70)],
        "on the original phase"
    );
}

#[test]
fn interval_at_one_period_ahead_with_skip_is_an_rcl_wall_timer() {
    let (clock, mc) = Clock::manual(Time::default());
    mc.advance(ms(7));
    let period = ms(100);
    let mut timer = clock.interval_at(clock.mono() + period, period);
    timer.set_missed_tick_behavior(MissedTick::Skip);
    assert_eq!(tick_now(&mut timer, &mc), at_ms(107), "the first tick comes one period after creation");
    assert_eq!(clock.mono(), at_ms(107));
    mc.advance(ms(250));
    assert_eq!(tick_now(&mut timer, &mc), at_ms(207), "after an overrun it fires once, late");
    assert_eq!(clock.mono(), at_ms(357));
    assert_eq!(tick_now(&mut timer, &mc), at_ms(407), "then stays on the original phase");
    assert_eq!(clock.mono(), at_ms(407));
}

#[test]
fn interval_at_first_ticks_at_start() {
    let (clock, mc) = Clock::manual(Time::default());
    mc.advance(ms(100));
    let mut ahead = clock.interval_at(at_ms(130), ms(50));
    let mut behind = clock.interval_at(at_ms(40), ms(50));
    assert!(poll_once(&mut pin!(ahead.tick())).is_pending());
    assert_eq!(poll_once(&mut pin!(behind.tick())), Poll::Ready(at_ms(40)), "a start already passed is due at once");
    assert_eq!(tick_now(&mut ahead, &mc), at_ms(130));
    assert_eq!(clock.mono(), at_ms(130));
}

#[test]
fn an_interval_reports_its_period_and_behavior_and_resets_one_period_from_now() {
    let (clock, mc) = Clock::manual(Time::default());
    let mut interval = clock.interval(ms(10));
    assert_eq!((interval.period(), interval.missed_tick_behavior()), (ms(10), MissedTick::Burst));
    interval.set_missed_tick_behavior(MissedTick::Skip);
    assert_eq!(interval.missed_tick_behavior(), MissedTick::Skip);

    assert_eq!(tick_now(&mut interval, &mc), Mono::from_ns(0));
    mc.advance(ms(3));
    interval.reset();
    assert_eq!(mc.next_deadline(), Some(at_ms(13)));
    assert_eq!(tick_now(&mut interval, &mc), at_ms(13));
}

#[test]
fn a_tick_dropped_before_it_completes_consumes_nothing() {
    let (clock, mc) = Clock::manual(Time::default());
    let mut interval = clock.interval(ms(10));
    assert_eq!(tick_now(&mut interval, &mc), Mono::from_ns(0));
    for _ in 0..3 {
        assert!(poll_once(&mut pin!(interval.tick())).is_pending(), "lost a select");
    }
    mc.advance(ms(10));
    assert_eq!(poll_once(&mut pin!(interval.tick())), Poll::Ready(at_ms(10)));
}

#[test]
#[should_panic(expected = "`period` must be non-zero")]
fn a_zero_period_panics_like_tokio() {
    let (clock, _mc) = Clock::manual(Time::default());
    let _ = clock.interval(Duration::ZERO);
}

#[test]
#[should_panic(expected = "`period` must be non-zero")]
fn a_zero_period_panics_on_the_system_clock_too() {
    let _ = Clock::system().interval_at(Mono::from_ns(0), Duration::ZERO);
}

// ---- Timeout ----

#[test]
fn a_manual_timeout_returns_the_output_of_a_future_that_finishes_first() {
    let (clock, mc) = Clock::manual(Time::default());
    let mut timeout = clock.timeout(ms(10), clock.sleep(ms(5)));
    assert!(poll_once(&mut timeout).is_pending());
    assert_eq!(mc.advance_to_next(), Some(at_ms(5)), "the future's own deadline");
    assert_eq!(poll_once(&mut timeout), Poll::Ready(Ok(())));
    assert_eq!(mc.next_deadline(), None, "a completed timeout is no longer pending, dropped or not");
}

#[test]
fn a_manual_timeout_elapses_when_its_deadline_comes_first() {
    let (clock, mc) = Clock::manual(Time::default());
    let mut timeout = clock.timeout(ms(10), clock.sleep(ms(30)));
    assert!(poll_once(&mut timeout).is_pending());
    assert_eq!(mc.advance_to_next(), Some(at_ms(10)), "advance_to_next stops at the timeout's deadline");
    let Poll::Ready(result) = poll_once(&mut timeout) else { panic!("the deadline has come") };
    assert_eq!(result, Err(Elapsed));
    assert_eq!(mc.next_deadline(), Some(at_ms(30)), "the future given up on lives as long as the timeout");
    drop(timeout);
    assert_eq!(mc.next_deadline(), None);

    // `?` into a boxed error, as call sites do
    let boxed = |result: Result<(), Elapsed>| -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        result?;
        Ok(())
    };
    assert_eq!(boxed(result).unwrap_err().to_string(), "deadline has elapsed");
}

#[test]
fn when_the_future_and_the_deadline_are_both_ready_the_future_wins() {
    let (clock, mc) = Clock::manual(Time::default());
    assert_eq!(poll_once(&mut clock.timeout(Duration::ZERO, futures::future::ready(7))), Poll::Ready(Ok(7)));
    let mut both = clock.timeout(ms(10), clock.sleep(ms(10)));
    assert!(poll_once(&mut both).is_pending());
    mc.advance(ms(10));
    assert_eq!(poll_once(&mut both), Poll::Ready(Ok(())));

    // the system clock decides both without a timer, so without a runtime
    let system = Clock::system();
    assert_eq!(poll_once(&mut system.timeout(Duration::ZERO, futures::future::ready(7))), Poll::Ready(Ok(7)));
    assert_eq!(poll_once(&mut system.timeout_at(Mono::from_ns(0), futures::future::ready(8))), Poll::Ready(Ok(8)));
    assert_eq!(poll_once(&mut system.timeout(Duration::ZERO, futures::future::pending::<()>())), Poll::Ready(Err(Elapsed)));
}

#[tokio::test]
async fn a_system_timeout_returns_whichever_of_future_and_deadline_comes_first() {
    let clock = Clock::system();
    // the future first; an async block, so not Unpin
    let sleep = clock.sleep(ms(10));
    assert_eq!(
        clock
            .timeout(ms(500), async move {
                sleep.await;
                5
            })
            .await,
        Ok(5)
    );

    let start = clock.mono();
    assert_eq!(clock.timeout(ms(20), futures::future::pending::<()>()).await, Err(Elapsed));
    assert!(clock.mono().saturating_duration_since(start) >= ms(20));

    // both ready by the first poll: the future wins
    let both = clock.timeout(ms(10), clock.sleep(ms(10)));
    clock.sleep(ms(20)).await;
    assert_eq!(both.await, Ok(()));
}

#[test]
fn dropping_a_timeout_deregisters_its_deadline() {
    let (clock, mc) = Clock::manual(Time::default());
    let unpolled = clock.timeout(ms(10), futures::future::pending::<()>());
    assert_eq!(mc.next_deadline(), Some(at_ms(10)), "pending from creation");
    drop(unpolled);
    assert_eq!(mc.next_deadline(), None);

    let mut polled = clock.timeout(ms(10), futures::future::pending::<()>());
    assert!(poll_once(&mut polled).is_pending());
    drop(polled);
    assert_eq!(mc.next_deadline(), None);
    assert_eq!(mc.advance_to_next(), None);
}

/// Review 2026-10-26 (a stress test that passed: evidence for "nothing
/// found"). Task and thread sleepers on a manual clock race two threads that
/// move it: every one of them wakes, and none before its deadline.
#[test]
fn manual_sleepers_racing_concurrent_moves_all_wake_at_or_after_their_deadline() {
    const HANG: Duration = Duration::from_secs(10);
    // time enabled only for the hang detector; the sleepers use the manual clock
    let rt = tokio::runtime::Builder::new_multi_thread().worker_threads(3).enable_time().build().unwrap();
    let (clock, mc) = Clock::manual(Time::default());
    let mut tasks = Vec::new();
    for i in 0..400u64 {
        let clock = clock.clone();
        tasks.push(rt.spawn(async move {
            let deadline = Mono::from_ns(1 + (i * 7919) % 5_000);
            if i % 3 == 0 {
                let mut it = clock.interval_at(deadline, Duration::from_nanos(97));
                for _ in 0..5 {
                    let due = it.tick().await;
                    assert!(clock.mono() >= due);
                }
            } else {
                clock.sleep_until(deadline).await;
                assert!(clock.mono() >= deadline);
            }
        }));
    }
    let threads: Vec<_> = (0..3)
        .map(|t| {
            let clock = clock.clone();
            std::thread::spawn(move || {
                let deadline = Mono::from_ns(100 + t * 1_000);
                clock.sleep_until_blocking(deadline);
                assert!(clock.mono() >= deadline);
            })
        })
        .collect();
    let movers: Vec<_> = (0..2)
        .map(|m| {
            let mc = mc.clone();
            std::thread::spawn(move || {
                for step in 0..20_000u64 {
                    if (step + m) % 2 == 0 {
                        mc.advance(Duration::from_nanos(1));
                    } else {
                        mc.advance_to_next();
                    }
                }
            })
        })
        .collect();
    for m in movers {
        m.join().unwrap();
    }
    mc.advance(Duration::from_secs(1));
    rt.block_on(async {
        for t in tasks {
            tokio::time::timeout(HANG, t).await.expect("a sleeper was never woken").unwrap();
        }
    });
    for t in threads {
        t.join().unwrap();
    }
}

#[test]
fn a_timeout_of_a_future_that_is_not_unpin_works_once_pinned() {
    let (clock, mc) = Clock::manual(Time::default());
    let sleep = clock.sleep(ms(5));
    let mut on_stack = pin!(clock.timeout(ms(10), async move {
        sleep.await;
        42
    }));
    assert!(poll_once(&mut on_stack).is_pending());
    mc.advance(ms(5));
    assert_eq!(poll_once(&mut on_stack), Poll::Ready(Ok(42)));

    let mut boxed = Box::pin(clock.timeout(ms(10), async { futures::future::pending::<()>().await }));
    assert!(poll_once(&mut boxed).is_pending());
    assert_eq!(mc.advance_to_next(), Some(at_ms(15)));
    assert_eq!(poll_once(&mut boxed), Poll::Ready(Err(Elapsed)));
}
