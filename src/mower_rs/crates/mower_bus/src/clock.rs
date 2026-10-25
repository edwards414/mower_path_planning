//! Time for modules: wall (`Time`) and monotonic (`Mono`) readings, async
//! and blocking sleeps, intervals and timeouts, from either the system clock
//! or a `ManualClock` that tests and replay move by hand. Contract:
//! README.md, "Clock".
//!
//! Requirements beyond the README:
//! - `Clock::system()`: wall = `SystemTime::now()`, mono = `Instant` since one
//!   process-wide origin (so `Mono`s from any two system clocks compare);
//!   async sleeps on `tokio::time` (need a runtime with time enabled),
//!   blocking ones on `std::thread::sleep`;
//! - `Clock::manual(start)`: mono starts at `Mono(0)` and wall at `start`;
//!   both move only through the `ManualClock` (`advance*` moves both,
//!   `set_wall` steps wall only); no runtime needed; sleepers (async and
//!   blocking) and intervals whose deadline has passed wake in deadline
//!   order on every move, each observing the new time;
//! - `Sleep` is `Send + Unpin`; a `Sleep` or `Interval` whose deadline has
//!   already passed is ready on its first poll;
//! - `Interval`: `interval(p)` ticks at once then every `p`
//!   (`MissedTick::Burst` default, tokio's semantics for Burst / Delay /
//!   Skip); `interval_at(start, p)` first ticks at `start`; `tick()` returns
//!   the deadline it fired for; a zero period panics, as in tokio;
//! - `Timeout` (`tokio::time::timeout` on a clock): polls the future first,
//!   so its output wins when both are ready; `Unpin` / `Send` when the
//!   future is; its deadline is pending from creation until it completes or
//!   drops.
//!
//! How the manual clock keeps this: one mutex over both readings and the
//! pending deadlines, in (deadline, registration) order. A `Sleep` (an
//! `Interval` and a `Timeout` hold one) registers when it is created or
//! re-armed, not on its first poll, so `ManualClock::next_deadline` knows a
//! deadline from the moment it exists; it deregisters when it drops. A move
//! takes the due entries out under the lock and wakes them once the lock is
//! released: tasks through the `Waker` of their last poll, std threads
//! through the condvar they wait on.

use std::collections::BTreeMap;
use std::fmt;
use std::future::Future;
use std::ops::{Add, Sub};
use std::pin::Pin;
use std::sync::{Arc, Condvar, Mutex, MutexGuard, OnceLock};
use std::task::{ready, Context, Poll, Waker};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

const NANOS_PER_SEC: i64 = 1_000_000_000;

/// Wall / ROS time: nanoseconds since the UNIX epoch (what header stamps
/// carry).
///
/// Arithmetic saturates at the ends of the `i64` range (1677-09-21 and
/// 2262-04-11) instead of panicking or wrapping: no real stamp comes near
/// them, and a garbage stamp must not take the daemon down.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Time(i64);

impl Time {
    pub const fn from_ns(ns: i64) -> Self {
        Time(ns)
    }

    pub const fn ns(self) -> i64 {
        self.0
    }

    /// From `builtin_interfaces/Time` (`nanosec` above 1e9 carries over).
    pub fn from_parts(sec: i32, nanosec: u32) -> Self {
        // Under 2^31 s plus under 2^32 ns: far inside an i64 of nanoseconds.
        Time(sec as i64 * NANOS_PER_SEC + nanosec as i64)
    }

    /// To `builtin_interfaces/Time`: floor seconds and `nanosec` in
    /// `0..1_000_000_000`, also before the epoch.
    ///
    /// Outside what an `i32` of seconds holds (1901-12-13 to 2038-01-19) it
    /// saturates at the first or last instant that fits; the `as i32` used so
    /// far would wrap a 2039 stamp to 1903.
    pub fn to_parts(self) -> (i32, u32) {
        let sec = self.0.div_euclid(NANOS_PER_SEC);
        let nanosec = self.0.rem_euclid(NANOS_PER_SEC) as u32;
        match i32::try_from(sec) {
            Ok(sec) => (sec, nanosec),
            Err(_) if sec > 0 => (i32::MAX, 999_999_999),
            Err(_) => (i32::MIN, 0),
        }
    }

    /// `rclcpp::Time::seconds()`: the nanosecond count divided by 1e9 in
    /// double, so ported code computes the same bits (see mower_base_core).
    pub fn as_secs_f64(self) -> f64 {
        self.0 as f64 / 1e9
    }

    /// Saturates outside the `i64` range.
    pub fn from_system(t: SystemTime) -> Self {
        match t.duration_since(UNIX_EPOCH) {
            Ok(after) => Time(0) + after,
            Err(before) => Time(0) - before.duration(),
        }
    }

    /// Exact on Linux and macOS, whose `SystemTime` holds nanoseconds over a
    /// wider range than `Time`.
    pub fn to_system(self) -> SystemTime {
        let d = Duration::from_nanos(self.0.unsigned_abs());
        if self.0 >= 0 {
            UNIX_EPOCH + d
        } else {
            UNIX_EPOCH - d
        }
    }

    /// `self - earlier`, `None` when `earlier` is later.
    pub fn duration_since(self, earlier: Time) -> Option<Duration> {
        // In i128 the difference cannot overflow, and when it is not
        // negative it fits a u64 (at most 2^64 - 1).
        let ns = self.0 as i128 - earlier.0 as i128;
        (ns >= 0).then(|| Duration::from_nanos(ns as u64))
    }
}

/// Saturates at the last `Time`.
impl Add<Duration> for Time {
    type Output = Time;
    fn add(self, d: Duration) -> Time {
        Time(i64::try_from(self.0 as i128 + d.as_nanos() as i128).unwrap_or(i64::MAX))
    }
}

/// Saturates at the first `Time`.
impl Sub<Duration> for Time {
    type Output = Time;
    fn sub(self, d: Duration) -> Time {
        Time(i64::try_from(self.0 as i128 - d.as_nanos() as i128).unwrap_or(i64::MIN))
    }
}

/// A monotonic reading: nanoseconds since the clock's origin. Replaces
/// `Instant`.
///
/// Readings of one clock compare, and so do those of any two
/// `Clock::system()` handles (one origin per process). Adding saturates at
/// 584 years, so `mono + Duration::MAX` is a deadline that never comes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Mono(u64);

impl Mono {
    pub const fn from_ns(ns: u64) -> Self {
        Mono(ns)
    }

    pub const fn ns(self) -> u64 {
        self.0
    }

    /// Zero when `earlier` is later.
    pub fn saturating_duration_since(self, earlier: Mono) -> Duration {
        Duration::from_nanos(self.0.saturating_sub(earlier.0))
    }

    /// `None` when `earlier` is later.
    pub fn checked_duration_since(self, earlier: Mono) -> Option<Duration> {
        self.0.checked_sub(earlier.0).map(Duration::from_nanos)
    }
}

/// Saturates at the last `Mono`.
impl Add<Duration> for Mono {
    type Output = Mono;
    fn add(self, d: Duration) -> Mono {
        Mono(self.0.saturating_add(nanos(d)))
    }
}

/// Saturates at `Mono(0)`.
impl Sub<Duration> for Mono {
    type Output = Mono;
    fn sub(self, d: Duration) -> Mono {
        Mono(self.0.saturating_sub(nanos(d)))
    }
}

/// `d` in nanoseconds, saturating at `u64::MAX` (584 years).
fn nanos(d: Duration) -> u64 {
    u64::try_from(d.as_nanos()).unwrap_or(u64::MAX)
}

/// Where `Clock::system()`'s `mono` counts from: its first use in the
/// process.
static ORIGIN: OnceLock<Instant> = OnceLock::new();

fn origin() -> Instant {
    *ORIGIN.get_or_init(Instant::now)
}

fn system_mono() -> Mono {
    let origin = origin();
    Mono(nanos(Instant::now().saturating_duration_since(origin)))
}

/// The furthest out a system `Sleep` arms its tokio timer, as tokio's own
/// `sleep` does (`Instant::far_future`): a `Mono` reaches 584 years past the
/// origin, while an `Instant` overflows sooner on some systems.
const FAR_FUTURE: Duration = Duration::from_secs(86_400 * 365 * 30);

/// `deadline` on tokio's timeline, which is std's `Instant` when time is not
/// paused.
fn tokio_instant(deadline: Mono) -> tokio::time::Instant {
    let far = Instant::now() + FAR_FUTURE;
    let at = origin().checked_add(Duration::from_nanos(deadline.0)).filter(|at| *at < far);
    tokio::time::Instant::from_std(at.unwrap_or(far))
}

/// The system clock or a manual one. Cheap to clone.
#[derive(Clone)]
pub struct Clock {
    kind: Kind,
}

#[derive(Clone)]
enum Kind {
    System,
    Manual(Arc<ManualState>),
}

/// A manual clock's shared state: the readings and every pending deadline
/// under one lock, and the condvar its blocking sleepers wait on.
struct ManualState {
    inner: Mutex<Manual>,
    threads: Condvar,
}

struct Manual {
    mono: Mono,
    wall: Time,
    /// Everything waiting, keyed by (deadline, registration number): the
    /// order to wake in. Every deadline here is later than `mono`.
    pending: BTreeMap<(Mono, u64), Waiter>,
    registrations: u64,
}

enum Waiter {
    /// A `Sleep`, an `Interval`'s included, with the waker of its last poll.
    Task(Option<Waker>),
    /// A std thread in `sleep_until_blocking`, waiting on `threads`.
    Thread,
}

impl ManualState {
    fn lock(&self) -> MutexGuard<'_, Manual> {
        self.inner.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Move to `target` if it is later, then wake what is due in deadline
    /// order. Waking happens after `inner` is released, because a waker may
    /// run anything, this clock included.
    fn move_to(&self, mut inner: MutexGuard<'_, Manual>, target: Mono) {
        if target <= inner.mono {
            return;
        }
        inner.wall = inner.wall + Duration::from_nanos(target.0 - inner.mono.0);
        inner.mono = target;
        let mut due = Vec::new();
        while let Some(entry) = inner.pending.first_entry() {
            if entry.key().0 > target {
                break;
            }
            due.push(entry.remove());
        }
        drop(inner);
        let mut threads = false;
        for waiter in due {
            match waiter {
                Waiter::Task(Some(waker)) => waker.wake(),
                Waiter::Task(None) => {}
                Waiter::Thread => threads = true,
            }
        }
        if threads {
            self.threads.notify_all();
        }
    }
}

impl Manual {
    fn register(&mut self, deadline: Mono, waiter: Waiter) -> (Mono, u64) {
        self.registrations += 1;
        let key = (deadline, self.registrations);
        self.pending.insert(key, waiter);
        key
    }
}

impl Clock {
    /// Wall time from `SystemTime`, `mono` from `Instant`. Only async sleeps
    /// and ticks that have to wait need a tokio runtime (with time enabled):
    /// they arm `tokio::time` timers.
    ///
    /// Under tokio's paused time (`start_paused`), those timers follow the
    /// paused clock while `mono` and `now` stay real; tests use
    /// [`Clock::manual`] instead.
    pub fn system() -> Self {
        Clock { kind: Kind::System }
    }

    /// A clock that moves only through the returned handle: `mono` starts at
    /// `Mono(0)` and wall time at `start`. Needs no runtime.
    pub fn manual(start: Time) -> (Clock, ManualClock) {
        let state = Arc::new(ManualState {
            inner: Mutex::new(Manual { mono: Mono(0), wall: start, pending: BTreeMap::new(), registrations: 0 }),
            threads: Condvar::new(),
        });
        (Clock { kind: Kind::Manual(state.clone()) }, ManualClock { state })
    }

    pub fn is_manual(&self) -> bool {
        matches!(self.kind, Kind::Manual(_))
    }

    /// Wall / ROS time, for header stamps. It can step (NTP, `set_wall`), so
    /// periods and ages are measured on [`Clock::mono`].
    pub fn now(&self) -> Time {
        match &self.kind {
            Kind::System => Time::from_system(SystemTime::now()),
            Kind::Manual(state) => state.lock().wall,
        }
    }

    /// Monotonic time, for periods, ages and deadlines.
    pub fn mono(&self) -> Mono {
        match &self.kind {
            Kind::System => system_mono(),
            Kind::Manual(state) => state.lock().mono,
        }
    }

    pub fn sleep(&self, d: Duration) -> Sleep {
        self.sleep_until(self.mono() + d)
    }

    /// Ready once `mono()` has reached `deadline`, at once if it has already.
    pub fn sleep_until(&self, deadline: Mono) -> Sleep {
        let kind = match &self.kind {
            Kind::System => SleepKind::System { timer: None },
            Kind::Manual(state) => {
                let mut inner = state.lock();
                let key = (inner.mono < deadline).then(|| inner.register(deadline, Waiter::Task(None)));
                SleepKind::Manual { state: state.clone(), key }
            }
        };
        Sleep { deadline, kind }
    }

    /// For dedicated std threads; never call it from async code.
    pub fn sleep_blocking(&self, d: Duration) {
        self.sleep_until_blocking(self.mono() + d)
    }

    /// For dedicated std threads; never call it from async code.
    pub fn sleep_until_blocking(&self, deadline: Mono) {
        match &self.kind {
            Kind::System => loop {
                // `thread::sleep` measures on its own clock; checking `mono`
                // again is what guarantees the deadline has passed on ours.
                let now = system_mono();
                if now >= deadline {
                    return;
                }
                std::thread::sleep(deadline.saturating_duration_since(now));
            },
            Kind::Manual(state) => {
                let mut inner = state.lock();
                if inner.mono >= deadline {
                    return;
                }
                let key = inner.register(deadline, Waiter::Thread);
                while inner.mono < deadline {
                    inner = state.threads.wait(inner).unwrap_or_else(|e| e.into_inner());
                }
                // The move that woke us normally took it out already.
                inner.pending.remove(&key);
            }
        }
    }

    /// First tick at once, then every `period`, with [`MissedTick::Burst`]:
    /// tokio's `interval`. Panics if `period` is zero.
    pub fn interval(&self, period: Duration) -> Interval {
        self.interval_at(self.mono(), period)
    }

    /// First tick at `start` (at once if it has passed), then every
    /// `period`. Panics if `period` is zero.
    ///
    /// `interval_at(mono() + p, p)` with [`MissedTick::Skip`] is an rcl / r2r
    /// wall timer: the first tick one period after creation, a single late
    /// tick after an overrun, and the original phase throughout.
    pub fn interval_at(&self, start: Mono, period: Duration) -> Interval {
        assert!(period > Duration::ZERO, "`period` must be non-zero.");
        Interval { clock: self.clone(), sleep: self.sleep_until(start), period, behavior: MissedTick::default() }
    }

    /// `fut`'s output, or [`Elapsed`] if `d` passes first:
    /// `tokio::time::timeout` on this clock.
    pub fn timeout<F: Future>(&self, d: Duration, fut: F) -> Timeout<F> {
        self.timeout_at(self.mono() + d, fut)
    }

    /// `fut`'s output, or [`Elapsed`] if `mono()` reaches `deadline` first.
    pub fn timeout_at<F: Future>(&self, deadline: Mono, fut: F) -> Timeout<F> {
        Timeout { fut, sleep: self.sleep_until(deadline) }
    }
}

impl std::fmt::Debug for Clock {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(if self.is_manual() { "Clock::Manual" } else { "Clock::System" })
    }
}

/// The handle that moves a manual clock. Cheap to clone.
///
/// "Pending" below means every live [`Sleep`] and [`Interval`] of the clock
/// whose deadline is still ahead (from the moment it is created, polled or
/// not), and every thread blocked in a sleep on it.
#[derive(Clone)]
pub struct ManualClock {
    state: Arc<ManualState>,
}

impl ManualClock {
    pub fn clock(&self) -> Clock {
        Clock { kind: Kind::Manual(self.state.clone()) }
    }

    /// Move mono and wall forward by `d`, waking what is due.
    pub fn advance(&self, d: Duration) {
        let inner = self.state.lock();
        let target = inner.mono + d;
        self.state.move_to(inner, target);
    }

    /// Move to `t` (no-op if `t` is not later than now), waking what is due.
    pub fn advance_to(&self, t: Mono) {
        self.state.move_to(self.state.lock(), t);
    }

    /// Jump to the earliest pending sleeper / interval deadline and return
    /// it; `None` (and no move) when nothing is pending.
    ///
    /// Woken tasks have to run before they register their next deadline, so
    /// an event-driven test lets its tasks settle between calls.
    pub fn advance_to_next(&self) -> Option<Mono> {
        let inner = self.state.lock();
        let &(next, _) = inner.pending.keys().next()?;
        self.state.move_to(inner, next);
        Some(next)
    }

    /// The earliest pending deadline, without moving.
    pub fn next_deadline(&self) -> Option<Mono> {
        self.state.lock().pending.keys().next().map(|&(deadline, _)| deadline)
    }

    /// Step wall time (as NTP would); mono is untouched, so sleepers are not
    /// woken.
    pub fn set_wall(&self, t: Time) {
        self.state.lock().wall = t;
    }
}

impl fmt::Debug for ManualClock {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("ManualClock")
    }
}

/// A sleep on a [`Clock`].
///
/// On the system clock it arms a `tokio::time` timer the first time it is
/// polled before its deadline, which needs a tokio runtime with time
/// enabled. On a manual clock it is pending with the clock from creation
/// until it is due or dropped.
#[must_use = "futures do nothing unless you `.await` or poll them"]
pub struct Sleep {
    deadline: Mono,
    kind: SleepKind,
}

enum SleepKind {
    System {
        /// tokio's timer and the deadline it is armed for: created on the
        /// first poll that has to wait, re-armed when the deadline moves (an
        /// interval's next tick) so an interval allocates it once.
        timer: Option<(Pin<Box<tokio::time::Sleep>>, Mono)>,
    },
    Manual {
        state: Arc<ManualState>,
        /// The registration in `Manual::pending`, unless the deadline had
        /// passed. It may be gone already: a move takes out what is due.
        key: Option<(Mono, u64)>,
    },
}

impl Sleep {
    /// Move the deadline. `Interval` calls it between ticks, when no task
    /// can be waiting on this sleep (it holds `&mut Interval`).
    fn reset(&mut self, deadline: Mono) {
        self.deadline = deadline;
        if let SleepKind::Manual { state, key } = &mut self.kind {
            let mut inner = state.lock();
            let old = key.take().and_then(|key| inner.pending.remove(&key));
            *key = (inner.mono < deadline).then(|| inner.register(deadline, Waiter::Task(None)));
            drop(inner);
            drop(old); // its waker, outside the lock
        }
    }

    /// Stop waiting without completing (a `Timeout` whose future finished
    /// first): the deadline is no longer pending, and tokio's timer is freed.
    fn disarm(&mut self) {
        match &mut self.kind {
            SleepKind::System { timer } => *timer = None,
            SleepKind::Manual { state, key } => {
                if let Some(key) = key.take() {
                    let removed = state.lock().pending.remove(&key);
                    drop(removed); // its waker, after the lock
                }
            }
        }
    }
}

impl Future for Sleep {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        let this = self.get_mut();
        let deadline = this.deadline;
        match &mut this.kind {
            SleepKind::System { timer } => {
                // Checked first: tokio's timer can take a driver turn to fire
                // for a deadline that has already passed, and a sleep that
                // never waits needs no timer (nor a runtime).
                if system_mono() >= deadline {
                    return Poll::Ready(());
                }
                let (timer, armed) = timer.get_or_insert_with(|| (Box::pin(tokio::time::sleep_until(tokio_instant(deadline))), deadline));
                if *armed != deadline {
                    timer.as_mut().reset(tokio_instant(deadline));
                    *armed = deadline;
                }
                timer.as_mut().poll(cx)
            }
            SleepKind::Manual { state, key } => {
                let mut inner = state.lock();
                if inner.mono >= deadline {
                    let stale = key.take().and_then(|key| inner.pending.remove(&key));
                    drop(inner);
                    drop(stale);
                    return Poll::Ready(());
                }
                let key = *key.get_or_insert_with(|| inner.register(deadline, Waiter::Task(None)));
                let mut old = None;
                if let Waiter::Task(waker) = inner.pending.entry(key).or_insert(Waiter::Task(None)) {
                    if !waker.as_ref().is_some_and(|w| w.will_wake(cx.waker())) {
                        old = waker.replace(cx.waker().clone());
                    }
                }
                drop(inner);
                drop(old);
                Poll::Pending
            }
        }
    }
}

impl Drop for Sleep {
    fn drop(&mut self) {
        // A sleep nobody can await any more must not show up in
        // `next_deadline` (nor make `advance_to_next` stop there).
        if let SleepKind::Manual { state, key: Some(key) } = &self.kind {
            let removed = state.lock().pending.remove(key);
            drop(removed); // its waker, after the lock
        }
    }
}

impl fmt::Debug for Sleep {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Sleep").field("deadline", &self.deadline).finish_non_exhaustive()
    }
}

/// A future with a deadline on a [`Clock`] ([`Clock::timeout`]): its output,
/// or [`Elapsed`] if the deadline comes first.
///
/// The future is polled first, as in tokio, so when both are ready its
/// output wins. The deadline is pending (`ManualClock::next_deadline`) from
/// creation until the timeout completes or drops. `Unpin` and `Send` when
/// the future is.
#[must_use = "futures do nothing unless you `.await` or poll them"]
pub struct Timeout<F> {
    fut: F,
    sleep: Sleep,
}

impl<F: Future> Future for Timeout<F> {
    type Output = Result<F::Output, Elapsed>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // SAFETY: `fut` is pinned structurally, `sleep` is not. That holds
        // because `Timeout` is `Unpin` only when `F` is (the auto impl; there
        // is no manual one), has no `Drop` impl, and nothing moves `fut` out
        // of it; `sleep` is `Unpin` and only ever used through `&mut`.
        let (fut, sleep) = unsafe {
            let this = self.get_unchecked_mut();
            (Pin::new_unchecked(&mut this.fut), &mut this.sleep)
        };
        if let Poll::Ready(out) = fut.poll(cx) {
            sleep.disarm();
            return Poll::Ready(Ok(out));
        }
        Pin::new(sleep).poll(cx).map(|()| Err(Elapsed))
    }
}

impl<F> fmt::Debug for Timeout<F> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Timeout").field("deadline", &self.sleep.deadline).finish_non_exhaustive()
    }
}

/// The error of a [`Timeout`] whose deadline came first.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Elapsed;

impl fmt::Display for Elapsed {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("deadline has elapsed")
    }
}

impl std::error::Error for Elapsed {}

/// What an [`Interval`] does after missing ticks (tokio's semantics).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum MissedTick {
    /// Fire the missed ticks back to back until caught up.
    #[default]
    Burst,
    /// Fire once, then every period from that moment.
    Delay,
    /// Fire once, then stay on the original phase (an rcl wall timer).
    Skip,
}

/// Periodic deadlines on a [`Clock`]: tokio's `Interval` on either clock.
///
/// As in tokio, a tick at most 5 ms late counts as on time (the next one is
/// due a period after it); the [`MissedTick`] behaviour applies to later
/// ones.
pub struct Interval {
    clock: Clock,
    /// Armed for the next tick's deadline.
    sleep: Sleep,
    period: Duration,
    behavior: MissedTick,
}

/// tokio's `Interval::poll_tick` treats a tick this late or less as on time.
const ON_TIME: Duration = Duration::from_millis(5);

/// The deadline after the tick due at `due` that fires at `now` (tokio's
/// `poll_tick` and `MissedTickBehavior::next_timeout`).
fn next_tick(behavior: MissedTick, period: Duration, due: Mono, now: Mono) -> Mono {
    if now <= due + ON_TIME {
        return due + period;
    }
    match behavior {
        MissedTick::Burst => due + period,
        MissedTick::Delay => now + period,
        MissedTick::Skip => {
            // The first deadline of the original phase after `now`; on a
            // deadline exactly, the next one (rcl would fire at once).
            let into_period = (now.0 - due.0) as u128 % period.as_nanos();
            now + period - Duration::from_nanos(into_period as u64)
        }
    }
}

impl Interval {
    /// Wait for the next deadline and return it.
    ///
    /// Cancel-safe like tokio's: a `tick()` dropped before it completes (the
    /// losing branch of a `select!`) consumes no tick.
    pub async fn tick(&mut self) -> Mono {
        std::future::poll_fn(|cx| self.poll_tick(cx)).await
    }

    fn poll_tick(&mut self, cx: &mut Context<'_>) -> Poll<Mono> {
        ready!(Pin::new(&mut self.sleep).poll(cx));
        Poll::Ready(self.fire())
    }

    /// [`Interval::tick`] for dedicated std threads.
    pub fn tick_blocking(&mut self) -> Mono {
        self.clock.sleep_until_blocking(self.sleep.deadline);
        self.fire()
    }

    /// The tick due at the armed deadline happens now: arm the next one and
    /// return this one's deadline.
    fn fire(&mut self) -> Mono {
        let due = self.sleep.deadline;
        let next = next_tick(self.behavior, self.period, due, self.clock.mono());
        self.sleep.reset(next);
        due
    }

    pub fn period(&self) -> Duration {
        self.period
    }

    pub fn missed_tick_behavior(&self) -> MissedTick {
        self.behavior
    }

    pub fn set_missed_tick_behavior(&mut self, behavior: MissedTick) {
        self.behavior = behavior;
    }

    /// The next deadline becomes one period from now.
    pub fn reset(&mut self) {
        let next = self.clock.mono() + self.period;
        self.sleep.reset(next);
    }
}

impl fmt::Debug for Interval {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Interval")
            .field("clock", &self.clock)
            .field("next", &self.sleep.deadline)
            .field("period", &self.period)
            .field("missed_tick_behavior", &self.behavior)
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ms(n: u64) -> Mono {
        Mono(n * 1_000_000)
    }

    #[test]
    fn a_late_tick_schedules_the_next_one_by_tokios_rules() {
        let p = Duration::from_millis(10);
        for b in [MissedTick::Burst, MissedTick::Delay, MissedTick::Skip] {
            assert_eq!(next_tick(b, p, ms(10), ms(10)), ms(20), "{b:?} on time");
            assert_eq!(next_tick(b, p, ms(10), ms(15)), ms(20), "{b:?} 5 ms late still counts as on time");
        }
        assert_eq!(next_tick(MissedTick::Burst, p, ms(10), ms(37)), ms(20));
        assert_eq!(next_tick(MissedTick::Delay, p, ms(10), ms(37)), ms(47));
        assert_eq!(next_tick(MissedTick::Skip, p, ms(10), ms(37)), ms(40));
        assert_eq!(next_tick(MissedTick::Skip, p, ms(10), ms(30)), ms(40), "exactly on the phase");
    }

    #[test]
    fn far_deadlines_arm_tokio_far_ahead_instead_of_overflowing_an_instant() {
        let far = tokio_instant(Mono(u64::MAX)).into_std();
        assert!(far > Instant::now() + FAR_FUTURE - Duration::from_secs(60));
        let near = tokio_instant(system_mono() + Duration::from_millis(5)).into_std();
        assert!(near < Instant::now() + Duration::from_secs(60));
        assert_eq!(tokio_instant(Mono(0)).into_std(), origin());
    }
}
