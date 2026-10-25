//! Time for modules: wall (`Time`) and monotonic (`Mono`) readings, async
//! and blocking sleeps, and intervals, from either the system clock or a
//! `ManualClock` that tests and replay move by hand. Contract: README.md,
//! "Clock".
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
//!   the deadline it fired for; a zero period panics, as in tokio.

use std::future::Future;
use std::ops::{Add, Sub};
use std::pin::Pin;
use std::sync::Arc;
use std::task::{Context, Poll};
use std::time::{Duration, SystemTime};

/// Wall / ROS time: nanoseconds since the UNIX epoch (what header stamps
/// carry).
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
        let _ = (sec, nanosec);
        todo!("E1 clock")
    }

    /// To `builtin_interfaces/Time`: floor seconds and `nanosec` in
    /// `0..1_000_000_000`, also before the epoch.
    pub fn to_parts(self) -> (i32, u32) {
        todo!("E1 clock")
    }

    pub fn as_secs_f64(self) -> f64 {
        todo!("E1 clock")
    }

    pub fn from_system(t: SystemTime) -> Self {
        let _ = t;
        todo!("E1 clock")
    }

    pub fn to_system(self) -> SystemTime {
        todo!("E1 clock")
    }

    /// `self - earlier`, `None` when `earlier` is later.
    pub fn duration_since(self, earlier: Time) -> Option<Duration> {
        let _ = earlier;
        todo!("E1 clock")
    }
}

impl Add<Duration> for Time {
    type Output = Time;
    fn add(self, d: Duration) -> Time {
        let _ = d;
        todo!("E1 clock")
    }
}

impl Sub<Duration> for Time {
    type Output = Time;
    fn sub(self, d: Duration) -> Time {
        let _ = d;
        todo!("E1 clock")
    }
}

/// A monotonic reading: nanoseconds since the clock's origin. Replaces
/// `Instant`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Mono(u64);

impl Mono {
    pub const fn from_ns(ns: u64) -> Self {
        Mono(ns)
    }

    pub const fn ns(self) -> u64 {
        self.0
    }

    pub fn saturating_duration_since(self, earlier: Mono) -> Duration {
        let _ = earlier;
        todo!("E1 clock")
    }

    pub fn checked_duration_since(self, earlier: Mono) -> Option<Duration> {
        let _ = earlier;
        todo!("E1 clock")
    }
}

impl Add<Duration> for Mono {
    type Output = Mono;
    fn add(self, d: Duration) -> Mono {
        let _ = d;
        todo!("E1 clock")
    }
}

/// Saturates at `Mono(0)`.
impl Sub<Duration> for Mono {
    type Output = Mono;
    fn sub(self, d: Duration) -> Mono {
        let _ = d;
        todo!("E1 clock")
    }
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

/// The manual clock's shared state (implementation-defined).
struct ManualState {}

impl Clock {
    pub fn system() -> Self {
        Clock { kind: Kind::System }
    }

    /// A clock that moves only through the returned handle.
    pub fn manual(start: Time) -> (Clock, ManualClock) {
        let _ = start;
        todo!("E1 clock")
    }

    pub fn is_manual(&self) -> bool {
        matches!(self.kind, Kind::Manual(_))
    }

    pub fn now(&self) -> Time {
        todo!("E1 clock")
    }

    pub fn mono(&self) -> Mono {
        todo!("E1 clock")
    }

    pub fn sleep(&self, d: Duration) -> Sleep {
        let _ = d;
        todo!("E1 clock")
    }

    pub fn sleep_until(&self, deadline: Mono) -> Sleep {
        let _ = deadline;
        todo!("E1 clock")
    }

    /// For dedicated std threads; never call it from async code.
    pub fn sleep_blocking(&self, d: Duration) {
        let _ = d;
        todo!("E1 clock")
    }

    /// For dedicated std threads; never call it from async code.
    pub fn sleep_until_blocking(&self, deadline: Mono) {
        let _ = deadline;
        todo!("E1 clock")
    }

    pub fn interval(&self, period: Duration) -> Interval {
        let _ = period;
        todo!("E1 clock")
    }

    pub fn interval_at(&self, start: Mono, period: Duration) -> Interval {
        let _ = (start, period);
        todo!("E1 clock")
    }
}

impl std::fmt::Debug for Clock {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(if self.is_manual() { "Clock::Manual" } else { "Clock::System" })
    }
}

/// The handle that moves a manual clock. Cheap to clone.
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
        let _ = d;
        todo!("E1 clock")
    }

    /// Move to `t` (no-op if `t` is not later than now), waking what is due.
    pub fn advance_to(&self, t: Mono) {
        let _ = t;
        todo!("E1 clock")
    }

    /// Jump to the earliest pending sleeper / interval deadline and return
    /// it; `None` (and no move) when nothing is pending.
    pub fn advance_to_next(&self) -> Option<Mono> {
        todo!("E1 clock")
    }

    pub fn next_deadline(&self) -> Option<Mono> {
        todo!("E1 clock")
    }

    /// Step wall time (as NTP would); mono is untouched, so sleepers are not
    /// woken.
    pub fn set_wall(&self, t: Time) {
        let _ = t;
        todo!("E1 clock")
    }
}

/// A sleep on a [`Clock`].
pub struct Sleep {}

impl Future for Sleep {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        let _ = cx;
        todo!("E1 clock")
    }
}

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

/// Periodic deadlines on a [`Clock`].
pub struct Interval {}

impl Interval {
    /// Wait for the next deadline and return it.
    pub async fn tick(&mut self) -> Mono {
        todo!("E1 clock")
    }

    /// [`Interval::tick`] for dedicated std threads.
    pub fn tick_blocking(&mut self) -> Mono {
        todo!("E1 clock")
    }

    pub fn period(&self) -> Duration {
        todo!("E1 clock")
    }

    pub fn missed_tick_behavior(&self) -> MissedTick {
        todo!("E1 clock")
    }

    pub fn set_missed_tick_behavior(&mut self, behavior: MissedTick) {
        let _ = behavior;
        todo!("E1 clock")
    }

    /// The next deadline becomes one period from now.
    pub fn reset(&mut self) {
        todo!("E1 clock")
    }
}
