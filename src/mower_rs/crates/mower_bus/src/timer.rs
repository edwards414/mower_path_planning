//! Wall timers tied to a scope: what an r2r wall timer is to its node.
//! Contract: README.md, "Model" (closing a scope) and the cheat sheet.
//!
//! r2r's `Timer::tick()` fails once its node is dropped, and that is what
//! ends the modules' `while timer.tick().await.is_ok()` loops today. A
//! [`Timer`] keeps that: its ticks fail with `Closed` once its scope closes,
//! also while one waits, so the same loop on the bus ends when the
//! supervisor closes the instance's scope.
//!
//! How: an [`Interval`] in rcl's schedule plus a look at the scope. Each
//! poll checks `is_closed()` before the interval, which is what makes
//! `Closed` win over a tick that is due as well; a tick that waits is woken
//! by `Scope::closed`, which needs no hook in the scope and leaves nothing
//! behind when the tick is dropped.

use std::fmt;
use std::future::Future;
use std::task::Poll;
use std::time::Duration;

use crate::{BusError, Clock, Interval, MissedTick, Mono, Scope};

/// `Scope::wall_timer`: rcl's first tick one period ahead and
/// [`MissedTick::Skip`] for overruns.
pub(crate) fn wall_timer(scope: Scope, clock: &Clock, period: Duration) -> Timer {
    let mut interval = clock.interval_at(clock.mono() + period, period);
    interval.set_missed_tick_behavior(MissedTick::Skip);
    Timer { scope, interval }
}

/// A wall timer of a [`Scope`] ([`Scope::wall_timer`]): an rcl / r2r wall
/// timer whose ticks fail once the scope closes.
pub struct Timer {
    scope: Scope,
    interval: Interval,
}

impl Timer {
    /// Wait for the next tick and return its deadline. `Err(Closed)` once
    /// the scope is closed or closing, also when the close comes while this
    /// waits; when the close and the tick come together, `Closed` wins.
    ///
    /// Cancel-safe like [`Interval::tick`]: a `tick()` dropped before it
    /// completes consumes no tick.
    pub async fn tick(&mut self) -> Result<Mono, BusError> {
        let Timer { scope, interval } = self;
        let mut closed = std::pin::pin!(scope.closed());
        std::future::poll_fn(|cx| {
            if scope.is_closed() || closed.as_mut().poll(cx).is_ready() {
                return Poll::Ready(Err(scope.closed_error()));
            }
            interval.poll_tick(cx).map(Ok)
        })
        .await
    }

    pub fn period(&self) -> Duration {
        self.interval.period()
    }
}

impl fmt::Debug for Timer {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Timer").field("scope", &self.scope.name()).field("interval", &self.interval).finish()
    }
}
