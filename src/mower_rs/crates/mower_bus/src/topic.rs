//! Typed topics: the registry of channels, writers with their own retained
//! history, readers with their own KEEP_LAST queue, and wake-ups for async
//! (`Stream`, `recv`) and blocking (`recv_timeout`) readers.
//!
//! Contract: README.md, "Model" and "Topics". The erased endpoints in
//! `erased.rs` share these channels: a channel exists from the first endpoint
//! of any kind on its name, its codec (`crate::codec::Codec`) is set by the
//! first typed endpoint, and erased readers receive the same `Arc` (as
//! `AnyMsg`) the typed readers do.
//!
//! Requirements beyond the README:
//! - publish never blocks on a reader and never runs user code; no lock is
//!   held while waking readers;
//! - works without any async runtime (publish from a std thread, blocking
//!   readers on std threads, async readers on any executor);
//! - `Publisher` and `Subscriber` are `Send + Sync + Unpin`; `Publisher` is
//!   `Clone` (clones are one writer, one history);
//! - dropping a `Subscriber` unregisters it; dropping the last clone of a
//!   `Publisher` unregisters the writer and drops its history;
//! - closing the scope does both for every endpoint of the scope (via
//!   `Scope::on_close` with `Weak` references), and ends its readers' streams
//!   at once, discarding what they still had queued.

use std::marker::PhantomData;
use std::pin::Pin;
use std::sync::Arc;
use std::task::{Context, Poll};
use std::time::Duration;

use futures::Stream;

use crate::{BusError, Message, Qos, RecvError, Scope, TopicInfo};

/// The topic half of the bus registry.
#[derive(Default)]
pub(crate) struct Registry {}

impl Registry {
    pub(crate) fn info(&self) -> Vec<TopicInfo> {
        todo!("E1 topics")
    }
}

pub(crate) fn publisher<T: Message>(scope: &Scope, name: &str, qos: Qos) -> Result<Publisher<T>, BusError> {
    let _ = (scope, name, qos);
    todo!("E1 topics")
}

pub(crate) fn subscribe<T: Message>(scope: &Scope, name: &str, qos: Qos) -> Result<Subscriber<T>, BusError> {
    let _ = (scope, name, qos);
    todo!("E1 topics")
}

/// A writer on one topic. Clones are the same writer and share its history.
pub struct Publisher<T: Message> {
    _t: PhantomData<fn() -> T>,
}

impl<T: Message> Clone for Publisher<T> {
    fn clone(&self) -> Self {
        todo!("E1 topics")
    }
}

impl<T: Message> Publisher<T> {
    pub fn name(&self) -> &str {
        todo!("E1 topics")
    }

    pub fn qos(&self) -> Qos {
        todo!("E1 topics")
    }

    /// Clone `msg` once into an `Arc` and deliver it. Never blocks; fails
    /// only with [`BusError::Closed`].
    pub fn publish(&self, msg: &T) -> Result<(), BusError> {
        let _ = msg;
        todo!("E1 topics")
    }

    pub fn publish_owned(&self, msg: T) -> Result<(), BusError> {
        let _ = msg;
        todo!("E1 topics")
    }

    pub fn publish_arc(&self, msg: Arc<T>) -> Result<(), BusError> {
        let _ = msg;
        todo!("E1 topics")
    }

    /// Live readers of the channel, typed and erased.
    pub fn subscription_count(&self) -> usize {
        todo!("E1 topics")
    }
}

/// A reader of one topic with its own KEEP_LAST queue.
pub struct Subscriber<T: Message> {
    _t: PhantomData<fn() -> T>,
}

impl<T: Message> Subscriber<T> {
    pub fn name(&self) -> &str {
        todo!("E1 topics")
    }

    pub fn qos(&self) -> Qos {
        todo!("E1 topics")
    }

    /// The next message; `None` once the scope is closed.
    pub async fn recv(&mut self) -> Option<Arc<T>> {
        todo!("E1 topics")
    }

    /// The next queued message without waiting: `Empty` or `Closed` if none.
    pub fn try_recv(&mut self) -> Result<Arc<T>, RecvError> {
        todo!("E1 topics")
    }

    /// Block the calling thread up to `timeout` for the next message:
    /// `Timeout` or `Closed` if none. For dedicated std threads; never call
    /// it from async code.
    pub fn recv_timeout(&mut self, timeout: Duration) -> Result<Arc<T>, RecvError> {
        let _ = timeout;
        todo!("E1 topics")
    }

    /// Drain the queue and return the newest message in it, if any. Does not
    /// wait. The skipped ones do not count as dropped.
    pub fn latest(&mut self) -> Option<Arc<T>> {
        todo!("E1 topics")
    }

    /// Messages lost to a full queue since this reader was created.
    pub fn dropped(&self) -> u64 {
        todo!("E1 topics")
    }

    pub fn is_closed(&self) -> bool {
        todo!("E1 topics")
    }
}

impl<T: Message> Stream for Subscriber<T> {
    type Item = Arc<T>;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Arc<T>>> {
        let _ = cx;
        todo!("E1 topics")
    }
}
