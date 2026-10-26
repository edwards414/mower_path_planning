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
//! - publish never waits for a reader: it holds the channel's lock, and each
//!   reader's queue lock in turn, only to queue the message. What it does
//!   with no lock held, on the publisher's thread, is the only user code it
//!   runs: waking the readers' tasks and threads, and dropping the messages
//!   it pushed out of full queues;
//! - works without any async runtime (publish from a std thread, blocking
//!   readers on std threads, async readers on any executor);
//! - `Publisher` and `Subscriber` are `Send + Sync + Unpin`; `Publisher` is
//!   `Clone` (clones are one writer, one history);
//! - dropping a `Subscriber` unregisters it and frees what it had queued at
//!   once, even while a `publisher_changes` stream of it lives on; dropping
//!   the last clone of a `Publisher` unregisters the writer and drops its
//!   history;
//! - closing the scope does both for every endpoint of the scope (via
//!   `Scope::on_close` with `Weak` references), and ends its readers' streams
//!   at once, discarding what they still had queued. A reader's close does
//!   the bus's part (closed, unregistered, its watches taken out) before any
//!   user code, so that a waker or drop glue that panics leaves no closed
//!   reader registered and no watch asleep.
//!
//! How:
//! - a message travels as the `AnyMsg` its publisher made, so typed readers
//!   `downcast` it back without a copy and erased readers wrap the same `Arc`
//!   with the channel's codec;
//! - a publish takes the channel's lock, numbers the message, retains it if
//!   its writer is transient-local and pushes it into every reader's inbox
//!   (each behind its own short lock). Pushing under the channel's lock is
//!   what gives every reader the same order across publishing threads, and
//!   what lets a late joiner take history and register without missing or
//!   repeating a message. Wake-ups, and freeing the messages that full
//!   inboxes pushed out, wait until the publish holds no lock, and each is a
//!   call of its own (`crate::wake::UserCode`): one that panics skips none
//!   of the others;
//! - a reader can count the channel's live writers and watch them come and
//!   go (`publisher_count`, `publisher_changes`); a writer coming or going
//!   wakes the watches, which costs publishing nothing. A writer can do the
//!   same for the readers (`subscription_count`, `subscription_changes`);
//! - lock order: registry, then channel, then inbox. Readers only ever take
//!   their own inbox's lock.

use std::collections::{BTreeMap, VecDeque};
use std::fmt;
use std::marker::PhantomData;
use std::pin::Pin;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Condvar, Mutex, MutexGuard, OnceLock, PoisonError, Weak};
use std::task::{Context, Poll, Waker};
use std::time::{Duration, Instant};

use futures::Stream;

use crate::codec::{AnyMsg, Codec};
use crate::wake::UserCode;
use crate::{ros_type_name, BusError, Message, Qos, RecvError, Scope, TopicInfo};

/// The topic half of the bus registry: one channel per name.
///
/// Channels are never removed: a name's type stays fixed for the bus's
/// lifetime (README.md, "Type checking"), and a process uses a fixed set of
/// names.
#[derive(Default)]
pub(crate) struct Registry {
    channels: Mutex<BTreeMap<String, Arc<Channel>>>,
    /// Writer ids, unique per bus: how a publish finds its writer's history.
    next_writer: AtomicU64,
}

impl Registry {
    pub(crate) fn info(&self) -> Vec<TopicInfo> {
        // The map is sorted by name. Copy it out so that a busy channel does
        // not hold up endpoint creation on other names.
        let channels: Vec<Arc<Channel>> = lock(&self.channels).values().cloned().collect();
        channels.iter().map(|channel| channel.info()).collect()
    }

    fn channel(&self, name: &str) -> Arc<Channel> {
        let mut channels = lock(&self.channels);
        if let Some(channel) = channels.get(name) {
            return Arc::clone(channel);
        }
        let channel = Arc::new(Channel::new(name));
        channels.insert(name.to_string(), Arc::clone(&channel));
        channel
    }
}

pub(crate) fn publisher<T: Message>(scope: &Scope, name: &str, qos: Qos) -> Result<Publisher<T>, BusError> {
    let writer = Writer::open(scope, name, qos, Some(Codec::of::<T>()))?;
    Ok(Publisher { writer, _t: PhantomData })
}

pub(crate) fn subscribe<T: Message>(scope: &Scope, name: &str, qos: Qos) -> Result<Subscriber<T>, BusError> {
    let reader = Reader::open(scope, name, qos, Some(Codec::of::<T>()))?;
    Ok(Subscriber { reader, _t: PhantomData })
}

/// A writer on one topic. Clones are the same writer and share its history.
pub struct Publisher<T: Message> {
    writer: Arc<Writer>,
    _t: PhantomData<fn() -> T>,
}

impl<T: Message> Clone for Publisher<T> {
    fn clone(&self) -> Self {
        Publisher { writer: Arc::clone(&self.writer), _t: PhantomData }
    }
}

impl<T: Message> Publisher<T> {
    pub fn name(&self) -> &str {
        self.writer.channel.name()
    }

    pub fn qos(&self) -> Qos {
        self.writer.qos
    }

    /// Clone `msg` once into an `Arc` and deliver it. Never waits for a
    /// reader (README.md, "`publish`"); fails only with [`BusError::Closed`].
    pub fn publish(&self, msg: &T) -> Result<(), BusError> {
        self.publish_arc(Arc::new(msg.clone()))
    }

    pub fn publish_owned(&self, msg: T) -> Result<(), BusError> {
        self.publish_arc(Arc::new(msg))
    }

    pub fn publish_arc(&self, msg: Arc<T>) -> Result<(), BusError> {
        self.writer.publish(msg)
    }

    /// Live readers of the channel, typed and erased.
    pub fn subscription_count(&self) -> usize {
        self.writer.channel.subscription_count()
    }

    /// The channel's live reader count each time readers have come or gone,
    /// the mirror of [`Subscriber::publisher_changes`]: an item means at
    /// least one did since the previous item (or since the stream was made),
    /// and several changes in between give one item. The stream ends when
    /// this publisher's scope closes, also after the publisher is dropped.
    ///
    /// Make it before reading [`Publisher::subscription_count`] for the
    /// starting state, so that no change falls in between. Costs nothing on
    /// the publish path.
    pub fn subscription_changes(&self) -> SubscriptionChanges {
        self.writer.watch_subscriptions()
    }
}

impl<T: Message> fmt::Debug for Publisher<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Publisher")
            .field("name", &self.name())
            .field("type", &std::any::type_name::<T>())
            .finish_non_exhaustive()
    }
}

/// A reader of one topic with its own KEEP_LAST queue.
pub struct Subscriber<T: Message> {
    reader: Reader,
    _t: PhantomData<fn() -> T>,
}

impl<T: Message> Subscriber<T> {
    pub fn name(&self) -> &str {
        self.reader.channel.name()
    }

    pub fn qos(&self) -> Qos {
        self.reader.qos
    }

    /// The next message; `None` once the scope is closed.
    pub async fn recv(&mut self) -> Option<Arc<T>> {
        std::future::poll_fn(|cx| self.reader.poll_recv(cx)).await.map(downcast::<T>)
    }

    /// The next queued message without waiting: `Empty` or `Closed` if none.
    pub fn try_recv(&mut self) -> Result<Arc<T>, RecvError> {
        self.reader.try_recv().map(downcast::<T>)
    }

    /// Block the calling thread up to `timeout` for the next message:
    /// `Timeout` or `Closed` if none. For dedicated std threads; never call
    /// it from async code.
    pub fn recv_timeout(&mut self, timeout: Duration) -> Result<Arc<T>, RecvError> {
        self.reader.recv_timeout(timeout).map(downcast::<T>)
    }

    /// Drain the queue and return the newest message in it, if any. Does not
    /// wait. The skipped ones do not count as dropped.
    pub fn latest(&mut self) -> Option<Arc<T>> {
        self.reader.latest().map(downcast::<T>)
    }

    /// Messages lost to a full queue since this reader was created.
    pub fn dropped(&self) -> u64 {
        self.reader.dropped()
    }

    pub fn is_closed(&self) -> bool {
        self.reader.is_closed()
    }

    /// Live writers on the channel, typed and JSON: one per writer (a
    /// publisher and its clones count once), none whose scope has closed.
    /// rclcpp's `get_publisher_count()`.
    pub fn publisher_count(&self) -> usize {
        self.reader.channel.publisher_count()
    }

    /// The channel's live writer count each time writers have come or gone:
    /// an item means at least one did since the previous item (or since the
    /// stream was made), and several changes in between give one item. A
    /// module restart can therefore give an item with the count unchanged.
    /// The stream ends when this reader's scope closes.
    ///
    /// It is a separate object, so it can be polled next to `recv` (in one
    /// `select!`) and can outlive the subscriber. Make it before reading
    /// [`Subscriber::publisher_count`] for the starting state, so that no
    /// change falls in between. Costs nothing on the publish path.
    pub fn publisher_changes(&self) -> PublisherChanges {
        self.reader.watch_publishers()
    }
}

impl<T: Message> Stream for Subscriber<T> {
    type Item = Arc<T>;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Arc<T>>> {
        self.get_mut().reader.poll_recv(cx).map(|msg| msg.map(downcast::<T>))
    }
}

impl<T: Message> fmt::Debug for Subscriber<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Subscriber")
            .field("name", &self.name())
            .field("type", &std::any::type_name::<T>())
            .finish_non_exhaustive()
    }
}

/// Only `T`s reach a `Subscriber<T>`: the channel's type was `T` when the
/// subscriber was created (fixed or checked), it never changes, and every
/// writer publishes that type (a `JsonPublisher` decodes with its codec).
fn downcast<T: Message>(msg: AnyMsg) -> Arc<T> {
    msg.downcast::<T>().expect("a channel carries only the type it was fixed to")
}

/// The channel an endpoint of `scope` on `name` attaches to, created by the
/// first endpoint of any kind. A typed endpoint brings its codec: the first
/// one fixes the channel's type, any later one must match it.
fn open_channel(scope: &Scope, name: &str, codec: Option<Codec>) -> Result<Arc<Channel>, BusError> {
    crate::check_name(name)?;
    // Before the channel exists, so that a closed scope leaves no trace. A
    // close racing with the creation is caught by `Scope::on_close`.
    if scope.is_closed() {
        return Err(BusError::Closed { scope: scope.name().to_string() });
    }
    let channel = scope.bus().topic_registry().channel(name);
    if let Some(codec) = codec {
        channel.fix_type(codec)?;
    }
    Ok(channel)
}

/// One topic: its type, its readers' inboxes, its writers' history and the
/// counters introspection reports.
pub(crate) struct Channel {
    name: String,
    /// Fixed by the first typed endpoint, never changed afterwards. Read
    /// without the lock: erased readers wrap every message with it.
    codec: OnceLock<Codec>,
    state: Mutex<ChannelState>,
}

#[derive(Default)]
struct ChannelState {
    readers: Vec<Arc<Inbox>>,
    writers: Vec<WriterEntry>,
    /// Messages published so far. The count at a publish numbers its message
    /// in the channel's order, which is how history from several writers is
    /// merged.
    published: u64,
    last_published: Option<Instant>,
    /// Writers registered or unregistered so far: what a `PublisherChanges`
    /// compares with the count it saw last.
    writer_changes: u64,
    /// Watches waiting for the next writer change (which wakes and removes
    /// them all) or for their reader's close.
    watchers: Vec<Watcher>,
    next_watch: u64,
    /// Readers registered or unregistered so far: what a
    /// `SubscriptionChanges` compares with the count it saw last.
    reader_changes: u64,
    /// `SubscriptionChanges` waiting, by id: woken and removed all by the
    /// next reader change, and by any writer's close (each looks whether it
    /// was its own).
    reader_watchers: Vec<(u64, Waker)>,
}

/// A `PublisherChanges` that waits.
struct Watcher {
    id: u64,
    /// The reader the watch belongs to: closing it ends the watch.
    inbox: Weak<Inbox>,
    waker: Waker,
}

impl ChannelState {
    /// A writer came or went: wake every waiting watch.
    fn writers_changed(&mut self, after: &mut AfterUnlock) {
        self.writer_changes += 1;
        for watcher in self.watchers.drain(..) {
            after.wake.push(Wake::Task(watcher.waker));
        }
    }

    /// Store watch `id`'s waker; the one it replaces is returned, to be
    /// dropped with no lock held.
    fn register_watcher(&mut self, id: u64, inbox: &Arc<Inbox>, waker: &Waker) -> Option<Waker> {
        match self.watchers.iter_mut().find(|watcher| watcher.id == id) {
            Some(watcher) if watcher.waker.will_wake(waker) => None,
            Some(watcher) => Some(std::mem::replace(&mut watcher.waker, waker.clone())),
            None => {
                self.watchers.push(Watcher { id, inbox: Arc::downgrade(inbox), waker: waker.clone() });
                None
            }
        }
    }

    fn take_watcher(&mut self, id: u64) -> Option<Waker> {
        let i = self.watchers.iter().position(|watcher| watcher.id == id)?;
        Some(self.watchers.swap_remove(i).waker)
    }

    /// A reader came or went: wake every waiting subscription watch.
    fn readers_changed(&mut self, wake: &mut Vec<Waker>) {
        self.reader_changes += 1;
        wake.extend(self.reader_watchers.drain(..).map(|(_, waker)| waker));
    }

    /// As `register_watcher`, for subscription watch `id`.
    fn register_reader_watcher(&mut self, id: u64, waker: &Waker) -> Option<Waker> {
        match self.reader_watchers.iter_mut().find(|(watcher, _)| *watcher == id) {
            Some((_, current)) if current.will_wake(waker) => None,
            Some((_, current)) => Some(std::mem::replace(current, waker.clone())),
            None => {
                self.reader_watchers.push((id, waker.clone()));
                None
            }
        }
    }

    fn take_reader_watcher(&mut self, id: u64) -> Option<Waker> {
        let i = self.reader_watchers.iter().position(|(watcher, _)| *watcher == id)?;
        Some(self.reader_watchers.swap_remove(i).1)
    }
}

/// A writer as its channel sees it.
struct WriterEntry {
    id: u64,
    /// The scope's name and the writer's QoS, for introspection.
    scope: String,
    qos: Qos,
    /// A transient-local writer's last messages with their numbers; `None`
    /// for a volatile writer, which keeps nothing.
    history: Option<History>,
}

struct History {
    depth: usize,
    msgs: VecDeque<(u64, AnyMsg)>,
}

impl Channel {
    fn new(name: &str) -> Self {
        Channel { name: name.to_string(), codec: OnceLock::new(), state: Mutex::new(ChannelState::default()) }
    }

    pub(crate) fn name(&self) -> &str {
        &self.name
    }

    /// The channel's type, once a typed endpoint has fixed it.
    pub(crate) fn codec(&self) -> Option<&Codec> {
        self.codec.get()
    }

    /// Fix the channel's type to `codec`'s if it has none yet, else check
    /// that it is the same type.
    fn fix_type(&self, codec: Codec) -> Result<(), BusError> {
        let existing = self.codec.get_or_init(|| codec);
        if existing.type_id == codec.type_id {
            Ok(())
        } else {
            Err(BusError::TypeMismatch { name: self.name.clone(), existing: existing.type_name, requested: codec.type_name })
        }
    }

    fn subscription_count(&self) -> usize {
        lock(&self.state).readers.len()
    }

    /// Writers leave the list when their last clone drops or their scope's
    /// close reaches them, so its length is the count of live writers.
    pub(crate) fn publisher_count(&self) -> usize {
        lock(&self.state).writers.len()
    }

    fn info(&self) -> TopicInfo {
        let type_name = self.codec.get().map(|codec| codec.type_name);
        let ros_type = type_name.and_then(ros_type_name);
        let state = lock(&self.state);
        TopicInfo {
            name: self.name.clone(),
            type_name,
            ros_type,
            publishers: state.writers.iter().map(|writer| writer.scope.clone()).collect(),
            publisher_qos: state.writers.iter().map(|writer| writer.qos).collect(),
            subscribers: state.readers.iter().map(|inbox| inbox.scope.clone()).collect(),
            subscriber_qos: state.readers.iter().map(|inbox| inbox.qos).collect(),
            retained: state.writers.iter().filter_map(|writer| writer.history.as_ref()).map(|h| h.msgs.len()).sum(),
            published: state.published,
            last_published: state.last_published,
        }
    }

    /// Register a reader. A transient-local one first gets the retained
    /// history; doing both under the lock publishes take means it neither
    /// misses nor repeats a message published meanwhile.
    fn add_reader(&self, inbox: &Arc<Inbox>, transient_local: bool) {
        let mut watches = Vec::new();
        {
            let mut state = lock(&self.state);
            if transient_local {
                lock(&inbox.queue).msgs.extend(retained(&state.writers, inbox.depth));
            }
            state.readers.push(Arc::clone(inbox));
            state.readers_changed(&mut watches);
        }
        wake_all(watches);
    }

    /// The caller holds its own reference to `inbox`, so this never frees
    /// it (or the messages in it) under the lock.
    fn remove_reader(&self, inbox: &Arc<Inbox>) {
        let mut watches = Vec::new();
        {
            let mut state = lock(&self.state);
            if let Some(i) = state.readers.iter().position(|r| Arc::ptr_eq(r, inbox)) {
                state.readers.remove(i);
                state.readers_changed(&mut watches);
            }
        }
        wake_all(watches);
    }

    /// Unregister a reader whose scope closed, and take out the wakers of
    /// its waiting watches, and of the subscription watches it changes, for
    /// the caller to wake. The inbox is closed first: a watch that looks
    /// after that sees it closed, and one that looked before is registered
    /// by now and woken by the caller.
    fn close_reader(&self, inbox: &Arc<Inbox>) -> Vec<Waker> {
        let mut state = lock(&self.state);
        let mut watches = Vec::new();
        if let Some(i) = state.readers.iter().position(|r| Arc::ptr_eq(r, inbox)) {
            state.readers.remove(i);
            state.readers_changed(&mut watches);
        }
        let mut i = 0;
        while i < state.watchers.len() {
            if Weak::as_ptr(&state.watchers[i].inbox) == Arc::as_ptr(inbox) {
                watches.push(state.watchers.swap_remove(i).waker);
            } else {
                i += 1;
            }
        }
        watches
    }

    /// A writer's scope closed: every waiting subscription watch looks again,
    /// and those of that writer end.
    fn writer_closed(&self) {
        let watches: Vec<Waker> = lock(&self.state).reader_watchers.drain(..).map(|(_, waker)| waker).collect();
        wake_all(watches);
    }

    fn add_writer(&self, entry: WriterEntry) {
        let mut after = AfterUnlock::new();
        {
            let mut state = lock(&self.state);
            state.writers.push(entry);
            state.writers_changed(&mut after);
        }
        after.run();
    }

    fn remove_writer(&self, id: u64) {
        let mut after = AfterUnlock::new();
        {
            let mut state = lock(&self.state);
            if let Some(i) = state.writers.iter().position(|writer| writer.id == id) {
                let removed = state.writers.remove(i);
                // Its history is freed with the rest, with no lock held.
                for (_, msg) in removed.history.into_iter().flat_map(|history| history.msgs) {
                    after.release.push(msg);
                }
                state.writers_changed(&mut after);
            }
        }
        after.run();
    }

    /// Deliver `msg` from writer `writer`: number it, retain it if the
    /// writer keeps history and queue it for every reader, then wake the
    /// readers with no lock held. `false` if the writer is no longer
    /// registered, i.e. its scope has closed.
    fn publish(&self, writer: u64, msg: AnyMsg) -> bool {
        let now = Instant::now();
        let mut after = AfterUnlock::new();
        {
            let mut guard = lock(&self.state);
            let state = &mut *guard;
            let Some(entry) = state.writers.iter_mut().find(|w| w.id == writer) else {
                return false;
            };
            state.published += 1;
            // `now` was read before the lock: a racing publish may have
            // stored a later one.
            state.last_published = state.last_published.max(Some(now));
            if let Some(history) = &mut entry.history {
                if history.msgs.len() >= history.depth {
                    if let Some((_, oldest)) = history.msgs.pop_front() {
                        after.release.push(oldest);
                    }
                }
                history.msgs.push_back((state.published, Arc::clone(&msg)));
            }
            // Still under the channel's lock, so that every reader gets
            // publishes from different threads in the same order.
            for inbox in &state.readers {
                inbox.push(&msg, &mut after);
            }
        }
        after.run();
        true
    }
}

/// Wake each of `wakers` on its own (`UserCode`), with no lock held.
fn wake_all(wakers: Vec<Waker>) {
    if wakers.is_empty() {
        return;
    }
    let mut user = UserCode::default();
    for waker in wakers {
        user.wake(waker);
    }
    user.finish();
}

/// What a new transient-local reader of `depth` starts with: the newest
/// `depth` messages over every writer's history, oldest first in the
/// channel's publish order. What does not fit is not counted as dropped; it
/// was never queued for this reader.
fn retained(writers: &[WriterEntry], depth: usize) -> Vec<AnyMsg> {
    let mut all: Vec<&(u64, AnyMsg)> =
        writers.iter().filter_map(|writer| writer.history.as_ref()).flat_map(|history| &history.msgs).collect();
    all.sort_unstable_by_key(|(n, _)| *n);
    let skip = all.len().saturating_sub(depth);
    all[skip..].iter().map(|(_, msg)| Arc::clone(msg)).collect()
}

/// One reader's KEEP_LAST queue and what wakes it.
pub(crate) struct Inbox {
    /// The reader's scope name and QoS, for introspection.
    scope: String,
    qos: Qos,
    depth: usize,
    queue: Mutex<Queue>,
    /// Signals the thread waiting in `recv_timeout`, if there is one.
    ready: Condvar,
}

#[derive(Default)]
struct Queue {
    msgs: VecDeque<AnyMsg>,
    dropped: u64,
    closed: bool,
    /// The task that last found the queue empty. A push takes it, so a burst
    /// wakes the task once.
    waker: Option<Waker>,
    /// A thread waits on `ready`. At most one can: receiving takes `&mut`.
    /// A push clears it when it signals, so a burst signals once.
    sleeping: bool,
}

impl Inbox {
    fn new(scope: &str, qos: Qos) -> Self {
        let depth = qos.effective_depth();
        Inbox { scope: scope.to_string(), qos, depth, queue: Mutex::new(Queue::default()), ready: Condvar::new() }
    }

    /// Queue `msg`, pushing out the oldest message if the queue is full. The
    /// wake-up and the pushed-out message are left in `after`.
    fn push(self: &Arc<Self>, msg: &AnyMsg, after: &mut AfterUnlock) {
        let mut queue = lock(&self.queue);
        if queue.closed {
            return;
        }
        if queue.msgs.len() >= self.depth {
            if let Some(oldest) = queue.msgs.pop_front() {
                after.release.push(oldest);
            }
            queue.dropped += 1;
        }
        queue.msgs.push_back(Arc::clone(msg));
        if let Some(waker) = queue.waker.take() {
            after.wake.push(Wake::Task(waker));
        }
        if std::mem::take(&mut queue.sleeping) {
            after.wake.push(Wake::Thread(Arc::clone(self)));
        }
    }

    /// End the reader (its scope closed) and signal a thread waiting in
    /// `recv_timeout`. What it still had queued and the waker of the task
    /// waiting on it are handed back: freeing and waking them runs user
    /// code, which the caller runs last. `None` if it was closed already.
    fn close(&self) -> Option<Discarded> {
        let (queued, waker, sleeping) = {
            let mut queue = lock(&self.queue);
            if queue.closed {
                return None;
            }
            queue.closed = true;
            (std::mem::take(&mut queue.msgs), queue.waker.take(), std::mem::take(&mut queue.sleeping))
        };
        if sleeping {
            self.ready.notify_one();
        }
        Some(Discarded { queued, waker })
    }
}

/// What a closed reader leaves: its queued messages and its task's waker.
struct Discarded {
    queued: VecDeque<AnyMsg>,
    waker: Option<Waker>,
}

/// The receiving half shared by [`Subscriber`] and `ErasedSubscriber`:
/// messages come out as the `AnyMsg` that was published.
pub(crate) struct Reader {
    channel: Arc<Channel>,
    inbox: Arc<Inbox>,
    qos: Qos,
}

impl Reader {
    pub(crate) fn open(scope: &Scope, name: &str, qos: Qos, codec: Option<Codec>) -> Result<Reader, BusError> {
        let channel = open_channel(scope, name, codec)?;
        let inbox = Arc::new(Inbox::new(scope.name(), qos));
        channel.add_reader(&inbox, qos.is_transient_local());
        // The hook keeps the channel (the registry does anyway) but only a
        // `Weak` inbox: a reader dropped early is not kept alive by it.
        let hook = {
            let channel = Arc::clone(&channel);
            let inbox = Arc::downgrade(&inbox);
            move || {
                let Some(inbox) = inbox.upgrade() else { return };
                // The bus's part first: closed, unregistered, its watches
                // taken out. Then the user code, one call at a time, so that
                // a waker or a message's drop that panics skips nothing:
                // the watches are woken, then the reader's task, then what
                // it had queued is freed.
                let discarded = inbox.close();
                let watches = channel.close_reader(&inbox);
                let mut user = UserCode::default();
                for waker in watches {
                    user.wake(waker);
                }
                if let Some(Discarded { queued, waker }) = discarded {
                    if let Some(waker) = waker {
                        user.wake(waker);
                    }
                    for msg in queued {
                        user.run(|| drop(msg));
                    }
                }
                user.finish();
            }
        };
        let reader = Reader { channel, inbox, qos };
        // Fails only if the scope closed since `open_channel` checked;
        // dropping `reader` then unregisters it again.
        scope.on_close(Box::new(hook))?;
        Ok(reader)
    }

    pub(crate) fn channel(&self) -> &Channel {
        &self.channel
    }

    pub(crate) fn poll_recv(&mut self, cx: &mut Context<'_>) -> Poll<Option<AnyMsg>> {
        let mut queue = lock(&self.inbox.queue);
        if let Some(msg) = queue.msgs.pop_front() {
            return Poll::Ready(Some(msg));
        }
        if queue.closed {
            return Poll::Ready(None);
        }
        if queue.waker.as_ref().is_some_and(|waker| waker.will_wake(cx.waker())) {
            return Poll::Pending;
        }
        // The waker this replaces goes after the lock is released: it may
        // hold the last reference to another task.
        let replaced = queue.waker.replace(cx.waker().clone());
        drop(queue);
        drop(replaced);
        Poll::Pending
    }

    pub(crate) fn try_recv(&mut self) -> Result<AnyMsg, RecvError> {
        let mut queue = lock(&self.inbox.queue);
        match queue.msgs.pop_front() {
            Some(msg) => Ok(msg),
            None if queue.closed => Err(RecvError::Closed),
            None => Err(RecvError::Empty),
        }
    }

    pub(crate) fn recv_timeout(&mut self, timeout: Duration) -> Result<AnyMsg, RecvError> {
        // `None`: too far away to represent, so wait as long as it takes.
        let deadline = Instant::now().checked_add(timeout);
        let mut queue = lock(&self.inbox.queue);
        loop {
            if let Some(msg) = queue.msgs.pop_front() {
                return Ok(msg);
            }
            if queue.closed {
                return Err(RecvError::Closed);
            }
            let left = deadline.map(|deadline| deadline.saturating_duration_since(Instant::now()));
            if left.is_some_and(|left| left.is_zero()) {
                return Err(RecvError::Timeout);
            }
            queue.sleeping = true;
            queue = match left {
                Some(left) => self.inbox.ready.wait_timeout(queue, left).unwrap_or_else(PoisonError::into_inner).0,
                None => self.inbox.ready.wait(queue).unwrap_or_else(PoisonError::into_inner),
            };
            queue.sleeping = false;
        }
    }

    pub(crate) fn latest(&mut self) -> Option<AnyMsg> {
        let mut queue = lock(&self.inbox.queue);
        let newest = queue.msgs.pop_back()?;
        // Skipped messages are freed after the lock is released; the queue
        // keeps its buffer when there was nothing to skip.
        let skipped = if queue.msgs.is_empty() { VecDeque::new() } else { std::mem::take(&mut queue.msgs) };
        drop(queue);
        drop(skipped);
        Some(newest)
    }

    pub(crate) fn dropped(&self) -> u64 {
        lock(&self.inbox.queue).dropped
    }

    pub(crate) fn is_closed(&self) -> bool {
        lock(&self.inbox.queue).closed
    }

    /// The stream behind `publisher_changes`, starting from now.
    pub(crate) fn watch_publishers(&self) -> PublisherChanges {
        let mut state = lock(&self.channel.state);
        state.next_watch += 1;
        PublisherChanges {
            channel: Arc::clone(&self.channel),
            inbox: Arc::clone(&self.inbox),
            id: state.next_watch,
            seen: state.writer_changes,
        }
    }
}

impl Drop for Reader {
    fn drop(&mut self) {
        // Unregistered first, so that no publish queues anything after the
        // queue is emptied below.
        self.channel.remove_reader(&self.inbox);
        // A `PublisherChanges` may keep the inbox for as long as it lives:
        // what was queued (a 160 kB grid) and the waker go now, with no lock
        // held. The inbox stays open; only its scope's close ends the watch.
        let (queued, waker) = {
            let mut queue = lock(&self.inbox.queue);
            (std::mem::take(&mut queue.msgs), queue.waker.take())
        };
        drop(waker);
        drop(queued);
    }
}

/// The writer count of a reader's topic each time writers have come or gone
/// ([`Subscriber::publisher_changes`],
/// [`crate::ErasedSubscriber::publisher_changes`]): a
/// `Stream<Item = usize>` that ends when the reader's scope closes.
///
/// `Send + Sync + Unpin + 'static`, so it can be spawned or kept next to the
/// reader in a `select!`, and it can outlive the reader.
pub struct PublisherChanges {
    channel: Arc<Channel>,
    /// The reader's inbox: when its scope closes, the stream ends. Held
    /// strongly, so the stream can outlive the subscriber; the subscriber's
    /// drop empties it.
    inbox: Arc<Inbox>,
    /// Its slot in `ChannelState::watchers`.
    id: u64,
    /// `ChannelState::writer_changes` at the last item, or when it was made.
    seen: u64,
}

impl Stream for PublisherChanges {
    type Item = usize;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<usize>> {
        let this = self.get_mut();
        let stale_waker;
        let poll = {
            let mut state = lock(&this.channel.state);
            // Read under the channel's lock: see `Channel::close_reader`.
            if lock(&this.inbox.queue).closed {
                stale_waker = state.take_watcher(this.id);
                Poll::Ready(None)
            } else if state.writer_changes != this.seen {
                this.seen = state.writer_changes;
                stale_waker = state.take_watcher(this.id);
                Poll::Ready(Some(state.writers.len()))
            } else {
                stale_waker = state.register_watcher(this.id, &this.inbox, cx.waker());
                Poll::Pending
            }
        };
        drop(stale_waker);
        poll
    }
}

impl Drop for PublisherChanges {
    fn drop(&mut self) {
        let waker = lock(&self.channel.state).take_watcher(self.id);
        drop(waker);
    }
}

impl fmt::Debug for PublisherChanges {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PublisherChanges").field("name", &self.channel.name()).finish_non_exhaustive()
    }
}

/// The reader count of a writer's topic each time readers have come or gone
/// ([`Publisher::subscription_changes`]), the mirror of [`PublisherChanges`]:
/// a `Stream<Item = usize>` that ends when the writer's scope closes.
///
/// `Send + Sync + Unpin + 'static`, so it can be spawned or kept next to the
/// writer in a `select!`, and it can outlive the publisher.
pub struct SubscriptionChanges {
    channel: Arc<Channel>,
    /// The writer's closed flag: set when its scope closes, which ends the
    /// stream. Held strongly, so the stream can outlive the publisher.
    closed: Arc<AtomicBool>,
    /// Its slot in `ChannelState::reader_watchers`.
    id: u64,
    /// `ChannelState::reader_changes` at the last item, or when it was made.
    seen: u64,
}

impl Stream for SubscriptionChanges {
    type Item = usize;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<usize>> {
        let this = self.get_mut();
        let stale_waker;
        let poll = {
            let mut state = lock(&this.channel.state);
            // Read under the channel's lock: the close sets the flag before
            // it takes the lock to wake the watches.
            if this.closed.load(Ordering::Acquire) {
                stale_waker = state.take_reader_watcher(this.id);
                Poll::Ready(None)
            } else if state.reader_changes != this.seen {
                this.seen = state.reader_changes;
                stale_waker = state.take_reader_watcher(this.id);
                Poll::Ready(Some(state.readers.len()))
            } else {
                stale_waker = state.register_reader_watcher(this.id, cx.waker());
                Poll::Pending
            }
        };
        drop(stale_waker);
        poll
    }
}

impl Drop for SubscriptionChanges {
    fn drop(&mut self) {
        let waker = lock(&self.channel.state).take_reader_watcher(self.id);
        drop(waker);
    }
}

impl fmt::Debug for SubscriptionChanges {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SubscriptionChanges").field("name", &self.channel.name()).finish_non_exhaustive()
    }
}

/// The sending half shared by a [`Publisher`] or `JsonPublisher` and its
/// clones: one writer, one history. The last clone to go unregisters it.
pub(crate) struct Writer {
    channel: Arc<Channel>,
    id: u64,
    /// The scope's name, for `BusError::Closed`.
    scope: String,
    qos: Qos,
    /// Set when the scope closes, before the writer is unregistered. Shared
    /// with its `SubscriptionChanges`, which can outlive it.
    closed: Arc<AtomicBool>,
}

impl Writer {
    pub(crate) fn open(scope: &Scope, name: &str, qos: Qos, codec: Option<Codec>) -> Result<Arc<Writer>, BusError> {
        let channel = open_channel(scope, name, codec)?;
        let id = scope.bus().topic_registry().next_writer.fetch_add(1, Ordering::Relaxed);
        let history = qos.is_transient_local().then(|| History { depth: qos.effective_depth(), msgs: VecDeque::new() });
        channel.add_writer(WriterEntry { id, scope: scope.name().to_string(), qos, history });
        let closed = Arc::new(AtomicBool::new(false));
        let writer = Arc::new(Writer { channel, id, scope: scope.name().to_string(), qos, closed: Arc::clone(&closed) });
        let (weak, closed, channel) = (Arc::downgrade(&writer), Arc::downgrade(&closed), Arc::clone(&writer.channel));
        // As for readers: on failure, dropping `writer` unregisters it again.
        scope.on_close(Box::new(move || {
            // Its subscription watches end, the publisher dropped or not.
            let Some(closed) = closed.upgrade() else { return };
            closed.store(true, Ordering::Release);
            if let Some(writer) = weak.upgrade() {
                writer.channel.remove_writer(writer.id);
            }
            channel.writer_closed();
        }))?;
        Ok(writer)
    }

    pub(crate) fn channel(&self) -> &Channel {
        &self.channel
    }

    /// The stream behind `subscription_changes`, starting from now.
    pub(crate) fn watch_subscriptions(&self) -> SubscriptionChanges {
        let mut state = lock(&self.channel.state);
        state.next_watch += 1;
        SubscriptionChanges {
            channel: Arc::clone(&self.channel),
            closed: Arc::clone(&self.closed),
            id: state.next_watch,
            seen: state.reader_changes,
        }
    }

    /// `Closed` once the writer's scope has closed.
    pub(crate) fn check_open(&self) -> Result<(), BusError> {
        if self.closed.load(Ordering::Acquire) {
            Err(self.closed_error())
        } else {
            Ok(())
        }
    }

    pub(crate) fn publish(&self, msg: AnyMsg) -> Result<(), BusError> {
        self.check_open()?;
        // Unregistered: the scope closed after the check above.
        if self.channel.publish(self.id, msg) {
            Ok(())
        } else {
            Err(self.closed_error())
        }
    }

    fn closed_error(&self) -> BusError {
        BusError::Closed { scope: self.scope.clone() }
    }
}

impl Drop for Writer {
    fn drop(&mut self) {
        self.channel.remove_writer(self.id);
    }
}

/// What a publish leaves for when it holds no lock: the readers to wake, and
/// the messages it pushed out of full queues (the last reference to a big
/// one frees it, which takes a while). Waking comes first, so readers get
/// going while the rest is freed.
struct AfterUnlock {
    wake: Batch<Wake, 16>,
    release: Batch<AnyMsg, 8>,
}

enum Wake {
    Task(Waker),
    Thread(Arc<Inbox>),
}

impl AfterUnlock {
    fn new() -> Self {
        AfterUnlock { wake: Batch::new(), release: Batch::new() }
    }

    /// Each waker and each drop on its own (`UserCode`): one that panics
    /// keeps no other reader asleep. The first panic goes on afterwards, to
    /// whoever published or closed.
    fn run(self) {
        let AfterUnlock { mut wake, mut release } = self;
        let mut user = UserCode::default();
        for wake in wake.drain() {
            match wake {
                Wake::Task(waker) => user.wake(waker),
                Wake::Thread(inbox) => inbox.ready.notify_one(),
            }
        }
        for msg in release.drain() {
            user.run(|| drop(msg));
        }
        user.finish();
    }
}

/// Up to `N` values on the stack, any more on the heap: a publish to a
/// handful of readers allocates nothing for its wake-ups.
struct Batch<T, const N: usize> {
    inline: [Option<T>; N],
    len: usize,
    spill: Vec<T>,
}

impl<T, const N: usize> Batch<T, N> {
    fn new() -> Self {
        Batch { inline: [const { None }; N], len: 0, spill: Vec::new() }
    }

    fn push(&mut self, value: T) {
        if self.len < N {
            self.inline[self.len] = Some(value);
            self.len += 1;
        } else {
            self.spill.push(value);
        }
    }

    /// Everything pushed, in push order.
    fn drain(&mut self) -> impl Iterator<Item = T> + '_ {
        let len = std::mem::take(&mut self.len);
        self.inline[..len].iter_mut().filter_map(Option::take).chain(self.spill.drain(..))
    }
}

/// A poisoned lock still holds consistent data: no bus lock is held while
/// user code runs, and no update under one can panic half-way.
fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(PoisonError::into_inner)
}

#[cfg(test)]
mod tests {
    use std::cell::RefCell;
    use std::sync::Weak;
    use std::task::Wake;

    use serde::{Deserialize, Serialize};

    use super::*;
    use crate::Bus;

    #[test]
    fn a_batch_keeps_push_order_past_its_inline_capacity() {
        let mut batch: Batch<u32, 4> = Batch::new();
        for i in 0..10 {
            batch.push(i);
        }
        assert_eq!(batch.drain().collect::<Vec<_>>(), (0..10).collect::<Vec<_>>());
        assert_eq!(batch.drain().count(), 0);
        batch.push(42);
        assert_eq!(batch.drain().collect::<Vec<_>>(), [42]);
    }

    /// Records, each time it is woken, whether the channel's and the inbox's
    /// locks were free at that moment.
    struct Probe {
        channel: Arc<Channel>,
        inbox: Weak<Inbox>,
        wakes: Mutex<Vec<bool>>,
    }

    impl Wake for Probe {
        fn wake(self: Arc<Self>) {
            self.wake_by_ref();
        }

        fn wake_by_ref(self: &Arc<Self>) {
            let inbox = self.inbox.upgrade().expect("the reader is alive");
            let free = self.channel.state.try_lock().is_ok() && inbox.queue.try_lock().is_ok();
            self.wakes.lock().unwrap().push(free);
        }
    }

    #[test]
    fn a_parked_reader_is_woken_once_per_burst_with_no_lock_held() {
        let bus = Bus::new();
        let scope = bus.scope("m");
        let p = scope.publisher::<u32>("/n", Qos::default()).unwrap();
        let mut sub = scope.subscribe::<u32>("/n", Qos::default()).unwrap();
        let probe = Arc::new(Probe {
            channel: Arc::clone(&sub.reader.channel),
            inbox: Arc::downgrade(&sub.reader.inbox),
            wakes: Mutex::default(),
        });
        let waker = Waker::from(Arc::clone(&probe));
        let mut cx = Context::from_waker(&waker);

        assert!(Pin::new(&mut sub).poll_next(&mut cx).is_pending());
        p.publish(&1).unwrap();
        p.publish(&2).unwrap();
        assert_eq!(*probe.wakes.lock().unwrap(), [true]);
        assert_eq!(Pin::new(&mut sub).poll_next(&mut cx), Poll::Ready(Some(Arc::new(1))));
        assert_eq!(Pin::new(&mut sub).poll_next(&mut cx), Poll::Ready(Some(Arc::new(2))));

        // the scope's close wakes it the same way
        assert!(Pin::new(&mut sub).poll_next(&mut cx).is_pending());
        scope.close();
        assert_eq!(*probe.wakes.lock().unwrap(), [true, true]);
        assert_eq!(Pin::new(&mut sub).poll_next(&mut cx), Poll::Ready(None));
    }

    /// The channel (and inbox) whose locks `Tracked`'s drop checks.
    type Watched = (Arc<Channel>, Option<Arc<Inbox>>);

    #[test]
    fn a_waiting_publisher_watch_is_woken_with_no_lock_held_and_leaves_no_slot_behind() {
        let bus = Bus::new();
        let scope = bus.scope("m");
        let sub = scope.subscribe::<u32>("/n", Qos::default()).unwrap();
        let channel = Arc::clone(&sub.reader.channel);
        let inbox = Arc::downgrade(&sub.reader.inbox);
        let probe = Arc::new(Probe { channel: Arc::clone(&channel), inbox, wakes: Mutex::default() });
        let waker = Waker::from(Arc::clone(&probe));
        let mut cx = Context::from_waker(&waker);
        let mut watch = sub.reader.watch_publishers();
        let mut poll = |watch: &mut PublisherChanges| Pin::new(watch).poll_next(&mut cx);

        assert!(poll(&mut watch).is_pending());
        assert!(poll(&mut watch).is_pending(), "polled again: same slot");
        assert_eq!(lock(&channel.state).watchers.len(), 1);
        let p = scope.publisher::<u32>("/n", Qos::default()).unwrap();
        assert_eq!(*probe.wakes.lock().unwrap(), [true]);
        assert_eq!(poll(&mut watch), Poll::Ready(Some(1)));
        assert!(poll(&mut watch).is_pending());
        drop(p);
        assert_eq!(*probe.wakes.lock().unwrap(), [true, true]);
        assert_eq!(poll(&mut watch), Poll::Ready(Some(0)));

        // a watch dropped while it waits takes its slot with it
        let mut other = sub.reader.watch_publishers();
        assert!(poll(&mut other).is_pending());
        assert!(poll(&mut watch).is_pending());
        assert_eq!(lock(&channel.state).watchers.len(), 2);
        drop(other);
        assert_eq!(lock(&channel.state).watchers.len(), 1);

        // the reader's scope closing wakes it the same way
        scope.close();
        assert_eq!(*probe.wakes.lock().unwrap(), [true, true, true]);
        assert_eq!(poll(&mut watch), Poll::Ready(None));
        assert!(lock(&channel.state).watchers.is_empty());
    }

    #[test]
    fn a_waiting_subscription_watch_is_woken_with_no_lock_held_and_leaves_no_slot_behind() {
        let bus = Bus::new();
        let scope = bus.scope("ros_bridge");
        let p = scope.publisher::<u32>("/n", Qos::default()).unwrap();
        // a reader to lend its inbox to the probe; it is the count's 1
        let sub = scope.subscribe::<u32>("/n", Qos::default()).unwrap();
        let channel = Arc::clone(&p.writer.channel);
        let probe =
            Arc::new(Probe { channel: Arc::clone(&channel), inbox: Arc::downgrade(&sub.reader.inbox), wakes: Mutex::default() });
        let waker = Waker::from(Arc::clone(&probe));
        let mut cx = Context::from_waker(&waker);
        let mut watch = p.subscription_changes();
        let mut poll = |watch: &mut SubscriptionChanges| Pin::new(watch).poll_next(&mut cx);

        assert!(poll(&mut watch).is_pending());
        assert!(poll(&mut watch).is_pending(), "polled again: same slot");
        assert_eq!(lock(&channel.state).reader_watchers.len(), 1);
        let other = bus.scope("map_manage").subscribe::<u32>("/n", Qos::default()).unwrap();
        assert_eq!(*probe.wakes.lock().unwrap(), [true]);
        assert_eq!(poll(&mut watch), Poll::Ready(Some(2)));
        assert!(poll(&mut watch).is_pending());
        drop(other);
        assert_eq!(*probe.wakes.lock().unwrap(), [true, true]);
        assert_eq!(poll(&mut watch), Poll::Ready(Some(1)));

        // a watch dropped while it waits takes its slot with it
        let mut second = p.subscription_changes();
        assert!(poll(&mut second).is_pending());
        assert!(poll(&mut watch).is_pending());
        assert_eq!(lock(&channel.state).reader_watchers.len(), 2);
        drop(second);
        assert_eq!(lock(&channel.state).reader_watchers.len(), 1);

        // the writer's scope closing wakes it the same way, the publisher
        // gone or not
        drop(p);
        scope.close();
        assert_eq!(*probe.wakes.lock().unwrap(), [true, true, true]);
        assert_eq!(poll(&mut watch), Poll::Ready(None));
        assert!(lock(&channel.state).reader_watchers.is_empty());
    }

    thread_local! {
        static WATCHED: RefCell<Option<Watched>> = const { RefCell::new(None) };
        /// One entry per `Tracked` freed while watching: were the locks free?
        static FREED: RefCell<Vec<bool>> = const { RefCell::new(Vec::new()) };
    }

    /// A message with drop glue of its own (user code, as far as the bus
    /// knows).
    #[derive(Clone, Serialize, Deserialize)]
    struct Tracked(u32);

    impl Drop for Tracked {
        fn drop(&mut self) {
            WATCHED.with(|watched| {
                if let Some((channel, inbox)) = &*watched.borrow() {
                    let free = channel.state.try_lock().is_ok() && inbox.as_ref().is_none_or(|i| i.queue.try_lock().is_ok());
                    FREED.with(|freed| freed.borrow_mut().push(free));
                }
            });
        }
    }

    /// Publish `Tracked(0..3)` while watching, and how each one freed went.
    fn freed_while_publishing(p: &Publisher<Tracked>, channel: &Arc<Channel>, inbox: Option<&Arc<Inbox>>) -> Vec<bool> {
        WATCHED.with(|w| *w.borrow_mut() = Some((Arc::clone(channel), inbox.cloned())));
        for i in 0..3 {
            p.publish_owned(Tracked(i)).unwrap();
        }
        let unwatched = WATCHED.with(|w| w.borrow_mut().take());
        drop(unwatched);
        FREED.with(|f| f.take())
    }

    /// A waker that records, when its last clone drops, whether the
    /// channel's and the inbox's locks were free at that moment.
    struct DropProbe {
        channel: Arc<Channel>,
        inbox: Weak<Inbox>,
        freed: Arc<Mutex<Vec<bool>>>,
    }

    // Not a no-op waker in effect: the probe is its drop glue, not `wake`.
    // (`unknown_lints`: clippy before 1.99 does not know this lint.)
    #[allow(unknown_lints, clippy::manual_noop_waker)]
    impl Wake for DropProbe {
        fn wake(self: Arc<Self>) {}
    }

    impl Drop for DropProbe {
        fn drop(&mut self) {
            let inbox_free = self.inbox.upgrade().is_none_or(|inbox| inbox.queue.try_lock().is_ok());
            self.freed.lock().unwrap().push(self.channel.state.try_lock().is_ok() && inbox_free);
        }
    }

    #[test]
    fn a_dropped_reader_frees_its_queue_and_waker_with_no_lock_held_while_its_watch_lives() {
        let bus = Bus::new();
        let scope = bus.scope("ros_bridge");
        let p = scope.publisher::<Tracked>("/map_grid", Qos::default()).unwrap();

        // what it had queued
        let sub = scope.subscribe::<Tracked>("/map_grid", Qos::default()).unwrap();
        for i in 0..3 {
            p.publish_owned(Tracked(i)).unwrap();
        }
        let watch = sub.publisher_changes();
        WATCHED.with(|w| *w.borrow_mut() = Some((Arc::clone(&sub.reader.channel), Some(Arc::clone(&sub.reader.inbox)))));
        drop(sub);
        let unwatched = WATCHED.with(|w| w.borrow_mut().take());
        drop(unwatched);
        assert_eq!(FREED.with(|f| f.take()), [true, true, true], "freed at the drop, with no lock held");
        assert!(lock(&watch.inbox.queue).msgs.capacity() == 0, "the queue's buffer too");
        drop(watch);

        // the waker of a task parked on it
        let mut sub = scope.subscribe::<Tracked>("/map_grid", Qos::default()).unwrap();
        let freed = Arc::new(Mutex::new(Vec::new()));
        let probe =
            DropProbe { channel: Arc::clone(&sub.reader.channel), inbox: Arc::downgrade(&sub.reader.inbox), freed: freed.clone() };
        let waker = Waker::from(Arc::new(probe));
        assert!(Pin::new(&mut sub).poll_next(&mut Context::from_waker(&waker)).is_pending());
        drop(waker); // the inbox holds the last clone now
        let watch = sub.publisher_changes();
        drop(sub);
        assert_eq!(*freed.lock().unwrap(), [true], "dropped at the drop, with no lock held");
        assert!(!lock(&watch.inbox.queue).closed, "the watch lives on until the scope closes");
    }

    #[test]
    fn messages_pushed_out_of_full_queues_and_history_are_freed_with_no_lock_held() {
        let bus = Bus::new();
        let scope = bus.scope("m");
        // a depth-1 reader of a volatile writer: each publish pushes the
        // previous message out of the queue, which held its last reference
        let p = scope.publisher::<Tracked>("/queue", Qos::default()).unwrap();
        let sub = scope.subscribe::<Tracked>("/queue", Qos::default().keep_last(1)).unwrap();
        let freed = freed_while_publishing(&p, &p.writer.channel, Some(&sub.reader.inbox));
        assert_eq!(freed, [true, true], "Tracked(0) and Tracked(1)");
        assert_eq!(sub.dropped(), 2);

        // a latched writer nobody reads: the same out of its history
        let p = scope.publisher::<Tracked>("/history", Qos::latched()).unwrap();
        assert_eq!(freed_while_publishing(&p, &p.writer.channel, None), [true, true]);
    }
}
