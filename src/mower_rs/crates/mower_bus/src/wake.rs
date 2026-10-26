//! Running code the bus does not own: the wakers of the tasks that wait on
//! it and the drop glue of the messages it frees.
//!
//! The bus often calls such code in a batch: a publish wakes every reader
//! and frees what full queues pushed out, a close wakes everyone waiting on
//! the scope's endpoints. A waker or a `Drop` that panics must not skip the
//! rest of the batch, or a task whose waker was taken out but never called
//! sleeps for good. [`UserCode`] runs each call on its own and re-raises the
//! first panic once the batch is done. Callers run it with no lock held,
//! after the bus's own state is consistent.
//!
//! [`WakerSet`] keeps the wakers of the tasks waiting for one event, each
//! under its own key, so that a task that gives up takes its waker back out.

use std::any::Any;
use std::collections::{btree_map, BTreeMap};
use std::panic::{catch_unwind, resume_unwind, AssertUnwindSafe};
use std::task::Waker;

/// Tasks waiting for one event, each under its own key, so that a waiter
/// that gives up takes its waker back out instead of leaving it to pile up,
/// and one polled again replaces its waker instead of adding one. Woken in
/// the order they started waiting.
///
/// It lives under a lock, so a waker it lets go of is handed back to be
/// dropped (or woken) once the lock is released.
#[derive(Default)]
pub(crate) struct WakerSet {
    last_key: u64,
    wakers: BTreeMap<u64, Waker>,
}

impl WakerSet {
    /// Add or refresh the waker under `key`, picking a key on first use.
    /// Returns the waker it replaces.
    #[must_use = "drop the replaced waker once the lock is released"]
    pub(crate) fn register(&mut self, key: &mut Option<u64>, waker: &Waker) -> Option<Waker> {
        let key = *key.get_or_insert_with(|| {
            self.last_key += 1;
            self.last_key
        });
        match self.wakers.entry(key) {
            btree_map::Entry::Occupied(slot) if slot.get().will_wake(waker) => None,
            btree_map::Entry::Occupied(mut slot) => Some(slot.insert(waker.clone())),
            btree_map::Entry::Vacant(slot) => {
                slot.insert(waker.clone());
                None
            }
        }
    }

    /// Take the waker under `key` out.
    #[must_use = "drop the removed waker once the lock is released"]
    pub(crate) fn remove(&mut self, key: u64) -> Option<Waker> {
        self.wakers.remove(&key)
    }

    /// Empty the set, to wake everyone once the lock is released.
    pub(crate) fn take(&mut self) -> Vec<Waker> {
        std::mem::take(&mut self.wakers).into_values().collect()
    }

    #[cfg(test)]
    pub(crate) fn is_empty(&self) -> bool {
        self.wakers.is_empty()
    }
}

/// A batch of calls into user code, each run on its own: one that panics
/// skips none of the others. [`UserCode::finish`] re-raises the first
/// panic. The payloads are kept until then, so that nothing can unwind out
/// of [`UserCode::run`] (dropping a payload can panic too).
#[derive(Default)]
#[must_use = "`finish` re-raises what the calls panicked with"]
pub(crate) struct UserCode {
    panics: Vec<Box<dyn Any + Send>>,
}

impl UserCode {
    pub(crate) fn run(&mut self, call: impl FnOnce()) {
        if let Err(panic) = catch_unwind(AssertUnwindSafe(call)) {
            self.panics.push(panic);
        }
    }

    pub(crate) fn wake(&mut self, waker: Waker) {
        self.run(|| waker.wake());
    }

    /// Re-raise the first panic of the batch, if any.
    pub(crate) fn finish(self) {
        let mut panics = self.panics.into_iter();
        if let Some(first) = panics.next() {
            drop(panics);
            resume_unwind(first);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};

    #[test]
    fn every_call_runs_and_the_first_panic_is_re_raised_at_the_end() {
        let ran = AtomicUsize::new(0);
        let mut user = UserCode::default();
        user.run(|| panic!("first"));
        user.run(|| {
            ran.fetch_add(1, Ordering::SeqCst);
        });
        user.run(|| panic!("second"));
        user.run(|| {
            ran.fetch_add(1, Ordering::SeqCst);
        });
        assert_eq!(ran.load(Ordering::SeqCst), 2, "the calls after a panic still run");
        let panic = catch_unwind(AssertUnwindSafe(|| user.finish())).expect_err("finish re-raises");
        assert_eq!(panic.downcast_ref::<&str>(), Some(&"first"));
        UserCode::default().finish(); // nothing to re-raise
    }
}
