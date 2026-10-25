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

use std::any::Any;
use std::panic::{catch_unwind, resume_unwind, AssertUnwindSafe};
use std::task::Waker;

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
