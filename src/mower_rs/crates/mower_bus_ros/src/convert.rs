//! Values r2r cannot convert (README.md, "Values r2r cannot convert").
//!
//! The bus and DDS carry the same Rust type, but r2r copies every value into
//! its C message to publish, send or answer it, and asserts what the C type
//! fixes: the length of a fixed-size array, the bound of a bounded sequence,
//! a string without NUL. A value that breaks one panics inside r2r. On r2r
//! that was the publishing module's panic, and only that module restarted;
//! in a forwarder it would make `run` stop every name. So each conversion is
//! guarded, and a value r2r cannot convert is logged, counted (`invalid`)
//! and dropped.

use std::cell::Cell;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::Once;

thread_local! {
    /// Set while this thread runs a guarded conversion.
    static GUARDED: Cell<bool> = const { Cell::new(false) };
}

/// Run `f`, a conversion or a publish (which converts), and return what it
/// panicked with instead of unwinding.
///
/// The process's panic hook is not called for such a panic: a module that
/// publishes a bad value at 50 Hz would write 50 lines a second to stderr,
/// where the forwarder logs once until it works again (`Log`). The hook is
/// wrapped once, the first time a forwarder guards a conversion, and passes
/// every other panic on to the hook it replaced.
pub(crate) fn guarded<R>(f: impl FnOnce() -> R) -> Result<R, String> {
    static QUIET: Once = Once::new();
    QUIET.call_once(|| {
        let hook = std::panic::take_hook();
        std::panic::set_hook(Box::new(move |info| {
            // `try_with`: a panic in a thread-local's destructor must not
            // panic again here.
            if !GUARDED.try_with(Cell::get).unwrap_or(false) {
                hook(info);
            }
        }));
    });
    let outer = GUARDED.replace(true);
    let result = catch_unwind(AssertUnwindSafe(f));
    GUARDED.set(outer);
    result.map_err(|panic| {
        let what = panic.downcast_ref::<&str>().map(|s| s.to_string()).or_else(|| panic.downcast_ref::<String>().cloned());
        what.unwrap_or_else(|| "a panic without a message".to_string())
    })
}

/// Whether r2r can convert `value`, by converting it: for the calls in which
/// r2r converts while it holds a lock of its own (a service response, a
/// client request). A panic there would poison that lock, and the spin
/// thread would panic at its next `spin_once`.
pub(crate) fn convertible<T: r2r::WrappedTypesupport + 'static>(value: &T) -> Result<(), String> {
    guarded(|| drop(r2r::NativeMsg::<T>::from(value)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_value_r2r_cannot_convert_is_an_error_and_others_pass() {
        assert_eq!(guarded(|| 7), Ok(7));
        assert_eq!(guarded::<u32>(|| panic!("on purpose")), Err("on purpose".to_string()));
        let fix = |n| r2r::sensor_msgs::msg::NavSatFix { position_covariance: vec![0.0; n], ..Default::default() };
        assert_eq!(convertible(&fix(9)), Ok(()));
        let short = convertible(&fix(8)).unwrap_err();
        assert!(short.contains("position_covariance") && short.contains('9'), "{short}");
        let nul = r2r::std_msgs::msg::String { data: "a\0b".into() };
        assert!(convertible(&nul).is_err());
        // a guard inside a guard keeps the outer one's flag, and the flag is
        // clear again after both
        let nested = guarded(|| (guarded::<()>(|| panic!("inner")), GUARDED.with(Cell::get)));
        assert_eq!(nested, Ok((Err("inner".to_string()), true)));
        assert!(!GUARDED.with(Cell::get));
    }
}
