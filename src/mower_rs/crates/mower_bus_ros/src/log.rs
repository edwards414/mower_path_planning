//! Each kind of failure is logged once, until the forwarder succeeds again:
//! a ROS client that retries a dead service twice a second must not write
//! two lines a second, but the next outage after a recovery is logged
//! again.

use std::sync::atomic::{AtomicU32, Ordering};

use mower_bus::BusError;

/// The kinds a forwarder tells apart; each is one bit of `Log::logged`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Failure {
    /// A bus call found no server, or its server went away before answering.
    BusUnavailable,
    /// The bus server dropped the request without answering.
    BusNoResponse,
    /// The bridge's own scope is closed.
    BusClosed,
    /// Any other bus error (a bug: names and types were checked at declare).
    BusOther,
    /// No DDS server matched within the probe window.
    DdsUnavailable,
    /// The DDS server went away while a call was pending.
    DdsLost,
    /// The DDS server did not answer within the call's deadline.
    DdsTimeout,
    /// An r2r call failed: creating the DDS writer, publishing, counting
    /// its readers, sending a request or a response.
    Dds,
}

impl Failure {
    pub(crate) fn of(error: &BusError) -> Failure {
        match error {
            BusError::ServiceUnavailable { .. } => Failure::BusUnavailable,
            BusError::NoResponse { .. } => Failure::BusNoResponse,
            BusError::Closed { .. } => Failure::BusClosed,
            _ => Failure::BusOther,
        }
    }

    fn bit(self) -> u32 {
        1 << (self as u32)
    }
}

/// One forwarder's logger: the node's logger, its direction and name.
pub(crate) struct Log {
    logger: String,
    what: String,
    logged: AtomicU32,
}

impl Log {
    /// `what` names the forwarder in every line, e.g. "service_to_ros /save_map".
    pub(crate) fn new(logger: &str, what: String) -> Self {
        Log { logger: logger.to_string(), what, logged: AtomicU32::new(0) }
    }

    /// A failure expected in operation (a peer is down): warn once per kind.
    pub(crate) fn warn(&self, failure: Failure, message: impl FnOnce() -> String) {
        if self.first(failure) {
            r2r::log_warn!(&self.logger, "{}: {} (logged once until it works again)", self.what, message());
        }
    }

    /// A failure that should not happen: error once per kind.
    pub(crate) fn error(&self, failure: Failure, message: impl FnOnce() -> String) {
        if self.first(failure) {
            r2r::log_error!(&self.logger, "{}: {} (logged once until it works again)", self.what, message());
        }
    }

    /// Something worked: every kind is logged again the next time.
    pub(crate) fn ok(&self) {
        // A load first: the common case is a forwarder that never failed,
        // and a store per message would bounce the cache line between the
        // tasks of a service's concurrent calls.
        if self.logged.load(Ordering::Relaxed) != 0 {
            self.logged.store(0, Ordering::Relaxed);
        }
    }

    fn first(&self, failure: Failure) -> bool {
        self.logged.fetch_or(failure.bit(), Ordering::Relaxed) & failure.bit() == 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn each_kind_is_logged_once_until_something_works() {
        let log = Log::new("test", "to_ros /x".into());
        assert!(log.first(Failure::Dds));
        assert!(!log.first(Failure::Dds));
        assert!(log.first(Failure::BusUnavailable), "another kind");
        log.ok();
        assert!(log.first(Failure::Dds));
        let unavailable = BusError::ServiceUnavailable { name: "/s".into() };
        assert_eq!(Failure::of(&unavailable), Failure::BusUnavailable);
        assert_eq!(Failure::of(&BusError::NoResponse { name: "/s".into() }), Failure::BusNoResponse);
        assert_eq!(Failure::of(&BusError::UnknownName { name: "/s".into() }), Failure::BusOther);
    }
}
