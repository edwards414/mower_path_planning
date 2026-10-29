//! The mission-operation lease of navigation_guard.py, which mower_map,
//! mower_record and mower_coverage hold while they change what navigation
//! uses (E9-07: test_qos_configuration.py checks it only on the Python
//! node). A release the nav server does not confirm is never a success, and
//! the mutation stays blocked; unknown navigation state counts as active.

use std::time::Duration;

use mower_bus::{Bus, Clock};
use mower_rs_common::guard::Guard;
use r2r::mower_interface::srv::MissionOperationLock::{Request, Response};

/// How nav_action_server's lease service answers a release.
#[derive(Clone, Copy)]
enum Release {
    Confirmed,
    Refused,
    Unanswered,
}

/// The lease service, granting every acquire, and a guard of the
/// `map_manage` scope that has seen navigation idle.
fn guard(bus: &Bus, release: Release) -> Guard {
    let mut lease = bus.scope("nav_action_server").service::<Request, Response>("/mission_operation_lock").unwrap();
    tokio::spawn(async move {
        while let Some(request) = lease.recv().await {
            let (success, message) = match (request.message.acquire, release) {
                (true, _) | (false, Release::Confirmed) => (true, ""),
                (false, Release::Refused) => (false, "lease not owned"),
                // dropped unanswered: the bus fails the call
                (false, Release::Unanswered) => continue,
            };
            let _ = request.respond(Response { success, message: message.into() });
        }
    });
    let client = bus.scope("map_manage").client::<Request, Response>("/mission_operation_lock").unwrap();
    let mut guard = Guard::on_bus("/map_manage:guard_test".into(), client, Clock::system(), "guard_test".into());
    (guard.nav_seen, guard.nav_active) = (true, false);
    guard
}

fn run(test: impl std::future::Future<Output = ()>) {
    let runtime = tokio::runtime::Builder::new_multi_thread().worker_threads(2).enable_all().build().unwrap();
    runtime.block_on(async { tokio::time::timeout(Duration::from_secs(30), test).await.expect("the test hung") });
}

#[test]
fn a_confirmed_release_frees_the_guard() {
    run(async {
        let bus = Bus::new();
        let mut guard = guard(&bus, Release::Confirmed);
        guard.acquire("rebuild free space").await.unwrap();
        assert!(guard.busy());
        assert!(guard.release().await);
        assert!(!guard.busy());
        guard.acquire("rebuild the risk map").await.unwrap();
    });
}

/// Refused or unanswered, a release is no success: the local lock stays
/// latched (no next mutation in this node) and navigation stays blocked by
/// the lease the server may still hold.
#[test]
fn an_unconfirmed_release_is_no_success_and_keeps_the_guard_locked() {
    run(async {
        for release in [Release::Refused, Release::Unanswered] {
            let bus = Bus::new();
            let mut guard = guard(&bus, release);
            guard.acquire("rebuild free space").await.unwrap();
            assert!(!guard.release().await);
            assert!(guard.busy());
            assert_eq!(guard.acquire("rebuild the risk map").await, Err("Another mutation is already active in this node".to_string()));
        }
    });
}

/// Without the nav server's activity flag nothing may change: unknown is
/// active.
#[test]
fn unknown_or_active_navigation_blocks_a_mutation() {
    run(async {
        let bus = Bus::new();
        let mut guard = guard(&bus, Release::Confirmed);
        guard.nav_seen = false;
        assert_eq!(guard.acquire("rebuild free space").await, Err("Cannot rebuild free space: navigation state is not available".to_string()));
        (guard.nav_seen, guard.nav_active) = (true, true);
        assert_eq!(guard.acquire("rebuild free space").await, Err("Cannot rebuild free space: navigation is active".to_string()));
        assert!(!guard.busy());
    });
}
