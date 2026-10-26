//! Services on the in-process bus (README.md, "Services" and "Erased
//! access"): typed calls and their error paths, what closing a scope does to
//! them, `wait_for_service`, `call_json` and introspection, on tokio and with
//! no runtime at all.

use std::any::type_name;
use std::future::Future;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::pin::Pin;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::task::{Context, Poll, Waker};
use std::thread;
use std::time::Duration;

use futures::task::ArcWake;
use futures::{FutureExt, Stream, StreamExt};
use mower_bus::{Bus, BusError, Client, Request, ResponseFuture, Service, ServiceInfo};
use serde::{Deserialize, Serialize};
use serde_json::json;

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
struct AddRequest {
    a: i64,
    b: i64,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
struct AddResponse {
    sum: i64,
}

/// Laid out like r2r's generated service types, for `ros_type`.
#[allow(non_snake_case)]
mod std_srvs {
    pub mod srv {
        pub mod Trigger {
            use serde::{Deserialize, Serialize};

            #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
            pub struct Request {}

            #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
            pub struct Response {
                pub success: bool,
                pub message: String,
            }
        }
    }
}
use std_srvs::srv::Trigger;

fn add(a: i64, b: i64) -> AddRequest {
    AddRequest { a, b }
}

fn sum(sum: i64) -> AddResponse {
    AddResponse { sum }
}

fn unavailable(name: &str) -> BusError {
    BusError::ServiceUnavailable { name: name.into() }
}

fn closed(scope: &str) -> BusError {
    BusError::Closed { scope: scope.into() }
}

/// Fail the test instead of hanging it.
async fn within<F: Future>(fut: F) -> F::Output {
    tokio::time::timeout(Duration::from_secs(5), fut).await.expect("timed out")
}

/// Answer every request with the sum, until the server's scope closes.
fn spawn_adder(mut srv: Service<AddRequest, AddResponse>) -> tokio::task::JoinHandle<()> {
    tokio::spawn(async move {
        while let Some(req) = srv.next().await {
            let answer = sum(req.message.a + req.message.b);
            let _ = req.respond(answer);
        }
    })
}

/// A waker that records being woken: checks wake-ups without timing.
#[derive(Default)]
struct Woken(AtomicBool);

impl ArcWake for Woken {
    fn wake_by_ref(arc_self: &Arc<Self>) {
        arc_self.0.store(true, Ordering::SeqCst);
    }
}

impl Woken {
    fn new() -> Arc<Self> {
        Arc::default()
    }

    /// Whether it was woken since the last look.
    fn take(&self) -> bool {
        self.0.swap(false, Ordering::SeqCst)
    }
}

fn poll_with<F: Future + Unpin>(fut: &mut F, woken: &Arc<Woken>) -> Poll<F::Output> {
    let waker = futures::task::waker(woken.clone());
    Pin::new(fut).poll(&mut Context::from_waker(&waker))
}

fn poll_next_with<S: Stream + Unpin>(stream: &mut S, woken: &Arc<Woken>) -> Poll<Option<S::Item>> {
    let waker = futures::task::waker(woken.clone());
    Pin::new(stream).poll_next(&mut Context::from_waker(&waker))
}

#[tokio::test]
async fn a_call_round_trips_to_the_server_and_back() {
    let bus = Bus::new();
    let (server, caller) = (bus.scope("adder"), bus.scope("caller"));
    let mut srv = server.service::<AddRequest, AddResponse>("/add").unwrap();
    let client = caller.client::<AddRequest, AddResponse>("/add").unwrap();
    assert_eq!((srv.name(), client.name()), ("/add", "/add"));

    let answer = client.request(&add(2, 3)).unwrap();
    let req = within(srv.recv()).await.unwrap();
    assert_eq!(req.message, add(2, 3));
    assert_eq!(req.respond(sum(5)), Ok(()));
    assert_eq!(within(answer).await, Ok(sum(5)));
}

#[test]
fn a_waiting_server_is_woken_by_a_call_and_its_stream_ends_when_its_scope_closes() {
    let bus = Bus::new();
    let (server, caller) = (bus.scope("server"), bus.scope("caller"));
    let mut srv = server.service::<AddRequest, AddResponse>("/add").unwrap();
    let client = caller.client::<AddRequest, AddResponse>("/add").unwrap();
    let woken = Woken::new();

    assert!(poll_next_with(&mut srv, &woken).is_pending());
    let mut answer = client.request(&add(1, 2)).unwrap();
    assert!(woken.take(), "a call wakes the waiting server");
    let Poll::Ready(Some(req)) = poll_next_with(&mut srv, &woken) else {
        panic!("the call is not handed out");
    };
    assert!(poll_with(&mut answer, &woken).is_pending());
    req.respond(sum(3)).unwrap();
    assert!(woken.take(), "the answer wakes the caller");
    assert_eq!(poll_with(&mut answer, &woken), Poll::Ready(Ok(sum(3))));

    assert!(poll_next_with(&mut srv, &woken).is_pending());
    server.close();
    assert!(woken.take(), "the close wakes the waiting server");
    assert!(matches!(poll_next_with(&mut srv, &woken), Poll::Ready(None)));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn several_clients_and_their_clones_each_get_their_own_answer() {
    let bus = Bus::new();
    let server = bus.scope("adder");
    let _adder = spawn_adder(server.service("/add").unwrap());
    let scopes: Vec<_> = (0..3).map(|i| bus.scope(&format!("caller{i}"))).collect();
    let clients: Vec<Client<AddRequest, AddResponse>> = scopes.iter().map(|s| s.client("/add").unwrap()).collect();

    // Every client and a clone of it, all calls in flight at once.
    let calls: Vec<_> = clients
        .iter()
        .flat_map(|client| [client.clone(), client.clone()])
        .enumerate()
        .map(|(i, client)| client.request(&add(i as i64, 100)).unwrap())
        .collect();
    let answers = within(futures::future::join_all(calls)).await;
    assert_eq!(answers, (0..6).map(|i| Ok(sum(i + 100))).collect::<Vec<_>>());
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_server_that_spawns_a_task_per_request_answers_them_concurrently() {
    const CALLS: usize = 8;
    let bus = Bus::new();
    let (server, caller) = (bus.scope("adder"), bus.scope("caller"));
    let mut srv = server.service::<AddRequest, AddResponse>("/add").unwrap();
    // No handler answers before all of them hold a request: served one at
    // a time, this would never finish.
    let all_in_hand = Arc::new(tokio::sync::Barrier::new(CALLS));
    let serving = tokio::spawn(async move {
        while let Some(req) = srv.next().await {
            let all_in_hand = all_in_hand.clone();
            tokio::spawn(async move {
                all_in_hand.wait().await;
                let answer = sum(req.message.a + req.message.b);
                req.respond(answer).unwrap();
            });
        }
    });

    let client = caller.client::<AddRequest, AddResponse>("/add").unwrap();
    let calls: Vec<_> = (0..CALLS as i64).map(|i| client.request(&add(i, i)).unwrap()).collect();
    let answers = within(futures::future::join_all(calls)).await;
    assert_eq!(answers, (0..CALLS as i64).map(|i| Ok(sum(2 * i))).collect::<Vec<_>>());
    server.close();
    within(serving).await.unwrap();
}

#[test]
fn a_second_server_is_refused_until_the_first_drops_or_its_scope_closes() {
    let bus = Bus::new();
    let (first, second, third) = (bus.scope("first"), bus.scope("second"), bus.scope("third"));
    let taken = |server: &str| BusError::ServiceTaken { name: "/add".into(), server: server.into() };

    let srv = first.service::<AddRequest, AddResponse>("/add").unwrap();
    assert_eq!(second.service::<AddRequest, AddResponse>("/add").unwrap_err(), taken("first"));
    assert_eq!(first.service::<AddRequest, AddResponse>("/add").unwrap_err(), taken("first"));
    drop(srv);

    let srv = second.service::<AddRequest, AddResponse>("/add").unwrap();
    assert_eq!(third.service::<AddRequest, AddResponse>("/add").unwrap_err(), taken("second"));
    second.close();
    let _srv = third.service::<AddRequest, AddResponse>("/add").unwrap();
    // The old server's late drop leaves the new one's name alone.
    drop(srv);
    assert_eq!(first.service::<AddRequest, AddResponse>("/add").unwrap_err(), taken("third"));
}

#[test]
fn the_first_endpoint_fixes_a_names_types_for_the_bus_lifetime() {
    let bus = Bus::new();
    let scope = bus.scope("m");
    let mismatch = |existing, requested| BusError::TypeMismatch { name: "/add".into(), existing, requested };

    // A client can be the first.
    let client = scope.client::<AddRequest, AddResponse>("/add").unwrap();
    assert_eq!(
        scope.service::<Trigger::Request, AddResponse>("/add").unwrap_err(),
        mismatch(type_name::<AddRequest>(), type_name::<Trigger::Request>()),
    );
    assert_eq!(
        scope.client::<AddRequest, Trigger::Response>("/add").unwrap_err(),
        mismatch(type_name::<AddResponse>(), type_name::<Trigger::Response>()),
    );
    // Still fixed once every endpoint is gone.
    drop(client);
    assert_eq!(
        scope.service::<Trigger::Request, Trigger::Response>("/add").unwrap_err(),
        mismatch(type_name::<AddRequest>(), type_name::<Trigger::Request>()),
    );
    assert!(scope.service::<AddRequest, AddResponse>("/add").is_ok());
}

#[tokio::test]
async fn service_names_must_be_absolute() {
    let bus = Bus::new();
    let scope = bus.scope("m");
    assert!(matches!(scope.service::<AddRequest, AddResponse>("add"), Err(BusError::InvalidName { .. })));
    assert!(matches!(scope.client::<AddRequest, AddResponse>("/add/"), Err(BusError::InvalidName { .. })));
    assert!(matches!(scope.call_json("~/add", json!({})).await, Err(BusError::InvalidName { .. })));
    assert!(bus.services().is_empty());
}

#[test]
fn endpoints_cannot_be_created_on_a_closed_scope_and_fix_no_type() {
    let bus = Bus::new();
    let gone = bus.scope("gone");
    gone.close();
    assert_eq!(gone.service::<AddRequest, AddResponse>("/add").unwrap_err(), closed("gone"));
    assert_eq!(gone.client::<AddRequest, AddResponse>("/add").unwrap_err(), closed("gone"));
    assert!(bus.services().is_empty());
    assert!(bus.scope("m").service::<Trigger::Request, Trigger::Response>("/add").is_ok());
}

#[test]
fn a_call_with_no_live_server_fails_at_once_with_service_unavailable() {
    let bus = Bus::new();
    let (server, caller) = (bus.scope("server"), bus.scope("caller"));
    let client = caller.client::<AddRequest, AddResponse>("/add").unwrap();
    assert_eq!(client.request(&add(1, 2)).unwrap_err(), unavailable("/add"));

    let srv = server.service::<AddRequest, AddResponse>("/add").unwrap();
    assert!(client.request(&add(1, 2)).is_ok());
    drop(srv);
    assert_eq!(client.request(&add(1, 2)).unwrap_err(), unavailable("/add"));
}

#[tokio::test]
async fn a_request_dropped_unanswered_fails_its_call_with_no_response() {
    let bus = Bus::new();
    let (server, caller) = (bus.scope("server"), bus.scope("caller"));
    let mut srv = server.service::<AddRequest, AddResponse>("/add").unwrap();
    let client = caller.client::<AddRequest, AddResponse>("/add").unwrap();
    let no_response = BusError::NoResponse { name: "/add".into() };

    let answer = client.request(&add(1, 2)).unwrap();
    drop(within(srv.next()).await.unwrap());
    assert_eq!(within(answer).await, Err(no_response.clone()));

    // Moving the message out drops the rest of the request, unanswered.
    let answer = client.request(&add(3, 4)).unwrap();
    let Request { message, .. } = within(srv.next()).await.unwrap();
    assert_eq!(message, add(3, 4));
    assert_eq!(within(answer).await, Err(no_response));
}

#[tokio::test]
async fn an_answer_nobody_waits_for_is_dropped_and_respond_still_succeeds() {
    let bus = Bus::new();
    let (server, caller) = (bus.scope("server"), bus.scope("caller"));
    let mut srv = server.service::<AddRequest, AddResponse>("/add").unwrap();
    let client = caller.client::<AddRequest, AddResponse>("/add").unwrap();

    drop(client.request(&add(1, 2)).unwrap());
    // The server still gets the call.
    let req = within(srv.next()).await.unwrap();
    assert_eq!(req.message, add(1, 2));
    assert_eq!(req.respond(sum(3)), Ok(()));
}

#[tokio::test]
async fn closing_the_servers_scope_fails_queued_and_handed_out_calls_and_frees_the_name() {
    let bus = Bus::new();
    let (server, caller) = (bus.scope("server"), bus.scope("caller"));
    let mut srv = server.service::<AddRequest, AddResponse>("/add").unwrap();
    let client = caller.client::<AddRequest, AddResponse>("/add").unwrap();
    let handed_out = client.request(&add(1, 1)).unwrap();
    let queued = client.request(&add(2, 2)).unwrap();
    let req = within(srv.next()).await.unwrap();
    assert_eq!(req.message, add(1, 1));

    server.close();
    assert_eq!(within(handed_out).await, Err(unavailable("/add")));
    assert_eq!(within(queued).await, Err(unavailable("/add")));
    assert!(within(srv.next()).await.is_none());
    assert_eq!(req.respond(sum(2)), Err(closed("server")));
    assert!(!client.is_available());
    assert_eq!(client.request(&add(3, 3)).unwrap_err(), unavailable("/add"));

    // The restarted module serves the name again.
    let restarted = bus.scope("server");
    let _adder = spawn_adder(restarted.service("/add").unwrap());
    assert_eq!(within(client.request(&add(3, 3)).unwrap()).await, Ok(sum(6)));
}

#[tokio::test]
async fn dropping_the_service_fails_queued_and_handed_out_calls_and_frees_the_name() {
    let bus = Bus::new();
    let (server, caller) = (bus.scope("server"), bus.scope("caller"));
    let mut srv = server.service::<AddRequest, AddResponse>("/add").unwrap();
    let client = caller.client::<AddRequest, AddResponse>("/add").unwrap();
    let answers: Vec<_> = (0..3).map(|i| client.request(&add(i, i)).unwrap()).collect();
    let early = within(srv.next()).await.unwrap();
    let late = within(srv.next()).await.unwrap();

    drop(srv);
    for answer in answers {
        assert_eq!(within(answer).await, Err(unavailable("/add")));
    }
    // With the scope still open, an answer to a failed call is just dropped;
    // once it has closed, a late answer gets `Closed`.
    assert_eq!(early.respond(sum(0)), Ok(()));
    let _again = server.service::<AddRequest, AddResponse>("/add").unwrap();
    assert!(client.is_available());
    server.close();
    assert_eq!(late.respond(sum(2)), Err(closed("server")));
}

#[tokio::test]
async fn closing_the_callers_scope_fails_its_calls_in_flight_with_closed() {
    let bus = Bus::new();
    let (server, caller, other) = (bus.scope("server"), bus.scope("caller"), bus.scope("other"));
    let mut srv = server.service::<AddRequest, AddResponse>("/add").unwrap();
    let client = caller.client::<AddRequest, AddResponse>("/add").unwrap();
    let bystander = other.client::<AddRequest, AddResponse>("/add").unwrap();

    let mut handed_out = client.request(&add(1, 2)).unwrap();
    let mut answered = client.request(&add(3, 4)).unwrap();
    let queued = client.request(&add(5, 6)).unwrap();
    let unaffected = bystander.request(&add(7, 8)).unwrap();
    let woken = Woken::new();
    assert!(poll_with(&mut handed_out, &woken).is_pending());
    let req = within(srv.next()).await.unwrap();
    within(srv.next()).await.unwrap().respond(sum(7)).unwrap();

    caller.close();
    assert!(woken.take(), "the close wakes the waiting caller");
    assert_eq!(poll_with(&mut handed_out, &woken), Poll::Ready(Err(closed("caller"))));
    // Even an answer that came before the close is not taken after it.
    assert_eq!(poll_with(&mut answered, &woken), Poll::Ready(Err(closed("caller"))));
    assert_eq!(within(queued).await, Err(closed("caller")));
    // The server may still answer; the answer is dropped.
    assert_eq!(req.respond(sum(3)), Ok(()));
    assert_eq!(client.request(&add(1, 2)).unwrap_err(), closed("caller"));

    // Other scopes' calls go on (the queued call reaches the server anyway).
    let _adder = spawn_adder(srv);
    assert_eq!(within(unaffected).await, Ok(sum(15)));
}

#[tokio::test]
async fn wait_for_service_resolves_when_a_server_registers_or_at_once_if_one_is_live() {
    let bus = Bus::new();
    let (server, caller) = (bus.scope("server"), bus.scope("caller"));
    let client = caller.client::<AddRequest, AddResponse>("/add").unwrap();
    assert!(!client.is_available());

    let mut wait = Box::pin(client.wait_for_service());
    let woken = Woken::new();
    assert!(poll_with(&mut wait, &woken).is_pending());
    let srv = server.service::<AddRequest, AddResponse>("/add").unwrap();
    assert!(woken.take(), "the server's registration wakes the waiter");
    assert_eq!(poll_with(&mut wait, &woken), Poll::Ready(Ok(())));
    assert!(client.is_available());
    assert_eq!(client.wait_for_service().now_or_never(), Some(Ok(())));

    // Gone again: a spawned waiter sees the next server.
    drop(srv);
    assert!(!client.is_available());
    let waiter = {
        let client = client.clone();
        tokio::spawn(async move { client.wait_for_service().await })
    };
    tokio::task::yield_now().await;
    let _srv = server.service::<AddRequest, AddResponse>("/add").unwrap();
    assert_eq!(within(waiter).await.unwrap(), Ok(()));
}

#[test]
fn wait_for_service_fails_with_closed_when_the_clients_scope_closes_first() {
    let bus = Bus::new();
    let caller = bus.scope("caller");
    let client = caller.client::<AddRequest, AddResponse>("/add").unwrap();
    let mut wait = Box::pin(client.wait_for_service());
    let woken = Woken::new();
    assert!(poll_with(&mut wait, &woken).is_pending());

    caller.close();
    assert!(woken.take(), "the close wakes the waiter");
    assert_eq!(poll_with(&mut wait, &woken), Poll::Ready(Err(closed("caller"))));
    // A closed scope stays closed, server or not.
    let _srv = bus.scope("server").service::<AddRequest, AddResponse>("/add").unwrap();
    assert_eq!(client.wait_for_service().now_or_never(), Some(Err(closed("caller"))));
}

#[tokio::test]
async fn call_json_reaches_a_typed_server_through_the_names_codecs() {
    let bus = Bus::new();
    let (server, bridge) = (bus.scope("server"), bus.scope("ws_bridge"));
    let mut srv = server.service::<AddRequest, AddResponse>("/add").unwrap();
    let serve_one = async {
        let req = srv.next().await.unwrap();
        assert_eq!(req.message, add(2, 40));
        req.respond(sum(42)).unwrap();
    };
    let call = bridge.call_json("/add", json!({"a": 2, "b": 40}));
    let (answer, ()) = within(futures::future::join(call, serve_one)).await;
    assert_eq!(answer, Ok(json!({"sum": 42})));

    // r2r-shaped types, with an empty request.
    let mut srv = server.service::<Trigger::Request, Trigger::Response>("/save_map").unwrap();
    let _saver = tokio::spawn(async move {
        while let Some(req) = srv.next().await {
            let _ = req.respond(Trigger::Response { success: true, message: "saved".into() });
        }
    });
    let answer = within(bridge.call_json("/save_map", json!({}))).await;
    assert_eq!(answer, Ok(json!({"success": true, "message": "saved"})));
}

#[tokio::test]
async fn call_json_refuses_unknown_names_and_bad_json_before_calling() {
    let bus = Bus::new();
    let (server, bridge) = (bus.scope("server"), bus.scope("ws_bridge"));
    let mut srv = server.service::<AddRequest, AddResponse>("/add").unwrap();

    assert_eq!(bridge.call_json("/nope", json!({})).await, Err(BusError::UnknownName { name: "/nope".into() }));
    let bad = bridge.call_json("/add", json!({"a": "two", "b": 1})).await;
    assert!(matches!(&bad, Err(BusError::Json { name, .. }) if name == "/add"), "{bad:?}");
    // Nothing reached the server, and nothing was counted.
    assert!(futures::poll!(srv.next()).is_pending());
    assert_eq!(bus.services()[0].calls, 0);
}

#[tokio::test]
async fn call_json_fails_like_a_typed_call() {
    let bus = Bus::new();
    let (server, bridge) = (bus.scope("server"), bus.scope("ws_bridge"));
    let request = json!({"a": 1, "b": 2});

    // Declared by a client, no server.
    let _client = bus.scope("app").client::<AddRequest, AddResponse>("/add").unwrap();
    assert_eq!(bridge.call_json("/add", request.clone()).await, Err(unavailable("/add")));

    // Dropped unanswered.
    let mut srv = server.service::<AddRequest, AddResponse>("/add").unwrap();
    let drop_one = async { drop(srv.next().await) };
    let (answer, ()) = within(futures::future::join(bridge.call_json("/add", request.clone()), drop_one)).await;
    assert_eq!(answer, Err(BusError::NoResponse { name: "/add".into() }));

    // The calling scope closes mid-call, then refuses further calls.
    let mut call = Box::pin(bridge.call_json("/add", request.clone()));
    let woken = Woken::new();
    assert!(poll_with(&mut call, &woken).is_pending());
    bridge.close();
    assert!(woken.take(), "the close wakes the JSON caller");
    assert_eq!(poll_with(&mut call, &woken), Poll::Ready(Err(closed("ws_bridge"))));
    assert_eq!(bridge.call_json("/add", request).await, Err(closed("ws_bridge")));
}

#[test]
fn introspection_lists_each_names_server_clients_and_calls_sorted_by_name() {
    let bus = Bus::new();
    let (nav, app, coverage) = (bus.scope("nav"), bus.scope("app"), bus.scope("coverage"));
    let srv = nav.service::<Trigger::Request, Trigger::Response>("/save_map").unwrap();
    let from_app = app.client::<Trigger::Request, Trigger::Response>("/save_map").unwrap();
    let from_app_clone = from_app.clone();
    let from_coverage = coverage.client::<Trigger::Request, Trigger::Response>("/save_map").unwrap();
    let adder = app.client::<AddRequest, AddResponse>("/add").unwrap();

    // Calls that reached the server count, answered or not; one refused at
    // call time does not.
    drop(from_app.request(&Trigger::Request {}).unwrap());
    drop(from_coverage.request(&Trigger::Request {}).unwrap());
    assert!(adder.request(&add(1, 2)).is_err());

    let add_info = ServiceInfo {
        name: "/add".into(),
        request_type: type_name::<AddRequest>(),
        response_type: type_name::<AddResponse>(),
        ros_type: None,
        server: None,
        clients: vec!["app".into()],
        calls: 0,
    };
    let save_map_info = ServiceInfo {
        name: "/save_map".into(),
        request_type: type_name::<Trigger::Request>(),
        response_type: type_name::<Trigger::Response>(),
        ros_type: Some("std_srvs/srv/Trigger".into()),
        server: Some("nav".into()),
        // A client and its clone are one client.
        clients: vec!["app".into(), "coverage".into()],
        calls: 2,
    };
    assert_eq!(bus.services(), vec![add_info, save_map_info]);

    // Dropped clients and those of a closed scope go, and so does a dropped
    // server; the count stays.
    let save_map = || bus.services().into_iter().find(|s| s.name == "/save_map").unwrap();
    drop(from_coverage);
    drop(from_app_clone);
    assert_eq!(save_map().clients, vec!["app".to_string()]);
    app.close();
    assert!(save_map().clients.is_empty());
    drop(srv);
    let last = save_map();
    assert_eq!((last.server, last.calls), (None, 2));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_response_future_can_be_spawned_on_tokio() {
    let bus = Bus::new();
    let (server, caller) = (bus.scope("adder"), bus.scope("caller"));
    let _adder = spawn_adder(server.service("/add").unwrap());
    let client = caller.client::<AddRequest, AddResponse>("/add").unwrap();
    let answer = tokio::spawn(client.request(&add(20, 22)).unwrap());
    assert_eq!(within(answer).await.unwrap(), Ok(sum(42)));
}

#[test]
fn a_call_answered_from_a_std_thread_completes_without_any_runtime() {
    let bus = Bus::new();
    let (server, caller) = (bus.scope("server"), bus.scope("caller"));
    let mut srv = server.service::<AddRequest, AddResponse>("/add").unwrap();
    // The server hands each request to a std thread of its own to answer.
    let serving = std::thread::spawn(move || {
        let mut answering = Vec::new();
        while let Some(req) = futures::executor::block_on(srv.next()) {
            answering.push(std::thread::spawn(move || {
                let answer = sum(req.message.a + req.message.b);
                req.respond(answer).unwrap();
            }));
        }
        answering.into_iter().for_each(|thread| thread.join().unwrap());
    });

    let client = caller.client::<AddRequest, AddResponse>("/add").unwrap();
    for i in 0..3 {
        let answer = futures::executor::block_on(client.request(&add(i, 1)).unwrap());
        assert_eq!(answer, Ok(sum(i + 1)));
    }
    server.close();
    serving.join().unwrap();
}

/// A waker that panics when woken.
struct Exploding;

impl ArcWake for Exploding {
    fn wake_by_ref(_: &Arc<Self>) {
        panic!("waker exploded");
    }
}

fn exploding() -> Waker {
    futures::task::waker(Arc::new(Exploding))
}

fn poll_by<F: Future + Unpin>(fut: &mut F, waker: &Waker) -> Poll<F::Output> {
    Pin::new(fut).poll(&mut Context::from_waker(waker))
}

/// The contract for the callers. It would hold even if the server's close
/// skipped the later answers: dropping an unsent answer wakes its caller
/// with `ServiceUnavailable` all the same. The next test is the one a
/// skipped wake-up shows in.
#[test]
fn a_caller_whose_waker_panics_keeps_no_other_caller_asleep_when_the_server_closes() {
    let bus = Bus::new();
    let (server, caller) = (bus.scope("server"), bus.scope("caller"));
    let _srv = server.service::<AddRequest, AddResponse>("/add").unwrap();
    let client = caller.client::<AddRequest, AddResponse>("/add").unwrap();
    let mut broken = client.request(&add(1, 1)).unwrap(); // queued first, failed first
    let mut fine = client.request(&add(2, 2)).unwrap();
    assert!(poll_by(&mut broken, &exploding()).is_pending());
    let woken = Woken::new();
    assert!(poll_with(&mut fine, &woken).is_pending());

    assert!(catch_unwind(AssertUnwindSafe(|| server.close())).is_err(), "the waker's panic goes on");
    assert!(woken.take(), "the other caller was woken");
    assert_eq!(poll_with(&mut fine, &woken), Poll::Ready(Err(unavailable("/add"))));
}

#[test]
fn a_caller_whose_waker_panics_keeps_the_server_task_from_sleeping_through_its_close() {
    let bus = Bus::new();
    let (server, caller) = (bus.scope("server"), bus.scope("caller"));
    let mut srv = server.service::<AddRequest, AddResponse>("/add").unwrap();
    let client = caller.client::<AddRequest, AddResponse>("/add").unwrap();
    let mut broken = client.request(&add(1, 1)).unwrap();
    assert!(poll_by(&mut broken, &exploding()).is_pending());
    let server_task = Woken::new();
    let Poll::Ready(Some(in_hand)) = poll_next_with(&mut srv, &server_task) else { panic!("the call is not handed out") };
    assert!(poll_next_with(&mut srv, &server_task).is_pending(), "the server task waits for the next one");

    assert!(catch_unwind(AssertUnwindSafe(|| server.close())).is_err(), "the waker's panic goes on");
    assert!(server_task.take(), "the server task was woken to its stream's end");
    assert!(matches!(poll_next_with(&mut srv, &server_task), Poll::Ready(None)));
    assert_eq!(in_hand.respond(sum(2)), Err(closed("server")));
}

#[test]
fn a_call_whose_waker_panics_keeps_no_other_call_asleep_when_its_scope_closes() {
    let bus = Bus::new();
    let (server, caller) = (bus.scope("server"), bus.scope("caller"));
    let _srv = server.service::<AddRequest, AddResponse>("/add").unwrap(); // never answers
    let client = caller.client::<AddRequest, AddResponse>("/add").unwrap();
    let mut broken = client.request(&add(1, 1)).unwrap(); // the first to watch the scope
    let mut fine = client.request(&add(2, 2)).unwrap();
    assert!(poll_by(&mut broken, &exploding()).is_pending());
    let woken = Woken::new();
    assert!(poll_with(&mut fine, &woken).is_pending());

    assert!(catch_unwind(AssertUnwindSafe(|| caller.close())).is_err(), "the waker's panic goes on");
    assert!(woken.take(), "the other call was woken");
    assert_eq!(poll_with(&mut fine, &woken), Poll::Ready(Err(closed("caller"))));
}

#[test]
fn a_waiter_whose_waker_panics_keeps_no_other_waiter_asleep_when_a_server_comes() {
    let bus = Bus::new();
    let caller = bus.scope("caller");
    let client = caller.client::<AddRequest, AddResponse>("/add").unwrap();
    let mut broken = Box::pin(client.wait_for_service()); // the first to wait
    let mut fine = Box::pin(client.wait_for_service());
    assert!(poll_by(&mut broken, &exploding()).is_pending());
    let woken = Woken::new();
    assert!(poll_with(&mut fine, &woken).is_pending());

    let server = bus.scope("server");
    let registered = catch_unwind(AssertUnwindSafe(|| server.service::<AddRequest, AddResponse>("/add")));
    assert!(registered.is_err(), "the waker's panic goes on to whoever registers");
    assert!(woken.take(), "the other waiter was woken");
    // the panic dropped the new server on its way out: the name is free
    let _srv = server.service::<AddRequest, AddResponse>("/add").unwrap();
    assert_eq!(poll_with(&mut fine, &woken), Poll::Ready(Ok(())));
}

#[test]
fn handles_are_send_and_unpin_and_the_response_future_can_be_spawned() {
    fn send_unpin<T: Send + Unpin>() {}
    fn spawnable<T: Send + Unpin + 'static>() {}
    fn cloneable<T: Clone>() {}
    fn movable<T: Send + 'static>() {}
    fn send<T: Send>(_: &T) {}

    send_unpin::<Service<AddRequest, AddResponse>>();
    send_unpin::<Client<AddRequest, AddResponse>>();
    spawnable::<ResponseFuture<AddResponse>>();
    cloneable::<Client<AddRequest, AddResponse>>();
    // A request moves to the task or thread that answers it.
    movable::<Request<AddRequest, AddResponse>>();
    // ws_bridge spawns its JSON calls, modules their waits.
    let bus = Bus::new();
    let scope = bus.scope("m");
    send(&scope.call_json("/add", json!({})));
    let client = scope.client::<AddRequest, AddResponse>("/add").unwrap();
    send(&client.wait_for_service());
}

/// Review 2026-10-26 (a stress test that passed: evidence for "nothing
/// found"). Calls race server restarts, dropped requests and closes of the
/// calling scopes, typed and JSON: every call resolves, with an outcome the
/// README allows.
///
/// Ignored by default because it soaks for 3 s by design. Run it with
/// `cargo test -p mower_bus --test services -- --ignored`.
#[test]
#[ignore = "soaks for 3 s; run with --ignored"]
fn calls_racing_server_restarts_and_caller_closes_always_resolve_with_an_allowed_outcome() {
    const HANG: Duration = Duration::from_secs(10);
    let rt = tokio::runtime::Builder::new_multi_thread().worker_threads(4).enable_time().build().unwrap();
    let bus = Bus::new();
    let stop = Arc::new(AtomicBool::new(false));
    let answered = Arc::new(AtomicU64::new(0));

    // A server that restarts: serve, answer for a while (some answers from
    // spawned tasks, some requests dropped), close the scope, again.
    let server = {
        let (bus, stop) = (bus.clone(), stop.clone());
        rt.spawn(async move {
            let mut round = 0u64;
            while !stop.load(Ordering::SeqCst) {
                round += 1;
                let scope = bus.scope("server");
                let mut srv = match scope.service::<u64, u64>("/x") {
                    Ok(srv) => srv,
                    Err(e) => panic!("round {round}: {e}"),
                };
                let serving = tokio::spawn(async move {
                    let mut n = 0u64;
                    while let Some(req) = srv.next().await {
                        n += 1;
                        match n % 5 {
                            0 => drop(req),
                            1 => {
                                tokio::spawn(async move {
                                    tokio::task::yield_now().await;
                                    let v = req.message + 1;
                                    let _ = req.respond(v);
                                });
                            }
                            _ => {
                                let v = req.message + 1;
                                let _ = req.respond(v);
                            }
                        }
                    }
                });
                tokio::time::sleep(Duration::from_micros(300 + round % 700)).await;
                scope.close();
                tokio::time::timeout(HANG, serving).await.expect("server stream ended after close").unwrap();
            }
        })
    };

    // Callers whose scopes close now and then, mid-call.
    let callers: Vec<_> = (0..6)
        .map(|c| {
            let (bus, stop, answered) = (bus.clone(), stop.clone(), answered.clone());
            rt.spawn(async move {
                let mut k = 0u64;
                while !stop.load(Ordering::SeqCst) {
                    let scope = bus.scope(&format!("caller{c}"));
                    let client = scope.client::<u64, u64>("/x").unwrap();
                    for _ in 0..50 {
                        k += 1;
                        if k.is_multiple_of(37) {
                            let s = scope.clone();
                            tokio::spawn(async move { s.close() });
                        }
                        let fut = match client.request(&k) {
                            Ok(fut) => fut,
                            Err(BusError::ServiceUnavailable { .. }) | Err(BusError::Closed { .. }) => {
                                // refused at once: let the server run (bus futures do not yield on their own)
                                tokio::task::yield_now().await;
                                continue;
                            }
                            Err(e) => panic!("outer error {e}"),
                        };
                        match tokio::time::timeout(HANG, fut).await.expect("a call hung") {
                            Ok(v) => {
                                assert_eq!(v, k + 1);
                                answered.fetch_add(1, Ordering::Relaxed);
                            }
                            Err(BusError::ServiceUnavailable { .. })
                            | Err(BusError::NoResponse { .. })
                            | Err(BusError::Closed { .. }) => {}
                            Err(e) => panic!("inner error {e}"),
                        }
                        if k.is_multiple_of(3) && !stop.load(Ordering::SeqCst) {
                            // wait_for_service races the restarts too; once the
                            // server loop has stopped it may wait for good
                            if tokio::time::timeout(HANG, client.wait_for_service()).await.is_err() {
                                assert!(stop.load(Ordering::SeqCst), "wait_for_service hung while servers kept coming");
                            }
                        }
                        if stop.load(Ordering::SeqCst) {
                            break;
                        }
                    }
                    scope.close();
                }
            })
        })
        .collect();

    // JSON calls from a bridge scope as well.
    let bridge = {
        let (bus, stop) = (bus.clone(), stop.clone());
        rt.spawn(async move {
            let scope = bus.scope("ws_bridge");
            while !stop.load(Ordering::SeqCst) {
                match tokio::time::timeout(HANG, scope.call_json("/x", json!(41))).await.expect("json call hung") {
                    Ok(v) => assert_eq!(v, json!(42)),
                    Err(BusError::ServiceUnavailable { .. }) | Err(BusError::NoResponse { .. }) | Err(BusError::UnknownName { .. }) => {}
                    Err(e) => panic!("json error {e}"),
                }
                tokio::task::yield_now().await;
            }
        })
    };

    thread::sleep(Duration::from_secs(3));
    stop.store(true, Ordering::SeqCst);
    rt.block_on(async {
        tokio::time::timeout(HANG, server).await.expect("server loop hung").unwrap();
        for c in callers {
            tokio::time::timeout(HANG, c).await.expect("caller hung").unwrap();
        }
        tokio::time::timeout(HANG, bridge).await.expect("bridge hung").unwrap();
    });
    assert!(answered.load(Ordering::Relaxed) > 100, "{}", answered.load(Ordering::Relaxed));
}
