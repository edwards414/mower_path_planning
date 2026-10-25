//! Services: one server per name, requests handed out one at a time,
//! answered through the `Request` itself. Contract: README.md, "Services".
//!
//! Requirements beyond the README:
//! - no async runtime needed (the response future can be awaited on any
//!   executor; `respond` can be called from a std thread);
//! - `Service`, `Client` and `ResponseFuture` are `Send + Unpin`; `Client` is
//!   `Clone`; `ResponseFuture` is `'static` (it can be spawned);
//! - the request's JSON path (`call_json`) uses the codecs the server
//!   registered (`crate::codec::Codec` for the request and response types);
//! - a `Request` dropped unanswered -> the caller gets `NoResponse`; the
//!   server's scope closing (or its `Service` dropping) -> queued and
//!   in-flight callers get `ServiceUnavailable`, and the name is free for a
//!   new server;
//! - closing a client's scope fails its in-flight calls with `Closed`; the
//!   server may still answer, the answer is dropped;
//! - `request` checks for a server at call time: none -> outer `Err(
//!   ServiceUnavailable)` (a ROS 2 request to a missing server is lost
//!   silently and its caller waits forever).

use std::future::Future;
use std::marker::PhantomData;
use std::pin::Pin;
use std::task::{Context, Poll};

use futures::Stream;

use crate::{BusError, Message, Scope, ServiceInfo};

/// The service half of the bus registry.
#[derive(Default)]
pub(crate) struct Registry {}

impl Registry {
    pub(crate) fn info(&self) -> Vec<ServiceInfo> {
        todo!("E1 services")
    }
}

pub(crate) fn serve<Req: Message, Resp: Message>(scope: &Scope, name: &str) -> Result<Service<Req, Resp>, BusError> {
    let _ = (scope, name);
    todo!("E1 services")
}

pub(crate) fn client<Req: Message, Resp: Message>(scope: &Scope, name: &str) -> Result<Client<Req, Resp>, BusError> {
    let _ = (scope, name);
    todo!("E1 services")
}

pub(crate) async fn call_json(scope: &Scope, name: &str, request: serde_json::Value) -> Result<serde_json::Value, BusError> {
    let _ = (scope, name, request);
    todo!("E1 services")
}

/// The server side of one service name: a stream of requests.
pub struct Service<Req: Message, Resp: Message> {
    _t: PhantomData<fn(Req) -> Resp>,
}

impl<Req: Message, Resp: Message> Service<Req, Resp> {
    pub fn name(&self) -> &str {
        todo!("E1 services")
    }

    /// The next request; `None` once the scope is closed.
    pub async fn recv(&mut self) -> Option<Request<Req, Resp>> {
        todo!("E1 services")
    }
}

impl<Req: Message, Resp: Message> Stream for Service<Req, Resp> {
    type Item = Request<Req, Resp>;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Request<Req, Resp>>> {
        let _ = cx;
        todo!("E1 services")
    }
}

/// One call, as the server sees it (r2r's `ServiceRequest`).
pub struct Request<Req: Message, Resp: Message> {
    pub message: Req,
    _t: PhantomData<fn(Resp)>,
}

impl<Req: Message, Resp: Message> Request<Req, Resp> {
    /// Answer the call. `Ok` even if the caller has given up (the answer is
    /// dropped); `Closed` if the server's scope has closed.
    pub fn respond(self, response: Resp) -> Result<(), BusError> {
        let _ = response;
        todo!("E1 services")
    }
}

/// A client of one service name. Clones share nothing but the name.
pub struct Client<Req: Message, Resp: Message> {
    _t: PhantomData<fn(Req) -> Resp>,
}

impl<Req: Message, Resp: Message> Clone for Client<Req, Resp> {
    fn clone(&self) -> Self {
        todo!("E1 services")
    }
}

impl<Req: Message, Resp: Message> Client<Req, Resp> {
    pub fn name(&self) -> &str {
        todo!("E1 services")
    }

    /// Send `msg` (cloned) and get the future of the answer: r2r's
    /// `client.request(&req)?.await?`. Outer error: `Closed`, or
    /// `ServiceUnavailable` when no server is live at call time.
    pub fn request(&self, msg: &Req) -> Result<ResponseFuture<Resp>, BusError> {
        let _ = msg;
        todo!("E1 services")
    }

    pub fn is_available(&self) -> bool {
        todo!("E1 services")
    }

    /// Resolve once a server is live (at once if one is). `Closed` if the
    /// client's scope closes first.
    pub async fn wait_for_service(&self) -> Result<(), BusError> {
        todo!("E1 services")
    }
}

/// The answer to one call: `ServiceUnavailable`, `NoResponse` or `Closed`
/// when there will be none.
pub struct ResponseFuture<Resp> {
    _t: PhantomData<fn() -> Resp>,
}

impl<Resp: Message> Future for ResponseFuture<Resp> {
    type Output = Result<Resp, BusError>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let _ = cx;
        todo!("E1 services")
    }
}
