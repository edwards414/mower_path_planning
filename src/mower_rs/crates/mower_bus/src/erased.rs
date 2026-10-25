//! Endpoints that do not know their message type: what ws_bridge (the app's
//! subscribe / publish), the recorder and `mowerctl` use. Contract:
//! README.md, "Erased access". They live on the same channels as the typed
//! endpoints in `topic.rs` and follow the same delivery, queue, durability
//! and scope-close rules.

use std::pin::Pin;
use std::sync::Arc;
use std::task::{Context, Poll};

use futures::Stream;

use crate::codec::{AnyMsg, Codec};
use crate::{BusError, Message, Qos, RecvError, Scope};

pub(crate) fn subscribe(scope: &Scope, name: &str, qos: Qos) -> Result<ErasedSubscriber, BusError> {
    let _ = (scope, name, qos);
    todo!("E1 erased topics")
}

pub(crate) fn publisher(scope: &Scope, name: &str, qos: Qos) -> Result<JsonPublisher, BusError> {
    let _ = (scope, name, qos);
    todo!("E1 erased topics")
}

/// One message whose type only its codec knows.
#[derive(Clone)]
pub struct ErasedMsg {
    value: AnyMsg,
    codec: Codec,
}

impl ErasedMsg {
    pub(crate) fn new(value: AnyMsg, codec: Codec) -> Self {
        ErasedMsg { value, codec }
    }

    pub fn type_name(&self) -> &'static str {
        self.codec.type_name
    }

    /// Serialize with serde (README.md: that is the layout the app gets).
    pub fn to_json(&self) -> Result<serde_json::Value, BusError> {
        match (self.codec.to_json)(self.value.as_ref()) {
            Some(Ok(json)) => Ok(json),
            Some(Err(error)) => Err(BusError::Json { name: self.codec.type_name.to_string(), error }),
            None => Err(BusError::Json { name: self.codec.type_name.to_string(), error: "value/codec type mismatch".into() }),
        }
    }

    /// The message itself, without a copy, if it is a `T`.
    pub fn downcast<T: Message>(&self) -> Option<Arc<T>> {
        self.value.clone().downcast::<T>().ok()
    }
}

impl std::fmt::Debug for ErasedMsg {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ErasedMsg").field("type", &self.codec.type_name).finish_non_exhaustive()
    }
}

/// A reader of one topic that yields [`ErasedMsg`]s.
pub struct ErasedSubscriber {}

impl ErasedSubscriber {
    pub fn name(&self) -> &str {
        todo!("E1 erased topics")
    }

    /// The channel's type, once a typed endpoint has fixed it.
    pub fn type_name(&self) -> Option<&'static str> {
        todo!("E1 erased topics")
    }

    /// The next message; `None` once the scope is closed.
    pub async fn recv(&mut self) -> Option<ErasedMsg> {
        todo!("E1 erased topics")
    }

    pub fn try_recv(&mut self) -> Result<ErasedMsg, RecvError> {
        todo!("E1 erased topics")
    }

    pub fn dropped(&self) -> u64 {
        todo!("E1 erased topics")
    }
}

impl Stream for ErasedSubscriber {
    type Item = ErasedMsg;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<ErasedMsg>> {
        let _ = cx;
        todo!("E1 erased topics")
    }
}

/// A writer on one topic that takes JSON and decodes it with the channel's
/// codec. Counts as a writer (and a transient-local one keeps history) like
/// a typed [`crate::Publisher`]. Clones are the same writer.
#[derive(Clone)]
pub struct JsonPublisher {}

impl JsonPublisher {
    pub fn name(&self) -> &str {
        todo!("E1 erased topics")
    }

    /// `UnknownName` while no typed endpoint has fixed the type, `Json` when
    /// the value does not decode, `Closed` after the scope closed.
    pub fn publish(&self, json: &serde_json::Value) -> Result<(), BusError> {
        let _ = json;
        todo!("E1 erased topics")
    }
}
