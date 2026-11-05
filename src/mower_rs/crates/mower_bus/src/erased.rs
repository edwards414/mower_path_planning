//! Endpoints that do not know their message type: what ws_bridge (the app's
//! subscribe / publish), the recorder and `mowerctl` use. Contract:
//! README.md, "Erased access". They live on the same channels as the typed
//! endpoints in `topic.rs` and follow the same delivery, queue, durability
//! and scope-close rules.

use std::fmt;
use std::pin::Pin;
use std::sync::Arc;
use std::task::{Context, Poll};

use futures::Stream;

use crate::codec::{AnyMsg, Codec};
use crate::topic::{Reader, Stamped, Writer};
use crate::{BusError, Message, PublisherChanges, Qos, RecvError, Scope, Time};

/// `capacity`: [`Scope::subscribe_erased_with`]'s queue bound.
pub(crate) fn subscribe(scope: &Scope, name: &str, qos: Qos, capacity: Option<usize>) -> Result<ErasedSubscriber, BusError> {
    Ok(ErasedSubscriber { reader: Reader::open_with(scope, name, qos, None, capacity)? })
}

pub(crate) fn publisher(scope: &Scope, name: &str, qos: Qos) -> Result<JsonPublisher, BusError> {
    Ok(JsonPublisher { writer: Writer::open(scope, name, qos, None)? })
}

/// One message whose type only its codec knows, with the instant it was
/// published.
///
/// It is `serde::Serialize` as its value is, through the codec, so any serde
/// serializer takes it without a copy: `mower_cdr::to_vec` makes the CDR
/// rosbag2 records (E5-4), `serde_json` the JSON of [`ErasedMsg::to_json`].
#[derive(Clone)]
pub struct ErasedMsg {
    value: AnyMsg,
    codec: Codec,
    published: Time,
}

impl ErasedMsg {
    pub(crate) fn new(value: AnyMsg, codec: Codec, published: Time) -> Self {
        ErasedMsg { value, codec, published }
    }

    pub fn type_name(&self) -> &'static str {
        self.codec.type_name
    }

    /// When it was published: the bus clock's wall time, read by the
    /// publish (for a message from history, by its own publish).
    pub fn published(&self) -> Time {
        self.published
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

impl serde::Serialize for ErasedMsg {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        match (self.codec.serialize)(self.value.as_ref()) {
            Some(value) => erased_serde::serialize(value, serializer),
            None => Err(serde::ser::Error::custom(format!("{}: value/codec type mismatch", self.codec.type_name))),
        }
    }
}

impl std::fmt::Debug for ErasedMsg {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ErasedMsg").field("type", &self.codec.type_name).finish_non_exhaustive()
    }
}

/// A reader of one topic that yields [`ErasedMsg`]s.
pub struct ErasedSubscriber {
    reader: Reader,
}

impl ErasedSubscriber {
    pub fn name(&self) -> &str {
        self.reader.channel().name()
    }

    /// The channel's type, once a typed endpoint has fixed it.
    pub fn type_name(&self) -> Option<&'static str> {
        self.reader.channel().codec().map(|codec| codec.type_name)
    }

    /// The next message; `None` once the scope is closed.
    pub async fn recv(&mut self) -> Option<ErasedMsg> {
        let value = std::future::poll_fn(|cx| self.reader.poll_recv(cx)).await?;
        Some(self.wrap(value))
    }

    /// Block the calling thread up to `timeout` for the next message, as
    /// [`crate::Subscriber::recv_timeout`]. For dedicated std threads.
    pub fn recv_timeout(&mut self, timeout: std::time::Duration) -> Result<ErasedMsg, RecvError> {
        let value = self.reader.recv_timeout(timeout)?;
        Ok(self.wrap(value))
    }

    pub fn try_recv(&mut self) -> Result<ErasedMsg, RecvError> {
        let value = self.reader.try_recv()?;
        Ok(self.wrap(value))
    }

    pub fn dropped(&self) -> u64 {
        self.reader.dropped()
    }

    /// Live writers on the channel, as [`crate::Subscriber::publisher_count`]
    /// counts them.
    pub fn publisher_count(&self) -> usize {
        self.reader.channel().publisher_count()
    }

    /// The live writer count each time writers have come or gone, ending
    /// when this reader's scope closes: see
    /// [`crate::Subscriber::publisher_changes`]. The ROS bridge drops a
    /// latched value on the DDS side from it when a topic's last writer goes.
    pub fn publisher_changes(&self) -> PublisherChanges {
        self.reader.watch_publishers()
    }

    /// A channel carries messages only once a typed endpoint has fixed its
    /// type (a `JsonPublisher` cannot publish before), so a reader that
    /// attached earlier finds the codec by the time its first message comes.
    fn wrap(&self, (value, published): Stamped) -> ErasedMsg {
        let codec = *self.reader.channel().codec().expect("a message on a channel whose type is not fixed");
        ErasedMsg::new(value, codec, published)
    }
}

impl Stream for ErasedSubscriber {
    type Item = ErasedMsg;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<ErasedMsg>> {
        let this = self.get_mut();
        this.reader.poll_recv(cx).map(|value| value.map(|value| this.wrap(value)))
    }
}

impl fmt::Debug for ErasedSubscriber {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ErasedSubscriber")
            .field("name", &self.name())
            .field("type", &self.type_name())
            .finish_non_exhaustive()
    }
}

/// A writer on one topic that takes JSON and decodes it with the channel's
/// codec. Counts as a writer (and a transient-local one keeps history) like
/// a typed [`crate::Publisher`]. Clones are the same writer.
#[derive(Clone)]
pub struct JsonPublisher {
    writer: Arc<Writer>,
}

impl JsonPublisher {
    pub fn name(&self) -> &str {
        self.writer.channel().name()
    }

    /// `UnknownName` while no typed endpoint has fixed the type, `Json` when
    /// the value does not decode, `Closed` after the scope closed.
    pub fn publish(&self, json: &serde_json::Value) -> Result<(), BusError> {
        self.writer.check_open()?;
        let name = self.name();
        let codec = self.writer.channel().codec().ok_or_else(|| BusError::UnknownName { name: name.to_string() })?;
        // Decoded before the channel's lock is taken: a big value takes a while.
        let msg = (codec.from_json)(json.clone()).map_err(|error| BusError::Json { name: name.to_string(), error })?;
        self.writer.publish(msg)
    }
}

impl fmt::Debug for JsonPublisher {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("JsonPublisher").field("name", &self.name()).finish_non_exhaustive()
    }
}
