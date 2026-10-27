//! What the clients reach through the bridge: topics, publishers and
//! services, on whichever backend `--backend` chose. The protocol (client.rs),
//! the listeners and the gate (lib.rs) are the same for every backend.
//!
//! - `r2r`: today's door, one r2r node with a spin thread (r2r_hub.rs).
//! - `bus`: mower_bus's erased access in the module's scope, with the
//!   names that are only on DDS carried by mower_rsd's bus bridge
//!   (bus_hub.rs).

use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;

use serde_json::Value;

use crate::bus_hub::BusHub;
use crate::client::ClientHandle;
use crate::entry::TopicEntry;
use crate::r2r_hub::R2rHub;

#[derive(Clone)]
pub enum Hub {
    R2r(R2rHub),
    Bus(BusHub),
}

impl Hub {
    /// Subscribe `client` to `topic` with its throttle: the topic's shared
    /// entry, with the client attached (a latched topic replays at once).
    pub async fn subscribe(
        &self, topic: &str, type_hint: Option<String>, client: &ClientHandle, throttle_ms: u64,
    ) -> Result<Arc<TopicEntry>, String> {
        match self {
            Hub::R2r(hub) => {
                let entry = hub.subscribe(topic, type_hint).await?;
                entry.add_client(client.clone(), throttle_ms);
                Ok(entry)
            }
            Hub::Bus(hub) => hub.subscribe(topic, type_hint, client, throttle_ms),
        }
    }

    /// Publish `msg` on `topic`. The bus backend takes the type from the
    /// bus, where a typed endpoint fixed it; the hint is r2r's.
    pub async fn publish(&self, topic: &str, type_hint: Option<String>, msg: Value) -> Result<(), String> {
        match self {
            Hub::R2r(hub) => hub.publish(topic, type_hint, msg).await,
            Hub::Bus(hub) => hub.publish(topic, msg),
        }
    }

    /// Call a service, bounded by `timeout` as a whole. `service_type` is
    /// ws_bridge.yaml's, which the r2r backend creates its client with; the
    /// bus backend uses the codecs of the name.
    pub async fn call_service(&self, service: &str, service_type: &str, args: Value, timeout: Duration) -> Result<Value, String> {
        match self {
            Hub::R2r(hub) => hub.call_service(service, service_type, args, timeout).await,
            Hub::Bus(hub) => hub.call_service(service, args, timeout).await,
        }
    }

    /// Every topic name with its types, for `/rosapi/topics`.
    pub async fn topics_and_types(&self) -> HashMap<String, Vec<String>> {
        match self {
            Hub::R2r(hub) => hub.topics_and_types().await,
            Hub::Bus(hub) => hub.topics_and_types(),
        }
    }

    /// The wall clock, for `/rosapi/get_time`.
    pub fn now(&self) -> std::time::Duration {
        match self {
            Hub::R2r(_) => std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default(),
            Hub::Bus(hub) => hub.now(),
        }
    }

    pub fn shutdown(&self) {
        match self {
            Hub::R2r(hub) => hub.shutdown(),
            Hub::Bus(hub) => hub.shutdown(),
        }
    }
}
