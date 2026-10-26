//! A message type seen through `fn` pointers: what erased endpoints and JSON
//! calls use to handle values whose type they do not know (README.md,
//! "Erased access").

use std::any::{Any, TypeId};
use std::sync::Arc;

use crate::Message;

/// Shared form of a message whose type is known only to its codec.
pub(crate) type AnyMsg = Arc<dyn Any + Send + Sync>;

#[derive(Clone, Copy)]
pub(crate) struct Codec {
    pub type_id: TypeId,
    pub type_name: &'static str,
    /// Serialize a value of this codec's type. `None` if `value` is of
    /// another type (a bug in the caller).
    pub to_json: fn(&(dyn Any + Send + Sync)) -> Option<Result<serde_json::Value, String>>,
    /// Deserialize JSON into a value of this codec's type.
    pub from_json: fn(serde_json::Value) -> Result<AnyMsg, String>,
}

impl Codec {
    pub fn of<T: Message>() -> Codec {
        Codec {
            type_id: TypeId::of::<T>(),
            type_name: std::any::type_name::<T>(),
            to_json: |value| value.downcast_ref::<T>().map(|v| serde_json::to_value(v).map_err(|e| e.to_string())),
            from_json: |json| serde_json::from_value::<T>(json).map(|v| Arc::new(v) as AnyMsg).map_err(|e| e.to_string()),
        }
    }

    pub fn is<T: 'static>(&self) -> bool {
        self.type_id == TypeId::of::<T>()
    }
}

impl std::fmt::Debug for Codec {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_tuple("Codec").field(&self.type_name).finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::{Deserialize, Serialize};

    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    struct Pose {
        x: f64,
        frame_id: String,
    }

    #[test]
    fn a_codec_round_trips_its_type_and_refuses_others() {
        let c = Codec::of::<Pose>();
        assert!(c.is::<Pose>() && !c.is::<String>());
        let value: AnyMsg = Arc::new(Pose { x: 1.5, frame_id: "map".into() });
        let json = (c.to_json)(value.as_ref()).unwrap().unwrap();
        assert_eq!(json, serde_json::json!({"x": 1.5, "frame_id": "map"}));
        let back = (c.from_json)(json).unwrap();
        assert_eq!(back.downcast_ref::<Pose>(), Some(&Pose { x: 1.5, frame_id: "map".into() }));
        assert!((c.to_json)(&String::from("no")).is_none());
        assert!((c.from_json)(serde_json::json!({"x": "not a number"})).is_err());
    }
}
