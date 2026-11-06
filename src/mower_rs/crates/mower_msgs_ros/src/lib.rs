//! Where the plain types meet r2r's (E9-22): what the bridge's `ros` feature
//! needs once the bus carries `mower_msgs`. The impls are generated
//! (`mower_schema/gen/gen_types.py`, `src/generated.rs`); nothing in production
//! uses them yet.
//!
//! A conversion moves each field: no string or array is copied. Every field on both
//! sides is named, so a field that differs in name or type stops the build. The
//! conversion refuses nothing, and neither does the plain type: a fixed array of the
//! wrong length or a NUL still panics inside r2r when it copies the value to C, so
//! the bridge's guards (`mower_bus_ros/src/convert.rs`) stay where they are.

mod generated;

/// A plain message type and its r2r twin.
pub trait RosMirror: mower_msgs::MessageType {
    type Ros: r2r::WrappedTypesupport + PartialEq + Send + Sync + 'static;
    fn into_ros(self) -> Self::Ros;
    fn from_ros(ros: Self::Ros) -> Self;
    /// For a value the bus shares (`Arc<T>`): one clone, where r2r's type on the bus
    /// needed none.
    fn to_ros(&self) -> Self::Ros {
        self.clone().into_ros()
    }
}

/// A plain service and its r2r twin, whose request and response are the twins of its own.
pub trait ServiceMirror: mower_msgs::ServiceType<Request: RosMirror, Response: RosMirror> {
    type Ros: 'static + Send + Sync + r2r::WrappedServiceTypeSupport<
        Request = <Self::Request as RosMirror>::Ros,
        Response = <Self::Response as RosMirror>::Ros,
    >;
}

/// A plain action and its r2r twin, whose goal, result and feedback are the twins of its own.
pub trait ActionMirror: mower_msgs::ActionType<Goal: RosMirror, Result: RosMirror, Feedback: RosMirror> {
    type Ros: 'static + Send + Sync + r2r::WrappedActionTypeSupport<
        Goal = <Self::Goal as RosMirror>::Ros,
        Result = <Self::Result as RosMirror>::Ros,
        Feedback = <Self::Feedback as RosMirror>::Ros,
    >;
}
