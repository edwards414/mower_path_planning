//! The one conversion from an r2r `QosProfile` to the bus's `Qos`, so that
//! both ends of a bridged name behave like the source endpoint (README.md,
//! "QoS").

use std::fmt;

use mower_bus::{Durability, Qos, Reliability};
use r2r::qos::{DurabilityPolicy, HistoryPolicy, ReliabilityPolicy};
use r2r::QosProfile;

/// A profile the bus has no equivalent for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct UnsupportedQos {
    pub reason: &'static str,
}

impl fmt::Display for UnsupportedQos {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "no bus equivalent: {}", self.reason)
    }
}

impl std::error::Error for UnsupportedQos {}

/// The bus QoS that keeps, delivers and replays like `qos` does on the
/// robot's RMW, rmw_cyclonedds_cpp:
///
/// - KEEP_LAST `depth`, also for the system-default and unknown history; a
///   depth of 0 (`RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT`) keeps 1;
/// - the system-default and unknown reliability are reliable, and the
///   system-default and unknown durability are volatile.
///
/// So `QosProfile::system_default()` gives `Qos::system_default()` (R V 1),
/// what rmw_cyclonedds announces for it (docs/BUS_TOPICS.md 5.5 c). Under
/// rmw_fastrtps a system-default *writer* is transient local instead; the
/// bus follows the robot.
///
/// Refused, because the bus would behave differently and silently:
/// KEEP_ALL (the bus keeps only the last `depth`), a depth rmw_cyclonedds
/// refuses (above `i32::MAX`), and the best-available policies (what they
/// resolve to depends on the endpoints DDS finds). No module uses any of
/// them.
///
/// Deadline, lifespan, liveliness and `avoid_ros_namespace_conventions`
/// stay on the DDS endpoint only: the bus has none of them (mower_bus
/// README, "Differences from DDS").
pub fn bus_qos(qos: &QosProfile) -> Result<Qos, UnsupportedQos> {
    let unsupported = |reason| Err(UnsupportedQos { reason });
    let depth = match qos.history {
        HistoryPolicy::KeepLast | HistoryPolicy::SystemDefault | HistoryPolicy::Unknown => qos.depth.max(1),
        HistoryPolicy::KeepAll => return unsupported("KEEP_ALL history (the bus keeps the last `depth` messages)"),
    };
    if depth > i32::MAX as usize {
        return unsupported("a history depth above i32::MAX, which rmw_cyclonedds refuses");
    }
    let reliability = match qos.reliability {
        ReliabilityPolicy::BestEffort => Reliability::BestEffort,
        ReliabilityPolicy::Reliable | ReliabilityPolicy::SystemDefault | ReliabilityPolicy::Unknown => {
            Reliability::Reliable
        }
        // BestAvailable, which exists from Iron on.
        _ => return unsupported("best-available reliability"),
    };
    let durability = match qos.durability {
        DurabilityPolicy::TransientLocal => Durability::TransientLocal,
        DurabilityPolicy::Volatile | DurabilityPolicy::SystemDefault | DurabilityPolicy::Unknown => {
            Durability::Volatile
        }
        _ => return unsupported("best-available durability"),
    };
    Ok(Qos { depth, reliability, durability })
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use r2r::qos::LivelinessPolicy;

    use super::*;

    #[test]
    fn the_profiles_in_use_map_to_their_bus_names() {
        let cases = [
            ("default", QosProfile::default(), Qos::default()),
            ("default().keep_last(10)", QosProfile::default().keep_last(10), Qos::default()),
            ("default().keep_last(1)", QosProfile::default().keep_last(1), Qos::default().keep_last(1)),
            ("sensor_data", QosProfile::sensor_data(), Qos::sensor_data()),
            ("system_default", QosProfile::system_default(), Qos::system_default()),
            (
                "the modules' latched()",
                QosProfile::default().keep_last(1).reliable().transient_local(),
                Qos::latched(),
            ),
            ("services_default", QosProfile::services_default(), Qos::default()),
            ("parameter_events", QosProfile::parameter_events(), Qos::default().keep_last(1000)),
            (
                "default().keep_last(1).best_effort().volatile()",
                QosProfile::default().keep_last(1).best_effort().volatile(),
                Qos::default().keep_last(1).best_effort(),
            ),
        ];
        for (what, r2r_qos, bus) in cases {
            assert_eq!(bus_qos(&r2r_qos), Ok(bus), "{what}");
        }
    }

    #[test]
    fn system_defaults_and_unknowns_resolve_as_rmw_cyclonedds_does() {
        let r2r_default = QosProfile::system_default();
        assert_eq!(r2r_default.history, HistoryPolicy::SystemDefault);
        assert_eq!(r2r_default.depth, 0, "RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT");
        assert_eq!(bus_qos(&r2r_default), Ok(Qos { depth: 1, ..Qos::system_default() }));
        // one policy spelled out, the others left to the system
        assert_eq!(bus_qos(&QosProfile::system_default().keep_last(5)), Ok(Qos::default().keep_last(5)));
        assert_eq!(bus_qos(&QosProfile::system_default().transient_local()), Ok(Qos::latched()));
        assert_eq!(bus_qos(&QosProfile::system_default().best_effort()), Ok(Qos::system_default().best_effort()));
        // a depth with the system-default history is still KEEP_LAST depth
        assert_eq!(bus_qos(&QosProfile { depth: 7, ..QosProfile::system_default() }), Ok(Qos::default().keep_last(7)));
        assert_eq!(bus_qos(&QosProfile::unknown()), Ok(Qos::system_default()));
        assert_eq!(bus_qos(&QosProfile::default().keep_last(0)), Ok(Qos::default().keep_last(1)));
    }

    #[test]
    fn transient_local_stays_transient_local() {
        for depth in [1, 5] {
            let qos = bus_qos(&QosProfile::default().keep_last(depth).transient_local()).unwrap();
            assert!(qos.is_transient_local());
            assert_eq!(qos.depth, depth);
            let qos = bus_qos(&QosProfile::sensor_data().keep_last(depth).transient_local()).unwrap();
            assert_eq!((qos.durability, qos.reliability), (Durability::TransientLocal, Reliability::BestEffort));
        }
    }

    #[test]
    fn what_the_bus_does_not_have_is_left_to_the_dds_endpoint() {
        let qos = QosProfile::default()
            .deadline(Duration::from_millis(100))
            .lifespan(Duration::from_secs(1))
            .liveliness(LivelinessPolicy::ManualByTopic)
            .liveliness_lease_duration(Duration::from_secs(2));
        assert_eq!(bus_qos(&qos), Ok(Qos::default()));
    }

    #[test]
    fn profiles_without_a_bus_equivalent_are_refused() {
        assert!(bus_qos(&QosProfile::default().keep_all()).is_err());
        assert!(bus_qos(&QosProfile::default().keep_last(i32::MAX as usize + 1)).is_err());
        assert!(bus_qos(&QosProfile::default().keep_last(i32::MAX as usize)).is_ok());
        assert!(bus_qos(&QosProfile::default().reliability(ReliabilityPolicy::BestAvailable)).is_err());
        assert!(bus_qos(&QosProfile::default().durability(DurabilityPolicy::BestAvailable)).is_err());
    }
}
