//! Quality of service, with r2r's builder names so E2 diffs stay one-to-one
//! (README.md, "Topics", has the mapping and what each field does in
//! process).

/// What a reader that joins late gets.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Durability {
    /// Only what is published after it subscribed.
    Volatile,
    /// Also the retained history of every live `TransientLocal` writer.
    TransientLocal,
}

/// Kept for introspection and the ROS bridge; in process nothing is lost
/// except to a full KEEP_LAST queue, whichever this says.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Reliability {
    Reliable,
    BestEffort,
}

/// KEEP_LAST `depth` plus reliability and durability. A depth of 0 counts
/// as 1.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Qos {
    pub depth: usize,
    pub reliability: Reliability,
    pub durability: Durability,
}

impl Default for Qos {
    fn default() -> Self {
        Qos::default()
    }
}

impl Qos {
    /// rclcpp's / r2r's default: keep last 10, reliable, volatile.
    #[allow(clippy::should_implement_trait)]
    pub const fn default() -> Self {
        Qos { depth: 10, reliability: Reliability::Reliable, durability: Durability::Volatile }
    }

    /// `rmw_qos_profile_sensor_data`: keep last 5, best effort, volatile.
    pub const fn sensor_data() -> Self {
        Qos { depth: 5, reliability: Reliability::BestEffort, durability: Durability::Volatile }
    }

    /// The latched topics' profile: keep last 1, reliable, transient local.
    pub const fn latched() -> Self {
        Qos { depth: 1, reliability: Reliability::Reliable, durability: Durability::TransientLocal }
    }

    pub const fn keep_last(self, depth: usize) -> Self {
        Qos { depth, ..self }
    }

    pub const fn reliable(self) -> Self {
        Qos { reliability: Reliability::Reliable, ..self }
    }

    pub const fn best_effort(self) -> Self {
        Qos { reliability: Reliability::BestEffort, ..self }
    }

    pub const fn volatile(self) -> Self {
        Qos { durability: Durability::Volatile, ..self }
    }

    pub const fn transient_local(self) -> Self {
        Qos { durability: Durability::TransientLocal, ..self }
    }

    /// The queue length actually used: `depth`, at least 1.
    pub const fn effective_depth(self) -> usize {
        if self.depth == 0 { 1 } else { self.depth }
    }

    pub const fn is_transient_local(self) -> bool {
        matches!(self.durability, Durability::TransientLocal)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn builders_match_the_r2r_chains_in_use() {
        assert_eq!(Qos::default().keep_last(1).reliable().transient_local(), Qos::latched());
        assert_eq!(Qos::default(), <Qos as Default>::default());
        let q = Qos::default().keep_last(1).best_effort().volatile();
        assert_eq!((q.depth, q.reliability, q.durability), (1, Reliability::BestEffort, Durability::Volatile));
        assert_eq!(Qos::sensor_data().depth, 5);
        assert_eq!(Qos::default().keep_last(0).effective_depth(), 1);
        assert!(Qos::latched().is_transient_local() && !Qos::default().is_transient_local());
    }
}
