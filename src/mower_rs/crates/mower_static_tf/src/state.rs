//! robot_state_publisher 3.3.4's `/joint_states` callback, as a pure state
//! machine of the stamps it has published and the wall clock it saw last.
//!
//! - **Refused:** a message whose `name` and `position` lengths differ
//!   (rsp: an ERROR, or a WARN throttled to 10 s when there are no positions
//!   at all), with nothing changed.
//! - **The clock going back** (`now` before the previous callback's): every
//!   joint's last published stamp is forgotten, so the next message goes out.
//! - **The throttle:** the message goes out when its stamp is at least
//!   `publish_interval` past the least recent last-published stamp of its
//!   joints, a joint never published counting as 0 and the wall clock `now`
//!   capping it, or always with `ignore_timestamp`. At mower_base's 25 Hz and
//!   rsp's 20 Hz `publish_frequency` that is every other message.
//! - **What goes out:** the positions (the first of a name that comes twice),
//!   then the mimic joints whose joint has one (`position * multiplier +
//!   offset`, in the order of their names), each a transform of the moving
//!   joint it names, sorted by joint name, all with the message's stamp. A
//!   name that is no moving joint gives no transform; a message with none
//!   still goes out, empty. Then every name of the message has the stamp as
//!   its last published one.

use std::collections::BTreeMap;

use crate::urdf::Robot;

/// A joint state as rsp reads it.
#[derive(Clone, Debug, PartialEq)]
pub struct JointState<'a> {
    /// `header.stamp`, in nanoseconds (rclcpp::Time).
    pub stamp_ns: i64,
    pub name: &'a [String],
    pub position: &'a [f64],
}

/// What a joint state does.
#[derive(Clone, Debug, PartialEq)]
pub enum Outcome {
    /// Published: (joint, position) of each transform, in rsp's order.
    Publish(Vec<(String, f64)>),
    /// Within the interval of the last publish.
    Throttled,
    /// Refused: rsp's ERROR, or `empty_position` for its throttled WARN.
    Invalid { empty_position: bool },
}

#[derive(Clone, Debug)]
pub struct Throttle {
    interval_ns: i64,
    ignore_timestamp: bool,
    last_publish: BTreeMap<String, i64>,
    last_callback: Option<i64>,
    /// Set when the last callback found the clock gone back.
    pub moved_backwards: bool,
}

impl Throttle {
    /// `publish_interval_ms_ = milliseconds(uint64(1000.0 / publish_frequency))`.
    pub fn new(publish_frequency: f64, ignore_timestamp: bool) -> Throttle {
        let interval_ms = (1000.0 / publish_frequency) as u64;
        Throttle {
            interval_ns: (interval_ms as i64).saturating_mul(1_000_000),
            ignore_timestamp,
            last_publish: BTreeMap::new(),
            last_callback: None,
            moved_backwards: false,
        }
    }

    pub fn interval_ns(&self) -> i64 {
        self.interval_ns
    }

    /// The callback for `msg`, with the wall clock reading `now_ns`.
    pub fn on_joint_state(&mut self, robot: &Robot, msg: &JointState<'_>, now_ns: i64) -> Outcome {
        self.moved_backwards = false;
        if msg.name.len() != msg.position.len() {
            return Outcome::Invalid { empty_position: msg.position.is_empty() };
        }
        if self.last_callback.is_some_and(|last| last > now_ns) {
            self.moved_backwards = true;
            self.last_publish.clear();
        }
        self.last_callback = Some(now_ns);
        let mut last_published = now_ns;
        for name in msg.name {
            let t = *self.last_publish.entry(name.clone()).or_insert(0);
            if t < last_published {
                last_published = t;
            }
        }
        let max_publish = last_published.saturating_add(self.interval_ns);
        if !(self.ignore_timestamp || msg.stamp_ns >= max_publish) {
            return Outcome::Throttled;
        }
        let mut positions: BTreeMap<&str, f64> = BTreeMap::new();
        for (name, p) in msg.name.iter().zip(msg.position) {
            positions.entry(name.as_str()).or_insert(*p);
        }
        for (name, mimic) in robot.mimics() {
            if let Some(p) = positions.get(mimic.joint.as_str()).copied() {
                positions.entry(name).or_insert(p * mimic.multiplier + mimic.offset);
            }
        }
        let out = positions
            .into_iter()
            .filter(|(name, _)| robot.moving(name).is_some())
            .map(|(name, p)| (name.to_string(), p))
            .collect();
        for name in msg.name {
            self.last_publish.insert(name.clone(), msg.stamp_ns);
        }
        Outcome::Publish(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ROBOT: &str = r#"<robot name="t"><link name="b"/><link name="l"/><link name="r"/><link name="g"/>
        <joint name="left" type="continuous"><parent link="b"/><child link="l"/><axis xyz="0 0 1"/></joint>
        <joint name="right" type="continuous"><parent link="b"/><child link="r"/><axis xyz="0 0 1"/>
          <mimic joint="left" multiplier="-1" offset="0.5"/></joint>
        <joint name="gps" type="fixed"><parent link="b"/><child link="g"/></joint></robot>"#;

    const MS: i64 = 1_000_000;

    fn names(n: &[&str]) -> Vec<String> {
        n.iter().map(|s| s.to_string()).collect()
    }

    fn run(t: &mut Throttle, robot: &Robot, n: &[&str], p: &[f64], stamp_ms: i64, now_ms: i64) -> Outcome {
        let n = names(n);
        t.on_joint_state(robot, &JointState { stamp_ns: stamp_ms * MS, name: &n, position: p }, now_ms * MS)
    }

    #[test]
    fn every_other_message_at_25_hz_and_the_interval_is_inclusive() {
        let robot = crate::urdf::parse(ROBOT).unwrap();
        let mut t = Throttle::new(20.0, false);
        assert_eq!(t.interval_ns(), 50 * MS);
        let base = 1_793_000_000_000;
        let mut published = Vec::new();
        for k in 0..10 {
            let s = base + 40 * k;
            if let Outcome::Publish(_) = run(&mut t, &robot, &["left"], &[0.0], s, s + 1) {
                published.push(k);
            }
        }
        assert_eq!(published, [0, 2, 4, 6, 8]);
        // exactly 50 ms after the last published stamp goes; 1 ns less not
        let last = base + 40 * 8;
        let n = names(&["left"]);
        let js = |stamp_ns| JointState { stamp_ns, name: &n, position: &[1.0] };
        assert_eq!(t.on_joint_state(&robot, &js((last + 50) * MS - 1), (last + 60) * MS), Outcome::Throttled);
        assert!(matches!(t.on_joint_state(&robot, &js((last + 50) * MS), (last + 60) * MS), Outcome::Publish(_)));
    }

    #[test]
    fn mimics_unknown_and_fixed_names_and_the_order() {
        let robot = crate::urdf::parse(ROBOT).unwrap();
        let mut t = Throttle::new(20.0, false);
        // mimic: right = -left + 0.5, after the positions, by name
        assert_eq!(run(&mut t, &robot, &["left"], &[2.0], 1000, 1001),
                   Outcome::Publish(vec![("left".into(), 2.0), ("right".into(), -1.5)]));
        // a given position wins over the mimic's
        assert_eq!(run(&mut t, &robot, &["right", "left"], &[7.0, 2.0], 2000, 2001),
                   Outcome::Publish(vec![("left".into(), 2.0), ("right".into(), 7.0)]));
        // a fixed joint's name and an unknown one: no transform, but published
        // (empty) and remembered
        assert_eq!(run(&mut t, &robot, &["gps", "nope"], &[1.0, 1.0], 3000, 3001), Outcome::Publish(vec![]));
        assert_eq!(run(&mut t, &robot, &["gps"], &[1.0], 3010, 3011), Outcome::Throttled);
        // a name twice: the first position
        assert_eq!(run(&mut t, &robot, &["left", "left"], &[3.0, 4.0], 4000, 4001),
                   Outcome::Publish(vec![("left".into(), 3.0), ("right".into(), -2.5)]));
    }

    #[test]
    fn invalid_messages_change_nothing() {
        let robot = crate::urdf::parse(ROBOT).unwrap();
        let mut t = Throttle::new(20.0, false);
        assert_eq!(run(&mut t, &robot, &["left", "right"], &[1.0], 1000, 1001), Outcome::Invalid { empty_position: false });
        assert_eq!(run(&mut t, &robot, &["left"], &[], 1000, 1001), Outcome::Invalid { empty_position: true });
        assert!(matches!(run(&mut t, &robot, &["left"], &[1.0], 1000, 1001), Outcome::Publish(_)));
    }

    #[test]
    fn a_zero_stamp_never_goes_and_a_future_one_holds_the_rest_back() {
        let robot = crate::urdf::parse(ROBOT).unwrap();
        let mut t = Throttle::new(20.0, false);
        assert_eq!(run(&mut t, &robot, &["left"], &[1.0], 0, 1000), Outcome::Throttled);
        // ten seconds ahead goes; afterwards the wall clock caps the last
        // published stamp: a stamp of now is 50 ms short
        assert!(matches!(run(&mut t, &robot, &["left"], &[1.0], 11_000, 1040), Outcome::Publish(_)));
        assert_eq!(run(&mut t, &robot, &["left"], &[1.0], 1080, 1081), Outcome::Throttled);
        assert!(matches!(run(&mut t, &robot, &["left"], &[1.0], 1200, 1150), Outcome::Publish(_)));
    }

    #[test]
    fn the_wall_clock_going_back_forgets_what_was_published() {
        let robot = crate::urdf::parse(ROBOT).unwrap();
        let mut t = Throttle::new(20.0, false);
        assert!(matches!(run(&mut t, &robot, &["left"], &[1.0], 5000, 5001), Outcome::Publish(_)));
        assert_eq!(run(&mut t, &robot, &["left"], &[1.0], 5010, 5011), Outcome::Throttled);
        // back 10 s: the next one goes whatever its stamp
        assert!(matches!(run(&mut t, &robot, &["left"], &[1.0], 5020, 1), Outcome::Publish(_)));
        assert!(t.moved_backwards);
        let mut always = Throttle::new(20.0, true);
        for k in 0..3 {
            assert!(matches!(run(&mut always, &robot, &["left"], &[1.0], 5000 + k, 5001 + k), Outcome::Publish(_)));
        }
    }
}
