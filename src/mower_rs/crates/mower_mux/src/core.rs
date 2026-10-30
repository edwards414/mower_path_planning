//! twist_mux 4.5.0's selection rules, free of IO and of any clock: the
//! caller says when each message arrived (docs/ROS_FREE_PLAN.md step E4;
//! every rule below was measured against the real twist_mux by
//! `tools/mux_oracle.py`, and `tests/replay.rs` replays what it recorded).
//!
//! - **An input or a lock expires** when its timeout is positive and its last
//!   message arrived more than the timeout ago (strictly), or it never had
//!   one: twist_mux's `stamp_` starts at 0. A timeout of zero (or below) never
//!   expires, also without a message: such an input counts as fresh from the
//!   start and keeps masking the inputs below it.
//! - **A lock is locked** when it has expired or its last message was true.
//!   The lock priority is the highest priority of the locked locks, starting
//!   from 0 with a strict `<`, so a priority-0 lock changes nothing.
//! - **An input is masked** when it has expired or its priority is below the
//!   lock priority (`<`: a priority equal to the lock's is not masked).
//! - **The winner** is the first input, in handle order, of the highest
//!   priority among the unmasked ones, again from 0 with a strict `<`: a tie
//!   goes to the name that sorts first, and priority 0 never wins.
//! - **An input is forwarded** when it is the winner the moment it arrives,
//!   as it is, at once. Nothing else ever publishes: no timer, no zero after a
//!   timeout, and a lock message only changes the state.
//!
//! Arrival times are monotonic ([`Mono`]): the one deliberate difference from
//! twist_mux, which compares them on the node clock, i.e. the wall clock.
//! A wall-clock step therefore changes nothing here, where twist_mux would
//! see every input expire at once (a step forward) or take stale ones, a
//! stale "unlocked" among them, for fresh (a step back).

use mower_bus::Mono;

/// One `topics.<name>` or `locks.<name>` handle of the configuration.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Handle {
    /// The parameter prefix, e.g. `topics.app_manual`: twist_mux's handle name.
    pub name: String,
    /// The topic as the parameter gives it (diagnostics and logs show this).
    pub topic: String,
    /// `rclcpp::Duration` of the `timeout` seconds: the double product
    /// truncated to nanoseconds.
    pub timeout_ns: i64,
    /// Clamped to 0..=255, as twist_mux clamps it.
    pub priority: u8,
}

impl Handle {
    /// A handle from the parameter values, converted as twist_mux converts
    /// them.
    pub fn new(name: &str, topic: &str, timeout_s: f64, priority: i64) -> Handle {
        Handle {
            name: name.to_string(),
            topic: topic.to_string(),
            timeout_ns: timeout_ns(timeout_s),
            priority: priority.clamp(0, 255) as u8,
        }
    }

    /// `rclcpp::Duration::seconds()` of the timeout.
    pub fn timeout_s(&self) -> f64 {
        self.timeout_ns as f64 / 1e9
    }

    fn expired(&self, last: Option<Mono>, now: Mono) -> bool {
        self.timeout_ns > 0
            && match last {
                None => true,
                Some(t) => now.ns() as i128 - t.ns() as i128 > self.timeout_ns as i128,
            }
    }
}

/// `std::chrono::duration_cast<nanoseconds>(duration<double>(s))`: the product
/// in double, truncated toward zero (saturating, where C++ would be UB).
pub fn timeout_ns(seconds: f64) -> i64 {
    (seconds * 1e9) as i64
}

#[derive(Clone, Debug)]
struct Input {
    handle: Handle,
    last: Option<Mono>,
}

#[derive(Clone, Debug)]
struct Lock {
    handle: Handle,
    last: Option<Mono>,
    data: bool,
}

/// The mux's state: when each input and lock last had a message, and each
/// lock's last value. Handles keep the order they are given in, which
/// `config::Config` makes twist_mux's.
#[derive(Clone, Debug)]
pub struct Mux {
    inputs: Vec<Input>,
    locks: Vec<Lock>,
}

impl Mux {
    /// Everything expired and every lock with a positive timeout locked, as
    /// twist_mux is when it starts.
    pub fn new(inputs: &[Handle], locks: &[Handle]) -> Mux {
        Mux {
            inputs: inputs.iter().map(|h| Input { handle: h.clone(), last: None }).collect(),
            locks: locks.iter().map(|h| Lock { handle: h.clone(), last: None, data: false }).collect(),
        }
    }

    pub fn inputs(&self) -> impl Iterator<Item = &Handle> {
        self.inputs.iter().map(|i| &i.handle)
    }

    pub fn locks(&self) -> impl Iterator<Item = &Handle> {
        self.locks.iter().map(|l| &l.handle)
    }

    /// Input `i` had a message at `now`: whether to forward it.
    pub fn on_input(&mut self, i: usize, now: Mono) -> bool {
        self.inputs[i].last = Some(now);
        self.winner(now) == Some(i)
    }

    /// Lock `j` had `data` at `now`. Never forwards anything.
    pub fn on_lock(&mut self, j: usize, data: bool, now: Mono) {
        let lock = &mut self.locks[j];
        lock.last = Some(now);
        lock.data = data;
    }

    /// Whether lock `j` is locked at `now`: expired or true.
    pub fn is_locked(&self, j: usize, now: Mono) -> bool {
        let lock = &self.locks[j];
        lock.handle.expired(lock.last, now) || lock.data
    }

    /// The highest priority of the locked locks, from 0 (strict `<`).
    pub fn lock_priority(&self, now: Mono) -> u8 {
        let mut priority = 0;
        for j in 0..self.locks.len() {
            if self.is_locked(j, now) && priority < self.locks[j].handle.priority {
                priority = self.locks[j].handle.priority;
            }
        }
        priority
    }

    /// Whether input `i` is masked at `now` under `lock_priority`.
    pub fn is_masked(&self, i: usize, lock_priority: u8, now: Mono) -> bool {
        let input = &self.inputs[i];
        input.handle.expired(input.last, now) || input.handle.priority < lock_priority
    }

    /// The first input of the highest priority among the unmasked ones, from
    /// 0 (strict `<`); `None` when no input with a priority above 0 is
    /// unmasked.
    pub fn winner(&self, now: Mono) -> Option<usize> {
        let lock_priority = self.lock_priority(now);
        let mut priority = 0;
        let mut winner = None;
        for i in 0..self.inputs.len() {
            if !self.is_masked(i, lock_priority, now) && priority < self.inputs[i].handle.priority {
                priority = self.inputs[i].handle.priority;
                winner = Some(i);
            }
        }
        winner
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: u64 = 1_000_000;

    fn at(ms: u64) -> Mono {
        Mono::from_ns(ms * MS)
    }

    /// The production handles, in twist_mux's order (twist_mux_topics.yaml).
    fn production() -> Mux {
        let inputs = [
            Handle::new("topics.app_manual", "/joy_cmd", 0.5, 90),
            Handle::new("topics.keyboard", "/keyboard_cmd_vel", 0.5, 80),
            Handle::new("topics.navigation", "/nav_cmd_vel", 0.5, 10),
            Handle::new("topics.navigation_safety_stop", "/navigation_safety_stop", 0.2, 50),
            Handle::new("topics.physical_joystick", "/physical_joy_cmd", 0.5, 100),
        ];
        let locks = [Handle::new("locks.navigation_coordinator", "/navigation_coordinator_lock", 0.3, 50)];
        Mux::new(&inputs, &locks)
    }

    const JOY: usize = 0;
    const KB: usize = 1;
    const NAV: usize = 2;
    const SAFETY: usize = 3;
    const PJOY: usize = 4;
    const LOCK: usize = 0;

    #[test]
    fn the_timeouts_are_the_truncated_nanoseconds() {
        assert_eq!(timeout_ns(0.2), 200_000_000);
        assert_eq!(timeout_ns(0.3), 300_000_000);
        assert_eq!(timeout_ns(0.5), 500_000_000);
        // 0.1 + 0.2 is 0.30000000000000004: 300000000.00000006 truncates
        assert_eq!(timeout_ns(0.1 + 0.2), 300_000_000);
        assert_eq!(timeout_ns(1e-10), 0);
        assert_eq!(timeout_ns(-0.5), -500_000_000);
        assert_eq!(Handle::new("a", "/a", 0.5, 300).priority, 255);
        assert_eq!(Handle::new("a", "/a", 0.5, -3).priority, 0);
        assert_eq!(Handle::new("a", "/a", 0.2, 1).timeout_s(), 0.2);
    }

    /// 3.4 #8: a lock that never had a message is expired, so locked: nav
    /// (10) never passes, from the start.
    #[test]
    fn a_lock_never_heard_from_is_locked() {
        let mut mux = production();
        for k in 0..20 {
            assert!(!mux.on_input(NAV, at(k * 50)));
        }
        assert_eq!(mux.lock_priority(at(0)), 50);
    }

    /// 3.4 #9: a fresh false unlocks; the lock message itself forwards
    /// nothing, and nav waits for its own next message.
    #[test]
    fn a_fresh_false_lock_unlocks_and_publishes_nothing() {
        let mut mux = production();
        assert!(!mux.on_input(NAV, at(0)));
        mux.on_lock(LOCK, false, at(10));
        assert_eq!(mux.lock_priority(at(10)), 0);
        assert!(mux.on_input(NAV, at(50)));
    }

    /// 3.4 #10: a true lock masks what is below 50 (nav), not what equals it
    /// (the safety stop).
    #[test]
    fn a_true_lock_masks_below_its_priority_not_at_it() {
        let mut mux = production();
        mux.on_lock(LOCK, true, at(0));
        assert!(!mux.on_input(NAV, at(10)));
        assert!(mux.on_input(SAFETY, at(20)));
        assert!(mux.is_masked(NAV, 50, at(20)) && !mux.is_masked(SAFETY, 50, at(20)));
    }

    /// 3.4 #11 at the nanosecond: a lock 0.3 s old exactly still holds, one
    /// nanosecond more and it is expired.
    #[test]
    fn a_lock_expires_one_nanosecond_after_its_timeout() {
        let mut mux = production();
        mux.on_lock(LOCK, false, at(1000));
        let edge = 1000 * MS + 300 * MS;
        assert!(mux.on_input(NAV, Mono::from_ns(edge)));
        assert!(!mux.is_locked(LOCK, Mono::from_ns(edge)));
        assert!(!mux.on_input(NAV, Mono::from_ns(edge + 1)));
        assert!(mux.is_locked(LOCK, Mono::from_ns(edge + 1)));
    }

    /// 3.4 #6 at the nanosecond: joy 0.5 s old exactly still wins over the
    /// safety stop; one nanosecond more and the safety stop is forwarded.
    #[test]
    fn an_input_expires_one_nanosecond_after_its_timeout() {
        let mut mux = production();
        mux.on_lock(LOCK, true, at(0));
        assert!(mux.on_input(JOY, at(100)));
        let edge = 100 * MS + 500 * MS;
        mux.on_lock(LOCK, true, Mono::from_ns(edge - 10 * MS));
        assert!(!mux.on_input(SAFETY, Mono::from_ns(edge)));
        assert!(mux.on_input(SAFETY, Mono::from_ns(edge + 1)));
    }

    /// 3.4 #3: of several fresh inputs only the highest goes; the others are
    /// dropped, not delayed.
    #[test]
    fn only_the_highest_fresh_input_goes() {
        let mut mux = production();
        mux.on_lock(LOCK, true, at(0));
        assert!(mux.on_input(SAFETY, at(10)));
        assert!(mux.on_input(JOY, at(20)));
        assert!(!mux.on_input(SAFETY, at(30)));
        assert!(mux.on_input(KB, at(600)), "joy expired at 520");
        assert!(mux.on_input(PJOY, at(610)));
        assert!(!mux.on_input(KB, at(620)));
        assert!(mux.on_input(JOY, at(1200)), "both teleops expired");
    }

    /// 3.4 #4, #5: a tie goes to the name that sorts first, whatever the
    /// order of the yaml; priority 0 is never selected.
    #[test]
    fn a_tie_goes_to_the_first_name_and_priority_zero_never_wins() {
        // twist_mux's order: aaa, bbb (config::Config sorts them)
        let inputs = [Handle::new("topics.aaa", "/a", 0.5, 50), Handle::new("topics.bbb", "/b", 0.5, 50),
                      Handle::new("topics.zero", "/z", 0.5, 0)];
        let mut mux = Mux::new(&inputs, &[]);
        assert!(mux.on_input(1, at(0)), "bbb alone");
        assert!(mux.on_input(0, at(10)));
        assert!(!mux.on_input(1, at(20)), "aaa fresh: the tie is aaa's");
        let mut zero = Mux::new(&inputs[2..], &[]);
        assert!(!zero.on_input(0, at(0)));
        assert_eq!(zero.winner(at(0)), None);
    }

    /// 3.4 #12, #13: timeout 0 never expires. Such an input is fresh before
    /// its first message and masks what is below it from the start; such a
    /// lock is free until its first true, then locked for good.
    #[test]
    fn timeout_zero_never_expires() {
        let mut never = Mux::new(&[Handle::new("topics.never", "/n", 0.0, 40), Handle::new("topics.low", "/l", 0.5, 30)], &[]);
        assert!(!never.on_input(1, at(0)), "never masks low without a message");
        assert!(never.on_input(0, at(10_000)));
        assert!(!never.on_input(1, at(3_600_000)));
        let mut quiet = Mux::new(&[Handle::new("topics.in50", "/i", 0.5, 50)], &[Handle::new("locks.quiet", "/q", 0.0, 60)]);
        assert!(quiet.on_input(0, at(0)), "a timeout-0 lock never heard from is free");
        quiet.on_lock(0, true, at(100));
        assert!(!quiet.on_input(0, at(3_600_000)), "and locked for good after a true");
        quiet.on_lock(0, false, at(3_600_001));
        assert!(quiet.on_input(0, at(3_600_002)));
    }

    /// A priority-0 lock is locked when expired, but changes nothing
    /// (twist_mux warns it is redundant).
    #[test]
    fn a_priority_zero_lock_changes_nothing() {
        let mut mux = Mux::new(&[Handle::new("topics.n", "/n", 0.5, 1)], &[Handle::new("locks.z", "/z", 0.5, 0)]);
        assert!(mux.is_locked(0, at(0)));
        assert_eq!(mux.lock_priority(at(0)), 0);
        assert!(mux.on_input(0, at(0)));
        mux.on_lock(0, true, at(1));
        assert!(mux.on_input(0, at(2)));
    }

    /// 3.4 #7, #15: with every input expired nothing is forwarded, and the
    /// mux has no output of its own: only an arriving input can be.
    #[test]
    fn nothing_goes_once_everything_expired() {
        let mut mux = production();
        mux.on_lock(LOCK, true, at(0));
        assert!(mux.on_input(SAFETY, at(0)));
        assert_eq!(mux.winner(at(200)), Some(SAFETY));
        assert_eq!(mux.winner(at(201)), None, "nothing after the safety stop's 0.2 s");
    }

    /// The release sequence of 3.5: the manual guard's zero 0.2 s after the
    /// last joy is forwarded, then nothing for 0.5 s (joy still fresh masks
    /// the safety stop), then the safety stop at 20 Hz.
    #[test]
    fn the_half_second_of_silence_after_the_operator_lets_go() {
        let mut mux = production();
        let mut out = Vec::new();
        for k in 0..60u64 {
            let t = k * 50;
            mux.on_lock(LOCK, true, at(t));
            if mux.on_input(SAFETY, at(t + 1)) {
                out.push(("safety", t));
            }
            if (500..1500).contains(&t) && t % 100 == 0 && mux.on_input(JOY, at(t + 2)) {
                out.push(("joy", t));
            }
            if t == 1600 && mux.on_input(JOY, at(t + 2)) {
                out.push(("zero", t));
            }
        }
        let safety_after: Vec<u64> = out.iter().filter(|(s, t)| *s == "safety" && *t > 1000).map(|(_, t)| *t).collect();
        assert_eq!(out.iter().find(|(s, _)| *s == "zero"), Some(&("zero", 1600)));
        // the zero at 1602 keeps joy fresh until 2102: safety from 2150 on
        assert_eq!(safety_after.first(), Some(&2150));
    }
}
