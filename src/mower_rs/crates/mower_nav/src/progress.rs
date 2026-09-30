//! Coverage execution progress (`/coverage_progress`,
//! `/coverage_progress_status`): the bookkeeping behind
//! `mower_interface/msg/CoverageProgress`, kept free of ROS types so it can
//! be unit tested. Progress is weighted by the length of each split segment
//! handed to Nav2 FollowPath; driving to the coverage start counts as 0 %.
//! `mower_mission/navigation/coverage_progress.py` is the Python twin and
//! must stay in step.

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Status {
    #[default]
    Idle = 0,
    NavigatingToStart = 1,
    Running = 2,
    Canceling = 3,
    Canceled = 4,
    Succeeded = 5,
    Failed = 6,
}

impl Status {
    pub fn text(self) -> &'static str {
        match self {
            Status::Idle => "idle",
            Status::NavigatingToStart => "navigating_to_start",
            Status::Running => "running",
            Status::Canceling => "canceling",
            Status::Canceled => "canceled",
            Status::Succeeded => "succeeded",
            Status::Failed => "failed",
        }
    }

    pub fn is_terminal(self) -> bool {
        matches!(self, Status::Idle | Status::Canceled | Status::Succeeded | Status::Failed)
    }
}

/// `P` is the pose type of the segment start / goal (a PoseStamped in the node).
#[derive(Debug, Clone, Default)]
pub struct Progress<P> {
    pub status: Status,
    pub mission_id: String,
    pub zone_id: i32,
    /// 1-based; 0 while no coverage segment is active.
    pub current_segment_index: i32,
    pub completed_segments: i32,
    pub current_segment_progress: f64,
    pub overall_progress: f64,
    pub completed_distance_m: f64,
    pub total_distance_m: f64,
    pub remaining_distance_m: f64,
    pub current_segment_start: P,
    pub current_segment_goal: P,
    pub message: String,
    segment_distances: Vec<f64>,
    /// Distance of the segments already completed.
    completed_before_m: f64,
}

impl<P: Clone + Default> Progress<P> {
    pub fn total_segments(&self) -> i32 {
        self.segment_distances.len() as i32
    }

    pub fn current_segment_distance_m(&self) -> f64 {
        self.segment_distance(self.current_segment_index)
    }

    fn segment_distance(&self, index: i32) -> f64 {
        if index < 1 {
            return 0.0;
        }
        self.segment_distances.get(index as usize - 1).copied().unwrap_or(0.0)
    }

    /// A new execution: everything reset, driving towards the start.
    pub fn begin(&mut self, mission_id: &str, zone_id: i32) {
        *self = Self::default();
        self.status = Status::NavigatingToStart;
        self.mission_id = mission_id.to_string();
        self.zone_id = zone_id;
        self.message = "Coverage navigation accepted".into();
    }

    /// The split segments' lengths, known before the start is reached.
    pub fn set_segments(&mut self, distances: Vec<f64>) {
        self.segment_distances = distances.into_iter().map(|d| if d.is_finite() && d > 0.0 { d } else { 0.0 }).collect();
        self.total_distance_m = self.segment_distances.iter().sum();
        self.recompute(0.0);
    }

    pub fn start_segment(&mut self, index: i32, start: P, goal: P) {
        if self.status == Status::Canceling {
            return;
        }
        self.status = Status::Running;
        self.current_segment_index = index;
        self.current_segment_progress = 0.0;
        self.current_segment_start = start;
        self.current_segment_goal = goal;
        self.message = format!("Following coverage segment {index}/{}", self.total_segments());
        self.recompute(0.0);
    }

    /// Nav2 FollowPath feedback: distance left along the active segment.
    /// Returns whether anything changed. Progress within a segment never
    /// goes backwards (the controller's estimate jitters).
    pub fn update_remaining(&mut self, remaining_m: f64) -> bool {
        if !matches!(self.status, Status::Running | Status::Canceling) || self.current_segment_index < 1 || !remaining_m.is_finite() {
            return false;
        }
        let d = self.current_segment_distance_m();
        if d <= 0.0 {
            return false;
        }
        let fraction = ((d - remaining_m.max(0.0)) / d).clamp(0.0, 1.0);
        if fraction <= self.current_segment_progress {
            return false;
        }
        self.current_segment_progress = fraction;
        self.recompute(fraction * d);
        true
    }

    pub fn complete_segment(&mut self) {
        let d = self.current_segment_distance_m();
        self.completed_segments += 1;
        self.completed_before_m += d;
        self.current_segment_progress = 1.0;
        self.recompute(0.0);
    }

    pub fn canceling(&mut self, message: &str) {
        if self.status.is_terminal() {
            return;
        }
        self.status = Status::Canceling;
        self.message = message.to_string();
    }

    /// Final state. Success fills everything up; cancel and failure keep
    /// the last distances so the app can show how far the mower got.
    pub fn finish(&mut self, status: Status, message: &str) {
        self.status = status;
        self.message = message.to_string();
        if status == Status::Succeeded {
            self.completed_segments = self.total_segments();
            self.completed_before_m = self.total_distance_m;
            self.current_segment_progress = 1.0;
            self.recompute(0.0);
            self.overall_progress = 1.0;
        }
    }

    fn recompute(&mut self, done_in_segment_m: f64) {
        self.completed_distance_m = (self.completed_before_m + done_in_segment_m).min(self.total_distance_m);
        self.remaining_distance_m = (self.total_distance_m - self.completed_distance_m).max(0.0);
        self.overall_progress = if self.total_distance_m > 0.0 {
            (self.completed_distance_m / self.total_distance_m).clamp(0.0, 1.0)
        } else {
            0.0
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    type P = Progress<(f64, f64)>;

    fn three_segments() -> P {
        let mut p = P::default();
        p.begin("d-1", 3);
        p.set_segments(vec![10.0, 20.0, 70.0]);
        p
    }

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    #[test]
    fn begin_resets_and_totals_the_segments() {
        let mut p = three_segments();
        p.start_segment(2, (0.0, 0.0), (1.0, 0.0));
        p.begin("d-2", 4);
        p.set_segments(vec![10.0, 20.0, 70.0]);
        assert_eq!(p.status, Status::NavigatingToStart);
        assert_eq!((p.mission_id.as_str(), p.zone_id), ("d-2", 4));
        assert_eq!((p.current_segment_index, p.total_segments(), p.completed_segments), (0, 3, 0));
        assert!(close(p.total_distance_m, 100.0) && close(p.remaining_distance_m, 100.0) && close(p.overall_progress, 0.0));
    }

    #[test]
    fn feedback_moves_the_segment_and_distance_weighted_overall() {
        let mut p = three_segments();
        p.start_segment(1, (0.0, 0.0), (10.0, 0.0));
        assert!(p.update_remaining(4.0));
        assert!(close(p.current_segment_progress, 0.6));
        assert!(close(p.completed_distance_m, 6.0) && close(p.overall_progress, 0.06));
        p.complete_segment();
        assert_eq!(p.completed_segments, 1);
        assert!(close(p.overall_progress, 0.10));
        p.start_segment(2, (10.0, 0.0), (10.0, 20.0));
        assert!(close(p.current_segment_progress, 0.0));
        p.update_remaining(10.0);
        // by distance 20 / 100, not by segment count (1.5 / 3)
        assert!(close(p.overall_progress, 0.20));
        assert!(close(p.remaining_distance_m, 80.0));
    }

    #[test]
    fn segment_progress_never_goes_backwards_or_outside_the_segment() {
        let mut p = three_segments();
        p.start_segment(1, (0.0, 0.0), (10.0, 0.0));
        p.update_remaining(4.0);
        assert!(!p.update_remaining(5.0));
        assert!(close(p.current_segment_progress, 0.6));
        p.update_remaining(-3.0);
        assert!(close(p.current_segment_progress, 1.0) && close(p.completed_distance_m, 10.0));
        assert!(!p.update_remaining(f64::NAN));
    }

    #[test]
    fn feedback_before_the_first_segment_is_ignored() {
        let mut p = three_segments();
        assert!(!p.update_remaining(2.0));
        assert!(close(p.overall_progress, 0.0));
    }

    #[test]
    fn success_fills_up_and_cancel_or_failure_keep_the_last_progress() {
        let mut p = three_segments();
        p.start_segment(1, (0.0, 0.0), (10.0, 0.0));
        p.complete_segment();
        p.start_segment(2, (0.0, 0.0), (10.0, 0.0));
        p.update_remaining(5.0);
        let mut canceled = p.clone();
        canceled.canceling("cancel requested");
        assert_eq!(canceled.status, Status::Canceling);
        canceled.finish(Status::Canceled, "canceled");
        assert_eq!((canceled.completed_segments, canceled.current_segment_index), (1, 2));
        assert!(close(canceled.completed_distance_m, 25.0));
        let mut failed = p.clone();
        failed.finish(Status::Failed, "Nav2 failed");
        assert!(close(failed.overall_progress, 0.25));
        p.finish(Status::Succeeded, "done");
        assert_eq!(p.completed_segments, 3);
        assert!(close(p.overall_progress, 1.0) && close(p.remaining_distance_m, 0.0));
    }

    #[test]
    fn a_canceling_mission_does_not_flip_back_to_running() {
        let mut p = three_segments();
        p.canceling("stop");
        p.start_segment(1, (0.0, 0.0), (1.0, 0.0));
        assert_eq!(p.status, Status::Canceling);
        let mut done = three_segments();
        done.finish(Status::Failed, "x");
        done.canceling("late");
        assert_eq!(done.status, Status::Failed);
    }

    #[test]
    fn zero_length_plans_do_not_divide_by_zero() {
        let mut p = P::default();
        p.begin("d", -1);
        p.set_segments(vec![0.0, f64::NAN]);
        p.start_segment(1, (0.0, 0.0), (0.0, 0.0));
        assert!(!p.update_remaining(0.0));
        assert!(close(p.overall_progress, 0.0));
        p.finish(Status::Succeeded, "done");
        assert!(close(p.overall_progress, 1.0));
    }
}
