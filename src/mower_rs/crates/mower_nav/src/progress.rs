//! Coverage execution progress (`/coverage_progress`,
//! `/coverage_progress_status`): the bookkeeping behind
//! `mower_interface/msg/CoverageProgress`, kept free of ROS types so it can
//! be unit tested. Progress is weighted by the length of each split segment
//! handed to Nav2 FollowPath; driving to the coverage start counts as 0 %.
//! `mower_mission/navigation/coverage_progress.py` is the Python twin and
//! must stay in step (including the path hash and the checkpoint JSON, so a
//! checkpoint written by one server can be resumed by the other).
//!
//! Checkpoint (M2) and resume (M3): the node writes a [`Checkpoint`] as the
//! execution advances; a later goal with `resume_segment_index > 0` is only
//! admitted when [`Checkpoint::resume_block_reason`] finds the same path
//! (hash over zone, poses, split points and the split parameters, i.e. the
//! same segments) and a segment no further than the first unfinished one.

use sha2::{Digest, Sha256};

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
    /// [`path_hash`] of the executed goal; empty until the plan is split.
    pub path_hash: String,
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

    /// A resumed execution: segments before `index` (1-based) count as done.
    pub fn resume_from(&mut self, index: i32) {
        let done = (index - 1).clamp(0, self.total_segments());
        self.completed_segments = done;
        self.completed_before_m = self.segment_distances[..done as usize].iter().sum();
        self.message = format!("Resuming at coverage segment {index}/{}", self.total_segments());
        self.recompute(0.0);
    }

    /// The durable part, for the checkpoint file.
    pub fn checkpoint(&self, updated_at: f64) -> Checkpoint {
        Checkpoint {
            mission_id: self.mission_id.clone(),
            zone_id: self.zone_id,
            status: self.status.text().to_string(),
            current_segment_index: self.current_segment_index,
            total_segments: self.total_segments(),
            completed_segments: self.completed_segments,
            completed_distance_m: self.completed_distance_m,
            total_distance_m: self.total_distance_m,
            overall_progress: self.overall_progress,
            path_hash: self.path_hash.clone(),
            updated_at,
        }
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

/// Identity of an executed coverage plan. Same text in both servers:
/// `v1;zone=<id>;tol=..;max=..;turn=..;min=..;|x,y,z;...|x,y;...` with
/// every number as `{:.4}`, hashed with SHA-256 (lowercase hex).
pub fn path_hash(zone_id: i32, split_params: [f64; 4], poses: &[[f64; 3]], split_points: &[[f64; 2]]) -> String {
    let [tol, max, turn, min] = split_params;
    let mut text = format!("v1;zone={zone_id};tol={tol:.4};max={max:.4};turn={turn:.4};min={min:.4};|");
    for [x, y, z] in poses {
        text.push_str(&format!("{x:.4},{y:.4},{z:.4};"));
    }
    text.push('|');
    for [x, y] in split_points {
        text.push_str(&format!("{x:.4},{y:.4};"));
    }
    Sha256::digest(text.as_bytes()).iter().map(|b| format!("{b:02x}")).collect()
}

pub const CHECKPOINT_VERSION: i64 = 1;

/// The checkpoint file (`progress_checkpoint_path`), spec section 8.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Checkpoint {
    pub mission_id: String,
    pub zone_id: i32,
    pub status: String,
    pub current_segment_index: i32,
    pub total_segments: i32,
    pub completed_segments: i32,
    pub completed_distance_m: f64,
    pub total_distance_m: f64,
    pub overall_progress: f64,
    pub path_hash: String,
    /// Unix time, seconds.
    pub updated_at: f64,
}

impl Checkpoint {
    pub fn to_json(&self) -> String {
        serde_json::json!({
            "version": CHECKPOINT_VERSION,
            "mission_id": self.mission_id,
            "zone_id": self.zone_id,
            "status": self.status,
            "current_segment_index": self.current_segment_index,
            "total_segments": self.total_segments,
            "completed_segments": self.completed_segments,
            "completed_distance_m": self.completed_distance_m,
            "total_distance_m": self.total_distance_m,
            "overall_progress": self.overall_progress,
            "path_hash": self.path_hash,
            "updated_at": self.updated_at,
        })
        .to_string()
    }

    /// `None` for anything that is not a well-formed version-1 checkpoint.
    pub fn from_json(text: &str) -> Option<Checkpoint> {
        let v: serde_json::Value = serde_json::from_str(text).ok()?;
        if v.get("version")?.as_i64()? != CHECKPOINT_VERSION {
            return None;
        }
        let int = |k: &str| v.get(k).and_then(|x| x.as_i64()).and_then(|x| i32::try_from(x).ok());
        let num = |k: &str| v.get(k).and_then(|x| x.as_f64()).filter(|x| x.is_finite());
        let text = |k: &str| v.get(k).and_then(|x| x.as_str()).map(str::to_string);
        Some(Checkpoint {
            mission_id: text("mission_id")?,
            zone_id: int("zone_id")?,
            status: text("status")?,
            current_segment_index: int("current_segment_index")?,
            total_segments: int("total_segments")?,
            completed_segments: int("completed_segments")?,
            completed_distance_m: num("completed_distance_m")?,
            total_distance_m: num("total_distance_m")?,
            overall_progress: num("overall_progress")?,
            path_hash: text("path_hash")?,
            updated_at: num("updated_at").unwrap_or(0.0),
        })
    }

    /// Unfinished work that a resume could pick up.
    pub fn resumable(&self) -> bool {
        self.status != Status::Succeeded.text()
            && !self.path_hash.is_empty()
            && self.total_segments > 0
            && (0..self.total_segments).contains(&self.completed_segments)
    }

    /// The segment a resume starts at (1-based): the first unfinished one.
    pub fn resume_segment_index(&self) -> i32 {
        self.completed_segments + 1
    }

    /// Why a goal for `path_hash` may not start at `index`; `None` = allowed.
    pub fn resume_block_reason(&self, path_hash: &str, index: i32) -> Option<String> {
        if !self.resumable() {
            return Some("no unfinished coverage checkpoint to resume".into());
        }
        if self.path_hash != path_hash {
            return Some("coverage path or split parameters changed since the checkpoint; regenerate and start over".into());
        }
        if index < 1 || index > self.resume_segment_index() {
            return Some(format!(
                "resume segment {index} is outside 1..={} (segments {} of {} were completed)",
                self.resume_segment_index(),
                self.completed_segments,
                self.total_segments
            ));
        }
        None
    }

    /// Progress to report after a restart: an execution that was still
    /// running when the checkpoint was written was interrupted.
    pub fn restored_progress<P: Clone + Default>(&self) -> Progress<P> {
        let mut p = Progress::<P>::default();
        p.status = match self.status.as_str() {
            "succeeded" => Status::Succeeded,
            "canceled" => Status::Canceled,
            _ => Status::Failed,
        };
        p.mission_id = self.mission_id.clone();
        p.zone_id = self.zone_id;
        p.current_segment_index = self.current_segment_index;
        p.completed_segments = self.completed_segments;
        p.completed_distance_m = self.completed_distance_m;
        p.total_distance_m = self.total_distance_m;
        p.remaining_distance_m = (self.total_distance_m - self.completed_distance_m).max(0.0);
        p.overall_progress = self.overall_progress;
        p.path_hash = self.path_hash.clone();
        // total_segments without the lengths: equal parts are enough to report
        let n = self.total_segments.max(0) as usize;
        p.segment_distances = vec![if n > 0 { self.total_distance_m / n as f64 } else { 0.0 }; n];
        p.message = if p.status == Status::Failed && self.status != "failed" {
            format!("Restored from checkpoint; the execution was interrupted while {}", self.status)
        } else {
            "Restored from checkpoint".into()
        };
        p
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
    fn resume_counts_the_earlier_segments_as_done() {
        let mut p = three_segments();
        p.resume_from(3);
        assert_eq!(p.completed_segments, 2);
        assert!(close(p.completed_distance_m, 30.0) && close(p.overall_progress, 0.30));
        p.start_segment(3, (0.0, 0.0), (1.0, 0.0));
        p.update_remaining(35.0);
        assert!(close(p.overall_progress, 0.65));
        let mut q = three_segments();
        q.resume_from(1);
        assert_eq!(q.completed_segments, 0);
        q.resume_from(9);
        assert_eq!(q.completed_segments, 3);
    }

    /// Pinned: the Python twin must produce the same digest.
    #[test]
    fn path_hash_is_pinned_and_sensitive_to_every_input() {
        let poses = [[0.0, 0.0, 0.0], [1.0, 0.5, 0.0], [2.0, -0.25, 0.0]];
        let splits = [[1.0, 0.5]];
        let params = [0.1, 5.0, 0.8, 0.25];
        let h = path_hash(3, params, &poses, &splits);
        assert_eq!(h, "068606bf6d49b4059c29ad4ebc76d2cc15a146329e9fade2f9e6f14745c9b79a");
        assert_ne!(h, path_hash(4, params, &poses, &splits));
        assert_ne!(h, path_hash(3, [0.1, 6.0, 0.8, 0.25], &poses, &splits));
        assert_ne!(h, path_hash(3, params, &poses[..2], &splits));
        assert_ne!(h, path_hash(3, params, &poses, &[]));
        // below the 0.1 mm print precision: same plan
        assert_eq!(h, path_hash(3, params, &[[0.00001, 0.0, 0.0], poses[1], poses[2]], &splits));
    }

    fn checkpoint_after_segment_one() -> Checkpoint {
        let mut p = three_segments();
        p.path_hash = "abc".into();
        p.start_segment(1, (0.0, 0.0), (1.0, 0.0));
        p.complete_segment();
        p.start_segment(2, (0.0, 0.0), (1.0, 0.0));
        p.update_remaining(10.0);
        p.finish(Status::Canceled, "stop");
        p.checkpoint(1790000000.5)
    }

    #[test]
    fn checkpoint_json_round_trips_and_rejects_garbage() {
        let c = checkpoint_after_segment_one();
        assert_eq!(Checkpoint::from_json(&c.to_json()), Some(c.clone()));
        assert_eq!(c.status, "canceled");
        assert_eq!((c.completed_segments, c.current_segment_index, c.total_segments), (1, 2, 3));
        assert!(Checkpoint::from_json("").is_none());
        assert!(Checkpoint::from_json("{\"version\": 2}").is_none());
        let other_version = c.to_json().replace("\"version\":1", "\"version\":2");
        assert!(Checkpoint::from_json(&other_version).is_none());
        let missing = c.to_json().replace("\"path_hash\"", "\"hash\"");
        assert!(Checkpoint::from_json(&missing).is_none());
    }

    #[test]
    fn resume_needs_the_same_path_and_no_skipped_segment() {
        let c = checkpoint_after_segment_one();
        assert!(c.resumable());
        assert_eq!(c.resume_segment_index(), 2);
        assert_eq!(c.resume_block_reason("abc", 2), None);
        assert_eq!(c.resume_block_reason("abc", 1), None);
        assert!(c.resume_block_reason("abc", 3).is_some());
        assert!(c.resume_block_reason("abc", 0).is_some());
        assert!(c.resume_block_reason("xyz", 2).unwrap().contains("changed"));
        let done = Checkpoint { status: "succeeded".into(), completed_segments: 3, ..c.clone() };
        assert!(!done.resumable() && done.resume_block_reason("abc", 1).is_some());
        assert!(!Checkpoint::default().resumable());
    }

    #[test]
    fn a_restored_running_checkpoint_reads_as_interrupted() {
        let c = Checkpoint { status: "running".into(), ..checkpoint_after_segment_one() };
        let p: P = c.restored_progress();
        assert_eq!(p.status, Status::Failed);
        assert!(p.message.contains("interrupted"));
        assert_eq!((p.total_segments(), p.completed_segments, p.zone_id), (3, 1, 3));
        assert!(close(p.overall_progress, c.overall_progress));
        let canceled: P = checkpoint_after_segment_one().restored_progress();
        assert_eq!(canceled.status, Status::Canceled);
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
