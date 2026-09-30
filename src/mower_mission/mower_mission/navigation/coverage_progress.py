"""Coverage execution progress bookkeeping (ROS-free).

Backs ``/coverage_progress`` and ``/coverage_progress_status``
(``mower_interface/msg/CoverageProgress``). Progress is weighted by the
length of each split segment handed to Nav2 FollowPath; driving to the
coverage start counts as 0 %. ``mower_rs/crates/mower_nav/src/progress.rs``
is the Rust twin and must stay in step.
"""

import math

STATUS_IDLE = 0
STATUS_NAVIGATING_TO_START = 1
STATUS_RUNNING = 2
STATUS_CANCELING = 3
STATUS_CANCELED = 4
STATUS_SUCCEEDED = 5
STATUS_FAILED = 6

STATUS_TEXT = {
    STATUS_IDLE: 'idle',
    STATUS_NAVIGATING_TO_START: 'navigating_to_start',
    STATUS_RUNNING: 'running',
    STATUS_CANCELING: 'canceling',
    STATUS_CANCELED: 'canceled',
    STATUS_SUCCEEDED: 'succeeded',
    STATUS_FAILED: 'failed',
}

_TERMINAL = (STATUS_IDLE, STATUS_CANCELED, STATUS_SUCCEEDED, STATUS_FAILED)


class CoverageProgressTracker:
    """Plain state; the node copies it into a CoverageProgress message."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.status = STATUS_IDLE
        self.mission_id = ''
        self.zone_id = -1
        self.current_segment_index = 0
        self.completed_segments = 0
        self.current_segment_progress = 0.0
        self.overall_progress = 0.0
        self.completed_distance_m = 0.0
        self.total_distance_m = 0.0
        self.remaining_distance_m = 0.0
        self.current_segment_start = None
        self.current_segment_goal = None
        self.message = ''
        self._segment_distances = []
        self._completed_before_m = 0.0

    @property
    def status_text(self) -> str:
        return STATUS_TEXT[self.status]

    @property
    def total_segments(self) -> int:
        return len(self._segment_distances)

    @property
    def current_segment_distance_m(self) -> float:
        index = self.current_segment_index
        if 1 <= index <= len(self._segment_distances):
            return self._segment_distances[index - 1]
        return 0.0

    def begin(self, mission_id: str, zone_id: int) -> None:
        """Start a new execution, driving towards the coverage start."""
        self.reset()
        self.status = STATUS_NAVIGATING_TO_START
        self.mission_id = mission_id
        try:
            self.zone_id = int(zone_id)
        except (TypeError, ValueError):
            self.zone_id = -1
        self.message = 'Coverage navigation accepted'

    def set_segments(self, distances) -> None:
        self._segment_distances = [
            float(d) if math.isfinite(d) and d > 0.0 else 0.0
            for d in distances
        ]
        self.total_distance_m = sum(self._segment_distances)
        self._recompute(0.0)

    def start_segment(self, index: int, start, goal) -> None:
        if self.status == STATUS_CANCELING:
            return
        self.status = STATUS_RUNNING
        self.current_segment_index = index
        self.current_segment_progress = 0.0
        self.current_segment_start = start
        self.current_segment_goal = goal
        self.message = (
            f'Following coverage segment {index}/{self.total_segments}'
        )
        self._recompute(0.0)

    def update_remaining(self, remaining_m) -> bool:
        """Apply FollowPath feedback (path length left in the segment).

        Returns whether anything changed. Progress within a segment never
        goes backwards (the controller's estimate jitters).
        """
        try:
            remaining_m = float(remaining_m)
        except (TypeError, ValueError):
            return False
        if (
            self.status not in (STATUS_RUNNING, STATUS_CANCELING)
            or self.current_segment_index < 1
            or not math.isfinite(remaining_m)
        ):
            return False
        distance = self.current_segment_distance_m
        if distance <= 0.0:
            return False
        fraction = min(
            max((distance - max(remaining_m, 0.0)) / distance, 0.0), 1.0
        )
        if fraction <= self.current_segment_progress:
            return False
        self.current_segment_progress = fraction
        self._recompute(fraction * distance)
        return True

    def complete_segment(self) -> None:
        self.completed_segments += 1
        self._completed_before_m += self.current_segment_distance_m
        self.current_segment_progress = 1.0
        self._recompute(0.0)

    def canceling(self, message: str) -> None:
        if self.status in _TERMINAL:
            return
        self.status = STATUS_CANCELING
        self.message = message

    def finish(self, status: int, message: str) -> None:
        """Final state; cancel and failure keep the last distances."""
        self.status = status
        self.message = message
        if status == STATUS_SUCCEEDED:
            self.completed_segments = self.total_segments
            self._completed_before_m = self.total_distance_m
            self.current_segment_progress = 1.0
            self._recompute(0.0)
            self.overall_progress = 1.0

    def _recompute(self, done_in_segment_m: float) -> None:
        self.completed_distance_m = min(
            self._completed_before_m + done_in_segment_m,
            self.total_distance_m,
        )
        self.remaining_distance_m = max(
            self.total_distance_m - self.completed_distance_m, 0.0
        )
        if self.total_distance_m > 0.0:
            self.overall_progress = min(
                max(self.completed_distance_m / self.total_distance_m, 0.0),
                1.0,
            )
        else:
            self.overall_progress = 0.0
