"""Coverage execution progress bookkeeping (ROS-free).

Backs ``/coverage_progress`` and ``/coverage_progress_status``
(``mower_interface/msg/CoverageProgress``). Progress is weighted by the
length of each split segment handed to Nav2 FollowPath; driving to the
coverage start counts as 0 %. ``mower_rs/crates/mower_nav/src/progress.rs``
is the Rust twin and must stay in step, including the path hash and the
checkpoint JSON, so a checkpoint written by one server can be resumed by the
other.

Checkpoint (M2) and resume (M3): the node writes a checkpoint as the
execution advances; a later goal with ``resume_segment_index > 0`` is only
admitted when :func:`resume_block_reason` finds the same path (hash over
zone, poses, split points and split parameters, i.e. the same segments) and
a segment no further than the first unfinished one.
"""

import hashlib
import json
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
        self.path_hash = ''
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

    def resume_from(self, index: int) -> None:
        """Resumed execution: segments before ``index`` count as done."""
        done = min(max(index - 1, 0), self.total_segments)
        self.completed_segments = done
        self._completed_before_m = sum(self._segment_distances[:done])
        self.message = (
            f'Resuming at coverage segment {index}/{self.total_segments}'
        )
        self._recompute(0.0)

    def checkpoint(self, updated_at: float) -> dict:
        """The durable part, for the checkpoint file."""
        return {
            'version': CHECKPOINT_VERSION,
            'mission_id': self.mission_id,
            'zone_id': self.zone_id,
            'status': self.status_text,
            'current_segment_index': self.current_segment_index,
            'total_segments': self.total_segments,
            'completed_segments': self.completed_segments,
            'completed_distance_m': float(self.completed_distance_m),
            'total_distance_m': float(self.total_distance_m),
            'overall_progress': float(self.overall_progress),
            'path_hash': self.path_hash,
            'updated_at': float(updated_at),
        }

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


CHECKPOINT_VERSION = 1


def path_hash(zone_id, split_params, poses, split_points) -> str:
    """Identity of an executed coverage plan (same text as the Rust twin).

    ``v1;zone=<id>;tol=..;max=..;turn=..;min=..;|x,y,z;...|x,y;...`` with
    every number as ``{:.4f}``, SHA-256, lowercase hex.
    """
    tol, max_len, turn, min_len = split_params
    parts = [
        f'v1;zone={int(zone_id)};tol={tol:.4f};max={max_len:.4f};'
        f'turn={turn:.4f};min={min_len:.4f};|'
    ]
    parts.extend(f'{x:.4f},{y:.4f},{z:.4f};' for x, y, z in poses)
    parts.append('|')
    parts.extend(f'{x:.4f},{y:.4f};' for x, y in split_points)
    return hashlib.sha256(''.join(parts).encode()).hexdigest()


_CHECKPOINT_INTS = (
    'zone_id', 'current_segment_index', 'total_segments', 'completed_segments',
)
_CHECKPOINT_FLOATS = (
    'completed_distance_m', 'total_distance_m', 'overall_progress',
)


def checkpoint_to_json(checkpoint: dict) -> str:
    return json.dumps(checkpoint)


def checkpoint_from_json(text: str):
    """Return the checkpoint dict, or None unless a well-formed version 1."""
    try:
        data = json.loads(text)
    except (TypeError, ValueError):
        return None
    if not isinstance(data, dict) or data.get('version') != CHECKPOINT_VERSION:
        return None
    out = {'version': CHECKPOINT_VERSION}
    for key in ('mission_id', 'status', 'path_hash'):
        if not isinstance(data.get(key), str):
            return None
        out[key] = data[key]
    for key in _CHECKPOINT_INTS:
        value = data.get(key)
        if isinstance(value, bool) or not isinstance(value, int):
            return None
        out[key] = value
    for key in _CHECKPOINT_FLOATS:
        value = data.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return None
        if not math.isfinite(value):
            return None
        out[key] = float(value)
    updated_at = data.get('updated_at', 0.0)
    out['updated_at'] = (
        float(updated_at)
        if isinstance(updated_at, (int, float)) and not isinstance(
            updated_at, bool)
        else 0.0
    )
    return out


def checkpoint_resumable(checkpoint) -> bool:
    """Unfinished work that a resume could pick up."""
    return (
        checkpoint is not None
        and checkpoint['status'] != STATUS_TEXT[STATUS_SUCCEEDED]
        and bool(checkpoint['path_hash'])
        and checkpoint['total_segments'] > 0
        and 0 <= checkpoint['completed_segments'] < checkpoint['total_segments']
    )


def resume_segment_index(checkpoint) -> int:
    """First unfinished segment (1-based)."""
    return checkpoint['completed_segments'] + 1


def resume_block_reason(checkpoint, hash_value: str, index: int):
    """Why a goal for ``hash_value`` may not start at ``index``; None = ok."""
    if not checkpoint_resumable(checkpoint):
        return 'no unfinished coverage checkpoint to resume'
    if checkpoint['path_hash'] != hash_value:
        return (
            'coverage path or split parameters changed since the '
            'checkpoint; regenerate and start over'
        )
    first_unfinished = resume_segment_index(checkpoint)
    if index < 1 or index > first_unfinished:
        return (
            f'resume segment {index} is outside 1..={first_unfinished} '
            f'(segments {checkpoint["completed_segments"]} of '
            f'{checkpoint["total_segments"]} were completed)'
        )
    return None


def restored_tracker(checkpoint) -> CoverageProgressTracker:
    """Progress to report after a restart (running = interrupted)."""
    p = CoverageProgressTracker()
    p.status = {
        'succeeded': STATUS_SUCCEEDED,
        'canceled': STATUS_CANCELED,
    }.get(checkpoint['status'], STATUS_FAILED)
    p.mission_id = checkpoint['mission_id']
    p.zone_id = checkpoint['zone_id']
    p.current_segment_index = checkpoint['current_segment_index']
    p.completed_segments = checkpoint['completed_segments']
    p.completed_distance_m = checkpoint['completed_distance_m']
    p.total_distance_m = checkpoint['total_distance_m']
    p.remaining_distance_m = max(
        checkpoint['total_distance_m'] - checkpoint['completed_distance_m'],
        0.0,
    )
    p.overall_progress = checkpoint['overall_progress']
    p.path_hash = checkpoint['path_hash']
    # total_segments without the lengths: equal parts are enough to report
    n = max(checkpoint['total_segments'], 0)
    p._segment_distances = [
        checkpoint['total_distance_m'] / n if n else 0.0
    ] * n
    if p.status == STATUS_FAILED and checkpoint['status'] != 'failed':
        p.message = (
            'Restored from checkpoint; the execution was interrupted while '
            f'{checkpoint["status"]}'
        )
    else:
        p.message = 'Restored from checkpoint'
    return p
