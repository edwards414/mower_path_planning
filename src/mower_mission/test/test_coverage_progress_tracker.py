"""CoverageProgressTracker: the same cases as mower_nav's progress.rs tests."""

import math

import pytest

from mower_mission.navigation.coverage_progress import (
    CoverageProgressTracker,
    STATUS_CANCELED,
    STATUS_CANCELING,
    STATUS_FAILED,
    STATUS_NAVIGATING_TO_START,
    STATUS_SUCCEEDED,
)


def _three_segments():
    p = CoverageProgressTracker()
    p.begin('d-1', 3)
    p.set_segments([10.0, 20.0, 70.0])
    return p


def test_begin_resets_and_totals_the_segments():
    p = _three_segments()
    p.start_segment(2, 'a', 'b')
    p.begin('d-2', 4)
    p.set_segments([10.0, 20.0, 70.0])
    assert p.status == STATUS_NAVIGATING_TO_START
    assert p.status_text == 'navigating_to_start'
    assert (p.mission_id, p.zone_id) == ('d-2', 4)
    assert (p.current_segment_index, p.total_segments) == (0, 3)
    assert p.completed_segments == 0
    assert p.total_distance_m == pytest.approx(100.0)
    assert p.remaining_distance_m == pytest.approx(100.0)
    assert p.overall_progress == 0.0


def test_feedback_moves_the_segment_and_distance_weighted_overall():
    p = _three_segments()
    p.start_segment(1, 'a', 'b')
    assert p.update_remaining(4.0)
    assert p.current_segment_progress == pytest.approx(0.6)
    assert p.completed_distance_m == pytest.approx(6.0)
    assert p.overall_progress == pytest.approx(0.06)
    p.complete_segment()
    assert p.completed_segments == 1
    assert p.overall_progress == pytest.approx(0.10)
    p.start_segment(2, 'b', 'c')
    assert p.current_segment_progress == 0.0
    p.update_remaining(10.0)
    # by distance 20 / 100, not by segment count (1.5 / 3)
    assert p.overall_progress == pytest.approx(0.20)
    assert p.remaining_distance_m == pytest.approx(80.0)


def test_segment_progress_never_goes_backwards_or_outside_the_segment():
    p = _three_segments()
    p.start_segment(1, 'a', 'b')
    p.update_remaining(4.0)
    assert not p.update_remaining(5.0)
    assert p.current_segment_progress == pytest.approx(0.6)
    p.update_remaining(-3.0)
    assert p.current_segment_progress == pytest.approx(1.0)
    assert p.completed_distance_m == pytest.approx(10.0)
    assert not p.update_remaining(math.nan)
    assert not p.update_remaining(None)
    assert not p.update_remaining('not-a-number')


def test_feedback_before_the_first_segment_is_ignored():
    p = _three_segments()
    assert not p.update_remaining(2.0)
    assert p.overall_progress == 0.0


def test_success_fills_up_and_cancel_or_failure_keep_the_last_progress():
    def halfway():
        p = _three_segments()
        p.start_segment(1, 'a', 'b')
        p.complete_segment()
        p.start_segment(2, 'b', 'c')
        p.update_remaining(5.0)
        return p

    canceled = halfway()
    canceled.canceling('cancel requested')
    assert canceled.status == STATUS_CANCELING
    canceled.finish(STATUS_CANCELED, 'canceled')
    assert (canceled.completed_segments, canceled.current_segment_index) == (
        1, 2,
    )
    assert canceled.completed_distance_m == pytest.approx(25.0)
    failed = halfway()
    failed.finish(STATUS_FAILED, 'Nav2 failed')
    assert failed.overall_progress == pytest.approx(0.25)
    done = halfway()
    done.finish(STATUS_SUCCEEDED, 'done')
    assert done.completed_segments == 3
    assert done.overall_progress == 1.0
    assert done.remaining_distance_m == 0.0


def test_a_canceling_mission_does_not_flip_back_to_running():
    p = _three_segments()
    p.canceling('stop')
    p.start_segment(1, 'a', 'b')
    assert p.status == STATUS_CANCELING
    done = _three_segments()
    done.finish(STATUS_FAILED, 'x')
    done.canceling('late')
    assert done.status == STATUS_FAILED


def test_zero_length_plans_do_not_divide_by_zero():
    p = CoverageProgressTracker()
    p.begin('d', -1)
    p.set_segments([0.0, math.nan])
    p.start_segment(1, 'a', 'a')
    assert not p.update_remaining(0.0)
    assert p.overall_progress == 0.0
    p.finish(STATUS_SUCCEEDED, 'done')
    assert p.overall_progress == 1.0
