"""CoverageProgressTracker: the same cases as mower_nav's progress.rs tests."""

import math

import pytest

from mower_mission.navigation.coverage_progress import (
    checkpoint_from_json,
    checkpoint_resumable,
    checkpoint_to_json,
    CoverageProgressTracker,
    path_hash,
    restored_tracker,
    resume_block_reason,
    resume_segment_index,
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


def test_resume_counts_the_earlier_segments_as_done():
    p = _three_segments()
    p.resume_from(3)
    assert p.completed_segments == 2
    assert p.completed_distance_m == pytest.approx(30.0)
    assert p.overall_progress == pytest.approx(0.30)
    p.start_segment(3, 'a', 'b')
    p.update_remaining(35.0)
    assert p.overall_progress == pytest.approx(0.65)
    q = _three_segments()
    q.resume_from(1)
    assert q.completed_segments == 0
    q.resume_from(9)
    assert q.completed_segments == 3


def test_path_hash_matches_the_rust_twin_and_every_input():
    poses = [(0.0, 0.0, 0.0), (1.0, 0.5, 0.0), (2.0, -0.25, 0.0)]
    splits = [(1.0, 0.5)]
    params = (0.1, 5.0, 0.8, 0.25)
    h = path_hash(3, params, poses, splits)
    # pinned in mower_nav/src/progress.rs too
    assert h == (
        '068606bf6d49b4059c29ad4ebc76d2cc15a146329e9fade2f9e6f14745c9b79a'
    )
    assert h != path_hash(4, params, poses, splits)
    assert h != path_hash(3, (0.1, 6.0, 0.8, 0.25), poses, splits)
    assert h != path_hash(3, params, poses[:2], splits)
    assert h != path_hash(3, params, poses, [])
    assert h == path_hash(
        3, params, [(0.00001, 0.0, 0.0), poses[1], poses[2]], splits
    )


def _checkpoint_after_segment_one():
    p = _three_segments()
    p.path_hash = 'abc'
    p.start_segment(1, 'a', 'b')
    p.complete_segment()
    p.start_segment(2, 'b', 'c')
    p.update_remaining(10.0)
    p.finish(STATUS_CANCELED, 'stop')
    return p.checkpoint(1790000000.5)


def test_checkpoint_json_round_trips_and_rejects_garbage():
    c = _checkpoint_after_segment_one()
    assert checkpoint_from_json(checkpoint_to_json(c)) == c
    assert c['status'] == 'canceled'
    assert (
        c['completed_segments'], c['current_segment_index'],
        c['total_segments'],
    ) == (1, 2, 3)
    assert checkpoint_from_json('') is None
    assert checkpoint_from_json('{"version": 2}') is None
    assert checkpoint_from_json('[1]') is None
    assert checkpoint_from_json(checkpoint_to_json({**c, 'version': 2})) is None
    broken = dict(c)
    del broken['path_hash']
    assert checkpoint_from_json(checkpoint_to_json(broken)) is None
    assert checkpoint_from_json(
        checkpoint_to_json({**c, 'total_segments': 'x'})
    ) is None


def test_the_rust_checkpoint_json_is_readable():
    text = (
        '{"completed_distance_m":25.0,"completed_segments":1,'
        '"current_segment_index":2,"mission_id":"d-1",'
        '"overall_progress":0.25,"path_hash":"abc","status":"canceled",'
        '"total_distance_m":100.0,"total_segments":3,'
        '"updated_at":1790000000.5,"version":1,"zone_id":3}'
    )
    c = checkpoint_from_json(text)
    assert c is not None and c['total_distance_m'] == 100.0
    assert resume_segment_index(c) == 2


def test_resume_needs_the_same_path_and_no_skipped_segment():
    c = _checkpoint_after_segment_one()
    assert checkpoint_resumable(c)
    assert resume_segment_index(c) == 2
    assert resume_block_reason(c, 'abc', 2) is None
    assert resume_block_reason(c, 'abc', 1) is None
    assert resume_block_reason(c, 'abc', 3) is not None
    assert resume_block_reason(c, 'abc', 0) is not None
    assert 'changed' in resume_block_reason(c, 'xyz', 2)
    done = {**c, 'status': 'succeeded', 'completed_segments': 3}
    assert not checkpoint_resumable(done)
    assert resume_block_reason(done, 'abc', 1) is not None
    assert not checkpoint_resumable(None)


def test_a_restored_running_checkpoint_reads_as_interrupted():
    c = {**_checkpoint_after_segment_one(), 'status': 'running'}
    p = restored_tracker(c)
    assert p.status == STATUS_FAILED
    assert 'interrupted' in p.message
    assert (p.total_segments, p.completed_segments, p.zone_id) == (3, 1, 3)
    assert p.overall_progress == pytest.approx(c['overall_progress'])
    canceled = restored_tracker(_checkpoint_after_segment_one())
    assert canceled.status == STATUS_CANCELED
