"""Unit tests for nav_action_server cancel and status services."""

import json
from time import monotonic
from types import SimpleNamespace
from uuid import uuid4

from mower_mission.navigation import nav_action_server as nav_module

import pytest

import rclpy
from rclpy.action import GoalResponse
from rclpy.parameter import Parameter

from geometry_msgs.msg import PoseStamped, TwistStamped
from mower_interface.action import Waypoint
from mower_interface.srv import (
    CancelNavigationDispatch,
    ConfirmNavigationDispatch,
    MissionOperationLock,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_srvs.srv import Trigger


class ImmediateFuture:
    """Completed future used by the fake Nav2 action goal."""

    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class RaisingFuture:
    def done(self):
        return True

    def result(self):
        raise RuntimeError('result transport lost')


class DeferredFuture:
    """Manually completed future used for late goal-response races."""

    def __init__(self):
        self._done = False
        self._result = None
        self._callbacks = []

    def done(self):
        return self._done

    def result(self):
        if not self._done:
            raise RuntimeError('future is not complete')
        return self._result

    def add_done_callback(self, callback):
        if self._done:
            callback(self)
        else:
            self._callbacks.append(callback)

    def complete(self, result):
        self._done = True
        self._result = result
        callbacks = tuple(self._callbacks)
        self._callbacks.clear()
        for callback in callbacks:
            callback(self)


_DEFAULT_RESULT_FUTURE = object()


class FakeNavGoalHandle:
    """Goal handle whose cancel acknowledgment and result are immediate."""

    def __init__(self, navigator):
        self._navigator = navigator

    def cancel_goal_async(self):
        self._navigator.cancel_calls += 1
        self._navigator.complete = True
        self._navigator.result = nav_module.TaskResult.CANCELED
        return ImmediateFuture(SimpleNamespace(goals_canceling=[self]))


class FakeNavResultFuture:
    """Result future whose completion follows the fake navigator task."""

    def __init__(self, navigator):
        self._navigator = navigator

    def done(self):
        return self._navigator.complete

    def result(self):
        status_by_result = {
            nav_module.TaskResult.SUCCEEDED:
                nav_module.GoalStatus.STATUS_SUCCEEDED,
            nav_module.TaskResult.CANCELED:
                nav_module.GoalStatus.STATUS_CANCELED,
            nav_module.TaskResult.FAILED:
                nav_module.GoalStatus.STATUS_ABORTED,
        }
        return SimpleNamespace(
            status=status_by_result.get(
                self._navigator.result,
                nav_module.GoalStatus.STATUS_UNKNOWN,
            )
        )


class FakeNavigator:
    """Small BasicNavigator stand-in for service callback tests."""

    def __init__(self):
        """Initialize fake task state."""
        self.cancel_calls = 0
        self.complete = False
        self.result = nav_module.TaskResult.SUCCEEDED
        self.feedback = SimpleNamespace(distance_remaining=1.25)
        self.goal_handle = FakeNavGoalHandle(self)
        self.result_future = FakeNavResultFuture(self)

    def cancelTask(self):
        """Record cancel requests without touching Nav2."""
        self.cancel_calls += 1

    def set_parameters(self, parameters):
        """Accept propagated node parameters like BasicNavigator."""
        return []

    def isTaskComplete(self):
        """Return the configured task completion state."""
        return self.complete

    def getFeedback(self):
        """Return the configured feedback object."""
        return self.feedback

    def getResult(self):
        """Return the configured task result."""
        return self.result

    def destroy_node(self):
        """Match BasicNavigator teardown API."""


def _valid_navigation_goal(*, with_dispatch=True):
    goal = Waypoint.Goal()
    goal.path.header.frame_id = 'map'
    for x in (0.0, 1.0):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.orientation.w = 1.0
        goal.path.poses.append(pose)
    if with_dispatch:
        goal.dispatch_id = uuid4().hex
    return goal


def _arm_fake_nav2_dispatch(
    nav_server,
    *,
    generation=1,
    goal_handle=None,
    result_future=_DEFAULT_RESULT_FUTURE,
):
    """Install one internally correlated fake Nav2 dispatch."""
    handle = goal_handle or nav_server.navigator.goal_handle
    future = (
        nav_server.navigator.result_future
        if result_future is _DEFAULT_RESULT_FUTURE
        else result_future
    )
    nav_server._nav2_dispatch_generation = max(
        nav_server._nav2_dispatch_generation,
        generation,
    )
    nav_server._nav2_active_generation = generation
    nav_server._nav2_dispatch_in_flight_or_active = True
    nav_server._nav2_current_goal_handle = handle
    nav_server._nav2_current_result_future = future
    nav_server.navigator.goal_handle = handle
    nav_server.navigator.result_future = future
    return handle, future


def _feed_valid_imu(nav_server):
    imu = Imu()
    imu.header.frame_id = 'imu_link'
    imu.header.stamp = nav_server.get_clock().now().to_msg()
    imu.orientation.w = 1.0
    for covariance in (
        imu.orientation_covariance,
        imu.angular_velocity_covariance,
        imu.linear_acceleration_covariance,
    ):
        covariance[0] = 0.01
        covariance[4] = 0.01
        covariance[8] = 0.01
    nav_server._imu_health_callback(imu)


@pytest.fixture
def nav_server(monkeypatch):
    """Create NavActionServer with BasicNavigator replaced by a fake."""
    monkeypatch.setattr(nav_module, 'BasicNavigator', FakeNavigator)
    if not rclpy.ok():
        rclpy.init()
    node = nav_module.NavActionServer(parameter_overrides=[
        Parameter('require_navigation_health', value=False),
    ])
    try:
        yield node
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


@pytest.fixture
def health_nav_server(monkeypatch):
    """Create a server whose immutable production health gate starts enabled."""
    monkeypatch.setattr(nav_module, 'BasicNavigator', FakeNavigator)
    if not rclpy.ok():
        rclpy.init()
    node = nav_module.NavActionServer()
    try:
        yield node
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_cancel_service_requests_bounded_execution_cancel(nav_server):
    """The service flags cancellation; execution confirms Nav2 termination."""
    nav_server._active_task_name = 'Follow coverage segment 1'
    nav_server._nav_state = 'running'
    _arm_fake_nav2_dispatch(nav_server)

    response = nav_server.cancel_nav2_srv(
        Trigger.Request(),
        Trigger.Response(),
    )

    assert response.success
    assert nav_server.navigator.cancel_calls == 0
    assert nav_server._external_cancel_requested is True
    assert 'Follow coverage segment 1' in response.message

    confirmed, result = nav_server._cancel_nav_task_and_wait()
    assert confirmed
    assert result == nav_module.TaskResult.CANCELED
    assert nav_server.navigator.cancel_calls == 1


def test_cancel_service_is_noop_when_idle(nav_server):
    """Cancel while idle should not leave status stuck in canceling."""
    response = nav_server.cancel_nav2_srv(
        Trigger.Request(),
        Trigger.Response(),
    )

    assert response.success
    assert response.message == 'No active Nav2 task to cancel'
    assert nav_server.navigator.cancel_calls == 0
    assert nav_server._nav_state == 'idle'


def test_status_service_reports_running_feedback(nav_server):
    """Running status should include feedback from server-owned navigator."""
    nav_server._active_task_name = 'Follow coverage segment 2'
    nav_server._last_feedback_message = 'distance_remaining=1.250'
    nav_server._set_nav_state(
        'running',
        'Navigation running: Follow coverage segment 2',
    )

    response = nav_server.check_nav_status_srv(
        Trigger.Request(),
        Trigger.Response(),
    )

    assert response.success
    payload = json.loads(response.message)
    assert payload['state'] == 'running'
    assert payload['task'] == 'Follow coverage segment 2'
    assert 'distance_remaining=1.250' in payload['message']


def test_status_service_reports_terminal_state_as_json(nav_server):
    """Terminal outcome is represented by JSON, not Trigger.success."""
    nav_server._last_task_name = 'Follow coverage segment 3'
    nav_server._set_nav_state(
        'canceled',
        'Follow coverage segment 3 canceled',
    )

    response = nav_server.check_nav_status_srv(
        Trigger.Request(),
        Trigger.Response(),
    )

    assert response.success
    assert json.loads(response.message) == {
        'state': 'canceled',
        'task': 'Follow coverage segment 3',
        'message': 'Follow coverage segment 3 canceled',
        'ready': True,
        'block_reason': None,
    }


def test_goal_callback_rejects_empty_and_concurrent_goals(nav_server):
    """Both action names share one reservation and reject empty paths."""
    empty_goal = Waypoint.Goal()
    assert nav_server._goal_callback(empty_goal) == GoalResponse.REJECT

    missing_dispatch = _valid_navigation_goal(with_dispatch=False)
    assert nav_server._goal_callback(missing_dispatch) == GoalResponse.REJECT

    valid_goal = _valid_navigation_goal()
    assert nav_server._goal_callback(valid_goal) == GoalResponse.ACCEPT
    assert nav_server._nav_state == 'pending_confirmation'
    assert nav_server._goal_callback(valid_goal) == GoalResponse.REJECT

    response = nav_server.cancel_nav2_srv(
        Trigger.Request(),
        Trigger.Response(),
    )
    assert response.success
    assert nav_server._nav_state == 'canceling'
    assert nav_server.navigator.cancel_calls == 0


def test_dispatch_confirmation_is_correlated_and_cancel_locks_immediately(
    nav_server,
):
    """A delayed confirm cannot unlock another goal; cancel revokes motion."""
    goal = _valid_navigation_goal()
    assert nav_server._goal_callback(goal) == GoalResponse.ACCEPT

    wrong = ConfirmNavigationDispatch.Request()
    wrong.dispatch_id = uuid4().hex
    response = nav_server.confirm_navigation_dispatch_srv(
        wrong,
        ConfirmNavigationDispatch.Response(),
    )
    assert not response.success
    assert not nav_server._dispatch_confirmed

    current = ConfirmNavigationDispatch.Request()
    current.dispatch_id = goal.dispatch_id
    response = nav_server.confirm_navigation_dispatch_srv(
        current,
        ConfirmNavigationDispatch.Response(),
    )
    assert response.success
    assert nav_server._dispatch_confirmed

    handle = SimpleNamespace(request=goal)
    assert nav_server._cancel_callback(handle) == nav_module.CancelResponse.ACCEPT
    assert nav_server._nav_state == 'canceling'
    assert nav_server._external_cancel_requested


def test_correlated_cancel_cannot_target_another_goal(nav_server):
    goal = _valid_navigation_goal()
    assert nav_server._goal_callback(goal) == GoalResponse.ACCEPT

    stale = CancelNavigationDispatch.Request()
    stale.dispatch_id = uuid4().hex
    response = nav_server.cancel_navigation_dispatch_srv(
        stale,
        CancelNavigationDispatch.Response(),
    )
    assert response.success
    assert not response.terminal_confirmed
    assert nav_server._nav_state == 'pending_confirmation'

    current = CancelNavigationDispatch.Request()
    current.dispatch_id = goal.dispatch_id
    response = nav_server.cancel_navigation_dispatch_srv(
        current,
        CancelNavigationDispatch.Response(),
    )
    assert response.success
    assert not response.terminal_confirmed
    assert nav_server._nav_state == 'canceling'


def test_uncertain_dispatch_becomes_terminal_after_monitor_confirmation(
    nav_server,
):
    dispatch_id = uuid4().hex
    nav_server._nav2_task_uncertain = True
    nav_server._uncertain_dispatch_id = dispatch_id
    nav_server._clear_uncertain_nav2_task(
        nav_module.TaskResult.CANCELED,
        source='test',
    )

    request = CancelNavigationDispatch.Request()
    request.dispatch_id = dispatch_id
    response = nav_server.cancel_navigation_dispatch_srv(
        request,
        CancelNavigationDispatch.Response(),
    )
    assert response.success
    assert response.terminal_confirmed


def test_correlated_cancel_retries_after_outer_action_releases_goal(nav_server):
    dispatch_id = uuid4().hex
    result_future = ImmediateFuture(SimpleNamespace(
        status=nav_module.GoalStatus.STATUS_CANCELED,
    ))
    nav_server.navigator.result_future = result_future
    nav_server.navigator.complete = False
    nav_server._nav2_task_uncertain = True
    nav_server._uncertain_dispatch_id = dispatch_id
    nav_server._goal_reserved = False
    nav_server._pending_dispatch_id = None
    _arm_fake_nav2_dispatch(nav_server, result_future=result_future)

    request = CancelNavigationDispatch.Request()
    request.dispatch_id = dispatch_id
    response = nav_server.cancel_navigation_dispatch_srv(
        request,
        CancelNavigationDispatch.Response(),
    )

    assert response.success
    assert response.terminal_confirmed
    # The correlated result was already terminal, so no extra cancel is sent.
    assert nav_server.navigator.cancel_calls == 0
    assert dispatch_id in nav_server._terminal_dispatch_ids
    assert nav_server._uncertain_dispatch_id is None
    assert not nav_server._nav2_task_uncertain


def test_correlated_cancel_never_targets_another_uncertain_dispatch(nav_server):
    dispatch_id = uuid4().hex
    nav_server._nav2_task_uncertain = True
    nav_server._uncertain_dispatch_id = dispatch_id
    nav_server._goal_reserved = False
    nav_server._pending_dispatch_id = None

    request = CancelNavigationDispatch.Request()
    request.dispatch_id = uuid4().hex
    response = nav_server.cancel_navigation_dispatch_srv(
        request,
        CancelNavigationDispatch.Response(),
    )

    assert response.success
    assert not response.terminal_confirmed
    assert nav_server.navigator.cancel_calls == 0
    assert nav_server._uncertain_dispatch_id == dispatch_id
    assert nav_server._nav2_task_uncertain


def test_runtime_cannot_disable_navigation_health_gate(health_nav_server):
    result = health_nav_server.set_parameters([
        Parameter('require_navigation_health', value=False),
    ])[0]

    assert not result.successful
    assert health_nav_server.get_parameter(
        'require_navigation_health'
    ).value is True


@pytest.mark.parametrize(
    'future',
    [
        RaisingFuture(),
        ImmediateFuture(SimpleNamespace(
            status=nav_module.GoalStatus.STATUS_UNKNOWN,
        )),
    ],
)
def test_unreadable_or_unknown_nav2_result_is_not_terminal(future):
    terminal, result, unconfirmed = (
        nav_module.NavActionServer._task_result_from_future(future)
    )

    assert not terminal
    assert result is None
    assert unconfirmed


def test_stale_terminal_snapshot_cannot_clear_newer_dispatch(nav_server):
    old_future = ImmediateFuture(SimpleNamespace(
        status=nav_module.GoalStatus.STATUS_SUCCEEDED,
    ))
    _arm_fake_nav2_dispatch(
        nav_server,
        generation=1,
        result_future=old_future,
    )
    newer_handle = object()
    newer_future = object()

    def finish_old_after_new_dispatch(_future):
        nav_server._nav2_active_generation = 2
        nav_server._nav2_dispatch_in_flight_or_active = True
        nav_server._nav2_current_goal_handle = newer_handle
        nav_server._nav2_current_result_future = newer_future
        return True, nav_module.TaskResult.SUCCEEDED, False

    nav_server._task_result_from_future = finish_old_after_new_dispatch

    terminal, result = nav_server._navigator_terminal_snapshot()

    assert not terminal
    assert result is None
    assert nav_server._nav2_active_generation == 2
    assert nav_server._nav2_current_goal_handle is newer_handle
    assert nav_server._nav2_current_result_future is newer_future


def test_terminal_cleanup_requires_matching_generation(nav_server):
    handle, future = _arm_fake_nav2_dispatch(
        nav_server,
        generation=7,
    )

    assert not nav_server._mark_nav2_dispatch_terminal(
        expected_generation=None,
    )
    assert not nav_server._mark_nav2_dispatch_terminal(
        expected_generation=6,
    )
    assert nav_server._nav2_active_generation == 7
    assert nav_server._nav2_current_goal_handle is handle
    assert nav_server._nav2_current_result_future is future

    assert nav_server._mark_nav2_dispatch_terminal(
        expected_generation=7,
    )
    assert nav_server._nav2_active_generation is None


def test_late_stale_goal_does_not_pollute_newer_navigator_mirror(nav_server):
    nav_server.set_parameters([
        Parameter('nav2_goal_response_timeout_s', value=0.1),
    ])
    response_future = DeferredFuture()
    action_client = SimpleNamespace(
        wait_for_server=lambda timeout_sec: True,
        send_goal_async=lambda goal, feedback_callback: response_future,
    )
    nav_server._nav2_dispatch_generation = 1
    nav_server._nav2_active_generation = 1
    nav_server._nav2_dispatch_in_flight_or_active = True

    with pytest.raises(RuntimeError, match='acceptance is uncertain'):
        nav_server._send_nav2_goal_bounded(action_client, object())

    newer_handle = object()
    newer_future = object()
    nav_server._nav2_dispatch_generation = 2
    nav_server._nav2_active_generation = 2
    nav_server._nav2_current_goal_handle = newer_handle
    nav_server._nav2_current_result_future = newer_future
    nav_server.navigator.goal_handle = newer_handle
    nav_server.navigator.result_future = newer_future

    class LateHandle:
        accepted = True

        def __init__(self):
            self.cancel_calls = 0
            self.result_calls = 0

        def cancel_goal_async(self):
            self.cancel_calls += 1
            return ImmediateFuture(SimpleNamespace(goals_canceling=[self]))

        def get_result_async(self):
            self.result_calls += 1
            return ImmediateFuture(SimpleNamespace(
                status=nav_module.GoalStatus.STATUS_CANCELED,
            ))

    late_handle = LateHandle()
    response_future.complete(late_handle)

    assert late_handle.cancel_calls == 1
    assert late_handle.result_calls == 0
    assert nav_server.navigator.goal_handle is newer_handle
    assert nav_server.navigator.result_future is newer_future
    assert nav_server._nav2_current_goal_handle is newer_handle
    assert nav_server._nav2_current_result_future is newer_future


def test_cancel_rebuilds_missing_result_future_for_same_generation(nav_server):
    recovered_future = ImmediateFuture(SimpleNamespace(
        status=nav_module.GoalStatus.STATUS_CANCELED,
    ))

    class RecoveringHandle:
        def __init__(self):
            self.cancel_calls = 0
            self.result_calls = 0

        def cancel_goal_async(self):
            self.cancel_calls += 1
            return ImmediateFuture(SimpleNamespace(goals_canceling=[self]))

        def get_result_async(self):
            self.result_calls += 1
            if self.result_calls == 1:
                raise RuntimeError('transient get-result transport failure')
            return recovered_future

    handle = RecoveringHandle()
    _arm_fake_nav2_dispatch(
        nav_server,
        generation=7,
        goal_handle=handle,
        result_future=None,
    )
    nav_server.set_parameters([
        Parameter('nav2_cancel_timeout_s', value=0.2),
        Parameter('nav2_cancel_poll_s', value=0.01),
    ])

    confirmed, result = nav_server._cancel_nav_task_and_wait()

    assert confirmed
    assert result == nav_module.TaskResult.CANCELED
    assert handle.cancel_calls == 1
    assert handle.result_calls == 2
    assert nav_server.navigator.result_future is recovered_future
    assert nav_server._nav2_active_generation is None


def test_nav2_readiness_wait_is_bounded(nav_server):
    """An unavailable lifecycle service must fail within the configured bound."""
    nav_server.set_parameters([
        Parameter('nav2_ready_timeout_s', value=0.1),
        Parameter('nav2_ready_poll_s', value=0.05),
    ])
    nav_server._nav2_state_client = SimpleNamespace(
        wait_for_service=lambda timeout_sec: False,
    )
    goal_handle = SimpleNamespace(is_cancel_requested=False)

    started_at = monotonic()
    ready, message = nav_server._wait_for_nav2_ready(goal_handle)

    assert not ready
    assert monotonic() - started_at < 0.5
    assert 'timed out' in message


@pytest.mark.parametrize(
    'value',
    [float('nan'), float('inf'), float('-inf'), -0.1, 'not-a-number'],
)
def test_invalid_nav2_feedback_can_never_trigger_early_success(
    nav_server,
    value,
):
    feedback = SimpleNamespace(distance_remaining=value)

    assert nav_server._feedback_distance(feedback) is None


def test_cancel_before_running_never_releases_coordinator_lock(nav_server):
    published_locks = []
    nav_server._navigation_coordinator_lock_pub = SimpleNamespace(
        publish=lambda message: published_locks.append(message.data),
    )
    nav_server._goal_reserved = True
    nav_server._nav_state = 'canceling'
    nav_server._external_cancel_requested = True
    _arm_fake_nav2_dispatch(nav_server)
    canceled = []
    goal_handle = SimpleNamespace(
        is_cancel_requested=False,
        canceled=lambda: canceled.append(True),
        abort=lambda: None,
    )

    assert not nav_server._wait_for_nav_task(goal_handle, 'race test')
    assert canceled == [True]
    assert published_locks
    assert all(published_locks)


def test_lost_nav2_dispatch_response_latches_uncertain_and_blocks_goals(
    nav_server,
):
    dispatch_id = uuid4().hex
    nav_server._goal_reserved = True
    nav_server._pending_dispatch_id = dispatch_id
    nav_server.navigator.goal_handle = None
    aborted = []
    goal_handle = SimpleNamespace(abort=lambda: aborted.append(True))

    def accepted_then_transport_error():
        raise RuntimeError('goal response lost after possible acceptance')

    result = nav_server._run_reserved_goal(
        goal_handle,
        lambda _: nav_server._start_nav2_dispatch(
            accepted_then_transport_error
        ),
    )

    assert not result.success
    assert aborted == [True]
    assert nav_server._nav2_task_uncertain
    assert nav_server._uncertain_dispatch_id == dispatch_id
    assert nav_server._nav_state == 'uncertain'
    assert nav_server._goal_callback(_valid_navigation_goal()) == GoalResponse.REJECT


def test_stale_completed_handle_cannot_clear_lost_new_dispatch(nav_server):
    stale_handle = nav_server.navigator.goal_handle
    stale_future = ImmediateFuture(SimpleNamespace())
    nav_server.navigator.result_future = stale_future
    nav_server.navigator.complete = True
    dispatch_id = uuid4().hex
    nav_server._goal_reserved = True
    nav_server._pending_dispatch_id = dispatch_id
    goal_handle = SimpleNamespace(abort=lambda: None)

    def accepted_then_response_lost():
        # BasicNavigator still exposes the previous task's completed handle and
        # future when the new goal response is lost before assignment.
        assert nav_server.navigator.goal_handle is stale_handle
        assert nav_server.navigator.result_future is stale_future
        raise RuntimeError('new goal response lost')

    nav_server._run_reserved_goal(
        goal_handle,
        lambda _: nav_server._start_nav2_dispatch(
            accepted_then_response_lost
        ),
    )

    assert nav_server._nav2_task_uncertain
    nav_server._monitor_uncertain_nav2_task()
    assert nav_server._nav2_task_uncertain
    assert nav_server._nav_state == 'uncertain'


def test_dense_coverage_splits_are_rejected_instead_of_fake_success():
    path = _valid_navigation_goal().path
    path.poses.clear()
    split_points = []
    for index in range(16):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = index * 0.04
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
        if 0 < index < 15:
            split_points.append(pose.pose)

    segments = nav_module._split_path_by_coverage_points(
        path,
        split_points,
        0.001,
    )

    assert nav_module._path_distance(path) > 0.5
    assert all(nav_module._path_distance(segment) < 0.05 for segment in segments)
    assert nav_module._coverage_segments_block_reason(segments) is not None


@pytest.mark.parametrize('distance', [0.051, 0.101, 0.149])
def test_short_segment_above_goal_tolerance_is_still_rejected(distance):
    path = _valid_navigation_goal().path
    path.poses[1].pose.position.x = distance

    reason = nav_module._coverage_segments_block_reason([path])

    assert reason is not None
    assert f'{distance:.3f} m' in reason


def test_uncertain_nav2_task_blocks_goals_until_cancel_confirms_terminal(
    nav_server,
):
    """An aborted outer action must not release an unconfirmed Nav2 motion."""
    nav_server._goal_reserved = False
    nav_server._last_task_name = 'Follow coverage segment 1'
    _arm_fake_nav2_dispatch(nav_server)
    nav_server._mark_nav2_task_uncertain('cancel confirmation timed out')
    assert nav_server._nav_state == 'uncertain'

    goal = _valid_navigation_goal()
    assert nav_server._goal_callback(goal) == GoalResponse.REJECT

    response = nav_server.cancel_nav2_srv(
        Trigger.Request(),
        Trigger.Response(),
    )
    assert response.success
    assert nav_server._nav2_task_uncertain is False
    assert nav_server._nav_state == 'idle'
    assert nav_server._goal_callback(goal) == GoalResponse.ACCEPT


def test_mission_mutation_lock_is_atomic_with_goal_admission(nav_server):
    """A mutation lease must reject navigation until its owner releases it."""
    acquire = MissionOperationLock.Request()
    acquire.owner = 'test-owner'
    acquire.operation = 'edit geometry'
    acquire.acquire = True
    assert nav_server.mission_operation_lock_srv(
        acquire,
        MissionOperationLock.Response(),
    ).success

    goal = _valid_navigation_goal()
    assert nav_server._goal_callback(goal) == GoalResponse.REJECT

    wrong_release = MissionOperationLock.Request()
    wrong_release.owner = 'other-owner'
    wrong_release.acquire = False
    assert not nav_server.mission_operation_lock_srv(
        wrong_release,
        MissionOperationLock.Response(),
    ).success

    release = MissionOperationLock.Request()
    release.owner = 'test-owner'
    release.acquire = False
    assert nav_server.mission_operation_lock_srv(
        release,
        MissionOperationLock.Response(),
    ).success
    assert nav_server._goal_callback(goal) == GoalResponse.ACCEPT


def test_manual_velocity_blocks_goal_and_active_goal_requests_cancel(nav_server):
    """Every muxed manual source is mutually exclusive with autonomy."""
    moving = TwistStamped()
    moving.twist.linear.x = 0.2
    stopped = TwistStamped()
    goal = _valid_navigation_goal()

    nav_server._manual_cmd_callback(moving, '/joy_cmd')
    assert nav_server._goal_callback(goal) == GoalResponse.REJECT

    nav_server._manual_cmd_callback(stopped, '/joy_cmd')
    assert nav_server._goal_callback(goal) == GoalResponse.REJECT
    nav_server._manual_command_deadlines['/joy_cmd'] = monotonic() - 1.0
    assert nav_server._goal_callback(goal) == GoalResponse.ACCEPT
    nav_server._manual_cmd_callback(moving, '/physical_joy_cmd')
    assert nav_server._external_cancel_requested is True
    assert nav_server._nav_state == 'canceling'


def test_fresh_manual_zero_cancels_running_goal_to_prevent_late_resume(
    nav_server,
):
    goal = _valid_navigation_goal()
    assert nav_server._goal_callback(goal) == GoalResponse.ACCEPT
    nav_server._nav_state = 'running'
    stopped = TwistStamped()

    nav_server._manual_cmd_callback(stopped, '/keyboard_cmd_vel')

    assert nav_server._external_cancel_requested
    assert nav_server._nav_state == 'canceling'


def test_real_navigation_requires_and_monitors_fresh_pose_and_gps(
    health_nav_server,
):
    """Production health gate rejects startup and cancels stale localization."""
    nav_server = health_nav_server
    goal = _valid_navigation_goal()
    assert nav_server._goal_callback(goal) == GoalResponse.REJECT

    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav_server.get_clock().now().to_msg()
    pose.pose.orientation.w = 1.0
    nav_server._robot_pose_health_callback(pose)
    fix = NavSatFix()
    fix.header.stamp = nav_server.get_clock().now().to_msg()
    fix.status.status = NavSatStatus.STATUS_FIX
    fix.latitude = 23.5
    fix.longitude = 120.5
    fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    fix.position_covariance[0] = 0.0001
    fix.position_covariance[4] = 0.0001
    nav_server._gps_health_callback(fix)
    gps_odom = Odometry()
    gps_odom.header.frame_id = 'map'
    gps_odom.header.stamp = nav_server.get_clock().now().to_msg()
    gps_odom.pose.covariance[0] = 0.0001
    gps_odom.pose.covariance[7] = 0.0001
    nav_server._gps_odometry_health_callback(gps_odom)
    _feed_valid_imu(nav_server)
    assert nav_server._goal_callback(goal) == GoalResponse.ACCEPT

    nav_server._valid_fix_received_at_by_topic['/fix'] = monotonic() - 10.0
    nav_server._monitor_navigation_health()
    assert nav_server._external_cancel_requested is True
    assert nav_server._nav_state == 'canceling'


@pytest.mark.parametrize(
    ('variance_x', 'variance_y', 'covariance_xy'),
    [
        (0.0, 0.0, 0.0),
        (0.0001, 0.0, 0.0),
        (0.0001, 0.0001, 0.0001),
    ],
)
def test_navigation_health_rejects_degenerate_gps_covariance(
    health_nav_server,
    variance_x,
    variance_y,
    covariance_xy,
):
    nav_server = health_nav_server
    fix = NavSatFix()
    fix.header.stamp = nav_server.get_clock().now().to_msg()
    fix.status.status = NavSatStatus.STATUS_FIX
    fix.latitude = 23.5
    fix.longitude = 120.5
    fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
    fix.position_covariance[0] = variance_x
    fix.position_covariance[1] = covariance_xy
    fix.position_covariance[3] = covariance_xy
    fix.position_covariance[4] = variance_y

    nav_server._gps_health_callback(fix)

    assert '/fix' not in nav_server._valid_fix_received_at_by_topic
    assert 'degenerate' in nav_server._fix_rejections_by_topic['/fix']

    gps_odom = Odometry()
    gps_odom.header.frame_id = 'map'
    gps_odom.header.stamp = nav_server.get_clock().now().to_msg()
    gps_odom.pose.covariance[0] = variance_x
    gps_odom.pose.covariance[1] = covariance_xy
    gps_odom.pose.covariance[6] = covariance_xy
    gps_odom.pose.covariance[7] = variance_y

    nav_server._gps_odometry_health_callback(gps_odom)

    assert nav_server._last_gps_odometry_received_at is None
    assert 'degenerate' in (
        nav_server._gps_odometry_rejection_reason or ''
    )


def test_navigation_health_rejects_replayed_sources_and_bad_pose(
    health_nav_server,
):
    """Fresh relay callbacks must not disguise stale source timestamps."""
    nav_server = health_nav_server
    pose = PoseStamped()
    pose.header.frame_id = 'odom'
    pose.header.stamp = nav_server.get_clock().now().to_msg()
    pose.pose.orientation.w = 1.0
    nav_server._robot_pose_health_callback(pose)
    assert 'frame must be map' in (
        nav_server._navigation_health_block_reason_locked() or ''
    )

    pose.header.frame_id = 'map'
    pose.header.stamp.sec = 1
    pose.header.stamp.nanosec = 0
    nav_server._robot_pose_health_callback(pose)
    assert 'timestamp is stale' in (
        nav_server._navigation_health_block_reason_locked() or ''
    )

    fix = NavSatFix()
    fix.header.stamp = nav_server.get_clock().now().to_msg()
    fix.status.status = NavSatStatus.STATUS_FIX
    fix.latitude = 91.0
    fix.longitude = 120.5
    fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    fix.position_covariance[0] = 0.04
    fix.position_covariance[4] = 0.04
    nav_server._gps_health_callback(fix)
    assert '/fix' not in nav_server._valid_fix_received_at_by_topic

    gps_odom = Odometry()
    gps_odom.header.frame_id = 'odom'
    gps_odom.header.stamp = nav_server.get_clock().now().to_msg()
    gps_odom.pose.covariance[0] = 0.04
    gps_odom.pose.covariance[7] = 0.04
    nav_server._gps_odometry_health_callback(gps_odom)
    assert nav_server._last_gps_odometry_received_at is None
    assert 'frame_id must be map' in (
        nav_server._gps_odometry_rejection_reason or ''
    )
