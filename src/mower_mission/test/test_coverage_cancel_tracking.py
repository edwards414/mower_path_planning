"""Behavioral regression tests for bounded coverage cancel tracking."""

import ast
import threading as real_threading
from pathlib import Path
from types import SimpleNamespace


COVERAGE_NODE = (
    Path(__file__).resolve().parents[1] / 'mower_mission/coverage_node.py'
)


class ManualTimer:
    """A deterministic replacement for ``threading.Timer``."""

    created = []

    def __init__(self, interval, callback):
        self.interval = interval
        self.callback = callback
        self.daemon = False
        self.started = False
        self.canceled = False
        self.fired = False
        self.__class__.created.append(self)

    def start(self):
        self.started = True

    def cancel(self):
        self.canceled = True

    def fire(self):
        if self.canceled or self.fired:
            return
        self.fired = True
        self.callback()


class ManualFuture:
    """Minimal future whose completion is controlled by the test."""

    def __init__(self):
        self.callbacks = []
        self.cancel_count = 0
        self._done = False
        self._result = None

    def add_done_callback(self, callback):
        self.callbacks.append(callback)
        if self._done:
            callback(self)

    def done(self):
        return self._done

    def cancel(self):
        self.cancel_count += 1
        self._done = True
        for callback in tuple(self.callbacks):
            callback(self)

    def result(self):
        return self._result

    def set_result(self, result):
        self._result = result
        self._done = True
        for callback in tuple(self.callbacks):
            callback(self)


class BrokenCallbackFuture(ManualFuture):
    def add_done_callback(self, callback):
        raise RuntimeError('callback channel unavailable')


class FakeCancelClient:
    def __init__(self):
        self.futures = []
        self.requests = []

    def wait_for_service(self, timeout_sec):
        return True

    def call_async(self, request):
        self.requests.append(request)
        future = ManualFuture()
        self.futures.append(future)
        return future


class FakeLogger:
    def __getattr__(self, _name):
        return lambda *_args, **_kwargs: None


class FakeHandle:
    def __init__(self, result_future):
        self.result_future = result_future

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        raise RuntimeError('force correlated fallback')


def _load_coverage_methods():
    tree = ast.parse(COVERAGE_NODE.read_text(encoding='utf-8'))
    wanted = {
        '_request_nav2_cancel_fallback',
        '_track_and_cancel_navigation_goal',
        '_shutdown_navigation_goal_tracking',
    }
    functions = [
        node
        for class_node in tree.body
        if isinstance(class_node, ast.ClassDef)
        and class_node.name == 'CoveragePlanner'
        for node in class_node.body
        if isinstance(node, ast.FunctionDef) and node.name in wanted
    ]
    request_type = type('Request', (), {'dispatch_id': ''})
    namespace = {
        'threading': SimpleNamespace(
            Lock=real_threading.Lock,
            Event=real_threading.Event,
            Timer=ManualTimer,
        ),
        'CancelNavigationDispatch': SimpleNamespace(Request=request_type),
        '_proven_action_result': lambda future: future.result(),
    }
    module = ast.fix_missing_locations(
        ast.Module(body=functions, type_ignores=[])
    )
    exec(compile(module, str(COVERAGE_NODE), 'exec'), namespace)
    return namespace


class FakeCoverage:
    def __init__(self, methods):
        for name, method in methods.items():
            if name.startswith('_') and callable(method):
                setattr(self, name, method.__get__(self, type(self)))
        self._goal_tracking_lock = real_threading.Lock()
        self._unconfirmed_goal_handles = {}
        self._goal_tracking_timers = set()
        self._fallback_cancel_attempts = {}
        self._goal_tracking_shutdown = False
        self._exec_goal_handle = None
        self._sequence_active_goal = None
        self._cancel_navigation_dispatch_client = FakeCancelClient()
        self.cleared_goals = []

    def get_parameter(self, _name):
        return SimpleNamespace(value=5.0)

    def get_logger(self):
        return FakeLogger()

    def _clear_active_navigation_goal(
        self,
        handle,
        dispatch_id,
        result_future,
    ):
        self.cleared_goals.append((handle, dispatch_id, result_future))


def _new_node():
    ManualTimer.created = []
    return FakeCoverage(_load_coverage_methods())


def _active_timers(interval):
    return [
        timer
        for timer in ManualTimer.created
        if timer.interval == interval
        and timer.started
        and not timer.canceled
        and not timer.fired
    ]


def test_direct_and_late_goal_tracking_share_one_dispatch_attempt():
    node = _new_node()
    dispatch_id = 'dispatch-a'
    direct_cancel = node._request_nav2_cancel_fallback(
        'acceptance timeout',
        dispatch_id,
    )
    assert callable(direct_cancel)
    assert len(node._cancel_navigation_dispatch_client.requests) == 1

    result_future = ManualFuture()
    handle = FakeHandle(result_future)
    assert node._track_and_cancel_navigation_goal(
        handle,
        'late acceptance',
        dispatch_id,
        result_future=result_future,
    )
    assert len(node._cancel_navigation_dispatch_client.requests) == 1

    for timer in tuple(_active_timers(2.0)):
        timer.fire()
    assert len(node._cancel_navigation_dispatch_client.requests) == 1

    timeout = _active_timers(5.0)
    assert len(timeout) == 1
    timeout[0].fire()
    assert node._cancel_navigation_dispatch_client.futures[0].cancel_count == 1

    retry = _active_timers(2.0)
    assert retry
    retry[-1].fire()
    assert len(node._cancel_navigation_dispatch_client.requests) == 2
    newer_attempt = node._fallback_cancel_attempts[dispatch_id]

    # A late completion from the timed-out request cannot clear its successor.
    node._cancel_navigation_dispatch_client.futures[0].set_result(
        SimpleNamespace(success=True, terminal_confirmed=True)
    )
    assert node._fallback_cancel_attempts[dispatch_id] is newer_attempt


def test_terminal_and_shutdown_cancel_all_tracking_resources():
    node = _new_node()
    result_future = ManualFuture()
    handle = FakeHandle(result_future)
    assert node._track_and_cancel_navigation_goal(
        handle,
        'cancel failed',
        'dispatch-b',
        result_future=result_future,
    )
    pending = node._cancel_navigation_dispatch_client.futures[0]
    assert node._goal_tracking_timers
    assert node._fallback_cancel_attempts

    result_future.set_result(object())
    assert id(handle) not in node._unconfirmed_goal_handles
    assert not node._goal_tracking_timers
    assert not node._fallback_cancel_attempts
    assert pending.cancel_count == 1
    assert all(timer.canceled for timer in ManualTimer.created)
    assert node.cleared_goals == [(handle, 'dispatch-b', result_future)]

    # Seed another direct attempt and verify idempotent node teardown.
    node._request_nav2_cancel_fallback('shutdown', 'dispatch-c')
    shutdown_future = node._cancel_navigation_dispatch_client.futures[-1]
    calls_before_shutdown = len(
        node._cancel_navigation_dispatch_client.requests
    )
    node._shutdown_navigation_goal_tracking()
    node._shutdown_navigation_goal_tracking()
    assert node._goal_tracking_shutdown
    assert not node._unconfirmed_goal_handles
    assert not node._goal_tracking_timers
    assert not node._fallback_cancel_attempts
    assert shutdown_future.cancel_count == 1
    assert node._request_nav2_cancel_fallback(
        'post shutdown',
        'dispatch-d',
    ) is None
    assert len(node._cancel_navigation_dispatch_client.requests) == (
        calls_before_shutdown
    )


def test_terminal_fallback_response_cleans_goal_once():
    node = _new_node()
    result_future = ManualFuture()
    handle = FakeHandle(result_future)
    assert node._track_and_cancel_navigation_goal(
        handle,
        'cancel failed',
        'dispatch-terminal',
        result_future=result_future,
    )

    node._cancel_navigation_dispatch_client.futures[0].set_result(
        SimpleNamespace(success=True, terminal_confirmed=True)
    )
    assert id(handle) not in node._unconfirmed_goal_handles
    assert not node._goal_tracking_timers
    assert not node._fallback_cancel_attempts
    assert all(timer.canceled for timer in ManualTimer.created)
    assert node.cleared_goals == [
        (handle, 'dispatch-terminal', result_future)
    ]

    # Replaying either completion source must remain idempotent.
    result_future.set_result(object())
    assert node.cleared_goals == [
        (handle, 'dispatch-terminal', result_future)
    ]


def test_broken_result_callback_still_starts_correlated_cancel_tracking():
    node = _new_node()
    result_future = BrokenCallbackFuture()
    handle = FakeHandle(result_future)

    assert node._track_and_cancel_navigation_goal(
        handle,
        'result channel failed',
        'dispatch-broken-result',
        result_future=result_future,
    )

    assert len(node._cancel_navigation_dispatch_client.requests) == 1
    assert id(handle) in node._unconfirmed_goal_handles
    assert node._goal_tracking_timers

    # There was no callback channel, so local completion is not terminal proof
    # for the tracker; the correlated cancel path remains responsible.
    result_future.set_result(object())
    assert node.cleared_goals == []
    assert id(handle) in node._unconfirmed_goal_handles


def test_destroy_node_stops_tracking_before_ros_entity_destruction():
    tree = ast.parse(COVERAGE_NODE.read_text(encoding='utf-8'))
    destroy = next(
        node
        for class_node in tree.body
        if isinstance(class_node, ast.ClassDef)
        and class_node.name == 'CoveragePlanner'
        for node in class_node.body
        if isinstance(node, ast.FunctionDef) and node.name == 'destroy_node'
    )
    source = ast.unparse(destroy)
    assert source.index('self._shutdown_navigation_goal_tracking()') < (
        source.index('super().destroy_node()')
    )
