"""Unit tests for nav_action_server cancel and status services."""

from types import SimpleNamespace

from mower_mission.navigation import nav_action_server as nav_module

import pytest

import rclpy

from std_srvs.srv import Trigger


class FakeNavigator:
    """Small BasicNavigator stand-in for service callback tests."""

    def __init__(self):
        """Initialize fake task state."""
        self.cancel_calls = 0
        self.complete = False
        self.result = nav_module.TaskResult.SUCCEEDED
        self.feedback = SimpleNamespace(distance_remaining=1.25)

    def cancelTask(self):
        """Record cancel requests without touching Nav2."""
        self.cancel_calls += 1

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


@pytest.fixture
def nav_server(monkeypatch):
    """Create NavActionServer with BasicNavigator replaced by a fake."""
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


def test_cancel_service_cancels_server_owned_navigator(nav_server):
    """The public cancel service must cancel nav_action_server.navigator."""
    nav_server._active_task_name = 'Follow coverage segment 1'
    nav_server._nav_state = 'running'

    response = nav_server.cancel_nav2_srv(
        Trigger.Request(),
        Trigger.Response(),
    )

    assert response.success
    assert nav_server.navigator.cancel_calls == 1
    assert nav_server._external_cancel_requested is True
    assert 'Follow coverage segment 1' in response.message


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
    nav_server._nav_state = 'running'

    response = nav_server.check_nav_status_srv(
        Trigger.Request(),
        Trigger.Response(),
    )

    assert not response.success
    assert 'Follow coverage segment 2' in response.message
    assert 'distance_remaining=1.250' in response.message


def test_status_service_maps_completed_nav2_result(nav_server):
    """Completed Nav2 task should update service status from navigator."""
    nav_server._active_task_name = 'Follow coverage segment 3'
    nav_server._nav_state = 'running'
    nav_server.navigator.complete = True
    nav_server.navigator.result = nav_module.TaskResult.CANCELED

    response = nav_server.check_nav_status_srv(
        Trigger.Request(),
        Trigger.Response(),
    )

    assert response.success
    assert nav_server._nav_state == 'canceled'
    assert response.message == 'Follow coverage segment 3 canceled'
