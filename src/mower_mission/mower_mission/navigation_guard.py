"""Fail-closed cross-node guard for mutations during navigation."""

import threading
from functools import wraps
from time import monotonic
from uuid import uuid4

from mower_interface.srv import MissionOperationLock
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Bool


def guarded_mission_mutation(operation: str):
    """Wrap a synchronous ROS service callback in the central mutation lease."""
    def decorate(callback):
        @wraps(callback)
        def wrapped(self, request, response):
            if not self._acquire_mission_mutation(response, operation):
                return response
            try:
                return callback(self, request, response)
            finally:
                if not self._release_mission_mutation():
                    previous = str(getattr(response, 'message', '')).strip()
                    response.success = False
                    response.message = (
                        f'{previous}; ' if previous else ''
                    ) + (
                        'operation may have completed, but mutation-lock '
                        'release is unconfirmed; navigation remains blocked. '
                        'Inspect the robot, then restart this node and '
                        'nav_action_server'
                    )
                    self.get_logger().error(response.message)
        return wrapped
    return decorate


class NavigationActivityGuard:
    """Mixin that tracks the nav server's transient-local activity flag."""

    def _init_navigation_activity_guard(self) -> None:
        # Unknown is treated as active. Mission mutation must not become
        # available merely because the nav action server is absent/restarting.
        self._nav_operation_active = True
        self._nav_operation_seen = False
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self._nav_operation_subscription = self.create_subscription(
            Bool,
            '/nav_operation_active',
            self._on_nav_operation_active,
            qos,
        )
        self._mutation_owner = (
            f'{self.get_fully_qualified_name()}:{uuid4().hex}'
        )
        self._local_mutation_lock = threading.Lock()
        self._mission_mutation_lease_held = False
        self._mutation_client_group = ReentrantCallbackGroup()
        self._mission_operation_lock_client = self.create_client(
            MissionOperationLock,
            '/mission_operation_lock',
            callback_group=self._mutation_client_group,
        )

    def _on_nav_operation_active(self, msg: Bool) -> None:
        self._nav_operation_active = bool(msg.data)
        self._nav_operation_seen = True

    def _mutation_block_reason(self) -> str | None:
        if not self._nav_operation_seen:
            return 'navigation state is not available'
        if self._nav_operation_active:
            return 'navigation is active'
        return None

    def _reject_mutation(self, response, operation: str) -> bool:
        """Populate a service response and return true when mutation is unsafe."""
        reason = self._mutation_block_reason()
        if reason is None:
            return False
        response.success = False
        response.message = f'Cannot {operation}: {reason}'
        self.get_logger().warn(response.message)
        return True

    def _wait_for_lock_response(self, future, timeout_s: float):
        """Wait for a lock response while another executor thread services it."""
        done = threading.Event()
        future.add_done_callback(lambda _future: done.set())
        deadline = monotonic() + timeout_s
        while not done.wait(timeout=min(0.05, max(0.0, deadline - monotonic()))):
            if monotonic() >= deadline:
                return None
        try:
            return future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Mission operation lock failed: {exc}')
            return None

    def _acquire_mission_mutation(
        self,
        response,
        operation: str,
        timeout_s: float = 3.0,
    ) -> bool:
        """Acquire the nav server's atomic mutation lease, or reject safely."""
        if self._reject_mutation(response, operation):
            return False
        if not self._local_mutation_lock.acquire(blocking=False):
            response.success = False
            response.message = 'Another mutation is already active in this node'
            return False
        if not self._mission_operation_lock_client.wait_for_service(
            timeout_sec=1.0
        ):
            self._local_mutation_lock.release()
            response.success = False
            response.message = 'Mission operation guard is unavailable'
            return False

        req = MissionOperationLock.Request()
        req.owner = self._mutation_owner
        req.operation = operation
        req.acquire = True
        result = self._wait_for_lock_response(
            self._mission_operation_lock_client.call_async(req), timeout_s
        )
        if result is None:
            # The server may have acquired the lease even though its response
            # was lost. Keep the local lock latched too: availability must not
            # win over accepting an overlapping real-robot operation.
            response.success = False
            response.message = (
                'Mission operation guard response timed out; inspect robot '
                'state, then restart both this node and nav_action_server'
            )
            self.get_logger().error(response.message)
            return False
        if not result.success:
            self._local_mutation_lock.release()
            response.success = False
            response.message = result.message
            return False
        self._mission_mutation_lease_held = True
        return True

    def _release_mission_mutation(self, timeout_s: float = 3.0) -> bool:
        """Release a held mutation lease; stay fail-closed if uncertain."""
        if not self._mission_mutation_lease_held:
            return False
        req = MissionOperationLock.Request()
        req.owner = self._mutation_owner
        req.operation = ''
        req.acquire = False
        result = self._wait_for_lock_response(
            self._mission_operation_lock_client.call_async(req), timeout_s
        )
        if result is None or not result.success:
            message = (
                'Mission mutation lease release is unconfirmed; navigation '
                'remains blocked'
            )
            if result is not None and result.message:
                message += f': {result.message}'
            self.get_logger().error(message)
            return False
        self._mission_mutation_lease_held = False
        self._local_mutation_lock.release()
        return True
