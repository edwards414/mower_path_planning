#!/usr/bin/env python3

"""Final robot-side validation and watchdog for drivetrain commands."""

import math
import secrets
import time

from geometry_msgs.msg import TwistStamped
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import Header


class VelocityCommandGuard(Node):
    """Validate mux output and stop it on a steady-clock receipt timeout."""

    def __init__(self):
        super().__init__('velocity_command_guard')

        def immutable():
            return ParameterDescriptor(
                read_only=True,
                description='Safety limit; set only when the node starts',
            )

        self.declare_parameter('command_timeout_s', 0.20, immutable())
        self.declare_parameter('max_input_age_s', 0.25, immutable())
        self.declare_parameter('max_future_skew_s', 0.05, immutable())
        self.declare_parameter('max_linear_x_m_s', 0.50, immutable())
        self.declare_parameter('max_angular_z_rad_s', 1.00, immutable())
        self.declare_parameter('require_command_session', False, immutable())
        self.declare_parameter('command_clock_period_s', 0.05, immutable())

        self._command_timeout_s = max(
            0.05,
            float(self.get_parameter('command_timeout_s').value),
        )
        self._max_input_age_s = max(
            0.0,
            float(self.get_parameter('max_input_age_s').value),
        )
        self._max_future_skew_s = max(
            0.0,
            float(self.get_parameter('max_future_skew_s').value),
        )
        self._max_linear_x = max(
            0.0,
            float(self.get_parameter('max_linear_x_m_s').value),
        )
        self._max_angular_z = max(
            0.0,
            float(self.get_parameter('max_angular_z_rad_s').value),
        )
        self._require_command_session = bool(
            self.get_parameter('require_command_session').value
        )
        self._command_clock_period_s = max(
            0.02,
            float(self.get_parameter('command_clock_period_s').value),
        )
        limits = (
            self._command_timeout_s,
            self._max_input_age_s,
            self._max_future_skew_s,
            self._max_linear_x,
            self._max_angular_z,
            self._command_clock_period_s,
        )
        if not all(math.isfinite(value) for value in limits):
            raise ValueError('velocity guard safety limits must be finite')

        self._publisher = self.create_publisher(
            TwistStamped,
            'cmd_vel_out',
            1,
        )
        self.create_subscription(
            TwistStamped,
            'cmd_vel_in',
            self._command_callback,
            1,
        )
        self._nonzero_active = False
        self._last_nonzero_received_at = None
        self._last_source_stamp_ns = None
        self._last_rejection_log_at = float('-inf')
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._command_session_id = (
            f'manual-session-v1:{secrets.token_hex(16)}'
            if self._require_command_session
            else None
        )
        self._command_clock_publisher = None
        self._command_clock_timer = None
        if self._command_session_id is not None:
            self._command_clock_publisher = self.create_publisher(
                Header,
                'command_clock',
                1,
            )
            self._command_clock_timer = self.create_timer(
                self._command_clock_period_s,
                self._publish_command_clock,
                clock=self._steady_clock,
            )
            self._publish_command_clock()
        self._watchdog = self.create_timer(
            min(0.05, self._command_timeout_s / 2.0),
            self._watchdog_callback,
            clock=self._steady_clock,
        )
        self._publish_zero()

    @staticmethod
    def _stamp_is_zero(message: TwistStamped) -> bool:
        return (
            int(message.header.stamp.sec) == 0
            and int(message.header.stamp.nanosec) == 0
        )

    @classmethod
    def _source_stamp_ns(cls, message: TwistStamped) -> int | None:
        if cls._stamp_is_zero(message):
            return None
        return (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )

    def _apply_timestamp_barrier(
        self,
        message: TwistStamped,
    ) -> tuple[str | None, int | None, bool]:
        """Advance a trusted stamped ordering barrier before payload checks."""
        stamp_ns = self._source_stamp_ns(message)
        if stamp_ns is None:
            return None, None, False
        age_s = (
            self.get_clock().now().nanoseconds - stamp_ns
        ) / 1_000_000_000.0
        if age_s < -self._max_future_skew_s:
            return (
                'velocity timestamp is too far in the future',
                stamp_ns,
                False,
            )
        previous_stamp = self._last_source_stamp_ns
        if previous_stamp is not None and stamp_ns < previous_stamp:
            return 'velocity source timestamp moved backward', stamp_ns, False
        is_new = previous_stamp is None or stamp_ns > previous_stamp
        if is_new:
            # A newer malformed/overspeed command still represents a newer
            # stop barrier. A queued older valid motion must never resume after
            # the guard rejected it and published zero.
            self._last_source_stamp_ns = stamp_ns
        return None, stamp_ns, is_new

    def _block_reason(
        self,
        message: TwistStamped,
        source_stamp_ns: int | None,
        timestamp_is_new: bool,
    ) -> str | None:
        if self._require_command_session:
            if message.header.frame_id != self._command_session_id:
                return 'manual velocity command session is missing or stale'
            if source_stamp_ns is None:
                return 'manual velocity requires the robot command clock'

        twist = message.twist
        values = (
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
        )
        if not all(math.isfinite(float(value)) for value in values):
            return 'velocity contains NaN or infinity'
        unsupported = (
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
        )
        if any(abs(float(value)) > 1e-9 for value in unsupported):
            return 'unsupported lateral or non-yaw velocity component'
        if abs(float(twist.linear.x)) > self._max_linear_x:
            return 'linear velocity exceeds robot safety limit'
        if abs(float(twist.angular.z)) > self._max_angular_z:
            return 'angular velocity exceeds robot safety limit'

        # Zero stamps request receipt-time semantics for trusted local sources.
        # The untrusted Flutter path requires a robot-issued nonzero stamp.
        if source_stamp_ns is not None:
            age_s = (
                self.get_clock().now().nanoseconds - source_stamp_ns
            ) / 1_000_000_000.0
            if age_s > self._max_input_age_s:
                return 'velocity timestamp is stale'
            moving = (
                abs(float(twist.linear.x)) > 1e-9
                or abs(float(twist.angular.z)) > 1e-9
            )
            if moving and not timestamp_is_new:
                received_at = self._last_nonzero_received_at
                replay_expired = (
                    received_at is None
                    or time.monotonic() - received_at
                    > self._command_timeout_s
                )
                if not self._nonzero_active or replay_expired:
                    return 'replayed velocity cannot resume stopped motion'
        return None

    def _publish_command_clock(self) -> None:
        publisher = self._command_clock_publisher
        session_id = self._command_session_id
        if publisher is None or session_id is None:
            return
        message = Header()
        message.stamp = self.get_clock().now().to_msg()
        message.frame_id = session_id
        publisher.publish(message)

    def _command_callback(self, message: TwistStamped) -> None:
        reason, source_stamp_ns, timestamp_is_new = (
            self._apply_timestamp_barrier(message)
        )
        if reason is None:
            reason = self._block_reason(
                message,
                source_stamp_ns,
                timestamp_is_new,
            )
        if reason is not None:
            # A rejection is a stop boundary even when its source stamp is
            # untrusted (for example, far in the future). Advance only to the
            # robot's own clock so any command already queued before this
            # rejection cannot become a fresh resume command afterward.
            rejection_barrier_ns = self.get_clock().now().nanoseconds
            previous_stamp = self._last_source_stamp_ns
            if (
                previous_stamp is None
                or rejection_barrier_ns > previous_stamp
            ):
                self._last_source_stamp_ns = rejection_barrier_ns
            self._nonzero_active = False
            self._last_nonzero_received_at = None
            self._publish_zero()
            now = time.monotonic()
            if now - self._last_rejection_log_at >= 1.0:
                self.get_logger().error(
                    f'Rejected drivetrain command: {reason}'
                )
                self._last_rejection_log_at = now
            return

        output = TwistStamped()
        # Replace every upstream stamp with robot time. This preserves ordering
        # across the manual guard -> mux -> final guard boundary without ever
        # trusting or forwarding a remote clock value.
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = 'base_footprint'
        output.twist.linear.x = float(message.twist.linear.x)
        output.twist.angular.z = float(message.twist.angular.z)
        self._publisher.publish(output)

        moving = (
            abs(output.twist.linear.x) > 1e-9
            or abs(output.twist.angular.z) > 1e-9
        )
        if not moving:
            self._nonzero_active = False
            self._last_nonzero_received_at = None
            return

        refresh_freshness = source_stamp_ns is None
        if source_stamp_ns is not None:
            refresh_freshness = timestamp_is_new
        self._nonzero_active = True
        if refresh_freshness:
            self._last_nonzero_received_at = time.monotonic()

    def _watchdog_callback(self) -> None:
        received_at = self._last_nonzero_received_at
        if (
            not self._nonzero_active
            or received_at is None
            or time.monotonic() - received_at <= self._command_timeout_s
        ):
            return
        self._nonzero_active = False
        self._last_nonzero_received_at = None
        self._publish_zero()
        self.get_logger().error(
            'Drivetrain command receipt timeout; forced velocity to zero'
        )

    def _publish_zero(self) -> None:
        stop = TwistStamped()
        # A forced zero is also an ordering barrier. In particular, a fresh
        # manual-guard process must make queued motion from its old DDS writer
        # older than this stop when both arrive at the final guard.
        stop.header.stamp = self.get_clock().now().to_msg()
        stop.header.frame_id = 'base_footprint'
        self._publisher.publish(stop)

    def destroy_node(self):
        self._publish_zero()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VelocityCommandGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
