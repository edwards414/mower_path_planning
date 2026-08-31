"""Safety tests for the final drivetrain command trust boundary."""

import time
from types import SimpleNamespace

from geometry_msgs.msg import TwistStamped
import pytest
import rclpy

from mower_bringup.velocity_command_guard import VelocityCommandGuard


@pytest.fixture
def guard():
    if not rclpy.ok():
        rclpy.init()
    node = VelocityCommandGuard()
    published = []
    node._publisher = SimpleNamespace(publish=published.append)
    try:
        yield node, published
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


@pytest.mark.parametrize('invalid', [float('nan'), float('inf'), float('-inf')])
def test_invalid_stream_forces_zero_instead_of_refreshing_old_motion(
    guard,
    invalid,
):
    node, published = guard
    moving = TwistStamped()
    moving.twist.linear.x = 0.2
    node._command_callback(moving)
    assert published[-1].twist.linear.x == pytest.approx(0.2)

    malformed = TwistStamped()
    malformed.twist.linear.x = invalid
    node._command_callback(malformed)

    assert published[-1].twist.linear.x == 0.0
    assert published[-1].twist.angular.z == 0.0
    assert not node._nonzero_active


def test_future_timestamp_is_rejected_and_replaced_with_robot_time(
    guard,
):
    node, published = guard
    command = TwistStamped()
    command.twist.linear.x = 0.2
    future = node.get_clock().now().nanoseconds + 10_000_000_000
    command.header.stamp.sec = future // 1_000_000_000
    command.header.stamp.nanosec = future % 1_000_000_000

    node._command_callback(command)

    assert published[-1].twist.linear.x == 0.0
    output_stamp = (
        published[-1].header.stamp.sec * 1_000_000_000
        + published[-1].header.stamp.nanosec
    )
    assert output_stamp > 0
    assert output_stamp != future


def test_future_rejection_blocks_motion_queued_before_the_rejection(guard):
    node, published = guard
    now_ns = node.get_clock().now().nanoseconds

    moving = TwistStamped()
    moving.twist.linear.x = 0.2
    moving_stamp = now_ns - 2_000_000
    moving.header.stamp.sec = moving_stamp // 1_000_000_000
    moving.header.stamp.nanosec = moving_stamp % 1_000_000_000
    node._command_callback(moving)
    assert published[-1].twist.linear.x == pytest.approx(0.2)

    future_stop = TwistStamped()
    future_stamp = now_ns + 10_000_000_000
    future_stop.header.stamp.sec = future_stamp // 1_000_000_000
    future_stop.header.stamp.nanosec = future_stamp % 1_000_000_000
    node._command_callback(future_stop)
    assert published[-1].twist.linear.x == 0.0

    queued = TwistStamped()
    queued.twist.linear.x = 0.2
    queued_stamp = now_ns - 1_000_000
    queued.header.stamp.sec = queued_stamp // 1_000_000_000
    queued.header.stamp.nanosec = queued_stamp % 1_000_000_000
    node._command_callback(queued)

    assert published[-1].twist.linear.x == 0.0
    assert not node._nonzero_active


def test_steady_receipt_watchdog_emits_zero(guard):
    node, published = guard
    moving = TwistStamped()
    moving.twist.angular.z = 0.4
    node._command_callback(moving)
    node._last_nonzero_received_at = (
        time.monotonic() - node._command_timeout_s - 0.01
    )

    node._watchdog_callback()

    assert published[-1].twist.linear.x == 0.0
    assert published[-1].twist.angular.z == 0.0
    assert not node._nonzero_active


def test_replayed_stamped_command_cannot_refresh_or_resume_motion(guard):
    node, published = guard
    command = TwistStamped()
    command.twist.linear.x = 0.2
    stamp = node.get_clock().now().nanoseconds
    command.header.stamp.sec = stamp // 1_000_000_000
    command.header.stamp.nanosec = stamp % 1_000_000_000
    node._command_callback(command)
    first_receipt = node._last_nonzero_received_at

    node._command_callback(command)
    assert node._last_nonzero_received_at == first_receipt

    node._last_nonzero_received_at = (
        time.monotonic() - node._command_timeout_s - 0.01
    )
    node._command_callback(command)
    assert published[-1].twist.linear.x == 0.0
    assert not node._nonzero_active


def test_newer_stamped_stop_blocks_older_queued_motion(guard):
    node, published = guard
    moving = TwistStamped()
    moving.twist.linear.x = 0.2
    moving.header.stamp.sec = 100
    node._max_input_age_s = float('inf')
    node._max_future_skew_s = float('inf')
    node._command_callback(moving)

    stop = TwistStamped()
    stop.header.stamp.sec = 200
    node._command_callback(stop)
    assert published[-1].twist.linear.x == 0.0

    queued = TwistStamped()
    queued.twist.linear.x = 0.2
    queued.header.stamp.sec = 150
    node._command_callback(queued)

    assert published[-1].twist.linear.x == 0.0
    assert not node._nonzero_active


def test_newer_invalid_command_is_also_a_stop_ordering_barrier(guard):
    node, published = guard
    node._max_input_age_s = float('inf')
    node._max_future_skew_s = float('inf')

    moving = TwistStamped()
    moving.twist.linear.x = 0.2
    moving.header.stamp.sec = 100
    node._command_callback(moving)

    invalid = TwistStamped()
    invalid.twist.linear.x = node._max_linear_x + 1.0
    invalid.header.stamp.sec = 200
    node._command_callback(invalid)
    assert published[-1].twist.linear.x == 0.0

    queued = TwistStamped()
    queued.twist.linear.x = 0.2
    queued.header.stamp.sec = 150
    node._command_callback(queued)

    assert published[-1].twist.linear.x == 0.0
    assert not node._nonzero_active


def test_manual_guard_accepts_only_current_robot_clock_and_session(guard):
    node, published = guard
    node._require_command_session = True
    node._command_session_id = 'manual-session-v1:test'

    missing_session = TwistStamped()
    missing_session.twist.linear.x = 0.2
    node._command_callback(missing_session)
    assert published[-1].twist.linear.x == 0.0

    command = TwistStamped()
    command.header.frame_id = node._command_session_id
    stamp = node.get_clock().now().nanoseconds
    command.header.stamp.sec = stamp // 1_000_000_000
    command.header.stamp.nanosec = stamp % 1_000_000_000
    command.twist.linear.x = 0.2
    node._command_callback(command)

    assert published[-1].twist.linear.x == pytest.approx(0.2)
    assert node._nonzero_active


def test_manual_guard_rejects_stale_session_even_with_fresh_stamp(guard):
    node, published = guard
    node._require_command_session = True
    node._command_session_id = 'manual-session-v1:current'
    command = TwistStamped()
    command.header.frame_id = 'manual-session-v1:old'
    stamp = node.get_clock().now().nanoseconds
    command.header.stamp.sec = stamp // 1_000_000_000
    command.header.stamp.nanosec = stamp % 1_000_000_000
    command.twist.angular.z = 0.2

    node._command_callback(command)

    assert published[-1].twist.angular.z == 0.0
    assert not node._nonzero_active


def test_manual_guard_restart_stop_blocks_old_writer_motion_at_final_guard(
    guard,
):
    manual, manual_published = guard
    manual._require_command_session = True
    manual._command_session_id = 'manual-session-v1:new'

    # A newly started manual guard publishes a robot-stamped zero. Deliver that
    # barrier to a separate final guard before an old DDS writer's queued frame.
    manual._publish_zero()
    restart_stop = manual_published[-1]
    assert restart_stop.header.stamp.sec != 0
    stop_ns = (
        restart_stop.header.stamp.sec * 1_000_000_000
        + restart_stop.header.stamp.nanosec
    )
    old_stamp_ns = stop_ns - 1
    old_motion = TwistStamped()
    old_motion.twist.linear.x = 0.2
    old_motion.header.stamp.sec = old_stamp_ns // 1_000_000_000
    old_motion.header.stamp.nanosec = old_stamp_ns % 1_000_000_000

    final = VelocityCommandGuard()
    final_published = []
    final._publisher = SimpleNamespace(publish=final_published.append)
    try:
        final._command_callback(restart_stop)
        final._command_callback(old_motion)
        assert final_published[-1].twist.linear.x == 0.0
        assert not final._nonzero_active
    finally:
        final.destroy_node()
