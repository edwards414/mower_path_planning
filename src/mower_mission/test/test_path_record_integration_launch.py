"""Launch tests for path_record_node ROS service integration."""

import tempfile
import time

from geometry_msgs.msg import TransformStamped

import launch

import launch_ros.actions

import launch_testing.actions

from mower_interface.srv import ChannelPathList, ChennalPathList

import pytest

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger

from tf2_ros import TransformBroadcaster

from visualization_msgs.msg import MarkerArray


@pytest.mark.launch_test
def generate_test_description():
    """Launch path_record_node under test."""
    save_dir = tempfile.mkdtemp(prefix='mower_path_record_')

    path_recorder = launch_ros.actions.Node(
        package='mower_mission',
        executable='path_record_node',
        name='path_record_node_test',
        output='screen',
        parameters=[{
            'save_dir': save_dir,
            'min_dist': 0.05,
            'min_dt': 0.0,
        }],
    )

    return launch.LaunchDescription([
        path_recorder,
        launch_testing.actions.ReadyToTest(),
    ]), {
        'path_recorder': path_recorder,
        'save_dir': save_dir,
    }


def _spin_until_future(node: Node, future, timeout_sec=5.0):
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and time.monotonic() < deadline and not future.done():
        rclpy.spin_once(node, timeout_sec=0.05)
    assert future.done(), 'Timed out waiting for ROS service response'
    return future.result()


def _call_trigger(node: Node, service_name: str):
    client = node.create_client(Trigger, service_name)
    assert client.wait_for_service(timeout_sec=5.0), (
        f'{service_name} service was not available'
    )

    future = client.call_async(Trigger.Request())
    return _spin_until_future(node, future)


def _broadcast_robot_tf(
    node: Node,
    broadcaster: TransformBroadcaster,
    x: float,
    y: float,
    repeat_count: int = 20,
):
    for _ in range(repeat_count):
        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)
        rclpy.spin_once(node, timeout_sec=0.05)


class TestPathRecordLaunch:
    """Verify path_record_node behavior through ROS graph APIs."""

    @classmethod
    def setup_class(cls):
        """Initialize rclpy once for this launch test class."""
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def teardown_class(cls):
        """Shut down rclpy after the launch tests finish."""
        if rclpy.ok():
            rclpy.shutdown()

    def setup_method(self):
        """Create a ROS client node and TF broadcaster for each test."""
        self.node = rclpy.create_node('path_record_launch_test_client')
        self.tf_broadcaster = TransformBroadcaster(self.node)

    def teardown_method(self):
        """Destroy the ROS client node after each test."""
        self.node.destroy_node()

    def test_chennal_record_services_create_marker_array(self):
        """Record a legacy chennal path and read it back as MarkerArray."""
        received_arrays = []
        self.node.create_subscription(
            MarkerArray,
            '/chennal_path_array',
            lambda msg: received_arrays.append(msg),
            10,
        )

        _broadcast_robot_tf(self.node, self.tf_broadcaster, 0.0, 0.0)
        start_response = _call_trigger(self.node, '/chennal_record_start')
        assert start_response.success

        _broadcast_robot_tf(self.node, self.tf_broadcaster, 0.6, 0.0)
        end_response = _call_trigger(self.node, '/chennal_record_end')
        assert end_response.success

        legacy_client = self.node.create_client(
            ChennalPathList,
            '/get_chennal_path_list',
        )
        assert legacy_client.wait_for_service(timeout_sec=5.0)
        legacy_response = _spin_until_future(
            self.node,
            legacy_client.call_async(ChennalPathList.Request()),
        )

        assert legacy_response.success
        assert len(legacy_response.chennal_path_array.markers) == 1
        assert len(legacy_response.chennal_path_array.markers[0].points) >= 2

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not received_arrays:
            rclpy.spin_once(self.node, timeout_sec=0.05)

        assert received_arrays
        assert len(received_arrays[-1].markers) == 1

    def test_channel_alias_services_share_chennal_path_list(self):
        """Record via channel aliases and read the shared channel list."""
        _broadcast_robot_tf(self.node, self.tf_broadcaster, 1.0, 0.0)
        start_response = _call_trigger(self.node, '/channel_record_start')
        assert start_response.success

        _broadcast_robot_tf(self.node, self.tf_broadcaster, 1.8, 0.0)
        end_response = _call_trigger(self.node, '/channel_record_end')
        assert end_response.success

        alias_client = self.node.create_client(
            ChannelPathList,
            '/get_channel_path_list',
        )
        assert alias_client.wait_for_service(timeout_sec=5.0)
        alias_response = _spin_until_future(
            self.node,
            alias_client.call_async(ChannelPathList.Request()),
        )

        assert alias_response.success
        assert len(alias_response.channel_path_array.markers) >= 1
        assert len(alias_response.channel_path_array.markers[-1].points) >= 2
