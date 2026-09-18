"""Launch tests for path_record_node ROS service integration."""

import tempfile
import time

import launch

import launch_ros.actions

import launch_testing.actions

from mower_interface.srv import (
    ChannelPathList,
    ChennalPathList,
    GetZoneList,
    MissionOperationLock,
)

import pytest

import rclpy
from nav_msgs.msg import Odometry

from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from std_msgs.msg import Bool
from std_srvs.srv import Trigger

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


def _publish_robot_pose(
    node: Node,
    publisher: Publisher,
    x: float,
    y: float,
    repeat_count: int = 20,
):
    # path_record_node reads its map -> base_footprint pose from the EKF map
    # odometry topic (robot_pose_source_topic, default /odometry/global).
    for _ in range(repeat_count):
        odom = Odometry()
        odom.header.stamp = node.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.w = 1.0
        publisher.publish(odom)
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
        """Create a ROS client node and pose publisher for each test."""
        self.node = rclpy.create_node('path_record_launch_test_client')
        self.pose_publisher = self.node.create_publisher(
            Odometry, '/odometry/global', 10)
        self._lock_owner = None

        def operation_lock(req, res):
            if req.acquire:
                if self._lock_owner is not None:
                    res.success = False
                    res.message = 'busy'
                else:
                    self._lock_owner = req.owner
                    res.success = True
                    res.message = 'acquired'
            elif self._lock_owner == req.owner:
                self._lock_owner = None
                res.success = True
                res.message = 'released'
            else:
                res.success = False
                res.message = 'not owner'
            return res

        self.node.create_service(
            MissionOperationLock,
            '/mission_operation_lock',
            operation_lock,
        )
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.nav_active_pub = self.node.create_publisher(
            Bool, '/nav_operation_active', qos
        )
        self.nav_active_pub.publish(Bool(data=False))

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

        _publish_robot_pose(self.node, self.pose_publisher, 0.0, 0.0)
        start_response = _call_trigger(self.node, '/chennal_record_start')
        assert start_response.success

        _publish_robot_pose(self.node, self.pose_publisher, 0.6, 0.0)
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
        _publish_robot_pose(self.node, self.pose_publisher, 1.0, 0.0)
        start_response = _call_trigger(self.node, '/channel_record_start')
        assert start_response.success

        _publish_robot_pose(self.node, self.pose_publisher, 1.8, 0.0)
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

    def test_zone_recording_rejects_empty_end_and_competing_starts(self):
        """Only one recorder may run and an empty zone must never be saved."""
        no_active_response = _call_trigger(self.node, '/record_zone_end')
        assert not no_active_response.success

        start_response = _call_trigger(self.node, '/record_zone_start')
        assert start_response.success

        duplicate_response = _call_trigger(self.node, '/record_zone_start')
        assert not duplicate_response.success
        risk_response = _call_trigger(self.node, '/risk_zone_start')
        assert not risk_response.success
        channel_response = _call_trigger(self.node, '/channel_record_start')
        assert not channel_response.success

        insufficient_response = _call_trigger(self.node, '/record_zone_end')
        assert not insufficient_response.success

        cancel_response = _call_trigger(self.node, '/record_cancel')
        assert cancel_response.success

        # The rejected end must not have added an empty marker to the list.
        zone_list_client = self.node.create_client(
            GetZoneList, '/get_record_zone_list'
        )
        assert zone_list_client.wait_for_service(timeout_sec=5.0)
        zone_info = _spin_until_future(
            self.node,
            zone_list_client.call_async(GetZoneList.Request()),
        )
        assert zone_info.success
        assert zone_info.zone_list.markers == []
