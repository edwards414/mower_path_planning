#!/usr/bin/env python3

import math

from geometry_msgs.msg import Point, PoseStamped

import rclpy
from rclpy.node import Node

from std_msgs.msg import ColorRGBA

from visualization_msgs.msg import Marker, MarkerArray


def _quaternion_from_yaw(yaw: float):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class TempDockPosePublisher(Node):
    """Temporary fixed dock pose publisher for docking bringup tests."""

    def __init__(self):
        super().__init__('temp_dock_pose_publisher')

        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('detected_pose_topic', '/detected_dock_pose')
        self.declare_parameter('marker_topic', '/dock_markers')
        self.declare_parameter('dock_pose_x', 0.0)
        self.declare_parameter('dock_pose_y', 0.0)
        self.declare_parameter('dock_pose_z', 0.0)
        self.declare_parameter('dock_pose_yaw', 0.0)
        self.declare_parameter('staging_x_offset', -0.8)
        self.declare_parameter('staging_yaw_offset', 3.14159)

        self.pose_pub = self.create_publisher(
            PoseStamped,
            self.get_parameter('detected_pose_topic').value,
            10,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.get_parameter('marker_topic').value,
            10,
        )

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.timer_cb)
        self.get_logger().warn(
            'Publishing a temporary fixed /detected_dock_pose. '
            'Use only for docking bringup tests, not production docking.'
        )

    def timer_cb(self):
        pose = self._dock_pose_msg()
        self.pose_pub.publish(pose)
        self.marker_pub.publish(self._marker_array(pose))

    def _dock_pose_msg(self) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.get_parameter('frame_id').value
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(self.get_parameter('dock_pose_x').value)
        pose.pose.position.y = float(self.get_parameter('dock_pose_y').value)
        pose.pose.position.z = float(self.get_parameter('dock_pose_z').value)
        yaw = float(self.get_parameter('dock_pose_yaw').value)
        qx, qy, qz, qw = _quaternion_from_yaw(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _marker_array(self, pose: PoseStamped) -> MarkerArray:
        dock_yaw = float(self.get_parameter('dock_pose_yaw').value)
        staging_x_offset = float(self.get_parameter('staging_x_offset').value)
        staging_yaw_offset = float(self.get_parameter('staging_yaw_offset').value)

        dock_marker = self._arrow_marker(
            marker_id=1,
            ns='dock_pose',
            x=pose.pose.position.x,
            y=pose.pose.position.y,
            yaw=dock_yaw,
            rgba=(0.1, 0.7, 1.0, 0.9),
        )

        sx = pose.pose.position.x + staging_x_offset * math.cos(dock_yaw)
        sy = pose.pose.position.y + staging_x_offset * math.sin(dock_yaw)
        staging_marker = self._arrow_marker(
            marker_id=2,
            ns='dock_staging_pose',
            x=sx,
            y=sy,
            yaw=dock_yaw + staging_yaw_offset,
            rgba=(1.0, 0.8, 0.1, 0.9),
        )

        line_marker = self._line_marker(
            marker_id=3,
            x1=pose.pose.position.x,
            y1=pose.pose.position.y,
            x2=sx,
            y2=sy,
        )

        marker_array = MarkerArray()
        marker_array.markers.extend([dock_marker, staging_marker, line_marker])
        return marker_array

    def _arrow_marker(self, marker_id: int, ns: str, x: float, y: float,
                      yaw: float, rgba) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.get_parameter('frame_id').value
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.08
        qx, qy, qz, qw = _quaternion_from_yaw(yaw)
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = 0.55
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color = self._color(*rgba)
        return marker

    def _line_marker(self, marker_id: int, x1: float, y1: float,
                     x2: float, y2: float) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.get_parameter('frame_id').value
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'dock_to_staging'
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color = self._color(0.9, 0.9, 0.9, 0.7)
        p1 = Point()
        p1.x = x1
        p1.y = y1
        p1.z = 0.05
        p2 = Point()
        p2.x = x2
        p2.y = y2
        p2.z = 0.05
        marker.points.extend([p1, p2])
        return marker

    @staticmethod
    def _color(r: float, g: float, b: float, a: float) -> ColorRGBA:
        color = ColorRGBA()
        color.r = r
        color.g = g
        color.b = b
        color.a = a
        return color


def main(args=None):
    rclpy.init(args=args)
    node = TempDockPosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
