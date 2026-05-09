#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import Buffer, TransformException, TransformListener


class AprilTagDockPosePublisher(Node):
    """Publish Nav2 docking's detected_dock_pose from an AprilTag TF."""

    def __init__(self):
        super().__init__('apriltag_dock_pose_publisher')
        self.declare_parameter('camera_frame', 'back_camera_link')
        self.declare_parameter('tag_frame', 'home_dock_tag')
        self.declare_parameter('detected_pose_topic', '/detected_dock_pose')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('transform_timeout_sec', 0.1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(
            PoseStamped,
            self.get_parameter('detected_pose_topic').value,
            10,
        )

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.timer_cb)
        self.warned_missing_tf = False
        self.get_logger().info(
            'AprilTagDockPosePublisher publishing '
            f'{self.get_parameter("detected_pose_topic").value} from '
            f'{self.get_parameter("camera_frame").value} -> '
            f'{self.get_parameter("tag_frame").value}'
        )

    def timer_cb(self):
        camera_frame = str(self.get_parameter('camera_frame').value)
        tag_frame = str(self.get_parameter('tag_frame').value)
        timeout = Duration(
            seconds=float(self.get_parameter('transform_timeout_sec').value)
        )

        try:
            transform = self.tf_buffer.lookup_transform(
                camera_frame,
                tag_frame,
                Time(),
                timeout=timeout,
            )
        except TransformException as exc:
            if not self.warned_missing_tf:
                self.get_logger().warn(
                    'Waiting for AprilTag TF '
                    f'{camera_frame} -> {tag_frame}: {exc}'
                )
                self.warned_missing_tf = True
            return

        self.warned_missing_tf = False
        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        self.pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDockPosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
