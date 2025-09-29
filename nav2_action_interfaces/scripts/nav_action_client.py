#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_action_interfaces.action import Waypoint
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

class NavActionClient(Node):
    def __init__(self):
        super().__init__('nav_action_client')
        self._action_client = ActionClient(self, Waypoint, 'nav_action')
        self.nav = BasicNavigator()
    def send_goal(self, path):
        goal_msg = Waypoint.Goal()
        goal_msg.path = path

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)

    def create_path(self):
        path = Path()
        width = 4.0
        points = [
        (0.0, 0.0),
        (width, 0.0),
        (width, width),
        (0.0, width),
        (0.0, 0.0)
        ]
        # print(points)
        for (x, y) in points:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.nav.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            pose.pose.orientation.z = 0.0
            path.poses.append(pose)
        return path

def main(args=None):
    rclpy.init(args=args)

    action_client = NavActionClient()

    path = action_client.create_path()
    future = action_client.send_goal(path)

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()