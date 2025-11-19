#!/usr/bin/env python3

"""Navigation action client for waypoint following."""

# Copyright 2024 fxrbindi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import Pose, PoseStamped

from nav2_action_interfaces.action import Waypoint

from nav2_simple_commander.robot_navigator import BasicNavigator

from nav_msgs.msg import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter


class NavActionClient(Node):
    """Navigation action client for sending waypoint goals."""

    def __init__(self, is_nav=False):
        """
        Initialize the navigation action client.

        Args:
        ----
        is_nav : bool
            是否使用nav2的action server

        """
        super().__init__('nav_action_client')
        self.get_logger().info('nav_action_client init')
        self.set_parameters(
            [Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self._action_client = ActionClient(self, Waypoint, 'nav_action')
        self._action_client_split_path = ActionClient(
            self, Waypoint, 'nav_action_follow_path'
        )
        self.nav = BasicNavigator() if is_nav else None

    def send_goal_split_path(
            self, path: Path, coverage_split_points: list[Pose]
    ):
        """
        Send a goal to the action server to execute a split path.

        Args:
            path (Path): The path to execute.
            coverage_split_points (list[Pose]): The split points.
        """
        self.get_logger().info('send_goal_split_path')
        goal_msg = Waypoint.Goal()
        goal_msg.path = path
        goal_msg.coverage_split_points = coverage_split_points

        self._action_client_split_path.wait_for_server()
        self._send_goal_future = (
            self._action_client_split_path.send_goal_async(goal_msg)
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def send_goal(self, path: Path):
        """
        Send a goal to the action server.

        Args:
        ----
        path : Path
            The path to execute.

        """
        goal_msg = Waypoint.Goal()
        goal_msg.path = path
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Handle the goal response from the action server.

        Args:
        ----
        future : Future
            The future object containing the goal handle.

        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by server')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Handle the result from the action server.

        Args:
        ----
        future : Future
            The future object containing the result.

        """
        result = future.result().result
        self.get_logger().info('Navigation action completed successfully')
        self.get_logger().info(f'Result: {result}')

    def _create_path(self):
        """
        Create a test path for navigation.

        Returns
        -------
        Path
            A square path for testing.

        """
        path = Path()
        width = 4.0
        points = [
            (0.0, 0.0), (width, 0.0), (width, width),
            (0.0, width), (0.0, 0.0)
        ]
        # print(points)
        for x, y in points:
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
    """Run the navigation action client node."""
    rclpy.init(args=args)

    action_client = NavActionClient(is_nav=True)
    # path = action_client._create_path()

    # action_client.send_goal(path)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
