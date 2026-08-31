#!/usr/bin/env python3

"""Navigation action client for waypoint following."""

import uuid

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

from mower_interface.action import Waypoint
from mower_interface.srv import ConfirmNavigationDispatch

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
        self._action_client = ActionClient(self, Waypoint, 'nav_action')
        self._action_client_split_path = ActionClient(
            self, Waypoint, 'nav_action_follow_path'
        )
        self._confirm_dispatch_client = self.create_client(
            ConfirmNavigationDispatch,
            '/confirm_navigation_dispatch',
        )
        self.nav = BasicNavigator() if is_nav else None
        if self.nav is not None:
            self.nav.set_parameters([
                Parameter(
                    'use_sim_time',
                    Parameter.Type.BOOL,
                    bool(self.get_parameter('use_sim_time').value),
                )
            ])

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
        goal_msg.dispatch_id = uuid.uuid4().hex

        self._action_client_split_path.wait_for_server()
        self._send_goal_future = (
            self._action_client_split_path.send_goal_async(goal_msg)
        )
        self._send_goal_future.add_done_callback(
            lambda future, dispatch_id=goal_msg.dispatch_id: (
                self.goal_response_callback(future, dispatch_id)
            )
        )

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
        goal_msg.dispatch_id = uuid.uuid4().hex
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(
            lambda future, dispatch_id=goal_msg.dispatch_id: (
                self.goal_response_callback(future, dispatch_id)
            )
        )

    def goal_response_callback(self, future, dispatch_id):
        """Handle the goal response from the action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by server')
            return

        self.get_logger().info('Goal accepted :)')
        if not self._confirm_dispatch_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                'Dispatch confirmation service unavailable; canceling goal'
            )
            goal_handle.cancel_goal_async()
            return
        request = ConfirmNavigationDispatch.Request()
        request.dispatch_id = dispatch_id
        confirm_future = self._confirm_dispatch_client.call_async(request)
        confirm_future.add_done_callback(
            lambda done, handle=goal_handle: (
                self._dispatch_confirmation_callback(done, handle)
            )
        )

    def _dispatch_confirmation_callback(self, future, goal_handle):
        """Only begin result tracking after correlated dispatch confirmation."""
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f'Dispatch confirmation failed: {exc}; canceling goal'
            )
            goal_handle.cancel_goal_async()
            return
        if response is None or not response.success:
            self.get_logger().error(
                'Dispatch confirmation rejected; canceling goal'
            )
            goal_handle.cancel_goal_async()
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle the result from the action server."""
        result = future.result().result
        self.get_logger().info('Navigation action completed successfully')
        self.get_logger().info(f'Result: {result}')

    def _create_path(self):
        """Create a test path for navigation."""
        path = Path()
        width = 4.0
        points = [
            (0.0, 0.0), (width, 0.0), (width, width),
            (0.0, width), (0.0, 0.0)
        ]
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
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
