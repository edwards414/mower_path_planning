#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav2_action_interfaces.action import Waypoint
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator

class NavActionClient(Node):
    def __init__(self, is_nav = False):
        super().__init__('nav_action_client')
        self.set_parameters([
            Parameter('use_sim_time',Parameter.Type.BOOL,True)
        ])
        self._action_client = ActionClient(self, Waypoint, 'nav_action')
        self._action_client_split_path = ActionClient(self, Waypoint, 'nav_action_follow_path')
        self.nav = BasicNavigator() if is_nav else None

    def send_goal_split_path(self, path :Path,coverage_split_points :[Pose]):
        self.get_logger().info('send_goal_split_path')
        goal_msg = Waypoint.Goal()
        goal_msg.path = path
        goal_msg.coverage_split_points=coverage_split_points

        self._action_client_split_path.wait_for_server()
        self._send_goal_future = self._action_client_split_path.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def send_goal(self, path :Path):
        goal_msg = Waypoint.Goal()
        goal_msg.path = path
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by server')
            return 

        self.get_logger().info('Goal accepted :)')
        # self._get_result_future = goal_handle.get_result_async()
        # self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def _create_path(self):
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

    action_client = NavActionClient(is_nav=True)
    path = action_client._create_path()
    
    action_client.send_goal(path)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()