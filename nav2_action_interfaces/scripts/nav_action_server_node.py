#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Path
from nav2_action_interfaces.action import Waypoint

class NavActionServer(Node):
    def __init__(self):
        super().__init__('nav_action_server')
        self.get_logger().info('NavActionServer initialized')
        self.navigator = BasicNavigator()
        self.action_server = ActionServer(
            self,
            Waypoint,
            'nav_action',
            self.execute_callback
        )
        
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal')
        path = goal_handle.request.path
        self.get_logger().info(f"path length: {len(path.poses)}")
       
        for pose in path.poses:
            print(f"pose1: {pose.pose.position.x}, {pose.pose.position.y}")
            print("--------------------------------")
        for i in range(len(path.poses)-1):
            pose1 = path.poses[i]
            pose2 = path.poses[i+1]
            # 计算pose1到pose2的欧拉角yaw，并将其转换为四元数，更新pose1和pose2的orientation
            import math

            def euler_to_quaternion(roll, pitch, yaw):
                """
                将欧拉角转换为四元数（x, y, z, w）
                """
                qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
                qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
                qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
                qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
                return (qx, qy, qz, qw)

            dx = pose2.pose.position.x - pose1.pose.position.x
            dy = pose2.pose.position.y - pose1.pose.position.y
            yaw = math.atan2(dy, dx)
            q = euler_to_quaternion(0, 0, yaw)

            pose1.pose.orientation.x = q[0]
            pose1.pose.orientation.y = q[1]
            pose1.pose.orientation.z = q[2]
            pose1.pose.orientation.w = q[3]

            pose2.pose.orientation.x = q[0]
            pose2.pose.orientation.y = q[1]
            pose2.pose.orientation.z = q[2]
            pose2.pose.orientation.w = q[3]
            print(f"pose1: {pose1.pose.position.x}, {pose1.pose.position.y}, pose2: {pose2.pose.position.x}, {pose2.pose.position.y}")
            print("--------------------------------")
            nav_path = self.navigator.getPath(pose1, pose2)
            smoothed_path = self.navigator.smoothPath(nav_path)
            self.navigator.followPath(smoothed_path)
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                self.get_logger().info(f"Feedback: {feedback.distance_to_goal}")
        #     #     # pass
        goal_handle.succeed()
        result = Waypoint.Result()
        result.success = True
        return result
    
    def destroy_node(self):
        self.action_server.destroy()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    nav_action_server = NavActionServer()
    rclpy.spin(nav_action_server)
    nav_action_server.destroy_node()
    rclpy.shutdown()
    nav_action_server.navigator.lifecycleShutdown()

if __name__ == '__main__':
    main()