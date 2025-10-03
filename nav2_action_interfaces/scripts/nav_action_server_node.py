#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Path
from nav2_action_interfaces.action import Waypoint

class NavActionServer(Node):
    def __init__(self):
        super().__init__('nav_action_server')
        self.get_logger().info('NavActionServer initialized')

        self.set_parameters([
            Parameter('use_sim_time',Parameter.Type.BOOL,True)
        ])
        self.navigator = BasicNavigator()
        self.action_server = ActionServer(
            self,
            Waypoint,
            'nav_action',
            self.execute_callback
        )

        self.action_server_follow_path = ActionServer(
            self,
            Waypoint,
            'nav_action_follow_path',
            self.test2_execute_callback
        )
        self.split_path_pub = self.create_publisher(Path, '/split_path', 1)
        self.coverage_split_points_pub = self.create_publisher(Marker, '/coverage_split_points', 1)
        self.coverage_split_points = []
        # self.action_server_follow_pat
    def test_execute_callback(self, goal_handle):
        path = goal_handle.request.path
        self.navigator.followPath(path)
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            self.get_logger().info(f"Feedback: {feedback.distance_to_goal}")
        goal_handle.succeed()
        result = Waypoint.Result()
        result.success = True
        return result


    #發佈單條路徑
    def test2_execute_callback(self, goal_handle): 
        import time
        self.get_logger().info('執行目標')
        path = goal_handle.request.path
        coverage_split_points = goal_handle.request.coverage_split_points

        self.get_logger().info(f"path 長度: {len(path.poses)}")
        self.get_logger().info(f"coverage_split_points 長度: {len(coverage_split_points)}")
        for pose in coverage_split_points:
            x, y = pose.position.x, pose.position.y
            self.coverage_split_points.append([x,y])
        
        # self.coverage_split_points = coverage_split_points
        # self.publish_split_points_marker()

        self.navigator.goToPose(path.poses[0])

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f"反饋: {feedback}")
        
        current_split_pose_index = 0
        split_path = Path()
        # 設置 header 信息
        split_path.header.frame_id = 'map'  # 或者使用 path.header.frame_id
        split_path.header.stamp = self.navigator.get_clock().now().to_msg()

        for pose in path.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y

            # # 判斷是否到達切割點
            if [x,y] in self.coverage_split_points[1:]:
                split_path.poses.append(pose)
                self.split_path_pub.publish(split_path)
                self.navigator.followPath(split_path)
                while not self.navigator.isTaskComplete():
                    feedback = self.navigator.getFeedback()
                    if feedback:
                        self.get_logger().info(f"反饋: {feedback}")
                current_split_pose_index += 1
                # 重新初始化新的 split_path
                split_path = Path()
                split_path.header.frame_id = 'map'  # 重新設置 header
                split_path.header.stamp = self.navigator.get_clock().now().to_msg()
                split_path.poses.append(pose)
                self.get_logger().info(f"發佈第{current_split_pose_index}段路徑，共{len(split_path.poses)}點")
            else:
                # self.get_logger().info(f"目前共蒐集{i}點")
                split_path.poses.append(pose)

        self.get_logger().info(f"self.coverage_split_points: {self.coverage_split_points}")
        self.get_logger().info(f"current_split_pose_index: {current_split_pose_index}")
        
        self.get_logger().info(f"pose type: {type(pose)}")
        # 處理最後一段（如果有剩餘點）
        # if len(split_path.poses) > 1:
        #     self.get_logger().info(f"發佈最後一段路徑，共{len(split_path.poses)}點")
            # self.navigator.followPath(split_path)
            # while not self.navigator.isTaskComplete():
            #     feedback = self.navigator.getFeedback()
            #     self.get_logger().info(f"反饋: {feedback.distance_to_goal}")
        goal_handle.succeed()
        result = Waypoint.Result()
        result.success = True
        return result

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

    def publish_split_points_marker(self):
        """
        在 RViz 上可视化覆盖路径的分割点
        """
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'coverage_split_points'
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        
        # 设置球体大小
        marker.scale.x = 0.15  # 球体直径
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        
        # 设置颜色 (红色，半透明)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        # 添加所有分割点
        for pose in self.coverage_split_points:
            point = Point()
            point.x = pose.position.x
            point.y = pose.position.y
            point.z = 0.1  # 稍微抬高一点，避免与地面重叠
            marker.points.append(point)
            
            # 为每个点添加颜色（可选，如果不添加则使用 marker.color）
            color = ColorRGBA()
            color.r = 1.0
            color.g = 0.0
            color.b = 0.0
            color.a = 0.8
            marker.colors.append(color)
        
        self.get_logger().info(f'发布 {len(self.coverage_split_points)} 个分割点到 RViz')
        self.coverage_split_points_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    nav_action_server = NavActionServer()
    rclpy.spin(nav_action_server)
    nav_action_server.destroy_node()
    rclpy.shutdown()
    nav_action_server.navigator.lifecycleShutdown()

if __name__ == '__main__':
    main()
    main()