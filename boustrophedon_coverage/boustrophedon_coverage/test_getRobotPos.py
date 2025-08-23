#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose, do_transform_point
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
import time

class CleanRobotCoveragePathPlanning(Node):
    def __init__(self, executor=None):
        super().__init__('clean_robot_coverage_path_planning')
        self.executor = executor  # 保存 executor 引用
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.robotPosMap = None 

    def getRobotPos(self):
        print("getRobotPos")
        # 初始化变量
        self.robot_x = None
        self.robot_y = None
        self.robot_theta = None
        self.transform_completed = False
        def odom_callback(msg):
            self.get_logger().info(f"odom_callback: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}")
            odom_pos = PoseStamped()
            odom_pos.header = msg.header
            odom_pos.header.stamp = rclpy.time.Time()
            odom_pos.pose.position.x = msg.pose.pose.position.x
            odom_pos.pose.position.y = msg.pose.pose.position.y
            odom_pos.pose.position.z = msg.pose.pose.position.z
            # 銷毀訂閱
            self.destroy_subscription(odom_sub)
            while not self.tf_buffer.can_transform('map', 'odom', rclpy.time.Time()):
                rclpy.spin_once(self, timeout_sec=0.1)
                # print("wait for tf transform")
                self.get_logger().debug("wait for tf transform")
            self.executor.create_task(self.transform_to_map, odom_pos)
        odom_sub = self.create_subscription(Odometry, '/odom', odom_callback, 10)
        while not self.transform_completed:
                rclpy.spin_once(self, timeout_sec=0.1)
        return (self.robot_x, self.robot_y, self.robot_theta)
    def transform_to_map(self, odom_pos):
    #     """獨立的 TF 轉換方法"""
        while True:
            try: 
                point_target = self.tf_buffer.transform(odom_pos, 'map', timeout=Duration(seconds=0.1))
                if point_target is not None:
                    self.robot_x = point_target.pose.position.x
                    self.robot_y = point_target.pose.position.y
                    self.robot_theta = point_target.pose.orientation.z
                    break
            except Exception as e:
                self.get_logger().warn(f"TF transform unavailable: {e}")
        self.transform_completed = True

        
def main(args=None):
    print("start main")
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=2)
    
    # 傳入 executor
    Clean_bot_node = CleanRobotCoveragePathPlanning(executor)
    executor.add_node(Clean_bot_node)
    
    # 直接呼叫，不用 create_task（因為內部會用 executor）
    print(Clean_bot_node.getRobotPos())
    
    try:
        executor.spin()
    finally:
        print("finish get robot pos")
        executor.shutdown()
        Clean_bot_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
