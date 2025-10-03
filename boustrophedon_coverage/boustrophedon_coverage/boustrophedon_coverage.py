#!/usr/bin/env python3
from typing import List, Tuple
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point32, Pose
from geometry_msgs.msg import PolygonStamped
import matplotlib.pyplot as plt

from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

from rclpy.callback_groups import ReentrantCallbackGroup

from std_srvs.srv import Trigger,SetBool
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.action import ActionClient
from nav2_action_interfaces.action import Waypoint

#custom action client for nav2
from .nav_action_client import NavActionClient  




class CoveragePlanner(Node):
    def __init__(self):
        super().__init__('boustrophedon_coverage')        
        # 參數
        self.get_logger().info("boustrophedon_coverage 初始化")
        self.declare_parameter('strip_width_m', 0.2)          # 割草機有效割幅
        self.declare_parameter('waypoint_spacing_m', 0.1)     # 路徑點間距
        self.declare_parameter('free_threshold', 25)           # 佔據格 <= 此值視為可行
        self.declare_parameter('unknown_as_obstacle', True)    # 未知(-1)是否當作障礙
        self.declare_parameter('inflate_radius_m', 0.08)       # 安全膨脹半徑(機身+裕度)

        #qos setting
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        #訂閱區
        # self.map_sub = self.create_subscription(
        #     OccupancyGrid, '/map', self.getCellMatAndFreeSpace, qos
        # )
   

        self.free_space_pub = self.create_publisher(OccupancyGrid, '/free_space', 1)
        self.free_space_inflated_pub = self.create_publisher(OccupancyGrid, '/free_space_inflated', 1)
        
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos)
        self.polygon_point_sub = self.create_subscription(Marker, '/recorded_path_polygon', self.polygon_point_callback, qos)
        self.polygon_points = None

        # 建立服務 service
        self.create_service(Trigger, '/generate_polygon_mask', self.handle_generate_polygon_mask)
        self.create_service(Trigger, '/waypoint_pub', self.waypoint_pub_srv)
        self.create_service(Trigger, '/cencel_nav2', self.cancel_nav2_srv)
        self.create_service(Trigger, '/check_nav_status', self.check_nav_status_srv)

        #建立服務 client
        self.waypoint_active_client = self.create_client(SetBool, '/record_path_status')

        #建立action client
        self.nav_action_client = NavActionClient()

        #發布區
        self.path_pub = self.create_publisher(Path, '/coverage_path', 1)

        self.waypoint_active = False
        self.latest_map = None

        self.nav = BasicNavigator()
        self.coverage_path = Path()
        self.coverage_split_points = []
        # self.map_split_line = self.create_publisher(Marker, '/coverage_split_lines', 1)
        # self.free_pub = self.create_publisher(OccupancyGrid, '/free_space', 1)
    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg
        self.get_logger().info("收到 map")

    def polygon_point_callback(self, msg: Marker):
        self.polygon_points = msg.points
        self.get_logger().info("收到 polygon points")

    def handle_generate_polygon_mask(self, request, response):
        if self.polygon_points is None:
            response.success = False
            response.message = "尚未收到 polygon points"
            return response
    
        self.get_logger().info("收到 polygon points")
        
        # 根據 polygon 的邊界計算地圖大小
        min_x = min(pt.x for pt in self.polygon_points)
        max_x = max(pt.x for pt in self.polygon_points)
        min_y = min(pt.y for pt in self.polygon_points)
        max_y = max(pt.y for pt in self.polygon_points)
        
        # 設定地圖參數
        resolution = 0.05  # 5cm 解析度
        margin = 1.0  # 邊界裕度
        
        # 計算地圖尺寸
        map_width = max_x - min_x + 2 * margin
        map_height = max_y - min_y + 2 * margin
        W = int(map_width / resolution)
        H = int(map_height / resolution)
        
        # 設定地圖原點（左下角）
        ox = min_x - margin
        oy = min_y - margin
        
        # 將 polygon 的點轉換為像素座標
        poly_px = []
        for pt in self.polygon_points:
            x = int((pt.x - ox) / resolution)
            y = int((pt.y - oy) / resolution)
            poly_px.append([x, y])
        poly_px = np.array([poly_px], dtype=np.int32)

        # 建立遮罩
        mask = np.zeros((H, W), dtype=np.uint8)
        cv2.fillPoly(mask, [poly_px], 1)

        # 建立地圖數據（polygon 內為自由空間，外為障礙）
        occ_masked = np.where(mask == 1, 0, 100)  # 遮罩內為自由空間(0)，外為障礙(100)

        # 發布遮罩後的地圖
        masked_map = OccupancyGrid()
        masked_map.header.stamp = self.get_clock().now().to_msg()
        masked_map.header.frame_id = 'map'
        masked_map.info.resolution = resolution
        masked_map.info.width = W
        masked_map.info.height = H
        masked_map.info.origin.position.x = ox
        masked_map.info.origin.position.y = oy
        masked_map.info.origin.position.z = 0.0
        masked_map.info.origin.orientation.x = 0.0
        masked_map.info.origin.orientation.y = 0.0
        masked_map.info.origin.orientation.z = 0.0
        masked_map.info.origin.orientation.w = 1.0
        masked_map.data = occ_masked.flatten().tolist()
        
        self.free_space_pub.publish(masked_map)
        self.on_map(masked_map)

        response.success = True
        response.message = "已根據 polygon 生成地圖遮罩"
        return response

    def on_map(self, map_msg: OccupancyGrid):
        """
        根據地圖生成牛耕式覆蓋路徑，並修正路徑點的朝向與格式
        """
        import math
        import numpy as np

        def euler_to_quaternion(roll, pitch, yaw):
            """
            將歐拉角轉換為四元數（x, y, z, w）
            """
            qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
            qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
            qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
            qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
            return (qx, qy, qz, qw)

        def cal_two_point_orientation(x1, y1, x2, y2):
            """
            計算兩點之間的朝向（歐拉角yaw），並返回對應的四元數
            """
            dx = x2 - x1
            dy = y2 - y1
            yaw = math.atan2(dy, dx)
            return euler_to_quaternion(0, 0, yaw)

        info = map_msg.info
        H, W = info.height, info.width
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y

        occ = np.asarray(map_msg.data, dtype=np.int16).reshape(H, W)
        free_th = int(self.get_parameter('free_threshold').value)
        unknown_as_obstacle = bool(self.get_parameter('unknown_as_obstacle').value)

        # 建立可行遮罩
        free_mask = (occ >= 0) & (occ <= free_th)
        # 障礙膨脹
        inflate_r_m = float(self.get_parameter('inflate_radius_m').value)
        r_cells = max(0, int(math.ceil(inflate_r_m / res)))
        if r_cells > 0:
            occ_mask = (~free_mask).astype(np.uint8)
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r_cells + 1, 2 * r_cells + 1))
            occ_mask = cv2.dilate(occ_mask, k)
            free_mask = (occ_mask == 0)
        # 發布膨脹後的地圖
        inflated_map = OccupancyGrid()
        inflated_map.header = map_msg.header
        inflated_map.header.frame_id = 'map'
        inflated_map.info = map_msg.info
        inflated_data = np.where(free_mask, 0, 100).astype(np.int8)
        inflated_map.data = inflated_data.flatten().tolist()
        self.free_space_inflated_pub.publish(inflated_map)

        # 進一步縮減可行區，確保路徑點不會貼近膨脹邊緣
        if r_cells > 0:
            shrink_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r_cells + 1, 2 * r_cells + 1))
            safe_mask = cv2.erode(free_mask.astype(np.uint8), shrink_k)
            safe_mask = (safe_mask == 1)
        else:
            safe_mask = free_mask

        # 條帶設定：以 X 方向切直條(沿 Y 掃描)
        strip_w_m = float(self.get_parameter('strip_width_m').value)
        strip_cols = max(1, int(round(strip_w_m / res)))
        midcols = list(range(strip_cols // 2, W, strip_cols))

        # 產生往返路徑
        spacing = float(self.get_parameter('waypoint_spacing_m').value)
        points = []
        reverse = False

        for mc in midcols:
            # 沿條帶中心列，找連續可行段
            segments = []
            start = None
            for i in range(H):
                ok = bool(safe_mask[i, mc])
                is_last = (i == H - 1)
                if ok and start is None:
                    start = i
                if (not ok or is_last) and start is not None:
                    end = i if (not ok) else i
                    segments.append((start, end))
                    start = None

            # 交替方向，形成牛耕(往返)
            segs = segments[::-1] if reverse else segments
            for (s, e) in segs:
                y0 = oy + (s + 0.5) * res
                y1 = oy + (e + 0.5) * res
                x  = ox + (mc + 0.5) * res
                # densify
                if y1 >= y0:
                    ys = list(np.arange(y0, y1, max(res, spacing))) + [y1]
                else:
                    ys = list(np.arange(y0, y1, -max(res, spacing))) + [y1]
                ys = ys[::-1] if reverse else ys
                for y in ys:
                    points.append((x, y))

                p = Pose()
                p.position.x = x
                p.position.y = y0
                p.position.z = 0.0
                p.orientation.w = 1.0
                p.orientation.z = 0.0
                self.coverage_split_points.append(p)

                p = Pose()
                p.position.x = x
                p.position.y = y1
                p.position.z = 0.0
                p.orientation.w = 1.0
                p.orientation.z = 0.0
                self.coverage_split_points.append(p)


            reverse = not reverse
              
        # 修正路徑點，確保每個點的朝向正確
        self.coverage_path = Path()
        self.coverage_path.header = map_msg.header
        self.coverage_path.header.frame_id = 'map'

        for idx, (x, y) in enumerate(points):
            goal_pose = PoseStamped()
            goal_pose.header = self.coverage_path.header
            goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
            goal_pose.pose.position.x = float(x)
            goal_pose.pose.position.y = float(y)
            goal_pose.pose.position.z = 0.0

            # 計算朝向
            if idx < len(points) - 1:
                x2, y2 = points[idx + 1]
                qx, qy, qz, qw = cal_two_point_orientation(x, y, x2, y2)
            elif idx > 0:
                x2, y2 = points[idx - 1]
                qx, qy, qz, qw = cal_two_point_orientation(x2, y2, x, y)
            else:
                # 只有一個點，朝向正前
                qx, qy, qz, qw = euler_to_quaternion(0, 0, 0)

            goal_pose.pose.orientation.x = qx
            goal_pose.pose.orientation.y = qy
            goal_pose.pose.orientation.z = qz
            goal_pose.pose.orientation.w = qw

            self.coverage_path.poses.append(goal_pose)

        self.path_pub.publish(self.coverage_path)

    #發佈coverage path to action server 
    def waypoint_pub_srv(self, req, res):        # 停止使用 self.path_pub
        set_bool_req = SetBool.Request()
        set_bool_req.data = True
        self.waypoint_active_client.call_async(set_bool_req)
        self.nav_action_client.send_goal_split_path(self.coverage_path,self.coverage_split_points)
        res.success = True
        res.message = 'Waypoint published'
        return res

    def cancel_nav2_srv(self, req, res):
        # self.waypoint_active = False
        self.nav.cancelTask()
        res.success = True
        res.message = 'Nav2 canceled'
        return res

    def check_nav_status_srv(self, req, res):
        res.success = self.nav.isTaskComplete()
        if res.success:
            res.message = 'Navigation completed'
        else:
            feedback = self.nav.getFeedback()
            res.message = f'Navigation in progress: {feedback}'
        return res

    def record_path_status_srv(self, req, res):
        self.waypoint_active = req.data  # 直接赋值
        res.success = True
        res.message = f'Path recording status: {self.waypoint_active}'
        return res
class val():
    def __init__(self):
        self.x = None   
        self.y = None
        self.qx = None
        self.qy = None 
        self.qz = None
        self.qw = None
    def is_empty(self):
        return self.x == None and self.y == None and self.qx == None and self.qy == None and self.qz == None and self.qw == None
def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()