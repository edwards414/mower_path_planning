#!/usr/bin/env python3
from typing import List, Tuple
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

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
from boustrophedon_coverage_interfaces.srv import GetZoneList

class CoveragePlanner(Node):
    def __init__(self):
        super().__init__('boustrophedon_coverage')

        self.set_parameters([
                Parameter('use_sim_time',Parameter.Type.BOOL,True)
        ])
        
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
   

        
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos)
        self.polygon_point_sub = self.create_subscription(Marker, '/recorded_path_polygon', self.polygon_point_callback, qos)
        self.polygon_points = None

        # 建立服務 service
        self.create_service(Trigger, '/generate_polygon_mask', self.handle_generate_polygon_mask)
        self.create_service(Trigger, '/waypoint_pub', self.waypoint_pub_srv)
        self.create_service(Trigger, '/cencel_nav2', self.cancel_nav2_srv)
        self.create_service(Trigger, '/check_nav_status', self.check_nav_status_srv)
        self.create_service(Trigger, '/create_risk_map', self.create_risk_map_srv)

        #建立服務 client
        self.waypoint_active_client = self.create_client(SetBool, '/record_path_status')
        self.get_risk_zone_list_client = self.create_client(GetZoneList, '/get_risk_zone_list')

        #建立action client
        self.nav_action_client = NavActionClient()

        #發布區
        self.free_space_pub = self.create_publisher(OccupancyGrid, '/free_space', 1)
        self.free_space_inflated_pub = self.create_publisher(OccupancyGrid, '/free_space_inflated', 1)
        self.path_pub = self.create_publisher(Path, '/coverage_path', 1)
        self.risk_map_pub = self.create_publisher(OccupancyGrid, '/risk_map', 1)

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
    
    # 生成牛耕式覆蓋路徑 輸入 zone map
    def generate_coverage_path(self, zone_map):

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

        info = zone_map.info
        H, W = info.height, info.width
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y

        occ = np.asarray(zone_map.data, dtype=np.int16).reshape(H, W)
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
        inflated_map.header = zone_map.header
        inflated_map.header.frame_id = 'map'
        inflated_map.info = zone_map.info
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
        self.coverage_path.header = zone_map.header
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

    def create_risk_map_srv(self, req, res):
        """创建风险地图服务"""
        try:
            # 检查是否有地图数据
            if self.latest_map is None:
                res.success = False
                res.message = "没有可用的地图数据"
                return res
            
            # 等待风险区域列表服务可用
            if not self.get_risk_zone_list_client.wait_for_service(timeout_sec=5.0):
                res.success = False
                res.message = "风险区域列表服务不可用"
                return res
            
            # 调用获取风险区域列表服务
            risk_zone_req = GetZoneList.Request()
            future = self.get_risk_zone_list_client.call_async(risk_zone_req)
            
            # 等待服务响应
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if not future.done():
                res.success = False
                res.message = "获取风险区域列表超时"
                return res
            
            risk_zone_response = future.result()
            
            if not risk_zone_response.success:
                res.success = False
                res.message = f"获取风险区域列表失败: {risk_zone_response.message}"
                return res
            
            # 生成风险地图
            risk_map = self._generate_risk_map(self.latest_map, risk_zone_response.zone_list)
            
            if risk_map is not None:
                self.risk_map_pub.publish(risk_map)
                res.success = True
                res.message = f"成功创建风险地图，包含 {len(risk_zone_response.zone_list.markers)} 个风险区域"
            else:
                res.success = False
                res.message = "生成风险地图失败"
            
            return res
            
        except Exception as e:
            self.get_logger().error(f"创建风险地图时发生错误: {e}")
            res.success = False
            res.message = f"创建风险地图时发生错误: {str(e)}"
            return res

    def _generate_risk_map(self, base_map: OccupancyGrid, risk_zones):
        """根据基础地图和风险区域生成风险地图，使用与free_space相同的原点和分辨率"""
        try:
            # 创建风险地图，使用与base_map相同的参数（这样就与free_space保持一致）
            risk_map = OccupancyGrid()
            risk_map.header = base_map.header
            risk_map.header.frame_id = 'map'
            risk_map.info = base_map.info  # 直接使用base_map的info，确保原点和分辨率一致
            
            # 获取地图参数
            width = base_map.info.width
            height = base_map.info.height
            resolution = base_map.info.resolution
            origin_x = base_map.info.origin.position.x
            origin_y = base_map.info.origin.position.y
            
            # 将地图数据转换为numpy数组
            map_data = np.array(base_map.data, dtype=np.int8).reshape(height, width)
            
            # 创建风险地图数据副本
            risk_data = map_data.copy()
            
            # 处理每个风险区域
            for marker in risk_zones.markers:
                if len(marker.points) < 3:  # 至少需要3个点形成多边形
                    continue
                
                # 将世界坐标转换为地图坐标
                polygon_points = []
                for point in marker.points:
                    map_x = int((point.x - origin_x) / resolution)
                    map_y = int((point.y - origin_y) / resolution)
                    # 注意：地图坐标系中，y轴是反向的
                    map_y = height - 1 - map_y
                    
                    # 确保坐标在地图范围内
                    map_x = max(0, min(width - 1, map_x))
                    map_y = max(0, min(height - 1, map_y))
                    
                    polygon_points.append([map_x, map_y])
                
                # 使用OpenCV填充多边形区域
                polygon_points = np.array(polygon_points, dtype=np.int32)
                
                # 创建掩码
                mask = np.zeros((height, width), dtype=np.uint8)
                cv2.fillPoly(mask, [polygon_points], 255)
                
                # 在风险区域内设置风险值 (使用50表示风险区域，介于自由空间0和障碍物100之间)
                risk_data[mask == 255] = 50
            
            # 将数据转换回列表格式
            risk_map.data = risk_data.flatten().tolist()
            
            self.get_logger().info(f"成功生成风险地图，处理了 {len(risk_zones.markers)} 个风险区域")
            self.get_logger().info(f"风险地图参数 - 分辨率: {resolution}, 原点: ({origin_x}, {origin_y}), 尺寸: {width}x{height}")
            return risk_map
            
        except Exception as e:
            self.get_logger().error(f"生成风险地图时发生错误: {e}")
            return None

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