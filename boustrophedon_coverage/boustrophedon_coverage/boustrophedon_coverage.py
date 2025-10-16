#!/usr/bin/env python3
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


from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

from std_srvs.srv import Trigger,SetBool
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_action_interfaces.action import Waypoint

#custom action client for nav2
from .nav_action_client import NavActionClient  
from boustrophedon_coverage_interfaces.srv import GetZoneList

from .zone_map_client import ZoneMapClient
from boustrophedon_coverage_interfaces.msg import ZoneMap

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
        self.declare_parameter('unknown_as_obstacle', True)    # 未知(-1)是否當作障礙

        #qos setting
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
   
        # 建立服務 service
        self.create_service(Trigger, '/generate_coverage_path', self.generate_coverage_path_srv)
        self.create_service(Trigger, '/waypoint_pub', self.waypoint_pub_srv)
        self.create_service(Trigger, '/cencel_nav2', self.cancel_nav2_srv)
        self.create_service(Trigger, '/check_nav_status', self.check_nav_status_srv)

        # 创建回调组用于服务调用

        #建立服務 client
        
        self.waypoint_active_client = self.create_client(
            SetBool, '/record_path_status', 
        )

        #建立action client
        self.nav_action_client = NavActionClient()
        self.zone_map_client = ZoneMapClient()
        #發布區
        self.path_pub = self.create_publisher(Path, '/coverage_path', 1)
        self.path_marker_pub = self.create_publisher(MarkerArray, '/coverage_path_markers', 1)
        self.free_space_inflated_pub = self.create_publisher(OccupancyGrid, '/free_space_inflated', qos)
        self.risk_map_inflated_pub = self.create_publisher(OccupancyGrid, '/risk_map_inflated', qos)

        # 訂閱地圖話題
        self.free_space_map = None 
        self.risk_map = None
        self.free_space_inflated_map = None
        self.risk_map_inflated_map = None
        
        # 訂閱原始地圖
        # self.sub_free_space_map = self.create_subscription(OccupancyGrid,'/free_space', self.free_space_map_callback, qos)
        self.sub_risk_map = self.create_subscription(OccupancyGrid,'/risk_map', self.risk_map_callback, qos)
        
        # 訂閱膨脹後的地圖 - 這是您需要的關鍵訂閱
        # self.sub_free_space_inflated = self.create_subscription(OccupancyGrid, '/free_space_inflated', self.free_space_inflated_callback, qos)
        self.sub_risk_map_inflated = self.create_subscription(OccupancyGrid, '/risk_map_inflated', self.risk_map_inflated_callback, qos)

        self.waypoint_active = False
       #self.latest_map = None

        self.nav = BasicNavigator()
        self.coverage_path = Path()
        self.coverage_split_points = []
        self.zone_map_list = []
    
    # def free_space_map_callback(self, msg: OccupancyGrid):
    #     self.free_space_map = msg
    #     self.get_logger().info("收到 free_space 地圖")
        
    def risk_map_callback(self, msg: OccupancyGrid):
        self.risk_map = msg
        self.get_logger().info("收到 risk_map 地圖")
        
    # def free_space_inflated_callback(self, msg: OccupancyGrid):
    #     """訂閱膨脹後的自由空間地圖"""
    #     self.free_space_inflated_map = msg
    #     self.get_logger().info("收到 free_space_inflated 地圖")
        
    def risk_map_inflated_callback(self, msg: OccupancyGrid):
        """訂閱膨脹後的風險地圖"""
        self.risk_map_inflated_map = msg
        self.get_logger().info("收到 risk_map_inflated 地圖")

    def generate_coverage_path_srv(self, req, res):
        """服務回調：生成覆蓋路徑"""
        # try:    
        if self.risk_map_inflated_map is None:
            res.success = False
            res.message = "缺少 risk_map_inflated 地圖數據"
            return res
        
        # 調用路徑生成函數
        success = self.generate_coverage_path()
        
        if success:
            res.success = True
            res.message = "覆蓋路徑生成成功"
        else:
            res.success = False
            res.message = "覆蓋路徑生成失敗"
                
        # except Exception as e:
        #     res.success = False
        #     res.message = f"生成路徑時發生錯誤: {str(e)}"
        #     self.get_logger().error(f"生成路徑錯誤: {e}")
            
        return res

    def _validate_maps_compatibility(self, map1: OccupancyGrid, map2: OccupancyGrid) -> bool:
        """驗證兩個地圖是否兼容（相同的分辨率、尺寸和原點）"""
        info1, info2 = map1.info, map2.info
        
        # 檢查分辨率
        if abs(info1.resolution - info2.resolution) > 1e-6:
            return False
            
        # 檢查尺寸
        if info1.width != info2.width or info1.height != info2.height:
            return False
            
        # 檢查原點
        if (abs(info1.origin.position.x - info2.origin.position.x) > 1e-6 or
            abs(info1.origin.position.y - info2.origin.position.y) > 1e-6):
            return False
            
        return True

    # 生成牛耕式覆蓋路徑 輸入 zone map
    def generate_coverage_path(self):
        """
        根據地圖生成牛耕式覆蓋路徑，並修正路徑點的朝向與格式
        """
        import math
        import numpy as np

        def _euler_to_quaternion(roll, pitch, yaw):
            """
            將歐拉角轉換為四元數（x, y, z, w）
            """
            qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
            qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
            qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
            qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
            return (qx, qy, qz, qw)

        def _cal_two_point_orientation(x1, y1, x2, y2):
            """
            計算兩點之間的朝向（歐拉角yaw），並返回對應的四元數
            """
            dx = x2 - x1
            dy = y2 - y1
            yaw = math.atan2(dy, dx)
            return _euler_to_quaternion(0, 0, yaw)

        def _generate_coverage_boustrophedon_path(safe_map: np.ndarray)-> list:
            """
            生成直線覆蓋路徑
            """
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
                    ok = bool(safe_map[i, mc])
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
                reverse = not reverse
            return points
        def _generate_coverage_zigzag_path(self, safe_map: np.ndarray):
            """
            生成zigzag覆蓋路徑
            """
            pass
        def _generate_coverage_diagonal_path(self, safe_map: np.ndarray):
            """
            生成diagonal覆蓋路徑
            """
            pass
        def _generate_coverage_spiral_path(self, safe_map: np.ndarray):
            """
            生成spiral覆蓋路徑
            """
            pass

        def _transform_coverage_path_points(points: list,header) -> Path:
            """
            將路徑點轉換為Path消息
            """
            coverage_path = Path()
            coverage_path.header = header
            coverage_path.header.frame_id = 'map'

            for idx, (x, y) in enumerate(points):
                goal_pose = PoseStamped()
                goal_pose.header = coverage_path.header
                goal_pose.header.stamp = self.get_clock().now().to_msg() #使用node的clock
                goal_pose.pose.position.x = float(x)
                goal_pose.pose.position.y = float(y)
                goal_pose.pose.position.z = 0.0

                # 計算朝向
                if idx < len(points) - 1:
                    x2, y2 = points[idx + 1]
                    qx, qy, qz, qw = _cal_two_point_orientation(x, y, x2, y2)
                elif idx > 0:
                    x2, y2 = points[idx - 1]
                    qx, qy, qz, qw = _cal_two_point_orientation(x2, y2, x, y)
                else:
                    # 只有一個點，朝向正前
                    qx, qy, qz, qw = _euler_to_quaternion(0, 0, 0)

                goal_pose.pose.orientation.x = qx
                goal_pose.pose.orientation.y = qy
                goal_pose.pose.orientation.z = qz
                goal_pose.pose.orientation.w = qw

                coverage_path.poses.append(goal_pose)
            return coverage_path


        self.zone_map_list = self.zone_map_client.get_zone_maps()
        for i in range(len(self.zone_map_list)):
            info = self.zone_map_list[i].mask_map.info
            H, W = info.height, info.width
            res = info.resolution
            ox, oy = info.origin.position.x, info.origin.position.y
            # 取得原始risk_map數據
            if self.risk_map is not None:
                risk_map_data = np.asarray(self.risk_map.data, dtype=np.int16).reshape(H, W)
            else:
                risk_map_data = np.zeros((H, W), dtype=np.int16)

            # 取得區域map_inflated遮罩
            mask_map_inflated_data = np.asarray(self.zone_map_list[i].mask_map_inflated.data, dtype=np.int16).reshape(H, W)

            # 根據規則生成safe_map: 在膨脹區域內且非risk
            safe_map = np.logical_and(mask_map_inflated_data == 0, risk_map_data == 0).astype(np.uint8)
            points = _generate_coverage_boustrophedon_path(safe_map)
            coverage_path = _transform_coverage_path_points(points,self.zone_map_list[i].mask_map.header)
            self.zone_map_list[i].path = coverage_path
        #這邊
        # 修正路徑點，確保每個點的朝向正確
        
        # 將zone_map_list的path轉換為MarkerArray
        marker_array = MarkerArray()
        
        for zone_idx, zone_map in enumerate(self.zone_map_list):
            if zone_map.path and len(zone_map.path.poses) > 0:
                # 為每個zone創建一個線條marker
                line_marker = Marker()
                line_marker.header.frame_id = zone_map.path.header.frame_id
                line_marker.header.stamp = self.get_clock().now().to_msg()
                line_marker.ns = f"zone_{zone_map.zone_id}_path"
                line_marker.id = zone_idx
                line_marker.type = Marker.LINE_STRIP
                line_marker.action = Marker.ADD
                
                # 設置線條屬性
                line_marker.scale.x = 0.1  # 線條寬度
                line_marker.color.r = 1.0 if zone_idx == 0 else 0.0
                line_marker.color.g = 0.0 if zone_idx == 0 else 1.0
                line_marker.color.b = 0.0
                line_marker.color.a = 1.0
                
                # 添加路徑點
                for pose_stamped in zone_map.path.poses:
                    point = Point()
                    point.x = pose_stamped.pose.position.x
                    point.y = pose_stamped.pose.position.y
                    point.z = pose_stamped.pose.position.z
                    line_marker.points.append(point)
                
                marker_array.markers.append(line_marker)
                
                # 可選：為每個路徑點創建箭頭marker顯示方向
                for i, pose_stamped in enumerate(zone_map.path.poses[::5]):  # 每5個點顯示一個箭頭
                    arrow_marker = Marker()
                    arrow_marker.header.frame_id = zone_map.path.header.frame_id
                    arrow_marker.header.stamp = self.get_clock().now().to_msg()
                    arrow_marker.ns = f"zone_{zone_map.zone_id}_arrows"
                    arrow_marker.id = zone_idx * 1000 + i  # 確保ID唯一
                    arrow_marker.type = Marker.ARROW
                    arrow_marker.action = Marker.ADD
                    
                    # 設置箭頭位置和方向
                    arrow_marker.pose = pose_stamped.pose
                    
                    # 設置箭頭屬性
                    arrow_marker.scale.x = 0.3  # 箭頭長度
                    arrow_marker.scale.y = 0.05  # 箭頭寬度
                    arrow_marker.scale.z = 0.05  # 箭頭高度
                    arrow_marker.color.r = 0.5 if zone_idx == 0 else 0.0
                    arrow_marker.color.g = 0.0 if zone_idx == 0 else 0.5
                    arrow_marker.color.b = 0.5
                    arrow_marker.color.a = 0.8
                    
                    marker_array.markers.append(arrow_marker)
        
        # 發布MarkerArray
        self.path_marker_pub.publish(marker_array)
        self.get_logger().info(f"發布了 {len(marker_array.markers)} 個路徑markers")
        return True

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