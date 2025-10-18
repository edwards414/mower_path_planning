#!/usr/bin/env python3
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from std_srvs.srv import Trigger,SetBool
from nav2_simple_commander.robot_navigator import BasicNavigator

from .path_generators.boustrophedon import _generate_coverage_boustrophedon_path
from .utils.path_utils import _transform_coverage_path_points
from .utils.nav_action_client import NavActionClient
from .utils.zone_map_client import ZoneMapClient

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
        self.sub_risk_map = self.create_subscription(OccupancyGrid,'/risk_map', self.risk_map_callback, qos)
        
        # 訂閱膨脹後的地圖 - 這是您需要的關鍵訂閱
        self.sub_risk_map_inflated = self.create_subscription(OccupancyGrid, '/risk_map_inflated', self.risk_map_inflated_callback, qos)

        self.waypoint_active = False
        self.nav = BasicNavigator()
        self.coverage_path = Path()
        self.coverage_split_points = []
        self.zone_map_list = []
        
    def risk_map_callback(self, msg: OccupancyGrid):
        self.risk_map = msg
        self.get_logger().info("收到 risk_map 地圖")
        
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
                
        return res

    # 生成牛耕式覆蓋路徑 輸入 zone map
    def generate_coverage_path(self):
        self.zone_map_list = self.zone_map_client.get_zone_maps()
        for i in range(len(self.zone_map_list)):
            info = self.zone_map_list[i].mask_map.info
            H, W = info.height, info.width
            res = info.resolution
            ox, oy = info.origin.position.x, info.origin.position.y
            # 取得原始risk_map數據
            if self.risk_map_inflated_map is not None:
                risk_map_data = np.asarray(self.risk_map_inflated_map.data, dtype=np.int16).reshape(H, W)
            else:
                risk_map_data = np.zeros((H, W), dtype=np.int16)

            # 取得區域map_inflated遮罩
            mask_map_inflated_data = np.asarray(self.zone_map_list[i].mask_map_inflated.data, dtype=np.int16).reshape(H, W)

            # 根據規則生成safe_map: 在膨脹區域內且非risk
            safe_map = np.logical_and(mask_map_inflated_data == 0, risk_map_data == 0).astype(np.uint8)
            
            points = _generate_coverage_boustrophedon_path(safe_map=safe_map,
                                                            strip_width_m=self.get_parameter('strip_width_m').value,
                                                            waypoint_spacing_m=self.get_parameter('waypoint_spacing_m').value,
                                                            res=res,
                                                            H=H,
                                                            W=W,
                                                            origin_x=ox,
                                                            origin_y=oy,
                                                            angle_deg=0.0)

            coverage_path = _transform_coverage_path_points(points,self.zone_map_list[i].mask_map.header)
            self.zone_map_list[i].path = coverage_path
        #這邊
        # 修正路徑點，確保每個點的朝向正確
        vivid_colors = [
                    (1.0, 0.0, 0.0),  # 紅
                    (0.0, 1.0, 0.0),  # 綠
                    (0.0, 0.0, 1.0),  # 藍
                    (1.0, 1.0, 0.0),  # 黃
                    (1.0, 0.0, 1.0),  # 紫
                    (0.0, 1.0, 1.0),  # 青
                    (1.0, 0.5, 0.0),  # 橙
                    (0.5, 0.0, 1.0),  # 靛
                    (0.0, 0.5, 1.0),  # 天藍
                    (0.5, 1.0, 0.0),  # 螢光黃綠
                ]
        # 將zone_map_list的path轉換為MarkerArray
        marker_array = MarkerArray()
        color_i = 0 #顏色
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
                line_marker.scale.x = 0.03  # 線條寬度
                # 使用更鮮明的顏色（取自一組亮麗的顏色表 / 彩虹色輪）
                
                rgb = vivid_colors[color_i % len(vivid_colors)]
                line_marker.color.r = rgb[0]
                line_marker.color.g = rgb[1]
                line_marker.color.b = rgb[2]
                line_marker.color.a = 0.8
                
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
                    arrow_marker.scale.x = 0.15  # 箭頭長度
                    arrow_marker.scale.y = 0.04  # 箭頭寬度
                    arrow_marker.scale.z = 0.08  # 箭頭高度
                    arrow_marker.color.r = 0.5 if zone_idx == 0 else 0.0
                    arrow_marker.color.g = 0.0 if zone_idx == 0 else 0.5
                    arrow_marker.color.b = 0.5
                    arrow_marker.color.a = 0.8
                    
                    marker_array.markers.append(arrow_marker)
            color_i += 1
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

        
def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()