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
from geometry_msgs.msg import PoseStamped, Point32
from geometry_msgs.msg import PolygonStamped
import matplotlib.pyplot as plt

from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

from rclpy.callback_groups import ReentrantCallbackGroup

from std_srvs.srv import Trigger
class CoveragePlanner(Node):
    def __init__(self):
        super().__init__('boustrophedon_coverage')        
        # 參數
        self.get_logger().info("boustrophedon_coverage 初始化")
        self.declare_parameter('strip_width_m', 0.1)          # 割草機有效割幅
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
   
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos)

        self.free_space_pub = self.create_publisher(OccupancyGrid, '/free_space', 1)
        self.free_space_inflated_pub = self.create_publisher(OccupancyGrid, '/free_space_inflated', 1)
        self.polygon_point_sub = self.create_subscription(Marker, '/recorded_path_polygon', self.polygon_point_callback, 10)
        self.polygon_points = None

        # 建立服務 client
        self.create_service(Trigger, '/generate_polygon_mask', self.handle_generate_polygon_mask)

        self.latest_map = None
        
        #發布區
        # self.map_split_line = self.create_publisher(Marker, '/coverage_split_lines', 1)
        self.path_pub = self.create_publisher(Path, '/coverage_path', 1)
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
        
        if self.latest_map is None:
            response.success = False
            response.message = "尚未收到 map"
            return response
    
        self.get_logger().info("收到 polygon points")
        # 將 polygon 轉換為地圖遮罩
        map_msg = self.latest_map
        info = map_msg.info
        H, W = info.height, info.width
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y

        # 將 polygon 的點轉換為像素座標
        poly_px = []
        for pt in self.polygon_points:
            x = int((pt.x - ox) / res)
            y = int((pt.y - oy) / res)
            poly_px.append([x, y])
        poly_px = np.array([poly_px], dtype=np.int32)

        # 建立遮罩
        mask = np.zeros((H, W), dtype=np.uint8)
        cv2.fillPoly(mask, [poly_px], 1)

        # 將遮罩應用到地圖
        occ = np.asarray(map_msg.data, dtype=np.int16).reshape(H, W)
        occ_masked = np.where(mask == 1, occ, 100)  # 遮罩外設為障礙

        # 發布遮罩後的地圖
        masked_map = OccupancyGrid()
        masked_map.header = map_msg.header
        masked_map.header.frame_id = 'map'
        masked_map.info = map_msg.info
        masked_map.data = occ_masked.flatten().tolist()
        self.free_space_pub.publish(masked_map)

        self.on_map(masked_map)


        response.success = True
        response.message = "已根據 polygon 生成地圖遮罩"
        return response

    def on_map(self, map_msg: OccupancyGrid):
        info = map_msg.info
        H, W = info.height, info.width
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y

        occ = np.asarray(map_msg.data, dtype=np.int16).reshape(H, W)
        free_th = int(self.get_parameter('free_threshold').value)
        unknown_as_obstacle = bool(self.get_parameter('unknown_as_obstacle').value)

        # 建立可行遮罩
        free_mask = (occ >= 0) & (occ <= free_th)
        # # 障礙膨脹
        inflate_r_m = float(self.get_parameter('inflate_radius_m').value)
        r_cells = max(0, int(math.ceil(inflate_r_m / res)))
        if r_cells > 0:
            occ_mask = (~free_mask).astype(np.uint8)
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r_cells + 1, 2 * r_cells + 1))
            occ_mask = cv2.dilate(occ_mask, k)
            free_mask = (occ_mask == 0)
        # 發布膨脹後的地圖（free_mask為True的為可行區，其餘為障礙）
        inflated_map = OccupancyGrid()
        inflated_map.header = map_msg.header
        inflated_map.header.frame_id = 'map'
        inflated_map.info = map_msg.info

        # 轉成0/100格式，0為可行，100為障礙
        inflated_data = np.where(free_mask, 0, 100).astype(np.int8)
        inflated_map.data = inflated_data.flatten().tolist()
        self.free_space_inflated_pub.publish(inflated_map)

        # ==========================
        # 讓規劃路徑考慮到膨脹層的縮減
        # ==========================
        # 這裡我們將free_mask進一步縮減，確保路徑點不會貼近膨脹邊緣
        # 例如再進行一次膨脹，然後取反，作為安全區域
        if r_cells > 0:
            # 再膨脹一次，縮減可行區
            shrink_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r_cells + 1, 2 * r_cells + 1))
            safe_mask = cv2.erode(free_mask.astype(np.uint8), shrink_k)
            safe_mask = (safe_mask == 1)
        else:
            safe_mask = free_mask

        # 條帶設定：以 X 方向切直條(沿 Y 掃描)
        strip_w_m = float(self.get_parameter('strip_width_m').value)
        strip_cols = max(1, int(round(strip_w_m / res)))
        midcols = list(range(strip_cols // 2, W, strip_cols))

        # 視覺化條帶分割線
        cols_for_viz = max(1, int(math.ceil(W / strip_cols)))
        
        # self.publish_split_lines(map_msg, rows=1, cols=cols_for_viz, line_width=0.03)

        # 產生往返路徑
        spacing = float(self.get_parameter('waypoint_spacing_m').value)
        points = []
        reverse = False

        for mc in midcols:
            # 沿條帶中心列，找連續可行段（這裡用縮減後的safe_mask）
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
                points.extend([(x, y) for y in ys])
                
            reverse = not reverse
        # 發布 Path
        path = Path()
        path.header = map_msg.header
        path.header.frame_id = map_msg.header.frame_id or 'map'
        for (x, y) in points:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        
        self.path_pub.publish(path)
    # def mask_map(self,submap:np.ndarray):
    #     free_mask = submap
    #     free_th = int(free_threshold.value)
    #     free_mask = (free_mask >= 0) & (free_mask <= free_th)
    #     if unknow_as_obstacle.value:
    #         free_mask &= (occ >= 0)

    #     #膨脹
    #     inflate_r_m = float(self.get_parameter('inflate_radius_m').value)
    #     r_cells = max(0, int(math.ceil(inflate_r_m / res)))
    #     if r_cells > 0:
    #         occ_mask = (~free_mask).astype(np.uint8)
    #         k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r_cells + 1, 2 * r_cells + 1))
    #         occ_mask = cv2.dilate(occ_mask, k)
    #         free_mask = (occ_mask == 0)
    #     return free_mask

    # def generate_turn_points(self, current_x: float, current_y: float, 
    #                        next_x: float, turn_radius: float, reverse: bool):
    #     """
    #     生成圓角轉彎的路徑點
        
    #     Args:
    #         current_x, current_y: 當前段結束點
    #         next_x: 下一條帶的x座標
    #         turn_radius: 轉彎半徑
    #         reverse: 是否反向
        
    #     Returns:
    #         轉彎路徑點列表
    #     """
    #     turn_points = []
        
    #     # 計算轉彎中心點
    #     if reverse:
    #         # 反向時，轉彎中心在左側
    #         turn_center_x = current_x - turn_radius
    #         turn_center_y = current_y
    #     else:
    #         # 正向時，轉彎中心在右側
    #         turn_center_x = current_x + turn_radius
    #         turn_center_y = current_y
        
    #     # 計算轉彎角度範圍
    #     if reverse:
    #         start_angle = 0  # 從右側開始
    #         end_angle = np.pi  # 轉到左側
    #         angle_step = np.pi / 8  # 分8段
    #     else:
    #         start_angle = np.pi  # 從左側開始
    #         end_angle = 0  # 轉到右側
    #         angle_step = -np.pi / 8  # 分8段
        
    #     # 生成圓弧點
    #     angles = np.arange(start_angle, end_angle, angle_step)
    #     for angle in angles:
    #         x = turn_center_x + turn_radius * np.cos(angle)
    #         y = turn_center_y + turn_radius * np.sin(angle)
    #         turn_points.append((x, y))
        
    #     # 添加轉彎結束點（連接到下一條帶）
    #     if reverse:
    #         final_x = next_x + turn_radius
    #     else:
    #         final_x = next_x - turn_radius
        
    #     turn_points.append((final_x, current_y))
        
    #     return turn_points



    # def publish_split_lines(self, map_msg: OccupancyGrid, rows: int, cols: int, line_width: float = 0.03):
    #     info = map_msg.info
    #     w, h = info.width, info.height
    #     res = info.resolution
    #     ox, oy = info.origin.position.x, info.origin.position.y

    #     marker = Marker()
    #     marker.header.frame_id = map_msg.header.frame_id or 'map'
    #     marker.header.stamp = self.get_clock().now().to_msg()
    #     marker.ns = 'split'
    #     marker.id = 0
    #     marker.type = Marker.LINE_LIST
    #     marker.action = Marker.ADD
    #     marker.pose.orientation.w = 1.0
    #     marker.scale.x = line_width
    #     marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

    #     def add_line(x0, y0, x1, y1):
    #         p0 = Point(x=float(x0), y=float(y0), z=0.0)
    #         p1 = Point(x=float(x1), y=float(y1), z=0.0)
    #         marker.points.append(p0)
    #         marker.points.append(p1)

    #     # 垂直分割線（忽略邊界，只畫內部分割）
    #     for k in range(1, cols):
    #         j = (w * k) // cols
    #         x = ox + j * res
    #         add_line(x, oy, x, oy + h * res)

    #     # 水平分割線
    #     for k in range(1, rows):
    #         i = (h * k) // rows
    #         y = oy + i * res
    #         add_line(ox, y, ox + w * res, y)

    #     self.map_split_line.publish(marker)
    #     self.get_logger().info('split lines published')
        

def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()