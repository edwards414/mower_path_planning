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
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator

nav = BasicNavigator()
class CoveragePlanner(Node):
    def __init__(self):
        super().__init__('boustrophedon_coverage')        
        # 參數
        self.declare_parameter('strip_width_m', 0.15)          # 割草機有效割幅
        self.declare_parameter('waypoint_spacing_m', 0.15)     # 路徑點間距
        self.declare_parameter('free_threshold', 25)           # 佔據格 <= 此值視為可行
        self.declare_parameter('unknown_as_obstacle', True)    # 未知(-1)是否當作障礙
        self.declare_parameter('inflate_radius_m', 0.1)       # 安全膨脹半徑(機身+裕度)



        #qos setting
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        #訂閱區
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.getCellMatAndFreeSpace, qos
        )
        self.map_sub2 = self.create_subscription(
            OccupancyGrid, '/map', self.on_map, qos
        )

        #發布區
        self.map_split_line = self.create_publisher(Marker, '/coverage_split_lines', 1)
        self.path_pub = self.create_publisher(Path, '/coverage_path', 1)
        self.free_pub = self.create_publisher(OccupancyGrid, '/free_space', 1)
    def getCellMatAndFreeSpace(self,map_msg: OccupancyGrid):
        size_of_cell = 5
        row, col = map_msg.info.height, map_msg.info.width
        print(row, col)
        occ = np.asarray(map_msg.data, dtype=np.int16).reshape(row, col)
        sub_map = [[0 for i in range(size_of_cell)] for j in range(size_of_cell)]
        # map = np.array(map_msg.data)
        # map_r , map_l = np.array_split(map, size_of_cell,axis=0)
        # print(map_r)
        i, j = 0, 0
        for sub_map_row in np.array_split(occ, size_of_cell, axis=0):
            for sub_map_col in np.array_split(sub_map_row, size_of_cell, axis=1):
                sub_map[i][j] = sub_map_col
                j += 1
            i += 1
            j = 0
        self.publish_split_lines(map_msg, rows=size_of_cell, cols=size_of_cell, line_width=0.03)

    def get_CellMat_and_FreeSpace(self,map_msg  : ) 


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
        # 障礙膨脹
        inflate_r_m = float(self.get_parameter('inflate_radius_m').value)
        r_cells = max(0, int(math.ceil(inflate_r_m / res)))
        if r_cells > 0:
            occ_mask = (~free_mask).astype(np.uint8)
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r_cells + 1, 2 * r_cells + 1))
            occ_mask = cv2.dilate(occ_mask, k)
            free_mask = (occ_mask == 0)

        # plt.imshow(free_mask)
        # plt.show()
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
            # 沿條帶中心列，找連續可行段
            segments = []
            start = None
            for i in range(H):
                ok = bool(free_mask[i, mc])
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

    def generate_turn_points(self, current_x: float, current_y: float, 
                           next_x: float, turn_radius: float, reverse: bool):
        """
        生成圓角轉彎的路徑點
        
        Args:
            current_x, current_y: 當前段結束點
            next_x: 下一條帶的x座標
            turn_radius: 轉彎半徑
            reverse: 是否反向
        
        Returns:
            轉彎路徑點列表
        """
        turn_points = []
        
        # 計算轉彎中心點
        if reverse:
            # 反向時，轉彎中心在左側
            turn_center_x = current_x - turn_radius
            turn_center_y = current_y
        else:
            # 正向時，轉彎中心在右側
            turn_center_x = current_x + turn_radius
            turn_center_y = current_y
        
        # 計算轉彎角度範圍
        if reverse:
            start_angle = 0  # 從右側開始
            end_angle = np.pi  # 轉到左側
            angle_step = np.pi / 8  # 分8段
        else:
            start_angle = np.pi  # 從左側開始
            end_angle = 0  # 轉到右側
            angle_step = -np.pi / 8  # 分8段
        
        # 生成圓弧點
        angles = np.arange(start_angle, end_angle, angle_step)
        for angle in angles:
            x = turn_center_x + turn_radius * np.cos(angle)
            y = turn_center_y + turn_radius * np.sin(angle)
            turn_points.append((x, y))
        
        # 添加轉彎結束點（連接到下一條帶）
        if reverse:
            final_x = next_x + turn_radius
        else:
            final_x = next_x - turn_radius
        
        turn_points.append((final_x, current_y))
        
        return turn_points



    def publish_split_lines(self, map_msg: OccupancyGrid, rows: int, cols: int, line_width: float = 0.03):
        info = map_msg.info
        w, h = info.width, info.height
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y

        marker = Marker()
        marker.header.frame_id = map_msg.header.frame_id or 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'split'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = line_width
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

        def add_line(x0, y0, x1, y1):
            p0 = Point(x=float(x0), y=float(y0), z=0.0)
            p1 = Point(x=float(x1), y=float(y1), z=0.0)
            marker.points.append(p0)
            marker.points.append(p1)

        # 垂直分割線（忽略邊界，只畫內部分割）
        for k in range(1, cols):
            j = (w * k) // cols
            x = ox + j * res
            add_line(x, oy, x, oy + h * res)

        # 水平分割線
        for k in range(1, rows):
            i = (h * k) // rows
            y = oy + i * res
            add_line(ox, y, ox + w * res, y)

        self.map_split_line.publish(marker)
        self.get_logger().info('split lines published')
        

def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlanner()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    
    # rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()