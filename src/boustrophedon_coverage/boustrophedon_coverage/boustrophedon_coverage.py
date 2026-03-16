#!/usr/bin/env python3

# Copyright 2024 fxrbindi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Boustrophedon coverage path planning module.

This module implements a coverage path planning system for
autonomous lawn mowers using the boustrophedon (back-and-forth)
pattern. It integrates with ROS2 Nav2 for autonomous navigation.
Multiple path generation modes are supported: boustrophedon,
spiral, and zigzag patterns.
"""

from boustrophedon_coverage_interfaces.srv import ZoneExecPath

from geometry_msgs.msg import Point

from nav2_simple_commander.robot_navigator import BasicNavigator

from nav_msgs.msg import OccupancyGrid, Path

import numpy as np

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from std_srvs.srv import SetBool, Trigger

from visualization_msgs.msg import Marker, MarkerArray

from .path_generators.boustrophedon import (
    _generate_coverage_boustrophedon_path
)
from .utils.nav_action_client import NavActionClient
from .utils.path_utils import (
    _transform_coverage_path_points,
    _transform_coverage_split_points,
)
from .utils.zone_map_client import ZoneMapClient


class CoveragePlanner(Node):
    """CoveragePlanner class."""

    def __init__(self):
        """Initialize the CoveragePlanner class."""
        super().__init__('boustrophedon_coverage')
        self.set_parameters(
            [Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        # 參數
        self.get_logger().info('boustrophedon_coverage 初始化')
        self.declare_parameter('strip_width_m', 0.2)  # 割草機有效割幅
        self.declare_parameter('waypoint_spacing_m', 0.1)  # 路徑點間距
        self.declare_parameter('unknown_as_obstacle', True)  # 未知(-1)是否當作障礙

        # qos setting — 發布用
        qos_vol = QoSProfile(depth=1)
        qos_vol.durability = QoSDurabilityPolicy.VOLATILE
        qos_vol.reliability = QoSReliabilityPolicy.RELIABLE

        # 訂閱地圖用 TRANSIENT_LOCAL，讓晚加入也能收到最新地圖
        qos_tl = QoSProfile(depth=1)
        qos_tl.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos_tl.reliability = QoSReliabilityPolicy.RELIABLE

        self.cb_group = ReentrantCallbackGroup()

        # 建立服務 service
        self.create_service(
            Trigger, '/generate_coverage_path', self.generate_coverage_path_srv,
            callback_group=self.cb_group
        )
        self.create_service(ZoneExecPath, '/zone_exec_path',
                            self.zone_exec_path_srv,
                            callback_group=self.cb_group)
        self.create_service(Trigger, '/cencel_nav2', self.cancel_nav2_srv,
                            callback_group=self.cb_group)
        self.create_service(Trigger, '/check_nav_status',
                            self.check_nav_status_srv,
                            callback_group=self.cb_group)
        # 创建回调组用于服务调用
        # 建立服務 client
        self.waypoint_active_client = self.create_client(
            SetBool,
            '/record_path_status',
        )
        # 建立action client - 直接在此节点中创建
        self._action_client_split_path = NavActionClient()
        self.zone_map_client = ZoneMapClient()
        # 發布區
        self.path_pub = self.create_publisher(Path, '/coverage_path', 1)
        self.path_marker_pub = self.create_publisher(
            MarkerArray, '/coverage_path_markers', 1
        )
        self.free_space_inflated_pub = self.create_publisher(
            OccupancyGrid, '/free_space_inflated', qos_vol
        )
        self.risk_map_inflated_pub = self.create_publisher(
            OccupancyGrid, '/risk_map_inflated', qos_vol
        )

        # 訂閱地圖話題
        self.free_space_map = None
        self.risk_map = None
        self.free_space_inflated_map = None
        self.risk_map_inflated_map = None

        # 訂閱原始地圖
        self.sub_risk_map = self.create_subscription(
            OccupancyGrid, '/risk_map', self.risk_map_callback, qos_tl,
            callback_group=self.cb_group
        )

        # 訂閱膨脹後的地圖 - TRANSIENT_LOCAL 確保晚加入也能收到
        self.sub_risk_map_inflated = self.create_subscription(
            OccupancyGrid,
            '/risk_map_inflated',
            self.risk_map_inflated_callback,
            qos_tl,
            callback_group=self.cb_group
        )

        self.waypoint_active = False
        self.nav = BasicNavigator()
        self.coverage_path = Path()
        self.coverage_split_points = []
        self.zone_map_list = []

    def risk_map_callback(self, msg: OccupancyGrid):
        """Handle the risk map subscription callback."""
        self.risk_map = msg
        self.get_logger().info('收到 risk_map 地圖')

    def risk_map_inflated_callback(self, msg: OccupancyGrid):
        """訂閱膨脹後的風險地圖."""
        self.risk_map_inflated_map = msg
        self.get_logger().info('收到 risk_map_inflated 地圖')

    def generate_coverage_path_srv(self, req, res):
        """服務回調：生成覆蓋路徑."""
        # try:
        if self.risk_map_inflated_map is None:
            res.success = False
            res.message = '缺少 risk_map_inflated 地圖數據'
            return res

        # 調用路徑生成函數
        success = self.generate_coverage_path()

        if success:
            res.success = True
            res.message = '覆蓋路徑生成成功'
        else:
            res.success = False
            res.message = '覆蓋路徑生成失敗'

        return res

    # 生成牛耕式覆蓋路徑 輸入 zone map
    def generate_coverage_path(self):
        """Generate coverage path for all zones."""
        self.zone_map_list = self.zone_map_client.get_zone_maps()
        for i in range(len(self.zone_map_list)):
            info = self.zone_map_list[i].mask_map.info
            H, W = info.height, info.width
            res = info.resolution
            ox, oy = info.origin.position.x, info.origin.position.y
            # 取得原始risk_map數據
            if self.risk_map_inflated_map is not None:
                risk_map_data = np.asarray(
                    self.risk_map_inflated_map.data, dtype=np.int16
                ).reshape(H, W)
            else:
                risk_map_data = np.zeros((H, W), dtype=np.int16)

            # 取得區域map_inflated遮罩
            mask_map_inflated_data = np.asarray(
                self.zone_map_list[i].mask_map_inflated.data, dtype=np.int16
            ).reshape(H, W)

            # 根據規則生成safe_map: 在膨脹區域內且非risk
            safe_map = np.logical_and(
                mask_map_inflated_data == 0, risk_map_data == 0
            ).astype(np.uint8)

            coverage_path, coverage_split_points = (
                _generate_coverage_boustrophedon_path(
                    safe_map=safe_map,
                    strip_width_m=self.get_parameter('strip_width_m').value,
                    waypoint_spacing_m=self.get_parameter(
                        'waypoint_spacing_m').value,
                    res=res,
                    H=H,
                    W=W,
                    origin_x=ox,
                    origin_y=oy,
                    angle_deg=0.0,
                )
            )

            coverage_path = _transform_coverage_path_points(
                points=coverage_path,
                map_header=self.zone_map_list[i].mask_map.header
            )

            coverage_split_points = _transform_coverage_split_points(
                points=coverage_split_points,
                map_header=self.zone_map_list[i].mask_map.header,
            )
            self.zone_map_list[i].path = coverage_path
            self.zone_map_list[i].coverage_split_points = coverage_split_points
        # 這邊
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
        color_i = 0  # 顏色
        for zone_idx, zone_map in enumerate(self.zone_map_list):
            if zone_map.path and len(zone_map.path.poses) > 0:
                # 為每個zone創建一個線條marker
                line_marker = Marker()
                line_marker.header.frame_id = zone_map.path.header.frame_id
                line_marker.header.stamp = self.get_clock().now().to_msg()
                line_marker.ns = f'zone_{zone_map.zone_id}_path'
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
                # 每5個點顯示一個箭頭
                for i, pose_stamped in enumerate(zone_map.path.poses[::5]):
                    arrow_marker = Marker()
                    arrow_marker.header.frame_id = (
                        zone_map.path.header.frame_id
                    )
                    arrow_marker.header.stamp = self.get_clock().now().to_msg()
                    arrow_marker.ns = f'zone_{zone_map.zone_id}_arrows'
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
        self.get_logger().info(f'發布了 {len(marker_array.markers)} 個路徑markers')
        return True

    # 發佈coverage path to action server
    def zone_exec_path_srv(self, req, res):
        """
        Execute a path for a given zone.

        Args
        ----
        req : ZoneExecPath.Request
            The service request containing the zone_id to execute.
        res : ZoneExecPath.Response
            The service response containing success status and message.

        Notes
        -----
        command: ros2 service call /zone_exec_path
        boustrophedon_coverage_interfaces/srv/ZoneExecPath
        "zone_id: 'zone_1'"

        """
        self.get_logger().info(f'zone_exec_path_srv start: {req.zone_id}')
        zone_id = req.zone_id
        zone_map = None
        for zone in self.zone_map_list:
            if zone.zone_id == zone_id:
                zone_map = zone
                break

        if zone_map:
            # 调用发送目标的方法
            self._action_client_split_path.send_goal_split_path(
                path=zone_map.path,
                coverage_split_points=zone_map.coverage_split_points
            )
            res.success = True
            res.message = 'Goal sent to navigation action server'
        else:
            res.success = False
            res.message = 'Zone not found'

        return res

    def cancel_nav2_srv(self, req, res):
        """Cancel the current navigation task."""
        self.nav.cancelTask()
        res.success = True
        res.message = 'Nav2 canceled'
        return res

    def check_nav_status_srv(self, req, res):
        """Check the status of the current navigation task."""
        res.success = self.nav.isTaskComplete()
        if res.success:
            res.message = 'Navigation completed'
        else:
            feedback = self.nav.getFeedback()
            res.message = f'Navigation in progress: {feedback}'
        return res

    def record_path_status_srv(self, req, res):
        """
        Record the status of the path.

        Args
        ----
        req : SetBool.Request
            The service request containing the path recording status.
        res : SetBool.Response
            The service response containing success status and message.

        Returns
        -------
        SetBool.Response
            The response message with success status and message.

        """
        self.waypoint_active = req.data  # 直接赋值
        res.success = True
        res.message = f'Path recording status: {self.waypoint_active}'
        return res


def main(args=None):
    """Initialize and run the CoveragePlanner node."""
    rclpy.init(args=args)
    node = CoveragePlanner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
