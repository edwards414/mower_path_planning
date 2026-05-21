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

import json

import math

import os

from mower_interface.srv import ChannelPathList, ChannelRoute, GetZoneList

from geometry_msgs.msg import Point, PoseStamped

from nav_msgs.msg import Path

from mower_interface.srv import ChennalPathList

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from std_srvs.srv import Trigger

from tf2_ros import Buffer, TransformListener

from visualization_msgs.msg import Marker, MarkerArray

from .utils.path_record_utils import path_to_marker, simplify_path


class PathRecorder(Node):

    def __init__(self):
        super().__init__('path_recorder')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('min_dist', 0.05)
        self.declare_parameter('min_dt', 0.10)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('save_dir', 'zone_record')
        self.declare_parameter('polygon_simplify_dist', 0.1)
        self.get_logger().info('path_recorder ready.')

        os.makedirs(self.get_parameter('save_dir').value, exist_ok=True)

        polygon_qos = QoSProfile(depth=1)
        polygon_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        polygon_qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.path_pub = self.create_publisher(Path, '/recorded_path', 10)
        self.zone_marker_pub = self.create_publisher(
            Marker, '/zone_markers', polygon_qos)
        self.zone_list_pub = self.create_publisher(
            MarkerArray, '/zone_list', polygon_qos)

        self.risk_zone_marker_pub = self.create_publisher(
            Marker, '/risk_zone_markers', polygon_qos)
        self.risk_zone_list_pub = self.create_publisher(
            MarkerArray, '/risk_zone_list', polygon_qos)

        self.chennal_path_pub = self.create_publisher(
            Path, '/chennal_path', 10)
        self.chennal_path_array_pub = self.create_publisher(
            MarkerArray, '/chennal_path_array', polygon_qos)
        self.channel_path_pub = self.create_publisher(
            Path, '/channel_path', 10)
        self.channel_path_array_pub = self.create_publisher(
            MarkerArray, '/channel_path_array', polygon_qos)

        self.timer_period = 0.1
        self.timer = self.create_timer(
            self.timer_period, self.publish_path_timer)

        self.create_service(
            Trigger, '/risk_zone_start', self.risk_zone_start_srv)
        self.create_service(
            Trigger, '/risk_zone_end', self.risk_zone_end_srv)
        self.create_service(
            Trigger, '/save_zone_list', self.save_zone_list_srv)
        self.create_service(
            Trigger, '/load_zone_list', self.load_zone_list_srv)

        self.create_service(
            Trigger, '/record_zone_start', self.record_zone_start_srv)
        self.create_service(
            Trigger, '/record_zone_end', self.record_zone_end_srv)

        self.create_service(
            Trigger, '/channel_record_start', self.chennal_record_start_srv)
        self.create_service(
            Trigger, '/channel_record_end', self.chennal_record_end_srv)
        self.create_service(
            Trigger, '/chennal_record_start', self.chennal_record_start_srv)
        self.create_service(
            Trigger, '/chennal_record_end', self.chennal_record_end_srv)

        self.create_service(
            Trigger, '/get_record_zone_info', self.get_record_zone_info_srv)
        self.create_service(
            GetZoneList, '/get_record_zone_list',
            self.get_record_zone_list_srv)
        self.create_service(
            GetZoneList, '/get_risk_zone_list', self.get_risk_zone_list_srv)
        self.create_service(
            ChannelPathList, '/get_channel_path_list',
            self.get_channel_path_list_srv)
        self.create_service(
            ChennalPathList, '/get_chennal_path_list',
            self.get_chennal_path_list_srv)
        self.create_service(
            ChannelRoute, '/get_channel_route',
            self.get_channel_route_srv)

        self.risk_path = Path()
        self.risk_path.header.frame_id = self.get_parameter(
            'frame_id').value

        self.record_zone_status = False
        self.record_zone_id = 0
        self.record_zone_name = 'zone_' + str(self.record_zone_id)
        self.record_zone_marker = Marker()
        self.record_zone_list = MarkerArray()

        self.risk_zone_status = False
        self.risk_zone_id = 0
        self.risk_zone_name = 'risk_zone_' + str(self.risk_zone_id)
        self.risk_zone_marker = Marker()
        self.risk_zone_list = MarkerArray()

        self.path = Path()
        self.path.header.frame_id = self.get_parameter('frame_id').value
        self.last_pt = None
        self.last_t = self.get_clock().now()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_robot_pos = None
        self.initialized = False

        self.init_timer = self.create_timer(0.5, self.try_initialize)

        self.chennal_record_status = False
        self.chennal_record_id = 0
        self.chennal_record_name = 'chennal_record_' + \
            str(self.chennal_record_id)
        self.chennal_path = Path()
        self.chennal_path.header.frame_id = self.get_parameter(
            'frame_id').value
        self.chennal_path_array = MarkerArray()

    def try_initialize(self):
        def publish_empty_markerarrays():
            """清除 RViz 上的 zone_list 與 risk_zone_list."""
            frame = self.get_parameter('frame_id').value
            now = self.get_clock().now().to_msg()

            delete_zone_marker = Marker()
            delete_zone_marker.header.frame_id = frame
            delete_zone_marker.header.stamp = now
            delete_zone_marker.ns = 'zones'
            delete_zone_marker.action = Marker.DELETEALL
            self.zone_marker_pub.publish(delete_zone_marker)

            delete_risk_marker = Marker()
            delete_risk_marker.header.frame_id = frame
            delete_risk_marker.header.stamp = now
            delete_risk_marker.ns = 'risk_zones'
            delete_risk_marker.action = Marker.DELETEALL
            self.risk_zone_marker_pub.publish(delete_risk_marker)

            self.record_zone_list = MarkerArray()
            self.risk_zone_list = MarkerArray()

            self.get_logger().info(
                '✅ 已清空 RViz zone_markers 與 risk_zone_markers')

        if not self.initialized:
            robot_pos = self.get_robot_pos()
            if robot_pos is not None:
                self.last_robot_pos = robot_pos
                self.initialized = True

                publish_empty_markerarrays()

                self.zone_list_pub.publish(self.record_zone_list)
                self.risk_zone_list_pub.publish(self.risk_zone_list)

                self.get_logger().info(
                    f'機器人位置初始化成功: '
                    f'x={robot_pos.pose.position.x:.3f}, '
                    f'y={robot_pos.pose.position.y:.3f}')
                self.path.header.frame_id = self.get_parameter(
                    'frame_id').value
                self.path.header.stamp = self.get_clock().now().to_msg()
                self.path_pub.publish(self.path)
                self.init_timer.cancel()
            else:
                self.get_logger().warn('等待TF可用以初始化機器人位置...')

    def get_robot_pos(self):
        """讀取tf，將odom座標轉換成目標map座標系下的位置."""
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', now)
            pose = PoseStamped()
            pose.header.stamp = trans.header.stamp
            pose.header.frame_id = 'map'
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation
            return pose
        except Exception as e:
            self.get_logger().warn(f'TF查詢失敗: {e}')
            return None

    def create_polygon_from_path(self, poses):
        """根據路徑創建多邊形."""
        if len(poses) < 3:
            return None
        simplify_dist = self.get_parameter('polygon_simplify_dist').value

        simplified_poses = simplify_path(poses, simplify_dist)

        marker = Marker()
        marker.header.frame_id = self.path.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'zones'
        marker.id = self.record_zone_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.a = 0.5
        marker.color.r = 0.6
        marker.color.g = 0.0
        marker.color.b = 1.0

        marker.points = []
        for pose in simplified_poses:
            point = Point()
            point.x = pose.pose.position.x
            point.y = pose.pose.position.y
            point.z = 0.0
            marker.points.append(point)

        if len(simplified_poses) > 2:
            first_pt = simplified_poses[0].pose.position
            last_pt = simplified_poses[-1].pose.position
            dist = math.sqrt(
                (first_pt.x - last_pt.x)**2 + (first_pt.y - last_pt.y)**2)

            if dist < 1.0:
                point = Point()
                point.x = first_pt.x
                point.y = first_pt.y
                point.z = 0.0
                marker.points.append(point)

        return marker

    def create_risk_polygon_from_path(self, poses):
        """為風險區域創建多邊形."""
        if len(poses) < 3:
            return None
        simplify_dist = self.get_parameter('polygon_simplify_dist').value

        simplified_poses = simplify_path(poses, simplify_dist)

        marker = Marker()
        marker.header.frame_id = self.risk_path.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'risk_zones'
        marker.id = self.risk_zone_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color.a = 0.8
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        marker.points = []
        for pose in simplified_poses:
            point = Point()
            point.x = pose.pose.position.x
            point.y = pose.pose.position.y
            point.z = 0.0
            marker.points.append(point)

        if len(simplified_poses) > 2:
            first_pt = simplified_poses[0].pose.position
            last_pt = simplified_poses[-1].pose.position
            dist = math.sqrt(
                (first_pt.x - last_pt.x)**2 + (first_pt.y - last_pt.y)**2)

            if dist < 1.0:
                point = Point()
                point.x = first_pt.x
                point.y = first_pt.y
                point.z = 0.0
                marker.points.append(point)

        return marker

    def publish_path_timer(self):
        """定時發布trace path."""
        if self.record_zone_status:
            if not self.initialized:
                return

            robot_pos = self.get_robot_pos()
            now = self.get_clock().now()

            if robot_pos is None:
                return

            dt = (now - rclpy.time.Time.from_msg(
                self.last_robot_pos.header.stamp)).nanoseconds * 1e-9
            dx = robot_pos.pose.position.x - \
                self.last_robot_pos.pose.position.x
            dy = robot_pos.pose.position.y - \
                self.last_robot_pos.pose.position.y

            if (math.hypot(dx, dy) >= self.get_parameter('min_dist').value
                    and dt >= self.get_parameter('min_dt').value):
                self.path.poses.append(robot_pos)
                self.last_pt = robot_pos.pose.position
                self.last_robot_pos = robot_pos
                self.path.header.stamp = self.get_clock().now().to_msg()

                self.path_pub.publish(self.path)

                if len(self.path.poses) >= 3:
                    self.record_zone_marker = (
                        self.create_polygon_from_path(self.path.poses))
                    if self.record_zone_marker is not None:
                        self.zone_marker_pub.publish(self.record_zone_marker)

        if self.risk_zone_status:
            if not self.initialized:
                return

            robot_pos = self.get_robot_pos()
            now = self.get_clock().now()

            if robot_pos is None:
                return

            if self.last_robot_pos is None:
                self.last_robot_pos = robot_pos
                return

            dt = (now - rclpy.time.Time.from_msg(
                self.last_robot_pos.header.stamp)).nanoseconds * 1e-9
            dx = robot_pos.pose.position.x - \
                self.last_robot_pos.pose.position.x
            dy = robot_pos.pose.position.y - \
                self.last_robot_pos.pose.position.y

            if (math.hypot(dx, dy) >= self.get_parameter('min_dist').value
                    and dt >= self.get_parameter('min_dt').value):
                self.risk_path.poses.append(robot_pos)
                self.last_robot_pos = robot_pos
                self.risk_path.header.stamp = self.get_clock().now().to_msg()

                if len(self.risk_path.poses) >= 3:
                    self.risk_zone_marker = (
                        self.create_risk_polygon_from_path(
                            self.risk_path.poses))
                    if self.risk_zone_marker is not None:
                        self.risk_zone_marker_pub.publish(
                            self.risk_zone_marker)

        if self.chennal_record_status:
            if not self.initialized:
                return

            robot_pos = self.get_robot_pos()
            now = self.get_clock().now()

            if robot_pos is None:
                return

            if self.last_robot_pos is None:
                self.last_robot_pos = robot_pos
                return

            dt = (now - rclpy.time.Time.from_msg(
                self.last_robot_pos.header.stamp)).nanoseconds * 1e-9
            dx = robot_pos.pose.position.x - \
                self.last_robot_pos.pose.position.x
            dy = robot_pos.pose.position.y - \
                self.last_robot_pos.pose.position.y

            if (math.hypot(dx, dy) >= self.get_parameter('min_dist').value
                    and dt >= self.get_parameter('min_dt').value):
                self.chennal_path.poses.append(robot_pos)
                self.last_robot_pos = robot_pos
                self.chennal_path.header.stamp = (
                    self.get_clock().now().to_msg())
                self.chennal_path_pub.publish(self.chennal_path)
                self.channel_path_pub.publish(self.chennal_path)

    def record_zone_start_srv(self, req, res):
        """記錄區域起始點."""
        if self.risk_zone_status:
            self.get_logger().warn('無法開始記錄普通區域：目前正在記錄風險區域')
            res.success = False
            res.message = '無法開始記錄普通區域：目前正在記錄風險區域，請先結束風險區域記錄'
            return res

        self.get_logger().info('記錄區域起始點')
        self.record_zone_status = True
        self.record_zone_id += 1
        self.record_zone_name = 'zone_' + str(self.record_zone_id)

        self.path = Path()
        self.path.header.frame_id = self.get_parameter('frame_id').value

        robot_pos = self.get_robot_pos()
        if robot_pos is not None:
            self.last_robot_pos = robot_pos

        self.record_zone_marker = Marker()
        self.zone_marker_pub.publish(self.record_zone_marker)

        res.success = True
        res.message = '成功記錄區域起始點'
        return res

    def record_zone_end_srv(self, req, res):
        """記錄區域結束點."""
        self.get_logger().info('記錄區域結束點')
        self.record_zone_status = False
        self.record_zone_list.markers.append(self.record_zone_marker)
        self.record_zone_marker = Marker()

        self.zone_list_pub.publish(self.record_zone_list)

        res.success = True
        res.message = f'成功記錄區域結束點 #{len(self.record_zone_list.markers)}'
        return res

    def save_zone_list_srv(self, req, res):
        """合并的区域列表保存服务."""
        success_count = 0
        error_messages = []

        if self._save_zone_list():
            success_count += 1
        else:
            error_messages.append('储存普通区域列表失败')

        if self._save_risk_zone_list():
            success_count += 1
        else:
            error_messages.append('储存风险区域列表失败')

        if self._save_chennal_path_list():
            success_count += 1
        else:
            error_messages.append('储存 channel 路径列表失败')

        if success_count == 3:
            res.success = True
            res.message = '成功储存所有列表（普通区域 + 风险区域 + channel 路径）'
        elif success_count > 0:
            res.success = True
            res.message = f'部分成功储存列表。错误: {"; ".join(error_messages)}'
        else:
            res.success = False
            res.message = f'储存列表失败: {"; ".join(error_messages)}'

        return res

    def load_zone_list_srv(self, req, res):
        """合并的区域列表加载服务."""
        success_count = 0
        error_messages = []

        if self._load_zone_list():
            success_count += 1
            self.zone_list_pub.publish(self.record_zone_list)
        else:
            error_messages.append('載入普通區域列表失敗')

        if self._load_risk_zone_list():
            success_count += 1
            self.risk_zone_list_pub.publish(self.risk_zone_list)
        else:
            error_messages.append('載入風險區域列表失敗')

        if self._load_chennal_path_list():
            success_count += 1
            self.chennal_path_array_pub.publish(self.chennal_path_array)
            self.channel_path_array_pub.publish(self.chennal_path_array)
        else:
            error_messages.append('載入 channel 路徑列表失敗')

        if success_count == 3:
            res.success = True
            res.message = '成功載入所有列表（普通區域 + 風險區域 + channel 路徑）'
        elif success_count > 0:
            res.success = True
            res.message = f'部分成功載入列表。错误: {"; ".join(error_messages)}'
        else:
            res.success = False
            res.message = f'載入列表失敗: {"; ".join(error_messages)}'

        return res

    def risk_zone_start_srv(self, req, res):
        """風險區域開始記錄服務."""
        if self.record_zone_status:
            self.get_logger().warn('無法開始記錄風險區域：目前正在記錄普通區域')
            res.success = False
            res.message = '無法開始記錄風險區域：目前正在記錄普通區域，請先結束普通區域記錄'
            return res

        self.get_logger().info('開始記錄風險區域')
        self.risk_zone_status = True
        self.risk_zone_id += 1
        self.risk_zone_name = 'risk_zone_' + str(self.risk_zone_id)

        self.risk_path = Path()
        self.risk_path.header.frame_id = self.get_parameter('frame_id').value

        robot_pos = self.get_robot_pos()
        if robot_pos is not None:
            self.last_robot_pos = robot_pos

        self.risk_zone_marker = Marker()
        self.risk_zone_marker_pub.publish(self.risk_zone_marker)

        res.success = True
        res.message = '成功開始記錄風險區域'
        return res

    def risk_zone_end_srv(self, req, res):
        """風險區域結束記錄服務."""
        self.get_logger().info('結束記錄風險區域')
        self.risk_zone_status = False

        if hasattr(self, 'risk_zone_marker') and self.risk_zone_marker.points:
            self.risk_zone_list.markers.append(self.risk_zone_marker)
            self.risk_zone_list_pub.publish(self.risk_zone_list)

        self.risk_zone_marker = Marker()

        res.success = True
        res.message = (
            f'成功結束記錄風險區域 #{len(self.risk_zone_list.markers)}')
        return res

    def risk_zone_save_srv(self, req, res):
        """風險區域儲存服務."""
        if self._save_risk_zone_list():
            res.success = True
            res.message = '成功儲存風險區域'
        else:
            res.success = False
            res.message = '儲存風險區域失敗'
        return res

    def risk_zone_load_srv(self, req, res):
        """風險區域載入服務."""
        if self._load_risk_zone_list():
            self.risk_zone_list_pub.publish(self.risk_zone_list)
            res.success = True
            res.message = '成功載入風險區域'
        else:
            res.success = False
            res.message = '載入風險區域失敗'
        return res

    def get_record_zone_info_srv(self, req, res):
        """獲取普通區域列表資訊."""
        zone_ids = [marker.id for marker in self.record_zone_list.markers]
        self.get_logger().info(f'record_zone_list marker ids: {zone_ids}')
        res.success = True
        res.message = '返回test'
        return res

    def get_record_zone_list_srv(self, req, res):
        """獲取當前記錄的普通區域列表."""
        try:
            res.success = True
            res.message = (
                f'成功獲取普通區域列表，共 '
                f'{len(self.record_zone_list.markers)} 個區域')
            res.zone_list = self.record_zone_list
            return res
        except Exception as e:
            self.get_logger().error(f'獲取普通區域列表失敗: {e}')
            res.success = False
            res.message = f'獲取普通區域列表失敗: {str(e)}'
            res.zone_list = MarkerArray()
            return res

    def get_risk_zone_list_srv(self, req, res):
        """獲取當前記錄的風險區域列表."""
        try:
            res.success = True
            res.message = (
                f'成功獲取風險區域列表，共 '
                f'{len(self.risk_zone_list.markers)} 個風險區域')
            res.zone_list = self.risk_zone_list
            return res
        except Exception as e:
            self.get_logger().error(f'獲取風險區域列表失敗: {e}')
            res.success = False
            res.message = f'獲取風險區域列表失敗: {str(e)}'
            res.zone_list = MarkerArray()
            return res

    def _save_risk_zone_list(self):
        """儲存風險區域列表."""
        try:
            risk_zones_data = []
            for marker in self.risk_zone_list.markers:
                marker_data = {
                    'id': marker.id,
                    'ns': marker.ns,
                    'points': [[p.x, p.y, p.z] for p in marker.points]
                }
                risk_zones_data.append(marker_data)

            with open(self.get_parameter('save_dir').value +
                      '/risk_zone_list.json', 'w') as f:
                json.dump(risk_zones_data, f, indent=2)
            return True
        except Exception as e:
            self.get_logger().error(f'儲存風險區域列表失敗: {e}')
            return False

    def _load_risk_zone_list(self):
        """載入風險區域列表."""
        try:
            with open(self.get_parameter('save_dir').value +
                      '/risk_zone_list.json', 'r') as f:
                risk_zones_data = json.load(f)

            self.risk_zone_list = MarkerArray()
            for marker_data in risk_zones_data:
                marker = Marker()
                marker.header.frame_id = self.get_parameter('frame_id').value
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = marker_data['ns']
                marker.id = marker_data['id']
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.03
                marker.color.a = 0.8
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

                marker.points = []
                for point_data in marker_data['points']:
                    point = Point()
                    point.x = point_data[0]
                    point.y = point_data[1]
                    point.z = point_data[2]
                    marker.points.append(point)

                self.risk_zone_list.markers.append(marker)

            return True
        except Exception as e:
            self.get_logger().error(f'載入風險區域列表失敗: {e}')
            return False

    def _save_zone_list(self):
        """儲存普通區域列表."""
        try:
            zones_data = []
            for marker in self.record_zone_list.markers:
                marker_data = {
                    'id': marker.id,
                    'ns': marker.ns,
                    'points': [[p.x, p.y, p.z] for p in marker.points]
                }
                zones_data.append(marker_data)

            with open(self.get_parameter('save_dir').value +
                      '/zone_list.json', 'w') as f:
                json.dump(zones_data, f, indent=2)
            return True
        except Exception as e:
            self.get_logger().error(f'儲存區域列表失敗: {e}')
            return False

    def _load_zone_list(self):
        """載入普通區域列表."""
        try:
            with open(self.get_parameter('save_dir').value +
                      '/zone_list.json', 'r') as f:
                zones_data = json.load(f)

            self.record_zone_list = MarkerArray()
            for marker_data in zones_data:
                marker = Marker()
                marker.header.frame_id = self.get_parameter('frame_id').value
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = marker_data['ns']
                marker.id = marker_data['id']
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.02
                marker.color.a = 0.5
                marker.color.r = 0.6
                marker.color.g = 0.0
                marker.color.b = 1.0

                marker.points = []
                for point_data in marker_data['points']:
                    point = Point()
                    point.x = point_data[0]
                    point.y = point_data[1]
                    point.z = point_data[2]
                    marker.points.append(point)

                self.record_zone_list.markers.append(marker)

            return True
        except Exception as e:
            self.get_logger().error(f'載入區域列表失敗: {e}')
            return False

    def chennal_record_start_srv(self, req, res):
        """開始記錄 chennal 路徑."""
        self.get_logger().info('開始記錄 chennal 路徑')

        self.chennal_record_status = True
        self.chennal_record_id += 1
        self.chennal_record_name = 'chennal_record_' + \
            str(self.chennal_record_id)

        self.chennal_path = Path()
        self.chennal_path.header.frame_id = self.get_parameter(
            'frame_id').value
        self.chennal_path.header.stamp = self.get_clock().now().to_msg()

        robot_pos = self.get_robot_pos()
        if robot_pos is not None:
            self.last_robot_pos = robot_pos
            self.chennal_path.poses.append(robot_pos)
            self.chennal_path_pub.publish(self.chennal_path)

        res.success = True
        res.message = f'成功開始記錄 chennal 路徑 #{self.chennal_record_id}'
        return res

    def chennal_record_end_srv(self, req, res):
        """結束記錄 chennal 路徑並添加到路徑列表中."""
        self.get_logger().info('結束記錄 chennal 路徑')

        if not self.chennal_record_status:
            res.success = False
            res.message = '沒有正在進行的 chennal 路徑記錄'
            return res

        self.chennal_record_status = False

        robot_pos = self.get_robot_pos()
        if robot_pos is not None and len(self.chennal_path.poses) > 0:
            last_pose = self.chennal_path.poses[-1]
            dx = robot_pos.pose.position.x - last_pose.pose.position.x
            dy = robot_pos.pose.position.y - last_pose.pose.position.y
            if math.hypot(dx, dy) >= self.get_parameter('min_dist').value:
                self.chennal_path.poses.append(robot_pos)

        if len(self.chennal_path.poses) > 0:
            self.chennal_path_array.markers.append(
                path_to_marker(self.chennal_path,
                               ns='chennal_path',
                               marker_id=self.chennal_record_id,
                               color=(0.0, 1.0, 0.0),
                               scale=0.1))
            self.chennal_path_array_pub.publish(self.chennal_path_array)
            self.channel_path_array_pub.publish(self.chennal_path_array)
        res.success = True
        res.message = '成功結束記錄 chennal 路徑'
        return res

    def get_channel_path_list_srv(self, req, res):
        """獲取 channel 路徑列表."""
        res.success = True
        res.message = '成功獲取 channel 路徑列表'
        res.channel_path_array = self.chennal_path_array
        return res

    def get_chennal_path_list_srv(self, req, res):
        """獲取 chennal 路徑列表."""
        res.success = True
        res.message = '成功獲取 chennal 路徑列表'
        res.chennal_path_array = self.chennal_path_array
        return res

    # ── Channel Router ────────────────────────────────────────────────────────

    def get_channel_route_srv(self, req, res):
        """返回連接兩個 zone 的通道路徑，方向保證從 zone_from 走向 zone_to."""
        zone_from_id = req.zone_from_id
        zone_to_id = req.zone_to_id
        proximity_m = float(req.proximity_m) if req.proximity_m > 0.0 else 1.5
        self.get_logger().info(
            f'get_channel_route: zone_from={zone_from_id}, '
            f'zone_to={zone_to_id}, proximity={proximity_m:.2f}m'
        )

        if not self.chennal_path_array.markers:
            res.success = False
            res.message = '尚無通道數據，請先錄製或載入通道路徑'
            res.channel_path = Path()
            res.matched_channel_id = -1
            return res

        result = self._find_channel_for_zones(
            zone_from_id, zone_to_id, proximity_m
        )
        if result is None:
            res.success = False
            res.message = (
                f'找不到連接 zone {zone_from_id} → zone {zone_to_id} 的通道，'
                f'共搜尋 {len(self.chennal_path_array.markers)} 條通道'
            )
            res.channel_path = Path()
            res.matched_channel_id = -1
            return res

        channel_path, channel_id = result
        res.success = True
        res.message = (
            f'找到通道 #{channel_id}，'
            f'連接 zone {zone_from_id} → zone {zone_to_id}，'
            f'共 {len(channel_path.poses)} 個路徑點'
        )
        res.channel_path = channel_path
        res.matched_channel_id = channel_id
        return res

    def _find_channel_for_zones(self, zone_from_id, zone_to_id, proximity_m=1.5):
        """
        搜尋 chennal_path_array 中連接兩個 zone 的通道.

        判斷邏輯：通道起點/終點需落在對應 zone 內部，
        或距其邊界 proximity_m 以內。
        返回 (Path, channel_id) 或 None。
        路徑方向保證：從 zone_from 端出發走向 zone_to 端。
        """
        zone_from_pts = self._get_zone_polygon(zone_from_id)
        zone_to_pts = self._get_zone_polygon(zone_to_id)

        if zone_from_pts is None:
            self.get_logger().error(
                f'找不到 zone {zone_from_id} 的多邊形數據'
            )
            return None
        if zone_to_pts is None:
            self.get_logger().error(
                f'找不到 zone {zone_to_id} 的多邊形數據'
            )
            return None

        for marker in self.chennal_path_array.markers:
            if len(marker.points) < 2:
                continue

            start = marker.points[0]
            end = marker.points[-1]

            start_near_from = self._point_near_zone(
                start.x, start.y, zone_from_pts, proximity_m
            )
            end_near_to = self._point_near_zone(
                end.x, end.y, zone_to_pts, proximity_m
            )
            if start_near_from and end_near_to:
                self.get_logger().info(
                    f'通道 #{marker.id}: start 近 zone {zone_from_id}，'
                    f'end 近 zone {zone_to_id}，正向匹配'
                )
                return self._channel_marker_to_path(marker), marker.id

            start_near_to = self._point_near_zone(
                start.x, start.y, zone_to_pts, proximity_m
            )
            end_near_from = self._point_near_zone(
                end.x, end.y, zone_from_pts, proximity_m
            )
            if start_near_to and end_near_from:
                self.get_logger().info(
                    f'通道 #{marker.id}: start 近 zone {zone_to_id}，'
                    f'end 近 zone {zone_from_id}，反向匹配，翻轉路徑'
                )
                return self._channel_marker_to_path(marker, reverse=True), marker.id

        return None

    def _get_zone_polygon(self, zone_id):
        """從 record_zone_list 取得指定 zone_id 的多邊形點列表."""
        for marker in self.record_zone_list.markers:
            if marker.id == zone_id:
                return [(p.x, p.y) for p in marker.points]
        return None

    def _point_near_zone(self, x, y, polygon_pts, proximity_m):
        """
        判斷點是否在 zone 內部或距邊界 proximity_m 以內.
        """
        if self._point_in_polygon(x, y, polygon_pts):
            return True
        return self._min_dist_to_polygon(x, y, polygon_pts) <= proximity_m

    @staticmethod
    def _point_in_polygon(x, y, polygon_pts):
        """Ray-casting point-in-polygon test."""
        n = len(polygon_pts)
        if n < 3:
            return False
        inside = False
        j = n - 1
        for i in range(n):
            xi, yi = polygon_pts[i]
            xj, yj = polygon_pts[j]
            if ((yi > y) != (yj > y) and
                    x < (xj - xi) * (y - yi) / (yj - yi) + xi):
                inside = not inside
            j = i
        return inside

    @staticmethod
    def _min_dist_to_polygon(x, y, polygon_pts):
        """返回點到多邊形各邊的最小距離."""
        min_dist = float('inf')
        n = len(polygon_pts)
        for i in range(n):
            x1, y1 = polygon_pts[i]
            x2, y2 = polygon_pts[(i + 1) % n]
            dx, dy = x2 - x1, y2 - y1
            seg_len_sq = dx * dx + dy * dy
            if seg_len_sq == 0:
                d = math.hypot(x - x1, y - y1)
            else:
                t = max(0.0, min(
                    1.0, ((x - x1) * dx + (y - y1) * dy) / seg_len_sq
                ))
                d = math.hypot(x - (x1 + t * dx), y - (y1 + t * dy))
            if d < min_dist:
                min_dist = d
        return min_dist

    def _channel_marker_to_path(self, marker, reverse=False):
        """將通道 Marker (LINE_STRIP) 轉換為帶航向角的 nav_msgs/Path."""
        path = Path()
        path.header.frame_id = self.get_parameter('frame_id').value
        path.header.stamp = self.get_clock().now().to_msg()

        points = (
            list(reversed(marker.points)) if reverse else list(marker.points)
        )
        n = len(points)
        for i, pt in enumerate(points):
            pose = PoseStamped()
            pose.header.frame_id = path.header.frame_id
            pose.header.stamp = path.header.stamp
            pose.pose.position.x = pt.x
            pose.pose.position.y = pt.y
            pose.pose.position.z = 0.0

            if i < n - 1:
                dx = points[i + 1].x - pt.x
                dy = points[i + 1].y - pt.y
            elif i > 0:
                dx = pt.x - points[i - 1].x
                dy = pt.y - points[i - 1].y
            else:
                dx, dy = 1.0, 0.0

            yaw = math.atan2(dy, dx)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)

        return path

    def _save_chennal_path_list(self):
        """内部方法：保存 chennal 路径列表."""
        try:
            chennal_paths_data = []
            for marker in self.chennal_path_array.markers:
                marker_data = {
                    'id': marker.id,
                    'ns': marker.ns,
                    'points': [[p.x, p.y, p.z] for p in marker.points],
                    'color': {
                        'r': marker.color.r,
                        'g': marker.color.g,
                        'b': marker.color.b,
                        'a': marker.color.a
                    },
                    'scale': marker.scale.x
                }
                chennal_paths_data.append(marker_data)

            save_path = (self.get_parameter('save_dir').value +
                         '/chennal_path_list.json')
            with open(save_path, 'w') as f:
                json.dump(chennal_paths_data, f, indent=2)

            return True
        except Exception as e:
            self.get_logger().error(f'保存 chennal 路径列表失败: {e}')
            return False

    def _load_chennal_path_list(self):
        """内部方法：加载 chennal 路径列表."""
        try:
            save_path = (self.get_parameter('save_dir').value +
                         '/chennal_path_list.json')
            with open(save_path, 'r') as f:
                chennal_paths_data = json.load(f)

            self.chennal_path_array = MarkerArray()
            for marker_data in chennal_paths_data:
                marker = Marker()
                marker.header.frame_id = self.get_parameter('frame_id').value
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = marker_data['ns']
                marker.id = marker_data['id']
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = marker_data.get('scale', 0.1)

                color_data = marker_data.get(
                    'color', {'r': 0.0, 'g': 1.0, 'b': 0.0, 'a': 0.8})
                marker.color.r = color_data['r']
                marker.color.g = color_data['g']
                marker.color.b = color_data['b']
                marker.color.a = color_data['a']

                marker.points = []
                for point_data in marker_data['points']:
                    point = Point()
                    point.x = point_data[0]
                    point.y = point_data[1]
                    point.z = point_data[2]
                    marker.points.append(point)

                self.chennal_path_array.markers.append(marker)

            return True
        except Exception as e:
            self.get_logger().error(f'加载 chennal 路径列表失败: {e}')
            return False


def main():
    rclpy.init()
    node = PathRecorder()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
