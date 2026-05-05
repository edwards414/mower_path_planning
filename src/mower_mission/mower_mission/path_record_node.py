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

from mower_interface.srv import GetZoneList

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
            ChennalPathList, '/get_chennal_path_list',
            self.get_chennal_path_list_srv)

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
            error_messages.append('储存 chennal 路径列表失败')

        if success_count == 3:
            res.success = True
            res.message = '成功储存所有列表（普通区域 + 风险区域 + chennal 路径）'
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

        if success_count == 2:
            res.success = True
            res.message = '成功載入所有區域列表（普通區域 + 風險區域）'
        elif success_count == 1:
            res.success = True
            res.message = f'部分成功載入區域列表。错误: {"; ".join(error_messages)}'
        else:
            res.success = False
            res.message = f'載入區域列表失敗: {"; ".join(error_messages)}'

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
        res.success = True
        res.message = '成功結束記錄 chennal 路徑'
        return res

    def get_chennal_path_list_srv(self, req, res):
        """獲取 chennal 路徑列表."""
        res.success = True
        res.message = '成功獲取 chennal 路徑列表'
        res.chennal_path_array = self.chennal_path_array
        return res

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
