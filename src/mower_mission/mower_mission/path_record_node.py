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

import copy
import json

import math

import os

import time

from mower_interface.srv import ChannelPathList, ChannelRoute, EditZone, \
    GetZoneList, SiteOp

from geometry_msgs.msg import Point, PoseStamped

from nav_msgs.msg import Odometry, Path

from mower_interface.srv import ChennalPathList

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from std_msgs.msg import String

from std_srvs.srv import Trigger


from visualization_msgs.msg import Marker, MarkerArray

from .utils import site_store
from .navigation_guard import (
    NavigationActivityGuard,
    guarded_mission_mutation,
)

from .utils.path_record_utils import (
    path_to_marker, rear_light_update, simplify_path)


def _write_json_atomic(path, payload):
    """Replace a JSON snapshot atomically so power loss cannot truncate it."""
    tmp_path = path + '.tmp'
    with open(tmp_path, 'w') as file_obj:
        json.dump(payload, file_obj, indent=2)
        file_obj.flush()
        os.fsync(file_obj.fileno())
    os.replace(tmp_path, path)
    directory = os.path.dirname(path) or '.'
    directory_fd = os.open(
        directory,
        os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0),
    )
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)


class PathRecorder(Node, NavigationActivityGuard):

    def __init__(self):
        super().__init__('path_recorder')

        self.declare_parameter('odom_topic', '/odom')
        # map -> base_footprint source. The EKF map filter publishes exactly
        # this pose on /odometry/global; mission.launch.py points the node at
        # the 5 Hz throttled copy. A tf2 TransformListener would instead cost
        # the full 77 Hz /tf stream (~50 % of a LubanCat core in rclpy).
        self.declare_parameter('robot_pose_source_topic', '/odometry/global')
        self.declare_parameter('robot_pose_max_age_s', 1.0)
        self.declare_parameter('min_dist', 0.05)
        self.declare_parameter('min_dt', 0.10)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('save_dir', 'zone_record')
        self.declare_parameter('sites_dir', '~/.mower/sites')
        self.declare_parameter('max_site_datum_distance_m', 100.0)
        self.declare_parameter('polygon_simplify_dist', 0.1)
        # Red breathing rear light while a zone / risk zone / channel is being
        # recorded (mower_hardware driver -> STM32 0x03 overlay); '' disables.
        self.declare_parameter('rear_light_topic', '/mower_base/rear_light')
        self.declare_parameter('rear_light_period_s', 2.0)
        self.get_logger().info('path_recorder ready.')

        os.makedirs(self.get_parameter('save_dir').value, exist_ok=True)

        polygon_qos = QoSProfile(depth=1)
        polygon_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        polygon_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self._init_navigation_activity_guard()

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
        # Transient-local like the driver's subscription (a volatile writer
        # would not match it). Re-sent by the sampler while recording: the
        # driver drops the overlay after ~6 s without a refresh, so a dead
        # recorder cannot leave the robot claiming it is still recording.
        rear_topic = self.get_parameter('rear_light_topic').value
        self.rear_light_pub = (
            self.create_publisher(String, rear_topic, polygon_qos)
            if rear_topic else None)
        self._rear_light_on = False
        self._rear_light_sent = 0.0

        # The 10 Hz trace-path sampler only runs while a zone / risk zone /
        # channel recording is active: the start services re-arm it and the
        # callback cancels it again when nothing is being recorded. Idle, an
        # rclpy timer wake-up costs ~6 ms on the LubanCat, i.e. ~6 % of a
        # core for a callback that did nothing.
        self.timer_period = 0.1
        self.timer = self.create_timer(
            self.timer_period, self.publish_path_timer)
        self.timer.cancel()

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

        # 取消目前進行中的記錄（不收尾、不加入清單）。App 的「取消」按鈕用這個，
        # 因為 *_end 服務一律會把多邊形 append 進清單，沒有丟棄的途徑。
        self.create_service(
            Trigger, '/record_cancel', self.record_cancel_srv)

        # App 直接編輯物件：新增（用 app 畫的多邊形）/ 刪除（依 id）/ 更新（依 id 換頂點）。
        self.create_service(EditZone, '/edit_zone', self.edit_zone_srv)

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

        # 場地庫：把目前的 zone/risk/channel 集合以 WGS84 存成具名場地，
        # 下次（map frame 重新原點化之後）載入時再投影回當下座標系。
        self.create_service(SiteOp, '/site_op', self.site_op_srv)
        self.site_list_pub = self.create_publisher(
            String, '/site_list', polygon_qos)
        self.map_datum = None
        self.active_site = None
        self._active_manifest_blocked_reason = None
        self._working_state_ready = False
        self.create_subscription(
            String, '/adapter/map_datum', self._on_map_datum, polygon_qos)

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

        self._robot_pose_source = None
        self._robot_pose_unavailable_logged = False
        self.create_subscription(
            Odometry,
            str(self.get_parameter('robot_pose_source_topic').value),
            self._on_robot_pose_source,
            10,
        )

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

        self._restore_startup_persistence()
        self._publish_site_list()

    def try_initialize(self):
        def clear_inprogress_markers():
            """清除 RViz 上殘留的錄製中多邊形（/zone_markers, /risk_zone_markers）."""
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

            self.get_logger().info(
                '✅ 已清空 RViz zone_markers 與 risk_zone_markers')

        if not self.initialized:
            robot_pos = self.get_robot_pos()
            if robot_pos is not None:
                self.last_robot_pos = robot_pos
                self.initialized = True

                clear_inprogress_markers()

                # 若 TF 可用前已有 load（場地或 zone_record），不可清掉已載入的清單。
                if not (self.record_zone_list.markers
                        or self.risk_zone_list.markers
                        or self.chennal_path_array.markers):
                    self.record_zone_list = MarkerArray()
                    self.risk_zone_list = MarkerArray()

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
                self.get_logger().warn('等待機器人位置（map 座標）以完成初始化...')

    def _on_robot_pose_source(self, odom: Odometry) -> None:
        """Cache the latest map-frame pose sample (see robot_pose_source_topic)."""
        if (odom.header.frame_id != self.get_parameter('frame_id').value
                or odom.child_frame_id != 'base_footprint'):
            return
        position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        values = (position.x, position.y, position.z,
                  orientation.x, orientation.y, orientation.z, orientation.w)
        if not all(math.isfinite(float(value)) for value in values):
            return
        if abs(math.sqrt(sum(float(v) ** 2 for v in values[3:])) - 1.0) > 1e-2:
            return
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self._robot_pose_source = pose

    def get_robot_pos(self):
        """回傳最新的 map 座標系機器人位置（過期或尚未收到時回傳 None）."""
        pose = self._robot_pose_source
        if pose is not None:
            age_s = (self.get_clock().now()
                     - rclpy.time.Time.from_msg(pose.header.stamp)
                     ).nanoseconds * 1e-9
            max_age_s = max(
                0.1, float(self.get_parameter('robot_pose_max_age_s').value))
            if -0.5 <= age_s <= max_age_s:
                self._robot_pose_unavailable_logged = False
                return copy.deepcopy(pose)
        if not self._robot_pose_unavailable_logged:
            self._robot_pose_unavailable_logged = True
            self.get_logger().warn(
                '機器人位置不可用: '
                f'{self.get_parameter("robot_pose_source_topic").value} '
                '尚未收到或已過期')
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

    def _arm_record_timer(self):
        """Start sampling the robot pose (called when a recording starts)."""
        self.timer.reset()

    def publish_path_timer(self):
        """定時發布trace path."""
        recording = (self.record_zone_status or self.risk_zone_status
                     or self.chennal_record_status)
        self._update_rear_light(recording)
        if not recording:
            self.timer.cancel()
            return
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

    def _update_rear_light(self, recording):
        if self.rear_light_pub is None:
            return
        now = time.monotonic()
        effect = rear_light_update(
            recording, self._rear_light_on, now - self._rear_light_sent,
            self.get_parameter('rear_light_period_s').value)
        if effect is None:
            return
        self.rear_light_pub.publish(
            String(data=json.dumps(
                {'effect': effect, 'source': 'path_record'})))
        self._rear_light_on = effect == 'recording'
        self._rear_light_sent = now

    def _active_recording_kind(self):
        """Return the one active recorder mode, if any."""
        if self.record_zone_status:
            return 'zone'
        if self.risk_zone_status:
            return 'risk'
        if self.chennal_record_status:
            return 'channel'
        return None

    def _reject_start_while_recording(self, res, requested_kind):
        active_kind = self._active_recording_kind()
        if active_kind is None:
            if self._reject_blocked_manifest(res, f'開始 {requested_kind} 錄製'):
                return res
            if self._acquire_mission_mutation(
                res,
                f'record {requested_kind} geometry',
            ):
                # Recording may describe a new field. Detach the prior named
                # site durably so a later edit cannot overwrite that site with
                # this newly recorded working geometry by accident.
                if self.active_site is not None:
                    try:
                        site_store.clear_active_site(self._sites_dir())
                        self.active_site = None
                    except Exception as exc:  # noqa: BLE001
                        res.success = False
                        res.message = (
                            '無法清除舊場地關聯，拒絕開始錄製: '
                            f'{exc}'
                        )
                        if not self._release_mission_mutation():
                            res.message += (
                                '；操作鎖釋放也未確認，導航仍被禁止；'
                                '請檢查機器後重啟 path_record_node 與 '
                                'nav_action_server'
                            )
                        return res
                    try:
                        self._publish_site_list()
                    except Exception as exc:  # noqa: BLE001
                        self.get_logger().warn(
                            f'舊場地關聯已清除，但清單發佈失敗: {exc}'
                        )
                return None
            return res
        res.success = False
        res.message = (
            f'無法開始 {requested_kind} 記錄：'
            f'目前正在進行 {active_kind} 記錄'
        )
        self.get_logger().warn(res.message)
        return res

    def record_zone_start_srv(self, req, res):
        """記錄區域起始點."""
        rejected = self._reject_start_while_recording(res, 'zone')
        if rejected is not None:
            return rejected

        self.get_logger().info('記錄區域起始點')
        self.record_zone_status = True
        self._arm_record_timer()
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
        if not self.record_zone_status:
            res.success = False
            res.message = '沒有正在進行的 zone 記錄'
            return res
        if (self.record_zone_marker is None
                or len(self.record_zone_marker.points) < 3):
            res.success = False
            res.message = '區域點數不足，至少需要 3 個點'
            return res

        self.get_logger().info('記錄區域結束點')
        self.record_zone_list.markers.append(self.record_zone_marker)
        if not self._save_zone_list():
            self.record_zone_list.markers.pop()
            res.success = False
            res.message = (
                '區域已收尾但持久化失敗；錄製仍保留，'
                '請檢查磁碟後重試結束'
            )
            self.get_logger().error(res.message)
            return res

        self.record_zone_status = False
        self.record_zone_marker = Marker()
        self.path = Path()
        self.path.header.frame_id = self.get_parameter('frame_id').value
        self.last_robot_pos = None
        self._working_state_ready = True

        self.zone_list_pub.publish(self.record_zone_list)
        release_confirmed = self._release_mission_mutation()

        res.success = release_confirmed
        res.message = f'成功記錄區域結束點 #{len(self.record_zone_list.markers)}'
        if not release_confirmed:
            res.message += (
                '；資料已儲存，但操作鎖釋放未確認，導航仍被禁止；'
                '請檢查機器後重啟 path_record_node 與 nav_action_server'
            )
        return res

    def record_cancel_srv(self, req, res):
        """取消目前進行中的記錄：停止取樣、清空 in-progress 路徑，不加入清單。"""
        frame_id = self.get_parameter('frame_id').value
        cancelled = None

        if self.record_zone_status:
            self.record_zone_status = False
            self.path = Path()
            self.path.header.frame_id = frame_id
            self.record_zone_marker = Marker()
            self.zone_marker_pub.publish(self.record_zone_marker)
            cancelled = 'zone'

        if self.risk_zone_status:
            self.risk_zone_status = False
            self.risk_path = Path()
            self.risk_path.header.frame_id = frame_id
            self.risk_zone_marker = Marker()
            self.risk_zone_marker_pub.publish(self.risk_zone_marker)
            cancelled = 'risk'

        if self.chennal_record_status:
            self.chennal_record_status = False
            self.chennal_path = Path()
            self.chennal_path.header.frame_id = frame_id
            self.chennal_path_pub.publish(self.chennal_path)
            self.channel_path_pub.publish(self.chennal_path)
            cancelled = 'channel'

        self.last_robot_pos = None

        res.success = True
        if cancelled is None:
            res.message = '目前沒有進行中的記錄'
        else:
            res.message = f'已取消 {cancelled} 記錄'
            if not self._release_mission_mutation():
                res.success = False
                res.message += (
                    '；操作鎖釋放未確認，導航仍被禁止；請檢查機器後重啟 '
                    'path_record_node 與 nav_action_server'
                )
        self.get_logger().info(res.message)
        return res

    def _marker_from_xy(self, ns, marker_id, color, scale, pts, closed):
        """以原始 (x, y) 點建立一個 LINE_STRIP marker。"""
        marker = Marker()
        marker.header.frame_id = self.get_parameter('frame_id').value
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = int(marker_id)
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.points = []
        for (x, y) in pts:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.0
            marker.points.append(p)
        if closed and len(pts) >= 3:
            fx, fy = pts[0]
            lx, ly = pts[-1]
            if math.hypot(fx - lx, fy - ly) > 1e-6:
                p = Point()
                p.x = float(fx)
                p.y = float(fy)
                p.z = 0.0
                marker.points.append(p)
        return marker

    @guarded_mission_mutation('edit mission geometry')
    def edit_zone_srv(self, req, res):
        """App 直接編輯物件：新增 / 刪除 / 更新（工作區 / 禁入區 / 通道）。"""
        if self._reject_blocked_manifest(res, '編輯任務幾何'):
            return res
        kind = req.kind
        if kind == 'zone':
            mlist = self.record_zone_list
            pubs = [self.zone_list_pub]
            save = self._save_zone_list
            ns, color, scale, closed = 'zones', (0.6, 0.0, 1.0, 0.5), 0.02, True
        elif kind == 'risk':
            mlist = self.risk_zone_list
            pubs = [self.risk_zone_list_pub]
            save = self._save_risk_zone_list
            ns, color, scale, closed = \
                'risk_zones', (1.0, 0.0, 0.0, 0.8), 0.03, True
        elif kind == 'channel':
            mlist = self.chennal_path_array
            pubs = [self.chennal_path_array_pub, self.channel_path_array_pub]
            save = self._save_chennal_path_list
            ns, color, scale, closed = \
                'channels', (0.0, 0.7, 1.0, 0.8), 0.03, False
        else:
            res.success = False
            res.message = f'未知 kind: {kind}（要 zone/risk/channel）'
            return res

        op = req.op
        pts = [(p.x, p.y) for p in req.points]
        min_pts = 3 if closed else 2
        original_markers = copy.deepcopy(mlist.markers)

        if op == 'delete':
            before = len(mlist.markers)
            mlist.markers = [m for m in mlist.markers if m.id != req.id]
            if len(mlist.markers) == before:
                res.success = False
                res.message = f'找不到 {kind} id={req.id}'
                return res
            res.id = req.id
            res.message = f'已刪除 {kind} id={req.id}'
        elif op == 'add':
            if len(pts) < min_pts:
                res.success = False
                res.message = f'{kind} 頂點不足（{len(pts)} < {min_pts}）'
                return res
            new_id = max((m.id for m in mlist.markers), default=0) + 1
            mlist.markers.append(
                self._marker_from_xy(ns, new_id, color, scale, pts, closed))
            res.id = new_id
            res.message = f'已新增 {kind} id={new_id}'
        elif op == 'update':
            if len(pts) < min_pts:
                res.success = False
                res.message = f'{kind} 頂點不足（{len(pts)} < {min_pts}）'
                return res
            target = next((m for m in mlist.markers if m.id == req.id), None)
            if target is None:
                res.success = False
                res.message = f'找不到 {kind} id={req.id}'
                return res
            rebuilt = self._marker_from_xy(ns, req.id, color, scale, pts, closed)
            target.points = rebuilt.points
            target.header.stamp = self.get_clock().now().to_msg()
            res.id = req.id
            res.message = f'已更新 {kind} id={req.id}'
        else:
            res.success = False
            res.message = f'未知 op: {op}（要 add/delete/update）'
            return res

        try:
            persisted = bool(save())
        except Exception as e:
            persisted = False
            self.get_logger().error(f'edit_zone 存檔失敗: {e}')
        if not persisted:
            mlist.markers = original_markers
            res.success = False
            res.message += '；持久化失敗，變更已回復'
            self.get_logger().error(res.message)
            return res

        if not self._update_active_site():
            mlist.markers = original_markers
            try:
                rollback_persisted = bool(save())
            except Exception as e:
                rollback_persisted = False
                self.get_logger().error(
                    f'edit_zone 工作檔回復失敗: {e}'
                )
            res.success = False
            res.message += '；場地檔同步失敗，變更已回復'
            if not rollback_persisted:
                self._active_manifest_blocked_reason = (
                    '編輯回復後的工作檔狀態無法確認'
                )
                res.message += '（工作檔回復也失敗，請立即檢查磁碟）'
            self.get_logger().error(res.message)
            return res

        # App-created markers choose their id from the current list rather
        # than from the physical recorder's counters.  Keep all three
        # counters aligned before the next manual recording starts, otherwise
        # an App add followed by a recording can create duplicate marker ids.
        self._restore_id_counters()
        for pub in pubs:
            pub.publish(mlist)

        res.success = True
        self.get_logger().info(res.message)
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
            self._working_state_ready = True
        elif success_count > 0:
            res.success = False
            res.message = f'部分成功储存列表。错误: {"; ".join(error_messages)}'
        else:
            res.success = False
            res.message = f'储存列表失败: {"; ".join(error_messages)}'

        # 注意：不同步 active site 檔 — /save_zone_list 在每次錄製結束後都會被
        # app 呼叫，若同步會把「錄新場地」的內容默默寫進舊場地。場地檔只在
        # /site_op save 與 /edit_zone（明確編輯場地物件）時更新。

        return res

    @guarded_mission_mutation('load mission geometry')
    def load_zone_list_srv(self, req, res):
        """合并的区域列表加载服务."""
        if (self.record_zone_status or self.risk_zone_status
                or self.chennal_record_status):
            # 載入會重設 id 計數器，會跟進行中錄製的 marker id 撞號。
            res.success = False
            res.message = '錄製進行中，請先結束或取消錄製再載入'
            return res

        original = (
            copy.deepcopy(self.record_zone_list),
            copy.deepcopy(self.risk_zone_list),
            copy.deepcopy(self.chennal_path_array),
        )
        error_messages = []

        zone_ok = self._load_zone_list()
        if not zone_ok:
            error_messages.append('載入普通區域列表失敗')

        risk_ok = self._load_risk_zone_list()
        if not risk_ok:
            error_messages.append('載入風險區域列表失敗')

        channel_ok = self._load_chennal_path_list()
        if not channel_ok:
            error_messages.append('載入 channel 路徑列表失敗')

        if not (zone_ok and risk_ok and channel_ok):
            (self.record_zone_list, self.risk_zone_list,
             self.chennal_path_array) = original
            res.success = False
            res.message = (
                f'載入列表失敗，既有資料已保留: '
                f'{"; ".join(error_messages)}'
            )
            return res

        self._restore_id_counters()
        self._working_state_ready = True
        self.zone_list_pub.publish(self.record_zone_list)
        self.risk_zone_list_pub.publish(self.risk_zone_list)
        self.chennal_path_array_pub.publish(self.chennal_path_array)
        self.channel_path_array_pub.publish(self.chennal_path_array)
        res.success = True
        res.message = '成功載入所有列表（普通區域 + 風險區域 + channel 路徑）'

        return res

    # ── 場地庫（named sites, WGS84-anchored）───────────────────────────────────

    def _on_map_datum(self, msg):
        try:
            data = json.loads(msg.data)
            datum = {
                'lat': float(data['origin_lat']),
                'lon': float(data['origin_lon']),
                'bearing_rad': float(data.get('bearing_rad', 0.0)),
                'source': data.get('source', ''),
            }
            if (
                not all(math.isfinite(datum[key]) for key in (
                    'lat', 'lon', 'bearing_rad'
                ))
                or not -90.0 <= datum['lat'] <= 90.0
                or not -180.0 <= datum['lon'] <= 180.0
            ):
                raise ValueError('datum contains invalid coordinates')
            self.map_datum = datum
        except (ValueError, KeyError, TypeError) as e:
            self.get_logger().warn(f'解析 /adapter/map_datum 失敗: {e}')

    def _sites_dir(self):
        return os.path.expanduser(self.get_parameter('sites_dir').value)

    def _working_file_paths(self):
        save_dir = self.get_parameter('save_dir').value
        return (
            os.path.join(save_dir, 'zone_list.json'),
            os.path.join(save_dir, 'risk_zone_list.json'),
            os.path.join(save_dir, 'chennal_path_list.json'),
        )

    def _publish_geometry_lists(self):
        self.zone_list_pub.publish(self.record_zone_list)
        self.risk_zone_list_pub.publish(self.risk_zone_list)
        self.chennal_path_array_pub.publish(self.chennal_path_array)
        self.channel_path_array_pub.publish(self.chennal_path_array)

    def _save_all_working_state(self):
        """Attempt every work-file save and report all-or-nothing success."""
        results = (
            self._save_zone_list(),
            self._save_risk_zone_list(),
            self._save_chennal_path_list(),
        )
        return all(results)

    def _restore_startup_persistence(self):
        """Atomically restore all work files and their named-site association."""
        manifest_path = site_store.active_site_path(self._sites_dir())
        manifest_exists = os.path.exists(manifest_path)
        work_paths = self._working_file_paths()

        # A pristine installation has neither work files nor a manifest. Avoid
        # three expected file-not-found errors, but create all three empty
        # snapshots now. A later *_end service persists only its own list, so
        # the other two files must already exist for atomic startup recovery.
        if not manifest_exists and not any(os.path.exists(p) for p in work_paths):
            if self._save_all_working_state():
                self._working_state_ready = True
            else:
                self._active_manifest_blocked_reason = (
                    '無法初始化三份工作檔'
                )
                self.get_logger().error(
                    '任務幾何持久化目錄未就緒'
                )
            return

        original = (
            copy.deepcopy(self.record_zone_list),
            copy.deepcopy(self.risk_zone_list),
            copy.deepcopy(self.chennal_path_array),
        )
        work_results = (
            self._load_zone_list(),
            self._load_risk_zone_list(),
            self._load_chennal_path_list(),
        )
        work_ok = all(work_results)
        if work_ok:
            self._working_state_ready = True
            self._restore_id_counters()
            self._publish_geometry_lists()
        else:
            (self.record_zone_list, self.risk_zone_list,
             self.chennal_path_array) = original
            if not manifest_exists:
                self._active_manifest_blocked_reason = (
                    '三份工作檔不完整'
                )

        manifest_name = None
        if manifest_exists:
            try:
                manifest_name = site_store.read_active_site(self._sites_dir())
            except (OSError, ValueError) as exc:
                self._active_manifest_blocked_reason = (
                    f'啟用場地 manifest 損壞: {exc}'
                )
            if (
                manifest_name is None
                and self._active_manifest_blocked_reason is None
            ):
                self._active_manifest_blocked_reason = (
                    '啟用場地 manifest 在啟動時消失'
                )

        if manifest_name is not None:
            site_exists = os.path.exists(
                site_store.site_path(self._sites_dir(), manifest_name)
            )
            if work_ok and site_exists:
                self.active_site = manifest_name
                self.get_logger().info(
                    f'已復原工作檔與啟用場地「{manifest_name}」'
                )
            else:
                reason = (
                    '三份工作檔不完整'
                    if not work_ok
                    else f'場地檔「{manifest_name}」不存在'
                )
                self._active_manifest_blocked_reason = reason

        if self._active_manifest_blocked_reason is not None:
            self.active_site = None
            self.get_logger().error(
                f'持久化狀態未就緒: '
                f'{self._active_manifest_blocked_reason}；'
                '請使用 /site_op load 重新啟用場地'
            )

    def _reject_blocked_manifest(self, response, operation):
        if self._active_manifest_blocked_reason is None:
            return False
        response.success = False
        response.message = (
            f'無法{operation}：持久化場地狀態未就緒'
            f'（{self._active_manifest_blocked_reason}），'
            '請先使用 /site_op load 重新載入場地'
        )
        self.get_logger().error(response.message)
        return True

    def _publish_site_list(self):
        payload = site_store.list_sites(
            self._sites_dir(), active=self.active_site)
        self.site_list_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False)))
        return payload

    def _restore_id_counters(self):
        """載入後把遞增計數器對齊清單裡的最大 id，避免之後錄製撞號."""
        self.record_zone_id = max(
            (m.id for m in self.record_zone_list.markers), default=0)
        self.risk_zone_id = max(
            (m.id for m in self.risk_zone_list.markers), default=0)
        self.chennal_record_id = max(
            (m.id for m in self.chennal_path_array.markers), default=0)

    def _site_state_objects(self):
        """把 in-memory MarkerArray 轉成 site_store 用的純 dict."""
        def polys(mlist):
            return [{'id': m.id, 'ns': m.ns,
                     'points': [[p.x, p.y] for p in m.points]}
                    for m in mlist.markers]

        channels = [{'id': m.id, 'ns': m.ns,
                     'points': [[p.x, p.y] for p in m.points],
                     'color': {'r': m.color.r, 'g': m.color.g,
                               'b': m.color.b, 'a': m.color.a},
                     'scale': m.scale.x}
                    for m in self.chennal_path_array.markers]
        return (polys(self.record_zone_list),
                polys(self.risk_zone_list),
                channels)

    def _apply_site_objects(self, zones, risk_zones, channels):
        """用 site 內容（已轉回當下 map frame 的 XY）重建三個 MarkerArray.

        先在區域變數建好三份再一次替換 self.*：壞掉的場地檔在建構途中丟
        例外時，不會留下「一半新一半舊」的 in-memory 狀態。
        """
        zone_list = MarkerArray()
        for o in zones:
            zone_list.markers.append(self._marker_from_xy(
                o['ns'], o['id'], (0.6, 0.0, 1.0, 0.5), 0.02,
                o['points'], True))

        risk_list = MarkerArray()
        for o in risk_zones:
            risk_list.markers.append(self._marker_from_xy(
                o['ns'], o['id'], (1.0, 0.0, 0.0, 0.8), 0.03,
                o['points'], True))

        channel_array = MarkerArray()
        for o in channels:
            c = {'r': 0.0, 'g': 1.0, 'b': 0.0, 'a': 0.8}
            c.update(o.get('color') or {})
            channel_array.markers.append(self._marker_from_xy(
                o['ns'], o['id'], (c['r'], c['g'], c['b'], c['a']),
                o.get('scale', 0.1), o['points'], False))

        self.record_zone_list = zone_list
        self.risk_zone_list = risk_list
        self.chennal_path_array = channel_array

    def _update_active_site(self):
        """編輯物件後同步覆寫啟用中的場地檔，讓場地與工作狀態一致."""
        if self._active_manifest_blocked_reason is not None:
            self.get_logger().error(
                '啟用場地 manifest 未就緒，拒絕自動同步'
            )
            return False
        if not self.active_site:
            return True
        if self.map_datum is None:
            self.get_logger().error('無 datum，無法同步場地檔')
            return False
        try:
            created = None
            existing_source = None
            try:
                manifest_name = site_store.read_active_site(self._sites_dir())
                if manifest_name != self.active_site:
                    self._active_manifest_blocked_reason = (
                        'manifest 與記憶體的啟用場地不一致'
                    )
                    return False
                existing = site_store.read_site(
                    self._sites_dir(), self.active_site)
                created = existing.get('created_at')
                existing_source = existing.get('datum', {}).get('source')
            except (OSError, ValueError) as exc:
                self._active_manifest_blocked_reason = (
                    f'無法讀取啟用場地: {exc}'
                )
                return False
            # datum 來源改變（如開機後 fallback → navsat 鎖定）時不自動覆寫：
            # 檔內的 WGS84 是唯一副本，寧可略過同步也不能寫入位移後的座標。
            # 使用者可用 /site_op save 明確以新 datum 重存。
            if (existing_source is not None
                    and existing_source != self.map_datum.get('source', '')):
                self.get_logger().warn(
                    f'datum 來源已由 {existing_source} 變為 '
                    f'{self.map_datum.get("source", "")}，跳過場地「'
                    f'{self.active_site}」自動同步')
                return False
            zones, risks, channels = self._site_state_objects()
            site = site_store.build_site(
                self.active_site, self.map_datum, zones, risks, channels,
                created_at=created)
            site_store.write_site(self._sites_dir(), site)
            try:
                self._publish_site_list()
            except Exception as e:
                # The named-site file is already durable. A list-topic publish
                # failure must not make edit_zone roll back only the work file
                # and leave the two persisted copies divergent.
                self.get_logger().warn(f'場地清單發佈失敗: {e}')
            return True
        except Exception as e:
            self._active_manifest_blocked_reason = (
                f'場地檔同步結果無法確認: {e}'
            )
            self.get_logger().error(f'場地檔同步失敗: {e}')
            return False

    def site_op_srv(self, req, res):
        """場地庫操作：save / load / delete / rename / list."""
        op = req.op
        name = site_store.valid_name(req.name)
        if (
            op not in ('list', 'load')
            and self._reject_blocked_manifest(res, f'執行場地 {op}')
        ):
            res.sites_json = json.dumps(
                self._publish_site_list(), ensure_ascii=False
            )
            return res
        mutation_lease = False
        if op != 'list':
            mutation_lease = self._acquire_mission_mutation(
                res,
                f'{op or "unknown"} a named site',
            )
            if not mutation_lease:
                res.sites_json = json.dumps(
                    self._publish_site_list(), ensure_ascii=False)
                return res

        try:
            if op == 'list':
                res.success = True
                res.message = '成功取得場地清單'
            elif name is None:
                res.success = False
                res.message = f'場地名稱無效: {req.name!r}'
            elif op == 'save':
                res.success, res.message = self._site_save(name)
            elif op == 'load':
                res.success, res.message = self._site_load(name)
            elif op == 'delete':
                res.success, res.message = self._site_delete(name)
            elif op == 'rename':
                res.success, res.message = self._site_rename(
                    name, site_store.valid_name(req.new_name))
            else:
                res.success = False
                res.message = f'未知 op: {op}（要 save/load/delete/rename/list）'
        except Exception as e:
            res.success = False
            res.message = f'場地操作失敗: {e}'
            self.get_logger().error(res.message)
        finally:
            if mutation_lease and not self._release_mission_mutation():
                previous = res.message
                res.success = False
                res.message = (
                    f'{previous}；' if previous else ''
                ) + (
                    '場地操作可能已完成，但操作鎖釋放未確認，導航仍被禁止；'
                    '請檢查機器後重啟 path_record_node 與 '
                    'nav_action_server'
                )

        res.sites_json = json.dumps(
            self._publish_site_list(), ensure_ascii=False)
        if res.success:
            self.get_logger().info(res.message)
        return res

    def _site_save(self, name):
        if self.map_datum is None:
            return False, 'datum 尚未就緒（等待 /adapter/map_datum），無法儲存場地'
        if self.map_datum.get('source') != 'navsat':
            return False, 'GPS datum 尚未由 NavSatFix 確認，拒絕儲存可執行場地'
        zones, risks, channels = self._site_state_objects()
        if not (zones or risks or channels):
            return False, '目前沒有任何物件可存成場地'
        if not self._save_all_working_state():
            return False, '三份工作檔未能完整儲存，拒絕啟用場地'
        sites_dir = self._sites_dir()
        old_manifest = site_store.read_active_site(sites_dir)
        old_site = None
        target_existed = os.path.exists(site_store.site_path(sites_dir, name))
        created = None
        try:
            if target_existed:
                old_site = site_store.read_site(sites_dir, name)
                created = old_site.get('created_at')
        except (OSError, ValueError) as exc:
            return False, f'既有場地檔損壞，拒絕覆寫: {exc}'
        site = site_store.build_site(
            name, self.map_datum, zones, risks, channels, created_at=created)
        site_store.write_site(sites_dir, site)
        try:
            site_store.write_active_site(sites_dir, name)
        except Exception as exc:  # noqa: BLE001
            rollback_errors = []
            try:
                if target_existed:
                    site_store.write_site(sites_dir, old_site)
                elif os.path.exists(site_store.site_path(sites_dir, name)):
                    site_store.delete_site(sites_dir, name)
            except Exception as rollback_exc:  # noqa: BLE001
                rollback_errors.append(f'場地檔: {rollback_exc}')
            try:
                if old_manifest is None:
                    site_store.clear_active_site(sites_dir)
                else:
                    site_store.write_active_site(sites_dir, old_manifest)
            except Exception as rollback_exc:  # noqa: BLE001
                rollback_errors.append(f'manifest: {rollback_exc}')
            message = f'啟用場地 manifest 寫入失敗: {exc}'
            if rollback_errors:
                self._active_manifest_blocked_reason = '; '.join(
                    rollback_errors
                )
                message += f'；回復失敗: {"; ".join(rollback_errors)}'
            return False, message
        self.active_site = name
        self._active_manifest_blocked_reason = None
        self._working_state_ready = True
        return True, (
            f'已儲存場地「{name}」'
            f'（{len(zones)} 工作區 / {len(risks)} 禁區 / {len(channels)} 通道，'
            f'datum: {self.map_datum["source"]}）')

    def _site_load(self, name):
        if self.map_datum is None:
            return False, 'datum 尚未就緒（等待 /adapter/map_datum），無法載入場地'
        if (self.record_zone_status or self.risk_zone_status
                or self.chennal_record_status):
            # 載入會重設 id 計數器，會跟進行中錄製的 marker id 撞號。
            return False, '錄製進行中，請先結束或取消錄製再載入場地'
        if not os.path.exists(site_store.site_path(self._sites_dir(), name)):
            return False, f'找不到場地「{name}」'
        site = site_store.read_site(self._sites_dir(), name)

        # datum 來源必須一致：用 fallback datum 投影 RTK 存的場地（或反過來）
        # 會把區域放到錯的位置，之後的自動同步還會把檔案裡的真值改寫壞。
        site_source = site.get('datum', {}).get('source', '')
        cur_source = self.map_datum.get('source', '')
        if cur_source != 'navsat':
            return False, (
                f'GPS 尚未由 NavSatFix 確認（目前 datum 為 '
                f'{cur_source or "未知"}），拒絕啟用可執行場地'
            )
        if site_source != cur_source:
            return False, (
                f'此場地以 {site_source} datum 儲存，與目前 {cur_source} '
                f'不相容 — 請在相同定位條件下重新錄製或另存')

        site_datum = site.get('datum', {})
        try:
            site_lat = float(site_datum['lat'])
            site_lon = float(site_datum['lon'])
            current_lat = float(self.map_datum['lat'])
            current_lon = float(self.map_datum['lon'])
            max_distance_m = float(
                self.get_parameter('max_site_datum_distance_m').value
            )
        except (KeyError, TypeError, ValueError):
            return False, '場地 datum 格式無效，拒絕載入'
        if (
            not all(math.isfinite(value) for value in (
                site_lat, site_lon, current_lat, current_lon, max_distance_m
            ))
            or not -90.0 <= site_lat <= 90.0
            or not -180.0 <= site_lon <= 180.0
            or max_distance_m <= 0.0
        ):
            return False, '場地 datum 或允許距離設定無效，拒絕載入'
        datum_distance_m = math.hypot(*site_store.xy_from_ll(
            site_lat,
            site_lon,
            self.map_datum,
        ))
        if datum_distance_m > max_distance_m:
            return False, (
                f'場地「{name}」原點距目前定位約 {datum_distance_m:.1f} m，'
                f'超過安全上限 {max_distance_m:.1f} m，拒絕啟用'
            )

        sites_dir = self._sites_dir()
        old_manifest_valid = True
        try:
            old_manifest = site_store.read_active_site(sites_dir)
        except (OSError, ValueError) as exc:
            old_manifest = None
            old_manifest_valid = False
            old_manifest_error = str(exc)
        old_state = (
            copy.deepcopy(self.record_zone_list),
            copy.deepcopy(self.risk_zone_list),
            copy.deepcopy(self.chennal_path_array),
            self.active_site,
            self._active_manifest_blocked_reason,
            self._working_state_ready,
        )
        zones, risks, channels = site_store.site_to_xy(site, self.map_datum)
        self._apply_site_objects(zones, risks, channels)
        self._restore_id_counters()

        # 工作檔（zone_record/*.json）同步成剛載入的場地，維持 auto_coverage 一致。
        synced = self._save_all_working_state()
        if not synced:
            (self.record_zone_list, self.risk_zone_list,
             self.chennal_path_array, self.active_site,
             self._active_manifest_blocked_reason,
             self._working_state_ready) = old_state
            self._restore_id_counters()
            repaired = self._save_all_working_state()
            message = '工作檔同步失敗；場地未啟用，既有資料已回復'
            if not repaired:
                self._active_manifest_blocked_reason = '舊工作檔回復失敗'
                message += '（舊工作檔回復也失敗，請立即備份並檢查磁碟）'
            return False, message

        try:
            site_store.write_active_site(sites_dir, name)
        except Exception as exc:  # noqa: BLE001
            (self.record_zone_list, self.risk_zone_list,
             self.chennal_path_array, self.active_site,
             self._active_manifest_blocked_reason,
             self._working_state_ready) = old_state
            self._restore_id_counters()
            repaired = self._save_all_working_state()
            manifest_repaired = old_manifest_valid
            if old_manifest_valid:
                try:
                    if old_manifest is None:
                        site_store.clear_active_site(sites_dir)
                    else:
                        site_store.write_active_site(
                            sites_dir,
                            old_manifest,
                        )
                except Exception as manifest_exc:  # noqa: BLE001
                    manifest_repaired = False
                    self.get_logger().error(
                        f'舊 manifest 回復失敗: {manifest_exc}'
                    )
            if not repaired:
                self._active_manifest_blocked_reason = (
                    'manifest 寫入失敗且舊工作檔回復失敗'
                )
            if not manifest_repaired:
                detail = (
                    old_manifest_error
                    if not old_manifest_valid
                    else '舊 manifest 回復失敗'
                )
                self._active_manifest_blocked_reason = detail
            message = f'啟用場地 manifest 寫入失敗: {exc}'
            if not repaired:
                message += '；舊工作檔回復也失敗，請立即檢查磁碟'
            if not manifest_repaired:
                message += '；舊 manifest 狀態無法確認'
            return False, message

        self.active_site = name
        self._active_manifest_blocked_reason = None
        self._working_state_ready = True
        self._publish_geometry_lists()

        msg = (f'已載入場地「{name}」'
               f'（{len(zones)} 工作區 / {len(risks)} 禁區 / {len(channels)} 通道）')
        return True, msg

    def _site_delete(self, name):
        sites_dir = self._sites_dir()
        path = site_store.site_path(sites_dir, name)
        if not os.path.exists(path):
            return False, f'找不到場地「{name}」'
        backup = site_store.read_site(sites_dir, name)
        manifest_name = site_store.read_active_site(sites_dir)
        if self.active_site != manifest_name:
            self._active_manifest_blocked_reason = (
                'manifest 與記憶體的啟用場地不一致'
            )
            return False, self._active_manifest_blocked_reason
        was_active = manifest_name == name
        try:
            if was_active:
                site_store.clear_active_site(sites_dir)
            site_store.delete_site(sites_dir, name)
        except Exception as exc:  # noqa: BLE001
            rollback_errors = []
            try:
                if not os.path.exists(path):
                    site_store.write_site(sites_dir, backup)
            except Exception as rollback_exc:  # noqa: BLE001
                rollback_errors.append(f'場地檔: {rollback_exc}')
            if was_active:
                try:
                    site_store.write_active_site(sites_dir, name)
                except Exception as rollback_exc:  # noqa: BLE001
                    rollback_errors.append(f'manifest: {rollback_exc}')
            if rollback_errors:
                self._active_manifest_blocked_reason = '; '.join(
                    rollback_errors
                )
            message = f'刪除場地失敗: {exc}'
            if rollback_errors:
                message += f'；回復失敗: {"; ".join(rollback_errors)}'
            return False, message
        if was_active:
            self.active_site = None
        return True, f'已刪除場地「{name}」'

    def _site_rename(self, name, new_name):
        if new_name is None:
            return False, '新場地名稱無效'
        sites_dir = self._sites_dir()
        old_path = site_store.site_path(sites_dir, name)
        new_path = site_store.site_path(sites_dir, new_name)
        if not os.path.exists(old_path):
            return False, f'找不到場地「{name}」'
        if os.path.exists(new_path):
            return False, f'場地「{new_name}」已存在'
        old_site = site_store.read_site(sites_dir, name)
        new_site = copy.deepcopy(old_site)
        new_site['name'] = new_name
        manifest_name = site_store.read_active_site(sites_dir)
        if self.active_site != manifest_name:
            self._active_manifest_blocked_reason = (
                'manifest 與記憶體的啟用場地不一致'
            )
            return False, self._active_manifest_blocked_reason
        was_active = manifest_name == name
        try:
            site_store.write_site(sites_dir, new_site)
            if was_active:
                site_store.write_active_site(sites_dir, new_name)
            site_store.delete_site(sites_dir, name)
        except Exception as exc:  # noqa: BLE001
            rollback_errors = []
            try:
                if not os.path.exists(old_path):
                    site_store.write_site(sites_dir, old_site)
                if os.path.exists(new_path):
                    site_store.delete_site(sites_dir, new_name)
            except Exception as rollback_exc:  # noqa: BLE001
                rollback_errors.append(f'場地檔: {rollback_exc}')
            if was_active:
                try:
                    site_store.write_active_site(sites_dir, name)
                except Exception as rollback_exc:  # noqa: BLE001
                    rollback_errors.append(f'manifest: {rollback_exc}')
            if rollback_errors:
                self._active_manifest_blocked_reason = '; '.join(
                    rollback_errors
                )
            message = f'場地改名失敗: {exc}'
            if rollback_errors:
                message += f'；回復失敗: {"; ".join(rollback_errors)}'
            return False, message
        if was_active:
            self.active_site = new_name
        return True, f'已將場地「{name}」改名為「{new_name}」'

    def risk_zone_start_srv(self, req, res):
        """風險區域開始記錄服務."""
        rejected = self._reject_start_while_recording(res, 'risk')
        if rejected is not None:
            return rejected

        self.get_logger().info('開始記錄風險區域')
        self.risk_zone_status = True
        self._arm_record_timer()
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
        if not self.risk_zone_status:
            res.success = False
            res.message = '沒有正在進行的 risk 記錄'
            return res
        if (self.risk_zone_marker is None
                or len(self.risk_zone_marker.points) < 3):
            res.success = False
            res.message = '風險區域點數不足，至少需要 3 個點'
            return res

        self.get_logger().info('結束記錄風險區域')
        self.risk_zone_list.markers.append(self.risk_zone_marker)
        if not self._save_risk_zone_list():
            self.risk_zone_list.markers.pop()
            res.success = False
            res.message = (
                '風險區域已收尾但持久化失敗；錄製仍保留，'
                '請檢查磁碟後重試結束'
            )
            self.get_logger().error(res.message)
            return res

        self.risk_zone_status = False
        self.risk_zone_list_pub.publish(self.risk_zone_list)

        self.risk_zone_marker = Marker()
        self.risk_path = Path()
        self.risk_path.header.frame_id = self.get_parameter('frame_id').value
        self.last_robot_pos = None
        self._working_state_ready = True
        release_confirmed = self._release_mission_mutation()

        res.success = release_confirmed
        res.message = (
            f'成功結束記錄風險區域 #{len(self.risk_zone_list.markers)}')
        if not release_confirmed:
            res.message += (
                '；資料已儲存，但操作鎖釋放未確認，導航仍被禁止；'
                '請檢查機器後重啟 path_record_node 與 nav_action_server'
            )
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

    @guarded_mission_mutation('load risk geometry')
    def risk_zone_load_srv(self, req, res):
        """風險區域載入服務."""
        original = copy.deepcopy(self.risk_zone_list)
        if self._load_risk_zone_list():
            self.risk_zone_list_pub.publish(self.risk_zone_list)
            res.success = True
            res.message = '成功載入風險區域'
        else:
            self.risk_zone_list = original
            res.success = False
            res.message = '載入風險區域失敗，既有資料已保留'
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

            _write_json_atomic(
                self.get_parameter('save_dir').value
                + '/risk_zone_list.json',
                risk_zones_data,
            )
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

            _write_json_atomic(
                self.get_parameter('save_dir').value + '/zone_list.json',
                zones_data,
            )
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
        rejected = self._reject_start_while_recording(res, 'channel')
        if rejected is not None:
            return rejected

        self.get_logger().info('開始記錄 chennal 路徑')

        self.chennal_record_status = True
        self.chennal_record_id += 1
        self._arm_record_timer()
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

        robot_pos = self.get_robot_pos()
        if robot_pos is not None and len(self.chennal_path.poses) > 0:
            last_pose = self.chennal_path.poses[-1]
            dx = robot_pos.pose.position.x - last_pose.pose.position.x
            dy = robot_pos.pose.position.y - last_pose.pose.position.y
            if math.hypot(dx, dy) >= self.get_parameter('min_dist').value:
                self.chennal_path.poses.append(robot_pos)

        if len(self.chennal_path.poses) < 2:
            res.success = False
            res.message = 'chennal 路徑點數不足，至少需要 2 個點'
            return res

        completed_marker = path_to_marker(
            self.chennal_path,
            ns='chennal_path',
            marker_id=self.chennal_record_id,
            color=(0.0, 1.0, 0.0),
            scale=0.1,
        )
        self.chennal_path_array.markers.append(completed_marker)
        if not self._save_chennal_path_list():
            self.chennal_path_array.markers.pop()
            res.success = False
            res.message = (
                'channel 路徑已收尾但持久化失敗；錄製仍保留，'
                '請檢查磁碟後重試結束'
            )
            self.get_logger().error(res.message)
            return res

        self.chennal_record_status = False
        self.chennal_path_array_pub.publish(self.chennal_path_array)
        self.channel_path_array_pub.publish(self.chennal_path_array)
        self.chennal_path = Path()
        self.chennal_path.header.frame_id = self.get_parameter('frame_id').value
        self.last_robot_pos = None
        self._working_state_ready = True
        release_confirmed = self._release_mission_mutation()
        res.success = release_confirmed
        res.message = '成功結束記錄 chennal 路徑'
        if not release_confirmed:
            res.message += (
                '；資料已儲存，但操作鎖釋放未確認，導航仍被禁止；'
                '請檢查機器後重啟 path_record_node 與 nav_action_server'
            )
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
            _write_json_atomic(save_path, chennal_paths_data)

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
    # Service callbacks block on the mission-operation lock client
    # (navigation_guard._wait_for_lock_response), which another executor
    # thread has to service, so this node cannot run single-threaded.
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
