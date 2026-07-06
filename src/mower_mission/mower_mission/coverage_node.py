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
autonomous lawn mowers using zigzag and spiral patterns. It integrates
with ROS2 Nav2 for autonomous navigation.
"""

import threading

from mower_interface.action import Waypoint
from mower_interface.srv import (
    ChannelRoute, ZoneExecPath, ZoneMapList, ZoneSequence,
)
from rclpy.action import ActionClient

from geometry_msgs.msg import Point

from nav_msgs.msg import OccupancyGrid, Path

import cv2

import numpy as np

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from std_srvs.srv import SetBool, Trigger

from visualization_msgs.msg import Marker, MarkerArray

from .coverage.path_validator import SafeMap
from .coverage_backend.factory import create_backend
from .utils.path_utils import (
    _transform_coverage_path_points,
    _transform_coverage_split_points,
)



class CoveragePlanner(Node):
    """CoveragePlanner class."""

    def __init__(self):
        """Initialize the CoveragePlanner class."""
        super().__init__('boustrophedon_coverage')
        self.set_parameters(
            [Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.get_logger().info('boustrophedon_coverage 初始化')
        self.declare_parameter('strip_width_m', 0.8)
        self.declare_parameter('unknown_as_obstacle', True)
        self.declare_parameter('min_safe_component_area_m2', 0.05)
        self.declare_parameter('coverage_pattern', 'zigzag')
        self.declare_parameter('coverage_backend', 'rust')
        self.declare_parameter('allow_backend_fallback', True)
        # boundary_ring: when true, trace each zone's outer contour as a
        # perimeter pass (mowed before the area fill). User-controllable from the
        # planning UI (PyQt checkbox / Flutter); custom image missions may also
        # enable it automatically. Global flag — its value persists across
        # missions, so reset to false for normal missions if no ring is wanted.
        self.declare_parameter('boundary_ring', False)

        self._backend = create_backend(
            name=str(self.get_parameter('coverage_backend').value),
            allow_fallback=bool(self.get_parameter('allow_backend_fallback').value),
        )
        self.get_logger().info(
            f'coverage backend: {type(self._backend).__name__}'
        )

        qos_vol = QoSProfile(depth=1)
        qos_vol.durability = QoSDurabilityPolicy.VOLATILE
        qos_vol.reliability = QoSReliabilityPolicy.RELIABLE

        qos_tl = QoSProfile(depth=1)
        qos_tl.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos_tl.reliability = QoSReliabilityPolicy.RELIABLE

        self.cb_group = ReentrantCallbackGroup()

        self.create_service(
            Trigger, '/generate_coverage_path', self.generate_coverage_path_srv,
            callback_group=self.cb_group
        )
        self.create_service(ZoneExecPath, '/zone_exec_path',
                            self.zone_exec_path_srv,
                            callback_group=self.cb_group)
        self.waypoint_active_client = self.create_client(
            SetBool,
            '/record_path_status',
        )
        self._zone_map_list_client = self.create_client(
            ZoneMapList, '/get_zone_map_list_srv',
            callback_group=self.cb_group,
        )

        self._nav_follow_path_client = ActionClient(
            self, Waypoint, 'nav_action_follow_path'
        )
        self._channel_route_client = self.create_client(
            ChannelRoute, '/get_channel_route',
            callback_group=self.cb_group
        )
        self._sequence_thread = None
        self._sequence_cancel = threading.Event()

        self.create_service(
            ZoneSequence, '/run_zone_sequence',
            self.run_zone_sequence_srv,
            callback_group=self.cb_group
        )
        self.create_service(
            Trigger, '/stop_zone_sequence',
            self.stop_zone_sequence_srv,
            callback_group=self.cb_group
        )

        self.path_pub = self.create_publisher(Path, '/coverage_path', 1)
        self.path_marker_pub = self.create_publisher(
            MarkerArray, '/coverage_path_markers', 1
        )
        self.invalid_segments_pub = self.create_publisher(
            MarkerArray, '/coverage_invalid_segments', 1
        )
        self.connectors_pub = self.create_publisher(
            MarkerArray, '/coverage_connectors', 1
        )
        self.free_space_inflated_pub = self.create_publisher(
            OccupancyGrid, '/free_space_inflated', qos_vol
        )
        self.risk_map_inflated_pub = self.create_publisher(
            OccupancyGrid, '/risk_map_inflated', qos_vol
        )

        self.free_space_map = None
        self.risk_map = None
        self.free_space_inflated_map = None
        self.risk_map_inflated_map = None

        self.sub_risk_map = self.create_subscription(
            OccupancyGrid, '/risk_map', self.risk_map_callback, qos_tl,
            callback_group=self.cb_group
        )

        self.sub_risk_map_inflated = self.create_subscription(
            OccupancyGrid,
            '/risk_map_inflated',
            self.risk_map_inflated_callback,
            qos_tl,
            callback_group=self.cb_group
        )

        self.waypoint_active = False
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
        if self.risk_map_inflated_map is None:
            res.success = False
            res.message = '缺少 risk_map_inflated 地圖數據'
            return res

        success = self.generate_coverage_path()

        if success:
            res.success = True
            res.message = '覆蓋路徑生成成功'
        else:
            res.success = False
            res.message = '覆蓋路徑生成失敗'

        return res

    def generate_coverage_path(self):
        """Generate coverage path for all zones."""
        self._clear_path_visuals()
        resp = self._blocking_service_call(
            self._zone_map_list_client, ZoneMapList.Request()
        )
        self.zone_map_list = list(resp.zone_map_list) if resp else []
        if not self.zone_map_list:
            self.get_logger().error('沒有可用的 zone map')
            return False

        pattern = str(self.get_parameter('coverage_pattern').value).lower()
        if pattern not in ('zigzag', 'spiral'):
            self.get_logger().error(
                f'unsupported coverage_pattern "{pattern}"; expected '
                'zigzag or spiral'
            )
            return False
        self.get_logger().info(f'coverage_pattern={pattern}')

        for i in range(len(self.zone_map_list)):
            info = self.zone_map_list[i].mask_map.info
            H, W = info.height, info.width
            res = info.resolution
            ox, oy = info.origin.position.x, info.origin.position.y
            risk_map_data = self._risk_map_data_for_zone(
                self.zone_map_list[i].mask_map
            )
            if risk_map_data is None:
                return False

            mask_map_inflated_data = np.asarray(
                self.zone_map_list[i].mask_map_inflated.data, dtype=np.int16
            ).reshape(H, W)

            safe_map = np.logical_and(
                mask_map_inflated_data == 0, risk_map_data == 0
            ).astype(np.uint8)
            safe_map, component_sizes, kept_component_sizes = (
                self._backend.filter_safe_components(
                    safe_map,
                    resolution=res,
                    min_area_m2=self.get_parameter(
                        'min_safe_component_area_m2'
                    ).value,
                    keep_largest_only=True,
                )
            )
            if not kept_component_sizes:
                self.get_logger().error(
                    f'zone {self.zone_map_list[i].zone_id}: no safe '
                    'component remains after filtering'
                )
                return False
            if component_sizes != kept_component_sizes:
                self.get_logger().warn(
                    f'zone {self.zone_map_list[i].zone_id}: safe components '
                    f'{component_sizes[:8]}, keeping {kept_component_sizes}'
                )

            self.get_logger().info(
                f'zone {self.zone_map_list[i].zone_id}: '
                f'safe_cells={int(np.count_nonzero(safe_map))}, '
                f'risk_cells={int(np.count_nonzero(risk_map_data != 0))}'
            )

            _gen_kw = dict(
                safe_map=safe_map,
                strip_width_m=self.get_parameter('strip_width_m').value,
                res=res,
                H=H,
                W=W,
                origin_x=ox,
                origin_y=oy,
            )
            if pattern == 'spiral':
                coverage_pts, split_pts, invalid_segs = (
                    self._backend.generate_spiral_path(**_gen_kw)
                )
            else:
                coverage_pts, split_pts, invalid_segs = (
                    self._backend.generate_zigzag_path(**_gen_kw)
                )

            safe_map_struct = SafeMap(
                grid=safe_map.astype(bool),
                resolution=res,
                origin_x=ox,
                origin_y=oy,
            )

            if invalid_segs:
                self.get_logger().warn(
                    f'zone {self.zone_map_list[i].zone_id}: '
                    f'{len(invalid_segs)} unsafe segment(s) — '
                    f'running ConnectorPlanner'
                )
                raw_coverage_pts = coverage_pts
                coverage_pts, connector_viz, unresolved = self._apply_connectors(
                    coverage_pts, invalid_segs, safe_map_struct, i, self._backend
                )
                if unresolved:
                    self._publish_invalid_segments(
                        raw_coverage_pts, unresolved, i,
                        self.zone_map_list[i].mask_map.header.frame_id or 'map'
                    )
                    self.get_logger().error(
                        f'zone {self.zone_map_list[i].zone_id}: '
                        f'{len(unresolved)} unsafe connector(s) unresolved; '
                        'coverage path not published'
                    )
                    return False
                if connector_viz:
                    self._publish_connectors(
                        connector_viz, i,
                        self.zone_map_list[i].mask_map.header.frame_id or 'map'
                    )

            final_validation = self._backend.validate_path(coverage_pts, safe_map_struct)
            if not final_validation.valid:
                self._publish_invalid_segments(
                    coverage_pts, final_validation.invalid_segments, i,
                    self.zone_map_list[i].mask_map.header.frame_id or 'map'
                )
                self.get_logger().error(
                    f'zone {self.zone_map_list[i].zone_id}: final path is '
                    f'unsafe: {final_validation.message}; coverage path not '
                    'published'
                )
                return False

            # Optionally prepend an outer-contour perimeter pass ("boundary
            # ring") so the zone's outline is mowed before the area is filled.
            # Merge it into the point list BEFORE transforming and re-run the
            # same validation + connector planning as the fill: otherwise the
            # straight jump from the ring back to the fill's start (and any ring
            # segment grazing an inflated risk region) can cut across risk zones.
            if bool(self.get_parameter('boundary_ring').value):
                ring_pts = self._outer_boundary_ring(safe_map, res, ox, oy)
                if ring_pts:
                    coverage_pts = ring_pts + coverage_pts
                    self.get_logger().info(
                        f'zone {self.zone_map_list[i].zone_id}: added boundary '
                        f'ring ({len(ring_pts)} pts); re-validating combined path'
                    )
                    ring_validation = self._backend.validate_path(
                        coverage_pts, safe_map_struct
                    )
                    if ring_validation.invalid_segments:
                        raw_ring_pts = coverage_pts
                        coverage_pts, ring_conn_viz, ring_unresolved = (
                            self._apply_connectors(
                                coverage_pts,
                                ring_validation.invalid_segments,
                                safe_map_struct, i, self._backend,
                            )
                        )
                        if ring_unresolved:
                            self._publish_invalid_segments(
                                raw_ring_pts, ring_unresolved, i,
                                self.zone_map_list[i].mask_map.header.frame_id
                                or 'map'
                            )
                            self.get_logger().error(
                                f'zone {self.zone_map_list[i].zone_id}: '
                                f'{len(ring_unresolved)} unsafe boundary-ring '
                                'connector(s) unresolved; coverage path not '
                                'published'
                            )
                            return False
                        if ring_conn_viz:
                            self._publish_connectors(
                                ring_conn_viz, i,
                                self.zone_map_list[i].mask_map.header.frame_id
                                or 'map'
                            )
                    ring_final = self._backend.validate_path(
                        coverage_pts, safe_map_struct
                    )
                    if not ring_final.valid:
                        self._publish_invalid_segments(
                            coverage_pts, ring_final.invalid_segments, i,
                            self.zone_map_list[i].mask_map.header.frame_id or 'map'
                        )
                        self.get_logger().error(
                            f'zone {self.zone_map_list[i].zone_id}: boundary-ring '
                            f'path unsafe: {ring_final.message}; not published'
                        )
                        return False

            coverage_path = _transform_coverage_path_points(
                points=coverage_pts,
                map_header=self.zone_map_list[i].mask_map.header
            )

            coverage_split_points = _transform_coverage_split_points(
                points=split_pts,
                map_header=self.zone_map_list[i].mask_map.header,
            )
            self.zone_map_list[i].path = coverage_path
            self.zone_map_list[i].coverage_split_points = coverage_split_points

        vivid_colors = [
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
            (1.0, 1.0, 0.0),
            (1.0, 0.0, 1.0),
            (0.0, 1.0, 1.0),
            (1.0, 0.5, 0.0),
            (0.5, 0.0, 1.0),
            (0.0, 0.5, 1.0),
            (0.5, 1.0, 0.0),
        ]
        marker_array = MarkerArray()
        color_i = 0
        for zone_idx, zone_map in enumerate(self.zone_map_list):
            if zone_map.path and len(zone_map.path.poses) > 0:
                line_marker = Marker()
                line_marker.header.frame_id = zone_map.path.header.frame_id
                line_marker.header.stamp = self.get_clock().now().to_msg()
                line_marker.ns = f'zone_{zone_map.zone_id}_path'
                line_marker.id = zone_idx
                line_marker.type = Marker.LINE_STRIP
                line_marker.action = Marker.ADD

                line_marker.scale.x = 0.03

                rgb = vivid_colors[color_i % len(vivid_colors)]
                line_marker.color.r = rgb[0]
                line_marker.color.g = rgb[1]
                line_marker.color.b = rgb[2]
                line_marker.color.a = 0.8

                for pose_stamped in zone_map.path.poses:
                    point = Point()
                    point.x = pose_stamped.pose.position.x
                    point.y = pose_stamped.pose.position.y
                    point.z = pose_stamped.pose.position.z
                    line_marker.points.append(point)

                marker_array.markers.append(line_marker)

                for i, pose_stamped in enumerate(zone_map.path.poses[::5]):
                    arrow_marker = Marker()
                    arrow_marker.header.frame_id = (
                        zone_map.path.header.frame_id
                    )
                    arrow_marker.header.stamp = self.get_clock().now().to_msg()
                    arrow_marker.ns = f'zone_{zone_map.zone_id}_arrows'
                    arrow_marker.id = zone_idx * 1000 + i
                    arrow_marker.type = Marker.ARROW
                    arrow_marker.action = Marker.ADD

                    arrow_marker.pose = pose_stamped.pose

                    arrow_marker.scale.x = 0.15
                    arrow_marker.scale.y = 0.04
                    arrow_marker.scale.z = 0.08
                    arrow_marker.color.r = 0.5 if zone_idx == 0 else 0.0
                    arrow_marker.color.g = 0.0 if zone_idx == 0 else 0.5
                    arrow_marker.color.b = 0.5
                    arrow_marker.color.a = 0.8

                    marker_array.markers.append(arrow_marker)
            color_i += 1
        self.path_marker_pub.publish(marker_array)
        self.get_logger().info(f'發布了 {len(marker_array.markers)} 個路徑markers')
        return True

    def _clear_path_visuals(self):
        """Clear previous coverage markers before generating a new path."""
        marker = Marker()
        marker.action = Marker.DELETEALL
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.path_marker_pub.publish(marker_array)
        self.invalid_segments_pub.publish(marker_array)
        self.connectors_pub.publish(marker_array)

        empty_path = Path()
        empty_path.header.frame_id = 'map'
        empty_path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(empty_path)

    def _outer_boundary_ring(self, safe_map, res, origin_x, origin_y):
        """Outer contour of the safe region as a closed ring of world (x, y)
        points — used as a perimeter pass so a custom shape's outline is mowed.
        Returns [] if no contour is found."""
        contours, _ = cv2.findContours(
            safe_map.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
        )
        if not contours:
            return []
        contour = max(contours, key=cv2.contourArea)
        pts = [
            (
                origin_x + (float(col) + 0.5) * res,
                origin_y + (float(row) + 0.5) * res,
            )
            for col, row in contour[:, 0, :]
        ]
        if len(pts) >= 2:
            pts.append(pts[0])  # close the loop
        return pts

    def _risk_map_data_for_zone(self, zone_map: OccupancyGrid):
        """Return risk data aligned to a zone map grid."""
        risk_msg = self.risk_map_inflated_map
        if risk_msg is None:
            self.get_logger().error('缺少 risk_map_inflated 地圖數據')
            return None

        risk_info = risk_msg.info
        expected_len = risk_info.height * risk_info.width
        if len(risk_msg.data) != expected_len:
            self.get_logger().error(
                'risk_map_inflated data size does not match metadata: '
                f'{len(risk_msg.data)} != {expected_len}'
            )
            return None

        risk_data = np.asarray(risk_msg.data, dtype=np.int16).reshape(
            risk_info.height,
            risk_info.width,
        )

        if self._same_grid_info(zone_map.info, risk_info):
            return risk_data

        self.get_logger().warn(
            'risk_map_inflated metadata differs from zone map; resampling '
            'risk map into zone grid'
        )
        return self._resample_risk_map_to_zone(risk_data, risk_info, zone_map.info)

    @staticmethod
    def _same_grid_info(a, b, tol=1e-6):
        return (
            a.height == b.height
            and a.width == b.width
            and abs(a.resolution - b.resolution) <= tol
            and abs(a.origin.position.x - b.origin.position.x) <= tol
            and abs(a.origin.position.y - b.origin.position.y) <= tol
        )

    def _resample_risk_map_to_zone(self, risk_data, risk_info, zone_info):
        zone_h = zone_info.height
        zone_w = zone_info.width
        zone_res = zone_info.resolution
        zone_ox = zone_info.origin.position.x
        zone_oy = zone_info.origin.position.y
        risk_res = risk_info.resolution
        risk_ox = risk_info.origin.position.x
        risk_oy = risk_info.origin.position.y

        yy, xx = np.indices((zone_h, zone_w))
        world_x = zone_ox + (xx + 0.5) * zone_res
        world_y = zone_oy + (yy + 0.5) * zone_res
        risk_cols = np.floor((world_x - risk_ox) / risk_res).astype(np.int64)
        risk_rows = np.floor((world_y - risk_oy) / risk_res).astype(np.int64)

        inside = (
            (risk_rows >= 0)
            & (risk_rows < risk_info.height)
            & (risk_cols >= 0)
            & (risk_cols < risk_info.width)
        )

        if not np.all(inside):
            outside_count = int(np.size(inside) - np.count_nonzero(inside))
            self.get_logger().warn(
                f'{outside_count} zone cells are outside risk_map_inflated '
                'bounds'
            )

        sampled = np.zeros((zone_h, zone_w), dtype=np.int16)
        sampled[inside] = risk_data[risk_rows[inside], risk_cols[inside]]
        if self.get_parameter('unknown_as_obstacle').value:
            sampled[~inside] = 100
        return sampled

    def _apply_connectors(
        self,
        points: list,
        invalid_segs: list,
        safe_map_struct: SafeMap,
        zone_idx: int,
        backend=None,
    ) -> tuple[list, list[list], list[tuple[int, int]]]:
        """Replace each invalid direct connection with an A* planned path.

        Returns (new_points, connector_viz, unresolved) where connector_viz is
        a list of point lists (one per connector) for yellow visualisation
        markers. Unresolved segments must not be published as a runnable path.
        """
        invalid_set = set(invalid_segs)
        new_points: list = []
        connector_viz: list[list] = []
        unresolved: list[tuple[int, int]] = []

        for i, pt in enumerate(points):
            new_points.append(pt)
            if i >= len(points) - 1:
                continue
            if (i, i + 1) not in invalid_set:
                continue

            _backend = backend if backend is not None else self._backend
            connector = _backend.plan_connector(pt, points[i + 1], safe_map_struct)
            if connector and len(connector) > 2:
                for cp in connector[1:-1]:   # skip endpoints (already in pts)
                    new_points.append(cp)
                connector_viz.append(connector)
                self.get_logger().info(
                    f'zone {zone_idx}: connector ({i}→{i+1}) '
                    f'{len(connector)} pts'
                )
            else:
                unresolved.append((i, i + 1))
                self.get_logger().error(
                    f'zone {zone_idx}: no safe connector from '
                    f'{pt} to {points[i+1]}'
                )

        return new_points, connector_viz, unresolved

    def _publish_connectors(
        self,
        connector_viz: list[list],
        zone_idx: int,
        frame_id: str,
    ):
        """Publish yellow line markers for each A* connector path."""
        marker_array = MarkerArray()
        for k, pts in enumerate(connector_viz):
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = f'zone_{zone_idx}_connector'
            m.id = zone_idx * 10000 + k
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.04
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 1.0
            for x, y in pts:
                pt = Point()
                pt.x, pt.y = float(x), float(y)
                m.points.append(pt)
            marker_array.markers.append(m)
        self.connectors_pub.publish(marker_array)

    def _publish_invalid_segments(
        self,
        points: list,
        invalid_segments: list,
        zone_idx: int,
        frame_id: str,
    ):
        """Publish red line markers for each unsafe segment."""
        marker_array = MarkerArray()
        for k, (i, j) in enumerate(invalid_segments):
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = f'zone_{zone_idx}_invalid'
            m.id = zone_idx * 10000 + k
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.05
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = 1.0
            for idx in (i, j):
                pt = Point()
                pt.x, pt.y = float(points[idx][0]), float(points[idx][1])
                m.points.append(pt)
            marker_array.markers.append(m)
        self.invalid_segments_pub.publish(marker_array)

    def zone_exec_path_srv(self, req, res):
        """
        Execute a path for a given zone.

        Args
        ----
        req : ZoneExecPath.Request
            The service request containing the zone_id to execute.
        res : ZoneExecPath.Response
            The service response containing success status and message.

        """
        self.get_logger().info(f'zone_exec_path_srv start: {req.zone_id}')
        zone_id = req.zone_id
        zone_map = None
        for zone in self.zone_map_list:
            if zone.zone_id == zone_id:
                zone_map = zone
                break

        if not zone_map:
            res.success = False
            res.message = 'Zone not found'
            return res

        # Fire-and-forget on the executor-spun client (bounded server wait);
        # execution runs in the background. See _send_follow_path.
        dispatched = self._send_follow_path(
            zone_map.path, zone_map.coverage_split_points, block=False
        )
        res.success = dispatched
        res.message = (
            'Goal sent to navigation action server'
            if dispatched else 'nav action server 不可用'
        )
        return res

    # ── Zone Sequence Mission ─────────────────────────────────────────────────

    def run_zone_sequence_srv(self, req, res):
        """啟動多 zone 任務序列：覆蓋 → 走通道 → 覆蓋 → ..."""
        if self._sequence_thread and self._sequence_thread.is_alive():
            res.success = False
            res.message = '已有任務序列執行中，請先呼叫 /stop_zone_sequence'
            return res

        zone_ids = list(req.zone_ids)
        if not zone_ids:
            res.success = False
            res.message = 'zone_ids 不可為空'
            return res

        missing = [
            z for z in zone_ids
            if self._get_zone_map(z) is None
            or not self._get_zone_map(z).path.poses
        ]
        if missing:
            res.success = False
            res.message = (
                f'以下 zone 尚無覆蓋路徑，請先呼叫 /generate_coverage_path: '
                f'{missing}'
            )
            return res

        proximity_m = (
            float(req.channel_proximity_m) if req.channel_proximity_m > 0 else 1.5
        )
        self._sequence_cancel.clear()
        self._sequence_thread = threading.Thread(
            target=self._run_sequence,
            args=(zone_ids, proximity_m),
            daemon=True,
        )
        self._sequence_thread.start()

        res.success = True
        res.message = (
            f'任務序列已啟動: zones={zone_ids}, '
            f'channel_proximity={proximity_m:.2f}m'
        )
        return res

    def stop_zone_sequence_srv(self, req, res):
        """取消正在執行的任務序列."""
        self._sequence_cancel.set()
        res.success = True
        res.message = (
            '已送出取消請求，序列將在當前步驟完成後停止'
            if self._sequence_thread and self._sequence_thread.is_alive()
            else '目前無執行中的任務序列'
        )
        return res

    def _run_sequence(self, zone_ids, channel_proximity_m):
        """背景執行緒：依序完成多個 zone 覆蓋，zone 間走通道銜接."""
        self.get_logger().info(f'任務序列開始: zones={zone_ids}')

        for i, zone_id in enumerate(zone_ids):
            if self._sequence_cancel.is_set():
                self.get_logger().warn(f'任務序列在 zone {zone_id} 前已取消')
                return

            zone_map = self._get_zone_map(zone_id)

            self.get_logger().info(
                f'[{i + 1}/{len(zone_ids)}] 執行 zone {zone_id} 覆蓋路徑，'
                f'共 {len(zone_map.path.poses)} 個路徑點'
            )
            ok = self._send_follow_path(
                zone_map.path, list(zone_map.coverage_split_points), block=True
            )
            if not ok:
                self.get_logger().error(
                    f'Zone {zone_id} 覆蓋路徑執行失敗或被取消，任務序列中止'
                )
                return

            self.get_logger().info(f'Zone {zone_id} 覆蓋完成')

            if self._sequence_cancel.is_set() or i >= len(zone_ids) - 1:
                continue

            next_zone_id = zone_ids[i + 1]
            self.get_logger().info(
                f'尋找通道: zone {zone_id} → zone {next_zone_id}'
            )

            route_req = ChannelRoute.Request()
            route_req.zone_from_id = zone_id
            route_req.zone_to_id = next_zone_id
            route_req.proximity_m = channel_proximity_m
            route_res = self._blocking_service_call(
                self._channel_route_client, route_req
            )

            if route_res is None or not route_res.success:
                msg = route_res.message if route_res else '服務呼叫超時'
                self.get_logger().error(
                    f'取得通道路徑失敗: {msg}，任務序列中止'
                )
                return

            self.get_logger().info(
                f'走通道 #{route_res.matched_channel_id} '
                f'(zone {zone_id} → zone {next_zone_id})，'
                f'共 {len(route_res.channel_path.poses)} 個路徑點'
            )
            ok = self._send_follow_path(route_res.channel_path, [], block=True)
            if not ok:
                self.get_logger().error(
                    f'通道 {zone_id}→{next_zone_id} 導航失敗或被取消，任務序列中止'
                )
                return

            self.get_logger().info(
                f'通道 zone {zone_id} → zone {next_zone_id} 完成'
            )

        self.get_logger().info(f'任務序列全部完成: zones={zone_ids}')

    def _get_zone_map(self, zone_id):
        """從 zone_map_list 查找指定 zone_id 的 ZoneMap."""
        for z in self.zone_map_list:
            if z.zone_id == zone_id:
                return z
        return None

    def _send_follow_path(
        self, path, coverage_split_points, block=False, timeout_s=600.0
    ):
        """Send a nav_action_follow_path goal on the executor-spun client — the
        single source of truth for both /zone_exec_path and the zone sequence.

        block=False → fire-and-forget (returns True once dispatched).
        block=True  → wait for the result (returns the success bool).
        """
        if not self._nav_follow_path_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('nav action server 不可用')
            return False

        goal = Waypoint.Goal()
        goal.path = path
        goal.coverage_split_points = list(coverage_split_points)

        if not block:
            self._exec_goal_future = (
                self._nav_follow_path_client.send_goal_async(goal)
            )
            return True

        done = threading.Event()
        result_box = [False]

        def _on_goal(future):
            handle = future.result()
            if not handle.accepted:
                self.get_logger().warn('Nav goal 被 action server 拒絕')
                done.set()
                return
            handle.get_result_async().add_done_callback(_on_result)

        def _on_result(future):
            result_box[0] = future.result().result.success
            done.set()

        self._nav_follow_path_client.send_goal_async(goal).add_done_callback(
            _on_goal
        )
        if not done.wait(timeout=timeout_s):
            self.get_logger().error(f'Nav action 超時（{timeout_s:.0f}s）')
            return False
        return result_box[0]

    def _blocking_service_call(self, client, req, timeout_s=10.0):
        """呼叫 ROS2 service 並阻塞直到收到回應，回傳 response 或 None."""
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service 不可用')
            return None

        done = threading.Event()
        result_box = [None]

        def _cb(future):
            try:
                result_box[0] = future.result()
            except Exception as e:
                self.get_logger().error(f'Service call 異常: {e}')
            done.set()

        client.call_async(req).add_done_callback(_cb)

        if not done.wait(timeout=timeout_s):
            self.get_logger().error(f'Service call 超時（{timeout_s:.0f}s）')
            return None
        return result_box[0]

    def record_path_status_srv(self, req, res):
        """Record the status of the path."""
        self.waypoint_active = req.data
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
