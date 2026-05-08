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

from mower_interface.srv import ZoneExecPath

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

from .coverage.connector_planner import plan_connector
from .coverage.path_validator import SafeMap, validate_path
from .coverage.safe_map_filter import filter_safe_components
from .path_generators.speiral import _generate_coverage_spiral_path
from .path_generators.zigzag import _generate_coverage_zigzag_path
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
        self.get_logger().info('boustrophedon_coverage 初始化')
        self.declare_parameter('strip_width_m', 0.8)
        self.declare_parameter('waypoint_spacing_m', 0.2)
        self.declare_parameter('unknown_as_obstacle', True)
        self.declare_parameter('min_safe_component_area_m2', 0.05)
        self.declare_parameter('coverage_pattern', 'zigzag')

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
        self.create_service(Trigger, '/cencel_nav2', self.cancel_nav2_srv,
                            callback_group=self.cb_group)
        self.create_service(Trigger, '/check_nav_status',
                            self.check_nav_status_srv,
                            callback_group=self.cb_group)

        self.waypoint_active_client = self.create_client(
            SetBool,
            '/record_path_status',
        )
        self._action_client_split_path = NavActionClient()
        self.zone_map_client = ZoneMapClient()

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
        self.zone_map_list = self.zone_map_client.get_zone_maps()
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
                filter_safe_components(
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
                waypoint_spacing_m=self.get_parameter('waypoint_spacing_m').value,
                res=res,
                H=H,
                W=W,
                origin_x=ox,
                origin_y=oy,
            )
            if pattern == 'spiral':
                coverage_pts, split_pts, invalid_segs = (
                    _generate_coverage_spiral_path(**_gen_kw)
                )
            else:
                coverage_pts, split_pts, invalid_segs = (
                    _generate_coverage_zigzag_path(**_gen_kw, angle_deg=0.0)
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
                    coverage_pts, invalid_segs, safe_map_struct, i
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

            final_validation = validate_path(coverage_pts, safe_map_struct)
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

            connector = plan_connector(pt, points[i + 1], safe_map_struct)
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

        if zone_map:
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
