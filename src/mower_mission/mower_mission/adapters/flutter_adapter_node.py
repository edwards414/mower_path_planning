"""Flutter adapter node — republishes ROS messages as compact JSON DTOs.

Wraps the Flutter-facing topics documented in
docs/flutter_frontend_function_spec.md §6 / §7 so the mobile client can stay
oblivious to TF, OccupancyGrid byte layout, and MarkerArray traversal.

All ROS-free conversion logic lives in `mower_mission.adapters.dto`.
"""

from __future__ import annotations

import json

from geometry_msgs.msg import PoseStamped
from mower_interface.srv import ZoneMapList
from nav_msgs.msg import OccupancyGrid
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import String
from tf2_ros import Buffer, LookupException, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from mower_mission.adapters.dto import (
    marker_array_to_marker_layer,
    occupancy_grid_to_map_layer,
    params_to_coverage_settings,
    zone_map_list_to_summaries,
)


_MAP_TOPICS = [
    ('/map_grid', 'map_grid'),
    ('/free_space_inflated', 'free_space_inflated'),
    ('/risk_map_inflated', 'risk_map_inflated'),
    ('/chennal_map_inflated', 'chennal_map_inflated'),
]

_MARKER_TOPICS = [
    ('/zone_list', 'zones'),
    ('/risk_zone_list', 'risk_zones'),
    ('/chennal_path_array', 'channels'),
    ('/coverage_path_markers', 'coverage_path'),
    ('/coverage_invalid_segments', 'invalid_segments'),
    ('/coverage_connectors', 'connectors'),
]

_COVERAGE_PARAM_NAMES = [
    'strip_width_m',
    'waypoint_spacing_m',
    'unknown_as_obstacle',
    'coverage_pattern',
]
_MAP_PARAM_NAMES = ['inflate_radius_m']


def _latched_qos() -> QoSProfile:
    qos = QoSProfile(depth=1)
    qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = QoSReliabilityPolicy.RELIABLE
    return qos


class FlutterAdapter(Node):

    def __init__(self):
        super().__init__('flutter_adapter')
        self.get_logger().info('flutter_adapter init')

        latched = _latched_qos()
        self._map_pubs: dict[str, rclpy.publisher.Publisher] = {}
        self._marker_pubs: dict[str, rclpy.publisher.Publisher] = {}

        for src, name in _MAP_TOPICS:
            adapter_topic = f'/adapter/map_layers/{name}'
            self._map_pubs[name] = self.create_publisher(
                String, adapter_topic, latched
            )
            self.create_subscription(
                OccupancyGrid, src,
                self._make_grid_cb(name), latched,
            )

        for src, name in _MARKER_TOPICS:
            adapter_topic = f'/adapter/marker_layers/{name}'
            self._marker_pubs[name] = self.create_publisher(
                String, adapter_topic, 10
            )
            self.create_subscription(
                MarkerArray, src,
                self._make_marker_cb(name), 10,
            )

        self._robot_pose_pub = self.create_publisher(
            PoseStamped, '/adapter/robot_pose', 10
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_timer(0.2, self._publish_robot_pose)  # 5 Hz

        # Parameter snapshots are accumulated asynchronously by callbacks and
        # published from the latest cached values on every coverage_settings
        # tick. None means "not yet known".
        self._coverage_param_cache: dict[str, object] | None = None
        self._map_param_cache: dict[str, object] | None = None

        self._coverage_settings_pub = self.create_publisher(
            String, '/adapter/coverage_settings', latched
        )
        self._coverage_param_client = self.create_client(
            GetParameters, '/boustrophedon_coverage/get_parameters'
        )
        self._map_param_client = self.create_client(
            GetParameters, '/map_manage/get_parameters'
        )
        self.create_timer(1.0, self._publish_coverage_settings)

        self._zone_summary_pub = self.create_publisher(
            String, '/adapter/zone_summaries', latched
        )
        self._zone_map_list_client = self.create_client(
            ZoneMapList, '/get_zone_map_list_srv'
        )
        self.create_timer(0.5, self._publish_zone_summaries)

    # ── topic relays ─────────────────────────────────────────────────────────

    def _make_grid_cb(self, name: str):
        def cb(msg: OccupancyGrid) -> None:
            try:
                dto = occupancy_grid_to_map_layer(name, msg)
                self._map_pubs[name].publish(String(data=json.dumps(dto)))
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(
                    f'failed to encode {name}: {exc!r}'
                )
        return cb

    def _make_marker_cb(self, name: str):
        def cb(msg: MarkerArray) -> None:
            try:
                dto = marker_array_to_marker_layer(name, msg)
                self._marker_pubs[name].publish(String(data=json.dumps(dto)))
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(
                    f'failed to encode markers {name}: {exc!r}'
                )
        return cb

    # ── robot pose ───────────────────────────────────────────────────────────

    def _publish_robot_pose(self) -> None:
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
        except (LookupException, TransformException):
            return  # TF not yet available — just skip this tick

        pose = PoseStamped()
        pose.header = tf.header
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation
        self._robot_pose_pub.publish(pose)

    # ── coverage settings snapshot ───────────────────────────────────────────

    def _publish_coverage_settings(self) -> None:
        # Kick off async refreshes, then publish whatever we have cached.
        self._refresh_param_cache(
            self._coverage_param_client, _COVERAGE_PARAM_NAMES,
            attr='_coverage_param_cache',
        )
        self._refresh_param_cache(
            self._map_param_client, _MAP_PARAM_NAMES,
            attr='_map_param_cache',
        )
        if self._coverage_param_cache is None and self._map_param_cache is None:
            return
        dto = params_to_coverage_settings(
            self._coverage_param_cache or {},
            self._map_param_cache or {},
        )
        self._coverage_settings_pub.publish(String(data=json.dumps(dto)))

    def _refresh_param_cache(self, client, names, *, attr: str) -> None:
        if not client.service_is_ready():
            return
        future = client.call_async(GetParameters.Request(names=names))

        def _store(fut, attr=attr, names=tuple(names)) -> None:
            try:
                response = fut.result()
            except Exception:  # noqa: BLE001
                return
            if response is None:
                return
            cache = {
                n: self._extract_param_value(v)
                for n, v in zip(names, response.values)
            }
            setattr(self, attr, cache)

        future.add_done_callback(_store)

    @staticmethod
    def _extract_param_value(value):
        # rcl_interfaces/msg/ParameterValue carries a discriminated `type` field.
        type_idx = value.type
        if type_idx == 1:
            return value.bool_value
        if type_idx == 2:
            return value.integer_value
        if type_idx == 3:
            return value.double_value
        if type_idx == 4:
            return value.string_value
        if type_idx == 5:
            return list(value.byte_array_value)
        if type_idx == 6:
            return list(value.bool_array_value)
        if type_idx == 7:
            return list(value.integer_array_value)
        if type_idx == 8:
            return list(value.double_array_value)
        if type_idx == 9:
            return list(value.string_array_value)
        return None

    # ── zone summaries ───────────────────────────────────────────────────────

    def _publish_zone_summaries(self) -> None:
        if not self._zone_map_list_client.service_is_ready():
            return
        future = self._zone_map_list_client.call_async(ZoneMapList.Request())
        future.add_done_callback(self._on_zone_summary_response)

    def _on_zone_summary_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'zone_map_list call failed: {exc!r}')
            return
        if response is None:
            return
        summaries = zone_map_list_to_summaries(response.zone_map_list)
        self._zone_summary_pub.publish(String(data=json.dumps(summaries)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FlutterAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
