"""Flutter adapter node — republishes ROS messages as compact JSON DTOs.

Wraps the Flutter-facing topics documented in
docs/flutter_frontend_function_spec.md §6 / §7 so the mobile client can stay
oblivious to TF, OccupancyGrid byte layout, and MarkerArray traversal.

All ROS-free conversion logic lives in `mower_mission.adapters.dto`.
"""

from __future__ import annotations

import json
import math

from geometry_msgs.msg import Point, PoseStamped
from mower_interface.srv import ZoneMapList
from nav_msgs.msg import OccupancyGrid, Odometry
from robot_localization.srv import ToLL
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from mower_mission.adapters.dto import (
    is_navsat_datum_point,
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
    'zigzag_angle_deg',
    'zigzag_auto_angle',
    'unknown_as_obstacle',
    'coverage_pattern',
    'boundary_ring',
]
_MAP_PARAM_NAMES = ['inflate_radius_m']

_DATUM_PROBE_M = 10.0  # metres along map +X used to derive the datum bearing


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
            # Marker arrays are complete layer snapshots. Keep both relay legs
            # latched so an adapter or app that starts later receives the same
            # current layer instead of retaining demo/previous-session data.
            self._marker_pubs[name] = self.create_publisher(
                String, adapter_topic, latched
            )
            self.create_subscription(
                MarkerArray, src,
                self._make_marker_cb(name), latched,
            )
            self._marker_pubs[name].publish(String(data=json.dumps({
                'name': name,
                'markers': [],
            })))

        self._robot_pose_pub = self.create_publisher(
            PoseStamped, '/adapter/robot_pose', 10
        )
        self.declare_parameter('robot_pose_max_age_s', 1.0)
        # The map -> base_footprint pose comes straight from the EKF map
        # filter's odometry output. mission.launch.py points this at the
        # 5 Hz throttled copy /odometry/global_slow; the raw 30 Hz topic is
        # the default so the node also works in the simulation launches and
        # under a bare `ros2 run`. A tf2 TransformListener would instead
        # subscribe to the whole 77 Hz /tf stream, and rclpy costs ~6 ms of
        # CPU per message on the LubanCat, which made this node the busiest
        # process on the robot for a 5 Hz relay.
        self.declare_parameter('robot_pose_source_topic', '/odometry/global')
        self.declare_parameter('robot_pose_source_frame', 'map')
        self.declare_parameter('robot_pose_child_frame', 'base_footprint')
        self.create_subscription(
            Odometry,
            str(self.get_parameter('robot_pose_source_topic').value),
            self._on_robot_pose_source,
            10,
        )

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

        # Map datum (geo-reference for the satellite base map). Prefer the live
        # navsat_transform datum (toLL). A configured fallback is opt-in only:
        # publishing a real-looking fixed coordinate when GPS is unavailable
        # can project a saved site onto the wrong physical location.
        self.declare_parameter('enable_map_datum_fallback', False)
        self.declare_parameter('map_datum_fallback_lat', 23.6939508)
        self.declare_parameter('map_datum_fallback_lon', 120.5376539)
        self.declare_parameter('map_datum_fallback_bearing_deg', 0.0)
        self._map_datum_pub = self.create_publisher(
            String, '/adapter/map_datum', latched
        )
        self._toll_client = self.create_client(ToLL, '/toLL')
        self._datum_locked = False
        self._fallback_disabled_logged = False
        self.create_timer(2.0, self._publish_map_datum)

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

    def _on_robot_pose_source(self, odom: Odometry) -> None:
        """Relay a validated map-frame odometry sample as /adapter/robot_pose."""
        stamp_ns = (
            int(odom.header.stamp.sec) * 1_000_000_000
            + int(odom.header.stamp.nanosec)
        )
        age_s = (
            (self.get_clock().now().nanoseconds - stamp_ns)
            / 1_000_000_000.0
        ) if stamp_ns > 0 else math.inf
        position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        values = (
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        quaternion_norm = math.sqrt(sum(
            float(value) ** 2 for value in values[3:]
        ))
        max_age_s = max(
            0.1,
            float(self.get_parameter('robot_pose_max_age_s').value),
        )
        source_frame = str(self.get_parameter('robot_pose_source_frame').value)
        child_frame = str(self.get_parameter('robot_pose_child_frame').value)
        if (
            odom.header.frame_id != source_frame
            or odom.child_frame_id != child_frame
            or not all(math.isfinite(float(value)) for value in values)
            or not math.isfinite(quaternion_norm)
            or abs(quaternion_norm - 1.0) > 1e-2
            or not -0.5 <= age_s <= max_age_s
        ):
            # Do not turn a frozen/invalid pose into a fresh pose relay.
            return

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
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

    # ── map datum (satellite geo-reference) ──────────────────────────────────

    def _publish_map_datum(self) -> None:
        # Once navsat gives a real datum we lock it (latched pub keeps it live).
        if self._datum_locked:
            return
        if not self._toll_client.service_is_ready():
            self._publish_fallback_datum()
            return
        req = ToLL.Request()
        req.map_point = Point(x=0.0, y=0.0, z=0.0)
        self._toll_client.call_async(req).add_done_callback(self._on_datum_origin)

    def _on_datum_origin(self, future) -> None:
        try:
            res = future.result()
        except Exception:  # noqa: BLE001
            res = None
        if res is None or not is_navsat_datum_point(
            res.ll_point.latitude, res.ll_point.longitude
        ):
            # navsat datum not established (the default (0, 0, 0) answer, or
            # a non-finite one) → fallback.
            self._publish_fallback_datum()
            return
        lat0 = res.ll_point.latitude
        lon0 = res.ll_point.longitude
        req = ToLL.Request()
        req.map_point = Point(x=_DATUM_PROBE_M, y=0.0, z=0.0)
        self._toll_client.call_async(req).add_done_callback(
            lambda fut: self._on_datum_bearing(fut, lat0, lon0)
        )

    def _on_datum_bearing(self, future, lat0: float, lon0: float) -> None:
        try:
            res = future.result()
        except Exception:  # noqa: BLE001
            res = None
        if res is None or not is_navsat_datum_point(
            res.ll_point.latitude, res.ll_point.longitude
        ):
            self._publish_fallback_datum()
            return
        bearing = self._bearing_to_north(
            lat0, lon0, res.ll_point.latitude, res.ll_point.longitude
        )
        self._publish_datum(lat0, lon0, bearing, 'navsat')
        self._datum_locked = True
        self.get_logger().info(
            f'map datum from navsat: ({lat0:.6f}, {lon0:.6f}), '
            f'bearing {math.degrees(bearing):.1f} deg'
        )

    def _publish_fallback_datum(self) -> None:
        if not bool(
            self.get_parameter('enable_map_datum_fallback').value
        ):
            if not self._fallback_disabled_logged:
                self.get_logger().warn(
                    'map datum unavailable; fixed fallback is disabled'
                )
                self._fallback_disabled_logged = True
            return
        lat = float(self.get_parameter('map_datum_fallback_lat').value)
        lon = float(self.get_parameter('map_datum_fallback_lon').value)
        bearing = math.radians(
            float(self.get_parameter('map_datum_fallback_bearing_deg').value)
        )
        self._publish_datum(lat, lon, bearing, 'fallback')

    def _publish_datum(
        self, lat: float, lon: float, bearing_rad: float, source: str
    ) -> None:
        self._map_datum_pub.publish(String(data=json.dumps({
            'origin_lat': lat,
            'origin_lon': lon,
            'bearing_rad': bearing_rad,
            'source': source,
        })))

    @staticmethod
    def _bearing_to_north(
        lat0: float, lon0: float, lat1: float, lon1: float
    ) -> float:
        """Bearing of the map +X axis clockwise from true north, derived from
        two toLL samples taken along map +X."""
        mlat = 111320.0
        mlon = 111320.0 * math.cos(math.radians(lat0))
        east = (lon1 - lon0) * mlon
        north = (lat1 - lat0) * mlat
        return math.atan2(east, north)


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
