"""Pure-Python converters from ROS message shapes to Flutter-friendly DTOs.

No rclpy or message-class imports — accepts duck-typed objects so the module
stays unit-testable without a live ROS bus. See
docs/flutter_frontend_function_spec.md §7 for the target DTO shapes.
"""

from __future__ import annotations

import base64
import math
from typing import Any, Iterable


# visualization_msgs/Marker.type → snake_case label.
_MARKER_TYPE_LABELS = {
    0: 'arrow',
    1: 'cube',
    2: 'sphere',
    3: 'cylinder',
    4: 'line_strip',
    5: 'line_list',
    6: 'cube_list',
    7: 'sphere_list',
    8: 'points',
    9: 'text_view_facing',
    10: 'mesh_resource',
    11: 'triangle_list',
}


def _marker_type_label(type_int: int) -> str:
    return _MARKER_TYPE_LABELS.get(type_int, f'type_{type_int}')


def color_rgba_to_hex(color) -> str:
    """std_msgs/ColorRGBA (rgba 0..1 floats) → #rrggbb."""
    def byte(v: float) -> int:
        return max(0, min(255, int(round(v * 255))))
    return f'#{byte(color.r):02x}{byte(color.g):02x}{byte(color.b):02x}'


def quaternion_to_yaw(q) -> float:
    """Extract yaw (Z-axis rotation) from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def is_navsat_datum_point(lat: float, lon: float) -> bool:
    """Whether a /toLL answer is a navsat datum point.

    navsat_transform answers the default (0, 0, 0) until its datum exists;
    anything non-finite (a UTM projection without a zone) is no datum either
    and must never be latched. ``abs(nan) < 1e-6`` is False, so the finite
    check has to be explicit.
    """
    lat = float(lat)
    lon = float(lon)
    if not (math.isfinite(lat) and math.isfinite(lon)):
        return False
    return abs(lat) >= 1e-6 or abs(lon) >= 1e-6


def occupancy_grid_to_map_layer(name: str, msg) -> dict:
    """nav_msgs/OccupancyGrid → MapLayer DTO (base64-encoded int8 data)."""
    info = msg.info
    raw = bytes((b & 0xFF) for b in msg.data)  # int8 → uint8 wire repr
    return {
        'name': name,
        'type': 'occupancy_grid',
        'resolution': info.resolution,
        'width': info.width,
        'height': info.height,
        'origin': {
            'x': info.origin.position.x,
            'y': info.origin.position.y,
        },
        'encoding': 'base64',
        'data': base64.b64encode(raw).decode('ascii'),
    }


def marker_array_to_marker_layer(name: str, msg) -> dict:
    """visualization_msgs/MarkerArray → MarkerLayer DTO."""
    out = []
    for marker in msg.markers:
        out.append({
            'id': marker.id,
            'type': _marker_type_label(marker.type),
            'color': color_rgba_to_hex(marker.color),
            'points': [{'x': p.x, 'y': p.y} for p in marker.points],
        })
    return {'name': name, 'markers': out}


def coverage_points_by_zone(marker_array) -> dict[int, int]:
    """Points of each zone's planned path in a /coverage_path_markers snapshot.

    mower_coverage draws zone N's path as LINE_STRIP markers in namespace
    ``zone_N_path`` (arrows and connectors use other namespaces). A snapshot
    without them (no plan yet, or a DELETEALL) yields no zones.
    """
    line_strip, add = 4, 0
    out: dict[int, int] = {}
    for m in getattr(marker_array, 'markers', []):
        if m.type != line_strip or m.action != add:
            continue
        ns = m.ns or ''
        if not (ns.startswith('zone_') and ns.endswith('_path')):
            continue
        try:
            zone = int(ns[len('zone_'):-len('_path')])
        except ValueError:
            continue
        out[zone] = out.get(zone, 0) + len(m.points)
    return out


def zone_map_list_to_summaries(zone_map_list: Iterable,
                               coverage_points: dict[int, int] | None = None) -> list[dict]:
    """mower_interface/ZoneMap[] → ZoneSummary[].

    map_manage's ZoneMap.path is never filled (the planner keeps its paths to
    itself), so a zone's planned path is taken from the coverage markers
    (``coverage_points_by_zone``); a path in the ZoneMap still counts.
    """
    coverage_points = coverage_points or {}
    summaries = []
    for zm in zone_map_list:
        inflated = getattr(zm, 'mask_map_inflated', None)
        path = getattr(zm, 'path', None)
        path_poses = getattr(path, 'poses', []) if path is not None else []
        has_map = bool(inflated and getattr(inflated, 'data', []))
        point_count = max(len(path_poses), coverage_points.get(zm.zone_id, 0))
        summaries.append({
            'zoneId': zm.zone_id,
            'pointCount': point_count,
            'hasMap': has_map,
            'hasCoveragePath': point_count > 0,
        })
    return summaries


def params_to_coverage_settings(coverage_params: dict[str, Any],
                                map_params: dict[str, Any]) -> dict:
    """Compose CoverageSettings DTO from parameter snapshots."""
    return {
        'stripWidthM': coverage_params.get('strip_width_m'),
        'waypointSpacingM': coverage_params.get('waypoint_spacing_m'),
        'zigzagAngleDeg': coverage_params.get('zigzag_angle_deg'),
        'zigzagAutoAngle': coverage_params.get('zigzag_auto_angle'),
        'inflateRadiusM': map_params.get('inflate_radius_m'),
        'unknownAsObstacle': coverage_params.get('unknown_as_obstacle'),
        'coveragePattern': coverage_params.get('coverage_pattern'),
        'boundaryRing': coverage_params.get('boundary_ring'),
    }


def service_response_to_envelope(resp) -> dict:
    """Normalise std_srvs/Trigger and custom srv responses to {success, message}.

    Custom srvs without a `success` field are treated as success=True so the
    Flutter app can show a uniform envelope.
    """
    success = getattr(resp, 'success', True)
    message = getattr(resp, 'message', '') or ''
    return {'success': bool(success), 'message': message}
