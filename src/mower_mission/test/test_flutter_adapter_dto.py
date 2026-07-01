"""Unit tests for adapters/dto.py — Flutter adapter DTO converters.

These tests construct ROS-message-shaped objects via SimpleNamespace so the
converters stay testable without importing rclpy / nav_msgs / etc.
"""

import base64
from types import SimpleNamespace

import pytest

from mower_mission.adapters.dto import (
    color_rgba_to_hex,
    marker_array_to_marker_layer,
    occupancy_grid_to_map_layer,
    params_to_coverage_settings,
    quaternion_to_yaw,
    service_response_to_envelope,
    zone_map_list_to_summaries,
)


def _make_occupancy_grid(width=4, height=3, resolution=0.05,
                         origin_x=-1.0, origin_y=-2.0,
                         data=None):
    if data is None:
        data = list(range(width * height))
    return SimpleNamespace(
        info=SimpleNamespace(
            width=width,
            height=height,
            resolution=resolution,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=origin_x, y=origin_y, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=data,
    )


def _make_color(r, g, b, a=1.0):
    return SimpleNamespace(r=r, g=g, b=b, a=a)


def _make_point(x, y, z=0.0):
    return SimpleNamespace(x=x, y=y, z=z)


def _make_marker(marker_id, marker_type, points, color):
    return SimpleNamespace(
        id=marker_id, type=marker_type, points=points, color=color,
    )


# ──────────────────────────────────────────────────────────────────────────────
# OccupancyGrid → MapLayer
# ──────────────────────────────────────────────────────────────────────────────


def test_occupancy_grid_dto_shape():
    grid = _make_occupancy_grid(
        width=2, height=2, resolution=0.05,
        origin_x=-1.5, origin_y=-2.5,
        data=[0, 100, -1, 50],
    )
    dto = occupancy_grid_to_map_layer('risk_map_inflated', grid)
    assert dto['name'] == 'risk_map_inflated'
    assert dto['type'] == 'occupancy_grid'
    assert dto['width'] == 2
    assert dto['height'] == 2
    assert dto['resolution'] == pytest.approx(0.05)
    assert dto['origin'] == {'x': -1.5, 'y': -2.5}
    assert dto['encoding'] == 'base64'


def test_occupancy_grid_data_base64_roundtrips():
    raw = [0, 100, -1, 50, 25, 75]
    grid = _make_occupancy_grid(width=3, height=2, data=raw)
    dto = occupancy_grid_to_map_layer('free_space', grid)
    decoded = base64.b64decode(dto['data'])
    # data is signed int8 in OccupancyGrid — values like -1 should survive
    decoded_signed = [b if b < 128 else b - 256 for b in decoded]
    assert decoded_signed == raw


def test_occupancy_grid_data_length_matches_width_times_height():
    grid = _make_occupancy_grid(width=10, height=4, data=[0] * 40)
    dto = occupancy_grid_to_map_layer('map_grid', grid)
    decoded = base64.b64decode(dto['data'])
    assert len(decoded) == 40


# ──────────────────────────────────────────────────────────────────────────────
# MarkerArray → MarkerLayer
# ──────────────────────────────────────────────────────────────────────────────


# visualization_msgs/Marker.type constants
LINE_STRIP = 4
POINTS = 8
SPHERE = 2


def test_marker_array_line_strip():
    marker = _make_marker(
        marker_id=7,
        marker_type=LINE_STRIP,
        points=[_make_point(0.1, 0.2), _make_point(1.0, 1.5)],
        color=_make_color(1.0, 0.0, 0.0, 1.0),
    )
    msg = SimpleNamespace(markers=[marker])
    dto = marker_array_to_marker_layer('invalid_segments', msg)
    assert dto['name'] == 'invalid_segments'
    assert len(dto['markers']) == 1
    out = dto['markers'][0]
    assert out['id'] == 7
    assert out['type'] == 'line_strip'
    assert out['color'] == '#ff0000'
    assert out['points'] == [
        {'x': 0.1, 'y': 0.2},
        {'x': 1.0, 'y': 1.5},
    ]


def test_marker_array_points_type_label():
    marker = _make_marker(
        marker_id=1, marker_type=POINTS,
        points=[_make_point(0.0, 0.0)],
        color=_make_color(0.0, 1.0, 0.0, 1.0),
    )
    msg = SimpleNamespace(markers=[marker])
    dto = marker_array_to_marker_layer('split_points', msg)
    assert dto['markers'][0]['type'] == 'points'
    assert dto['markers'][0]['color'] == '#00ff00'


def test_marker_array_unknown_type_falls_back_to_int_string():
    marker = _make_marker(
        marker_id=2, marker_type=99,
        points=[],
        color=_make_color(0.0, 0.0, 1.0, 1.0),
    )
    msg = SimpleNamespace(markers=[marker])
    dto = marker_array_to_marker_layer('weird', msg)
    assert dto['markers'][0]['type'] == 'type_99'


def test_marker_array_empty():
    msg = SimpleNamespace(markers=[])
    dto = marker_array_to_marker_layer('zones', msg)
    assert dto == {'name': 'zones', 'markers': []}


# ──────────────────────────────────────────────────────────────────────────────
# Color helper
# ──────────────────────────────────────────────────────────────────────────────


def test_color_rgba_to_hex_clamps_and_rounds():
    assert color_rgba_to_hex(_make_color(1.0, 0.0, 0.0)) == '#ff0000'
    assert color_rgba_to_hex(_make_color(0.5, 0.5, 0.5)) == '#808080'
    # values outside [0,1] should clamp
    assert color_rgba_to_hex(_make_color(1.5, -0.2, 0.0)) == '#ff0000'


# ──────────────────────────────────────────────────────────────────────────────
# Quaternion → yaw helper
# ──────────────────────────────────────────────────────────────────────────────


def test_quaternion_to_yaw_identity():
    yaw = quaternion_to_yaw(SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
    assert yaw == pytest.approx(0.0)


def test_quaternion_to_yaw_90_degrees():
    # 90° rotation about Z axis: q = (0, 0, sin(45°), cos(45°))
    import math
    s = math.sin(math.pi / 4)
    c = math.cos(math.pi / 4)
    yaw = quaternion_to_yaw(SimpleNamespace(x=0.0, y=0.0, z=s, w=c))
    assert yaw == pytest.approx(math.pi / 2)


# ──────────────────────────────────────────────────────────────────────────────
# ZoneMap[] → ZoneSummary[]
# ──────────────────────────────────────────────────────────────────────────────


def _make_zone_map(zone_id, mask_data=(), inflated_data=(), path_poses=()):
    return SimpleNamespace(
        zone_id=zone_id,
        mask_map=SimpleNamespace(data=list(mask_data)),
        mask_map_inflated=SimpleNamespace(data=list(inflated_data)),
        path=SimpleNamespace(poses=list(path_poses)),
        coverage_split_points=[],
    )


def test_zone_summaries_full():
    zm = _make_zone_map(
        zone_id=1,
        inflated_data=[0] * 4,
        path_poses=[object(), object(), object()],
    )
    summaries = zone_map_list_to_summaries([zm])
    assert summaries == [{
        'zoneId': 1,
        'pointCount': 3,
        'hasMap': True,
        'hasCoveragePath': True,
    }]


def test_zone_summaries_no_map_no_path():
    zm = _make_zone_map(zone_id=2)
    summaries = zone_map_list_to_summaries([zm])
    assert summaries[0]['hasMap'] is False
    assert summaries[0]['hasCoveragePath'] is False
    assert summaries[0]['pointCount'] == 0


def test_zone_summaries_empty_list():
    assert zone_map_list_to_summaries([]) == []


# ──────────────────────────────────────────────────────────────────────────────
# CoverageSettings DTO
# ──────────────────────────────────────────────────────────────────────────────


def test_params_to_coverage_settings_full():
    coverage_params = {
        'strip_width_m': 0.8,
        'unknown_as_obstacle': True,
        'coverage_pattern': 'zigzag',
        'boundary_ring': True,
    }
    map_params = {'inflate_radius_m': 0.55}
    dto = params_to_coverage_settings(coverage_params, map_params)
    assert dto == {
        'stripWidthM': 0.8,
        'inflateRadiusM': 0.55,
        'unknownAsObstacle': True,
        'coveragePattern': 'zigzag',
        'boundaryRing': True,
    }


def test_params_to_coverage_settings_missing_keys_yield_none():
    dto = params_to_coverage_settings({}, {})
    assert dto == {
        'stripWidthM': None,
        'inflateRadiusM': None,
        'unknownAsObstacle': None,
        'coveragePattern': None,
        'boundaryRing': None,
    }


# ──────────────────────────────────────────────────────────────────────────────
# ServiceResult envelope
# ──────────────────────────────────────────────────────────────────────────────


def test_service_response_trigger_shape():
    resp = SimpleNamespace(success=True, message='ok')
    env = service_response_to_envelope(resp)
    assert env == {'success': True, 'message': 'ok'}


def test_service_response_only_success_field():
    resp = SimpleNamespace(success=False)
    env = service_response_to_envelope(resp)
    assert env == {'success': False, 'message': ''}


def test_service_response_no_success_field_defaults_true():
    # Some custom srv types only carry data, no success/message — assume ok
    resp = SimpleNamespace(zone_map_list=[])
    env = service_response_to_envelope(resp)
    assert env == {'success': True, 'message': ''}
