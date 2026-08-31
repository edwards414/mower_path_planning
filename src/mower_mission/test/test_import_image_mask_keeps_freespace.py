"""Regression tests for import_image_mask_srv freespace handling.

An image mission must NOT replace the robot-collected freespace: when collected
freespace exists, the import keeps base_map = collected and makes the coverage
zone = image ∩ collected (image is only a range limiter). With no collected
freespace, the import falls back to the image-only behavior.
"""

import base64

import numpy as np

import pytest

import rclpy

from geometry_msgs.msg import Pose
from nav_msgs.msg import MapMetaData, OccupancyGrid
from mower_interface.srv import ImportImageMask
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray

from mower_mission.map_manage_node import MapManage


@pytest.fixture
def map_manage():
    """Create a MapManage node with rclpy initialised."""
    if not rclpy.ok():
        rclpy.init()
    node = MapManage()
    # Unit-test the map mutation itself; central lease admission is covered by
    # test_nav_action_server_services.
    node._acquire_mission_mutation = lambda response, operation: True
    node._release_mission_mutation = lambda: True
    try:
        yield node
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _free_grid(free_box, res, ox, oy, n):
    """n×n OccupancyGrid @res, origin (ox, oy); free (0) inside `free_box`."""
    data = np.full((n, n), 100, dtype=np.int8)
    fx0, fx1, fy0, fy1 = free_box
    for r in range(n):
        wy = oy + (r + 0.5) * res
        for c in range(n):
            wx = ox + (c + 0.5) * res
            if fx0 <= wx < fx1 and fy0 <= wy < fy1:
                data[r, c] = 0
    g = OccupancyGrid()
    g.header.frame_id = 'map'
    g.info = MapMetaData()
    g.info.resolution = res
    g.info.width = n
    g.info.height = n
    g.info.origin.position.x = ox
    g.info.origin.position.y = oy
    g.info.origin.orientation.w = 1.0
    g.data = data.flatten().tolist()
    return g


def _import_request(robot_x, robot_y, side_px=80, res=0.1):
    """All-free `side_px`²  image placed with its start pixel at (robot_x, robot_y)."""
    req = ImportImageMask.Request()
    req.mask_encoding = 'base64_u8_row_major'
    req.width = side_px
    req.height = side_px
    req.resolution_m = res
    req.zone_id = 9001
    req.free_mask_data = base64.b64encode(
        np.full((side_px, side_px), 255, dtype=np.uint8).tobytes()).decode()
    req.risk_mask_data = ''
    req.robot_pose_header.frame_id = 'map'
    pose = Pose()
    pose.position.x = float(robot_x)
    pose.position.y = float(robot_y)
    pose.orientation.w = 1.0
    req.robot_pose_map = pose
    req.start_x_m = 0.0
    req.start_y_m = 0.0
    req.image_heading_rad = 0.0
    return req


def _free_cells(grid):
    return int(np.count_nonzero(np.asarray(grid.data, dtype=np.int16) == 0))


def test_import_keeps_collected_freespace_and_clips_zone(map_manage):
    # Collected freespace: free over world [0,4]×[0,4] (16 m²).
    collected = _free_grid((0.0, 4.0, 0.0, 4.0), 0.05, -0.5, -0.5, 100)
    map_manage.collected_free_space = collected
    map_manage.base_map = collected
    map_manage.map_msg = collected
    collected_free = _free_cells(collected)

    # 8 m all-free image at (2,2) → covers ~[2,10]² → overlaps collected on [2,4]² (4 m²).
    res = ImportImageMask.Response()
    map_manage.import_image_mask_srv(_import_request(2.0, 2.0), res)

    assert res.success, res.message
    # Freespace KEPT (not replaced by the image).
    assert map_manage.base_map is map_manage.collected_free_space
    assert _free_cells(map_manage.base_map) == collected_free
    assert _free_cells(map_manage.collected_free_space) == collected_free
    # Coverage zone = image ∩ collected ≈ 4 m² (NOT the full ~64 m² image).
    zone_area = _free_cells(map_manage.zone_map_list[0].mask_map) * 0.1 * 0.1
    assert 3.0 <= zone_area <= 5.0
    expected_inflated = map_manage._create_free_space_inflated(
        map_manage.zone_map_list[0].mask_map
    )
    assert map_manage.zone_map_list[0].mask_map_inflated.data == (
        expected_inflated.data
    )
    assert _free_cells(expected_inflated) < _free_cells(
        map_manage.zone_map_list[0].mask_map
    )


def test_import_without_collected_is_image_only(map_manage):
    # No collected freespace -> fallback: the image becomes the base map.
    assert map_manage.collected_free_space is None

    res = ImportImageMask.Response()
    map_manage.import_image_mask_srv(_import_request(0.0, 0.0), res)

    assert res.success, res.message
    # base_map is the image (full ~64 m²), not a kept collected grid.
    assert map_manage.base_map is map_manage.zone_map_list[0].mask_map \
        or _free_cells(map_manage.base_map) == _free_cells(map_manage.zone_map_list[0].mask_map)
    image_area = _free_cells(map_manage.zone_map_list[0].mask_map) * 0.1 * 0.1
    assert image_area > 50.0
    assert _free_cells(
        map_manage.zone_map_list[0].mask_map_inflated
    ) < _free_cells(map_manage.zone_map_list[0].mask_map)
    # Local uses raw geometry for footprint collision checks; global is an
    # eroded configuration-space map for NavFn.
    assert _free_cells(map_manage.map_msg) > _free_cells(
        map_manage.global_map_msg
    )


def test_import_rejects_image_narrower_than_safety_clearance(map_manage):
    res = ImportImageMask.Response()
    map_manage.import_image_mask_srv(
        _import_request(0.0, 0.0, side_px=10, res=0.1), res
    )

    assert not res.success
    assert '安全內縮後沒有可用割草區域' in res.message
    assert map_manage.zone_map_list == []


def test_image_only_restore_fails_without_mutating_active_image(map_manage):
    imported = ImportImageMask.Response()
    map_manage.import_image_mask_srv(_import_request(0.0, 0.0), imported)
    assert imported.success, imported.message
    active_base = map_manage.base_map
    active_zones = map_manage.zone_map_list
    active_risk = map_manage.risk_map
    backup_zones = map_manage._free_zone_backup
    active_local_map = map_manage.map_msg
    active_global_map = map_manage.global_map_msg

    restored = Trigger.Response()
    map_manage.restore_free_space_srv(Trigger.Request(), restored)

    assert not restored.success
    assert '沒有採集的自由空間' in restored.message
    assert map_manage.base_map is active_base
    assert map_manage.zone_map_list is active_zones
    assert map_manage.risk_map is active_risk
    assert map_manage._free_zone_backup is backup_zones
    assert map_manage.map_msg is active_local_map
    assert map_manage.global_map_msg is active_global_map


def test_failed_freespace_candidate_preserves_previous_state(map_manage):
    collected = _free_grid((0.0, 1.0, 0.0, 1.0), 0.05, 0.0, 0.0, 20)
    previous_zones = map_manage.zone_map_list
    map_manage.base_map = collected
    map_manage.collected_free_space = collected

    assert map_manage._create_zone_maps_and_freespace(MarkerArray()) is None
    assert map_manage.base_map is collected
    assert map_manage.collected_free_space is collected
    assert map_manage.zone_map_list is previous_zones
