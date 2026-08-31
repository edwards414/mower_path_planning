"""ROS-free safety regressions for local/global Nav2 map composition."""

import numpy as np
import pytest

from mower_mission.nav_map_fusion import (
    fuse_navigation_grid,
    union_free_space_grids,
    world_to_grid_index,
)


def test_world_to_grid_handles_rotated_origins_and_rejects_outside_points():
    assert world_to_grid_index(
        -0.5,
        1.5,
        origin_x=1.0,
        origin_y=1.0,
        origin_yaw=np.pi / 2.0,
        resolution=1.0,
        width=3,
        height=3,
    ) == (0, 1)

    with pytest.raises(ValueError, match='outside'):
        world_to_grid_index(
            -0.01,
            0.0,
            origin_x=0.0,
            origin_y=0.0,
            origin_yaw=0.0,
            resolution=1.0,
            width=3,
            height=3,
        )


@pytest.mark.parametrize('value', [np.nan, np.inf, -np.inf])
def test_world_to_grid_rejects_non_finite_channel_points(value):
    with pytest.raises(ValueError, match='finite'):
        world_to_grid_index(
            value,
            0.0,
            origin_x=0.0,
            origin_y=0.0,
            origin_yaw=0.0,
            resolution=1.0,
            width=3,
            height=3,
        )


def test_channel_free_space_is_unioned_into_the_navigation_base():
    base = np.array([[0, 100, -1], [100, 100, 0]], dtype=np.int8)
    channel = np.array([[100, 0, 100], [100, 0, 100]], dtype=np.int8)

    combined = union_free_space_grids(base, channel)

    np.testing.assert_array_equal(
        combined,
        np.array([[0, 0, 100], [100, 0, 0]], dtype=np.int8),
    )


def test_channel_union_rejects_a_geometry_shape_mismatch():
    with pytest.raises(ValueError, match='same shape'):
        union_free_space_grids(
            np.zeros((2, 2), dtype=np.int8),
            np.zeros((1, 2), dtype=np.int8),
        )


def test_fusion_blocks_base_obstacles_unknowns_and_risk():
    base = np.array(
        [
            [100, 0, 0],
            [0, -1, 0],
        ],
        dtype=np.int16,
    )
    risk = np.array(
        [
            [0, 100, 0],
            [0, 0, 0],
        ],
        dtype=np.int16,
    )

    fused = fuse_navigation_grid(
        base,
        base_resolution=0.5,
        base_origin_x=-1.0,
        base_origin_y=2.0,
        risk_grid=risk,
        risk_resolution=0.5,
        risk_origin_x=-1.0,
        risk_origin_y=2.0,
    )

    np.testing.assert_array_equal(
        fused,
        np.array(
            [
                [100, 100, 0],
                [0, 100, 0],
            ],
            dtype=np.int8,
        ),
    )


def test_mismatched_extent_is_resampled_conservatively():
    base = np.zeros((2, 4), dtype=np.int8)
    risk = np.array(
        [
            [100, 0],
            [0, 0],
        ],
        dtype=np.int8,
    )

    fused = fuse_navigation_grid(
        base,
        base_resolution=1.0,
        base_origin_x=0.0,
        base_origin_y=0.0,
        risk_grid=risk,
        risk_resolution=1.0,
        risk_origin_x=1.0,
        risk_origin_y=0.0,
    )

    # Columns outside the risk grid are blocked; its occupied cell is mapped
    # onto the base grid without changing the base map's dimensions.
    np.testing.assert_array_equal(
        fused,
        np.array(
            [
                [100, 100, 0, 100],
                [100, 0, 0, 100],
            ],
            dtype=np.int8,
        ),
    )


def test_coarser_base_cell_blocks_on_any_risk_overlap():
    base = np.zeros((1, 1), dtype=np.int8)
    risk = np.array([[0, 0], [0, 100]], dtype=np.int8)

    fused = fuse_navigation_grid(
        base,
        base_resolution=1.0,
        base_origin_x=0.0,
        base_origin_y=0.0,
        risk_grid=risk,
        risk_resolution=0.5,
        risk_origin_x=0.0,
        risk_origin_y=0.0,
    )

    assert fused.tolist() == [[100]]


def test_orientation_mismatch_is_rejected_instead_of_dropping_risk():
    with pytest.raises(ValueError, match='orientations must match'):
        fuse_navigation_grid(
            np.zeros((1, 1), dtype=np.int8),
            base_resolution=1.0,
            base_origin_x=0.0,
            base_origin_y=0.0,
            base_yaw=0.0,
            risk_grid=np.zeros((1, 1), dtype=np.int8),
            risk_resolution=1.0,
            risk_origin_x=0.0,
            risk_origin_y=0.0,
            risk_yaw=0.1,
        )
