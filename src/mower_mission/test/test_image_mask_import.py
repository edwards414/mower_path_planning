"""Tests for app image-mask import helpers."""

import base64
from types import SimpleNamespace

import numpy as np
import pytest

from mower_mission.image_mask_import import (
    decode_u8_mask,
    rasterize_image_masks,
    yaw_from_quaternion,
)


def test_decode_u8_mask_roundtrips_base64_row_major():
    raw = bytes([255, 0, 0, 255])
    decoded = decode_u8_mask(
        base64.b64encode(raw).decode('ascii'),
        2,
        2,
        field_name='free_mask_data',
    )

    assert decoded.dtype == np.uint8
    assert decoded.tolist() == [[255, 0], [0, 255]]


def test_decode_u8_mask_rejects_wrong_size():
    encoded = base64.b64encode(bytes([255, 0])).decode('ascii')

    with pytest.raises(ValueError, match='width\\*height'):
        decode_u8_mask(
            encoded,
            2,
            2,
            field_name='free_mask_data',
        )


def test_rasterize_image_masks_flips_image_y_to_map_y():
    free_mask = np.array([[255, 0], [0, 255]], dtype=np.uint8)
    risk_mask = np.array([[0, 255], [0, 0]], dtype=np.uint8)

    free_grid, risk_grid, origin_x, origin_y = rasterize_image_masks(
        free_mask=free_mask,
        risk_mask=risk_mask,
        resolution=0.5,
        robot_x=0.0,
        robot_y=0.0,
        robot_yaw=0.0,
        start_x=0.0,
        start_y=0.0,
        image_heading=0.0,
    )

    assert origin_x == pytest.approx(0.0)
    assert origin_y == pytest.approx(0.0)
    assert free_grid.tolist() == [[100, 0], [0, 100]]
    assert risk_grid.tolist() == [[0, 0], [0, 100]]


def test_yaw_from_quaternion_reads_map_heading():
    q = SimpleNamespace(x=0.0, y=0.0, z=2 ** 0.5 / 2, w=2 ** 0.5 / 2)

    assert yaw_from_quaternion(q) == pytest.approx(np.pi / 2)
