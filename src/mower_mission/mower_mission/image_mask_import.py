"""Pure helpers for importing app-generated image masks."""

from __future__ import annotations

import base64
import binascii
import math

import numpy as np


def decode_u8_mask(
    encoded: str,
    width: int,
    height: int,
    *,
    field_name: str,
    optional: bool = False,
) -> np.ndarray:
    """Decode a base64 row-major uint8 mask into a HxW array."""
    if not encoded:
        if optional:
            return np.zeros((height, width), dtype=np.uint8)
        raise ValueError(f'{field_name} is required')

    try:
        raw = base64.b64decode(encoded, validate=True)
    except (binascii.Error, ValueError) as exc:
        raise ValueError(f'{field_name} must be valid base64') from exc

    expected = width * height
    if len(raw) != expected:
        raise ValueError(
            f'{field_name} length {len(raw)} does not match '
            f'width*height {expected}'
        )
    return np.frombuffer(raw, dtype=np.uint8).reshape(height, width)


def yaw_from_quaternion(q) -> float:
    """Extract planar yaw from a quaternion-shaped object."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def rasterize_image_masks(
    *,
    free_mask: np.ndarray,
    risk_mask: np.ndarray,
    resolution: float,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    start_x: float,
    start_y: float,
    image_heading: float,
) -> tuple[np.ndarray, np.ndarray, float, float]:
    """Transform image-local masks into axis-aligned map-frame occupancy grids."""
    height, width = free_mask.shape
    theta = robot_yaw - image_heading
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)

    def local_to_map(x: float, y: float) -> tuple[float, float]:
        dx = x - start_x
        dy = y - start_y
        return (
            robot_x + cos_t * dx - sin_t * dy,
            robot_y + sin_t * dx + cos_t * dy,
        )

    world_w = width * resolution
    world_h = height * resolution
    corners = [
        local_to_map(0.0, 0.0),
        local_to_map(world_w, 0.0),
        local_to_map(0.0, world_h),
        local_to_map(world_w, world_h),
    ]
    min_x = min(p[0] for p in corners)
    max_x = max(p[0] for p in corners)
    min_y = min(p[1] for p in corners)
    max_y = max(p[1] for p in corners)
    out_w = max(1, int(math.ceil((max_x - min_x) / resolution)))
    out_h = max(1, int(math.ceil((max_y - min_y) / resolution)))

    free_grid = np.full((out_h, out_w), 100, dtype=np.int8)
    risk_grid = np.zeros((out_h, out_w), dtype=np.int8)

    # Inverse transform: map → image-local metres → image row/col.
    for out_r in range(out_h):
        map_y = min_y + (out_r + 0.5) * resolution
        for out_c in range(out_w):
            map_x = min_x + (out_c + 0.5) * resolution
            dx = map_x - robot_x
            dy = map_y - robot_y
            local_x = start_x + cos_t * dx + sin_t * dy
            local_y = start_y - sin_t * dx + cos_t * dy
            src_c = int(math.floor(local_x / resolution))
            src_r = int(math.floor((world_h - local_y) / resolution))
            if src_c < 0 or src_c >= width or src_r < 0 or src_r >= height:
                continue
            if free_mask[src_r, src_c] == 255:
                free_grid[out_r, out_c] = 0
            if risk_mask[src_r, src_c] == 255:
                risk_grid[out_r, out_c] = 100

    return free_grid, risk_grid, min_x, min_y

