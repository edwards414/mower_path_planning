"""ROS-free helpers for building the occupancy grid consumed by Nav2."""

import math

import numpy as np


def world_to_grid_index(
    x,
    y,
    *,
    origin_x,
    origin_y,
    origin_yaw,
    resolution,
    width,
    height,
):
    """Convert a world point into a bounded OccupancyGrid cell.

    Out-of-bounds points are rejected instead of being clamped onto the map
    edge, because clamping a malformed channel can authorize a corridor that
    was never recorded. The inverse origin rotation follows the ROS
    OccupancyGrid convention.
    """
    values = (x, y, origin_x, origin_y, origin_yaw, resolution)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError('grid coordinates must be finite')
    if resolution <= 0.0 or width <= 0 or height <= 0:
        raise ValueError('grid geometry must be positive')

    dx = float(x) - float(origin_x)
    dy = float(y) - float(origin_y)
    cos_yaw = math.cos(float(origin_yaw))
    sin_yaw = math.sin(float(origin_yaw))
    local_x = cos_yaw * dx + sin_yaw * dy
    local_y = -sin_yaw * dx + cos_yaw * dy
    column = math.floor(local_x / float(resolution))
    row = math.floor(local_y / float(resolution))
    if not (0 <= column < int(width) and 0 <= row < int(height)):
        raise ValueError('channel point is outside the navigation map')
    return int(column), int(row)


def union_free_space_grids(base_grid, channel_grid=None):
    """Return the traversable union of base free-space and a channel grid."""
    base = np.asarray(base_grid)
    if base.ndim != 2 or base.size == 0:
        raise ValueError('base_grid must be a non-empty 2-D array')
    if channel_grid is None:
        return np.where(base == 0, 0, 100).astype(np.int8)

    channel = np.asarray(channel_grid)
    if channel.shape != base.shape:
        raise ValueError('base and channel grids must have the same shape')
    return np.where((base == 0) | (channel == 0), 0, 100).astype(np.int8)


def fuse_navigation_grid(
    base_grid,
    *,
    base_resolution,
    base_origin_x,
    base_origin_y,
    base_yaw=0.0,
    risk_grid=None,
    risk_resolution=None,
    risk_origin_x=0.0,
    risk_origin_y=0.0,
    risk_yaw=0.0,
):
    """Merge free-space and inflated-risk occupancy into the base geometry.

    Only value ``0`` is traversable; occupied and unknown cells are blocked.
    When geometry differs, every base cell overlapping an occupied risk cell is
    blocked. Base cells extending outside the risk grid are blocked as well, so
    a partial image-risk map cannot authorize transit through unknown terrain.

    Current map producers create grids with a shared axis orientation. A yaw
    mismatch is rejected instead of approximated because an
    under-conservative reprojection would be unsafe for a Nav2 static layer.
    """
    base = np.asarray(base_grid)
    if base.ndim != 2 or base.size == 0:
        raise ValueError('base_grid must be a non-empty 2-D array')
    if not math.isfinite(base_resolution) or base_resolution <= 0.0:
        raise ValueError('base_resolution must be finite and > 0')

    base_blocked = base != 0
    if risk_grid is None:
        return np.where(base_blocked, 100, 0).astype(np.int8)

    risk = np.asarray(risk_grid)
    if risk.ndim != 2 or risk.size == 0:
        raise ValueError('risk_grid must be a non-empty 2-D array')
    if risk_resolution is None:
        raise ValueError('risk_resolution is required with risk_grid')
    if not math.isfinite(risk_resolution) or risk_resolution <= 0.0:
        raise ValueError('risk_resolution must be finite and > 0')

    values = (
        base_origin_x,
        base_origin_y,
        base_yaw,
        risk_origin_x,
        risk_origin_y,
        risk_yaw,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('grid origins and yaws must be finite')

    yaw_delta = math.atan2(
        math.sin(risk_yaw - base_yaw),
        math.cos(risk_yaw - base_yaw),
    )
    if abs(yaw_delta) > 1e-9:
        raise ValueError('base and risk grid orientations must match')

    # Express the risk origin in the base grid's local coordinates. The grids
    # are axis-aligned there even when their shared world yaw is non-zero.
    delta_x = risk_origin_x - base_origin_x
    delta_y = risk_origin_y - base_origin_y
    cos_yaw = math.cos(base_yaw)
    sin_yaw = math.sin(base_yaw)
    risk_local_x = cos_yaw * delta_x + sin_yaw * delta_y
    risk_local_y = -sin_yaw * delta_x + cos_yaw * delta_y

    base_height, base_width = base.shape
    risk_height, risk_width = risk.shape
    base_x0 = np.arange(base_width, dtype=np.float64) * base_resolution
    base_x1 = base_x0 + base_resolution
    base_y0 = np.arange(base_height, dtype=np.float64) * base_resolution
    base_y1 = base_y0 + base_resolution

    risk_x1 = risk_local_x + risk_width * risk_resolution
    risk_y1 = risk_local_y + risk_height * risk_resolution
    tolerance = 1e-9 * max(
        1.0,
        abs(base_resolution),
        abs(risk_resolution),
        abs(risk_local_x),
        abs(risk_local_y),
        abs(risk_x1),
        abs(risk_y1),
    )

    outside_x = (base_x0 < risk_local_x - tolerance) | (
        base_x1 > risk_x1 + tolerance
    )
    outside_y = (base_y0 < risk_local_y - tolerance) | (
        base_y1 > risk_y1 + tolerance
    )
    outside_risk = outside_y[:, None] | outside_x[None, :]
    index_tolerance = tolerance / risk_resolution

    # Query every risk cell overlapped by each base cell. An integral image
    # keeps this conservative operation vectorized when resolutions differ.
    col0 = np.floor(
        (base_x0 - risk_local_x) / risk_resolution + index_tolerance
    ).astype(np.int64)
    col1 = np.ceil(
        (base_x1 - risk_local_x) / risk_resolution - index_tolerance
    ).astype(np.int64)
    row0 = np.floor(
        (base_y0 - risk_local_y) / risk_resolution + index_tolerance
    ).astype(np.int64)
    row1 = np.ceil(
        (base_y1 - risk_local_y) / risk_resolution - index_tolerance
    ).astype(np.int64)

    col0 = np.clip(col0, 0, risk_width)
    col1 = np.clip(col1, 0, risk_width)
    row0 = np.clip(row0, 0, risk_height)
    row1 = np.clip(row1, 0, risk_height)

    risk_blocked = (risk != 0).astype(np.int64)
    integral = np.pad(risk_blocked, ((1, 0), (1, 0))).cumsum(0).cumsum(1)
    overlap_count = (
        integral[row1[:, None], col1[None, :]]
        - integral[row0[:, None], col1[None, :]]
        - integral[row1[:, None], col0[None, :]]
        + integral[row0[:, None], col0[None, :]]
    )
    blocked = base_blocked | outside_risk | (overlap_count > 0)
    return np.where(blocked, 100, 0).astype(np.int8)
