"""Pure occupancy-grid safety transforms shared by map management."""

import math

import cv2
import numpy as np


def erode_free_space_grid(
    grid,
    *,
    resolution_m: float,
    inflate_radius_m: float,
) -> np.ndarray:
    """Return 0 only where free space has the requested obstacle clearance.

    Cells outside the finite OccupancyGrid are obstacles. OpenCV's erosion
    default does not enforce that boundary for an all-free image, so the zero
    border is explicit here.
    """
    data = np.asarray(grid, dtype=np.int16)
    if data.ndim != 2 or data.size == 0:
        raise ValueError('free-space grid must be a non-empty 2D array')
    if not math.isfinite(resolution_m) or resolution_m <= 0.0:
        raise ValueError('resolution_m must be finite and positive')
    if not math.isfinite(inflate_radius_m) or inflate_radius_m < 0.0:
        raise ValueError('inflate_radius_m must be finite and non-negative')

    radius_cells = int(math.ceil(inflate_radius_m / resolution_m))
    free_mask = (data == 0).astype(np.uint8)
    if radius_cells > 0:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (2 * radius_cells + 1, 2 * radius_cells + 1),
        )
        free_mask = cv2.erode(
            free_mask,
            kernel,
            borderType=cv2.BORDER_CONSTANT,
            borderValue=0,
        )
    return np.where(free_mask == 1, 0, 100).astype(np.int8)
