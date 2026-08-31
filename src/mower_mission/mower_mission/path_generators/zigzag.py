#!/usr/bin/env python3

# Copyright 2024 fxrbindi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Zigzag back-and-forth coverage path generator."""

from __future__ import annotations

import math

import numpy as np

from ..coverage.path_validator import SafeMap, validate_path


MIN_U_TURN_RADIUS_M = 0.35


def _generate_coverage_zigzag_path(
    safe_map: np.ndarray,
    strip_width_m: float,
    waypoint_spacing_m: float,
    res: float,
    H: int,
    W: int,
    origin_x: float,
    origin_y: float,
    angle_deg: float = 0.0,
) -> tuple[
    list[tuple[float, float]],
    list[tuple[float, float]],
    list[tuple[int, int]],
]:
    """Generate a back-and-forth zigzag coverage path.

    This is the single implementation for the old "boustrophedon" naming and
    the current UI-facing "zigzag" naming.
    """
    safe_map = safe_map.astype(bool, copy=False)
    if abs(angle_deg) >= 1e-6:
        return _generate_rotated_zigzag_path(
            safe_map,
            strip_width_m,
            waypoint_spacing_m,
            res,
            H,
            W,
            origin_x,
            origin_y,
            angle_deg,
        )

    spacing = waypoint_spacing_m
    strip_cols = max(1, int(round(strip_width_m / res)))
    midcols = list(range(strip_cols // 2, W, strip_cols))
    runs: list[list[tuple[float, float]]] = []
    reverse = False

    for mc in midcols:
        segments = []
        start = None
        for i in range(H):
            ok = bool(safe_map[i, mc])
            is_last = i == H - 1
            if ok and start is None:
                start = i
            if (not ok or is_last) and start is not None:
                end = i - 1 if (not ok) else i
                segments.append((start, end))
                start = None

        segs = segments[::-1] if reverse else segments
        for s, e in segs:
            y0 = origin_y + (s + 0.5) * res
            y1 = origin_y + (e + 0.5) * res
            x = origin_x + (mc + 0.5) * res
            if y1 >= y0:
                ys = list(np.arange(y0, y1, max(res, spacing))) + [y1]
            else:
                ys = list(np.arange(y0, y1, -max(res, spacing))) + [y1]
            ys = ys[::-1] if reverse else ys
            runs.append([(x, y) for y in ys])
        reverse = not reverse

    safe_map_struct = SafeMap(
        grid=safe_map.astype(bool),
        resolution=res,
        origin_x=origin_x,
        origin_y=origin_y,
    )
    points, coverage_split_points = _build_path_with_u_turns(
        runs,
        safe_map_struct,
        waypoint_spacing_m=max(res, spacing),
        res=res,
    )
    invalid_segments = _find_invalid_segments(
        points, safe_map, res, origin_x, origin_y
    )
    return points, coverage_split_points, invalid_segments


def _generate_rotated_zigzag_path(
    safe_map: np.ndarray,
    strip_width_m: float,
    waypoint_spacing_m: float,
    res: float,
    H: int,
    W: int,
    origin_x: float,
    origin_y: float,
    angle_deg: float,
) -> tuple[
    list[tuple[float, float]],
    list[tuple[float, float]],
    list[tuple[int, int]],
]:
    """Generate zigzag lanes in a rotated planning frame.

    The implementation intentionally mirrors mower_coverage_core's Rust
    backend so both backends expose the same public planning contract.
    """
    angle_rad = math.radians(angle_deg)
    center_x = origin_x + W * res / 2.0
    center_y = origin_y + H * res / 2.0
    cos_neg = math.cos(-angle_rad)
    sin_neg = math.sin(-angle_rad)

    rotated_safe_points: list[tuple[float, float]] = []
    for row in range(H):
        for col in range(W):
            if not safe_map[row, col]:
                continue
            world_x = origin_x + (col + 0.5) * res
            world_y = origin_y + (row + 0.5) * res
            dx = world_x - center_x
            dy = world_y - center_y
            rotated_safe_points.append((
                cos_neg * dx - sin_neg * dy + center_x,
                sin_neg * dx + cos_neg * dy + center_y,
            ))

    if not rotated_safe_points:
        return [], [], []

    min_x = min(point[0] for point in rotated_safe_points)
    max_x = max(point[0] for point in rotated_safe_points)
    width_rotated = max_x - min_x
    strip_count = max(1, int(math.floor(width_rotated / strip_width_m)))
    if strip_count == 1:
        strip_centers = [(min_x + max_x) / 2.0]
    else:
        strip_centers = [
            min_x
            + strip_width_m / 2.0
            + index * (width_rotated - strip_width_m) / (strip_count - 1)
            for index in range(strip_count)
        ]

    rotated_points: list[tuple[float, float]] = []
    rotated_split_points: list[tuple[float, float]] = []
    reverse = False
    min_distance = max(waypoint_spacing_m, res) * 0.5

    for strip_x in strip_centers:
        candidates = [
            point for point in rotated_safe_points
            if abs(point[0] - strip_x) <= strip_width_m / 2.0
        ]
        candidates.sort(key=lambda point: (point[1], point[0]))
        if reverse:
            candidates.reverse()
        if not candidates:
            reverse = not reverse
            continue

        previous = None
        strip_last = None
        for point in candidates:
            if previous is not None and math.hypot(
                point[0] - previous[0], point[1] - previous[1]
            ) < min_distance:
                continue
            rotated_points.append(point)
            previous = point
            strip_last = point
        if strip_last is not None:
            rotated_split_points.append(strip_last)
        reverse = not reverse

    cos_pos = math.cos(angle_rad)
    sin_pos = math.sin(angle_rad)

    def rotate_back(point: tuple[float, float]) -> tuple[float, float]:
        dx = point[0] - center_x
        dy = point[1] - center_y
        return (
            cos_pos * dx - sin_pos * dy + center_x,
            sin_pos * dx + cos_pos * dy + center_y,
        )

    points = [rotate_back(point) for point in rotated_points]
    split_points = [rotate_back(point) for point in rotated_split_points]
    invalid_segments = _find_invalid_segments(
        points, safe_map, res, origin_x, origin_y
    )
    return points, split_points, invalid_segments


def _build_path_with_u_turns(
    runs: list[list[tuple[float, float]]],
    safe_map: SafeMap,
    waypoint_spacing_m: float,
    res: float,
) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    """Join adjacent zigzag runs with safe U-turns when there is room."""
    if not runs:
        return [], []

    points: list[tuple[float, float]] = []
    split_points: list[tuple[float, float]] = []
    current_run = list(runs[0])

    for next_run_raw in runs[1:]:
        next_run = list(next_run_raw)
        u_turn = _make_u_turn(
            current_run,
            next_run,
            safe_map,
            waypoint_spacing_m,
            res,
        )
        if u_turn is None:
            _extend_unique(points, current_run)
            if current_run:
                split_points.append(current_run[-1])
            current_run = next_run
            continue

        current_prefix, turn_points, next_suffix = u_turn
        _extend_unique(points, current_prefix)
        _extend_unique(points, turn_points)
        if turn_points:
            split_points.append(turn_points[-1])
        current_run = next_suffix

    _extend_unique(points, current_run)
    if current_run:
        split_points.append(current_run[-1])

    return points, split_points


def _make_u_turn(
    current_run: list[tuple[float, float]],
    next_run: list[tuple[float, float]],
    safe_map: SafeMap,
    waypoint_spacing_m: float,
    res: float,
) -> tuple[
    list[tuple[float, float]],
    list[tuple[float, float]],
    list[tuple[float, float]],
] | None:
    """Return adjusted runs and a rounded U-turn, or None if unsafe."""
    if len(current_run) < 2 or len(next_run) < 2:
        return None

    incoming_dy = current_run[-1][1] - current_run[0][1]
    outgoing_dy = next_run[-1][1] - next_run[0][1]
    if abs(incoming_dy) < res or abs(outgoing_dy) < res:
        return None
    if incoming_dy * outgoing_dy >= 0:
        return None

    start = current_run[-1]
    end = next_run[0]
    lane_gap = abs(end[0] - start[0])
    if lane_gap < res * 0.5:
        return None
    if abs(start[1] - end[1]) > max(waypoint_spacing_m, res) * 1.5:
        return None

    radius = lane_gap / 2.0
    if radius < MIN_U_TURN_RADIUS_M:
        return None

    boundary_y = (start[1] + end[1]) / 2.0
    top_turn = incoming_dy > 0.0
    center_y = boundary_y - radius if top_turn else boundary_y + radius

    current_prefix = _trim_run_to_turn_y(
        current_run,
        center_y,
        keep_below=top_turn,
    )
    next_suffix = _trim_run_from_turn_y(
        next_run,
        center_y,
        skip_above=top_turn,
    )
    turn_start = (start[0], center_y)
    turn_end = (end[0], center_y)
    current_prefix = _with_endpoint(current_prefix, turn_start)
    next_suffix = _with_startpoint(next_suffix, turn_end)

    samples = max(
        3,
        int(math.ceil((math.pi * radius) / max(waypoint_spacing_m, res))) + 1,
    )
    turn_points = []
    for idx in range(samples):
        t = idx / (samples - 1)
        x = start[0] + (end[0] - start[0]) * t
        arc_offset = radius * math.sin(math.pi * t)
        y = center_y + arc_offset if top_turn else center_y - arc_offset
        turn_points.append((x, y))

    validation_points = []
    if current_prefix:
        validation_points.append(current_prefix[-1])
    validation_points.extend(turn_points)
    if len(next_suffix) > 1:
        validation_points.append(next_suffix[1])

    if not validate_path(validation_points, safe_map).valid:
        return None

    return current_prefix, turn_points, next_suffix


def _trim_run_to_turn_y(
    run: list[tuple[float, float]],
    turn_y: float,
    keep_below: bool,
) -> list[tuple[float, float]]:
    if keep_below:
        return [pt for pt in run if pt[1] <= turn_y]
    return [pt for pt in run if pt[1] >= turn_y]


def _trim_run_from_turn_y(
    run: list[tuple[float, float]],
    turn_y: float,
    skip_above: bool,
) -> list[tuple[float, float]]:
    if skip_above:
        return [pt for pt in run if pt[1] <= turn_y]
    return [pt for pt in run if pt[1] >= turn_y]


def _with_endpoint(
    points: list[tuple[float, float]],
    endpoint: tuple[float, float],
) -> list[tuple[float, float]]:
    adjusted = list(points)
    _append_unique(adjusted, endpoint)
    return adjusted


def _with_startpoint(
    points: list[tuple[float, float]],
    startpoint: tuple[float, float],
) -> list[tuple[float, float]]:
    adjusted = list(points)
    if adjusted and _same_point(adjusted[0], startpoint):
        return adjusted
    return [startpoint] + adjusted


def _extend_unique(
    points: list[tuple[float, float]],
    new_points: list[tuple[float, float]],
):
    for pt in new_points:
        _append_unique(points, pt)


def _append_unique(
    points: list[tuple[float, float]],
    point: tuple[float, float],
):
    if points and _same_point(points[-1], point):
        return
    points.append(point)


def _same_point(
    p0: tuple[float, float],
    p1: tuple[float, float],
    tol: float = 1e-9,
) -> bool:
    return abs(p0[0] - p1[0]) <= tol and abs(p0[1] - p1[1]) <= tol


def _find_invalid_segments(
    points: list[tuple[float, float]],
    safe_map: np.ndarray,
    res: float,
    origin_x: float,
    origin_y: float,
) -> list[tuple[int, int]]:
    """Return consecutive point pairs whose direct line is unsafe."""
    if len(points) < 2:
        return []
    safe_map_struct = SafeMap(
        grid=safe_map.astype(bool),
        resolution=res,
        origin_x=origin_x,
        origin_y=origin_y,
    )
    result = validate_path(points, safe_map_struct)
    return result.invalid_segments
