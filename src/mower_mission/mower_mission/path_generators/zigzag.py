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
    angle_rad = np.deg2rad(angle_deg)
    coverage_split_points = []
    if abs(angle_rad) < 1e-6:
        strip_cols = max(1, int(round(strip_width_m / res)))
        midcols = list(range(strip_cols // 2, W, strip_cols))

        spacing = waypoint_spacing_m
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

    yy, xx = np.indices((H, W))
    xs = origin_x + (xx + 0.5) * res
    ys = origin_y + (yy + 0.5) * res
    coords = np.stack([xs, ys], axis=-1)
    center_x = origin_x + W * res / 2
    center_y = origin_y + H * res / 2
    c = np.cos(-angle_rad)
    s = np.sin(-angle_rad)
    rotM = np.array([[c, -s], [s, c]])
    coords_rot = coords - np.array([center_x, center_y])
    coords_rot = coords_rot @ rotM.T
    coords_rot = coords_rot + np.array([center_x, center_y])
    all_rot_points = coords_rot[safe_map.astype(bool)]
    minx, maxx = np.min(all_rot_points[:, 0]), np.max(all_rot_points[:, 0])
    width_rot = maxx - minx
    n_strips = max(1, int(np.floor(width_rot / strip_width_m)))
    strip_centers_x = np.linspace(
        minx + strip_width_m / 2,
        maxx - strip_width_m / 2,
        n_strips,
    )
    points = []
    reverse = False
    for scx in strip_centers_x:
        dist_to_strip = np.abs(all_rot_points[:, 0] - scx)
        mask = dist_to_strip <= strip_width_m / 2
        candidates = all_rot_points[mask]
        if len(candidates) == 0:
            reverse = not reverse
            continue
        sort_order = np.argsort(candidates[:, 1])
        if reverse:
            sort_order = sort_order[::-1]
        ordered = candidates[sort_order]
        prev_pt = None
        strip_last_point = None
        for pt in ordered:
            if prev_pt is not None:
                dist = np.linalg.norm(pt - prev_pt)
                if dist < max(res, waypoint_spacing_m) * 0.5:
                    continue
            prev_pt = pt
            strip_last_point = pt
            points.append(tuple(pt))
        if strip_last_point is not None:
            coverage_split_points.append(tuple(strip_last_point))
        reverse = not reverse
    if angle_deg != 0.0:
        c_inv = np.cos(angle_rad)
        s_inv = np.sin(angle_rad)
        rotMinv = np.array([[c_inv, -s_inv], [s_inv, c_inv]])
        points_np = np.array(points) - np.array([center_x, center_y])
        points_np = points_np @ rotMinv.T
        points_np = points_np + np.array([center_x, center_y])
        points = [tuple(pt) for pt in points_np]
        if len(coverage_split_points) > 0:
            split_points_np = np.array(coverage_split_points) - np.array(
                [center_x, center_y]
            )
            split_points_np = split_points_np @ rotMinv.T
            split_points_np = split_points_np + np.array([center_x, center_y])
            coverage_split_points = [tuple(pt) for pt in split_points_np]

    invalid_segments = _find_invalid_segments(
        points, safe_map, res, origin_x, origin_y
    )
    return points, coverage_split_points, invalid_segments


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
