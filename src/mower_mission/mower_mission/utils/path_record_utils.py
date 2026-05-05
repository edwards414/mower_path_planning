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

import math

from geometry_msgs.msg import Point

from visualization_msgs.msg import Marker


def simplify_path(poses, distance_threshold):
    """使用Douglas-Peucker算法简化路径."""
    if len(poses) < 3:
        return poses

    points = [(p.pose.position.x, p.pose.position.y) for p in poses]
    simplified_indices = douglas_peucker(points, distance_threshold)
    return [poses[i] for i in simplified_indices]


def douglas_peucker(points, epsilon):
    """Douglas-Peucker算法实现."""
    if len(points) < 3:
        return list(range(len(points)))

    start = points[0]
    end = points[-1]
    max_dist = 0
    max_index = 0

    for i in range(1, len(points) - 1):
        dist = point_to_line_distance(points[i], start, end)
        if dist > max_dist:
            max_dist = dist
            max_index = i

    if max_dist > epsilon:
        left_indices = douglas_peucker(points[:max_index + 1], epsilon)
        right_indices = douglas_peucker(points[max_index:], epsilon)

        result = left_indices + [max_index + i for i in right_indices[1:]]
        return result
    else:
        return [0, len(points) - 1]


def point_to_line_distance(point, line_start, line_end):
    """计算点到直线的距离."""
    x0, y0 = point
    x1, y1 = line_start
    x2, y2 = line_end

    line_length_sq = (x2 - x1) ** 2 + (y2 - y1) ** 2
    if line_length_sq == 0:
        return math.sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2)

    numerator = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
    return numerator / math.sqrt(line_length_sq)


def path_to_marker(path, ns='chennal_path', marker_id=0,
                   color=(0.0, 1.0, 0.0), scale=0.1):
    """將 Path 轉換為 visualization_msgs/Marker."""
    marker = Marker()
    marker.header = path.header
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = scale
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 0.8

    marker.points = []
    for pose in path.poses:
        p = Point()
        p.x = pose.pose.position.x
        p.y = pose.pose.position.y
        p.z = pose.pose.position.z if hasattr(pose.pose.position, 'z') else 0.0
        marker.points.append(p)

    return marker
