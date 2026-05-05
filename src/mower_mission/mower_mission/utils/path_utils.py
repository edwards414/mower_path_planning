"""Path utility functions for coverage planning."""

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

from geometry_msgs.msg import Pose, PoseStamped

from nav_msgs.msg import OccupancyGrid, Path

from std_msgs.msg import Header


def _euler_to_quaternion(roll, pitch, yaw):
    """將歐拉角轉換為四元數（x, y, z, w）."""
    qx = (math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2)
          - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2))
    qy = (math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
          + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2))
    qz = (math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
          - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2))
    qw = (math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2)
          + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2))
    return (qx, qy, qz, qw)


def _cal_two_point_orientation(x1, y1, x2, y2):
    """計算兩點之間的朝向（歐拉角yaw），並返回對應的四元數."""
    dx = x2 - x1
    dy = y2 - y1
    yaw = math.atan2(dy, dx)
    return _euler_to_quaternion(0, 0, yaw)


def _transform_coverage_path_points(points: list, map_header: Header) -> Path:
    """將路徑點轉換為Path消息."""
    coverage_path = Path()
    coverage_path.header = map_header
    coverage_path.header.frame_id = 'map'

    for idx, (x, y) in enumerate(points):
        goal_pose = PoseStamped()
        goal_pose.header = coverage_path.header
        goal_pose.header.stamp = map_header.stamp
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.position.z = 0.0

        if idx < len(points) - 1:
            x2, y2 = points[idx + 1]
            qx, qy, qz, qw = _cal_two_point_orientation(x, y, x2, y2)
        elif idx > 0:
            x2, y2 = points[idx - 1]
            qx, qy, qz, qw = _cal_two_point_orientation(x2, y2, x, y)
        else:
            qx, qy, qz, qw = _euler_to_quaternion(0, 0, 0)

        goal_pose.pose.orientation.x = qx
        goal_pose.pose.orientation.y = qy
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw

        coverage_path.poses.append(goal_pose)
    return coverage_path


def _transform_coverage_split_points(
    points: list[tuple[float, float]], map_header: Header
) -> list[Pose]:
    """將分割點轉換為PoseStamped消息."""
    coverage_split_points = []
    for x, y in points:
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        coverage_split_points.append(pose)
    return coverage_split_points


def _validate_maps_compatibility(
    self, map1: OccupancyGrid, map2: OccupancyGrid
) -> bool:
    """驗證兩個地圖是否兼容."""
    info1, info2 = map1.info, map2.info

    if abs(info1.resolution - info2.resolution) > 1e-6:
        return False

    if info1.width != info2.width or info1.height != info2.height:
        return False

    if (
        abs(info1.origin.position.x - info2.origin.position.x) > 1e-6
        or abs(info1.origin.position.y - info2.origin.position.y) > 1e-6
    ):
        return False

    return True


class val:
    """Value holder for pose data."""

    def __init__(self):
        """Initialize all pose values to None."""
        self.x = None
        self.y = None
        self.qx = None
        self.qy = None
        self.qz = None
        self.qw = None

    def is_empty(self):
        """Check if all values are None."""
        return (
            self.x is None
            and self.y is None
            and self.qx is None
            and self.qy is None
            and self.qz is None
            and self.qw is None
        )
