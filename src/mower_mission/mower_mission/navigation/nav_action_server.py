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
import math
from collections import deque
from time import monotonic

from geometry_msgs.msg import Point, Pose

from mower_interface.action import Waypoint
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Path

from rcl_interfaces.msg import Log
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger

from visualization_msgs.msg import Marker


def _path_frame_id(path: Path) -> str:
    return path.header.frame_id or 'map'


def _new_path_like(path: Path) -> Path:
    split_path = Path()
    split_path.header.frame_id = _path_frame_id(path)
    split_path.header.stamp = path.header.stamp
    return split_path


def _path_distance(path: Path) -> float:
    """Return cumulative 2D path distance in meters."""
    distance = 0.0
    for prev, curr in zip(path.poses, path.poses[1:]):
        distance += math.hypot(
            curr.pose.position.x - prev.pose.position.x,
            curr.pose.position.y - prev.pose.position.y,
        )
    return distance


def _angle_diff(a: float, b: float) -> float:
    """Return the smallest absolute angle difference in radians."""
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def _segment_heading(prev_pose, next_pose) -> float:
    dx = next_pose.pose.position.x - prev_pose.pose.position.x
    dy = next_pose.pose.position.y - prev_pose.pose.position.y
    return math.atan2(dy, dx)


def _pose_distance(prev_pose, next_pose) -> float:
    return math.hypot(
        next_pose.pose.position.x - prev_pose.pose.position.x,
        next_pose.pose.position.y - prev_pose.pose.position.y,
    )


def _split_path_by_coverage_points(
    path: Path,
    coverage_split_points: list[Pose],
    split_tolerance_m: float,
) -> list[Path]:
    """Split a coverage path, always preserving the final segment."""
    if len(path.poses) < 2:
        return []

    split_xy = [
        (p.position.x, p.position.y) for p in coverage_split_points
    ]

    def _is_split_point(x, y):
        return any(
            math.hypot(x - sx, y - sy) <= split_tolerance_m
            for sx, sy in split_xy
        )

    segments = []
    current = _new_path_like(path)

    for idx, pose in enumerate(path.poses):
        current.poses.append(pose)
        is_internal_pose = 0 < idx < len(path.poses) - 1
        if is_internal_pose and _is_split_point(
            pose.pose.position.x, pose.pose.position.y
        ):
            if len(current.poses) > 1:
                segments.append(current)
            current = _new_path_like(path)
            current.poses.append(pose)

    if len(current.poses) > 1:
        segments.append(current)

    return segments


def _split_path_by_turn_angle(
    path: Path,
    turn_angle_rad: float,
    min_segment_length_m: float,
) -> list[Path]:
    """Split a path at sharp turns so the controller can re-align heading."""
    if len(path.poses) < 3 or turn_angle_rad <= 0.0:
        return [path] if len(path.poses) >= 2 else []

    segments = []
    current = _new_path_like(path)
    current.poses.append(path.poses[0])
    current_distance = 0.0

    for idx in range(1, len(path.poses) - 1):
        prev_pose = path.poses[idx - 1]
        pose = path.poses[idx]
        next_pose = path.poses[idx + 1]
        current_distance += _pose_distance(prev_pose, pose)
        current.poses.append(pose)

        incoming = _segment_heading(prev_pose, pose)
        outgoing = _segment_heading(pose, next_pose)
        turn_angle = _angle_diff(incoming, outgoing)
        if (
            turn_angle >= turn_angle_rad
            and current_distance >= min_segment_length_m
        ):
            if len(current.poses) > 1:
                segments.append(current)
            current = _new_path_like(path)
            current.poses.append(pose)
            current_distance = 0.0

    current_distance += _pose_distance(path.poses[-2], path.poses[-1])
    current.poses.append(path.poses[-1])
    if len(current.poses) > 1:
        segments.append(current)

    return segments


def _split_paths_by_turn_angle(
    paths: list[Path],
    turn_angle_rad: float,
    min_segment_length_m: float,
) -> list[Path]:
    """Apply sharp-turn splitting to every path segment."""
    split_paths = []
    for path in paths:
        split_paths.extend(
            _split_path_by_turn_angle(
                path,
                turn_angle_rad,
                min_segment_length_m,
            )
        )
    return split_paths


def _split_path_by_max_distance(path: Path, max_distance_m: float) -> list[Path]:
    """Split a path into shorter consecutive chunks."""
    if len(path.poses) < 2:
        return []
    if max_distance_m <= 0.0:
        return [path]

    segments = []
    current = _new_path_like(path)
    current.poses.append(path.poses[0])
    current_distance = 0.0

    for idx in range(1, len(path.poses)):
        prev = path.poses[idx - 1]
        pose = path.poses[idx]
        current_distance += math.hypot(
            pose.pose.position.x - prev.pose.position.x,
            pose.pose.position.y - prev.pose.position.y,
        )
        current.poses.append(pose)

        is_last_pose = idx == len(path.poses) - 1
        if current_distance >= max_distance_m and not is_last_pose:
            if len(current.poses) > 1:
                segments.append(current)
            current = _new_path_like(path)
            current.poses.append(pose)
            current_distance = 0.0

    if len(current.poses) > 1:
        segments.append(current)

    return segments


def _split_paths_by_max_distance(
    paths: list[Path],
    max_distance_m: float,
) -> list[Path]:
    """Apply max-distance splitting to every path segment."""
    split_paths = []
    for path in paths:
        split_paths.extend(_split_path_by_max_distance(path, max_distance_m))
    return split_paths


class NavActionServer(Node):

    def __init__(self):
        super().__init__('nav_action_server')
        self.get_logger().info('NavActionServer initialized')

        self.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])
        self.navigator = BasicNavigator()
        self.controller_id = 'FollowPath'
        self.goal_checker_id = 'general_goal_checker'
        self.action_server = ActionServer(
            self,
            Waypoint,
            'nav_action',
            self.execute_callback
        )

        self.action_server_follow_path = ActionServer(
            self,
            Waypoint,
            'nav_action_follow_path',
            self.single_path_execute_callback
        )
        self.declare_parameter('split_tolerance_m', 0.1)
        self.declare_parameter('max_follow_segment_length_m', 4.0)
        self.declare_parameter('turn_split_angle_rad', 0.8)
        self.declare_parameter('turn_split_min_segment_length_m', 0.25)
        self.declare_parameter('coverage_segment_success_distance_m', 0.25)
        self.split_path_pub = self.create_publisher(Path, '/split_path', 1)
        self.coverage_split_points_pub = self.create_publisher(
            Marker, '/coverage_split_points', 1
        )
        self.coverage_split_points = []
        self.recent_nav2_logs = deque(maxlen=40)
        self._active_goal_handle = None
        self._active_task_name = None
        self._external_cancel_requested = False
        self._nav_state = 'idle'
        self._last_feedback_message = 'last_feedback=None'
        self._last_status_message = 'Navigation idle'
        self.create_subscription(Log, '/rosout', self._rosout_callback, 100)
        self.create_service(Trigger, '/cancel_nav2', self.cancel_nav2_srv)
        self.create_service(Trigger, '/cencel_nav2', self.cancel_nav2_srv)
        self.create_service(
            Trigger,
            '/check_nav_status',
            self.check_nav_status_srv,
        )

    def _rosout_callback(self, msg: Log):
        if msg.level < Log.WARN:
            return
        nav2_nodes = (
            'controller_server',
            'planner_server',
            'bt_navigator',
            'behavior_server',
            'velocity_smoother',
            'local_costmap',
            'global_costmap',
        )
        if not any(name in msg.name for name in nav2_nodes):
            return
        self.recent_nav2_logs.append((monotonic(), msg.name, msg.msg))

    def _navigator_task_error(self) -> str:
        get_task_error = getattr(self.navigator, 'getTaskError', None)
        if get_task_error is None:
            return 'Nav2 task error: getTaskError() unavailable'
        try:
            error_code, error_msg = get_task_error()
        except Exception as exc:
            return f'Nav2 task error unavailable: {exc}'
        return f'Nav2 task error: code={error_code}, msg="{error_msg}"'

    def _feedback_summary(self, feedback) -> str:
        if feedback is None:
            return 'last_feedback=None'

        fields = []
        for name in ('distance_to_goal', 'distance_remaining', 'speed'):
            if hasattr(feedback, name):
                value = getattr(feedback, name)
                if isinstance(value, float):
                    fields.append(f'{name}={value:.3f}')
                else:
                    fields.append(f'{name}={value}')
        return ', '.join(fields) if fields else f'last_feedback={feedback}'

    def _set_nav_state(self, state: str, message: str):
        self._nav_state = state
        self._last_status_message = message

    def cancel_nav2_srv(self, req, res):
        """Cancel the Nav2 task owned by this action server."""
        if self._nav_state not in ('running', 'canceling'):
            res.success = True
            res.message = 'No active Nav2 task to cancel'
            return res

        self._external_cancel_requested = True
        task_name = self._active_task_name or 'Nav2 task'
        try:
            self.navigator.cancelTask()
        except Exception as exc:
            res.success = False
            res.message = f'Failed to request Nav2 cancel: {exc}'
            self.get_logger().error(res.message)
            return res

        self._set_nav_state('canceling', f'Cancel requested for {task_name}')
        res.success = True
        res.message = self._last_status_message
        self.get_logger().warn(res.message)
        return res

    def check_nav_status_srv(self, req, res):
        """Return status for the Nav2 task owned by this action server."""
        if self._nav_state in ('running', 'canceling'):
            try:
                is_complete = self.navigator.isTaskComplete()
                feedback = self.navigator.getFeedback()
            except Exception as exc:
                res.success = False
                res.message = f'Navigation status unavailable: {exc}'
                return res

            if feedback is not None:
                self._last_feedback_message = self._feedback_summary(feedback)

            if is_complete:
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self._set_nav_state(
                        'completed',
                        f'{self._active_task_name or "Navigation"} completed',
                    )
                elif result == TaskResult.CANCELED:
                    self._set_nav_state(
                        'canceled',
                        f'{self._active_task_name or "Navigation"} canceled',
                    )
                else:
                    self._set_nav_state(
                        'failed',
                        f'{self._active_task_name or "Navigation"} failed: '
                        f'{result}',
                    )
                self._active_goal_handle = None
                self._active_task_name = None

            if not is_complete:
                res.success = False
                res.message = (
                    f'Navigation {self._nav_state}: '
                    f'{self._active_task_name or "unknown task"}, '
                    f'{self._last_feedback_message}'
                )
                return res

        res.success = self._nav_state in ('idle', 'completed', 'canceled')
        res.message = self._last_status_message
        return res

    def _feedback_distance(self, feedback) -> float | None:
        """Return remaining distance for FollowPath or NavigateToPose feedback."""
        for name in ('distance_to_goal', 'distance_remaining'):
            if hasattr(feedback, name):
                return float(getattr(feedback, name))
        return None

    def _recent_nav2_log_summary(self, since: float) -> str:
        recent = [
            (name, msg)
            for stamp, name, msg in self.recent_nav2_logs
            if stamp >= since
        ]
        if not recent:
            return 'recent_nav2_logs=None'

        lines = []
        for name, msg in recent[-5:]:
            lines.append(f'[{name}] {msg}')
        return 'recent_nav2_logs=' + ' | '.join(lines)

    def _stamp_path_for_execution(self, path: Path):
        """Refresh path stamps right before passing them to Nav2."""
        frame_id = _path_frame_id(path)
        now = self.navigator.get_clock().now().to_msg()
        path.header.frame_id = frame_id
        path.header.stamp = now
        for pose in path.poses:
            pose.header.frame_id = pose.header.frame_id or frame_id
            pose.header.stamp = now

    def _wait_for_nav_task(
        self,
        goal_handle,
        task_name: str,
        success_distance_m: float | None = None,
    ) -> bool:
        started_at = monotonic()
        last_feedback = None
        self._active_goal_handle = goal_handle
        self._active_task_name = task_name
        self._set_nav_state('running', f'Navigation running: {task_name}')
        while not self.navigator.isTaskComplete():
            if goal_handle.is_cancel_requested or self._external_cancel_requested:
                cancel_source = (
                    'external service'
                    if self._external_cancel_requested
                    else 'action client'
                )
                self.get_logger().warn(f'{task_name} 已取消 ({cancel_source})')
                self.navigator.cancelTask()
                goal_handle.canceled()
                self._external_cancel_requested = False
                self._active_goal_handle = None
                self._active_task_name = None
                self._set_nav_state('canceled', f'{task_name} canceled')
                return False
            feedback = self.navigator.getFeedback()
            if feedback:
                last_feedback = feedback
                self._last_feedback_message = self._feedback_summary(feedback)
                self.get_logger().info(f'{task_name} 反饋: {feedback}')
                distance = self._feedback_distance(feedback)
                if (
                    success_distance_m is not None
                    and distance is not None
                    and distance <= success_distance_m
                ):
                    elapsed = monotonic() - started_at
                    self.get_logger().info(
                        f'{task_name} 已接近終點 '
                        f'({distance:.3f} m <= '
                        f'{success_distance_m:.3f} m)，'
                        f'耗時 {elapsed:.1f}s，切換下一段'
                    )
                    self.navigator.cancelTask()
                    return True

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self._set_nav_state('completed', f'{task_name} completed')
            self._active_goal_handle = None
            self._active_task_name = None
            return True

        if result == TaskResult.CANCELED:
            goal_handle.canceled()
            self._external_cancel_requested = False
            self._active_goal_handle = None
            self._active_task_name = None
            self._set_nav_state('canceled', f'{task_name} canceled')
            return False

        elapsed = monotonic() - started_at
        task_error = self._navigator_task_error()
        feedback_summary = self._feedback_summary(last_feedback)
        nav2_log_summary = self._recent_nav2_log_summary(started_at)
        self.get_logger().error(
            f'{task_name} 失敗: {result}，耗時 {elapsed:.1f}s，'
            f'{feedback_summary}，{task_error}，{nav2_log_summary}'
        )
        goal_handle.abort()
        self._active_goal_handle = None
        self._active_task_name = None
        self._set_nav_state(
            'failed',
            f'{task_name} failed: {result}, {task_error}',
        )
        return False

    def _abort_goal(self, goal_handle, message: str):
        self.get_logger().error(message)
        goal_handle.abort()
        result = Waypoint.Result()
        result.success = False
        return result

    # 發佈單條路徑
    def single_path_execute_callback(self, goal_handle):
        """
        Execute a single path.

        Parameters
        ----------
        goal_handle : GoalHandle
            The goal handle.

        """
        self.get_logger().info('執行目標')
        self._external_cancel_requested = False
        path = goal_handle.request.path
        coverage_split_points = goal_handle.request.coverage_split_points

        self.get_logger().info(f'path 長度: {len(path.poses)}')
        self.get_logger().info(
            f'coverage_split_points 長度: {len(coverage_split_points)}'
        )

        if not path.poses:
            return self._abort_goal(goal_handle, '收到空路徑，取消導航')

        self.coverage_split_points = list(coverage_split_points)
        self.publish_split_points_marker()

        split_tolerance = self.get_parameter('split_tolerance_m').value
        max_segment_length = self.get_parameter(
            'max_follow_segment_length_m'
        ).value
        turn_split_angle = self.get_parameter('turn_split_angle_rad').value
        turn_split_min_length = self.get_parameter(
            'turn_split_min_segment_length_m'
        ).value
        segment_success_distance = self.get_parameter(
            'coverage_segment_success_distance_m'
        ).value

        self._stamp_path_for_execution(path)
        self.get_logger().info('導航到覆蓋路徑起點')
        self.navigator.goToPose(path.poses[0])
        if not self._wait_for_nav_task(goal_handle, 'Nav to coverage start'):
            result = Waypoint.Result()
            result.success = False
            return result

        split_paths = _split_path_by_coverage_points(
            path, coverage_split_points, split_tolerance
        )
        coverage_split_count = len(split_paths)
        split_paths = _split_paths_by_turn_angle(
            split_paths,
            turn_split_angle,
            turn_split_min_length,
        )
        turn_split_count = len(split_paths)
        split_paths = _split_paths_by_max_distance(
            split_paths,
            max_segment_length,
        )
        self.get_logger().info(
            f'coverage path 依 split points 切成 {coverage_split_count} 段，'
            f'再依轉角 >= {turn_split_angle:.2f} rad 切成 '
            f'{turn_split_count} 段，'
            f'再依最大 {max_segment_length:.1f} m 切成 '
            f'{len(split_paths)} 段'
        )

        for idx, split_path in enumerate(split_paths, start=1):
            self._stamp_path_for_execution(split_path)
            self.split_path_pub.publish(split_path)
            distance = _path_distance(split_path)
            start = split_path.poses[0].pose.position
            goal = split_path.poses[-1].pose.position
            self.get_logger().info(
                f'執行第 {idx}/{len(split_paths)} 段 coverage path，'
                f'共 {len(split_path.poses)} 點，長度 {distance:.3f} m，'
                f'起點 ({start.x:.3f}, {start.y:.3f})，'
                f'終點 ({goal.x:.3f}, {goal.y:.3f})'
            )
            if len(split_path.poses) < 2 or distance < 0.05:
                self.get_logger().warn(
                    f'第 {idx} 段 path 太短，跳過 followPath'
                )
                continue
            self.navigator.followPath(
                split_path,
                self.controller_id,
                self.goal_checker_id,
            )
            if not self._wait_for_nav_task(
                goal_handle,
                f'Follow coverage segment {idx}',
                success_distance_m=segment_success_distance,
            ):
                result = Waypoint.Result()
                result.success = False
                return result

        goal_handle.succeed()
        self._active_goal_handle = None
        self._active_task_name = None
        self._set_nav_state('completed', 'Coverage navigation completed')
        result = Waypoint.Result()
        result.success = True
        return result

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal')
        self._external_cancel_requested = False
        path = goal_handle.request.path
        self.get_logger().info(f'path length: {len(path.poses)}')

        for pose in path.poses:
            print(f'pose1: {pose.pose.position.x}, {pose.pose.position.y}')
            print('--------------------------------')
        for i in range(len(path.poses)-1):
            pose1 = path.poses[i]
            pose2 = path.poses[i+1]

            def euler_to_quaternion(roll, pitch, yaw):
                """将欧拉角转换为四元数（x, y, z, w）."""
                qx = (
                    math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2)
                    - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
                )
                qy = (
                    math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
                    + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
                )
                qz = (
                    math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
                    - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
                )
                qw = (
                    math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2)
                    + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
                )
                return (qx, qy, qz, qw)

            dx = pose2.pose.position.x - pose1.pose.position.x
            dy = pose2.pose.position.y - pose1.pose.position.y
            yaw = math.atan2(dy, dx)
            q = euler_to_quaternion(0, 0, yaw)

            pose1.pose.orientation.x = q[0]
            pose1.pose.orientation.y = q[1]
            pose1.pose.orientation.z = q[2]
            pose1.pose.orientation.w = q[3]

            pose2.pose.orientation.x = q[0]
            pose2.pose.orientation.y = q[1]
            pose2.pose.orientation.z = q[2]
            pose2.pose.orientation.w = q[3]
            print(
                f'pose1: {pose1.pose.position.x}, {pose1.pose.position.y}, '
                f'pose2: {pose2.pose.position.x}, {pose2.pose.position.y}'
            )
            print('--------------------------------')
            nav_path = self.navigator.getPath(pose1, pose2)
            smoothed_path = self.navigator.smoothPath(nav_path)
            self.navigator.followPath(
                smoothed_path,
                self.controller_id,
                self.goal_checker_id,
            )
            if not self._wait_for_nav_task(
                goal_handle,
                f'Follow path segment {i + 1}',
            ):
                result = Waypoint.Result()
                result.success = False
                return result
        goal_handle.succeed()
        self._active_goal_handle = None
        self._active_task_name = None
        self._set_nav_state('completed', 'Navigation completed')
        result = Waypoint.Result()
        result.success = True
        return result

    def destroy_node(self):
        self.action_server.destroy()
        self.action_server_follow_path.destroy()
        self.navigator.destroy_node()
        super().destroy_node()

    def publish_split_points_marker(self):
        """在 RViz 上可视化覆盖路径的分割点."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'coverage_split_points'
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD

        # 设置球体大小
        marker.scale.x = 0.15  # 球体直径
        marker.scale.y = 0.15
        marker.scale.z = 0.15

        # 设置颜色 (红色，半透明)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        # 添加所有分割点
        for pose in self.coverage_split_points:
            point = Point()
            point.x = pose.position.x
            point.y = pose.position.y
            point.z = 0.1  # 稍微抬高一点，避免与地面重叠
            marker.points.append(point)

            # 为每个点添加颜色（可选，如果不添加则使用 marker.color）
            color = ColorRGBA()
            color.r = 1.0
            color.g = 0.0
            color.b = 0.0
            color.a = 0.8
            marker.colors.append(color)

        self.get_logger().info(
            f'发布 {len(self.coverage_split_points)} 个分割点到 RViz'
        )
        self.coverage_split_points_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    nav_action_server = NavActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(nav_action_server)
    try:
        executor.spin()
    finally:
        executor.remove_node(nav_action_server)
        nav_action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
