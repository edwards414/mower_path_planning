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
import json
import math
import os
import threading
from collections import deque
from time import monotonic, sleep, time
from uuid import UUID

from geometry_msgs.msg import Point, Pose, PoseStamped, TwistStamped
from lifecycle_msgs.srv import GetState
from action_msgs.msg import GoalStatus

from mower_interface.action import Waypoint
from mower_interface.msg import CoverageProgress
from mower_interface.srv import (
    CancelNavigationDispatch,
    ConfirmNavigationDispatch,
    GetCoverageProgress,
    MissionOperationLock,
)
from mower_mission.navigation import coverage_progress
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_msgs.action import FollowPath, NavigateToPose
from nav_msgs.msg import Odometry, Path

from rcl_interfaces.msg import Log, ParameterDescriptor
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)

from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Bool, ColorRGBA
from std_srvs.srv import Trigger

from visualization_msgs.msg import Marker

# Minimum spacing of feedback-driven /coverage_progress messages; state
# transitions publish immediately.
PROGRESS_FEEDBACK_PERIOD_S = 0.5


def _path_frame_id(path: Path) -> str:
    return path.header.frame_id or 'map'


def _canonical_dispatch_id(value) -> str | None:
    """Return one canonical UUID token or None for malformed/reused input."""
    try:
        return UUID(str(value).strip()).hex
    except (AttributeError, TypeError, ValueError):
        return None


def _navigation_path_block_reason(path: Path) -> str | None:
    """Reject paths that could produce fake success or undefined Nav2 math."""
    if len(path.poses) < 2:
        return 'navigation path requires at least two poses'
    path_frame = _path_frame_id(path)
    if path_frame != 'map':
        return f'navigation path frame must be map, got {path_frame}'
    for index, stamped_pose in enumerate(path.poses):
        pose_frame = stamped_pose.header.frame_id or path_frame
        if pose_frame != path_frame:
            return f'navigation pose {index} frame does not match path frame'
        pose = stamped_pose.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            return f'navigation pose {index} contains a non-finite value'
        quaternion_norm = math.sqrt(sum(
            float(value) ** 2
            for value in (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
        ))
        if not math.isclose(quaternion_norm, 1.0, abs_tol=1e-3):
            return f'navigation pose {index} has an invalid quaternion'
    return None


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


def _coverage_segments_block_reason(
    split_paths: list[Path],
    minimum_distance_m: float = 0.15,
) -> str | None:
    """Reject segments that Nav2 can accept without producing any motion."""
    if not split_paths:
        return 'coverage split produced no executable segments'
    for index, split_path in enumerate(split_paths, start=1):
        if len(split_path.poses) < 2:
            return f'coverage segment {index} has fewer than two poses'
        distance = _path_distance(split_path)
        if not math.isfinite(distance):
            return f'coverage segment {index} has a non-finite distance'
        if distance <= minimum_distance_m:
            return (
                f'coverage segment {index} is only {distance:.3f} m; '
                'planner split points must be coalesced above the 0.05 m '
                'goal tolerance plus one 0.10 m mission-map cell'
            )
    return None


def _imu_covariance_block_reason(
    covariance,
    max_sigma: float,
    label: str,
) -> str | None:
    """Reject malformed or overly uncertain 3x3 IMU covariance matrices."""
    if len(covariance) < 9:
        return f'IMU {label} covariance is unavailable or invalid'
    values = tuple(float(value) for value in covariance[:9])
    if not all(math.isfinite(value) for value in values):
        return f'IMU {label} covariance is unavailable or invalid'
    matrix = (
        values[0:3],
        values[3:6],
        values[6:9],
    )
    entry_scale = max(1e-12, *(abs(value) for value in values))
    symmetry_tolerance = 1e-6 * entry_scale
    if any(
        abs(matrix[row][column] - matrix[column][row])
        > symmetry_tolerance
        for row in range(3)
        for column in range(row + 1, 3)
    ):
        return f'IMU {label} covariance is unavailable or invalid'
    max_variance = max(0.0, float(max_sigma)) ** 2
    # A symmetric covariance matrix is positive semidefinite only if all
    # principal minors are non-negative. Gershgorin's bound then provides a
    # dependency-free, conservative upper bound on its largest eigenvalue.
    diagonal = tuple(matrix[index][index] for index in range(3))
    if any(value <= 0.0 for value in diagonal):
        return f'IMU {label} covariance is unavailable or invalid'
    minor_epsilon = 1e-9 * entry_scale ** 2
    determinant_epsilon = 1e-9 * entry_scale ** 3
    principal_minors = (
        diagonal[0] * diagonal[1] - matrix[0][1] ** 2,
        diagonal[0] * diagonal[2] - matrix[0][2] ** 2,
        diagonal[1] * diagonal[2] - matrix[1][2] ** 2,
    )
    determinant = (
        matrix[0][0]
        * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1]
        * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2]
        * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    )
    if (
        any(value < -minor_epsilon for value in principal_minors)
        or determinant < -determinant_epsilon
    ):
        return f'IMU {label} covariance is unavailable or invalid'
    eigenvalue_upper_bound = max(
        matrix[row][row]
        + sum(
            abs(matrix[row][column])
            for column in range(3)
            if column != row
        )
        for row in range(3)
    )
    if eigenvalue_upper_bound > max_variance:
        return (
            f'IMU {label} uncertainty exceeds '
            f'{max(0.0, float(max_sigma)):.3f}'
        )
    return None


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

    def __init__(self, *, parameter_overrides=None):
        super().__init__(
            'nav_action_server',
            parameter_overrides=parameter_overrides,
        )
        self.get_logger().info('NavActionServer initialized')

        # Action execution is long-running. It must be reentrant so action
        # cancellation can be processed while execute_callback is active. The
        # control services live in a separate mutually-exclusive group so
        # status/cancel also remain responsive without racing each other.
        self._action_callback_group = ReentrantCallbackGroup()
        self._control_callback_group = MutuallyExclusiveCallbackGroup()
        # Lock admission must not queue behind a bounded Nav2 cancel in the
        # control group. Both paths still serialize their state through
        # _state_lock, while this dedicated group prevents a late acquire from
        # arriving after the requester's guard timeout and latching a stale
        # mutation owner.
        self._mutation_callback_group = MutuallyExclusiveCallbackGroup()
        self._health_callback_group = MutuallyExclusiveCallbackGroup()
        self._state_lock = threading.RLock()
        self._navigator_lock = threading.RLock()
        self._goal_reserved = False
        self._active_goal_handle = None
        self._active_task_name = None
        self._last_task_name = None
        self._external_cancel_requested = False
        # Accepted ROS actions stay motion-locked until their caller has
        # received the goal handle and explicitly confirms the dispatch.  This
        # prevents a late goal response from starting an untracked mower after
        # the initiating service has already timed out.
        self._dispatch_confirmation_event = threading.Event()
        self._dispatch_confirmed = False
        self._pending_dispatch_id = None
        self._seen_dispatch_ids = set()
        self._terminal_dispatch_ids = set()
        self._uncertain_dispatch_id = None
        self._nav_state = 'idle'
        self._last_feedback_message = 'last_feedback=None'
        self._last_status_message = 'Navigation idle'
        # Latched when an action returns without proving that its Nav2 goal is
        # terminal. The outer ROS action may be aborted, but accepting another
        # goal in that state could overlap two real robot motions.
        self._nav2_task_uncertain = False
        # True from immediately before a Nav2 action dispatch until a definite
        # rejection or terminal result. The goal-handle assignment itself is
        # not a safe boundary: Nav2 may accept a goal whose local response is
        # then lost before BasicNavigator stores that handle.
        self._nav2_dispatch_in_flight_or_active = False
        self._nav2_current_goal_handle = None
        self._nav2_current_result_future = None
        self._nav2_dispatch_generation = 0
        self._nav2_active_generation = None
        # Mission geometry/map mutations use a central lease. Goal admission
        # and lease admission share _state_lock, making "start navigation" vs
        # "change the plan" an atomic decision across ROS processes.
        self._mutation_owner = None
        self._mutation_operation = None
        self._manual_command_deadlines = {}
        self._invalid_manual_command_deadlines = {}
        self._valid_fix_received_at_by_topic = {}
        self._fix_rejections_by_topic = {}
        self._last_gps_odometry_received_at = None
        self._gps_odometry_rejection_reason = None
        self._last_robot_pose_received_at = None
        self._robot_pose_rejection_reason = None
        self._last_imu_received_at = None
        self._imu_rejection_reason = None

        self.navigator = BasicNavigator()
        self.navigator.set_parameters([
            Parameter(
                'use_sim_time',
                Parameter.Type.BOOL,
                bool(self.get_parameter('use_sim_time').value),
            )
        ])
        self.controller_id = 'FollowPath'
        self.goal_checker_id = 'general_goal_checker'
        # Use action clients owned by this node. BasicNavigator's convenience
        # methods synchronously wait forever for the server and goal response;
        # a lost response can otherwise deadlock cancellation while holding the
        # navigator lock.
        self._navigate_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self._action_callback_group,
        )
        self._follow_path_client = ActionClient(
            self,
            FollowPath,
            'follow_path',
            callback_group=self._action_callback_group,
        )
        self.action_server = ActionServer(
            self,
            Waypoint,
            'nav_action',
            self._guarded_execute_callback,
            callback_group=self._action_callback_group,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        self.action_server_follow_path = ActionServer(
            self,
            Waypoint,
            'nav_action_follow_path',
            self._guarded_single_path_execute_callback,
            callback_group=self._action_callback_group,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self.declare_parameter('split_tolerance_m', 0.1)
        # 0.0 disables max-distance chopping (see _split_path_by_max_distance
        # guard): each straight boustrophedon row runs as ONE continuous
        # FollowPath. Turn/coverage-point splits still apply. Every segment must
        # now reach a terminal Nav2 SUCCEEDED result; feedback-distance cancel
        # was unsafe because BasicNavigator feedback is not goal-correlated.
        self.declare_parameter('max_follow_segment_length_m', 0.0)
        self.declare_parameter('turn_split_angle_rad', 0.8)
        self.declare_parameter('turn_split_min_segment_length_m', 0.25)
        self.declare_parameter('nav2_ready_timeout_s', 30.0)
        self.declare_parameter('nav2_ready_poll_s', 0.2)
        self.declare_parameter('nav2_cancel_timeout_s', 3.0)
        self.declare_parameter('nav2_cancel_poll_s', 0.05)
        self.declare_parameter('nav2_action_server_timeout_s', 3.0)
        self.declare_parameter('nav2_goal_response_timeout_s', 3.0)
        self.declare_parameter('dispatch_confirmation_timeout_s', 5.0)
        self.declare_parameter('progress_checkpoint_enabled', True)
        self.declare_parameter(
            'progress_checkpoint_path',
            '~/.ros/mower_mission/coverage_progress.json',
        )
        self.declare_parameter('progress_checkpoint_interval_sec', 1.0)
        def immutable_safety_parameter(description):
            return ParameterDescriptor(
                read_only=True,
                description=description,
            )

        # These values define whether the real mower may move. Launch-time
        # overrides remain supported (the simulator starts with health checks
        # disabled), but a runtime parameter service must never weaken them.
        self.declare_parameter(
            'manual_command_hold_s',
            0.75,
            immutable_safety_parameter('Manual/autonomy exclusion hold'),
        )
        self.declare_parameter(
            'require_navigation_health',
            True,
            immutable_safety_parameter('Require pose/GPS/IMU health'),
        )
        self.declare_parameter(
            'navigation_health_timeout_s',
            0.30,
            immutable_safety_parameter('Sensor freshness deadline'),
        )
        self.declare_parameter(
            'navigation_health_max_future_skew_s',
            0.5,
            immutable_safety_parameter('Maximum future sensor stamp skew'),
        )
        self.declare_parameter(
            'max_gps_horizontal_sigma_m',
            0.015,
            immutable_safety_parameter('Maximum GPS horizontal uncertainty'),
        )
        self.declare_parameter(
            'max_imu_orientation_sigma_rad',
            0.35,
            immutable_safety_parameter('Maximum IMU orientation uncertainty'),
        )
        self.declare_parameter(
            'max_imu_angular_velocity_sigma_rad_s',
            0.10,
            immutable_safety_parameter(
                'Maximum IMU angular-velocity uncertainty'
            ),
        )
        self.declare_parameter(
            'max_imu_linear_acceleration_sigma_m_s2',
            0.50,
            immutable_safety_parameter(
                'Maximum IMU linear-acceleration uncertainty'
            ),
        )
        self.declare_parameter(
            'gps_fix_topic',
            '/fix',
            immutable_safety_parameter('Canonical raw GPS health source'),
        )
        self.declare_parameter(
            'gps_odometry_topic',
            '/odometry/gps',
            immutable_safety_parameter('Fused GPS odometry health source'),
        )
        self.declare_parameter(
            'imu_topic',
            '/imu/data',
            immutable_safety_parameter('IMU health source'),
        )
        self._nav2_state_client = self.create_client(
            GetState,
            'bt_navigator/get_state',
            callback_group=self._action_callback_group,
        )
        self.split_path_pub = self.create_publisher(Path, '/split_path', 1)
        self.coverage_split_points_pub = self.create_publisher(
            Marker, '/coverage_split_points', 1
        )
        self.coverage_split_points = []
        self.recent_nav2_logs = deque(maxlen=40)
        nav_active_qos = QoSProfile(depth=1)
        nav_active_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        nav_active_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self._nav_operation_active_pub = self.create_publisher(
            Bool,
            '/nav_operation_active',
            nav_active_qos,
        )
        self._navigation_safety_stop_pub = self.create_publisher(
            TwistStamped,
            '/navigation_safety_stop',
            10,
        )
        self._navigation_coordinator_lock_pub = self.create_publisher(
            Bool,
            '/navigation_coordinator_lock',
            10,
        )
        self._publish_nav_operation_active(False)
        self._progress_lock = threading.Lock()
        self._progress = coverage_progress.CoverageProgressTracker()
        self._progress_robot_pose = PoseStamped()
        self._progress_last_published = None
        self._checkpoint_path = None
        if bool(self.get_parameter('progress_checkpoint_enabled').value):
            self._checkpoint_path = os.path.expanduser(str(
                self.get_parameter('progress_checkpoint_path').value
            ))
        self._checkpoint_interval_s = float(
            self.get_parameter('progress_checkpoint_interval_sec').value
        )
        self._checkpoint = self._load_checkpoint()
        self._checkpoint_last_write = None
        if self._checkpoint is not None:
            self._progress = coverage_progress.restored_tracker(
                self._checkpoint
            )
        self._coverage_progress_pub = self.create_publisher(
            CoverageProgress,
            '/coverage_progress',
            nav_active_qos,
        )
        self._update_progress(lambda progress: True)
        for topic in ('/joy_cmd', '/physical_joy_cmd', '/keyboard_cmd_vel'):
            self.create_subscription(
                TwistStamped,
                topic,
                lambda msg, source=topic: self._manual_cmd_callback(
                    msg, source
                ),
                10,
                callback_group=self._action_callback_group,
            )
        self.create_subscription(
            PoseStamped,
            '/adapter/robot_pose',
            self._robot_pose_health_callback,
            10,
            callback_group=self._health_callback_group,
        )
        gps_fix_topic = str(
            self.get_parameter('gps_fix_topic').value
        ).strip() or '/fix'
        gps_odometry_topic = str(
            self.get_parameter('gps_odometry_topic').value
        ).strip() or '/odometry/gps'
        imu_topic = str(
            self.get_parameter('imu_topic').value
        ).strip() or '/imu/data'
        self.create_subscription(
            Imu,
            imu_topic,
            self._imu_health_callback,
            qos_profile_sensor_data,
            callback_group=self._health_callback_group,
        )
        self.create_subscription(
            NavSatFix,
            gps_fix_topic,
            lambda msg, source=gps_fix_topic: self._gps_health_callback(
                msg, source
            ),
            qos_profile_sensor_data,
            callback_group=self._health_callback_group,
        )
        self.create_subscription(
            Odometry,
            gps_odometry_topic,
            self._gps_odometry_health_callback,
            10,
            callback_group=self._health_callback_group,
        )
        self.create_subscription(Log, '/rosout', self._rosout_callback, 100)
        self.create_service(
            Trigger,
            '/cancel_nav2',
            self.cancel_nav2_srv,
            callback_group=self._control_callback_group,
        )
        self.create_service(
            Trigger,
            '/cencel_nav2',
            self.cancel_nav2_srv,
            callback_group=self._control_callback_group,
        )
        self.create_service(
            GetCoverageProgress,
            '/coverage_progress_status',
            self.coverage_progress_status_srv,
            callback_group=self._control_callback_group,
        )
        self.create_service(
            Trigger,
            '/check_nav_status',
            self.check_nav_status_srv,
            callback_group=self._control_callback_group,
        )
        self.create_service(
            ConfirmNavigationDispatch,
            '/confirm_navigation_dispatch',
            self.confirm_navigation_dispatch_srv,
            callback_group=self._control_callback_group,
        )
        self.create_service(
            CancelNavigationDispatch,
            '/cancel_navigation_dispatch',
            self.cancel_navigation_dispatch_srv,
            callback_group=self._control_callback_group,
        )
        self.create_service(
            MissionOperationLock,
            '/mission_operation_lock',
            self.mission_operation_lock_srv,
            callback_group=self._mutation_callback_group,
        )
        self.create_timer(
            0.5,
            self._monitor_uncertain_nav2_task,
            callback_group=self._control_callback_group,
        )
        # The 20 Hz tick (health monitor, then the fail-safe lock heartbeat)
        # runs on its own thread instead of an executor timer: every rclpy
        # timer wake-up through the MultiThreadedExecutor costs ~12 ms of
        # CPU on the LubanCat (the executor rebuilds its wait set in Python
        # for each event), i.e. a quarter of a core for a callback that
        # mostly publishes two small messages. The thread keeps the same
        # cadence (/navigation_safety_stop timeout 0.2 s, coordinator lock
        # 0.3 s) and the same linearisation: the decision and its publish
        # happen under _state_lock, as they did in the timer callback.
        self._health_tick_period_s = 0.05
        self._health_thread_stop = threading.Event()
        self._health_thread = threading.Thread(
            target=self._health_loop,
            name='nav-health-tick',
            daemon=True,
        )
        self._health_thread.start()

    def _navigation_admission_block_reason_locked(self) -> str | None:
        """Return the exact reason a new autonomous goal is currently unsafe."""
        if self._nav2_task_uncertain:
            return 'previous Nav2 task termination is unconfirmed'
        if self._goal_reserved:
            return 'another navigation goal is active'
        if self._mutation_owner is not None:
            return (
                'mission mutation is active: '
                f'{self._mutation_operation or "unknown"}'
            )
        if self._manual_motion_active_locked():
            return 'manual velocity command is active'
        return self._navigation_health_block_reason_locked()

    def _goal_callback(self, goal_request):
        """Accept one non-empty navigation goal across both action names."""
        path_reason = _navigation_path_block_reason(goal_request.path)
        if path_reason is not None:
            self.get_logger().warn(f'Rejecting navigation goal: {path_reason}')
            return GoalResponse.REJECT
        split_values = (
            value
            for pose in goal_request.coverage_split_points
            for value in (
                pose.position.x,
                pose.position.y,
                pose.position.z,
            )
        )
        if not all(math.isfinite(float(value)) for value in split_values):
            self.get_logger().warn(
                'Rejecting navigation goal with non-finite split points'
            )
            return GoalResponse.REJECT
        dispatch_id = _canonical_dispatch_id(goal_request.dispatch_id)
        if dispatch_id is None:
            self.get_logger().warn(
                'Rejecting navigation goal without a valid dispatch_id'
            )
            return GoalResponse.REJECT
        resume_index = int(getattr(goal_request, 'resume_segment_index', 0))
        if resume_index != 0:
            try:
                hash_value = self._goal_path_hash(
                    goal_request.zone_id,
                    goal_request.path,
                    goal_request.coverage_split_points,
                )
            except (TypeError, ValueError, OverflowError):
                hash_value = ''
            with self._progress_lock:
                resume_reason = coverage_progress.resume_block_reason(
                    self._checkpoint, hash_value, resume_index
                )
            if resume_reason is not None:
                self.get_logger().warn(
                    f'Rejecting navigation goal: cannot resume: {resume_reason}'
                )
                return GoalResponse.REJECT

        with self._state_lock:
            reason = self._navigation_admission_block_reason_locked()
            if reason is not None:
                self.get_logger().warn(
                    f'Rejecting navigation goal: {reason}'
                )
                return GoalResponse.REJECT
            if dispatch_id in self._seen_dispatch_ids:
                self.get_logger().warn(
                    'Rejecting navigation goal with a reused dispatch_id'
                )
                return GoalResponse.REJECT
            self._seen_dispatch_ids.add(dispatch_id)
            self._goal_reserved = True
            self._external_cancel_requested = False
            self._dispatch_confirmed = False
            self._pending_dispatch_id = dispatch_id
            self._dispatch_confirmation_event.clear()
            self._last_task_name = 'Navigation goal'
            self._last_feedback_message = 'last_feedback=None'
            self._nav_state = 'pending_confirmation'
            self._last_status_message = (
                'Navigation goal accepted; waiting for dispatch confirmation'
            )

        self._publish_nav_operation_active(True)
        return GoalResponse.ACCEPT

    def _source_stamp_age_s(self, stamp) -> float | None:
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if stamp_ns <= 0:
            return None
        return (
            self.get_clock().now().nanoseconds - stamp_ns
        ) / 1_000_000_000.0

    def _source_stamp_block_reason(self, stamp, source: str) -> str | None:
        age_s = self._source_stamp_age_s(stamp)
        if age_s is None or not math.isfinite(age_s):
            return f'{source} source timestamp is unavailable'
        timeout_s = max(
            0.10,
            float(self.get_parameter('navigation_health_timeout_s').value),
        )
        future_skew_s = max(
            0.0,
            float(
                self.get_parameter(
                    'navigation_health_max_future_skew_s'
                ).value
            ),
        )
        if age_s < -future_skew_s:
            return f'{source} source timestamp is in the future'
        if age_s > timeout_s:
            return f'{source} source timestamp is stale'
        return None

    def _robot_pose_health_callback(self, msg: PoseStamped) -> None:
        values = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        quaternion_norm = math.sqrt(sum(
            float(value) ** 2
            for value in (
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            )
        ))
        reason = None
        if msg.header.frame_id != 'map':
            reason = 'robot pose frame must be map'
        elif not all(math.isfinite(float(value)) for value in values):
            reason = 'robot pose contains non-finite values'
        elif not math.isfinite(quaternion_norm) or abs(
            quaternion_norm - 1.0
        ) > 1e-2:
            reason = 'robot pose quaternion is not normalized'
        else:
            reason = self._source_stamp_block_reason(
                msg.header.stamp,
                'robot pose',
            )
        if reason is None:
            with self._progress_lock:
                self._progress_robot_pose = msg
        with self._state_lock:
            if reason is None:
                self._last_robot_pose_received_at = monotonic()
                self._robot_pose_rejection_reason = None
            else:
                self._last_robot_pose_received_at = None
                self._robot_pose_rejection_reason = reason

    def _gps_health_callback(
        self,
        msg: NavSatFix,
        source: str = '/fix',
    ) -> None:
        """Accept only recent, finite GPS with known sub-threshold covariance."""
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        covariance = list(msg.position_covariance)
        max_sigma = max(
            0.0,
            float(self.get_parameter('max_gps_horizontal_sigma_m').value),
        )
        reason = None
        valid_statuses = {
            NavSatStatus.STATUS_FIX,
            NavSatStatus.STATUS_SBAS_FIX,
            NavSatStatus.STATUS_GBAS_FIX,
        }
        valid_covariance_types = {
            NavSatFix.COVARIANCE_TYPE_APPROXIMATED,
            NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN,
            NavSatFix.COVARIANCE_TYPE_KNOWN,
        }
        stamp_reason = self._source_stamp_block_reason(
            msg.header.stamp,
            f'GPS {source}',
        )
        if stamp_reason is not None:
            reason = stamp_reason
        elif (
            msg.status.status not in valid_statuses
            or not math.isfinite(lat)
            or not math.isfinite(lon)
            or not -90.0 <= lat <= 90.0
            or not -180.0 <= lon <= 180.0
            or (abs(lat) < 1e-9 and abs(lon) < 1e-9)
        ):
            reason = 'GPS has no valid fix'
        elif (
            msg.position_covariance_type not in valid_covariance_types
            or len(covariance) < 9
        ):
            reason = 'GPS covariance is unknown'
        else:
            variance_x = float(covariance[0])
            variance_y = float(covariance[4])
            covariance_xy = float(covariance[1])
            covariance_yx = float(covariance[3])
            if (
                not math.isfinite(variance_x)
                or not math.isfinite(variance_y)
                or not math.isfinite(covariance_xy)
                or not math.isfinite(covariance_yx)
                or variance_x < 0.0
                or variance_y < 0.0
                or abs(covariance_xy - covariance_yx) > 1e-6
            ):
                reason = 'GPS horizontal covariance is invalid'
            else:
                covariance_cross = 0.5 * (covariance_xy + covariance_yx)
                discriminant = math.sqrt(
                    max(
                        0.0,
                        (variance_x - variance_y) ** 2
                        + 4.0 * covariance_cross ** 2,
                    )
                )
                eigen_max = 0.5 * (
                    variance_x + variance_y + discriminant
                )
                eigen_min = 0.5 * (
                    variance_x + variance_y - discriminant
                )
                if eigen_min <= 1e-12:
                    reason = 'GPS horizontal covariance is degenerate'
                elif eigen_max > max_sigma * max_sigma:
                    reason = (
                        'GPS horizontal uncertainty exceeds '
                        f'{max_sigma:.3f} m'
                    )
        with self._state_lock:
            if reason is None:
                self._valid_fix_received_at_by_topic[source] = monotonic()
                self._fix_rejections_by_topic.pop(source, None)
            else:
                self._valid_fix_received_at_by_topic.pop(source, None)
                self._fix_rejections_by_topic[source] = reason

    def _imu_health_callback(self, msg: Imu) -> None:
        """Require a fresh, finite and covariance-qualified orientation."""
        orientation = msg.orientation
        values = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        )
        reason = self._source_stamp_block_reason(msg.header.stamp, 'IMU')
        if reason is None and msg.header.frame_id != 'imu_link':
            reason = 'IMU frame_id must be imu_link'
        if reason is None and not all(
            math.isfinite(float(value)) for value in values
        ):
            reason = 'IMU contains non-finite values'
        quaternion_norm = math.sqrt(sum(
            float(value) ** 2
            for value in (
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
        ))
        if reason is None and not math.isclose(
            quaternion_norm,
            1.0,
            abs_tol=1e-2,
        ):
            reason = 'IMU orientation quaternion is not normalized'
        covariance_sets = (
            (
                msg.orientation_covariance,
                float(self.get_parameter(
                    'max_imu_orientation_sigma_rad'
                ).value),
                'orientation',
            ),
            (
                msg.angular_velocity_covariance,
                float(self.get_parameter(
                    'max_imu_angular_velocity_sigma_rad_s'
                ).value),
                'angular velocity',
            ),
            (
                msg.linear_acceleration_covariance,
                float(self.get_parameter(
                    'max_imu_linear_acceleration_sigma_m_s2'
                ).value),
                'linear acceleration',
            ),
        )
        if reason is None:
            for covariance, max_sigma, label in covariance_sets:
                reason = _imu_covariance_block_reason(
                    covariance,
                    max_sigma,
                    label,
                )
                if reason is not None:
                    break
        with self._state_lock:
            if reason is None:
                self._last_imu_received_at = monotonic()
                self._imu_rejection_reason = None
            else:
                self._last_imu_received_at = None
                self._imu_rejection_reason = reason

    def _gps_odometry_health_callback(self, msg: Odometry) -> None:
        """Prove navsat_transform is producing fresh, precise map odometry."""
        position = msg.pose.pose.position
        covariance = list(msg.pose.covariance)
        max_sigma = max(
            0.0,
            float(self.get_parameter('max_gps_horizontal_sigma_m').value),
        )
        reason = self._source_stamp_block_reason(
            msg.header.stamp,
            'GPS odometry',
        )
        if reason is None and msg.header.frame_id != 'map':
            reason = 'GPS odometry frame_id must be map'
        if reason is None and not all(math.isfinite(float(value)) for value in (
            position.x,
            position.y,
            position.z,
        )):
            reason = 'GPS odometry position is invalid'
        if reason is None and len(covariance) < 36:
            reason = 'GPS odometry covariance is unavailable'
        if reason is None:
            variance_x = float(covariance[0])
            variance_y = float(covariance[7])
            covariance_xy = float(covariance[1])
            covariance_yx = float(covariance[6])
            if (
                not all(math.isfinite(value) for value in (
                    variance_x,
                    variance_y,
                    covariance_xy,
                    covariance_yx,
                ))
                or variance_x < 0.0
                or variance_y < 0.0
                or abs(covariance_xy - covariance_yx) > 1e-6
            ):
                reason = 'GPS odometry covariance is invalid'
            else:
                covariance_cross = 0.5 * (covariance_xy + covariance_yx)
                discriminant = math.sqrt(max(
                    0.0,
                    (variance_x - variance_y) ** 2
                    + 4.0 * covariance_cross ** 2,
                ))
                eigen_max = 0.5 * (
                    variance_x + variance_y + discriminant
                )
                eigen_min = 0.5 * (
                    variance_x + variance_y - discriminant
                )
                if eigen_min <= 1e-12:
                    reason = 'GPS odometry horizontal covariance is degenerate'
                elif eigen_max > max_sigma * max_sigma:
                    reason = (
                        'GPS odometry horizontal uncertainty exceeds '
                        f'{max_sigma:.3f} m'
                    )
        with self._state_lock:
            if reason is None:
                self._last_gps_odometry_received_at = monotonic()
                self._gps_odometry_rejection_reason = None
            else:
                self._last_gps_odometry_received_at = None
                self._gps_odometry_rejection_reason = reason

    def _navigation_health_block_reason_locked(self) -> str | None:
        if not bool(self.get_parameter('require_navigation_health').value):
            return None
        timeout_s = max(
            0.10,
            float(self.get_parameter('navigation_health_timeout_s').value),
        )
        now = monotonic()
        if (
            self._last_robot_pose_received_at is None
            or now - self._last_robot_pose_received_at > timeout_s
        ):
            if self._robot_pose_rejection_reason:
                return self._robot_pose_rejection_reason
            return 'robot pose/TF is unavailable or stale'
        fix_times = tuple(self._valid_fix_received_at_by_topic.values())
        if not fix_times:
            if self._fix_rejections_by_topic:
                return '; '.join(sorted(set(
                    self._fix_rejections_by_topic.values()
                )))
            return 'GPS fix is unavailable'
        if now - max(fix_times) > timeout_s:
            return 'GPS fix is stale'
        if (
            self._last_gps_odometry_received_at is None
            or now - self._last_gps_odometry_received_at > timeout_s
        ):
            if self._gps_odometry_rejection_reason:
                return self._gps_odometry_rejection_reason
            return 'GPS odometry from navsat_transform is unavailable or stale'
        if (
            self._last_imu_received_at is None
            or now - self._last_imu_received_at > timeout_s
        ):
            if self._imu_rejection_reason:
                return self._imu_rejection_reason
            return 'IMU is unavailable or stale'
        return None

    def _health_tick(self) -> None:
        """20 Hz: cancel on stale health, then heartbeat the fail-safe lock."""
        self._monitor_navigation_health()
        self._publish_safety_stop_if_needed()

    def _health_loop(self) -> None:
        """Drive _health_tick at a fixed cadence until the node is destroyed."""
        period = self._health_tick_period_s
        next_at = monotonic() + period
        while not self._health_thread_stop.wait(max(0.0, next_at - monotonic())):
            next_at += period
            if next_at < monotonic():
                # Fell behind (GIL contention): resynchronise instead of
                # bursting to catch up; a late tick is what twist_mux's
                # timeouts are for.
                next_at = monotonic() + period
            try:
                self._health_tick()
            except Exception as exc:  # noqa: BLE001 - the heartbeat must outlive one bad tick
                self.get_logger().error(f'navigation health tick failed: {exc!r}')

    def _monitor_navigation_health(self) -> None:
        """Cancel an accepted mission if pose or GPS health becomes stale."""
        with self._state_lock:
            reason = self._navigation_health_block_reason_locked()
            should_cancel = self._goal_reserved and reason is not None
            dispatch_id = self._pending_dispatch_id if should_cancel else None
        if should_cancel:
            self._request_navigation_cancel(
                f'Navigation safety stop: {reason}',
                expected_dispatch_id=dispatch_id,
            )
            self.get_logger().error(self._last_status_message)

    def _manual_motion_active_locked(self) -> bool:
        """Return whether any muxed manual source is still within its timeout."""
        now = monotonic()
        expired = [
            source for source, deadline in self._manual_command_deadlines.items()
            if deadline <= now
        ]
        for source in expired:
            self._manual_command_deadlines.pop(source, None)
            self._invalid_manual_command_deadlines.pop(source, None)
        return bool(self._manual_command_deadlines)

    def _manual_cmd_callback(self, msg: TwistStamped, source: str) -> None:
        """Make manual motion and autonomous navigation mutually exclusive."""
        components = (
            float(msg.twist.linear.x),
            float(msg.twist.linear.y),
            float(msg.twist.linear.z),
            float(msg.twist.angular.x),
            float(msg.twist.angular.y),
            float(msg.twist.angular.z),
        )
        invalid = not all(math.isfinite(value) for value in components)
        cancel_dispatch_id = None
        with self._state_lock:
            # twist_mux arbitrates on source freshness and priority, not on
            # whether the Twist is non-zero. A fresh manual zero can mask Nav2
            # and later reveal an old autonomous command when its publisher
            # dies. Treat every fresh manual frame as exclusive ownership and
            # cancel autonomy rather than allowing that unexpected resume.
            hold_s = max(
                0.5,
                float(self.get_parameter('manual_command_hold_s').value),
            )
            deadline = monotonic() + hold_s
            self._manual_command_deadlines[source] = deadline
            if invalid:
                self._invalid_manual_command_deadlines[source] = deadline
            else:
                self._invalid_manual_command_deadlines.pop(source, None)
            if self._goal_reserved:
                cancel_dispatch_id = self._pending_dispatch_id
            if self._nav2_task_uncertain:
                self._external_cancel_requested = True
        if invalid:
            self.get_logger().error(
                f'Non-finite manual velocity received on {source}; '
                'forcing safety stop'
            )
        if cancel_dispatch_id is not None:
            self._request_navigation_cancel(
                f'Manual velocity on {source} requested navigation stop',
                expected_dispatch_id=cancel_dispatch_id,
            )
            self.get_logger().warn(self._last_status_message)

    def _publish_safety_zero(self) -> None:
        stop = TwistStamped()
        stop.header.stamp = self.get_clock().now().to_msg()
        stop.header.frame_id = 'base_footprint'
        self._navigation_safety_stop_pub.publish(stop)

    def _publish_safety_stop_if_needed(self) -> None:
        """Heartbeat the fail-safe Nav2 lock and publish an immediate zero."""
        with self._state_lock:
            self._manual_motion_active_locked()
            autonomy_authorized = (
                self._goal_reserved
                and self._nav_state == 'running'
                and not self._external_cancel_requested
                and self._mutation_owner is None
                and not self._nav2_task_uncertain
                and not self._invalid_manual_command_deadlines
                and self._navigation_health_block_reason_locked() is None
            )
            lock_active = not autonomy_authorized
            # Keep the decision and publish linearized with state changes.  A
            # timer that computed "unlocked" must not publish after a cancel
            # callback has already published the newer locked state.
            # twist_mux treats true OR timeout as locked.
            self._navigation_coordinator_lock_pub.publish(
                Bool(data=lock_active)
            )
        if lock_active:
            self._publish_safety_zero()

    def mission_operation_lock_srv(self, req, res):
        """Atomically serialize mission mutation against navigation goals."""
        owner = str(req.owner).strip()
        operation = str(req.operation).strip() or 'mission mutation'
        if not owner:
            res.success = False
            res.message = 'Mutation lock owner must not be empty'
            return res

        with self._state_lock:
            if req.acquire:
                if (
                    self._goal_reserved
                    or self._nav2_task_uncertain
                    or self._manual_motion_active_locked()
                ):
                    res.success = False
                    res.message = (
                        'Navigation/manual motion is active or termination is '
                        'uncertain'
                    )
                    return res
                if self._mutation_owner is not None:
                    res.success = False
                    res.message = (
                        'Another mission mutation is active: '
                        f'{self._mutation_operation or "unknown"}'
                    )
                    return res
                self._mutation_owner = owner
                self._mutation_operation = operation
                res.success = True
                res.message = f'Mutation lock acquired for {operation}'
                return res

            if self._mutation_owner != owner:
                res.success = False
                res.message = 'Mutation lock is not owned by this requester'
                return res
            self._mutation_owner = None
            self._mutation_operation = None
            res.success = True
            res.message = 'Mutation lock released'
            return res

    def _request_navigation_cancel(
        self,
        message: str,
        expected_dispatch_id: str | None = None,
    ) -> bool:
        """Atomically match a goal, request cancel, and revoke its velocity."""
        with self._state_lock:
            if not self._goal_reserved:
                return False
            if (
                expected_dispatch_id is not None
                and expected_dispatch_id != self._pending_dispatch_id
            ):
                return False
            self._external_cancel_requested = True
            self._dispatch_confirmation_event.set()
            self._nav_state = 'canceling'
            self._last_status_message = message
        self._update_progress(
            lambda progress: progress.canceling(message) or True
        )
        self._publish_nav_operation_active(True)
        self._publish_safety_zero()
        self._publish_safety_stop_if_needed()
        return True

    def _cancel_callback(self, goal_handle):
        """Accept cancellation while a reentrant action callback is running."""
        dispatch_id = _canonical_dispatch_id(goal_handle.request.dispatch_id)
        if dispatch_id is None:
            return CancelResponse.REJECT
        # Revoke autonomous velocity before returning ACCEPT.  The execute
        # callback performs the bounded Nav2 cancellation, but it must not have
        # a window in which a remotely timed-out caller has asked to cancel and
        # the coordinator still authorizes motion.
        if not self._request_navigation_cancel(
            'Navigation action cancel requested',
            expected_dispatch_id=dispatch_id,
        ):
            return CancelResponse.REJECT
        self.get_logger().warn('Navigation action cancel requested')
        return CancelResponse.ACCEPT

    def _progress_message(self) -> CoverageProgress:
        """Snapshot of the tracker as a message (caller holds _progress_lock)."""
        p = self._progress
        msg = CoverageProgress()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.status = p.status
        msg.status_text = p.status_text
        msg.mission_id = p.mission_id
        msg.zone_id = p.zone_id
        msg.current_segment_index = p.current_segment_index
        msg.total_segments = p.total_segments
        msg.completed_segments = p.completed_segments
        msg.current_segment_progress = float(p.current_segment_progress)
        msg.overall_progress = float(p.overall_progress)
        msg.current_segment_distance_m = float(p.current_segment_distance_m)
        msg.completed_distance_m = float(p.completed_distance_m)
        msg.total_distance_m = float(p.total_distance_m)
        msg.remaining_distance_m = float(p.remaining_distance_m)
        msg.current_pose = self._progress_robot_pose
        if p.current_segment_start is not None:
            msg.current_segment_start = p.current_segment_start
        if p.current_segment_goal is not None:
            msg.current_segment_goal = p.current_segment_goal
        checkpoint = self._checkpoint
        msg.checkpoint_available = bool(
            p.path_hash
            and coverage_progress.checkpoint_resumable(checkpoint)
            and checkpoint['path_hash'] == p.path_hash
            and checkpoint['mission_id'] == p.mission_id
        )
        msg.message = p.message
        return msg

    def _checkpoint_message(self) -> CoverageProgress:
        """The checkpoint on disk as a message (caller holds _progress_lock)."""
        checkpoint = self._checkpoint
        if checkpoint is None:
            msg = CoverageProgress()
            msg.zone_id = -1
            msg.message = 'No coverage checkpoint'
            return msg
        saved = self._progress
        self._progress = coverage_progress.restored_tracker(checkpoint)
        try:
            msg = self._progress_message()
        finally:
            self._progress = saved
        msg.status_text = checkpoint['status']
        if coverage_progress.checkpoint_resumable(checkpoint):
            msg.message = (
                'Resumable at segment '
                f'{coverage_progress.resume_segment_index(checkpoint)}/'
                f'{checkpoint["total_segments"]}'
            )
        else:
            msg.message = 'Checkpoint is not resumable'
        return msg

    def _load_checkpoint(self):
        if self._checkpoint_path is None:
            return None
        try:
            with open(self._checkpoint_path, encoding='utf-8') as f:
                text = f.read()
        except OSError:
            return None
        checkpoint = coverage_progress.checkpoint_from_json(text)
        if checkpoint is None:
            self.get_logger().warn(
                'ignoring unreadable coverage checkpoint '
                f'{self._checkpoint_path}'
            )
            return None
        resumable = coverage_progress.checkpoint_resumable(checkpoint)
        self.get_logger().info(
            f'coverage checkpoint: mission {checkpoint["mission_id"]} '
            f'zone {checkpoint["zone_id"]} {checkpoint["status"]} '
            f'({checkpoint["completed_segments"]}/'
            f'{checkpoint["total_segments"]} segments)'
            + (', resumable' if resumable else '')
        )
        return checkpoint

    def _write_checkpoint(self, checkpoint) -> None:
        """Write via a temporary file and rename (never half a file)."""
        path = self._checkpoint_path
        try:
            os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
            tmp = path + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as f:
                f.write(coverage_progress.checkpoint_to_json(checkpoint))
            os.replace(tmp, path)
        except OSError as exc:
            self.get_logger().warn(
                f'coverage checkpoint {path} not written: {exc}'
            )

    def _split_params(self):
        return tuple(float(self.get_parameter(name).value) for name in (
            'split_tolerance_m',
            'max_follow_segment_length_m',
            'turn_split_angle_rad',
            'turn_split_min_segment_length_m',
        ))

    def _goal_path_hash(self, zone_id, path, split_points) -> str:
        return coverage_progress.path_hash(
            zone_id,
            self._split_params(),
            [
                (p.pose.position.x, p.pose.position.y, p.pose.position.z)
                for p in path.poses
            ],
            [(p.position.x, p.position.y) for p in split_points],
        )

    def _update_progress(self, update, force: bool = True) -> None:
        """Apply ``update(tracker) -> changed`` and publish the result.

        Feedback-driven updates (``force=False``) publish at most every
        PROGRESS_FEEDBACK_PERIOD_S. A checkpoint is written with every forced
        update once the plan is known, and from feedback at most every
        ``progress_checkpoint_interval_sec``.
        """
        checkpoint = None
        with self._progress_lock:
            if not update(self._progress):
                return
            now = monotonic()
            if (
                not force
                and self._progress_last_published is not None
                and now - self._progress_last_published
                < PROGRESS_FEEDBACK_PERIOD_S
            ):
                return
            self._progress_last_published = now
            due = force or self._checkpoint_last_write is None or (
                now - self._checkpoint_last_write
                >= max(self._checkpoint_interval_s, 0.0)
            )
            if (
                self._checkpoint_path is not None
                and self._progress.path_hash
                and due
            ):
                checkpoint = self._progress.checkpoint(time())
                self._checkpoint = checkpoint
                self._checkpoint_last_write = now
            msg = self._progress_message()
        self._coverage_progress_pub.publish(msg)
        if checkpoint is not None:
            self._write_checkpoint(checkpoint)

    def _finish_progress(self, succeeded: bool) -> None:
        """Final /coverage_progress of an execution."""
        with self._state_lock:
            nav_state = self._nav_state
            message = self._last_status_message
        if succeeded:
            status = coverage_progress.STATUS_SUCCEEDED
        elif nav_state == 'canceled':
            status = coverage_progress.STATUS_CANCELED
        else:
            status = coverage_progress.STATUS_FAILED
        self._update_progress(
            lambda progress: progress.finish(status, message) or True
        )

    def coverage_progress_status_srv(self, req, res):
        with self._progress_lock:
            res.progress = self._progress_message()
            res.checkpoint = self._checkpoint_message()
        res.success = True
        res.message = (
            res.progress.status_text
            if res.progress.mission_id
            else 'No coverage execution yet'
        )
        return res

    def _run_reserved_goal(self, goal_handle, callback):
        """Run an accepted goal and always release the shared busy guard."""
        with self._state_lock:
            mission_id = self._pending_dispatch_id or ''
        zone_id = getattr(
            getattr(goal_handle, 'request', None), 'zone_id', -1
        )
        self._update_progress(
            lambda progress: progress.begin(mission_id, zone_id) or True
        )
        succeeded = False
        try:
            result = callback(goal_handle)
            succeeded = bool(getattr(result, 'success', False))
            return result
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f'Unhandled navigation action error: {exc!r}'
            )
            try:
                goal_handle.abort()
            except Exception:  # noqa: BLE001
                pass
            with self._state_lock:
                dispatch_generation = self._nav2_active_generation
                dispatch_unconfirmed = (
                    dispatch_generation is not None
                    and self._nav2_dispatch_in_flight_or_active
                )
            terminal, _ = self._navigator_terminal_snapshot()
            with self._state_lock:
                nav_goal_exists = self._nav2_current_goal_handle is not None
            if (dispatch_unconfirmed or nav_goal_exists) and not terminal:
                latched = self._mark_nav2_task_uncertain(
                    f'Navigation action crashed: {exc}',
                    expected_generation=dispatch_generation,
                )
                if not latched:
                    self._set_nav_state(
                        'failed',
                        f'Navigation failed after terminal dispatch: {exc}',
                    )
            else:
                self._set_nav_state('failed', f'Navigation failed: {exc}')
            result = Waypoint.Result()
            result.success = False
            return result
        finally:
            self._finish_progress(succeeded)
            with self._state_lock:
                finished_dispatch_id = self._pending_dispatch_id
                self._goal_reserved = False
                self._active_goal_handle = None
                self._active_task_name = None
                self._external_cancel_requested = False
                self._dispatch_confirmed = False
                self._pending_dispatch_id = None
                self._dispatch_confirmation_event.clear()
                uncertain = self._nav2_task_uncertain
                if finished_dispatch_id is not None:
                    if uncertain:
                        self._uncertain_dispatch_id = finished_dispatch_id
                    else:
                        self._terminal_dispatch_ids.add(finished_dispatch_id)
            self._publish_nav_operation_active(uncertain)

    def _guarded_execute_callback(self, goal_handle):
        # Keep the legacy ``nav_action`` name, but route it through the same
        # bounded, terminal-only FollowPath implementation. The historical
        # planner/smoother loop used synchronous BasicNavigator calls and could
        # wait forever after a lost action response.
        return self._run_reserved_goal(
            goal_handle,
            self.single_path_execute_callback,
        )

    def _guarded_single_path_execute_callback(self, goal_handle):
        return self._run_reserved_goal(
            goal_handle,
            self.single_path_execute_callback,
        )

    def _finish_goal_canceled(self, goal_handle):
        """Terminate the reserved action goal after a cancel.

        rcl only allows the CANCELED transition from CANCELING, i.e. after an
        accepted action-level cancel request. A cancel that arrived through
        ``/cancel_nav2`` or a manual command leaves the goal EXECUTING, where
        ABORTED is the only terminal transition; the navigation state still
        reports ``canceled`` so callers do not see a spurious failure.
        """
        try:
            goal_handle.canceled()
        except Exception:  # noqa: BLE001 - rclpy raises a pybind RCLError
            goal_handle.abort()

    def _cancel_before_nav_task(self, goal_handle):
        """Finish an accepted action canceled before a Nav2 task starts."""
        with self._state_lock:
            cancel_requested = self._external_cancel_requested
        if not goal_handle.is_cancel_requested and not cancel_requested:
            return None

        self._finish_goal_canceled(goal_handle)
        self._set_nav_state('canceled', 'Navigation canceled before task start')
        result = Waypoint.Result()
        result.success = False
        return result

    def _wait_for_dispatch_confirmation(self, goal_handle):
        """Keep Nav2 locked until the action caller confirms goal receipt."""
        timeout_s = max(
            0.1,
            float(
                self.get_parameter(
                    'dispatch_confirmation_timeout_s'
                ).value
            ),
        )
        deadline = monotonic() + timeout_s
        while monotonic() < deadline:
            remaining = max(0.0, deadline - monotonic())
            self._dispatch_confirmation_event.wait(
                timeout=min(0.05, remaining)
            )
            with self._state_lock:
                confirmed = self._dispatch_confirmed
                canceled = self._external_cancel_requested
            if canceled or goal_handle.is_cancel_requested:
                return self._cancel_before_nav_task(goal_handle)
            if confirmed:
                self._set_nav_state(
                    'starting',
                    'Navigation dispatch confirmed; preparing Nav2 task',
                )
                return None

        return self._abort_goal(
            goal_handle,
            'Navigation dispatch confirmation timed out; Nav2 was not started',
        )

    def _wait_for_nav2_ready(self, goal_handle) -> tuple[bool, str]:
        """Wait boundedly for bt_navigator while honoring cancellation."""
        timeout_s = max(
            0.1,
            float(self.get_parameter('nav2_ready_timeout_s').value),
        )
        poll_s = max(
            0.05,
            float(self.get_parameter('nav2_ready_poll_s').value),
        )
        deadline = monotonic() + timeout_s
        last_state = 'unavailable'

        while monotonic() < deadline:
            with self._state_lock:
                external_cancel = self._external_cancel_requested
            if goal_handle.is_cancel_requested or external_cancel:
                return False, 'canceled'

            remaining = deadline - monotonic()
            if not self._nav2_state_client.wait_for_service(
                timeout_sec=min(poll_s, remaining)
            ):
                continue

            future = self._nav2_state_client.call_async(GetState.Request())
            while not future.done() and monotonic() < deadline:
                with self._state_lock:
                    external_cancel = self._external_cancel_requested
                if goal_handle.is_cancel_requested or external_cancel:
                    return False, 'canceled'
                sleep(min(poll_s, max(0.0, deadline - monotonic())))

            if not future.done():
                break

            try:
                response = future.result()
                last_state = response.current_state.label or 'unknown'
            except Exception as exc:  # noqa: BLE001
                last_state = f'error: {exc}'
            if last_state.lower() == 'active':
                return True, 'Nav2 is active'

            sleep(min(poll_s, max(0.0, deadline - monotonic())))

        return (
            False,
            f'Nav2 readiness timed out after {timeout_s:.1f}s '
            f'(last state: {last_state})',
        )

    def _cancel_nav_task_and_wait(self) -> tuple[bool, TaskResult | None]:
        """Cancel the current BasicNavigator goal and await its terminal result."""
        timeout_s = max(
            0.1,
            float(self.get_parameter('nav2_cancel_timeout_s').value),
        )
        poll_s = max(
            0.01,
            float(self.get_parameter('nav2_cancel_poll_s').value),
        )
        deadline = monotonic() + timeout_s

        # BasicNavigator.cancelTask() waits for the cancel response without a
        # timeout. Use its current public goal handle directly so both the
        # cancel acknowledgement and the terminal result are bounded here.
        with self._state_lock:
            dispatch_unconfirmed = self._nav2_dispatch_in_flight_or_active
            dispatch_generation = self._nav2_active_generation
            current_goal_handle = self._nav2_current_goal_handle
            current_result_future = self._nav2_current_result_future
        # Never mistake the previous task's completed future for proof that a
        # just-dispatched goal with a lost response is terminal.
        if (
            dispatch_generation is None
            or (dispatch_unconfirmed and current_goal_handle is None)
        ):
            return False, None
        terminal, result, unconfirmed = self._task_result_from_future(
            current_result_future
        )
        if terminal:
            cleared = self._mark_nav2_dispatch_terminal(
                expected_generation=dispatch_generation,
            )
            return (True, result) if cleared else (False, None)
        if current_goal_handle is None:
            return False, None
        try:
            cancel_future = current_goal_handle.cancel_goal_async()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Nav2 cancel dispatch failed: {exc}')
            return False, None

        if current_result_future is None or unconfirmed:
            current_result_future = self._recover_nav2_result_future(
                current_goal_handle,
                expected_generation=dispatch_generation,
                deadline=deadline,
                poll_s=poll_s,
            )
            if current_result_future is None:
                # Cancellation was still sent, but without a correlated result
                # channel its terminal effect cannot be proven.
                return False, None

        while not cancel_future.done() and monotonic() < deadline:
            sleep(min(poll_s, max(0.0, deadline - monotonic())))

        if not cancel_future.done():
            return False, None
        try:
            cancel_response = cancel_future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Nav2 cancel request failed: {exc}')
            return False, None
        if cancel_response is None:
            return False, None
        if not getattr(cancel_response, 'goals_canceling', ()):
            terminal, result, unconfirmed = self._task_result_from_future(
                current_result_future
            )
            if unconfirmed or not terminal:
                return False, None
            cleared = self._mark_nav2_dispatch_terminal(
                expected_generation=dispatch_generation,
            )
            return (True, result) if cleared else (False, None)

        while monotonic() < deadline:
            terminal, result, unconfirmed = self._task_result_from_future(
                current_result_future
            )
            if unconfirmed:
                return False, None
            if terminal:
                cleared = self._mark_nav2_dispatch_terminal(
                    expected_generation=dispatch_generation,
                )
                return (True, result) if cleared else (False, None)
            sleep(min(poll_s, max(0.0, deadline - monotonic())))

        return False, None

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
            with self._navigator_lock:
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
        with self._state_lock:
            self._nav_state = state
            self._last_status_message = message
            uncertain = self._nav2_task_uncertain
            reserved = self._goal_reserved
        self._publish_nav_operation_active(reserved or uncertain)
        if state in ('canceling', 'uncertain') or uncertain:
            self._publish_safety_zero()
        self._publish_safety_stop_if_needed()

    def _publish_nav_operation_active(self, active: bool) -> None:
        """Publish the fail-safe cross-node navigation mutation guard."""
        self._nav_operation_active_pub.publish(Bool(data=bool(active)))

    def _navigator_terminal_snapshot(self) -> tuple[bool, TaskResult | None]:
        """Return a non-blocking snapshot of the correlated Nav2 result."""
        try:
            with self._state_lock:
                dispatch_generation = self._nav2_active_generation
                dispatch_unconfirmed = (
                    self._nav2_dispatch_in_flight_or_active
                )
                current_goal_handle = self._nav2_current_goal_handle
                current_result_future = self._nav2_current_result_future
            if (
                dispatch_generation is None
                or (dispatch_unconfirmed and current_goal_handle is None)
            ):
                return False, None
            terminal, result, unconfirmed = self._task_result_from_future(
                current_result_future
            )
            if unconfirmed or not terminal:
                return False, None
            if not self._mark_nav2_dispatch_terminal(
                expected_generation=dispatch_generation,
            ):
                return False, None
            return True, result
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f'Unable to verify Nav2 terminal state: {exc}'
            )
            return False, None

    def _mark_nav2_task_uncertain(
        self,
        message: str,
        *,
        expected_generation=None,
    ) -> bool:
        """Latch a motion fault until Nav2 termination is actually observed."""
        with self._state_lock:
            if (
                expected_generation is not None
                and self._nav2_active_generation != expected_generation
            ):
                return False
            self._nav2_task_uncertain = True
            # Update the fault flag and its public state atomically. A matching
            # late rejection can then either win before this CAS or clear it
            # afterward; it cannot interleave between the two assignments.
            self._nav_state = 'uncertain'
            self._last_status_message = (
                f'{message}; Nav2 task termination remains unconfirmed'
            )
        self._publish_nav_operation_active(True)
        self._publish_safety_zero()
        self._publish_safety_stop_if_needed()
        return True

    @staticmethod
    def _task_result_from_future(result_future):
        """Return (terminal_proven, result, terminal_state_unreadable)."""
        if result_future is None or not result_future.done():
            return False, None, False
        try:
            response = result_future.result()
            status = response.status
        except Exception:  # noqa: BLE001
            # A broken result RPC does not prove the physical goal stopped.
            return False, None, True
        if status == GoalStatus.STATUS_SUCCEEDED:
            return True, TaskResult.SUCCEEDED, False
        if status == GoalStatus.STATUS_CANCELED:
            return True, TaskResult.CANCELED, False
        if status == GoalStatus.STATUS_ABORTED:
            return True, TaskResult.FAILED, False
        # UNKNOWN and non-terminal action statuses are not terminal evidence,
        # even if the local future happens to be marked done.
        return False, None, True

    def _recover_nav2_result_future(
        self,
        goal_handle,
        *,
        expected_generation,
        deadline: float,
        poll_s: float,
    ):
        """Boundedly rebuild a lost result channel for the same Nav2 goal."""
        last_error = None
        while monotonic() < deadline:
            with self._state_lock:
                if (
                    self._nav2_active_generation != expected_generation
                    or self._nav2_current_goal_handle is not goal_handle
                ):
                    return None
            try:
                result_future = goal_handle.get_result_async()
            except Exception as exc:  # noqa: BLE001
                last_error = exc
            else:
                if result_future is not None:
                    _, _, unreadable = self._task_result_from_future(
                        result_future
                    )
                    if unreadable:
                        last_error = RuntimeError(
                            'rebuilt result future ended without terminal proof'
                        )
                        sleep(min(
                            poll_s,
                            max(0.0, deadline - monotonic()),
                        ))
                        continue
                    with self._state_lock:
                        if (
                            self._nav2_active_generation
                            != expected_generation
                            or self._nav2_current_goal_handle is not goal_handle
                        ):
                            return None
                        self._nav2_current_result_future = result_future
                        # Keep the BasicNavigator compatibility mirror atomic
                        # with the generation check so stale A cannot pollute B.
                        with self._navigator_lock:
                            self.navigator.goal_handle = goal_handle
                            self.navigator.result_future = result_future
                            self.navigator.feedback = None
                    return result_future
                last_error = RuntimeError(
                    'get_result_async returned no future'
                )
            sleep(min(poll_s, max(0.0, deadline - monotonic())))

        if last_error is not None:
            self.get_logger().error(
                'Unable to rebuild the correlated Nav2 result channel: '
                f'{last_error}'
            )
        return None

    def _nav2_feedback_callback(self, feedback_message) -> None:
        with self._navigator_lock:
            self.navigator.feedback = feedback_message.feedback

    def _resolve_definitive_nav2_rejection(self, generation) -> None:
        """Atomically resolve only the matching late rejected dispatch."""
        with self._state_lock:
            if self._nav2_active_generation != generation:
                return
            was_uncertain = self._nav2_task_uncertain
            self._nav2_dispatch_in_flight_or_active = False
            self._nav2_active_generation = None
            self._nav2_current_goal_handle = None
            self._nav2_current_result_future = None
            if was_uncertain:
                self._nav2_task_uncertain = False
                self._external_cancel_requested = False
                if self._uncertain_dispatch_id is not None:
                    self._terminal_dispatch_ids.add(
                        self._uncertain_dispatch_id
                    )
                    self._uncertain_dispatch_id = None
                self._nav_state = 'idle'
                self._last_status_message = (
                    'Late Nav2 goal response definitively rejected; ready'
                )
            reserved = self._goal_reserved
        if was_uncertain:
            self._publish_nav_operation_active(reserved)
            self._publish_safety_stop_if_needed()

    def _send_nav2_goal_bounded(self, action_client, goal_message) -> bool:
        """Bound server/acceptance waits and cancel every late acceptance."""
        server_timeout_s = max(
            0.1,
            float(self.get_parameter('nav2_action_server_timeout_s').value),
        )
        response_timeout_s = max(
            0.1,
            float(self.get_parameter('nav2_goal_response_timeout_s').value),
        )
        if not action_client.wait_for_server(timeout_sec=server_timeout_s):
            return False
        with self._state_lock:
            dispatch_generation = self._nav2_active_generation
        if dispatch_generation is None:
            raise RuntimeError('Nav2 dispatch has no active generation')

        send_future = action_client.send_goal_async(
            goal_message,
            feedback_callback=self._nav2_feedback_callback,
        )
        timed_out = threading.Event()
        late_adopt_lock = threading.Lock()
        late_adopt_started = False

        def _adopt_and_cancel_late_goal(done_future):
            nonlocal late_adopt_started
            if not timed_out.is_set():
                return
            with late_adopt_lock:
                if late_adopt_started:
                    return
                late_adopt_started = True
            try:
                late_handle = done_future.result()
                if late_handle is None or not late_handle.accepted:
                    self._resolve_definitive_nav2_rejection(
                        dispatch_generation
                    )
                    return
                # Retain the accepted handle before any operation that may
                # throw; even without a result future it remains cancelable.
                with self._state_lock:
                    generation_matches = (
                        self._nav2_active_generation == dispatch_generation
                    )
                    if generation_matches:
                        self._nav2_current_goal_handle = late_handle
                        self._external_cancel_requested = True
                        # Do not let a stale late-A callback overwrite the
                        # BasicNavigator mirror for a newer generation B.
                        with self._navigator_lock:
                            self.navigator.goal_handle = late_handle
                            self.navigator.feedback = None
                try:
                    late_handle.cancel_goal_async()
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error(
                        f'Late Nav2 cancel dispatch failed: {exc}'
                    )
                if not generation_matches:
                    return
                try:
                    late_result_future = late_handle.get_result_async()
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error(
                        'Late Nav2 result channel is unavailable; bounded '
                        f'cancel recovery will retry it: {exc}'
                    )
                    return
                with self._state_lock:
                    if (
                        self._nav2_active_generation == dispatch_generation
                        and self._nav2_current_goal_handle is late_handle
                    ):
                        self._nav2_current_result_future = late_result_future
                        with self._navigator_lock:
                            self.navigator.result_future = late_result_future
                self.get_logger().error(
                    'Nav2 accepted a goal after the bounded response deadline; '
                    'correlated cancellation is being tracked'
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f'Late Nav2 goal response could not be canceled: {exc}'
                )

        send_future.add_done_callback(_adopt_and_cancel_late_goal)
        deadline = monotonic() + response_timeout_s
        while not send_future.done() and monotonic() < deadline:
            sleep(min(0.02, max(0.0, deadline - monotonic())))
        if not send_future.done():
            timed_out.set()
            # Resolve completion racing the timeout flag. add_done_callback()
            # has already registered the only code allowed to adopt this goal.
            if send_future.done():
                _adopt_and_cancel_late_goal(send_future)
            raise RuntimeError(
                'Nav2 goal response timed out; acceptance is uncertain'
            )

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False
        # As above, retain the accepted handle first. get_result_async() can
        # itself fail after Nav2 has already begun executing the goal.
        with self._state_lock:
            if self._nav2_active_generation != dispatch_generation:
                generation_matches = False
            else:
                generation_matches = True
                self._nav2_current_goal_handle = goal_handle
                with self._navigator_lock:
                    self.navigator.goal_handle = goal_handle
                    self.navigator.feedback = None
                    self.navigator.status = None
        if not generation_matches:
            goal_handle.cancel_goal_async()
            raise RuntimeError('Nav2 dispatch generation changed after accept')
        try:
            result_future = goal_handle.get_result_async()
        except Exception:
            with self._state_lock:
                if (
                    self._nav2_active_generation == dispatch_generation
                    and self._nav2_current_goal_handle is goal_handle
                ):
                    self._external_cancel_requested = True
            try:
                goal_handle.cancel_goal_async()
            except Exception as cancel_exc:  # noqa: BLE001
                self.get_logger().error(
                    f'Nav2 cancel dispatch after result-channel loss failed: '
                    f'{cancel_exc}'
                )
            raise
        with self._state_lock:
            result_generation_matches = (
                self._nav2_active_generation == dispatch_generation
                and self._nav2_current_goal_handle is goal_handle
            )
            if result_generation_matches:
                self._nav2_current_result_future = result_future
                with self._navigator_lock:
                    self.navigator.result_future = result_future
        if not result_generation_matches:
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f'Stale Nav2 goal cancellation failed: {exc}'
                )
            raise RuntimeError(
                'Nav2 dispatch generation changed before result correlation'
            )
        return True

    def _start_nav2_dispatch(self, command) -> bool:
        """Dispatch while latching uncertainty before crossing into Nav2."""
        with self._navigator_lock:
            baseline_goal_handle = getattr(
                self.navigator,
                'goal_handle',
                None,
            )
            baseline_result_future = getattr(
                self.navigator,
                'result_future',
                None,
            )
            # BasicNavigator does not clear feedback between goals. Do not let
            # stale data from the previous segment leak into this dispatch.
            self.navigator.feedback = None
        with self._state_lock:
            self._nav2_dispatch_generation += 1
            dispatch_generation = self._nav2_dispatch_generation
            self._nav2_active_generation = dispatch_generation
            self._nav2_dispatch_in_flight_or_active = True
            self._nav2_current_goal_handle = None
            self._nav2_current_result_future = None
        accepted = command()
        if accepted is False:
            self._resolve_definitive_nav2_rejection(dispatch_generation)
            return False
        if accepted is not True:
            raise RuntimeError(
                'Nav2 dispatch returned no explicit acceptance result'
            )
        with self._navigator_lock:
            current_goal_handle = getattr(
                self.navigator,
                'goal_handle',
                None,
            )
            current_result_future = getattr(
                self.navigator,
                'result_future',
                None,
            )
            self.navigator.feedback = None
        if (
            current_goal_handle is None
            or current_goal_handle is baseline_goal_handle
            or current_result_future is None
            or current_result_future is baseline_result_future
        ):
            raise RuntimeError(
                'Nav2 accepted a dispatch without a new correlated goal '
                'handle/result future'
            )
        with self._state_lock:
            if self._nav2_active_generation != dispatch_generation:
                raise RuntimeError(
                    'Nav2 dispatch generation changed before correlation'
                )
            self._nav2_current_goal_handle = current_goal_handle
            self._nav2_current_result_future = current_result_future
        return True

    def _mark_nav2_dispatch_terminal(
        self,
        *,
        expected_generation,
    ) -> bool:
        """Clear the dispatch latch only after a proven terminal result."""
        with self._state_lock:
            if (
                expected_generation is None
                or self._nav2_active_generation != expected_generation
            ):
                return False
            self._nav2_dispatch_in_flight_or_active = False
            self._nav2_active_generation = None
            self._nav2_current_goal_handle = None
            self._nav2_current_result_future = None
        return True

    def _clear_uncertain_nav2_task(
        self,
        result: TaskResult | None,
        *,
        source: str,
    ) -> None:
        with self._state_lock:
            if not self._nav2_task_uncertain:
                return
            self._nav2_task_uncertain = False
            self._external_cancel_requested = False
            # The correlated terminal observer already cleared the Nav2
            # dispatch through _mark_nav2_dispatch_terminal(). Do not repeat
            # that mutation here without its generation token: a stale fault
            # cleanup must never erase a newer dispatch.
            if self._uncertain_dispatch_id is not None:
                self._terminal_dispatch_ids.add(
                    self._uncertain_dispatch_id
                )
                self._uncertain_dispatch_id = None
        self._set_nav_state(
            'idle',
            f'Previous Nav2 fault is terminal ({source}, result={result}); ready',
        )
        self.get_logger().warn(self._last_status_message)

    def _monitor_uncertain_nav2_task(self) -> None:
        """Automatically release the fault only after a terminal observation."""
        with self._state_lock:
            uncertain = self._nav2_task_uncertain
            has_correlated_handle = self._nav2_current_goal_handle is not None
            retry_cancel = uncertain and (
                self._external_cancel_requested or has_correlated_handle
            )
            orphan_dispatch = (
                uncertain
                and self._nav2_dispatch_in_flight_or_active
            )
            if retry_cancel:
                self._external_cancel_requested = False
        if not uncertain:
            return
        if retry_cancel:
            confirmed, result = self._cancel_nav_task_and_wait()
            if confirmed:
                self._clear_uncertain_nav2_task(
                    result,
                    source='correlated safety cancel retry',
                )
            else:
                self._mark_nav2_task_uncertain(
                    'Safety cancel could not confirm Nav2 terminal state'
                )
            return
        if orphan_dispatch:
            with self._state_lock:
                if self._nav2_current_goal_handle is None:
                    # There is no correlated handle with which to prove that
                    # the possibly accepted Nav2 goal stopped. Keep the mux
                    # locked until the full stack is restarted.
                    return
        terminal, result = self._navigator_terminal_snapshot()
        if terminal:
            self._clear_uncertain_nav2_task(result, source='monitor')

    def _status_json(self, message: str | None = None) -> str:
        """Return the stable machine-readable /check_nav_status payload."""
        with self._state_lock:
            block_reason = self._navigation_admission_block_reason_locked()
            payload = {
                'state': self._nav_state,
                'task': self._active_task_name or self._last_task_name,
                'message': message or self._last_status_message,
                'ready': block_reason is None,
                'block_reason': block_reason,
            }
        return json.dumps(payload, ensure_ascii=False, separators=(',', ':'))

    def confirm_navigation_dispatch_srv(self, req, res):
        """Authorize the one reserved action after its handle was received."""
        dispatch_id = _canonical_dispatch_id(req.dispatch_id)
        with self._state_lock:
            if (
                not self._goal_reserved
                or self._nav_state != 'pending_confirmation'
                or self._external_cancel_requested
                or dispatch_id is None
                or dispatch_id != self._pending_dispatch_id
            ):
                res.success = False
                res.message = (
                    'No pending navigation dispatch can be confirmed'
                )
                return res
            self._dispatch_confirmed = True
            self._dispatch_confirmation_event.set()
        res.success = True
        res.message = 'Navigation dispatch confirmed'
        return res

    def cancel_nav2_srv(self, req, res):
        """Cancel the Nav2 task owned by this action server."""
        with self._state_lock:
            nav_state = self._nav_state
            nav2_task_uncertain = self._nav2_task_uncertain
            task_name = (
                self._active_task_name
                or self._last_task_name
                or 'Nav2 task'
            )

        if nav2_task_uncertain:
            confirmed, result = self._cancel_nav_task_and_wait()
            if confirmed:
                self._clear_uncertain_nav2_task(
                    result,
                    source='cancel retry',
                )
                res.success = True
                res.message = (
                    f'{task_name} is terminal; navigation fault cleared'
                )
            else:
                message = (
                    f'{task_name} termination is still unconfirmed; '
                    'new navigation remains blocked'
                )
                self._mark_nav2_task_uncertain(message)
                res.success = False
                res.message = message
            return res

        if nav_state not in (
            'pending_confirmation', 'starting', 'running', 'canceling'
        ):
            res.success = True
            res.message = 'No active Nav2 task to cancel'
            return res

        if not self._request_navigation_cancel(
            f'Cancel requested for {task_name}'
        ):
            res.success = True
            res.message = 'No active Nav2 task to cancel'
            return res
        res.success = True
        res.message = f'Cancel requested for {task_name}'
        self.get_logger().warn(res.message)
        return res

    def cancel_navigation_dispatch_srv(self, req, res):
        """Cancel only the action matching a timed-out dispatch token."""
        dispatch_id = _canonical_dispatch_id(req.dispatch_id)
        res.terminal_confirmed = False
        if dispatch_id is None:
            res.success = False
            res.message = 'Invalid navigation dispatch token'
            return res
        with self._state_lock:
            if dispatch_id in self._terminal_dispatch_ids:
                res.success = True
                res.terminal_confirmed = True
                res.message = 'Navigation dispatch is terminal'
                return res
            task_name = (
                self._active_task_name
                or self._last_task_name
                or 'Nav2 task'
            )
            matching_uncertain = (
                self._nav2_task_uncertain
                and dispatch_id == self._uncertain_dispatch_id
            )
        if matching_uncertain:
            # The outer ROS action has already returned, so _goal_reserved and
            # _pending_dispatch_id are intentionally clear. Retry the bounded
            # Nav2 cancellation using the retained token instead of falling
            # through the active-goal matcher and reporting a false no-op.
            confirmed, result = self._cancel_nav_task_and_wait()
            if confirmed:
                self._clear_uncertain_nav2_task(
                    result,
                    source='correlated cancel retry',
                )
                res.success = True
                res.terminal_confirmed = True
                res.message = (
                    f'{task_name} is terminal; correlated fault cleared'
                )
            else:
                message = (
                    f'{task_name} termination is still unconfirmed; '
                    'correlated navigation remains blocked'
                )
                self._mark_nav2_task_uncertain(message)
                res.success = False
                res.message = message
            return res
        if not self._request_navigation_cancel(
            f'Correlated cancel requested for {task_name}',
            expected_dispatch_id=dispatch_id,
        ):
            # The action can finish between the first registry check and the
            # atomic token comparison above. Recheck before reporting unknown.
            with self._state_lock:
                terminal = dispatch_id in self._terminal_dispatch_ids
            res.success = True
            res.terminal_confirmed = terminal
            res.message = (
                'Navigation dispatch is terminal'
                if terminal
                else 'Matching dispatch is not active, but terminal state is '
                'not confirmed'
            )
            return res
        res.success = True
        res.terminal_confirmed = False
        res.message = f'Correlated cancel requested for {task_name}'
        self.get_logger().warn(res.message)
        return res

    def check_nav_status_srv(self, req, res):
        """Return the state maintained by the action execution callback.

        Trigger.success describes the status query, not the navigation result.
        The navigation outcome is always represented by ``message.state``.
        Keeping this callback side-effect free also avoids competing with the
        execution callback for BasicNavigator's action futures.
        """
        with self._state_lock:
            message = self._last_status_message
            if self._nav_state in ('running', 'canceling'):
                message = (
                    f'{self._last_status_message}; '
                    f'{self._last_feedback_message}'
                )
        res.success = True
        res.message = self._status_json(message)
        return res

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
    ) -> bool:
        started_at = monotonic()
        last_feedback = None
        with self._state_lock:
            self._active_goal_handle = goal_handle
            self._active_task_name = task_name
            self._last_task_name = task_name
            canceled_before_running = (
                goal_handle.is_cancel_requested
                or self._external_cancel_requested
                or self._nav_state == 'canceling'
            )
            if not canceled_before_running:
                self._nav_state = 'running'
                self._last_status_message = (
                    f'Navigation running: {task_name}'
                )
            uncertain = self._nav2_task_uncertain
            reserved = self._goal_reserved
        self._publish_nav_operation_active(reserved or uncertain)
        if canceled_before_running:
            self._publish_safety_zero()
        self._publish_safety_stop_if_needed()
        terminal_generation = None
        while True:
            with self._state_lock:
                dispatch_generation = self._nav2_active_generation
                result_future = self._nav2_current_result_future
            is_complete, result, unconfirmed = self._task_result_from_future(
                result_future
            )
            if unconfirmed:
                message = (
                    f'{task_name} result channel ended without a proven '
                    'terminal Nav2 status'
                )
                goal_handle.abort()
                with self._state_lock:
                    self._active_goal_handle = None
                    self._active_task_name = None
                self._mark_nav2_task_uncertain(message)
                return False
            if is_complete:
                terminal_generation = dispatch_generation
                break

            with self._state_lock:
                external_cancel_requested = self._external_cancel_requested
            if goal_handle.is_cancel_requested or external_cancel_requested:
                cancel_source = (
                    'external service'
                    if external_cancel_requested
                    else 'action client'
                )
                self.get_logger().warn(f'{task_name} 已取消 ({cancel_source})')
                cancel_confirmed, _ = self._cancel_nav_task_and_wait()
                if not cancel_confirmed:
                    message = (
                        f'{task_name} cancel confirmation timed out; '
                        'refusing to continue navigation'
                    )
                    self.get_logger().error(message)
                    goal_handle.abort()
                    with self._state_lock:
                        self._active_goal_handle = None
                        self._active_task_name = None
                    self._mark_nav2_task_uncertain(message)
                    return False
                self._finish_goal_canceled(goal_handle)
                with self._state_lock:
                    self._external_cancel_requested = False
                    self._active_goal_handle = None
                    self._active_task_name = None
                self._set_nav_state('canceled', f'{task_name} canceled')
                return False
            with self._navigator_lock:
                feedback = self.navigator.getFeedback()
            if feedback:
                last_feedback = feedback
                # FollowPath's distance_to_goal is the path length left.
                remaining = getattr(feedback, 'distance_to_goal', None)
                self._update_progress(
                    lambda progress: progress.update_remaining(remaining),
                    force=False,
                )
                with self._state_lock:
                    self._last_feedback_message = self._feedback_summary(
                        feedback
                    )
                self.get_logger().info(f'{task_name} 反饋: {feedback}')
            sleep(0.05)

        with self._state_lock:
            cancel_after_completion = self._external_cancel_requested
        if (
            terminal_generation is None
            or not self._mark_nav2_dispatch_terminal(
                expected_generation=terminal_generation,
            )
        ):
            message = (
                f'{task_name} terminal result belonged to a stale Nav2 '
                'dispatch generation; refusing to continue'
            )
            self.get_logger().error(message)
            goal_handle.abort()
            with self._state_lock:
                self._active_goal_handle = None
                self._active_task_name = None
            self._mark_nav2_task_uncertain(message)
            return False
        if goal_handle.is_cancel_requested or cancel_after_completion:
            self._finish_goal_canceled(goal_handle)
            with self._state_lock:
                self._external_cancel_requested = False
                self._active_goal_handle = None
                self._active_task_name = None
            self._set_nav_state('canceled', f'{task_name} canceled')
            return False

        if result == TaskResult.SUCCEEDED:
            # A Waypoint action may contain multiple Nav2 tasks. Only the outer
            # execute callback marks the whole action completed.
            self._set_nav_state(
                'starting',
                f'{task_name} completed; preparing the next task',
            )
            with self._state_lock:
                self._active_goal_handle = None
                self._active_task_name = None
            return True

        if result == TaskResult.CANCELED:
            self._finish_goal_canceled(goal_handle)
            with self._state_lock:
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
        with self._state_lock:
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
        self._set_nav_state('failed', message)
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
        confirmation_result = self._wait_for_dispatch_confirmation(goal_handle)
        if confirmation_result is not None:
            return confirmation_result

        self.get_logger().info('執行目標')
        path = goal_handle.request.path
        coverage_split_points = goal_handle.request.coverage_split_points
        zone_id = int(getattr(goal_handle.request, 'zone_id', -1))
        resume_index = int(
            getattr(goal_handle.request, 'resume_segment_index', 0)
        )

        self.get_logger().info(f'path 長度: {len(path.poses)}')
        self.get_logger().info(
            f'coverage_split_points 長度: {len(coverage_split_points)}'
        )

        if not path.poses:
            return self._abort_goal(goal_handle, '收到空路徑，取消導航')

        canceled_result = self._cancel_before_nav_task(goal_handle)
        if canceled_result is not None:
            return canceled_result

        # Nav2 lifecycle activation is asynchronous. The commander helper waits
        # forever, so use a bounded/cancelable lifecycle-state poll instead.
        self.get_logger().info('等待 Nav2 啟用中…')
        nav2_ready, readiness_message = self._wait_for_nav2_ready(goal_handle)
        if not nav2_ready:
            if readiness_message == 'canceled':
                canceled_result = self._cancel_before_nav_task(goal_handle)
                if canceled_result is not None:
                    return canceled_result
            return self._abort_goal(goal_handle, readiness_message)
        self.get_logger().info(readiness_message)

        canceled_result = self._cancel_before_nav_task(goal_handle)
        if canceled_result is not None:
            return canceled_result

        self.coverage_split_points = list(coverage_split_points)
        self.publish_split_points_marker()

        try:
            split_tolerance = float(
                self.get_parameter('split_tolerance_m').value
            )
            max_segment_length = float(
                self.get_parameter('max_follow_segment_length_m').value
            )
            turn_split_angle = float(
                self.get_parameter('turn_split_angle_rad').value
            )
            turn_split_min_length = float(self.get_parameter(
                'turn_split_min_segment_length_m'
            ).value)
        except (TypeError, ValueError, OverflowError):
            return self._abort_goal(
                goal_handle,
                'coverage execution parameters must be finite numbers',
            )
        coverage_parameters = {
            'split_tolerance_m': split_tolerance,
            'max_follow_segment_length_m': max_segment_length,
            'turn_split_angle_rad': turn_split_angle,
            'turn_split_min_segment_length_m': turn_split_min_length,
        }
        for name, value in coverage_parameters.items():
            if not math.isfinite(value) or value < 0.0:
                return self._abort_goal(
                    goal_handle,
                    f'{name} must be finite and non-negative',
                )

        # Validate every derived segment before the mower moves to the path
        # start. Previously, many sub-5 cm splits were silently skipped and
        # the outer action could report a completely untraversed path as done.
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
        segment_block_reason = _coverage_segments_block_reason(split_paths)
        if segment_block_reason is not None:
            return self._abort_goal(goal_handle, segment_block_reason)
        if resume_index > len(split_paths):
            return self._abort_goal(
                goal_handle,
                f'resume segment {resume_index} does not exist '
                f'({len(split_paths)} segments)',
            )
        first = max(resume_index, 1)
        distances = [_path_distance(p) for p in split_paths]
        hash_value = self._goal_path_hash(
            zone_id, path, coverage_split_points
        )

        def start_navigating(progress):
            progress.set_segments(distances)
            progress.path_hash = hash_value
            if resume_index > 0:
                progress.resume_from(resume_index)
                progress.message = (
                    'Resuming: navigating to the start of segment '
                    f'{resume_index}'
                )
            else:
                progress.message = 'Navigating to the coverage start'
            return True

        self._update_progress(start_navigating)

        self._stamp_path_for_execution(path)
        navigate_goal = NavigateToPose.Goal()
        navigate_goal.pose = path.poses[0]
        if first > 1:
            self.get_logger().info(f'從檢查點續割：導航到第 {first} 段起點')
            start = PoseStamped()
            start.header = path.poses[0].header
            start.pose = split_paths[first - 1].poses[0].pose
            navigate_goal.pose = start
        else:
            self.get_logger().info('導航到覆蓋路徑起點')
        nav_started = self._start_nav2_dispatch(
            lambda: self._send_nav2_goal_bounded(
                self._navigate_to_pose_client,
                navigate_goal,
            )
        )
        if not nav_started:
            return self._abort_goal(
                goal_handle,
                'Nav2 rejected navigation to the coverage start',
            )
        if not self._wait_for_nav_task(goal_handle, 'Nav to coverage start'):
            result = Waypoint.Result()
            result.success = False
            return result

        self.get_logger().info(
            f'coverage path 依 split points 切成 {coverage_split_count} 段，'
            f'再依轉角 >= {turn_split_angle:.2f} rad 切成 '
            f'{turn_split_count} 段，'
            f'再依最大 {max_segment_length:.1f} m 切成 '
            f'{len(split_paths)} 段'
        )

        for idx, split_path in enumerate(split_paths, start=1):
            if idx < first:
                continue
            canceled_result = self._cancel_before_nav_task(goal_handle)
            if canceled_result is not None:
                return canceled_result
            self._stamp_path_for_execution(split_path)
            self.split_path_pub.publish(split_path)
            distance = _path_distance(split_path)
            self._update_progress(
                lambda progress, idx=idx, split_path=split_path: (
                    progress.start_segment(
                        idx, split_path.poses[0], split_path.poses[-1]
                    )
                    or True
                )
            )
            start = split_path.poses[0].pose.position
            goal = split_path.poses[-1].pose.position
            self.get_logger().info(
                f'執行第 {idx}/{len(split_paths)} 段 coverage path，'
                f'共 {len(split_path.poses)} 點，長度 {distance:.3f} m，'
                f'起點 ({start.x:.3f}, {start.y:.3f})，'
                f'終點 ({goal.x:.3f}, {goal.y:.3f})'
            )
            follow_goal = FollowPath.Goal()
            follow_goal.path = split_path
            follow_goal.controller_id = self.controller_id
            follow_goal.goal_checker_id = self.goal_checker_id
            nav_started = self._start_nav2_dispatch(
                lambda follow_goal=follow_goal: self._send_nav2_goal_bounded(
                    self._follow_path_client,
                    follow_goal,
                )
            )
            if not nav_started:
                return self._abort_goal(
                    goal_handle,
                    f'Nav2 rejected coverage segment {idx}',
                )
            if not self._wait_for_nav_task(
                goal_handle,
                f'Follow coverage segment {idx}',
            ):
                result = Waypoint.Result()
                result.success = False
                return result
            self._update_progress(
                lambda progress: progress.complete_segment() or True
            )

        canceled_result = self._cancel_before_nav_task(goal_handle)
        if canceled_result is not None:
            return canceled_result
        goal_handle.succeed()
        with self._state_lock:
            self._active_goal_handle = None
            self._active_task_name = None
        self._set_nav_state('completed', 'Coverage navigation completed')
        result = Waypoint.Result()
        result.success = True
        return result

    def execute_callback(self, goal_handle):
        """Route historical direct callers through bounded coverage execution."""
        return self.single_path_execute_callback(goal_handle)

    def destroy_node(self):
        self._health_thread_stop.set()
        if (
            self._health_thread.is_alive()
            and threading.current_thread() is not self._health_thread
        ):
            self._health_thread.join(timeout=1.0)
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
    # At least one thread runs the long action callback while another services
    # status/cancel requests. Do not rely on host CPU-affinity auto-detection.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(nav_action_server)
    try:
        executor.spin()
    finally:
        executor.remove_node(nav_action_server)
        nav_action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
