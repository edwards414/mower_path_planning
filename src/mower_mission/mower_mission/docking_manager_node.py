#!/usr/bin/env python3

import json
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import DockRobot, UndockRobot
from nav2_msgs.srv import ReloadDockDatabase

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import String
from std_srvs.srv import Trigger

from tf2_ros import Buffer, TransformException, TransformListener


DOCK_FEEDBACK_STATES = {
    0: 'none',
    1: 'nav_to_staging_pose',
    2: 'initial_perception',
    3: 'controlling',
    4: 'wait_for_charge',
    5: 'retry',
}


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class DockingManager(Node):
    """Temporary mission-level wrapper for Nav2 docking actions."""

    def __init__(self):
        super().__init__('docking_manager')
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter('default_dock_id', 'home_dock')
        self.declare_parameter('dock_type', 'mower_reverse_dock')
        self.declare_parameter('max_staging_time', 120.0)
        self.declare_parameter('max_undocking_time', 30.0)
        self.declare_parameter('navigate_to_staging_pose', True)
        self.declare_parameter('dock_action_name', '/dock_robot')
        self.declare_parameter('undock_action_name', '/undock_robot')
        self.declare_parameter(
            'reload_database_service_name',
            '/docking_server/reload_database',
        )
        self.declare_parameter('dock_database_path', '')

        self.dock_client = ActionClient(
            self,
            DockRobot,
            self.get_parameter('dock_action_name').value,
            callback_group=self.cb_group,
        )
        self.undock_client = ActionClient(
            self,
            UndockRobot,
            self.get_parameter('undock_action_name').value,
            callback_group=self.cb_group,
        )
        self.reload_database_client = self.create_client(
            ReloadDockDatabase,
            self.get_parameter('reload_database_service_name').value,
            callback_group=self.cb_group,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.status_pub = self.create_publisher(
            String, '/mower_docking_state', 10
        )
        self.create_timer(1.0, self._publish_status)

        self.create_service(
            Trigger,
            '/mower_dock_home',
            self.dock_home_srv,
            callback_group=self.cb_group,
        )
        self.create_service(
            Trigger,
            '/mower_undock',
            self.undock_srv,
            callback_group=self.cb_group,
        )
        self.create_service(
            Trigger,
            '/mower_cancel_docking',
            self.cancel_docking_srv,
            callback_group=self.cb_group,
        )
        self.create_service(
            Trigger,
            '/mower_docking_status',
            self.docking_status_srv,
            callback_group=self.cb_group,
        )
        self.create_service(
            Trigger,
            '/record_home_dock_pose',
            self.record_home_dock_pose_srv,
            callback_group=self.cb_group,
        )

        self.active_goal_handle = None
        self.active_goal_kind = None
        self.status = {
            'state': 'idle',
            'dock_id': self.get_parameter('default_dock_id').value,
            'dock_type': self.get_parameter('dock_type').value,
            'num_retries': 0,
            'error_code': 0,
            'error_msg': '',
            'message': 'docking manager ready',
            'charging_confirmed': False,
        }
        self.get_logger().info('Temporary DockingManager initialized')

    def dock_home_srv(self, req, res):
        if self.active_goal_handle is not None:
            res.success = False
            res.message = 'Docking/undocking goal already active'
            return res

        if not self.dock_client.wait_for_server(timeout_sec=1.0):
            res.success = False
            res.message = '/dock_robot action server unavailable'
            return res

        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id = self.get_parameter('default_dock_id').value
        goal.max_staging_time = float(
            self.get_parameter('max_staging_time').value
        )
        goal.navigate_to_staging_pose = bool(
            self.get_parameter('navigate_to_staging_pose').value
        )

        self.status.update({
            'state': 'goal_requested',
            'dock_id': goal.dock_id,
            'num_retries': 0,
            'error_code': 0,
            'error_msg': '',
            'message': 'dock goal requested',
            'charging_confirmed': False,
        })
        self._publish_status()

        future = self.dock_client.send_goal_async(
            goal,
            feedback_callback=self._dock_feedback_callback,
        )
        future.add_done_callback(self._dock_goal_response_callback)

        res.success = True
        res.message = f'Dock goal sent for dock_id={goal.dock_id}'
        return res

    def undock_srv(self, req, res):
        if self.active_goal_handle is not None:
            res.success = False
            res.message = 'Docking/undocking goal already active'
            return res

        if not self.undock_client.wait_for_server(timeout_sec=1.0):
            res.success = False
            res.message = '/undock_robot action server unavailable'
            return res

        goal = UndockRobot.Goal()
        goal.dock_type = self.get_parameter('dock_type').value
        goal.max_undocking_time = float(
            self.get_parameter('max_undocking_time').value
        )

        self.status.update({
            'state': 'undocking',
            'dock_type': goal.dock_type,
            'error_code': 0,
            'error_msg': '',
            'message': 'undock goal requested',
        })
        self._publish_status()

        future = self.undock_client.send_goal_async(goal)
        future.add_done_callback(self._undock_goal_response_callback)

        res.success = True
        res.message = f'Undock goal sent for dock_type={goal.dock_type}'
        return res

    def cancel_docking_srv(self, req, res):
        if self.active_goal_handle is None:
            res.success = True
            res.message = 'No active docking goal'
            return res

        self.status.update({
            'state': 'canceling',
            'message': f'canceling {self.active_goal_kind} goal',
        })
        self._publish_status()
        future = self.active_goal_handle.cancel_goal_async()
        future.add_done_callback(self._cancel_done_callback)
        res.success = True
        res.message = f'Cancel requested for {self.active_goal_kind} goal'
        return res

    def docking_status_srv(self, req, res):
        res.success = True
        res.message = json.dumps(self.status, ensure_ascii=False)
        return res

    def record_home_dock_pose_srv(self, req, res):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                Time(),
                timeout=Duration(seconds=1.0),
            )
        except TransformException as exc:
            res.success = False
            res.message = f'Cannot record dock pose: {exc}'
            return res

        trans = transform.transform.translation
        rot = transform.transform.rotation
        yaw = _yaw_from_quaternion(rot)
        path = self._dock_database_path()
        self._write_single_dock_database(path, trans.x, trans.y, yaw)

        if not self.reload_database_client.wait_for_service(timeout_sec=1.0):
            res.success = True
            res.message = (
                f'Recorded home_dock to {path}, but reload service unavailable'
            )
            return res

        request = ReloadDockDatabase.Request()
        request.filepath = str(path)
        future = self.reload_database_client.call_async(request)
        future.add_done_callback(self._reload_database_done_callback)

        res.success = True
        res.message = (
            'Recorded home_dock final pose '
            f'x={trans.x:.3f}, y={trans.y:.3f}, yaw={yaw:.3f}; '
            f'reloading {path}'
        )
        return res

    def _dock_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.active_goal_handle = None
            self.active_goal_kind = None
            self.status.update({
                'state': 'rejected',
                'message': 'dock goal rejected',
            })
            self._publish_status()
            return

        self.active_goal_handle = goal_handle
        self.active_goal_kind = 'dock'
        self.status.update({
            'state': 'accepted',
            'message': 'dock goal accepted',
        })
        self._publish_status()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._dock_result_callback)

    def _undock_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.active_goal_handle = None
            self.active_goal_kind = None
            self.status.update({
                'state': 'rejected',
                'message': 'undock goal rejected',
            })
            self._publish_status()
            return

        self.active_goal_handle = goal_handle
        self.active_goal_kind = 'undock'
        self.status.update({
            'state': 'undocking',
            'message': 'undock goal accepted',
        })
        self._publish_status()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._undock_result_callback)

    def _dock_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        state = DOCK_FEEDBACK_STATES.get(
            int(feedback.state),
            f'unknown_{feedback.state}',
        )
        self.status.update({
            'state': state,
            'num_retries': int(feedback.num_retries),
            'message': f'docking feedback: {state}',
        })
        self._publish_status()

    def _dock_result_callback(self, future):
        result = future.result().result
        success = bool(result.success)
        self.status.update({
            'state': 'docked' if success else 'failed',
            'num_retries': int(result.num_retries),
            'error_code': int(result.error_code),
            'error_msg': getattr(result, 'error_msg', ''),
            'message': 'dock complete' if success else 'dock failed',
            'charging_confirmed': False,
        })
        self.active_goal_handle = None
        self.active_goal_kind = None
        self._publish_status()

    def _undock_result_callback(self, future):
        result = future.result().result
        success = bool(result.success)
        self.status.update({
            'state': 'idle_ready' if success else 'failed',
            'error_code': int(result.error_code),
            'error_msg': getattr(result, 'error_msg', ''),
            'message': 'undock complete' if success else 'undock failed',
        })
        self.active_goal_handle = None
        self.active_goal_kind = None
        self._publish_status()

    def _cancel_done_callback(self, future):
        self.status.update({
            'state': 'canceled',
            'message': 'cancel request completed',
        })
        self.active_goal_handle = None
        self.active_goal_kind = None
        self._publish_status()

    def _reload_database_done_callback(self, future):
        try:
            response = future.result()
            success = bool(response.success)
        except Exception as exc:
            self.get_logger().error(f'Dock database reload failed: {exc}')
            return
        if success:
            self.get_logger().info('Dock database reload requested successfully')
        else:
            self.get_logger().error('Dock database reload returned success=false')

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps(self.status, ensure_ascii=False)
        self.status_pub.publish(msg)

    def _dock_database_path(self) -> Path:
        configured = str(self.get_parameter('dock_database_path').value)
        if configured:
            path = Path(configured)
        else:
            bringup_share = Path(get_package_share_directory('mower_bringup'))
            path = bringup_share / 'config' / 'dock_database.yaml'
        path.parent.mkdir(parents=True, exist_ok=True)
        return path

    def _write_single_dock_database(self, path: Path, x: float, y: float, yaw: float):
        dock_type = self.get_parameter('dock_type').value
        dock_id = self.get_parameter('default_dock_id').value
        content = (
            'docks:\n'
            f'  {dock_id}:\n'
            f'    type: "{dock_type}"\n'
            '    frame: "map"\n'
            f'    pose: [{x:.6f}, {y:.6f}, {yaw:.6f}]\n'
            f'    id: "{dock_id}"\n'
        )
        path.write_text(content, encoding='utf-8')


def main(args=None):
    rclpy.init(args=args)
    node = DockingManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
