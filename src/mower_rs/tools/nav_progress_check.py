#!/usr/bin/env python3
"""Behavioural check of the navigation server's coverage progress.

Runs a navigation server (mower_nav, or the rclpy nav_action_server) against
a fake Nav2 (bt_navigator/get_state, navigate_to_pose, follow_path with a
falling distance_to_goal) and drives one Waypoint goal per scenario through
nav_action_follow_path, recording every /coverage_progress message and the
final /coverage_progress_status answer.

Scenarios (a straight 6 m path with split points at 2 m and 4 m, i.e. three
2 m FollowPath segments):
  A  zone 7 runs to the end: the mission id is the dispatch id, the status
     goes navigating_to_start -> running -> succeeded, overall progress never
     falls, feedback moves it inside a segment, and the last message says
     3/3 segments, 6 m, 100 %.
  B  no zone id, the action is canceled halfway through segment 2:
     canceling is published, then canceled with 1 completed segment and the
     overall progress between 1/3 and 2/3 (kept, not reset).

Needs a sourced ROS 2 Jazzy environment with mower_interface and nav2_msgs:

  python3 src/mower_rs/tools/nav_progress_check.py -- <path to mower_nav>
  python3 src/mower_rs/tools/nav_progress_check.py -- \\
      python3 -m mower_mission.navigation.nav_action_server

Exits 1 on the first failed expectation.
"""

import argparse
import math
import subprocess
import sys
import threading
import time
import uuid

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Pose, PoseStamped
from lifecycle_msgs.srv import GetState
from mower_interface.action import Waypoint
from mower_interface.msg import CoverageProgress
from mower_interface.srv import ConfirmNavigationDispatch, GetCoverageProgress
from nav2_msgs.action import FollowPath, NavigateToPose
from nav_msgs.msg import Path

SEGMENT_TIME_S = 1.5  # per FollowPath goal; > 2 feedback publish periods


def straight_path(length_m=6.0, step_m=0.25):
    # poses 0.25 m apart: exactly one lies within the 0.1 m split tolerance
    path = Path()
    path.header.frame_id = 'map'
    for i in range(int(round(length_m / step_m)) + 1):
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.pose.position.x = i * step_m
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)
    return path


def split_point(x):
    p = Pose()
    p.position.x = x
    p.orientation.w = 1.0
    return p


def path_length(path):
    pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


class FakeNav2(Node):
    def __init__(self):
        super().__init__('fake_nav2')
        group = ReentrantCallbackGroup()
        self.follow_count = 0
        self.hold_segment = None  # FollowPath goal that stops halfway
        self.create_service(GetState, 'bt_navigator/get_state', self._state, callback_group=group)
        ActionServer(
            self, NavigateToPose, 'navigate_to_pose', self._navigate,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group,
        )
        ActionServer(
            self, FollowPath, 'follow_path', self._follow,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group,
        )

    def _state(self, _req, res):
        res.current_state.id = 3
        res.current_state.label = 'active'
        return res

    def _navigate(self, goal_handle):
        time.sleep(0.2)
        goal_handle.succeed()
        return NavigateToPose.Result()

    def _follow(self, goal_handle):
        self.follow_count += 1
        index = self.follow_count
        length = path_length(goal_handle.request.path)
        steps = 15
        for i in range(1, steps + 1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowPath.Result()
            if index == self.hold_segment and i > steps // 2:
                time.sleep(0.05)  # stuck halfway until canceled
                continue
            fb = FollowPath.Feedback()
            fb.distance_to_goal = float(length * (1.0 - i / steps))
            fb.speed = 0.3
            goal_handle.publish_feedback(fb)
            time.sleep(SEGMENT_TIME_S / steps)
        while index == self.hold_segment and not goal_handle.is_cancel_requested:
            time.sleep(0.05)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return FollowPath.Result()
        goal_handle.succeed()
        return FollowPath.Result()


class Caller(Node):
    def __init__(self):
        super().__init__('nav_progress_check')
        self.messages = []
        self.lock = threading.Lock()
        latched = QoSProfile(depth=10)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(CoverageProgress, '/coverage_progress', self._progress, latched)
        self.action = ActionClient(self, Waypoint, 'nav_action_follow_path')
        self.confirm = self.create_client(ConfirmNavigationDispatch, '/confirm_navigation_dispatch')
        self.status = self.create_client(GetCoverageProgress, '/coverage_progress_status')

    def _progress(self, msg):
        with self.lock:
            self.messages.append(msg)

    def of(self, mission_id):
        with self.lock:
            return [m for m in self.messages if m.mission_id == mission_id]


def wait(future, timeout_s):
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.02)
    return future.result() if future.done() else None


class Check:
    def __init__(self, fake, caller):
        self.fake = fake
        self.caller = caller
        self.failures = []

    def expect(self, ok, what):
        print(f'  [{"ok" if ok else "FAIL"}] {what}', flush=True)
        if not ok:
            self.failures.append(what)
        return ok

    def dispatch(self, zone_id=None):
        goal = Waypoint.Goal()
        goal.path = straight_path()
        goal.coverage_split_points = [split_point(2.0), split_point(4.0)]
        goal.dispatch_id = uuid.uuid4().hex
        if zone_id is not None:
            goal.zone_id = zone_id
        handle = wait(self.caller.action.send_goal_async(goal), 5.0)
        if not self.expect(handle is not None and handle.accepted, 'goal accepted'):
            return None, None
        req = ConfirmNavigationDispatch.Request()
        req.dispatch_id = goal.dispatch_id
        wait(self.caller.confirm.call_async(req), 5.0)
        return goal.dispatch_id, handle

    def final_status(self):
        return wait(self.caller.status.call_async(GetCoverageProgress.Request()), 5.0)

    def scenario_a(self):
        print('scenario A: zone 7 runs to the end', flush=True)
        self.fake.hold_segment = None
        self.fake.follow_count = 0
        mission, handle = self.dispatch(zone_id=7)
        if handle is None:
            return
        result = wait(handle.get_result_async(), 30.0)
        self.expect(result is not None and result.result.success, 'action succeeded')
        time.sleep(0.3)
        msgs = self.caller.of(mission)
        self.expect(len(msgs) >= 5, f'{len(msgs)} progress messages for the mission')
        if not msgs:
            return
        statuses = [m.status for m in msgs]
        self.expect(statuses[0] == CoverageProgress.STATUS_NAVIGATING_TO_START, 'starts navigating_to_start')
        self.expect(CoverageProgress.STATUS_RUNNING in statuses, 'running is published')
        self.expect(all(m.zone_id == 7 for m in msgs), 'zone id 7 echoed')
        overall = [m.overall_progress for m in msgs]
        self.expect(all(b >= a - 1e-6 for a, b in zip(overall, overall[1:])), 'overall progress never falls')
        self.expect(
            any(0.0 < m.current_segment_progress < 1.0 and m.status == CoverageProgress.STATUS_RUNNING for m in msgs),
            'feedback moves progress inside a segment',
        )
        last = msgs[-1]
        self.expect(last.status == CoverageProgress.STATUS_SUCCEEDED and last.status_text == 'succeeded', 'ends succeeded')
        self.expect((last.completed_segments, last.total_segments) == (3, 3), f'3/3 segments ({last.completed_segments}/{last.total_segments})')
        self.expect(abs(last.total_distance_m - 6.0) < 1e-3 and abs(last.overall_progress - 1.0) < 1e-6, 'total 6 m, 100 %')
        self.expect(abs(last.remaining_distance_m) < 1e-6, 'nothing remaining')
        status = self.final_status()
        self.expect(
            status is not None and status.success and status.progress.mission_id == mission
            and status.progress.status == CoverageProgress.STATUS_SUCCEEDED,
            '/coverage_progress_status returns the final state',
        )

    def scenario_b(self):
        print('scenario B: canceled halfway through segment 2', flush=True)
        self.fake.hold_segment = 2
        self.fake.follow_count = 0
        mission, handle = self.dispatch()
        if handle is None:
            return
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            msgs = self.caller.of(mission)
            if msgs and msgs[-1].current_segment_index == 2 and msgs[-1].current_segment_progress >= 0.4:
                break
            time.sleep(0.05)
        wait(handle.cancel_goal_async(), 5.0)
        result = wait(handle.get_result_async(), 20.0)
        self.expect(result is not None and not result.result.success, 'action ended without success')
        time.sleep(0.3)
        msgs = self.caller.of(mission)
        if not self.expect(bool(msgs), 'progress published'):
            return
        self.expect(all(m.zone_id == -1 for m in msgs), 'no zone id -> -1')
        statuses = [m.status for m in msgs]
        self.expect(CoverageProgress.STATUS_CANCELING in statuses, 'canceling is published')
        last = msgs[-1]
        self.expect(last.status == CoverageProgress.STATUS_CANCELED, f'ends canceled ({last.status_text})')
        self.expect(last.completed_segments == 1 and last.current_segment_index == 2, 'segment 1 done, segment 2 active')
        self.expect(1.0 / 3.0 < last.overall_progress < 2.0 / 3.0, f'progress kept ({last.overall_progress:.3f})')
        status = self.final_status()
        self.expect(
            status is not None and status.progress.mission_id == mission
            and status.progress.status == CoverageProgress.STATUS_CANCELED,
            '/coverage_progress_status returns the canceled state',
        )


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--only', default='ab', help='scenarios to run, e.g. "a"')
    ap.add_argument('command', nargs='+', help='navigation server command line')
    args = ap.parse_args()

    rclpy.init()
    fake = FakeNav2()
    caller = Caller()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(fake)
    executor.add_node(caller)
    threading.Thread(target=executor.spin, daemon=True).start()

    server = subprocess.Popen(
        args.command + ['--ros-args', '-p', 'require_navigation_health:=false'],
    )
    check = Check(fake, caller)
    try:
        if not caller.action.wait_for_server(timeout_sec=20.0):
            check.expect(False, 'nav_action_follow_path is up')
        else:
            for s in args.only:
                getattr(check, f'scenario_{s}')()
    finally:
        server.terminate()
        try:
            server.wait(timeout=10)
        except subprocess.TimeoutExpired:
            server.kill()
        executor.shutdown()
        rclpy.try_shutdown()
    if check.failures:
        print(f'{len(check.failures)} expectation(s) failed', flush=True)
        sys.exit(1)
    print('all expectations met', flush=True)


if __name__ == '__main__':
    main()
