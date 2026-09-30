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
  E  the server restarts (as --restart-as, if given: the checkpoint file is
     shared by both servers): /coverage_progress_status restores B's
     execution from the checkpoint file and reports it resumable at
     segment 2; D and C then run against the restarted server.
  D  resume goals the checkpoint does not cover are rejected: a different
     path, and a segment past the first unfinished one.
  C  resume at segment 2: Nav2 is sent to the start of segment 2 (x = 2 m),
     only segments 2 and 3 are followed, progress starts at 1/3 and ends
     succeeded; the checkpoint is then no longer resumable.

The checkpoint goes to a temporary directory (progress_checkpoint_path).
Needs a sourced ROS 2 Jazzy environment with mower_interface and nav2_msgs:

  python3 src/mower_rs/tools/nav_progress_check.py -- <path to mower_nav>
  python3 src/mower_rs/tools/nav_progress_check.py -- \\
      python3 -m mower_mission.navigation.nav_action_server
  python3 src/mower_rs/tools/nav_progress_check.py \\
      --restart-as 'python3 -m mower_mission.navigation.nav_action_server' \\
      -- <path to mower_nav>

Exits 1 on the first failed expectation.
"""

import argparse
import math
import os
import shlex
import subprocess
import sys
import tempfile
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
        self.navigate_goals = []
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
        self.navigate_goals.append(goal_handle.request.pose)
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


class Server:
    """The navigation server process, restartable."""

    def __init__(self, command, checkpoint_path, restart_as=None):
        self.args = [
            '--ros-args',
            '-p', 'require_navigation_health:=false',
            '-p', f'progress_checkpoint_path:={checkpoint_path}',
        ]
        self.command = command
        self.restart_as = restart_as
        self.process = None

    def start(self):
        self.process = subprocess.Popen(self.command + self.args)

    def restart(self):
        self.stop()
        if self.restart_as:
            print(f'  restarting as: {" ".join(self.restart_as)}', flush=True)
            self.command = self.restart_as
        self.start()

    def stop(self):
        if self.process is None:
            return
        self.process.terminate()
        try:
            self.process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.process.kill()
        self.process = None


class Check:
    def __init__(self, fake, caller, server):
        self.fake = fake
        self.caller = caller
        self.server = server
        self.failures = []

    def expect(self, ok, what):
        print(f'  [{"ok" if ok else "FAIL"}] {what}', flush=True)
        if not ok:
            self.failures.append(what)
        return ok

    def dispatch(self, zone_id=None, resume=0, length_m=6.0, expect_accept=True):
        goal = Waypoint.Goal()
        goal.path = straight_path(length_m)
        goal.coverage_split_points = [split_point(2.0), split_point(4.0)]
        goal.dispatch_id = uuid.uuid4().hex
        if zone_id is not None:
            goal.zone_id = zone_id
        goal.resume_segment_index = resume
        handle = wait(self.caller.action.send_goal_async(goal), 5.0)
        accepted = handle is not None and handle.accepted
        if not expect_accept:
            self.expect(handle is not None and not accepted, 'goal rejected')
            return None, None
        if not self.expect(accepted, 'goal accepted'):
            return None, None
        req = ConfirmNavigationDispatch.Request()
        req.dispatch_id = goal.dispatch_id
        wait(self.caller.confirm.call_async(req), 5.0)
        return goal.dispatch_id, handle

    def scenario_e(self):
        print('scenario E: restart, the checkpoint is restored', flush=True)
        before = self.final_status()
        self.server.restart()
        if not self.expect(self.caller.action.wait_for_server(timeout_sec=20.0), 'server back'):
            return
        # the service can come up a little after the action server
        status = None
        deadline = time.monotonic() + 10.0
        while status is None and time.monotonic() < deadline:
            if self.caller.status.wait_for_service(timeout_sec=1.0):
                status = self.final_status()
        if not self.expect(status is not None, '/coverage_progress_status answers'):
            return
        c = status.checkpoint
        self.expect(before is not None and c.mission_id == before.progress.mission_id, 'checkpoint of the last execution')
        self.expect(c.checkpoint_available, f'resumable ({c.message})')
        self.expect((c.completed_segments, c.total_segments, c.status_text) == (1, 3, 'canceled'), f'1/3 segments, canceled ({c.completed_segments}/{c.total_segments} {c.status_text})')
        self.expect(status.progress.mission_id == c.mission_id and status.progress.checkpoint_available, 'progress restored from it')

    def scenario_d(self):
        print('scenario D: resume goals outside the checkpoint are rejected', flush=True)
        self.dispatch(resume=2, length_m=7.0, expect_accept=False)
        self.dispatch(resume=3, expect_accept=False)
        status = self.final_status()
        self.expect(status is not None and status.checkpoint.checkpoint_available, 'checkpoint still resumable')

    def scenario_c(self):
        print('scenario C: resume at segment 2', flush=True)
        self.fake.hold_segment = None
        self.fake.follow_count = 0
        self.fake.navigate_goals.clear()
        mission, handle = self.dispatch(resume=2)
        if handle is None:
            return
        result = wait(handle.get_result_async(), 30.0)
        self.expect(result is not None and result.result.success, 'action succeeded')
        time.sleep(0.3)
        self.expect(self.fake.follow_count == 2, f'2 segments followed ({self.fake.follow_count})')
        starts = [p.pose.position.x for p in self.fake.navigate_goals]
        self.expect(len(starts) == 1 and abs(starts[0] - 2.0) < 1e-6, f'navigated to x = 2 m ({starts})')
        msgs = self.caller.of(mission)
        if not self.expect(bool(msgs), 'progress published'):
            return
        running = [m for m in msgs if m.status == CoverageProgress.STATUS_RUNNING]
        self.expect(bool(running) and running[0].current_segment_index == 2, 'first running segment is 2')
        self.expect(bool(running) and abs(running[0].overall_progress - 1.0 / 3.0) < 0.02, 'starts at 1/3')
        last = msgs[-1]
        self.expect(last.status == CoverageProgress.STATUS_SUCCEEDED and last.completed_segments == 3, 'ends succeeded, 3/3')
        status = self.final_status()
        self.expect(status is not None and not status.checkpoint.checkpoint_available, 'nothing left to resume')

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
    ap.add_argument('--only', default='abedc', help='scenarios to run, in order, e.g. "a"')
    ap.add_argument('--restart-as', help='server command line to restart into in scenario E')
    ap.add_argument('command', nargs='+', help='navigation server command line')
    args = ap.parse_args()

    rclpy.init()
    fake = FakeNav2()
    caller = Caller()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(fake)
    executor.add_node(caller)
    threading.Thread(target=executor.spin, daemon=True).start()

    tmp = tempfile.TemporaryDirectory()
    server = Server(
        args.command,
        os.path.join(tmp.name, 'coverage_progress.json'),
        shlex.split(args.restart_as) if args.restart_as else None,
    )
    server.start()
    check = Check(fake, caller, server)
    try:
        if not caller.action.wait_for_server(timeout_sec=20.0):
            check.expect(False, 'nav_action_follow_path is up')
        else:
            for s in args.only:
                getattr(check, f'scenario_{s}')()
    finally:
        server.stop()
        tmp.cleanup()
        executor.shutdown()
        rclpy.try_shutdown()
    if check.failures:
        print(f'{len(check.failures)} expectation(s) failed', flush=True)
        sys.exit(1)
    print('all expectations met', flush=True)


if __name__ == '__main__':
    main()
