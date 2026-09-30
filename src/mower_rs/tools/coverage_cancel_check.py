#!/usr/bin/env python3
"""Behavioural check of mower_coverage's correlated-cancel tracking.

Runs the real mower_coverage binary against fake navigation-side endpoints
(the Waypoint action server `nav_action_follow_path`, /cancel_navigation_dispatch,
/confirm_navigation_dispatch, /check_nav_status, the zone map service, the
mission lock and the latched maps) and times what it sends. It replaces the
rclpy unit tests of the removed coverage_node tracker
(test_coverage_cancel_tracking.py), black-box, so the node's code is tested
as it runs on the robot.

Scenarios (times from the dispatch):
  A  confirmation fails, the action accepts the cancel but never ends:
     the 2 s retry sends ONE /cancel_navigation_dispatch, whose
     terminal_confirmed answer ends the tracking (no further requests).
  B  the goal is accepted after the 3 s acceptance timeout and the cancel
     service hangs: one request at ~3 s; the late goal's tracker shares that
     attempt (no second request while it is pending); after the 5 s request
     timeout the retry sends the second one; once the goal is terminal no
     more are sent.
  C  confirmation fails and the action ends the goal on cancel: the action
     result proves the terminal state, no fallback request at all.
  D  tracking is in progress (cancel service hangs) and the node gets
     SIGINT: it exits promptly, and nothing is sent after the signal.

Needs a sourced ROS 2 Jazzy environment with mower_interface built:

  python3 src/mower_rs/tools/coverage_cancel_check.py --binary <path to mower_coverage>

Exits 1 on the first failed expectation.
"""

import argparse
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from mower_interface.action import Waypoint
from mower_interface.msg import ZoneMap
from mower_interface.srv import (
    CancelNavigationDispatch,
    ConfirmNavigationDispatch,
    MissionOperationLock,
    ZoneExecPath,
    ZoneMapList,
)
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

RES, N = 0.05, 80  # a 4 x 4 m zone: fast to plan


def grid(data):
    g = OccupancyGrid()
    g.header.frame_id = 'map'
    g.info.resolution = RES
    g.info.width = N
    g.info.height = N
    g.info.origin.orientation.w = 1.0
    g.data = data
    return g


def zone_cells(inflate):
    lo, hi = 0.3 + inflate, N * RES - 0.3 - inflate
    return [
        0 if lo < (c + 0.5) * RES < hi and lo < (r + 0.5) * RES < hi else 100
        for r in range(N) for c in range(N)
    ]


class FakeNavigation(Node):
    """Everything the coverage node talks to, with scripted behaviour."""

    def __init__(self):
        super().__init__('fake_navigation_side')
        group = ReentrantCallbackGroup()
        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.lock = threading.Lock()
        self.reset()

        zone = ZoneMap()
        zone.zone_id = 1
        zone.mask_map = grid(zone_cells(0.0))
        zone.mask_map_inflated = grid(zone_cells(0.75))
        self.zone = zone

        self.create_service(ZoneMapList, '/get_zone_map_list_srv',
                            self.on_zone_list, callback_group=group)
        self.create_service(MissionOperationLock, '/mission_operation_lock',
                            self.on_lock, callback_group=group)
        self.create_service(CancelNavigationDispatch, '/cancel_navigation_dispatch',
                            self.on_cancel_dispatch, callback_group=group)
        self.create_service(ConfirmNavigationDispatch, '/confirm_navigation_dispatch',
                            self.on_confirm, callback_group=group)
        self.create_service(Trigger, '/check_nav_status',
                            self.on_status, callback_group=group)
        self.risk = self.create_publisher(OccupancyGrid, '/risk_map_inflated', latched)
        self.risk.publish(grid([0] * (N * N)))
        self.nav_active = self.create_publisher(Bool, '/nav_operation_active', latched)
        self.nav_active.publish(Bool(data=False))
        self.action = ActionServer(
            self, Waypoint, 'nav_action_follow_path',
            execute_callback=self.on_execute,
            goal_callback=self.on_goal,
            cancel_callback=self.on_action_cancel,
            callback_group=group)

    def reset(self, **behaviour):
        # end a goal left running by the previous scenario
        self.end_goal_now = True
        time.sleep(0.3)
        with self.lock:
            self.accept_delay_s = behaviour.get('accept_delay_s', 0.0)
            self.confirm_success = behaviour.get('confirm_success', True)
            # 'terminal' | 'hang'
            self.cancel_dispatch = behaviour.get('cancel_dispatch', 'terminal')
            self.end_goal_on_cancel = behaviour.get('end_goal_on_cancel', False)
            self.end_goal_now = False
            self.dispatch_requests = []   # (time, dispatch_id)
            self.action_cancels = []      # time
            self.goals = []               # (time accepted, dispatch_id)

    # ---- services
    def on_zone_list(self, req, res):
        res.zone_map_list = [self.zone]
        return res

    def on_lock(self, req, res):
        res.success = True
        res.message = 'ok'
        return res

    def on_status(self, req, res):
        res.success = True
        res.message = '{}'
        return res

    def on_confirm(self, req, res):
        res.success = self.confirm_success
        res.message = 'confirmed' if self.confirm_success else 'scripted confirmation failure'
        return res

    def on_cancel_dispatch(self, req, res):
        with self.lock:
            self.dispatch_requests.append((time.monotonic(), req.dispatch_id))
            mode = self.cancel_dispatch
        if mode == 'hang':
            time.sleep(12.0)  # longer than the node's 5 s request timeout
            res.success = False
            res.message = 'late answer'
            return res
        res.success = True
        res.terminal_confirmed = True
        res.message = 'terminal'
        return res

    # ---- action
    def on_goal(self, goal):
        time.sleep(self.accept_delay_s)
        with self.lock:
            self.goals.append((time.monotonic(), goal.dispatch_id))
        return GoalResponse.ACCEPT

    def on_action_cancel(self, goal_handle):
        with self.lock:
            self.action_cancels.append(time.monotonic())
        return CancelResponse.ACCEPT

    def on_execute(self, goal_handle):
        while rclpy.ok():
            if goal_handle.is_cancel_requested and (self.end_goal_on_cancel or self.end_goal_now):
                goal_handle.canceled()
                return Waypoint.Result(success=False)
            if self.end_goal_now:
                goal_handle.abort()
                return Waypoint.Result(success=False)
            time.sleep(0.05)
        return Waypoint.Result(success=False)


class Check:
    def __init__(self, binary, fake, caller):
        self.binary, self.fake, self.caller = binary, fake, caller
        self.failures = []

    def expect(self, ok, what):
        print(('  ok    ' if ok else '  FAIL  ') + what, flush=True)
        if not ok:
            self.failures.append(what)

    def start_node(self):
        proc = subprocess.Popen([self.binary], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        gen = self.caller.create_client(Trigger, '/generate_coverage_path')
        if not gen.wait_for_service(timeout_sec=15.0):
            proc.kill()
            raise RuntimeError('mower_coverage did not come up')
        future = gen.call_async(Trigger.Request())
        self.wait(lambda: future.done(), 20.0)
        if not (future.done() and future.result().success):
            proc.kill()
            detail = future.result().message if future.done() else 'no answer'
            raise RuntimeError(f'generate_coverage_path failed: {detail}')
        self.caller.destroy_client(gen)
        return proc

    def dispatch(self):
        client = self.caller.create_client(ZoneExecPath, '/zone_exec_path')
        client.wait_for_service(timeout_sec=5.0)
        t0 = time.monotonic()
        future = client.call_async(ZoneExecPath.Request(zone_id=1))
        return t0, future

    @staticmethod
    def wait(cond, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not cond():
            time.sleep(0.05)
        return cond()

    def stop(self, proc):
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
        return proc.stdout.read() if proc.stdout else ''

    def requests_between(self, t0, a, b):
        with self.fake.lock:
            return [t - t0 for t, _ in self.fake.dispatch_requests if a <= t - t0 < b]

    def scenario_a(self):
        print('A: confirmation fails, goal never ends, cancel service confirms terminal')
        self.fake.reset(confirm_success=False, cancel_dispatch='terminal', end_goal_on_cancel=False)
        proc = self.start_node()
        t0, _ = self.dispatch()
        time.sleep(9.0)
        with self.fake.lock:
            cancels = [t - t0 for t in self.fake.action_cancels]
            reqs = [t - t0 for t, _ in self.fake.dispatch_requests]
        self.expect(len(cancels) >= 1 and cancels[0] < 2.5, f'action-level cancel sent ({cancels})')
        self.expect(len(reqs) == 1, f'exactly one correlated cancel request ({reqs})')
        self.expect(bool(reqs) and 1.5 <= reqs[0] <= 3.5, 'it came from the 2 s retry')
        self.stop(proc)

    def scenario_b(self):
        print('B: late acceptance, cancel service hangs')
        self.fake.reset(accept_delay_s=4.0, cancel_dispatch='hang')
        proc = self.start_node()
        t0, _ = self.dispatch()
        time.sleep(12.0)
        first = self.requests_between(t0, 0.0, 7.8)
        second = self.requests_between(t0, 7.8, 12.0)
        self.expect(len(first) == 1 and 2.5 <= first[0] <= 4.0,
                    f'one request at the 3 s acceptance timeout, shared by the late goal ({first})')
        self.expect(len(second) == 1,
                    f'one retry after the 5 s request timeout ({second})')
        with self.fake.lock:
            ids = {d for _, d in self.fake.dispatch_requests}
            cancels = [t - t0 for t in self.fake.action_cancels]
        self.expect(len(ids) == 1, 'every request names the same dispatch id')
        self.expect(bool(cancels) and 3.5 <= cancels[0] <= 6.0, f'the late goal is canceled ({cancels})')
        self.fake.end_goal_now = True
        time.sleep(1.0)
        t_end = time.monotonic() - t0
        time.sleep(6.0)
        after = self.requests_between(t0, t_end, t_end + 6.0)
        self.expect(after == [], f'nothing is sent once the goal is terminal ({after})')
        self.stop(proc)

    def scenario_c(self):
        print('C: confirmation fails, the action ends the goal on cancel')
        self.fake.reset(confirm_success=False, end_goal_on_cancel=True)
        proc = self.start_node()
        t0, _ = self.dispatch()
        time.sleep(7.0)
        with self.fake.lock:
            cancels = len(self.fake.action_cancels)
            reqs = [t - t0 for t, _ in self.fake.dispatch_requests]
        self.expect(cancels >= 1, 'action-level cancel sent')
        self.expect(reqs == [], f'no correlated cancel request ({reqs})')
        self.stop(proc)

    def scenario_d(self):
        print('D: SIGINT while tracking with a hanging cancel service')
        self.fake.reset(confirm_success=False, cancel_dispatch='hang', end_goal_on_cancel=False)
        proc = self.start_node()
        t0, _ = self.dispatch()
        self.wait(lambda: len(self.fake.dispatch_requests) >= 1, 6.0)
        t_sig = time.monotonic()
        proc.send_signal(signal.SIGINT)
        try:
            code = proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            code = None
            proc.kill()
        took = time.monotonic() - t_sig
        self.expect(code is not None and took < 8, f'exits after SIGINT (code {code}, {took:.1f} s)')
        time.sleep(3.0)
        with self.fake.lock:
            late = [t - t_sig for t, _ in self.fake.dispatch_requests if t > t_sig + 0.2]
        self.expect(late == [], f'nothing sent after the signal ({late})')


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--binary', required=True, help='path to the mower_coverage binary')
    ap.add_argument('--only', default='abcd', help='scenarios to run, e.g. "ab"')
    args = ap.parse_args()

    rclpy.init()
    fake = FakeNavigation()
    caller = rclpy.create_node('coverage_cancel_check')
    executor = MultiThreadedExecutor(num_threads=16)
    executor.add_node(fake)
    executor.add_node(caller)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    check = Check(os.path.abspath(args.binary), fake, caller)
    try:
        for s in args.only:
            getattr(check, f'scenario_{s}')()
    finally:
        executor.shutdown()
        rclpy.try_shutdown()
    if check.failures:
        print(f'{len(check.failures)} expectation(s) failed', flush=True)
        sys.exit(1)
    print('all expectations met', flush=True)


if __name__ == '__main__':
    main()
