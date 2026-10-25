#!/usr/bin/env python3
"""Behavioural check of mower_nav's in-process navigation (nav_backend nav_core).

With nav_backend:=nav_core mower_nav runs mower_nav_core's NavStack itself
(docs/ROS_FREE_PLAN.md D6, crates/mower_nav/src/core_backend.rs) and no Nav2
runs at all. This starts the real binary with nav_core.pose_source:=tf and
require_navigation_health:=false and plays everything around it:

  maps    /map_grid and /map_grid_global (reliable, transient_local): a
          10 x 6 m free field at 0.05 m (x -2 .. 8 m, y -3 .. 3 m) with a
          0.1 m lethal border
  robot   map -> odom (fixed: x 0.3 m, y -0.2 m, yaw 0.1 rad) and
          odom -> base_footprint on /tf, and /odom, at 50 Hz from a
          kinematic robot that follows the last /nav_cmd_vel (and stops
          0.5 s after the last one, like twist_mux)
  caller  Waypoint goals through nav_action_follow_path, confirmed on
          /confirm_navigation_dispatch; /coverage_progress, /nav_cmd_vel and
          /plan are recorded

It first checks that something other than this harness subscribes
/map_grid, /map_grid_global, /odom and /tf and publishes /nav_cmd_vel, i.e.
that the in-process stack runs at all, and stops there if not. Scenarios,
always in this order (D needs a stack that has never had a map):

  D  no map yet: the mission fails nav2_ready_timeout_s (--ready-timeout,
     3 s) after the dispatch confirmation with the readiness message.
  A  from (0, -0.3) a 3 m straight path from (1, 0) along +x, split at x = 2
     and 3 m: navigating to the start, then each 1 m segment succeeds,
     /coverage_progress ends succeeded 3/3, /plan was published, the robot
     stops within 0.1 m of the last pose, and /nav_cmd_vel (base_footprint)
     never steps back in time.
  B  the same mission, canceled 0.3 m into segment 1 at speed: the goal ends
     CANCELED and /nav_cmd_vel is zero within 0.5 s and stays zero.
  C  a wall at x = 1.2 .. 1.4 m in /map_grid only (the global map stays
     free), the coverage start beyond it at x = 2.5 m: the plan runs through
     the wall, the controller stops in front of it, the tree's recoveries
     retry (~35 s), and the mission fails with "Nav2 task error: code=104"
     (PATIENCE_EXCEEDED); the robot never reaches the wall.

Needs a sourced ROS 2 Jazzy environment with mower_interface:

  python3 src/mower_rs/tools/nav_core_check.py -- <path to mower_nav>
  python3 src/mower_rs/tools/nav_core_check.py --only a -- <path to mower_nav>

mower_nav's output goes to a file (--log keeps it); its last warnings and
errors are printed when an expectation fails. Exits 1 if any did.
"""

import argparse
import array
import math
import os
import subprocess
import sys
import tempfile
import threading
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, TwistStamped
from mower_interface.action import Waypoint
from mower_interface.msg import CoverageProgress
from mower_interface.srv import ConfirmNavigationDispatch
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

SIM_HZ = 50.0
MAP_FROM_ODOM = (0.3, -0.2, 0.1)
CMD_TIMEOUT_S = 0.5  # twist_mux's /nav_cmd_vel timeout
SETTLE_S = 3.0  # the 1 Hz global and 2 Hz local costmaps take a new map and pose
FIELD_CELLS = (200, 120)
RESOLUTION = 0.05
FIELD_ORIGIN = (-2.0, -3.0)
BORDER_CELLS = 2
WALL_X = (1.2, 1.4)
A_START = (0.0, -0.3, 0.0)
A_PATH = (1.0, 4.0)  # x range, along +x at y = 0
A_SPLITS = (2.0, 3.0)
C_START = (0.0, 0.05, 0.05)
C_PATH = (2.5, 3.5)
STATUS = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_CANCELED: 'canceled', GoalStatus.STATUS_ABORTED: 'aborted'}
HARNESS_NODES = ('nav_core_check', 'nav_core_check_robot')
# what the in-process stack must have on the graph before anything can move
BACKEND_ENDPOINTS = [('subscribes', t) for t in ('/map_grid', '/map_grid_global', '/odom', '/tf')] + [('publishes', '/nav_cmd_vel')]


def quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def compose(t, p):
    """t (x, y, yaw) applied to pose p."""
    c, s = math.cos(t[2]), math.sin(t[2])
    return (c * p[0] - s * p[1] + t[0], s * p[0] + c * p[1] + t[1], t[2] + p[2])


def inverse(t):
    c, s = math.cos(t[2]), math.sin(t[2])
    return (-(c * t[0] + s * t[1]), s * t[0] - c * t[1], -t[2])


def transform(stamp, parent, child, pose):
    t = TransformStamped()
    t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, parent, child
    t.transform.translation.x, t.transform.translation.y = pose[0], pose[1]
    _, _, t.transform.rotation.z, t.transform.rotation.w = quat(pose[2])
    return t


def grid(wall_x=None):
    """The field's cells: free, the lethal border, and the wall if given."""
    w, h = FIELD_CELLS
    data = array.array('b', bytes(w * h))
    for j in range(h):
        for i in range(w):
            x = FIELD_ORIGIN[0] + (i + 0.5) * RESOLUTION
            border = min(i, j, w - 1 - i, h - 1 - j) < BORDER_CELLS
            if border or (wall_x is not None and wall_x[0] <= x < wall_x[1]):
                data[j * w + i] = 100
    return data


def straight_path(x0, x1, step_m=0.25):
    # poses 0.25 m apart: exactly one lies within the 0.1 m split tolerance
    path = Path()
    path.header.frame_id = 'map'
    for i in range(int(round((x1 - x0) / step_m)) + 1):
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.pose.position.x = x0 + i * step_m
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)
    return path


def split_point(x):
    p = Pose()
    p.position.x = x
    p.orientation.w = 1.0
    return p


def wait_until(predicate, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


def dedup(values):
    return [v for i, v in enumerate(values) if i == 0 or v != values[i - 1]]


def status_name(status):
    return STATUS.get(status, f'status {status}')


def is_zero(cmd):
    """A recorded /nav_cmd_vel (receipt, stamp, frame, v, w) that stops the robot."""
    return abs(cmd[3]) < 1e-6 and abs(cmd[4]) < 1e-6


class Robot(Node):
    """Maps, tf and /odom, from a kinematic robot driven by /nav_cmd_vel."""

    def __init__(self):
        super().__init__('nav_core_check_robot')
        self.lock = threading.Lock()
        self.pose = (0.0, 0.0, 0.0)  # base_footprint in odom
        self.twist = (0.0, 0.0)
        self.last_cmd = None
        self.last_step = None
        self.cmds = []  # (receipt, stamp ns, frame_id, v, w)
        self.plans = []  # receipt
        self.track = []  # (time, x, y) in map
        self.published = {}  # map topic -> wall it was last published with
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pubs = {topic: self.create_publisher(OccupancyGrid, topic, latched) for topic in ('/map_grid', '/map_grid_global')}
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 100)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(TwistStamped, '/nav_cmd_vel', self.on_cmd, 10)
        # best effort matches any /plan publisher; the plans are only counted
        self.create_subscription(Path, '/plan', self.on_plan, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(1.0 / SIM_HZ, self.step)

    def on_cmd(self, msg):
        now = time.monotonic()
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        with self.lock:
            self.cmds.append((now, stamp, msg.header.frame_id, msg.twist.linear.x, msg.twist.angular.z))
            self.twist = (msg.twist.linear.x, msg.twist.angular.z)
            self.last_cmd = now

    def on_plan(self, _msg):
        with self.lock:
            self.plans.append(time.monotonic())

    def step(self):
        now = time.monotonic()
        with self.lock:
            # integrate over the time that really passed: a late timer must
            # not slow the robot down
            dt = min(now - self.last_step, 0.2) if self.last_step is not None else 0.0
            self.last_step = now
            if self.last_cmd is None or now - self.last_cmd > CMD_TIMEOUT_S:
                self.twist = (0.0, 0.0)
            v, w = self.twist
            x, y, yaw = self.pose
            self.pose = (x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt, yaw + w * dt)
            pose = self.pose
            mx, my, _ = compose(MAP_FROM_ODOM, pose)
            self.track.append((now, mx, my))
        stamp = self.get_clock().now().to_msg()
        tf = TFMessage()
        tf.transforms = [transform(stamp, 'map', 'odom', MAP_FROM_ODOM), transform(stamp, 'odom', 'base_footprint', pose)]
        self.tf_pub.publish(tf)
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, 'odom', 'base_footprint'
        o.pose.pose.position.x, o.pose.pose.position.y = pose[0], pose[1]
        _, _, o.pose.pose.orientation.z, o.pose.pose.orientation.w = quat(pose[2])
        o.twist.twist.linear.x, o.twist.twist.angular.z = v, w
        self.odom_pub.publish(o)

    def place(self, map_pose):
        """Put the robot at map_pose, standing still."""
        with self.lock:
            self.pose = compose(inverse(MAP_FROM_ODOM), map_pose)
            self.twist = (0.0, 0.0)
            self.last_cmd = None

    def map_pose(self):
        with self.lock:
            return compose(MAP_FROM_ODOM, self.pose)

    def speed(self):
        with self.lock:
            return self.twist[0]

    def set_maps(self, local_wall):
        """Both maps free, /map_grid with the wall if local_wall; only a
        changed map is published again."""
        for topic, wall in (('/map_grid_global', None), ('/map_grid', WALL_X if local_wall else None)):
            if topic in self.published and self.published[topic] == wall:
                continue
            g = OccupancyGrid()
            g.header.frame_id = 'map'
            g.header.stamp = self.get_clock().now().to_msg()
            g.info.map_load_time = g.header.stamp
            g.info.width, g.info.height = FIELD_CELLS
            g.info.resolution = RESOLUTION
            g.info.origin.position.x, g.info.origin.position.y = FIELD_ORIGIN
            g.info.origin.orientation.w = 1.0
            g.data = grid(wall)
            self.map_pubs[topic].publish(g)
            self.published[topic] = wall

    def cmds_between(self, t0, t1):
        with self.lock:
            return [c for c in self.cmds if t0 <= c[0] <= t1]

    def plans_between(self, t0, t1):
        with self.lock:
            return [t for t in self.plans if t0 <= t <= t1]

    def max_x_between(self, t0, t1):
        with self.lock:
            return max((x for t, x, _ in self.track if t0 <= t <= t1), default=float('nan'))


class Caller(Node):
    def __init__(self):
        super().__init__('nav_core_check')
        self.lock = threading.Lock()
        self.messages = []
        latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(CoverageProgress, '/coverage_progress', self._progress, latched)
        self.action = ActionClient(self, Waypoint, 'nav_action_follow_path')
        self.confirm = self.create_client(ConfirmNavigationDispatch, '/confirm_navigation_dispatch')

    def _progress(self, msg):
        with self.lock:
            self.messages.append(msg)

    def of(self, mission_id):
        with self.lock:
            return [m for m in self.messages if m.mission_id == mission_id]

    def backend_endpoints(self):
        """[(kind, topic, nodes other than the harness)] of BACKEND_ENDPOINTS."""
        out = []
        for kind, topic in BACKEND_ENDPOINTS:
            infos = self.get_subscriptions_info_by_topic(topic) if kind == 'subscribes' else self.get_publishers_info_by_topic(topic)
            out.append((kind, topic, sorted({i.node_name for i in infos if i.node_name not in HARNESS_NODES})))
        return out


class Server:
    """The mower_nav process; its output goes to a log file."""

    def __init__(self, command, args, log_path):
        self.command = command + args
        self.log_path = log_path
        self.process = None
        self.log = None

    def start(self):
        self.log = open(self.log_path, 'w')
        self.process = subprocess.Popen(self.command, stdout=self.log, stderr=subprocess.STDOUT)

    def exit_code(self):
        return self.process.poll()

    def stop(self):
        if self.process is None:
            return
        self.process.terminate()
        try:
            self.process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.log.close()
        self.process = None

    def output(self):
        try:
            with open(self.log_path, errors='replace') as f:
                return [line.rstrip() for line in f]
        except OSError:
            return []

    def problems(self, n=40):
        """The last n warnings, errors and panics (with the panic's message line)."""
        lines = self.output()
        keep = [line for i, line in enumerate(lines)
                if any(k in line for k in ('[WARN]', '[ERROR]', '[FATAL]', 'panicked')) or (i > 0 and 'panicked' in lines[i - 1])]
        return keep[-n:]


class Mission:
    def __init__(self, dispatch_id, handle):
        self.dispatch_id = dispatch_id
        self.handle = handle
        self.result = handle.get_result_async()
        self.confirmed_at = None
        self.ended_at = None


class Check:
    def __init__(self, robot, caller, server, ready_timeout_s):
        self.robot = robot
        self.caller = caller
        self.server = server
        self.ready_timeout_s = ready_timeout_s
        self.failures = []

    def expect(self, ok, what):
        print(f'  [{"ok" if ok else "FAIL"}] {what}', flush=True)
        if not ok:
            self.failures.append(what)
        return ok

    def wait(self, future, timeout_s):
        """The future's result; None after timeout_s or once mower_nav has exited."""
        wait_until(lambda: future.done() or self.server.exit_code() is not None, timeout_s)
        return future.result() if future.done() else None

    def wait_until(self, predicate, timeout_s):
        return wait_until(lambda: predicate() or self.server.exit_code() is not None, timeout_s) and predicate()

    def startup(self):
        print('mower_nav with nav_backend nav_core, pose from /tf', flush=True)
        up = False
        deadline = time.monotonic() + 20.0
        while not up and time.monotonic() < deadline and self.server.exit_code() is None:
            up = self.caller.action.wait_for_server(timeout_sec=0.5)
        code = self.server.exit_code()
        if not self.expect(up, 'nav_action_follow_path is up' + (f' (mower_nav exited with {code})' if code is not None else '')):
            return False
        endpoints = []
        deadline = time.monotonic() + 10.0
        while True:
            endpoints = self.caller.backend_endpoints()
            if all(nodes for _, _, nodes in endpoints) or time.monotonic() > deadline:
                break
            time.sleep(0.2)
        missing = {}
        for kind, topic, nodes in endpoints:
            if not nodes:
                missing.setdefault(kind, []).append(topic)
        if missing:
            what = '; '.join(f'nobody {kind} {", ".join(topics)}' for kind, topics in missing.items())
        else:
            nodes = sorted({n for _, _, ns in endpoints for n in ns})
            what = f'{", ".join(nodes)} subscribes /map_grid, /map_grid_global, /odom and /tf and publishes /nav_cmd_vel'
        if not self.expect(not missing, f'in-process stack on the graph: {what}'):
            print('  mower_nav does not run the in-process stack (nav_backend:=nav_core unknown?): nothing to test', flush=True)
            return False
        return self.expect(self.caller.confirm.wait_for_service(timeout_sec=5.0), '/confirm_navigation_dispatch is up')

    def prepare(self, start, local_wall):
        self.robot.place(start)
        self.robot.set_maps(local_wall)
        time.sleep(SETTLE_S)

    def dispatch(self, x_range, splits):
        goal = Waypoint.Goal()
        goal.path = straight_path(*x_range)
        goal.coverage_split_points = [split_point(x) for x in splits]
        goal.dispatch_id = uuid.uuid4().hex
        handle = self.wait(self.caller.action.send_goal_async(goal), 5.0)
        if not self.expect(handle is not None and handle.accepted, 'goal accepted'):
            return None
        mission = Mission(goal.dispatch_id, handle)
        req = ConfirmNavigationDispatch.Request()
        req.dispatch_id = goal.dispatch_id
        res = self.wait(self.caller.confirm.call_async(req), 5.0)
        mission.confirmed_at = time.monotonic()
        if not self.expect(res is not None and res.success, 'dispatch confirmed'):
            self.end(mission)
            return None
        return mission

    def outcome(self, mission, timeout_s):
        res = self.wait(mission.result, timeout_s)
        if res is None:
            code = self.server.exit_code()
            self.expect(False, f'goal ended within {timeout_s:.0f} s' if code is None else f'mower_nav exited with {code} before the goal ended')
            self.end(mission)
            return None
        mission.ended_at = time.monotonic()
        return res

    def end(self, mission):
        """Leave nothing running for the next scenario."""
        if not mission.result.done():
            self.wait(mission.handle.cancel_goal_async(), 5.0)
            self.wait(mission.result, 10.0)

    def last_progress(self, mission):
        msgs = self.caller.of(mission.dispatch_id)
        return msgs[-1] if msgs else None

    def expect_commands(self, t0, t1):
        cmds = self.robot.cmds_between(t0, t1)
        back = sum(1 for a, b in zip(cmds, cmds[1:]) if b[1] < a[1])
        frames = sorted({c[2] for c in cmds})
        self.expect(bool(cmds) and back == 0 and frames == ['base_footprint'],
                    f'{len(cmds)} /nav_cmd_vel (frame {", ".join(frames) or "-"}), stamps never step back ({back} did)')

    def scenario_d(self):
        print(f'scenario D: no map yet, readiness times out after {self.ready_timeout_s:.1f} s', flush=True)
        self.robot.place(A_START)
        time.sleep(1.0)  # tf and /odom reach the stack, the maps never do
        before = self.robot.map_pose()
        mission = self.dispatch(A_PATH, A_SPLITS)
        if mission is None:
            return
        res = self.outcome(mission, self.ready_timeout_s + 10.0)
        if res is None:
            return
        elapsed = mission.ended_at - mission.confirmed_at
        self.expect(res.status == GoalStatus.STATUS_ABORTED and not res.result.success, f'goal {status_name(res.status)}')
        self.expect(self.ready_timeout_s - 0.25 <= elapsed <= self.ready_timeout_s + 3.0,
                    f'{elapsed:.1f} s after the confirmation (nav2_ready_timeout_s {self.ready_timeout_s:.1f} s)')
        last = self.last_progress(mission)
        text = last.message if last is not None else ''
        self.expect(last is not None and last.status == CoverageProgress.STATUS_FAILED and 'readiness timed out' in text.lower(),
                    f'fails with the readiness message ({text!r})')
        moved = math.dist(before[:2], self.robot.map_pose()[:2])
        self.expect(moved < 1e-3, f'the robot did not move ({moved:.3f} m)')

    def scenario_a(self):
        print('scenario A: 3 straight 1 m segments to the end', flush=True)
        self.prepare(A_START, local_wall=False)
        t0 = time.monotonic()
        mission = self.dispatch(A_PATH, A_SPLITS)
        if mission is None:
            return
        res = self.outcome(mission, 120.0)
        if res is None:
            return
        self.expect(res.status == GoalStatus.STATUS_SUCCEEDED and res.result.success,
                    f'goal {status_name(res.status)} after {mission.ended_at - mission.confirmed_at:.1f} s')
        time.sleep(1.0)  # the robot comes to a stop
        t1 = time.monotonic()
        msgs = self.caller.of(mission.dispatch_id)
        if self.expect(bool(msgs), f'{len(msgs)} /coverage_progress messages for the mission'):
            self.expect(msgs[0].status == CoverageProgress.STATUS_NAVIGATING_TO_START, f'starts {msgs[0].status_text}')
            running = dedup([m.current_segment_index for m in msgs if m.status == CoverageProgress.STATUS_RUNNING])
            self.expect(running == [1, 2, 3], f'segments run in order after the start was reached ({running})')
            completed = dedup([m.completed_segments for m in msgs])
            self.expect(completed == [0, 1, 2, 3], f'each segment succeeds (completed {completed})')
            last = msgs[-1]
            self.expect(last.status == CoverageProgress.STATUS_SUCCEEDED and (last.completed_segments, last.total_segments) == (3, 3)
                        and abs(last.overall_progress - 1.0) < 1e-6,
                        f'ends {last.status_text}, {last.completed_segments}/{last.total_segments}, {last.overall_progress:.0%}')
        plans = self.robot.plans_between(t0, t1)
        self.expect(bool(plans), f'{len(plans)} /plan messages')
        x, y, _ = self.robot.map_pose()
        d = math.dist((x, y), (A_PATH[1], 0.0))
        self.expect(d <= 0.1, f'the robot stops {d:.3f} m from the last pose ({x:.3f}, {y:.3f})')
        self.expect_commands(t0, t1)

    def scenario_b(self):
        print('scenario B: canceled 0.3 m into segment 1', flush=True)
        self.prepare(A_START, local_wall=False)
        t0 = time.monotonic()
        mission = self.dispatch(A_PATH, A_SPLITS)
        if mission is None:
            return

        def at_speed_in_segment_1():
            last = self.last_progress(mission)
            return (last is not None and last.status == CoverageProgress.STATUS_RUNNING and last.current_segment_index == 1
                    and self.robot.map_pose()[0] >= A_PATH[0] + 0.3 and self.robot.speed() >= 0.1)

        if not self.expect(self.wait_until(at_speed_in_segment_1, 60.0), 'segment 1 runs at speed'):
            self.end(mission)
            return
        canceled_at = time.monotonic()
        cancel = self.wait(mission.handle.cancel_goal_async(), 5.0)
        self.expect(cancel is not None and len(cancel.goals_canceling) == 1, 'cancel accepted')
        res = self.outcome(mission, 10.0)
        if res is None:
            return
        self.expect(res.status == GoalStatus.STATUS_CANCELED and not res.result.success, f'goal {status_name(res.status)}')
        time.sleep(1.0)
        t1 = time.monotonic()
        cmds = self.robot.cmds_between(canceled_at, t1)
        zero_at = next((c[0] for c in cmds if is_zero(c)), None)
        nonzero_after = [c for c in cmds if zero_at is not None and c[0] > zero_at and not is_zero(c)]
        self.expect(zero_at is not None and zero_at - canceled_at <= 0.5,
                    f'/nav_cmd_vel zero {zero_at - canceled_at:.3f} s after the cancel' if zero_at is not None
                    else f'/nav_cmd_vel zero within 0.5 s of the cancel (none of {len(cmds)} since)')
        self.expect(zero_at is not None and not nonzero_after, f'and stays zero ({len(nonzero_after)} non-zero after it)')
        last = self.last_progress(mission)
        self.expect(last is not None and last.status == CoverageProgress.STATUS_CANCELED,
                    f'/coverage_progress ends {last.status_text if last is not None else "-"}')
        self.expect_commands(t0, t1)

    def scenario_c(self):
        print('scenario C: a wall in /map_grid only, between the robot and the start', flush=True)
        self.prepare(C_START, local_wall=True)
        t0 = time.monotonic()
        mission = self.dispatch(C_PATH, ())
        if mission is None:
            return
        res = self.outcome(mission, 90.0)
        if res is None:
            return
        self.expect(res.status == GoalStatus.STATUS_ABORTED and not res.result.success,
                    f'goal {status_name(res.status)} after {mission.ended_at - mission.confirmed_at:.1f} s')
        t1 = time.monotonic()
        last = self.last_progress(mission)
        text = last.message if last is not None else ''
        self.expect(last is not None and last.status == CoverageProgress.STATUS_FAILED and 'Nav2 task error: code=104' in text,
                    f'fails with code 104 ({text!r})')
        max_x = self.robot.max_x_between(t0, t1)
        self.expect(max_x < WALL_X[0], f'the robot stays in front of the wall (x <= {max_x:.2f} m)')
        self.expect_commands(t0, t1)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--only', default='dabc', help='scenarios to run, e.g. "a" (always in the order D, A, B, C)')
    ap.add_argument('--ready-timeout', type=float, default=3.0, help="mower_nav's nav2_ready_timeout_s (scenario D)")
    ap.add_argument('--log', help="keep mower_nav's output in this file")
    ap.add_argument('command', nargs='+', help='mower_nav command line')
    args = ap.parse_args()
    scenarios = [s for s in 'dabc' if s in args.only.lower()]

    rclpy.init()
    robot = Robot()
    caller = Caller()
    # rclpy's MultiThreadedExecutor starved the 50 Hz robot (mower_nav_core's oracles)
    executor = SingleThreadedExecutor()
    executor.add_node(robot)
    executor.add_node(caller)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    tmp = tempfile.TemporaryDirectory()
    server = Server(args.command, [
        '--ros-args',
        '-p', 'nav_backend:=nav_core',
        '-p', 'nav_core.pose_source:=tf',
        '-p', 'require_navigation_health:=false',
        '-p', f'nav2_ready_timeout_s:={args.ready_timeout:.1f}',
        '-p', f'progress_checkpoint_path:={os.path.join(tmp.name, "coverage_progress.json")}',
    ], args.log or os.path.join(tmp.name, 'mower_nav.log'))
    robot.place(A_START)
    server.start()
    check = Check(robot, caller, server, args.ready_timeout)
    try:
        if check.startup():
            for s in scenarios:
                code = server.exit_code()
                if code is not None:
                    check.expect(False, f'mower_nav is still running (exited with {code})')
                    break
                getattr(check, f'scenario_{s}')()
        if check.failures:
            lines = server.problems()
            if lines:
                print(f"mower_nav's last {len(lines)} warnings and errors:", flush=True)
                for line in lines:
                    print(f'  {line}', flush=True)
            code = server.exit_code()
            if code is not None:
                tail = server.output()[-15:]
                print(f'mower_nav exited with {code}' + ('; its last lines:' if tail else ', no output'), flush=True)
                for line in tail:
                    print(f'  {line}', flush=True)
    finally:
        server.stop()
        tmp.cleanup()
        executor.shutdown()
        # a spin thread still inside rcl at interpreter exit aborts the
        # process ("terminate called without an active exception", exit 134)
        spin.join(5.0)
        rclpy.try_shutdown()
    if check.failures:
        print(f'{len(check.failures)} expectation(s) failed', flush=True)
        sys.exit(1)
    print('all expectations met', flush=True)


if __name__ == '__main__':
    main()
