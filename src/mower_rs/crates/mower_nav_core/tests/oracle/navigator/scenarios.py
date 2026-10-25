#!/usr/bin/env python3
"""Runs scenarios.json against the real nav2 navigation servers (Jazzy
1.3.13) and writes what happened as JSON (tests/data/navigator.json).

Starts controller_server, planner_server, behavior_server and bt_navigator
with mower_nav2's nav2_no_map_params.yaml, configures and activates them, and
plays each scenario on the wall clock: it publishes the maps on
/map_grid_global (the global costmap's static layer) and /map_grid (the local
one's), map -> odom (static) and odom -> base_footprint plus /odom at 50 Hz
from a kinematic robot that follows the last /cmd_vel, sends and cancels
NavigateToPose goals at the scenario's times, and records every goal's result
(status, error code, time), the last feedback, the compute_path_to_pose and
follow_path goals the tree sent (first seen, how and when they ended), the
plans on /plan, when the two
costmaps updated (their publications' stamps) and the robot's final pose.
A goal names the tree to run in `behavior_tree`: the port's
navigate_to_pose_w_replanning_and_clearing.xml next to this file, or nav2's
default (empty). Between scenarios the robot is moved back and nothing runs.

  python3 scenarios.py <nav2_no_map_params.yaml> <scenarios.json> <out.json>

ORACLE_ONLY=name,name runs only those scenarios (for debugging).
"""

import json
import math
import os
import subprocess
import sys
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import TransformStamped, TwistStamped
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

STATUS = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_CANCELED: 'canceled', GoalStatus.STATUS_ABORTED: 'aborted'}
SIM_HZ = 50.0
SERVERS = [
    ('controller_server', 'nav2_controller'),
    ('planner_server', 'nav2_planner'),
    ('behavior_server', 'nav2_behaviors'),
    ('bt_navigator', 'nav2_bt_navigator'),
]
CLEARING_XML = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'navigate_to_pose_w_replanning_and_clearing.xml')


def quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def compose(t, p):
    """t (x, y, yaw) applied to pose p."""
    c, s = math.cos(t[2]), math.sin(t[2])
    return (c * p[0] - s * p[1] + t[0], s * p[0] + c * p[1] + t[1], t[2] + p[2])


def inverse(t):
    c, s = math.cos(t[2]), math.sin(t[2])
    return (-(c * t[0] + s * t[1]), s * t[0] - c * t[1], -t[2])


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Harness(Node):
    def __init__(self, map_from_odom):
        super().__init__('navigator_scenarios')
        self.map_from_odom = map_from_odom
        self.lock = threading.Lock()
        self.pose = (0.0, 0.0, 0.0)  # base in odom
        self.twist = (0.0, 0.0)
        self.kinematic = False
        self.t0 = None
        self.t0_ros = 0.0
        self.last_step = None
        self.rec = None
        # every compute_path_to_pose / follow_path goal ever seen: first seen,
        # terminal status and when (monotonic); finished goals stay in the
        # status list for a while, so a scenario counts the goals first seen
        # after it started
        self.goal_log = {'follow_goals': {}, 'compute_goals': {}}
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.global_map_pub = self.create_publisher(OccupancyGrid, '/map_grid_global', latched)
        self.local_map_pub = self.create_publisher(OccupancyGrid, '/map_grid', latched)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        for topic, key in (('/follow_path/_action/status', 'follow_goals'), ('/compute_path_to_pose/_action/status', 'compute_goals')):
            self.create_subscription(GoalStatusArray, topic, lambda m, k=key: self.on_status(k, m), latched)
        for topic, key in (('/global_costmap/costmap', 'global_updates'), ('/local_costmap/costmap', 'local_updates')):
            self.create_subscription(OccupancyGrid, topic, lambda m, k=key: self.on_costmap(k, m), latched)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.lifecycle = {name: self.create_client(ChangeState, f'/{name}/change_state') for name, _ in SERVERS}
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = 'map', 'odom'
        t.transform.translation.x, t.transform.translation.y = map_from_odom[0], map_from_odom[1]
        _, _, t.transform.rotation.z, t.transform.rotation.w = quat(map_from_odom[2])
        self.static_tf.sendTransform(t)
        self.create_timer(1.0 / SIM_HZ, self.step)

    def now_s(self):
        return time.monotonic() - self.t0 if self.t0 is not None else -1.0

    def ros_rel(self, stamp):
        return round(stamp_s(stamp) - self.t0_ros, 4)

    def on_cmd(self, msg):
        with self.lock:
            self.twist = (msg.twist.linear.x, msg.twist.angular.z)
            if self.rec is not None:
                self.rec['cmds'] += 1

    def on_plan(self, msg):
        with self.lock:
            if self.rec is not None:
                self.rec['plans'].append(self.ros_rel(msg.header.stamp))

    def on_status(self, key, msg):
        now = time.monotonic()
        with self.lock:
            log = self.goal_log[key]
            for s in msg.status_list:
                g = log.setdefault(bytes(s.goal_info.goal_id.uuid), {'seen': now, 'status': None, 'end': None})
                if s.status in STATUS and g['status'] is None:
                    g['status'], g['end'] = STATUS[s.status], now

    def on_costmap(self, key, msg):
        with self.lock:
            if self.rec is not None:
                self.rec[key].append(self.ros_rel(msg.header.stamp))

    def start_clock(self):
        """Scenario t = 0, on the monotonic and the ROS (system) clock."""
        self.t0 = time.monotonic()
        self.t0_ros = self.get_clock().now().nanoseconds * 1e-9

    def step(self):
        now = time.monotonic()
        with self.lock:
            x, y, yaw = self.pose
            # integrate over the time that really passed: a late timer must
            # not slow the robot down
            dt = min(now - self.last_step, 0.2) if self.last_step is not None else 0.0
            self.last_step = now
            if self.kinematic:
                v, w = self.twist
                x, y, yaw = x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt, yaw + w * dt
                self.pose = (x, y, yaw)
            v, w = self.twist if self.kinematic else (0.0, 0.0)
        stamp = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, 'odom', 'base_footprint'
        t.transform.translation.x, t.transform.translation.y = x, y
        _, _, t.transform.rotation.z, t.transform.rotation.w = quat(yaw)
        self.tf.sendTransform(t)
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, 'odom', 'base_footprint'
        o.pose.pose.position.x, o.pose.pose.position.y = x, y
        _, _, o.pose.pose.orientation.z, o.pose.pose.orientation.w = quat(yaw)
        o.twist.twist.linear.x, o.twist.twist.angular.z = v, w
        self.odom_pub.publish(o)

    def publish_map(self, pub, spec):
        g = OccupancyGrid()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.info.width, g.info.height, g.info.resolution = spec['width'], spec['height'], spec['resolution']
        g.info.origin.position.x, g.info.origin.position.y = spec['origin']
        g.info.origin.orientation.w = 1.0
        g.data = grid(spec)
        pub.publish(g)


def grid(spec):
    """The map's cells: 0, or 100 inside `lethal_x` [x0, x1) or the square
    `ring` [cx, cy, r_in, r_out] (r_in <= Chebyshev distance < r_out)."""
    w, h, res = spec['width'], spec['height'], spec['resolution']
    ox, oy = spec['origin']
    data = [0] * (w * h)
    for j in range(h):
        for i in range(w):
            x, y = ox + (i + 0.5) * res, oy + (j + 0.5) * res
            lethal = False
            if 'lethal_x' in spec:
                lethal |= spec['lethal_x'][0] <= x < spec['lethal_x'][1]
            if 'ring' in spec:
                cx, cy, r_in, r_out = spec['ring']
                d = max(abs(x - cx), abs(y - cy))
                lethal |= r_in <= d < r_out
            if lethal:
                data[j * w + i] = 100
    return data


def wait(fut, timeout=10.0):
    end = time.monotonic() + timeout
    while not fut.done():
        if time.monotonic() > end:
            raise TimeoutError('future did not complete')
        time.sleep(0.002)
    return fut.result()


def lifecycle(h, name, transition):
    req = ChangeState.Request()
    req.transition.id = transition
    if not wait(h.lifecycle[name].call_async(req), timeout=30.0).success:
        raise RuntimeError(f'{name}: lifecycle transition {transition} failed')


def run_scenario(h, sc, maps, tree):
    h.publish_map(h.global_map_pub, maps[sc['global']])
    h.publish_map(h.local_map_pub, maps[sc['local']])
    with h.lock:
        h.kinematic = False
        h.twist = (0.0, 0.0)
        h.pose = compose(inverse(h.map_from_odom), sc['start'])
        h.t0 = None
    time.sleep(2.5)  # the 1 Hz global and 2 Hz local costmaps take the maps and the pose
    goals = []
    with h.lock:
        h.rec = {'cmds': 0, 'plans': [], 'global_updates': [], 'local_updates': []}
        h.kinematic = sc['robot'] == 'kinematic'
        h.start_clock()
    for ev in sorted(sc['events'], key=lambda e: e['t']):
        delay = ev['t'] - h.now_s()
        if delay > 0:
            time.sleep(delay)
        if ev['op'] == 'send':
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.pose.position.x, goal.pose.pose.position.y = ev['pose'][0], ev['pose'][1]
            _, _, goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = quat(ev['pose'][2])
            goal.behavior_tree = CLEARING_XML if tree == 'clearing' else ''
            rec = {'sent_t': round(h.now_s(), 4), 'status': None, 'error_code': None, 'result_t': None, 'feedback': None}

            def feedback(msg, rec=rec):
                f = msg.feedback
                rec['feedback'] = {'number_of_recoveries': f.number_of_recoveries, 'distance_remaining': round(f.distance_remaining, 4)}
            gh = wait(h.client.send_goal_async(goal, feedback_callback=feedback))
            rec['accepted'] = gh.accepted

            def done(fut, rec=rec):
                r = fut.result()
                rec['result_t'] = round(h.now_s(), 4)
                rec['status'] = STATUS.get(r.status, r.status)
                rec['error_code'] = r.result.error_code
                rec['error_msg'] = r.result.error_msg
            gh.get_result_async().add_done_callback(done)
            goals.append((gh, rec))
        elif ev['op'] == 'cancel':
            gh, rec = goals[ev['goal']]
            resp = wait(gh.cancel_goal_async())
            rec['cancel_accepted'] = len(resp.goals_canceling) > 0
    # run to the duration, or until every goal has its result
    while h.now_s() < sc['duration'] and any(rec['status'] is None for _, rec in goals):
        time.sleep(0.01)
    time.sleep(0.5)  # the last commands and status messages
    for gh, rec in goals:
        if rec['status'] is None:
            wait(gh.cancel_goal_async())
    end = time.monotonic() + 5.0
    while any(rec['status'] is None for _, rec in goals) and time.monotonic() < end:
        time.sleep(0.01)
    with h.lock:
        h.kinematic = False
        final = compose(h.map_from_odom, h.pose)
        r = h.rec
        h.rec = None
        timeline = {}
        for key, log in h.goal_log.items():
            mine = sorted((g for g in log.values() if g['seen'] >= h.t0), key=lambda g: g['seen'])
            timeline[key] = [[round(g['seen'] - h.t0, 3), g['status'], round(g['end'] - h.t0, 3) if g['end'] else None] for g in mine]
        h.t0 = None
    return {
        'goals': [rec for _, rec in goals],
        'final_pose': [round(v, 5) for v in final],
        'cmds': r['cmds'],
        'plans': r['plans'],
        'follow_goals': timeline['follow_goals'],
        'compute_goals': timeline['compute_goals'],
        'global_updates': r['global_updates'],
        'local_updates': r['local_updates'],
    }


def main():
    params, scenarios_file, out_file = sys.argv[1:4]
    spec = json.load(open(scenarios_file))
    # the binaries themselves: terminating `ros2 run` leaves its child running
    servers = [
        subprocess.Popen(
            [f'/opt/ros/jazzy/lib/{pkg}/{name}', '--ros-args', '--params-file', params],
            stdout=subprocess.DEVNULL, stderr=open(f'/tmp/navigator_oracle_{name}.log', 'w'))
        for name, pkg in SERVERS
    ]
    rclpy.init()
    h = Harness(tuple(spec['map_from_odom']))
    # rclpy's MultiThreadedExecutor starved the 50 Hz robot (controller_server oracle)
    executor = SingleThreadedExecutor()
    executor.add_node(h)
    threading.Thread(target=executor.spin, daemon=True).start()
    out = {'nav2': '1.3.13', 'trees': {}}
    try:
        for name, _ in SERVERS:
            if not h.lifecycle[name].wait_for_service(timeout_sec=20.0):
                raise RuntimeError(f'{name} did not come up')
        h.publish_map(h.global_map_pub, spec['maps']['empty'])
        h.publish_map(h.local_map_pub, spec['maps']['empty'])
        for name, _ in SERVERS:
            lifecycle(h, name, 1)  # configure
        for name, _ in SERVERS:
            lifecycle(h, name, 3)  # activate
        if not h.client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('no /navigate_to_pose')
        for tree in ('clearing', 'default'):
            out['trees'][tree] = {}
            only = os.environ.get('ORACLE_ONLY')  # name,name: a subset, for debugging
            for sc in spec['scenarios']:
                if tree not in sc['trees'] or (only and sc['name'] not in only.split(',')):
                    continue
                res = run_scenario(h, sc, spec['maps'], tree)
                out['trees'][tree][sc['name']] = res
                print(tree, sc['name'], [(g['status'], g['error_code'], g['result_t'], g['feedback']) for g in res['goals']],
                      'plans', len(res['plans']), 'compute', len(res['compute_goals']), 'follow', len(res['follow_goals']),
                      'end', res['final_pose'], flush=True)
    finally:
        for p in servers:
            p.terminate()
        for p in servers:
            p.wait(timeout=10)
        executor.shutdown()
        rclpy.shutdown()
    with open(out_file, 'w') as f:
        json.dump(out, f, indent=1)
        f.write('\n')


if __name__ == '__main__':
    main()
