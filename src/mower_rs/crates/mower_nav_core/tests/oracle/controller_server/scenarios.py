#!/usr/bin/env python3
"""Runs scenarios.json against the real nav2 controller_server (Jazzy 1.3.13)
and writes what happened as JSON (tests/data/controller_server.json).

Starts `controller_server` with mower_nav2's nav2_no_map_params.yaml,
configures and activates it, and plays each scenario on the wall clock: it
publishes the map on /map_grid (the local costmap's static layer), map -> odom
(static) and odom -> base_footprint plus /odom at 50 Hz from a kinematic robot
that follows the last /cmd_vel, sends and cancels FollowPath goals at the
scenario's times, and records every goal's result (status, error code, time),
every /cmd_vel and the robot's final pose. One server runs all scenarios;
between them the robot is moved back and nothing is executing.

  python3 scenarios.py <nav2_no_map_params.yaml> <scenarios.json> <out.json>
"""

import json
import math
import subprocess
import sys
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import FollowPath
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

STATUS = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_CANCELED: 'canceled', GoalStatus.STATUS_ABORTED: 'aborted'}
SIM_HZ = 50.0


def quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def compose(t, p):
    """t (x, y, yaw) applied to pose p."""
    c, s = math.cos(t[2]), math.sin(t[2])
    return (c * p[0] - s * p[1] + t[0], s * p[0] + c * p[1] + t[1], t[2] + p[2])


def inverse(t):
    c, s = math.cos(t[2]), math.sin(t[2])
    return (-(c * t[0] + s * t[1]), s * t[0] - c * t[1], -t[2])


class Harness(Node):
    def __init__(self, map_from_odom):
        super().__init__('controller_server_scenarios')
        self.map_from_odom = map_from_odom
        self.lock = threading.Lock()
        self.pose = (0.0, 0.0, 0.0)  # base in odom
        self.twist = (0.0, 0.0)
        self.kinematic = False
        self.t0 = None
        self.t0_ros = 0.0
        self.last_step = None
        self.cmds = []
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map_grid', latched)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self.on_cmd, 10)
        self.client = ActionClient(self, FollowPath, '/follow_path')
        self.lifecycle = self.create_client(ChangeState, '/controller_server/change_state')
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = 'map', 'odom'
        t.transform.translation.x, t.transform.translation.y = map_from_odom[0], map_from_odom[1]
        _, _, t.transform.rotation.z, t.transform.rotation.w = quat(map_from_odom[2])
        self.static_tf.sendTransform(t)
        self.create_timer(1.0 / SIM_HZ, self.step)

    def now_s(self):
        return time.monotonic() - self.t0 if self.t0 is not None else -1.0

    def on_cmd(self, msg):
        with self.lock:
            if self.t0 is not None:
                # the server's own stamp (its now()), on the scenario clock
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - self.t0_ros
                self.cmds.append([round(stamp, 4), msg.twist.linear.x, msg.twist.angular.z])
            self.twist = (msg.twist.linear.x, msg.twist.angular.z)

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

    def publish_map(self, spec):
        g = OccupancyGrid()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.info.width, g.info.height, g.info.resolution = spec['width'], spec['height'], spec['resolution']
        g.info.origin.position.x, g.info.origin.position.y = spec['origin']
        g.info.origin.orientation.w = 1.0
        data = [0] * (spec['width'] * spec['height'])
        if spec['lethal_x']:
            x0, x1 = spec['lethal_x']
            for j in range(spec['height']):
                for i in range(spec['width']):
                    x = spec['origin'][0] + (i + 0.5) * spec['resolution']
                    if x0 <= x < x1:
                        data[j * spec['width'] + i] = 100
        g.data = data
        self.map_pub.publish(g)


def wait(fut, timeout=10.0):
    end = time.monotonic() + timeout
    while not fut.done():
        if time.monotonic() > end:
            raise TimeoutError('future did not complete')
        time.sleep(0.002)
    return fut.result()


def lifecycle(h, transition):
    req = ChangeState.Request()
    req.transition.id = transition
    if not wait(h.lifecycle.call_async(req)).success:
        raise RuntimeError(f'lifecycle transition {transition} failed')


def make_path(ev):
    p = Path()
    p.header.frame_id = 'map'
    if ev.get('length') is None:
        return p
    x0, y0 = ev['from']
    for i in range(int(round(ev['length'] / 0.1)) + 1):
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.pose.position.x, ps.pose.position.y = x0 + 0.1 * i, y0
        ps.pose.orientation.w = 1.0
        p.poses.append(ps)
    return p


def run_scenario(h, sc, maps):
    h.publish_map(maps[sc['map']])
    with h.lock:
        h.kinematic = False
        h.twist = (0.0, 0.0)
        h.pose = compose(inverse(h.map_from_odom), sc['start'])
        h.cmds = []
        h.t0 = None
    time.sleep(1.5)  # the 2 Hz local costmap catches up with the map and the pose
    goals = []
    with h.lock:
        h.kinematic = sc['robot'] == 'kinematic'
        h.start_clock()
    for ev in sorted(sc['events'], key=lambda e: e['t']):
        delay = ev['t'] - h.now_s()
        if delay > 0:
            time.sleep(delay)
        if ev['op'] == 'send':
            goal = FollowPath.Goal()
            goal.path = make_path(ev)
            goal.controller_id = 'FollowPath'
            goal.goal_checker_id = 'general_goal_checker'
            rec = {'sent_t': round(h.now_s(), 4), 'status': None, 'error_code': None, 'result_t': None}
            gh = wait(h.client.send_goal_async(goal))
            rec['accepted'] = gh.accepted

            def done(fut, rec=rec):
                r = fut.result()
                rec['result_t'] = round(h.now_s(), 4)
                rec['status'] = STATUS.get(r.status, r.status)
                rec['error_code'] = r.result.error_code
            gh.get_result_async().add_done_callback(done)
            goals.append((gh, rec))
        elif ev['op'] == 'cancel':
            gh, rec = goals[ev['goal']]
            resp = wait(gh.cancel_goal_async())
            rec.setdefault('cancel_accepted', []).append(len(resp.goals_canceling) > 0)
    rest = sc['duration'] - h.now_s()
    if rest > 0:
        time.sleep(rest)
    # leave nothing running for the next scenario
    for gh, rec in goals:
        if rec['status'] is None:
            wait(gh.cancel_goal_async())
    end = time.monotonic() + 5.0
    while any(rec['status'] is None for _, rec in goals) and time.monotonic() < end:
        time.sleep(0.01)
    with h.lock:
        h.kinematic = False
        final = compose(h.map_from_odom, h.pose)
        cmds = list(h.cmds)
        h.t0 = None
    return {'goals': [rec for _, rec in goals], 'cmds': cmds, 'final_pose': [round(v, 5) for v in final]}


def main():
    params, scenarios_file, out_file = sys.argv[1:4]
    spec = json.load(open(scenarios_file))
    # the binary itself: terminating `ros2 run` leaves its child running
    server = subprocess.Popen(
        ['/opt/ros/jazzy/lib/nav2_controller/controller_server', '--ros-args', '--params-file', params],
        stdout=subprocess.DEVNULL, stderr=open('/tmp/controller_server_oracle.log', 'w'))
    rclpy.init()
    h = Harness(tuple(spec['map_from_odom']))
    # rclpy's MultiThreadedExecutor starved the 50 Hz robot here
    executor = SingleThreadedExecutor()
    executor.add_node(h)
    threading.Thread(target=executor.spin, daemon=True).start()
    out = {'nav2_controller': '1.3.13', 'scenarios': {}}
    try:
        if not h.lifecycle.wait_for_service(timeout_sec=20.0):
            raise RuntimeError('controller_server did not come up')
        h.publish_map(spec['maps']['empty'])
        lifecycle(h, 1)  # configure
        lifecycle(h, 3)  # activate
        if not h.client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('no /follow_path')
        for sc in spec['scenarios']:
            res = run_scenario(h, sc, spec['maps'])
            out['scenarios'][sc['name']] = res
            print(sc['name'], [(g['status'], g['error_code'], g['result_t']) for g in res['goals']], flush=True)
    finally:
        server.terminate()
        server.wait(timeout=10)
        executor.shutdown()
        rclpy.shutdown()
    with open(out_file, 'w') as f:
        json.dump(out, f, indent=1)
        f.write('\n')


if __name__ == '__main__':
    main()
