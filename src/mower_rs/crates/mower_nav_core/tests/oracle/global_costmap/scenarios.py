#!/usr/bin/env python3
"""Records the real nav2 global costmap and planner (Jazzy 1.3.13) on the maps
of scenarios.json and writes them as JSON (tests/data/global_costmap.json).

Starts `planner_server` (which owns the global costmap) with mower_nav2's
nav2_no_map_params.yaml, so with the plugins, `robot_radius` and inflation
parameters of that file, and for each case:

  * publishes the map on /map_grid_global (the static layer's topic);
  * records the master grid the node publishes on /global_costmap/costmap_raw
    (`nav2_msgs/Costmap`: the raw cost of every cell);
  * waits until the update after the one that took the map is over (it
    repaints the master grid once more), then sends the case's
    ComputePathToPose goals one after the other, each with its own start pose
    (`use_start`), and records status, error code and every pose of the path;
  * records the cells of the master grid that differ several update periods
    later (`changed_after_plans`, [index, cost]): what NavFn's clearRobotCell
    wrote into it and no update repainted.

Nothing here depends on time: the same maps and goals give the same file.

  python3 scenarios.py <nav2_no_map_params.yaml> <scenarios.json> <out.json>
"""

import json
import math
import os
import random
import subprocess
import sys
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import StaticTransformBroadcaster

STATUS = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_CANCELED: 'canceled', GoalStatus.STATUS_ABORTED: 'aborted'}
HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.normpath(os.path.join(HERE, '../../data'))
# after the plans: three 1 Hz updates that could have repainted the master grid
SETTLE_S = 3.5
# after a new map: the update that takes it and the one after repaint
REPAINT_S = 2.5


def load_fixture(name):
    """A run-length fixture of tests/data: cells 100 / 0, starting with 100."""
    f = json.load(open(os.path.join(DATA, name)))
    data, value = [], 100
    for n in f['runs']:
        data.extend([value] * n)
        value = 100 - value
    assert len(data) == f['width'] * f['height'], name
    return {'width': f['width'], 'height': f['height'], 'resolution': f['resolution'],
            'origin_x': f['origin_x'], 'origin_y': f['origin_y'], 'data': data}


def random_grid(spec):
    """Free space with rectangles of lethal and of unknown and a sprinkle of
    the odd values (99 stays free, -2 and 101 read as >= 100), inside a lethal
    border where `border` says so."""
    rng = random.Random(spec['seed'])
    w, h = spec['width'], spec['height']
    data = [0] * (w * h)

    def rect(v, most):
        x0, y0, sx, sy = rng.randrange(w), rng.randrange(h), rng.randint(1, most), rng.randint(1, most)
        for y in range(y0, min(h, y0 + sy)):
            for x in range(x0, min(w, x0 + sx)):
                data[y * w + x] = v
    for _ in range(spec['lethal_rects']):
        rect(100, 8)
    for _ in range(spec['unknown_rects']):
        rect(-1, 10)
    for _ in range(spec['odd_cells']):
        data[rng.randrange(h) * w + rng.randrange(w)] = rng.choice([30, 99, -2, 101, 100, -1])
    if spec.get('border'):
        for y in range(h):
            for x in range(w):
                if x == 0 or y == 0 or x == w - 1 or y == h - 1:
                    data[y * w + x] = 100
    return {'width': w, 'height': h, 'resolution': spec['resolution'],
            'origin_x': spec['origin'][0], 'origin_y': spec['origin'][1], 'data': data}


class Harness(Node):
    def __init__(self):
        super().__init__('global_costmap_scenarios')
        self.lock = threading.Lock()
        self.costmaps = []  # (monotonic time received, message)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map_grid_global', latched)
        self.create_subscription(Costmap, '/global_costmap/costmap_raw', self.on_costmap, latched)
        self.client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.lifecycle = self.create_client(ChangeState, '/planner_server/change_state')
        # the costmap only needs some robot pose to update; every goal brings its own start
        self.static_tf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = 'map', 'base_footprint'
        t.transform.rotation.w = 1.0
        self.static_tf.sendTransform(t)

    def on_costmap(self, msg):
        with self.lock:
            self.costmaps.append((time.monotonic(), msg))
            del self.costmaps[:-4]

    def publish_map(self, grid):
        g = OccupancyGrid()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.info.width, g.info.height, g.info.resolution = grid['width'], grid['height'], grid['resolution']
        g.info.origin.position.x, g.info.origin.position.y = grid['origin_x'], grid['origin_y']
        g.info.origin.orientation.w = 1.0
        g.data = grid['data']
        self.map_pub.publish(g)

    def costmap_after(self, delay, grid=None):
        """The first master grid published at least `delay` s from now; with
        `grid`, the first that has taken that map (its size and its lethal
        cells), however long the 1 Hz update and the 1 Hz publisher take."""
        lethal = [v >= 100 or v < -1 for v in grid['data']] if grid else None
        after = time.monotonic() + delay
        end = after + 20.0
        while time.monotonic() < end:
            with self.lock:
                late = [m for t, m in self.costmaps if t >= after]
            for m in late:
                if lethal is not None and (m.metadata.size_x != grid['width'] or m.metadata.size_y != grid['height']
                                           or [c == 254 for c in m.data] != lethal):
                    continue
                return {'size_x': m.metadata.size_x, 'size_y': m.metadata.size_y, 'resolution': m.metadata.resolution,
                        'origin_x': m.metadata.origin.position.x, 'origin_y': m.metadata.origin.position.y,
                        'data': list(m.data)}
            time.sleep(0.02)
        raise TimeoutError('no /global_costmap/costmap_raw')


def wait(fut, timeout=30.0):
    end = time.monotonic() + timeout
    while not fut.done():
        if time.monotonic() > end:
            raise TimeoutError('future did not complete')
        time.sleep(0.002)
    return fut.result()


def pose_stamped(p):
    m = PoseStamped()
    m.header.frame_id = 'map'
    m.pose.position.x, m.pose.position.y = p[0], p[1]
    m.pose.orientation.z, m.pose.orientation.w = math.sin(p[2] / 2.0), math.cos(p[2] / 2.0)
    return m


def plan(h, start, goal):
    g = ComputePathToPose.Goal()
    g.start, g.goal, g.use_start, g.planner_id = pose_stamped(start), pose_stamped(goal), True, 'GridBased'
    gh = wait(h.client.send_goal_async(g))
    if not gh.accepted:
        return {'start': start, 'goal': goal, 'status': 'rejected', 'error_code': None, 'poses': []}
    r = wait(gh.get_result_async())
    poses = [[p.pose.position.x, p.pose.position.y, p.pose.orientation.z, p.pose.orientation.w] for p in r.result.path.poses]
    return {'start': start, 'goal': goal, 'status': STATUS.get(r.status, r.status), 'error_code': r.result.error_code, 'poses': poses}


def lifecycle(h, transition):
    req = ChangeState.Request()
    req.transition.id = transition
    if not wait(h.lifecycle.call_async(req)).success:
        raise RuntimeError(f'planner_server: lifecycle transition {transition} failed')


def main():
    params, scenarios_file, out_file = sys.argv[1:4]
    spec = json.load(open(scenarios_file))
    # the binary itself: terminating `ros2 run` leaves its child running
    server = subprocess.Popen(['/opt/ros/jazzy/lib/nav2_planner/planner_server', '--ros-args', '--params-file', params],
                              stdout=subprocess.DEVNULL, stderr=open('/tmp/global_costmap_oracle_planner_server.log', 'w'))
    rclpy.init()
    h = Harness()
    executor = SingleThreadedExecutor()
    executor.add_node(h)
    threading.Thread(target=executor.spin, daemon=True).start()
    out = {'nav2': '1.3.13', 'cases': []}
    try:
        if not h.lifecycle.wait_for_service(timeout_sec=20.0):
            raise RuntimeError('planner_server did not come up')
        grids = [load_fixture(c['fixture']) if 'fixture' in c else random_grid(c['random']) for c in spec['cases']]
        h.publish_map(grids[0])
        lifecycle(h, 1)  # configure
        lifecycle(h, 3)  # activate
        if not h.client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('no /compute_path_to_pose')
        for k, (case, grid) in enumerate(zip(spec['cases'], grids)):
            # the first map is already there; publishing it again would have
            # the costmap repaint at some update during the plans
            if k > 0:
                h.publish_map(grid)
            res = {'name': case['name']}
            if 'fixture' in case:
                res['fixture'] = case['fixture']
            else:
                res['grid'] = grid
            res['costmap'] = h.costmap_after(0.0, grid)
            # The update after the one that took a new map repaints the
            # master grid once more (the inflation layer's bounds), and so
            # paints over what a plan cleared in between: wait it out, or the
            # cells left changed would depend on when the plans ran.
            time.sleep(REPAINT_S)
            res['plans'] = [plan(h, p['start'], p['goal']) for p in case['plans']]
            after = h.costmap_after(SETTLE_S)['data']
            res['changed_after_plans'] = [[i, v] for i, (u, v) in enumerate(zip(res['costmap']['data'], after)) if u != v]
            out['cases'].append(res)
            print(case['name'], f"{grid['width']} x {grid['height']}",
                  [(p['status'], p['error_code'], len(p['poses'])) for p in res['plans']], flush=True)
        print('topics', sorted(n for n, _ in h.get_topic_names_and_types() if n.startswith('/global_costmap')), flush=True)
    finally:
        server.terminate()
        server.wait(timeout=10)
        executor.shutdown()
        rclpy.shutdown()
    with open(out_file, 'w') as f:
        json.dump(out, f, separators=(',', ':'))
        f.write('\n')


if __name__ == '__main__':
    main()
