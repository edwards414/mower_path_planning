#!/usr/bin/env python3
"""The outdoor harness's DDS recorder (I5): runs in the observer sidecar (the
sim's network and PID namespaces, unthrottled) and writes one JSON line per
message with its wall-clock receipt time. Development only.

    observe.py --set teleop|mission|loc --out /tmp/obs/x.jsonl [--seconds N]

Each line: {"t": receipt (time.time()), "k": topic, ...fields}. The fields
depend on the topic's kind: twist (stamp ns, linear.x, angular.z), header,
odom (x, y, yaw, vx, wz), fix (status, lat, lon, cov), imu (yaw, wz),
pose, json (a std_msgs/String holding JSON) and raw (message_to_ordereddict,
low-rate topics only). Types come from the graph, so a topic that appears
late is still recorded. SIGTERM/SIGINT end it cleanly.
"""
import argparse
import json
import math
import os
import signal
import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

SETS = {
    'teleop': [('/app_joy_cmd', 'twist'), ('/joy_cmd', 'twist'), ('/cmd_vel_guard_input', 'twist'),
               ('/drivetrain_guarded_cmd_vel', 'twist'), ('/manual_command_clock', 'header'), ('/odom', 'odom')],
    'mission': [('/odometry/global', 'odom'), ('/odom', 'odom'), ('/fix', 'fix'), ('/imu/data', 'imu'),
                ('/drivetrain_guarded_cmd_vel', 'twist'), ('/nav_cmd_vel', 'twist'),
                ('/navigation_safety_stop', 'twist'), ('/cmd_vel_guard_input', 'twist'),
                ('/coverage_progress', 'raw'), ('/zone_sequence_status', 'json'),
                ('/adapter/map_datum', 'json_latched'), ('/adapter/robot_pose', 'pose'),
                ('/adapter/marker_layers/coverage_path', 'json_latched')],
    'loc': [('/odometry/global', 'odom'), ('/odometry/local', 'odom'), ('/odom', 'odom'), ('/fix', 'fix'),
            ('/imu/data', 'imu'), ('/adapter/map_datum', 'json_latched'), ('/adapter/robot_pose', 'pose'),
            ('/gps/status', 'json'), ('/drivetrain_guarded_cmd_vel', 'twist')],
}
BE = QoSProfile(depth=200, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE)
LATCHED = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def stamp_ns(m):
    h = getattr(m, 'header', None)
    if h is None:
        return None
    return h.stamp.sec * 1_000_000_000 + h.stamp.nanosec


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def fin(v, n=6):
    return round(v, n) if math.isfinite(v) else str(v)


def extract(kind, m):
    if kind == 'twist':
        tw = m.twist if hasattr(m, 'twist') else m
        d = {'s': stamp_ns(m), 'lx': fin(tw.linear.x), 'az': fin(tw.angular.z)}
        if hasattr(m, 'header'):
            d['f'] = m.header.frame_id[-8:]
        return d
    if kind == 'header':
        return {'s': m.stamp.sec * 1_000_000_000 + m.stamp.nanosec, 'f': m.frame_id[-8:]}
    if kind == 'odom':
        p, q, tw = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
        return {'s': stamp_ns(m), 'x': fin(p.x, 4), 'y': fin(p.y, 4), 'yaw': fin(yaw(q), 5),
                'vx': fin(tw.linear.x, 4), 'wz': fin(tw.angular.z, 4)}
    if kind == 'fix':
        return {'s': stamp_ns(m), 'st': m.status.status, 'lat': fin(m.latitude, 9), 'lon': fin(m.longitude, 9),
                'cov': fin(m.position_covariance[0], 6)}
    if kind == 'imu':
        return {'s': stamp_ns(m), 'yaw': fin(yaw(m.orientation), 5), 'wz': fin(m.angular_velocity.z, 4)}
    if kind == 'pose':
        p, q = m.pose.position, m.pose.orientation
        return {'s': stamp_ns(m), 'x': fin(p.x, 4), 'y': fin(p.y, 4), 'yaw': fin(yaw(q), 5)}
    if kind in ('json', 'json_latched'):
        try:
            return {'j': json.loads(m.data)}
        except ValueError:
            return {'raw': m.data[:500]}
    return {'m': json.loads(json.dumps(message_to_ordereddict(m), default=str))}


class Observer(Node):
    def __init__(self, topics, out):
        super().__init__('outdoor_observer')
        self.want = dict(topics)
        self.subs = {}
        self.f = open(out, 'a', buffering=1 << 16)
        self.n = {}
        self.create_timer(1.0, self.discover)
        self.create_timer(1.0, self.f.flush)
        self.discover()

    def discover(self):
        types = dict(self.get_topic_names_and_types())
        for topic, kind in self.want.items():
            if topic in self.subs or topic not in types:
                continue
            msg = get_message(types[topic][0])
            qos = LATCHED if kind.endswith('_latched') else BE
            self.subs[topic] = self.create_subscription(
                msg, topic, (lambda t, k: lambda m: self.on(t, k, m))(topic, kind), qos)
            self.write({'t': time.time(), 'k': '_sub', 'topic': topic, 'type': types[topic][0]})

    def on(self, topic, kind, m):
        d = {'t': round(time.time(), 4), 'k': topic}
        d.update(extract(kind, m))
        self.n[topic] = self.n.get(topic, 0) + 1
        self.write(d)

    def write(self, d):
        self.f.write(json.dumps(d, separators=(',', ':')) + '\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--set', required=True, choices=sorted(SETS))
    ap.add_argument('--out', required=True)
    ap.add_argument('--seconds', type=float, default=3600.0)
    ap.add_argument('--extra', default='', help='more topic:kind pairs, comma separated')
    args = ap.parse_args()
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    topics = list(SETS[args.set])
    for pair in filter(None, args.extra.split(',')):
        t, k = pair.rsplit(':', 1)
        topics.append((t, k))
    rclpy.init()
    node = Observer(topics, args.out)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    stop = {'v': False}
    signal.signal(signal.SIGTERM, lambda *_: stop.update(v=True))
    signal.signal(signal.SIGINT, lambda *_: stop.update(v=True))
    end = time.monotonic() + args.seconds
    while not stop['v'] and time.monotonic() < end:
        ex.spin_once(timeout_sec=0.1)
    node.write({'t': time.time(), 'k': '_end', 'counts': node.n})
    node.f.flush()
    node.f.close()
    print('observe:', json.dumps(node.n), file=sys.stderr)
    os._exit(0)


if __name__ == '__main__':
    main()
