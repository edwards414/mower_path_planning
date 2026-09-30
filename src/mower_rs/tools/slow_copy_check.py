#!/usr/bin/env python3
"""Check a throttled copy against its source topic.

`mower_base` publishes `/odom_slow` and `mower_localize`'s map EKF publishes
`/odometry/global_slow` themselves (`mower_rs_common::throttle`) instead of
the two `topic_tools throttle` processes in `mission.launch.py`. This records
a source topic and one or more slow copies of it for a while and reports, per
copy:

* the rate, and the spacing between forwarded messages;
* how many copies are byte-for-byte a message seen on the source (the
  serialized CDR, so the identical `Odometry`, stamp and all, not a
  re-stamped one);
* the publishers of the copy and of the source with their QoS, as
  `ros2 topic info -v` would print them.

Run a C++ throttle on the same source next to the module to compare the two
on one stream::

    ros2 run topic_tools throttle messages /odom 5.0 /odom_slow_cpp &
    slow_copy_check.py --seconds 30 --pair /odom:/odom_slow \\
        --pair /odom:/odom_slow_cpp --out /tmp/odom_slow.json

Exit status 1 when a copy received nothing or carried a message its source
never did. Nothing here runs on the robot.
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

# Reliable + volatile matches every publisher involved: a volatile reader
# accepts a transient-local writer, and all of them are reliable.
READER_QOS = QoSProfile(depth=200, reliability=QoSReliabilityPolicy.RELIABLE)


def _qos(endpoint):
    q = endpoint.qos_profile
    return {
        'node': f'{endpoint.node_namespace.rstrip("/")}/{endpoint.node_name}',
        'reliability': q.reliability.name,
        'durability': q.durability.name,
        'history': q.history.name,
        'depth': q.depth,
        'liveliness': q.liveliness.name,
    }


class Recorder(Node):
    def __init__(self, pairs, msg_type):
        super().__init__('slow_copy_check')
        self.pairs = pairs
        self.lock = threading.Lock()
        self.recording = False
        self.arrivals = {}
        cls = get_message(msg_type)
        for topic in sorted({t for pair in pairs for t in pair}):
            self.arrivals[topic] = []
            self.create_subscription(
                cls, topic, lambda raw, t=topic: self._on(t, raw), READER_QOS,
                raw=True)

    def _on(self, topic, raw):
        now = time.monotonic()
        with self.lock:
            if self.recording:
                self.arrivals[topic].append((now, bytes(raw)))

    def endpoints(self):
        return {
            topic: [_qos(e) for e in self.get_publishers_info_by_topic(topic)]
            for topic in self.arrivals
        }


def _spacing_ms(times):
    gaps = [(b - a) * 1e3 for a, b in zip(times, times[1:])]
    if not gaps:
        return None
    return {
        'min': round(min(gaps), 1),
        'median': round(statistics.median(gaps), 1),
        'max': round(max(gaps), 1),
    }


def _rate(times):
    if len(times) < 2:
        return 0.0
    return (len(times) - 1) / (times[-1] - times[0])


def report(node, seconds):
    out = {'seconds': seconds, 'endpoints': node.endpoints(), 'topics': {},
           'pairs': []}
    for topic, rows in node.arrivals.items():
        times = [t for t, _ in rows]
        out['topics'][topic] = {
            'count': len(rows),
            'rate_hz': round(_rate(times), 3),
            'spacing_ms': _spacing_ms(times),
        }
    ok = True
    for source, slow in node.pairs:
        seen = {raw for _, raw in node.arrivals[source]}
        rows = node.arrivals[slow]
        # The copy of a message can overtake the original on the reader, so
        # compare against everything the source carried in the whole run.
        identical = sum(1 for _, raw in rows if raw in seen)
        pair_ok = bool(rows) and identical == len(rows)
        ok = ok and pair_ok
        out['pairs'].append({
            'source': source,
            'slow': slow,
            'slow_count': len(rows),
            'identical_to_a_source_message': identical,
            'ratio': round(len(rows) / max(1, len(node.arrivals[source])), 4),
            'ok': pair_ok,
        })
    # Two copies of one source (the module's and a C++ throttle's): how many
    # messages they both forwarded. They cannot agree on every one, because
    # each measures the period from its own arrival times.
    out['overlap'] = []
    for i, (source, a) in enumerate(node.pairs):
        for other_source, b in node.pairs[i + 1:]:
            if other_source != source:
                continue
            common = ({raw for _, raw in node.arrivals[a]}
                      & {raw for _, raw in node.arrivals[b]})
            out['overlap'].append({'a': a, 'b': b, 'both': len(common)})
    out['ok'] = ok
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n', 1)[0])
    ap.add_argument('--pair', action='append', required=True,
                    metavar='SOURCE:SLOW',
                    help='a source topic and a throttled copy of it')
    ap.add_argument('--type', default='nav_msgs/msg/Odometry')
    ap.add_argument('--seconds', type=float, default=30.0)
    ap.add_argument('--settle', type=float, default=3.0,
                    help='discovery time before recording starts')
    ap.add_argument('--out')
    args = ap.parse_args()
    pairs = []
    for item in args.pair:
        source, _, slow = item.partition(':')
        if not source or not slow:
            ap.error(f'--pair {item!r}: expected SOURCE:SLOW')
        pairs.append((source, slow))

    rclpy.init()
    node = Recorder(pairs, args.type)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    time.sleep(args.settle)
    with node.lock:
        node.recording = True
    time.sleep(args.seconds)
    with node.lock:
        node.recording = False
    doc = report(node, args.seconds)
    text = json.dumps(doc, indent=2, sort_keys=True)
    print(text)
    if args.out:
        with open(args.out, 'w') as f:
            f.write(text + '\n')
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    return 0 if doc['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
