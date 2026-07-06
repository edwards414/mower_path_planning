# Copyright 2024 fxrbindi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Graph-state recorder (the "② graph plane").

rosbag2 records topics, NOT the node/service graph. This node periodically
snapshots the live graph — nodes, topics (with publisher/subscriber counts),
services — and publishes it as JSON on /graph_snapshot so it lands in the bag.

It also diffs node presence between ticks and logs a WARN whenever a node
disappears: exactly the failure mode that made map_manage_node's silent death
so hard to debug. If a *watched* node vanishes it can additionally raise a fault
(Bool on fault_topic) so the recorder flushes its snapshot buffer.
"""
import json

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String


def _full_name(name, namespace):
    if namespace == '/':
        return '/' + name
    return namespace.rstrip('/') + '/' + name


class GraphSnapshot(Node):
    def __init__(self):
        super().__init__('graph_snapshot')
        self.declare_parameter('period_s', 2.0)
        # Nodes whose disappearance is treated as a fault (empty = none).
        self.declare_parameter('watch_nodes', [''])
        self.declare_parameter('fault_topic', '/mower_recorder/fault')

        period = float(self.get_parameter('period_s').value)
        self._watch = {w for w in self.get_parameter('watch_nodes').value if w}

        self._snap_pub = self.create_publisher(String, '/graph_snapshot', 10)
        self._event_pub = self.create_publisher(String, '/graph_events', 10)
        self._fault_pub = self.create_publisher(
            Bool, self.get_parameter('fault_topic').value, 10)

        self._prev_nodes = set()
        self._timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f'graph_snapshot up (period={period}s, watch={sorted(self._watch)})')

    def _tick(self):
        node_set = {
            _full_name(n, ns)
            for n, ns in self.get_node_names_and_namespaces()
        }
        topics = [
            {
                'name': name,
                'types': types,
                'pubs': self.count_publishers(name),
                'subs': self.count_subscribers(name),
            }
            for name, types in self.get_topic_names_and_types()
        ]
        services = [
            {'name': name, 'types': types}
            for name, types in self.get_service_names_and_types()
        ]

        stamp = self.get_clock().now().to_msg()
        snap = {
            'stamp': {'sec': stamp.sec, 'nanosec': stamp.nanosec},
            'nodes': sorted(node_set),
            'topics': topics,
            'services': services,
        }
        msg = String()
        msg.data = json.dumps(snap, ensure_ascii=False)
        self._snap_pub.publish(msg)

        # Diff node presence -> log + events + optional fault.
        if self._prev_nodes:
            for gone in sorted(self._prev_nodes - node_set):
                self.get_logger().warn(f'NODE DISAPPEARED: {gone}')
                self._emit_event('node_disappeared', gone)
                if gone in self._watch:
                    self.get_logger().error(f'WATCHED NODE DOWN: {gone} -> fault')
                    self._fault_pub.publish(Bool(data=True))
            for appeared in sorted(node_set - self._prev_nodes):
                self._emit_event('node_appeared', appeared)
        self._prev_nodes = node_set

    def _emit_event(self, kind, target):
        m = String()
        m.data = json.dumps({'event': kind, 'target': target}, ensure_ascii=False)
        self._event_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = GraphSnapshot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
