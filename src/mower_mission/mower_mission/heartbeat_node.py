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
"""Robot liveness heartbeat (an LWT-style 'is the robot alive' signal).

Publishes ``/robot/online`` (std_msgs/Bool) at a fixed rate. It is True only
while a freshness source (default ``/odom``) has been received within
``stale_timeout_s``. Two layers of detection:

* source goes stale (robot's drivers stopped) -> publishes False;
* this node itself dies (robot down / graph gone) -> the topic stops
  entirely, so a consumer's own receive-timeout also flags the robot offline.

The topic is latched (TRANSIENT_LOCAL) so a late-joining consumer (e.g. the
app reconnecting through rosbridge) immediately gets the last known state.

Run with use_sim_time:=false (real wall-clock) so liveness reflects real
message-arrival timing rather than a possibly-absent /clock.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class HeartbeatNode(Node):
    """Publish /robot/online based on freshness of a source topic."""

    def __init__(self):
        super().__init__('robot_heartbeat')
        self.declare_parameter('source_topic', '/odom')
        self.declare_parameter('stale_timeout_s', 2.0)
        self.declare_parameter('publish_rate_hz', 2.0)

        self._source_topic = str(self.get_parameter('source_topic').value)
        self._stale_timeout = float(self.get_parameter('stale_timeout_s').value)
        rate = max(0.1, float(self.get_parameter('publish_rate_hz').value))

        self._last_source_s = None
        self._last_published = None

        qos = QoSProfile(depth=1)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL  # latched
        self._pub = self.create_publisher(Bool, '/robot/online', qos)

        self.create_subscription(
            Odometry, self._source_topic, self._on_source, 10
        )
        self._timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'robot_heartbeat: source={self._source_topic} '
            f'stale_timeout={self._stale_timeout}s rate={rate}Hz'
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_source(self, _msg) -> None:
        self._last_source_s = self._now_s()

    def _tick(self) -> None:
        online = (
            self._last_source_s is not None
            and (self._now_s() - self._last_source_s) <= self._stale_timeout
        )
        self._pub.publish(Bool(data=bool(online)))
        if online != self._last_published:
            self._last_published = online
            self.get_logger().info(f'/robot/online -> {online}')


def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatNode()
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
