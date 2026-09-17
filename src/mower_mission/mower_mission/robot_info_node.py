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
"""What is running on this robot, for the app and the updater.

Publishes ``/robot/info`` (std_msgs/String, JSON, latched + periodic)::

    {
      "robot_id": "lubancat",
      "api_version": 1,
      "software": {"version": "0.6.0", "git_sha": "...", "build_unix": ..., "image": "ghcr.io/...:v0.6.0",
                   "digest": "sha256:...", "tag": "stable"},
      "firmware": {"running": {...0x87 fields...} | null,
                   "bundled": {"version": ..., "git_sha": ..., ...} | null,
                   "sync": {"action": "up_to_date|flashed|failed|no_device", "time": ..., "error": ...} | null,
                   "up_to_date": true | false | null},
      "update": {"state": "idle|pulling|restarting|failed|...", "message": "...", "time": ...} | null,
      "busy": false,
      "uptime_s": 123.4
    }

Sources: the image environment (MOWER_VERSION ...), ``/mower_base/firmware_info``
from the ros2_control driver (the 0x87 frame), the bundled firmware manifest,
and the JSON files the host updater / firmware-sync leave in the shared state
dir (see host_request.py).

Lights: while the host reports an update in progress (``update.state`` in
``pulling`` / ``restarting`` / ``rebooting``) the node asks the base for the
amber orbit effect on ``/mower_base/led_command`` (latched JSON, consumed by
the mower_hardware driver, see firmware/LED_COMMAND_MODES.md mode 0x06) and
restores the steady white when the update is over.

The node also drops ``<state_dir>/robot_status.json`` (busy / moving /
nav_running, once a second) so the host-side auto-update timer can tell
whether the robot is idle without talking ROS.

Services (std_srvs/Trigger):

* ``/system/update``  – ask the host to pull the current image tag and
  restart the robot software (refused while the robot is moving per /odom
  or ``/nav_operation_active`` is true).
* ``/system/restart`` – restart the robot software container.
"""

import json
import os
import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from mower_mission import host_request
from mower_mission import identity as ident
from mower_mission.version import ROBOT_API_VERSION, software_identity


class RobotInfoNode(Node):
    """Aggregate version / update state and publish it as /robot/info."""

    def __init__(self):
        super().__init__('robot_info')
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('firmware_info_topic', '/mower_base/firmware_info')
        self.declare_parameter(
            'firmware_manifest',
            os.environ.get(
                'MOWER_FIRMWARE_MANIFEST',
                '/opt/mower/firmware/mower_robot_firmware.json',
            ),
        )
        self.declare_parameter('state_dir', host_request.state_dir())
        # identity.json (deploy/host/mower-pair) wins; env / hostname are the
        # development fallback
        identity = ident.load_identity(host_request.state_dir()) or {}
        self.declare_parameter(
            'robot_id',
            identity.get('robot_id') or os.environ.get('MOWER_ROBOT_ID') or socket.gethostname(),
        )
        self.declare_parameter('robot_name', identity.get('name') or socket.gethostname())
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('nav_active_topic', '/nav_operation_active')
        self.declare_parameter('moving_speed_threshold', 0.02)
        self.declare_parameter('busy_timeout_s', 2.0)
        self.declare_parameter('led_topic', '/mower_base/led_command')
        # amber orbit while updating, steady white (the boot show's final
        # frame) otherwise; period is one revolution in ms
        self.declare_parameter('led_update_rgb', [255, 180, 0])
        self.declare_parameter('led_update_period_ms', 1600)
        self.declare_parameter('led_normal_rgb', [110, 110, 110])

        rate = max(0.1, float(self.get_parameter('publish_rate_hz').value))
        self._firmware_manifest_path = str(self.get_parameter('firmware_manifest').value)
        self._state_dir = str(self.get_parameter('state_dir').value)
        self._robot_id = str(self.get_parameter('robot_id').value)
        self._robot_name = str(self.get_parameter('robot_name').value)
        self._paired = bool(identity)
        self._moving_threshold = float(self.get_parameter('moving_speed_threshold').value)
        self._busy_timeout = float(self.get_parameter('busy_timeout_s').value)

        self._software = software_identity()
        self._firmware_running = None
        self._bundled_firmware = self._load_json_file(self._firmware_manifest_path)
        self._last_moving_s = None
        self._nav_running = False
        self._start_s = time.monotonic()
        self._led_updating = None  # what we last asked the lights to show

        latched = QoSProfile(depth=1)
        latched.reliability = QoSReliabilityPolicy.RELIABLE
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self._pub = self.create_publisher(String, '/robot/info', latched)
        led_topic = str(self.get_parameter('led_topic').value)
        self._led_pub = (
            self.create_publisher(String, led_topic, latched) if led_topic else None
        )

        fw_topic = str(self.get_parameter('firmware_info_topic').value)
        if fw_topic:
            self.create_subscription(String, fw_topic, self._on_firmware_info, latched)
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value), self._on_odom, 10
        )
        # nav_action_server's latched cross-node guard: True while a
        # navigation / coverage run is in progress.
        self.create_subscription(
            Bool, str(self.get_parameter('nav_active_topic').value), self._on_nav_active, latched
        )

        self.create_service(Trigger, '/system/update', self._on_update)
        self.create_service(Trigger, '/system/restart', self._on_restart)

        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'robot_info: id={self._robot_id} software={self._software["version"]} '
            f'api={ROBOT_API_VERSION} state_dir={self._state_dir} '
            f'bundled_firmware='
            f'{(self._bundled_firmware or {}).get("version", "none")}'
        )
        self._tick()

    # ---- inputs --------------------------------------------------------------

    def _on_firmware_info(self, msg: String) -> None:
        try:
            info = json.loads(msg.data)
        except ValueError:
            self.get_logger().warning('ignoring malformed firmware_info payload')
            return
        if info != self._firmware_running:
            self._firmware_running = info
            self.get_logger().info(
                f'STM32 firmware {info.get("version")}+{info.get("git_sha")}'
            )
            self._tick()

    def _on_nav_active(self, msg: Bool) -> None:
        self._nav_running = bool(msg.data)

    def _on_odom(self, msg: Odometry) -> None:
        v = msg.twist.twist
        if abs(v.linear.x) > self._moving_threshold or abs(v.angular.z) > self._moving_threshold:
            self._last_moving_s = time.monotonic()

    def _busy(self) -> bool:
        moving = (
            self._last_moving_s is not None
            and (time.monotonic() - self._last_moving_s) <= self._busy_timeout
        )
        return moving or self._nav_running

    # ---- services ------------------------------------------------------------

    def _on_update(self, _req, res):
        return self._request_host('update', res, check_busy=True)

    def _on_restart(self, _req, res):
        return self._request_host('restart', res, check_busy=True)

    def _request_host(self, action: str, res, check_busy: bool):
        if check_busy and self._busy():
            res.success = False
            res.message = 'robot is busy (moving or navigating); stop it first'
            return res
        try:
            path = host_request.write_request(action, requested_by='app', directory=self._state_dir)
        except OSError as exc:
            res.success = False
            res.message = f'could not write host request: {exc}'
            self.get_logger().error(res.message)
            return res
        self.get_logger().warning(f'host action {action!r} requested via {path}')
        res.success = True
        res.message = f'{action} requested; watch /robot/info update.state'
        return res

    # ---- output --------------------------------------------------------------

    def _tick(self) -> None:
        snapshot = self._snapshot()
        self._pub.publish(String(data=json.dumps(snapshot, separators=(',', ':'))))
        self._update_lights(snapshot.get('update') or {})
        self._write_robot_status(snapshot)

    def _update_lights(self, update: dict) -> None:
        """Amber orbit while the host updates, steady white afterwards."""
        if self._led_pub is None:
            return
        updating = str(update.get('state', '')) in ('pulling', 'restarting', 'rebooting')
        if updating == self._led_updating:
            return
        self._led_updating = updating
        if updating:
            r, g, b = (int(v) for v in self.get_parameter('led_update_rgb').value)
            request = {
                'mode': 6,
                'r': r,
                'g': g,
                'b': b,
                'period_ms': int(self.get_parameter('led_update_period_ms').value),
            }
        else:
            r, g, b = (int(v) for v in self.get_parameter('led_normal_rgb').value)
            request = {'mode': 1, 'r': r, 'g': g, 'b': b, 'period_ms': 0}
        self._led_pub.publish(String(data=json.dumps(request, separators=(',', ':'))))
        self.get_logger().info(f'lights: {"update orbit" if updating else "normal"}')

    def _write_robot_status(self, snapshot: dict) -> None:
        """Idle/busy for the host auto-update timer (deploy/host/mower-update.sh)."""
        path = os.path.join(self._state_dir, 'robot_status.json')
        moving = (
            self._last_moving_s is not None
            and (time.monotonic() - self._last_moving_s) <= self._busy_timeout
        )
        data = {
            'time': int(time.time()),
            'busy': bool(snapshot.get('busy')),
            'moving': bool(moving),
            'nav_running': bool(self._nav_running),
            'software': self._software.get('version'),
        }
        try:
            os.makedirs(self._state_dir, exist_ok=True)
            tmp = f'{path}.tmp'
            with open(tmp, 'w') as f:
                json.dump(data, f)
            os.replace(tmp, path)
        except OSError as exc:
            self.get_logger().warning(f'cannot write robot_status.json: {exc}', throttle_duration_sec=60)

    def _snapshot(self) -> dict:
        image = host_request.read_json(host_request.IMAGE_FILE, self._state_dir) or {}
        sync = host_request.read_json(host_request.FIRMWARE_SYNC_FILE, self._state_dir)
        update = host_request.read_json(host_request.UPDATE_STATUS_FILE, self._state_dir)

        software = dict(self._software)
        for key in ('image', 'digest', 'tag', 'pulled_at'):
            if image.get(key) and not software.get(key):
                software[key] = image[key]

        bundled = None
        if self._bundled_firmware:
            bundled = {
                k: self._bundled_firmware.get(k)
                for k in ('version', 'semver', 'git_sha', 'build_unix', 'dirty', 'size', 'crc32')
            }
        running = self._firmware_running
        up_to_date = None
        if running and bundled:
            up_to_date = (
                not running.get('unversioned')
                and running.get('semver') == bundled.get('semver')
                and running.get('git_sha') == (bundled.get('git_sha') or '')[:8]
                and running.get('build_unix') == bundled.get('build_unix')
                and bool(running.get('dirty')) == bool(bundled.get('dirty'))
            )

        sync_summary = None
        if sync:
            sync_summary = {k: sync.get(k) for k in ('action', 'time', 'error')}

        return {
            'robot_id': self._robot_id,
            'name': self._robot_name,
            'pairing_required': self._paired,
            'api_version': ROBOT_API_VERSION,
            'software': software,
            'firmware': {
                'running': running,
                'bundled': bundled,
                'sync': sync_summary,
                'up_to_date': up_to_date,
            },
            'update': update,
            'busy': self._busy(),
            'uptime_s': round(time.monotonic() - self._start_s, 1),
        }

    def _load_json_file(self, path: str):
        try:
            with open(path) as f:
                return json.load(f)
        except (OSError, ValueError):
            return None


def main(args=None):
    rclpy.init(args=args)
    node = RobotInfoNode()
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
