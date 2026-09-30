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
"""Auto-coverage driver: sequence the coverage pipeline robustly.

Runs once at startup (when mission is launched with auto_coverage:=true) and
drives load_zone_list -> create_free_space -> create_risk_map ->
generate_coverage_path so a path is ready without manual service calls.

Unlike fixed timers, this WAITS for each async step to actually finish before
the next: create_free_space/create_risk_map return immediately and do their work
in a callback, so we wait for /free_space_inflated and /risk_map_inflated to be
published (that's the real completion) before proceeding. This avoids the
"風險地圖需要先建立自由空間" / "缺少 risk_map_inflated" race.
"""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


def _latched() -> QoSProfile:
    q = QoSProfile(depth=1)
    q.reliability = QoSReliabilityPolicy.RELIABLE
    q.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    return q


class AutoCoverage(Node):
    def __init__(self):
        super().__init__('auto_coverage')
        self.declare_parameter('step_timeout_s', 60.0)
        self._timeout = float(self.get_parameter('step_timeout_s').value)

        self._free_space_seen = threading.Event()
        self._risk_map_seen = threading.Event()
        self._navigation_idle_seen = threading.Event()
        self.create_subscription(
            OccupancyGrid, '/free_space_inflated',
            lambda _m: self._free_space_seen.set(), _latched())
        self.create_subscription(
            OccupancyGrid, '/risk_map_inflated',
            lambda _m: self._risk_map_seen.set(), _latched())
        self.create_subscription(
            Bool,
            '/nav_operation_active',
            lambda message: (
                self._navigation_idle_seen.clear()
                if message.data
                else self._navigation_idle_seen.set()
            ),
            _latched(),
        )

        self.cli_load = self.create_client(Trigger, '/load_zone_list')
        self.cli_fs = self.create_client(Trigger, '/create_free_space')
        self.cli_risk = self.create_client(Trigger, '/create_risk_map')
        self.cli_cov = self.create_client(Trigger, '/generate_coverage_path')

    # ── helpers ──────────────────────────────────────────────────────────────

    def _call(self, cli, name) -> bool:
        if not cli.wait_for_service(timeout_sec=self._timeout):
            self.get_logger().error(f'auto_coverage: {name} service unavailable')
            return False
        fut = cli.call_async(Trigger.Request())
        t0 = time.time()
        while rclpy.ok() and not fut.done() and time.time() - t0 < self._timeout:
            time.sleep(0.05)
        if not fut.done():
            self.get_logger().error(f'auto_coverage: {name} timed out')
            return False
        resp = fut.result()
        ok = bool(getattr(resp, 'success', True))
        self.get_logger().info(
            f'auto_coverage: {name} -> success={ok} '
            f'msg={getattr(resp, "message", "")}')
        return ok

    def _wait(self, event, what) -> bool:
        if event.wait(timeout=self._timeout):
            self.get_logger().info(f'auto_coverage: {what} ready')
            return True
        self.get_logger().error(f'auto_coverage: timed out waiting for {what}')
        return False

    def run_sequence(self) -> bool:
        # Give the freshly-launched nodes a moment to advertise services.
        time.sleep(3.0)
        if not self._wait(
            self._navigation_idle_seen,
            'navigation coordinator idle state',
        ):
            return False
        if not self._call(self.cli_load, '/load_zone_list'):
            return False
        # create_free_space is async -> wait for /free_space_inflated.
        if not self._call(self.cli_fs, '/create_free_space'):
            return False
        if not self._wait(self._free_space_seen, '/free_space_inflated'):
            return False
        # create_risk_map is async -> wait for /risk_map_inflated.
        if not self._call(self.cli_risk, '/create_risk_map'):
            return False
        if not self._wait(self._risk_map_seen, '/risk_map_inflated'):
            return False
        # /risk_map_inflated arriving on OUR subscription doesn't guarantee
        # the coverage planner (boustrophedon_coverage, mower_rs
        # mower_coverage) has processed it yet (separate subscriber). Retry
        # generate_coverage_path until the planner has the map.
        for attempt in range(8):
            time.sleep(1.5)
            if not self.cli_cov.wait_for_service(timeout_sec=self._timeout):
                self.get_logger().error('auto_coverage: coverage service gone')
                return False
            fut = self.cli_cov.call_async(Trigger.Request())
            t0 = time.time()
            while rclpy.ok() and not fut.done() and time.time() - t0 < self._timeout:
                time.sleep(0.05)
            resp = fut.result() if fut.done() else None
            ok = bool(getattr(resp, 'success', False)) if resp else False
            msg = getattr(resp, 'message', '') if resp else 'timeout'
            if ok:
                self.get_logger().info('auto_coverage: coverage path ready ✓')
                return True
            self.get_logger().warn(
                f'auto_coverage: generate attempt {attempt + 1} failed ({msg}); '
                'retrying')
        self.get_logger().error('auto_coverage: coverage generation gave up')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = AutoCoverage()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    exit_code = 0
    try:
        if not node.run_sequence():
            exit_code = 1
            node.get_logger().fatal(
                'auto_coverage: startup sequence failed; exiting nonzero'
            )
        else:
            node.get_logger().info('auto_coverage: done (idle)')
            # Keep the successful one-shot node alive. A failed sequence must
            # exit so launch supervision and CI can observe the failure.
            while rclpy.ok():
                time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
