"""End-to-end test of pid_autotune_node against a simulated STM32 base.

Needs rclpy + the generated mower_interface srv (skipped outside a ROS
environment). The fake base mimics what mower_hardware exposes: it consumes
``/mower_base/wheel_override`` and ``/mower_base/pid_command`` and publishes
``/mower_base/telemetry`` at 20 Hz from two first-order motors driven by the
firmware's PI loop (or raw PWM in open loop).
"""

import json
import math
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('mower_interface.srv')

from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from mower_interface.srv import PidAutotune  # noqa: E402
from mower_mission.pid_autotune_node import PidAutotuneNode  # noqa: E402

MAX_RPM = 58.0
PWM_MAX = 200.0
DT = 0.02


class FakeBase(Node):
    """Two identical geared DC motors (K=0.29 rpm/count, tau=0.25 s, 2-tick delay)."""

    def __init__(self):
        super().__init__('fake_base')
        best_effort = QoSProfile(depth=1)
        best_effort.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.gains = {w: {'kp': 2.0, 'ki': 0.6, 'kd': 0.0} for w in ('left', 'right')}
        self.closed_loop = True
        self.flash_valid = False
        self.pid_seq = 0
        self.persist_count = 0
        self.pid_commands = []
        self.override = (0, 0, 0.0)  # left, right, expires (monotonic)
        self.max_override = 0
        self.motor_flags = 0x01  # 0x81 flags; 0x04 = DRIVER_ALARM (BTS7960 IS, advisory)
        self.rpm = {'left': 0.0, 'right': 0.0}
        self.integral = {'left': 0.0, 'right': 0.0}
        self.hist = {'left': [0.0, 0.0, 0.0], 'right': [0.0, 0.0, 0.0]}
        self.pwm = {'left': 0.0, 'right': 0.0}
        self.target = {'left': 0.0, 'right': 0.0}
        self.seq = 0
        self.tick = 0
        self.pub = self.create_publisher(String, '/mower_base/telemetry', best_effort)
        self.create_subscription(String, '/mower_base/wheel_override', self._on_override, best_effort)
        self.create_subscription(String, '/mower_base/pid_command', self._on_pid, 4)
        self.create_timer(DT, self._step)

    def _on_override(self, msg):
        d = json.loads(msg.data)
        self.override = (int(d['left_permille']), int(d['right_permille']),
                         time.monotonic() + min(1000, int(d['ttl_ms'])) / 1000.0)
        self.max_override = max(self.max_override, abs(int(d['left_permille'])))

    def _on_pid(self, msg):
        d = json.loads(msg.data)
        self.pid_commands.append(d)
        self.gains = {w: {k: float(d[w][k]) for k in ('kp', 'ki', 'kd')} for w in ('left', 'right')}
        self.closed_loop = bool(d.get('closed_loop', 1))
        if d.get('persist'):
            self.persist_count += 1
            self.flash_valid = True
        self.pid_seq = (self.pid_seq + 7) % 256

    def _step(self):
        l, r, until = self.override
        if time.monotonic() > until:
            l = r = 0
        for w, cmd in (('left', l), ('right', r)):
            self.target[w] = cmd / 1000.0 * MAX_RPM
            if self.closed_loop:
                if cmd == 0:
                    self.integral[w] = 0.0
                    u = 0.0
                else:
                    err = self.target[w] - self.rpm[w]
                    self.integral[w] = max(-200.0, min(200.0, self.integral[w] + err * DT))
                    u = self.gains[w]['kp'] * err + self.gains[w]['ki'] * self.integral[w]
                    u = max(-PWM_MAX, min(PWM_MAX, u))
            else:
                u = cmd / 1000.0 * PWM_MAX
            self.pwm[w] = u
            self.hist[w].append(u)
            u_d = self.hist[w][-3]
            a = math.exp(-DT / 0.25)
            self.rpm[w] = a * self.rpm[w] + (1 - a) * 0.29 * u_d
        self.tick += 1
        if self.tick % 2 == 0:  # 0x85 every 50 ms
            self.seq = (self.seq + 1) % 256
            self._publish()

    def _publish(self):
        flags = (0x01 if self.closed_loop else 0) | (0x02 if self.flash_valid else 0) | 0x08
        doc = {
            't': time.time(), 'feedback_age_s': 0.01, 'crc_errors': 0,
            'wheel': {
                w: {'target_rpm': round(self.target[w], 2), 'measured_rpm': round(self.rpm[w], 2),
                    'pid_output': int(round(self.pwm[w])), 'total_counts': 0}
                for w in ('left', 'right')
            } | {'flags': 0x01 | (0x02 if self.closed_loop else 0), 'seq': self.seq},
            'motor': {'valid': True, 'flags': self.motor_flags, 'command_age_ms': 10},
            'pid': {'valid': True, 'left': self.gains['left'], 'right': self.gains['right'],
                    'flags': flags, 'last_rx_seq': self.pid_seq},
            'led': {'valid': True, 'mode': 1},
            'power': {'valid': True, 'state': 0, 'flags': 0x02},
        }
        self.pub.publish(String(data=json.dumps(doc)))


class Harness:
    def __init__(self, with_base=True, **overrides):
        rclpy.init()
        self.base = FakeBase() if with_base else None
        params = [rclpy.Parameter(k, value=v) for k, v in overrides.items()]
        self.tuner = PidAutotuneNode(parameter_overrides=params)
        self.client_node = Node('autotune_test_client')
        self.client = self.client_node.create_client(PidAutotune, '/pid_autotune')
        self.statuses = []
        latched = QoSProfile(depth=1)
        latched.reliability = QoSReliabilityPolicy.RELIABLE
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.client_node.create_subscription(
            String, '/pid_autotune/status', lambda m: self.statuses.append(json.loads(m.data)), latched)
        self.exec = SingleThreadedExecutor()
        for n in (self.base, self.tuner, self.client_node):
            if n is not None:
                self.exec.add_node(n)
        self.thread = threading.Thread(target=self.exec.spin, daemon=True)
        self.thread.start()
        assert self.client.wait_for_service(timeout_sec=5.0)

    def call(self, op):
        req = PidAutotune.Request()
        req.op = op
        fut = self.client.call_async(req)
        deadline = time.monotonic() + 5.0
        while not fut.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert fut.done(), f'{op} did not answer'
        return fut.result()

    def state(self):
        return self.statuses[-1]['state'] if self.statuses else None

    def wait_state(self, *states, timeout=30.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.state() in states:
                return self.statuses[-1]
            time.sleep(0.05)
        raise AssertionError(f'expected {states}, last status {self.statuses[-1] if self.statuses else None}')

    def close(self):
        self.exec.shutdown()
        self.thread.join(timeout=5)
        for n in (self.base, self.tuner, self.client_node):
            if n is not None:
                n.destroy_node()
        rclpy.shutdown()


FAST = {'hold_s': 1.5, 'verify_hold_s': 2.0, 'telemetry_timeout_s': 1.0}


def test_full_run_apply_writes_flash_once():
    h = Harness(**FAST)
    try:
        time.sleep(0.5)  # let telemetry flow
        res = h.call('start')
        assert res.success, res.message
        st = h.wait_state('review', 'failed', 'aborted', timeout=40)
        assert st['state'] == 'review', st['message']
        assert h.base.max_override == 700
        # the model must look like the simulated motor
        for w in ('left', 'right'):
            m = st['model'][w]
            assert abs(m['gain'] - 0.29) / 0.29 < 0.15, m
            assert abs(m['tau'] - 0.25) / 0.25 < 0.30, m
            assert st['new_gains'][w]['kp'] > 0 and st['new_gains'][w]['ki'] > 0
            assert st['verify'][w]['overshoot_pct'] <= 25
            assert len(st['samples']['open_loop'][w]) > 40
            assert len(st['samples']['verify'][w]) > 30
        assert h.base.persist_count == 0
        # wheels are released (override 0) and coast to a stop while we think about it
        assert h.base.override[0] == 0 and h.base.override[1] == 0
        time.sleep(1.0)
        assert abs(h.base.rpm['left']) < 0.5
        res = h.call('apply')
        assert res.success, res.message
        st = h.wait_state('done', 'failed', timeout=10)
        assert st['state'] == 'done', st['message']
        assert h.base.persist_count == 1
        assert h.base.closed_loop
        assert h.base.gains['left']['kp'] == pytest.approx(st['new_gains']['left']['kp'])
        # idle again: a second start is accepted
        time.sleep(0.3)
        assert h.call('start').success
        h.call('abort')
        h.wait_state('aborted', timeout=10)
    finally:
        h.close()


def test_abort_restores_previous_gains():
    h = Harness(**FAST)
    try:
        time.sleep(0.5)
        old = json.loads(json.dumps(h.base.gains))
        assert h.call('start').success
        h.wait_state('open_loop', timeout=10)
        time.sleep(0.8)
        assert not h.base.closed_loop
        assert h.call('abort').success
        st = h.wait_state('aborted', timeout=10)
        assert st['error']
        time.sleep(0.3)
        assert h.base.closed_loop
        assert h.base.gains == old
        assert h.base.persist_count == 0
        assert h.base.override[0] == 0
        assert not h.call('apply').success
    finally:
        h.close()


def test_discard_restores_previous_gains():
    h = Harness(**FAST)
    try:
        time.sleep(0.5)
        old = json.loads(json.dumps(h.base.gains))
        assert h.call('start').success
        st = h.wait_state('review', 'failed', 'aborted', timeout=40)
        assert st['state'] == 'review', st['message']
        assert h.base.gains != old  # new gains are live in RAM during review
        assert h.call('discard').success
        st = h.wait_state('idle', timeout=10)
        assert st['error'] is None
        time.sleep(0.3)
        assert h.base.gains == old
        assert h.base.persist_count == 0
    finally:
        h.close()


def test_driver_alarm_flag_does_not_block_a_run():
    # the BTS7960 IS pin trips on ordinary drive current; the run must go on
    h = Harness(**FAST)
    try:
        h.base.motor_flags = 0x05
        time.sleep(0.5)
        assert h.call('start').success
        st = h.wait_state('review', 'failed', 'aborted', timeout=40)
        assert st['state'] == 'review', st['message']
        h.call('discard')
        h.wait_state('idle', timeout=10)
    finally:
        h.close()


def test_start_fails_without_telemetry():
    h = Harness(with_base=False, **FAST)  # nobody publishes telemetry
    try:
        assert h.call('start').success  # accepted, then fails in precheck
        st = h.wait_state('failed', timeout=10)
        assert 'telemetry' in st['error']
    finally:
        h.close()
