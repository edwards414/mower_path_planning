#!/usr/bin/env python3
"""Black-box check that mower_pid_autotune only subscribes to the base telemetry
while a session runs.

A permanent /mower_base/telemetry subscription cost ~10 % of mower_rsd's CPU
on XR-02 while idle (perf, 2026-10-25): every 20 Hz frame woke the node's spin
loop. The node now subscribes on "start" and drops the subscription once the
session is over. Runs the real binary against a fake base that publishes a
telemetry frame every 50 ms, counting the frame's subscribers:

  idle      no subscriber, for 4 s after start-up
  session   "start" subscribes: the frame (power state 1, not RUNNING) reaches
            the precheck, which fails with "power state 1 is not RUNNING"
            instead of "no /mower_base/telemetry"; one subscriber meanwhile,
            none 1 s after the session failed. Twice, so a second session
            subscribes again.
  no frames the base goes quiet: the precheck fails with "no
            /mower_base/telemetry" after telemetry_timeout_s plus the
            subscription grace (~1.5 s); once frames come back the leftover
            subscription is dropped on its own.

Needs a sourced ROS 2 Jazzy environment with mower_interface built:

  python3 src/mower_rs/tools/autotune_telemetry_check.py --binary <path to mower_pid_autotune>

Prints AUTOTUNE_TELEMETRY_OK and exits 0, or exits 1 if any check failed.
"""

import argparse
import json
import subprocess
import sys
import time

import rclpy
from mower_interface.srv import PidAutotune
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

TOPIC = '/mower_base/telemetry'
NOT_RUNNING = String(data=json.dumps({'power': {'valid': True, 'state': 1}}))
TERMINAL = ('failed', 'done', 'aborted')


class Check:
    def __init__(self):
        self.node = Node('autotune_telemetry_check')
        self.pub = self.node.create_publisher(String, TOPIC, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.statuses = []
        self.node.create_subscription(String, '/pid_autotune/status', lambda m: self.statuses.append(json.loads(m.data)), latched)
        self.client = self.node.create_client(PidAutotune, '/pid_autotune')
        self.feeding = True
        self.node.create_timer(0.05, self.feed)
        self.ok = True

    def feed(self):
        if self.feeding:
            self.pub.publish(NOT_RUNNING)

    def subscribers(self):
        return self.pub.get_subscription_count()

    def spin_for(self, seconds, watch=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if watch:
                watch()

    def expect(self, cond, what):
        print(('PASS ' if cond else 'FAIL ') + what, flush=True)
        self.ok &= bool(cond)

    def session(self, label, expect_message):
        self.statuses.clear()
        t0 = time.monotonic()
        fut = self.client.call_async(PidAutotune.Request(op='start'))
        while not fut.done() and time.monotonic() - t0 < 5.0:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        reply = fut.result() if fut.done() else None
        self.expect(reply is not None and reply.success, f'{label}: start -> {reply.message if reply else "no reply"}')
        peak = [0]
        ended = [None]

        def watch():
            peak[0] = max(peak[0], self.subscribers())
            if ended[0] is None and self.statuses and self.statuses[-1]['state'] in TERMINAL:
                ended[0] = time.monotonic() - t0

        self.spin_for(4.0, watch)
        last = self.statuses[-1] if self.statuses else {}
        took = f'{ended[0]:.2f} s' if ended[0] is not None else 'never'
        self.expect(last.get('state') == 'failed' and expect_message in (last.get('message') or ''),
                    f'{label}: {last.get("state")} after {took}: {last.get("message")}')
        return peak[0]


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--binary', required=True, help='path to the mower_pid_autotune binary')
    args = ap.parse_args()

    proc = subprocess.Popen([args.binary], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rclpy.init()
    c = Check()
    try:
        if not c.client.wait_for_service(timeout_sec=15.0):
            c.expect(False, 'service /pid_autotune came up')
            return 1
        idle_peak = [0]
        c.spin_for(4.0, lambda: idle_peak.__setitem__(0, max(idle_peak[0], c.subscribers())))
        c.expect(idle_peak[0] == 0, f'idle: at most {idle_peak[0]} telemetry subscribers over 4 s')

        for n in (1, 2):
            peak = c.session(f'session {n}', 'power state 1 is not RUNNING')
            c.expect(peak == 1, f'session {n}: {peak} telemetry subscriber(s) while it ran')
            c.spin_for(1.0)
            c.expect(c.subscribers() == 0, f'session {n}: {c.subscribers()} telemetry subscribers 1 s after it ended')

        c.feeding = False
        c.session('no frames', 'no /mower_base/telemetry')
        c.feeding = True
        c.spin_for(1.5)
        c.expect(c.subscribers() == 0, f'frames back: {c.subscribers()} telemetry subscribers')
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
        c.node.destroy_node()
        rclpy.shutdown()
    if not c.ok:
        return 1
    print('AUTOTUNE_TELEMETRY_OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
