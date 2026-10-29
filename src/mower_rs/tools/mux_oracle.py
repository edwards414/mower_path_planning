#!/usr/bin/env python3
"""The twist_mux oracle (step E4 of docs/ROS_FREE_PLAN.md): the real C++
twist_mux next to the Rust mux module of mower_rsd, fed the same timed input
scripts, every output and every /diagnostics message recorded and compared.

Runs inside the runtime image, offline, on a DDS domain of its own::

    docker run --rm -u 0 --network none --cap-add NET_ADMIN -e ROS_DOMAIN_ID=71 \\
      -v <repo>:/r:ro -v <out>:/o [-v <dir with mower_rsd>:/b:ro] \\
      --entrypoint bash mower_path_planning:ros-free-test -c \\
      'source /opt/ros/jazzy/setup.bash && source /mower_ws/install/setup.bash && \\
       python3 /r/src/mower_rs/tools/mux_oracle.py run --out /o --cpp 2 [--rust /b/mower_rsd]'
    python3 tools/mux_oracle.py compare <out>          # on the host, no ROS needed
    python3 tools/mux_oracle.py vectors <out> <crates/mower_mux/tests/vectors>

``run`` starts ``--cpp`` C++ twist_mux processes (node ``twist_mux_cppN``,
output ``/oracle/cppN/cmd_vel_guard_input``) and, with ``--rust``,
``mower_rsd --modules mux`` (node ``twist_mux``, output
``/cmd_vel_guard_input``), each with the production twist_mux_topics.yaml
(its key renamed to the node). Three sessions, each with fresh processes:

- ``main``: S01-S14, S16, S17 and the random segment, 1.5 s of silence
  between two scenarios (every timeout is at most 0.5 s, so each starts with
  every input expired and the lock locked, as after a start);
- ``startup`` (S15): the inputs run first, the muxes are started under them;
- ``generic`` (S18): another configuration with a tie, a priority-0 input,
  a timeout-0 input and lock and a priority-0 lock.

The probe is one rclpy node on a SingleThreadedExecutor (a MultiThreaded one
starves the scheduler): writers with the production QoS (the safety stop and
the lock R V 10, /joy_cmd and /nav_cmd_vel R V 1, teleop R V 10), each input
tagged ``<source>#<seq>`` in its frame_id, so every output names the input it
forwarded. ``compare`` checks, per scenario and against a reference model of
twist_mux 4.5.0 (receive time = send time):

- the forwarded sequence: identical across instances, apart from decisions
  that lie within ``--margin`` (3 ms) of a timeout or of another input (they
  depend on the delivery order and are listed, not counted); S16 compares
  counts, S17 the first forwarded sequence number (within one);
- the content: every output equals the input it forwards (stamp, frame_id,
  the six twist components);
- the latency of each instance against the first C++ one, per forwarded
  message present on both: p50 and p99 of the difference;
- /diagnostics: the "Node starting up" status first (when the probe matched
  the new writer in time), then two a second, with
  the same level, message, hardware id, keys and topic/timeout/priority text,
  and masked / locked / current priority as the model has them when the
  message arrived (again up to the margin).

Two C++ instances measure the noise floor: what two copies of the same binary
already disagree on, on the same DDS samples.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import os
import random
import signal
import subprocess
import sys
import threading
import time

TOOLS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(TOOLS, '../../..'))
PRODUCTION_YAML = os.path.join(REPO, 'src/mower_bringup/config/twist_mux_topics.yaml')

# The production inputs: key -> (topic, writer depth). The guards' writers are
# R V 1 (velocity_command_guard, velocity_smoother), nav's R V 10 (the bridge's
# to_ros), teleop's R V 10.
PRODUCTION_KEYS = {
    'safety': ('/navigation_safety_stop', 10),
    'nav': ('/nav_cmd_vel', 1),
    'joy': ('/joy_cmd', 1),
    'pjoy': ('/physical_joy_cmd', 10),
    'kb': ('/keyboard_cmd_vel', 10),
}
PRODUCTION_LOCKS = {'lock': ('/navigation_coordinator_lock', 10)}

GENERIC_YAML = """\
{node}:
  ros__parameters:
    use_stamped: true
    topics:
      bbb: {{topic: /g_bbb, timeout: 0.5, priority: 50}}
      aaa: {{topic: /g_aaa, timeout: 0.5, priority: 50}}
      zero: {{topic: /g_zero, timeout: 0.5, priority: 0}}
      never: {{topic: /g_never, timeout: 0.0, priority: 40}}
      low: {{topic: /g_low, timeout: 0.5, priority: 30}}
    locks:
      quiet: {{topic: /g_quiet, timeout: 0.0, priority: 60}}
      zerolock: {{topic: /g_zerolock, timeout: 0.5, priority: 0}}
      high: {{topic: /g_high, timeout: 0.3, priority: 45}}
"""
GENERIC_KEYS = {'bbb': ('/g_bbb', 10), 'aaa': ('/g_aaa', 10), 'zero': ('/g_zero', 10),
                'never': ('/g_never', 10), 'low': ('/g_low', 10)}
GENERIC_LOCKS = {'quiet': ('/g_quiet', 10), 'zerolock': ('/g_zerolock', 10), 'high': ('/g_high', 10)}

# The same rules on the production topics, one lock per session: the names
# mower_bridge's manifest has for the mux, so that a mux in mower_rsd gets
# them through the bridge too (the /g_* names above it would not bridge).
GENERIC_LOCK_VARIANTS = {'high': (0.3, 45), 'zerolock': (0.5, 0), 'quiet': (0.0, 60)}
GENERIC2_YAML = """\
{node}:
  ros__parameters:
    use_stamped: true
    topics:
      bbb: {{topic: /keyboard_cmd_vel, timeout: 0.5, priority: 50}}
      aaa: {{topic: /joy_cmd, timeout: 0.5, priority: 50}}
      zero: {{topic: /nav_cmd_vel, timeout: 0.5, priority: 0}}
      never: {{topic: /physical_joy_cmd, timeout: 0.0, priority: 40}}
      low: {{topic: /navigation_safety_stop, timeout: 0.5, priority: 30}}
    locks:
      {variant}: {{topic: /navigation_coordinator_lock, timeout: {timeout}, priority: {priority}}}
"""
GENERIC2_KEYS = {'aaa': ('/joy_cmd', 1), 'bbb': ('/keyboard_cmd_vel', 10), 'zero': ('/nav_cmd_vel', 1),
                 'never': ('/physical_joy_cmd', 10), 'low': ('/navigation_safety_stop', 10)}
GENERIC2_LOCKS = {'lk': ('/navigation_coordinator_lock', 10)}

GAP_S = 1.5
RUST_OUT = '/cmd_vel_guard_input'


def cpp_out(name):
    return f'/oracle/{name}/cmd_vel_guard_input'


# ---------------------------------------------------------------------------
# twist_mux's configuration and a reference model of its rules
# ---------------------------------------------------------------------------

def parse_config(text):
    """The handles twist_mux builds from a parameter file: rclcpp lists the
    parameters sorted by full name, and getTopicHandles takes the prefixes in
    that order (topic, timeout, priority each)."""
    import yaml
    doc = yaml.safe_load(text)
    [section] = doc.values()
    params = section['ros__parameters']
    flat = {}

    def walk(prefix, value):
        if isinstance(value, dict):
            for k, v in value.items():
                walk(f'{prefix}.{k}' if prefix else str(k), v)
        else:
            flat[prefix] = value
    walk('', params)

    def handles(kind):
        prefixes = []
        for name in sorted(k for k in flat if k.startswith(kind + '.')):
            prefix = name.rsplit('.', 1)[0]
            if prefix not in prefixes:
                prefixes.append(prefix)
        return [{'name': p, 'topic': flat[p + '.topic'], 'timeout': float(flat[p + '.timeout']),
                 'priority': max(0, min(255, int(flat[p + '.priority'])))} for p in prefixes]
    return {'inputs': handles('topics'), 'locks': handles('locks')}


def timeout_ns(seconds):
    # rclcpp::Duration(std::chrono::duration<double>(s)): duration_cast
    # truncates the double product toward zero
    return int(seconds * 1e9)


class Model:
    """twist_mux 4.5.0 as tests/vectors replay it: an input is expired when
    its timeout is positive and its last message is older than that, or it
    never had one; a lock is locked when expired or true; the lock priority
    is the highest of the locked locks (from 0, strict <), an input is masked
    when expired or below it, and the winner is the first input of the highest
    priority among the unmasked ones (from 0, strict <, so priority 0 never
    wins). An input is forwarded when it is the winner as it arrives."""

    def __init__(self, config):
        self.inputs = config['inputs']
        self.locks = config['locks']

    @staticmethod
    def expired(timeout, last, t):
        to = timeout_ns(timeout)
        return to > 0 and (last is None or t - last > to)

    def lock_priority(self, state, t):
        p = 0
        for lk in self.locks:
            last, data = state['locks'].get(lk['name'], (None, False))
            if (self.expired(lk['timeout'], last, t) or data) and p < lk['priority']:
                p = lk['priority']
        return p

    def winner(self, state, t):
        lp = self.lock_priority(state, t)
        p, name = 0, None
        for h in self.inputs:
            last = state['inputs'].get(h['name'])
            masked = self.expired(h['timeout'], last, t) or h['priority'] < lp
            if not masked and p < h['priority']:
                p, name = h['priority'], h['name']
        return name

    def diag(self, state, t, lp=None):
        lp = self.lock_priority(state, t) if lp is None else lp
        out = {}
        for h in self.inputs:
            last = state['inputs'].get(h['name'])
            masked = self.expired(h['timeout'], last, t) or h['priority'] < lp
            out['velocity ' + h['name']] = 'masked' if masked else 'unmasked'
        for lk in self.locks:
            last, data = state['locks'].get(lk['name'], (None, False))
            out['lock ' + lk['name']] = 'locked' if (self.expired(lk['timeout'], last, t) or data) else 'free'
        out['current priority'] = str(lp)
        return out


class Timeline:
    """Per handle, the times of its events (and a lock's data), so that the
    state at any instant is a bisection away."""

    def __init__(self, events):
        self.times, self.data, self.kind = {}, {}, {}
        for t, kind, handle, value in events:
            self.times.setdefault(handle, []).append(t)
            self.data.setdefault(handle, []).append(value)
            self.kind[handle] = kind

    def state(self, cut, handle=None, at=None):
        """What the mux holds once every event up to `cut` arrived, with
        `handle`'s last message at `at` if given."""
        st = {'inputs': {}, 'locks': {}}
        for h, ts in self.times.items():
            k = bisect.bisect_right(ts, cut)
            if k == 0:
                continue
            if self.kind[h] == 'input':
                st['inputs'][h] = ts[k - 1]
            else:
                st['locks'][h] = (ts[k - 1], bool(self.data[h][k - 1]))
        if handle is not None:
            st['inputs'][handle] = at
        return st


def decisions(model, events, margin_ns):
    """For each input event, (frame_id, forwarded?, ambiguous?). Ambiguous:
    the decision changes when it is taken `margin_ns` earlier or later, or with
    the other handles' events of that window arrived or not (their order of
    delivery is not fixed)."""
    tl = Timeline(events)
    out = []
    for t, kind, handle, value in events:
        if kind != 'input':
            continue
        forwarded = model.winner(tl.state(t, handle, t), t) == handle
        ambiguous = any(model.winner(tl.state(cut, handle, t), t_eval) != handle if forwarded
                        else model.winner(tl.state(cut, handle, t), t_eval) == handle
                        for cut in (t - margin_ns, t + margin_ns)
                        for t_eval in (t - margin_ns, t, t + margin_ns))
        # the next message of the same input within the margin: a depth-1
        # reader may hold only that one by the time the executor takes it
        ts = tl.times[handle]
        k = bisect.bisect_right(ts, t)
        if k < len(ts) and ts[k] - t <= margin_ns:
            ambiguous = True
        out.append((value, forwarded, ambiguous))
    return out


# ---------------------------------------------------------------------------
# The scenarios: (name, duration, [(t, key, kwargs)]) with t from its start
# ---------------------------------------------------------------------------

def stream(key, period, start, end, **kw):
    out, t = [], start
    while t < end - 1e-9:
        out.append((round(t, 6), key, kw))
        t += period
    return out


def scenario_list(random_s, seed):
    S = []
    S.append(('S01_nothing', 1.0, []))
    S.append(('S02_nav_lock_never_sent', 1.0, stream('nav', 0.05, 0.0, 1.0)))
    S.append(('S03_lock_false_nav', 1.2, stream('lock', 0.05, 0.0, 1.1, data=False)
              + stream('nav', 0.05, 0.025, 1.0)))
    S.append(('S04_lock_true_nav', 1.2, stream('lock', 0.05, 0.0, 1.1, data=True)
              + stream('nav', 0.05, 0.025, 1.0)))
    S.append(('S05_lock_true_nav_safety', 1.2, stream('lock', 0.05, 0.0, 1.1, data=True)
              + stream('safety', 0.05, 0.0125, 1.0) + stream('nav', 0.05, 0.025, 1.0)))
    # joy over safety, then safety back 0.5 s after the last joy (25 ms off it)
    S.append(('S06_joy_over_safety_then_release', 2.3, stream('lock', 0.05, 0.0, 2.2, data=True)
              + stream('safety', 0.05, 0.0125, 2.0) + stream('joy', 0.1, 0.3375, 1.3)))
    S.append(('S07_lock_true_only', 1.0, stream('lock', 0.05, 0.0, 1.0, data=True)))
    # the lock stops while nav goes on: locked 0.3 s after its last message
    S.append(('S08_lock_expires', 1.6, stream('lock', 0.05, 0.0, 0.5, data=False)
              + stream('nav', 0.05, 0.025, 1.5)))
    odd = [(0.075, 'nav', {'stamp': 'zero'}), (0.125, 'nav', {'stamp': 'future5'}),
           (0.175, 'nav', {'stamp': 'past5'}), (0.225, 'nav', {'frame': 'custom_frame_s09'}),
           (0.275, 'nav', {'twist': (0.1, 0.25, -0.5, 0.01, -0.02, 0.3)})]
    S.append(('S09_odd_stamps_verbatim', 0.9, stream('lock', 0.05, 0.0, 0.8, data=False)
              + [(0.025, 'nav', {})] + odd))
    S.append(('S10_teleop_priorities', 1.3, stream('lock', 0.05, 0.0, 1.2, data=True)
              + stream('safety', 0.05, 0.0125, 1.0) + stream('kb', 0.1, 0.0375, 1.0)
              + stream('pjoy', 0.1, 0.3125, 0.7)))
    S.append(('S11_one_joy_then_safety', 1.5, stream('lock', 0.05, 0.0, 1.5, data=True)
              + stream('safety', 0.05, 0.0125, 1.4) + [(0.1375, 'joy', {})]))
    # production idle: lock then safety in the same tick (nav's order)
    idle = []
    for k in range(600):
        t = round(k * 0.05, 6)
        idle.append((t, 'lock', {'data': True}))
        idle.append((t, 'safety', {}))
    S.append(('S12_production_idle_30s', 30.0, idle))
    # a manual takeover while navigating, the manual guard's zero 0.2 s
    # after the last joy, nav back 0.5 s after that zero
    S.append(('S13_manual_takeover', 4.5, stream('lock', 0.05, 0.0, 4.5, data=False)
              + stream('nav', 0.05, 0.025, 4.5) + stream('joy', 0.1, 1.0375, 3.0)
              + [(3.1375, 'joy', {'twist': (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)})]))
    # nav stops (lock and safety), nav2 goes on
    S.append(('S14_nav_dies', 2.0, stream('lock', 0.05, 0.0, 1.0, data=False)
              + stream('nav', 0.05, 0.025, 2.0)))
    # joy at 200 Hz: the depth-1 readers' drops (counts only)
    S.append(('S16_joy_200hz', 1.4, stream('lock', 0.05, 0.0, 1.3, data=True)
              + stream('safety', 0.05, 0.0125, 1.2) + stream('joy', 0.005, 0.1, 1.1)))
    # one joy, then safety every 2 ms at joy ages 0.490 .. 0.510 s
    # (anchored: what comes after the joy keeps its distance to the joy as
    # it was actually sent)
    S.append(('S17_boundary_scan', 0.8, stream('lock', 0.05, 0.0, 0.7, data=True)
              + [(0.05, 'joy', {'anchor': True})]
              + [(round(0.05 + 0.490 + 0.002 * k, 6), 'safety', {}) for k in range(11)]))
    if random_s > 0:
        S.append(('R01_random', float(random_s), random_script(random_s, seed)))
    return S


def random_script(seconds, seed):
    """Each second, every input is on with probability 0.3 and the lock is
    absent (0.2), false (0.4) or true (0.4); what is on arrives as a Poisson
    process of 20 Hz."""
    rng = random.Random(seed)
    out = []
    for w in range(int(seconds)):
        for key in ('safety', 'nav', 'joy', 'pjoy', 'kb', 'lock'):
            if key == 'lock':
                r = rng.random()
                if r < 0.2:
                    continue
                kw = {'data': r >= 0.6}
            else:
                if rng.random() >= 0.3:
                    continue
                kw = {}
            t = w + rng.expovariate(20.0)
            while t < w + 1:
                out.append((round(t, 6), key, dict(kw)))
                t += rng.expovariate(20.0)
    return out


# startup (S15): nav throughout, the muxes start at 2 s, lock false from 4 s
STARTUP = (stream('nav', 0.05, 0.025, 7.0) + stream('lock', 0.05, 4.0, 7.0, data=False))
STARTUP_MUX_START_S = 2.0
STARTUP_S = 7.5

# generic (S18), from the start of the muxes
GENERIC = (
    stream('low', 0.05, 1.0, 1.5)                                     # never (timeout 0) masks it from the start
    + stream('bbb', 0.05, 2.0, 3.0) + stream('aaa', 0.05, 2.3, 3.0)   # a tie: aaa first by name
    + stream('zero', 0.05, 3.5, 4.0)                                  # priority 0: never selected
    + stream('high', 0.05, 4.5, 5.5, data=True) + stream('low', 0.05, 4.6, 5.4)
    + [(5.0, 'never', {}), (5.2, 'aaa', {})]                          # below 45 masked, 50 not
    + [(6.0, 'never', {})] + stream('low', 0.05, 6.5, 7.0)            # never wins, fresh forever
    + stream('zerolock', 0.05, 7.5, 8.0, data=True) + [(7.75, 'never', {})]   # a priority-0 lock: no effect
    + [(8.5, 'quiet', {'data': True}), (9.0, 'never', {}), (9.2, 'aaa', {})]  # timeout-0 lock: locked for good
    + [(9.5, 'quiet', {'data': False}), (9.7, 'aaa', {})]
)
GENERIC_S = 10.5

# the generic sessions on the production topics: what all three share, then
# what is particular to the lock
GENERIC2_COMMON = (
    stream('low', 0.05, 1.0, 1.5)                                     # never (timeout 0) masks it from the start
    + stream('bbb', 0.05, 2.0, 3.0) + stream('aaa', 0.05, 2.3, 3.0)   # a tie: aaa first by name
    + stream('zero', 0.05, 3.5, 4.0)                                  # priority 0: never selected
)
GENERIC2 = {
    # a 45 lock: below it masked (never, low), 50 not (aaa); then never wins
    'high': (GENERIC2_COMMON + stream('lk', 0.05, 4.5, 5.5, data=True) + stream('low', 0.05, 4.6, 5.4)
             + [(5.0, 'never', {}), (5.2, 'aaa', {})] + [(6.0, 'never', {})] + stream('low', 0.05, 6.5, 7.0), 7.5),
    # a priority-0 lock: no effect
    'zerolock': (GENERIC2_COMMON + stream('lk', 0.05, 4.5, 5.0, data=True) + [(4.75, 'never', {})]
                 + stream('low', 0.05, 5.5, 6.0), 6.5),
    # a timeout-0 lock: locked for good after one true, free after one false
    'quiet': (GENERIC2_COMMON + [(4.5, 'lk', {'data': True}), (5.0, 'never', {}), (5.2, 'aaa', {}),
                                 (5.5, 'lk', {'data': False}), (5.7, 'aaa', {})], 6.5),
}


# ---------------------------------------------------------------------------
# run (inside the image)
# ---------------------------------------------------------------------------

def enable_lo_multicast():
    import fcntl
    import socket
    import struct
    SIOCGIFFLAGS, SIOCSIFFLAGS, IFF_MULTICAST = 0x8913, 0x8914, 0x1000
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    flags = struct.unpack('16sH', fcntl.ioctl(s, SIOCGIFFLAGS, struct.pack('16sH', b'lo', 0))[:18])[1]
    fcntl.ioctl(s, SIOCSIFFLAGS, struct.pack('16sH', b'lo', flags | IFF_MULTICAST))


class Muxes:
    """The processes under test of one session."""

    def __init__(self, out_dir, session, yaml_text_for, n_cpp, rust_bin):
        self.procs = []
        self.instances = []
        for k in range(1, n_cpp + 1):
            name = f'twist_mux_cpp{k}'
            path = os.path.join(out_dir, f'{session}_{name}.yaml')
            with open(path, 'w') as f:
                f.write(yaml_text_for(name))
            cmd = ['/opt/ros/jazzy/lib/twist_mux/twist_mux', '--ros-args', '-r', f'__node:={name}',
                   '--params-file', path, '-r', f'/cmd_vel_out:={cpp_out(name)}']
            self.instances.append({'name': name, 'kind': 'cpp', 'out': cpp_out(name), 'cmd': cmd})
        if rust_bin:
            path = os.path.join(out_dir, f'{session}_twist_mux.yaml')
            with open(path, 'w') as f:
                f.write(yaml_text_for('twist_mux'))
            cmd = [rust_bin, '--modules', 'mux', '--ros-args', '-r', 'twist_mux:cmd_vel_out:=/cmd_vel_guard_input',
                   '--params-file', path]
            self.instances.append({'name': 'twist_mux', 'kind': 'rust', 'out': RUST_OUT, 'cmd': cmd})
        self.out_dir, self.session = out_dir, session

    def start(self):
        env = dict(os.environ, RCUTILS_LOGGING_BUFFERED_STREAM='1')
        for inst in self.instances:
            log = open(os.path.join(self.out_dir, f'{self.session}_{inst["name"]}.log'), 'w')
            self.procs.append((subprocess.Popen(inst['cmd'], stdout=log, stderr=subprocess.STDOUT, env=env), log))

    def stop(self):
        for p, _ in self.procs:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
        deadline = time.monotonic() + 10
        codes = []
        for p, log in self.procs:
            try:
                codes.append(p.wait(max(0.1, deadline - time.monotonic())))
            except subprocess.TimeoutExpired:
                p.kill()
                codes.append('killed')
            log.close()
        return codes


def make_probe(keys, locks, instances):
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import TwistStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool

    def qos(depth):
        return QoSProfile(depth=depth, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.VOLATILE)

    class Probe(Node):
        def __init__(self):
            super().__init__('e4_mux_oracle')
            self.lk = threading.Lock()
            self.sent = []
            self.out = {i['name']: [] for i in instances}
            self.diag = []
            self.seq = {}
            self.pubs = {k: self.create_publisher(TwistStamped, t, qos(d)) for k, (t, d) in keys.items()}
            self.lock_pubs = {k: self.create_publisher(Bool, t, qos(d)) for k, (t, d) in locks.items()}
            for inst in instances:
                self.create_subscription(TwistStamped, inst['out'],
                                         (lambda n: lambda m: self._out(n, m))(inst['name']), qos(1000))
            self.create_subscription(DiagnosticArray, '/diagnostics', self._diag, qos(1000))

        def _out(self, name, m):
            t = time.monotonic()
            # (wall clock unused here; outputs are timed on the monotonic clock)
            tw = m.twist
            with self.lk:
                self.out[name].append([t, m.header.frame_id, m.header.stamp.sec, m.header.stamp.nanosec,
                                       tw.linear.x, tw.linear.y, tw.linear.z, tw.angular.x, tw.angular.y,
                                       tw.angular.z])

        def _diag(self, m):
            t, wall = time.monotonic(), time.time_ns()
            with self.lk:
                for s in m.status:
                    level = s.level if isinstance(s.level, int) else int.from_bytes(s.level, 'little')
                    self.diag.append([t, s.name, level, s.message, s.hardware_id,
                                      [[kv.key, kv.value] for kv in s.values],
                                      [m.header.stamp.sec, m.header.stamp.nanosec], m.header.frame_id, wall])

        def send(self, key, kw):
            if key in self.lock_pubs:
                msg = Bool(data=bool(kw.get('data')))
                t = time.monotonic()
                self.lock_pubs[key].publish(msg)
                with self.lk:
                    self.sent.append([t, key, 'lock', bool(msg.data), 0, 0, None])
                return
            n = self.seq.get(key, 0)
            self.seq[key] = n + 1
            m = TwistStamped()
            now = self.get_clock().now().nanoseconds
            stamp = {'zero': 0, 'future5': now + 5 * 10**9, 'past5': now - 5 * 10**9}.get(kw.get('stamp'), now)
            m.header.stamp.sec, m.header.stamp.nanosec = stamp // 10**9, stamp % 10**9
            m.header.frame_id = kw.get('frame') or f'{key}#{n}'
            tw = kw.get('twist') or (0.01 * (1 + list(keys).index(key)), 0.0, 0.0, 0.0, 0.0, 0.001 * n)
            (m.twist.linear.x, m.twist.linear.y, m.twist.linear.z,
             m.twist.angular.x, m.twist.angular.y, m.twist.angular.z) = tw
            t = time.monotonic()
            self.pubs[key].publish(m)
            with self.lk:
                self.sent.append([t, key, 'input', m.header.frame_id, m.header.stamp.sec, m.header.stamp.nanosec,
                                  list(tw)])

    return Probe()


def sleep_until(deadline):
    while True:
        d = deadline - time.monotonic()
        if d <= 0:
            return
        time.sleep(d - 0.0008 if d > 0.002 else 0)


def play(probe, script, t0):
    """Send `script` on time; after an event marked `anchor`, the rest keeps
    its distance to it as it was actually sent."""
    shift = 0.0
    for t, key, kw in sorted(script, key=lambda e: e[0]):
        sleep_until(t0 + t + shift)
        probe.send(key, kw)
        if kw.get('anchor'):
            shift = (time.monotonic() - t0) - t


def run_session(args, session, yaml_text_for, keys, locks, body):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    muxes = Muxes(args.out, session, yaml_text_for, args.cpp, args.rust)
    rclpy.init()
    probe = make_probe(keys, locks, muxes.instances)
    ex = SingleThreadedExecutor()
    ex.add_node(probe)
    threading.Thread(target=ex.spin, daemon=True).start()
    marks = body(probe, muxes)
    time.sleep(1.0)
    codes = muxes.stop()
    time.sleep(0.5)
    with probe.lk:
        doc = {'session': session, 'instances': [{k: v for k, v in i.items()} for i in muxes.instances],
               'config': yaml_text_for('X'), 'scenarios': marks, 'sent': probe.sent, 'out': probe.out,
               'diag': probe.diag, 'exit_codes': codes}
    with open(os.path.join(args.out, f'{session}.json'), 'w') as f:
        json.dump(doc, f)
    print(f'[oracle] {session}: {len(doc["sent"])} sent, outputs '
          + ', '.join(f'{k} {len(v)}' for k, v in doc['out'].items()) + f', exit {codes}', flush=True)
    ex.shutdown(timeout_sec=2.0)
    probe.destroy_node()
    rclpy.shutdown()


def wait_for_outputs(probe, instances, timeout=30.0):
    """Discovery: until every instance's output has the probe as a reader and
    the probe has every input's reader (polled, never slept for)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ok = all(probe.count_publishers(i['out']) >= 1 for i in instances)
        writers = list(probe.pubs.values()) + list(probe.lock_pubs.values())
        ok = ok and all(p.get_subscription_count() >= len(instances) for p in writers)
        if ok:
            return True
        time.sleep(0.1)
    return False


def cmd_run(args):
    os.makedirs(args.out, exist_ok=True)
    enable_lo_multicast()
    production = open(args.config).read()
    head, _, rest = production.partition('\n')
    assert head.strip() == 'twist_mux:', head

    def prod_yaml(node):
        return f'{node}:\n{rest}'

    def main_body(probe, muxes):
        muxes.start()
        if not wait_for_outputs(probe, muxes.instances):
            print('[oracle] discovery timed out', flush=True)
        time.sleep(2.5)
        marks = []
        for name, duration, script in scenario_list(args.random_s, args.seed):
            if args.scenarios and not any(name.startswith(s) for s in args.scenarios):
                continue
            t0 = time.monotonic()
            play(probe, script, t0)
            sleep_until(t0 + duration)
            marks.append([name, t0, t0 + duration])
            print(f'[oracle] {name} done', flush=True)
            time.sleep(GAP_S)
        return marks

    def startup_body(probe, muxes):
        t0 = time.monotonic()
        # from this thread: a fork from another one stalls the sender
        play(probe, [e for e in STARTUP if e[0] < STARTUP_MUX_START_S], t0)
        sleep_until(t0 + STARTUP_MUX_START_S)
        muxes.start()
        play(probe, [e for e in STARTUP if e[0] >= STARTUP_MUX_START_S], t0)
        sleep_until(t0 + STARTUP_S)
        return [['S15_startup', t0, t0 + STARTUP_S]]

    def generic2_body(variant):
        def body(probe, muxes):
            muxes.start()
            if not wait_for_outputs(probe, muxes.instances):
                print('[oracle] discovery timed out', flush=True)
            time.sleep(1.0)
            script, duration = GENERIC2[variant]
            t0 = time.monotonic()
            play(probe, script, t0)
            sleep_until(t0 + duration)
            return [[f'S18_generic_{variant}', t0, t0 + duration]]
        return body

    def generic_body(probe, muxes):
        muxes.start()
        if not wait_for_outputs(probe, muxes.instances):
            print('[oracle] discovery timed out', flush=True)
        # the start: the first diagnostics of every instance, which comes
        # with the construction ("Node starting up")
        time.sleep(1.0)
        t0 = time.monotonic()
        play(probe, GENERIC, t0)
        sleep_until(t0 + GENERIC_S)
        return [['S18_generic', t0, t0 + GENERIC_S]]

    sessions = args.sessions or ['main', 'startup', 'generic_high', 'generic_zerolock', 'generic_quiet']
    for variant, (timeout, priority) in GENERIC_LOCK_VARIANTS.items():
        if f'generic_{variant}' in sessions:
            run_session(args, f'generic_{variant}',
                        lambda n, v=variant, to=timeout, pr=priority: GENERIC2_YAML.format(
                            node=n, variant=v, timeout=to, priority=pr),
                        GENERIC2_KEYS, GENERIC2_LOCKS, generic2_body(variant))
    if 'main' in sessions:
        run_session(args, 'main', prod_yaml, PRODUCTION_KEYS, PRODUCTION_LOCKS, main_body)
    if 'startup' in sessions:
        run_session(args, 'startup', prod_yaml, PRODUCTION_KEYS, PRODUCTION_LOCKS, startup_body)
    if 'generic' in sessions:
        # the first form of S18 (C++ only: its /g_* names do not bridge)
        run_session(args, 'generic', lambda n: GENERIC_YAML.format(node=n), GENERIC_KEYS, GENERIC_LOCKS,
                    generic_body)
    os._exit(0)


# ---------------------------------------------------------------------------
# compare (anywhere)
# ---------------------------------------------------------------------------

def session_model(doc):
    config = parse_config(doc['config'])
    if doc['session'] == 'generic':
        keys, locks = GENERIC_KEYS, GENERIC_LOCKS
    elif doc['session'].startswith('generic_'):
        keys, locks = GENERIC2_KEYS, GENERIC2_LOCKS
    else:
        keys, locks = PRODUCTION_KEYS, PRODUCTION_LOCKS
    topic_handle = {h['topic']: h['name'] for h in config['inputs'] + config['locks']}
    key_handle = {k: topic_handle[t] for k, (t, _) in {**keys, **locks}.items()}
    return Model(config), key_handle


def events_of(doc, key_handle, t0, t1):
    """[t_ns since t0, kind, handle, frame_id | data] of what was sent in [t0, t1)."""
    out = []
    for t, key, kind, value, *_ in doc['sent']:
        if t0 <= t < t1:
            out.append([int(round((t - t0) * 1e9)), kind, key_handle[key], value])
    return out


def percentile(xs, p):
    if not xs:
        return float('nan')
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(math.ceil(p / 100.0 * len(xs))) - 1)]


SESSIONS = ('main', 'startup', 'generic', 'generic_high', 'generic_zerolock', 'generic_quiet')


def up_times(doc, names):
    """When each instance was first heard from (a status or an output)."""
    out = {}
    for name in names:
        times = [d[0] for d in doc['diag'] if d[1].startswith(name + ': ')] + [o[0] for o in doc['out'][name]]
        out[name] = min(times) if times else float('inf')
    return out


def cmd_compare(args):
    """Every instance against the first C++ one. With two C++ ones, the
    second measures the noise floor of this very run (the machine's load, the
    probe's stalls): another instance may differ from the reference by twice
    what the second C++ copy does, and its latency p99 by 5 ms more than the
    copy's (p50 by 1 ms whatever the noise). The model is reported, not
    required: it assumes every message arrived 0 ms after it was sent."""
    failures = []
    summary = {'sessions': {}}
    margin = int(args.margin * 1e9)
    for session in SESSIONS:
        path = os.path.join(args.dir, f'{session}.json')
        if not os.path.exists(path):
            continue
        doc = json.load(open(path))
        model, key_handle = session_model(doc)
        names = [i['name'] for i in doc['instances']]
        cpp = [i['name'] for i in doc['instances'] if i['kind'] == 'cpp']
        ref = cpp[0]
        noise_of = cpp[1] if len(cpp) > 1 else None
        sent_by_frame = {s[3]: s for s in doc['sent'] if s[2] == 'input'}
        srep = summary['sessions'][session] = {'scenarios': {}}
        up = up_times(doc, names)
        print(f'===== {session}: instances {names}, reference {ref}, noise floor {noise_of}, '
              f'exit codes {doc.get("exit_codes")}')
        # content: every output is the input it names, bit for bit
        for name in names:
            bad = []
            for o in doc['out'][name]:
                s = sent_by_frame.get(o[1])
                if s is None or (s[4], s[5]) != (o[2], o[3]) or [float(x) for x in s[6]] != o[4:10]:
                    bad.append(o[1])
            if bad:
                failures.append(f'{session} {name}: {len(bad)} outputs differ from the input they name: {bad[:5]}')
            print(f'  content {name}: {len(doc["out"][name])} outputs, {len(bad)} not verbatim')
        lat = {n: [] for n in names}
        for sc, t0, t1 in doc['scenarios']:
            end = t1 + GAP_S
            events = events_of(doc, key_handle, t0, end)
            decided = decisions(model, events, margin)
            ambiguous = {f for f, fw, amb in decided if amb}
            # the same input's next message arriving at the reference within
            # the margin of it, however far apart they were sent (a delivery
            # stall bunches them): a depth-1 reader further on may hold only
            # the newer one by the time its consumer takes it
            recv = {o[1]: o[0] for o in doc['out'][ref] if t0 <= o[0] < end}
            by_handle = {}
            for e in events:
                if e[1] == 'input':
                    by_handle.setdefault(e[2], []).append(e[3])
            for frames in by_handle.values():
                for f, g in zip(frames, frames[1:]):
                    if f in recv and g in recv and abs(recv[g] - recv[f]) <= args.margin:
                        ambiguous.add(f)
            start_ns = {s[3]: int(round((s[0] - t0) * 1e9)) for s in doc['sent'] if s[2] == 'input'}
            cut = None
            if session == 'startup':
                # nothing may go before the first lock false; the rest is
                # compared from the moment every instance was up
                first_lock = min(s[0] for s in doc['sent'] if s[2] == 'lock')
                for name in names:
                    early = [o[1] for o in doc['out'][name] if o[0] < first_lock]
                    if early:
                        failures.append(f'startup {name}: forwarded before the first lock false: {early[:5]}')
                cut = int(round((max(first_lock + 0.1, max(up.values()) + 0.5) - t0) * 1e9))
            keep = (lambda f: start_ns.get(f, 0) > cut) if cut is not None else (lambda f: True)
            expected = [f for f, fw, amb in decided if fw and f not in ambiguous and keep(f)]
            row = {name: [o[1] for o in doc['out'][name] if t0 <= o[0] < end] for name in names}
            clean = {name: [f for f in row[name] if f not in ambiguous and keep(f)] for name in names}
            rep = srep['scenarios'][sc] = {'model': len(expected), 'ambiguous': len(ambiguous), 'instances': {}}
            line = f'  {sc:<34} model {len(expected):5d} ambiguous {len(ambiguous):4d} |'
            for name in names:
                seq = row[name]
                if sc.startswith('S16'):
                    n = sum(1 for f in seq if f.startswith('joy#'))
                    verdict = f'joy {n}/200'
                    rep['instances'][name] = {'joy_forwarded': n}
                elif sc.startswith('S17'):
                    first = next((f for f in seq if f.startswith('safety#')), None)
                    verdict = f'first {first}'
                    rep['instances'][name] = {'first_safety': first}
                else:
                    ok = clean[name] == expected
                    verdict = 'model' if ok else 'not the model'
                    rep['instances'][name] = {'forwarded': len(seq), 'model': ok}
                    if not ok:
                        extra = [f for f in clean[name] if f not in expected]
                        missing = [f for f in expected if f not in clean[name]]
                        print(f'    {name} against the model: extra {len(extra)} {extra[:4]}, '
                              f'missing {len(missing)} {missing[:4]}')
                line += f' {name} {len(seq):5d} {verdict}'
            print(line)
            if sc.startswith('S16'):
                counts = {n: rep['instances'][n]['joy_forwarded'] for n in names}
                noise = abs(counts[noise_of] - counts[ref]) if noise_of else 0
                allowed = max(4, 2 * noise) if args.s16_tolerance < 0 else args.s16_tolerance
                for name in names:
                    if name not in (ref, noise_of) and abs(counts[name] - counts[ref]) > allowed:
                        failures.append(f'{session} S16 {name}: joy forwarded {counts[name]} against {counts[ref]} '
                                        f'(allowed {allowed})')
                continue
            if sc.startswith('S17'):
                firsts = [rep['instances'][n]['first_safety'] for n in names]
                seqs = [int(f.split('#')[1]) for f in firsts if f]
                if len(seqs) != len(firsts) or max(seqs) - min(seqs) > 1:
                    failures.append(f'{session} S17: first forwarded safety {firsts} differ by more than one')
                continue
            # against the reference, away from the boundaries
            diffs = {}
            for name in names[1:]:
                ca, cb = clean[ref], clean[name]
                differ = sorted(set(ca) ^ set(cb))
                order = [f for f in ca if f in set(cb)] != [f for f in cb if f in set(ca)]
                diffs[name] = (differ, order)
                rep.setdefault('vs_reference', {})[name] = {'differ': len(differ), 'order': order}
                if differ or order:
                    print(f'    {name} vs {ref}: {len(differ)} decisions differ {differ[:6]}, order {order}')
            noise = len(diffs[noise_of][0]) if noise_of else 0
            for name, (differ, order) in diffs.items():
                if name == noise_of:
                    continue
                allowed = 2 * noise if noise_of else (args.random_tolerance if sc.startswith('R01') else 0)
                if order or len(differ) > allowed:
                    failures.append(f'{session} {sc} {name}: {len(differ)} decisions differ from {ref} '
                                    f'(allowed {allowed}, the noise floor {noise}), order differs: {order}: '
                                    f'{differ[:6]}')
            for name in names:
                for o in doc['out'][name]:
                    if t0 <= o[0] < end and o[1] in sent_by_frame:
                        lat[name].append((o[1], o[0] - sent_by_frame[o[1]][0]))
        ref_lat = dict(lat[ref])
        noise_p99 = None
        for name in names:
            xs = [l for _, l in lat[name]]
            print(f'  latency {name}: n {len(xs)} p50 {percentile(xs, 50)*1e3:.2f} p90 {percentile(xs, 90)*1e3:.2f} '
                  f'p99 {percentile(xs, 99)*1e3:.2f} max {max(xs, default=float("nan"))*1e3:.2f} ms')
            srep.setdefault('latency_ms', {})[name] = {'n': len(xs), 'p50': percentile(xs, 50) * 1e3,
                                                     'p99': percentile(xs, 99) * 1e3}
        diffs = {}
        for name in names[1:]:
            d = [l - ref_lat[f] for f, l in lat[name] if f in ref_lat]
            p50, p99 = percentile(d, 50) * 1e3, percentile(d, 99) * 1e3
            diffs[name] = (p50, p99)
            print(f'  latency {name} - {ref}: n {len(d)} p50 {p50:+.2f} p99 {p99:+.2f} '
                  f'max {max(d, default=float("nan"))*1e3:+.2f} ms')
            srep.setdefault('latency_diff_ms', {})[name] = {'n': len(d), 'p50': p50, 'p99': p99}
        if noise_of:
            noise_p99 = diffs[noise_of][1]
        for name, (p50, p99) in diffs.items():
            if name == noise_of or not args.latency:
                continue
            p99_allowed = max(args.p99_ms, (noise_p99 or 0.0) + args.p99_ms)
            if not (p50 <= args.p50_ms and p99 <= p99_allowed):
                failures.append(f'{session} {name}: latency over {ref} p50 {p50:.2f} ms (allowed {args.p50_ms}) / '
                                f'p99 {p99:.2f} ms (allowed {p99_allowed:.2f})')
        check_diagnostics(doc, model, key_handle, names, failures, margin, srep)
    print()
    if failures:
        print(f'MUX_ORACLE_FAIL ({len(failures)})')
        for f in failures:
            print('  ' + f)
    else:
        print('MUX_ORACLE_OK')
    with open(os.path.join(args.dir, 'compare.json'), 'w') as f:
        json.dump({'failures': failures, 'summary': summary}, f, indent=1)
    return 1 if failures else 0


def check_diagnostics(doc, model, key_handle, names, failures, margin, srep):
    """The first status of each instance is diagnostic_updater's "Node
    starting up" (published as the task is added, no values; a reader that
    matches the new writer late misses it, so it is reported, not
    required); then two a second. Each must hold what the model holds when it
    was published (its header stamp, on the monotonic clock through the
    receive time): masked / locked under the current lock priority, or
    under the one of twist_mux's previous 1 s tick. The two timers of a second
    run in an order rclcpp's executor does not fix, and the updater's message
    carries the priority of twist_mux's last tick: when it comes first, a
    second old (its first one has no handles at all: twist_mux has not
    handed it its status yet). The two timers are on two clocks (a wall timer
    and the node clock), so which comes first can change while a process runs.
    The statics (level, message, hardware id, keys, the topic/timeout/priority
    text) must be the same on every instance."""
    tl = Timeline(events_of(doc, key_handle, 0.0, float('inf')))
    statics = {}
    slack = margin + 2_000_000
    for name in names:
        ds = [d for d in doc['diag'] if d[1].startswith(name + ': ')]
        if not ds:
            failures.append(f'{doc["session"]} {name}: no diagnostics')
            continue
        first = ds[0]
        starting = first[3] == 'Node starting up' and first[5] == [] and first[2] == 0
        rest = [d for d in ds if d[3] != 'Node starting up']
        span = rest[-1][0] - rest[0][0] if len(rest) > 1 else 0.0
        fresh, stale, wrong, random_wrong = 0, 0, 0, 0
        stat = None
        for d in rest:
            t, sname, level, message, hwid, values, stamp, frame = d[:8]
            # publish time on the monotonic clock
            tp = t
            if len(d) > 8:
                lag = (d[8] - (stamp[0] * 10**9 + stamp[1])) / 1e9
                tp = t - min(max(lag, 0.0), 1.0)
            stat = (sname.split(': ', 1)[1], level, message, hwid, tuple(k_ for k_, _ in values),
                    tuple(v.split('(', 1)[1] if '(' in v else (v if k_.endswith('[sec]') else '')
                          for k_, v in values))
            got = {}
            for k_, v in values:
                if k_.startswith('velocity ') or k_.startswith('lock '):
                    got[k_] = v.strip().split(' ', 1)[0]
                elif k_ == 'current priority':
                    got[k_] = v
            tn = int(round(tp * 1e9))
            if d is rest[0] and list(got) == ['current priority'] and got['current priority'] == '0':
                # the updater's first tick before twist_mux's: its status has
                # no handles yet
                stale += 1
                continue
            windows = [(c, at) for c in (tn - slack, tn + slack) for at in (tn - slack, tn + slack)]
            if any(got == model.diag(tl.state(c), at) for c, at in windows):
                fresh += 1
                continue
            prev = tn - 1_000_000_000
            stale_lps = {model.lock_priority(tl.state(prev + dt), prev + dt) for dt in (-20_000_000, 0, 20_000_000)}
            if tp - rest[0][0] < 0.5:
                stale_lps.add(0)    # the first tick: twist_mux's status starts at priority 0
            if any(got == model.diag(tl.state(c), at, lp) for c, at in windows for lp in stale_lps):
                stale += 1
            elif any(sc.startswith('R01') and t0 <= tp < t1 for sc, t0, t1 in doc['scenarios']):
                random_wrong += 1   # the probe's stalls delay what the mux hears
            else:
                wrong += 1
        statics[name] = stat
        # pairs: the first of the first one to the last of the last one
        rate = (len(rest) - 2) / span if span > 0 else 0.0
        print(f'  diagnostics {name}: {len(ds)} msgs, starts with "Node starting up": {starting}, {rate:.2f}/s over '
              f'{span:.1f} s, current {fresh}, last tick\'s priority {stale}, wrong {wrong} (+{random_wrong} in R01)')
        srep.setdefault('diagnostics', {})[name] = {'msgs': len(ds), 'starting_up': starting, 'per_s': rate,
                                                    'span_s': span, 'fresh': fresh, 'stale': stale, 'wrong': wrong,
                                                    'random_wrong': random_wrong}
        if wrong or random_wrong > 0.02 * len(rest):
            failures.append(f'{doc["session"]} {name}: {wrong} (+{random_wrong} in R01) diagnostics disagree with '
                            f'the model')
        if span > 2 and abs(rate - 2.0) > 0.1:
            failures.append(f'{doc["session"]} {name}: {rate:.2f} diagnostics a second, not two')
    ref = names[0]
    for name in names[1:]:
        if statics.get(name) != statics.get(ref):
            failures.append(f'{doc["session"]} {name}: diagnostics fields differ from {ref}: {statics.get(name)} vs '
                            f'{statics.get(ref)}')


# ---------------------------------------------------------------------------
# vectors (anywhere): what mower_mux's replay test reads
# ---------------------------------------------------------------------------

def cmd_vectors(args):
    os.makedirs(args.dest, exist_ok=True)
    written = []
    for session in SESSIONS:
        path = os.path.join(args.dir, f'{session}.json')
        if not os.path.exists(path):
            continue
        doc = json.load(open(path))
        model, key_handle = session_model(doc)
        inst = next(i['name'] for i in doc['instances'] if i['kind'] == 'cpp')
        config = parse_config(doc['config'])
        for sc, t0, t1 in doc['scenarios']:
            if sc.startswith('R01'):
                t1 = t0 + args.random_s
            end = t1 + GAP_S
            events = events_of(doc, key_handle, t0, end)
            # received 1 ms after it was sent
            events = [[e[0] + 1_000_000] + e[1:] for e in events]
            decided = decisions(model, events, int(args.margin * 1e9))
            near = {f for f, _, amb in decided if amb}
            forwarded = [o[1] for o in doc['out'][inst] if t0 <= o[0] < end]
            if sc.startswith('R01'):
                frames = {e[3] for e in events if e[1] == 'input'}
                forwarded = [f for f in forwarded if f in frames]
                # what no arrival time can explain: a delivery that stalled
                # (any output 20 ms or more late) leaves what the muxes heard
                # unknown from then on for up to the longest timeout, and the
                # instances disagree there
                sent_at = {s[3]: s[0] for s in doc['sent'] if s[2] == 'input'}
                stalls = [(sent_at[o[1]] - 0.3, o[0] + 0.6) for i in doc['instances'] for o in doc['out'][i['name']]
                          if o[1] in sent_at and o[0] - sent_at[o[1]] > 0.02]
                for f in frames:
                    if any(a <= sent_at[f] <= b for a, b in stalls):
                        near.add(f)
                rows = [{o[1] for o in doc['out'][i['name']] if t0 <= o[0] < end} for i in doc['instances']]
                near |= set.union(*rows) - set.intersection(*rows)
            vec = {
                'scenario': sc,
                'source': f'twist_mux 4.5.0 ({inst}), tools/mux_oracle.py session {session}',
                'config': config,
                'events': events,
                'forwarded': forwarded,
                'near_boundary': sorted(near),
                'compare': 'count' if sc.startswith('S16') else 'first_within_one' if sc.startswith('S17')
                else 'after_lock' if sc.startswith('S15') else 'exact',
            }
            out = os.path.join(args.dest, f'{sc.lower()}.json')
            with open(out, 'w') as f:
                json.dump(vec, f, separators=(',', ':'))
                f.write('\n')
            written.append(out)
    print('\n'.join(written))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('run')
    r.add_argument('--out', required=True)
    r.add_argument('--cpp', type=int, default=2, help='C++ twist_mux instances (2: the noise floor)')
    r.add_argument('--rust', help='mower_rsd to run with --modules mux')
    r.add_argument('--config', default=PRODUCTION_YAML)
    r.add_argument('--sessions', nargs='*',
                   choices=['main', 'startup', 'generic', 'generic_high', 'generic_zerolock', 'generic_quiet'])
    r.add_argument('--scenarios', nargs='*', help='prefixes, e.g. S03 S12')
    r.add_argument('--random-s', type=int, default=600)
    r.add_argument('--seed', type=int, default=4)
    c = sub.add_parser('compare')
    c.add_argument('dir')
    c.add_argument('--margin', type=float, default=0.003)
    c.add_argument('--latency', action='store_true', help='fail on the latency thresholds')
    c.add_argument('--random-tolerance', type=int, default=10,
                   help='R01: decisions away from a boundary that may differ from the reference (the noise floor)')
    c.add_argument('--s16-tolerance', type=int, default=-1,
                   help='S16: the most the joy counts may differ (the noise floor; -1: report only)')
    c.add_argument('--p50-ms', type=float, default=1.0)
    c.add_argument('--p99-ms', type=float, default=5.0)
    v = sub.add_parser('vectors')
    v.add_argument('dir')
    v.add_argument('dest')
    v.add_argument('--margin', type=float, default=0.003)
    v.add_argument('--random-s', type=int, default=120, help='how much of the random segment to keep')
    args = ap.parse_args()
    if args.cmd == 'run':
        cmd_run(args)
    elif args.cmd == 'compare':
        sys.exit(cmd_compare(args))
    else:
        cmd_vectors(args)


if __name__ == '__main__':
    main()
