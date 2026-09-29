#!/usr/bin/env python3
"""Shadow comparison: the mower_rs nodes as separate binaries vs. as one
``mower_rsd`` process, fed the same synthetic inputs.

Phase A5 of docs/ROS_FREE_PLAN.md keeps the same acceptance rule as the
earlier ports: the daemon is only a packaging change, so every module's
output must be identical to the separate binary's, byte for byte once the
unavoidably time-dependent fields are normalised away.

Run it twice from the same fixtures and diff::

    ./shadow_compare.py --mode split  --bin-dir <target>/release --out /tmp/split.json
    ./shadow_compare.py --mode daemon --bin-dir <target>/release --out /tmp/daemon.json
    ./shadow_compare.py --diff /tmp/split.json /tmp/daemon.json

The harness is one rclpy node that

* serves the graph the modules expect: ``/boustrophedon_coverage`` and
  ``/map_manage`` parameter services, ``/get_zone_map_list_srv``, ``/toLL``;
* publishes a fixed input script (odometry, GPS, IMU, base telemetry, the
  map and marker layers, and joystick / mux velocity commands, the manual
  one stamped with the session id the guard advertises);
* records every output topic of robot_status, mower_adapter, mower_battery
  and the two velocity guards.

Normalisation (identical rules for both runs): message stamps and the
wall-clock / age / uptime fields inside the JSON payloads are dropped,
because they are sampling-instant noise in either configuration. Everything
else — values, key order, layer bytes, the `/robot/online` sequence, the
guard's accept/reject decisions — has to match exactly.
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import re
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters
from robot_localization.srv import ToLL
from sensor_msgs.msg import BatteryState, Imu, NavSatFix
from std_msgs.msg import Bool, Header, String
from visualization_msgs.msg import Marker, MarkerArray

from mower_interface.msg import ZoneMap
from mower_interface.srv import ZoneMapList

LATCHED = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
BEST_EFFORT = QoSProfile(
    depth=1, history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)

# Output topics recorded from each module under test.
OUTPUTS = [
    ('/robot/online', Bool, LATCHED),
    ('/robot/info', String, LATCHED),
    ('/robot/telemetry', String, 10),
    ('/mower_base/led_command', String, LATCHED),
    ('/battery_state', BatteryState, 10),
    ('/aon_battery_state', BatteryState, 10),
    ('/joy_cmd', TwistStamped, 10),
    ('/drivetrain_guarded_cmd_vel', TwistStamped, 10),
    ('/manual_command_clock', Header, 10),
    ('/adapter/robot_pose', PoseStamped, 10),
    ('/adapter/map_layers/map_grid', String, LATCHED),
    ('/adapter/map_layers/free_space_inflated', String, LATCHED),
    ('/adapter/map_layers/risk_map_inflated', String, LATCHED),
    ('/adapter/map_layers/chennal_map_inflated', String, LATCHED),
    ('/adapter/marker_layers/zones', String, LATCHED),
    ('/adapter/marker_layers/risk_zones', String, LATCHED),
    ('/adapter/marker_layers/channels', String, LATCHED),
    ('/adapter/marker_layers/coverage_path', String, LATCHED),
    ('/adapter/marker_layers/invalid_segments', String, LATCHED),
    ('/adapter/marker_layers/connectors', String, LATCHED),
    ('/adapter/coverage_settings', String, LATCHED),
    ('/adapter/zone_summaries', String, LATCHED),
    ('/adapter/map_datum', String, LATCHED),
]

# Fields that are sampling noise rather than behaviour: dropped from both
# runs before the comparison.
VOLATILE_KEYS = re.compile(
    r'(^|_)(age_s|uptime_s|uptime|time|timestamp|time_s|stamp|_ts|now|'
    r'last_seen|since_s|elapsed_s)$'
)
# Machine state, not module behaviour: the container's load and temperature
# move between the two runs whatever the code does.
VOLATILE_PATHS = {('host', 'load1'), ('host', 'mem_used_pct'),
                  ('host', 'cpu_temp_c'), ('host', 'uptime_s')}
# `manual-session-v1:<32 hex>` from /dev/urandom at every start: different by
# design in every process, and the guard's rule (only its own id is accepted)
# is what /joy_cmd proves.
SESSION_ID = re.compile(r'^manual-session-v1:[0-9a-f]{32}$')

COVERAGE_PARAMS = {
    'strip_width_m': 0.35,
    'waypoint_spacing_m': 0.2,
    'zigzag_angle_deg': 30.0,
    'pattern': 'zigzag',
    'boundary_ring': True,
    'unknown_as_obstacle': False,
}
MAP_PARAMS = {'inflate_radius_m': 0.75, 'chennal_width_m': 1.2}


def parameter_value(value):
    pv = ParameterValue()
    if isinstance(value, bool):
        pv.type = ParameterType.PARAMETER_BOOL
        pv.bool_value = value
    elif isinstance(value, int):
        pv.type = ParameterType.PARAMETER_INTEGER
        pv.integer_value = value
    elif isinstance(value, float):
        pv.type = ParameterType.PARAMETER_DOUBLE
        pv.double_value = value
    else:
        pv.type = ParameterType.PARAMETER_STRING
        pv.string_value = str(value)
    return pv


def grid(name, width, height, resolution, seed):
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.width = width
    msg.info.height = height
    msg.info.resolution = resolution
    msg.info.origin.position.x = -1.0
    msg.info.origin.position.y = -2.0
    msg.info.origin.orientation.w = 1.0
    msg.data = [((x * 7 + seed) % 101) - 1 for x in range(width * height)]
    return msg


def markers(ns, count, seed):
    array = MarkerArray()
    for i in range(count):
        m = Marker()
        m.header.frame_id = 'map'
        m.ns = ns
        m.id = i
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.05
        m.color.r = ((seed + i) % 10) / 10.0
        m.color.g = 0.5
        m.color.b = 0.25
        m.color.a = 1.0
        for k in range(4):
            p = Marker().pose.position.__class__()
            p.x = float(i + k)
            p.y = float(seed - k)
            p.z = 0.0
            m.points.append(p)
        array.markers.append(m)
    return array


def base_telemetry(step):
    """The STM32 frame both robot_status and mower_battery read."""
    return json.dumps({
        'charger': {
            'valid': True, 'online': True,
            'voltage_v': round(24.0 + (step % 5) * 0.1, 3),
            'current_a': round(1.5 + (step % 3) * 0.25, 3),
        },
        'analog': {
            'valid': True,
            'main_battery_valid': True,
            'main_battery_v': round(23.9 + (step % 4) * 0.05, 3),
            'aon_battery_valid': True,
            'aon_battery_v': 3.78,
        },
        'pid': {'kp': 1.0, 'ki': 0.2, 'kd': 0.0},
        'lights': {'head': False},
    }, separators=(',', ':'))


class Harness(Node):
    def __init__(self, steps, rate_hz, warmup_steps):
        super().__init__('shadow_harness')
        self.steps = steps
        self.warmup_steps = warmup_steps
        self.period = 1.0 / rate_hz
        self.records = {}
        self.session = None

        # --- the graph the modules under test expect ----------------------
        self.create_service(
            GetParameters, '/boustrophedon_coverage/get_parameters',
            lambda req, res: self._params(COVERAGE_PARAMS, req, res))
        self.create_service(
            GetParameters, '/map_manage/get_parameters',
            lambda req, res: self._params(MAP_PARAMS, req, res))
        self.create_service(
            ZoneMapList, '/get_zone_map_list_srv', self._zone_map_list)
        self.create_service(ToLL, '/toLL', self._to_ll)

        # --- inputs --------------------------------------------------------
        self.pub = {
            'odom_slow': self.create_publisher(Odometry, '/odom_slow', 10),
            'global_slow': self.create_publisher(
                Odometry, '/odometry/global_slow', 10),
            'fix': self.create_publisher(
                NavSatFix, '/fix', qos_profile_sensor_data),
            'filtered': self.create_publisher(
                NavSatFix, '/gps/filtered', qos_profile_sensor_data),
            'imu': self.create_publisher(
                Imu, '/imu/data', qos_profile_sensor_data),
            'base': self.create_publisher(
                String, '/mower_base/telemetry', BEST_EFFORT),
            'nav_active': self.create_publisher(
                Bool, '/nav_operation_active', LATCHED),
            'mux': self.create_publisher(
                TwistStamped, '/cmd_vel_guard_input', 10),
            'joy': self.create_publisher(TwistStamped, '/app_joy_cmd', 10),
        }
        for topic, _name in [('/map_grid', 'a'), ('/free_space_inflated', 'b'),
                             ('/risk_map_inflated', 'c'),
                             ('/chennal_map_inflated', 'd')]:
            self.pub[topic] = self.create_publisher(
                OccupancyGrid, topic, LATCHED)
        for topic in ['/zone_list', '/risk_zone_list', '/chennal_path_array',
                      '/coverage_path_markers', '/coverage_invalid_segments',
                      '/coverage_connectors']:
            self.pub[topic] = self.create_publisher(
                MarkerArray, topic, LATCHED)

        # --- outputs -------------------------------------------------------
        for topic, msg_type, qos in OUTPUTS:
            self.records[topic] = []
            self.create_subscription(
                msg_type, topic,
                (lambda t: lambda m: self._record(t, m))(topic), qos)
        # the manual guard's session id has to come back in our stamps
        self.create_subscription(
            Header, '/manual_command_clock', self._on_clock, 10)

        self.step = 0
        self.timer = None
        self.startup = None

    def start(self):
        """Begin the input script. Called once both configurations are up.

        The script starts at a negative step: that is the warm-up, where the
        streaming inputs are already flowing but nothing is recorded, so the
        comparison does not depend on which tick each configuration happened
        to see first. Recording — and the latched map / marker inputs — begin
        at step 0, identically for both."""
        self.startup = {t: list(v) for t, v in self.records.items()}
        self.records = {t: [] for t in self.records}
        self.step = -self.warmup_steps
        self.timer = self.create_timer(self.period, self._tick)

    # -- fake services ------------------------------------------------------
    def _params(self, table, req, res):
        res.values = [parameter_value(table.get(n, '')) for n in req.names]
        return res

    def _zone_map_list(self, req, res):
        for i in range(2):
            z = ZoneMap()
            z.zone_id = i + 3
            z.mask_map_inflated = grid('zone', 4, 3, 0.05, i)
            pose = PoseStamped()
            pose.pose.orientation.w = 1.0
            z.path.poses = [pose] * (i + 1)
            res.zone_map_list.append(z)
        return res

    def _to_ll(self, req, res):
        # A fixed, non-zero datum so the adapter locks the navsat branch
        # rather than the fallback, in both runs.
        res.ll_point = GeoPoint()
        res.ll_point.latitude = 23.5 + req.map_point.x * 1e-5
        res.ll_point.longitude = 120.5 + req.map_point.y * 1e-5
        res.ll_point.altitude = 0.0
        return res

    def _on_clock(self, msg):
        self.session = msg.frame_id

    # -- input script -------------------------------------------------------
    def _tick(self):
        s = self.step
        self.step += 1
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = 0.1 * s
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = 0.25
        self.pub['odom_slow'].publish(odom)

        glob = Odometry()
        glob.header.stamp = now
        glob.header.frame_id = 'map'
        glob.child_frame_id = 'base_footprint'
        glob.pose.pose.position.x = 0.2 * s
        glob.pose.pose.position.y = -0.1 * s
        glob.pose.pose.orientation.w = 1.0
        self.pub['global_slow'].publish(glob)

        fix = NavSatFix()
        fix.header.stamp = now
        fix.header.frame_id = 'gps_link'
        fix.status.status = 2
        fix.latitude = 23.5 + s * 1e-6
        fix.longitude = 120.5 + s * 1e-6
        fix.altitude = 30.0
        fix.position_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0,
                                   0.0, 0.0, 0.04]
        fix.position_covariance_type = 2
        self.pub['fix'].publish(fix)
        self.pub['filtered'].publish(fix)

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = 'imu_link'
        imu.orientation.w = 1.0
        imu.angular_velocity.z = 0.01 * (s % 7)
        imu.linear_acceleration.x = 0.05 * (s % 5)
        imu.linear_acceleration.z = 9.81
        self.pub['imu'].publish(imu)

        self.pub['base'].publish(String(data=base_telemetry(s)))

        if s == 0:
            # everything before this was warm-up
            for t in self.records:
                self.records[t] = []
            self.pub['nav_active'].publish(Bool(data=False))
            for i, topic in enumerate(['/map_grid', '/free_space_inflated',
                                       '/risk_map_inflated',
                                       '/chennal_map_inflated']):
                self.pub[topic].publish(grid(topic, 8, 6, 0.05, i))
            for i, topic in enumerate(
                    ['/zone_list', '/risk_zone_list', '/chennal_path_array',
                     '/coverage_path_markers', '/coverage_invalid_segments',
                     '/coverage_connectors']):
                self.pub[topic].publish(markers(topic, 2, i))

        # Velocity commands: valid, then out of range, then stale, so the
        # guards' accept / reject / forced-zero paths are all exercised.
        cmd = TwistStamped()
        cmd.header.stamp = now
        cmd.header.frame_id = 'base_footprint'
        if s % 10 == 4:
            cmd.twist.linear.x = 9.0          # over max_linear_x -> reject
        elif s % 10 == 7:
            cmd.header.stamp.sec -= 5         # stale -> reject
            cmd.twist.linear.x = 0.2
        else:
            cmd.twist.linear.x = round(0.05 * (s % 6), 3)
            cmd.twist.angular.z = round(0.1 * (s % 3), 3)
        self.pub['mux'].publish(cmd)

        joy = TwistStamped()
        joy.header.stamp = now
        # wrong id before the clock arrives, correct id after: both guards'
        # session rule is part of what is compared
        joy.header.frame_id = self.session or 'no-session'
        joy.twist.linear.x = round(0.02 * (s % 4), 3)
        self.pub['joy'].publish(joy)

    # -- recording ----------------------------------------------------------
    def _record(self, topic, msg):
        self.records[topic].append(normalise(msg))

    def done(self):
        return self.step >= self.steps


def normalise(msg):
    """A message as plain data, with the time-dependent parts removed."""
    def field(value):
        if hasattr(value, 'get_fields_and_field_types'):
            out = {}
            for name in value.get_fields_and_field_types():
                if name == 'stamp':
                    continue
                out[name] = field(getattr(value, name))
            return out
        if isinstance(value, (bytes, bytearray)):
            return base64.b64encode(bytes(value)).decode()
        if isinstance(value, (list, tuple)) or hasattr(value, 'tolist'):
            items = value.tolist() if hasattr(value, 'tolist') else list(value)
            return [field(v) for v in items]
        if isinstance(value, float):
            # NaN / inf are legitimate values here (BatteryState); keep them
            # distinguishable and stable across json round-trips.
            if value != value:
                return 'nan'
            if value in (float('inf'), float('-inf')):
                return str(value)
            return round(value, 9)
        if isinstance(value, str) and SESSION_ID.match(value):
            return 'manual-session-v1:<random>'
        return value

    data = field(msg)
    if set(data) == {'data'} and isinstance(data['data'], str):
        text = data['data']
        try:
            return strip_volatile(json.loads(text))
        except (ValueError, TypeError):
            return text
    return data


def strip_volatile(value, path=()):
    if isinstance(value, dict):
        return {k: strip_volatile(v, path + (k,)) for k, v in value.items()
                if not VOLATILE_KEYS.search(k)
                and (path + (k,)) not in VOLATILE_PATHS}
    if isinstance(value, list):
        return [strip_volatile(v, path) for v in value]
    if isinstance(value, float):
        return round(value, 9)
    if isinstance(value, str) and SESSION_ID.match(value):
        return 'manual-session-v1:<random>'
    return value


SPLIT_BINARIES = [
    ('robot_status', ['robot_status'], []),
    ('mower_battery', ['mower_battery'], []),
    ('mower_adapter', ['mower_adapter'], []),
    ('velocity_command_guard', ['velocity_command_guard'], [
        '-r', '__node:=manual_velocity_guard',
        '-r', 'cmd_vel_in:=/app_joy_cmd',
        '-r', 'cmd_vel_out:=/joy_cmd',
        '-r', 'command_clock:=/manual_command_clock',
    ]),
    ('velocity_command_guard', ['velocity_command_guard'], [
        '-r', 'cmd_vel_in:=/cmd_vel_guard_input',
        '-r', 'cmd_vel_out:=/drivetrain_guarded_cmd_vel',
    ]),
]

DAEMON_REMAPS = [
    '-r', 'manual_velocity_guard:cmd_vel_in:=/app_joy_cmd',
    '-r', 'manual_velocity_guard:cmd_vel_out:=/joy_cmd',
    '-r', 'manual_velocity_guard:command_clock:=/manual_command_clock',
    '-r', 'velocity_command_guard:cmd_vel_in:=/cmd_vel_guard_input',
    '-r', 'velocity_command_guard:cmd_vel_out:=/drivetrain_guarded_cmd_vel',
]


# Module set -> the binaries that make it up, for the CPU comparison. The
# first four are the ones the shadow comparison drives with real inputs; the
# rest are added to show what an idle process costs just by existing.
EXTRA_BINARIES = [
    ('map', 'mower_map', ['-r', '__node:=map_manage']),
    ('coverage', 'mower_coverage', ['-r', '__node:=boustrophedon_coverage']),
    ('nav', 'mower_nav', ['-r', '__node:=nav_action_server']),
    ('record', 'mower_record', ['-r', '__node:=path_record_node']),
    ('pid_autotune', 'mower_pid_autotune', ['-r', '__node:=pid_autotune']),
]
DEFAULT_MODULES = 'status,battery,adapter,guards'


def spawn(mode, bin_dir, params_file, log_dir, modules=DEFAULT_MODULES):
    wanted = modules.split(',')
    procs = []
    if mode == 'split':
        for i, (binary, argv, ros) in enumerate(SPLIT_BINARIES):
            cmd = [os.path.join(bin_dir, binary)] + argv[1:] + [
                '--ros-args', '--params-file', params_file] + ros
            procs.append(run(cmd, log_dir, f'{i}_{binary}'))
        for module, binary, ros in EXTRA_BINARIES:
            if module in wanted:
                cmd = [os.path.join(bin_dir, binary), '--ros-args',
                       '--params-file', params_file] + ros
                procs.append(run(cmd, log_dir, binary))
    else:
        cmd = [os.path.join(bin_dir, 'mower_rsd'),
               '--modules', modules,
               '--ros-args', '--params-file', params_file] + DAEMON_REMAPS
        procs.append(run(cmd, log_dir, 'mower_rsd'))
    return procs


HZ = os.sysconf('SC_CLK_TCK')


def proc_cpu(pid):
    """(utime + stime) in seconds and the thread count, from /proc."""
    try:
        with open(f'/proc/{pid}/stat') as f:
            fields = f.read().rsplit(') ', 1)[1].split()
        cpu = (int(fields[11]) + int(fields[12])) / HZ
        threads = len(os.listdir(f'/proc/{pid}/task'))
        return cpu, threads
    except (OSError, IndexError, ValueError):
        return None, None


def measure(procs, seconds, spin):
    """utime+stime over `seconds`, per process, while `spin()` keeps the fake
    inputs flowing at the same rate for both configurations."""
    start = {p.pid: proc_cpu(p.pid) for p in procs}
    t0 = time.time()
    while time.time() - t0 < seconds:
        spin()
    elapsed = time.time() - t0
    rows = []
    for p in procs:
        cpu1, threads = proc_cpu(p.pid)
        cpu0, _ = start[p.pid]
        if cpu0 is None or cpu1 is None:
            continue
        rows.append((p.pid, os.path.basename(p.args[0]),
                     100.0 * (cpu1 - cpu0) / elapsed, threads))
    return elapsed, rows


def run(cmd, log_dir, name):
    log = open(os.path.join(log_dir, f'{name}.log'), 'w')
    return subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=['split', 'daemon'])
    ap.add_argument('--bin-dir')
    ap.add_argument('--params-file')
    ap.add_argument('--log-dir', default='/tmp/shadow')
    ap.add_argument('--out')
    ap.add_argument('--steps', type=int, default=120)
    ap.add_argument('--rate', type=float, default=10.0)
    ap.add_argument('--settle', type=float, default=4.0)
    ap.add_argument('--warmup', type=float, default=3.0)
    ap.add_argument('--diff', nargs=2)
    ap.add_argument('--modules', default=DEFAULT_MODULES,
                    help='module set for --measure (split spawns the matching '
                         'binaries)')
    ap.add_argument('--measure', type=float, default=0.0,
                    help='instead of recording outputs, sample /proc for this '
                         'many seconds and print CPU and thread counts')
    args = ap.parse_args()

    if args.diff:
        return diff(*args.diff)

    os.makedirs(args.log_dir, exist_ok=True)
    rclpy.init()
    harness = Harness(args.steps, args.rate,
                      int(args.warmup * args.rate))
    # The harness's services and latched inputs have to exist before the
    # modules look for them, exactly as the real graph does.
    deadline = time.time() + 1.0
    while time.time() < deadline:
        rclpy.spin_once(harness, timeout_sec=0.05)

    procs = spawn(args.mode, args.bin_dir, args.params_file, args.log_dir,
                  args.modules)
    try:
        if args.measure:
            deadline = time.time() + args.settle
            while time.time() < deadline:
                rclpy.spin_once(harness, timeout_sec=0.05)
            harness.start()
            elapsed, rows = measure(
                procs, args.measure,
                lambda: rclpy.spin_once(harness, timeout_sec=0.02))
            total_cpu = sum(r[2] for r in rows)
            total_threads = sum(r[3] for r in rows)
            print(f'{args.mode}: modules {args.modules}, '
                  f'{elapsed:.1f} s window')
            print(f'{"pid":>8} {"process":<26} {"cpu % of one core":>18} '
                  f'{"threads":>8}')
            for pid, name, cpu, threads in rows:
                print(f'{pid:>8} {name:<26} {cpu:>18.2f} {threads:>8}')
            print(f'{"":>8} {"TOTAL":<26} {total_cpu:>18.2f} '
                  f'{total_threads:>8}  ({len(rows)} process(es))')
            if args.out:
                with open(args.out, 'w') as f:
                    json.dump({'mode': args.mode, 'modules': args.modules,
                               'window_s': elapsed, 'rows': rows,
                               'total_cpu_pct': total_cpu,
                               'total_threads': total_threads}, f, indent=1)
            return 0

        # Everything from the first message on is recorded, including the
        # start-up transients: a latched topic is published once and would be
        # lost by a later reset.
        deadline = time.time() + args.settle
        while time.time() < deadline:
            rclpy.spin_once(harness, timeout_sec=0.05)
        harness.start()
        deadline = time.time() + (args.steps + harness.warmup_steps) / args.rate + 3.0
        while not harness.done() and time.time() < deadline:
            rclpy.spin_once(harness, timeout_sec=0.05)
        # drain the last replies
        deadline = time.time() + 2.0
        while time.time() < deadline:
            rclpy.spin_once(harness, timeout_sec=0.05)
    finally:
        for p in procs:
            p.send_signal(signal.SIGTERM)
        for p in procs:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()
        codes = [p.returncode for p in procs]
        harness.destroy_node()
        rclpy.shutdown()

    out = {'mode': args.mode, 'exit_codes': codes,
           'startup': harness.startup or {},
           'records': harness.records}
    with open(args.out, 'w') as f:
        json.dump(out, f, indent=1, sort_keys=True)
    print(f'{args.mode}: exit codes {codes}')
    for topic in sorted(harness.records):
        print(f'  {topic:<48} {len(harness.records[topic]):>4} msgs')
    return 0


# Values that come out of a wall-clock-driven low-pass filter (the battery
# estimator's sag filter and coulomb integrator, and the copies of them in the
# telemetry JSON) cannot be bit-identical across two runs: their dt is real
# time. Everything else is compared exactly.
# `/robot/telemetry` gets a looser one: it is a 10 Hz snapshot of 10 Hz
# inputs, so the two runs' timers sometimes catch a different input sample —
# one 100 ms step of odometry, or a rate estimate one message behind. That is
# the same sampling-instant difference the rclpy-to-Rust ports recorded.
TOLERANCE = {
    '/battery_state': 1e-3,
    '/aon_battery_state': 1e-3,
    '/robot/telemetry': 2e-2,
}
# The startup section is about latched topics: what a late subscriber sees.
# A periodic topic sampled while the graph is still coming up reflects the
# start order of the processes, which is not a contract.
STARTUP_TOPICS = {t for t, _, qos in OUTPUTS if qos is LATCHED}


def close(a, b, tol):
    """Structural equality with a tolerance on the numbers."""
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(close(a[k], b[k], tol) for k in a)
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(close(x, y, tol) for x, y in zip(a, b))
    if isinstance(a, (int, float)) and isinstance(b, (int, float)) \
            and not isinstance(a, bool) and not isinstance(b, bool):
        return abs(a - b) <= tol * max(1.0, abs(a), abs(b))
    return a == b


def diff(a_path, b_path):
    a = json.load(open(a_path))
    b = json.load(open(b_path))
    bad = 0
    print(f'{"topic":<48} {"split":>7} {"daemon":>7}  verdict')
    for section in ('startup', 'records'):
      if section == 'startup':
          print('-- start-up, before the first input (latched topics) --')
      else:
          print('-- the input script --')
      topics = sorted(set(a.get(section, {})) | set(b.get(section, {})))
      if section == 'startup':
          topics = [t for t in topics if t in STARTUP_TOPICS]
      for topic in topics:
        ra = a.get(section, {}).get(topic, [])
        rb = b.get(section, {}).get(topic, [])
        tol = TOLERANCE.get(topic, 0.0)
        # Counts differ by a message or two because the two runs start their
        # timers at different phases; compare the common prefix of the
        # distinct payloads, which is what "same behaviour" means here.
        ua = dedupe(ra)
        ub = dedupe(rb)
        n = min(len(ua), len(ub))
        if n == 0 and not ua and not ub:
            verdict = 'both empty'
        elif n == 0:
            verdict = 'MISSING'
            bad += 1
        elif all(close(x, y, tol) for x, y in zip(ua[:n], ub[:n])):
            verdict = f'IDENTICAL ({n} distinct)'
            if tol:
                verdict += f' within {tol:g}'
        else:
            verdict = 'DIFFERENT'
            bad += 1
            for i in range(n):
                if not close(ua[i], ub[i], tol):
                    print(f'   first difference at #{i}:')
                    print(f'     split : {json.dumps(ua[i])[:400]}')
                    print(f'     daemon: {json.dumps(ub[i])[:400]}')
                    break
        print(f'{topic:<48} {len(ra):>7} {len(rb):>7}  {verdict}')
    if a['exit_codes'] and b['exit_codes']:
        print(f'exit codes: split {a["exit_codes"]} daemon {b["exit_codes"]}')
    print('RESULT:', 'IDENTICAL' if bad == 0 else f'{bad} topic(s) differ')
    return 0 if bad == 0 else 1


def dedupe(records):
    """Consecutive duplicates collapsed: a latched or periodic topic repeats
    the same payload at whatever rate each run happens to tick at."""
    out = []
    for r in records:
        if not out or out[-1] != r:
            out.append(r)
    return out


if __name__ == '__main__':
    sys.exit(main())
