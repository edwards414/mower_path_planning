#!/usr/bin/env python3
"""Differential harness for Phase C: robot_localization vs mower_rs/mower_localize.

One rclpy process plays the whole upstream side of the localization graph --
``/tf_static`` (the IMU and GPS antenna offsets), ``/odom`` at 25 Hz with
covariance, ``/imu/data`` at 10 Hz and ``/fix`` at 4 Hz, with stale and
out-of-order stamps injected -- records everything the localization stack
publishes, and finally asks ``/toLL``, ``/fromLL`` and ``/datum`` a few fixed
questions.

The same run is done twice, once against ``dual_ekf_navsat.launch.py`` and once
against ``mower_localize``, and the two recordings are compared::

    localize_compare.py --record --seconds 30 --out /tmp/cpp.json
    localize_compare.py --record --seconds 30 --out /tmp/rust.json
    localize_compare.py --diff /tmp/cpp.json /tmp/rust.json

Stamps are generated as ``t0 + k*dt`` from the harness's own start time, and
recorded relative to ``t0``, so the two runs produce the same *relative*
timeline even though they happen at different wall-clock times: the filters
stamp their output with the last measurement's time, so a sample can be matched
to its counterpart exactly. Samples whose stamp comes from a wall-clock tick
instead (robot_localization predicts forward and re-stamps when the measurement
queue has been empty for longer than ``sensor_timeout``) have no counterpart by
construction and are reported separately rather than compared.
"""

import argparse
import json
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from geographic_msgs.msg import GeoPoint, GeoPose
from geometry_msgs.msg import Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from robot_localization.srv import FromLL, SetDatum, ToLL
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from tf2_msgs.msg import TFMessage

NS = 1_000_000_000

# Taipei; UTM zone 51 north, well away from a zone edge.
LAT0 = 25.0330000
LON0 = 121.5654000
ALT0 = 12.5

ODOM_HZ = 25.0
IMU_HZ = 10.0
FIX_HZ = 4.0

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
OUTPUT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
)
STATIC_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
)


def quat_from_yaw(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def stamp(ns):
    msg_sec = ns // NS
    return int(msg_sec), int(ns - msg_sec * NS)


class Trajectory:
    """A deterministic drive: two seconds at rest, a one-second ramp, then a
    gentle arc, integrated once at 1 kHz.

    The rest at the start is not cosmetic. ``navsat_transform`` takes its datum
    from whichever fix happens to be the latest when the odometry, IMU and GPS
    inputs have all been seen, and that race resolves differently in two
    separate runs; while the robot is standing still every candidate fix is the
    same one, so both runs anchor the map frame identically and the comparison
    measures the filters instead of the start-up race.

    Integrated up front, too: an on-demand integration starved the stack of CPU
    and halved its update rate.
    """

    STEP = 0.001
    REST = 2.0
    RAMP = 1.0

    @classmethod
    def twist(cls, t):
        if t < cls.REST:
            return 0.0, 0.0
        tt = t - cls.REST
        scale = min(tt / cls.RAMP, 1.0)
        return scale * (0.5 + 0.1 * math.sin(0.7 * tt)), scale * 0.2 * math.sin(0.3 * tt)

    @classmethod
    def accel(cls, t):
        """Numerical d/dt of vx, plus the centripetal term; gravity is added by
        the caller."""
        h = cls.STEP
        ax = (cls.twist(t + h)[0] - cls.twist(max(t - h, 0.0))[0]) / (2 * h)
        vx, wz = cls.twist(t)
        return ax, vx * wz

    def __init__(self, seconds):
        n = int((seconds + 1.0) / self.STEP) + 2
        self.x = [0.0] * n
        self.y = [0.0] * n
        self.yaw = [0.0] * n
        x = y = yaw = 0.0
        for i in range(1, n):
            vx, wz = self.twist((i - 1) * self.STEP)
            x += vx * math.cos(yaw) * self.STEP
            y += vx * math.sin(yaw) * self.STEP
            yaw += wz * self.STEP
            self.x[i], self.y[i], self.yaw[i] = x, y, yaw

    def pose(self, t):
        i = min(max(int(round(t / self.STEP)), 0), len(self.x) - 1)
        return self.x[i], self.y[i], self.yaw[i]


class Harness(Node):
    def __init__(self, seconds, sensor_offsets=True):
        super().__init__('localize_compare')
        self.seconds = seconds
        self.sensor_offsets = sensor_offsets
        self.traj = Trajectory(seconds + 2.0)
        self.t0_ns = self.get_clock().now().nanoseconds
        self.started = time.monotonic()

        self.odom_pub = self.create_publisher(Odometry, '/odom', SENSOR_QOS)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', SENSOR_QOS)
        self.fix_pub = self.create_publisher(NavSatFix, '/fix', SENSOR_QOS)
        self.static_pub = self.create_publisher(TFMessage, '/tf_static', STATIC_QOS)

        self.records = {
            '/odometry/local': [],
            '/odometry/global': [],
            '/odometry/gps': [],
            '/gps/filtered': [],
            'tf:odom->base_footprint': [],
            'tf:map->odom': [],
            'tf_static:map->utm': [],
        }
        self.first_seen = {}

        self.create_subscription(
            Odometry, '/odometry/local',
            lambda m: self._odom_record('/odometry/local', m), OUTPUT_QOS)
        self.create_subscription(
            Odometry, '/odometry/global',
            lambda m: self._odom_record('/odometry/global', m), OUTPUT_QOS)
        self.create_subscription(
            Odometry, '/odometry/gps',
            lambda m: self._odom_record('/odometry/gps', m), OUTPUT_QOS)
        self.create_subscription(NavSatFix, '/gps/filtered', self._fix_record,
                                 OUTPUT_QOS)
        self.create_subscription(TFMessage, '/tf', self._tf_record, OUTPUT_QOS)
        self.create_subscription(TFMessage, '/tf_static', self._tf_static_record,
                                 STATIC_QOS)

        self.toll = self.create_client(ToLL, '/toLL')
        self.fromll = self.create_client(FromLL, '/fromLL')
        self.datum = self.create_client(SetDatum, '/datum')

        self._publish_static()

    # ---- recording ---------------------------------------------------------
    def _rel(self, header):
        return header.stamp.sec * NS + header.stamp.nanosec - self.t0_ns

    def _note_first(self, key):
        if key not in self.first_seen:
            self.first_seen[key] = round(time.monotonic() - self.started, 3)

    def _odom_record(self, key, m):
        self._note_first(key)
        self.records[key].append({
            'stamp': self._rel(m.header),
            'frame_id': m.header.frame_id,
            'child_frame_id': m.child_frame_id,
            'p': [m.pose.pose.position.x, m.pose.pose.position.y,
                  m.pose.pose.position.z],
            'q': [m.pose.pose.orientation.x, m.pose.pose.orientation.y,
                  m.pose.pose.orientation.z, m.pose.pose.orientation.w],
            'v': [m.twist.twist.linear.x, m.twist.twist.linear.y,
                  m.twist.twist.linear.z],
            'w': [m.twist.twist.angular.x, m.twist.twist.angular.y,
                  m.twist.twist.angular.z],
            'pcov': [float(v) for v in m.pose.covariance],
            'tcov': [float(v) for v in m.twist.covariance],
        })

    def _fix_record(self, m):
        self._note_first('/gps/filtered')
        self.records['/gps/filtered'].append({
            'stamp': self._rel(m.header),
            'frame_id': m.header.frame_id,
            'status': int(m.status.status),
            'p': [m.latitude, m.longitude, m.altitude],
            'pcov': [float(v) for v in m.position_covariance],
        })

    def _tf_entry(self, t):
        return {
            'stamp': self._rel(t.header),
            'p': [t.transform.translation.x, t.transform.translation.y,
                  t.transform.translation.z],
            'q': [t.transform.rotation.x, t.transform.rotation.y,
                  t.transform.rotation.z, t.transform.rotation.w],
        }

    def _tf_record(self, msg):
        for t in msg.transforms:
            key = f'tf:{t.header.frame_id}->{t.child_frame_id}'
            if key in self.records:
                self._note_first(key)
                self.records[key].append(self._tf_entry(t))

    def _tf_static_record(self, msg):
        for t in msg.transforms:
            key = f'tf_static:{t.header.frame_id}->{t.child_frame_id}'
            if key in self.records:
                self._note_first(key)
                # The cartesian transform is stamped with the wall clock, so
                # only its value is comparable.
                entry = self._tf_entry(t)
                entry.pop('stamp')
                self.records[key].append(entry)

    # ---- the upstream side -------------------------------------------------
    def _publish_static(self):
        def make(parent, child, xyz):
            t = TransformStamped()
            t.header.stamp.sec, t.header.stamp.nanosec = stamp(self.t0_ns)
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x, t.transform.translation.y, \
                t.transform.translation.z = xyz
            t.transform.rotation.w = 1.0
            return t

        # A real antenna offset and a level IMU a little behind it. With the
        # offsets at zero the filter's only remaining wall-clock-dependent
        # term disappears (prepareTwist's and prepareAcceleration's lever-arm
        # products use the angular velocity and the tick-differentiated
        # angular acceleration), which is what --no-sensor-offsets isolates.
        gps = (0.12, 0.0, 0.62) if self.sensor_offsets else (0.0, 0.0, 0.0)
        imu = (-0.05, 0.02, 0.18) if self.sensor_offsets else (0.0, 0.0, 0.0)
        self.static_pub.publish(TFMessage(transforms=[
            make('base_footprint', 'gps_link', gps),
            make('base_footprint', 'imu_link', imu),
        ]))

    def _odom_msg(self, t, stamp_ns):
        x, y, yaw = self.traj.pose(t)
        vx, wz = self.traj.twist(t)
        m = Odometry()
        m.header.stamp.sec, m.header.stamp.nanosec = stamp(stamp_ns)
        m.header.frame_id = 'odom'
        m.child_frame_id = 'base_footprint'
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.orientation = quat_from_yaw(yaw)
        pcov = [0.0] * 36
        for i, v in enumerate((0.02, 0.02, 1e6, 1e6, 1e6, 0.05)):
            pcov[i * 6 + i] = v
        m.pose.covariance = pcov
        m.twist.twist.linear.x = vx
        m.twist.twist.angular.z = wz
        tcov = [0.0] * 36
        for i, v in enumerate((0.004, 0.004, 1e6, 1e6, 1e6, 0.01)):
            tcov[i * 6 + i] = v
        m.twist.covariance = tcov
        return m

    def _imu_msg(self, t, stamp_ns):
        _, _, yaw = self.traj.pose(t)
        _, wz = self.traj.twist(t)
        ax, ay = self.traj.accel(t)
        m = Imu()
        m.header.stamp.sec, m.header.stamp.nanosec = stamp(stamp_ns)
        m.header.frame_id = 'imu_link'
        m.orientation = quat_from_yaw(yaw)
        m.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.02]
        m.angular_velocity.z = wz
        m.angular_velocity_covariance = [
            0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.002]
        # A level IMU reports gravity in +z; remove_gravitational_acceleration
        # is on, so this is what the filter must subtract.
        m.linear_acceleration.x = ax
        m.linear_acceleration.y = ay
        m.linear_acceleration.z = 9.80665
        m.linear_acceleration_covariance = [
            0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.05]
        return m

    def _fix_msg(self, t, stamp_ns):
        x, y, _ = self.traj.pose(t)
        m = NavSatFix()
        m.header.stamp.sec, m.header.stamp.nanosec = stamp(stamp_ns)
        m.header.frame_id = 'gps_link'
        m.status.status = NavSatStatus.STATUS_GBAS_FIX
        m.status.service = NavSatStatus.SERVICE_GPS
        m.latitude = LAT0 + y / 111320.0
        m.longitude = LON0 + x / (111320.0 * math.cos(math.radians(LAT0)))
        m.altitude = ALT0
        m.position_covariance = [
            0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.0016]
        m.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        return m

    def play(self):
        """Publish the stream in real time, with the stamps of a perfect
        sensor set plus the injected pathologies."""
        odom_dt, imu_dt, fix_dt = 1 / ODOM_HZ, 1 / IMU_HZ, 1 / FIX_HZ
        next_odom = next_imu = next_fix = 0.0
        injected = {'stale': 0, 'out_of_order': 0, 'duplicate': 0}
        k_odom = 0
        while True:
            elapsed = time.monotonic() - self.started
            if elapsed >= self.seconds:
                break
            if elapsed >= next_odom:
                t = next_odom
                ns = self.t0_ns + int(round(t * NS))
                # 1. a stale stamp every 5 s: 0.6 s in the past, which the
                #    filter must reject outright.
                if k_odom and k_odom % 125 == 0:
                    self.odom_pub.publish(
                        self._odom_msg(t, ns - int(0.6 * NS)))
                    injected['stale'] += 1
                # 2. an out-of-order pair every 3 s: k+1 before k.
                if k_odom and k_odom % 75 == 0:
                    self.odom_pub.publish(
                        self._odom_msg(t + odom_dt, ns + int(odom_dt * NS)))
                    injected['out_of_order'] += 1
                # 3. a duplicate stamp every 7 s.
                if k_odom and k_odom % 175 == 0:
                    self.odom_pub.publish(self._odom_msg(t, ns))
                    injected['duplicate'] += 1
                self.odom_pub.publish(self._odom_msg(t, ns))
                next_odom += odom_dt
                k_odom += 1
            if elapsed >= next_imu:
                ns = self.t0_ns + int(round(next_imu * NS))
                self.imu_pub.publish(self._imu_msg(next_imu, ns))
                next_imu += imu_dt
            if elapsed >= next_fix:
                ns = self.t0_ns + int(round(next_fix * NS))
                self.fix_pub.publish(self._fix_msg(next_fix, ns))
                next_fix += fix_dt
            nxt = min(next_odom, next_imu, next_fix)
            sleep = nxt - (time.monotonic() - self.started)
            if sleep > 0:
                time.sleep(min(sleep, 0.001))
        self.injected = injected
        # Let what is still in flight arrive before the service questions.
        time.sleep(1.0)

    # ---- services ----------------------------------------------------------
    def _call(self, client, request, timeout=3.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.005)
        return future.result() if future.done() else None

    def ask_services(self):
        answers = {'toLL': [], 'fromLL': [], 'datum': None, 'toLL_after_datum': []}
        points = [(0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (-3.5, 7.25, 1.0)]
        for x, y, z in points:
            r = self._call(self.toll, ToLL.Request(map_point=Point(x=x, y=y, z=z)))
            answers['toLL'].append(
                None if r is None
                else [r.ll_point.latitude, r.ll_point.longitude,
                      r.ll_point.altitude])
        for lat, lon, alt in [(LAT0, LON0, ALT0),
                              (LAT0 + 0.0001, LON0 + 0.0001, ALT0),
                              (LAT0 - 0.0002, LON0 + 0.00035, 0.0)]:
            r = self._call(self.fromll, FromLL.Request(
                ll_point=GeoPoint(latitude=lat, longitude=lon, altitude=alt)))
            answers['fromLL'].append(
                None if r is None else [r.map_point.x, r.map_point.y, r.map_point.z])
        # /datum re-anchors the transform; do it last so nothing recorded above
        # is affected, then read the new answer back.
        geo = GeoPose()
        geo.position = GeoPoint(latitude=LAT0 + 0.001, longitude=LON0 - 0.001,
                                altitude=ALT0)
        geo.orientation.w = 1.0
        r = self._call(self.datum, SetDatum.Request(geo_pose=geo))
        answers['datum'] = 'ok' if r is not None else None
        time.sleep(1.0)
        for x, y, z in points:
            r = self._call(self.toll, ToLL.Request(map_point=Point(x=x, y=y, z=z)))
            answers['toLL_after_datum'].append(
                None if r is None
                else [r.ll_point.latitude, r.ll_point.longitude,
                      r.ll_point.altitude])
        return answers


# ---- comparison ------------------------------------------------------------
def _max_abs(a, b):
    return max(abs(x - y) for x, y in zip(a, b)) if a and b else 0.0


def compare(left, right, tolerance):
    """What must be identical, and what can only be reported.

    Identical, and therefore what ``ok`` is about: the set of topics, their
    frames, the published rates, the datum and the Cartesian transform, and the
    ``/toLL`` / ``/fromLL`` answers -- those are pure functions of the datum.

    Reported as numbers: the filter state. Two runs of a wall-clock-driven EKF
    never produce the same trajectory, because ``prepareTwist`` and
    ``prepareAcceleration`` take the lever-arm terms from the *current* filter
    state and from an angular acceleration differentiated on the timer, and
    because a tick that finds an empty queue predicts forward and re-stamps.
    Run the C++ stack against itself to see that floor before reading the
    Rust-against-C++ numbers as a difference between the implementations.
    """
    report = {'topics': {}, 'ok': True}
    for key in sorted(set(left['records']) | set(right['records'])):
        lrec = {r['stamp']: r for r in left['records'].get(key, [])
                if 'stamp' in r}
        rrec = {r['stamp']: r for r in right['records'].get(key, [])
                if 'stamp' in r}
        entry = {
            'count': [len(left['records'].get(key, [])),
                      len(right['records'].get(key, []))],
            'rate_hz': [round(len(left['records'].get(key, [])) /
                              max(left['seconds'], 1e-9), 2),
                        round(len(right['records'].get(key, [])) /
                              max(right['seconds'], 1e-9), 2)],
            'first_message_s': [left['first_seen'].get(key),
                                right['first_seen'].get(key)],
        }
        if not lrec and not rrec:
            # e.g. the unstamped cartesian static transform
            lval = left['records'].get(key, [])
            rval = right['records'].get(key, [])
            if lval and rval:
                entry['max_err'] = max(_max_abs(lval[-1]['p'], rval[-1]['p']),
                                       _max_abs(lval[-1]['q'], rval[-1]['q']))
                entry['matched'] = 1
                if entry['max_err'] > tolerance:
                    report['ok'] = False
            report['topics'][key] = entry
            continue
        shared = sorted(set(lrec) & set(rrec))
        entry['matched'] = len(shared)
        entry['unmatched'] = [len(lrec) - len(shared), len(rrec) - len(shared)]
        worst = 0.0
        worst_where = None
        # A filter carries its state forward, so the honest summary of two
        # wall-clock-driven runs is not only the worst error but when the two
        # first stopped agreeing: until then they are the same filter fed the
        # same measurements, after it one of them has taken a timeout predict
        # or lost a best-effort input that the other kept.
        diverged_at = None
        per_field = {}
        for s in shared:
            a, b = lrec[s], rrec[s]
            sample = 0.0
            for field in ('p', 'q', 'v', 'w', 'pcov', 'tcov'):
                if field in a and field in b:
                    err = _max_abs(a[field], b[field])
                    per_field[field] = max(per_field.get(field, 0.0), err)
                    sample = max(sample, err)
                    if err > worst:
                        worst, worst_where = err, (s, field)
            if diverged_at is None and sample > tolerance:
                diverged_at = round(s / 1e9, 3)
            for field in ('frame_id', 'child_frame_id', 'status'):
                if field in a and a.get(field) != b.get(field):
                    report['ok'] = False
                    entry.setdefault('mismatched_fields', []).append(field)
        entry['max_err'] = worst
        entry['max_err_at'] = worst_where
        entry['max_err_per_field'] = per_field
        entry['agreed_until_s'] = diverged_at
        if not shared:
            report['ok'] = False
        lhz, rhz = entry['rate_hz']
        if min(lhz, rhz) <= 0 or abs(lhz - rhz) / max(lhz, rhz) > 0.15:
            report['ok'] = False
            entry['rate_mismatch'] = True
        report['topics'][key] = entry

    svc = {}
    for name in ('toLL', 'fromLL', 'toLL_after_datum'):
        la = left['services'].get(name) or []
        ra = right['services'].get(name) or []
        worst = 0.0
        for a, b in zip(la, ra):
            if a is None or b is None:
                svc[name] = 'missing answer'
                report['ok'] = False
                break
            worst = max(worst, _max_abs(a, b))
        else:
            svc[name] = {'max_err': worst, 'n': len(la)}
            if len(la) != len(ra) or worst > 1e-6:
                report['ok'] = False
    svc['datum'] = [left['services'].get('datum'), right['services'].get('datum')]
    if svc['datum'][0] != 'ok' or svc['datum'][1] != 'ok':
        report['ok'] = False
    report['services'] = svc
    report['injected'] = [left.get('injected'), right.get('injected')]
    return report


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--record', action='store_true')
    ap.add_argument('--seconds', type=float, default=30.0)
    ap.add_argument('--out')
    ap.add_argument('--diff', nargs=2)
    ap.add_argument('--tolerance', type=float, default=1e-6)
    ap.add_argument('--no-sensor-offsets', action='store_true')
    args = ap.parse_args()

    if args.diff:
        with open(args.diff[0]) as f:
            left = json.load(f)
        with open(args.diff[1]) as f:
            right = json.load(f)
        report = compare(left, right, args.tolerance)
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0 if report['ok'] else 1

    if not args.record or not args.out:
        ap.error('--record --out FILE, or --diff A B')

    rclpy.init()
    node = Harness(args.seconds, sensor_offsets=not args.no_sensor_offsets)
    # One executor thread owns every callback: a single-threaded spin_once in
    # the publishing loop could not keep up with ~140 messages a second and
    # silently lost two thirds of the stack's output.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    # Give the stack time to discover the publishers before the stream starts,
    # and let the latched /tf_static reach it.
    time.sleep(2.0)
    node.started = time.monotonic()
    node.play()
    services = node.ask_services()
    doc = {
        'seconds': args.seconds,
        'records': node.records,
        'first_seen': node.first_seen,
        'services': services,
        'injected': node.injected,
    }
    with open(args.out, 'w') as f:
        json.dump(doc, f)
    counts = {k: len(v) for k, v in node.records.items()}
    print(json.dumps({'counts': counts, 'first_seen': node.first_seen,
                      'injected': node.injected}, indent=2, sort_keys=True))
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
