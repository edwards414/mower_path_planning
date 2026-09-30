"""Read-only parameter feed for the desktop dashboard (mower_sudio_app).

Publishes ``/robot/telemetry`` (std_msgs/String, JSON, reliable/volatile, 10 Hz)
with everything the dashboard shows, so it needs a single rosbridge
subscription and none of the raw sensor topics have to be exposed::

    {
      "time": 1789621163.2, "robot_id": "lubancat", "seq": 1234,
      "gps": {"valid": true, "age_s": 0.1, "status": 2, "status_text": "RTK FIXED",
              "service": 1, "lat": 25.0, "lon": 121.5, "alt": 12.3,
              "h_acc_m": 0.014, "v_acc_m": 0.02, "frame_id": "gps",
              "filtered": {"lat": ..., "lon": ..., "alt": ...} | null,
              "pvt": {"num_sv": 21, "fix_type": 3, "carr_soln": 2, "h_acc_m": 0.014,
                      "p_dop": 1.2, "g_speed_mps": 0.3, "head_deg": 91.0} | null},
      "imu": {"valid": true, "age_s": 0.01, "rate_hz": 100.0, "frame_id": "imu_link",
              "roll_deg": 0.1, "pitch_deg": -0.4, "yaw_deg": 91.2,
              "gyro_dps": [x, y, z], "accel_mps2": [x, y, z]},
      "odom": {"valid": true, "age_s": 0.02, "x": .., "y": .., "yaw_deg": ..,
               "vx": .., "wz": ..},
      "base": {"valid": true, "age_s": 0.05, ...the /mower_base/telemetry JSON...},
      "battery": {"valid": true, "age_s": 0.4, "present": true, "pct": 0.63,
                  "voltage_v": 23.4, "current_a": null, "status": "discharging",
                  "aon": {"present": true, "pct": 0.8, "voltage_v": 3.9} | null},
      "link": {"valid": true, "age_s": 2.1, ...link_status.json from the host...},
      "host": {"load1": 0.8, "load5": 0.7, "load15": 0.6, "mem_used_pct": 41.2,
               "cpu_temp_c": 52.0, "uptime_s": <this node>, "boot_uptime_s": 1260,
               "sample_s": 2.0,
               "cpu": {"pct": 38.2, "user": 25.1, "system": 9.0, "iowait": 0.4,
                       "irq": 3.7, "steal": 0.0, "cores": [40.1, 35.0, 38.2, 39.4],
                       "freq": [{"cpus": [0, 1, 2, 3], "cur_mhz": 1416,
                                 "max_mhz": 1992, "limit_mhz": 1416}],
                       "temps_c": {"soc-thermal": 52.0, "gpu-thermal": 50.1}},
               "mem": {"total_mb": 3895.0, "used_mb": 611.0, "cache_mb": 950.0,
                       "free_mb": 2387.0, "avail_mb": 3284.0,
                       "swap_total_mb": 0.0, "swap_used_mb": 0.0},
               "disks": [{"role": "state"|"bags"|"root", "path": "/home/mower/.mower",
                          "dev": "mmcblk0p3", "fstype": "ext4", "total_gb": 30.7,
                          "used_gb": 7.5, "avail_gb": 21.9, "used_pct": 25.6}],
               "io": [{"dev": "mmcblk0", "model": "TWSC", "size_gb": 31.3,
                       "read_kbs": 0.0, "write_kbs": 12.3, "util_pct": 0.9,
                       "life_time": [1, 1], "pre_eol": 1}],
               "tasks": {"procs": 182, "threads": 412, "running": 3},
               "procs": [{"pid": 812, "name": "ros2_control_node", "node": null,
                          "state": "S", "cpu": 27.2, "mem": 1.3, "rss_mb": 50.1,
                          "threads": 12}, ...10 busiest]},
      "info": {...the latest /robot/info JSON... } | null
    }

Sources: ``/fix`` (+ ``/gps/filtered``, optional u-blox ``navpvt``),
``/imu/data``, ``/odom``, ``/mower_base/telemetry`` (mower_hardware),
``/battery_state`` + ``/aon_battery_state`` (battery_state_node),
``/robot/info`` (robot_info_node), ``<state_dir>/link_status.json``
(deploy/host/mower-link-status.py) and /proc + /sys for the host block
(``host_stats.py``, refreshed every 2 s; CPU %, per-core %, per-process % and
disk throughput are deltas, so ``None`` in the first sample; process CPU is
% of one core like ``top``; ``mem_used_pct`` is (total - available) / total
while ``mem.used_mb`` follows ``free``).

Every block carries ``valid`` (seen at least once) and ``age_s`` (time since
the last sample) so the dashboard can grey out stale data itself.
"""

import json
import math
import os
import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu, NavSatFix, NavSatStatus
from std_msgs.msg import String

from mower_mission import host_request
from mower_mission.host_stats import HostSampler

try:  # only present in the gps image / a dev container with the driver
    from ublox_msgs.msg import NavPVT
except ImportError:  # pragma: no cover - depends on the image
    NavPVT = None

BATTERY_STATUS_TEXT = {
    BatteryState.POWER_SUPPLY_STATUS_CHARGING: 'charging',
    BatteryState.POWER_SUPPLY_STATUS_DISCHARGING: 'discharging',
    BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING: 'not_charging',
    BatteryState.POWER_SUPPLY_STATUS_FULL: 'full',
}

FIX_STATUS_TEXT = {
    NavSatStatus.STATUS_NO_FIX: 'NO FIX',
    NavSatStatus.STATUS_FIX: '3D FIX',
    NavSatStatus.STATUS_SBAS_FIX: 'DGPS',
    NavSatStatus.STATUS_GBAS_FIX: 'RTK FIXED',
}
# u-blox NAV-PVT flags bits 6-7
CARR_SOLN_TEXT = {0: None, 1: 'RTK FLOAT', 2: 'RTK FIXED'}


def quat_to_euler_deg(x, y, z, w):
    """Return (roll, pitch, yaw) in degrees, ZYX convention (REP-103)."""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


class _Sample:
    """Latest message of one source plus when it arrived (monotonic)."""

    def __init__(self):
        self.msg = None
        self.t = None
        self.count = 0
        self._rate_window = []

    def set(self, msg, now):
        self.msg = msg
        self.t = now
        self.count += 1
        self._rate_window.append(now)
        cutoff = now - 2.0
        while self._rate_window and self._rate_window[0] < cutoff:
            self._rate_window.pop(0)

    def age(self, now):
        return None if self.t is None else round(now - self.t, 3)

    def rate_hz(self, now):
        if len(self._rate_window) < 2:
            return 0.0
        span = self._rate_window[-1] - self._rate_window[0]
        return round((len(self._rate_window) - 1) / span, 1) if span > 0 else 0.0


class TelemetryNode(Node):
    """Fold sensor / base / host state into one JSON topic."""

    def __init__(self):
        super().__init__('telemetry')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('gps_fix_topic', '/fix')
        self.declare_parameter('gps_filtered_topic', '/gps/filtered')
        self.declare_parameter('navpvt_topic', '/ublox_gps_node/navpvt')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('base_telemetry_topic', '/mower_base/telemetry')
        self.declare_parameter('battery_topic', '/battery_state')
        self.declare_parameter('aon_battery_topic', '/aon_battery_state')
        self.declare_parameter('robot_info_topic', '/robot/info')
        self.declare_parameter('state_dir', host_request.state_dir())
        self.declare_parameter(
            'robot_id', os.environ.get('MOWER_ROBOT_ID') or socket.gethostname()
        )

        rate = max(0.5, float(self.get_parameter('publish_rate_hz').value))
        self._state_dir = str(self.get_parameter('state_dir').value)
        self._robot_id = str(self.get_parameter('robot_id').value)
        self._seq = 0
        self._start = time.monotonic()

        self._fix = _Sample()
        self._filtered = _Sample()
        self._pvt = _Sample()
        self._imu = _Sample()
        self._odom = _Sample()
        self._base = _Sample()
        self._battery = _Sample()
        self._aon_battery = _Sample()
        self._info = _Sample()
        self._link = _Sample()
        self._link_mtime = None
        self._host = {}
        self._host_sampler = HostSampler(self._state_dir)

        latched = QoSProfile(depth=1)
        latched.reliability = QoSReliabilityPolicy.RELIABLE
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        best_effort = QoSProfile(depth=1)
        best_effort.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # reliable (not best-effort): rosbridge picks its subscriber QoS from
        # the publishers it has discovered, and a best-effort publisher that
        # shows up late leaves the app on a reliable subscription with no data.
        self._pub = self.create_publisher(String, '/robot/telemetry', 10)

        p = lambda name: str(self.get_parameter(name).value)  # noqa: E731
        self.create_subscription(
            NavSatFix, p('gps_fix_topic'), lambda m: self._store(self._fix, m),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            NavSatFix, p('gps_filtered_topic'), lambda m: self._store(self._filtered, m),
            qos_profile_sensor_data,
        )
        if NavPVT is not None and p('navpvt_topic'):
            self.create_subscription(
                NavPVT, p('navpvt_topic'), lambda m: self._store(self._pvt, m),
                qos_profile_sensor_data,
            )
        self.create_subscription(
            Imu, p('imu_topic'), lambda m: self._store(self._imu, m), qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, p('odom_topic'), lambda m: self._store(self._odom, m), 10
        )
        self.create_subscription(
            String, p('base_telemetry_topic'), self._on_base, best_effort
        )
        self.create_subscription(
            BatteryState, p('battery_topic'), lambda m: self._store(self._battery, m), 10
        )
        self.create_subscription(
            BatteryState, p('aon_battery_topic'), lambda m: self._store(self._aon_battery, m), 10
        )
        self.create_subscription(String, p('robot_info_topic'), self._on_info, latched)

        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'telemetry: /robot/telemetry @ {rate:g} Hz, state_dir={self._state_dir}, '
            f'navpvt={"yes" if NavPVT is not None else "no ublox_msgs"}'
        )

    # ---- inputs --------------------------------------------------------------

    def _store(self, sample, msg):
        sample.set(msg, time.monotonic())

    def _on_base(self, msg: String):
        try:
            self._store(self._base, json.loads(msg.data))
        except ValueError:
            self.get_logger().warning('bad JSON on base telemetry topic', throttle_duration_sec=10)

    def _on_info(self, msg: String):
        try:
            self._store(self._info, json.loads(msg.data))
        except ValueError:
            self.get_logger().warning('bad JSON on /robot/info', throttle_duration_sec=10)

    def _poll_link(self, now):
        """Re-read link_status.json only when the host rewrote it."""
        path = os.path.join(self._state_dir, 'link_status.json')
        try:
            mtime = os.stat(path).st_mtime
        except OSError:
            return
        if mtime == self._link_mtime:
            return
        self._link_mtime = mtime
        try:
            with open(path, encoding='utf-8') as f:
                self._store(self._link, json.load(f))
        except (OSError, ValueError):
            pass

    # ---- output --------------------------------------------------------------

    def _gps_block(self, now):
        fix = self._fix.msg
        block = {'valid': fix is not None, 'age_s': self._fix.age(now),
                 'rate_hz': self._fix.rate_hz(now)}
        if fix is not None:
            cov = list(fix.position_covariance)
            h_acc = math.sqrt(max(cov[0], cov[4], 0.0)) if len(cov) == 9 else None
            v_acc = math.sqrt(max(cov[8], 0.0)) if len(cov) == 9 else None
            block.update({
                'status': int(fix.status.status),
                'status_text': FIX_STATUS_TEXT.get(int(fix.status.status), f'status {fix.status.status}'),
                'service': int(fix.status.service),
                'lat': fix.latitude, 'lon': fix.longitude, 'alt': fix.altitude,
                'h_acc_m': None if h_acc is None else round(h_acc, 4),
                'v_acc_m': None if v_acc is None else round(v_acc, 4),
                'cov_type': int(fix.position_covariance_type),
                'frame_id': fix.header.frame_id,
            })
        filt = self._filtered.msg
        block['filtered'] = None if filt is None else {
            'lat': filt.latitude, 'lon': filt.longitude, 'alt': filt.altitude,
            'age_s': self._filtered.age(now),
        }
        pvt = self._pvt.msg
        if pvt is None:
            block['pvt'] = None
        else:
            carr = (int(pvt.flags) >> 6) & 0x3
            block['pvt'] = {
                'num_sv': int(pvt.num_sv), 'fix_type': int(pvt.fix_type),
                'carr_soln': carr, 'carr_text': CARR_SOLN_TEXT.get(carr),
                'h_acc_m': pvt.h_acc / 1000.0, 'v_acc_m': pvt.v_acc / 1000.0,
                'p_dop': pvt.p_dop / 100.0, 'g_speed_mps': pvt.g_speed / 1000.0,
                'head_deg': pvt.head_mot / 1e5, 'age_s': self._pvt.age(now),
            }
            if carr:  # the carrier solution is more specific than NavSatStatus
                block['status_text'] = CARR_SOLN_TEXT[carr]
        return block

    def _imu_block(self, now):
        imu = self._imu.msg
        block = {'valid': imu is not None, 'age_s': self._imu.age(now),
                 'rate_hz': self._imu.rate_hz(now)}
        if imu is not None:
            q = imu.orientation
            roll, pitch, yaw = quat_to_euler_deg(q.x, q.y, q.z, q.w)
            g, a = imu.angular_velocity, imu.linear_acceleration
            block.update({
                'frame_id': imu.header.frame_id,
                'roll_deg': round(roll, 2), 'pitch_deg': round(pitch, 2), 'yaw_deg': round(yaw, 2),
                'gyro_dps': [round(math.degrees(v), 3) for v in (g.x, g.y, g.z)],
                'accel_mps2': [round(v, 3) for v in (a.x, a.y, a.z)],
                'has_orientation': bool(imu.orientation_covariance[0] >= 0.0),
            })
        return block

    def _odom_block(self, now):
        odom = self._odom.msg
        block = {'valid': odom is not None, 'age_s': self._odom.age(now)}
        if odom is not None:
            p = odom.pose.pose.position
            q = odom.pose.pose.orientation
            _, _, yaw = quat_to_euler_deg(q.x, q.y, q.z, q.w)
            block.update({
                'x': round(p.x, 3), 'y': round(p.y, 3), 'yaw_deg': round(yaw, 2),
                'vx': round(odom.twist.twist.linear.x, 3),
                'wz': round(odom.twist.twist.angular.z, 3),
                'frame_id': odom.header.frame_id,
            })
        return block

    def _battery_block(self, now):
        """``/battery_state`` (main pack) with the AON cell nested under ``aon``."""
        block = {'valid': self._battery.msg is not None, 'age_s': self._battery.age(now)}
        block.update(battery_fields(self._battery.msg))
        aon = self._aon_battery.msg
        block['aon'] = None if aon is None else battery_fields(aon)
        return block

    def _json_block(self, sample, now):
        block = {'valid': sample.msg is not None, 'age_s': sample.age(now)}
        if sample.msg is not None:
            block.update(sample.msg)
        return block

    def _tick(self):
        now = time.monotonic()
        self._poll_link(now)
        block = self._host_sampler.poll(now)
        if block is not None:
            self._host = block
        self._seq += 1
        host = dict(self._host)
        host['uptime_s'] = round(now - self._start, 1)
        doc = {
            'time': round(time.time(), 3),
            'robot_id': self._robot_id,
            'seq': self._seq,
            'gps': self._gps_block(now),
            'imu': self._imu_block(now),
            'odom': self._odom_block(now),
            'base': self._json_block(self._base, now),
            'battery': self._battery_block(now),
            'link': self._json_block(self._link, now),
            'host': host,
            'info': self._info.msg,
        }
        msg = String()
        msg.data = json.dumps(_finite(doc), separators=(',', ':'))
        self._pub.publish(msg)


def battery_fields(msg):
    """Pick the BatteryState fields the dashboard shows; NaN -> null via _finite."""
    if msg is None:
        return {'present': False, 'pct': None, 'voltage_v': None, 'current_a': None,
                'status': None}
    return {
        'present': bool(msg.present),
        'pct': round(float(msg.percentage), 3),
        'voltage_v': round(float(msg.voltage), 2),
        'current_a': round(float(msg.current), 2),
        'status': BATTERY_STATUS_TEXT.get(int(msg.power_supply_status), 'unknown'),
    }


def _finite(value):
    """NaN / inf (e.g. NavSatFix altitude without a fix) -> null, recursively.

    Dart's json.decode rejects the bare ``NaN`` Python would otherwise emit.
    """
    if hasattr(value, 'item') and not isinstance(value, (dict, list, tuple)):
        value = value.item()  # numpy scalar (message array elements) -> python
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: _finite(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_finite(v) for v in value]
    return value


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
