#!/usr/bin/env python3
"""Is navsat_transform's datum rotated? A start-up check before a GPS run.

navsat_transform locks ``map -> utm`` from the map EKF's yaw and the IMU's yaw
at the moment its ``delay`` ends. When ``/odom`` reaches the map EKF before the
IMU does (the usual order with ``rust_base``), the EKF starts at yaw 0 and its
heading is still converging then, so the datum comes out rotated by what is
left: 10-15 degrees in the container, robot_localization and mower_localize
alike. Every GPS position is then rotated about the map origin by that angle
until navsat restarts, and a robot following a straight strip curves off it.

The check asks navsat itself, so it needs no tf and no UTM arithmetic: /toLL
of the map origin, then /fromLL of a point 100 m true north of it. With a good
datum that point lies where ``magnetic_declination_radians + yaw_offset`` (read
from /navsat_transform) put north -- straight along map +y when both are 0, as
in dual_ekf_navsat_params.yaml. It works against either stack and only calls
read-only services::

    datum_check.py                 # exit 0 within --tolerance-deg, 1 outside, 2 no datum
    datum_check.py --fix           # outside the tolerance: re-set the datum, check again

``--fix`` calls /datum with the current origin and a zero heading, which keeps
the map origin where it is and removes the rotation (upstream's manual datum:
the map EKF's pose is taken as zero there, facing the IMU's zero). Both app
adapters lock ``/adapter/map_datum`` (origin and bearing) once, from the first
datum, so after a fix the satellite overlay keeps the old bearing until the
adapter restarts. Run it before loading a site or starting a mission.

On the robot, inside the running container (nothing is copied onto it; add
``--fix`` after ``python3 -`` to re-set)::

    ssh cat@<robot> 'sudo docker exec -i mower-lawan_node-1 bash -c \\
        "source /mower_ws/install/setup.bash && python3 -"' < datum_check.py
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node

from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point
from rcl_interfaces.srv import GetParameters
from robot_localization.srv import FromLL, SetDatum, ToLL

NORTH_M = 100.0
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


class Check(Node):
    def __init__(self):
        super().__init__('datum_check')
        self.to_ll = self.create_client(ToLL, '/toLL')
        self.from_ll = self.create_client(FromLL, '/fromLL')
        self.datum = self.create_client(SetDatum, '/datum')
        self.params = self.create_client(
            GetParameters, '/navsat_transform/get_parameters')

    def call(self, client, request, timeout=3.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'{client.srv_name} is not available')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'{client.srv_name} did not answer')
        return future.result()

    def origin(self):
        """The map origin's (lat, lon, alt), or None before the datum
        (navsat answers (0, 0, 0) then)."""
        ll = self.call(self.to_ll, ToLL.Request(map_point=Point())).ll_point
        values = (ll.latitude, ll.longitude, ll.altitude)
        if not all(math.isfinite(v) for v in values) or (
                ll.latitude == 0.0 and ll.longitude == 0.0):
            return None
        return values

    def expected_deg(self):
        """Where north should point, from navsat's own two offsets."""
        request = GetParameters.Request(
            names=['magnetic_declination_radians', 'yaw_offset'])
        try:
            values = self.call(self.params, request).values
            return math.degrees(sum(v.double_value for v in values))
        except RuntimeError as e:
            print(f'  ({e}; assuming declination and yaw_offset are 0)')
            return 0.0

    def rotation_deg(self, origin):
        """How far the map frame is turned from east-north-up, degrees
        counter-clockwise, as navsat projects a point due north."""
        lat, lon, alt = origin
        # Metres per radian of latitude here (meridional radius of curvature).
        m = WGS84_A * (1.0 - WGS84_E2) / (
            1.0 - WGS84_E2 * math.sin(math.radians(lat)) ** 2) ** 1.5
        north = GeoPoint(
            latitude=lat + math.degrees(NORTH_M / m), longitude=lon, altitude=alt)
        p = self.call(self.from_ll, FromLL.Request(ll_point=north)).map_point
        return 90.0 - math.degrees(math.atan2(p.y, p.x)), math.hypot(p.x, p.y)

    def set_datum(self, origin):
        request = SetDatum.Request()
        request.geo_pose.position.latitude = origin[0]
        request.geo_pose.position.longitude = origin[1]
        request.geo_pose.position.altitude = origin[2]
        request.geo_pose.orientation.w = 1.0
        self.call(self.datum, request)


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def measure(node):
    origin = node.origin()
    if origin is None:
        return None, None
    expected = node.expected_deg()
    rotation, dist = node.rotation_deg(origin)
    err = wrap_deg(rotation - expected)
    print(f'  map origin ({origin[0]:.7f}, {origin[1]:.7f}); a point {NORTH_M:.0f} m '
          f'north projects {dist:.2f} m away')
    print(f'  map frame turned {rotation:+.2f} deg from ENU, expected {expected:+.2f}: '
          f'datum error {err:+.2f} deg')
    return origin, err


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--tolerance-deg', type=float, default=2.0)
    ap.add_argument('--fix', action='store_true',
                    help='outside the tolerance, re-set the datum at the same '
                         'origin with the rotation removed, then check again')
    args = ap.parse_args()

    rclpy.init()
    node = Check()
    try:
        print('datum check:')
        origin, err = measure(node)
        if origin is None:
            print('  NO_DATUM: /toLL answers (0, 0) -- navsat has not locked a datum yet')
            return 2
        if abs(err) <= args.tolerance_deg:
            print(f'  DATUM_OK (within {args.tolerance_deg} deg)')
            return 0
        if not args.fix:
            print(f'  DATUM_ROTATED by {err:+.1f} deg: every GPS position is turned that '
                  'much about the map origin; --fix re-sets it')
            return 1
        print('  re-setting the datum at the same origin, zero heading ...')
        node.set_datum(origin)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            time.sleep(0.5)
            if node.origin() is not None:
                break
        print('after /datum:')
        origin, err = measure(node)
        if origin is None or abs(err) > args.tolerance_deg:
            print('  DATUM_FIX_FAILED')
            return 1
        print('  DATUM_FIXED. The app adapters locked /adapter/map_datum from the old '
              'datum: restart the adapter before relying on the satellite overlay.')
        return 0
    except RuntimeError as e:
        print(f'  ERROR: {e}')
        return 2
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
