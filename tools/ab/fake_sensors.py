#!/usr/bin/env python3
"""Fake WIT IMU and fake u-blox ZED-F9P on ptys, for the image A/B (tools/ab).

mower_rsd's `imu` and `gps` modules open /dev/imu_usb and /dev/gps_rtk; without
hardware they fail and restart every 2 s. This gives both a deterministic,
stationary device so the production run carries /imu/data, /fix and
/gps/status like a robot standing still outdoors with an RTK fix.

  IMU: WIT frames 0x51 (accel), 0x52 (gyro), 0x53 (angle), 0x54 (mag) in a
       cycle, 10 cycles/s (the driver publishes one /imu/data per 0x53).
  GPS: ACK-ACKs every UBX-CFG frame it receives (class 0x06), then per 250 ms
       epoch NAV-PVT, NAV-HPPOSLLH, NAV-RELPOSNED (v1), NAV-EOE with one iTOW,
       and NAV-SAT once a second. RTK fixed, 17 SVs, constant position.

Layouts follow src/mower_rs/crates/mower_ubx/src/lib.rs and mower_imu/src/wit.rs.
Nothing here touches the real devices; it only exists for this test. Stdlib
only: it runs inside the image under test, which need not have ROS.
"""

import argparse
import os
import select
import struct
import time
import tty

LAT = 25.0330000
LON = 121.5654000
HEIGHT_MM = 45_123
HMSL_MM = 30_000
YAW_DEG = 30.0


def make_pty(link):
    master, slave = os.openpty()
    tty.setraw(master)
    tty.setraw(slave)
    name = os.ttyname(slave)
    try:
        os.unlink(link)
    except FileNotFoundError:
        pass
    os.symlink(name, link)
    # keep the slave open so the master never sees EIO between driver restarts
    return master, slave


# ---- WIT IMU -----------------------------------------------------------------
def wit_frame(kind, regs):
    f = bytes([0x55, kind]) + b''.join(struct.pack('<h', r) for r in regs)
    return f + bytes([sum(f) & 0xFF])


def imu_cycle():
    g = 9.8
    accel = [0, 0, int(round(g / (16.0 * 9.8) * 32768))]  # 1 g on z
    gyro = [0, 0, 0]
    angle = [0, 0, int(round(YAW_DEG / 180.0 * 32768))]
    mag = [100, -200, 300]
    return (wit_frame(0x51, accel + [0]) + wit_frame(0x52, gyro + [0])
            + wit_frame(0x53, angle + [0]) + wit_frame(0x54, mag + [0]))


# ---- UBX ---------------------------------------------------------------------
def ubx(cls, mid, payload):
    body = bytes([cls, mid]) + struct.pack('<H', len(payload)) + payload
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return b'\xb5\x62' + body + bytes([a, b])


class UbxIn:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(b'\xb5\x62')
            if i < 0:
                self.buf.clear()
                return out
            del self.buf[:i]
            if len(self.buf) < 6:
                return out
            n = struct.unpack_from('<H', self.buf, 4)[0]
            if len(self.buf) < 8 + n:
                return out
            out.append((self.buf[2], self.buf[3]))
            del self.buf[:8 + n]


def pvt(itow, t):
    p = bytearray(92)
    struct.pack_into('<I', p, 0, itow)
    struct.pack_into('<H', p, 4, t.tm_year)
    p[6], p[7], p[8], p[9], p[10] = t.tm_mon, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec
    p[11] = 0x07
    p[20] = 3                      # 3D
    p[21] = 0x01 | 0x02 | 0x80     # gnssFixOK, diffSoln, carrier fixed
    p[23] = 17
    struct.pack_into('<i', p, 24, int(round(LON * 1e7)))
    struct.pack_into('<i', p, 28, int(round(LAT * 1e7)))
    struct.pack_into('<i', p, 32, HEIGHT_MM)
    struct.pack_into('<i', p, 36, HMSL_MM)
    struct.pack_into('<I', p, 40, 14)
    struct.pack_into('<I', p, 44, 23)
    struct.pack_into('<i', p, 60, 0)
    struct.pack_into('<H', p, 76, 150)
    return bytes(p)


def hpposllh(itow):
    p = bytearray(36)
    struct.pack_into('<I', p, 4, itow)
    struct.pack_into('<i', p, 8, int(round(LON * 1e7)))
    struct.pack_into('<i', p, 12, int(round(LAT * 1e7)))
    struct.pack_into('<i', p, 16, HEIGHT_MM)
    struct.pack_into('<i', p, 20, HMSL_MM)
    struct.pack_into('<I', p, 28, 141)
    struct.pack_into('<I', p, 32, 230)
    return bytes(p)


def relposned(itow):
    p = bytearray(64)
    p[0] = 1
    struct.pack_into('<I', p, 4, itow)
    struct.pack_into('<i', p, 8, 1234)    # N cm
    struct.pack_into('<i', p, 12, -567)   # E cm
    struct.pack_into('<i', p, 16, 89)     # D cm
    struct.pack_into('<i', p, 20, 1360)
    struct.pack_into('<I', p, 36, 140)
    struct.pack_into('<I', p, 40, 150)
    struct.pack_into('<I', p, 44, 300)
    struct.pack_into('<I', p, 60, 0x001 | 0x002 | 0x004 | 0x200 | (2 << 3))
    return bytes(p)


def navsat(itow):
    sats = [(0, 3, 42, 55, 120), (0, 7, 38, 30, 250), (2, 11, 40, 70, 10),
            (3, 21, 36, 25, 300), (6, 9, 33, 15, 80)]
    p = bytearray(struct.pack('<IBBBB', itow, 1, len(sats), 0, 0))
    for gnss, sv, cno, elev, azim in sats:
        p += struct.pack('<BBBbhhI', gnss, sv, cno, elev, azim, -12, 0x08 | 0x40 | 7)
    return bytes(p)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--imu-link', default='/dev/imu_usb')
    ap.add_argument('--gps-link', default='/dev/gps_rtk')
    ap.add_argument('--seconds', type=float, default=600.0)
    ap.add_argument('--mute-file', default='',
                    help='while this file exists both devices send nothing and '
                         'drop what they receive (a USB hub reset, as the robot sees it)')
    args = ap.parse_args()

    imu_m, _imu_s = make_pty(args.imu_link)
    gps_m, _gps_s = make_pty(args.gps_link)
    for fd in (imu_m, gps_m):
        os.set_blocking(fd, False)
    ubx_in = UbxIn()
    cycle = imu_cycle()
    t0 = time.monotonic()
    next_imu = t0
    next_epoch = t0
    epoch = 0
    acks = 0
    print(f'fake_sensors: IMU on {args.imu_link}, GPS on {args.gps_link}', flush=True)
    while time.monotonic() - t0 < args.seconds:
        now = time.monotonic()
        timeout = max(0.0, min(next_imu, next_epoch) - now)
        r, _, _ = select.select([imu_m, gps_m], [], [], timeout)
        for fd in r:
            try:
                data = os.read(fd, 4096)
            except (BlockingIOError, OSError):
                continue
            if args.mute_file and os.path.exists(args.mute_file):
                continue
            if fd == gps_m:
                for cls, mid in ubx_in.feed(data):
                    if cls == 0x06:
                        try:
                            os.write(gps_m, ubx(0x05, 0x01, bytes([cls, mid])))
                            acks += 1
                        except (BlockingIOError, OSError):
                            pass
            # bytes written to the IMU (none expected) are dropped
        now = time.monotonic()
        muted = bool(args.mute_file) and os.path.exists(args.mute_file)
        if now >= next_imu:
            try:
                if not muted:
                    os.write(imu_m, cycle)
            except (BlockingIOError, OSError):
                pass
            next_imu += 0.1
            if next_imu < now:
                next_imu = now + 0.1
        if now >= next_epoch:
            itow = (int(time.time() * 1000) % 604_800_000) // 250 * 250
            t = time.gmtime()
            frames = (ubx(0x01, 0x07, pvt(itow, t)) + ubx(0x01, 0x14, hpposllh(itow))
                      + ubx(0x01, 0x3C, relposned(itow)))
            if epoch % 4 == 0:
                frames += ubx(0x01, 0x35, navsat(itow))
            frames += ubx(0x01, 0x61, struct.pack('<I', itow))
            try:
                if not muted:
                    os.write(gps_m, frames)
            except (BlockingIOError, OSError):
                pass
            epoch += 1
            next_epoch += 0.25
            if next_epoch < now:
                next_epoch = now + 0.25
    print(f'fake_sensors: done, {epoch} GPS epochs, {acks} CFG acks', flush=True)


if __name__ == '__main__':
    main()
