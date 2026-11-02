#!/usr/bin/env python3
"""Row 5: record a work zone by driving its perimeter with the app's
joystick through ws_bridge (I6, client container), the way the app does it.
The driver is mac-robot's backend_e2e.py drive_polygon (turn in place at the
corners, pure pursuit along the edges, on /adapter/robot_pose), with the sim
pair. Development only.

    ws_zone_record.py --out /tmp/client/zone.json [--zone 8,6]

The zone is a W (east) x H (north) rectangle with the robot's start (map
0,0) as its south-west corner: /record_zone_start, the perimeter
north-east-south-west, /record_zone_end, /save_zone_list. Prints
`RECORD_START <time>` when recording starts (the runner times faults from
it). Keeps the pose stream (5 Hz) and the answers.
"""
import argparse
import asyncio
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import safety  # noqa: E402

import websockets  # noqa: E402


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Link:
    def __init__(self, ws):
        self.ws = ws
        self.latest = {}
        self.pending = {}
        self.n = 0
        self.poses = []
        self.calls = []

    async def reader(self):
        async for raw in self.ws:
            m = json.loads(raw)
            if m.get('op') == 'publish':
                self.latest[m['topic']] = m['msg']
                if m['topic'] == '/adapter/robot_pose':
                    p = self.pose()
                    if p:
                        self.poses.append([round(time.time(), 3)] + [round(v, 4) for v in p])
            elif m.get('op') == 'service_response':
                fut = self.pending.pop(m.get('id'), None)
                if fut and not fut.done():
                    fut.set_result(m)

    async def call(self, service, args=None, timeout=60.0):
        self.n += 1
        cid = f'zr-{self.n}'
        fut = asyncio.get_running_loop().create_future()
        self.pending[cid] = fut
        t = time.time()
        await self.ws.send(json.dumps({'op': 'call_service', 'id': cid, 'service': service, 'args': args or {}}))
        try:
            r = await asyncio.wait_for(fut, timeout)
            v, ok = r.get('values', {}), r.get('result', False)
        except asyncio.TimeoutError:
            v, ok = {'error': 'timeout'}, False
        self.calls.append([round(t, 3), service, ok, v, round(time.time() - t, 3)])
        print(service, ok, json.dumps(v)[:200], flush=True)
        return v, ok

    def pose(self):
        p = self.latest.get('/adapter/robot_pose', {}).get('pose')
        if not p:
            return None
        q = p['orientation']
        yaw = math.atan2(2 * (q['w'] * q['z'] + q['x'] * q['y']), 1 - 2 * (q['y'] ** 2 + q['z'] ** 2))
        return p['position']['x'], p['position']['y'], yaw

    async def joy(self, v, w):
        clock = self.latest.get('/manual_command_clock')
        if not clock:
            return
        msg = {'header': {'stamp': clock['stamp'], 'frame_id': clock['frame_id']},
               'twist': {'linear': {'x': v, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': w}}}
        await self.ws.send(json.dumps({'op': 'publish', 'topic': '/app_joy_cmd', 'msg': msg}))


async def drive_polygon(link, pts, vmax=0.22, wmax=0.75, limit_s=600):
    t0 = time.monotonic()
    for i in range(1, len(pts)):
        a, b = pts[i - 1], pts[i]
        seg = (b[0] - a[0], b[1] - a[1])
        L = math.hypot(*seg)
        ux, uy = seg[0] / L, seg[1] / L
        heading = math.atan2(uy, ux)
        while time.monotonic() - t0 < limit_s:
            p = link.pose()
            err = wrap(heading - p[2])
            if abs(err) < math.radians(3):
                break
            await link.joy(0.0, max(-wmax, min(wmax, 1.6 * err)) if abs(err) > math.radians(12) else math.copysign(0.25, err))
            await asyncio.sleep(0.1)
        await link.joy(0.0, 0.0)
        await asyncio.sleep(0.4)
        while time.monotonic() - t0 < limit_s:
            p = link.pose()
            s = (p[0] - a[0]) * ux + (p[1] - a[1]) * uy
            if s >= L - 0.05:
                break
            look = min(L, s + 0.8)
            tx, ty = a[0] + ux * look, a[1] + uy * look
            err = wrap(math.atan2(ty - p[1], tx - p[0]) - p[2])
            v = vmax * max(0.35, min(1.0, (L - s) / 0.6))
            await link.joy(v, max(-wmax, min(wmax, 1.8 * err)))
            await asyncio.sleep(0.1)
        for _ in range(5):
            await link.joy(0.0, 0.0)
            await asyncio.sleep(0.1)
        p = link.pose()
        print(f'  corner {i}: at ({p[0]:.2f},{p[1]:.2f}) target ({b[0]:.2f},{b[1]:.2f}) t={time.monotonic() - t0:.0f}s',
              flush=True)


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--url', default=os.environ.get('MOWER_WS_URL', 'ws://sim:9090'))
    ap.add_argument('--out', required=True)
    ap.add_argument('--zone', default='8,6')
    args = ap.parse_args()
    safety.check(args.url)
    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
    zw, zh = (float(v) for v in args.zone.split(','))
    doc = {'args': vars(args)}
    async with websockets.connect(args.url, additional_headers=safety.headers('outdoor-zone'), max_size=None) as ws:
        link = Link(ws)
        rt = asyncio.create_task(link.reader())
        for topic, typ in (('/manual_command_clock', 'std_msgs/msg/Header'),
                           ('/adapter/robot_pose', 'geometry_msgs/msg/PoseStamped')):
            await ws.send(json.dumps({'op': 'subscribe', 'topic': topic, 'type': typ}))
        await ws.send(json.dumps({'op': 'advertise', 'topic': '/app_joy_cmd', 'type': 'geometry_msgs/msg/TwistStamped'}))
        await asyncio.sleep(2.0)
        x0, y0, _ = link.pose()
        doc['start_pose'] = link.pose()
        await link.call('/record_zone_start')
        doc['t_record_start'] = time.time()
        print(f'RECORD_START {doc["t_record_start"]:.4f}', flush=True)
        await drive_polygon(link, [(x0, y0), (x0, y0 + zh), (x0 + zw, y0 + zh), (x0 + zw, y0), (x0, y0)])
        doc['t_record_end'] = time.time()
        await link.call('/record_zone_end')
        await link.call('/save_zone_list')
        await asyncio.sleep(1.0)
        rt.cancel()
        doc['poses'] = link.poses
        doc['calls'] = link.calls
    json.dump(doc, open(args.out, 'w'))
    print('ZONE_DONE', flush=True)


if __name__ == '__main__':
    asyncio.run(main())
