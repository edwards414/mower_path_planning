#!/usr/bin/env python3
"""Row 2's app joystick, as the app drives it, for a soak (I6, client
container). Development only: the sim through its pairing gate.

    ws_teleop_soak.py --out /tmp/client/teleop.json [--seconds 120] [--hold 5]
                      [--pause 1.5] [--blk-hold 45]

Like the app (mower_lawer_app mission_mock_provider.dart and
manual_control_overlay.dart): subscribe /manual_command_clock; while the
stick is held, every 100 ms publish /app_joy_cmd (TwistStamped) echoing the
latest clock stamp and session id, but only while that clock frame is at
most 0.8 s old (the app's _manualCommandClockTimeout; otherwise it sends
nothing); on release, one zero. Holds of --hold s alternate forward and
backward (0.2 m/s) with --pause s between them, for --seconds. Each command
carries its own angular.z (1e-4 rad/s x its sequence number), so the
observer can tell which ones the manual guard forwarded.

--blk-hold S: then one more hold of S s, announced on stdout as
`BLK_HOLD_START <time>` so the runner can black the link out under it; the
connection is dropped and re-opened (as the app does) when no frame came
for 2 s, and two short holds follow once it is back.

Everything is kept: each send (seq, wall time, echoed stamp, the clock
frame's age at the send, values, connection), each clock frame, the ticks
the app would have skipped, connection events.
"""
import argparse
import asyncio
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import safety  # noqa: E402

import websockets  # noqa: E402

CLOCK_MAX_AGE = 0.8
SILENCE_S = 2.0


class App:
    def __init__(self, url):
        self.url = url
        self.ws = None
        self.conn = 0
        self.clock = None          # (sec, nanosec, session, received)
        self.last_rx = 0.0
        self.sends, self.clocks, self.events, self.skipped = [], [], [], []
        self.seq = 0

    def ev(self, kind, **kw):
        self.events.append(dict(t=round(time.time(), 4), e=kind, **kw))

    async def run_conn(self):
        """Keep one connection open, reconnecting like the app."""
        while True:
            try:
                ws = await websockets.connect(self.url, additional_headers=safety.headers('outdoor-teleop'),
                                              max_size=None, open_timeout=4, close_timeout=1,
                                              ping_interval=None)
            except Exception as e:  # noqa: BLE001
                self.ev('connect_fail', why=type(e).__name__)
                await asyncio.sleep(0.5)
                continue
            self.conn += 1
            self.ws = ws
            self.last_rx = time.time()
            self.ev('connected', conn=self.conn)
            await ws.send(json.dumps({'op': 'subscribe', 'topic': '/manual_command_clock', 'type': 'std_msgs/msg/Header',
                                      'qos': {'durability': 'volatile', 'reliability': 'reliable'}}))
            await ws.send(json.dumps({'op': 'advertise', 'topic': '/app_joy_cmd', 'type': 'geometry_msgs/msg/TwistStamped'}))
            watchdog = asyncio.create_task(self.silence(ws))
            try:
                async for raw in ws:
                    now = time.time()
                    self.last_rx = now
                    m = json.loads(raw)
                    if m.get('op') == 'publish' and m.get('topic') == '/manual_command_clock':
                        st = m['msg']['stamp']
                        self.clock = (st['sec'], st['nanosec'], m['msg']['frame_id'], now)
                        self.clocks.append([round(now, 4), st['sec'] * 1_000_000_000 + st['nanosec']])
            except Exception as e:  # noqa: BLE001
                self.ev('conn_error', why=type(e).__name__)
            watchdog.cancel()
            self.ws = None
            self.clock = None
            self.ev('disconnected', conn=self.conn)
            try:
                ws.transport.abort()
            except Exception:  # noqa: BLE001
                pass
            await asyncio.sleep(0.2)

    async def silence(self, ws):
        while True:
            await asyncio.sleep(0.25)
            if time.time() - self.last_rx > SILENCE_S:
                self.ev('silence_drop', conn=self.conn)
                ws.transport.abort()
                return

    async def publish(self, lx, az, kind):
        now = time.time()
        c = self.clock
        ws = self.ws
        if ws is None or c is None or (kind == 'move' and now - c[3] > CLOCK_MAX_AGE):
            self.skipped.append([round(now, 4), kind, 'no_conn' if ws is None else 'no_clock' if c is None else 'clock_old'])
            return
        msg = {'header': {'stamp': {'sec': c[0], 'nanosec': c[1]}, 'frame_id': c[2]},
               'twist': {'linear': {'x': lx, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': az}}}
        try:
            await ws.send(json.dumps({'op': 'publish', 'topic': '/app_joy_cmd', 'msg': msg}, separators=(',', ':')))
        except Exception as e:  # noqa: BLE001
            self.skipped.append([round(now, 4), kind, 'send_' + type(e).__name__])
            return
        self.sends.append([self.seq if kind == 'move' else 0, round(now, 4), c[0] * 1_000_000_000 + c[1],
                           round(now - c[3], 4), lx, az, self.conn])

    async def hold(self, seconds, direction, tag):
        self.ev('hold_start', tag=tag, dir=direction)
        t0 = time.monotonic()
        k = 0
        while time.monotonic() - t0 < seconds:
            self.seq += 1
            await self.publish(0.2 * direction, round(1e-4 * self.seq, 4), 'move')
            k += 1
            await asyncio.sleep(max(0.0, t0 + 0.1 * k - time.monotonic()))
        await self.publish(0.0, 0.0, 'release')
        self.ev('release', tag=tag)


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--url', default=os.environ.get('MOWER_WS_URL', 'ws://sim:9090'))
    ap.add_argument('--out', required=True)
    ap.add_argument('--seconds', type=float, default=120.0)
    ap.add_argument('--hold', type=float, default=5.0)
    ap.add_argument('--pause', type=float, default=1.5)
    ap.add_argument('--blk-hold', type=float, default=0.0)
    args = ap.parse_args()
    safety.check(args.url)
    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
    app = App(args.url)
    conn = asyncio.create_task(app.run_conn())
    t_wait = time.time()
    while app.clock is None and time.time() - t_wait < 30:
        await asyncio.sleep(0.05)
    app.ev('ready', clock=app.clock is not None)
    start = time.monotonic()
    direction = 1
    n = 0
    while time.monotonic() - start < args.seconds:
        n += 1
        await app.hold(min(args.hold, max(0.5, args.seconds - (time.monotonic() - start))), direction, f'soak{n}')
        direction = -direction
        await asyncio.sleep(args.pause)
    if args.blk_hold > 0:
        await asyncio.sleep(1.0)
        print(f'BLK_HOLD_START {time.time():.4f}', flush=True)
        await app.hold(args.blk_hold, direction, 'blk')
        direction = -direction
        t_wait = time.time()
        while app.clock is None and time.time() - t_wait < 20:
            await asyncio.sleep(0.05)
        app.ev('back', clock=app.clock is not None, after_s=round(time.time() - t_wait, 2))
        for i in range(2):
            await asyncio.sleep(1.0)
            await app.hold(3.0, direction, f'after{i + 1}')
            direction = -direction
    await asyncio.sleep(1.5)
    conn.cancel()
    doc = {'args': vars(args), 'sends': app.sends, 'clocks': app.clocks, 'events': app.events,
           'skipped': app.skipped, 'connections': app.conn}
    with open(args.out, 'w') as f:
        json.dump(doc, f, separators=(',', ':'))
    moves = sum(1 for s in app.sends if s[0])
    print(f'teleop soak: {moves} moves sent, {len(app.skipped)} ticks skipped, {app.conn} connections, '
          f'{len(app.clocks)} clock frames', flush=True)


if __name__ == '__main__':
    asyncio.run(main())
