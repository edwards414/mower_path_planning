#!/usr/bin/env python3
"""Rows 7/8: one coverage mission through ws_bridge, as the app drives it,
with resume after any stop (I6, client container). Development only.

    ws_mission.py --out /tmp/client/mission.json [--zone 1] [--max-s 900]
                  [--no-resume]

/create_free_space, /create_risk_map, /create_chennal_map,
/generate_coverage_path, then /zone_exec_path {zone}. Every second:
/coverage_progress_status and /check_nav_status. When the execution ends
without succeeding and the checkpoint says it can continue, wait until
/check_nav_status is ready and call /resume_coverage {zone_id: -1} (every
5 s until one is accepted, at most 30). Ends on SUCCEEDED, on --max-s, or
when nothing can be resumed. Every call, answer and poll is kept with its
wall time.
"""
import argparse
import asyncio
import itertools
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import safety  # noqa: E402

import websockets  # noqa: E402

SUCCEEDED, FAILED, CANCELED = 5, 6, 4


class Bridge:
    def __init__(self, url):
        self.url = url
        self.ids = itertools.count(1)
        self.waiting = {}
        self.ws = None
        self.log = []

    async def connect(self):
        while True:
            try:
                self.ws = await websockets.connect(self.url, additional_headers=safety.headers('outdoor-mission'),
                                                   max_size=None, open_timeout=5, ping_interval=None)
                self.ev('connected')
                asyncio.create_task(self.reader(self.ws))
                return
            except Exception as e:  # noqa: BLE001
                self.ev('connect_fail', why=type(e).__name__)
                await asyncio.sleep(1.0)

    async def reader(self, ws):
        try:
            async for raw in ws:
                m = json.loads(raw)
                if m.get('op') == 'service_response' and m.get('id') in self.waiting:
                    self.waiting.pop(m['id']).set_result(m)
        except Exception as e:  # noqa: BLE001
            self.ev('conn_error', why=type(e).__name__)
        if self.ws is ws:
            self.ws = None
            for f in self.waiting.values():
                if not f.done():
                    f.set_result(None)
            self.waiting.clear()
            await self.connect()

    def ev(self, kind, **kw):
        self.log.append(dict(t=round(time.time(), 3), e=kind, **kw))

    async def call(self, service, args=None, timeout=30.0, record=True):
        while self.ws is None:
            await asyncio.sleep(0.2)
        i = f'c{next(self.ids)}'
        fut = asyncio.get_running_loop().create_future()
        self.waiting[i] = fut
        t = time.time()
        try:
            await self.ws.send(json.dumps({'op': 'call_service', 'service': service, 'id': i, 'args': args or {}}))
            m = await asyncio.wait_for(fut, timeout)
        except Exception as e:  # noqa: BLE001
            self.waiting.pop(i, None)
            m = None
            if record:
                self.ev('call', service=service, dt=round(time.time() - t, 3), ans=f'no answer ({type(e).__name__})')
            return None
        if record:
            v = (m or {}).get('values') or {}
            self.ev('call', service=service, dt=round(time.time() - t, 3), ok=(m or {}).get('result'),
                    ans={k: v.get(k) for k in ('success', 'message') if k in v})
        return m


def progress_of(m):
    v = (m or {}).get('values') or {}
    p, c = v.get('progress') or {}, v.get('checkpoint') or {}
    return {'status': p.get('status'), 'text': p.get('status_text'), 'mission': p.get('mission_id'),
            'seg': [p.get('current_segment_index'), p.get('completed_segments'), p.get('total_segments')],
            'overall': p.get('overall_progress'), 'ckpt': c.get('checkpoint_available'),
            'ckpt_seg': [c.get('completed_segments'), c.get('total_segments')]}


def nav_of(m):
    v = (m or {}).get('values') or {}
    msg = v.get('message')
    try:
        msg = json.loads(msg)
    except (TypeError, ValueError):
        pass
    if isinstance(msg, dict):
        return {'success': v.get('success'), **{k: msg.get(k) for k in ('state', 'ready', 'block_reason') if k in msg}}
    return {'success': v.get('success'), 'message': str(msg)[:200]}


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--url', default=os.environ.get('MOWER_WS_URL', 'ws://sim:9090'))
    ap.add_argument('--out', required=True)
    ap.add_argument('--zone', type=int, default=1)
    ap.add_argument('--max-s', type=float, default=900.0)
    ap.add_argument('--no-resume', action='store_true')
    args = ap.parse_args()
    safety.check(args.url)
    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
    b = Bridge(args.url)
    await b.connect()
    for s in ('/create_free_space', '/create_risk_map', '/create_chennal_map'):
        await b.call(s, timeout=60)
    await b.call('/generate_coverage_path', timeout=120)
    t_start = time.time()
    b.ev('start', zone=args.zone)
    await b.call('/zone_exec_path', {'zone_id': args.zone}, timeout=60)
    polls = []
    resumes = 0
    last_try = 0.0
    done = None
    seen_running = False
    while time.time() - t_start < args.max_s:
        pm = await b.call('/coverage_progress_status', timeout=5, record=False)
        nm = await b.call('/check_nav_status', timeout=5, record=False)
        p, n = progress_of(pm), nav_of(nm)
        polls.append([round(time.time(), 2), p, n])
        if p['status'] in (1, 2):
            seen_running = True
        if p['status'] == SUCCEEDED and seen_running:
            done = 'succeeded'
            break
        stopped = seen_running and p['status'] in (FAILED, CANCELED, 0)
        if stopped and not args.no_resume:
            if not p['ckpt']:
                if time.time() - last_try > 30:
                    done = 'stopped, no checkpoint'
                    break
            elif n.get('ready', n.get('success')) and time.time() - last_try >= 5:
                if resumes >= 30:
                    done = 'resume limit'
                    break
                last_try = time.time()
                resumes += 1
                m = await b.call('/resume_coverage', {'zone_id': -1}, timeout=30)
                ok = ((m or {}).get('values') or {}).get('success')
                b.ev('resume', n=resumes, ok=ok, at=p['ckpt_seg'])
                if ok:
                    seen_running = False
                    t_wait = time.time()
                    while time.time() - t_wait < 20:
                        pm = await b.call('/coverage_progress_status', timeout=5, record=False)
                        if progress_of(pm)['status'] in (1, 2, SUCCEEDED):
                            seen_running = True
                            break
                        await asyncio.sleep(0.5)
        elif stopped and args.no_resume:
            done = 'stopped'
            break
        await asyncio.sleep(1.0)
    doc = {'args': vars(args), 't_start': t_start, 'end': done or 'timeout', 'duration_s': round(time.time() - t_start, 1),
           'resumes': resumes, 'events': b.log, 'polls': polls}
    with open(args.out, 'w') as f:
        json.dump(doc, f, separators=(',', ':'))
    print(f'mission: {doc["end"]} after {doc["duration_s"]} s, {resumes} resumes', flush=True)


if __name__ == '__main__':
    asyncio.run(main())
