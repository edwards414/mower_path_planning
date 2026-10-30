#!/usr/bin/env python3
"""Host-side probes of one side through mower_ws_bridge, the way the app sees
the robot (rosbridge v2 JSON over WebSocket). Stdlib + websockets.

    probe.py wait     ws://127.0.0.1:P91 SECONDS
        until the loopback port answers /rosapi/topics; exit 1 at the deadline
    probe.py services ws://127.0.0.1:P91 OUT.json
        /rosapi/topics (names and types) and the read-only services of
        ws_bridge.yaml, through the loopback port (no gate, mowerctl's path)
    probe.py topics   ws://127.0.0.1:P90 SECONDS OUT.json
        through the pairing gate: subscribe to every topic /rosapi/topics
        lists for SECONDS; per topic the message count, the JSON shape of what
        arrived (every key path with its JSON type, inside JSON strings too),
        the distinct contents and the last one (stamps dropped, floats rounded)

MOWER_PAIR_FILE (robot_id, secret) signs the gate's headers as probe_app.py
does.
"""
import asyncio
import base64
import hashlib
import hmac
import json
import os
import sys
import time

import websockets

# The read-only services of the bridge's policy (ws_bridge.yaml; every other
# service there mutates something). /rosapi/get_time answers the time: only
# its result is kept.
SERVICES = [
    ('/site_op', {'op': 'list', 'name': '', 'new_name': ''}),
    ('/check_nav_status', {}),
    ('/coverage_progress_status', {}),
    ('/rosapi/get_time', {}),
]


def gate_headers():
    with open(os.environ['MOWER_PAIR_FILE']) as f:
        d = json.load(f)
    robot = d['robot_id']
    s = d['secret'].strip().upper()
    secret = base64.b32decode(s + '=' * (-len(s) % 8))
    client, t, nonce = 'mower-ab-probe', int(time.time()), os.urandom(16).hex()
    mac = hmac.new(secret, f'{robot}\n{client}\n{t}\n{nonce}'.encode(), hashlib.sha256).hexdigest()
    return {'X-Mower-Robot': robot, 'X-Mower-Client': client, 'X-Mower-Time': str(t),
            'X-Mower-Nonce': nonce, 'X-Mower-Mac': mac}


def connect(url, gated):
    kw = {'max_size': None, 'open_timeout': 10}
    if gated:
        kw['additional_headers'] = gate_headers()
    return websockets.connect(url, **kw)


def norm(value):
    """A message as the app gets it, minus what changes every sample: header
    stamps dropped, floats rounded to 1e-6, JSON documents in strings parsed."""
    if isinstance(value, dict):
        return {k: norm(v) for k, v in value.items() if k != 'stamp'}
    if isinstance(value, list):
        return [norm(v) for v in value]
    if isinstance(value, float):
        return round(value, 6)
    if isinstance(value, str) and value[:1] in '{[':
        try:
            return {'<json>': norm(json.loads(value))}
        except ValueError:
            pass
    return value


def shape(value, path, out):
    """Every key path of `value` with its JSON type ('a.b[].c: float')."""
    if isinstance(value, dict):
        if not value:
            out.add(f'{path}: {{}}')
        for k, v in value.items():
            shape(v, f'{path}.{k}' if path else k, out)
    elif isinstance(value, list):
        if not value:
            out.add(f'{path}[]: empty')
        for v in value:
            shape(v, path + '[]', out)
    elif isinstance(value, str) and value[:1] in '{[':
        try:
            shape(json.loads(value), path + '<json>', out)
        except ValueError:
            out.add(f'{path}: str')
    else:
        out.add(f'{path}: ' + {bool: 'bool', int: 'int', float: 'float', str: 'str',
                               type(None): 'null'}.get(type(value), type(value).__name__))
    return out


async def call(sock, service, args, ident, timeout=10.0):
    await sock.send(json.dumps({'op': 'call_service', 'service': service, 'args': args, 'id': ident}))
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        m = json.loads(await asyncio.wait_for(sock.recv(), max(0.01, end - time.monotonic())))
        if m.get('op') == 'service_response' and m.get('id') == ident:
            return m
    raise TimeoutError(service)


async def wait(url, seconds):
    end = time.monotonic() + seconds
    t0 = time.monotonic()
    while time.monotonic() < end:
        try:
            async with connect(url, gated=False) as sock:
                m = await call(sock, '/rosapi/topics', {}, 'w', timeout=5)
                if m.get('result'):
                    print(f'bridge up after {time.monotonic() - t0:.1f} s')
                    return 0
        except (OSError, asyncio.TimeoutError, TimeoutError, websockets.WebSocketException):
            pass
        await asyncio.sleep(1.0)
    print(f'bridge not up after {seconds} s')
    return 1


async def services(url, out):
    doc = {}
    async with connect(url, gated=False) as sock:
        m = await call(sock, '/rosapi/topics', {}, 'topics')
        v = m.get('values') or {}
        doc['/rosapi/topics'] = {'result': m.get('result'),
                                 'topics': dict(zip(v.get('topics', []), v.get('types', [])))}
        for i, (name, args) in enumerate(SERVICES):
            t0 = time.monotonic()
            try:
                m = await call(sock, name, args, f's{i}')
            except (asyncio.TimeoutError, TimeoutError):
                doc[name] = {'result': 'no answer'}
                continue
            entry = {'result': m.get('result'), 'rtt_ms': round((time.monotonic() - t0) * 1000, 1)}
            if name != '/rosapi/get_time':
                entry['values'] = norm(m.get('values'))
            doc[name] = entry
    with open(out, 'w') as f:
        json.dump(doc, f, indent=1, sort_keys=True, ensure_ascii=False)
    return 0


async def topics(url, seconds, out):
    async with connect(url, gated=True) as sock:
        m = await call(sock, '/rosapi/topics', {}, 'topics')
        v = m.get('values') or {}
        listed = dict(zip(v.get('topics', []), v.get('types', [])))
        for name in sorted(listed):
            await sock.send(json.dumps({'op': 'subscribe', 'topic': name}))
        seen = {name: {'type': t, 'count': 0, 'shape': set(), 'contents': []} for name, t in listed.items()}
        errors = []
        t0 = time.monotonic()
        end = t0 + seconds
        while time.monotonic() < end:
            try:
                raw = await asyncio.wait_for(sock.recv(), max(0.01, end - time.monotonic()))
            except asyncio.TimeoutError:
                break
            m = json.loads(raw)
            if m.get('op') == 'status':
                errors.append(m.get('msg'))
                continue
            if m.get('op') != 'publish' or m.get('topic') not in seen:
                continue
            e = seen[m['topic']]
            e['count'] += 1
            e.setdefault('first_s', round(time.monotonic() - t0, 2))
            e['max_bytes'] = max(e.get('max_bytes', 0), len(raw))
            shape(m.get('msg'), '', e['shape'])
            c = norm(m.get('msg'))
            e['last'] = c
            if c not in e['contents'] and len(e['contents']) < 50:
                e['contents'].append(c)
    for e in seen.values():
        e['shape'] = sorted(e['shape'])
    with open(out, 'w') as f:
        json.dump({'seconds': seconds, 'topics': seen, 'errors': errors}, f, indent=1, sort_keys=True,
                  ensure_ascii=False)
    return 0


def main():
    cmd, args = sys.argv[1], sys.argv[2:]
    if cmd == 'wait':
        return asyncio.run(wait(args[0], float(args[1])))
    if cmd == 'services':
        return asyncio.run(services(args[0], args[1]))
    if cmd == 'topics':
        return asyncio.run(topics(args[0], float(args[1]), args[2]))
    sys.exit(f'unknown command {cmd}')


if __name__ == '__main__':
    sys.exit(main())
