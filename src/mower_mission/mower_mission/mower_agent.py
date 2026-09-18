# Copyright 2024 fxrbindi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Robot side of the fleet backend (docs/BACKEND_ARCHITECTURE.md).

The robot never accepts inbound connections from the internet (4G is behind
carrier NAT). Instead this agent:

1. makes sure ``identity.json`` has a ``device_key`` and registers the robot
   with the backend (``POST /v1/robots/register``, provision token);
2. keeps one outbound WebSocket to ``/v1/relay/robot/<robot_id>`` open and
   sends a heartbeat every 10 s with ``/robot/info``, ``/robot/telemetry``
   and the LAN address, so the app can see the robot online;
3. when the hub says ``{"t":"open"}`` (a phone connected), opens a session to
   the local pairing gate (rosbridge_auth_proxy, ``ws://127.0.0.1:9090``)
   with the phone's X-Mower-* headers, so the robot still verifies every
   phone itself, and pipes mrelay1 frames (relay_protocol.py) both ways.

No ``MOWER_BACKEND_URL`` -> the agent exits quietly (simulation, bench).
Plain asyncio + websockets legacy API (works with the noble apt package and
newer pip releases), no rclpy: it reads ROS topics through the local
rosbridge on loopback like any other client.
"""

import argparse
import asyncio
import json
import logging
import os
import secrets
import signal
import socket
import sys
import time
import urllib.error
import urllib.request

import websockets
from websockets.legacy.client import connect

from mower_mission import identity as ident
from mower_mission import relay_protocol as rp
from mower_mission.version import ROBOT_API_VERSION

log = logging.getLogger('mower_agent')

ROBOT_CLIENT = '@robot'
HEARTBEAT_S = 10
TELEMETRY_THROTTLE_MS = 5000
REGISTER_RETRY_S = 30
RECONNECT_MAX_S = 60
LOCAL_GATE = 'ws://127.0.0.1:9090'
LOCAL_ROSBRIDGE = 'ws://127.0.0.1:9091'


# ---------------------------------------------------------------- identity


def new_device_key() -> str:
    """256 random bits, base32 without padding."""
    import base64
    return base64.b32encode(secrets.token_bytes(32)).decode().rstrip('=')


def ensure_device_key(state_dir: str):
    """Load identity.json and add a device_key the first time. Returns the
    identity dict or None when the robot has no identity (development)."""
    identity = ident.load_identity(state_dir)
    if identity is None:
        return None
    if not identity.get('device_key'):
        identity['device_key'] = new_device_key()
        identity['device_key_created'] = int(time.time())
        path = os.path.join(state_dir or ident.state_dir(), ident.IDENTITY_FILE)
        tmp = path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(identity, f, indent=2)
            f.write('\n')
        os.chmod(tmp, 0o600)
        os.replace(tmp, path)
        log.info('created device_key for %s', identity['robot_id'])
    return identity


def signed_headers(identity: dict) -> dict:
    now = int(time.time())
    nonce = secrets.token_hex(16)
    return {
        'X-Mower-Robot': identity['robot_id'],
        'X-Mower-Client': ROBOT_CLIENT,
        'X-Mower-Time': str(now),
        'X-Mower-Nonce': nonce,
        'X-Mower-Mac': ident.compute_mac(identity['device_key'], identity['robot_id'], ROBOT_CLIENT, now, nonce),
    }


def lan_address() -> str:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.255.255.255', 1))
        return s.getsockname()[0]
    except OSError:
        return ''
    finally:
        s.close()


# ---------------------------------------------------------------- register


def register_body(identity: dict, model: str) -> dict:
    return {
        'robot_id': identity['robot_id'],
        'name': identity.get('name', ''),
        'model': model,
        'device_key': identity['device_key'],
        'pairing_secret': identity['secret'],
    }


def user_agent(identity: dict) -> str:
    """Identify the agent to the backend. Cloudflare's Browser Integrity
    Check blocks urllib's default "Python-urllib/x.y" with error 1010."""
    return f"mower-agent/{ROBOT_API_VERSION} ({identity.get('robot_id', '?')})"


def register_once(base_url: str, token: str, identity: dict, model: str, timeout: float = 15.0):
    """POST /v1/robots/register. Returns (status, body dict). Raises OSError
    / URLError on network trouble."""
    req = urllib.request.Request(
        base_url.rstrip('/') + '/v1/robots/register',
        data=json.dumps(register_body(identity, model)).encode(),
        headers={
            'Content-Type': 'application/json',
            'Authorization': f'Bearer {token}',
            'User-Agent': user_agent(identity),
        },
        method='POST',
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return resp.status, json.loads(resp.read() or b'{}')
    except urllib.error.HTTPError as exc:
        try:
            body = json.loads(exc.read() or b'{}')
        except ValueError:
            body = {}
        return exc.code, body


def http_to_ws(base_url: str) -> str:
    if base_url.startswith('https://'):
        return 'wss://' + base_url[len('https://'):]
    if base_url.startswith('http://'):
        return 'ws://' + base_url[len('http://'):]
    return base_url


# ---------------------------------------------------------------- sessions


class Session:
    """One phone connection relayed to the local pairing gate."""

    def __init__(self, agent, sid: str, headers: dict):
        self.agent = agent
        self.sid = sid
        self.headers = headers
        self.inbox = asyncio.Queue()
        self.reassembler = rp.Reassembler()
        self.local = None
        self.task = asyncio.ensure_future(self.run())

    async def run(self):
        try:
            self.local = await connect(
                self.agent.gate_url,
                extra_headers=list(self.headers.items()),
                max_size=rp.MAX_MESSAGE,
                compression=None,
            )
        except websockets.InvalidStatusCode as exc:
            log.warning('session %s refused by the gate: HTTP %s', self.sid, exc.status_code)
            await self.agent.send_control({'t': 'open_err', 'sid': self.sid, 'code': exc.status_code,
                                           'reason': 'gate refused'})
            self.agent.sessions.pop(self.sid, None)
            return
        except (OSError, websockets.WebSocketException) as exc:
            log.warning('session %s: gate unreachable: %s', self.sid, exc)
            await self.agent.send_control({'t': 'open_err', 'sid': self.sid, 'code': 503,
                                           'reason': 'gate unreachable'})
            self.agent.sessions.pop(self.sid, None)
            return
        await self.agent.send_control({'t': 'opened', 'sid': self.sid})
        log.info('session %s open (%s)', self.sid, self.headers.get('x-mower-client') or self.headers.get('X-Mower-Client'))
        try:
            await asyncio.gather(self._to_local(), self._to_relay())
        except websockets.ConnectionClosed:
            pass
        finally:
            await self.close(1000, 'session ended')

    async def _to_local(self):
        while True:
            message = await self.inbox.get()
            if message is None:
                return
            await self.local.send(message)

    async def _to_relay(self):
        try:
            async for message in self.local:
                for frame in rp.encode_frames(self.sid, message):
                    await self.agent.send_raw(frame)
        finally:
            self.inbox.put_nowait(None)  # rosbridge side ended: stop _to_local too

    def feed(self, frame_type: int, payload: bytes):
        try:
            message = self.reassembler.feed(frame_type, payload)
        except ValueError as exc:
            log.warning('session %s: %s', self.sid, exc)
            asyncio.ensure_future(self.close(1009, str(exc)))
            return
        if message is not None:
            self.inbox.put_nowait(message)

    async def close(self, code: int = 1000, reason: str = '', notify: bool = True):
        if self.agent.sessions.pop(self.sid, None) is None:
            return
        if notify:
            await self.agent.send_control({'t': 'close', 'sid': self.sid, 'code': code, 'reason': reason})
        self.inbox.put_nowait(None)
        if self.local is not None:
            await self.local.close()


# ---------------------------------------------------------------- agent


class Agent:
    def __init__(self, identity: dict, backend_url: str, gate_url: str = LOCAL_GATE,
                 rosbridge_url: str = LOCAL_ROSBRIDGE, model: str = ''):
        self.identity = identity
        self.backend_url = backend_url.rstrip('/')
        # A gate bound to 0.0.0.0 is dialled on loopback.
        self.gate_url = gate_url.replace('://0.0.0.0:', '://127.0.0.1:')
        self.rosbridge_url = rosbridge_url
        self.model = model
        self.sessions = {}
        self.relay = None
        self.info = None
        self.telemetry = None
        self.heartbeats_sent = 0

    @property
    def relay_url(self) -> str:
        return http_to_ws(self.backend_url) + '/v1/relay/robot/' + self.identity['robot_id']

    # -- outbound helpers --

    async def send_control(self, obj: dict):
        if self.relay is not None:
            try:
                await self.relay.send(json.dumps(obj))
            except websockets.ConnectionClosed:
                pass

    async def send_raw(self, frame: bytes):
        if self.relay is not None:
            await self.relay.send(frame)

    def heartbeat(self) -> dict:
        return {'t': 'hb', 'info': self.info, 'telemetry': self.telemetry, 'lan': lan_address()}

    # -- tasks --

    async def run_relay_forever(self):
        delay = 1
        while True:
            try:
                await self.run_relay_once()
                delay = 1
            except websockets.InvalidStatusCode as exc:
                log.warning('relay refused us: HTTP %s (re-registering)', exc.status_code)
                await self.register(wait=False)
            except (OSError, websockets.WebSocketException, asyncio.TimeoutError) as exc:
                log.warning('relay connection ended: %s', exc)
            await self._close_all_sessions()
            log.info('reconnecting in %d s', delay)
            await asyncio.sleep(delay)
            delay = min(delay * 2, RECONNECT_MAX_S)

    async def run_relay_once(self):
        log.info('connecting to %s', self.relay_url)
        headers = signed_headers(self.identity)
        headers['User-Agent'] = user_agent(self.identity)
        async with connect(
            self.relay_url,
            extra_headers=list(headers.items()),
            subprotocols=[rp.SUBPROTOCOL],
            max_size=2 * 1024 * 1024,
            compression=None,
            ping_interval=20,
            ping_timeout=20,
        ) as ws:
            self.relay = ws
            log.info('relay connected')
            hb = asyncio.ensure_future(self._heartbeat_loop())
            try:
                async for message in ws:
                    if isinstance(message, str):
                        await self._on_control(message)
                    else:
                        self._on_frame(message)
            finally:
                hb.cancel()
                self.relay = None

    async def _heartbeat_loop(self):
        while True:
            await self.send_control(self.heartbeat())
            self.heartbeats_sent += 1
            await asyncio.sleep(HEARTBEAT_S)

    async def _on_control(self, text: str):
        try:
            msg = json.loads(text)
        except ValueError:
            return
        t = msg.get('t')
        if t == 'open':
            sid = msg.get('sid')
            headers = msg.get('headers') or {}
            if not isinstance(sid, str) or len(sid) != 2 * rp.SID_BYTES or not isinstance(headers, dict):
                return
            if sid in self.sessions:
                return
            self.sessions[sid] = Session(self, sid, {str(k): str(v) for k, v in headers.items()})
        elif t == 'close':
            session = self.sessions.get(msg.get('sid'))
            if session is not None:
                await session.close(notify=False)
        elif t in ('clients', 'pair_confirm', 'http'):
            log.info('control %s not implemented yet', t)

    def _on_frame(self, frame: bytes):
        decoded = rp.decode_frame(frame)
        if decoded is None:
            return
        sid, frame_type, payload = decoded
        session = self.sessions.get(sid)
        if session is None:
            asyncio.ensure_future(self.send_control({'t': 'close', 'sid': sid, 'code': 1000, 'reason': 'no such session'}))
            return
        session.feed(frame_type, payload)

    async def _close_all_sessions(self):
        for session in list(self.sessions.values()):
            await session.close(1012, 'relay lost', notify=False)

    async def register(self, token: str = None, wait: bool = True):
        """Register until it works (network) or fails for good (409)."""
        token = token if token is not None else os.environ.get('MOWER_PROVISION_TOKEN', '')
        if not token:
            log.warning('MOWER_PROVISION_TOKEN not set: skipping registration')
            return False
        while True:
            try:
                status, body = await asyncio.get_running_loop().run_in_executor(
                    None, register_once, self.backend_url, token, self.identity, self.model)
            except (OSError, urllib.error.URLError) as exc:
                log.warning('register: %s', exc)
                status, body = 0, {}
            if status in (200, 201):
                log.info('registered %s (%s)', self.identity['robot_id'], body.get('outcome'))
                return True
            if status == 409:
                log.error('register: %s (another device_key holds this robot_id); relay will be refused',
                          body.get('error'))
                return False
            if status == 401:
                log.error('register: bad provision token')
                return False
            if status:
                log.warning('register: HTTP %s %s', status, body.get('error') or body)
            if not wait:
                return False
            await asyncio.sleep(REGISTER_RETRY_S)

    async def watch_topics_forever(self):
        """Keep /robot/info and /robot/telemetry fresh through the local
        rosbridge (loopback, no pairing gate)."""
        while True:
            try:
                async with connect(self.rosbridge_url, max_size=1 << 20, compression=None) as ws:
                    await ws.send(json.dumps({'op': 'subscribe', 'topic': '/robot/info', 'type': 'std_msgs/msg/String'}))
                    await ws.send(json.dumps({'op': 'subscribe', 'topic': '/robot/telemetry',
                                              'type': 'std_msgs/msg/String', 'throttle_rate': TELEMETRY_THROTTLE_MS}))
                    async for message in ws:
                        self._on_topic(message)
            except (OSError, websockets.WebSocketException) as exc:
                log.debug('rosbridge watch: %s', exc)
            await asyncio.sleep(5)

    def _on_topic(self, message):
        try:
            msg = json.loads(message)
            if msg.get('op') != 'publish':
                return
            data = json.loads(msg['msg']['data'])
        except (ValueError, KeyError, TypeError):
            return
        if msg.get('topic') == '/robot/info':
            self.info = data
        elif msg.get('topic') == '/robot/telemetry':
            self.telemetry = data


async def run(args) -> int:
    backend = args.backend_url or os.environ.get('MOWER_BACKEND_URL', '')
    if not backend:
        log.warning('MOWER_BACKEND_URL not set: agent idle (development mode)')
        return 0
    identity = ensure_device_key(args.state_dir)
    if identity is None:
        log.warning('no identity.json in %s: agent idle', args.state_dir or ident.state_dir())
        return 0
    agent = Agent(identity, backend, args.gate, args.rosbridge, args.model)

    loop = asyncio.get_running_loop()
    stop = loop.create_future()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, lambda: None if stop.done() else stop.set_result(None))

    await agent.register()
    tasks = [asyncio.ensure_future(agent.run_relay_forever()),
             asyncio.ensure_future(agent.watch_topics_forever())]
    await stop
    for t in tasks:
        t.cancel()
    await agent._close_all_sessions()
    return 0


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--backend-url', default=None, help='https://api.mower.example (default MOWER_BACKEND_URL)')
    ap.add_argument('--state-dir', default=None, help='where identity.json lives (default MOWER_STATE_DIR)')
    ap.add_argument('--gate', default=LOCAL_GATE, help='rosbridge_auth_proxy URL')
    ap.add_argument('--rosbridge', default=LOCAL_ROSBRIDGE, help='rosbridge_websocket URL (loopback)')
    ap.add_argument('--model', default=os.environ.get('MOWER_MODEL', 'lubancat'))
    ap.add_argument('--log-level', default='INFO')
    args, _ = ap.parse_known_args(argv)  # ros2 launch appends --ros-args
    logging.basicConfig(level=args.log_level, format='[%(name)s] %(levelname)s %(message)s', stream=sys.stdout)
    sys.exit(asyncio.run(run(args)))


if __name__ == '__main__':
    main()
