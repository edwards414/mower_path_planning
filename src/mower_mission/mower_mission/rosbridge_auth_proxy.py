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
"""Pairing gate in front of rosbridge.

rosbridge for ROS 2 dropped its rosauth support, so the robot's WebSocket
door is this small proxy instead: it listens on the address the app (or the
relay) reaches, checks the pairing headers of every new connection against
``identity.json`` (see identity.py) during the HTTP upgrade and only then
pipes the WebSocket frames to rosbridge, which itself stays on loopback.

    app/relay --ws--> proxy (rosbridge_address:9090) --ws--> rosbridge (127.0.0.1:9091)

Without an identity file (development, simulation) the proxy passes every
connection through and logs a warning, so `make mission` keeps working.

Plain asyncio + websockets (legacy API, present from websockets 9 through
16 so the noble apt package and newer pip releases both work), no rclpy: it
must keep serving while the ROS graph is busy and needs nothing from ROS.
"""

import argparse
import asyncio
import http
import logging
import signal
import sys

import websockets
from websockets.legacy.client import connect
from websockets.legacy.server import WebSocketServerProtocol, serve

from mower_mission import identity as ident

log = logging.getLogger('rosbridge_auth_proxy')


class AuthProxy:
    def __init__(self, upstream: str, identity, max_size: int):
        self.upstream = upstream
        self.identity = identity
        self.max_size = max_size
        self.nonces = ident.NonceCache()

    def check(self, headers, peer):
        """(client id | None, rejection reason)."""
        if self.identity is None:
            return 'open', None
        ok, why = ident.verify_mac(self.identity, headers, self.nonces)
        if not ok:
            log.warning('rejected %s: %s', peer, why)
            return None, why
        log.info('paired client %s from %s', why, peer)
        return why, None

    async def handle(self, client, path=None):
        subprotocols = [client.subprotocol] if client.subprotocol else None
        try:
            async with connect(
                self.upstream,
                subprotocols=subprotocols,
                max_size=self.max_size,
                compression=None,
            ) as upstream:
                await asyncio.gather(self._pump(client, upstream), self._pump(upstream, client))
        except (websockets.ConnectionClosed, OSError) as exc:
            log.debug('connection ended: %s', exc)
        finally:
            await client.close()

    @staticmethod
    async def _pump(src, dst):
        try:
            async for message in src:
                await dst.send(message)
        except websockets.ConnectionClosed:
            pass
        finally:
            await dst.close()


def make_protocol(proxy: AuthProxy):
    class GateProtocol(WebSocketServerProtocol):
        async def process_request(self, path, request_headers):
            client, why = proxy.check(request_headers, self.remote_address)
            if client is None:
                body = f'pairing required: {why}\n'.encode()
                return http.HTTPStatus.UNAUTHORIZED, [('Content-Type', 'text/plain')], body
            self.mower_client = client
            return None

    return GateProtocol


async def run(args) -> None:
    identity = ident.load_identity(args.state_dir)
    if identity is None:
        log.warning('no identity.json in %s: proxy runs OPEN (development mode)',
                    args.state_dir or ident.state_dir())
    else:
        log.info('pairing gate for %s', identity['robot_id'])
    proxy = AuthProxy(args.upstream, identity, args.max_size)

    loop = asyncio.get_running_loop()
    stop = loop.create_future()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, lambda: None if stop.done() else stop.set_result(None))

    async with serve(
        proxy.handle,
        args.address,
        args.port,
        create_protocol=make_protocol(proxy),
        max_size=args.max_size,
        compression=None,
        ping_interval=20,
        ping_timeout=20,
    ):
        log.info('listening on ws://%s:%d -> %s', args.address, args.port, args.upstream)
        await stop


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--address', default='127.0.0.1', help='listen address (rosbridge_address)')
    ap.add_argument('--port', type=int, default=9090)
    ap.add_argument('--upstream', default='ws://127.0.0.1:9091', help='rosbridge_websocket URL')
    ap.add_argument('--state-dir', default=None, help='where identity.json lives (default MOWER_STATE_DIR)')
    ap.add_argument('--max-size', type=int, default=64 * 1024 * 1024, help='max WebSocket frame size (maps are big)')
    ap.add_argument('--log-level', default='INFO')
    # ros2 launch appends --ros-args ...; ignore them
    args, _ = ap.parse_known_args(argv)
    logging.basicConfig(level=args.log_level, format='[%(name)s] %(levelname)s %(message)s', stream=sys.stdout)
    asyncio.run(run(args))


if __name__ == '__main__':
    main()
