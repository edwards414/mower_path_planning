"""Phase 3 of the fleet backend in mower_agent.py: WHEP signaling relayed to
MediaMTX and TURN credentials written through the MediaMTX API. No ROS."""

import asyncio
import base64
import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import pytest

from mower_mission import mower_agent as agent_mod

IDENTITY = {'robot_id': 'MW-7K3Q9P', 'name': 'bench', 'secret': 'A' * 32,
            'device_key': 'B' * 52, 'created': 0}


class FakeMediaMtx(BaseHTTPRequestHandler):
    """/front/whep answers like MediaMTX; /v3/config/global/patch records
    the patch; anything else is 404."""
    patches = []
    requests = []

    def _read(self):
        n = int(self.headers.get('Content-Length') or 0)
        return self.rfile.read(n) if n else b''

    def do_POST(self):
        body = self._read()
        FakeMediaMtx.requests.append(('POST', self.path, dict(self.headers.items()), body))
        if self.path.startswith('/front/whep'):
            answer = b'v=0\r\na=answer\r\n'
            self.send_response(201)
            self.send_header('Content-Type', 'application/sdp')
            self.send_header('Location', '/front/whep/sess-1')
            self.send_header('ETag', '"1"')
            self.send_header('Set-Cookie', 'nope')
            self.send_header('Content-Length', str(len(answer)))
            self.end_headers()
            self.wfile.write(answer)
        else:
            self.send_error(404)

    def do_DELETE(self):
        FakeMediaMtx.requests.append(('DELETE', self.path, dict(self.headers.items()), b''))
        self.send_response(200)
        self.send_header('Content-Length', '0')
        self.end_headers()

    def do_PATCH(self):
        body = self._read()
        FakeMediaMtx.patches.append((self.path, json.loads(body)))
        self.send_response(200)
        self.send_header('Content-Length', '0')
        self.end_headers()

    def log_message(self, *args):  # quiet
        pass


@pytest.fixture
def mediamtx():
    FakeMediaMtx.patches.clear()
    FakeMediaMtx.requests.clear()
    server = HTTPServer(('127.0.0.1', 0), FakeMediaMtx)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield f'http://127.0.0.1:{server.server_address[1]}'
    server.shutdown()


class FakeRelay:
    def __init__(self):
        self.sent = []

    async def send(self, data):
        self.sent.append(json.loads(data))


def run(coro):
    return asyncio.new_event_loop().run_until_complete(coro)


def test_whep_post_and_delete_are_relayed(mediamtx):
    agent = agent_mod.Agent(IDENTITY, 'http://backend', mediamtx_url=mediamtx, mediamtx_api_url=mediamtx)
    agent.relay = FakeRelay()
    offer = b'v=0\r\no=- 1 1 IN IP4 0.0.0.0\r\n'

    async def scenario():
        await agent._on_control(json.dumps({
            't': 'http', 'rid': 'r1', 'method': 'POST', 'path': '/front/whep?x=1',
            'headers': {'content-type': 'application/sdp'},
            'body_b64': base64.b64encode(offer).decode(),
        }))
        # _on_control schedules the relay as a task; wait for its answer
        for _ in range(100):
            if agent.relay.sent:
                break
            await asyncio.sleep(0.05)
        assert agent.relay.sent, 'no http_res sent'
        res = agent.relay.sent[0]
        assert res['t'] == 'http_res' and res['rid'] == 'r1' and res['status'] == 201
        assert base64.b64decode(res['body_b64']) == b'v=0\r\na=answer\r\n'
        lowered = {k.lower(): v for k, v in res['headers'].items()}
        assert lowered['location'] == '/front/whep/sess-1'
        assert lowered['content-type'] == 'application/sdp'
        assert 'set-cookie' not in lowered
        method, path, headers, body = FakeMediaMtx.requests[0]
        assert (method, path, body) == ('POST', '/front/whep?x=1', offer)
        assert {k.lower(): v for k, v in headers.items()}['content-type'] == 'application/sdp'

        agent.relay.sent.clear()
        await agent._relay_http({'t': 'http', 'rid': 'r2', 'method': 'DELETE', 'path': '/front/whep/sess-1',
                                 'headers': {}, 'body_b64': ''})
        assert agent.relay.sent[0]['status'] == 200
        assert FakeMediaMtx.requests[1][:2] == ('DELETE', '/front/whep/sess-1')

    run(scenario())


def test_only_whep_paths_are_relayed(mediamtx):
    agent = agent_mod.Agent(IDENTITY, 'http://backend', mediamtx_url=mediamtx, mediamtx_api_url=mediamtx)
    agent.relay = FakeRelay()
    for method, path in [('POST', '/front/whip'), ('POST', '/v3/config/global/patch'),
                         ('POST', '/../front/whep'), ('PUT', '/front/whep'), ('POST', '/front/whep/a/b')]:
        run(agent._relay_http({'t': 'http', 'rid': 'x', 'method': method, 'path': path, 'headers': {}, 'body_b64': ''}))
    assert [m['status'] for m in agent.relay.sent] == [403] * 5
    assert FakeMediaMtx.requests == []


def test_unreachable_mediamtx_is_a_502():
    agent = agent_mod.Agent(IDENTITY, 'http://backend', mediamtx_url='http://127.0.0.1:1')
    agent.relay = FakeRelay()
    run(agent._relay_http({'t': 'http', 'rid': 'x', 'method': 'POST', 'path': '/front/whep', 'headers': {}, 'body_b64': ''}))
    assert agent.relay.sent[0]['status'] == 502


def test_ice_servers_mapping_and_patch(mediamtx):
    backend = {'iceServers': [
        {'urls': ['stun:stun.cloudflare.com:3478']},
        {'urls': ['turn:turn.cloudflare.com:3478?transport=udp',
                  'turn:turn.cloudflare.com:3478?transport=tcp',
                  'turn:turn.cloudflare.com:80?transport=tcp',
                  'turns:turn.cloudflare.com:443?transport=tcp'],
         'username': 'u', 'credential': 'p'},
    ], 'expires_at': 1}
    servers = agent_mod.mediamtx_ice_servers(backend['iceServers'])
    assert [s['url'] for s in servers] == ['turn:turn.cloudflare.com:3478?transport=udp',
                                           'turn:turn.cloudflare.com:3478?transport=tcp']
    assert servers[0] == {'url': 'turn:turn.cloudflare.com:3478?transport=udp', 'username': 'u',
                          'password': 'p', 'clientOnly': False}
    # STUN only (backend without a TURN key) -> MediaMTX gets an empty list
    assert agent_mod.mediamtx_ice_servers([{'urls': ['stun:stun.cloudflare.com:3478']}]) == []

    assert agent_mod.patch_mediamtx_config(mediamtx, {'webrtcICEServers2': servers}) == 200
    assert FakeMediaMtx.patches == [('/v3/config/global/patch', {'webrtcICEServers2': servers})]
