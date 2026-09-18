"""mrelay1 framing (relay_protocol.py) and the backend agent (mower_agent.py), no ROS."""

import asyncio
import json
import time
import warnings

import pytest

from mower_mission import identity as ident
from mower_mission import relay_protocol as rp

SID = '0011223344556677'


def test_text_message_becomes_one_final_frame():
    frames = list(rp.encode_frames(SID, '{"op":"subscribe"}'))
    assert len(frames) == 1
    assert frames[0][0] == rp.T_TEXT
    assert frames[0][1:9] == bytes.fromhex(SID)
    assert frames[0][9:] == b'{"op":"subscribe"}'
    assert rp.decode_frame(frames[0]) == (SID, rp.T_TEXT, b'{"op":"subscribe"}')


def test_large_message_is_chunked_and_reassembled():
    big = 'x' * (rp.CHUNK_SIZE * 2 + 5)
    frames = list(rp.encode_frames(SID, big))
    assert [f[0] for f in frames] == [rp.T_TEXT_MORE, rp.T_TEXT_MORE, rp.T_TEXT]
    assert sum(len(f) - 9 for f in frames) == len(big)
    r = rp.Reassembler()
    out = [r.feed(f[0], f[9:]) for f in frames]
    assert out[:2] == [None, None]
    assert out[2] == big


def test_binary_and_empty_messages():
    frames = list(rp.encode_frames(SID, b'\x00\x01\x02', chunk_size=2))
    assert [f[0] for f in frames] == [rp.T_BIN_MORE, rp.T_BIN]
    r = rp.Reassembler()
    assert r.feed(frames[0][0], frames[0][9:]) is None
    assert r.feed(frames[1][0], frames[1][9:]) == b'\x00\x01\x02'
    empty = list(rp.encode_frames(SID, ''))
    assert empty == [bytes([rp.T_TEXT]) + bytes.fromhex(SID)]
    assert rp.Reassembler().feed(rp.T_TEXT, b'') == ''


def test_malformed_frames_and_oversize():
    assert rp.decode_frame(b'') is None
    assert rp.decode_frame(b'\x09' + b'\x00' * 8) is None
    assert rp.decode_frame(b'\x01' + b'\x00' * 7) is None
    r = rp.Reassembler(max_message=10)
    with pytest.raises(ValueError):
        r.feed(rp.T_TEXT_MORE, b'x' * 11)
    # after the error the reassembler is usable again
    assert r.feed(rp.T_TEXT, b'ok') == 'ok'


# ------------------------------------------------------------------ agent

VECTOR_SECRET = 'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA'
ROBOT = 'MW-7K3Q9P'


def test_ensure_device_key_persists(tmp_path):
    from mower_mission import mower_agent as agent_mod

    assert agent_mod.ensure_device_key(str(tmp_path)) is None
    (tmp_path / 'identity.json').write_text(json.dumps(
        {'robot_id': ROBOT, 'name': 'bench', 'secret': VECTOR_SECRET, 'created': 0}))
    first = agent_mod.ensure_device_key(str(tmp_path))
    assert len(ident.secret_bytes(first['device_key'])) == 32
    second = agent_mod.ensure_device_key(str(tmp_path))
    assert second['device_key'] == first['device_key']
    body = agent_mod.register_body(first, 'lubancat')
    assert body == {'robot_id': ROBOT, 'name': 'bench', 'model': 'lubancat',
                    'device_key': first['device_key'], 'pairing_secret': VECTOR_SECRET}
    headers = agent_mod.signed_headers(first)
    assert headers['X-Mower-Client'] == '@robot'
    assert ident.verify_mac({'robot_id': ROBOT, 'secret': first['device_key']}, headers)[0]


def test_http_to_ws():
    from mower_mission import mower_agent as agent_mod

    assert agent_mod.http_to_ws('https://api.example') == 'wss://api.example'
    assert agent_mod.http_to_ws('http://127.0.0.1:8787/') == 'ws://127.0.0.1:8787/'


@pytest.mark.filterwarnings('ignore::DeprecationWarning')
def test_agent_relays_sessions_through_the_gate():
    """Fake hub + real auth proxy + echo rosbridge: heartbeat arrives, an
    'open' becomes a gated session, frames flow both ways, close propagates,
    and a session with bad pairing headers is reported as open_err 401."""
    websockets = pytest.importorskip('websockets')
    from websockets.legacy.server import serve

    from mower_mission import mower_agent as agent_mod
    from mower_mission import rosbridge_auth_proxy as proxy_mod

    identity = {'robot_id': ROBOT, 'name': 'bench', 'secret': VECTOR_SECRET,
                'device_key': agent_mod.new_device_key(), 'created': 0}

    async def scenario():
        hub_events = asyncio.Queue()
        hub_conn = {}

        async def echo(ws, path=None):
            async for m in ws:
                await ws.send('echo:' + m if isinstance(m, str) else b'echo:' + m)

        async def hub(ws, path=None):
            # the fake hub: verify the robot's headers, then hand every
            # message to the test through the queue
            headers = ws.request_headers
            ok, who = ident.verify_mac({'robot_id': ROBOT, 'secret': identity['device_key']}, headers)
            assert ok and who == '@robot', who
            assert ws.subprotocol == rp.SUBPROTOCOL
            hub_conn['ws'] = ws
            async for m in ws:
                await hub_events.put(m)

        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            upstream = await serve(echo, '127.0.0.1', 0)
            up_port = upstream.sockets[0].getsockname()[1]
            proxy = proxy_mod.AuthProxy(f'ws://127.0.0.1:{up_port}', identity, 1 << 20)
            gate = await serve(proxy.handle, '127.0.0.1', 0, create_protocol=proxy_mod.make_protocol(proxy))
            gate_port = gate.sockets[0].getsockname()[1]
            hub_server = await serve(hub, '127.0.0.1', 0, subprotocols=[rp.SUBPROTOCOL])
            hub_port = hub_server.sockets[0].getsockname()[1]

        agent = agent_mod.Agent(identity, f'http://127.0.0.1:{hub_port}',
                                gate_url=f'ws://127.0.0.1:{gate_port}',
                                rosbridge_url='ws://127.0.0.1:1')  # no rosbridge: info stays None
        relay_task = asyncio.ensure_future(agent.run_relay_forever())

        async def next_control():
            while True:
                m = await asyncio.wait_for(hub_events.get(), 5)
                if isinstance(m, str):
                    return json.loads(m)

        async def next_frame():
            while True:
                m = await asyncio.wait_for(hub_events.get(), 5)
                if isinstance(m, bytes):
                    return rp.decode_frame(m)

        try:
            hb = await next_control()
            assert hb['t'] == 'hb' and 'lan' in hb
            hub_ws = hub_conn['ws']

            # a paired phone: open -> opened, echo both ways
            now = int(time.time())
            nonce = 'b' * 32
            phone = {
                'x-mower-robot': ROBOT, 'x-mower-client': 'iphone-1', 'x-mower-time': str(now),
                'x-mower-nonce': nonce,
                'x-mower-mac': ident.compute_mac(VECTOR_SECRET, ROBOT, 'iphone-1', now, nonce),
            }
            await hub_ws.send(json.dumps({'t': 'open', 'sid': SID, 'client': 'iphone-1', 'headers': phone}))
            assert (await next_control()) == {'t': 'opened', 'sid': SID}

            await hub_ws.send(bytes([rp.T_TEXT]) + bytes.fromhex(SID) + b'{"op":"subscribe"}')
            assert (await next_frame()) == (SID, rp.T_TEXT, b'echo:{"op":"subscribe"}')

            # chunked inbound message is glued before it reaches rosbridge
            await hub_ws.send(bytes([rp.T_TEXT_MORE]) + bytes.fromhex(SID) + b'ab')
            await hub_ws.send(bytes([rp.T_TEXT]) + bytes.fromhex(SID) + b'cd')
            assert (await next_frame()) == (SID, rp.T_TEXT, b'echo:abcd')

            # hub closes the session: local gate connection goes away, no echo of the close
            await hub_ws.send(json.dumps({'t': 'close', 'sid': SID, 'code': 1000}))
            await asyncio.sleep(0.1)
            assert SID not in agent.sessions

            # unpaired phone (replayed nonce): gate says 401 -> open_err
            await hub_ws.send(json.dumps({'t': 'open', 'sid': 'ffffffffffffffff', 'client': 'x', 'headers': phone}))
            err = await next_control()
            assert err['t'] == 'open_err' and err['sid'] == 'ffffffffffffffff' and err['code'] == 401

            # a frame for an unknown session is answered with close
            await hub_ws.send(bytes([rp.T_TEXT]) + bytes.fromhex('0000000000000001') + b'x')
            closed = await next_control()
            assert closed['t'] == 'close' and closed['sid'] == '0000000000000001'
        finally:
            relay_task.cancel()
            for srv in (hub_server, gate, upstream):
                srv.close()
                await srv.wait_closed()

    asyncio.run(scenario())
