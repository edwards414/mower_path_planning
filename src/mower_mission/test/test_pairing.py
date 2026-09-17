"""Pairing identity / MAC scheme and the rosbridge auth proxy (no ROS)."""

import asyncio
import json
import time
import warnings

import pytest

from mower_mission import identity as ident

# Shared test vector with the app (mower_lawer_app test/pairing_test.dart):
# secret is 20 zero bytes -> base32 "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
VECTOR_SECRET = 'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA'
VECTOR_ROBOT = 'MW-7K3Q9P'
VECTOR_CLIENT = 'iphone-1234'
VECTOR_T = 1789600000
VECTOR_NONCE = '00112233445566778899aabbccddeeff'
VECTOR_MAC = ident.compute_mac(VECTOR_SECRET, VECTOR_ROBOT, VECTOR_CLIENT, VECTOR_T, VECTOR_NONCE)


def _identity(secret=VECTOR_SECRET):
    return {'robot_id': VECTOR_ROBOT, 'name': 'bench', 'secret': secret, 'created': 0}


def _headers(mac=None, robot=VECTOR_ROBOT, t=VECTOR_T, nonce=VECTOR_NONCE, client=VECTOR_CLIENT):
    return {
        'X-Mower-Robot': robot,
        'X-Mower-Client': client,
        'X-Mower-Time': str(t),
        'X-Mower-Nonce': nonce,
        'X-Mower-Mac': mac or ident.compute_mac(VECTOR_SECRET, robot, client, t, nonce),
    }


def test_robot_id_is_stable_and_readable():
    a = ident.robot_id_from_seed(b'machine-id-1')
    assert ident.ROBOT_ID_RE.match(a)
    assert a == ident.robot_id_from_seed(b'machine-id-1')
    assert a != ident.robot_id_from_seed(b'machine-id-2')
    for ch in a[3:]:
        assert ch not in 'ILOU'


def test_secret_roundtrip_and_vector():
    s = ident.new_secret()
    assert len(ident.secret_bytes(s)) == 20
    assert ident.secret_bytes(VECTOR_SECRET) == b'\x00' * 20
    # keep this in sync with the Dart test (mower_lawer_app test/pairing_test.dart)
    assert VECTOR_MAC == 'd33d2137cf8c6bb75ba80ff22b9afbf32a09f2346e83617e41e96026aa2cfd42'


def test_verify_mac_accepts_and_rejects():
    identity = _identity()
    now = VECTOR_T + 5
    ok, who = ident.verify_mac(identity, _headers(), now=now)
    assert ok and who == VECTOR_CLIENT
    # case-insensitive header names
    lower = {k.lower(): v for k, v in _headers().items()}
    assert ident.verify_mac(identity, lower, now=now)[0]
    assert ident.verify_mac(identity, {}, now=now) == (False, 'missing pairing headers')
    assert ident.verify_mac(identity, _headers(robot='MW-XXXXXX'), now=now)[0] is False
    assert ident.verify_mac(identity, _headers(mac='00' * 32), now=now)[0] is False
    assert ident.verify_mac(identity, _headers(), now=VECTOR_T + 200)[0] is False
    assert ident.verify_mac(identity, _headers(nonce='xyz'), now=now)[0] is False
    # wrong secret on the robot side
    assert ident.verify_mac(_identity(ident.new_secret()), _headers(), now=now)[0] is False


def test_nonce_replay_rejected_then_expires():
    identity = _identity()
    cache = ident.NonceCache(ttl_s=10)
    assert ident.verify_mac(identity, _headers(), cache, now=VECTOR_T)[0]
    ok, why = ident.verify_mac(identity, _headers(), cache, now=VECTOR_T + 1)
    assert not ok and why == 'replayed nonce'
    assert ident.verify_mac(identity, _headers(t=VECTOR_T + 20), cache, now=VECTOR_T + 20)[0]


def test_load_identity(tmp_path):
    assert ident.load_identity(str(tmp_path)) is None
    (tmp_path / 'identity.json').write_text(json.dumps(_identity()))
    assert ident.load_identity(str(tmp_path))['robot_id'] == VECTOR_ROBOT
    (tmp_path / 'identity.json').write_text(json.dumps({'robot_id': 'bad', 'secret': 'x'}))
    assert ident.load_identity(str(tmp_path)) is None


@pytest.mark.filterwarnings('ignore::DeprecationWarning')
def test_proxy_gates_connections(tmp_path):
    """Real proxy in front of an echo server: paired client passes, others 401."""
    websockets = pytest.importorskip('websockets')
    from websockets.legacy.client import connect
    from websockets.legacy.server import serve

    from mower_mission import rosbridge_auth_proxy as proxy_mod

    async def scenario():
        async def echo(ws, path=None):
            async for m in ws:
                await ws.send('echo:' + m)

        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            upstream = await serve(echo, '127.0.0.1', 0)
        up_port = upstream.sockets[0].getsockname()[1]
        proxy = proxy_mod.AuthProxy(f'ws://127.0.0.1:{up_port}', _identity(), 1 << 20)
        gate = await serve(proxy.handle, '127.0.0.1', 0, create_protocol=proxy_mod.make_protocol(proxy))
        port = gate.sockets[0].getsockname()[1]
        url = f'ws://127.0.0.1:{port}'
        try:
            # unpaired -> HTTP 401 during the handshake
            with pytest.raises(websockets.InvalidStatusCode) as exc:
                await connect(url)
            assert exc.value.status_code == 401

            # paired -> frames flow both ways through rosbridge (the echo here)
            now = int(time.time())
            nonce = 'a' * 32
            headers = _headers(t=now, nonce=nonce)
            async with connect(url, extra_headers=headers) as ws:
                await ws.send('{"op":"subscribe"}')
                assert await asyncio.wait_for(ws.recv(), 5) == 'echo:{"op":"subscribe"}'

            # the same hand-shake again is a replay
            with pytest.raises(websockets.InvalidStatusCode):
                await connect(url, extra_headers=headers)
        finally:
            gate.close()
            upstream.close()
            await gate.wait_closed()
            await upstream.wait_closed()

    asyncio.run(scenario())
