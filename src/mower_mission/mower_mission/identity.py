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
"""Robot identity and the app pairing hand-shake (docs/ROBOT_API.md, "配對").

``<state_dir>/identity.json`` is created once per robot by
``deploy/host/mower-pair`` (called from install.sh)::

    {"robot_id": "MW-7K3Q9P", "name": "lubancat", "secret": "<base32>",
     "created": 1789600000}

``robot_id`` is derived from the board's machine-id, ``secret`` is 160 random
bits. The QR code the operator scans carries both, so a paired app can prove
it knows the secret on every WebSocket connection:

    X-Mower-Robot:  MW-7K3Q9P
    X-Mower-Client: <stable id of the phone/app install>
    X-Mower-Time:   <unix seconds>
    X-Mower-Nonce:  <32 hex chars, fresh per connection>
    X-Mower-Mac:    hex(HMAC-SHA256(secret_bytes, "robot_id\\nclient\\ntime\\nnonce"))

``verify_mac`` accepts a time skew of ±MAC_MAX_SKEW_S and rejects a nonce seen
within that window (replay). Pure Python, no ROS, so it is unit-tested and
shared by the rosbridge auth proxy and robot_info.
"""

import base64
import hashlib
import hmac
import json
import os
import re
import secrets
import time

IDENTITY_FILE = 'identity.json'
MAC_MAX_SKEW_S = 60
MAC_HEADER_PREFIX = 'X-Mower-'
ROBOT_ID_RE = re.compile(r'^MW-[A-Z0-9]{6}$')
# Crockford-ish alphabet: no I, L, O, U so the sticker cannot be misread
_ID_ALPHABET = '0123456789ABCDEFGHJKMNPQRSTVWXYZ'


def state_dir() -> str:
    return os.environ.get('MOWER_STATE_DIR') or os.path.expanduser('~/.mower')


def robot_id_from_seed(seed: bytes) -> str:
    """Stable, human-readable id from a hardware seed (machine-id)."""
    digest = hashlib.sha256(b'mower-robot-id:' + seed).digest()
    chars = ''.join(_ID_ALPHABET[b % len(_ID_ALPHABET)] for b in digest[:6])
    return f'MW-{chars}'


def new_secret() -> str:
    """160 random bits, base32 without padding (what the QR carries)."""
    return base64.b32encode(secrets.token_bytes(20)).decode().rstrip('=')


def secret_bytes(secret: str) -> bytes:
    s = secret.strip().upper()
    s += '=' * (-len(s) % 8)
    return base64.b32decode(s)


def load_identity(directory=None):
    """Parsed identity.json or None (development / simulation)."""
    directory = directory or state_dir()
    try:
        with open(os.path.join(directory, IDENTITY_FILE)) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return None
    if not isinstance(data, dict) or not ROBOT_ID_RE.match(str(data.get('robot_id', ''))):
        return None
    if not data.get('secret'):
        return None
    return data


def compute_mac(secret: str, robot_id: str, client: str, t: int, nonce: str) -> str:
    message = f'{robot_id}\n{client}\n{int(t)}\n{nonce}'.encode()
    return hmac.new(secret_bytes(secret), message, hashlib.sha256).hexdigest()


class NonceCache:
    """Remember nonces for the skew window so a captured hand-shake cannot be
    replayed."""

    def __init__(self, ttl_s: int = 2 * MAC_MAX_SKEW_S):
        self._ttl = ttl_s
        self._seen = {}

    def check_and_add(self, nonce: str, now=None) -> bool:
        now = time.time() if now is None else now
        expired = [n for n, at in self._seen.items() if now - at > self._ttl]
        for n in expired:
            del self._seen[n]
        if nonce in self._seen:
            return False
        self._seen[nonce] = now
        return True


def verify_mac(identity: dict, headers, nonce_cache: NonceCache = None, now=None):
    """Validate the pairing headers of one connection.

    ``headers`` is any mapping with case-insensitive ``get`` (websockets'
    Headers) or a plain dict. Returns (ok, reason).
    """
    now = time.time() if now is None else now

    def get(name):
        value = headers.get(MAC_HEADER_PREFIX + name)
        if value is None and isinstance(headers, dict):
            for k, v in headers.items():
                if k.lower() == (MAC_HEADER_PREFIX + name).lower():
                    value = v
                    break
        return (value or '').strip()

    robot_id, client, t_raw, nonce, mac = (get(n) for n in ('Robot', 'Client', 'Time', 'Nonce', 'Mac'))
    if not (robot_id and client and t_raw and nonce and mac):
        return False, 'missing pairing headers'
    if robot_id != identity['robot_id']:
        return False, f'wrong robot ({robot_id})'
    try:
        t = int(t_raw)
    except ValueError:
        return False, 'bad time'
    if abs(now - t) > MAC_MAX_SKEW_S:
        return False, f'clock skew {int(now - t)} s'
    if not re.fullmatch(r'[0-9a-fA-F]{16,64}', nonce):
        return False, 'bad nonce'
    expected = compute_mac(identity['secret'], robot_id, client, t, nonce)
    if not hmac.compare_digest(expected, mac.lower()):
        return False, 'bad mac'
    if nonce_cache is not None and not nonce_cache.check_and_add(nonce.lower(), now):
        return False, 'replayed nonce'
    return True, client
