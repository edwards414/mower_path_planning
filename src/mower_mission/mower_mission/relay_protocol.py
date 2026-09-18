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
"""``mrelay1`` framing between the robot and the backend hub.

See docs/BACKEND_ARCHITECTURE.md §5. Cloudflare caps one WebSocket message at
1 MiB while rosbridge map messages run to tens of MB, so every rosbridge
message is cut into chunks here and glued back together on the phone (and
the other way round). The hub only adds or strips the 8-byte session id.

    robot <-> hub :  [type][sid: 8 bytes][payload]
    app   <-> hub :  [type][payload]

type: 0x01 text (last chunk), 0x11 text (more follows),
      0x02 binary (last chunk), 0x12 binary (more follows).

Pure Python, no ROS, unit-tested in test/test_relay_protocol.py.
"""

SUBPROTOCOL = 'mrelay1'
SID_BYTES = 8
T_TEXT = 0x01
T_TEXT_MORE = 0x11
T_BIN = 0x02
T_BIN_MORE = 0x12
MORE_BIT = 0x10
CHUNK_SIZE = 512 * 1024
MAX_MESSAGE = 64 * 1024 * 1024

_TYPES = {T_TEXT, T_TEXT_MORE, T_BIN, T_BIN_MORE}


def sid_bytes(sid_hex: str) -> bytes:
    b = bytes.fromhex(sid_hex)
    if len(b) != SID_BYTES:
        raise ValueError('bad session id')
    return b


def encode_frames(sid_hex: str, message, chunk_size: int = CHUNK_SIZE):
    """Yield robot->hub frames for one rosbridge message (str or bytes)."""
    if isinstance(message, str):
        payload = message.encode()
        final, more = T_TEXT, T_TEXT_MORE
    else:
        payload = bytes(message)
        final, more = T_BIN, T_BIN_MORE
    sid = sid_bytes(sid_hex)
    if not payload:
        yield bytes([final]) + sid
        return
    for start in range(0, len(payload), chunk_size):
        chunk = payload[start:start + chunk_size]
        last = start + chunk_size >= len(payload)
        yield bytes([final if last else more]) + sid + chunk


def decode_frame(frame: bytes):
    """(sid_hex, type, payload) of one hub->robot frame, or None if malformed."""
    if len(frame) < 1 + SID_BYTES or frame[0] not in _TYPES:
        return None
    return frame[1:1 + SID_BYTES].hex(), frame[0], frame[1 + SID_BYTES:]


class Reassembler:
    """Glue chunk payloads of one session back into rosbridge messages."""

    def __init__(self, max_message: int = MAX_MESSAGE):
        self._max = max_message
        self._parts = []
        self._size = 0

    def feed(self, frame_type: int, payload: bytes):
        """Return the complete message (str for text, bytes for binary) when
        this chunk finishes one, else None. Raises ValueError when the message
        grows past max_message."""
        self._size += len(payload)
        if self._size > self._max:
            self.reset()
            raise ValueError('relayed message too large')
        self._parts.append(payload)
        if frame_type & MORE_BIT:
            return None
        data = b''.join(self._parts)
        self.reset()
        if frame_type == T_TEXT:
            return data.decode('utf-8', errors='replace')
        return data

    def reset(self):
        self._parts = []
        self._size = 0
