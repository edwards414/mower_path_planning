#!/usr/bin/env python3
# Copyright 2026 fxrbindi
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
"""mower-check-run — 檢查錄製 run 目錄能不能交給 GrassVision 離線推論。

    mower-check-run <run_dir> [--json out.json] [--strict] ...
    python3 check_run.py <run_dir>            # 單檔可直接複製到推論端使用

對照 doc/Robot_Recording_R2_Pipeline_Spec.md 第 1 節，輸出 PASS / WARN / FAIL
清單：必要話題與頻率、影像與 camera_info 1:1 與尺寸（P-081）、/tf_static 相機鏈、
header.stamp 單調遞增與擷取延遲、標定檔、run_metadata 欄位，以及（若有）
_manifest.json 的大小與 sha256。

依賴：Python 3.8+、PyYAML。選用：zstandard（或 pyzstd／Python 3.14
compression.zstd）讀 zstd chunk、lz4 讀 lz4 chunk。沒有 zstd 時，zstd 壓縮的
telemetry bag 仍會透過 MCAP message index 檢查訊息數與接收頻率，但該 bag 的
header.stamp 與 /tf 內容會標成 SKIP。不需要 ROS。

結束碼：0 = 沒有 FAIL；1 = 有 FAIL（--strict 時 WARN 也算）；2 = 參數錯誤。
"""
import argparse
import hashlib
import json
import math
import os
import re
import statistics
import struct
import sys

import yaml

# ── spec constants ───────────────────────────────────────────────────────────
CAMERA_IMAGE = '/camera/front/image_raw/compressed'
CAMERA_INFO = '/camera/front/camera_info'
OPTICAL_FRAME = 'camera_front_optical_frame'
CAMERA_LINK = 'camera_front_link'
TF_CHAIN = ('base_footprint', 'base_link', CAMERA_LINK, OPTICAL_FRAME)
DEFAULT_CAMERA_HZ = 10.0

# topic -> expectation. Rates are receive rates (MCAP log_time).
# (fail_below, warn_below) in Hz; camera limits come from fps_recorded.
REQUIRED_TOPICS = {
    CAMERA_IMAGE: {'type': 'sensor_msgs/msg/CompressedImage', 'camera': True},
    CAMERA_INFO: {'type': 'sensor_msgs/msg/CameraInfo', 'camera': True},
    '/tf': {'type': 'tf2_msgs/msg/TFMessage', 'hz': (5.0, 20.0)},
    '/tf_static': {'type': 'tf2_msgs/msg/TFMessage', 'hz': None},
    '/imu/data': {'type': 'sensor_msgs/msg/Imu', 'hz': (2.0, 8.0)},
    '/odom': {'type': 'nav_msgs/msg/Odometry', 'hz': (2.0, 10.0)},
    '/fix': {'type': 'sensor_msgs/msg/NavSatFix', 'hz': (0.5, 3.0)},
    '/gps/status': {'type': 'std_msgs/msg/String', 'hz': (0.5, 0.8)},
    '/odometry/local': {'type': 'nav_msgs/msg/Odometry', 'hz': (5.0, 15.0)},
    '/odometry/global': {'type': 'nav_msgs/msg/Odometry', 'hz': (5.0, 15.0)},
    '/odometry/gps': {'type': 'nav_msgs/msg/Odometry', 'hz': (0.5, 2.0)},
}
RECOMMENDED_TOPICS = ('/cmd_vel', '/mower_base/telemetry', '/battery_state',
                      '/diagnostics', '/rosout', '/graph_snapshot',
                      '/graph_events')
# Single-publisher sensor topics whose header.stamp must not go backwards.
STAMP_TOPICS = (CAMERA_IMAGE, CAMERA_INFO, '/imu/data', '/odom', '/fix',
                '/odometry/local', '/odometry/global', '/odometry/gps')
REQUIRED_TF_PAIRS = {('odom', 'base_footprint'): 10.0, ('map', 'odom'): 5.0}
METADATA_FIELDS = (
    'profile', 'camera.model', 'camera.serial', 'camera.device',
    'camera.width', 'camera.height', 'camera.fps_native',
    'camera.fps_recorded', 'camera.format', 'camera.autofocus',
    'camera.exposure', 'camera_height_m', 'camera_pitch_deg',
    'extrinsics_source', 'calibration.intrinsics_file',
    'calibration.extrinsics_file', 'calibration.date', 'grass_height_cm',
    'weather', 'blade', 'heading_calibration', 'notes')
HEADER_TYPES = {
    'sensor_msgs/msg/CompressedImage', 'sensor_msgs/msg/CameraInfo',
    'sensor_msgs/msg/Image', 'sensor_msgs/msg/Imu',
    'sensor_msgs/msg/NavSatFix', 'sensor_msgs/msg/BatteryState',
    'sensor_msgs/msg/LaserScan', 'nav_msgs/msg/Odometry',
    'geometry_msgs/msg/TwistStamped', 'geometry_msgs/msg/PoseStamped',
    'diagnostic_msgs/msg/DiagnosticArray'}
ZSTD_MAGIC = b'\x28\xb5\x2f\xfd'

# ── MCAP (spec v0) reader: stdlib only, zstd/lz4 optional ────────────────────
MCAP_MAGIC = b'\x89MCAP0\r\n'
OP_HEADER, OP_FOOTER, OP_SCHEMA, OP_CHANNEL, OP_MESSAGE, OP_CHUNK = 1, 2, 3, 4, 5, 6
OP_MESSAGE_INDEX, OP_STATISTICS, OP_DATA_END = 7, 11, 15


class McapError(Exception):
    pass


def _zstd_decompressor():
    try:
        import zstandard  # noqa: WPS433
        d = zstandard.ZstdDecompressor()

        def _dec(data, size):
            if size:  # MCAP chunk: size known from the chunk record
                return d.decompress(data, max_output_size=size)
            return d.decompressobj().decompress(data)  # frame w/o size
        return _dec
    except ImportError:
        pass
    try:
        from compression import zstd  # Python 3.14+
        return lambda data, size: zstd.decompress(data)
    except ImportError:
        pass
    try:
        import pyzstd
        return lambda data, size: pyzstd.decompress(data)
    except ImportError:
        return None


def _lz4_decompressor():
    try:
        import lz4.frame
        return lambda data, size: lz4.frame.decompress(data)
    except ImportError:
        return None


_DECOMPRESSORS = {}


def decompressor(name):
    """Callable(data, uncompressed_size) or None if unsupported here."""
    name = (name or '').lower()
    if name == '':
        return lambda data, size: data
    if name not in _DECOMPRESSORS:
        _DECOMPRESSORS[name] = {'zstd': _zstd_decompressor,
                                'lz4': _lz4_decompressor}.get(
            name, lambda: None)()
    return _DECOMPRESSORS[name]


def _u16(b, o):
    return struct.unpack_from('<H', b, o)[0], o + 2


def _u32(b, o):
    return struct.unpack_from('<I', b, o)[0], o + 4


def _u64(b, o):
    return struct.unpack_from('<Q', b, o)[0], o + 8


def _mstr(b, o):
    n, o = _u32(b, o)
    return bytes(b[o:o + n]).decode('utf-8', 'replace'), o + n


def _parse_schema(b):
    sid, o = _u16(b, 0)
    name, o = _mstr(b, o)
    enc, o = _mstr(b, o)
    n, o = _u32(b, o)
    return sid, {'name': name, 'encoding': enc,
                 'data': bytes(b[o:o + n]).decode('utf-8', 'replace')}


def _parse_channel(b):
    cid, o = _u16(b, 0)
    sid, o = _u16(b, o)
    topic, o = _mstr(b, o)
    enc, o = _mstr(b, o)
    return cid, {'schema_id': sid, 'topic': topic, 'message_encoding': enc}


def _parse_message(b):
    cid, o = _u16(b, 0)
    _seq, o = _u32(b, o)
    log_time, o = _u64(b, o)
    pub_time, o = _u64(b, o)
    return cid, log_time, pub_time, b[o:]


def _iter_records(buf):
    o, n = 0, len(buf)
    while o + 9 <= n:
        op = buf[o]
        length = struct.unpack_from('<Q', buf, o + 1)[0]
        body = buf[o + 9:o + 9 + length]
        if len(body) < length:
            raise McapError('chunk record truncated')
        yield op, body
        o += 9 + length


def scan_mcap(path, on_message):
    """Stream one MCAP file.

    ``on_message(topic, schema, log_time, data)`` is called for every message;
    ``data`` is None when the message sits in a chunk this Python cannot
    decompress (its log_time then comes from the chunk's message index).
    Returns a summary dict.
    """
    schemas, channels = {}, {}
    summary = {'path': path, 'footer': False, 'chunk_compression': set(),
               'unreadable_chunks': 0, 'messages': 0, 'index_only': 0,
               'statistics': None, 'error': None}
    pending = []          # (channel_id, log_time) from unreadable chunks
    last_chunk_readable = True

    def _emit(cid, log_time, data):
        ch = channels.get(cid)
        if ch is None:
            pending.append((cid, log_time))  # channel defined later (summary)
            return
        on_message(ch['topic'], schemas.get(ch['schema_id'], {}), log_time,
                   data)
        summary['messages'] += 1

    def _handle(op, body, in_chunk):
        nonlocal last_chunk_readable
        if op == OP_SCHEMA:
            sid, sch = _parse_schema(body)
            schemas.setdefault(sid, sch)
        elif op == OP_CHANNEL:
            cid, ch = _parse_channel(body)
            channels.setdefault(cid, ch)
        elif op == OP_MESSAGE:
            cid, log_time, _pub, data = _parse_message(body)
            _emit(cid, log_time, bytes(data))
        elif op == OP_CHUNK and not in_chunk:
            _s, o = _u64(body, 0)
            _e, o = _u64(body, o)
            usize, o = _u64(body, o)
            _crc, o = _u32(body, o)
            comp, o = _mstr(body, o)
            rlen, o = _u64(body, o)
            summary['chunk_compression'].add(comp or 'none')
            fn = decompressor(comp)
            if fn is None:
                summary['unreadable_chunks'] += 1
                last_chunk_readable = False
                return
            last_chunk_readable = True
            records = fn(bytes(body[o:o + rlen]), usize)
            for rop, rbody in _iter_records(memoryview(records)):
                _handle(rop, rbody, True)
        elif op == OP_MESSAGE_INDEX and not last_chunk_readable:
            cid, o = _u16(body, 0)
            nbytes, o = _u32(body, o)
            end = o + nbytes
            while o + 16 <= end:
                log_time, o = _u64(body, o)
                _off, o = _u64(body, o)
                pending.append((cid, log_time))
        elif op == OP_STATISTICS:
            count, o = _u64(body, 0)
            summary['statistics'] = {'message_count': count}

    try:
        with open(path, 'rb') as f:
            if f.read(8) != MCAP_MAGIC:
                raise McapError('不是 MCAP 檔（magic 不符）')
            while True:
                hdr = f.read(9)
                if not hdr:
                    raise McapError('檔案結尾沒有 footer（錄製未正常結束？）')
                if len(hdr) < 9:
                    raise McapError('record header 被截斷')
                op = hdr[0]
                length = struct.unpack_from('<Q', hdr, 1)[0]
                body = f.read(length)
                if len(body) < length:
                    raise McapError('record 被截斷（錄製未正常結束？）')
                if op == OP_FOOTER:
                    summary['footer'] = True
                    break
                _handle(op, memoryview(body), False)
    except (McapError, struct.error, OSError) as e:
        summary['error'] = str(e)
    for cid, log_time in pending:
        ch = channels.get(cid)
        if ch is None:
            continue
        on_message(ch['topic'], schemas.get(ch['schema_id'], {}), log_time, None)
        summary['index_only'] += 1
    summary['chunk_compression'] = sorted(summary['chunk_compression'])
    summary['channels'] = {c['topic']: schemas.get(c['schema_id'], {}).get('name', '')
                           for c in channels.values()}
    return summary


# ── CDR (XCDR1) decoding of the few types we inspect ─────────────────────────
class Cdr:
    """Little-endian XCDR1 (what rmw_cyclonedds / rmw_fastrtps write)."""

    __slots__ = ('buf', 'pos')

    def __init__(self, data):
        if len(data) < 4:
            raise ValueError('CDR payload too short')
        if data[1] not in (0x01, 0x03):
            raise ValueError(f'unsupported CDR encapsulation 0x{data[1]:02x}')
        self.buf = data
        self.pos = 4

    def _align(self, n):
        rem = (self.pos - 4) % n
        if rem:
            self.pos += n - rem

    def _take(self, fmt, size):
        self._align(size)
        v = struct.unpack_from('<' + fmt, self.buf, self.pos)[0]
        self.pos += size
        return v

    def u32(self):
        return self._take('I', 4)

    def i32(self):
        return self._take('i', 4)

    def f64s(self, n):
        self._align(8)
        v = struct.unpack_from(f'<{n}d', self.buf, self.pos)
        self.pos += 8 * n
        return list(v)

    def string(self):
        n = self.u32()
        s = bytes(self.buf[self.pos:self.pos + n])
        self.pos += n
        return s.split(b'\0', 1)[0].decode('utf-8', 'replace')

    def u8seq(self):
        n = self.u32()
        v = self.buf[self.pos:self.pos + n]
        if len(v) < n:
            raise ValueError('uint8[] truncated')
        self.pos += n
        return v

    def header(self):
        sec = self.i32()
        nsec = self.u32()
        return sec * 1_000_000_000 + nsec, self.string()


def decode_header(data):
    return Cdr(data).header()


def decode_compressed_image(data):
    c = Cdr(data)
    stamp, frame = c.header()
    fmt = c.string()
    return stamp, frame, fmt, c.u8seq()


def decode_camera_info(data):
    c = Cdr(data)
    stamp, frame = c.header()
    height, width = c.u32(), c.u32()
    model = c.string()
    d = c.f64s(c.u32())
    k, r, p = c.f64s(9), c.f64s(9), c.f64s(12)
    return {'stamp': stamp, 'frame_id': frame, 'height': height,
            'width': width, 'distortion_model': model, 'd': d, 'k': k,
            'r': r, 'p': p}


def decode_tf_message(data):
    c = Cdr(data)
    out = []
    for _ in range(c.u32()):
        stamp, parent = c.header()
        child = c.string()
        t = c.f64s(3)
        q = c.f64s(4)
        out.append({'stamp': stamp, 'parent': parent.lstrip('/'),
                    'child': child.lstrip('/'), 't': t, 'q': q})
    return out


_HEADER_LINE = re.compile(r'^(std_msgs/(msg/)?)?Header\s+\w+$')


def schema_has_header(schema):
    text = schema.get('data') or ''
    if not text:
        return schema.get('name') in HEADER_TYPES
    for line in text.splitlines():
        s = line.split('#', 1)[0].strip()
        if not s:
            continue
        if s.startswith('==='):
            break
        if '=' in s:  # constant
            continue
        return bool(_HEADER_LINE.match(s))
    return False


# ── JPEG ─────────────────────────────────────────────────────────────────────
_SOF = {0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD,
        0xCE, 0xCF}


def jpeg_info(data):
    """{'ok', 'width', 'height', 'dht', 'eoi', 'error'} from the JPEG headers."""
    info = {'ok': False, 'width': None, 'height': None, 'dht': False,
            'eoi': False, 'error': None}
    n = len(data)
    if n < 4 or data[0] != 0xFF or data[1] != 0xD8:
        info['error'] = 'no SOI'
        return info
    tail = bytes(data[-64:]).rstrip(b'\x00')
    info['eoi'] = tail.endswith(b'\xff\xd9')
    i = 2
    while i + 4 <= n:
        if data[i] != 0xFF:
            info['error'] = f'bad marker at {i}'
            return info
        while i < n and data[i] == 0xFF:
            i += 1
        if i + 2 >= n:
            info['error'] = 'truncated header'
            return info
        marker = data[i]
        i += 1
        if marker == 0xD8 or marker == 0x01 or 0xD0 <= marker <= 0xD7:
            continue
        if marker == 0xD9:
            break
        seglen = (data[i] << 8) | data[i + 1]
        if seglen < 2:
            info['error'] = f'bad segment length at {i}'
            return info
        if marker in _SOF and i + 7 <= n:
            info['height'] = (data[i + 3] << 8) | data[i + 4]
            info['width'] = (data[i + 5] << 8) | data[i + 6]
        elif marker == 0xC4:
            info['dht'] = True
        elif marker == 0xDA:
            break
        i += seglen
    info['ok'] = bool(info['width'] and info['height'] and info['eoi'])
    if not info['ok'] and not info['error']:
        info['error'] = 'no EOI' if info['width'] else 'no SOF'
    return info


# ── rigid transforms (for the TF chain) ──────────────────────────────────────
def _qmat(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def _compose(a, b):
    ra, ta = a
    rb, tb = b
    r = [[sum(ra[i][k] * rb[k][j] for k in range(3)) for j in range(3)]
         for i in range(3)]
    t = [sum(ra[i][k] * tb[k] for k in range(3)) + ta[i] for i in range(3)]
    return r, t


def _rot_angle_deg(ra, rb):
    tr = sum(ra[k][i] * rb[k][i] for i in range(3) for k in range(3))
    return math.degrees(math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0))))


def _rpy_deg(r):
    pitch = math.asin(max(-1.0, min(1.0, -r[2][0])))
    roll = math.atan2(r[2][1], r[2][2])
    yaw = math.atan2(r[1][0], r[0][0])
    return [round(math.degrees(a), 3) for a in (roll, pitch, yaw)]


# ── report ───────────────────────────────────────────────────────────────────
class Report:
    def __init__(self):
        self.items = []

    def add(self, status, check, message, **data):
        item = {'status': status, 'check': check, 'message': message}
        if data:
            item['data'] = data
        self.items.append(item)

    def count(self, status):
        return sum(1 for i in self.items if i['status'] == status)

    @property
    def result(self):
        if self.count('FAIL'):
            return 'FAIL'
        return 'WARN' if self.count('WARN') else 'PASS'


class TopicData:
    __slots__ = ('type', 'has_header', 'log_times', 'stamps', 'frames',
                 'undecoded', 'decode_errors', 'msg_compressed', 'bytes')

    def __init__(self, type_name, has_header):
        self.type = type_name
        self.has_header = has_header
        self.log_times = []
        self.stamps = []
        self.frames = set()
        self.undecoded = 0
        self.decode_errors = 0
        self.msg_compressed = 0
        self.bytes = 0


class RunScan:
    """Everything check_run needs, collected in one pass over the bags."""

    def __init__(self):
        self.topics = {}
        self.images = {'count': 0, 'bad': 0, 'no_dht': 0, 'formats': set(),
                       'sizes': {}, 'bytes': 0, 'stamps': [], 'first_bad': None}
        self.infos = []
        self.tf_static = []
        self.tf_pairs = {}
        self.parents = {}
        self.zstd = decompressor('zstd')

    def on_message(self, topic, schema, log_time, data):
        td = self.topics.get(topic)
        if td is None:
            td = self.topics[topic] = TopicData(schema.get('name', ''),
                                                schema_has_header(schema))
        td.log_times.append(log_time)
        if data is None:
            td.undecoded += 1
            return
        td.bytes += len(data)
        if data[:4] == ZSTD_MAGIC:  # rosbag2 per-message compression (P-089)
            td.msg_compressed += 1
            if self.zstd is None:
                td.undecoded += 1
                return
            data = self.zstd(bytes(data), 0)
        try:
            self._decode(topic, td, log_time, data)
        except (ValueError, struct.error, IndexError):
            td.decode_errors += 1

    def _decode(self, topic, td, log_time, data):
        if topic == CAMERA_IMAGE:
            stamp, frame, fmt, img = decode_compressed_image(data)
            td.stamps.append((stamp, log_time))
            td.frames.add(frame)
            im = self.images
            im['count'] += 1
            im['bytes'] += len(img)
            im['formats'].add(fmt)
            im['stamps'].append(stamp)
            j = jpeg_info(img)
            if not j['ok']:
                im['bad'] += 1
                im['first_bad'] = im['first_bad'] or j['error']
            if not j['dht']:
                im['no_dht'] += 1
            if j['width']:
                key = (j['width'], j['height'])
                im['sizes'][key] = im['sizes'].get(key, 0) + 1
        elif topic == CAMERA_INFO:
            ci = decode_camera_info(data)
            td.stamps.append((ci['stamp'], log_time))
            td.frames.add(ci['frame_id'])
            self.infos.append(ci)
        elif td.type == 'tf2_msgs/msg/TFMessage':
            for tr in decode_tf_message(data):
                self.parents.setdefault(tr['child'], set()).add(tr['parent'])
                if topic == '/tf_static':
                    self.tf_static.append(tr)
                else:
                    self.tf_pairs.setdefault((tr['parent'], tr['child']),
                                             []).append(log_time)
        elif td.has_header:
            stamp, frame = decode_header(data)
            td.stamps.append((stamp, log_time))
            td.frames.add(frame)


# ── helpers ──────────────────────────────────────────────────────────────────
def _load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def _get(d, dotted):
    cur = d
    for part in dotted.split('.'):
        if not isinstance(cur, dict) or part not in cur:
            return None
        cur = cur[part]
    return cur


def _rate(log_times):
    if len(log_times) < 2:
        return None, None
    ts = sorted(log_times)
    span = (ts[-1] - ts[0]) / 1e9
    if span <= 0:
        return None, None
    gaps = [(b - a) / 1e9 for a, b in zip(ts, ts[1:])]
    return (len(ts) - 1) / span, max(gaps)


def find_bags(run_dir):
    """Relative dirs holding rosbag2 output (metadata.yaml or *.mcap)."""
    found = set()
    for dirpath, dirnames, files in os.walk(run_dir):
        dirnames[:] = [d for d in dirnames if not d.startswith('.')]
        if 'metadata.yaml' in files or any(f.endswith('.mcap') for f in files):
            found.add(os.path.relpath(dirpath, run_dir).replace(os.sep, '/'))
    return sorted(found)


def _bag_files(run_dir, rel, meta):
    base = os.path.join(run_dir, rel)
    if meta and meta.get('relative_file_paths'):
        return [os.path.join(base, p) for p in meta['relative_file_paths']]
    return sorted(os.path.join(base, f) for f in os.listdir(base)
                  if f.endswith('.mcap'))


def _read_bag_meta(path):
    info = (_load_yaml(path) or {}).get('rosbag2_bagfile_information') or {}
    topics = {}
    for entry in info.get('topics_with_message_count') or []:
        md = entry.get('topic_metadata') or {}
        if md.get('name'):
            topics[md['name']] = {'type': md.get('type', ''),
                                  'count': int(entry.get('message_count') or 0)}
    return {'storage_identifier': info.get('storage_identifier', ''),
            'topics': topics,
            'relative_file_paths': list(info.get('relative_file_paths') or []),
            'compression_mode': str(info.get('compression_mode') or ''),
            'compression_format': str(info.get('compression_format') or '')}


# ── the checks ───────────────────────────────────────────────────────────────
class Options:
    def __init__(self, **kw):
        self.profile = kw.get('profile', 'auto')
        self.allow_uncalibrated = kw.get('allow_uncalibrated', False)
        self.allow_placeholder = kw.get('allow_placeholder', False)
        self.verify_sha = kw.get('verify_sha', True)
        self.camera_hz = kw.get('camera_hz')


def check_run(run_dir, options=None):
    opt = options or Options()
    rep = Report()
    run_dir = os.path.abspath(run_dir)
    if not os.path.isdir(run_dir):
        rep.add('FAIL', 'run.dir', f'找不到目錄：{run_dir}')
        return rep, {}

    meta = {}
    meta_path = os.path.join(run_dir, 'run_metadata.yaml')
    if os.path.isfile(meta_path):
        try:
            meta = _load_yaml(meta_path) or {}
            rep.add('PASS', 'run.metadata', 'run_metadata.yaml 可讀')
        except yaml.YAMLError as e:
            rep.add('FAIL', 'run.metadata', f'run_metadata.yaml 無法解析：{e}')
    else:
        rep.add('WARN', 'run.metadata', '沒有 run_metadata.yaml')
    profile = opt.profile if opt.profile != 'auto' else \
        (meta.get('profile') or 'default')
    dc = profile == 'data_collection'
    ctx = {'run_dir': run_dir, 'profile': profile, 'meta': meta}

    scan = RunScan()
    _check_bags(run_dir, rep, scan, dc, ctx)
    if dc:
        _check_topics(rep, scan, meta, opt)
        _check_camera(run_dir, rep, scan, meta, opt)
        _check_tf(run_dir, rep, scan, meta)
        _check_calib(run_dir, rep, meta, opt)
        _check_metadata_fields(rep, meta)
    _check_stamps(rep, scan)
    _check_manifest(run_dir, rep, opt)
    ctx['topics'] = {t: len(d.log_times) for t, d in sorted(scan.topics.items())}
    return rep, ctx


def _check_bags(run_dir, rep, scan, dc, ctx):
    bags = find_bags(run_dir)
    ctx['bags'] = bags
    if not bags:
        rep.add('FAIL', 'bag.present', '找不到任何 rosbag（metadata.yaml／*.mcap）')
        return
    for rel in bags:
        meta_path = os.path.join(run_dir, rel, 'metadata.yaml')
        bmeta = None
        if not os.path.isfile(meta_path):
            rep.add('FAIL', f'bag.metadata[{rel}]',
                    f'{rel}/ 沒有 metadata.yaml（錄製沒有正常結束；'
                    f'`ros2 bag reindex {rel} -s mcap` 可重建）')
        else:
            try:
                bmeta = _read_bag_meta(meta_path)
            except Exception as e:  # noqa: BLE001
                rep.add('FAIL', f'bag.metadata[{rel}]', f'metadata.yaml 無法解析：{e}')
        if bmeta is not None:
            mode = bmeta['compression_mode'].lower()
            if mode in ('message', 'file'):
                rep.add('FAIL' if dc else 'WARN', f'bag.compression[{rel}]',
                        f'rosbag2 {mode} 壓縮（{bmeta["compression_format"]}）：'
                        'Foxglove／python mcap 讀不到 payload，改用 '
                        '--storage-preset-profile zstd_fast（P-089）')
            if bmeta['storage_identifier'] not in ('mcap', ''):
                rep.add('WARN', f'bag.storage[{rel}]',
                        f'storage={bmeta["storage_identifier"]}，預期 mcap')
        files = _bag_files(run_dir, rel, bmeta)
        missing = [f for f in files if not os.path.isfile(f)]
        if missing:
            rep.add('FAIL', f'bag.files[{rel}]', '缺檔：' + ', '.join(
                os.path.basename(m) for m in missing))
        before = {t: len(d.log_times) for t, d in scan.topics.items()}
        comp, problems, unreadable = set(), [], 0
        for f in files:
            if not os.path.isfile(f) or not f.endswith('.mcap'):
                continue
            s = scan_mcap(f, scan.on_message)
            comp.update(s['chunk_compression'])
            unreadable += s['unreadable_chunks']
            if s['error']:
                problems.append(f'{os.path.basename(f)}：{s["error"]}')
        if problems:
            rep.add('FAIL', f'bag.mcap[{rel}]', '；'.join(problems))
        else:
            rep.add('PASS', f'bag.mcap[{rel}]',
                    f'{len(files)} 個 mcap 完整（chunk 壓縮：'
                    f'{",".join(sorted(comp)) or "無 chunk"}）')
        if unreadable:
            rep.add('WARN', f'bag.decode[{rel}]',
                    f'{unreadable} 個 chunk 無法解壓（缺 zstandard／lz4）：'
                    '只檢查訊息數與接收頻率，stamp 與 TF 內容略過')
        if dc and rel == 'camera' and 'zstd' in comp:
            rep.add('WARN', 'bag.camera_compression',
                    'camera/ 用了 zstd：JPEG 再壓縮沒有效益，只會吃 RK3568 CPU')
        if dc and rel == 'bag' and comp == {'none'}:
            rep.add('WARN', 'bag.telemetry_compression',
                    'bag/ 沒有 chunk 壓縮；data_collection 預期 zstd_fast')
        if bmeta is not None:
            after = {t: len(d.log_times) for t, d in scan.topics.items()}
            bad = []
            for name, t in bmeta['topics'].items():
                got = after.get(name, 0) - before.get(name, 0)
                if got != t['count']:
                    bad.append(f'{name} metadata={t["count"]} mcap={got}')
            if bad:
                rep.add('FAIL', f'bag.counts[{rel}]',
                        '訊息數與 metadata.yaml 不符：' + '；'.join(bad[:5]))
            else:
                rep.add('PASS', f'bag.counts[{rel}]',
                        f'{len(bmeta["topics"])} 個話題的訊息數與 metadata.yaml 一致')


def _expected_camera_hz(meta, opt):
    if opt.camera_hz:
        return float(opt.camera_hz)
    v = _get(meta, 'camera.fps_recorded')
    return float(v) if isinstance(v, (int, float)) and v > 0 else DEFAULT_CAMERA_HZ


def _check_topics(rep, scan, meta, opt):
    cam_hz = _expected_camera_hz(meta, opt)
    for topic, exp in REQUIRED_TOPICS.items():
        td = scan.topics.get(topic)
        cid = f'topic[{topic}]'
        if td is None or not td.log_times:
            rep.add('FAIL', cid, '必要話題沒有錄到')
            continue
        if exp['type'] and td.type and td.type != exp['type']:
            rep.add('FAIL', cid, f'型別 {td.type}，預期 {exp["type"]}')
            continue
        n = len(td.log_times)
        if exp.get('camera'):
            limits = (0.5 * cam_hz, 0.8 * cam_hz, 1.2 * cam_hz, 2.0 * cam_hz)
        elif exp.get('hz'):
            limits = (exp['hz'][0], exp['hz'][1], None, None)
        else:
            rep.add('PASS', cid, f'{n} 則')
            continue
        hz, max_gap = _rate(td.log_times)
        if hz is None:
            rep.add('FAIL', cid, f'只有 {n} 則，無法計算頻率')
            continue
        fail_lo, warn_lo, warn_hi, fail_hi = limits
        text = f'{n} 則，{hz:.2f} Hz，最大間隔 {max_gap:.2f} s'
        if hz < fail_lo or (fail_hi and hz > fail_hi):
            rep.add('FAIL', cid, text + f'（預期約 {warn_lo:g} Hz 以上）', hz=hz)
        elif hz < warn_lo or (warn_hi and hz > warn_hi):
            rep.add('WARN', cid, text + '（偏離預期）', hz=hz)
        else:
            rep.add('PASS', cid, text, hz=hz)
        if exp.get('camera') and max_gap > 5.0 / cam_hz:
            rep.add('WARN', cid + '.gap', f'影像最大間隔 {max_gap:.2f} s（掉幀？）')
    for topic in RECOMMENDED_TOPICS:
        td = scan.topics.get(topic)
        if td is None or not td.log_times:
            rep.add('WARN', f'topic[{topic}]', '建議話題沒有錄到（規格 1.2）')
        else:
            rep.add('PASS', f'topic[{topic}]', f'{len(td.log_times)} 則')


def _check_camera(run_dir, rep, scan, meta, opt):
    im = scan.images
    if not im['count']:
        if scan.topics.get(CAMERA_IMAGE):
            rep.add('SKIP', 'camera.jpeg', '影像無法解碼，略過影像檢查')
        return
    if im['formats'] - {'jpeg'}:
        rep.add('FAIL', 'camera.format',
                f'format={sorted(im["formats"])}，規格要求 jpeg（直接轉送 MJPEG）')
    if im['bad']:
        rep.add('FAIL', 'camera.jpeg',
                f'{im["bad"]}/{im["count"]} 張 JPEG 不完整（{im["first_bad"]}）')
    else:
        rep.add('PASS', 'camera.jpeg', f'{im["count"]} 張 JPEG 皆有 SOI／SOF／EOI')
    if im['no_dht']:
        rep.add('WARN', 'camera.dht',
                f'{im["no_dht"]} 張缺 Huffman 表（MJPEG 常見；libjpeg-turbo／'
                'ffmpeg 可解，其他解碼器要先補 DHT）')
    sizes = im['sizes']
    if len(sizes) > 1:
        rep.add('FAIL', 'camera.size', f'影像尺寸不一致：{sizes}')
    img_size = next(iter(sizes)) if len(sizes) == 1 else None
    want = (_get(meta, 'camera.width'), _get(meta, 'camera.height'))
    if img_size and all(want) and tuple(want) != img_size:
        rep.add('FAIL', 'camera.size_vs_metadata',
                f'影像 {img_size[0]}x{img_size[1]}，run_metadata 寫 {want[0]}x{want[1]}')
    kb = im['bytes'] / im['count'] / 1024.0
    span = _rate(scan.topics[CAMERA_IMAGE].log_times)
    rep.add('INFO', 'camera.throughput',
            f'平均每張 {kb:.0f} KB'
            + (f'，約 {kb * span[0] / 1024.0:.2f} MB/s' if span[0] else ''))

    infos = scan.infos
    if not infos:
        rep.add('FAIL', 'camera.info', 'camera_info 無法解碼或沒有錄到')
        return
    dims = {(c['width'], c['height']) for c in infos}
    if len(dims) > 1:
        rep.add('FAIL', 'camera.info_size', f'camera_info 尺寸不一致：{dims}')
    ci = infos[-1]
    if img_size and (ci['width'], ci['height']) != img_size:
        rep.add('FAIL', 'camera.info_size',
                f'camera_info {ci["width"]}x{ci["height"]} ≠ 影像 '
                f'{img_size[0]}x{img_size[1]}（P-081）')
    elif img_size:
        rep.add('PASS', 'camera.info_size',
                f'camera_info 與影像同為 {img_size[0]}x{img_size[1]}')
    k = ci['k']
    calibrated = k[0] > 0 and k[4] > 0 and k[8] == 1.0
    if not calibrated:
        rep.add('WARN' if opt.allow_uncalibrated else 'FAIL', 'camera.intrinsics',
                'camera_info 的 K 未標定（全 0）：先做棋盤格標定'
                + ('（--allow-uncalibrated）' if opt.allow_uncalibrated else ''))
    else:
        w, h = ci['width'] or 1, ci['height'] or 1
        cx_off = abs(k[2] - w / 2.0) / w
        cy_off = abs(k[5] - h / 2.0) / h
        lvl = 'WARN' if cx_off > 0.15 or cy_off > 0.15 else 'PASS'
        rep.add(lvl, 'camera.intrinsics',
                f'fx={k[0]:.1f} fy={k[4]:.1f} cx={k[2]:.1f} cy={k[5]:.1f}，'
                f'{ci["distortion_model"]} D={len(ci["d"])}')
        if ci['distortion_model'] != 'plumb_bob' or len(ci['d']) != 5:
            rep.add('WARN', 'camera.distortion',
                    f'distortion_model={ci["distortion_model"]} D={len(ci["d"])}，'
                    '規格為 plumb_bob（5 個係數）')
    _compare_calib_file(run_dir, rep, ci)

    img_frames = scan.topics[CAMERA_IMAGE].frames
    info_frames = scan.topics[CAMERA_INFO].frames if CAMERA_INFO in scan.topics \
        else set()
    frames = img_frames | info_frames
    if frames != {OPTICAL_FRAME}:
        rep.add('FAIL', 'camera.frame_id', f'frame_id={sorted(frames)}，'
                f'規格為 {OPTICAL_FRAME}')
    else:
        rep.add('PASS', 'camera.frame_id', f'frame_id={OPTICAL_FRAME}')

    img_st = [s for s, _ in scan.topics[CAMERA_IMAGE].stamps]
    info_st = [c['stamp'] for c in infos]
    _check_pairing(rep, img_st, info_st)

    lat = [(lt - st) / 1e9 for st, lt in scan.topics[CAMERA_IMAGE].stamps]
    if lat:
        med = statistics.median(lat)
        p95 = sorted(lat)[int(0.95 * (len(lat) - 1))]
        text = f'接收時間 − stamp：中位 {med * 1000:.1f} ms，p95 {p95 * 1000:.1f} ms'
        if med < -0.05:
            rep.add('FAIL', 'camera.latency', text + '：stamp 比接收時間晚，'
                    '時鐘在相機啟動後被調整？重啟相機節點')
        elif med > 0.5:
            rep.add('WARN', 'camera.latency', text + '：延遲過大或時鐘偏移')
        elif med < 0.003:
            rep.add('WARN', 'camera.latency', text + '：stamp 像是接收時間而非擷取'
                    '時間（gscam 要 use_gst_timestamps:=true）')
        else:
            rep.add('PASS', 'camera.latency', text)


def _check_pairing(rep, img_st, info_st):
    si, sc = set(img_st), set(info_st)
    if not si:
        return
    lo, hi = min(si | sc), max(si | sc)
    edge = 2_000_000_000  # unmatched pairs within 2 s of start/end are OK
    only_img = [s for s in si - sc if lo + edge < s < hi - edge]
    only_info = [s for s in sc - si if lo + edge < s < hi - edge]
    edge_n = len(si ^ sc) - len(only_img) - len(only_info)
    if only_img or only_info:
        rep.add('FAIL', 'camera.pairing',
                f'影像與 camera_info 不是 1:1：只有影像 {len(only_img)}、'
                f'只有 camera_info {len(only_info)}（stamp 需完全相同）')
    else:
        rep.add('PASS', 'camera.pairing',
                f'{len(si & sc)} 組影像／camera_info stamp 1:1'
                + (f'（頭尾 {edge_n} 則未配對，可接受）' if edge_n else ''))


def _compare_calib_file(run_dir, rep, ci):
    path = os.path.join(run_dir, 'calib', 'camera_front.yaml')
    if not os.path.isfile(path):
        return
    try:
        cal = _load_yaml(path) or {}
        k = [float(v) for v in (_get(cal, 'camera_matrix.data') or [])]
        size = (cal.get('image_width'), cal.get('image_height'))
    except Exception as e:  # noqa: BLE001
        rep.add('WARN', 'calib.intrinsics_file', f'calib/camera_front.yaml 無法解析：{e}')
        return
    if len(k) == 9 and any(abs(a - b) > 1e-6 for a, b in zip(k, ci['k'])):
        rep.add('FAIL', 'calib.intrinsics_file',
                'camera_info 的 K 與 calib/camera_front.yaml 不同（錄到的不是這份標定）')
    elif size != (ci['width'], ci['height']):
        rep.add('FAIL', 'calib.intrinsics_file',
                f'calib/camera_front.yaml 尺寸 {size} 與 camera_info 不同')
    else:
        rep.add('PASS', 'calib.intrinsics_file', 'camera_info 與 calib/camera_front.yaml 一致')


def _check_tf(run_dir, rep, scan, meta):
    if '/tf_static' in scan.topics and not scan.tf_static \
            and scan.topics['/tf_static'].undecoded:
        rep.add('SKIP', 'tf_static.chain', '/tf_static 無法解碼（缺 zstandard？）')
    else:
        _check_tf_static(run_dir, rep, scan, meta)
    multi = {c: p for c, p in scan.parents.items() if len(p) > 1}
    if multi:
        rep.add('FAIL', 'tf.parents', 'frame 有多個 parent（TF authority 衝突，P-001／'
                'P-010）：' + '；'.join(f'{c}←{sorted(p)}' for c, p in multi.items()))
    if '/tf' in scan.topics and not scan.tf_pairs and scan.topics['/tf'].undecoded:
        rep.add('SKIP', 'tf.dynamic', '/tf 無法解碼（缺 zstandard？）')
        return
    for pair, min_hz in REQUIRED_TF_PAIRS.items():
        lts = scan.tf_pairs.get(pair)
        cid = f'tf[{pair[0]}->{pair[1]}]'
        if not lts:
            rep.add('FAIL', cid, '/tf 沒有這個轉換')
            continue
        hz, gap = _rate(lts)
        if hz is None or hz < min_hz:
            rep.add('WARN', cid, f'{len(lts)} 則，{(hz or 0):.1f} Hz（預期 ≥ {min_hz:g}）')
        else:
            rep.add('PASS', cid, f'{len(lts)} 則，{hz:.1f} Hz，最大間隔 {gap:.2f} s')


def _check_tf_static(run_dir, rep, scan, meta):
    latest = {}
    for tr in scan.tf_static:
        latest[tr['child']] = tr
    chain = []
    child = TF_CHAIN[-1]
    ok = True
    for want_parent in reversed(TF_CHAIN[:-1]):
        tr = latest.get(child)
        if tr is None or tr['parent'] != want_parent:
            got = tr['parent'] if tr else '（無）'
            rep.add('FAIL', 'tf_static.chain',
                    f'/tf_static 缺 {want_parent} → {child}（目前 parent：{got}）；'
                    '規格要求 ' + ' → '.join(TF_CHAIN))
            ok = False
            break
        chain.append(tr)
        child = want_parent
    if not ok:
        return
    t = ([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], [0.0, 0.0, 0.0])
    for tr in reversed(chain):
        t = _compose(t, (_qmat(tr['q']), tr['t']))
    r, xyz = t
    z_axis = [r[0][2], r[1][2], r[2][2]]
    down = math.degrees(math.asin(max(-1.0, min(1.0, -z_axis[2]))))
    text = (' → '.join(TF_CHAIN) + f'：xyz=({xyz[0]:.3f}, {xyz[1]:.3f}, '
            f'{xyz[2]:.3f}) m，rpy={_rpy_deg(r)}°，光軸俯角 {down:.1f}°')
    if z_axis[0] <= 0.3:
        rep.add('FAIL', 'tf_static.chain', text + '：光軸沒有朝前（optical frame 用反？P-009）')
    else:
        rep.add('PASS', 'tf_static.chain', text)
    pitch = meta.get('camera_pitch_deg')
    if isinstance(pitch, (int, float)) and abs(pitch - down) > 1.0:
        rep.add('WARN', 'tf_static.pitch',
                f'TF 俯角 {down:.1f}° 與 run_metadata camera_pitch_deg {pitch}° 不同')
    ext_path = os.path.join(run_dir, 'calib', 'extrinsics.yaml')
    if os.path.isfile(ext_path):
        try:
            ext = _load_yaml(ext_path) or {}
        except yaml.YAMLError:
            ext = {}
        d = _get(ext, 'derived.base_footprint_to_camera_front_optical_frame')
        if isinstance(d, dict) and d.get('xyz_m') and d.get('quaternion_xyzw'):
            dp = math.dist(d['xyz_m'], xyz)
            da = _rot_angle_deg(_qmat(d['quaternion_xyzw']), r)
            if dp > 0.005 or da > 0.5:
                rep.add('FAIL', 'tf_static.extrinsics',
                        f'/tf_static 與 calib/extrinsics.yaml 不一致（Δ{dp * 1000:.1f} mm，'
                        f'{da:.2f}°）')
            else:
                rep.add('PASS', 'tf_static.extrinsics', '/tf_static 與 calib/extrinsics.yaml 一致')


def _check_stamps(rep, scan):
    for topic in STAMP_TOPICS:
        td = scan.topics.get(topic)
        if td is None or not td.log_times:
            continue
        if not td.stamps:
            if td.undecoded:
                rep.add('SKIP', f'stamp[{topic}]', '無法解碼 header（缺 zstandard？）')
            continue
        st = [s for s, _ in sorted(td.stamps, key=lambda p: p[1])]
        back = sum(1 for a, b in zip(st, st[1:]) if b < a)
        dup = sum(1 for a, b in zip(st, st[1:]) if b == a)
        zero = sum(1 for s in st if s == 0)
        cid = f'stamp[{topic}]'
        strict = topic in (CAMERA_IMAGE, CAMERA_INFO)
        if zero:
            rep.add('FAIL', cid, f'{zero} 則 header.stamp = 0')
        elif back:
            rep.add('FAIL', cid, f'header.stamp 倒退 {back} 次（單調遞增失敗）')
        elif dup and strict:
            rep.add('FAIL', cid, f'{dup} 則重複 stamp（重複影格？）')
        elif dup:
            rep.add('WARN', cid, f'{dup} 則重複 stamp')
        else:
            rep.add('PASS', cid, f'{len(st)} 則 header.stamp 嚴格遞增')


def _check_calib(run_dir, rep, meta, opt):
    for rel in ('calib/camera_front.yaml', 'calib/extrinsics.yaml'):
        if not os.path.isfile(os.path.join(run_dir, rel)):
            rep.add('FAIL', 'calib.files', f'缺少 {rel}（規格 1.6）')
    ext_path = os.path.join(run_dir, 'calib', 'extrinsics.yaml')
    if os.path.isfile(ext_path):
        try:
            source = str((_load_yaml(ext_path) or {}).get('source', ''))
        except yaml.YAMLError:
            source = ''
        if source == 'measured' or source == 'cad':
            rep.add('PASS', 'calib.extrinsics_source', f'外參來源：{source}')
        elif source == 'placeholder':
            rep.add('WARN' if opt.allow_placeholder else 'FAIL',
                    'calib.extrinsics_source',
                    '外參仍是佔位值（source: placeholder），必須實測後覆寫')
        else:
            rep.add('FAIL', 'calib.extrinsics_source', f'外參 source={source!r} 無法辨識')
    src = meta.get('calibration', {}).get('intrinsics_source') \
        if isinstance(meta.get('calibration'), dict) else None
    if src == 'placeholder':
        rep.add('WARN' if opt.allow_uncalibrated else 'FAIL',
                'calib.intrinsics_source', '內參是自動產生的未標定佔位檔')


def _check_metadata_fields(rep, meta):
    missing, placeholders = [], []
    for field in METADATA_FIELDS:
        v = _get(meta, field)
        if v is None:
            missing.append(field)
        elif isinstance(v, str) and v.startswith('<') and v.endswith('>'):
            placeholders.append(field)
    for k in ('blade', 'camera.autofocus'):
        if isinstance(_get(meta, k), bool):
            placeholders.append(f'{k}（布林值：YAML 1.1 的 off/on，請加引號，P-099）')
    if missing or placeholders:
        rep.add('WARN', 'metadata.fields', '；'.join(
            ([f'缺欄位：{", ".join(missing)}'] if missing else [])
            + ([f'待填：{", ".join(placeholders)}'] if placeholders else [])))
    else:
        rep.add('PASS', 'metadata.fields', '規格 1.6 欄位齊全')


def _check_manifest(run_dir, rep, opt):
    path = os.path.join(run_dir, '_manifest.json')
    if not os.path.isfile(path):
        return
    try:
        with open(path) as f:
            man = json.load(f)
    except (OSError, ValueError) as e:
        rep.add('FAIL', 'manifest', f'_manifest.json 無法解析：{e}')
        return
    run_id = man.get('run_id') or os.path.basename(run_dir)
    listed, bad = set(), []
    for entry in man.get('files', []):
        key = entry.get('key', '')
        rel = key.split(f'/{run_id}/', 1)[1] if f'/{run_id}/' in key else key
        listed.add(rel)
        local = os.path.join(run_dir, rel)
        if not os.path.isfile(local):
            bad.append(f'缺 {rel}')
            continue
        if os.path.getsize(local) != entry.get('size'):
            bad.append(f'{rel} 大小不符')
            continue
        if opt.verify_sha:
            h = hashlib.sha256()
            with open(local, 'rb') as f:
                for chunk in iter(lambda: f.read(4 * 1024 * 1024), b''):
                    h.update(chunk)
            if h.hexdigest() != entry.get('sha256'):
                bad.append(f'{rel} sha256 不符')
    if bad:
        rep.add('FAIL', 'manifest', '_manifest.json 與檔案不符：' + '；'.join(bad[:8]))
    else:
        rep.add('PASS', 'manifest', f'_manifest.json 列出的 {len(listed)} 個檔案'
                + ('大小與 sha256 相符' if opt.verify_sha else '大小相符'))
    extra = []
    for dirpath, dirnames, files in os.walk(run_dir):
        dirnames[:] = [d for d in dirnames if not d.startswith('.')]
        for fn in files:
            rel = os.path.relpath(os.path.join(dirpath, fn), run_dir).replace(os.sep, '/')
            if not fn.startswith('.') and rel != '_manifest.json' and rel not in listed:
                extra.append(rel)
    if extra:
        rep.add('WARN', 'manifest.extra', '不在 manifest 的檔案：' + ', '.join(extra[:8]))


# ── CLI ──────────────────────────────────────────────────────────────────────
_LABEL = {'PASS': '通過', 'WARN': '警告', 'FAIL': '失敗', 'SKIP': '略過', 'INFO': '資訊'}


def format_report(rep, ctx):
    lines = [f'== mower-check-run：{ctx.get("run_dir", "")}',
             f'   profile={ctx.get("profile")}  bags={", ".join(ctx.get("bags", []))}']
    for item in rep.items:
        lines.append(f'{item["status"]:<4}  {item["check"]:<34} {item["message"]}')
    counts = '、'.join(f'{_LABEL[s]} {rep.count(s)}'
                      for s in ('PASS', 'WARN', 'FAIL', 'SKIP') if rep.count(s))
    lines.append(f'結果：{rep.result}（{counts}）')
    return '\n'.join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog='mower-check-run',
        description='檢查 mower_recorder 的 run 目錄（規格 1.1～1.6、2.1）')
    ap.add_argument('run_dir')
    ap.add_argument('--profile', default='auto',
                    help='auto（讀 run_metadata.yaml）、data_collection 或 default')
    ap.add_argument('--camera-hz', type=float, default=None,
                    help='預期影像頻率（預設取 camera.fps_recorded，否則 10）')
    ap.add_argument('--allow-uncalibrated', action='store_true',
                    help='內參未標定只算 WARN（錄棋盤格的 run 用）')
    ap.add_argument('--allow-placeholder-extrinsics', action='store_true',
                    help='外參佔位值只算 WARN')
    ap.add_argument('--no-sha', action='store_true',
                    help='驗 _manifest.json 時只比大小，不算 sha256')
    ap.add_argument('--json', metavar='PATH',
                    help='另存 JSON 結果（- 表示輸出到 stdout）')
    ap.add_argument('--strict', action='store_true', help='WARN 也回傳失敗')
    args = ap.parse_args(argv)

    opt = Options(profile=args.profile, allow_uncalibrated=args.allow_uncalibrated,
                  allow_placeholder=args.allow_placeholder_extrinsics,
                  verify_sha=not args.no_sha, camera_hz=args.camera_hz)
    rep, ctx = check_run(args.run_dir, opt)
    doc = {'run_dir': ctx.get('run_dir', args.run_dir), 'profile': ctx.get('profile'),
           'result': rep.result,
           'counts': {s: rep.count(s) for s in ('PASS', 'WARN', 'FAIL', 'SKIP', 'INFO')},
           'topics': ctx.get('topics', {}), 'items': rep.items}
    if args.json == '-':
        print(json.dumps(doc, ensure_ascii=False, indent=2))
    else:
        print(format_report(rep, ctx))
        if args.json:
            with open(args.json, 'w') as f:
                json.dump(doc, f, ensure_ascii=False, indent=2)
    if rep.result == 'FAIL' or (args.strict and rep.result == 'WARN'):
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
