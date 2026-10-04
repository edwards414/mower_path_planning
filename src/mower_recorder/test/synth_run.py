"""Synthetic data_collection runs for the pure-Python tests.

Bags are written with the official ``mcap`` writer and encoded with
``mcap_ros2``'s dynamic CDR serializer, i.e. independently of the decoder in
mower_recorder.check_run. The layout mimics rosbag2 (Jazzy, metadata v9):
``bag/bag_0.mcap`` (zstd chunks) and ``camera/camera_0.mcap`` (no chunk
compression) plus ``metadata.yaml`` next to each.
"""
import io
import json
import os

import yaml
from mcap.writer import CompressionType, Writer
from mcap_ros2._dynamic import serialize_dynamic

from mower_recorder import camera_extrinsics as ce

SEP = '=' * 80

TIME = 'MSG: builtin_interfaces/Time\nint32 sec\nuint32 nanosec'
HEADER = 'MSG: std_msgs/Header\nbuiltin_interfaces/Time stamp\nstring frame_id'


def _def(body, *deps):
    return '\n'.join([body] + [f'{SEP}\n{d}' for d in deps])


MSGDEFS = {
    'sensor_msgs/msg/CompressedImage': _def(
        '# This message contains a compressed image.\n'
        'std_msgs/Header header\nstring format\nuint8[] data', HEADER, TIME),
    'sensor_msgs/msg/CameraInfo': _def(
        'std_msgs/Header header\nuint32 height\nuint32 width\n'
        'string distortion_model\nfloat64[] d\nfloat64[9] k\nfloat64[9] r\n'
        'float64[12] p\nuint32 binning_x\nuint32 binning_y\n'
        'sensor_msgs/RegionOfInterest roi', HEADER, TIME,
        'MSG: sensor_msgs/RegionOfInterest\nuint32 x_offset\nuint32 y_offset\n'
        'uint32 height\nuint32 width\nbool do_rectify'),
    'tf2_msgs/msg/TFMessage': _def(
        'geometry_msgs/TransformStamped[] transforms',
        'MSG: geometry_msgs/TransformStamped\nstd_msgs/Header header\n'
        'string child_frame_id\ngeometry_msgs/Transform transform',
        'MSG: geometry_msgs/Transform\ngeometry_msgs/Vector3 translation\n'
        'geometry_msgs/Quaternion rotation',
        'MSG: geometry_msgs/Vector3\nfloat64 x\nfloat64 y\nfloat64 z',
        'MSG: geometry_msgs/Quaternion\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 w',
        HEADER, TIME),
    # Header-first types: only the header matters to check_run.
    'sensor_msgs/msg/Imu': _def(
        '# This is a message to hold data from an IMU\n#\n'
        'std_msgs/Header header\nfloat64[9] orientation_covariance', HEADER, TIME),
    'nav_msgs/msg/Odometry': _def(
        '# This represents an estimate of a position and velocity\n'
        'std_msgs/Header header\nstring child_frame_id', HEADER, TIME),
    'sensor_msgs/msg/NavSatFix': _def(
        '# Navigation Satellite fix\nuint8 COVARIANCE_TYPE_UNKNOWN = 0\n'
        'std_msgs/Header header\nfloat64 latitude\nfloat64 longitude',
        HEADER, TIME),
    'sensor_msgs/msg/BatteryState': _def(
        'std_msgs/Header header\nfloat32 voltage', HEADER, TIME),
    'diagnostic_msgs/msg/DiagnosticArray': _def(
        'std_msgs/Header header\nstring note', HEADER, TIME),
    'geometry_msgs/msg/TwistStamped': _def(
        'std_msgs/Header header\nfloat64 speed', HEADER, TIME),
    'std_msgs/msg/String': 'string data',
    'rcl_interfaces/msg/Log': _def(
        'builtin_interfaces/Time stamp\nstring msg', TIME),
}

TELEMETRY = {  # topic: (type, Hz)
    '/tf': ('tf2_msgs/msg/TFMessage', None),
    '/imu/data': ('sensor_msgs/msg/Imu', 10.0),
    '/odom': ('nav_msgs/msg/Odometry', 20.0),
    '/fix': ('sensor_msgs/msg/NavSatFix', 4.0),
    '/gps/status': ('std_msgs/msg/String', 1.0),
    '/odometry/local': ('nav_msgs/msg/Odometry', 30.0),
    '/odometry/global': ('nav_msgs/msg/Odometry', 30.0),
    '/odometry/gps': ('nav_msgs/msg/Odometry', 4.0),
    '/cmd_vel': ('geometry_msgs/msg/TwistStamped', 10.0),
    '/mower_base/telemetry': ('std_msgs/msg/String', 10.0),
    '/battery_state': ('sensor_msgs/msg/BatteryState', 1.0),
    '/diagnostics': ('diagnostic_msgs/msg/DiagnosticArray', 1.0),
    '/rosout': ('rcl_interfaces/msg/Log', 2.0),
    '/graph_snapshot': ('std_msgs/msg/String', 0.5),
    '/graph_events': ('std_msgs/msg/String', 0.5),
}

START_NS = 1_758_766_500_000_000_000  # 2025-09-25T02:15:00Z
EXTRINSICS = {
    'schema': ce.SCHEMA, 'source': 'measured', 'date': '2026-09-25',
    'measured_by': 'test', 'method': 'tape + level',
    'reference': {'axle_height_m': 0.09, 'axle_center_in_base_footprint_m': None},
    'camera': {'forward_of_axle_m': 0.35, 'left_of_center_m': 0.0,
               'height_m': 0.62, 'roll_deg': 0.0, 'pitch_deg': 20.0,
               'yaw_deg': 0.0},
}


def jpeg_bytes(size=(1280, 720), strip_dht=False):
    from PIL import Image
    buf = io.BytesIO()
    Image.new('RGB', size, (40, 120, 40)).save(buf, 'JPEG', quality=60)
    data = buf.getvalue()
    if strip_dht:  # emulate an MJPEG frame without Huffman tables
        out, i = bytearray(data[:2]), 2
        while i < len(data):
            marker = data[i + 1]
            seglen = (data[i + 2] << 8) | data[i + 3]
            if marker == 0xDA:
                out += data[i:]
                break
            if marker != 0xC4:
                out += data[i:i + 2 + seglen]
            i += 2 + seglen
        data = bytes(out)
    return data


def _stamp(ns):
    return {'sec': ns // 1_000_000_000, 'nanosec': ns % 1_000_000_000}


def _hdr(ns, frame):
    return {'stamp': _stamp(ns), 'frame_id': frame}


def _tf(ns, parent, child, t, q):
    return {'header': _hdr(ns, parent), 'child_frame_id': child,
            'transform': {'translation': dict(zip('xyz', t)),
                          'rotation': dict(zip('xyzw', q))}}


def tf_static_messages(ns, extrinsics=EXTRINSICS, chain=True):
    ref = ce.default_reference()
    res = ce.compute(extrinsics, ref)
    rsp = [_tf(ns, 'base_footprint', 'base_link',
               ref['base_link_in_base_footprint'].translation,
               ref['base_link_in_base_footprint'].quaternion)]
    msgs = [{'transforms': rsp}]
    if chain:
        for t, parent, child in (
                (res['base_link_to_camera_link'], 'base_link', 'camera_front_link'),
                (res['camera_link_to_optical'], 'camera_front_link',
                 'camera_front_optical_frame')):
            msgs.append({'transforms': [_tf(ns, parent, child, t.translation,
                                              t.quaternion)]})
    return msgs, res


class BagWriter:
    def __init__(self, path, compression):
        self.f = open(path, 'wb')
        self.w = Writer(self.f, compression=compression, chunk_size=64 * 1024)
        self.w.start(profile='ros2', library='synth_run')
        self.schemas, self.channels, self.enc = {}, {}, {}
        self.items = []

    def add(self, topic, type_name, msg, log_time, raw_transform=None):
        self.items.append((log_time, topic, type_name, msg, raw_transform))

    def close(self):
        counts = {}
        for log_time, topic, type_name, msg, raw in sorted(
                self.items, key=lambda it: it[0]):
            if type_name not in self.schemas:
                text = MSGDEFS[type_name]
                self.schemas[type_name] = self.w.register_schema(
                    name=type_name, encoding='ros2msg', data=text.encode())
                self.enc[type_name] = serialize_dynamic(type_name, text)[type_name]
            if topic not in self.channels:
                self.channels[topic] = self.w.register_channel(
                    topic=topic, message_encoding='cdr',
                    schema_id=self.schemas[type_name])
            data = self.enc[type_name](msg)
            if raw is not None:
                data = raw(data)
            self.w.add_message(channel_id=self.channels[topic], log_time=log_time,
                               publish_time=log_time, data=data, sequence=0)
            counts.setdefault(topic, [type_name, 0])[1] += 1
        self.w.finish()
        self.f.close()
        times = [it[0] for it in self.items]
        return counts, (min(times), max(times)) if times else (0, 0)


def write_metadata(bag_dir, counts, span, fname, compression_mode=''):
    doc = {'rosbag2_bagfile_information': {
        'version': 9, 'storage_identifier': 'mcap',
        'duration': {'nanoseconds': span[1] - span[0]},
        'starting_time': {'nanoseconds_since_epoch': span[0]},
        'message_count': sum(c for _t, c in counts.values()),
        'topics_with_message_count': [
            {'topic_metadata': {'name': topic, 'type': t,
                                'serialization_format': 'cdr',
                                'offered_qos_profiles': [],
                                'type_description_hash': 'RIHS01_0'},
             'message_count': c} for topic, (t, c) in sorted(counts.items())],
        'compression_format': 'zstd' if compression_mode else '',
        'compression_mode': compression_mode,
        'relative_file_paths': [fname],
        'files': [{'path': fname,
                   'starting_time': {'nanoseconds_since_epoch': span[0]},
                   'duration': {'nanoseconds': span[1] - span[0]},
                   'message_count': sum(c for _t, c in counts.values())}],
        'custom_data': None, 'ros_distro': 'jazzy'}}
    with open(os.path.join(bag_dir, 'metadata.yaml'), 'w') as f:
        yaml.safe_dump(doc, f, sort_keys=False)


def camera_info(ns, size, calibrated=True):
    w, h = size
    k = [0.9 * w, 0.0, w / 2.0, 0.0, 0.9 * w, h / 2.0, 0.0, 0.0, 1.0] \
        if calibrated else [0.0] * 9
    return {'header': _hdr(ns, 'camera_front_optical_frame'), 'height': h,
            'width': w, 'distortion_model': 'plumb_bob',
            'd': [0.01, -0.02, 0.0, 0.0, 0.0], 'k': k,
            'r': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            'p': [k[0], 0.0, k[2], 0.0, 0.0, k[4], k[5], 0.0, 0.0, 0.0, 1.0, 0.0],
            'binning_x': 0, 'binning_y': 0,
            'roi': {'x_offset': 0, 'y_offset': 0, 'height': 0, 'width': 0,
                    'do_rectify': False}}


def calib_yaml(size, calibrated=True):
    ci = camera_info(0, size, calibrated)
    return {'image_width': size[0], 'image_height': size[1],
            'camera_name': 'camera_front',
            'camera_matrix': {'rows': 3, 'cols': 3, 'data': ci['k']},
            'distortion_model': 'plumb_bob',
            'distortion_coefficients': {'rows': 1, 'cols': 5, 'data': ci['d']},
            'rectification_matrix': {'rows': 3, 'cols': 3, 'data': ci['r']},
            'projection_matrix': {'rows': 3, 'cols': 4, 'data': ci['p']}}


def make_run(root, run_id='mower-03_20260925T101500', *, duration_s=6.0,
             cam_hz=10.0, image_size=(1280, 720), info_size=None,
             drop_info=(), dup_image=None, omit=(), hz_override=None,
             tf_chain=True, extrinsics_source='measured', calibrated=True,
             telemetry_compression=CompressionType.ZSTD,
             camera_compression=CompressionType.NONE, message_compression=False,
             strip_dht=False, stamp_as_receive=False, extra_meta=None):
    run_dir = os.path.join(root, run_id)
    bag_dir = os.path.join(run_dir, 'bag')
    cam_dir = os.path.join(run_dir, 'camera')
    calib_dir = os.path.join(run_dir, 'calib')
    for d in (bag_dir, cam_dir, calib_dir):
        os.makedirs(d, exist_ok=True)
    extrinsics = dict(EXTRINSICS, source=extrinsics_source)
    statics, res = tf_static_messages(START_NS, extrinsics, chain=tf_chain)
    hz = dict((t, v[1]) for t, v in TELEMETRY.items())
    hz.update(hz_override or {})

    raw = None
    if message_compression:
        import zstandard
        cctx = zstandard.ZstdCompressor()
        raw = cctx.compress

    tele = BagWriter(os.path.join(bag_dir, 'bag_0.mcap'), telemetry_compression)
    for m in statics:
        tele.add('/tf_static', 'tf2_msgs/msg/TFMessage', m, START_NS, raw)
    n_tf = int(duration_s * 30)
    for i in range(n_tf):
        ns = START_NS + int(i * 1e9 / 30)
        if '/tf' not in omit:
            tele.add('/tf', 'tf2_msgs/msg/TFMessage', {'transforms': [
                _tf(ns, 'odom', 'base_footprint', (0.01 * i, 0, 0), (0, 0, 0, 1))]},
                ns + 1_000_000, raw)
            tele.add('/tf', 'tf2_msgs/msg/TFMessage', {'transforms': [
                _tf(ns, 'map', 'odom', (1.0, 2.0, 0), (0, 0, 0, 1))]},
                ns + 1_500_000, raw)
    for topic, (type_name, _hz) in TELEMETRY.items():
        rate = hz.get(topic)
        if topic in omit or topic == '/tf' or not rate:
            continue
        n = max(1, int(duration_s * rate))
        for i in range(n):
            ns = START_NS + int(i * 1e9 / rate)
            if type_name == 'std_msgs/msg/String':
                msg = {'data': json.dumps({'i': i})}
            elif type_name == 'rcl_interfaces/msg/Log':
                msg = {'stamp': _stamp(ns), 'msg': 'hello'}
            elif type_name == 'sensor_msgs/msg/Imu':
                msg = {'header': _hdr(ns, 'imu_link'),
                       'orientation_covariance': [0.0] * 9}
            elif type_name == 'nav_msgs/msg/Odometry':
                msg = {'header': _hdr(ns, 'odom'), 'child_frame_id': 'base_footprint'}
            elif type_name == 'sensor_msgs/msg/NavSatFix':
                msg = {'header': _hdr(ns, 'gps'), 'latitude': 23.7, 'longitude': 120.5}
            elif type_name == 'sensor_msgs/msg/BatteryState':
                msg = {'header': _hdr(ns, 'battery'), 'voltage': 25.0}
            elif type_name == 'diagnostic_msgs/msg/DiagnosticArray':
                msg = {'header': _hdr(ns, ''), 'note': 'ok'}
            else:
                msg = {'header': _hdr(ns, 'base_footprint'), 'speed': 0.1}
            tele.add(topic, type_name, msg, ns + 2_000_000, raw)
    tcounts, tspan = tele.close()
    write_metadata(bag_dir, tcounts, tspan, 'bag_0.mcap',
                   'message' if message_compression else '')

    cam = BagWriter(os.path.join(cam_dir, 'camera_0.mcap'), camera_compression)
    for m in statics:
        cam.add('/tf_static', 'tf2_msgs/msg/TFMessage', m, START_NS)
    img = jpeg_bytes(image_size, strip_dht=strip_dht)
    n_img = int(duration_s * cam_hz)
    for i in range(n_img):
        cap = START_NS + int(i * 1e9 / cam_hz) + 5_000_000
        stamp = cap
        if dup_image is not None and i == dup_image + 1:
            stamp = START_NS + int(dup_image * 1e9 / cam_hz) + 5_000_000
        recv = cap + (1_000_000 if stamp_as_receive else 45_000_000)
        if stamp_as_receive:
            stamp = recv - 500_000
        if 'image' not in omit:
            cam.add('/camera/front/image_raw/compressed',
                    'sensor_msgs/msg/CompressedImage',
                    {'header': _hdr(stamp, 'camera_front_optical_frame'),
                     'format': 'jpeg', 'data': img}, recv)
        if 'info' not in omit and i not in drop_info:
            cam.add('/camera/front/camera_info', 'sensor_msgs/msg/CameraInfo',
                    camera_info(stamp, info_size or image_size, calibrated),
                    recv + 100_000)
    ccounts, cspan = cam.close()
    write_metadata(cam_dir, ccounts, cspan, 'camera_0.mcap')

    with open(os.path.join(calib_dir, 'camera_front.yaml'), 'w') as f:
        yaml.safe_dump(calib_yaml(info_size or image_size, calibrated), f,
                       sort_keys=False)
    ext_doc = dict(extrinsics, derived=ce.derived_block(res))
    with open(os.path.join(calib_dir, 'extrinsics.yaml'), 'w') as f:
        yaml.safe_dump(ext_doc, f, sort_keys=False, allow_unicode=True)

    meta = {
        'run_id': run_id, 'robot_id': run_id.split('_')[0],
        'start_time': '2026-09-25T10:15:00+08:00', 'ros_distro': 'jazzy',
        'git': {'sha': 'unknown'}, 'gps_start': None,
        'recorded_topics': sorted(TELEMETRY) + ['/tf_static'],
        'snapshot_topics': [], 'profile': 'data_collection',
        'camera': {'model': 'UVC-1080', 'serial': 'SN123', 'device': '/dev/video0',
                   'width': image_size[0], 'height': image_size[1],
                   'fps_native': 25, 'fps_recorded': int(cam_hz),
                   'format': 'jpeg', 'autofocus': 'off', 'exposure': 'auto'},
        'camera_height_m': 0.62, 'camera_pitch_deg': 20.0,
        'extrinsics_source': extrinsics_source,
        'calibration': {'intrinsics_file': 'calib/camera_front.yaml',
                        'extrinsics_file': 'calib/extrinsics.yaml',
                        'date': '2026-09-25', 'intrinsics_source': 'calibrated'},
        'grass_height_cm': 8, 'weather': 'sunny', 'blade': 'off',
        'heading_calibration': 'straight_line_done', 'notes': '',
        'build': {'version': 'v0.7.0', 'git_sha': 'abc123', 'image': ''},
    }
    meta.update(extra_meta or {})
    with open(os.path.join(run_dir, 'run_metadata.yaml'), 'w') as f:
        yaml.safe_dump(meta, f, sort_keys=False, allow_unicode=True)
    return run_dir
