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
"""Front-camera extrinsics: measured YAML -> static TF + derived calib block.

Pure Python (no numpy, no ROS) so the same code runs in the launch file, in
recorder_manager (run_metadata / calib/extrinsics.yaml) and in pytest.

Conventions
-----------
* ``base_footprint`` and ``camera_front_link`` use REP-103 body axes:
  x forward, y left, z up.
* roll/pitch/yaw are fixed-axis XYZ as in URDF / tf2:
  ``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``. A positive pitch turns +x toward -z,
  i.e. **positive pitch = camera looks down**.
* ``camera_front_optical_frame`` is the REP-103 optical frame (x right, y down,
  z forward) = ``camera_front_link`` rotated by rpy (-pi/2, 0, -pi/2).

What is measured (see docs/資料收集錄製程序.md)
------------------------------------------------
``base_footprint`` of this robot is not a point you can find on the chassis:
the URDF puts it ~0.17 m in front of the drive-wheel axle and ~0.09 m above
the ground (P-086 / P-098). The YAML therefore stores *physical*
measurements relative to the midpoint of the drive-wheel axle projected on the
ground, and this module converts them with the reference geometry read from
the URDF (or given explicitly in the YAML).
"""
import math
import xml.etree.ElementTree as ET

SCHEMA = 'mower.camera_extrinsics/v1'
VALID_SOURCES = ('placeholder', 'measured', 'cad')

BASE_FOOTPRINT = 'base_footprint'
BASE_LINK = 'base_link'
CAMERA_LINK = 'camera_front_link'
OPTICAL_FRAME = 'camera_front_optical_frame'

# Fallback when the URDF cannot be processed (values of robot.urdf.xacro,
# 2026-09-25): base_footprint -> base_link is a pure yaw of 1.5708 rad and the
# drive wheels sit at base_link (0.13253 | -0.10347, 0.165534, -0.00295275).
DEFAULT_BASE_LINK_RPY = (0.0, 0.0, 1.5708)
DEFAULT_BASE_LINK_XYZ = (0.0, 0.0, 0.0)
DEFAULT_WHEEL_JOINTS_IN_BASE_LINK = (
    (0.13253, 0.165534, -0.00295275),
    (-0.10347, 0.165534, -0.00295275),
)

# camera_front_link -> camera_front_optical_frame (fixed, REP-103).
OPTICAL_RPY = (-math.pi / 2.0, 0.0, -math.pi / 2.0)


# ── small rigid-transform helpers (3x3 lists + xyz tuples) ───────────────────
def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def matrix_to_rpy(m):
    """Inverse of rpy_to_matrix (fixed-axis XYZ); pitch in [-pi/2, pi/2]."""
    sp = -m[2][0]
    sp = max(-1.0, min(1.0, sp))
    pitch = math.asin(sp)
    if abs(sp) < 1.0 - 1e-9:
        roll = math.atan2(m[2][1], m[2][2])
        yaw = math.atan2(m[1][0], m[0][0])
    else:  # gimbal lock: put everything into yaw
        roll = 0.0
        yaw = math.atan2(-m[0][1], m[1][1])
    return (roll, pitch, yaw)


def matrix_to_quaternion(m):
    """Rotation matrix -> (x, y, z, w), w >= 0."""
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w)
    q = (x / n, y / n, z / n, w / n)
    return q if q[3] >= 0.0 else tuple(-c for c in q)


def quaternion_to_matrix(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def _matvec(a, v):
    return tuple(sum(a[i][k] * v[k] for k in range(3)) for i in range(3))


def _transpose(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


class Transform:
    """Rigid transform T_parent_child: p_parent = R @ p_child + t."""

    __slots__ = ('rotation', 'translation')

    def __init__(self, rotation=None, translation=(0.0, 0.0, 0.0)):
        self.rotation = rotation or [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0],
                                     [0.0, 0.0, 1.0]]
        self.translation = tuple(float(v) for v in translation)

    @classmethod
    def from_xyz_rpy(cls, xyz, rpy):
        return cls(rpy_to_matrix(*rpy), xyz)

    def __matmul__(self, other):
        rot = _matmul(self.rotation, other.rotation)
        rt = _matvec(self.rotation, other.translation)
        return Transform(rot, tuple(rt[i] + self.translation[i] for i in range(3)))

    def inverse(self):
        rt = _transpose(self.rotation)
        t = _matvec(rt, self.translation)
        return Transform(rt, tuple(-v for v in t))

    def apply(self, point):
        r = _matvec(self.rotation, point)
        return tuple(r[i] + self.translation[i] for i in range(3))

    def rotate(self, vector):
        return _matvec(self.rotation, vector)

    @property
    def rpy(self):
        return matrix_to_rpy(self.rotation)

    @property
    def quaternion(self):
        return matrix_to_quaternion(self.rotation)


# ── URDF reference geometry ──────────────────────────────────────────────────
def _origin(joint):
    org = joint.find('origin')
    if org is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    xyz = tuple(float(v) for v in (org.get('xyz') or '0 0 0').split())
    rpy = tuple(float(v) for v in (org.get('rpy') or '0 0 0').split())
    return xyz, rpy


def urdf_reference(urdf_xml):
    """Read the geometry the extrinsics conversion needs from a URDF string.

    Returns a dict with ``base_link_in_base_footprint`` (Transform) and
    ``axle_center_in_base_footprint`` (xyz of the drive-wheel axle midpoint).
    Accepts a processed URDF or a plain-XML xacro (robot.urdf.xacro has no
    macros around these joints).
    """
    root = ET.fromstring(urdf_xml)
    joints = {}
    for j in root.iter('joint'):
        parent = j.find('parent')
        child = j.find('child')
        if parent is None or child is None:
            continue
        joints[j.get('name')] = (parent.get('link'), child.get('link'),
                                 j.get('type'), _origin(j))
    t_bf_bl = None
    wheels = []
    for name, (parent, child, jtype, (xyz, rpy)) in joints.items():
        if parent == BASE_FOOTPRINT and child == BASE_LINK:
            t_bf_bl = Transform.from_xyz_rpy(xyz, rpy)
        elif parent == BASE_LINK and jtype == 'continuous' and 'wheel' in name \
                and 'caster' not in name:
            wheels.append(xyz)
    if t_bf_bl is None:
        raise ValueError('URDF has no base_footprint -> base_link joint')
    if len(wheels) != 2:
        raise ValueError(f'expected 2 drive wheel joints, found {len(wheels)}')
    mid_bl = tuple((wheels[0][i] + wheels[1][i]) / 2.0 for i in range(3))
    return {
        'base_link_in_base_footprint': t_bf_bl,
        'axle_center_in_base_footprint': t_bf_bl.apply(mid_bl),
        'source': 'urdf',
    }


def default_reference():
    t_bf_bl = Transform.from_xyz_rpy(DEFAULT_BASE_LINK_XYZ, DEFAULT_BASE_LINK_RPY)
    w0, w1 = DEFAULT_WHEEL_JOINTS_IN_BASE_LINK
    mid_bl = tuple((w0[i] + w1[i]) / 2.0 for i in range(3))
    return {
        'base_link_in_base_footprint': t_bf_bl,
        'axle_center_in_base_footprint': t_bf_bl.apply(mid_bl),
        'source': 'builtin_default',
    }


# ── YAML model ───────────────────────────────────────────────────────────────
def _num(section, key, errors, where):
    v = section.get(key)
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        errors.append(f'{where}.{key} 必須是數字（目前：{v!r}）')
        return 0.0
    return float(v)


def validate(ext):
    """Return (errors, warnings); errors make the file unusable."""
    errors, warnings = [], []
    if not isinstance(ext, dict):
        return ['extrinsics YAML 不是 mapping'], []
    if ext.get('schema') not in (None, SCHEMA):
        warnings.append(f'schema={ext.get("schema")!r}，預期 {SCHEMA}')
    source = str(ext.get('source', '')).strip()
    if source not in VALID_SOURCES:
        errors.append(f'source 必須是 {"/".join(VALID_SOURCES)}（目前：{source!r}）')
    elif source == 'placeholder':
        warnings.append('source=placeholder：外參是佔位值，必須實測後覆寫')
    cam = ext.get('camera')
    ref = ext.get('reference') or {}
    if not isinstance(cam, dict):
        errors.append('缺少 camera 區塊')
        return errors, warnings
    for key in ('forward_of_axle_m', 'left_of_center_m', 'height_m',
                'roll_deg', 'pitch_deg', 'yaw_deg'):
        _num(cam, key, errors, 'camera')
    if not isinstance(ref, dict):
        errors.append('reference 必須是 mapping')
        ref = {}
    _num(ref, 'axle_height_m', errors, 'reference')
    if errors:
        return errors, warnings
    if not 0.05 <= cam['height_m'] <= 2.5:
        errors.append(f'camera.height_m={cam["height_m"]} 超出合理範圍 0.05～2.5 m')
    if not -1.0 <= cam['forward_of_axle_m'] <= 2.0:
        warnings.append(f'camera.forward_of_axle_m={cam["forward_of_axle_m"]} 異常')
    if not -5.0 <= cam['pitch_deg'] <= 80.0:
        errors.append(f'camera.pitch_deg={cam["pitch_deg"]} 超出 -5～80°（向下為正）')
    if abs(cam['roll_deg']) > 20.0 or abs(cam['yaw_deg']) > 45.0:
        warnings.append('camera.roll_deg／yaw_deg 偏大，請再確認量測')
    if not 0.02 <= ref['axle_height_m'] <= 0.5:
        errors.append(f'reference.axle_height_m={ref["axle_height_m"]} 超出 0.02～0.5 m')
    axle = ref.get('axle_center_in_base_footprint_m')
    if axle is not None and (not isinstance(axle, (list, tuple)) or len(axle) != 3):
        errors.append('reference.axle_center_in_base_footprint_m 必須是 [x, y, z] 或 null')
    return errors, warnings


def compute(ext, reference=None):
    """Measured extrinsics -> transforms.

    ``reference`` comes from urdf_reference()/default_reference(); an explicit
    ``reference.axle_center_in_base_footprint_m`` in the YAML wins for the axle.
    Raises ValueError when validate() reports errors.
    """
    errors, warnings = validate(ext)
    if errors:
        raise ValueError('; '.join(errors))
    reference = reference or default_reference()
    cam = ext['camera']
    ref = ext.get('reference') or {}
    axle = ref.get('axle_center_in_base_footprint_m')
    axle = tuple(float(v) for v in axle) if axle is not None else \
        tuple(reference['axle_center_in_base_footprint'])
    ground_z = axle[2] - float(ref['axle_height_m'])
    xyz = (axle[0] + float(cam['forward_of_axle_m']),
           axle[1] + float(cam['left_of_center_m']),
           ground_z + float(cam['height_m']))
    rpy = tuple(math.radians(float(cam[k]))
                for k in ('roll_deg', 'pitch_deg', 'yaw_deg'))
    t_bf_cam = Transform.from_xyz_rpy(xyz, rpy)
    t_cam_opt = Transform.from_xyz_rpy((0.0, 0.0, 0.0), OPTICAL_RPY)
    t_bf_bl = reference['base_link_in_base_footprint']
    return {
        'source': str(ext.get('source')),
        'warnings': warnings,
        'reference_source': reference.get('source', ''),
        'axle_center_in_base_footprint': axle,
        'base_footprint_height_m': -ground_z,
        'camera_height_m': float(cam['height_m']),
        'camera_pitch_deg': float(cam['pitch_deg']),
        'base_footprint_to_camera_link': t_bf_cam,
        'base_link_to_camera_link': t_bf_bl.inverse() @ t_bf_cam,
        'camera_link_to_optical': t_cam_opt,
        'base_footprint_to_optical': t_bf_cam @ t_cam_opt,
    }


def _round(v, n=6):
    return [round(float(x), n) for x in v]


def transform_dict(t, parent, child):
    rpy = t.rpy
    return {
        'parent': parent,
        'child': child,
        'xyz_m': _round(t.translation),
        'rpy_rad': _round(rpy),
        'rpy_deg': _round([math.degrees(a) for a in rpy], 4),
        'quaternion_xyzw': _round(t.quaternion, 9),
    }


def derived_block(result):
    """YAML-friendly summary written into <run>/calib/extrinsics.yaml."""
    return {
        'note': '由 mower_recorder 依上方量測值計算；/tf_static 發布的就是這些值',
        'reference_source': result['reference_source'],
        'axle_center_in_base_footprint_m': _round(result['axle_center_in_base_footprint']),
        'base_footprint_height_m': round(result['base_footprint_height_m'], 6),
        'camera_height_m': round(result['camera_height_m'], 6),
        'camera_pitch_deg': round(result['camera_pitch_deg'], 4),
        'base_footprint_to_camera_front_optical_frame': transform_dict(
            result['base_footprint_to_optical'], BASE_FOOTPRINT, OPTICAL_FRAME),
        'base_footprint_to_camera_front_link': transform_dict(
            result['base_footprint_to_camera_link'], BASE_FOOTPRINT, CAMERA_LINK),
        'published_tf': [
            transform_dict(result['base_link_to_camera_link'], BASE_LINK, CAMERA_LINK),
            transform_dict(result['camera_link_to_optical'], CAMERA_LINK, OPTICAL_FRAME),
        ],
    }


def static_tf_arguments(t, parent, child):
    """Arguments for `ros2 run tf2_ros static_transform_publisher` (Jazzy)."""
    x, y, z = t.translation
    qx, qy, qz, qw = t.quaternion
    args = []
    for flag, val in (('--x', x), ('--y', y), ('--z', z),
                      ('--qx', qx), ('--qy', qy), ('--qz', qz), ('--qw', qw)):
        args += [flag, repr(round(float(val), 9))]
    return args + ['--frame-id', parent, '--child-frame-id', child]
