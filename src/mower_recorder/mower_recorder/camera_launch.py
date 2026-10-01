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
"""Front camera for data collection: gscam MJPEG passthrough + static TF.

Driver choice (P-097, checked against the Jazzy arm64 binaries 2026-09):

* ``ros-jazzy-gscam`` 2.0.2 with ``image_encoding: jpeg`` publishes the MJPEG
  buffers of ``v4l2src`` unchanged as ``sensor_msgs/CompressedImage`` (no
  decode / re-encode) and, with ``use_gst_timestamps: true``, stamps them with
  the V4L2 capture time mapped to ROS time. ``videorate drop-only=true
  max-rate=N`` thins 25 -> 10 fps and keeps the original timestamps.
* usb_cam 0.8.1 publishes CompressedImage only for ``pixel_format: mjpeg``,
  which its own format table does not accept (``raw_mjpeg``), and copies the
  whole V4L2 buffer instead of ``bytesused``; v4l2_camera 0.7.3 decodes MJPEG
  and stamps with ``now()``; gscam2 has no Jazzy release.
* The binary does not depend on gstreamer1.0-plugins-good (v4l2src) /
  -plugins-base (videorate): mower_recorder/package.xml adds them.
* gscam computes the GStreamer->ROS clock offset once per stream: let chrony
  settle before starting, or restart the camera after a clock step.

Imported by launch/front_camera.launch.py and launch/data_collection.launch.py.
"""
import json
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mower_recorder import camera_extrinsics as ce

IMAGE_TOPIC = '/camera/front/image_raw/compressed'
INFO_TOPIC = '/camera/front/camera_info'
CAMERA_NAME = 'camera_front'

DEFAULT_V4L2_CONTROLS = (
    # autofocus off (new | old kernel control names) and no auto-exposure
    # frame-rate drop in dim light; names a camera lacks are ignored with a
    # GStreamer warning. Check with `v4l2-ctl -d /dev/video0 --list-ctrls`.
    'c,focus_automatic_continuous=0,focus_auto=0,'
    'exposure_dynamic_framerate=0,exposure_auto_priority=0')

CAMERA_ARGS = (
    ('camera_device', '/dev/video0', 'V4L2 device of the front USB camera'),
    ('camera_size', '1280x720', 'MJPEG mode WIDTHxHEIGHT (CAMERA_SIZE)'),
    ('camera_fps', '25', 'native MJPEG frame rate of that mode (CAMERA_FPS)'),
    ('record_fps', '10', 'rate published to ROS (videorate drop-only)'),
    ('v4l2_controls', DEFAULT_V4L2_CONTROLS,
     "v4l2src extra-controls structure ('' = leave the camera alone)"),
    ('gscam_pipeline', '', 'full GStreamer pipeline override (advanced)'),
    ('calib_dir', '~/.mower/calib',
     'robot-local calibration dir: camera_front.yaml + extrinsics.yaml'),
    ('intrinsics_file', '', "'' = <calib_dir>/camera_front.yaml"),
    ('extrinsics_file', '',
     "'' = <calib_dir>/extrinsics.yaml, else the package placeholder"),
    ('start_camera', 'true', 'false = TF only (camera driven elsewhere)'),
    ('publish_tf', 'true',
     'publish base_link -> camera_front_link -> camera_front_optical_frame'),
)


def declare_camera_arguments():
    return [DeclareLaunchArgument(n, default_value=d, description=h)
            for n, d, h in CAMERA_ARGS]


def _truthy(text):
    return str(text).strip().lower() in ('1', 'true', 'yes', 'on')


def build_pipeline(device, width, height, fps, record_fps, controls):
    src = f'v4l2src device={device}'
    if controls:
        src += f' extra-controls="{controls}"'
    parts = [src, f'image/jpeg,width={width},height={height},framerate={fps}/1']
    if record_fps and record_fps < fps:
        # drop-only keeps each kept frame's capture PTS (no re-timestamping)
        parts.append(f'videorate drop-only=true max-rate={int(round(record_fps))}')
    return ' ! '.join(parts)


def placeholder_intrinsics(width, height):
    """Uncalibrated camera_info (K = 0, right size) until calibration exists."""
    doc = {
        'image_width': width, 'image_height': height, 'camera_name': CAMERA_NAME,
        'camera_matrix': {'rows': 3, 'cols': 3, 'data': [0.0] * 9},
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {'rows': 1, 'cols': 5, 'data': [0.0] * 5},
        'rectification_matrix': {'rows': 3, 'cols': 3,
                                 'data': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]},
        'projection_matrix': {'rows': 3, 'cols': 4, 'data': [0.0] * 12},
    }
    path = os.path.join(tempfile.gettempdir(),
                        f'mower_camera_front_uncalibrated_{width}x{height}.yaml')
    with open(path, 'w') as f:
        f.write('# 自動產生：尚未標定（K 全 0）。完成棋盤格標定後放到 '
                '~/.mower/calib/camera_front.yaml\n')
        yaml.safe_dump(doc, f, sort_keys=False)
    return path


def robot_reference():
    """URDF geometry for the extrinsics conversion (falls back to constants)."""
    try:
        import xacro
        path = os.path.join(get_package_share_directory('mower_description'),
                            'mower_robot', 'real_robot.xacro')
        return ce.urdf_reference(xacro.process_file(path).toxml()), None
    except Exception as e:  # noqa: BLE001
        return ce.default_reference(), f'URDF 讀取失敗，改用內建幾何：{e}'


def resolve_camera(context):
    """Evaluate the launch arguments into everything the actions need."""
    def get(name):
        return LaunchConfiguration(name).perform(context).strip()

    width, height = (int(v) for v in get('camera_size').lower().split('x'))
    fps = int(float(get('camera_fps')))
    record_fps = float(get('record_fps'))
    controls = get('v4l2_controls')
    calib_dir = os.path.expanduser(get('calib_dir'))
    notes = []

    intrinsics = os.path.expanduser(get('intrinsics_file')) or \
        os.path.join(calib_dir, 'camera_front.yaml')
    intrinsics_placeholder = not os.path.isfile(intrinsics)
    if intrinsics_placeholder:
        notes.append(f'找不到內參 {intrinsics}：改用未標定佔位檔（check_run 會 FAIL，'
                     '錄棋盤格時請加 --allow-uncalibrated）')
        intrinsics = placeholder_intrinsics(width, height)

    extrinsics = os.path.expanduser(get('extrinsics_file')) or \
        os.path.join(calib_dir, 'extrinsics.yaml')
    if not os.path.isfile(extrinsics):
        notes.append(f'找不到外參 {extrinsics}：改用套件內的佔位值')
        extrinsics = os.path.join(get_package_share_directory('mower_recorder'),
                                  'config', 'camera_front_extrinsics.yaml')
    with open(extrinsics) as f:
        ext_doc = yaml.safe_load(f) or {}
    reference, ref_note = robot_reference()
    if ref_note:
        notes.append(ref_note)
    result = ce.compute(ext_doc, reference)  # invalid YAML -> launch fails
    notes += result['warnings']

    pipeline = get('gscam_pipeline') or build_pipeline(
        get('camera_device'), width, height, fps, record_fps, controls)
    autofocus = 'off' if ('focus_automatic_continuous=0' in controls
                          or 'focus_auto=0' in controls) else None
    camera_params = {
        'device': get('camera_device'), 'width': width, 'height': height,
        'fps_native': fps,
        'fps_recorded': int(record_fps) if record_fps.is_integer() else record_fps,
        'format': 'jpeg', 'autofocus': autofocus,
        'driver': 'gscam', 'pipeline': pipeline,
    }
    derived = {
        'summary': {'source': result['source'],
                    'camera_height_m': result['camera_height_m'],
                    'camera_pitch_deg': result['camera_pitch_deg']},
        'block': ce.derived_block(result),
    }
    return {
        'pipeline': pipeline, 'intrinsics': intrinsics,
        'intrinsics_placeholder': intrinsics_placeholder,
        'extrinsics': extrinsics, 'extrinsics_result': result,
        'camera_params': camera_params, 'derived': derived, 'notes': notes,
        'start_camera': _truthy(get('start_camera')),
        'publish_tf': _truthy(get('publish_tf')),
    }


def camera_actions(cam):
    """gscam + static TF + a readable summary of what is being published."""
    res = cam['extrinsics_result']
    opt = ce.transform_dict(res['base_footprint_to_optical'], ce.BASE_FOOTPRINT,
                            ce.OPTICAL_FRAME)
    actions = [LogInfo(msg=(
        f'[data_collection] camera: {cam["pipeline"]}\n'
        f'  intrinsics: {cam["intrinsics"]}\n'
        f'  extrinsics: {cam["extrinsics"]} (source={res["source"]})\n'
        f'  base_footprint -> optical: xyz={opt["xyz_m"]} rpy_deg={opt["rpy_deg"]}'))]
    actions += [LogInfo(msg=f'[data_collection] WARNING: {n}') for n in cam['notes']]
    if cam['start_camera']:
        actions.append(Node(
            package='gscam', executable='gscam_node', name=CAMERA_NAME,
            output='screen', respawn=True, respawn_delay=3.0,
            parameters=[{
                'gscam_config': cam['pipeline'],
                'image_encoding': 'jpeg',        # MJPEG passthrough
                'use_gst_timestamps': True,      # capture time, not now()
                'sync_sink': False,
                'preroll': False,
                'reopen_on_eof': True,
                'camera_name': CAMERA_NAME,
                'camera_info_url': 'file://' + cam['intrinsics'],
                'frame_id': ce.OPTICAL_FRAME,
                'use_sensor_data_qos': False,
            }],
            remappings=[('camera/image_raw/compressed', IMAGE_TOPIC),
                        ('camera/camera_info', INFO_TOPIC)],
        ))
    if cam['publish_tf']:
        for t, parent, child, name in (
                (res['base_link_to_camera_link'], ce.BASE_LINK, ce.CAMERA_LINK,
                 'camera_front_link_tf'),
                (res['camera_link_to_optical'], ce.CAMERA_LINK, ce.OPTICAL_FRAME,
                 'camera_front_optical_tf')):
            actions.append(Node(
                package='tf2_ros', executable='static_transform_publisher',
                name=name, output='log',
                arguments=ce.static_tf_arguments(t, parent, child)))
    return actions


def recorder_parameters(cam):
    """Parameters recorder_manager needs to describe the camera in the run."""
    return {
        'camera_params': json.dumps(cam['camera_params']),
        'intrinsics_file': cam['intrinsics'],
        'intrinsics_placeholder': cam['intrinsics_placeholder'],
        'extrinsics_file': cam['extrinsics'],
        'extrinsics_derived': json.dumps(cam['derived']),
    }
