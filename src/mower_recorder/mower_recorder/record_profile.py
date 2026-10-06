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
"""Recording profiles: rosbag2 command lines + data_collection run metadata.

Pure Python (only PyYAML) so recorder_manager_node stays thin and the command
lines can be unit-tested without ROS.

Two profiles ship with the package:

* ``record_topics.yaml`` (always-on, the app replays it): unchanged behaviour —
  one full recorder with rosbag2 per-message zstd (``compression: true``) and
  an optional RAM snapshot recorder.
* ``data_collection.yaml``: telemetry recorder with MCAP chunk compression
  (``storage_preset_profile: zstd_fast``, P-089), a second uncompressed
  recorder for the camera (``camera_recorder``), run metadata of spec 1.6 and
  the calibration files copied into ``<run>/calib``.
"""
import copy
import json
import os
import shutil

import yaml

FULL_RECORDER_NODE = 'mower_full_recorder'
CAMERA_RECORDER_NODE = 'mower_camera_recorder'
SNAPSHOT_RECORDER_NODE = 'mower_snapshot_recorder'

# Recorders whose process state means "this run is being recorded".
DATA_RECORDERS = ('full', 'camera')

DEFAULT_CONFIG = {
    'topics': [],
    'snapshot_topics': [],
    'compression': True,
    'max_bag_size_mb': 512,
    'max_bag_duration_s': 3600,
    'enable_snapshot': True,
    'snapshot_max_cache_mb': 256,
}

DEFAULT_CAMERA_RECORDER = {
    'enabled': False,
    'topics': [],
    'max_bag_size_mb': 256,
    'max_bag_duration_s': 0,
    'storage_preset_profile': '',
}

INTRINSICS_NAME = 'camera_front.yaml'
EXTRINSICS_NAME = 'extrinsics.yaml'

# Operator fields of spec 1.6 (the rest is derived from the camera launch and
# the extrinsics file). Missing ones are written as null so nobody has to
# guess whether the field was forgotten or unknown.
SESSION_FIELDS = ('grass_height_cm', 'weather', 'blade', 'heading_calibration',
                  'notes')
ONOFF_FIELDS = ('blade',)
CAMERA_ONOFF_FIELDS = ('autofocus',)


def load_config(path, log_warn=None, log_error=None):
    """Profile YAML merged over DEFAULT_CONFIG (same rules as before)."""
    cfg = copy.deepcopy(DEFAULT_CONFIG)
    if path and os.path.isfile(path):
        try:
            with open(path) as f:
                cfg.update(yaml.safe_load(f) or {})
        except Exception as e:  # noqa: BLE001 — keep the node alive
            if log_error:
                log_error(f'load config failed: {e}')
    elif log_warn:
        log_warn(f'config_file not found: {path!r}')
    cam = copy.deepcopy(DEFAULT_CAMERA_RECORDER)
    cam.update(cfg.get('camera_recorder') or {})
    cfg['camera_recorder'] = cam
    return cfg


def _compression_args(cfg):
    preset = str(cfg.get('storage_preset_profile') or '').strip()
    if preset and preset.lower() != 'none':
        # MCAP chunk compression: Foxglove / python mcap read it directly.
        return ['--storage-preset-profile', preset]
    if cfg.get('compression', True):
        # Legacy always-on profile: rosbag2 per-message zstd (P-089).
        return ['--compression-mode', 'message', '--compression-format', 'zstd']
    return []


def recorder_specs(cfg, run_dir, qos_path=''):
    """Build the ``ros2 bag record`` command of every recorder of a run.

    Returns a list of dicts ``{name, node_name, out_dir, topics, cmd}`` in
    start order. For the always-on profile the command lines are byte-for-byte
    those of the original recorder_manager_node (see test_record_profile.py).
    """
    qos = ['--qos-profile-overrides-path', qos_path] \
        if qos_path and os.path.isfile(qos_path) else []
    specs = []

    bag_dir = os.path.join(run_dir, 'bag')
    cmd = ['ros2', 'bag', 'record', '-s', 'mcap', '-o', bag_dir,
           '--node-name', FULL_RECORDER_NODE]
    cmd += _compression_args(cfg)
    cmd += ['--max-bag-size', str(int(cfg['max_bag_size_mb']) * 1024 * 1024)]
    cmd += ['--max-bag-duration', str(int(cfg['max_bag_duration_s']))]
    cmd += qos
    cmd += list(cfg['topics'])  # positional topics must come last
    specs.append({'name': 'full', 'node_name': FULL_RECORDER_NODE,
                  'out_dir': bag_dir, 'topics': list(cfg['topics']), 'cmd': cmd})

    cam = cfg.get('camera_recorder') or {}
    if cam.get('enabled') and cam.get('topics'):
        cam_dir = os.path.join(run_dir, 'camera')
        ccmd = ['ros2', 'bag', 'record', '-s', 'mcap', '-o', cam_dir,
                '--node-name', CAMERA_RECORDER_NODE]
        ccmd += _compression_args({
            'storage_preset_profile': cam.get('storage_preset_profile', ''),
            'compression': False})
        if int(cam.get('max_bag_size_mb') or 0) > 0:
            ccmd += ['--max-bag-size',
                     str(int(cam['max_bag_size_mb']) * 1024 * 1024)]
        if int(cam.get('max_bag_duration_s') or 0) > 0:
            ccmd += ['--max-bag-duration', str(int(cam['max_bag_duration_s']))]
        ccmd += qos
        ccmd += list(cam['topics'])
        specs.append({'name': 'camera', 'node_name': CAMERA_RECORDER_NODE,
                      'out_dir': cam_dir, 'topics': list(cam['topics']),
                      'cmd': ccmd})

    if cfg.get('enable_snapshot', True) and cfg.get('snapshot_topics'):
        snap_dir = os.path.join(run_dir, 'snapshots')
        scmd = ['ros2', 'bag', 'record', '-s', 'mcap', '-o', snap_dir,
                '--node-name', SNAPSHOT_RECORDER_NODE, '--snapshot-mode',
                '--max-cache-size',
                str(int(cfg['snapshot_max_cache_mb']) * 1024 * 1024)]
        scmd += qos
        scmd += list(cfg['snapshot_topics'])  # positional topics last
        specs.append({'name': 'snapshot', 'node_name': SNAPSHOT_RECORDER_NODE,
                      'out_dir': snap_dir,
                      'topics': list(cfg['snapshot_topics']), 'cmd': scmd})
    return specs


# ── data_collection run metadata ─────────────────────────────────────────────
def disk_low(path, min_free_mb, usage=shutil.disk_usage):
    """True when the file system holding ``path`` has under ``min_free_mb``
    MB free (0 = never). A path that does not exist yet is measured at its
    nearest existing parent."""
    if not min_free_mb or min_free_mb <= 0:
        return False
    probe = os.path.abspath(path)
    while not os.path.exists(probe) and os.path.dirname(probe) != probe:
        probe = os.path.dirname(probe)
    return usage(probe).free < min_free_mb * 1024 * 1024


def parse_mapping(text, what='value'):
    """YAML/JSON mapping string (launch parameter) -> dict ('' -> {})."""
    if text is None:
        return {}
    if isinstance(text, dict):
        return dict(text)
    text = str(text).strip()
    if not text:
        return {}
    data = yaml.safe_load(text)
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError(f'{what} must be a mapping, got {type(data).__name__}')
    return data


def load_yaml_file(path):
    if not path or not os.path.isfile(path):
        return {}
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f'{path}: top level must be a mapping')
    return data


def onoff(value):
    """YAML 1.1 turns unquoted off/on into booleans (P-099); store strings."""
    if isinstance(value, bool):
        return 'on' if value else 'off'
    if value is None:
        return None
    return str(value)


def _deep_merge(base, extra):
    out = dict(base)
    for k, v in (extra or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def build_run_metadata(base_meta, profile, *, session=None, run_info=None,
                       camera_params=None, extrinsics=None, calibration=None,
                       camera_topics=None, env=None):
    """Return run_metadata for a profile run (base_meta = legacy fields).

    * ``session``: operator YAML (grass height, weather, camera model ...)
    * ``run_info``: per-run overrides from the launch (same keys, wins)
    * ``camera_params``: settings the camera launch used (device, size, fps)
    * ``extrinsics``: camera_extrinsics.compute() summary dict
      (``source``, ``camera_height_m``, ``camera_pitch_deg``) or None
    * ``calibration``: result of copy_calibration()
    """
    env = os.environ if env is None else env
    user = _deep_merge(session or {}, run_info or {})
    meta = dict(base_meta)
    meta['profile'] = profile

    cam = {}
    cam.update(user.get('camera') or {})
    for k, v in (camera_params or {}).items():
        cam.setdefault(k, v)  # an explicit session value (e.g. model) wins
    for k in CAMERA_ONOFF_FIELDS:
        if k in cam:
            cam[k] = onoff(cam[k])
    ordered_cam = {}
    for k in ('model', 'serial', 'device', 'width', 'height', 'fps_native',
              'fps_recorded', 'format', 'autofocus', 'exposure'):
        ordered_cam[k] = cam.pop(k, None)
    ordered_cam.update(cam)
    meta['camera'] = ordered_cam

    ext = extrinsics or {}
    meta['camera_height_m'] = user.get('camera_height_m',
                                       ext.get('camera_height_m'))
    meta['camera_pitch_deg'] = user.get('camera_pitch_deg',
                                        ext.get('camera_pitch_deg'))
    meta['extrinsics_source'] = user.get('extrinsics_source',
                                         ext.get('source'))
    calib = dict(calibration or {})
    date = user.get('calibration_date') or calib.pop('date', None)
    meta['calibration'] = {
        'intrinsics_file': calib.pop('intrinsics_file', None),
        'extrinsics_file': calib.pop('extrinsics_file', None),
        'date': str(date) if date else None,
        **calib,
    }
    for k in SESSION_FIELDS:
        v = user.get(k)
        meta[k] = onoff(v) if k in ONOFF_FIELDS else v
    if meta.get('notes') is None:
        meta['notes'] = ''
    if camera_topics:
        meta['camera_topics'] = list(camera_topics)
    meta['build'] = {
        'version': env.get('MOWER_VERSION', ''),
        'git_sha': env.get('MOWER_GIT_SHA', ''),
        'image': env.get('MOWER_IMAGE', ''),
    }
    known = set(meta) | {'camera', 'calibration_date'}
    extra = {k: v for k, v in user.items() if k not in known}
    if extra:
        meta['session_extra'] = extra
    return meta


def copy_calibration(run_dir, intrinsics_path, extrinsics_path,
                     derived=None, intrinsics_placeholder=False):
    """Copy the calibration into <run>/calib and describe what was copied.

    The extrinsics file is copied with an extra ``derived`` block (what the
    TF publisher sent), so a run is self-describing even if the robot file
    changes later. Missing sources are reported, not fatal: recording must
    not stop because a file is absent (check_run flags it instead).
    """
    calib_dir = os.path.join(run_dir, 'calib')
    os.makedirs(calib_dir, exist_ok=True)
    info = {'intrinsics_file': None, 'extrinsics_file': None,
            'intrinsics_source': None, 'problems': []}
    if intrinsics_path and os.path.isfile(intrinsics_path):
        shutil.copyfile(intrinsics_path, os.path.join(calib_dir, INTRINSICS_NAME))
        info['intrinsics_file'] = f'calib/{INTRINSICS_NAME}'
        info['intrinsics_source'] = 'placeholder' if intrinsics_placeholder \
            else 'calibrated'
    else:
        info['problems'].append(f'intrinsics not found: {intrinsics_path!r}')
    if extrinsics_path and os.path.isfile(extrinsics_path):
        with open(extrinsics_path) as f:
            ext = yaml.safe_load(f) or {}
        if derived:
            ext['derived'] = derived
        with open(os.path.join(calib_dir, EXTRINSICS_NAME), 'w') as f:
            f.write('# 由 mower_recorder 在開錄時從 '
                    f'{os.path.basename(extrinsics_path)} 複製，並加上 derived 區塊\n')
            yaml.safe_dump(ext, f, allow_unicode=True, sort_keys=False)
        info['extrinsics_file'] = f'calib/{EXTRINSICS_NAME}'
        if isinstance(ext, dict) and ext.get('date'):
            info['date'] = str(ext.get('date'))
    else:
        info['problems'].append(f'extrinsics not found: {extrinsics_path!r}')
    if not info['problems']:
        info.pop('problems')
    return info


def json_param(text, what):
    """Launch passes structured data as JSON strings; '' -> {}."""
    if not text:
        return {}
    data = json.loads(text)
    if not isinstance(data, dict):
        raise ValueError(f'{what} must be a JSON object')
    return data
