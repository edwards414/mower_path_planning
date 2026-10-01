"""Recorder command lines and data_collection run metadata."""
import os

import yaml

from mower_recorder import record_profile as rp

CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      'config')


def _legacy_commands(cfg, run_dir, qos_path):
    """Verbatim logic of recorder_manager_node before the profile refactor."""
    bag_dir = os.path.join(run_dir, 'bag')
    cmd = ['ros2', 'bag', 'record', '-s', 'mcap', '-o', bag_dir,
           '--node-name', 'mower_full_recorder']
    if cfg.get('compression', True):
        cmd += ['--compression-mode', 'message',
                '--compression-format', 'zstd']
    cmd += ['--max-bag-size',
            str(int(cfg['max_bag_size_mb']) * 1024 * 1024)]
    cmd += ['--max-bag-duration', str(int(cfg['max_bag_duration_s']))]
    if qos_path and os.path.isfile(qos_path):
        cmd += ['--qos-profile-overrides-path', qos_path]
    cmd += list(cfg['topics'])
    out = [cmd]
    if cfg.get('enable_snapshot', True) and cfg.get('snapshot_topics'):
        snap_dir = os.path.join(run_dir, 'snapshots')
        scmd = ['ros2', 'bag', 'record', '-s', 'mcap', '-o', snap_dir,
                '--node-name', 'mower_snapshot_recorder', '--snapshot-mode',
                '--max-cache-size',
                str(int(cfg['snapshot_max_cache_mb']) * 1024 * 1024)]
        if qos_path and os.path.isfile(qos_path):
            scmd += ['--qos-profile-overrides-path', qos_path]
        scmd += list(cfg['snapshot_topics'])
        out.append(scmd)
    return out


def test_always_on_profile_is_unchanged():
    qos = os.path.join(CONFIG, 'qos_overrides.yaml')
    cfg = rp.load_config(os.path.join(CONFIG, 'record_topics.yaml'))
    specs = rp.recorder_specs(cfg, '/runs/r1', qos)
    assert [s['cmd'] for s in specs] == _legacy_commands(cfg, '/runs/r1', qos)
    assert [s['name'] for s in specs] == ['full', 'snapshot']
    assert '--compression-mode' in specs[0]['cmd']      # unchanged on purpose
    assert 'profile' not in cfg


def test_data_collection_profile_commands():
    qos = os.path.join(CONFIG, 'qos_overrides.yaml')
    cfg = rp.load_config(os.path.join(CONFIG, 'data_collection.yaml'))
    specs = {s['name']: s for s in rp.recorder_specs(cfg, '/runs/r2', qos)}
    assert set(specs) == {'full', 'camera'}             # no snapshot recorder
    tele = specs['full']['cmd']
    assert tele[tele.index('--storage-preset-profile') + 1] == 'zstd_fast'
    assert '--compression-mode' not in tele             # P-089
    assert tele[tele.index('-o') + 1] == '/runs/r2/bag'
    assert tele[tele.index('--max-bag-size') + 1] == str(512 * 1024 * 1024)
    for topic in ('/gps/status', '/odometry/local',
                  '/odometry/global', '/odometry/gps', '/tf', '/tf_static',
                  '/imu/data', '/odom', '/fix', '/cmd_vel', '/rosout'):
        assert topic in tele
    cam = specs['camera']['cmd']
    assert cam[cam.index('-o') + 1] == '/runs/r2/camera'
    assert cam[cam.index('--node-name') + 1] == 'mower_camera_recorder'
    assert '--storage-preset-profile' not in cam and '--compression-mode' not in cam
    assert cam[cam.index('--max-bag-size') + 1] == '268435456'
    assert '--max-bag-duration' not in cam
    assert cam[-3:] == ['/camera/front/image_raw/compressed',
                        '/camera/front/camera_info', '/tf_static']
    assert cam[cam.index('--qos-profile-overrides-path') + 1] == qos


def test_storage_preset_wins_over_message_compression():
    cfg = dict(rp.load_config(''), topics=['/a'], compression=True,
               storage_preset_profile='zstd_fast')
    cmd = rp.recorder_specs(cfg, '/r', '')[0]['cmd']
    assert '--compression-mode' not in cmd and '--storage-preset-profile' in cmd


def test_build_run_metadata_spec_fields(tmp_path):
    session = yaml.safe_load(open(os.path.join(CONFIG, 'data_collection_session.yaml')))
    session['blade'] = False   # what an unquoted `blade: off` becomes (P-099)
    base = {'run_id': 'r', 'robot_id': 'mower-03', 'git': {'sha': 'unknown'}}
    calib = rp.copy_calibration(str(tmp_path), '', '')
    meta = rp.build_run_metadata(
        base, 'data_collection', session=session,
        run_info={'weather': 'cloudy', 'grass_height_cm': 6, 'site': 'yard2'},
        camera_params={'device': '/dev/video0', 'width': 1280, 'height': 720,
                       'fps_native': 25, 'fps_recorded': 10, 'format': 'jpeg',
                       'autofocus': 'off'},
        extrinsics={'source': 'measured', 'camera_height_m': 0.62,
                    'camera_pitch_deg': 20.0},
        calibration=calib, camera_topics=['/camera/front/image_raw/compressed'],
        env={'MOWER_GIT_SHA': 'abc', 'MOWER_VERSION': 'v1'})
    for key in ('profile', 'camera', 'camera_height_m', 'camera_pitch_deg',
                'extrinsics_source', 'calibration', 'grass_height_cm', 'weather',
                'blade', 'heading_calibration', 'notes'):
        assert key in meta, key
    assert list(meta['camera'])[:10] == ['model', 'serial', 'device', 'width',
                                         'height', 'fps_native', 'fps_recorded',
                                         'format', 'autofocus', 'exposure']
    assert meta['weather'] == 'cloudy' and meta['grass_height_cm'] == 6
    assert meta['blade'] == 'off'
    assert meta['camera']['exposure'] == 'auto'
    assert meta['build']['git_sha'] == 'abc'
    assert meta['session_extra'] == {'site': 'yard2'}
    text = yaml.safe_dump(meta, allow_unicode=True, sort_keys=False)
    assert "blade: 'off'" in text                     # stays a string for PyYAML
    assert yaml.safe_load(text)['blade'] == 'off'


def test_copy_calibration(tmp_path):
    src = tmp_path / 'src'
    src.mkdir()
    (src / 'camera_front.yaml').write_text('image_width: 1280\n')
    (src / 'extrinsics.yaml').write_text('source: measured\ndate: 2026-09-25\n')
    run = tmp_path / 'run'
    run.mkdir()
    info = rp.copy_calibration(str(run), str(src / 'camera_front.yaml'),
                               str(src / 'extrinsics.yaml'),
                               derived={'camera_height_m': 0.62})
    assert info == {'intrinsics_file': 'calib/camera_front.yaml',
                    'extrinsics_file': 'calib/extrinsics.yaml',
                    'intrinsics_source': 'calibrated', 'date': '2026-09-25'}
    ext = yaml.safe_load((run / 'calib' / 'extrinsics.yaml').read_text())
    assert ext['derived'] == {'camera_height_m': 0.62}
    missing = rp.copy_calibration(str(run), '/nope.yaml', '')
    assert len(missing['problems']) == 2


def test_parse_mapping():
    assert rp.parse_mapping('') == {}
    assert rp.parse_mapping('{weather: rain, n: 2}') == {'weather': 'rain', 'n': 2}
    try:
        rp.parse_mapping('[1, 2]')
    except ValueError:
        pass
    else:
        raise AssertionError('list must be rejected')
