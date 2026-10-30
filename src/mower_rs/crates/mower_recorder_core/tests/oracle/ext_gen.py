#!/usr/bin/env python3
"""Vectors for the extrinsics and data_collection metadata port, written by the
real Python: mower_recorder's camera_extrinsics.py and record_profile.py as the
robot image installs them (Python 3.12, PyYAML 6.0.1, xacro).

    docker run --rm --network none -v "$PWD/src/mower_rs/crates/mower_recorder_core":/x -v "$PWD/deploy/test/golden/recorder":/golden:ro \\
      --entrypoint bash mower_path_planning:ros-free-test -c \\
      'source /opt/ros/jazzy/setup.bash && source /mower_ws/install/setup.bash &&
       python3 -P /x/tests/oracle/ext_gen.py --out /x/tests/vectors/extrinsics.json'

Values are stored typed (the tags of mower_pyfmt's vectors: N, B, I as digits,
F as repr, S, L, D as pairs), so every float is compared bit for bit.

- texts: the package placeholder, robot.urdf.xacro, and real_robot.xacro as
  xacro processes it (what camera_launch.robot_reference reads);
- reference: default_reference() and urdf_reference() of both URDFs, and
  urdf_reference() errors;
- matrix: rpy_to_matrix / matrix_to_rpy / matrix_to_quaternion /
  quaternion_to_matrix, Transform composition and inverse;
- rounds: round(x, n) for n in 4, 6, 9;
- compute: validate() and compute() of the placeholder, the F3 run's
  extrinsics and seeded random measurements (valid and not), with the default
  and the real URDF reference; per result every transform, transform_dict,
  derived_block (as copy_calibration dumps it) and static_tf_arguments;
- metadata: build_run_metadata() of test_record_profile's case, the F3 run
  (checked against the run's own run_metadata.yaml) and random sessions, as
  run_metadata.yaml text;
- copy_calibration: the files it writes and the info it returns.
"""
import argparse
import copy
import json
import math
import os
import random
import shutil
import sys
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from mower_recorder import camera_extrinsics as ce
from mower_recorder import record_profile as rp

ap = argparse.ArgumentParser()
ap.add_argument('--out', required=True)
ap.add_argument('--seed', type=int, default=11)
ap.add_argument('--random', type=int, default=130)
a = ap.parse_args()
R = random.Random(a.seed)


def enc(v):
    if v is None:
        return ['N']
    if isinstance(v, bool):
        return ['B', v]
    if isinstance(v, int):
        return ['I', str(v)]
    if isinstance(v, float):
        return ['F', repr(v)]
    if isinstance(v, str):
        return ['S', v]
    if isinstance(v, (list, tuple)):
        return ['L', [enc(x) for x in v]]
    if isinstance(v, dict):
        for k in v:
            assert isinstance(k, str), k
        return ['D', [[k, enc(x)] for k, x in v.items()]]
    raise TypeError(f'cannot encode {type(v).__name__}: {v!r}')


def fl(xs):
    return [repr(float(x)) for x in xs]


def tr(t):
    return {'r': [repr(x) for row in t.rotation for x in row], 't': fl(t.translation),
            'rpy': fl(t.rpy), 'q': fl(t.quaternion)}


def dump(v):
    return yaml.safe_dump(v, allow_unicode=True, sort_keys=False)


share = get_package_share_directory('mower_recorder')
desc = get_package_share_directory('mower_description')
placeholder_text = open(os.path.join(share, 'config', 'camera_front_extrinsics.yaml'), encoding='utf-8').read()
xacro_text = open(os.path.join(desc, 'mower_robot', 'robot.urdf.xacro'), encoding='utf-8').read()
real_urdf = xacro.process_file(os.path.join(desc, 'mower_robot', 'real_robot.xacro')).toxml()
placeholder = yaml.safe_load(placeholder_text)

out = {'generator': 'tests/oracle/ext_gen.py', 'python': sys.version.split()[0], 'pyyaml': yaml.__version__,
       'texts': {'placeholder_yaml': placeholder_text, 'robot_urdf_xacro': xacro_text, 'real_robot_urdf': real_urdf},
       'values': {'placeholder': enc(placeholder)}}

refs = {'default': ce.default_reference(), 'urdf_real': ce.urdf_reference(real_urdf),
        'urdf_xacro': ce.urdf_reference(xacro_text)}
out['reference'] = {k: {'base_link_in_base_footprint': tr(v['base_link_in_base_footprint']),
                        'axle': fl(v['axle_center_in_base_footprint']), 'source': v['source']}
                    for k, v in refs.items()}


def urdf_err(xml):
    try:
        ce.urdf_reference(xml)
    except Exception as e:  # noqa: BLE001
        return f'{type(e).__name__}: {e}'
    return None


bad = [
    '<robot name="r"/>',
    '<robot><joint name="base_joint" type="fixed"><parent link="base_footprint"/><child link="base_link"/>'
    '<origin xyz="0 0 0.1" rpy="0 0 1.5"/></joint><joint name="left_wheel_joint" type="continuous">'
    '<parent link="base_link"/><child link="lw"/><origin xyz="0.1 0.2 0"/></joint></robot>',
    '<robot><joint name="base_joint" type="fixed"><parent link="base_footprint"/><child link="base_link"/>'
    '</joint><joint name="a_wheel" type="continuous"><parent link="base_link"/><child link="a"/>'
    '<origin xyz="0.1 0.2 0.3" rpy=""/></joint><!-- a comment <joint> --><joint name="b_wheel" type="continuous">'
    '<parent link="base_link"/><child link="b"/><origin xyz="-0.1 0.2 -0.3"/></joint>'
    '<joint name="caster_wheel" type="continuous"><parent link="base_link"/><child link="c"/></joint></robot>',
]
out['urdf_cases'] = []
for xml in bad:
    case = {'xml': xml, 'error': urdf_err(xml)}
    if case['error'] is None:
        ref = ce.urdf_reference(xml)
        case['ref'] = {'base_link_in_base_footprint': tr(ref['base_link_in_base_footprint']),
                       'axle': fl(ref['axle_center_in_base_footprint']), 'source': ref['source']}
    out['urdf_cases'].append(case)

# ---- matrix helpers -----------------------------------------------------------------------
mats = []
for i in range(50):
    if i < 6:
        rpy = [(0.1, -0.4, 2.0), (-1.2, 0.3, -2.9), (0.0, 1.2, 0.5), (0.0, math.pi / 2, 0.3),
               (0.2, -math.pi / 2, 0.0), (math.pi, 0.0, -math.pi)][i]
    else:
        rpy = (R.uniform(-math.pi, math.pi), R.uniform(-math.pi / 2, math.pi / 2), R.uniform(-math.pi, math.pi))
    m = ce.rpy_to_matrix(*rpy)
    q = ce.matrix_to_quaternion(m)
    t1 = ce.Transform.from_xyz_rpy((R.uniform(-1, 1), R.uniform(-1, 1), R.uniform(-1, 1)), rpy)
    t2 = ce.Transform.from_xyz_rpy((R.uniform(-1, 1), R.uniform(-1, 1), R.uniform(-1, 1)),
                                   (R.uniform(-3, 3), R.uniform(-1.5, 1.5), R.uniform(-3, 3)))
    mats.append({'rpy': fl(rpy), 'm': [repr(x) for row in m for x in row], 'rpy_back': fl(ce.matrix_to_rpy(m)),
                 'q': fl(q), 'm_from_q': [repr(x) for row in ce.quaternion_to_matrix(q) for x in row],
                 't1': tr(t1), 't2': tr(t2), 'compose': tr(t1 @ t2), 'inverse': tr(t1.inverse()),
                 'apply': fl(t1.apply((0.3, -0.2, 0.1))), 'rotate': fl(t1.rotate((0, 0, 1)))})
out['matrix'] = mats

# ---- round --------------------------------------------------------------------------------
rounds = []
specials = [0.0, -0.0, 0.5, 1.5, 2.5, 0.125, 0.375, -0.125, 1e-7, -1e-7, 4e-7, 5e-7, 123456.0000005,
            0.0000045, 0.00000045, 1.0000005, 2.675, 1e300, -1e-300, float('inf'), float('nan'),
            0.1234565, 0.12345650000000001, 0.570242448, -0.4181190615, 1.5707963267948966]
for x in specials + [R.uniform(-10, 10) for _ in range(300)] + [R.uniform(-1, 1) * 10 ** R.randint(-12, 6)
                                                               for _ in range(300)]:
    for n in (4, 6, 9):
        rounds.append([repr(x), n, repr(round(x, n))])
# exact binary ties: k / 2^m at the last kept digit's half
for k in range(1, 400, 2):
    x = k / 2 ** 7
    rounds.append([repr(x), 6, repr(round(x, 6))])
    rounds.append([repr(x), 4, repr(round(x, 4))])
out['rounds'] = rounds

# ---- compute ------------------------------------------------------------------------------
def rand_num(lo, hi):
    k = R.random()
    if k < 0.7:
        return round(R.uniform(lo, hi), R.choice([2, 3, 4, 6, 12]))
    if k < 0.8:
        v = R.randint(int(math.floor(lo)), int(math.ceil(hi)))
        return v if lo <= v <= hi else R.uniform(lo, hi)
    if k < 0.86:
        return R.uniform(lo - (hi - lo), hi + (hi - lo))   # often out of range
    if k < 0.89:
        return R.choice(['0.3', True, None, [1], {'a': 1}, 'x'])
    if k < 0.91:
        return R.choice([float('nan'), float('inf'), -float('inf')])
    return R.uniform(lo, hi)


def rand_ext():
    ext = copy.deepcopy(placeholder)
    ext['source'] = R.choice(['measured', 'measured', 'measured', 'cad', 'placeholder', ' measured ', 'guess', None, 1])
    if R.random() < 0.1:
        ext['schema'] = R.choice(['other/v2', None, 3])
    if R.random() < 0.05:
        del ext['schema']
    cam = ext['camera']
    for key, (lo, hi) in (('forward_of_axle_m', (-1.0, 2.0)), ('left_of_center_m', (-0.3, 0.3)),
                          ('height_m', (0.05, 2.5)), ('roll_deg', (-20.0, 20.0)), ('pitch_deg', (-5.0, 80.0)),
                          ('yaw_deg', (-45.0, 45.0))):
        if R.random() < 0.7:
            cam[key] = rand_num(lo, hi)
        if R.random() < 0.02:
            del cam[key]
    refd = ext['reference']
    if R.random() < 0.6:
        refd['axle_height_m'] = rand_num(0.02, 0.5)
    k = R.random()
    if k < 0.2:
        refd['axle_center_in_base_footprint_m'] = [rand_num(-0.3, 0.3) if R.random() < 0.9 else R.randint(-1, 1)
                                                  for _ in range(3)]
    elif k < 0.25:
        refd['axle_center_in_base_footprint_m'] = R.choice([[0.1, 0.2], 'x', 5, ['0.1', 0, 1e-3]])
    if R.random() < 0.04:
        ext['reference'] = R.choice([None, [], 'x', {}, [1, 2]])
    if R.random() < 0.03:
        ext['camera'] = R.choice([None, 'cam', [1]])
    if R.random() < 0.01:
        return R.choice([None, [ext], 'text'])
    return ext


f3_dir = '/golden/F3/MW-E5FIX_20261030T080906'
f3_calib_text = open(os.path.join(f3_dir, 'calib', 'extrinsics.yaml'), encoding='utf-8').read()
f3_ext = yaml.safe_load(f3_calib_text)
f3_ext.pop('derived')

inputs = [('placeholder', placeholder), ('f3', f3_ext)] + [(f'random{i}', rand_ext()) for i in range(a.random)]
cases = []
for idx, (name, ext) in enumerate(inputs):
    for refname in (('none', 'urdf_real') if idx < 2 else (('none', 'urdf_real')[idx % 2],)):
        ref = None if refname == 'none' else refs[refname]
        case = {'id': f'{name}/{refname}', 'ext': enc(ext), 'ref': refname}
        errs, warns = ce.validate(ext)
        case['validate'] = [errs, warns]
        try:
            res = ce.compute(ext, ref)
        except (ValueError, TypeError) as e:
            case['error'] = [type(e).__name__, str(e)]
            cases.append(case)
            continue
        case['error'] = None
        r = {k: res[k] for k in ('source', 'warnings', 'reference_source')}
        r['axle'] = fl(res['axle_center_in_base_footprint'])
        for k in ('base_footprint_height_m', 'camera_height_m', 'camera_pitch_deg'):
            r[k] = repr(res[k])
        for k in ('base_footprint_to_camera_link', 'base_link_to_camera_link', 'camera_link_to_optical',
                  'base_footprint_to_optical'):
            r[k] = tr(res[k])
        case['result'] = r
        block = ce.derived_block(res)
        case['derived_yaml'] = dump(block)
        case['static_tf'] = {
            'base_link_to_camera_link': ce.static_tf_arguments(res['base_link_to_camera_link'], ce.BASE_LINK,
                                                               ce.CAMERA_LINK),
            'camera_link_to_optical': ce.static_tf_arguments(res['camera_link_to_optical'], ce.CAMERA_LINK,
                                                             ce.OPTICAL_FRAME),
            'base_footprint_to_optical': ce.static_tf_arguments(res['base_footprint_to_optical'],
                                                                ce.BASE_FOOTPRINT, ce.OPTICAL_FRAME),
        }
        cases.append(case)
out['compute'] = cases

# ---- copy_calibration -------------------------------------------------------------------------
INTR = 'image_width: 80\nimage_height: 60\n# a comment the copy keeps\ncamera_name: camera_front\n'
cc = []
work = tempfile.mkdtemp()
ext_texts = [
    ('measured', 'source: measured\ndate: "2026-09-25"\nreference:\n  axle_height_m: 0.09\n'),
    ('no_date', 'source: measured\ncamera: {height_m: 0.6}\n'),
    ('empty', ''),
    ('nulldoc', '~\n'),
    ('unicode', 'source: measured\nmeasured_by: "王小明"\nnote: "a\\tb"\ndate: ""\n'),
    ('f3', f3_calib_text.split('\n', 1)[1].split('\nderived:', 1)[0] + '\n'),
]
derived_opts = [None, {}, {'camera_height_m': 0.62}, ce.derived_block(ce.compute(placeholder))]
n = 0
for intr in (INTR, None):
    for ename, etext in ext_texts + [('missing', None)]:
        for d in derived_opts:
            for ph in (False, True):
                if R.random() < 0.6 and not (ename == 'f3' and d is derived_opts[3]):
                    continue
                n += 1
                case_dir = os.path.join(work, str(n))
                os.makedirs(os.path.join(case_dir, 'src'))
                run = os.path.join(case_dir, 'run')
                os.makedirs(run)
                ipath = os.path.join(case_dir, 'src', 'cam_intrinsics.yaml') if intr is not None else ''
                if intr is not None:
                    open(ipath, 'w').write(intr)
                if etext is None:
                    epath = R.choice(['', '/nonexistent/extrinsics.yaml'])
                    loaded = None
                else:
                    epath = os.path.join(case_dir, 'src', f'ext_{ename}.yaml')
                    open(epath, 'w', encoding='utf-8').write(etext)
                    loaded = yaml.safe_load(etext)
                info = rp.copy_calibration(run, ipath, epath, derived=copy.deepcopy(d), intrinsics_placeholder=ph)
                files = {}
                for f in ('camera_front.yaml', 'extrinsics.yaml'):
                    p = os.path.join(run, 'calib', f)
                    files[f] = open(p, encoding='utf-8').read() if os.path.exists(p) else None
                cc.append({'id': f'{ename}/{n}', 'intrinsics': intr, 'extrinsics_name': os.path.basename(epath)
                           if etext is not None else None, 'extrinsics_text': etext,
                           'extrinsics_loaded': enc(loaded), 'missing_extrinsics_path': epath if etext is None else None,
                           'derived': enc(d), 'placeholder': ph, 'info': enc(info), 'files': files})
shutil.rmtree(work)
out['copy_calibration'] = cc

# ---- build_run_metadata -------------------------------------------------------------------
meta_cases = []
session_t = yaml.safe_load(open(os.path.join(share, 'config', 'data_collection_session.yaml')))


def meta_case(cid, base, profile, **kw):
    env = kw.pop('env', {})
    meta = rp.build_run_metadata(base, profile, env=env, **kw)
    meta_cases.append({'id': cid, 'base': enc(base), 'profile': profile,
                       **{k: enc(v) for k, v in kw.items()}, 'env': env, 'yaml': dump(meta)})


# test_record_profile.test_build_run_metadata_spec_fields
s = copy.deepcopy(session_t)
s['blade'] = False
meta_case('test_build_run_metadata_spec_fields', {'run_id': 'r', 'robot_id': 'mower-03', 'git': {'sha': 'unknown'}},
          'data_collection', session=s, run_info={'weather': 'cloudy', 'grass_height_cm': 6, 'site': 'yard2'},
          camera_params={'device': '/dev/video0', 'width': 1280, 'height': 720, 'fps_native': 25, 'fps_recorded': 10,
                         'format': 'jpeg', 'autofocus': 'off'},
          extrinsics={'source': 'measured', 'camera_height_m': 0.62, 'camera_pitch_deg': 20.0},
          calibration={'intrinsics_file': None, 'extrinsics_file': None, 'intrinsics_source': None,
                       'problems': ["intrinsics not found: ''", "extrinsics not found: ''"]},
          camera_topics=['/camera/front/image_raw/compressed'], env={'MOWER_GIT_SHA': 'abc', 'MOWER_VERSION': 'v1'})

# the F3 run: its run_metadata.yaml rebuilt from its inputs
f3_meta_text = open(os.path.join(f3_dir, 'run_metadata.yaml'), encoding='utf-8').read()
f3_meta = yaml.safe_load(f3_meta_text)
base_keys = ['run_id', 'robot_id', 'start_time', 'ros_distro', 'git', 'gps_start', 'recorded_topics',
             'snapshot_topics']
f3_base = {k: f3_meta[k] for k in base_keys}
f3_session = rp.load_yaml_file('/golden/F3/session.yaml')
f3_res = ce.compute(f3_ext, ce.urdf_reference(real_urdf))
f3_derived = json.loads(json.dumps({'summary': {'source': f3_res['source'], 'camera_height_m': f3_res['camera_height_m'],
                                                'camera_pitch_deg': f3_res['camera_pitch_deg']},
                                    'block': ce.derived_block(f3_res)}))
f3_cam = json.loads(json.dumps({'device': '/dev/video0', 'width': 80, 'height': 60, 'fps_native': 25,
                                'fps_recorded': 10, 'format': 'jpeg', 'autofocus': 'off', 'driver': 'gscam',
                                'pipeline': f3_meta['camera']['pipeline']}))
f3_calib = {'intrinsics_file': 'calib/camera_front.yaml', 'extrinsics_file': 'calib/extrinsics.yaml',
            'intrinsics_source': 'calibrated', 'date': '2026-10-30'}
f3_env = {'MOWER_VERSION': 'f3f4630', 'MOWER_GIT_SHA': 'f3f4630eb97537f93fb454a190ede03e0e346066',
          'MOWER_IMAGE': 'ghcr.io/edwards414/mower_path_planning:main'}
meta_case('f3_run', f3_base, 'data_collection', session=f3_session, run_info={}, camera_params=f3_cam,
          extrinsics=f3_derived['summary'], calibration=f3_calib,
          camera_topics=['/camera/front/image_raw/compressed', '/camera/front/camera_info', '/tf_static'], env=f3_env)
assert meta_cases[-1]['yaml'] == f3_meta_text, 'the F3 run_metadata.yaml is not rebuilt'
# and its calib/extrinsics.yaml as copy_calibration wrote it
assert '# 由 mower_recorder 在開錄時從 extrinsics.yaml 複製，並加上 derived 區塊\n' + dump(
    dict(f3_ext, derived=f3_derived['block'])) == f3_calib_text
out['f3'] = {'ext': enc(f3_ext), 'derived': enc(f3_derived), 'calib_extrinsics_yaml': f3_calib_text,
             'run_metadata_yaml': f3_meta_text}


def rand_scalar():
    return R.choice(['sunny', 'off', 'on', False, True, None, '', 6, 7.5, '王', 'a: b', '2026-10-30', 'yes'])


for i in range(70):
    sess = {}
    for k in ('grass_height_cm', 'weather', 'blade', 'heading_calibration', 'notes', 'calibration_date',
              'camera_height_m', 'extrinsics_source', 'operator', 'site'):
        if R.random() < 0.5:
            sess[k] = rand_scalar()
    if R.random() < 0.6:
        sess['camera'] = {k: rand_scalar() for k in R.sample(['model', 'serial', 'exposure', 'autofocus', 'lens',
                                                              'device'], R.randint(0, 4))}
    run_info = {k: rand_scalar() for k in R.sample(['weather', 'notes', 'blade', 'grass_height_cm', 'extra1'],
                                                   R.randint(0, 3))}
    if R.random() < 0.3:
        run_info['camera'] = {'model': rand_scalar(), 'gain': R.randint(0, 9)}
    cam = R.choice([None, {}, {'device': '/dev/video0', 'width': 1280, 'height': 720, 'fps_native': 25,
                               'fps_recorded': R.choice([10, 7.5]), 'format': 'jpeg',
                               'autofocus': R.choice(['off', None, True]), 'driver': 'gscam', 'pipeline': 'v4l2src'}])
    ext = R.choice([None, {}, {'source': 'measured', 'camera_height_m': R.uniform(0.3, 1), 'camera_pitch_deg': 17.5},
                    {'source': 'placeholder'}])
    calib = R.choice([None, {}, {'intrinsics_file': 'calib/camera_front.yaml', 'extrinsics_file': None,
                                 'intrinsics_source': 'placeholder', 'problems': ['x']},
                      {'intrinsics_file': 'calib/camera_front.yaml', 'extrinsics_file': 'calib/extrinsics.yaml',
                       'intrinsics_source': 'calibrated', 'date': R.choice(['2026-09-25', '', 'x'])}])
    base = {'run_id': f'r{i}', 'robot_id': 'MW-1', 'start_time': '2026-10-30T08:09:06', 'git': {'sha': 'unknown'},
            'gps_start': R.choice([None, {'lat': 25.03, 'lon': 121.5, 'alt': float('nan')}])}
    if R.random() < 0.2:
        base['profile'] = 'old'
    topics = R.choice([None, [], ['/camera/front/image_raw/compressed', '/tf_static']])
    env = R.choice([{}, {'MOWER_VERSION': 'v9', 'MOWER_IMAGE': 'img'}])
    meta_case(f'random{i}', base, R.choice(['data_collection', 'x']), session=R.choice([sess, None]),
              run_info=R.choice([run_info, None]), camera_params=cam, extrinsics=ext, calibration=calib,
              camera_topics=topics, env=env)
out['metadata'] = meta_cases

# onoff
out['onoff'] = [[enc(v), enc(rp.onoff(v))] for v in (True, False, None, 'off', 'on', '', 0, 1.5, [1], {'a': 1})]

with open(a.out, 'w', encoding='ascii') as f:
    json.dump(out, f, ensure_ascii=True, separators=(',', ':'))
    f.write('\n')
print(f"{len(out['matrix'])} matrix, {len(out['rounds'])} rounds, {len(out['compute'])} compute "
      f"({sum(1 for c in out['compute'] if c['error'] is None)} computed), {len(out['copy_calibration'])} copy_calibration, "
      f"{len(out['metadata'])} metadata cases; {os.path.getsize(a.out)} bytes")
