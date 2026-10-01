"""Upload ordering, resume and manifest content (fake S3 client + moto)."""
import datetime
import json
import os

import boto3
import pytest
from moto import mock_aws

import synth_run
from mower_recorder import run_manifest as rm

TZ8 = datetime.timezone(datetime.timedelta(hours=8))
BUCKET = 'mower-test-bucket'


class FakeS3:
    """Records every call; optionally fails the n-th upload_file."""

    def __init__(self, fail_on_upload=None, objects=None):
        self.calls = []
        self.objects = {} if objects is None else objects
        self.fail_on_upload = fail_on_upload
        self.uploads = 0

    def upload_file(self, path, bucket, key, ExtraArgs=None):
        self.uploads += 1
        if self.fail_on_upload and self.uploads == self.fail_on_upload:
            self.calls.append(('upload_file_failed', key))
            raise ConnectionError('WiFi dropped')
        with open(path, 'rb') as f:
            self.objects[key] = f.read()
        self.calls.append(('upload_file', key))

    def put_object(self, Bucket, Key, Body, ContentType=None):
        self.objects[Key] = Body
        self.calls.append(('put_object', Key))

    def head_object(self, Bucket, Key):
        self.calls.append(('head_object', Key))
        if Key not in self.objects:
            err = Exception('404')
            err.response = {'Error': {'Code': '404'},
                            'ResponseMetadata': {'HTTPStatusCode': 404}}
            raise err
        return {'ContentLength': len(self.objects[Key])}


@pytest.fixture
def run(tmp_path):
    return synth_run.make_run(str(tmp_path / 'bags'))


def _base(run):
    return rm.run_key('bags', 'mower-03', os.path.basename(run))


def _upload(client, run):
    return rm.upload_run(client, BUCKET, _base(run), run,
                         os.path.basename(run), 'mower-03', tz=TZ8)


def test_manifest_is_uploaded_last_and_complete(run):
    s3 = FakeS3()
    res = _upload(s3, run)
    writes = [c for c in s3.calls if c[0] in ('upload_file', 'put_object')]
    assert writes[-1] == ('put_object', f'{_base(run)}/_manifest.json')
    assert all(c[0] == 'upload_file' for c in writes[:-1])
    files = rm.list_run_files(run)
    assert res == {'uploaded': len(files), 'skipped': 0, 'files': len(files),
                   'manifest_key': f'{_base(run)}/_manifest.json'}

    man = json.loads(s3.objects[f'{_base(run)}/_manifest.json'])
    assert list(man) == ['schema', 'robot_id', 'run_id', 'profile', 'started_at',
                         'ended_at', 'ros_distro', 'git_sha', 'files', 'topics',
                         'camera', 'camera_height_m']
    assert man['schema'] == 'mower.run_manifest/v1'
    assert man['profile'] == 'data_collection'
    assert man['started_at'].endswith('+08:00')
    assert man['git_sha'] == 'abc123'          # build.git_sha fallback
    assert man['camera'] == {'width': 1280, 'height': 720, 'fps_recorded': 10,
                             'format': 'jpeg'}
    assert man['camera_height_m'] == 0.62
    assert man['topics']['/camera/front/image_raw/compressed'] == {
        'type': 'sensor_msgs/msg/CompressedImage', 'count': 60}
    assert man['topics']['/tf_static']['count'] == 6   # both bags summed
    keys = {f['key'] for f in man['files']}
    assert keys == {f'{_base(run)}/{r}' for r in files}
    for f in man['files']:
        rel = f['key'].split(os.path.basename(run) + '/', 1)[1]
        assert f['size'] == os.path.getsize(os.path.join(run, rel))
        assert f['sha256'] == rm.sha256_file(os.path.join(run, rel))
    # local copies: manifest + marker; bookkeeping files are never uploaded
    assert os.path.isfile(os.path.join(run, '_manifest.json'))
    assert json.load(open(os.path.join(run, '.uploaded')))['files'] == len(files)
    assert not any('.upload_state' in k or k.endswith('.uploaded')
                   for k in s3.objects)


def test_manifest_lists_every_uploaded_file(run):
    """Coordinator rule: files = everything uploaded (params/, snapshots/,
    calib/, both MCAP groups + metadata.yaml, run_metadata.yaml), never the
    manifest itself or local bookkeeping."""
    os.makedirs(os.path.join(run, 'params'))
    with open(os.path.join(run, 'params', 'ekf_filter_node_map.yaml'), 'w') as f:
        f.write('ekf_filter_node_map:\n  ros__parameters: {two_d_mode: true}\n')
    open(os.path.join(run, 'params', 'empty.yaml'), 'w').close()  # size 0 is fine
    snap = os.path.join(run, 'snapshots')
    os.makedirs(snap)
    import shutil
    shutil.copy(os.path.join(run, 'bag', 'bag_0.mcap'),
                os.path.join(snap, 'snapshots_0.mcap'))
    shutil.copy(os.path.join(run, 'bag', 'metadata.yaml'),
                os.path.join(snap, 'metadata.yaml'))
    with open(os.path.join(snap, 'metadata.yaml')) as f:
        text = f.read().replace('bag_0.mcap', 'snapshots_0.mcap')
    with open(os.path.join(snap, 'metadata.yaml'), 'w') as f:
        f.write(text)
    open(os.path.join(run, '.DS_Store'), 'w').write('x')         # never uploaded

    s3 = FakeS3()
    _upload(s3, run)
    man_key = f'{_base(run)}/_manifest.json'
    man = json.loads(s3.objects[man_key])
    listed = {f['key'] for f in man['files']}
    uploaded = set(s3.objects) - {man_key}
    assert listed == uploaded
    rels = {k.split(os.path.basename(run) + '/', 1)[1] for k in listed}
    assert {'run_metadata.yaml', 'calib/camera_front.yaml', 'calib/extrinsics.yaml',
            'params/ekf_filter_node_map.yaml', 'params/empty.yaml',
            'snapshots/snapshots_0.mcap', 'snapshots/metadata.yaml',
            'bag/bag_0.mcap', 'bag/metadata.yaml',
            'camera/camera_0.mcap', 'camera/metadata.yaml'} == rels
    for f in man['files']:
        assert set(f) == {'key', 'size', 'sha256'} and len(f['sha256']) == 64
    empty = [f for f in man['files'] if f['key'].endswith('params/empty.yaml')][0]
    assert empty['size'] == 0
    assert empty['sha256'] == ('e3b0c44298fc1c149afbf4c8996fb924'
                               '27ae41e4649b934ca495991b7852b855')


def test_interrupted_upload_resumes(run):
    broken = FakeS3(fail_on_upload=3)
    with pytest.raises(ConnectionError):
        _upload(broken, run)
    assert not any(k.endswith('_manifest.json') for k in broken.objects)
    assert not os.path.exists(os.path.join(run, '.uploaded'))
    state = json.load(open(os.path.join(run, rm.UPLOAD_STATE)))
    assert len(state['files']) == 2 and not state['manifest_uploaded']

    healthy = FakeS3(objects=broken.objects)   # same bucket contents
    res = _upload(healthy, run)
    n = len(rm.list_run_files(run))
    assert (res['uploaded'], res['skipped']) == (n - 2, 2)
    uploaded = [c[1] for c in healthy.calls if c[0] == 'upload_file']
    assert len(uploaded) == n - 2
    assert healthy.calls[-1] == ('put_object', f'{_base(run)}/_manifest.json')


def test_state_claims_uploaded_but_object_missing(run):
    s3 = FakeS3()
    _upload(s3, run)
    victim = f'{_base(run)}/camera/camera_0.mcap'
    del s3.objects[victim]
    os.remove(os.path.join(run, '.uploaded'))
    s3.calls.clear()
    res = _upload(s3, run)
    assert ('upload_file', victim) in s3.calls
    assert res['uploaded'] == 1
    assert s3.calls[-1][1].endswith('_manifest.json')


def test_changed_file_is_rehashed_and_reuploaded(run):
    s3 = FakeS3()
    _upload(s3, run)
    with open(os.path.join(run, 'run_metadata.yaml'), 'a') as f:
        f.write('display_name: yard east\n')
    s3.calls.clear()
    res = _upload(s3, run)
    assert res['uploaded'] == 1
    man = json.loads(s3.objects[f'{_base(run)}/_manifest.json'])
    meta = [f for f in man['files'] if f['key'].endswith('run_metadata.yaml')][0]
    assert meta['sha256'] == rm.sha256_file(os.path.join(run, 'run_metadata.yaml'))


def test_unfinished_bag_blocks_before_any_request(run):
    os.remove(os.path.join(run, 'camera', 'metadata.yaml'))
    s3 = FakeS3()
    with pytest.raises(rm.RunNotReady) as e:
        _upload(s3, run)
    assert 'reindex' in str(e.value)
    assert s3.calls == []


def test_legacy_run_without_camera(tmp_path):
    run = synth_run.make_run(str(tmp_path))
    meta_path = os.path.join(run, 'run_metadata.yaml')
    open(meta_path, 'w').write('run_id: x\nrobot_id: mower\ngit: {sha: deadbeef}\n')
    man = rm.build_manifest(run, 'x', 'mower', 'bags/mower/x',
                            {'run_metadata.yaml': {'size': 1, 'sha256': 'a'}})
    assert man['profile'] == 'default'
    assert man['camera'] is None and man['camera_height_m'] is None
    assert man['git_sha'] == 'deadbeef'


@mock_aws
def test_moto_end_to_end_and_delete_order(run):
    s3 = boto3.client('s3', region_name='us-east-1')
    s3.create_bucket(Bucket=BUCKET)
    _upload(s3, run)
    ok, problems, man = rm.verify_remote(s3, BUCKET, _base(run))
    assert ok, problems
    assert man['run_id'] == os.path.basename(run)
    head = s3.head_object(Bucket=BUCKET, Key=f'{_base(run)}/bag/bag_0.mcap')
    assert head['Metadata']['sha256'] == rm.sha256_file(
        os.path.join(run, 'bag', 'bag_0.mcap'))

    deleted = []
    real_delete = s3.delete_object

    def spy(**kw):
        deleted.append(kw['Key'])
        return real_delete(**kw)
    s3.delete_object = spy
    n = rm.delete_run_objects(s3, BUCKET, _base(run))
    assert deleted[0].endswith('_manifest.json')
    assert n == len(rm.list_run_files(run)) + 1
    assert s3.list_objects_v2(Bucket=BUCKET).get('KeyCount') == 0


@mock_aws
def test_moto_verify_detects_missing_object(run):
    s3 = boto3.client('s3', region_name='us-east-1')
    s3.create_bucket(Bucket=BUCKET)
    _upload(s3, run)
    s3.delete_object(Bucket=BUCKET, Key=f'{_base(run)}/camera/camera_0.mcap')
    ok, problems, _ = rm.verify_remote(s3, BUCKET, _base(run))
    assert not ok and 'camera/camera_0.mcap' in problems[0]
    ok, problems, man = rm.verify_remote(s3, BUCKET, 'bags/mower-03/none')
    assert not ok and man is None


def test_run_key():
    assert rm.run_key('bags/', 'mower-03', 'r1') == 'bags/mower-03/r1'
    assert rm.run_key('', 'm', 'r') == 'bags/m/r'
