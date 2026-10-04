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
"""Run upload to R2 with a completion marker (``_manifest.json``).

Contract (doc/Robot_Recording_R2_Pipeline_Spec.md 2 / 2.1):

* every file of the run is uploaded first, ``_manifest.json`` strictly last;
  the manifest lists ``key``/``size``/``sha256`` of every file plus the topic
  counts of each rosbag ``metadata.yaml`` — consumers only trust runs that
  have a manifest;
* an interrupted upload resumes: ``<run>/.upload_state.json`` remembers what
  already reached R2 (size + mtime + sha256), and before the manifest is
  written every object is re-checked with HEAD;
* deleting a run on R2 removes the manifest first, so a half-deleted run is
  never mistaken for a complete one.

Pure Python + the S3 client passed in (boto3 / moto / a fake in tests).
"""
import datetime
import hashlib
import json
import os
import tempfile

import yaml

MANIFEST_NAME = '_manifest.json'
MANIFEST_SCHEMA = 'mower.run_manifest/v1'
UPLOADED_MARKER = '.uploaded'
UPLOAD_STATE = '.upload_state.json'
BAG_METADATA = 'metadata.yaml'
STATE_VERSION = 1
HASH_CHUNK = 4 * 1024 * 1024


class RunNotReady(Exception):
    """The run cannot be uploaded yet (e.g. a bag was not finalized)."""


# ── local run inspection ─────────────────────────────────────────────────────
def list_run_files(run_dir):
    """Relative POSIX paths of every file that belongs on R2, sorted.

    Dot files / dot directories (upload bookkeeping) and the manifest itself
    are excluded.
    """
    out = []
    for dirpath, dirnames, files in os.walk(run_dir):
        dirnames[:] = sorted(d for d in dirnames if not d.startswith('.'))
        for f in files:
            if f.startswith('.'):
                continue
            rel = os.path.relpath(os.path.join(dirpath, f), run_dir)
            rel = rel.replace(os.sep, '/')
            if rel == MANIFEST_NAME:
                continue
            out.append(rel)
    return sorted(out)


def bag_dirs(files):
    """Directories (relative) that contain rosbag2 storage files."""
    dirs = set()
    for rel in files:
        if rel.endswith(('.mcap', '.db3', '.mcap.zstd', '.db3.zstd')):
            dirs.add(rel.rsplit('/', 1)[0] if '/' in rel else '.')
    return sorted(dirs)


def read_bag_metadata(path):
    """Parse a rosbag2 metadata.yaml (Jazzy version 9, older ones too)."""
    with open(path) as f:
        doc = yaml.safe_load(f) or {}
    info = doc.get('rosbag2_bagfile_information') or {}
    topics = {}
    for entry in info.get('topics_with_message_count') or []:
        md = entry.get('topic_metadata') or {}
        name = md.get('name')
        if not name:
            continue
        topics[name] = {'type': md.get('type', ''),
                        'count': int(entry.get('message_count') or 0)}
    start = (info.get('starting_time') or {}).get('nanoseconds_since_epoch')
    duration = (info.get('duration') or {}).get('nanoseconds')
    return {
        'version': info.get('version'),
        'storage_identifier': info.get('storage_identifier', ''),
        'start_ns': int(start) if start is not None else None,
        'duration_ns': int(duration) if duration is not None else None,
        'message_count': int(info.get('message_count') or 0),
        'topics': topics,
        'relative_file_paths': list(info.get('relative_file_paths') or []),
        'compression_mode': str(info.get('compression_mode') or ''),
        'compression_format': str(info.get('compression_format') or ''),
        'ros_distro': str(info.get('ros_distro') or ''),
    }


def check_ready(run_dir, files=None):
    """Problems that must block an upload (empty list = ready)."""
    files = list_run_files(run_dir) if files is None else files
    problems = []
    for d in bag_dirs(files):
        meta = os.path.join(run_dir, d, BAG_METADATA)
        if not os.path.isfile(meta):
            problems.append(
                f'{d}/ 沒有 {BAG_METADATA}（錄製未正常結束？先執行 '
                f'`ros2 bag reindex {d} -s mcap`）')
            continue
        try:
            info = read_bag_metadata(meta)
        except Exception as e:  # noqa: BLE001
            problems.append(f'{d}/{BAG_METADATA} 無法解析：{e}')
            continue
        for rel in info['relative_file_paths']:
            if not os.path.isfile(os.path.join(run_dir, d, rel)):
                problems.append(f'{d}/{rel} 列在 metadata.yaml 但檔案不存在')
    return problems


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        while True:
            chunk = f.read(HASH_CHUNK)
            if not chunk:
                break
            h.update(chunk)
    return h.hexdigest()


def read_run_metadata(run_dir):
    path = os.path.join(run_dir, 'run_metadata.yaml')
    if not os.path.isfile(path):
        return {}
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return data if isinstance(data, dict) else {}


def run_key(prefix, robot_id, run_id):
    prefix = (prefix or 'bags').strip('/')
    return f'{prefix}/{robot_id}/{run_id}'


def _iso(ns, tz):
    if ns is None:
        return None
    dt = datetime.datetime.fromtimestamp(ns / 1e9, tz=datetime.timezone.utc)
    dt = dt.astimezone(tz) if tz is not None else dt.astimezone()
    return dt.isoformat(timespec='seconds')


def build_manifest(run_dir, run_id, robot_id, base_key, file_entries,
                   run_meta=None, tz=None):
    """Assemble the manifest dict (spec 2.1).

    ``file_entries``: {relpath: {'size': int, 'sha256': str}}.
    Topics and counts come from every rosbag ``metadata.yaml`` of the run;
    start/end are the union of the bags' time ranges.
    """
    run_meta = run_meta if run_meta is not None else read_run_metadata(run_dir)
    topics = {}
    starts, ends = [], []
    distro = ''
    for d in bag_dirs(list(file_entries)):
        meta_path = os.path.join(run_dir, d, BAG_METADATA)
        if not os.path.isfile(meta_path):
            continue
        info = read_bag_metadata(meta_path)
        distro = distro or info['ros_distro']
        if info['start_ns'] is not None:
            starts.append(info['start_ns'])
            ends.append(info['start_ns'] + (info['duration_ns'] or 0))
        for name, t in info['topics'].items():
            cur = topics.setdefault(name, {'type': t['type'], 'count': 0})
            cur['count'] += t['count']

    git = run_meta.get('git') if isinstance(run_meta.get('git'), dict) else {}
    git_sha = git.get('sha') or ''
    if not git_sha or git_sha == 'unknown':
        build = run_meta.get('build') if isinstance(run_meta.get('build'), dict) else {}
        git_sha = build.get('git_sha') or git_sha or 'unknown'

    camera = run_meta.get('camera')
    if isinstance(camera, dict):
        camera = {k: camera.get(k) for k in
                  ('width', 'height', 'fps_recorded', 'format')}
    else:
        camera = None

    started = _iso(min(starts), tz) if starts else run_meta.get('start_time')
    ended = _iso(max(ends), tz) if ends else None
    return {
        'schema': MANIFEST_SCHEMA,
        'robot_id': robot_id,
        'run_id': run_id,
        'profile': run_meta.get('profile') or 'default',
        'started_at': started,
        'ended_at': ended,
        'ros_distro': run_meta.get('ros_distro') or distro,
        'git_sha': git_sha,
        'files': [
            {'key': f'{base_key}/{rel}', 'size': int(e['size']),
             'sha256': e['sha256']}
            for rel, e in sorted(file_entries.items())
        ],
        'topics': dict(sorted(topics.items())),
        'camera': camera,
        'camera_height_m': run_meta.get('camera_height_m'),
    }


def _atomic_write(path, data):
    d = os.path.dirname(path) or '.'
    fd, tmp = tempfile.mkstemp(prefix='.tmp_', dir=d)
    try:
        with os.fdopen(fd, 'wb') as f:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def load_state(run_dir):
    path = os.path.join(run_dir, UPLOAD_STATE)
    try:
        with open(path) as f:
            state = json.load(f)
        if state.get('version') == STATE_VERSION:
            return state
    except (OSError, ValueError):
        pass
    return {'version': STATE_VERSION, 'bucket': None, 'base_key': None,
            'files': {}, 'manifest_uploaded': False}


def save_state(run_dir, state):
    _atomic_write(os.path.join(run_dir, UPLOAD_STATE),
                  json.dumps(state, indent=1, sort_keys=True).encode())


def _is_not_found(exc):
    resp = getattr(exc, 'response', None) or {}
    code = str((resp.get('Error') or {}).get('Code', ''))
    status = (resp.get('ResponseMetadata') or {}).get('HTTPStatusCode')
    return code in ('404', 'NoSuchKey', 'NotFound') or status == 404


def remote_size(client, bucket, key):
    """ContentLength of an object, or None if it does not exist."""
    try:
        return int(client.head_object(Bucket=bucket, Key=key)['ContentLength'])
    except Exception as e:  # noqa: BLE001
        if _is_not_found(e):
            return None
        raise


def upload_run(client, bucket, base_key, run_dir, run_id, robot_id, *,
               log=None, tz=None):
    """Upload one run; ``_manifest.json`` goes last. Safe to call again.

    Returns ``{'uploaded': n, 'skipped': n, 'files': n, 'manifest_key': k}``.
    Raises RunNotReady before touching R2 when a bag is not finalized; any
    client error propagates (the caller retries on the next tick and the
    state file lets the retry skip what already made it).
    """
    log = log or (lambda _msg: None)
    files = list_run_files(run_dir)
    problems = check_ready(run_dir, files)
    if problems:
        raise RunNotReady('; '.join(problems))

    state = load_state(run_dir)
    if state.get('bucket') != bucket or state.get('base_key') != base_key:
        state = {'version': STATE_VERSION, 'bucket': bucket,
                 'base_key': base_key, 'files': {}, 'manifest_uploaded': False}
    entries = {}
    skipped = set()
    uploaded = 0

    def _stat_sha(rel):
        """(path, stat, sha256, already_uploaded) — sha reused if unchanged."""
        path = os.path.join(run_dir, rel)
        st = os.stat(path)
        prev = state['files'].get(rel) or {}
        same = bool(prev.get('sha256')) and prev.get('size') == st.st_size \
            and prev.get('mtime_ns') == st.st_mtime_ns
        sha = prev['sha256'] if same else sha256_file(path)
        return path, st, sha, same and bool(prev.get('uploaded'))

    def _put(rel, path, st, sha):
        nonlocal uploaded
        key = f'{base_key}/{rel}'
        client.upload_file(path, bucket, key,
                           ExtraArgs={'Metadata': {'sha256': sha}})
        state['files'][rel] = {'size': st.st_size, 'mtime_ns': st.st_mtime_ns,
                               'sha256': sha, 'uploaded': True}
        state['manifest_uploaded'] = False
        save_state(run_dir, state)  # a crash after this line skips the file
        entries[rel] = {'size': st.st_size, 'sha256': sha}
        uploaded += 1
        log(f'uploaded {key} ({st.st_size} B)')

    for rel in files:
        path, st, sha, done = _stat_sha(rel)
        entries[rel] = {'size': st.st_size, 'sha256': sha}
        if done:
            skipped.add(rel)
            continue
        _put(rel, path, st, sha)

    # Everything the state claims must really be on R2 before the manifest.
    for rel in files:
        key = f'{base_key}/{rel}'
        size = remote_size(client, bucket, key)
        if size == entries[rel]['size']:
            continue
        log(f're-upload {rel}: remote size {size} != {entries[rel]["size"]}')
        state['files'].pop(rel, None)
        skipped.discard(rel)
        path, st, sha, _done = _stat_sha(rel)
        _put(rel, path, st, sha)
        if remote_size(client, bucket, key) != st.st_size:
            raise RuntimeError(f'{rel}: size mismatch on R2 after re-upload')

    run_meta = read_run_metadata(run_dir)
    manifest = build_manifest(run_dir, run_id, robot_id, base_key, entries,
                              run_meta=run_meta, tz=tz)
    body = (json.dumps(manifest, ensure_ascii=False, indent=2) + '\n').encode()
    _atomic_write(os.path.join(run_dir, MANIFEST_NAME), body)
    manifest_key = f'{base_key}/{MANIFEST_NAME}'
    client.put_object(Bucket=bucket, Key=manifest_key, Body=body,
                      ContentType='application/json')
    state['manifest_uploaded'] = True
    state['manifest_sha256'] = hashlib.sha256(body).hexdigest()
    save_state(run_dir, state)
    marker = {'r2_prefix': base_key, 'files': len(files),
              'manifest_key': manifest_key,
              'manifest_sha256': state['manifest_sha256']}
    _atomic_write(os.path.join(run_dir, UPLOADED_MARKER),
                  json.dumps(marker).encode())
    log(f'manifest -> {manifest_key} ({len(files)} files, '
        f'{uploaded} uploaded, {len(skipped)} already there)')
    return {'uploaded': uploaded, 'skipped': len(skipped), 'files': len(files),
            'manifest_key': manifest_key}


def iter_keys(client, bucket, prefix):
    token = None
    while True:
        kw = {'Bucket': bucket, 'Prefix': prefix}
        if token:
            kw['ContinuationToken'] = token
        resp = client.list_objects_v2(**kw)
        for obj in resp.get('Contents', []) or []:
            yield obj['Key']
        if not resp.get('IsTruncated'):
            break
        token = resp.get('NextContinuationToken')


def delete_run_objects(client, bucket, base_key):
    """Delete a run from R2, manifest first. Returns the number deleted."""
    prefix = base_key.rstrip('/') + '/'
    manifest_key = prefix + MANIFEST_NAME
    keys = list(iter_keys(client, bucket, prefix))
    n = 0
    if manifest_key in keys:
        client.delete_object(Bucket=bucket, Key=manifest_key)
        n += 1
    for k in keys:
        if k != manifest_key:
            client.delete_object(Bucket=bucket, Key=k)
            n += 1
    return n


def verify_remote(client, bucket, base_key):
    """Compare the R2 manifest with the objects next to it (sizes only).

    Returns (ok, problems, manifest_or_None). Downloading to check sha256 is
    the consumer's job (the worker does it after download).
    """
    key = f'{base_key.rstrip("/")}/{MANIFEST_NAME}'
    try:
        body = client.get_object(Bucket=bucket, Key=key)['Body'].read()
    except Exception as e:  # noqa: BLE001
        if _is_not_found(e):
            return False, [f'沒有 {MANIFEST_NAME}（上傳未完成）'], None
        raise
    manifest = json.loads(body)
    problems = []
    for f in manifest.get('files', []):
        size = remote_size(client, bucket, f['key'])
        if size is None:
            problems.append(f'缺少 {f["key"]}')
        elif size != f['size']:
            problems.append(f'{f["key"]} 大小 {size} != manifest {f["size"]}')
    return not problems, problems, manifest
