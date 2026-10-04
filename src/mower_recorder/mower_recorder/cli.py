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
"""mower-bag — list / download / delete recorded bags on Cloudflare R2.

Reads R2 creds from env vars or a .env file (searched: --env-file, ./.env,
~/.config/mower/r2.env, ~/mower_ws/.env):
  R2_ACCOUNT_ID  R2_BUCKET  R2_ACCESS_KEY_ID  R2_SECRET_ACCESS_KEY  R2_PREFIX

  mower-bag ls
  mower-bag get [<run_id>] [<dest_dir>]     # no run_id -> interactive pick
  mower-bag rm <run_id> [-y]
  mower-bag upload <run_dir>... | --all [--root DIR]   # manifest last, resumable
  mower-bag verify <run_id>                  # R2 manifest vs objects (sizes)
"""
import argparse
import os
import sys

from mower_recorder import run_manifest

_KEYS = ('R2_ACCOUNT_ID', 'R2_BUCKET', 'R2_ACCESS_KEY_ID',
         'R2_SECRET_ACCESS_KEY', 'R2_PREFIX')


def _load_env(env_file):
    cfg = {k: os.environ.get(k, '') for k in _KEYS}
    candidates = [env_file] if env_file else [
        os.path.join(os.getcwd(), '.env'),
        os.path.expanduser('~/.config/mower/r2.env'),
        os.path.expanduser('~/mower_ws/.env'),
    ]
    for path in candidates:
        if path and os.path.isfile(path):
            for line in open(path):
                line = line.strip()
                if not line or line.startswith('#') or '=' not in line:
                    continue
                k, v = line.split('=', 1)
                k = k.strip()
                if k in cfg and not cfg[k]:
                    cfg[k] = v.strip().strip('"').strip("'")
            break
    cfg['R2_PREFIX'] = cfg.get('R2_PREFIX') or 'bags'
    return cfg


def _client(cfg):
    try:
        import boto3
    except ImportError:
        sys.exit('boto3 not installed  (pip install boto3)')
    missing = [k for k in _KEYS[:4] if not cfg.get(k)]
    if missing:
        sys.exit(f'missing R2 creds: {", ".join(missing)}  (set env or .env)')
    return boto3.client(
        's3',
        endpoint_url=f'https://{cfg["R2_ACCOUNT_ID"]}.r2.cloudflarestorage.com',
        aws_access_key_id=cfg['R2_ACCESS_KEY_ID'],
        aws_secret_access_key=cfg['R2_SECRET_ACCESS_KEY'],
        region_name='auto',
    )


def _iter_objects(client, bucket, prefix):
    token = None
    while True:
        kw = {'Bucket': bucket, 'Prefix': prefix}
        if token:
            kw['ContinuationToken'] = token
        resp = client.list_objects_v2(**kw)
        for obj in resp.get('Contents', []):
            yield obj
        if not resp.get('IsTruncated'):
            break
        token = resp.get('NextContinuationToken')


def _runs(client, cfg):
    base = cfg['R2_PREFIX'].rstrip('/') + '/'
    runs = {}
    for obj in _iter_objects(client, cfg['R2_BUCKET'], base):
        parts = obj['Key'][len(base):].split('/')
        if len(parts) < 3:               # robot_id / run_id / file...
            continue
        key = f'{parts[0]}/{parts[1]}'
        r = runs.setdefault(
            key, {'robot': parts[0], 'run_id': parts[1], 'files': 0, 'bytes': 0,
                  'manifest': False})
        r['files'] += 1
        r['bytes'] += obj['Size']
        if '/'.join(parts[2:]) == run_manifest.MANIFEST_NAME:
            r['manifest'] = True   # upload finished (spec 2.1)
    return runs


def _human(n):
    for unit in ('B', 'KB', 'MB', 'GB', 'TB'):
        if n < 1024:
            return f'{n:.0f}{unit}'
        n /= 1024
    return f'{n:.0f}PB'


def _resolve(client, cfg, run_id):
    runs = _runs(client, cfg)
    hits = [k for k in runs if runs[k]['run_id'] == run_id or k == run_id]
    if len(hits) == 1:
        return hits[0]
    if not hits:
        sys.exit(f'run not found: {run_id}')
    sys.exit(f'ambiguous, use robot/run_id: {hits}')


def cmd_ls(client, cfg, _args):
    runs = _runs(client, cfg)
    if not runs:
        print('(no bags in R2)')
        return
    print(f'{"ROBOT":12} {"RUN_ID":22} {"FILES":>6} {"SIZE":>9}  MANIFEST')
    for k in sorted(runs):
        r = runs[k]
        print(f'{r["robot"]:12} {r["run_id"]:22} {r["files"]:6} '
              f'{_human(r["bytes"]):>9}  '
              f'{"done" if r["manifest"] else "incomplete"}')


def cmd_get(client, cfg, args):
    if args.run_id:
        run_key = _resolve(client, cfg, args.run_id)
    else:
        keys = sorted(_runs(client, cfg))
        if not keys:
            sys.exit('(no bags in R2)')
        for i, k in enumerate(keys):
            print(f'[{i}] {k}')
        run_key = keys[int(input('pick #: ').strip())]
    base = f'{cfg["R2_PREFIX"].rstrip("/")}/{run_key}/'
    dest = args.dest or os.path.join(os.getcwd(), run_key.split('/')[-1])
    n = 0
    for obj in _iter_objects(client, cfg['R2_BUCKET'], base):
        rel = obj['Key'][len(base):]
        out = os.path.join(dest, rel)
        os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
        client.download_file(cfg['R2_BUCKET'], obj['Key'], out)
        print(f'  {rel}')
        n += 1
    print(f'downloaded {n} files -> {dest}')


def cmd_rm(client, cfg, args):
    run_key = _resolve(client, cfg, args.run_id)
    base = f'{cfg["R2_PREFIX"].rstrip("/")}/{run_key}/'
    keys = [o['Key'] for o in _iter_objects(client, cfg['R2_BUCKET'], base)]
    if not args.yes and input(
            f'delete {len(keys)} objects under {base}? [y/N] ').strip().lower() \
            != 'y':
        print('aborted')
        return
    n = run_manifest.delete_run_objects(   # _manifest.json first
        client, cfg['R2_BUCKET'], base.rstrip('/'))
    print(f'deleted {n} objects')


def _pending_runs(root):
    runs = []
    for name in sorted(os.listdir(root)):
        run_dir = os.path.join(root, name)
        if os.path.isdir(run_dir) and not name.startswith('.') and \
                not os.path.isfile(os.path.join(run_dir, run_manifest.UPLOADED_MARKER)):
            runs.append(run_dir)
    return runs


def cmd_upload(client, cfg, args):
    """Same upload as bag_store_node (manifest last, resumable), one-shot."""
    root = os.path.expanduser(args.root)
    run_dirs = [os.path.abspath(os.path.expanduser(d)) for d in args.run_dirs]
    if args.all:
        run_dirs += _pending_runs(root)
    if not run_dirs:
        print('(nothing to upload)')
        return 0
    failed = 0
    for run_dir in run_dirs:
        run_id = os.path.basename(run_dir.rstrip('/'))
        meta = run_manifest.read_run_metadata(run_dir)
        robot = meta.get('robot_id') or args.robot_id
        base = run_manifest.run_key(cfg['R2_PREFIX'], robot, run_id)
        try:
            res = run_manifest.upload_run(
                client, cfg['R2_BUCKET'], base, run_dir, run_id, robot,
                log=print if args.verbose else None)
        except run_manifest.RunNotReady as e:
            print(f'SKIP {run_id}: {e}')
            failed += 1
            continue
        except Exception as e:  # noqa: BLE001 — resume by running it again
            print(f'FAIL {run_id}: {type(e).__name__}: {e}（再執行一次會接續上傳）')
            failed += 1
            continue
        print(f'OK   {run_id} -> {base}/ ({res["files"]} files, '
              f'{res["uploaded"]} uploaded, {res["skipped"]} already there)')
    return 1 if failed else 0


def cmd_verify(client, cfg, args):
    run_key = _resolve(client, cfg, args.run_id)
    base = f'{cfg["R2_PREFIX"].rstrip("/")}/{run_key}'
    ok, problems, manifest = run_manifest.verify_remote(
        client, cfg['R2_BUCKET'], base)
    if ok:
        print(f'OK   {base}: _manifest.json 列出 {len(manifest["files"])} 個檔案，'
              '大小都相符')
        return 0
    for p in problems:
        print(f'FAIL {base}: {p}')
    return 1


def main(argv=None):
    ap = argparse.ArgumentParser(prog='mower-bag')
    ap.add_argument('--env-file', default='')
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('ls', help='list bags in R2')
    g = sub.add_parser('get', help='download a run')
    g.add_argument('run_id', nargs='?')
    g.add_argument('dest', nargs='?')
    r = sub.add_parser('rm', help='delete a run from R2')
    r.add_argument('run_id')
    r.add_argument('-y', '--yes', action='store_true')
    u = sub.add_parser('upload', help='upload local runs (manifest last)')
    u.add_argument('run_dirs', nargs='*')
    u.add_argument('--all', action='store_true',
                   help='every run under --root without .uploaded')
    u.add_argument('--root', default=os.environ.get('MOWER_BAG_ROOT',
                                                    '~/.mower/bags'))
    u.add_argument('--robot-id', default=os.environ.get('MOWER_ROBOT_ID') or 'mower',
                   help='used when run_metadata.yaml has no robot_id')
    u.add_argument('-v', '--verbose', action='store_true')
    v = sub.add_parser('verify', help='check an uploaded run against its manifest')
    v.add_argument('run_id')
    args = ap.parse_args(argv)

    cfg = _load_env(args.env_file)
    client = _client(cfg)
    rc = {'ls': cmd_ls, 'get': cmd_get, 'rm': cmd_rm, 'upload': cmd_upload,
          'verify': cmd_verify}[args.cmd](client, cfg, args)
    return rc or 0


if __name__ == '__main__':
    sys.exit(main())
