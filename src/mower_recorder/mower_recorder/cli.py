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
"""
import argparse
import os
import sys

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
            key, {'robot': parts[0], 'run_id': parts[1], 'files': 0, 'bytes': 0})
        r['files'] += 1
        r['bytes'] += obj['Size']
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
    print(f'{"ROBOT":12} {"RUN_ID":22} {"FILES":>6} {"SIZE":>9}')
    for k in sorted(runs):
        r = runs[k]
        print(f'{r["robot"]:12} {r["run_id"]:22} {r["files"]:6} '
              f'{_human(r["bytes"]):>9}')


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
    for k in keys:
        client.delete_object(Bucket=cfg['R2_BUCKET'], Key=k)
    print(f'deleted {len(keys)} objects')


def main():
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
    args = ap.parse_args()

    cfg = _load_env(args.env_file)
    client = _client(cfg)
    {'ls': cmd_ls, 'get': cmd_get, 'rm': cmd_rm}[args.cmd](client, cfg, args)


if __name__ == '__main__':
    main()
