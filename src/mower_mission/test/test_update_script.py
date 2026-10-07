"""deploy/host/mower-update.sh: an update cut short (reboot, power loss) must
not block every later update. Runs the script against a fake `docker` on
PATH; needs flock (util-linux), so it is skipped on macOS."""
import fcntl
import json
import os
import shutil
import stat
import subprocess
import time
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[3] / 'deploy' / 'host' / 'mower-update.sh'
DIGEST = 'sha256:' + 'ab' * 32

FAKE_DOCKER = f'''#!/bin/bash
# image inspect -> local digest; manifest inspect -> same digest (up to date);
# compose ps -> the service is running. Anything else is not expected here.
case "$1 $2" in
  "image inspect") echo "ghcr.io/x/y@{DIGEST}" ;;
  "manifest inspect") echo '{{"Descriptor": {{"digest": "{DIGEST}"}}}}' ;;
  "compose ps") echo "lawan_node" ;;
  *) echo "unexpected docker $*" >&2; exit 1 ;;
esac
'''


@pytest.fixture
def robot(tmp_path):
    if shutil.which('flock') is None:
        pytest.skip('needs flock (util-linux)')
    mower = tmp_path / 'opt_mower'
    state = tmp_path / 'state'
    mower.mkdir()
    state.mkdir()
    (mower / '.env').write_text('IMAGE_TAG=main\n')
    fake = tmp_path / 'bin'
    fake.mkdir()
    docker = fake / 'docker'
    docker.write_text(FAKE_DOCKER)
    docker.chmod(docker.stat().st_mode | stat.S_IEXEC)
    env = dict(os.environ, MOWER_DIR=str(mower), MOWER_STATE_DIR=str(state),
               PATH=f'{fake}:{os.environ["PATH"]}')
    return state, env


def _run(env, *args):
    return subprocess.run(['bash', str(SCRIPT), *args], env=env, text=True,
                          capture_output=True, timeout=30)


def test_stale_pulling_state_from_an_interrupted_update_does_not_block(robot):
    state, env = robot
    (state / 'update_status.json').write_text(json.dumps(
        {'state': 'pulling', 'message': 'pulling …', 'tag': 'main', 'time': int(time.time()) - 3600}))
    res = _run(env, '--auto')
    assert res.returncode == 0, res.stderr
    assert 'interrupted while pulling' in res.stdout
    assert 'already in progress' not in res.stdout
    final = json.loads((state / 'update_status.json').read_text())
    assert final['state'] == 'up_to_date', final


def test_a_running_update_is_detected_by_its_lock_not_the_state_file(robot):
    state, env = robot
    (state / 'update_status.json').write_text(json.dumps(
        {'state': 'idle', 'message': '', 'tag': 'main', 'time': int(time.time())}))
    lock = open(state / '.update.lock', 'w')
    fcntl.flock(lock, fcntl.LOCK_EX)
    try:
        for args in (['--auto'], ['--check'], []):
            res = _run(env, *args)
            assert res.returncode == 0, res.stderr
            assert 'already in progress' in res.stdout, args
    finally:
        fcntl.flock(lock, fcntl.LOCK_UN)
        lock.close()
    assert json.loads((state / 'update_status.json').read_text())['state'] == 'idle'
