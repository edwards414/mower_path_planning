"""deploy/host/mower-update.sh: an update cut short (reboot, power loss) must
not block every later update, "up to date" means the service runs the tagged
image, and the CPU cap holds only while pulling. Runs the script against a
fake `docker` on PATH; needs flock (util-linux), so it is skipped on macOS."""
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

TAG_ID = 'sha256:' + 'cd' * 32

FAKE_DOCKER = '''#!/bin/bash
# Answers from environment variables so each test can shape the robot:
#   FAKE_LOCAL_DIGEST  RepoDigest of the image the tag points at on disk
#   FAKE_REMOTE_DIGEST what the registry says the tag is
#   FAKE_TAG_ID        image ID of the tagged image
#   FAKE_RUNNING_ID    image ID the running lawan_node container uses
#   FAKE_RUNNING       1 = the service is running
#   FAKE_PULLED_FILE   written by `pull`; afterwards the local digest is the remote one
#   FAKE_CPUFREQ       cpufreq dir whose policy0/scaling_max_freq `pull` records
#   FAKE_PULL_FAIL     1 = `pull` fails like a denied registry
#   FAKE_LOG           every call appended here
echo "$*" >> "${FAKE_LOG:-/dev/null}"
local_digest=$FAKE_LOCAL_DIGEST
[ -s "${FAKE_PULLED_FILE:-/nonexistent}" ] && local_digest=$(cat "$FAKE_PULLED_FILE")
case "$1 $2" in
  "image inspect")
    case "$*" in *"{{.Id}}"*) echo "$FAKE_TAG_ID" ;; *) echo "ghcr.io/x/y@$local_digest" ;; esac ;;
  "manifest inspect") echo "{\\"Descriptor\\": {\\"digest\\": \\"$FAKE_REMOTE_DIGEST\\"}}" ;;
  "compose ps") [ "${FAKE_RUNNING:-1}" = 1 ] && echo "cid-lawan" ;;
  "inspect --format") echo "$FAKE_RUNNING_ID" ;;
  "pull "*)
    [ -n "${FAKE_CPUFREQ:-}" ] && cat "$FAKE_CPUFREQ/policy0/scaling_max_freq" > "$FAKE_CPUFREQ/seen_during_pull"
    [ "${FAKE_PULL_FAIL:-0}" = 1 ] && { echo "Error response from daemon: denied"; exit 1; }
    [ -n "${FAKE_PULLED_FILE:-}" ] && echo "$FAKE_REMOTE_DIGEST" > "$FAKE_PULLED_FILE"
    echo "Status: Downloaded newer image" ;;
  "compose up") echo "Container lawan_node Started" ;;
  "image prune") ;;
  *) echo "unexpected docker $*" >&2; exit 1 ;;
esac
exit 0
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
               PATH=f'{fake}:{os.environ["PATH"]}',
               # the robot at rest: tag == registry, the service runs the tag
               FAKE_LOCAL_DIGEST=DIGEST, FAKE_REMOTE_DIGEST=DIGEST,
               FAKE_TAG_ID=TAG_ID, FAKE_RUNNING_ID=TAG_ID, FAKE_RUNNING='1',
               FAKE_LOG=str(tmp_path / 'docker.log'),
               # no cpufreq on the test host
               MOWER_CPUFREQ_DIR=str(tmp_path / 'no-cpufreq'))
    return state, env


def _docker_calls(env, prefix):
    log = Path(env['FAKE_LOG'])
    return [l for l in log.read_text().splitlines() if l.startswith(prefix)] if log.exists() else []


def _state(state):
    return json.loads((state / 'update_status.json').read_text())['state']


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


def test_a_pulled_image_the_service_does_not_run_yet_is_applied(robot):
    """The tag already matches the registry (the pull landed) but lawan_node
    still runs the previous image (the restart never happened: reset between
    pull and compose up, or a pull outside the script). That used to be
    "up_to_date" forever; it is one restart, then up_to_date for real."""
    state, env = robot
    env['FAKE_RUNNING_ID'] = 'sha256:' + 'ee' * 32
    res = _run(env, '--auto')
    assert res.returncode == 0, res.stderr
    assert 'up_to_date' not in res.stdout
    assert len(_docker_calls(env, 'compose up')) == 1
    assert _state(state) == 'idle'
    assert json.loads((state / 'image.json').read_text())['digest'] == DIGEST

    env['FAKE_RUNNING_ID'] = TAG_ID
    res = _run(env, '--auto')
    assert res.returncode == 0, res.stderr
    assert 'up_to_date' in res.stdout
    assert len(_docker_calls(env, 'compose up')) == 1, 'no second restart'
    assert len(_docker_calls(env, 'pull')) == 1, 'nothing pulled once it runs the tag'


def test_up_to_date_still_needs_a_running_service(robot):
    state, env = robot
    env['FAKE_RUNNING'] = '0'
    res = _run(env)
    assert res.returncode == 0, res.stderr
    assert len(_docker_calls(env, 'compose up')) == 1
    assert _state(state) == 'idle'


def _cpufreq(tmp_path, env, max_khz='1992000'):
    cpufreq = tmp_path / 'cpufreq'
    (cpufreq / 'policy0').mkdir(parents=True)
    (cpufreq / 'policy0' / 'scaling_max_freq').write_text(max_khz + '\n')
    env.update(MOWER_CPUFREQ_DIR=str(cpufreq), FAKE_CPUFREQ=str(cpufreq),
               FAKE_REMOTE_DIGEST='sha256:' + 'ff' * 32,
               FAKE_PULLED_FILE=str(tmp_path / 'pulled'))
    return cpufreq


def test_cpu_is_capped_while_pulling_and_restored_before_the_restart(robot, tmp_path):
    state, env = robot
    cpufreq = _cpufreq(tmp_path, env)
    res = _run(env)
    assert res.returncode == 0, res.stderr
    assert 'cpu capped at 1008 MHz' in res.stdout
    assert (cpufreq / 'seen_during_pull').read_text().strip() == '1008000'
    assert (cpufreq / 'policy0' / 'scaling_max_freq').read_text().strip() == '1992000'
    assert _state(state) == 'idle'
    assert len(_docker_calls(env, 'compose up')) == 1


def test_cpu_cap_is_restored_when_the_pull_fails(robot, tmp_path):
    state, env = robot
    cpufreq = _cpufreq(tmp_path, env)
    env['FAKE_PULL_FAIL'] = '1'
    res = _run(env)
    assert res.returncode == 1
    assert (cpufreq / 'seen_during_pull').read_text().strip() == '1008000'
    assert (cpufreq / 'policy0' / 'scaling_max_freq').read_text().strip() == '1992000'
    assert _state(state) == 'failed'


def test_cpu_cap_can_be_switched_off(robot, tmp_path):
    state, env = robot
    cpufreq = _cpufreq(tmp_path, env)
    env['MOWER_UPDATE_MAX_FREQ_KHZ'] = '0'
    res = _run(env)
    assert res.returncode == 0, res.stderr
    assert 'cpu capped' not in res.stdout
    assert (cpufreq / 'seen_during_pull').read_text().strip() == '1992000'
    assert _state(state) == 'idle'
