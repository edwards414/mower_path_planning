"""ROS-free tests for the /robot/info building blocks (host_request, version)
and a source-level check that rosbridge exposes what the app needs."""

import json
import re
from pathlib import Path

import pytest

from mower_mission import host_request
from mower_mission.version import ROBOT_API_VERSION, software_identity

PACKAGE_DIR = Path(__file__).resolve().parents[1]
ROSBRIDGE_CONFIG = PACKAGE_DIR.parent / 'mower_bringup/config/rosbridge_params.yaml'
API_DOC = PACKAGE_DIR.parents[1] / 'docs/ROBOT_API.md'


def test_write_request_is_atomic_json(tmp_path):
    path = host_request.write_request('update', requested_by='test', directory=str(tmp_path))
    assert Path(path).name == host_request.REQUEST_FILE
    assert not (tmp_path / (host_request.REQUEST_FILE + '.tmp')).exists()
    data = json.loads(Path(path).read_text())
    assert data['action'] == 'update'
    assert data['requested_by'] == 'test'
    assert isinstance(data['time'], int)


def test_write_request_rejects_unknown_action(tmp_path):
    with pytest.raises(ValueError):
        host_request.write_request('format-disk', directory=str(tmp_path))


def test_read_json_missing_or_malformed(tmp_path):
    assert host_request.read_json('nope.json', str(tmp_path)) is None
    (tmp_path / 'bad.json').write_text('{not json')
    assert host_request.read_json('bad.json', str(tmp_path)) is None
    (tmp_path / 'ok.json').write_text('{"state": "idle"}')
    assert host_request.read_json('ok.json', str(tmp_path)) == {'state': 'idle'}


def test_state_dir_from_env(monkeypatch, tmp_path):
    monkeypatch.setenv('MOWER_STATE_DIR', str(tmp_path))
    assert host_request.state_dir() == str(tmp_path)
    monkeypatch.delenv('MOWER_STATE_DIR')
    assert host_request.state_dir().endswith('.mower')


def test_software_identity_from_image_env(monkeypatch):
    monkeypatch.setenv('MOWER_VERSION', '0.6.0')
    monkeypatch.setenv('MOWER_GIT_SHA', 'abc123')
    monkeypatch.setenv('MOWER_BUILD_UNIX', '1789571662')
    monkeypatch.setenv('MOWER_IMAGE', 'ghcr.io/x/y:v0.6.0')
    ident = software_identity()
    assert ident == {
        'version': '0.6.0',
        'git_sha': 'abc123',
        'build_unix': 1789571662,
        'image': 'ghcr.io/x/y:v0.6.0',
    }
    monkeypatch.setenv('MOWER_BUILD_UNIX', 'garbage')
    assert software_identity()['build_unix'] is None
    for name in ('MOWER_VERSION', 'MOWER_GIT_SHA', 'MOWER_BUILD_UNIX', 'MOWER_IMAGE'):
        monkeypatch.delenv(name)
    assert software_identity()['version'] == 'dev'


def test_rosbridge_exposes_info_topic_and_system_services():
    text = ROSBRIDGE_CONFIG.read_text(encoding='utf-8')
    assert "'/robot/info'" in text.split('topics_sub_glob')[1].split('\n')[0]
    services_line = text.split('services_glob')[1].split('\n')[0]
    assert "'/system/update'" in services_line
    assert "'/system/restart'" in services_line
    # rosapi needs the topic too so the app can discover it
    rosapi_topics = text.split('rosapi:')[1].split('topics_glob')[1].split('\n')[0]
    assert "'/robot/info'" in rosapi_topics


def test_api_version_is_documented():
    """docs/ROBOT_API.md must list the current ROBOT_API_VERSION."""
    text = API_DOC.read_text(encoding='utf-8')
    versions = [int(v) for v in re.findall(r'^\|\s*(\d+)\s*\|', text, flags=re.M)]
    assert ROBOT_API_VERSION in versions, (
        f'add a row for api_version {ROBOT_API_VERSION} to docs/ROBOT_API.md'
    )
