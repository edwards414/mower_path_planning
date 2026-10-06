"""Front-camera video through MediaMTX (video_record.py) + the disk guard."""
import collections
import http.server
import json
import threading

from mower_recorder import record_profile, video_record


def test_record_patch_keeps_segments_in_the_run():
    body = video_record.record_patch('/home/mower/.mower/bags/MW-0CFP37_20261006T101500')
    assert body['record'] is True
    path = body['recordPath']
    assert path.startswith('/home/mower/.mower/bags/MW-0CFP37_20261006T101500/video/')
    # MediaMTX rejects a recordPath without %path or the full date and time
    for var in ('%path', '%Y', '%m', '%d', '%H', '%M', '%S'):
        assert var in path, var
    assert body['recordFormat'] == 'fmp4'
    assert body['recordDeleteAfter'] == '0s'  # its default deletes after 24 h
    assert video_record.STOP_PATCH == {'record': False}


class _Api(http.server.BaseHTTPRequestHandler):
    calls = []
    status = 200
    reply = b''

    def do_GET(self):  # noqa: N802 (http.server naming)
        type(self).calls.append((self.command, self.path, None))
        self.send_response(type(self).status)
        self.end_headers()
        self.wfile.write(type(self).reply)

    def do_PATCH(self):  # noqa: N802 (http.server naming)
        length = int(self.headers.get('Content-Length') or 0)
        type(self).calls.append((self.command, self.path,
                                 json.loads(self.rfile.read(length) or b'{}')))
        self.send_response(type(self).status)
        self.end_headers()
        self.wfile.write(type(self).reply)

    def log_message(self, *args):
        pass


def _serve():
    server = http.server.HTTPServer(('127.0.0.1', 0), _Api)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server, f'http://127.0.0.1:{server.server_address[1]}'


def test_patch_path_patches_the_camera_path():
    _Api.calls, _Api.status, _Api.reply = [], 200, b''
    server, url = _serve()
    try:
        ok, msg = video_record.patch_path(url + '/', 'front', {'record': False})
    finally:
        server.shutdown()
    assert ok, msg
    assert _Api.calls == [('PATCH', '/v3/config/paths/patch/front', {'record': False})]


def test_patch_path_reports_mediamtx_errors():
    _Api.calls, _Api.status = [], 400
    _Api.reply = b'{"error":"\'recordPath\' must contain %path"}'
    server, url = _serve()
    try:
        ok, msg = video_record.patch_path(url, 'front', {'record': True})
    finally:
        server.shutdown()
    assert not ok
    assert msg.startswith('HTTP 400') and 'recordPath' in msg

    # nothing listening: an error, not an exception
    ok, msg = video_record.patch_path(url, 'front', {'record': False}, timeout=1.0)
    assert not ok and msg


Usage = collections.namedtuple('Usage', 'total used free')


def test_disk_low_threshold_and_missing_paths(tmp_path):
    seen = []

    def usage(path):
        seen.append(path)
        return Usage(0, 0, 1500 * 1024 * 1024)

    assert record_profile.disk_low(str(tmp_path), 2048, usage=usage)
    assert not record_profile.disk_low(str(tmp_path), 1024, usage=usage)
    assert not record_profile.disk_low(str(tmp_path), 0, usage=usage)
    # a run dir that does not exist yet is measured on its parent
    record_profile.disk_low(str(tmp_path / 'bags' / 'run'), 2048, usage=usage)
    assert seen[-1] == str(tmp_path)


def test_path_ready_reads_the_live_state():
    server, url = _serve()
    try:
        _Api.calls, _Api.status, _Api.reply = [], 200, b'{"name":"front","ready":true}'
        assert video_record.path_ready(url, 'front') is True
        assert _Api.calls == [('GET', '/v3/paths/get/front', None)]
        _Api.reply = b'{"name":"front","ready":false}'
        assert video_record.path_ready(url, 'front') is False
        # no publisher: MediaMTX does not list the path
        _Api.status, _Api.reply = 404, b'{"error":"path not found"}'
        assert video_record.path_ready(url, 'front') is False
        _Api.status = 500
        assert video_record.path_ready(url, 'front') is None
    finally:
        server.shutdown()
    assert video_record.path_ready(url, 'front', timeout=1.0) is None
