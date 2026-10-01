"""check_run against synthetic runs (good run + one defect per test)."""
import json
import os

import pytest
from mcap.reader import make_reader
from mcap.writer import CompressionType

import synth_run
from mower_recorder import check_run as cr
from mower_recorder import run_manifest


def _status(rep, check):
    return [i['status'] for i in rep.items if i['check'] == check]


def _run(run_dir, **kw):
    rep, _ctx = cr.check_run(run_dir, cr.Options(**kw))
    return rep


@pytest.fixture
def good_run(tmp_path):
    return synth_run.make_run(str(tmp_path))


def test_good_run_passes(good_run):
    rep = _run(good_run)
    assert rep.result == 'PASS', [i for i in rep.items if i['status'] != 'PASS']
    for check in ('camera.pairing', 'camera.info_size', 'tf_static.chain',
                  'tf_static.extrinsics', 'bag.counts[bag]', 'bag.counts[camera]',
                  'stamp[/camera/front/image_raw/compressed]', 'camera.latency'):
        assert _status(rep, check) == ['PASS'], check


def test_cli_exit_codes_and_json(good_run, tmp_path, capsys):
    out = tmp_path / 'r.json'
    assert cr.main([good_run, '--json', str(out)]) == 0
    doc = json.loads(out.read_text())
    assert doc['result'] == 'PASS'
    assert doc['topics']['/camera/front/image_raw/compressed'] == 60
    assert '結果：PASS' in capsys.readouterr().out
    assert cr.main([str(tmp_path / 'nope')]) == 1


def test_camera_info_size_mismatch_fails(tmp_path):
    run = synth_run.make_run(str(tmp_path), info_size=(640, 480))
    rep = _run(run)
    assert 'FAIL' in _status(rep, 'camera.info_size')
    assert rep.result == 'FAIL'


def test_unpaired_camera_info_fails(tmp_path):
    run = synth_run.make_run(str(tmp_path), drop_info=(30, 31))
    assert _status(_run(run), 'camera.pairing') == ['FAIL']


def test_edge_unpaired_is_tolerated(tmp_path):
    run = synth_run.make_run(str(tmp_path), drop_info=(59,))
    assert _status(_run(run), 'camera.pairing') == ['PASS']


def test_duplicate_image_stamp_fails(tmp_path):
    run = synth_run.make_run(str(tmp_path), dup_image=20)
    rep = _run(run)
    assert _status(rep, 'stamp[/camera/front/image_raw/compressed]') == ['FAIL']


def test_missing_topic_and_low_rate(tmp_path):
    run = synth_run.make_run(str(tmp_path), omit=('/gps/status',),
                             hz_override={'/fix': 0.3, '/imu/data': 5.0})
    rep = _run(run)
    assert _status(rep, 'topic[/gps/status]') == ['FAIL']
    assert _status(rep, 'topic[/fix]') == ['FAIL']
    assert _status(rep, 'topic[/imu/data]') == ['WARN']


def test_low_camera_rate_fails(tmp_path):
    run = synth_run.make_run(str(tmp_path), cam_hz=4.0,
                             extra_meta={'camera': {'fps_recorded': 10,
                                                    'width': 1280, 'height': 720}})
    assert 'FAIL' in _status(_run(run), 'topic[/camera/front/image_raw/compressed]')


def test_missing_camera_tf_chain_fails(tmp_path):
    run = synth_run.make_run(str(tmp_path), tf_chain=False)
    assert _status(_run(run), 'tf_static.chain') == ['FAIL']


def test_extrinsics_mismatch_with_calib_file_fails(good_run):
    path = os.path.join(good_run, 'calib', 'extrinsics.yaml')
    import yaml
    doc = yaml.safe_load(open(path))
    doc['derived']['base_footprint_to_camera_front_optical_frame']['xyz_m'][2] += 0.05
    yaml.safe_dump(doc, open(path, 'w'))
    assert _status(_run(good_run), 'tf_static.extrinsics') == ['FAIL']


def test_placeholder_extrinsics(tmp_path):
    run = synth_run.make_run(str(tmp_path), extrinsics_source='placeholder')
    assert _status(_run(run), 'calib.extrinsics_source') == ['FAIL']
    assert _status(_run(run, allow_placeholder=True),
                   'calib.extrinsics_source') == ['WARN']


def test_uncalibrated_intrinsics(tmp_path):
    run = synth_run.make_run(str(tmp_path), calibrated=False)
    assert _status(_run(run), 'camera.intrinsics') == ['FAIL']
    assert _status(_run(run, allow_uncalibrated=True), 'camera.intrinsics') == ['WARN']


def test_receive_time_stamps_warn(tmp_path):
    run = synth_run.make_run(str(tmp_path), stamp_as_receive=True)
    assert _status(_run(run), 'camera.latency') == ['WARN']


def test_mjpeg_without_dht_is_warning_only(tmp_path):
    run = synth_run.make_run(str(tmp_path), strip_dht=True)
    rep = _run(run)
    assert _status(rep, 'camera.dht') == ['WARN']
    assert _status(rep, 'camera.jpeg') == ['PASS']


def test_message_compression_is_flagged_but_decoded(tmp_path):
    run = synth_run.make_run(str(tmp_path), message_compression=True,
                             telemetry_compression=CompressionType.NONE)
    rep = _run(run)
    assert _status(rep, 'bag.compression[bag]') == ['FAIL']      # P-089
    assert _status(rep, 'tf[odom->base_footprint]') == ['PASS']  # still decoded
    assert _status(rep, 'stamp[/imu/data]') == ['PASS']


def test_without_zstd_uses_message_index(good_run, monkeypatch):
    monkeypatch.setitem(cr._DECOMPRESSORS, 'zstd', None)
    rep = _run(good_run)
    assert _status(rep, 'bag.counts[bag]') == ['PASS']
    assert _status(rep, 'bag.decode[bag]') == ['WARN']
    assert _status(rep, 'topic[/imu/data]') == ['PASS']          # rate via index
    assert _status(rep, 'stamp[/imu/data]') == ['SKIP']
    assert _status(rep, 'camera.pairing') == ['PASS']            # camera/ unaffected
    assert _status(rep, 'tf_static.chain') == ['PASS']           # from camera/ bag


def test_truncated_bag_fails(good_run):
    path = os.path.join(good_run, 'camera', 'camera_0.mcap')
    data = open(path, 'rb').read()
    open(path, 'wb').write(data[:len(data) // 2])
    rep = _run(good_run)
    assert _status(rep, 'bag.mcap[camera]') == ['FAIL']
    assert _status(rep, 'bag.counts[camera]') == ['FAIL']


def test_missing_bag_metadata_fails(good_run):
    os.remove(os.path.join(good_run, 'camera', 'metadata.yaml'))
    assert _status(_run(good_run), 'bag.metadata[camera]') == ['FAIL']


def test_yaml_off_boolean_is_reported(tmp_path):
    run = synth_run.make_run(str(tmp_path), extra_meta={'blade': False})
    rep = _run(run)
    msg = [i['message'] for i in rep.items if i['check'] == 'metadata.fields'][0]
    assert 'P-099' in msg


def test_manifest_verification(good_run):
    files = run_manifest.list_run_files(good_run)
    entries = {rel: {'size': os.path.getsize(os.path.join(good_run, rel)),
                     'sha256': run_manifest.sha256_file(os.path.join(good_run, rel))}
               for rel in files}
    run_id = os.path.basename(good_run)
    man = run_manifest.build_manifest(good_run, run_id, 'mower-03',
                                      f'bags/mower-03/{run_id}', entries)
    with open(os.path.join(good_run, '_manifest.json'), 'w') as f:
        json.dump(man, f)
    assert _status(_run(good_run), 'manifest') == ['PASS']
    with open(os.path.join(good_run, 'run_metadata.yaml'), 'a') as f:
        f.write('# edited after upload\n')
    assert _status(_run(good_run), 'manifest') == ['FAIL']
    assert _status(_run(good_run, verify_sha=False), 'manifest') == ['FAIL']  # size


def test_reader_agrees_with_official_mcap_reader(good_run):
    for rel in ('bag/bag_0.mcap', 'camera/camera_0.mcap'):
        path = os.path.join(good_run, rel)
        mine = []
        cr.scan_mcap(path, lambda t, s, lt, d: mine.append((t, lt, bytes(d))))
        with open(path, 'rb') as f:
            ref = [(ch.topic, m.log_time, m.data)
                   for _s, ch, m in make_reader(f).iter_messages(log_time_order=False)]
        assert sorted(mine) == sorted(ref)


def test_jpeg_info():
    ok = cr.jpeg_info(synth_run.jpeg_bytes((1280, 720)))
    assert (ok['ok'], ok['width'], ok['height'], ok['dht']) == (True, 1280, 720, True)
    bad = cr.jpeg_info(synth_run.jpeg_bytes((1280, 720))[:500])
    assert not bad['ok']
    assert not cr.jpeg_info(b'\x00\x01garbage')['ok']
    assert cr.jpeg_info(synth_run.jpeg_bytes((64, 48), strip_dht=True))['dht'] is False
