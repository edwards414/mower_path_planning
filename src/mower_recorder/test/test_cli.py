"""mower-bag upload / verify / ls / rm against moto (no real R2)."""
import argparse
import os

import boto3
from moto import mock_aws

import synth_run
from mower_recorder import cli, run_manifest

BUCKET = 'mower-cli-test'
CFG = {'R2_BUCKET': BUCKET, 'R2_PREFIX': 'bags'}


def _client():
    s3 = boto3.client('s3', region_name='us-east-1')
    s3.create_bucket(Bucket=BUCKET)
    return s3


@mock_aws
def test_upload_all_then_verify_and_ls(tmp_path, capsys):
    root = tmp_path / 'bags'
    run = synth_run.make_run(str(root))
    s3 = _client()
    args = argparse.Namespace(run_dirs=[], all=True, root=str(root),
                              robot_id='mower', verbose=False)
    assert cli.cmd_upload(s3, CFG, args) == 0
    assert os.path.isfile(os.path.join(run, '.uploaded'))
    # already uploaded -> nothing pending the second time
    assert cli.cmd_upload(s3, CFG, args) == 0
    assert '(nothing to upload)' in capsys.readouterr().out

    run_id = os.path.basename(run)
    assert cli.cmd_verify(s3, CFG, argparse.Namespace(run_id=run_id)) == 0
    cli.cmd_ls(s3, CFG, None)
    out = capsys.readouterr().out
    assert run_id in out and 'done' in out

    # a run without manifest shows up as incomplete
    s3.delete_object(Bucket=BUCKET,
                     Key=f'bags/mower-03/{run_id}/{run_manifest.MANIFEST_NAME}')
    cli.cmd_ls(s3, CFG, None)
    assert 'incomplete' in capsys.readouterr().out
    assert cli.cmd_verify(s3, CFG, argparse.Namespace(run_id=run_id)) == 1


@mock_aws
def test_upload_skips_unfinished_run(tmp_path, capsys):
    root = tmp_path / 'bags'
    run = synth_run.make_run(str(root))
    os.remove(os.path.join(run, 'bag', 'metadata.yaml'))
    s3 = _client()
    args = argparse.Namespace(run_dirs=[run], all=False, root=str(root),
                              robot_id='mower', verbose=False)
    assert cli.cmd_upload(s3, CFG, args) == 1
    assert 'SKIP' in capsys.readouterr().out
    assert s3.list_objects_v2(Bucket=BUCKET).get('KeyCount') == 0


@mock_aws
def test_rm_deletes_everything(tmp_path):
    root = tmp_path / 'bags'
    run = synth_run.make_run(str(root))
    s3 = _client()
    cli.cmd_upload(s3, CFG, argparse.Namespace(
        run_dirs=[run], all=False, root=str(root), robot_id='mower', verbose=False))
    cli.cmd_rm(s3, CFG, argparse.Namespace(run_id=os.path.basename(run), yes=True))
    assert s3.list_objects_v2(Bucket=BUCKET).get('KeyCount') == 0
