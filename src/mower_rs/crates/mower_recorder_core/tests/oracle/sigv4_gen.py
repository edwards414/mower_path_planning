#!/usr/bin/env python3
"""SigV4 vectors for mower_recorder_core's S3 client, written by the robot
image's boto3 1.34.46 / botocore / s3transfer 0.10.1 with the clock frozen.

    docker run --rm --network none -v "$PWD/src/mower_rs/crates/mower_recorder_core":/x \\
      --entrypoint python3 mower_path_planning:ros-free-test -I /x/tests/oracle/sigv4_gen.py \\
      --out /x/tests/vectors/sigv4.json

Every request the uploader makes (s3transfer's upload_file below and above
the 8 MiB threshold, put_object of the manifest, head/get/list/delete,
abort) is sent through a real boto3 client whose transport is a stub
(`before-send` returns a canned answer), once against an HTTP endpoint (as
moto in G5) and once against an R2 endpoint over HTTPS. Each vector holds the
request exactly as botocore would put it on the wire (method, URL, every
header in order, body sha256) and botocore's own canonical request and
string to sign (its debug log).
"""
import argparse
import datetime
import hashlib
import io
import json
import logging
import os
import sys
import tempfile

import boto3
import botocore.auth
from botocore.awsrequest import AWSResponse
from boto3.s3.transfer import TransferConfig

FROZEN = datetime.datetime(2026, 10, 30, 8, 18, 51)
KEYS = ('AKIDE5TEST0000000000', 'e5-test-secret-key/0000000000000000000000')


class _FrozenDT(datetime.datetime):
    @classmethod
    def utcnow(cls):
        return FROZEN


class _Mod:
    datetime = _FrozenDT


botocore.auth.datetime = _Mod  # botocore.auth calls datetime.datetime.utcnow()


class _Log(logging.Handler):
    def __init__(self):
        super().__init__()
        self.lines = []

    def emit(self, record):
        self.lines.append(record.getMessage())


class _Raw(io.BytesIO):
    def stream(self, *a, **k):
        yield self.getvalue()


def answer(request):
    m, url = request.method, request.url
    status, headers, body = 200, {}, b''
    if m == 'PUT':
        headers['ETag'] = '"' + hashlib.md5(request.body.read() if hasattr(request.body, 'read') else (request.body or b'')).hexdigest() + '"'
    elif m == 'POST' and url.endswith('?uploads'):
        body = (b'<?xml version="1.0" encoding="UTF-8"?><InitiateMultipartUploadResult><Bucket>b</Bucket>'
                b'<Key>k</Key><UploadId>e5-upload-id/1</UploadId></InitiateMultipartUploadResult>')
    elif m == 'POST':
        body = b'<?xml version="1.0" encoding="UTF-8"?><CompleteMultipartUploadResult><ETag>"x-3"</ETag></CompleteMultipartUploadResult>'
    elif m == 'HEAD':
        headers['Content-Length'] = '42'
    elif m == 'GET' and 'list-type=2' in url:
        body = (b'<?xml version="1.0" encoding="UTF-8"?><ListBucketResult><IsTruncated>false</IsTruncated>'
                b'<KeyCount>0</KeyCount><EncodingType>url</EncodingType></ListBucketResult>')
    elif m == 'GET':
        body = b'{}'
    elif m == 'DELETE':
        status = 204
    r = AWSResponse(url, status, headers, _Raw(body))
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    log = _Log()
    lg = logging.getLogger('botocore.auth')
    lg.setLevel(logging.DEBUG)
    lg.addHandler(log)
    vectors = []
    tmp = tempfile.mkdtemp()
    small = os.path.join(tmp, 'small.yaml')
    open(small, 'wb').write(b'run_id: MW-E5FIX_20261030T081851\n')
    big = os.path.join(tmp, 'big.mp4')
    open(big, 'wb').write(b''.join(hashlib.sha256(i.to_bytes(4, 'little')).digest() for i in range(9 * 1024 * 1024 // 32)))
    for endpoint in ('http://127.0.0.1:9000', 'https://e5account0000000000000000000000.r2.cloudflarestorage.com'):
        client = boto3.client('s3', endpoint_url=endpoint, aws_access_key_id=KEYS[0],
                              aws_secret_access_key=KEYS[1], region_name='auto')
        sent = []

        def before_send(request, **kw):
            body = request.body
            if hasattr(body, 'read'):
                pos = body.tell()
                data = body.read()
                body.seek(pos)
            else:
                data = body if isinstance(body, bytes) else (body.encode() if body else b'')
            auth_logs = list(log.lines)
            log.lines.clear()
            cr = next((l.split('\n', 1)[1] for l in auth_logs if l.startswith('CanonicalRequest:')), None)
            sts = next((l.split('\n', 1)[1] for l in auth_logs if l.startswith('StringToSign:')), None)
            sent.append({'method': request.method, 'url': request.url,
                         'headers': [[k, v.decode() if isinstance(v, bytes) else v] for k, v in request.headers.items()],
                         'body_len': len(data), 'body_sha256': hashlib.sha256(data).hexdigest(),
                         'body': data.decode() if data.startswith(b'<') else None,
                         'canonical_request': cr, 'string_to_sign': sts})
            return answer(request)

        client.meta.events.register('before-send.s3.*', before_send)
        # one part at a time, so the order of the vectors is fixed
        cfg = TransferConfig(max_concurrency=1, use_threads=False)
        steps = [
            ('upload_file small', lambda: client.upload_file(small, 'mower-bags', 'bags/MW-E5FIX/MW-E5FIX_20261030T081851/run_metadata.yaml',
                                                              ExtraArgs={'Metadata': {'sha256': 'ab' * 32}}, Config=cfg)),
            ('upload_file big', lambda: client.upload_file(big, 'mower-bags', 'bags/MW-E5FIX/run 1/video/front ü+~.mp4',
                                                            ExtraArgs={'Metadata': {'sha256': 'cd' * 32}}, Config=cfg)),
            ('put_object manifest', lambda: client.put_object(Bucket='mower-bags', Key='bags/MW-E5FIX/r/_manifest.json',
                                                               Body=b'{\n  "schema": "x"\n}\n', ContentType='application/json')),
            ('head_object', lambda: client.head_object(Bucket='mower-bags', Key='bags/MW-E5FIX/r/bag/bag_0.mcap')),
            ('get_object', lambda: client.get_object(Bucket='mower-bags', Key='bags/MW-E5FIX/r/_manifest.json')['Body'].read()),
            ('list_objects_v2', lambda: client.list_objects_v2(Bucket='mower-bags', Prefix='bags/MW-E5FIX/r/')),
            ('list_objects_v2 token', lambda: client.list_objects_v2(Bucket='mower-bags', Prefix='bags/MW-E5FIX/r/',
                                                                      ContinuationToken='1/a+b=c')),
            ('delete_object', lambda: client.delete_object(Bucket='mower-bags', Key='bags/MW-E5FIX/r/_manifest.json')),
            ('abort', lambda: client.abort_multipart_upload(Bucket='mower-bags', Key='bags/MW-E5FIX/r/video/x.mp4',
                                                             UploadId='e5-upload-id/1')),
        ]
        for name, fn in steps:
            sent.clear()
            fn()
            for i, s in enumerate(sent):
                vectors.append(dict(s, endpoint=endpoint, step=name, index=i))
    doc = {'generator': 'tests/oracle/sigv4_gen.py', 'boto3': boto3.__version__,
           'timestamp': FROZEN.strftime('%Y%m%dT%H%M%SZ'), 'access_key': KEYS[0], 'secret_key': KEYS[1],
           'region': 'auto', 'vectors': vectors}
    json.dump(doc, open(a.out, 'w'), indent=1)
    print(f'{len(vectors)} requests')


if __name__ == '__main__':
    sys.exit(main())
