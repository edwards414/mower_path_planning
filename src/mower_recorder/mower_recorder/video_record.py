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
"""Front-camera video for a recording: MediaMTX records its own stream.

The camera belongs to the host (mower-camera.service: USB MJPG -> Rockchip
MPP H.264 -> RTSP into mediamtx, the app's live view), and V4L2 lets only one
process stream it, so the recorder never opens it. While a run is going it
asks MediaMTX to record the ``front`` path into ``<run>/video/`` instead:
fMP4 segments of the hardware-encoded stream as it is (no re-encode, about
1 GB/h at the default CAMERA_BPS=2000000). MediaMTX hot-reloads the record
settings (path_manager.go pathConfCanBeUpdated), so neither the publisher nor
the app's live view is interrupted. Segment file names carry the wall-clock
start time for lining the video up with the bag.

Pure Python (no rclpy) so it is testable off-robot.
"""
import json
import urllib.error
import urllib.request

VIDEO_DIR = 'video'

STOP_PATCH = {'record': False}


def record_patch(run_dir):
    """MediaMTX path settings that record into ``<run_dir>/video/``."""
    return {
        'record': True,
        # MediaMTX requires %path and either %s or the full date and time
        'recordPath': f'{run_dir}/{VIDEO_DIR}/%path_%Y-%m-%d_%H-%M-%S-%f',
        'recordFormat': 'fmp4',
        # its default deletes segments after 24 h; they belong to the run
        'recordDeleteAfter': '0s',
    }


def patch_path(api_url, path, body, timeout=3.0, opener=urllib.request.urlopen):
    """PATCH ``/v3/config/paths/patch/<path>``; returns (ok, message)."""
    url = f'{api_url.rstrip("/")}/v3/config/paths/patch/{path}'
    req = urllib.request.Request(
        url, data=json.dumps(body).encode('utf-8'), method='PATCH',
        headers={'Content-Type': 'application/json'})
    try:
        with opener(req, timeout=timeout) as res:
            return True, f'HTTP {res.status}'
    except urllib.error.HTTPError as e:
        detail = e.read().decode('utf-8', 'replace').strip()
        return False, f'HTTP {e.code} {detail}'.strip()
    except (urllib.error.URLError, OSError) as e:
        return False, str(getattr(e, 'reason', e))


def path_ready(api_url, path, timeout=3.0, opener=urllib.request.urlopen):
    """Whether the camera path has a live stream right now: True / False, or
    None when MediaMTX cannot be asked. A path without a publisher is not
    listed at all (HTTP 404)."""
    url = f'{api_url.rstrip("/")}/v3/paths/get/{path}'
    try:
        with opener(url, timeout=timeout) as res:
            return bool(json.loads(res.read().decode('utf-8') or '{}').get('ready'))
    except urllib.error.HTTPError as e:
        return False if e.code == 404 else None
    except (urllib.error.URLError, OSError, ValueError):
        return None
