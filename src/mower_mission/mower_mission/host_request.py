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
"""Ask the host OS to do something the container cannot.

The robot software runs in a container without systemd or the docker
socket. Actions that need the host (pull a new image and restart, reboot,
power off) are requested by writing ``<state_dir>/host.request``; the
host-side ``mower-host-request.path`` unit (deploy/host/) picks the file up,
performs the action and reports progress in ``<state_dir>/update_status.json``
which ``/robot/info`` relays back to the app.

``/usr/local/bin/mower-host-request`` (utils/mower-host-request) is the shell
equivalent used by the ros2_control driver's shutdown_command.
"""

import json
import os
import time

ACTIONS = ('update', 'restart', 'reboot', 'poweroff')

REQUEST_FILE = 'host.request'
UPDATE_STATUS_FILE = 'update_status.json'
IMAGE_FILE = 'image.json'
FIRMWARE_SYNC_FILE = 'firmware_sync.json'


def state_dir() -> str:
    """Directory shared with the host (bind-mounted ~/.mower)."""
    return os.environ.get('MOWER_STATE_DIR') or os.path.expanduser('~/.mower')


def write_request(action: str, requested_by: str = 'ros', directory=None) -> str:
    """Atomically write the request file; returns its path."""
    if action not in ACTIONS:
        raise ValueError(f'unknown host action {action!r}')
    directory = directory or state_dir()
    os.makedirs(directory, exist_ok=True)
    path = os.path.join(directory, REQUEST_FILE)
    tmp = f'{path}.tmp'
    with open(tmp, 'w') as f:
        json.dump(
            {'action': action, 'time': int(time.time()), 'requested_by': requested_by},
            f,
        )
        f.write('\n')
    os.replace(tmp, path)
    return path


def read_json(name: str, directory=None):
    """Return the parsed JSON file from the state dir, or None."""
    directory = directory or state_dir()
    path = os.path.join(directory, name)
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None
