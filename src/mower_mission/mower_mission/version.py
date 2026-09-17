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
"""Version identity of the robot software, as reported on ``/robot/info``.

ROBOT_API_VERSION is the contract between the robot and the app: the set of
rosbridge-visible topics, services and message layouts (``/adapter/*``,
``mower_interface`` srv/msg, ``/robot/*``, ``/system/*``). Bump it when a
change would break an app built against the previous number, and record
what changed in docs/ROBOT_API.md. The app carries the range it supports and
refuses to operate a robot outside it, so a mismatch is shown as "update the
app / update the robot" instead of failing silently.

The software version itself comes from the build: the Docker image is built
with MOWER_VERSION / MOWER_GIT_SHA / MOWER_BUILD_UNIX (git describe of the
repo, see .github/workflows/build.yml) and exports them as environment
variables. Outside a container they fall back to "dev".
"""

import os

ROBOT_API_VERSION = 2


def software_identity() -> dict:
    """Build identity of the running robot software, from the image env."""
    return {
        'version': os.environ.get('MOWER_VERSION', 'dev'),
        'git_sha': os.environ.get('MOWER_GIT_SHA', ''),
        'build_unix': _int_env('MOWER_BUILD_UNIX'),
        'image': os.environ.get('MOWER_IMAGE', ''),
    }


def _int_env(name: str):
    raw = os.environ.get(name, '')
    try:
        return int(raw) if raw else None
    except ValueError:
        return None
