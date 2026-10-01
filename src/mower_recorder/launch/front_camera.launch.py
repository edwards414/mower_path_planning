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
"""Front camera only (gscam MJPEG -> /camera/front/*) + its static TF.

For calibration sessions and bench checks; data_collection.launch.py
includes the same actions plus the recorder. Stop mower-camera.service first:
it holds /dev/video0 (docs/資料收集錄製程序.md).

    ros2 launch mower_recorder front_camera.launch.py calib_dir:=~/.mower/calib
"""
from launch import LaunchDescription
from launch.actions import OpaqueFunction

from mower_recorder import camera_launch


def _setup(context):
    return camera_launch.camera_actions(camera_launch.resolve_camera(context))


def generate_launch_description():
    return LaunchDescription(camera_launch.declare_camera_arguments() + [
        OpaqueFunction(function=_setup),
    ])
