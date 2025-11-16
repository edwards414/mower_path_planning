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
from subprocess import call
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # 2. 其他指令（每個間隔5秒執行）
    path_record = ExecuteProcess(
        cmd=['ros2', 'run', 'path_record', 'path_record'],
        output='screen'
    )
    map_manage = ExecuteProcess(
        cmd=['ros2', 'run', 'maphub', 'map_manage'],
        output='screen'
    )
    boustrophedon_coverage = ExecuteProcess(
        cmd=['ros2', 'run', 'boustrophedon_coverage', 'boustrophedon_coverage'],
        output='screen'
    )

    return LaunchDescription([
        path_record,
        map_manage,
        boustrophedon_coverage,
    ])
