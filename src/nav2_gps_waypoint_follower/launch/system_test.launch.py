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
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():
    # 1. 啟動 gps_waypoint_follower.launch.py
    # gps_waypoint_follower = ExecuteProcess(
    #     cmd=['ros2', 'launch', 'nav2_gps_waypoint_follower', 'gps_waypoint_follower.launch.py'],
    #     output='screen'
    # )

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
    load_zone_list = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/load_zone_list', 'std_srvs/srv/Trigger'],
        output='screen'
    )
    create_free_space = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/create_free_space', 'std_srvs/srv/Trigger', '{}'],
        output='screen'
    )

    create_risk_map = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/create_risk_map', 'std_srvs/srv/Trigger', '{}'],
        output='screen'
    )

    generate_coverage_path = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/generate_coverage_path', 'std_srvs/srv/Trigger', '{}'],
        output='screen'
    )

    # 使用多個TimerAction，每個間隔5秒
    timer_path_record = TimerAction(
        period=5.0,  # 5秒後啟動 path_record
        actions=[path_record]
    )

    timer_map_manage = TimerAction(
        period=10.0,  # 10秒後啟動 map_manage
        actions=[map_manage]
    )

    timer_boustrophedon_coverage = TimerAction(
        period=15.0,  # 15秒後啟動 boustrophedon_coverage
        actions=[boustrophedon_coverage]
    )

    timer_load_zone_list = TimerAction(
        period=20.0,  # 20秒後啟動 load_zone_list
        actions=[load_zone_list]
    )

    timer_create_free_space = TimerAction(
        period=25.0,  # 25秒後啟動 create_free_space
        actions=[create_free_space]
    )

    timer_create_risk_map = TimerAction(
        period=30.0,  # 30秒後啟動 create_risk_map
        actions=[create_risk_map]
    )

    timer_generate_coverage_path = TimerAction(
        period=35.0,  # 35秒後啟動 generate_coverage_path
        actions=[generate_coverage_path]
    )

    return LaunchDescription([
        # gps_waypoint_follower,
        timer_path_record,
        timer_map_manage,
        timer_boustrophedon_coverage,
        timer_load_zone_list,
        timer_create_free_space,
        timer_create_risk_map,
        timer_generate_coverage_path
    ])
