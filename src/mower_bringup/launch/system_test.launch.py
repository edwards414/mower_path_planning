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
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_sim = LaunchConfiguration('launch_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_launch_sim = DeclareLaunchArgument(
        'launch_sim',
        default_value='true',
        description='Launch Gazebo simulation from this launch file',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true',
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz when launch_sim is true',
    )

    mower_bringup_dir = get_package_share_directory('mower_bringup')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mower_bringup_dir,
                'launch',
                'sim.launch.py',
            )
        ),
        condition=IfCondition(launch_sim),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
        }.items(),
    )

    path_record = ExecuteProcess(
        cmd=['ros2', 'run', 'mower_mission', 'path_record_node'],
        output='screen'
    )
    map_manage = ExecuteProcess(
        cmd=['ros2', 'run', 'mower_mission', 'map_manage_node'],
        output='screen'
    )
    boustrophedon_coverage = ExecuteProcess(
        cmd=['ros2', 'run', 'mower_mission', 'coverage_node'],
        output='screen'
    )
    nav_action_server = ExecuteProcess(
        cmd=['ros2', 'run', 'mower_mission', 'nav_action_server'],
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

    twist_mux_config = os.path.join(
        mower_bringup_dir, 'config', 'twist_mux_topics.yaml'
    )
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_config],
        remappings=[('/cmd_vel_out', '/cmd_vel')],
    )

    timer_path_record = TimerAction(
        period=5.0,
        actions=[path_record]
    )

    timer_map_manage = TimerAction(
        period=10.0,
        actions=[map_manage]
    )

    timer_boustrophedon_coverage = TimerAction(
        period=15.0,
        actions=[boustrophedon_coverage]
    )

    timer_nav_action_server = TimerAction(
        period=18.0,
        actions=[nav_action_server]
    )

    timer_load_zone_list = TimerAction(
        period=20.0,
        actions=[load_zone_list]
    )

    timer_create_free_space = TimerAction(
        period=25.0,
        actions=[create_free_space]
    )

    timer_create_risk_map = TimerAction(
        period=30.0,
        actions=[create_risk_map]
    )

    timer_generate_coverage_path = TimerAction(
        period=35.0,
        actions=[generate_coverage_path]
    )

    return LaunchDescription([
        declare_launch_sim,
        declare_use_sim_time,
        declare_use_rviz,
        twist_mux,
        sim_launch,
        timer_path_record,
        timer_map_manage,
        timer_boustrophedon_coverage,
        timer_nav_action_server,
        timer_load_zone_list,
        timer_create_free_space,
        timer_create_risk_map,
        timer_generate_coverage_path
    ])
