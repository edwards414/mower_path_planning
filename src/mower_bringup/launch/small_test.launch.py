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
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    mower_nav2_dir = get_package_share_directory('mower_nav2')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(mower_nav2_dir, 'config', 'nav2_no_map_params.yaml'),
        description='Full path to the ROS 2 parameters file'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    configured_params = LaunchConfiguration('params_file')

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

    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mower_nav2_dir, 'launch', 'dual_ekf_navsat.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mower_nav2_dir, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'autostart': 'True',
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        robot_localization_cmd,
        navigation_cmd,
        path_record,
        map_manage,
        boustrophedon_coverage,
    ])
