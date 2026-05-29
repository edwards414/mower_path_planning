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
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    enable_localization = LaunchConfiguration('enable_localization')
    enable_navigation = LaunchConfiguration('enable_navigation')
    nav_autostart = LaunchConfiguration('nav_autostart')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    world = LaunchConfiguration('world')

    mower_sim_dir = get_package_share_directory('mower_sim')
    mower_nav2_dir = get_package_share_directory('mower_nav2')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz',
    )
    declare_enable_localization = DeclareLaunchArgument(
        'enable_localization',
        default_value='true',
        description='Launch dual EKF and navsat_transform',
    )
    declare_enable_navigation = DeclareLaunchArgument(
        'enable_navigation',
        default_value='true',
        description='Launch the Nav2 stack',
    )
    declare_nav_autostart = DeclareLaunchArgument(
        'nav_autostart',
        default_value='true',
        description='Automatically activate Nav2 lifecycle nodes',
    )
    declare_nav2_params_file = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(
            mower_nav2_dir,
            'config',
            'nav2_no_map_params.yaml',
        ),
        description='Full path to the Nav2 parameters file',
    )
    declare_x_pose = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Initial mower x position in Gazebo',
    )
    declare_y_pose = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Initial mower y position in Gazebo',
    )
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='mower_world.world',
        description=(
            'World: bare filename (looked up under mower_sim/worlds/), '
            'absolute path, or a built-in Gazebo world name (e.g. empty.sdf)'
        ),
    )

    sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mower_sim_dir, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'world': world,
        }.items(),
    )

    localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mower_nav2_dir,
                'launch',
                'dual_ekf_navsat.launch.py',
            )
        ),
        condition=IfCondition(enable_localization),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    delayed_localization = TimerAction(
        period=11.0,
        actions=[localization_cmd],
    )

    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mower_nav2_dir,
                'launch',
                'navigation.launch.py',
            )
        ),
        condition=IfCondition(enable_navigation),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': nav_autostart,
        }.items(),
    )

    delayed_navigation = TimerAction(
        period=14.0,
        actions=[navigation_cmd],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rviz,
        declare_enable_localization,
        declare_enable_navigation,
        declare_nav_autostart,
        declare_nav2_params_file,
        declare_x_pose,
        declare_y_pose,
        declare_world,
        sim_cmd,
        delayed_localization,
        delayed_navigation,
    ])
