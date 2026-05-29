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
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

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

    mower_sim_dir = get_package_share_directory('mower_sim')
    mower_bringup_dir = get_package_share_directory('mower_bringup')
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='mower_world.world',
        description=(
            'World: bare filename (looked up under mower_sim/worlds/), '
            'absolute path, or a built-in Gazebo world name (e.g. empty.sdf)'
        ),
    )

    def _build_gazebo_cmd(context):
        world_value = LaunchConfiguration('world').perform(context)
        if not os.path.isabs(world_value) and not any(
            sep in world_value for sep in ('/', '\\')
        ):
            candidate = os.path.join(mower_sim_dir, 'worlds', world_value)
            if os.path.isfile(candidate):
                world_value = candidate
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(mower_sim_dir, 'launch', 'gps_world.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'x_pose': x_pose,
                'y_pose': y_pose,
                'publish_robot_state_publisher': 'true',
                'launch_controllers': 'true',
                'gz_args': f'-v 4 -r {world_value}',
            }.items(),
        )]

    gazebo_cmd = OpaqueFunction(function=_build_gazebo_cmd)

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mower_bringup_dir, 'launch', 'rviz.launch.py')
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    delayed_rviz = TimerAction(
        period=8.0,
        actions=[rviz_cmd],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rviz,
        declare_x_pose,
        declare_y_pose,
        declare_world,
        gazebo_cmd,
        delayed_rviz,
    ])
