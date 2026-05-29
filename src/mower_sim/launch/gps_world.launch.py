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
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    publish_robot_state_publisher = LaunchConfiguration(
        'publish_robot_state_publisher'
    )
    launch_controllers = LaunchConfiguration('launch_controllers')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
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
    declare_publish_robot_state_publisher = DeclareLaunchArgument(
        'publish_robot_state_publisher',
        default_value='true',
        description='Launch robot_state_publisher from spawn_mowerbot',
    )
    declare_launch_controllers = DeclareLaunchArgument(
        'launch_controllers',
        default_value='true',
        description='Launch controllers after the mower is spawned',
    )

    mower_sim_dir = get_package_share_directory('mower_sim')
    mower_description_dir = get_package_share_directory('mower_description')
    gz_resource_paths = [
        os.path.dirname(mower_description_dir),
    ]
    existing_gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH')
    if existing_gz_resource_path:
        gz_resource_paths.append(existing_gz_resource_path)

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join(gz_resource_paths),
    )

    world_file = os.path.join(
        mower_sim_dir,
        'worlds',
        'mower_world.world'
    )

    declare_gz_args = DeclareLaunchArgument(
        'gz_args',
        # default_value=f'-v 2 -s -r {world_file} --headless-rendering',
        default_value=f'-v 4 -r {world_file}',
        description='Gazebo gz_sim arguments',
    )
    gz_args = LaunchConfiguration('gz_args')

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': gz_args,
        }.items()
    )

    spawn_mowerbot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mower_description_dir,
                'launch',
                'spawn_mowerbot.launch.py',
            )
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'use_sim_time': use_sim_time,
            'publish_robot_state_publisher': publish_robot_state_publisher,
            'launch_controllers': launch_controllers,
        }.items(),
    )

    delayed_spawn_mowerbot = TimerAction(
        period=2.0,
        actions=[spawn_mowerbot_cmd],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_x_pose)
    ld.add_action(declare_y_pose)
    ld.add_action(declare_publish_robot_state_publisher)
    ld.add_action(declare_launch_controllers)
    ld.add_action(declare_gz_args)
    ld.add_action(set_gz_resource_path)
    ld.add_action(gz_sim_launch)
    ld.add_action(delayed_spawn_mowerbot)
    return ld
