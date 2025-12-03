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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration



def generate_launch_description():

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    os.environ['TURTLEBOT3_MODEL'] = 'burger_cam_gps'
    # 取得 boustrophedon_coverage 套件的 share 目錄
    nav2_gps_waypoint_follower_dir = get_package_share_directory(
        'nav2_gps_waypoint_follower'
    )

    # world 檔案
    world_file = os.path.join(
        nav2_gps_waypoint_follower_dir,
        'worlds',
        'mower_world.world'
    )

    # 載入 Gazebo world
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-v 4 -s -r  {world_file} --headless-rendering'
        }.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_gps_waypoint_follower_dir,
                'launch',
                'robot_state_publisher.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_gps_waypoint_follower_dir, 'launch', 'spawn_turtlebot3.launch.py'
            )
        ),
        launch_arguments={
            'x_pose': '0.0',
            'y_pose': '0.0'
        }.items()
    )


    ld = LaunchDescription()
    # Add the commands to the launch description
    ld.add_action(declare_use_sim_time)
    ld.add_action(gz_sim_launch)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(robot_state_publisher_cmd)
    return ld
