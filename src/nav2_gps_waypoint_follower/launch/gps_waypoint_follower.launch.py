# Copyright (c) 2018 Intel Corporation
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
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml



def generate_launch_description():
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ'
    )

    use_rviz = LaunchConfiguration('use_rviz')
    # 取得 launch 目錄
    nav2_gps_waypoint_follower_dir = get_package_share_directory(
        'nav2_gps_waypoint_follower'
    )

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_gps_waypoint_follower_dir, 'launch', 'gps_world.launch.py'
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items()
    )


    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_gps_waypoint_follower_dir, 'launch', 'rviz.launch.py')
        ),
        condition=IfCondition(use_rviz)
    )

    # mapviz_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_dir, 'mapviz.launch.py')
    #     ),
    #     condition=IfCondition(use_mapviz)
    # )


    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)

    # simulator launch
    ld.add_action(gazebo_cmd)
    # clock bridge（必須在 Gazebo 之後啟動）

    # robot localization launch
    # ld.add_action(robot_localization_cmd)

    # # navigation2 launch
    # ld.add_action(navigation_cmd)
    # # viz launch
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(rviz_cmd)

    # ld.add_action(declare_use_mapviz_cmd)
    # ld.add_action(mapviz_cmd)

    return ld
