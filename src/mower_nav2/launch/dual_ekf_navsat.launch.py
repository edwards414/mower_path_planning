# Copyright 2018 Open Source Robotics Foundation, Inc.
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
import launch.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    gps_fix_topic = LaunchConfiguration('gps_fix_topic')

    mower_nav2_dir = get_package_share_directory('mower_nav2')
    rl_params_file = os.path.join(
        mower_nav2_dir, 'config', 'dual_ekf_navsat_params.yaml')
    return LaunchDescription(
        [
            declare_use_sim_time,
            DeclareLaunchArgument(
                'gps_fix_topic',
                default_value='/fix',
                description='Canonical raw GPS fix consumed by localization',
            ),
            launch.actions.DeclareLaunchArgument(
                'output_final_position', default_value='false'
            ),
            launch.actions.DeclareLaunchArgument(
                'output_location', default_value='~/dual_ekf_navsat_example_debug.txt'
            ),
            launch_ros.actions.Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node_odom',
                output='screen',
                parameters=[rl_params_file, {'use_sim_time': use_sim_time}],
                remappings=[
                    ('odometry/filtered', 'odometry/local'),
                    ('imu', 'imu/data'),
                ],
            ),
            launch_ros.actions.Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node_map',
                output='screen',
                parameters=[rl_params_file, {'use_sim_time': use_sim_time}],
                remappings=[
                    ('odometry/filtered', 'odometry/global'),
                    ('imu', 'imu/data'),
                ],
            ),
            launch_ros.actions.Node(
                package='robot_localization',
                executable='navsat_transform_node',
                name='navsat_transform',
                output='screen',
                parameters=[rl_params_file, {'use_sim_time': use_sim_time}],
                remappings=[
                    # navsat_transform subscribes to `imu` (not `imu/data`); the
                    # old ('imu/data','imu/data') was a no-op so it never got IMU
                    # yaw → datum never established → toLL returned 0.
                    ('imu', 'imu/data'),
                    ('gps/fix', gps_fix_topic),
                    ('gps/filtered', 'gps/filtered'),
                    ('odometry/gps', 'odometry/gps'),
                    ('odometry/filtered', 'odometry/global'),
                ],
            ),
        ]
    )
