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
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
import launch_ros.actions


# rust_localize:=true -- mower_rs/mower_localize (the ported EKFs and
# navsat_transform, docs/ROS_FREE_PLAN.md Phase C) instead of the three
# robot_localization processes, which were measured at 23 % of a core on the
# LubanCat. The node names, topics, transforms and services are unchanged; the
# switch is mutually exclusive with the C++ nodes, never additive, because two
# publishers of map->odom would fight over tf.
_TRUE = "('true', '1', 'yes', 'on')"


def _rust_localize_binary(rust_localize, rust_daemon):
    """The separate mower_localize process: switch on, daemon not running."""
    return IfCondition(PythonExpression([
        "'", rust_localize, "'.lower() in ", _TRUE,
        " and '", rust_daemon, "'.lower() not in ", _TRUE,
    ]))


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    gps_fix_topic = LaunchConfiguration('gps_fix_topic')
    rust_localize = LaunchConfiguration('rust_localize')
    rust_daemon = LaunchConfiguration('rust_daemon')
    cpp_localization = UnlessCondition(rust_localize)

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
            DeclareLaunchArgument(
                'rust_localize',
                default_value='false',
                description='mower_rs mower_localize (one process) instead of '
                            'the two ekf_node processes and '
                            'navsat_transform_node',
            ),
            DeclareLaunchArgument(
                'rust_daemon',
                default_value='false',
                description='The mower_rs modules run inside one mower_rsd '
                            'process started by robot.launch.py, so the '
                            'separate mower_localize binary stays down here',
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
                condition=cpp_localization,
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
                condition=cpp_localization,
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
                condition=cpp_localization,
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
            # One process, three nodes with the same names. No `name=`:
            # launch_ros would emit a bare `-r __node:=`, which renames every
            # node in the process; mower_localize names its nodes itself, and
            # the topic wiring the C++ nodes get from `remappings=` above is a
            # node-scoped parameter here.
            launch_ros.actions.Node(
                package='mower_rs',
                executable='mower_localize',
                output='screen',
                condition=_rust_localize_binary(rust_localize, rust_daemon),
                respawn=True,
                respawn_delay=2.0,
                arguments=[
                    '--ros-args',
                    '-p', ['navsat_transform:gps_fix_topic:=', gps_fix_topic],
                ],
            ),
        ]
    )
