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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('mower_recorder')
    default_cfg = os.path.join(pkg, 'config', 'record_topics.yaml')
    default_qos = os.path.join(pkg, 'config', 'qos_overrides.yaml')

    robot_id = LaunchConfiguration('robot_id')
    output_root = LaunchConfiguration('output_root')
    autostart = LaunchConfiguration('autostart')
    config_file = LaunchConfiguration('config_file')
    qos_overrides_path = LaunchConfiguration('qos_overrides_path')
    git_repo_dir = LaunchConfiguration('git_repo_dir')
    r2_env_file = LaunchConfiguration('r2_env_file')
    upload_policy = LaunchConfiguration('upload_policy')
    gps_topic = LaunchConfiguration('gps_topic')
    video_api_url = LaunchConfiguration('video_api_url')
    min_free_mb = LaunchConfiguration('min_free_mb')

    # Nodes whose disappearance raises a fault (flushes the snapshot buffer).
    # Edit here to watch your own critical nodes.
    watch_nodes = ['map_manage', 'boustrophedon_coverage']

    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='mower'),
        DeclareLaunchArgument('output_root', default_value='~/mower_bags'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('config_file', default_value=default_cfg),
        DeclareLaunchArgument('qos_overrides_path', default_value=default_qos),
        # Point this at a git repo to stamp the run with its commit sha.
        DeclareLaunchArgument('git_repo_dir', default_value=''),
        # Gitignored .env with R2 creds; '' = read plain R2_* env vars.
        DeclareLaunchArgument('r2_env_file', default_value=''),
        DeclareLaunchArgument('upload_policy', default_value='wifi_or_dock'),
        DeclareLaunchArgument('gps_topic', default_value='/fix'),
        # MediaMTX API (e.g. http://127.0.0.1:9997): record the front camera
        # into <run>/video/ while a run goes (video_record.py); '' = no video.
        DeclareLaunchArgument('video_api_url', default_value=''),
        # Stop a run when the disk gets under this many MB free; 0 = never.
        DeclareLaunchArgument('min_free_mb', default_value='2048'),

        Node(
            package='mower_recorder',
            executable='graph_snapshot_node',
            name='graph_snapshot',
            output='screen',
            parameters=[{
                'period_s': 2.0,
                'watch_nodes': watch_nodes,
                'fault_topic': '/mower_recorder/fault',
            }],
        ),
        Node(
            package='mower_recorder',
            executable='recorder_manager_node',
            name='recorder_manager',
            output='screen',
            parameters=[{
                'robot_id': robot_id,
                'output_root': output_root,
                'autostart': autostart,
                'config_file': config_file,
                'qos_overrides_path': qos_overrides_path,
                'git_repo_dir': git_repo_dir,
                'params_dump_nodes': [
                    '/boustrophedon_coverage', '/map_manage'],
                'gps_topic': gps_topic,
                'fault_topic': '/mower_recorder/fault',
                'video_api_url': video_api_url,
                'min_free_mb': min_free_mb,
            }],
        ),
        Node(
            package='mower_recorder',
            executable='bag_store_node',
            name='bag_store',
            output='screen',
            parameters=[{
                'output_root': output_root,
                'robot_id': robot_id,
                'r2_env_file': r2_env_file,
                'upload_policy': upload_policy,
                'wifi_interfaces': ['wlan0'],
                'dock_topic': '',
            }],
        ),
    ])
