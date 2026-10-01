# Copyright 2026 fxrbindi
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
"""data_collection profile: front camera into ROS + MCAP recording + R2.

Spec: doc/Robot_Recording_R2_Pipeline_Spec.md 1 / 2 (GrassVision workspace).
Runs next to the normal stack (robot.launch.py, record:=false) — e.g. in the
``data_collection`` compose project (deploy/docker-compose.data-collection.
yaml). Do not combine with record:=true: both would own /mower_recorder/*.

    ros2 launch mower_recorder data_collection.launch.py \
        robot_id:=mower-03 output_root:=~/.mower/bags calib_dir:=~/.mower/calib \
        session_file:=~/.mower/data_collection_session.yaml

Recording starts with ``ros2 service call /mower_recorder/start
std_srvs/srv/Trigger`` (autostart:=false by default, so the operator checks
the camera first) and must end with ``/mower_recorder/stop`` so both mcaps
are finalized. Field procedure: docs/資料收集錄製程序.md.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mower_recorder import camera_launch

# Parameters of these nodes land in <run>/params/ (EKF modes, navsat yaw
# offset / declination: needed offline, P-086). Each dump takes a few seconds
# before the recorders start.
PARAMS_DUMP_NODES = ['/ekf_filter_node_odom', '/ekf_filter_node_map',
                     '/navsat_transform']


def _setup(context):
    def get(name):
        return LaunchConfiguration(name).perform(context).strip()

    pkg = get_package_share_directory('mower_recorder')
    cam = camera_launch.resolve_camera(context)
    actions = camera_launch.camera_actions(cam)

    robot_id = get('robot_id') or os.environ.get('MOWER_ROBOT_ID') or 'mower'
    session = os.path.expanduser(get('session_file'))
    if not os.path.isfile(session):
        actions.append(LogInfo(msg=(
            f'[data_collection] WARNING: 找不到 {session}，run_metadata 使用範本 '
            'config/data_collection_session.yaml（草高、天氣等要手動補）')))
        session = os.path.join(pkg, 'config', 'data_collection_session.yaml')

    actions += [
        Node(
            package='mower_recorder', executable='graph_snapshot_node',
            name='graph_snapshot', output='screen',
            parameters=[{'period_s': 2.0, 'watch_nodes': [''],
                         'fault_topic': '/mower_recorder/fault'}],
        ),
        Node(
            package='mower_recorder', executable='recorder_manager_node',
            name='recorder_manager', output='screen',
            parameters=[{
                'robot_id': robot_id,
                'output_root': get('output_root'),
                'autostart': get('autostart').lower() in ('1', 'true', 'yes'),
                'config_file': get('config_file'),
                'qos_overrides_path': get('qos_overrides_path'),
                'git_repo_dir': get('git_repo_dir'),
                'params_dump_nodes': PARAMS_DUMP_NODES,
                'gps_topic': get('gps_topic'),
                'fault_topic': '/mower_recorder/fault',
                'session_file': session,
                'run_info': get('run_info'),
                **camera_launch.recorder_parameters(cam),
            }],
        ),
    ]
    if get('start_bag_store').lower() in ('1', 'true', 'yes'):
        actions.append(Node(
            package='mower_recorder', executable='bag_store_node',
            name='bag_store', output='screen',
            parameters=[{
                'output_root': get('output_root'),
                'robot_id': robot_id,
                'r2_env_file': get('r2_env_file'),
                'upload_policy': get('upload_policy'),
                'wifi_interfaces': ['wlan0'],
                'dock_topic': '',
            }],
        ))
    return actions


def generate_launch_description():
    pkg = get_package_share_directory('mower_recorder')
    args = [
        DeclareLaunchArgument('robot_id', default_value='',
                              description="'' = $MOWER_ROBOT_ID, else 'mower'"),
        DeclareLaunchArgument('output_root', default_value='~/.mower/bags'),
        DeclareLaunchArgument('autostart', default_value='false',
                              description='start recording 3 s after launch'),
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(pkg, 'config', 'data_collection.yaml')),
        DeclareLaunchArgument(
            'qos_overrides_path',
            default_value=os.path.join(pkg, 'config', 'qos_overrides.yaml')),
        DeclareLaunchArgument('git_repo_dir', default_value=''),
        DeclareLaunchArgument('gps_topic', default_value='/fix'),
        DeclareLaunchArgument(
            'session_file', default_value='~/.mower/data_collection_session.yaml',
            description='operator metadata (grass height, weather, camera model)'),
        DeclareLaunchArgument(
            'run_info', default_value='',
            description="per-run YAML overrides, e.g. \"{weather: cloudy, "
                        "grass_height_cm: 6}\""),
        DeclareLaunchArgument('start_bag_store', default_value='true'),
        DeclareLaunchArgument('r2_env_file', default_value='',
                              description="gitignored R2 KEY=VALUE file; '' = env"),
        DeclareLaunchArgument('upload_policy', default_value='wifi_or_dock',
                              description='wifi_or_dock | always | manual'),
    ]
    return LaunchDescription(
        args + camera_launch.declare_camera_arguments()
        + [OpaqueFunction(function=_setup)])
