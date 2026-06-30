from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'mower_mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fxrbindi',
    maintainer_email='edwards940428@gmail.com',
    description='Mower mission logic: coverage path, map management, path recording',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_dock_pose_publisher = '
            'mower_mission.apriltag_dock_pose_publisher:main',
            'coverage_node = mower_mission.coverage_node:main',
            'docking_manager_node = mower_mission.docking_manager_node:main',
            'flutter_adapter_node = '
            'mower_mission.adapters.flutter_adapter_node:main',
            'battery_simulator_node = '
            'mower_mission.battery_simulator_node:main',
            'map_manage_node = mower_mission.map_manage_node:main',
            'heartbeat_node = mower_mission.heartbeat_node:main',
            'auto_coverage_node = mower_mission.auto_coverage_node:main',
            'path_record_node = mower_mission.path_record_node:main',
            'temp_dock_pose_publisher = '
            'mower_mission.temp_dock_pose_publisher:main',
            'nav_action_server = mower_mission.navigation.nav_action_server:main',
            'nav_action_client = mower_mission.utils.nav_action_client:main',
            'zone_map_client = mower_mission.utils.zone_map_client:main',
            # legacy aliases
            'boustrophedon_coverage = mower_mission.coverage_node:main',
            'map_manage = mower_mission.map_manage_node:main',
            'path_record = mower_mission.path_record_node:main',
        ],
    },
)
