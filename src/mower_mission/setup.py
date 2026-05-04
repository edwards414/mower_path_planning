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
            'coverage_node = mower_mission.coverage_node:main',
            'map_manage_node = mower_mission.map_manage_node:main',
            'path_record_node = mower_mission.path_record_node:main',
            'nav_action_client = mower_mission.utils.nav_action_client:main',
            'zone_map_client = mower_mission.utils.zone_map_client:main',
            # legacy aliases
            'boustrophedon_coverage = mower_mission.coverage_node:main',
            'map_manage = mower_mission.map_manage_node:main',
            'path_record = mower_mission.path_record_node:main',
        ],
    },
)
