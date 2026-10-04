import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mower_recorder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fxrbindi',
    maintainer_email='edwards940428@gmail.com',
    description='Field bag recorder: curated rosbag2(mcap) + graph snapshot '
                '+ run metadata',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'graph_snapshot_node = mower_recorder.graph_snapshot_node:main',
            'recorder_manager_node = mower_recorder.recorder_manager_node:main',
            'bag_store_node = mower_recorder.bag_store_node:main',
            'mower-bag = mower_recorder.cli:main',
            'mower-check-run = mower_recorder.check_run:main',
        ],
    },
)
