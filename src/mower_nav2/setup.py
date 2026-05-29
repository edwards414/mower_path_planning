from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'mower_nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/launch',
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))
        ),
        ('share/' + package_name + '/config', glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fxrbindi',
    maintainer_email='edwards940428@gmail.com',
    description='Mower Nav2 stack: navigation, localization, GPS waypoint follower, and Nav2 params',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
