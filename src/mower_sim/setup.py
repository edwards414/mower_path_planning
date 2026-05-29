from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'mower_sim'

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
        ('share/' + package_name + '/worlds', glob(os.path.join('worlds', '*'))),
        ('share/' + package_name + '/urdf', glob(os.path.join('urdf', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fxrbindi',
    maintainer_email='edwards940428@gmail.com',
    description='Mower simulation: Gazebo worlds, URDF assets, and sim-only launch files',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
