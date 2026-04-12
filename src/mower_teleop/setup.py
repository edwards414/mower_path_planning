from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'mower_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fxrbindi',
    maintainer_email='fxrbindi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop_keyboard = mower_teleop.teleop_keyboard:main',
            'blade_teleop_joy = mower_teleop.blade_teleop_joy:main',
        ],
    },
)
