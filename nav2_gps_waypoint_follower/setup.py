from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'nav2_gps_waypoint_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        ('share/' + package_name + '/config', glob(os.path.join('config', '*'))),
        ('share/' + package_name + '/models/turtlebot3_burger_cam_gps', glob(os.path.join('models/turtlebot3_burger_cam_gps', '*'))),
        ('share/' + package_name + '/worlds', glob(os.path.join('worlds', '*'))),
        ('share/' + package_name + '/urdf', glob(os.path.join('urdf', '*'))),
        ('share/' + package_name + '/map', glob(os.path.join('map', '*'))),
        ('share/' + package_name + '/rviz', glob(os.path.join('rviz', '*'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fxrbindi',
    maintainer_email='edwards940428@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_pub = nav2_gps_waypoint_follower.waypoint_pub:main',
        ],
    },
)
