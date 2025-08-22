from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'boustrophedon_coverage'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'map'), glob(os.path.join('map', '*'))),
        (os.path.join('share', package_name, 'extra'), glob(os.path.join('extra', '*')))
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
            'boustrophedon_coverage = boustrophedon_coverage.boustrophedon_coverage:main',
            'clean_robot = boustrophedon_coverage.clean_robot_coverage_path_planning:main',
            'test_clean_robot = boustrophedon_coverage.clean_robot_coverage_path_planning:clean_robot:main',
        ],
    },
)
