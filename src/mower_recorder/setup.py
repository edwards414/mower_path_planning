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
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fxrbindi',
    maintainer_email='edwards940428@gmail.com',
    description='The bag recorder\'s profiles and its Python tools '
                '(mower-check-run, mower-bag); the recorder is mower_rsd\'s '
                'bag module (E5-7)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mower-bag = mower_recorder.cli:main',
            'mower-check-run = mower_recorder.check_run:main',
        ],
    },
)
