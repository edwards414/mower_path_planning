from setuptools import find_packages, setup

package_name = 'mower_qt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'PyQt5',
        'pyserial',
    ],
    zip_safe=True,
    maintainer='fxrbindi',
    maintainer_email='edwards940428@gmail.com',
    description='PyQt GUI for ROS2 service control',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mower_qt = mower_qt.mower_qt:main',
            'stm32_uart_monitor = mower_qt.stm32_uart_monitor:main',
        ],
    },
)
